"""
Local HTTP server that bridges a browser to the Protocol Dura VR drone.

Opens a UDP socket to the drone's video and control port, reassembles the
incoming MJPEG video stream and telemetry readings, and accepts flight
control input relayed from index.html over a small set of HTTP endpoints.
Uses only the Python standard library.

Usage
-----
python3 app.py [--drone-ip 192.168.0.1] [--drone-port 40000]
               [--http-port 8090] [--max-deflection 35]

While joined to the drone's WiFi access point, open http://localhost:8090/

Notes
-----
The axis to key mapping in KEY_MAP is a best guess, not yet confirmed
against a live flight. The speed cap (--max-deflection, also adjustable
from the page) limits how far any control axis can move from its centre
regardless of what key combination is held.
"""
import argparse
import json
import os
import socket
import struct
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

# Protocol constants.
HEARTBEAT_PAYLOAD = bytes([0x63, 0x63, 0x01, 0x00, 0x00, 0x00, 0x00])
HEARTBEAT_INTERVAL = 1.0

VIDEO_HEADER_LEN = 54
TELEMETRY_LEN = 15

CTRL_HEADER = bytes([0x63, 0x63, 0x0a, 0x00, 0x00, 0x0b, 0x00, 0x66])
CTRL_TRAILER = 0x99
AXIS_CENTER = 0x80
FLAG_BASE = 0x0c
FLAG_TAKEOFF = 0x10
FLAG_LAND = 0x40
CTRL_SEND_INTERVAL = 0.05  # roughly 20Hz, matches the drone's expected rate
BUTTON_BURST_DURATION = 0.4  # seconds to hold the takeoff/land flag

DEFAULT_MAX_DEFLECTION = 35
MAX_DEFLECTION_LIMIT = 127  # axis bytes centre on 0x80, hard limit either direction

# Axis slot (0 = byte 8, 1 = byte 9, 2 = byte 10, 3 = byte 11) and sign that
# each key drives. Not confirmed against a live flight yet. If a key moves
# the drone the wrong way, flip its sign here. If it moves the wrong axis,
# swap slots.
KEY_MAP = {
    "ArrowLeft":  (0, -1),   # yaw left
    "ArrowRight": (0, +1),   # yaw right
    "ArrowUp":    (1, +1),   # throttle up
    "ArrowDown":  (1, -1),   # throttle down
    "w": (3, +1),            # pitch forward
    "s": (3, -1),            # pitch back
    "a": (2, -1),            # roll left
    "d": (2, +1),            # roll right
}


def xor_checksum(body):
    """
    Compute the single-byte XOR checksum used by the control and telemetry
    packet formats.

    Parameters
    ----------
    body : bytes or list of int
        Bytes to checksum.

    Returns
    -------
    int
        XOR of every byte in `body`.
    """
    c = 0
    for b in body:
        c ^= b
    return c


def build_control_packet(axes, flags=FLAG_BASE):
    """
    Build an 18-byte type 0x0a control packet.

    Parameters
    ----------
    axes : sequence of int
        Four axis values (bytes 8 to 11), each centred on `AXIS_CENTER`.
    flags : int, optional
        Flags byte (offset 15). Defaults to the idle baseline.

    Returns
    -------
    bytes
        Complete packet, including header, checksum, and trailer.
    """
    body = bytes(axes) + bytes([0x80, 0x80, 0x80, flags])
    return CTRL_HEADER + body + bytes([xor_checksum(body), CTRL_TRAILER])


def build_button_packet(flag_bit):
    """
    Build a control packet for a one-shot button press, with all axes
    centred.

    Parameters
    ----------
    flag_bit : int
        Flag bit to set on top of the idle baseline, for example
        `FLAG_TAKEOFF` or `FLAG_LAND`.

    Returns
    -------
    bytes
        Complete 18-byte control packet.
    """
    return build_control_packet([AXIS_CENTER] * 4, FLAG_BASE | flag_bit)


class FrameBuffer:
    """
    Holds the most recently decoded video frame.

    Written to by the UDP reader thread as new frames complete. Read by
    each HTTP client thread serving the MJPEG stream. A condition variable
    lets readers block until a frame newer than the one they last sent is
    available, instead of polling.
    """

    def __init__(self):
        self._lock = threading.Condition()
        self._frame = None
        self._frame_no = 0

    def publish(self, jpeg_bytes):
        """Store a newly completed JPEG frame and wake any waiting readers."""
        with self._lock:
            self._frame = jpeg_bytes
            self._frame_no += 1
            self._lock.notify_all()

    def get_latest(self, after=None):
        """
        Block until a frame newer than `after` is available.

        Parameters
        ----------
        after : int or None
            Frame number already sent to this reader, or None for the
            first call.

        Returns
        -------
        tuple of (bytes, int)
            The frame bytes and its frame number.
        """
        with self._lock:
            while self._frame is None or self._frame_no == after:
                self._lock.wait()
            return self._frame, self._frame_no


class TelemetryState:
    """
    Holds the most recent telemetry reading from the drone.

    Written to by the UDP reader thread whenever a type 0x0b packet
    arrives. Read by the HTTP handler serving `/state`.
    """

    def __init__(self):
        self._lock = threading.Lock()
        self.value = None
        self.last_update = None

    def update(self, value):
        """Record a new telemetry reading."""
        with self._lock:
            self.value = value
            self.last_update = time.monotonic()

    def snapshot(self):
        """
        Return the current reading and its age.

        Returns
        -------
        tuple of (int or None, float or None)
            The last reading and how many seconds ago it arrived, or
            (None, None) if nothing has been received yet.
        """
        with self._lock:
            if self.value is None:
                return None, None
            age = time.monotonic() - self.last_update
            return self.value, age


class ControlState:
    """
    Tracks which control keys are currently held, the configured speed
    cap, and a client-side guess at whether the drone is airborne.

    Written to by the HTTP handlers for `/key`, `/speed`, and `/toggle`.
    Read by the control-sending loop to build outgoing packets.
    """

    def __init__(self, max_deflection):
        self._lock = threading.Lock()
        self.held_keys = set()
        self.max_deflection = max_deflection
        self.airborne = False  # client-side guess; the protocol itself has two one-shot buttons, not a toggle

    def set_key(self, key, down):
        """Record that `key` is now held or released."""
        with self._lock:
            if down:
                self.held_keys.add(key)
            else:
                self.held_keys.discard(key)

    def set_max_deflection(self, value):
        """
        Update the speed cap, clamped to a valid range.

        Parameters
        ----------
        value : int
            Requested maximum axis deflection from centre.

        Returns
        -------
        int
            The clamped value actually stored.
        """
        with self._lock:
            self.max_deflection = max(0, min(MAX_DEFLECTION_LIMIT, int(value)))
            return self.max_deflection

    def snapshot_axes(self):
        """
        Compute the four axis byte values for the currently held keys.

        Returns
        -------
        tuple of (list of int, bool)
            The four axis values, and whether any mapped key is held.
        """
        with self._lock:
            keys = set(self.held_keys)
            cap = self.max_deflection
        axes = [AXIS_CENTER] * 4
        for key in keys:
            mapping = KEY_MAP.get(key)
            if mapping is None:
                continue
            slot, sign = mapping
            axes[slot] = max(0, min(255, AXIS_CENTER + sign * cap))
        return axes, bool(keys & KEY_MAP.keys())

    def toggle_flight(self):
        """Flip the client-side airborne guess and return the new value."""
        with self._lock:
            self.airborne = not self.airborne
            return self.airborne


def udp_reader(sock, drone_addr, frame_buffer, telemetry_state, stop_event):
    """
    Send the periodic heartbeat, and receive video and telemetry packets.

    Runs until `stop_event` is set. Starts a background thread sending a
    heartbeat to `drone_addr`, then loops reading from `sock`, publishing
    completed video frames to `frame_buffer` and telemetry readings to
    `telemetry_state`.

    Parameters
    ----------
    sock : socket.socket
        Bound UDP socket, shared with the control-sending loop.
    drone_addr : tuple of (str, int)
        Drone address as (ip, port).
    frame_buffer : FrameBuffer
        Destination for reassembled video frames.
    telemetry_state : TelemetryState
        Destination for telemetry readings.
    stop_event : threading.Event
        Set to stop the loop and the heartbeat thread.
    """
    def send_heartbeat():
        while not stop_event.is_set():
            try:
                sock.sendto(HEARTBEAT_PAYLOAD, drone_addr)
            except OSError as e:
                print(f"[udp] heartbeat send failed: {e}")
            stop_event.wait(HEARTBEAT_INTERVAL)

    threading.Thread(target=send_heartbeat, daemon=True).start()

    chunks = {}
    current_frame_id = None
    total_chunks_expected = None

    while not stop_event.is_set():
        try:
            data, addr = sock.recvfrom(65535)
        except socket.timeout:
            continue
        except OSError:
            break

        if len(data) < 8 or data[0] != 0x63 or data[1] != 0x63:
            continue

        pkt_type = data[2]

        if pkt_type == 0x0b:
            if len(data) >= TELEMETRY_LEN:
                raw = data[10]
                value = raw - 256 if raw > 127 else raw
                telemetry_state.update(value)
            continue

        if pkt_type != 0x03:
            continue
        if len(data) < VIDEO_HEADER_LEN:
            continue

        frame_id = data[8]
        chunk_idx = data[48]
        total_chunks = data[50]
        plen = struct.unpack("<H", data[52:54])[0]
        payload = data[54:54 + plen]

        if frame_id != current_frame_id:
            current_frame_id = frame_id
            total_chunks_expected = total_chunks
            chunks = {}

        chunks[chunk_idx] = payload

        if total_chunks_expected and len(chunks) == total_chunks_expected:
            try:
                jpeg = b"".join(chunks[i] for i in range(1, total_chunks_expected + 1))
            except KeyError:
                chunks = {}
                continue
            if jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9"):
                frame_buffer.publish(jpeg)
            chunks = {}


def control_loop(sock, drone_addr, control_state, stop_event):
    """
    Continuously send control packets while any mapped key is held.

    Runs until `stop_event` is set, sampling `control_state` at a fixed
    interval and sending a packet only while at least one mapped key is
    currently down, matching the rate the drone expects.

    Parameters
    ----------
    sock : socket.socket
        Bound UDP socket, shared with `udp_reader`.
    drone_addr : tuple of (str, int)
        Drone address as (ip, port).
    control_state : ControlState
        Source of the current key state and speed cap.
    stop_event : threading.Event
        Set to stop the loop.
    """
    while not stop_event.is_set():
        axes, any_held = control_state.snapshot_axes()
        if any_held:
            try:
                sock.sendto(build_control_packet(axes), drone_addr)
            except OSError as e:
                print(f"[udp] control send failed: {e}")
        stop_event.wait(CTRL_SEND_INTERVAL)


def send_button_burst(sock, drone_addr, flag_bit):
    """
    Send a takeoff or land command as a short burst of identical packets.

    Parameters
    ----------
    sock : socket.socket
        Bound UDP socket to send on.
    drone_addr : tuple of (str, int)
        Drone address as (ip, port).
    flag_bit : int
        `FLAG_TAKEOFF` or `FLAG_LAND`.
    """
    end = time.monotonic() + BUTTON_BURST_DURATION
    pkt = build_button_packet(flag_bit)
    while time.monotonic() < end:
        try:
            sock.sendto(pkt, drone_addr)
        except OSError as e:
            print(f"[udp] button send failed: {e}")
        time.sleep(CTRL_SEND_INTERVAL)


INDEX_HTML_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html")
with open(INDEX_HTML_PATH, "rb") as f:
    INDEX_HTML = f.read()


def make_handler(frame_buffer, telemetry_state, control_state, sock, drone_addr):
    """
    Build the HTTP request handler class for the local web server.

    Parameters
    ----------
    frame_buffer : FrameBuffer
        Source for the `/stream.mjpg` endpoint.
    telemetry_state : TelemetryState
        Source for the telemetry value returned by `/state`.
    control_state : ControlState
        State updated by `/key` and `/speed`, and read by `/state`.
    sock : socket.socket
        Bound UDP socket used to send takeoff/land bursts.
    drone_addr : tuple of (str, int)
        Drone address as (ip, port).

    Returns
    -------
    type
        A `BaseHTTPRequestHandler` subclass ready to pass to
        `ThreadingHTTPServer`.
    """
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, fmt, *args):
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html")
                self.send_header("Content-Length", str(len(INDEX_HTML)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(INDEX_HTML)
            elif self.path == "/stream.mjpg":
                self._serve_stream()
            elif self.path == "/state":
                telem_value, telem_age = telemetry_state.snapshot()
                body = json.dumps({
                    "airborne": control_state.airborne,
                    "max_deflection": control_state.max_deflection,
                    "max_deflection_limit": MAX_DEFLECTION_LIMIT,
                    "telemetry": telem_value,
                    "telemetry_age": telem_age,
                }).encode()
                self.send_response(200)
                self.send_header("Content-Type", "application/json")
                self.send_header("Content-Length", str(len(body)))
                self.end_headers()
                self.wfile.write(body)
            else:
                self.send_error(404)

        def do_POST(self):
            length = int(self.headers.get("Content-Length", 0))
            raw = self.rfile.read(length) if length else b""
            if self.path == "/key":
                try:
                    data = json.loads(raw)
                    control_state.set_key(data["key"], bool(data["down"]))
                except (ValueError, KeyError):
                    pass
                self._ok()
            elif self.path == "/toggle":
                airborne = control_state.toggle_flight()
                flag = FLAG_TAKEOFF if airborne else FLAG_LAND
                threading.Thread(
                    target=send_button_burst, args=(sock, drone_addr, flag), daemon=True
                ).start()
                self._ok()
            elif self.path == "/speed":
                try:
                    data = json.loads(raw)
                    control_state.set_max_deflection(data["max_deflection"])
                except (ValueError, KeyError, TypeError):
                    pass
                self._ok()
            else:
                self.send_error(404)

        def _ok(self):
            self.send_response(204)
            self.end_headers()

        def _serve_stream(self):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            last_seen = None
            try:
                while True:
                    frame, last_seen = frame_buffer.get_latest(after=last_seen)
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(frame)}\r\n\r\n".encode())
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError):
                pass

    return Handler


def main():
    """Parse command-line arguments and run the server until interrupted."""
    parser = argparse.ArgumentParser()
    parser.add_argument("--drone-ip", default="192.168.0.1")
    parser.add_argument("--drone-port", type=int, default=40000)
    parser.add_argument("--http-port", type=int, default=8090)
    parser.add_argument(
        "--max-deflection", type=int, default=DEFAULT_MAX_DEFLECTION,
        help="Speed cap: max axis distance from centre (0 to 127). Keep low "
             f"until the key mapping is verified live (default: {DEFAULT_MAX_DEFLECTION}). "
             "Also adjustable live from the webpage.",
    )
    args = parser.parse_args()

    drone_addr = (args.drone_ip, args.drone_port)
    frame_buffer = FrameBuffer()
    telemetry_state = TelemetryState()
    control_state = ControlState(args.max_deflection)
    stop_event = threading.Event()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(2.0)
    sock.bind(("0.0.0.0", 0))
    print(f"[udp] bound local port {sock.getsockname()[1]}, target {drone_addr[0]}:{drone_addr[1]}")

    threading.Thread(
        target=udp_reader, args=(sock, drone_addr, frame_buffer, telemetry_state, stop_event), daemon=True
    ).start()
    threading.Thread(
        target=control_loop, args=(sock, drone_addr, control_state, stop_event), daemon=True
    ).start()

    server = ThreadingHTTPServer(
        ("0.0.0.0", args.http_port), make_handler(frame_buffer, telemetry_state, control_state, sock, drone_addr)
    )
    print(f"[http] serving on http://localhost:{args.http_port}/")
    print(f"[ctrl] speed cap (max axis deflection): {args.max_deflection}/{MAX_DEFLECTION_LIMIT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        server.shutdown()


if __name__ == "__main__":
    main()
