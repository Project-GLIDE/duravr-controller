# Protocol Dura VR: Local Control Interface

A local web interface for the Dura VR drone: live video feed and keyboard
flight control from a browser, talking directly to the drone's own WiFi
protocol. No cloud, no account.

## Quick start

```
python3 app.py
```

Then, while joined to the drone's WiFi access point (SSID `Dura VR-XXXXXX`,
gateway `192.168.0.1`), open **http://localhost:8090/**.

No dependencies: everything here is Python standard library only.

Useful flags:

```
python3 app.py --drone-ip 192.168.0.1 --drone-port 40000 --http-port 8090 --max-deflection 35
```

`--max-deflection` is the software speed cap (0 to 127, default 35): the
maximum any control axis is allowed to move from centre, regardless of what
key combination requests. Also adjustable live from the page.

## Status

Confirmed working:

- Video feed.
- Takeoff and land (spacebar).

Present but not fully confirmed:

- Movement (WASD and arrow keys). Packets are protocol-correct, but the
  mapping from key to physical direction is a guess pending a field test.
  See the table below.
- Telemetry reading (shown live on the page). The packet structure and its
  checksum are confirmed; the physical meaning of the value itself is not.
- The exact delay a fresh connection needs before it will act on a takeoff
  command. In practice, letting `app.py` run for a while (tens of seconds)
  before using the takeoff key has worked; no fixed minimum is established.

Known issues:

- Video frames occasionally arrive visibly corrupted (a solid colour band
  partway down the frame). This appears to originate at the drone's own
  camera encoder rather than in reassembly here: reconstructed frames match
  their raw captured bytes exactly, and the corrupted ones still decode as
  structurally valid JPEG.

## Axis mapping

| Key | Slot, sign in `KEY_MAP` | Guessed as | Observed |
|---|---|---|---|
| ArrowLeft | slot 0, -1 | yaw left | |
| ArrowRight | slot 0, +1 | yaw right | |
| ArrowUp | slot 1, +1 | throttle up | |
| ArrowDown | slot 1, -1 | throttle down | |
| W | slot 3, +1 | pitch forward | |
| S | slot 3, -1 | pitch back | |
| A | slot 2, -1 | roll left | **down** (2026-08-24) |
| D | slot 2, +1 | roll right | |

`KEY_MAP` in `app.py` is left unchanged until the "Observed" column is
filled in, so it can be corrected in one pass rather than piecemeal.

## Protocol reference

All traffic (video, keepalive, telemetry, control) is UDP between this host
and the drone at **192.168.0.1:40000**. Every packet starts with magic
`63 63`, followed by a one-byte type. The same reference tables are shown
on the running page itself.

| Type | Direction | Meaning |
|---|---|---|
| `0x01` | client to drone | heartbeat, sent every ~1s, required to keep the video stream alive |
| `0x01` | drone to client | heartbeat reply / device identification (`"Dura VR-XXXXXX"`) |
| `0x03` | drone to client | video frame chunk (MJPEG, chunked to ~1454-byte UDP packets) |
| `0x0a` | client to drone | joystick / button control, 18 bytes |
| `0x0b` | drone to client | telemetry, 15 bytes, streamed continuously (~10Hz) |

### Video (type `0x03`)

Every packet has an identical 54-byte header (magic, type, length, a
per-frame `frame_id`, a `chunk_idx`/`total_chunks` pair, per-chunk payload
length), followed by raw JPEG bytes for that chunk. Concatenating chunks
`1..total_chunks` in order for one `frame_id` yields a complete standard
JPEG (640x480, roughly 15fps).

### Control (type `0x0a`)

18 bytes, sent at ~20Hz while a stick is actively deflected, not sent
continuously at idle.

- 4 axis bytes, centred at `0x80` (neutral). Bytes 8+9 and 10+11 each pair
  up as one physical stick; which stick drives which direction is the part
  still unconfirmed (see "Axis mapping" above).
- Flags byte (offset 15): baseline `0x0c`; `+0x10` = takeoff; `+0x40` =
  land/stop.
- Checksum (offset 16) is the XOR of bytes 8 to 15.

### Telemetry (type `0x0b`)

15 bytes, streamed continuously regardless of anything the client sends.

- Checksummed the same way as the control packet: offset 13 is the XOR of
  bytes 8 to 12.
- The one byte that actually varies (offset 10, signed) drifts slowly over
  tens of seconds while the drone sits idle.

