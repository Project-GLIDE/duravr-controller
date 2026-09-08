# Protocol Dura VR: Local Control Interface

A local web interface for the Dura VR drone: live video feed and keyboard
flight control from a browser, talking directly to the drone's own WiFi
protocol. No cloud, no account.

See [DISCLAIMER.md](DISCLAIMER.md) for safety and accuracy disclaimers
before flying.

## Quick start

```sh
python3 app.py
```

Then, while joined to the drone's WiFi access point (SSID `Dura VR-XXXXXX`,
gateway `192.168.0.1`), open **http://localhost:8090/**.

No dependencies: everything here is Python standard library only.

Useful flags:

```sh
python3 app.py --drone-ip 192.168.0.1 --drone-port 40000 --http-port 8090
```

A `--max-deflection` flag exists to cap axis travel, but is not verified
to work as intended.

## Status

Verified working:

- Video feed.
- Takeoff and landing, toggled with spacebar.
- Emergency stop (`e`) cuts motor power immediately and fires
  unconditionally, regardless of toggle state.
- Speed mode toggle (`q`), between normal and high-speed.
- Movement (WASD and arrow keys) drives the correct physical stick axis.
- Compass-mode flag bit is understood but not wired up to any key.
- The 3 trim controls (control packet bytes 12-14) are identified by name
  from the original app's manual. Not wired up in `app.py` yet.
- Remote-control gate (`o` key): a client-side switch that stops or
  resumes sending movement commands. Correct by construction, since it
  simply withholds packets this server would otherwise send.
- Telemetry battery reading (shown live on the page).

Present but not fully verified:

- The sign of each movement key, meaning which direction it actually moves
  the drone.
- Telemetry altitude and gyro readings (shown live on the page). Not yet
  independently verified.
- The exact delay a fresh connection needs before it will act on a takeoff
  command. In practice, letting `app.py` run for a while (tens of seconds)
  before using the takeoff key has worked; no fixed minimum is
  established. Likely a sensor/IMU stabilization period on the drone
  itself.

Known issues:

- Some video feed frames appear corrupted. The root cause is not known
  yet.

## Axis mapping

| Key | Slot, sign in `KEY_MAP` | Action | Verified |
|---|---|---|---|
| ArrowLeft | slot 3, -1 | yaw left | [x] |
| ArrowRight | slot 3, +1 | yaw right | [x] |
| ArrowUp | slot 2, +1 | throttle up | [ ] |
| ArrowDown | slot 2, -1 | throttle down | [x] |
| W | slot 1, +1 | pitch forward | [ ] |
| S | slot 1, -1 | pitch back | [x] |
| A | slot 0, -1 | roll left | [x] |
| D | slot 0, +1 | roll right | [x] |

`W` and `ArrowUp` remain unverified and should be treated with the same
caution as any unverified key.

Verified: which control packet byte reflects which
physical stick axis, and that pushing the stick in the listed direction
increases that byte's value. Each row is one byte in the raw control
packet: for example, byte 8 is the right stick's left/right axis, and
pushing that stick right increases byte 8's value up from its `0x80`
centre (pushing left decreases it).

| Byte offset | Physical stick axis | Increases toward |
|---|---|---|
| 8 | right stick, left/right | right |
| 9 | right stick, up/down | up |
| 10 | left stick, up/down | up |
| 11 | left stick, left/right | right |

`app.py`'s `KEY_MAP` uses the standard convention that right stick =
pitch/roll and left stick = throttle/yaw, with increasing value meaning
forward, right, climb, or clockwise respectively. That direction-of-travel
part is not yet independently verified by an actual flight.

The 3 trim controls (control packet bytes 12-14, sticky rather than
spring-back) are named in the original app's manual:

| Byte | Name | Location |
|---|---|---|
| 12 | forward/backward (pitch) trimmer | between the two sticks |
| 13 | bank (roll) trimmer | below the right stick |
| 14 | turn (yaw) trimmer | below the left stick |

Each moves in steps of 2. Not wired up to any key in `app.py` yet.

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
per-frame `frame_id`, a `chunk_idx`/`total_chunks` pair, a total-frame-length
field, per-chunk payload length), followed by raw JPEG bytes for that
chunk. Concatenating chunks `1..total_chunks` in order for one `frame_id`
yields a complete standard JPEG (640x480, roughly 15fps).

The total-frame-length field (offset 12-13 of the header) is verified:
it matches the reassembled frame's actual byte length exactly, with zero
mismatches across every frame checked. `app.py` rejects a frame if this
doesn't match.

### Control (type `0x0a`)

18 bytes, sent at ~20Hz while a stick is actively deflected, not sent
continuously at idle. Axis identity (which byte is which physical stick
control) is in "Axis mapping" above.

- 4 axis bytes, centred at `0x80` (neutral). Bytes 8+9 are the right
  stick, 10+11 are the left stick.
- A second set of 3 sticky axis-like bytes (offsets 12-14, the trim
  controls), also centred at `0x80` with a narrower range (roughly
  ±20-40). Unlike the primary 4 axes, these hold their value rather than
  springing back to `0x80` when released.
- Flags byte (offset 15): a baseline value, plus optional action bits
  OR'd on top. The baseline is `0x0c` normally, or `0x04` while
  high-speed mode is active (these two are alternatives, never combined).
  Action bits, OR'd onto whichever baseline is current: `+0x10` takeoff,
  `+0x20` landing, `+0x40` emergency stop (motor cutoff, drops the drone
  immediately), `+0x02` compass mode is active. Takeoff/stop/land pulse
  briefly; the high-speed baseline and compass mode are sticky, they
  persist until toggled off again.
- Checksum (offset 16) is the XOR of bytes 8 to 15.

### Telemetry (type `0x0b`)

15 bytes, streamed continuously regardless of anything the client sends.
Checksummed the same way as the control packet: offset 13 is the XOR of
bytes 8 to 12. Three bytes vary independently, each with a distinct enough
behavior to identify a role:

- Byte 8 (unsigned): verified: battery level. Decreases slowly and
  monotonically, never observed to increase or reset.
- Byte 9 (signed): might be altitude. The raw value tracked the original
  app's displayed altitude closely in one comparison (displayed -0.9, raw
  reading -1), though not matched byte-exactly.
- Byte 10 (signed): might be a gyro rate or vibration reading.
