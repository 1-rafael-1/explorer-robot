# Link Protocol envelope: a frozen binary header with lossy streams and reliable commands

Three projects consume one contract over the expansion port's UART1: the robot firmware, the
ESP32-C3 Bridge firmware and the PC daemon. They will not ship in lockstep. The data connector
has no RTS/CTS, and the robot must never block a producer on the link. The envelope therefore
needs a frame boundary that survives arbitrary byte loss, a version story that lets an old robot
and a new Remote coexist, and two different delivery guarantees on the same wire: telemetry is
lossy and coalescing, while commands and edge events are reliable.

The envelope is the part all three share. The Bridge parses only the envelope and forwards; it
is not a domain peer and never interprets a payload. Payload contents, the baud and the firmware
task scheduling are separate decisions.

**Status:** accepted. The frame layout, classes, delivery guarantees and session lease are
settled. The payload catalogue is settled in ADR-0031, and the baud, frame timeout and ring
sizes in ADR-0033. The baseline stream rates remain open, and the reliability model is
provisional until the protocol crate's host tests exercise it. Command authority is settled in
ADR-0034: the lease is liveness, not authority. ADR-0035 adds a 2 s peer-silence **Deadman** on
the same `last_peer_frame`: the 5 s lease stays liveness and also disarms it, and the daemon
`PING`s at 1 Hz while a Session is open. ADR-0036 adds `BRIDGE_CONTROL` (`0x0004`) and makes it
the third acknowledged control frame after `HELLO`/`PING`; the Bridge now consumes one control
message (`THROTTLE` out, `BRIDGE_CONTROL` in) instead of being envelope-only. ADR-0037 puts the
pairing-token gate and the single-client enforcement at the Bridge, before any frame is forwarded;
the robot still holds only link state.

**Considered Options**

- **COBS or SLIP delimiter framing.** Rejected. Both make resynchronisation delimiter-exact,
  but they still need a CRC over the same bytes, add encoding overhead (COBS ~1 byte per 254,
  SLIP up to 2× in the worst case), and put a byte-stuffing codec on both ends. On a UART that
  already delivers bytes in order and only loses them in bursts, a bounded length plus a CRC
  gives the same resync property with less machinery.
- **A transparent byte pipe through the Bridge.** Rejected. With no RTS/CTS a blind pump can
  only overflow the UART, and it cannot apply latest-wins or throttle. The Bridge must
  understand the envelope.
- **A binary header with magic, length and CRC.** Accepted. One sliding-window parser handles
  corruption, truncation and false starts; the Bridge can forward without understanding
  payloads; and a 12-byte header is negligible against a 720-byte LiDAR cloud.
- **A credit window for telemetry.** Rejected. Credits are precise, but the robot cannot block
  a producer anyway: when it runs out of credit it drops or coalesces, which is exactly what a
  throttle hint achieves with less per-stream bookkeeping. Credits also require the receiver to
  account for frames it is about to discard.
- **A stateful session machine on the robot.** Rejected. The robot has no Session concept and
  streams regardless. A liveness lease — a timestamp refreshed by any valid PC→robot frame —
  gives the Remote a way to prove it is connected without putting session state on the robot's
  command path.
- **A codec per project.** Rejected. Three implementations of the same framing would drift,
  and the reliability rules are where the bugs live. One `no_std`, host-tested crate is the
  contract.

**Decision**

*Framing.*

Every frame is a fixed 12-byte header followed by 0–1024 payload bytes:

| offset | size | field | meaning |
|---|---|---|---|
| 0 | 2 | `magic` | `0xA5 0x5A` |
| 2 | 1 | `major` | envelope/payload major, starts at 1 |
| 3 | 1 | `class:2` | bits 7:6 — `00` stream, `01` command, `10` event, `11` control |
| 3 | 1 | `flags:6` | bits 5:0 |
| 4 | 2 | `id` | u16 little-endian, namespaced by class |
| 6 | 2 | `seq` | u16 little-endian, scope depends on class |
| 8 | 2 | `len` | payload bytes, u16 little-endian |
| 10 | 2 | `crc16` | u16 little-endian, CRC-16/CCITT-FALSE |
| 12 | `len` | payload | class-specific |

All multi-byte fields are little-endian; there is no byte stuffing and no delimiter.

Flags: bit 0 `ACK_REQUESTED`, bit 1 `IS_ACK`, bit 2 `RETRY`, bit 3 `FRAGMENT` (reserved, v1
rejects it), bits 4–5 reserved (send zero, ignore on receive).

CRC-16/CCITT-FALSE: polynomial `0x1021`, initial value `0xFFFF`, no reflection, no final XOR.
The check value for `"123456789"` is `0x29B1`. It covers bytes `0..10` plus the payload; the CRC
field is excluded. The codec exposes an incremental CRC so a large payload is not copied.

**The 12-byte header skeleton is frozen across major versions.** The magic, the header size,
the offset of `len` and `crc16`, and the byte order never change. A future major may change
flag meanings and payload semantics, never the offsets. This is what lets the Bridge forward a
newer robot or daemon without an update: it reads only magic, class, id, length and CRC and
ignores `major`.

*Limits and resync.*

| direction | max payload | max frame |
|---|---|---|
| robot → PC (streams, events) | 1024 B | 1036 B |
| PC → robot (commands, control) | 256 B | 268 B |

The receiver is a three-state machine: `SYNC` scans for the magic, `HEADER` collects 12 bytes,
`PAYLOAD` collects `len` bytes. An out-of-range length or a failed CRC discards the candidate
and advances one byte from its start, never trusting the length to skip. The payload is not
scanned for magic; CRC plus a timeout is the only judge. The assembly timeout is four times the
wire time of the largest allowed frame, with a 50 ms floor, and is derived from the negotiated
baud rather than fixed.

*Versioning.*

`major` is in every frame. `minor` and a 32-bit capability bitset travel in the session
handshake; capabilities are intersected and unknown bits are ignored. On a major mismatch the
receiver drops streams and events, ignores control frames, and answers commands with
`VERSION_MISMATCH`. The robot keeps streaming. The Bridge never rejects a frame on version.

*Namespaces.*

`class` is 2 bits and all four values are assigned in v1. `id` is a full 16 bits, allocated per
class: `0x0000` reserved, `0x0001–0x00FF` core, `0x0100–0x7FFF` extended, `0x8000–0xEFFF`
experimental, `0xF000–0xFFFF` reserved. Unknown ids are ignored for streams, events and control
frames, and rejected with `UNKNOWN_ID` for commands. The id table is a shared enum in the
protocol crate, so the app and the robot cannot drift.

*Delivery classes.*

| class | direction | delivery | seq scope | acknowledgement |
|---|---|---|---|---|
| stream | robot → PC | lossy, latest-wins | per stream id | never |
| command | PC → robot | reliable, at-most-once | per session epoch | stop-and-wait |
| event | robot → PC | reliable, at-least-once | per robot boot | mirrored |
| control | both | per frame | per sender when matched | only `HELLO`/`PING` |

An ACK is the original frame with `IS_ACK` set: same class, id and seq, payload = a status byte
plus optional result bytes. There is no separate ACK id, and `IS_ACK` reverses the normal
direction for commands and events.

Streams have one pending snapshot slot per stream. A newer snapshot replaces the pending one;
a dropped snapshot is never retransmitted. Commands are absolute and idempotent by construction
(drive-to-bearing, drive-to-pose, set-activity), never relative increments. The daemon keeps one
command in flight; the robot executes `last + 1`, re-ACKs `last` without re-executing, and
answers anything older with `STALE`. An unacknowledged command is *indeterminate*, not failed:
the daemon reconciles by querying state. Events use an eight-entry ring, are retransmitted until
the Remote ACKs them, and are deduplicated by `(boot_id, seq)`; ring overflow drops the oldest
and increments a counter reported in a later frame.

*Flow control.*

There is no RTS/CTS. Commands pace themselves with their single in-flight ACK. Telemetry uses
`THROTTLE`, a `CONTROL` frame with a 7-byte payload: stream id (`0xFFFF` = all), action
(`SET_RATE`/`PAUSE`/`RESUME`), rate in centi-Hz and a TTL. The Bridge is authoritative for UART
overflow and emits the hint on the robot-facing UART when its receive ring crosses a high
watermark; the daemon may also send one for network or UI pressure. The robot applies the most
restrictive active hint per stream and reverts to the last requested or baseline rate when the
TTL expires. The hint is advisory: if the robot cannot comply it ignores it, and the link
degrades to dropped frames, which the protocol already tolerates. TCP supplies
backpressure between the PC and the Bridge, so no credit accounting is needed.

*Session.*

The wire-level Session is a liveness lease, not a state machine. `HELLO` carries the major,
minor, capability bitset, a 32-bit epoch and requested stream rates; `HELLO_ACK` returns the
robot's versions, capabilities, a 32-bit boot id and its defaults. `PING` is answered by the
robot on the same seq. The daemon retries `HELLO` every second up to five times and sends no
commands until `HELLO_ACK` arrives. Any valid PC→robot frame renews the lease; the robot expires
the peer after 5 s of silence and keeps streaming at baseline. A new epoch resets command
dedupe; a new boot id tells the daemon to discard in-flight commands and re-handshake. The
daemon owns the Session and the Bridge enforces one client; the robot holds only link state.

*Codec.*

The codec is a shared `remote-protocol` crate: `#![no_std]`, no allocator, no HAL and no
embassy, with `heapless` as its only data-structure dependency and a `std` feature for host
tests. It exposes the header and frame encode/decode, the shared id/class/status enums, the CRC
and the reliability primitives (sequence, ACK tracking, dedupe, throttle). It is transport
agnostic: no UART, no clock, no logging. A `std` simulation of a lossy, truncating, stalling
link backs the host tests.

**Consequences**

- The Bridge stays an envelope-aware lossy pump. It can forward frames from a future major
  version, it never needs to understand payloads, and its only protocol-generated traffic is
  `THROTTLE`.
- RAM cost is bounded and small on both ends: roughly 3.6 KB of protocol buffers on the robot
  plus UART rings, and roughly 10 KB on the Bridge plus UART rings and the Wi-Fi heap.
- A maximum stream frame occupies the wire for one frame time, so a control frame waits at most
  one in-flight frame. That is acceptable for click-to-drive at the candidate bauds; emergency
  stop must not depend on this link.
- Commands are absolute by construction. This is a semantic requirement on every future command,
  not a convention: a duplicate or a post-reboot replay must be harmless.
- Event loss is visible rather than silent: ring overflow sets a dropped-events counter, and an
  event older than the retry window is discarded rather than replayed out of context.
- Throttle is advisory by design. If the robot ignores it, the link drops frames; it never
  blocks a producer or stalls the robot.
- The robot has no Session domain object. `epoch`, `boot_id`, `last_peer_frame`, `last_seq` and
  `last_status` are link state; per-stream and per-event counters are separate.
- Open items: the payload catalogue and id table, the baud and bandwidth budget, baseline
  stream rates, the configured frame timeout per baud, buffer and ring sizing, the Bridge's
  single-client enforcement, and command authority. The protocol crate's host tests are the
  acceptance gate for the reliability rules.
