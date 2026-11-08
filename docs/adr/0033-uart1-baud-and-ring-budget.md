# UART1 runs at 921600 8N1 with fixed rings and no cloud compression

ADR-0021 left the expansion UART at a provisional 115200 8N1. ADR-0031 then set the payload
catalogue, and the LiDAR cloud alone is 720 bytes at 10 Hz — 7.2 KB/s of payload, 7.3 KB/s
with the 12-byte header. At 115200 (11,520 B/s at 10 bits per byte) that stream is already
63% of the wire before any other stream, an event or a command. Three separate builds — robot,
Bridge, daemon — must therefore share one baud and one set of buffer sizes, and the numbers
have to survive the reserved `POSE`, `POSE_TRAIL`, `MAP` and `CAMERA_DETECTIONS` streams
without a protocol change.

**Status:** accepted. The baud, the frame timeout and the ring sizes are decided from the
budget now. Bench confirmation remains a verification gate (tickets 09 and 10), not a
precondition: if the harness shows signal integrity problems, the fallback is one constant,
not a protocol change.

**Considered Options**

- **Keep 115200.** Rejected. The cloud alone is 63% of the wire and the worst-case total is
  ~90%, leaving no room for events, commands or the reserved streams.
- **230400.** Rejected. ~45% at worst case and only ~20% headroom for the reserved catalogue;
  it is the same order of risk as 115200 with one more divisor.
- **460800.** Viable (~23% at worst case) and kept as the bench fallback, but a full 1036-byte
  frame occupies the wire for 22.5 ms, doubling the control/ACK wait, and the reserved
  catalogue could crowd it.
- **921600.** Accepted. ~11% at worst case today, the full 1036-byte frame is 11.2 ms, and a
  single 50 ms frame timeout covers both directions on both ends.
- **1.5–3 Mbps.** Rejected for v1. The gain over 921600 is irrelevant at 11% utilisation, and
  the port has no ferrite bead, no ESD clamp and 100 Ω series resistors into an unshielded
  pigtail; there is no reason to spend the signal-integrity margin before the bench says so.
- **Negotiate the baud on the wire.** Rejected. There is no framing until both ends agree on a
  bit rate, so a negotiation needs a second, slower protocol; a build-time constant shared by
  the three projects is simpler and a mismatch is silence, not corruption.
- **Coarsen or delta-encode the cloud.** Rejected for v1. At 921600 the cloud is ~8% of the
  wire, so the stateful encoder, keyframes and the `delta cloud` capability buy nothing. The
  capability bit stays reserved.

**Decision**

*Baud and frame timing.*

UART1 runs at **921600 8N1**, the same in both directions, fixed at build time in all three
projects.

| direction | max frame | wire time at 921600 | frame timeout (4×, 50 ms floor) |
|---|---|---|---|
| robot → PC | 1036 B | 11.2 ms | 50 ms |
| PC → robot | 268 B | 2.9 ms | 50 ms |

The 50 ms floor covers both directions at 921600, so a single constant serves the robot's
parser and the Bridge's parser. At 460800 the Bridge would need 90 ms, which is a second
constant for no benefit.

*The budget, robot → PC.*

Worst case: every change-based stream emitted at the 10 Hz sampling tick, eight events in the
ring retried four times a second, and control traffic. The event retry interval is a protocol
constant still to be fixed; 250 ms is the conservative placeholder.

| stream | payload | rate assumed | frame | bytes/s |
|---|---|---|---|---|
| `POWER` | 3 B | 10 Hz (change) | 15 | 150 |
| `MOTION` | 2 B | 10 Hz | 14 | 140 |
| `ACTIVITY` | ≤37 B | 10 Hz (change) | 49 | 490 |
| `PERCEPTION` | 1 B | 10 Hz | 13 | 130 |
| `LIDAR_CLOUD` | 720 B | 10 Hz | 732 | 7,320 |
| `CALIBRATION` | 4 B | 10 Hz (change) | 16 | 160 |
| events | ≤46 B | 8 × 4 Hz retry | 58 | 1,856 |
| control | small | — | — | 100 |
| **total** | | | | **≈10.3 KB/s** |

That is ~83 kbit/s of data and ~103 kbit/s on the wire.

| baud | wire capacity | utilisation |
|---|---|---|
| 115200 | 11,520 B/s | ~90% |
| 230400 | 23,040 B/s | ~45% |
| 460800 | 46,080 B/s | ~22% |
| **921600** | **92,160 B/s** | **~11%** |
| 1,500,000 | 150,000 B/s | ~7% |

PC → robot is dominated by nothing: commands are ≤52 B at human rates, event ACKs are 13 B,
and `HELLO`/`PING`/`THROTTLE` are occasional. The whole direction is under 1 KB/s. The baud is
symmetric because two divisors buy nothing and one constant is one fewer way to be wrong.

*Reserved headroom.*

Plausible additions — `POSE` at 20 Hz with a ~24 B frame (0.5 KB/s), a `POSE_TRAIL` at 20 Hz
with a ~128 B frame (2.6 KB/s), a `MAP` at 1 Hz with a 1 KiB frame (1.0 KB/s), and
`CAMERA_DETECTIONS` at 30 Hz with a ~64 B frame (1.9 KB/s) — add ~6.0 KB/s, taking the total
to ~16.3 KB/s or ~18% of 921600. The 1024-byte payload cap bounds every one of them: a reserved
stream running at the cap on every frame is a catalogue decision to revisit with rates, not a
baud that needs to be higher today.

*Buffers and rings.*

| location | size | why |
|---|---|---|
| robot UART1 RX (`BufferedUart`) | 4 KiB | ≈44 ms of PC→robot traffic; the task also wakes on RX, so this only covers scheduling stalls |
| robot UART1 TX (`BufferedUart`) | 2 KiB | holds a 1036-byte frame plus a control frame; a full max frame drains in 11.2 ms |
| robot `LinkCore` | ~3.6 KiB | ADR-0030's protocol buffers, unchanged |
| Bridge robot-facing RX | 8 KiB coalescing ring | one frame per stream plus the event ring; latest-wins means state, not a backlog |
| Bridge UHCI DMA buffer | 4 KiB | ≈43 ms of DMA slack; the esp-hal UHCI example uses 4092 B |
| Bridge robot-facing TX | 1 KiB | PC→robot commands are ≤268 B and rare |
| daemon | OS socket buffers | no fixed ring |

The Bridge's buffering is **coalescing, not delay**: 8 KiB is ~89 ms of raw arrival at
921600, but because a newer snapshot replaces the pending one per stream, the ring holds the
current state of six streams plus events, not seconds of history. A Wi-Fi stall longer than
that fills the ring, the Bridge emits `THROTTLE`, and the robot coalesces or drops; the link is
designed to lose streams. Reliable events are the only thing that must survive, and eight
entries fit in under 0.5 KiB, so the Bridge reserves room for them before it drops a stream.

The ring's descriptor table is also a limit: it holds 16 entries, and the robot's worst-case mix
(six coalesced streams plus many small reliable events) reaches that limit at roughly 2–3 KiB,
well below the 6 KiB byte watermark. A byte-only policy therefore drops frames without ever
asking the robot to slow down — ticket 14's induced-stall run caught exactly that
(`ring_entries=16`, `ring_used=736B`, no hint). The Bridge now watches both dimensions: it pauses
at 12 entries or 6 KiB and resumes only below 4 entries **and** 2 KiB. The byte watermarks are
unchanged; the entry watermarks are the descriptor table's three-quarters and one-quarter marks.

*The ESP32-C3 side.*

`esp-hal` 1.2.2 validates UART baud rates up to **5 Mbps**, so 921600 is 18% of the API
ceiling. The C3's UART FIFO is 128 bytes — 1.4 ms of slack at 921600 — and `esp-hal` ships
UHCI DMA with multi-KiB buffers. Interrupt-driven service is sufficient at this rate; UHCI is
recommended anyway because the Wi-Fi stack can hold a task for longer than the FIFO slack.
Ticket 09 has not yet run its UART soak, so this is the API/datasheet fact, not a measurement.

*Robot duty.*

10.3 KB/s at 921600 is 11% wire duty. The `BufferedUart` ISR drains or fills the full 32-byte
PL011 FIFO per interrupt, so a continuously busy TX is ~2,900 interrupts/s and the link task's
own encoding work is tens of microseconds per tick: well under 1% CPU on core0.

*No compression.*

The cloud stays the crate's 360-slot, whole-centimetre grid at 10 Hz. Coarsening would lose the
resolution the radar view needs, and delta encoding would add a stateful encoder, keyframe
policy and a capability bit to save ~8% of a link that is 89% idle. Revisit if a reserved
stream lands large enough to make the arithmetic uncomfortable; the first lever is the stream's
rate, then delta encoding, then the baud.

**Consequences**

- A 1036-byte cloud frame delays a queued control frame or ACK by at most 11.2 ms. Emergency
  stop still must not depend on the link (ticket 06).
- One frame timeout of 50 ms serves every parser at this baud; the timeout is still derived
  from the baud so the fallback stays correct.
- 460800 is a one-constant fallback for the bench. If it is ever taken, the Bridge's timeout
  becomes 90 ms and the control wait 22.5 ms; the protocol is unchanged.
- The rings are fixed sizes, not negotiated; the Bridge's are its own implementation choice
  within its RAM budget (`esp-bridge` owns its allocator and heap size).
- The reserved streams have room to land at plausible rates without touching the baud; a
  pathological stream at the 1024-byte cap on every frame is a catalogue problem.

**Open items**

- Bench-confirm the arithmetic and signal integrity on the harness (ticket 10): 10 minutes of
  sustained worst-case traffic at 921600, and a scope of `EXP_TX`/`EXP_RX` at the connector.
- Confirm the C3's sustained UART receive and UHCI behaviour under a Wi-Fi stall (ticket 09).
- Fix the event retry interval in `remote-protocol`; the budget assumes 250 ms.
- Re-check the budget if `POSE_TRAIL`, `MAP` or `CAMERA_DETECTIONS` gain payload layouts and
  rates.

**Amendment (2026-11-08, ticket 15).** The Bridge-originated `BRIDGE_STATUS` adds +28 B/s of
payload robot→PC (40 B/s on the wire with the 12-byte header, at 1 Hz) and ~40 B/s PC→robot as a
control frame (28 B plus the header). The daemon-side status is coalesced in the existing 8 KiB
ring, so it adds no backlog and no new buffer. No ring size or baud changes; the budget's ~11%
utilisation moves by well under a tenth of a percent (ADR-0042).
