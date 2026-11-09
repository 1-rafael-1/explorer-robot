# Bridge telemetry: one 1 Hz snapshot on two delivery classes

The Bridge is the only component that knows the Wi-Fi's RSSI, channel and association state, and
its own UART, DMA and watchdog health. ADR-0030 made it an envelope-aware forwarder: it emits
`THROTTLE` and consumes `BRIDGE_CONTROL` (ADR-0036), but it originates no payload of its own. The
fact that the Bridge already prints a rich status line every 10 s over USB and puts none of it on
the wire is the gap this decision closes.

Two consumers want that state for different reasons. The robot panel will show Bridge presence,
Wi-Fi quality and a System Info link section once the Link Task samples it, but the robot's Link
Task is blocked on the mainboard. The daemon and PC app want the same facts now, and the daemon
already owns an authenticated TCP connection to the Bridge (ADR-0037), so the telemetry adds no
robot relay. The app still needs the existing Session (`HELLO_ACK`), supplied on the bench by the
Robot Stub and on the real robot by the future Link Task.

Three existing rules constrain the shape. ADR-0031's catalogue is robot-originated, so the
Bridge's status cannot simply be appended to the robot's six-entry `Stream` catalogue. ADR-0033
sized the 921600 link at ~11% worst-case utilisation, which leaves room for a trickle but not for
a second telemetry catalogue. ADR-0030 renews the lease on any valid PC→robot frame, ADR-0035
arms the Deadman on the same timestamp, and ADR-0039 declares the robot stalled after 3 s without
a decoded robot frame. A Bridge-local 1 Hz frame in the PC→robot direction would keep the lease
alive and hide a dead robot unless the origin is accounted for.

**Status:** accepted. The Bridge publishes one `BRIDGE_STATUS` snapshot at 1 Hz from the same
28-byte payload to two consumers with the class chosen per direction: `control` `0x0005` to the
robot, `stream` `0x0007` to the daemon. The robot does not relay it. The lease (ADR-0030, amended
with ADR-0035) and the daemon's stall timer are narrowed to Remote-originated and robot-originated
frames respectively, so a healthy Bridge cannot mask a dead Remote or a dead robot.

**Considered Options**

- **Robot-relayed stream (Bridge → robot → daemon).** Rejected. It makes the PC path depend on the
  robot's Link Task for the relay and on the mainboard, adds a second hop and a second failure mode, and turns the
  robot into a relay for payloads it does not interpret. The Bridge already has the daemon's TCP
  connection; forwarding through the robot buys nothing.
- **A `BRIDGE_CONTROL`-style request/response.** Rejected. The data is periodic state, not an
  operator action: a poll adds a request in flight, a latency floor and per-field reply statuses,
  and the daemon would poll at the same 1 Hz anyway. Latest-wins coalescing is exactly the stream
  delivery class, not an exchange.
- **Control-only to both consumers.** Rejected. A periodic state snapshot would lose the stream
  class's per-id coalescing slot in the existing 8 KiB ring and force the daemon's relay to treat
  control frames as state. Control also has per-frame sequencing, which this payload does not want.
- **Bridge → daemon direct as a stream, plus control to the robot.** Accepted. It gives the PC the
  data without a robot relay (still within the existing Session), and gives the future panel a local
  copy that survives no-Remote periods.

**Decision**

*One payload, two ids.*

| consumer | direction | class | id | delivery |
|---|---|---|---|---|
| robot (panel / Link Task, future) | Bridge → robot | control | `0x0005` `BRIDGE_STATUS` | 1 Hz, lossy, unacknowledged |
| daemon → PC app | Bridge → PC | stream | `0x0007` `BRIDGE_STATUS` | 1 Hz, latest-wins in the 8 KiB ring |

The same 28-byte payload travels both ways:

| offset | field | encoding |
|---|---|---|
| 0 | `rssi_dbm` | `i8`; `0x80` (`i8::MIN`) = unknown |
| 1 | `channel` | `u8`; `0` = unknown |
| 2 | `station` | `0` down, `1` connecting, `2` associated |
| 3 | `flags` | bit 0 provisioned, bit 1 setup open, bit 2 link ready; bits 3–7 reserved |
| 4 | `uart_dropped_bytes` | `u32` little-endian |
| 8 | `dma_stalls` | `u32` little-endian |
| 12 | `crc_failures` | `u32` little-endian |
| 16 | `resyncs` | `u32` little-endian |
| 20 | `wdt_feeds` | `u32` little-endian |
| 24 | `wdt_withheld` | `u32` little-endian |

All multi-byte fields are little-endian and every byte is present; there is no optional field and
no length prefix beyond the envelope's `len`. The counters are cumulative since the Bridge's boot,
not deltas. Unknown RSSI and channel use reserved sentinels rather than a validity bit, matching
ADR-0031's `POWER` convention; reserved flag bits are sent as zero and cleared by the decoder.

*Origination and routing.*

The Bridge is the originator for both frames. `0x0007` is a Bridge-originated stream, not a robot
stream the Bridge happens to carry: it is deliberately not a variant of the robot's `Stream` enum,
so the robot's catalogue stays exactly six entries and `remote-link`'s stream slots stay
robot-shaped. The robot's `remote-link::LinkCore` decodes the control snapshot, stores it and
exposes `bridge_status()` for the future panel; it does not re-emit the payload. The daemon decodes
the stream and relays it to the app as part of its snapshot. The Bridge remains an envelope-aware
forwarder for everything else, but it now samples and encodes one payload of its own.

*The lease and the Deadman.*

ADR-0030's "any valid PC→robot frame renews the lease" and ADR-0035's Deadman are narrowed to
**Remote-originated** frames:

| renews the lease / re-arms the Deadman | does not |
|---|---|
| `HELLO`, `PING` | `THROTTLE` |
| commands | `BRIDGE_CONTROL`'s ACK |
| event ACKs | `BRIDGE_STATUS` |

`BRIDGE_STATUS` is Bridge-originated; without this rule a 1 Hz status would keep a dead Remote's
lease alive and disarm the Deadman (ADR-0035). `BRIDGE_CONTROL`'s ACK is also the Bridge's.
`THROTTLE` may be synthesised by the Bridge as well as sent by the daemon, and the robot cannot
tell the origin from the frame, so the id is excluded outright; the daemon's 1 Hz `PING` already
renews an idle but healthy Session, so excluding `THROTTLE` costs nothing. Event ACKs remain Remote
traffic: `IS_ACK` reverses the normal direction for events.

*The daemon's stall timer.*

ADR-0039 declares the robot stalled after 3 s without a decoded **robot** frame. The Bridge's
stream arrives on the same TCP connection, so it must not reset that timer: counting it would let a
healthy Bridge hide a dead robot indefinitely. The daemon relays the status to the app but keeps
the reconnect timer keyed to robot-originated frames only. After 3 s of robot silence the daemon
reconnects as ADR-0039 says, so the app shows `reconnecting`; the last Bridge status remains in the
app's state with its `ts` until a new Session, exposing staleness rather than claiming continuous
delivery across a robot outage.

*Budget.*

At 1 Hz the payload is 28 B/s each way. With the 12-byte header the frames are 40 B each, so the
addition is +28 B/s of payload (40 B/s on the wire) robot→PC and ~40 B/s PC→robot, which together
are well under a tenth of a percent of 921600. The daemon-side status is coalesced in the existing
8 KiB latest-wins ring, so a Wi-Fi stall drops stale snapshots rather than queueing them; the
robot-side control is one 40-byte frame in the PC→robot direction, which keeps that whole
direction under 1 KB/s. No ring size or baud changes (ADR-0033).

**Consequences**

- The daemon can decode and relay Bridge Wi-Fi and resilience state as soon as the Bridge is
  connected and the Session is open, because the Bridge talks to the daemon directly; the Link Task
  is still required to open that Session on the real robot, so the robot panel is a follow-up, not a
  dependency.
- One payload and two ids is the smallest change that satisfies both consumers: the robot's `Stream`
  catalogue stays six, and the daemon and robot share one 28-byte codec.
- The Bridge is no longer payload-agnostic for this one message. It must sample and encode its own
  state; it remains a forwarder for every other payload.
- The lease and Deadman rules gain an origin distinction. That is a semantic narrowing, not a new
  protocol element: the robot has to keep a list of renewing control ids rather than treating
  every valid PC→robot frame alike.
- The daemon's relay grows a Bridge slot and an app-facing message, and its stall timer becomes
  origin-aware. The app's SPA is still a placeholder, so only the daemon JSON contract lands now.
- `LinkCore::bridge_status()` stores the latest snapshot even with no Remote connected, so the panel
  can show Bridge state during a no-Session period, when it matters most.
- A lost snapshot is replaced by the next one, exactly like any stream; the control side gets no
  acknowledgement and no retry, and the counters are cumulative so one sample is self-contained.

**Open items**

- Sample the control snapshot in `task::remote_link`, read `LinkCore::bridge_status()` and publish it
  into a `state::link` module for the panel.
- Add the panel's `Link` screen, header connectivity/battery indicator and System Info link section.
- Render the daemon-side status in the SPA once it stops being a placeholder; the app JSON contract
  (`Snapshot.bridge` / `ServerMessage::Bridge`) is in place and needs no further daemon change.
- Bench-confirm the 1 Hz cadence and the counter meanings, and keep the Bridge's USB status line
  and the wire snapshot reading the same source. The budget change is small enough that no ring or
  baud re-check is needed beyond the existing ticket 10/14 gates.
- If a second Bridge-originated stream appears, give it its own `BridgeStream` id; do not extend the
  robot's `Stream` enum.
