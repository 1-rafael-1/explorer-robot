# bridge-core

The Bridge's HAL-agnostic rules, proven on the host without the C3. It is a member of the Bridge
workspace (`bridge/`); the operator page is
[docs/components/bridge.md](../../docs/components/bridge.md). The crate is `no_std`,
allocation-free and I/O-free: it owns no UART, socket, radio or clock. The firmware parses frames
with `link-protocol` and hands them here; `bridge-core` decides what crosses, what coalesces,
what is dropped and when to ask the robot to slow down. It also owns the Setup Mode protocol and
the credential record's codec, which need no HAL: the bounded window, the DHCP, DNS and portal
codecs, the versioned record and the pairing-code alphabet.

The risky forwarding rules are proven on the host rather than on the hardware. Its test suite runs
on `x86_64-unknown-linux-gnu` behind the `std` feature, and the same crate compiles for
`riscv32imc-unknown-none-elf` with the feature off.

## The four rule surfaces

1. **Coalescing** (`coalescing`) — a latest-wins ring of one pending slot per stream inside 8 KiB.
   A newer stream sample replaces the older one, so a Wi-Fi stall drops stale telemetry instead
   of queueing seconds of it. Reliable entries (events, command acknowledgements, control replies)
   are not coalesced; their room is reserved so stream pressure cannot take it. Every entry stores
   the wire frame's `Seq` and `Flags`, so the surviving entry carries the newest frame's header.
2. **Routing** (`routing`) — the Link Protocol's class rules plus exactly two *forwarding* control
   exceptions: `THROTTLE` out to the robot and `BRIDGE_CONTROL` in, consumed by the Bridge.
   `BRIDGE_STATUS` is Bridge-originated (ADR-0042) and is not a forwarding exception: the router
   ignores it. A `BRIDGE_CONTROL` arriving from the client is dropped and counted. A frame
   travelling the wrong way is ignored.
3. **Flow** (`flow`) — ring occupancy to `THROTTLE`: at 6 KiB **or** 12 queued entries the robot
   is told to `PAUSE(ALL_STREAMS, 1000 ms)`; it resumes only at or below 2 KiB **and** 4 entries,
   and the pause is refreshed every 500 ms while either mark stays above its low watermark. Hints
   are at least 500 ms apart. The hint is advisory and time-limited.
4. **Authentication** (`auth`) — the single-client gate. A connection has five seconds to
   authenticate, a new unauthenticated connection supersedes an older one, and exactly one
   authenticated client holds the slot. Five failed proofs within a minute close a one-minute
   refusal window; a success resets the counter. Comparison is constant time.

`pump` composes the four into `BridgePump`, the frame-level type the firmware wires to its HAL.

## Setup Mode protocol

- **`setup`** — the bounded window (ADR-0036): 10 minutes, closed ~30 s after a station rises
  while the window is open, raised by an unprovisioned boot, once per boot after five continuous
  minutes without a station link, or by a panel request. `SetupWindow` has no clock; the caller
  passes monotonic milliseconds.
- **`dhcp`** — a DHCPv4 server codec: DISCOVER/OFFER, REQUEST/ACK, a fixed two-address pool,
  release/decline/expiry, and the fixed reply options (message type, server id, lease, mask,
  router, DNS, broadcast). `handle` writes the reply into a caller buffer.
- **`dns`** — a single-question catch-all: A/ANY answers the AP's address, every other type is
  NOERROR with no answer, and EDNS/additional records are ignored.
- **`portal`** — HTTP request and form parsing, percent decoding, the best-of-N `ScanCache`
  (strongest RSSI per SSID), the status text, and the one-page HTML with inline CSS and the
  `fetch('/status')` poll. All writers take a caller buffer.

## The credential record and the pairing token

- **`record`** — the versioned, CRC-checked credential record (ADR-0036): magic `BRID`,
  version 1, flags, the station SSID and passphrase, the twelve-character pairing token and a
  CRC-16/CCITT-FALSE (the Link Protocol's CRC). `CredentialRecord::encode_into`/`decode` are the
  whole contract: a missing record, wrong magic, wrong version, bad CRC, bad flags, a credential
  length outside the WPA2-PSK range or a token outside the alphabet is an error the firmware turns
  into **unprovisioned** (fail closed into Setup Mode). A token-only record is valid and
  unprovisioned.
- **`pairing`** — the 60-bit pairing token as twelve unambiguous base32 characters (`0-9`, `A-Z`
  without `I`, `L`, `O`, `U`). `encode` masks a caller word to 60 bits and writes twelve characters
  most-significant chunk first; `write_grouped` renders `XXXX-XXXX-XXXX`. The characters' bytes are
  the HMAC key, so there is no decode step.

The firmware implements the `sequential-storage` `Value`/`Key` glue and the `esp-storage`
partition adapter; neither appears here, so the codec stays host-testable.

## The watchdog's liveness policy

- **`heartbeat`** — a fixed set of covered tasks, each owning one slot, plus the rule that a task
  is alive while its silence is at most the caller's maximum age. The firmware stamps beats from
  each task's own loop and the watchdog feeds the RTC watchdog only when `HeartbeatSet::stale`
  names no task. Wrapping subtraction keeps the rule correct across the `u32` millisecond
  boundary. `HeartbeatSet::seed` starts the window after the covered tasks are spawned.

A task can only be marked stale if it is covered, so the firmware covers a task only after its
spawn succeeds. No clock, timer, HAL or allocator is involved; everything is host-tested.

## Public API map

`BridgePump`:

| method | role |
|---|---|
| `new()` / `default()` | idle pump, empty ring, no client |
| `connect(now_ms) -> ConnectOutcome` | offer a new TCP connection to the gate |
| `authenticate(connection, proof, expected, now_ms) -> ProofOutcome` | present a proof |
| `tick(now_ms) -> Option<TimerEvent>` | expire the authentication deadline or the refusal window |
| `disconnect(connection) -> bool` | drop the current connection |
| `is_authenticated()` / `authenticated_connection()` / `current_connection()` | observe the gate (the last includes an in-flight authentication) |
| `on_robot_frame(&Frame) -> Decision` | route one UART-side frame |
| `on_client_frame(&Frame) -> Decision` | route one client-side frame |
| `poll_throttle(now_ms) -> Option<ThrottleHint>` | observe ring occupancy and return a hint |
| `offer_bridge_status(seq, payload) -> Decision` | queue the Bridge's own `BRIDGE_STATUS` for the daemon, coalesced as stream `0x0007`; `Ignored` without an authenticated client |
| `pop() -> Option<Entry>` / `drain(sink)` | take client-bound entries |
| `used_bytes()` / `used_entries()` / `reserved_bytes()` | ring occupancy and the event reservation |
| `stats() -> PumpStats` | the seven occupancy and drop counters as one snapshot |
| `dropped_streams()` / `dropped_reliable()` | ring discard counters |
| `dropped_without_client()` | robot frames dropped while no client was authenticated |
| `dropped_bridge_controls()` | client-side `BRIDGE_CONTROL` frames dropped |

Decision and state types:

- `Decision` — `Queued`, `DroppedNoClient`, `DroppedFull`, `BridgeControl(request)`,
  `ForwardToRobot`, `DroppedBridgeControl`, `Ignored`.
- `Entry { kind, payload }` and `EntryKind` — `Stream { id, seq, flags }` or
  `Reliable { class, id, seq, flags }`. The header fields are the parsed wire values.
- `Route` / `Destination` / `Side` — the routing filter's vocabulary.
- `ConnectOutcome`, `ProofOutcome`, `TimerEvent` — the gate's transitions.
- `Proof::from_bytes([u8; 32])` / `Proof::as_bytes()` / `Proof::matches()` — the opaque proof
  `bridge-net` computes.
- `WatermarkPolicy`, `PushOutcome`, `PushError`, `BridgeControlRequest`, `BridgeReply`, `Status`,
  `Class`, `Flags`, `Frame`, `Seq`, `Direction` — the remaining public surface.
- `HeartbeatSet`, `CoveredTask` — the watchdog's coverage and last-beat table and the tasks it
  can cover.

Constants are the ADR numbers, not tuning knobs: `CAPACITY` 8 KiB, `EVENT_RESERVED_BYTES` 512,
`STREAM_CAPACITY` 7.5 KiB, `CAPACITY_ENTRIES` 16, `HIGH_WATERMARK_BYTES` 6 KiB,
`LOW_WATERMARK_BYTES` 2 KiB, `HIGH_WATERMARK_ENTRIES` 12 (three quarters of the descriptor table),
`LOW_WATERMARK_ENTRIES` 4 (one quarter), `PAUSE_TTL_MS` 1000, `MIN_HINT_INTERVAL_MS` 500,
`DEADLINE_MS` 5000, `FAILURE_LIMIT` 5, `FAILURE_WINDOW_MS` 60_000 and `REFUSAL_MS` 60_000.

## Host tests

The tests use the explicit-target pattern so the host and firmware builds never share artifacts:

```sh
cd bridge
cargo test -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests
```

The suites are
`tests/{coalescing,routing,watermark,auth,pump,setup,dhcp,dns,portal,pairing,record,heartbeat}.rs`.
They exercise the crate's external contract — frames in, decisions and drained entries out — not
its internals. The
Bridge CI job also runs `cargo clippy -- -D warnings`, which applies this workspace's
pedantic/nursery table to `bridge-core` (the root `bridge-firmware` package deliberately does not
opt in because the pre-existing `stack-soak` spike is out of scope).

## What the firmware wires

The `bridge` binary keeps only HAL wiring:

- **UART RX**: UHCI RX DMA at 921600 8N1 (`GPIO4` TX, `GPIO3` RX) feeds
  `link-protocol::RobotToPcParser`; each frame goes to `pump.on_robot_frame()`. The UART is
  parsed regardless of a client, so `BRIDGE_CONTROL` and the Beacon's link-ready flag stay
  truthful with no Remote attached.
- **TCP RX**: one client at a time on the station stack. On accept the firmware calls
  `pump.connect()` and `pump.authenticate()`; on close it calls `pump.disconnect()` and drains
  what the client never read. Client bytes feed `PcToRobotParser`; each frame goes to
  `pump.on_client_frame()`, and `Decision::ForwardToRobot` is re-encoded into the bounded UART TX
  queue.
- **TCP TX**: `pump.pop()` is drained one entry per frame and re-encoded verbatim with the
  entry's stored class, flags, id, sequence and payload. The ring carries the wire header, so a
  forwarded frame keeps its original sequence number and flag bits; nothing is implied or reset.
- **Flow control**: `pump.poll_throttle()` watches both ring bytes and queued entries (the
  16-slot descriptor table fills before 6 KiB under the worst-case robot mix) and is encoded as a
  `THROTTLE` control frame into the same UART TX queue.
- **Timers**: `pump.tick()` expires the authentication deadline and the refusal window.
- **Status**: the ring occupancy and every drop counter are printed every ten seconds.

The firmware wires `bridge-net`'s HMAC preamble and the real single-client supersede path, the
setup window and the portal codecs to a second `embassy-net` stack, the credential record and the
robot-side `BRIDGE_CONTROL` replies; none of that changes the rules in this crate.
