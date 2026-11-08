# The robot's Link Task: one lifecycle owner, sampled telemetry, sealed dispatch

The robot side of the Link Protocol (ADR-0030) and its payload catalogue (ADR-0031) are
settled, but the robot has no code for them yet: `ExpansionResources` exists in
`src/resources.rs` and `main.rs` deliberately leaves it untouched. This decision shapes the
task that will own that hardware — its power/UART lifecycle, how it gets telemetry, how it
gets commands, where it runs, and what is host-testable.

The constraints are in the ADRs it consumes. ADR-0023 requires the branch's power and UART to
be sequenced by their single owner, with no muxed UART left behind while the branch powers
down. ADR-0030 forbids a producer from awaiting the link and forbids a Session object on the
robot. ADR-0031 says telemetry comes from the existing state modules and events from a curated
mapping. The robot has no hardware-in-the-loop rig, so anything that is not UART, power or
embassy should be testable on the host (ADR-0030 already chooses a `remote-protocol` crate).

**Status:** accepted. No code is written by this decision; it specifies the task that the
robot-side transport effort builds. The baud and ring sizes are ADR-0033's, and command
authority is settled in ADR-0034 (one Active Claim: anyone may stop, starts need a free claim).
ADR-0035 gives `LinkCore::tick` the peer-silence **Deadman** and has the Link Task call the
robot's one failsafe entry when it fires. ADR-0036 makes the Bridge a peer for `BRIDGE_CONTROL`
and gives it a credential record and a bounded Setup Mode; the Link Task gains the panel-triggered
setup action and its ACK. ADR-0037 makes the Bridge authenticate the one Remote against a pairing
token and broadcast discovery; the robot remains unaware of both.

**Considered Options**

- **Split bring-up from the protocol.** Rejected. ADR-0023's invariant is that the muxed UART
  and the branch enable have one owner; a protocol task that does not own either cannot
  guarantee the teardown order, and a second owner for the pads duplicates the LiDAR's
  sequence.
- **Let producers push stream snapshots.** Rejected. It either blocks a producer on the link
  or forces every producer to know about the wire. The state modules already hold everything
  ADR-0031 exposes, and sampling them keeps one copy of each truth.
- **Subscribe the link to the event bus.** Rejected. `EVENT_CHANNEL` is single-consumer by
  design; a second reader means changing the bus and giving the link a domain view it does not
  need. The orchestrator forwards the curated subset instead, which is what "sensor events →
  logged or forwarded" already means.
- **Submit drive commands from the Link Task and await the queue.** Rejected. A drive queue
  takes seconds; awaiting it would stall RX, ACKs and the 5 s lease for the whole drive. A
  single long-lived job task owns the await and publishes the completion.
- **Run the Link Task on core1 beside the LiDAR.** Rejected. It gives no latency win — the
  UART's interrupt ring absorbs task jitter — and costs a larger core1 stack plus cross-core
  reads of the state modules. Core0 has the room for the small amount of CPU this task uses.
- **Move to `postcard` or `ergot`.** Rejected. The frozen payload layouts are the contract and
  are small enough to hand-write; a derive would hide ADR-0031's "did you expose this variant"
  decision, and `ergot` is a whole RPC model that the envelope-only Bridge cannot forward.
- **A thin codec-only crate, with the task loop in the firmware.** Rejected as the only
  boundary. The parser, dedupe, event ring, coalescing and lease are where the bugs are and
  are exactly what a simulated link proves; only the UART, power, sampling and mapping need to
  stay in the binary.

**Decision**

*Naming.*

`expansion` is retired as a firmware name; it said nothing when the connector had no purpose
and it is wrong now that it carries the Remote Link. Firmware calls the thing by its job:

- **Link Port** — the connector and switched 5 V branch; `RemoteLinkResources` in
  `src/resources.rs` (renamed from `ExpansionResources`, unused today).
- **Link Task** — the core0 task and its module, `task::remote_link`; the retained-resource
  owner is `RemoteLinkSession`.
- The PCB and ADR net names stay `EXP_*` (`EXP_PWR`, `EXP_EN`, `EXP_BOOT`, `EXP_IRQ`); a
  schematic rename would be a board change with no firmware benefit.

*One owner, one lifetime.*

`RemoteLinkSession` mirrors `LidarSession`: it retains `UART1`, the TX/RX pads (GPIO 36/37),
the `EXP_PWR`, `EXP_EN` and `EXP_BOOT` outputs, the `EXP_IRQ` input, the static TX/RX rings and
`last_power_down`. It constructs no UART at boot.

Power-up (ADR-0023): wait out the minimum off interval, configure RX with `Pull::Up` and
`EXP_IRQ` with `Pull::Down`, assert `EXP_PWR`, wait the TPS22917 rise (`BRANCH_RISE`, 50 ms
provisional), drive `EXP_EN` high, then build the `BufferedUart`.

Power-down: stop the protocol (there is no device stop command — only the local TX path),
bounded-drain the shifter, drop the `BufferedUart` (deinit rings and IRQ), release both pads to
inputs with no pull, clear `UARTCR.EN`/`TXE`/`RXE`, drive and hold `EXP_EN` low, hold
`EXP_BOOT` low, drop `EXP_IRQ` to an input with no pull, then deassert `EXP_PWR` and record the
instant. Every wait is bounded.

The task powers the branch at boot and leaves it powered. A one-way command channel
(`Channel<Command, 4>`) accepts `Enable` and `Disable`, with the same inert-duplicate
semantics as the LiDAR's (ADR-0028): enabling while powered and disabling while off do
nothing, there is no reference count, and a caller that powers it off must power it back on. A
forgotten disable stays off — visible immediately, because the Remote loses its session. A
lock-free `status()` reports `Off`, `WaitingForBoard` or `Ready`.

`Disable` resets the link state: parser, pending stream slots, event ring, command dedupe and
lease. The next `Enable` mints a new **Boot Id**, so the daemon discards in-flight commands and
re-handshakes, and the event sequence can restart without colliding with the daemon's
`(boot_id, seq)` dedupe. This refines ADR-0030's "per robot boot": a Boot Id follows a *powered
link period*, which is a robot boot unless the port was deliberately powered down. A
presence-only recovery (below) does **not** mint one — the link state survives, so a retried
command is still recognised as a duplicate.

*Presence.*

`EXP_IRQ` is a **Bridge-ready presence line**, level-high. The Bridge drives it high once its
firmware has booted and its UART is up, before Wi-Fi joins: presence is the board, not the
network. The robot samples a level, not an edge, after a `PRESENCE_GRACE` of 1 s (provisional),
because a robot that reboots while the Bridge is already running never sees a low.

- `Ready` means the board is attached. It does not mean a Remote is connected; only a valid
  `HELLO` and the lease prove that. A present board with no handshake is a normal idle link:
  stream at baseline, log nothing, show nothing.
- A high→low edge while `Ready` means the Bridge reset: reset the RX parser, drop a partially
  written TX frame, and return to `WaitingForBoard`. Command dedupe and the event ring are
  **kept** — the Remote's epoch and the robot's Boot Id are unchanged, so a retried command
  must still be recognised as a duplicate and its pending events retransmitted after the
  reconnect.
- If the line is still low after the grace, reset the Bridge once with a 10 ms `EXP_EN` pulse
  (the concrete form of "power-cycle once") and wait another grace. Still low means absent: no
  further retry, no error spam.
- Presence **cannot** gate `EXP_PWR` — detection needs power. The branch is powered whenever
  the link is enabled; an empty connector draws nothing. The in-band `THROTTLE` owns flow
  control, so `EXP_IRQ` is reserved for presence in v1 rather than carrying two meanings.

While not `Ready`, the task keeps sampling and updating pending slots but suppresses
transmission. Events accumulate in the bounded ring (overflow counted as ADR-0030 requires)
and are retransmitted when the board appears.

*Output: streams.*

The task ticks at 10 Hz and samples the state modules, one lock at a time, in the documented
lock order (power → calibration → perception → motion → activity). Nothing takes two locks at
once, so the order cannot cycle. Rates are ADR-0031's: `MOTION` 10 Hz moving / 1 Hz idle,
`PERCEPTION` and `LIDAR_CLOUD` 10 Hz, `ACTIVITY` on change + 1 Hz, `POWER` and `CALIBRATION` on
change + 5 s. Emission is on change or heartbeat; latest-wins coalescing is the pending slot in
`LinkCore`. The cloud is encoded inside `perception::with_lidar` so the 720-byte array is never
cloned, and is skipped when the sensor's sequence has not advanced.

*Output: events.*

The orchestrator, the event bus's only consumer, maps `Events` with an exhaustive
`event_to_wire(&Events) -> Option<WireEvent>` and forwards the exposed ones into the link's
event channel with `try_send` plus an atomic dropped counter; it never blocks. Drive
completions do not travel on the bus, so the drive-job task publishes them directly into the
same channel with the originating `command_seq`. The mapping functions live in
`src/task/remote_link/` because they name firmware enums; their guarantee is compile-time.

*Input.*

The transport is `BufferedUart` on UART1 with interrupt rings — the path the LiDAR already
proves — with 4 KiB RX and 2 KiB TX (ADR-0033). `UART1_IRQ` moves from
`UartInterruptHandler<UART1>` to `BufferedInterruptHandler<UART1>`.

Envelope, CRC, class/id, payload decode and per-payload range checks live in
`remote-protocol`. Semantic checks — Procedure availability, family busy, drive queue busy —
live in the firmware, because only it knows the robot. The reply statuses are the crate's:
`VERSION_MISMATCH`, `UNKNOWN_ID`, `BAD_ARGUMENT`, `BAD_LENGTH`, `BUSY`, `REJECTED`, `STALE`.

*Dispatch.*

There is one vocabulary, and the existing entry paths are the seam:

- **Procedure start/stop** — `procedure::start(Procedure) -> StartOutcome` and
  `procedure::stop()` join `task::procedure`, moving the per-Procedure dispatch that today sits
  in `ui::handle_menu_item`, the `start_*` helpers and `stop_running`. The panel tap and the
  Link Task become two callers of one function. Coast-and-avoid's slow `start()` keeps running
  off the caller (its existing request channel), so the Link Task never awaits a LiDAR warm-up.
  `AttemptStraight` and `DistanceCalibration` are not remote-startable and answer `REJECTED`,
  matching the catalogue's `available` flag. What the panel shows while a Remote starts
  something stays ticket 05's question; this decision only makes Activity State truthful.
- **`STOP_MOTION`** — maps `brake` to `send_drive_interrupt`: `EmergencyBrake` for brake,
  `Stop` for coast. A `Signal`, so it never awaits.
- **`DRIVE_DISTANCE` / `ROTATE_BY`** — handed to a single long-lived `remote_drive_job` task
  that builds a one-step `DriveQueueBuilder`, awaits `submit()`, and publishes the completion.
  The Link Task ACKs at hand-off (`OK`, or `BUSY` when the queue refuses), and the completion
  event carries the command's seq. The job being single, a second drive command is `BUSY`
  rather than a second queue.

*Scheduling.*

Core0, the thread executor, spawned with the other init functions. The task never busy-waits,
never holds a state lock across an await, and its heaviest work is encoding a 720-byte cloud
plus CRC. 4 KiB of RX ring absorbs task-scheduling jitter: ~89 ms at 460800 and ~44 ms at
921600. Embassy's thread executor has no priorities, so the guarantee is cooperative yielding
plus the interrupt ring, not a priority number. If RX latency ever proves late on the bench,
the escape hatch is a dedicated interrupt executor for the UART drain, not a core move.

*Testability.*

Two crates, matching the repo's standing preference for HAL-agnostic, host-tested logic:

- **`remote-protocol`** — shared by the robot, the Bridge and the daemon: envelope, CRC,
  ids/classes/statuses, payload codecs, reliability primitives, and the `std` simulated link.
- **`remote-link`** — robot-side and new: a `LinkCore` owning the RX parser, pending stream
  slots, the event ring, lease expiry, command dedupe and TX priority, driven with plain `u64`
  milliseconds rather than a clock. Its surface is roughly `on_rx(bytes)`, `tick(now)`,
  `publish_stream(id, payload)`, `publish_event(event)` and `next_frame()`.

The firmware keeps the UART and power lifecycle, the sampler, the mapping functions and the
dispatch. Host tests cover resynchronisation, duplicate and retransmitted commands, lost ACKs,
event-ring overflow, throttle TTL, version mismatch and Boot Id changes against a lossy,
truncating, stalling simulated link. Codecs stay hand-written: no `postcard`, no `ergot`.

**Consequences**

- The robot gains exactly one place that drives `EXP_PWR`/`EXP_EN`/`EXP_BOOT` and owns UART1,
  so ADR-0023 holds by construction and a test mode can power the port down with one call.
- Powering the port off and on is a deliberate link-state reset: new Boot Id, cleared dedupe,
  dropped pending events. The daemon sees it as a robot restart, which is what it is.
- A Bridge reset is not a link reset: dedupe and the event ring survive, so a retried command
  cannot re-execute a volatile motion command.
- Telemetry cannot drift from the panel: both read the same state modules, and adding a
  producer needs no link work.
- `ui/mod.rs` loses its per-Procedure dispatch to `task/procedure`; that is a refactor of
  existing code, not a second command vocabulary.
- Procedure start/stop contention and who wins between the two operators remain ticket 05's
  decision; this decision only guarantees there is one path to contend over.
- The baud and ring sizes are settled in ADR-0033; the 50 ms rise, 200 ms off interval,
  1 s presence grace and 10 ms reset pulse are provisional pending the board.

**Open items**

- Implement `remote-protocol`, `remote-link` and `src/task/remote_link/`; the workspace/CI
  wiring is ticket 14's.
- Change `Events::ObstacleDetected` to carry `detected` before writing the mapping (ADR-0031).
- Add the mapping functions and the definition-site comments once the components exist
  (ADR-0031, ticket 18).
- Measure the TPS22917 rise, the C3's boot-to-UART-ready time and the off-state back-feed when
  the board exists; confirm or replace `BRANCH_RISE`, `POWER_CYCLE_OFF` and `PRESENCE_GRACE`.
- Confirm the C3's `EXP_IRQ` GPIO and that it asserts before the Wi-Fi join, in ticket 09's
  Bridge work.
- Revisit the single-client and takeover behaviour in tickets 05 and 15; this task only
  exposes the seam.

**Amendment (2026-11-08, ticket 15).** The Link Task also publishes `LinkCore::bridge_status()` into
`state::link` for the panel. It does not re-emit the payload; the Bridge sends the daemon stream
directly (ADR-0042). The panel's `Link` screen and header indicator extend ADR-0036.
