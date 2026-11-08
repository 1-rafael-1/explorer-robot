# Failsafe on link loss: a peer-silence deadman that stops only Remote-originated work

The Link Protocol (ADR-0030) gives the Remote a liveness lease but no authority, and ADR-0034 made
authority exclusivity while deliberately leaving "what happens to motion when the link dies" open.
Two situations now exist that did not before: work can be started from across the room, and the PC
can disappear while the robot is moving.

The robot already has local safety edges. The obstacle and floor-drop handlers send
`EmergencyBrake` unconditionally (`src/task/behavior/obstacle.rs`, `floor_drop.rs`), which cancels
the active intent, bumps the epoch and drains the queue. Those edges cover the *world* — a wall, a
ledge — not the *operator*. A robot whose teleoperator has gone quiet will happily keep coasting,
turning or avoiding obstacles with nobody watching it.

The link offers only one observable on the robot: silence. Any valid PC→robot frame renews the
lease, the robot holds no Session object, and `EXP_IRQ` says the Bridge board is attached, not that
a Remote is connected (ADR-0032). The question is therefore not "detect a Remote" but "what does
the robot infer from silence, and what does it stop".

**Status:** accepted. A **Deadman** — a 2 s peer-silence timer in the Link Task's `LinkCore::tick`
— stops **Remote-originated** claims with `EmergencyBrake`, releasing them through the one stop
verb. It never touches panel-originated work and never uses presence. The remote E-stop is
`STOP_MOTION` with the brake flag, not a new command. It narrows one sentence of ADR-0034: the stop
verb latches and interrupts but no longer writes Activity, so a failsafe abort can hold a Result
Report; the claim gains a transient `Panel`/`Remote` origin used only by the deadman, never for
authority. No firmware is written by this decision.

**Considered Options**

- **`EXP_IRQ` presence as the deadman signal.** Rejected. Presence is the board, not the Remote,
  it is sampled after power rather than streamed, and ADR-0032 explicitly says a Bridge reset is
  not a link loss. A present-but-silent board is a normal idle link, and a reset must not stop
  motion.
- **The 5 s lease as the timeout.** Rejected as the deadman figure. Five seconds is a long time
  for an unattended robot at teleop speed, and it binds one mechanism to two jobs. The lease stays
  the session's liveness and now also *disarms* the deadman; the timeout is a shorter threshold on
  the same `last_peer_frame`.
- **A dedicated motion-heartbeat command.** Rejected. `PING` exists, and the lease already forces
  the daemon to keep sending one; a second id would be duplication on a 12-byte budget that is not
  the problem. The daemon's obligation is a rate, not a new frame.
- **Stop all motion whenever a Remote is attached.** Rejected. The panel is the primary operator
  surface. A laptop sleeping must not abort a bench procedure the person standing at the robot is
  watching, and ADR-0034's whole point is that the panel is never subordinated to the Remote.
- **Stop only unbounded or interactive motion.** Rejected. It needs a per-Procedure "safe to keep
  moving" property and still leaves a long finite Remote command running unattended. The flat rule
  — any Remote-originated claim — is one line, matches a teleoperator's expectation, and needs no
  second semantic.
- **Stop only Manual Drive.** Rejected. In v1 there is no continuous teleop command —
  `SET_DIFFERENTIAL` is reserved and `DRIVE_DISTANCE`/`ROTATE_BY` are finite one-shots. The only
  unbounded Remote motion is coast-and-avoid, so this shape would make the deadman nearly inert.
- **Coast (`InterruptKind::Stop`) on loss.** Rejected. No operator is present to choose brake or
  coast, and a coast leaves the robot rolling on a slope. `EmergencyBrake` is the failing-safe
  choice and the same response as an obstacle.
- **`abandon` to Idle.** Rejected. It clears Activity and leaves nothing to review; the ticket
  requires the Result Report to survive on the panel.
- **A distinct remote E-stop command.** Rejected. It travels the same lossy wire it cannot outrun,
  and its only possible extra semantic — a latch that refuses new starts — is a lockout ADR-0034
  deliberately does not have.
- **A separate safety-supervisor task.** Rejected. `LinkCore` already owns the lease tick, is
  driven with plain milliseconds and is host-testable; a second owner of link liveness would be a
  second truth.
- **A peer-silence deadman on Remote-originated claims.** Accepted.

**Decision**

*Terms.*

- **Deadman** — the robot's 2 s peer-silence timer. See `CONTEXT.md`.
- **Link Loss** — the Deadman's verdict: a Remote-originated claim is live and no valid PC→robot
  frame has arrived for the timeout.

*Arming.*

The robot keeps a three-state view of the peer, seeded by ADR-0030's `last_peer_frame`:

| state | meaning | deadman |
|---|---|---|
| `Detached` | no valid PC→robot frame since the link was enabled | inert |
| `Attached` | a frame has been seen and `last_peer_frame` is within 2 s | armed |
| `peer_lost` edge | 2 s of silence while a Remote-originated claim is live | fires once |

The 5 s lease keeps its own meaning: it expires the Session and returns the peer to `Detached`;
the deadman has already fired by then. A frame is processed before the tick, so a reconnect on the
same epoch re-attaches without firing. `Detached` is why a robot with no Remote runs panel
procedures normally, and why a passing laptop visit cannot poison the next bench session.

The timeout is a threshold on frames, not a protocol element: the daemon sends `PING` at 1 Hz
while a Session is open. No command, event or capability bit is added.

*Scope.*

The deadman applies to a claim whose origin is `Remote`:

| claim | origin |
|---|---|
| `Drive` (Manual Drive) | always `Remote` |
| `Procedure(P)` started by `START_PROCEDURE` | `Remote` |
| `Procedure(P)` started by the panel | `Panel` |
| `Boot` | `Panel` |
| `Screen(RoomScan)` | `Panel` |

The origin is a transient tag on the claim, cleared with it. It is not an Activity field, not on
the wire, and not authority: anyone may still stop, starts still need a free claim, and no
takeover exists. It exists solely so the deadman can tell "my operator left" from "the local
operator is here".

A dead Remote must not hold the panel `BUSY`, so every Remote-originated claim is stopped, not
only the ones producing motion. A remote-started IMU test is released too.

*Response.*

The failsafe takes the existing path, not a parallel one:

1. `procedure::stop(Failsafe("Link lost"))` latches the one stop latch and sends
   `InterruptKind::EmergencyBrake`; the drive task brakes, cancels the active intent as
   `Cancelled`, bumps `CURRENT_EPOCH`, drains the queue and zeroes the tracks.
2. The claim owner observes the latch at its next poll, does its own hardware cleanup and calls
   `finish()`, which records `Failed("Link lost")` in Activity and releases the claim.

The interrupt is **conditional** on the live claim's origin. The drive queue is shared, so an
unconditional `EmergencyBrake` would also stop a panel-started procedure — exactly what this
decision forbids.

Coast-and-avoid needs the latch, not just the interrupt: it writes `MotorCommand::SetTracks`
directly to the motor driver, outside the drive queue (debt 19), so a brake holds only until the
mode's next state transition. `coast_obstacle_avoid::stop()` is what actually ends it.

*The stop verb and who writes Activity.*

ADR-0034 says `procedure::stop()` "latches the one stop latch, interrupts the drive and clears
Activity". That last clause is narrowed here, because `activity::fail()` is a no-op once the state
is Idle: a stop that clears first makes a reviewable failsafe report impossible.

- `procedure::stop(outcome)` latches and interrupts only. It writes no Activity and can be
  synchronous.
- The claim owner's terminal call is the only Activity writer. `finish()` replaces `abandon()` on
  the owner's stop path and applies the latched outcome: `Operator → activity::clear()` (Idle),
  `Failsafe(reason) → activity::fail(reason)` (Failed, held as a Result Report).
- The panel's running screen therefore leaves when the owner tears down, not the instant Stop is
  pressed — the bounded teardown ADR-0034 already accepts for the claim.
- A human Stop and link loss racing both latch; the first outcome wins, and either record is
  truthful. The latch stays sticky through the owner's teardown, so a refused start cannot disturb
  a sibling Procedure.

*Terminal record by claim kind.*

- `Procedure(P)` → `Failed("Link lost")`, held on the panel. The panel needs no new event:
  `refresh_running_screen` → `status_view` calls `.finished()` on any `Complete | Failed` stage,
  and Back clears it.
- `Drive` → Idle. ADR-0034 gives Manual Drive no Result Report; its outcome is the completion
  event.

No completion event is raised for either. The interactive remote-startable set is `Silent`, and the
run-to-completion set raises only on success, so ADR-0034's "no new stop event" holds.

*Autonomous modes started remotely.*

They stop. Coast-and-avoid is the only unbounded Remote motion in v1 and is precisely what a
deadman is for: the mode is onboard-safe against a wall but not against wandering unwatched. A
panel-started coast-and-avoid continues, because its operator is at the robot.

"Keep going" remains a defensible future policy for an onboard-safe mode a Remote starts. Taking
it would mean a per-Procedure property and a second meaning for "safe"; this decision does not take
it.

*Remote E-stop.*

The remote E-stop **is** `STOP_MOTION` with `brake = 1`. It maps to the same `procedure::stop()` and
`EmergencyBrake`, is acked and idempotent, and is not latched. In-band it is best-effort: it cannot
arrive when the link is dead, and it cannot outrun Wi-Fi, so the Deadman is the backstop. The app
may show "stopped" only on the ACK; without one it says "not confirmed — the robot stops on link
loss". The only latency promise is one command round trip plus dispatch. The panel Stop, and the
local obstacle/floor-drop brakes, remain the only immediate authority-independent safeguards.

*Partial failure.*

The robot guarantees exactly one thing: no frames for 2 s ⇒ a Remote-originated claim is stopped.
That single rule covers the four real cases:

| case | robot's view | outcome |
|---|---|---|
| link dead | silence | deadman fires |
| daemon wedged | silence | deadman fires |
| app frozen, daemon alive | frames keep arriving | robot keeps going; the daemon owns this |
| remote lying about state | frames keep arriving | bounded by local safety edges and the panel |

The app→daemon heartbeat is the daemon's: it should stop PINGing and send `STOP_MOTION` brake when
the app misses its own heartbeat for ~2 s. That is ticket 15's, not a robot protocol element.

*Deliberate link power-down.*

`Disable` fires the same failsafe before the branch goes down. After it, the Remote is gone and the
next `Enable` mints a new Boot Id, so it cannot observe the outcome; leaving a Remote-started
coast-and-avoid running would be the unattended motion this decision rules out.

*What the returning Remote learns.*

- `Procedure(P)`: `ACTIVITY` with `stage = failed`, detail `"Link lost"` — the only trace, since no
  completion event is raised.
- `Drive`: the drive job receives `Cancelled` and publishes `DRIVE_DISTANCE_COMPLETED` /
  `ROTATE_COMPLETED` with `status = cancelled` into the 8-entry event ring, retransmitted after
  reconnect. The volatile command is **determinate**, not "outcome unknown", unless the Boot Id
  changed or the ring overflowed; then the daemon reports "outcome unknown".

App rendering stays ticket 15's.

*Home.*

No new task and no supervisor. `LinkCore` already owns lease expiry and is driven by `tick(now)`
with plain `u64` milliseconds (ADR-0032), so the deadman and its `peer_lost` edge live there and
are host-testable. The Link Task observes the edge and calls the robot's one failsafe entry.

**Consequences**

- One rule covers link death, a wedged daemon and a deliberate power-down: 2 s of silence ends
  Remote-originated work. The robot's safety no longer depends on the Remote at all.
- The panel is never subordinated: a Remote disappearing cannot abort a locally started procedure,
  and a remote-started one no longer leaves the panel `BUSY` after its operator is gone.
- The stop verb gains an outcome, and the claim gains an origin. Both are small, local additions;
  neither changes who may start or stop anything.
- The daemon must PING at 1 Hz while a Session is open. That is a behavioural requirement, not a
  wire change, and it is cheap enough to be unconditional.
- A Remote-originated claim holds `BUSY` during its bounded teardown after the brake, exactly as a
  panel Stop does today.
- The brake is harsh and not configurable. On an incline that is the point; on a smooth floor it is
  one skid per lost link.
- The deadman's v1 teeth are coast-and-avoid. When `SET_DIFFERENTIAL` lands, the same rule covers
  it with no new design.
- The claim-origin tag means a Remote-originated `Procedure(P)` and a panel-originated one behave
  differently on loss. That is deliberate and is the only way to keep both "the panel is primary"
  and "the Remote's work dies with it" true at once.

**Open items**

- Implement `LinkCore`'s peer state and `peer_lost` edge, `procedure::claim(kind, origin)`,
  `procedure::stop(outcome)` and `finish()`, the `Drive` claim and the `Manual Drive` Activity
  variant; these ride on ADR-0034's claim refactor.
- Confirm the 2 s threshold and the 1 Hz PING on the board; both are provisional.
- Route coast-and-avoid through the drive subsystem (debt ticket 19) so the interrupt alone would
  reach it; until then the claim latch is what stops it, and the build must not assume otherwise.
- Ticket 15 owns the app→daemon heartbeat, "not confirmed" and "outcome unknown" rendering.

**Amendment (2026-11-08, ticket 15).** "Any valid PC→robot frame renews the lease", and the Link
Loss definition's "no valid PC→robot frame", are narrowed to **Remote-originated** PC→robot
frames. `HELLO`, `PING`, commands and event ACKs renew the lease and re-arm the Deadman;
`THROTTLE`, `BRIDGE_CONTROL`'s ACK and `BRIDGE_STATUS` do not. The last two are Bridge-originated,
and `THROTTLE` may be Bridge-synthesised, so the robot cannot tell its origin from the frame.
Without this, the Bridge's 1 Hz `BRIDGE_STATUS` control frame would keep a dead Remote's lease
alive and disarm the Deadman. The daemon's 1 Hz `PING` still renews an idle but healthy Session
(ADR-0042).
