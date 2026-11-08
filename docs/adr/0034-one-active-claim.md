# One Active Claim: the robot runs one thing at a time, and anyone may stop it

Two operators now exist. The Touch Panel and the Remote can both start work, and the firmware
grew a different guard for each kind of work: a test-mode slot, an autonomous-mode flag,
coast-and-avoid's own active flag and stop request, a per-family stop latch for tests and another
for calibrations, a single-queue flag, and no slot at all for calibrations, which were serialised
only because they happen to run inline in the drive dispatch loop. The Activity State is already
the single truth of *what* is running, but it records rather than arbitrates. The panel's modal
menus made the gap invisible: one operator could only be in one place at a time.

With a second operator, a start can arrive while the panel is not looking, and the motion paths no
longer have a modal menu keeping them apart. ADR-0032 settled the seam — `procedure::start` /
`procedure::stop` plus one `remote_drive_job` — and left authority to this decision. ADR-0028's
precedent (one-way commands, no reference count, accepted last-writer races) is the right starting
point for a sensor; motion needs one arbiter, but not an owner.

**Status:** accepted. Authority is exclusivity, not ownership: the panel is never locked out, the
Remote is subordinate by construction, and the lease stays liveness. The claim refactor and the
two motion-path debts are implementation work (tickets 19 and 20); no firmware is changed by this
decision. The lease is not reopened here: what happens to motion on link loss remains ticket 06's.
That ticket is resolved in ADR-0035, which also narrows one sentence here: `procedure::stop()`
latches and interrupts but no longer writes Activity, so the claim owner's terminal call can
record a failsafe failure, and the claim gains a transient `Panel`/`Remote` origin used only by
the deadman — not for authority.

**Considered Options**

- **An explicit owner (`Panel | Remote`) with a takeover gesture and a lockout.** Rejected. It
  needs new authority state that Activity does not hold, an actor on every write, and an answer to
  "how long does a lockout last" that fights the 5 s lease. The panel is the primary operator
  surface and must never be locked out, so the owner would have to be biased anyway; the bias is
  cheaper as a rule than as a mode.
- **Last-writer-wins over the existing per-family flags.** Rejected. Activity holds one truth, but
  the flags do not cover the same set: coast-and-avoid writes the motor driver directly, the
  calibrations have no slot, and the drive queue's busy flag never sees coast-and-avoid. Two
  operators could overlap a procedure and a drive command with no error.
- **The Remote is read-only in v1.** Rejected. It discards the point of the feature. Observation
  needs no authority; the question is only who may act.
- **The Session lease is the authority.** Rejected. The lease is liveness (ADR-0030): any valid
  frame renews it, it expires after 5 s, and the robot holds no Session object. Making it the
  authority would give the robot a second meaning for link state and make reconnect and takeover
  the same problem.
- **An actor field in Activity State.** Rejected. "Who started it" is provenance, not what the
  robot is doing. The `ACTIVITY` stream has no byte for it, every writer would change, and no
  policy here needs it: starts are not preempted, and the Remote already knows which commands it
  sent. A "Remote" badge, if wanted, is link state plus a UI surface (ticket 15).
- **A stop-reason event for the Remote.** Rejected. The Remote-startable set already makes
  `Running → Idle` unambiguous (below); a new event would be a second way to say "stopped".
- **One exclusive Active Claim.** Accepted. One atomic claim replaces six flags, one latch
  replaces two families, and the Activity writer becomes the claim's owner.

**Decision**

*Authority is exclusivity, not ownership.*

At most one **Active Claim** exists. It is the exclusive right to be the robot's current
operator-facing activity and to produce motion. Whoever holds it may act; everyone else is
`BUSY`. Any operator may stop the holder. There is no owner, no takeover mode, no lockout and no
lease-derived authority. "Takeover" is therefore the existing Stop: the panel follows the active
Activity into its running screen, where its only action is Stop, and then starts whatever it
wants.

*The claim.*

One module (the extended `task/procedure`) owns the claim, the stop latch and the live Activity
write. The claim's kind decides what it publishes and who can take it:

| kind | Activity written | held by |
|---|---|---|
| `Boot` | `Booting` | the initialisation flow, released when the main menu shows |
| `Procedure(P)` | `Procedure(P)` | a test task, a calibration, or coast-and-avoid |
| `Drive` | `Manual Drive` (below) | the Remote's single drive job |
| `Screen(RoomScan)` | nothing | the panel's radar screen |

The claim is a single atomic slot with a generation, not a mutex. `procedure::claim(kind)`
succeeds only when free; `procedure::start(procedure)` is the Procedure form, and keeps
ADR-0032's `StartOutcome` (`Started`, `Busy`, `Rejected`). The owner drives the claim with
`phase`, `complete`, `fail`, `abandon`, `is_stop_requested` and `wait_or_stop`; a terminal call
only affects the claim generation it was issued for, so a stopped run's late cleanup cannot
disturb the next claim. That generation is neither the Session epoch (command dedupe) nor the
drive's `CURRENT_EPOCH` (queued-command invalidation); it only scopes who may release the claim.

`procedure::stop()` latches the one stop latch, interrupts the drive and clears Activity. It does
**not** release the claim: the owner observes the latch, cleans up, and releases — a Procedure at
its next poll, the drive job when its queue returns `Cancelled`. During that bounded teardown a
new start is `BUSY` rather than overlapping the cleanup — the cross-operator version of the
existing "a refused start must not disturb a live sibling". `Boot` and `Screen(RoomScan)` have no
latch; a stop while they hold the claim has nothing to cancel, and the drive is idle by
construction (the interrupt still goes out).

*Start and stop rules.*

- A start needs a free claim. Held is `BUSY` (temporary) for both operators; there is no queue and
  no preemption.
- `REJECTED` is for a Procedure that cannot be started from the Remote at all (the hands-on set
  below, plus the not-yet-implemented Attempt Straight), and `UNKNOWN_ID`, `STALE`,
  `BAD_ARGUMENT` and `BAD_LENGTH` stay as ADR-0030 defines them.
- Anyone may stop. The panel's Stop and `STOP_MOTION` call the same `procedure::stop()`; the
  payload's brake flag chooses `EmergencyBrake` or `Stop` for the drive interrupt. A Remote stop of
  a panel Procedure is immediate and needs no confirmation, and vice versa.
- The claim serialises motion with one queue underneath. `QUEUE_BUSY` remains as the drive queue's
  internal single-run assertion, not as the policy, and the drive's `CURRENT_EPOCH` still
  invalidates queued commands on an interrupt. This is the ticket's "same epoch" constraint: both
  operators issue into the same queue and the same epoch, and a stop drains queued work.

*The panel follows Activity.*

The panel is Activity-driven and reads no other authority surface. Whenever Activity is live it
shows the running screen — from any screen, including System Info — and when Activity goes idle it
leaves the running screen for the landing screen of the Procedure it was showing (the Main Menu
for Manual Drive). A finished Result Report is not live: it holds until the operator dismisses it,
and a new claim may replace it. A value-entry screen is not an Activity, so a new claim may yank
an uncommitted value; losing a rare typed number is cheaper than teaching the robot what the panel
is showing.

*Hands-on Procedures.*

The catalogue's single `available` bit is enough; a second "panel-only" bit would describe a
difference the Remote cannot act on. The hands-on set keeps `available = 0`:

- **Motor Calibration** — tracks spin on the bench; an operator must watch and stop it.
- **Mag Calibration** — the operator physically rotates the robot.
- **Distance Calibration** — its drive step ends on the panel's value screen, where the operator
  measures and types the result.
- **Basic Motor Test** — a bench test whose legs cycle until the operator stops them.
- **Attempt Straight** — modelled but not implemented.

Coast-and-avoid and the five driving/IMU tests are Remote-startable. The test is whether the flow
needs hands or a measurement, not whether the robot moves: the Remote can watch the cloud and
motion streams.

*Manual Drive.*

A volatile Remote drive command (`DRIVE_DISTANCE`, `ROTATE_BY`) holds a `Drive` claim while it
runs and publishes a transient **Manual Drive** Activity with the wire id `0x000D` (the next free
id, never `0` idle / `1` boot / `2..0x000C` Procedures). It is Activity-only: not in the
Catalogue, not startable as a Procedure, no Result Report, and its outcome is the existing
`DRIVE_DISTANCE_COMPLETED` / `ROTATE_COMPLETED` event. The detail line carries the command
("Drive 150 cm", "Rotate 90°"). This is what lets the physical operator follow Remote motion into
a running screen with a Stop instead of reaching for the PC, and it keeps one truth: the panel
reads Activity, not a parallel motion flag. The `ACTIVITY` stream's `procedure_id` field keeps its
name; its range now names Activities, whose startable subset is the Catalogue.

*Stop visibility.*

No new event. For the Remote-startable set, `ACTIVITY` leaving `Running` without a terminal stage
and without a `PROCEDURE_COMPLETED` means an operator stopped it: Turns, Straight Drive and Arc
Drive raise the event on success, while Basic Motor, the IMU tests and coast-and-avoid are
interactive and only end on a Stop. The app renders "Stopped"; the reliable event, not a lossy
sample, carries every successful finish.

**Consequences**

- Six busy/stop mechanisms collapse into one claim and one latch: `TESTMODE_ACTIVE`,
  `AUTONOMOUS_MODE_ACTIVE`, coast-and-avoid's `ACTIVE`/`STOP_REQUESTED`, `TEST_STOP`,
  `CALIBRATION_STOP`, and the per-family `test_lifecycle`/`calibration_lifecycle`. The
  completion-event policy becomes one per-Procedure table.
- The per-family stop latch in ADR-0014 is superseded: its premise was that two families could be
  live at once. One Procedure is live at a time, so one latch reaches exactly the one that runs.
  The lifecycle module itself is unchanged in intent.
- Both operators are two callers of one gate, so the ticket's constraint holds by construction:
  no second start, no second motion path active, no conflicting drive commands in the same queue
  epoch.
- The panel has one read surface — Activity — so a remote-started Procedure and a remote drive
  command both appear and both offer Stop. The app's rendering remains ticket 15's.
- The queue's busy flag and the drive epoch stay as mechanisms; they are no longer policy.
- Two motion paths remain: coast-and-avoid writes `MotorCommand::SetTracks` directly (bypassing
  drive calibration/voltage compensation and the drive interrupt), and calibrations run inline in
  the drive dispatch loop. The claim makes them mutually exclusive, which is what authority
  needed, but the paths stay unequal. Recorded as debt, not fixed here.

**Open items**

- Implement the claim, one latch, the per-Procedure completion table and the panel-follow rule;
  refactor the producers onto one claim (tickets 19 and 20 cover the motion paths).
- Add the `Manual Drive` Activity variant and the `0x000D` id to `remote-protocol` and the
  `ACTIVITY` mapping; give the app a label for it when the payload mapping is written.
- Ticket 06 owns what happens to a Remote-started Procedure on link loss; this decision leaves
  motion on loss untouched.
- Ticket 15 owns how the app renders `BUSY`, "stopped by the other operator", and Manual Drive.
