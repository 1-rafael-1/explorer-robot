# The drive control law is a pure step model in `drive-core`

The drive intents' control law was only verifiable by driving the robot because it lived behind the
async task, timers, sensor channels and motor commands. The decision is to move it into `drive-core`,
a `no_std`, allocation-free, HAL-free model crate with one `Intent` type and one `step`, and to make
the drive task its adapter. `drive-geometry` holds the shared track width and sprocket
circumference. The pattern is ADR-0058's Bridge Radio Session.

**Status:** accepted.

**Considered Options**

- A shadow model beside the firmware. Rejected: two control laws would drift.
- A generic `Intent` trait or runner. Rejected: ADR-0014; the variance becomes runner parameters.
- Keep the policy in firmware and extract helpers. Rejected: the transitions and timings are the bug
  surface.
- Return motor speeds instead of actions. Rejected: completion, waits and faults are control-law
  facts.
- Keep the active-intent enum in the firmware. Rejected: all intents belong in one crate and one
  step.
- A `CorrectionTuning` struct now. Deferred: grouped constants are enough until bench iteration needs
  variants.
- A fake adapter or simulator trait. Rejected: the tests are the second caller; the bench feature is
  an integration check of the real model.
- Keep the geometry split. Rejected: one physical geometry needs one home.
- A panel-visible simulation Procedure. Rejected: the bench features are diagnostics, not
  Activities.
- Exact correction parity as a merge gate. Rejected: the policy is provisional until assembly.

**Decision**

`drive-core` owns the control law of all five intents behind one `step`; the drive task owns sensors,
time, motor commands, queue, epoch, interrupts, completion, teardown and logging. `drive-geometry`
owns the two shared lengths. The model is the production path, not a shadow. The correction phase is
isolated and provisional.

**Consequences**

- The control law gains host tests; the drive task loses the law and keeps the wiring.
- There is one implementation, not a model beside a firmware copy.
- Track width and sprocket circumference have one home, shared with calibration and future odometry.
- No wire, queue, epoch, sensor-lifecycle or floor-drop change.
- Real-motion parity is deferred to the assembled robot.

The interface, parity ledger, test matrix, uncertainty ledger, landing steps and execution rule are
in the drive-core implementation spec in the local issue tracker.
