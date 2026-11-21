# Motor calibration is a measured per-track, per-direction speed curve

The motor command is an open-loop percentage, and the only correction was a
single per-track multiplier. That multiplier could never do anything: the
procedure computed `max(left, right) / side` and then clamped it to `1.0`, so
every non-zero measurement collapsed to `1.0`. It was also direction-blind and
speed-blind, so it could not represent the forward/reverse and low/high-command
differences the Basic Motor Test exists to expose. The encoder scale was equally
untrustworthy: the firmware assumed 960 pulses per sprocket revolution, the
glossary said 1320, and the hardware (11 pulses per motor revolution behind a
30:1 gearbox, driving the sprocket directly) implies 330.

**Status:** accepted

**Considered Options**

- **Keep a single per-track multiplier.** Rejected: it cannot correct a track
  that is slower in reverse or nonlinear in command, and the current formula
  stores a no-op.
- **Balance only, against the faster track.** Rejected: corrections above `1.0`
  would demand more than the weaker motor can give. The slower track is the
  reference, so every correction attenuates and the robot's top speed is the
  weakest leg's top speed.
- **Global target across all four legs.** Rejected: it caps forward speed to
  match reverse, discarding capability for a difference that is real. The
  target is per direction — the slower of the two tracks in that direction.
- **Measure only 40, 60, 80 and 100 %.** Rejected: the calibration floor comes
  from the breakaway, and the floor is what makes the inverse well-defined. The
  sweep is the same ten-step sweep the Basic Motor Test already runs.
- **Store only the derived corrections.** Rejected: the curve is then
  un-re-derivable when the model changes. The raw measurements are the source of
  truth; the corrections are a cache.
- **Migrate the old calibration data.** Rejected: the only robot has never had a
  trustworthy motor or distance calibration, so a migration would preserve
  nothing but doubt.

**Decision**

- A **Motor Calibration Curve** maps a requested speed to the raw command that
  produces the **Target Speed**, separately for each track and direction, at the
  ten Power Steps from 10 % to 100 %, interpolated between them.
- The Target Speed at a Power Step is the slower of the two tracks in that
  direction, so the weakest leg sets the reduction and no correction exceeds
  `1.0`.
- Corrections apply only at or above the **Calibration Floor**, the highest
  Breakaway among the four legs. Below it the inverse is meaningless and the raw
  command is used.
- Each measurement starts from **Quiescence** (braked, no encoder pulses for a
  hold window), resets and verifies the counter, commands, discards a settle
  interval, and counts a fixed **Measurement Window** from timestamped encoder
  deltas. The old "reset 100 ms before the command and read the last stored
  absolute count" window admitted coast-down pulses and is gone.
- The encoder scale is a calibration output, not a datasheet constant: Distance
  Calibration stores counts per centimetre (from which counts per sprocket
  revolution follows), and the curve compares counts per second, so neither
  needs a hard-coded pulses-per-revolution figure.
- The stored payload is versioned, checksummed raw measurements plus the
  derived corrections, under a new flash key. The old motor key and the distance
  factor are invalidated; both calibrations must be re-run.

**Consequences**

- `MotorCalibration { left_factor, right_factor }` disappears. `SetTracks`
  applies the curve, then voltage compensation, then PWM.
- The calibration maths lives in a host-tested `no_std` model crate, not in the
  motor driver task.
- A failed calibration saves nothing and leaves the previous curve in force.
- The Basic Motor Test and Motor Calibration share one measurement engine; the
  test is read-only and the calibration is the only writer.
- Drive modes interpolate between the measured points; there is no feedback loop
  yet. A per-track speed loop is a separate roadmap item.
