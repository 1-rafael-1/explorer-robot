# Basic Motor Test telemetry is a dedicated panel-only readout that keeps its progress bar

The Basic Motor Test's forty Power Steps publish a panel-only `state::basic_motor_test` snapshot; the touch UI reads it on its tick and renders a multi-row **Basic Motor Test Readout** inside the existing running screen, above the progress bar. It does not enter Activity State, the `ACTIVITY` stream or the Link Protocol, and the Remote never sees it. Unlike the IMU Test and Rangefinder Test readouts, this one keeps the bar instead of suppressing it: the bar carries "where am I in the forty-step cycle", the readout carries per-step counts, and neither substitutes for the other — the difference is deliberate, not an oversight. The readout owns its own rules — the Breakaway latch, the last-completed identity and the live-count hold — so the test task keeps the timing and the motor commands and the model keeps the formatting. Extending `activity::Snapshot`, a `UiEvent` carrying the counts, direct drawing from the test task, and suppressing the bar as the other readouts do were rejected: they turn Activity State into a sensor snapshot and drag the wire with it, couple the task to the panel and lose the snapshot on a UI restart, break the single-drawer invariant and bypass touch/Stop, or throw away the only whole-cycle cue.

## Considered Options

- **Dedicated `state::basic_motor_test` snapshot + neutral `touch_ui` readout** — chosen. The task publishes at the sampling rate and the model formats every row, following [ADR-0044](0044-imu-test-telemetry-path.md)'s precedent.
- **Extend `activity::Snapshot`** — rejected: Activity State is what the robot is doing, phase and percent; encoder counts would make it a sensor snapshot, and the `ACTIVITY` stream would either lag or need a protocol change, putting a bench-only number on the wire.
- **A `UiEvent` carrying the counts** — rejected: couples the test task to the panel, loses the snapshot when the UI restarts, and formats outside the model.
- **Direct drawing from the test task** — rejected: breaks the single-drawer invariant and bypasses touch/Stop handling.
- **Suppress the progress bar as the IMU and Rangefinder readouts do** — rejected: the bar is the only whole-cycle cue; forty per-step counts cannot tell the operator where in the cycle they are.

## Consequences

- `state::basic_motor_test` joins the documented lock order as another independent-leaf entry; the test task writes it at the sampling rate and the UI reads it on its tick.
- The readout adds rows to the panel and ends above the progress bar; the task publishes once per sample while a Power Step runs, and once at each start and completion.
- The `ACTIVITY` stream and the Link Protocol are unchanged; the Remote never sees the readout.
- The IMU and Rangefinder readouts keep their current full-panel behaviour: they pass no percent, so the bar is drawn only when a percent is present.
- A failed stage detaches the Basic Motor Test readout so the failure reason occupies the panel; the IMU and Rangefinder screens keep theirs.
- The readout accepts three transitions — start a Power Step, sample a count, complete a Power Step — and derives every row, so the Breakaway latch, the last-completed identity and the live hold stay host-testable.
- A zero count is displayed as zero and neither ends nor fails the test.
