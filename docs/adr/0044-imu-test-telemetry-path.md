# IMU test telemetry is a dedicated snapshot rendered by the panel

The six-axis and nine-axis IMU test Procedures publish a 10 Hz, panel-only telemetry snapshot; the touch UI reads it and formats a multi-row readout inside the existing running screen. Live sensor data does not enter Activity State, and the test task neither draws nor formats. The UI tick drops from 250 ms to 100 ms so the readout updates at 10 Hz; the existing 272 × 144 status panel and 6 × 10 font hold the full nine-axis set. Direct drawing from the test task (the older simple-robot shape), extending Activity State, and sending a UI event carrying the data were rejected: they break the "UI is the only thing that draws" rule, turn Activity State into a sensor snapshot, or couple `testmode` to the panel.

## Considered Options

- **Dedicated `state::imu_test` snapshot + neutral `touch_ui::ImuReadout`** — chosen.
- **Extend `activity::Snapshot`** — rejected: Activity State is phase and percent; the `ACTIVITY` stream would either lag or need a protocol change.
- **`UiEvent` carrying the telemetry** — rejected: couples the test task to the UI, loses the snapshot when the UI restarts, and formats outside the UI model.
- **Direct drawing from the test task** — rejected: violates the single-drawer invariant and bypasses touch/Stop handling.

## Consequences

- `state::imu_test` joins the documented lock order; the test task writes it at 10 Hz and the UI reads it on its tick.
- `touch-ui` gains a neutral `ImuReadout` and `StatusView` gains a readout mode; the row buffer grows past the current 32-byte `TextBuf` capacity.
- The `ACTIVITY` stream and the Link Protocol are unchanged; the Remote never sees the readout.
- `TICK_INTERVAL_MS` drops from 250 ms to 100 ms. Equality checks keep other screens from flushing; an IMU test flushes a full frame at 10 Hz.
- The IMU test's periodic `defmt` reports drop from `info` to `debug`; with the repo's `DEFMT_LOG = "info"`, they are hidden by default and appear only when the log level is raised.
