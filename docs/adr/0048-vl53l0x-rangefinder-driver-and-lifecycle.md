# VL53L0X rangefinder pair: async edrv driver, raw range status and on-demand lifecycle

> **Superseded in part by [ADR-0049](0049-vl53l1x-async-driver.md).** The
> lifecycle, floor-drop detector, gate and test-mode decisions below remain
> valid. The driver choice (`edrv-vl53l0x`) and the 4-bit range-status
> assumptions are superseded by the in-repo VL53L1X driver and its 8-bit status.

The robot carries two VL53L0X time-of-flight modules, one per dedicated I2C
bus, as a floor-drop safety sensor (ADR-0026). The firmware had only a
synthetic stub behind `floor_drop_stub`; the pair was not wired into the drive
path. The sensors are unswitched 3.3 V, low power, have no moving parts, and
their `INT` lines are already wired. The available Rust drivers are either
blocking or young; the only genuinely async one is `edrv-vl53l0x`.

**Status:** accepted.

**Considered Options**

- **Use a blocking driver (`vl53l0x` or `vl53l0x-simple`) with a blocking I2C
  bus and a `try`/`nb` read.** Rejected for this firmware: it would make the
  two dedicated buses blocking and split the sensor path away from the async
  architecture. It remains the fallback if `edrv` fails on the bench.
- **Keep the synthetic stub and defer the hardware.** Rejected: the stub
  exercises the pipeline but cannot validate the driver, the range status or
  the on-demand lifecycle; the panel test mode is the bring-up path.
- **Leave the rangefinders always-on.** Rejected: the pair is a safety sensor,
  but it is fast to bring up and the drive path can arm it before motion;
  always-on spends power while the robot is idle. The LiDAR's wear-item
  rationale (ADR-0010) does not apply.
- **One task per sensor.** Rejected in favour of one task owning both: one
  command surface, one pair status, and one `select` over both `INT` lines and
  the command channel.
- **Deliver the test mode as a screen entry (like Room Scan).** Rejected in
  favour of a Procedure: it reuses the running-screen, claim and Stop
  machinery of the IMU tests. The new `ProcedureId` is panel-only because the
  Procedure is hands-on.

**Decision**

- **Driver.** Use `edrv-vl53l0x` 0.1.1 behind a new `rangefinder` workspace
  driver crate. The wrapper owns the I2C bus, the open-drain `XSHUT` and the
  `INT` input, and exposes `init`, `read`, `wait_ready`, `reset` and
  `standby`. `init` releases `XSHUT`, runs the ST initialisation sequence,
  sets the timing budget and starts continuous timed ranging at 50 ms.
  `read` reads `0x14`, then `0x1E..=0x1F`, then clears `0x0B`, in ST's order;
  it must not use `edrv`'s `read_range_continuous_millimeters`, which skips the
  status.
- **Reading.** A `RangefinderReading` carries the decoded `Reading` (`Valid`, `NoTarget`,
  `Invalid(RangeStatus)`) and the raw status byte. Raw status `11` is range
  complete; the other raw codes name the failure. A bus or readiness failure is
  an `Error`, not an invalid reading.
- **Floor-drop detector.** Each sensor has a 400 mm threshold. A valid reading
  above it sets `Drop` immediately; two consecutive valid readings below it
  clear to `Floor`. Invalid and no-target samples hold `Unknown` for 200 ms,
  then fail safe to `Drop`. A 150 ms gap without a sample marks the sensor
  stale and starts recovery. The pair's `FLOOR_DROP` is the OR of the two
  per-sensor states; the rangefinder task is its sole writer.
- **Lifecycle.** The pair is on demand. Its state is `Off`, `Arming`, `Armed`,
  `Degraded` or `Failed`. The drive gate in `handle_drive_action` arms the pair
  for every movement action, waits for `Armed` and requires the floor-drop flag
  to be clear before the first motor command; otherwise the action fails with
  `"floor-drop sensors not ready"` or `"floor drop detected"`. Both sensors
  must be armed; `Degraded` blocks motion but stays visible in the test mode.
  Disable on an explicit `Idle`/`Standby` or the end of the Activity, not on
  `EmergencyBrake`, so a floor drop stays visible.
- **Recovery.** Bus and readiness errors and a sustained invalid streak reset
  only the failed sensor: pulse `XSHUT`, wait, re-initialise, restart ranging.
  Up to three attempts per enable; then latch `Failed` until the next
  `Disable`/`Enable`.
- **Test mode.** A panel-only `Rangefinder Test` Procedure streams a
  two-column readout of both sensors (state, range, raw status, floor state and
  age) and the pair state, floor-drop aggregate and threshold. It publishes a
  typed `rangefinder` telemetry snapshot and relays it into an independent
  `rangefinder_test` leaf at 10 Hz; the UI is the only reader. The test stays
  open across faults and recovery until Stop. Its readout never enters Activity
  State or the Link Protocol's data path; only a new `ProcedureId` is added.
- **Stub.** Remove `floor_drop_stub` and `vl53l0x_stub`; the real driver is the
  default. The test mode is the bring-up path.

**Consequences**

- The first movement command may wait for the pair's bring-up and first valid
  reading; later movement in the same armed period does not. A movement action
  fails rather than driving without floor-drop protection.
- A single sensor failure blocks motion (`Degraded`), even though the other
  sensor still publishes. Direction-aware degraded motion is deferred.
- The panel test mode is the only place that shows per-sensor detail; the Link
  Protocol keeps the single `FLOOR_DROP` bit and `FloorDrop` event.
- `edrv-vl53l0x` is 0.1.1 and young; pin the exact version and bench-test both
  units before relying on it. The wrapper keeps a swap to a blocking driver
  local if that becomes necessary.
- Removing the stub removes the synthetic floor-drop path from desk testing;
  the test mode needs real sensors.
- The `ProcedureId` table gains a panel-only id; the Remote cannot start the
  test because it is hands-on.
