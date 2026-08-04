# Explorer Robot v3 — v2 Architecture Port Plan

## Problem Statement

The initial v3 scaffold implementation diverged from the v2 codebase — it created simplified, structurally different modules instead of faithfully carrying over the sophisticated architecture that proved itself on hardware. The drive subsystem's intent/queue/dispatch model, the calibration procedures, the testmode, the behavior handler pattern, and the file-level module organization were all lost. The v3 firmware must start from the v2 codebase and adapt surgically, not rewrite from scratch.

## Solution

A 1:1 file-level port of the simple-robot v2 codebase (commit `7b8ca7e`) into explorer-robot v3. Every v2 source file gets a v3 counterpart at the same module path. Files for removed hardware are deleted. Files needing hardware adaptation keep their name and module path but have their internals adapted. New capabilities (LiDAR stub, VL53L0X stub) get new files following v2's module patterns. The 58-file map was resolved through a grilling session recorded in this spec.

## User Stories

1. As a future developer, I want every v2 source file to have a deliberate disposition (KEEP, ADAPT, DELETE, NEW) documented in a single plan, so that I know exactly what to port, adapt, or discard.
2. As a future developer, I want the v2 module structure preserved 1:1 in v3, so that architectural knowledge transfers directly and code can be compared side-by-side.
3. As a future developer, I want the sophisticated drive subsystem (intent model, priority queue, dispatch, calibration procedures) ported intact, so that autonomous modes retain their proven control architecture.
4. As a future developer, I want hardware adaptations limited to bus plumbing and pin assignments, so that algorithmic code (IMU fusion, gap analysis, odometry, calibration convergence) ports without change.
5. As a future developer, I want dead hardware files deleted outright (ultrasonic, IR sensors, RC receiver, port expander), so that v3 contains no dead code.
6. As a future developer, I want the event system ported with updated event types (LiDAR scan, rangefinder readings replacing ultrasonic/IR events), so that sensor data flows through the same dispatch pattern.
7. As a future developer, I want the perception state rewritten around a LiDAR point cloud and VL53L0X rangefinder readings, so that autonomous modes query v3-appropriate data structures.
8. As a future developer, I want the testmode ported with 6 of 9 v2 tests (dropping ultrasonic/IR/coast-avoid-detection tests), so that hardware verification suites match v3 hardware.
9. As a future developer, I want flash storage persistence ported with simplified calibration data (2 motor factors instead of 4), so that calibration survives power cycles.
10. As a future developer, I want the IMU ported with SPI bus plumbing only, so that all 66KB of algorithmic code (quaternion fusion, DMP, calibration) ports 1:1.
11. As a future developer, I want core1 to run a dedicated LiDAR stub task that writes directly to the shared perception state via CriticalSectionRawMutex, so that autonomous modes can read point cloud data without cross-core channels.
12. As a robot operator, I want the port to compile and flash to the RP2350, so that hardware testing can begin.

## Implementation Decisions

### Port Strategy: 1:1 File Mapping

Every v2 source file maps to a v3 counterpart at the same module path. Files are categorized:

- **KEEP** (21 files): Copy verbatim. No changes needed — the code is hardware-agnostic (math, state machines, data structures).
- **ADAPT** (25 files): Copy the file, then surgically modify internals. Hardware dependencies (pin assignments, bus type, motor count) change but architecture and algorithms stay.
- **DELETE** (10 files): File removed. The hardware it controlled no longer exists in v3.
- **NEW** (2 files): `lidar_stub.rs` and `vl53l0x_stub.rs` — no v2 equivalent, created following v2 module patterns.

### Motor Driver Adaptation

The v2 motor driver controls 4 motors (2 per track, front/rear) via a PCA9555 port expander on I2C. v3 controls 2 motors (one per track) via direct GPIO. The adaptation:

- `MotorCalibration` struct stays in `motor_driver.rs` but simplifies from 4 factors (`left_front`, `left_rear`, `right_front`, `right_rear`) to 2 (`left_factor`, `right_factor`).
- Direction pins change from port expander bit-banging to direct GPIO `Output` pins. `MotorPins` struct holds `forward: Output` and `backward: Output` per track.
- Standby pin is direct GPIO.
- `MotorCommand` enum drops `SetAllMotors`, `SetAllMotorsRaw`, per-motor `Brake`/`Coast`/`SetSpeed`/`UpdateCalibration`. Keeps `SetTracks`, `BrakeAll`, `CoastAll`, `SetAllDriversEnable`, `LoadCalibration`, `UpdateCalibration` (per-track), `UpdateAllCalibration`.
- Voltage compensation logic carries over unchanged.
- Pin assignments: Left PWM on GPIO 0, right PWM on GPIO 3. Direction pairs on GPIO 1-2 (left) and GPIO 4-5 (right). Standby on GPIO 6.

### Encoder Reader Adaptation

v2 reads 4 encoder channels. v3 reads 2 (one per motor). `EncoderMeasurement` simplifies from 4 pulse count fields to 2 (`left`, `right`). Same PWM input mode, same command channel pattern, same sampling loop. Encoder pins: GPIO 7 (left), GPIO 8 (right).

### Drive Subsystem Port

The drive subsystem ports 1:1 structurally. All 14 files under `task/drive/` have v3 counterparts. The only internal changes:

- `dispatch.rs`: Maps `SetTracks` intent to the simplified 2-motor `motor_driver::send_command()` instead of the 4-motor port-expander path.
- `calibration/motor.rs`: 2-motor calibration procedure instead of 4-motor. Runs each track at fixed PWM, measures encoder pulses, computes `factor = max(left_pulses, right_pulses) / motor_pulses`.
- `calibration/imu.rs`: SPI bus plumbing instead of I2C. All algorithmic code (magnetometer calibration convergence, gyroscope bias estimation, DMP configuration) ports unchanged.

### Perception State Rewrite

v2's `perception.rs` holds an ultrasonic sweep buffer indexed by servo angle, with combined IR/ultrasonic obstacle flags. v3 replaces this with:

- `LidarPointCloud`: Fixed 360-element `[f32; 360]` array (distance in cm per degree, 0 = no return). Plus a `sequence: u64` for consumers to detect new scans. Method `is_obstacle_ahead(threshold_cm, cone_width_deg) -> bool`.
- `RangefinderReadings`: Four `Option<f32>` fields (`front_left_cm`, `front_center_cm`, `front_down_cm`, `rear_cm`). Method `any_below(threshold_cm) -> bool`.
- All atomic obstacle flags removed (`IR_DETECTED`, `ULTRASONIC_DETECTED`, `COMBINED_DETECTED`). Consumers check the LiDAR cloud or rangefinder readings directly.
- Lock order unchanged: power → calibration → perception → motion.

### Event System Adaptation

v2's `Events` enum has 22 variants. v3 keeps 13, adds 2, deletes 7:

- **Kept**: `Initialize`, `CalibrationDataLoaded`, `CalibrationStatus`, `CalibrationCompleted`, `BatteryMeasured`, `ObstacleDetected`, `ObstacleAvoidanceAttempted`, `RotaryTurned`, `RotaryButtonPressed`, `RotaryButtonHoldStart`, `RotaryButtonHoldEnd`, `TestingCompleted`
- **Added**: `LidarScan` (carries sequence number payload), `RangefinderReading` (signals new readings available)
- **Deleted**: All RC button events, `UltrasonicSweepCompleted`, `ImuCalibrationFlagsLoaded`
- **Modified**: `ObstacleDetected.source` changes from `Ir | Ultrasonic` to `Lidar | Rangefinder`
- **Deleted types**: `RCButtonId`, `UltrasonicReading`
- **Added type**: `ObstacleSource { Lidar, Rangefinder }`

Channel pattern (Channel<CriticalSectionRawMutex, Events, 64>) and `raise_event()`/`wait()` API are unchanged.

### Core Split

- **Core0**: Orchestrator, drive subsystem, encoder reader, UI (OLED, rotary encoder, RGB LED), I2C bus (OLED + VL53L0X array), battery monitoring, flash storage, testmode. Owns the main event loop.
- **Core1**: Dedicated LiDAR parser task. Emits placeholder point cloud data on a timer (stub mode). Future: real COIN-D6 UART parsing.

Cross-core communication: core1's LiDAR task calls `perception::update_lidar_points()` directly. The perception state mutex uses `CriticalSectionRawMutex` (safe across RP2350 cores since it disables interrupts on both). No cross-core channels needed.

### IMU SPI Adaptation

The `icm20948-rs` crate supports both I2C and SPI via its `Interface` enum. Only the driver initialization changes:

- v2: `Interface::I2c(i2c_device, address)` behind a shared bus mutex
- v3: `Interface::Spi(spi_device, cs_pin)` on a dedicated SPI bus

The SPI bus is not shared, so no mutex wrapping needed. All algorithmic code (quaternion fusion, DMP, magnetometer calibration, gyroscope bias, ~66KB across `task/sensors/imu.rs` and `task/drive/calibration/imu.rs`) ports unchanged.

### Flash Storage Adaptation

Same `sequential-storage` + `embedded-storage-async` architecture. Data payload simplifies:

- v2: 4 motor factors (4×f32 = 16 bytes) + IMU calibration flags + distance factor
- v3: 2 motor factors (2×f32 = 8 bytes) + IMU calibration flags + distance factor

Same `FlashCommand` channel, same `CalibrationKind` enum, same async `flash_storage` task pattern. Same FLASH_STORAGE memory region (last 8KB of flash).

### Testmode Adaptation

6 of 9 v2 tests port:

| Ported | Dropped |
|--------|---------|
| `basic_motor.rs` (adapted to 2 motors) | `ir_ultrasonic.rs` |
| `turns.rs` | `ultrasonic_sweep.rs` |
| `straight_drive.rs` | `coast_avoid_detection.rs` |
| `arc_drive.rs` | |
| `imu_6axis.rs` (SPI adaptation) | |
| `imu_9axis.rs` (SPI adaptation) | |

### UI Adaptation

UI subsystem ports with minor menu changes:
- Test menu drops ultrasonic/IR entries, adds LiDAR/rangefinder placeholder entries
- Rotary encoder button moves from port expander to direct GPIO
- I2C bus moves from core1 to core0 for display initialization
- Motor test screen: 2 motors instead of 4

### New Sensor Stubs

- `lidar_stub.rs`: Embassy task on core1. Emits a configurable 360° point cloud on a timer (default: clear ahead at 200cm, walls at ±90° at 50cm). Configurable via channel commands to set obstacle patterns for testing autonomous modes. Writes directly to `perception::update_lidar_points()`.
- `vl53l0x_stub.rs`: Embassy task on core0. Emits 4 fixed distance readings on a timer (default: all 200cm = clear). Configurable via channel commands. Writes to `perception::update_rangefinder_readings()` and raises `RangefinderReading` events.

### Complete File Disposition

**KEEP** (21 files — copy verbatim):
`system/state/power.rs`, `system/state/motion.rs`, `system/state/calibration.rs`, `system/helper/mod.rs`, `system/helper/string_helper.rs`, `task/battery_charge_read.rs`, `task/startup.rs`, `task/behavior/battery.rs`, `task/autonomous_mode/mod.rs`, `task/drive/mod.rs`, `task/drive/api.rs`, `task/drive/brake_coast.rs`, `task/drive/differential.rs`, `task/drive/distance.rs`, `task/drive/intent.rs`, `task/drive/queue.rs`, `task/drive/rotation.rs`, `task/drive/state.rs`, `task/drive/types.rs`, `task/drive/calibration/mod.rs`, `task/drive/sensors/mod.rs`, `task/drive/sensors/control.rs`, `task/drive/sensors/data.rs`, `task/indicators/rgb_led_indicate.rs`, `task/ui/render.rs`, `task/ui/state.rs`

**ADAPT** (25 files — copy then modify internals):
`main.rs`, `system/event.rs`, `system/state.rs`, `system/state/perception.rs`, `task/motor_driver.rs`, `task/orchestrate.rs`, `task/behavior/input.rs`, `task/behavior/obstacle.rs`, `task/autonomous_mode/coast_obstacle_avoid.rs`, `task/autonomous_mode/attempt_straight_line.rs`, `task/autonomous_mode/gap_analysis.rs`, `task/control/rotary_encoder.rs`, `task/drive/dispatch.rs`, `task/drive/calibration/imu.rs`, `task/drive/calibration/motor.rs`, `task/initialization/mod.rs`, `task/io/display.rs`, `task/io/flash_storage.rs`, `task/sensors/encoders.rs`, `task/sensors/imu.rs`, `task/testmode/mod.rs`, `task/testmode/basic_motor.rs`, `task/testmode/imu_6axis.rs`, `task/testmode/imu_9axis.rs`, `task/ui/mod.rs`, `task/ui/menu.rs`, `task/ui/screens.rs`

**DELETE** (10 files):
`system/button_actions.rs`, `task/control/rc_control.rs`, `task/io/port_expander.rs`, `task/sensors/ir_obstacle.rs`, `task/sensors/ultrasonic.rs`, `task/sensors/ultrasonic_correction.rs`, `task/testmode/coast_avoid_detection.rs`, `task/testmode/ir_ultrasonic.rs`, `task/testmode/ultrasonic_sweep.rs`

**NEW** (2 files):
`task/sensors/lidar_stub.rs`, `task/sensors/vl53l0x_stub.rs`

### Implementation Order

Files are ported in dependency order — modules that are imported by others come first:

1. **Scaffold**: `Cargo.toml` (v2 deps, stripped of dead crates), `rust-toolchain.toml`, `.cargo/config.toml`, `memory.x`, `build.rs`, `rustfmt.toml`, `CONTEXT.md`
2. **System state modules**: `system/state/power.rs`, `system/state/motion.rs`, `system/state/perception.rs` (rewritten), `system/state/calibration.rs`, `system/state.rs`
3. **System helpers**: `system/helper/`, `system/event.rs` (adapted)
4. **Motor driver**: `task/motor_driver.rs` (adapted — 2 motors, direct GPIO)
5. **Encoder reader**: `task/sensors/encoders.rs` (adapted — 2 channels)
6. **Drive subsystem**: All `task/drive/` files — types, intent, queue, dispatch, brake/coast, differential, distance, rotation, state, calibration (motor + IMU), sensors
7. **Sensor tasks**: `task/sensors/imu.rs` (SPI), `task/sensors/lidar_stub.rs` (new), `task/sensors/vl53l0x_stub.rs` (new), `task/battery_charge_read.rs`
8. **I/O**: `task/io/flash_storage.rs`, `task/io/display.rs`
9. **Indicators**: `task/indicators/rgb_led_indicate.rs`
10. **Control**: `task/control/rotary_encoder.rs`
11. **Behavior handlers**: `task/behavior/`
12. **Orchestrator**: `task/orchestrate.rs`
13. **Initialization**: `task/initialization/mod.rs`, `task/startup.rs`
14. **UI**: `task/ui/` (mod, menu, render, screens, state)
15. **Testmode**: `task/testmode/` (6 ported tests)
16. **Autonomous modes**: `task/autonomous_mode/` (coast-and-avoid, attempt-straight-line, gap analysis)
17. **Main entry**: `main.rs` (wire all tasks, pin assignments, dual-core spawn)

Groups 2–3 can run in parallel with group 4. Groups 7 can run parallel with 8–11. Group 14 is independent of drive/orchestrator and can run parallel with 4–13.

## Testing Decisions

Embedded Rust on bare metal without a hardware-in-the-loop rig means automated tests are impractical. Testing is deferred to on-hardware verification.

**Primary seam**: `cargo check` passes for the `thumbv8m.main-none-eabihf` target after each implementation group. This verifies the port compiles correctly against the RP2350 target.

**Intermediate seams**: Individual module groups compile independently. For example, all `system/state/` files can be checked as a unit; all `task/drive/` files as another.

After flashing to hardware, each subsystem is verified via defmt logs over RTT:
- Motor commands verified by encoder pulse counts and physical movement
- UI verified by navigating menus with the rotary encoder and observing OLED output
- Autonomous modes verified by placing the robot in a controlled environment and observing behavior with stub sensor data

## Out of Scope

- Real VL53L0X driver implementation (stubbed)
- Real COIN-D6 LiDAR driver implementation (stubbed)
- AI camera module integration
- ROS2 integration
- SLAM or mapping
- Waypoint navigation beyond attempt-straight-line
- New autonomous modes beyond coast-and-avoid and attempt-straight-line
- Custom chassis or PCB design
- Automated test suite (no HIL rig)

## Further Notes

This spec records the port plan resolved during a grilling session on 2026-08-03. The 1:1 file mapping ensures architectural fidelity to v2 — every design pattern, state machine, calibration algorithm, and control structure ports intact. Hardware adaptations are surgical: bus plumbing, pin assignments, and motor count. No algorithmic code is rewritten.

The primary reference is the simple-robot v2 codebase at commit `7b8ca7e`. The domain glossary is in `CONTEXT.md`. The original v3 scaffold spec is `0001-explorer-robot-v3-scaffold.md`.
