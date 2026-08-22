# Explorer Robot — Domain Glossary

> This glossary records the canonical terms for the explorer-robot v3 firmware. It is a living document — update it whenever a term is resolved during design or implementation.

## Hardware

- **Weight Distribution** — Motors (~400g) at the rear, battery (~100g) at the front. Remaining components (PCB, LiDAR, rangefinder) biased forward to achieve roughly centered center of mass for even track pressure and stable climbing.
- **Track** — Left or right side of the robot. Each track has three sprockets: rear drive sprocket (motor-coupled, toothed), middle road wheel (weight-bearing, smooth), and front idler (tensioner, smooth). Rear drive keeps the bottom track run in tension under load, maximizing traction and preventing track bunching.
- **Motor** — A single JGB37-520 6V 165RPM DC motor with hall encoder. Two motors (one per track).
- **Motor Driver** — The TB6612FNG dual H-bridge chip. Controls direction (via direct GPIO) and speed (via PWM) for both motors. One standby pin enables/disables the driver.
- **Encoder** — Single-channel hall sensor on each JGB37-520 motor. Produces 8 pulses per motor revolution, 1320 pulses per output shaft revolution (165:1 gear ratio). Pulse counting via PWM input mode. Direction is inferred from the motor command, not quadrature.
- **IMU** — ICM-20948 9-axis inertial measurement unit (accelerometer, gyroscope, magnetometer). Connected via dedicated SPI bus (CS, SCK, MOSI, MISO).
- **LiDAR** — COIN-D6 360° spinning dTOF LiDAR on dedicated UART0 (core1), with an active-high low-side power MOSFET (IRLZ44N) on GPIO15 for firmware-controlled power cycling. Emits continuous scan data at a native 0.9° resolution (400 points per revolution) with distances in millimetres, over UART at 230400 baud 8N1 with a start/stop command protocol. Driven by the `coin-d6` workspace crate; rewiring the core1 firmware task to it is a follow-up.
- **AI Cam** — Grove Vision AI V2 on-device ML camera module (Himax WiseEye2), reserved on dedicated UART1 (core0) with its own power MOSFET (IRLS44N). Not yet integrated — pins reserved only.
- **Rangefinder** — VL53L0X time-of-flight laser rangefinder. Single unit, front-down (angled downward for stair/drop detection), on the shared I2C0 bus. No XSHUT sequencing needed — single device at default address. The 360° LiDAR covers forward/lateral/rear arcs, so only the downward-facing sensor is retained (currently stubbed for development).
- **OLED** — SSD1306 128×64 monochrome display on I2C bus, shared with VL53L0X rangefinder.
- **Rotary Encoder** — EC11 quadrature rotary encoder with push button. Used for menu navigation (rotation) and selection (push).
- **RGB LED** — Common-cathode RGB LED. Indicates battery state (green → yellow → red) and obstacle alerts (flashing red).
- **Battery** — 2S LiPo (twin 18650, 8.4V max). Placed at the front to counterbalance the rear-mounted motors (~400g combined). Voltage read via ADC. Motors compensated to 6V target.
- **Standby Pin** — Direct GPIO that enables/disables the TB6612FNG motor driver. Active high.

### LiDAR

- **Spin** — One full 360° revolution of LiDAR returns: 400 points at the native 0.9° resolution.
- **Snapshot** — A single-revolution capture, as returned by `read_scan`.
- **Point** — One LiDAR return, `{ angle_deg, distance_mm, intensity }` (bearing in degrees, range in millimetres, return strength 0–255).
- **Ring-start** — The packet flag (`T == 1`, the low bit of the `CT` byte) that delimits the start of a new spin.
- **Validity ratio** — The minimum fraction of aggregated spins that must report a valid (`distance_mm > 0`) sample at an angular bucket for that bucket to be kept; otherwise the bucket is emitted as a no-return.

## Architecture

- **Core0** — Runs the orchestrator, drive subsystem, encoder reader, UI (OLED, rotary encoder, RGB LED), IMU (SPI0), I2C bus (OLED + VL53L0X rangefinder), flash storage, and the main event loop.
- **Core1** — Runs the LiDAR task. Currently runs a synthetic point-cloud stub; real COIN-D6 UART parsing is provided by the `coin-d6` crate, with rewiring the task to it still a follow-up.
- **Orchestrator** — Central event loop. Waits for events from the system event channel and dispatches them: calibration events → initialization module, obstacle/battery events → behavior handlers, rotary events → UI subsystem, sensor events → logged or forwarded. Pure routing — no domain logic.
- **Event System** — Typed, multi-producer single-consumer event channel (capacity 64). Sensor tasks and input tasks raise events; the orchestrator consumes them. The seam between producers and consumers.
- **Behavior Handler** — A module under `task/behavior/` that reacts to specific event types. Domain logic lives here — obstacle fusion (perception atomics + EmergencyBrake interrupt), battery state updates, obstacle avoidance completion. Called by the orchestrator.
- **Task Spawn Order** (core0) — Tasks are spawned in dependency order: orchestrator → battery → rgb_led → rotary_encoder → motor_driver → encoders → drive_queue_executor → drive → display → vl53l0x_stub → imu → flash_storage → testing → ui → autonomous_mode → startup. Core1 runs only lidar_stub.

### Message Passing

Three data flow patterns coexist, chosen by latency requirements:

- **Event bus** — Semantic events (obstacle detected, button pressed, calibration loaded) flow through `Events` channel → orchestrator → handlers. Used for state changes that multiple consumers may care about.
- **Direct channels** — High-frequency sensor data (encoder pulses, IMU orientation) flows point-to-point via dedicated `Channel`s directly into the drive subsystem, bypassing the event bus. Avoids event channel congestion.
- **Perception atomics** — `LIDAR_OBSTACLE`, `RANGEFINDER_OBSTACLE`, and `COMBINED_OBSTACLE` atomic booleans provide lock-free obstacle reads on the hot path. Written by behavior handlers on `ObstacleDetected` events and by sensor stubs; read by autonomous modes and the UI without acquiring a mutex.

## Driving

The `drive` module tree owns all motion control. Commands flow through a thin dispatch that routes to control modules. Two public entry points: `send_drive_command` (queue a command) and `send_drive_interrupt` (preempt).

- **SetTracks** — The primary motor command. Sets left and right track speeds (-100 to +100). Calibration and voltage compensation are applied before PWM output.
- **Drive Distance** — A `DriveAction` that commands travel of a specified distance (straight or curved arc) using encoder feedback with optional IMU correction. Completes when target is reached or an interrupt preempts it.
- **Rotate Exact** — A `DriveAction` that commands in-place rotation to a target angle using IMU feedback. Completes when the angle is reached within tolerance.
- **Drift Compensation** — Encoder-based correction that equalizes left/right track speeds to prevent veering. Handled by higher-level intents (distance, rotation) via IMU heading correction and encoder feedback.
- **Ramp-Down** — Progressive speed reduction as a distance or rotation command approaches its target. Prevents overshoot.
- **Voltage Compensation** — Scales PWM duty cycle to maintain 6V effective motor voltage as battery drains: `compensation = 6.0 / battery_voltage`.

### Drive Subsystem Internals

- **Drive Queue** — A builder (`DriveQueueBuilder`) that accumulates `DriveCommand` steps and submits them for sequential execution. A single `drive_queue_executor` task runs one queue at a time, emitting one queue-level completion.
- **Drive Task** — The main control loop (`drive()`). Polls active intents, steps idle when no intent is active, and handles interrupts. Selects over the command queue and the interrupt signal.
- **Intent** — A state machine that owns a specific motion behavior — rotation, distance drive, brake/coast settle, or idle. Each intent is an `ActiveIntent` variant carrying its controller state and completion flag. Commands that complete instantly (e.g., `Differential`) are not intents; they are fire-and-forget.
- **Dispatch** — The thin seam between the drive command queue and the control modules. Routes incoming `DriveCommand` envelopes, handles standby wake-up, and executes `IntentTeardown` descriptors on completion or interrupt. Owns no per-intent knowledge.
- **IntentTeardown** — An enum declaring what sensor streams must be stopped when an intent completes or is interrupted. Each `ActiveIntent` returns its teardown descriptor; the dispatch executes it. Keeps the dispatch thin — it knows to stop sensors but not which specific sensors each intent required.
- **InterruptKind** — An enum (`EmergencyBrake`, `Stop`, `CancelCurrent`) that specifies how the drive subsystem preempts the active intent. Sent via `send_drive_interrupt`.
- **EmergencyBrake** — An `InterruptKind::EmergencyBrake` sent when a combined obstacle is detected (LiDAR or rangefinder). Causes immediate active motor braking, cancels the active intent, bumps the command epoch, and drains queued commands. Mode-agnostic — dispatched unconditionally on any obstacle detection.
- **Epoch** — A monotonic counter incremented on each interrupt. Queued commands stamped with an old epoch are discarded when dequeued, preventing stale commands from executing after an interrupt.

## State Modules

Lock order (documented in each module): **power → calibration → perception → motion**

- **Power State** — Battery level (0–100%) and voltage. Accessed via `power::try_get_battery_voltage()` for hot-path readers.
- **Calibration State** — Motor calibration factors (`left_factor`/`right_factor`), IMU calibration status, distance calibration factor. Persisted to flash.
- **Perception State** — Dual-path architecture for obstacle detection. *Lock-free path:* `LIDAR_OBSTACLE`, `RANGEFINDER_OBSTACLE`, and `COMBINED_OBSTACLE` atomic booleans for hot-path reads. *Detailed path:* mutex-protected `LidarPointCloud` (360 distances, one per degree, with `sequence` counter for change detection) and `RangefinderReadings` (VL53L0X front-down distance).
- **ObstacleSource** — Enum (`Lidar` | `Rangefinder`) carried by `ObstacleDetected` events. Identifies which sensor triggered the detection.
- **ChangeDetected** — Enum returned by perception setters: `NoChange`, `ChangedToDetected`, `ChangedToCleared`. Enables edge-triggered reactions to obstacle state transitions without polling.
- **Motion State** — Track speeds, encoder pulse counts, computed speeds (cm/s), odometry. Lock-free atomic mirrors for high-frequency readers.

## Autonomous Modes

- **Coast-and-Avoid** — Drive forward until the combined obstacle flag signals detection. Brake, back up, random-angle turn (±45°–180° via nanorand), resume forward. The LiDAR stub runs threshold/cone logic internally; the mode reads only the pre-computed boolean. Simple, reliable.
- **Attempt Straight Line** — User sets target distance (100–1000 cm). The LiDAR point cloud is analyzed for navigable gaps; the widest gap within the forward cone (±60°) is chosen. The robot rotates toward the gap center, drives a leg, and repeats. IMU heading used for drift correction. Completes when target distance is reached or no forward path exists. *(Deferred — UI integration pending.)*

### Gap Analysis

- **Gap** — A contiguous angular arc in the LiDAR point cloud where no obstacle return is within threshold (default 30 cm). A valid gap has sufficient width at its constriction depth, is flanked by obstacles, and lies within the forward cone.
- **Constriction Depth** — The minimum distance across the angles that make up a gap. Determines how far the robot can safely travel through that gap.
- **Leg** — One rotate-then-drive maneuver through a chosen gap. The leg length is the constriction depth minus a safety margin, or the remaining target distance if shorter.
- **Accumulated Drift** — The signed sum of deviation angles across all legs. Later legs apply a correction toward the ideal straight line.
- **Correction Angle** — The gap midpoint angle that would cancel accumulated drift (= `-drift`). The gap closest to this angle is preferred.

## Calibration

- **Motor Calibration** — Per-track speed multipliers (`left_factor`/`right_factor`, range `(0.0, 1.0]`). The faster track is always the reference at 1.0; the slower track gets a factor < 1.0. Computed by running each track at fixed PWM and measuring encoder pulses.
- **Distance Calibration** — Single factor (range 0.5–2.0) mapping encoder pulses to real-world centimeters. Corrects for surface friction and tire wear.
- **IMU Calibration** — Magnetometer hard/soft iron calibration and motor interference compensation (`MagCalibration`). Gyroscope and accelerometer bias are handled internally by the ICM-20948 DMP.
- **Boot Sequence** — On `Initialize` event, the initialization module sends individual `GetData` commands to flash storage for motor, IMU, distance, and IMU flag calibration data. Each response raises a `CalibrationDataLoaded` event. When all four are received, the UI shows the main menu.
- **Flash Storage** — `sequential-storage` + `embedded-storage-async`. Saves motor calibration (2× f32 as `MotorCalibration`), distance factor (1× f32), and magnetometer calibration flags (`ImuCalibrationFlags`) to dedicated FLASH_STORAGE region (last 8KB of flash).

## Development Practices

- **No HIL (Hardware-in-the-Loop) rig** — The project has no automated hardware test rig. All tests that require physical hardware (sensors, motors) must be done manually on the target RP2350 board. `cargo check` and `cargo clippy` are the automated compile-time gates; the `coin-d6` crate additionally has host-side decoder/aggregation tests run with `cargo test -p coin-d6 --features std --target x86_64-unknown-linux-gnu` (the explicit host target is required because the default build target is `thumbv8m.main-none-eabihf`). Do not write `#[cfg(test)]` unit tests — they cannot exercise the real hardware and offer no value over compile-time checks. The `testmode` embassy tasks under `task/testmode/` serve as manual HIL verification procedures.
