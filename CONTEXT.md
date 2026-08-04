# Explorer Robot — Domain Glossary

> This glossary records the canonical terms for the explorer-robot v3 firmware. It is a living document — update it whenever a term is resolved during design or implementation.

## Hardware

- **Track** — Left or right side of the robot. Each track has one JGB37-520 motor mechanically coupled to the track (treads or wheels).
- **Motor** — A single JGB37-520 6V 165RPM DC motor with hall encoder. v3 has two motors (one per track); v2 had four (two per track: front and rear).
- **Motor Driver** — The TB6612FNG dual H-bridge chip. Controls direction (via direct GPIO) and speed (via PWM) for both motors. One standby pin enables/disables the driver.
- **Encoder** — Single-channel hall sensor on each JGB37-520 motor. Produces 8 pulses per motor revolution, 1320 pulses per output shaft revolution (165:1 gear ratio). Pulse counting via PWM input mode. Direction is inferred from the motor command, not quadrature.
- **IMU** — ICM-20948 9-axis inertial measurement unit (accelerometer, gyroscope, magnetometer). Connected via SPI bus (CS, SCK, MOSI, MISO). v2 used I2C.
- **LiDAR** — COIN-D6 360° spinning dTOF LiDAR on dedicated UART. Emits continuous scan data as a point cloud (one distance per degree). Owned by core1 (currently stubbed for development).
- **Rangefinder** — VL53L0X time-of-flight laser rangefinder. Four units on I2C bus: front-left (side collision), front-center (forward collision), front-down (stair/drop detection, angled downward), and rear. Single XSHUT line for address assignment at boot (currently stubbed for development).
- **OLED** — SSD1306 128×64 monochrome display on I2C bus, shared with VL53L0X array.
- **Rotary Encoder** — EC11 quadrature rotary encoder with push button. Used for menu navigation (rotation) and selection (push).
- **RGB LED** — Common-cathode RGB LED. Indicates battery state (green → yellow → red) and obstacle alerts (flashing red).
- **Battery** — 2S LiPo (8.4V max). Voltage read via ADC. Motors compensated to 6V target.
- **Standby Pin** — Direct GPIO that enables/disables the TB6612FNG motor driver. Active high.

## Architecture

- **Core0** — Runs the orchestrator, drive subsystem, encoder reader, UI (OLED, rotary encoder, RGB LED), I2C bus (OLED + VL53L0X array), and the main event loop.
- **Core1** — Runs the LiDAR task. Currently runs a synthetic point-cloud stub; real COIN-D6 UART parsing is planned.
- **Event System** — Typed, multi-producer single-consumer event channel (capacity 64). Tasks raise events; the orchestrator consumes and dispatches them to behavior handlers.
- **Orchestrator** — Central event loop in `task/orchestrate.rs`. Receives events and dispatches them to registered behavior handlers (battery, obstacle, input, calibration, autonomous modes).
- **Behavior Handler** — A module that responds to specific event types. Registered with the orchestrator. Handlers are functions that respond to specific event types by reading system state and issuing commands.
- **Drive Intent** — A command placed on the drive priority queue. Carries a `DriveCommand` variant, priority level, and preemption flag. Higher-priority intents (e.g., emergency brake) preempt lower ones.
- **Drive Queue** — Priority queue that holds pending drive intents. Drained by the drive task. Supports preemption.
- **Drive Task** — Consumes encoder data, computes speed/odometry, issues calibrated motor commands via the intent queue.
- **SetTracks** — The primary drive command. Sets left and right track speeds (-100 to +100). Calibration and voltage compensation are applied.
- **Voltage Compensation** — Scales PWM duty cycle to maintain 6V effective motor voltage as battery drains. Formula: `compensation = 6.0 / battery_voltage`.

## State Modules

Lock order (documented in each module): **power → calibration → perception → motion**

- **Power State** — Battery level (0–100%) and voltage. Accessed via `power::try_get_battery_voltage()` for hot-path readers.
- **Calibration State** — Motor calibration factors (`left_factor`/`right_factor`), IMU calibration status, distance calibration factor. Persisted to flash.
- **Perception State** — Dual-path architecture: `LIDAR_OBSTACLE`, `RANGEFINDER_OBSTACLE`, and `COMBINED_OBSTACLE` atomics for lock-free obstacle checks on the hot path, plus a mutex-protected LiDAR point cloud (360 distances, one per degree) and VL53L0X rangefinder readings (4 sensors). Replaces v2's ultrasonic sweep buffer.
- **Motion State** — Track speeds, encoder pulse counts, computed speeds (cm/s), odometry. Lock-free atomic mirrors for high-frequency readers.

## Autonomous Modes

- **Coast-and-Avoid** — Drive forward until LiDAR detects obstacle within threshold (default 30 cm). Brake, back up, random-angle turn (±45°–180° via nanorand), resume forward. Simple, reliable.
- **Attempt Straight Line** — User sets target distance (100–1000 cm). LiDAR point cloud analyzed for gaps: find widest gap within forward cone (±60°), orient toward gap center, drive a leg, repeat. IMU heading used for drift correction. Completes when target distance reached or dead-end encountered.

## Calibration

- **Motor Calibration** — Per-track speed multipliers (`left_factor`/`right_factor`, range 0.5–1.5). Computed by running each motor at fixed PWM and measuring encoder pulses. Compensates for manufacturing variation between motors.
- **Distance Calibration** — Single factor mapping encoder pulses to real-world centimeters.
- **IMU Calibration** — Magnetometer hard/soft iron calibration and motor interference compensation (`MagCalibration`). Gyroscope and accelerometer bias are handled internally by the ICM-20948 DMP. Carried over from v2.
- **Flash Storage** — `sequential-storage` + `embedded-storage-async`. Saves motor calibration (2× f32 as `MotorCalibration`), distance factor (1× f32), and magnetometer calibration flags (`ImuCalibrationFlags`) to dedicated FLASH_STORAGE region (last 8KB of flash).
