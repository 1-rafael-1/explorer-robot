# Explorer Robot v3 — Scaffold and Core Port

## Problem Statement

simple-robot v2 reached the limits of its hardware platform. Four TT motors with cheap hall encoders, an HC-SR04 ultrasonic sensor with servo sweeper, a shared I2C bus with a PCA9555 port expander, and a pin-exhausted RP2350 prevented the spatial awareness and navigation precision the original vision demanded. The platform works but cannot go further — every remaining limitation is a hardware constraint, not a software one.

v3 must start fresh on new hardware while preserving the v2 firmware architecture that proved itself: the event system, orchestrator, drive command model, state modules, and UI subsystem.

## Solution

A new explorer-robot firmware built from the v2 codebase but adapted to a simplified, more capable hardware set: two JGB37-520 encoder motors (one per track), a single TB6612FNG motor driver, a COIN-D6 360° spinning LiDAR for spatial awareness, four VL53L0X laser rangefinders for collision control and stair detection, and the ICM-20948 IMU moved to SPI. The port expander, ultrasonic sensor, IR sensors, and RC receiver are removed. The v3 scaffold ports the core architecture and strips dead hardware, with sensor stubs for the new devices.

## User Stories

1. As a robot operator, I want the firmware to build and flash onto the RP2350, so that I can begin testing the new hardware.
2. As a robot operator, I want two JGB37-520 motors to respond to speed commands via the TB6612FNG driver, so that the robot can move.
3. As a robot operator, I want hall encoder pulses from both motors to be read and counted, so that distance and speed can be measured.
4. As a robot operator, I want motor speed to be compensated for battery voltage sag, so that the robot drives consistently as the battery drains.
5. As a robot operator, I want per-track calibration factors to balance left and right track speeds, so that the robot drives straight.
6. As a robot operator, I want calibration data persisted to flash and loaded at boot, so that I don't have to recalibrate after every power cycle.
7. As a robot operator, I want the event system and orchestrator to route sensor events to behavior handlers, so that autonomous modes can respond to the environment.
8. As a robot operator, I want the OLED display and rotary encoder to show a menu system and hardware test screens, so that I can verify and calibrate components.
9. As a robot operator, I want the robot to drive forward and avoid obstacles using LiDAR data (coast-and-avoid mode), so that it navigates autonomously in simple environments.
10. As a robot operator, I want the robot to navigate toward a target distance by analyzing LiDAR gaps (attempt-straight-line mode), so that it can traverse more complex spaces.
11. As a robot operator, I want the four VL53L0X rangefinders to detect nearby obstacles and stairs, so that the robot doesn't collide with walls or fall off ledges.
12. As a robot operator, I want the ICM-20948 IMU to provide orientation data over SPI, so that heading is available for navigation and drift correction.
13. As a robot operator, I want the RGB LED to indicate battery state and obstacle alerts, so that I can see the robot's status at a glance.
14. As a future developer, I want stub implementations of the LiDAR and VL53L0X sensors, so that the autonomous modes can be developed and tested before real sensor drivers are ready.

## Implementation Decisions

### Hardware Configuration

- **Motors**: 2× JGB37-520 6V 165RPM with single-channel hall encoders, one per track. Both motors on one TB6612FNG driver (2 of 4 channels used).
- **Motor control pins**: 2× PWM (one per motor), 4× GPIO for direction (two per motor), 1× GPIO for driver standby. All direct GPIO — no port expander.
- **Encoders**: 2× PWM input mode for pulse counting. Single-channel — direction inferred from motor command, not quadrature.
- **IMU**: ICM-20948 on SPI bus (CS, SCK, MOSI, MISO). Faster than I2C, frees I2C bus for other devices.
- **Display**: SSD1306 OLED on I2C bus, carried over from v2.
- **Rangefinders**: 4× VL53L0X on I2C bus, sharing SDA/SCL with OLED. Single XSHUT line for address assignment at boot. Three front-facing (edge + downward), one rear-facing.
- **LiDAR**: COIN-D6 360° dTOF spinning LiDAR on dedicated UART. Streams continuous scan data.
- **Input**: EC11 rotary encoder with push button, carried over.
- **Status**: RGB LED (common cathode), carried over.
- **Power**: 2S LiPo (8.4V max) → LM2596 buck to 5V. Motors run off battery voltage with software compensation targeting 6V. Battery voltage read via ADC.
- **Reserved**: UART pins for future AI camera module. RC receiver dropped.

### Core Split

- **Core0**: orchestrator, drive subsystem, encoder reader, UI (OLED, rotary encoder, RGB LED), I2C bus (OLED + VL53L0X array). Owns the main event loop.
- **Core1**: LiDAR parser task. Receives continuous UART stream from COIN-D6, parses packets into a shared point cloud. Future: laser rangefinder processing.

### Ported Architecture (from v2)

- **Event system**: Typed events dispatched by the orchestrator to registered behavior handlers. Carried over as-is.
- **Orchestrator**: Central event loop in `task/orchestrate.rs`. Carried over as-is.
- **State modules**: `power` (battery voltage, LED mode), `motion` (encoder counts, speed, odometry), `perception` (point cloud from LiDAR, rangefinder readings — significant rewrite from v2's sweep buffer), `calibration` (motor factors, IMU calibration). Same lock order: power → calibration → perception → motion.
- **Drive command model**: Async channel-based queue (`SetTracks` as primary command), single-producer/single-executor. Simplified from v2: no `SetAllMotors`, `SetTrack` sets both motors on a track (only one motor per track now). Voltage compensation and calibration applied.
- **UI subsystem**: Menu system, hardware test screens, calibration routines. Carried over, adapted for new hardware tests.
- **Calibration persistence**: `sequential-storage` + `embedded-storage-async` for flash storage. Carried over.

### Simplifications from v2

- Motor driver: 2 motors instead of 4, no front/rear per track, no port expander. Direction pins are direct GPIO.
- Encoder reader: 2 channels instead of 4.
- Perception state: LiDAR point cloud replaces ultrasonic sweep buffer. No cone correction, no servo management.
- Sensors module: HC-SR04 driver, servo control, IR sensor polling all removed. Replaced by LiDAR parser and VL53L0X stubs.
- Control module: RC receiver input removed. AI camera UART reserved but not implemented.

### Dependencies (from v2 Cargo.toml)

- **Removed**: `hcsr04_async`, `moving_median`, `port-expander`, all network/USB/Pico-W crates from template.
- **Kept**: `embassy-rp`, `embassy-sync`, `embassy-executor`, `embassy-time`, `defmt`, `defmt-rtt`, `icm20948-rs` (SPI mode), `ssd1306-async`, `embedded-graphics`, `nalgebra`, `sequential-storage`, `embedded-storage-async`, `heapless`, `nanorand`, `micromath`, `libm`, `static_cell`, `embedded-hal-async`.
- **Added**: VL53L0X driver and COIN-D6 LiDAR parser to be added when real drivers are ready. Stubbed for now.

### Autonomous Modes

- **Coast-and-avoid**: Drive forward until LiDAR detects obstacle within threshold, back up, random turn. Simple, reliable. Uses crude LiDAR distance threshold instead of ultrasonic readings.
- **Attempt straight line**: User sets target distance. LiDAR-based gap analysis replaces ultrasonic sweep gap analysis. Navigates leg-by-leg toward goal while avoiding obstacles. Drift correction via IMU.

### Implementation Order

1. Scaffold: clean `Cargo.toml`, module structure, `CONTEXT.md`, build config
2. Motor driver: simplified 2-motor, 1-TB6612FNG, direct GPIO
3. Encoder reader: 2-channel pulse counting
4. State modules: port `power`, `motion`, `perception` (stubbed), `calibration`
5. Drive task: `SetTracks` as primary interface
6. Event system + orchestrator: port as-is
7. UI subsystem: port OLED, rotary encoder, RGB LED
8. Sensor stubs: VL53L0X and LiDAR mock tasks
9. Autonomous modes: coast-and-avoid + attempt-straight-line

## Testing Decisions

Embedded Rust on bare metal without a hardware-in-the-loop rig means automated tests are impractical for now. Testing is deferred to on-hardware verification:

- Each implementation step is verified by flashing to the RP2350 and observing behavior via defmt logs over RTT.
- Motor commands are verified by observing encoder pulse counts and physical movement.
- UI is verified by navigating the menu with the rotary encoder and observing OLED output.
- Autonomous modes are verified by placing the robot in a controlled environment and observing drive behavior.

What makes a good test in this context: a step is complete when it can be flashed to hardware and the expected behavior is observed through defmt logs and physical indicators (motor movement, LED state, OLED display).

## Out of Scope

- Real VL53L0X driver implementation (stubbed for now)
- Real COIN-D6 LiDAR driver implementation (stubbed for now)
- AI camera module integration (UART pins reserved only)
- ROS2 integration
- SLAM or mapping
- Waypoint navigation beyond attempt-straight-line
- New autonomous modes beyond coast-and-avoid and attempt-straight-line
- Custom chassis design (separate project)
- PCB design (separate project)
- Automated test suite (no HIL rig available)

## Further Notes

This spec covers the scaffold and core port only. The real sensor drivers (VL53L0X, COIN-D6) will be implemented in follow-up work once the core architecture is running on hardware. The stub sensors emit placeholder data that is structurally valid but not physically meaningful, allowing the autonomous mode logic to be developed and tested against known inputs.

The v2 codebase is the primary reference for all ported modules. Where v3 simplifies (fewer motors, no port expander, no ultrasonic), code is adapted rather than rewritten from scratch. Where v3 adds new capabilities (LiDAR point cloud, SPI IMU), new modules are created following the existing architectural patterns.
