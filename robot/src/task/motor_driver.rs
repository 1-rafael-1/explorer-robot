//! Motor Driver Task
//!
//! Controls 2× JGB37-520 motors (one per track) via a single TB6612FNG driver.
//! Direction and standby pins are direct GPIO.
//!
//! # Hardware Configuration

#![allow(clippy::missing_docs_in_private_items)]
//!
//! **Left Track Motor (PWM Slice 2):**
//! - PWM: GPIO 4 (PWM2A)
//! - Direction forward: GPIO 5 (direct)
//! - Direction backward: GPIO 6 (direct)
//!
//! **Right Track Motor (PWM Slice 3):**
//! - PWM: GPIO 7 (PWM3B)
//! - Direction forward: GPIO 8 (direct)
//! - Direction backward: GPIO 10 (direct)
//!
//! **Standby pin:** GPIO 12 (direct, active high)
//!
//! # Voltage Compensation
//!
//! Scales PWM duty to maintain 6V at the motors as the measured `+8V` rail
//! (`VM`) drains. The battery task stores this rail separately from the
//! pack-terminal voltage it displays.
//! At 8.4V rail: factor = 6.0/8.4 ≈ 0.714
//! At 6.0V rail: factor = 6.0/6.0 = 1.0
//!
//! # Calibration
//!
//! `SetTracks` resolves each requested track through the loaded Motor
//! Calibration Curve published in the calibration state leaf. Zero stays zero, a
//! magnitude below the Calibration Floor is passed through raw, and everything
//! else is the interpolated per-track, per-direction correction. An absent curve
//! is the identity, so an uncalibrated robot drives on the raw request. Voltage
//! compensation is applied afterwards.
//!
//! `SetTracksRaw` takes the same path but never reads the curve. The Motor
//! Calibration sweep uses it so it measures the raw drivetrain: a sweep that ran
//! through the loaded curve would measure the corrected duty and re-derive a
//! near-identity curve.
//!
//! # Architecture
//!
//! The motor driver uses a single-layer control system:
//! 1. **PWM Control**: Speed via PWM duty cycle on GPIO 4, 7
//! 2. **Direction Control**: Direct GPIO Output pins (forward/backward per track)
//! 3. **Standby**: Direct GPIO Output (active high, enable both driver halves)
//!
//! # Usage
//!
//! ```rust
//! // Drive straight at 50%
//! motor_driver::send_motor_command(MotorCommand::SetTracks { left_speed: 50, right_speed: 50 }).await;
//!
//! // Emergency stop
//! motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
//! ```

use defmt::{Format, debug, info, warn};
use embassy_rp::{
    gpio::{Level, Output},
    pwm::{Pwm, SetDutyCycle},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use embassy_time::{Duration, Instant, Timer};
use motor_cal::{Track as CurveTrack, resolve_track_speed};

use crate::system::state::{calibration, motion, power};

/// Target motor voltage (we compensate battery voltage down to this).
pub const TARGET_MOTOR_VOLTAGE: f32 = 6.0;

/// The left motor's drive leads are reversed relative to the right, so the
/// hardware output is the negation of the logically resolved left command.
///
/// The inversion is applied only to the left hardware output, after the curve
/// and voltage compensation are resolved, so the logical speeds, the curve and
/// the reported correction all stay logical. A curve saved under the old
/// polarity must be re-derived; the v4 pair calibration does that on its next
/// run.
const LEFT_MOTOR_INVERTED: bool = true;

/// Command queue depth — enough for drive task bursts without blocking.
const COMMAND_QUEUE_SIZE: usize = 16;

/// Command channel for motor control operations.
static MOTOR_COMMAND_CHANNEL: Channel<CriticalSectionRawMutex, MotorCommand, COMMAND_QUEUE_SIZE> = Channel::new();

/// Send a motor command to the driver task.
pub async fn send_motor_command(command: MotorCommand) {
    MOTOR_COMMAND_CHANNEL.sender().send(command).await;
}

async fn receive_motor_command() -> MotorCommand {
    MOTOR_COMMAND_CHANNEL.receiver().receive().await
}

// ── Public types ─────────────────────────────────────────────────────────────────

/// Track selection (left or right side of robot).
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum Track {
    Left,
    Right,
}

/// Motor direction states for the TB6612FNG.
#[derive(Debug, Clone, Copy, Eq, PartialEq, Format)]
pub enum MotorDirection {
    /// Hardware forward direction pin.
    Forward,
    /// Hardware backward direction pin; the drive vocabulary calls this `Reverse`.
    Backward,
    /// Both direction pins low (freewheel).
    Coast,
    /// Both direction pins high (electrical brake).
    Brake,
}

/// Motor control commands.
///
/// `SetTracks` is the primary calibrated command for normal operation.
/// `SetTracksRaw` is the same command without the curve resolution, for a
/// procedure that must measure the raw drivetrain.
/// `BrakeAll`, `CoastAll` are raw/safety commands.
#[derive(Debug, Clone, Copy, Format)]
pub enum MotorCommand {
    /// Set both tracks at once (calibrated + voltage compensated).
    /// Speed range: -100 (full backward) to +100 (full forward).
    SetTracks { left_speed: i8, right_speed: i8 },

    /// Set both tracks at once without consulting the calibration curve
    /// (raw + voltage compensated). Speed range: -100 to +100.
    ///
    /// This is the sweep's command: the Motor Calibration must measure the raw
    /// drivetrain, not the loaded curve it is about to replace.
    SetTracksRaw { left_speed: i8, right_speed: i8 },

    /// Brake all motors (emergency stop).
    BrakeAll,

    /// Coast all motors (freewheeling).
    CoastAll,

    /// Enable/disable the motor driver via standby pin.
    /// When disabled, all motor outputs are off (low power).
    SetAllDriversEnable { enabled: bool },
}

// ── Internal types ───────────────────────────────────────────────────────────────

/// Motor direction pin pair (forward, backward) on direct GPIO.
struct MotorPins<'a> {
    forward: Output<'a>,
    backward: Output<'a>,
}

impl MotorPins<'_> {
    fn set_direction(&mut self, direction: MotorDirection) {
        match direction {
            MotorDirection::Forward => {
                self.forward.set_high();
                self.backward.set_low();
            }
            MotorDirection::Backward => {
                self.forward.set_low();
                self.backward.set_high();
            }
            MotorDirection::Coast => {
                self.forward.set_low();
                self.backward.set_low();
            }
            MotorDirection::Brake => {
                self.forward.set_high();
                self.backward.set_high();
            }
        }
    }
}

/// PWM channel mapping for the two motors.
struct PwmChannels {
    left: embassy_rp::pwm::PwmOutput<'static>,
    right: embassy_rp::pwm::PwmOutput<'static>,
}

impl PwmChannels {
    fn set_speed(&mut self, track: Track, speed: i16) {
        let pwm = match track {
            Track::Left => &mut self.left,
            Track::Right => &mut self.right,
        };
        let _ = pwm.set_duty_cycle_fraction(speed.unsigned_abs(), 1000);
    }

    fn set_both(&mut self, left_speed: i16, right_speed: i16) {
        self.set_speed(Track::Left, left_speed);
        self.set_speed(Track::Right, right_speed);
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────────

/// Convert speed value to motor direction.
const fn speed_to_direction(speed: i16) -> MotorDirection {
    match speed {
        s if s > 0 => MotorDirection::Forward,
        s if s < 0 => MotorDirection::Backward,
        _ => MotorDirection::Coast,
    }
}

/// Minimum time between the motors becoming idle and a rail sample being
/// trusted, so a sample taken while the rail is still recovering is rejected.
const RAIL_IDLE_SETTLE: Duration = Duration::from_millis(250);

/// Get the latest motor-rail sample from system state (truly non-blocking).
/// Returns the voltage and the time it was measured, or `None` if no sample is
/// available yet or if the mutex is busy.
fn try_get_rail_sample() -> Option<(f32, Instant)> {
    power::try_get_rail_sample()
}

/// Wait for the first motor-rail voltage reading from system state.
async fn wait_for_rail_voltage() -> f32 {
    loop {
        if let Some((voltage, _)) = try_get_rail_sample() {
            return voltage;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// The PWM scale the driver applies at `rail_volts`.
///
/// This is the single rule both the driver and the Motor Calibration record use:
/// the 6 V target over the measured rail, or `1.0` when the rail is at or below
/// the target and cannot be compensated. It is pure, so a caller can record the
/// compensation for a rail it read at command time.
pub fn voltage_compensation(rail_volts: f32) -> f32 {
    if rail_volts < TARGET_MOTOR_VOLTAGE {
        1.0
    } else {
        TARGET_MOTOR_VOLTAGE / rail_volts
    }
}

/// Calculate the voltage compensation factor from the measured motor rail.
///
/// Scales duty cycle to maintain 6V at the motors as the rail drains.
///
/// Examples:
/// - At 8.4V rail: factor = 6.0/8.4 = 0.714 (71.4% duty cycle for 100% speed)
/// - At 7.2V rail: factor = 6.0/7.2 = 0.833 (83.3% duty cycle for 100% speed)
/// - At 6.0V rail: factor = 6.0/6.0 = 1.000 (100% duty cycle for 100% speed)
fn calculate_voltage_compensation(rail_voltage: f32) -> f32 {
    if rail_voltage < TARGET_MOTOR_VOLTAGE {
        warn!(
            "Motor rail {}V below target {}V — cannot compensate",
            rail_voltage, TARGET_MOTOR_VOLTAGE
        );
    }
    voltage_compensation(rail_voltage)
}

// ── Command processing ───────────────────────────────────────────────────────────

/// Process a `SetTracks`-family command: resolve each track through the loaded
/// curve when `apply_curve` is set, apply voltage compensation, set direction
/// pins and PWM, then publish the requested speeds to motion state.
///
async fn process_set_tracks(
    pwm: &mut PwmChannels,
    left_pins: &mut MotorPins<'_>,
    right_pins: &mut MotorPins<'_>,
    voltage_comp: f32,
    apply_curve: bool,
    left: i8,
    right: i8,
) {
    // Copy the curve out of the leaf; the lock is released before motion state
    // is touched, so the documented lock order is never inverted. A raw command
    // never consults the curve at all, so a calibration sweep measures the
    // drivetrain rather than the curve it is about to replace.
    let curve = if apply_curve {
        calibration::get_motor_curve().await
    } else {
        None
    };
    let final_left = resolve_track_speed(curve.as_ref(), CurveTrack::Left, left, voltage_comp);
    let final_right = resolve_track_speed(curve.as_ref(), CurveTrack::Right, right, voltage_comp);

    // The left leads are reversed relative to the right, so the physical output
    // is the negation of the logically resolved command. Apply it here, after
    // the curve and voltage compensation, and only to the left output, so the
    // curve, the reported correction and motion state all stay logical. A saved
    // curve from the old polarity must be re-derived; the v4 pair calibration
    // will do it.
    let hardware_left = if LEFT_MOTOR_INVERTED { -final_left } else { final_left };

    debug!(
        "SetTracks{}: logical [{}, {}] -> resolved_permille [{}, {}] (vcomp: {}); hardware_permille left {} (inverted={})",
        if apply_curve { "" } else { " (raw)" },
        left,
        right,
        final_left,
        final_right,
        voltage_comp,
        hardware_left,
        LEFT_MOTOR_INVERTED
    );
    #[cfg(any(feature = "bench_motor_calibrate", feature = "bench_motor_measure"))]
    info!(
        "bench-motor: left requested={} resolved_permille={} hardware_permille={} inverted={}",
        left, final_left, hardware_left, LEFT_MOTOR_INVERTED
    );

    left_pins.set_direction(speed_to_direction(hardware_left));
    right_pins.set_direction(speed_to_direction(final_right));
    pwm.set_both(hardware_left, final_right);

    // Publish the requested (pre-correction) speeds to motion state so other
    // tasks (drive, orchestrator) see what was asked for, not the applied duty.
    motion::set_track_speeds(left, right).await;
}

/// Process a motor command.
#[allow(clippy::too_many_lines)]
async fn process_command(
    pwm: &mut PwmChannels,
    left_pins: &mut MotorPins<'_>,
    right_pins: &mut MotorPins<'_>,
    standby: &mut Output<'_>,
    voltage_comp: f32,
    command: MotorCommand,
) {
    match command {
        MotorCommand::SetTracks {
            left_speed,
            right_speed,
        } => {
            process_set_tracks(pwm, left_pins, right_pins, voltage_comp, true, left_speed, right_speed).await;
        }
        MotorCommand::SetTracksRaw {
            left_speed,
            right_speed,
        } => {
            process_set_tracks(pwm, left_pins, right_pins, voltage_comp, false, left_speed, right_speed).await;
        }
        MotorCommand::BrakeAll => {
            debug!("Braking all motors");
            left_pins.set_direction(MotorDirection::Brake);
            right_pins.set_direction(MotorDirection::Brake);
            pwm.set_both(0, 0);
            motion::set_track_speeds(0, 0).await;
        }
        MotorCommand::CoastAll => {
            debug!("Coasting all motors");
            left_pins.set_direction(MotorDirection::Coast);
            right_pins.set_direction(MotorDirection::Coast);
            pwm.set_both(0, 0);
            motion::set_track_speeds(0, 0).await;
        }
        MotorCommand::SetAllDriversEnable { enabled } => {
            info!("Motor driver enable: {}", enabled);
            if enabled {
                standby.set_high();
            } else {
                standby.set_low();
            }
        }
    }
}

// ── Main task ────────────────────────────────────────────────────────────────────

/// Motor driver task — processes commands and controls PWM + GPIO direction pins.
///
/// # Arguments
/// * `pwm_left` — PWM slice for left track motor (GPIO 4, PWM2 CH A)
/// * `pwm_right` — PWM slice for right track motor (GPIO 7, PWM3 CH B)
/// * `left_fwd` — GPIO for left motor forward direction (GPIO 5)
/// * `left_bwd` — GPIO for left motor backward direction (GPIO 6)
/// * `right_fwd` — GPIO for right motor forward direction (GPIO 8)
/// * `right_bwd` — GPIO for right motor backward direction (GPIO 10)
/// * `standby_pin` — GPIO for TB6612FNG standby pin (GPIO 12, active high)
#[allow(clippy::similar_names)]
#[embassy_executor::task]
pub async fn motor_driver(
    pwm_left: Pwm<'static>,
    pwm_right: Pwm<'static>,
    left_fwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_5>,
    left_bwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_6>,
    right_fwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_8>,
    right_bwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_10>,
    standby_pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_12>,
) {
    info!("Motor driver task starting");

    // Split PWM slices into individual channel outputs. The left motor uses
    // channel A (GPIO 4, PWM2A) and the right motor channel B (GPIO 7, PWM3B),
    // so each slice yields the channel in a different position.
    let (left_ch, _) = pwm_left.split();
    let (_, right_ch) = pwm_right.split();

    let (Some(left_pwm), Some(right_pwm)) = (left_ch, right_ch) else {
        warn!("PWM channels not configured; motor driver exiting");
        return;
    };

    let mut pwm = PwmChannels {
        left: left_pwm,
        right: right_pwm,
    };

    // Initialize direction pins — start in coast
    let mut left_pins = MotorPins {
        forward: Output::new(left_fwd, Level::Low),
        backward: Output::new(left_bwd, Level::Low),
    };

    let mut right_pins = MotorPins {
        forward: Output::new(right_fwd, Level::Low),
        backward: Output::new(right_bwd, Level::Low),
    };

    // Standby pin — start disabled for safety
    let mut standby = Output::new(standby_pin, Level::Low);

    // The applied curve is read from the calibration state leaf on each
    // `SetTracks`, so the driver holds no calibration copy of its own.

    // Ensure motion state is zeroed
    motion::set_track_speeds(0, 0).await;

    // The motors start idle; this timestamp lets the loop reject rail samples
    // taken before the last stop.
    let mut idle_since = Instant::now();

    // Wait for the first motor-rail reading before allowing motor commands
    info!("Waiting for motor rail voltage reading...");
    let initial_voltage = wait_for_rail_voltage().await;
    let mut voltage_compensation = calculate_voltage_compensation(initial_voltage);
    info!(
        "Motor rail: {}V, voltage compensation: {}",
        initial_voltage, voltage_compensation
    );

    // Enable driver
    standby.set_high();
    info!("Motor driver enabled");

    // Track whether motors are currently active (any motor speed != 0)
    let mut motors_active = false;

    // Main command processing loop
    loop {
        let command = receive_motor_command().await;

        let was_motors_active = motors_active;
        let next_motors_active = match &command {
            MotorCommand::SetTracks {
                left_speed,
                right_speed,
            }
            | MotorCommand::SetTracksRaw {
                left_speed,
                right_speed,
            } => *left_speed != 0 || *right_speed != 0,
            MotorCommand::BrakeAll | MotorCommand::CoastAll => false,
            MotorCommand::SetAllDriversEnable { enabled } => {
                if *enabled {
                    was_motors_active
                } else {
                    false
                }
            }
        };

        // Record when the motors last became idle. A rail sample older than
        // this timestamp was taken while they were loaded.
        if was_motors_active && !next_motors_active {
            idle_since = Instant::now();
        }

        // Refresh the factor at a drive start or an idle-to-idle command, but
        // only from a sample taken after the motors last became idle. Mid-drive
        // commands and the stop transition keep the current factor: the loaded
        // rail sags, and chasing it would increase duty and sag further.
        if !was_motors_active
            && let Some((current_voltage, sampled_at)) = try_get_rail_sample()
            && sampled_at.saturating_duration_since(idle_since) >= RAIL_IDLE_SETTLE
        {
            let new_comp = calculate_voltage_compensation(current_voltage);
            if (new_comp - voltage_compensation).abs() > 0.01 {
                info!(
                    "Voltage compensation updated: {} -> {} (idle sample, {}V rail)",
                    voltage_compensation, new_comp, current_voltage
                );
                voltage_compensation = new_comp;
            }
        }

        motors_active = next_motors_active;
        process_command(
            &mut pwm,
            &mut left_pins,
            &mut right_pins,
            &mut standby,
            voltage_compensation,
            command,
        )
        .await;
    }
}
