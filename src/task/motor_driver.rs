//! Motor Driver Task
//!
//! Controls 2× JGB37-520 motors (one per track) via a single TB6612FNG driver.
//! Direction and standby pins are direct GPIO.
//!
//! # Hardware Configuration

#![allow(clippy::missing_docs_in_private_items)]
//!
//! **Left Track Motor (PWM Slice 0):**
//! - PWM: GPIO 0 (PWM0A)
//! - Direction forward: GPIO 1 (direct)
//! - Direction backward: GPIO 2 (direct)
//!
//! **Right Track Motor (PWM Slice 1):**
//! - PWM: GPIO 3 (PWM1A)
//! - Direction forward: GPIO 4 (direct)
//! - Direction backward: GPIO 5 (direct)
//!
//! **Standby pin:** GPIO 6 (direct, active high)
//!
//! # Voltage Compensation
//!
//! Scales PWM duty to maintain 6V to motors as battery drains.
//! At 8.4V battery: factor = 6.0/8.4 ≈ 0.714
//! At 6.0V battery: factor = 6.0/6.0 = 1.0
//!
//! # Calibration
//!
//! `MotorCalibration` has `left_factor` and `right_factor` multipliers (0.5–1.5).
//! Applied in `SetTracks` after voltage compensation to balance left/right tracks.
//!
//! # Architecture
//!
//! The motor driver uses a single-layer control system:
//! 1. **PWM Control**: Speed via PWM duty cycle on GPIO 0, 3
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
use embassy_time::{Duration, Timer};

use crate::system::state::{motion, power};

/// Target motor voltage (we compensate battery voltage down to this).
const TARGET_MOTOR_VOLTAGE: f32 = 6.0;

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
    Forward,
    Backward,
    Coast,
    Brake,
}

/// Motor control commands.
///
/// `SetTracks` is the primary calibrated command for normal operation.
/// Calibration and voltage compensation are applied.
/// `BrakeAll`, `CoastAll` are raw/safety commands.
#[derive(Debug, Clone, Copy, Format)]
pub enum MotorCommand {
    /// Set both tracks at once (calibrated + voltage compensated).
    /// Speed range: -100 (full backward) to +100 (full forward).
    SetTracks { left_speed: i8, right_speed: i8 },

    /// Brake all motors (emergency stop).
    BrakeAll,

    /// Coast all motors (freewheeling).
    CoastAll,

    /// Enable/disable the motor driver via standby pin.
    /// When disabled, all motor outputs are off (low power).
    SetAllDriversEnable { enabled: bool },

    /// Load calibration data from flash storage.
    LoadCalibration(MotorCalibration),

    /// Update both calibration factors at once.
    UpdateAllCalibration { left_factor: f32, right_factor: f32 },
}

/// Per-track calibration factors for speed balancing.
///
/// Each factor is a multiplier (0.5 to 1.5) applied to commanded speed.
/// Default is 1.0 (no correction).
#[derive(Debug, Clone, Copy, Format)]
pub struct MotorCalibration {
    pub left_factor: f32,
    pub right_factor: f32,
}

impl Default for MotorCalibration {
    fn default() -> Self {
        Self {
            left_factor: 1.0,
            right_factor: 1.0,
        }
    }
}

impl MotorCalibration {
    const MIN_FACTOR: f32 = 0.5;
    const MAX_FACTOR: f32 = 1.5;

    /// Create new calibration with specified factors (clamped to valid range).
    pub const fn new(left_factor: f32, right_factor: f32) -> Self {
        Self {
            left_factor: left_factor.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR),
            right_factor: right_factor.clamp(Self::MIN_FACTOR, Self::MAX_FACTOR),
        }
    }

    /// Get calibration factor for a track.
    pub const fn get_factor(&self, track: Track) -> f32 {
        match track {
            Track::Left => self.left_factor,
            Track::Right => self.right_factor,
        }
    }

    /// Apply calibration to a commanded speed, clamping to [-100, 100].
    #[allow(clippy::cast_possible_truncation)]
    pub fn apply(&self, track: Track, speed: i8) -> i8 {
        let factor = self.get_factor(track);
        let calibrated_f32 = f32::from(speed) * factor;
        let calibrated = if calibrated_f32 >= 0.0 {
            (calibrated_f32 + 0.5) as i8
        } else {
            (calibrated_f32 - 0.5) as i8
        };
        calibrated.clamp(-100, 100)
    }

    /// Apply calibration AND voltage compensation.
    ///
    /// Two corrections in sequence:
    /// 1. Motor calibration factor (compensates for motor variations)
    /// 2. Voltage compensation factor (compensates for battery voltage)
    #[allow(clippy::cast_possible_truncation)]
    pub fn apply_with_voltage_compensation(&self, track: Track, speed: i8, voltage_comp: f32) -> i8 {
        let calibrated = self.apply(track, speed);
        let final_f32 = f32::from(calibrated) * voltage_comp;
        let result = if final_f32 >= 0.0 {
            (final_f32 + 0.5) as i8
        } else {
            (final_f32 - 0.5) as i8
        };
        result.clamp(-100, 100)
    }
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
    fn set_speed(&mut self, track: Track, speed: i8) {
        let pwm = match track {
            Track::Left => &mut self.left,
            Track::Right => &mut self.right,
        };
        let _ = pwm.set_duty_cycle_percent(speed.unsigned_abs());
    }

    fn set_both(&mut self, left_speed: i8, right_speed: i8) {
        self.set_speed(Track::Left, left_speed);
        self.set_speed(Track::Right, right_speed);
    }
}

// ── Helpers ──────────────────────────────────────────────────────────────────────

/// Convert speed value to motor direction.
const fn speed_to_direction(speed: i8) -> MotorDirection {
    match speed {
        s if s > 0 => MotorDirection::Forward,
        s if s < 0 => MotorDirection::Backward,
        _ => MotorDirection::Coast,
    }
}

/// Get current battery voltage from system state (truly non-blocking).
/// Returns None if no voltage reading available yet OR if mutex is busy.
fn try_get_battery_voltage() -> Option<f32> {
    power::try_get_battery_voltage()
}

/// Wait for first battery voltage reading from system state.
async fn wait_for_battery_voltage() -> f32 {
    loop {
        if let Some(voltage) = try_get_battery_voltage() {
            return voltage;
        }
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Calculate voltage compensation factor.
///
/// Scales duty cycle to maintain 6V output to motors as battery drains.
///
/// Examples:
/// - At 8.4V battery: factor = 6.0/8.4 = 0.714 (71.4% duty cycle for 100% speed)
/// - At 7.2V battery: factor = 6.0/7.2 = 0.833 (83.3% duty cycle for 100% speed)
/// - At 6.0V battery: factor = 6.0/6.0 = 1.000 (100% duty cycle for 100% speed)
fn calculate_voltage_compensation(battery_voltage: f32) -> f32 {
    if battery_voltage < TARGET_MOTOR_VOLTAGE {
        warn!(
            "Battery voltage {}V below target {}V — cannot compensate",
            battery_voltage, TARGET_MOTOR_VOLTAGE
        );
        1.0
    } else {
        TARGET_MOTOR_VOLTAGE / battery_voltage
    }
}

// ── Command processing ───────────────────────────────────────────────────────────

/// Process a `SetTracks` command: apply calibration + voltage compensation,
/// set direction pins and PWM, then publish speeds to motion state.
async fn process_set_tracks(
    pwm: &mut PwmChannels,
    left_pins: &mut MotorPins<'_>,
    right_pins: &mut MotorPins<'_>,
    cal: &MotorCalibration,
    voltage_comp: f32,
    left: i8,
    right: i8,
) {
    let final_left = cal.apply_with_voltage_compensation(Track::Left, left, voltage_comp);
    let final_right = cal.apply_with_voltage_compensation(Track::Right, right, voltage_comp);

    debug!(
        "SetTracks: [{}, {}] -> calibrated [{}, {}] (vcomp: {})",
        left, right, final_left, final_right, voltage_comp
    );

    left_pins.set_direction(speed_to_direction(final_left));
    right_pins.set_direction(speed_to_direction(final_right));
    pwm.set_both(final_left, final_right);

    // Publish the commanded (pre-calibration) speeds to motion state
    // so other tasks (drive, orchestrator) can read them.
    motion::set_track_speeds(left, right).await;
}

/// Process a motor command.
#[allow(clippy::too_many_lines)]
async fn process_command(
    pwm: &mut PwmChannels,
    left_pins: &mut MotorPins<'_>,
    right_pins: &mut MotorPins<'_>,
    standby: &mut Output<'_>,
    cal: &mut MotorCalibration,
    voltage_comp: f32,
    command: MotorCommand,
) {
    match command {
        MotorCommand::SetTracks {
            left_speed,
            right_speed,
        } => {
            process_set_tracks(pwm, left_pins, right_pins, cal, voltage_comp, left_speed, right_speed).await;
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
        MotorCommand::LoadCalibration(new_cal) => {
            info!(
                "Loading calibration: left={}, right={}",
                new_cal.left_factor, new_cal.right_factor
            );
            *cal = new_cal;
        }
        MotorCommand::UpdateAllCalibration {
            left_factor,
            right_factor,
        } => {
            info!("Updating all calibration: left={}, right={}", left_factor, right_factor);
            *cal = MotorCalibration::new(left_factor, right_factor);
        }
    }
}

// ── Main task ────────────────────────────────────────────────────────────────────

/// Motor driver task — processes commands and controls PWM + GPIO direction pins.
///
/// # Arguments
/// * `pwm_left` — PWM slice for left track motor (GPIO 0, PWM0 CH A)
/// * `pwm_right` — PWM slice for right track motor (GPIO 3, PWM1 CH A)
/// * `left_fwd` — GPIO for left motor forward direction (GPIO 1)
/// * `left_bwd` — GPIO for left motor backward direction (GPIO 2)
/// * `right_fwd` — GPIO for right motor forward direction (GPIO 4)
/// * `right_bwd` — GPIO for right motor backward direction (GPIO 5)
/// * `standby_pin` — GPIO for TB6612FNG standby pin (GPIO 6, active high)
#[allow(clippy::similar_names)]
#[embassy_executor::task]
pub async fn motor_driver(
    pwm_left: Pwm<'static>,
    pwm_right: Pwm<'static>,
    left_fwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_1>,
    left_bwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_2>,
    right_fwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_4>,
    right_bwd: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_5>,
    standby_pin: embassy_rp::Peri<'static, embassy_rp::peripherals::PIN_6>,
) {
    info!("Motor driver task starting");

    // Split PWM slices into individual channel outputs
    let (left_ch, _) = pwm_left.split();
    let (right_ch, _) = pwm_right.split();

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

    // Initialize calibration with defaults — will be updated when flash sends data
    let mut calibration = MotorCalibration::default();
    info!("Motor driver initialized with default calibration");

    // Ensure motion state is zeroed
    motion::set_track_speeds(0, 0).await;

    // Wait for first battery voltage reading before allowing motor commands
    info!("Waiting for battery voltage reading...");
    let initial_voltage = wait_for_battery_voltage().await;
    let mut voltage_compensation = calculate_voltage_compensation(initial_voltage);
    info!(
        "Battery: {}V, voltage compensation: {}",
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
            } => *left_speed != 0 || *right_speed != 0,
            MotorCommand::BrakeAll | MotorCommand::CoastAll => false,
            MotorCommand::SetAllDriversEnable { enabled } => {
                if *enabled {
                    was_motors_active
                } else {
                    false
                }
            }
            MotorCommand::LoadCalibration(_) | MotorCommand::UpdateAllCalibration { .. } => was_motors_active,
        };

        // Only update voltage compensation when motors are idle before and after
        // Voltage sag during motor operation gives false readings that cause calibration instability
        if !was_motors_active
            && !next_motors_active
            && let Some(current_voltage) = try_get_battery_voltage()
        {
            let new_comp = calculate_voltage_compensation(current_voltage);
            if (new_comp - voltage_compensation).abs() > 0.01 {
                info!(
                    "Voltage compensation updated: {} -> {} (idle, {}V)",
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
            &mut calibration,
            voltage_compensation,
            command,
        )
        .await;
    }
}
