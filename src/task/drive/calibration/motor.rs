//! Motor calibration procedure (v3 — 2-motor).
//!
//! Runs each track at a fixed PWM, measures encoder pulses, and computes
//! per-track calibration factors that balance the two tracks.
//!
//! # Procedure
//!
//! 1. Enable motor drivers and start encoder sampling.
//! 2. Run left track alone, measure encoder pulses.
//! 3. Run right track alone, measure encoder pulses.
//! 4. Compute factors: `target = max(left_pulses, right_pulses)`,
//!    `left_factor = target / left_pulses`, `right_factor = target / right_pulses`.
//!    The faster track becomes the reference (factor = 1.0); the slower track is attenuated.
//! 5. Apply calibration via `UpdateAllCalibration`.
//! 6. Save to flash storage.
//! 7. Disable motor drivers.
//!
//! Any residual mismatch is corrected at runtime by the IMU-based heading correction.
//!
//! The running screen reads the current step and its percent from the activity
//! state; the measured pulse counts and the computed factors go to the log. A
//! procedure that cannot proceed — both tracks silent — records the reason in the
//! activity state instead of drawing it. The operator's Stop is honoured at every
//! step boundary.

use defmt::info;
use embassy_time::{Duration, Timer};

use super::{arm_stop, is_stop_requested, wait_or_stop};
use crate::{
    system::{
        event::{Events, raise_event},
        state::activity::{self, Activity, CalibrationKind},
    },
    task::{
        drive::sensors::data::{clear_encoder_measurement, wait_for_encoder_event_timeout},
        io::flash_storage,
        motor_driver::{self, MotorCalibration, MotorCommand},
        sensors::encoders as encoder_read,
    },
};

/// Coast duration between calibration runs (milliseconds).
const CALIBRATION_COAST_DURATION_MS: u64 = 500;
/// Encoder sample duration for calibration (milliseconds).
const CALIBRATION_SAMPLE_DURATION_MS: u64 = 1000;
/// Motor speed for calibration (0-100).
const CALIBRATION_SPEED: i8 = 60;
/// Steps the progress percent divides the procedure into.
const STEPS: usize = 4;

/// The track under calibration.
#[derive(Clone, Copy)]
enum Side {
    /// The left track.
    Left,
    /// The right track.
    Right,
}

/// Run the 2-motor calibration procedure.
///
/// Coordinates encoder measurements with motor commands to calculate
/// per-track calibration factors. The faster track is the reference;
/// the slower track is attenuated.
#[allow(clippy::too_many_lines)]
pub async fn run_motor_calibration() {
    info!("=== Starting Motor Calibration (2-motor) ===");

    arm_stop().await;
    activity::begin(Activity::Calibration(CalibrationKind::Motor), "Enabling drivers", false).await;

    // Enable motor drivers (take out of standby).
    info!("Enabling motor driver");
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
    Timer::after(Duration::from_millis(10)).await;

    // Start encoder readings at 50 Hz for calibration.
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(100)).await;

    // Track calibration factors (defaults to 1.0).
    let mut calibration = MotorCalibration::default();
    info!("Starting calibration with default factors: {:?}", calibration);

    let Some(left_pulses) = measure_track(Side::Left, 1).await else {
        finish_stopped().await;
        return;
    };
    let Some(right_pulses) = measure_track(Side::Right, 2).await else {
        finish_stopped().await;
        return;
    };

    // ── Step 3: Compute calibration factors ──────────────────────────────────
    info!("Step 3: Computing calibration factors");
    activity::set_running("Compute factors", Some(activity::percent_done(3, STEPS))).await;

    if left_pulses == 0 && right_pulses == 0 {
        info!("  ERROR: Both tracks show zero counts — calibration cannot proceed");
        fail_and_stop("Zero encoder — check wiring").await;
        return;
    }

    // The faster track becomes the reference (factor = 1.0).
    // The slower track gets attenuated.
    let target_pulses = left_pulses.max(right_pulses);
    let target_f32 = f32::from(target_pulses);

    let left_factor = if left_pulses > 0 {
        (target_f32 / f32::from(left_pulses)).clamp(0.0, 1.0)
    } else {
        1.0
    };
    let right_factor = if right_pulses > 0 {
        (target_f32 / f32::from(right_pulses)).clamp(0.0, 1.0)
    } else {
        1.0
    };

    info!("  Track pulses: Left={}, Right={}", left_pulses, right_pulses);
    info!("  Computed factors: left={}, right={}", left_factor, right_factor);

    calibration.left_factor = left_factor;
    calibration.right_factor = right_factor;

    // Apply calibration to motor driver.
    motor_driver::send_motor_command(MotorCommand::UpdateAllCalibration {
        left_factor,
        right_factor,
    })
    .await;

    // ── Step 4: Save calibration ────────────────────────────────────────────
    info!("Step 4: Saving calibration");
    activity::set_running("Save to flash", Some(activity::percent_done(4, STEPS))).await;

    // Validate factors.
    let all_valid = left_factor > 0.0 && left_factor <= 1.0 && right_factor > 0.0 && right_factor <= 1.0;

    if all_valid {
        flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
            flash_storage::CalibrationDataKind::Motor(calibration),
        ))
        .await;
        info!("✓ Calibration saved successfully");
    } else {
        info!("✗ ERROR: Calibration factors invalid - NOT saving to flash");
        info!("  Check encoder wiring and ensure motors are running during calibration");
        fail_and_stop("Invalid factors — check encoders").await;
        return;
    }

    // Stop encoder readings.
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

    // Disable motor drivers (return to standby).
    info!("Disabling motor driver");
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;

    info!("=== Calibration Complete ===");
    info!(
        "motor calibration factors: left={=f32} right={=f32}",
        calibration.left_factor, calibration.right_factor
    );
    activity::complete("Calibration saved").await;
    raise_event(Events::CalibrationCompleted).await;
}

/// Measure one track alone, returning its pulse count, or `None` if the operator
/// stopped the procedure.
async fn measure_track(side: Side, step: usize) -> Option<u16> {
    let (phase, left_speed, right_speed) = match side {
        Side::Left => ("Test left track", CALIBRATION_SPEED, 0),
        Side::Right => ("Test right track", 0, CALIBRATION_SPEED),
    };

    info!("{}", phase);
    activity::set_running(phase, Some(activity::percent_done(step - 1, STEPS))).await;

    // Stop, reset, clear, then restart for clean measurement.
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await;
    clear_encoder_measurement().await;
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await;

    if is_stop_requested() {
        return None;
    }

    // Run the track under test only, at calibration speed.
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed,
        right_speed,
    })
    .await;

    if wait_or_stop(CALIBRATION_SAMPLE_DURATION_MS).await {
        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        return None;
    }

    let pulses = wait_for_encoder_event_timeout(500).await.map_or_else(
        || {
            info!("    Warning: No encoder event received for {=str}", phase);
            0u16
        },
        |measurement| {
            let pulses = match side {
                Side::Left => measurement.left,
                Side::Right => measurement.right,
            };
            info!("    ✓ {=str} encoder count: {=u16}", phase, pulses);
            pulses
        },
    );

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    if wait_or_stop(CALIBRATION_COAST_DURATION_MS).await {
        return None;
    }

    Some(pulses)
}

/// Stop the procedure cleanly after the operator's Stop: release the motors and
/// the encoder sampler, and forget the activity the UI is already leaving.
async fn finish_stopped() {
    info!("Motor calibration stopped by the operator");
    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    activity::clear().await;
}

/// Release the hardware and record why the procedure could not continue.
async fn fail_and_stop(reason: &'static str) {
    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    activity::fail(reason).await;
    raise_event(Events::CalibrationCompleted).await;
}
