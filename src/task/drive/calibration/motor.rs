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

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::{
    system::helper::string_helper::status_text,
    task::{
        drive::sensors::data::{clear_encoder_measurement, wait_for_encoder_event_timeout},
        motor_driver::{self, MotorCalibration, MotorCommand},
    },
};

/// Coast duration between calibration runs (milliseconds).
const CALIBRATION_COAST_DURATION_MS: u64 = 500;
/// Encoder sample duration for calibration (milliseconds).
const CALIBRATION_SAMPLE_DURATION_MS: u64 = 1000;
/// Motor speed for calibration (0-100).
const CALIBRATION_SPEED: i8 = 60;

/// Run the 2-motor calibration procedure.
///
/// Coordinates encoder measurements with motor commands to calculate
/// per-track calibration factors. The faster track is the reference;
/// the slower track is attenuated to match.
#[allow(clippy::too_many_lines)]
pub async fn run_motor_calibration() {
    use heapless::String;

    use crate::{
        system::event,
        task::{io::flash_storage, sensors::encoders as encoder_read},
    };

    info!("=== Starting Motor Calibration (2-motor) ===");

    // Display calibration header
    event::raise_event(event::Events::CalibrationStatus {
        header: status_text("Motor Calibration"),
        line1: status_text("Enabling drivers"),
        line2: None,
        line3: None,
    })
    .await;

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

    // ── Step 1: Test left track ─────────────────────────────────────────────
    info!("Step 1: Testing left track");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 1/4"),
        line2: status_text("Test left track"),
        line3: None,
    })
    .await;

    // Stop, reset, clear, then restart for clean measurement.
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await;
    clear_encoder_measurement().await;
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await;

    // Run left track only at calibration speed.
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: CALIBRATION_SPEED,
        right_speed: 0,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let left_pulses = wait_for_encoder_event_timeout(500).await.map_or_else(
        || {
            info!("    Warning: No encoder event received for left track");
            0u16
        },
        |measurement| {
            info!("    ENCODER READINGS: {:?}", measurement);
            info!("    -> left encoder: {}", measurement.left);
            let pulses = measurement.left;
            info!("    ✓ Left track encoder count: {}", pulses);
            pulses
        },
    );

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // ── Step 2: Test right track ────────────────────────────────────────────
    info!("Step 2: Testing right track");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 2/4"),
        line2: status_text("Test right track"),
        line3: None,
    })
    .await;

    // Stop, reset, clear, then restart for clean measurement.
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(200)).await;
    encoder_read::send_command(encoder_read::EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await;
    clear_encoder_measurement().await;
    encoder_read::send_command(encoder_read::EncoderCommand::Start { interval_ms: 20 }).await;
    Timer::after(Duration::from_millis(200)).await;

    // Run right track only at calibration speed.
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: 0,
        right_speed: CALIBRATION_SPEED,
    })
    .await;

    Timer::after(Duration::from_millis(CALIBRATION_SAMPLE_DURATION_MS)).await;

    let right_pulses = wait_for_encoder_event_timeout(500).await.map_or_else(
        || {
            info!("    Warning: No encoder event received for right track");
            0u16
        },
        |measurement| {
            info!("    ENCODER READINGS: {:?}", measurement);
            info!("    -> right encoder: {}", measurement.right);
            let pulses = measurement.right;
            info!("    ✓ Right track encoder count: {}", pulses);
            pulses
        },
    );

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(CALIBRATION_COAST_DURATION_MS)).await;

    // ── Step 3: Compute calibration factors ──────────────────────────────────
    info!("Step 3: Computing calibration factors");
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 3/4"),
        line2: status_text("Compute factors"),
        line3: None,
    })
    .await;

    if left_pulses == 0 && right_pulses == 0 {
        info!("  ERROR: Both tracks show zero counts — calibration cannot proceed");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("CALIB FAILED"),
            line2: status_text("Zero encoder"),
            line3: status_text("Check wiring"),
        })
        .await;
        Timer::after(Duration::from_millis(3000)).await;
        // Stop and disable.
        encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;
        motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
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
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Step 4/4"),
        line2: status_text("Save to flash"),
        line3: None,
    })
    .await;

    // Validate factors.
    let all_valid = left_factor > 0.0 && left_factor <= 1.0 && right_factor > 0.0 && right_factor <= 1.0;

    if all_valid {
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("Saving..."),
            line2: status_text("To flash storage"),
            line3: status_text("Please wait..."),
        })
        .await;

        flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
            flash_storage::CalibrationDataKind::Motor(calibration),
        ))
        .await;

        info!("✓ Calibration saved successfully");
    } else {
        info!("✗ ERROR: Calibration factors invalid - NOT saving to flash");
        info!("  Check encoder wiring and ensure motors are running during calibration");
        event::raise_event(event::Events::CalibrationStatus {
            header: None,
            line1: status_text("CALIB FAILED"),
            line2: status_text("Invalid factors"),
            line3: status_text("Check encoders"),
        })
        .await;
        Timer::after(Duration::from_millis(3000)).await;
    }

    // Stop encoder readings.
    encoder_read::send_command(encoder_read::EncoderCommand::Stop).await;

    // Disable motor drivers (return to standby).
    info!("Disabling motor driver");
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;

    info!("=== Calibration Complete ===");

    // Show final results.
    let mut line2 = String::<20>::new();
    let _ = core::fmt::write(&mut line2, format_args!("Left: {:.2}", calibration.left_factor));
    let mut line3 = String::<20>::new();
    let _ = core::fmt::write(&mut line3, format_args!("Right: {:.2}", calibration.right_factor));
    event::raise_event(event::Events::CalibrationStatus {
        header: None,
        line1: status_text("Complete!"),
        line2: Some(line2),
        line3: Some(line3),
    })
    .await;

    event::raise_event(event::Events::CalibrationCompleted).await;
}
