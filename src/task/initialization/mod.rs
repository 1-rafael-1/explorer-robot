//! System initialization and calibration coordination.
//!
//! Orchestrates boot-time setup, calibration loading, and related UI updates.

use core::fmt::Write;

use defmt::info;
use heapless::String;

use crate::{
    system::state::{CalibrationStatus, calibration},
    task::{
        io::{
            display::{self, MAX_LINE_LEN},
            flash_storage::{self, CalibrationDataKind, CalibrationKind},
        },
        motor_driver::{self, MotorCommand},
        ui::{self, UiEvent},
    },
};

/// Handle system initialization.
pub async fn handle_initialize() {
    info!("System initializing");

    // Display initialization message.
    display::display_update(display::DisplayAction::Clear).await;
    let mut txt: String<MAX_LINE_LEN> = String::new();
    let _ = write!(txt, "Initializing...");
    display::display_update(display::DisplayAction::ShowText(txt, 0)).await;

    // Request motor calibration from flash.
    info!("Requesting motor calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Motor,
    ))
    .await;

    // Request IMU calibration from flash.
    info!("Requesting IMU calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::ImuFlags,
    ))
    .await;

    // Request distance calibration from flash.
    info!("Requesting distance calibration from flash");
    flash_storage::send_flash_command(flash_storage::FlashCommand::GetData(
        flash_storage::CalibrationKind::Distance,
    ))
    .await;

    // Note: initialization completes when calibration data arrives via events.
}

/// Handle calibration data loaded from flash storage.
pub async fn handle_calibration_data_loaded(kind: CalibrationKind, data: Option<CalibrationDataKind>) {
    match kind {
        CalibrationKind::Motor => {
            if let Some(CalibrationDataKind::Motor(motor_cal)) = data {
                info!(
                    "Motor calibration loaded: left_factor={} right_factor={}",
                    motor_cal.left_factor, motor_cal.right_factor
                );

                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.motor_cal_status = CalibrationStatus::Loaded;
                }

                motor_driver::send_motor_command(MotorCommand::LoadCalibration(motor_driver::MotorCalibration::new(
                    motor_cal.left_factor,
                    motor_cal.right_factor,
                )))
                .await;

                let mut txt: String<MAX_LINE_LEN> = String::new();
                let _ = write!(txt, "Calibration loaded");
                display::display_update(display::DisplayAction::ShowText(txt, 1)).await;
            } else {
                info!("No motor calibration found - using defaults");

                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.motor_cal_status = CalibrationStatus::NotAvailable;
                }

                let mut txt: String<MAX_LINE_LEN> = String::new();
                let _ = write!(txt, "Need motor calib");
                display::display_update(display::DisplayAction::ShowText(txt, 1)).await;
            }
        }
        CalibrationKind::Distance => {
            if let Some(CalibrationDataKind::Distance(factor)) = data {
                info!("Distance calibration loaded: factor={}", factor);
                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.distance_cal_status = CalibrationStatus::Loaded;
                    state.distance_factor = factor;
                }
            } else {
                info!("No distance calibration found - using default 1.0");
                {
                    let mut state = calibration::CALIBRATION_STATE.lock().await;
                    state.distance_cal_status = CalibrationStatus::NotAvailable;
                }
            }
        }
        CalibrationKind::ImuFlags => {
            let mag = data.as_ref().is_some_and(|d| {
                if let CalibrationDataKind::ImuFlags(flags) = d {
                    flags.mag
                } else {
                    false
                }
            });
            handle_imu_calibration_flags_loaded(mag).await;
        }
    }

    check_initialization_complete().await;
}

/// Handle IMU calibration flags loaded from flash.
async fn handle_imu_calibration_flags_loaded(mag: bool) {
    {
        let mut state = calibration::CALIBRATION_STATE.lock().await;
        state.mag_cal_status = if mag {
            CalibrationStatus::Loaded
        } else {
            CalibrationStatus::NotAvailable
        };
        // imu_status is managed by handle_calibration_data_loaded
        // to avoid overriding the data-load result.
    }

    if ui::ui_initialized().await {
        ui::send_ui_event(UiEvent::ShowMainMenu).await;
    }

    check_initialization_complete().await;
}

/// Check if initialization is complete and update display if so.
async fn check_initialization_complete() {
    let should_show_menu = {
        let state = calibration::CALIBRATION_STATE.lock().await;

        if state.is_initialized() {
            info!("System initialization complete");
            info!("  Motor calibration: {:?}", state.motor_cal_status);
            info!("  IMU calibration: {:?}", state.imu_cal_status);
            info!("  Distance calibration: {:?}", state.distance_cal_status);
            true
        } else {
            false
        }
    };

    if should_show_menu && !ui::ui_is_calibrating().await {
        ui::send_ui_event(UiEvent::ShowMainMenu).await;
    }
}

/// Handle calibration status updates.
pub async fn handle_calibration_status(
    header: Option<heapless::String<MAX_LINE_LEN>>,
    line1: Option<heapless::String<MAX_LINE_LEN>>,
    line2: Option<heapless::String<MAX_LINE_LEN>>,
    line3: Option<heapless::String<MAX_LINE_LEN>>,
) {
    if let Some(text) = header {
        display::display_update(display::DisplayAction::ShowText(text, 0)).await;
    }
    if let Some(text) = line1 {
        display::display_update(display::DisplayAction::ShowText(text, 1)).await;
    }
    if let Some(text) = line2 {
        display::display_update(display::DisplayAction::ShowText(text, 2)).await;
    }
    if let Some(text) = line3 {
        display::display_update(display::DisplayAction::ShowText(text, 3)).await;
    }
}
