//! UI rendering helpers.
//!
//! Renders UI screens based on the current UI state and system data.
//!
//! Includes `LiDAR` and rangefinder placeholder test renderers, plus
//! motor, encoder, and IMU telemetry screens.

use heapless::String;

use super::{
    screens,
    state::{UiMode, UiState},
};
use crate::{
    system::state::{calibration, power},
    task::io::display::{self, DisplayAction},
};

/// Render the current UI view based on the UI state.
pub async fn render_current_ui(state: &UiState) {
    match state.mode {
        UiMode::MainMenu => screens::render_main_menu(state.main_index).await,
        UiMode::CalibrateMenu => screens::render_calibrate_menu(state.calibrate_index).await,
        UiMode::DriveModeMenu => screens::render_drive_mode_menu(state.drive_mode_index).await,
        UiMode::TestMenu => screens::render_test_menu(state.test_index).await,
        UiMode::SystemInfo { scroll_offset } => {
            let info = build_system_info_data().await;
            screens::render_system_info(scroll_offset as usize, &info).await;
        }
        UiMode::RunningTurnsTest => {
            screens::render_turns_test().await;
        }
        UiMode::RunningStraightDriveTest => {
            screens::render_straight_drive_test().await;
        }
        UiMode::RunningArcDriveTest => {
            screens::render_arc_drive_test().await;
        }
        UiMode::RunningImu6Test => {
            screens::render_imu6_test().await;
        }
        UiMode::RunningImu9Test => {
            screens::render_imu9_test().await;
        }
        UiMode::RunningBasicMotorTest => {
            screens::render_basic_motor_test().await;
        }
        UiMode::RunningLidarTest => {
            screens::render_lidar_test().await;
        }
        UiMode::RunningRangefinderTest => {
            screens::render_rangefinder_test().await;
        }
        UiMode::RunningAutonomous { mode } => {
            let label = drive_mode_label(mode);
            screens::render_autonomous_running(label).await;
        }
        UiMode::Calibrating { kind } => {
            let label = super::menu::calibration_label(kind);
            screens::render_calibrating(label).await;
        }
        UiMode::EnteringDistance { .. } | UiMode::EnteringAttemptStraightDistance { .. } => {
            // Rendering handled by the rotary-turn handlers — this arm exists
            // for exhaustiveness but should not be reached via render_current_ui.
        }
    }
}

/// Human-readable label for a drive mode.
const fn drive_mode_label(mode: crate::system::state::DriveMode) -> &'static str {
    match mode {
        crate::system::state::DriveMode::CoastAndAvoid => "Coast & Avoid",
        crate::system::state::DriveMode::AttemptStraightLine => "Attempt Straight",
    }
}

/// Write a single line of text to the display.
pub async fn show_line(line: u8, msg: &str) {
    let mut s: String<20> = String::new();
    for ch in msg.chars() {
        if s.push(ch).is_err() {
            break;
        }
    }
    display::display_update(DisplayAction::ShowText(s, line)).await;
}

/// Build a snapshot of system info for the UI renderer.
pub async fn build_system_info_data() -> screens::SystemInfoData {
    // Read Power first (via accessors) to preserve lock order: Power → Calibration.
    let battery = power::get_battery_snapshot().await;
    let calibration_state = calibration::CALIBRATION_STATE.lock().await;
    screens::SystemInfoData {
        battery_level: battery.level,
        battery_voltage: battery.voltage,
        motor_calibration_status: calibration_state.motor_cal_status,
        mag_calibration_status: calibration_state.mag_cal_status,
        distance_calibration_status: calibration_state.distance_cal_status,
    }
}
