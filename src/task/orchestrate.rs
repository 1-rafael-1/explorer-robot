//! System orchestration
//!
//! Manages robot behavior by coordinating state changes and event handling.
//!
//! # Architecture
//! This module implements a central event loop that:
//! - Waits for system events (button presses, sensor readings, etc.)
//! - Routes events to domain-specific handlers
//!
//! Each event is handled by a dedicated module.

use defmt::info;

use crate::{
    system::event::{Events, wait},
    task::{
        behavior, initialization,
        ui::{self, UiEvent},
    },
};

/// Main coordination task that implements the system's event loop.
#[embassy_executor::task]
pub async fn orchestrate() {
    info!("Orchestrator starting");

    loop {
        let event = wait().await;
        handle_event(event).await;
    }
}

/// Routes events to their respective handlers.
#[allow(clippy::too_many_lines)]
async fn handle_event(event: Events) {
    match event {
        Events::Initialize => initialization::handle_initialize().await,
        Events::CalibrationDataLoaded(kind, data) => {
            initialization::handle_calibration_data_loaded(kind, data).await;
        }
        Events::CalibrationStatus {
            header,
            line1,
            line2,
            line3,
        } => initialization::handle_calibration_status(header, line1, line2, line3).await,
        Events::CalibrationCompleted => {
            ui::send_ui_event(UiEvent::CalibrationCompleted).await;
        }
        Events::BatteryMeasured { level, voltage } => {
            behavior::battery::handle_battery_measured(level, voltage).await;
        }
        Events::ObstacleDetected { source, detected } => {
            behavior::obstacle::handle_obstacle_detected(source, detected).await;
        }
        Events::ObstacleAvoidanceAttempted => {
            behavior::obstacle::handle_obstacle_avoidance_attempted();
        }
        Events::RotaryTurned(direction) => {
            ui::send_ui_event(UiEvent::RotaryTurned(direction)).await;
        }
        Events::RotaryButtonPressed => {
            ui::send_ui_event(UiEvent::RotaryButtonPressed).await;
        }
        Events::RotaryButtonHoldStart => {
            ui::send_ui_event(UiEvent::RotaryButtonHoldStart).await;
        }
        Events::RotaryButtonHoldEnd => {
            ui::send_ui_event(UiEvent::RotaryButtonHoldEnd).await;
        }
        Events::TestingCompleted => {
            ui::send_ui_event(UiEvent::TestingCompleted).await;
        }
        Events::LidarScanCompleted => {
            // Placeholder: log LiDAR scan completion for now.
            info!("Lidar scan completed");
        }
        Events::RangefinderReading => {
            // Rangefinder readings are polled directly from perception state
            // by consumers — this event serves as a wake-up signal.
        }
    }
}
