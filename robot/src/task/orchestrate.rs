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
use touch_ui::Procedure;

use crate::{
    system::{
        event::{Events, wait},
        state::activity::Stage,
    },
    task::{
        behavior, initialization, remote_link,
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
    // Forward the curated subset to the Link Task before local handling. The
    // forwarding never blocks and the local handlers still consume the Event.
    remote_link::forward_event(&event);

    match event {
        Events::Initialize => initialization::handle_initialize().await,
        Events::CalibrationDataLoaded(kind, data) => {
            initialization::handle_calibration_data_loaded(kind, data).await;
        }
        Events::ProcedureFinished {
            procedure,
            stage,
            detail,
            generation,
        } => handle_procedure_finished(procedure, stage, detail, generation).await,
        Events::BatteryMeasured { level, voltage } => {
            behavior::battery::handle_battery_measured(level, voltage).await;
        }
        Events::ObstacleDetected { source, detected } => {
            behavior::obstacle::handle_obstacle_detected(source, detected);
        }
        Events::ObstacleAvoidanceAttempted => {
            behavior::obstacle::handle_obstacle_avoidance_attempted();
        }
        Events::FloorDropDetected { detected } => {
            behavior::floor_drop::handle_floor_drop_detected(detected);
        }
    }
}

/// Map a terminal Procedure outcome onto the panel's existing reaction.
///
/// The event carries the identity, so this needs no Activity read. The
/// run-to-completion tests leave the running screen only on success, the motor,
/// magnetometer and DMP compass calibrations react on either terminal stage so a
/// finished calibration never looks hung, and every other Procedure — distance
/// calibration, the interactive tests and coast-and-avoid — has no UI event.
/// The same decision is the one place that names which Procedures have a panel
/// lifecycle; the claim raises the event once. The detail is logged so the
/// terminal line the claim recorded is visible alongside the routing decision.
async fn handle_procedure_finished(procedure: Procedure, stage: Stage, detail: &'static str, generation: u32) {
    info!(
        "Procedure finished: {=str} stage={} detail={=str} generation={=u32}",
        procedure.label(),
        stage,
        detail,
        generation
    );

    match procedure {
        Procedure::Turns | Procedure::StraightDrive | Procedure::ArcDrive if stage == Stage::Complete => {
            ui::send_ui_event(UiEvent::TestingCompleted { procedure, generation }).await;
        }
        Procedure::MotorCalibration | Procedure::DmpCompassCalibration => {
            ui::send_ui_event(UiEvent::CalibrationCompleted { generation }).await;
        }
        Procedure::DistanceCalibration
        | Procedure::CoastAndAvoid
        | Procedure::AttemptStraight
        | Procedure::BasicMotor
        | Procedure::Imu6Axis
        | Procedure::Imu9Axis
        | Procedure::Turns
        | Procedure::StraightDrive
        | Procedure::ArcDrive => {}
    }
}
