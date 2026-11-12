//! Calibration procedures for the drive system.
//!
//! Holds the motor (track balance), magnetometer, DMP compass and distance
//! calibration routines. Each procedure publishes its phase, percent and result
//! to [`crate::system::state::activity`]; nothing here formats display text.
//!
//! # Ownership
//!
//! Each calibration runs on a task of its own, spawned by
//! [`calibration_controller`]. The caller takes the one Active Claim (ADR-0034)
//! through [`crate::task::procedure::dispatch`] before calling [`start`], so the claim
//! travels with the [`CalibrationCommand`] to the controller and the task reports
//! its phases and terminal outcome through it. The claim is already held and
//! published, so a calibration body's first call is [`Claim::phase`], never a
//! second claim or start.
//!
//! The distance calibration holds its claim across the drive step and the value
//! entry, because the factor is forced to 1.0 for the drive: releasing the slot
//! before the operator's commit or cancel would let another Procedure run on that
//! forced factor. A stop of the drive restores the factor and finishes the claim,
//! and the value entry ends it on the commit or cancel the panel offers there.
//!
//! # Stopping
//!
//! A calibration polls its `Claim` at its wait points; the panel's Stop latches the
//! one Active Claim through [`crate::task::procedure::stop`] and the running task
//! cleans up and records its terminal outcome. The distance calibration also drives
//! through the drive subsystem, whose interrupt the same stop sends, so its
//! fixed-distance leg stops mid-motion rather than at a step boundary.

pub mod distance;
pub mod dmp;
pub mod imu;
pub mod motor;

use defmt::{error, warn};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use touch_ui::Procedure;

use self::distance::{DistanceDecision, DriveOutcome};
use crate::task::{
    drive::types::ImuCalibrationKind,
    procedure::Claim,
    ui::{UiEvent, send_ui_event},
};

/// A calibration the controller can spawn.
#[derive(Clone, Copy)]
pub enum CalibrationCommand {
    /// The motor (track balance) calibration.
    Motor,
    /// The magnetometer calibration.
    Mag,
    /// The DMP compass calibration, which guides the operator with the Prompt.
    DmpMag,
    /// The distance calibration, which holds its claim through the value entry.
    Distance,
}

/// Spawn requests for the controller, each carrying the claim it runs under.
static CALIBRATION_COMMAND: Channel<CriticalSectionRawMutex, (CalibrationCommand, Claim), 4> = Channel::new();

/// Initialize calibration support (spawns the controller task).
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(calibration_controller(spawner).unwrap());
}

/// Start the calibration `procedure`, routing the caller's `claim` to its task.
///
/// The dispatch seam took the claim with [`crate::task::procedure::start`], which
/// already claimed the one slot and published the starting Activity. This maps the
/// Procedure to its [`CalibrationCommand`] and hands `(command, claim)` to the
/// controller. A Procedure that is not a calibration starts nothing and abandons
/// the claim defensively — the panel never routes one here.
pub async fn start(claim: Claim, procedure: Procedure) {
    let command = match procedure {
        Procedure::MotorCalibration => CalibrationCommand::Motor,
        Procedure::MagCalibration => CalibrationCommand::Mag,
        Procedure::DmpCompassCalibration => CalibrationCommand::DmpMag,
        Procedure::DistanceCalibration => CalibrationCommand::Distance,
        _ => {
            warn!("calibration: {} is not a calibration", procedure.label());
            claim.abandon().await;
            return;
        }
    };

    CALIBRATION_COMMAND.send((command, claim)).await;
}

/// Controller task that spawns a calibration task per request.
///
/// A spawn that fails abandons the claim so a dead task does not leave the slot
/// held or the starting Activity on screen; the task itself releases only through
/// its terminal call.
#[embassy_executor::task]
async fn calibration_controller(spawner: Spawner) {
    loop {
        let (command, claim) = CALIBRATION_COMMAND.receive().await;
        let task_started = match command {
            CalibrationCommand::Motor => motor_calibration_task(claim).map_or_else(
                |_| {
                    error!("calibration: motor spawn failed");
                    false
                },
                |token| {
                    spawner.spawn(token);
                    true
                },
            ),
            CalibrationCommand::Mag => mag_calibration_task(claim).map_or_else(
                |_| {
                    error!("calibration: mag spawn failed");
                    false
                },
                |token| {
                    spawner.spawn(token);
                    true
                },
            ),
            CalibrationCommand::DmpMag => dmp_calibration_task(claim).map_or_else(
                |_| {
                    error!("calibration: dmp compass spawn failed");
                    false
                },
                |token| {
                    spawner.spawn(token);
                    true
                },
            ),
            CalibrationCommand::Distance => distance_calibration_task(claim).map_or_else(
                |_| {
                    error!("calibration: distance spawn failed");
                    false
                },
                |token| {
                    spawner.spawn(token);
                    true
                },
            ),
        };

        if !task_started {
            // A task slot was unavailable: no body will run, so clear the starting
            // Activity and release the claim rather than leave it held.
            claim.abandon().await;
        }
    }
}

/// Motor calibration task.
///
/// The body makes the claim's terminal call itself, so the task holds nothing
/// back after it returns.
#[embassy_executor::task]
async fn motor_calibration_task(claim: Claim) {
    motor::run_motor_calibration(&claim).await;
}

/// Magnetometer calibration task.
///
/// The body makes the claim's terminal call itself, so the task holds nothing
/// back after it returns.
#[embassy_executor::task]
async fn mag_calibration_task(claim: Claim) {
    imu::run_imu_calibration(ImuCalibrationKind::Mag, &claim).await;
}

/// DMP compass calibration task.
///
/// The body makes the claim's terminal call itself, so the task holds nothing
/// back after it returns.
#[embassy_executor::task]
async fn dmp_calibration_task(claim: Claim) {
    dmp::run_dmp_compass_calibration(&claim).await;
}

/// Distance calibration task: the fixed-distance drive, then the value entry.
///
/// The task owns the claim across both, so the slot stays held while the operator
/// measures and no other Procedure can start with the forced factor of 1.0. The
/// drive step makes its own terminal call when it stops or fails; on success the
/// task publishes the value screen's phase, tells the UI to show it, and makes the
/// terminal call once the operator's [`DistanceDecision`] arrives.
#[embassy_executor::task]
async fn distance_calibration_task(claim: Claim) {
    distance::begin().await;

    match distance::run_drive_step(&claim).await {
        DriveOutcome::Complete => {}
        // The step already restored the factor and finished the claim, or failed
        // it with the reason on the running screen: a second terminal call here
        // would be a stale no-op, so the task simply ends.
        DriveOutcome::Stopped | DriveOutcome::Failed => return,
    }

    claim.phase("Enter measured distance", None).await;
    send_ui_event(UiEvent::DistanceDriveFinished).await;

    match distance::wait_for_decision_or_stop(&claim).await {
        Some(DistanceDecision::Commit(measured_cm)) => {
            distance::commit(measured_cm).await;
        }
        // A cancel or a stop both restore the factor the run overrode.
        Some(DistanceDecision::Cancel) | None => distance::abort().await,
    }
    claim.finish().await;
}
