//! Calibration procedures for the drive system.
//!
//! Holds the motor (track balance), magnetometer and distance calibration
//! routines. Each procedure publishes its phase, percent and result to
//! [`crate::system::state::activity`]; nothing here formats display text.
//!
//! # Ownership
//!
//! A motor or magnetometer calibration runs on a task of its own, spawned by
//! [`calibration_controller`]. The caller takes the one Active Claim (ADR-0034)
//! with [`crate::task::procedure::start`] before calling [`start`], so the claim
//! travels with the [`CalibrationCommand`] to the controller and the task reports
//! its phases and terminal outcome through it. The claim is already held and
//! published, so a calibration body's first call is [`Claim::phase`], never a
//! second claim or start.
//!
//! Distance calibration still runs on its old path, with [`CALIBRATION_STOP`] and
//! [`calibration_lifecycle`], until ticket 06 moves it onto the same controller.
//!
//! # Stopping
//!
//! A motor or magnetometer calibration polls its `Claim` at its wait points; the
//! panel's Stop latches the one Active Claim through
//! [`crate::task::procedure::stop`] and the running task cleans up and records its
//! terminal outcome. Distance calibration is still stopped through
//! [`request_stop`], which latches the calibration family's stop — which its drive
//! step polls — and interrupts the drive in flight, so its fixed-distance leg
//! stops mid-motion rather than at a step boundary.

pub mod distance;
pub mod imu;
pub mod motor;

use defmt::{error, warn};
use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use touch_ui::Procedure;

use crate::{
    system::event::Events,
    task::{
        drive::types::ImuCalibrationKind,
        procedure::{Claim, Completion, Lifecycle, StopLatch},
    },
};

/// A calibration the controller can spawn.
///
/// Ticket 06 adds `Distance` when that calibration moves onto the controller.
#[derive(Clone, Copy)]
pub enum CalibrationCommand {
    /// The motor (track balance) calibration.
    Motor,
    /// The magnetometer calibration.
    Mag,
}

/// Spawn requests for the controller, each carrying the claim it runs under.
static CALIBRATION_COMMAND: Channel<CriticalSectionRawMutex, (CalibrationCommand, Claim), 4> = Channel::new();

/// The stop request the distance calibration polls.
///
/// Removed with [`calibration_lifecycle`] when distance moves onto the controller
/// in ticket 06.
static CALIBRATION_STOP: StopLatch = StopLatch::new();

/// Request that the running distance calibration stop.
///
/// Latches the calibration family's stop its drive step polls, and interrupts any
/// drive in flight — stopping that leg is not a step boundary. The motor and
/// magnetometer calibrations observe the Active Claim instead.
pub fn request_stop() {
    CALIBRATION_STOP.request();
}

/// The lifecycle the distance calibration runs `procedure` under.
///
/// It has no single-active slot — it still runs on its own old path — and raises
/// no completion event: its flow crosses the value-entry screen, so the Panel, not
/// the procedure, reports the finished calibration. Only distance calls this until
/// ticket 06 removes it with the rest of the per-family lifecycle.
const fn calibration_lifecycle(procedure: Procedure) -> Lifecycle {
    Lifecycle::new(
        procedure,
        &CALIBRATION_STOP,
        None,
        match procedure {
            Procedure::DistanceCalibration => Completion::Silent,
            _ => Completion::OnBoth(calibration_completed),
        },
    )
}

/// The completion event a non-distance calibration lifecycle raises.
///
/// The motor and magnetometer calibrations now raise it through their `Claim`, so
/// this copy only serves [`calibration_lifecycle`]'s non-distance arm until ticket
/// 06 deletes both.
const fn calibration_completed() -> Events {
    Events::CalibrationCompleted
}

/// Initialize calibration support (spawns the controller task).
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(calibration_controller(spawner).unwrap());
}

/// Start the calibration `procedure`, routing the caller's `claim` to its task.
///
/// The caller took the claim with [`crate::task::procedure::start`], which already
/// claimed the one slot and published the starting Activity. This maps the
/// Procedure to its [`CalibrationCommand`] and hands `(command, claim)` to the
/// controller. A Procedure that is not a calibration starts nothing and abandons
/// the claim defensively — the panel never routes one here.
pub async fn start(claim: Claim, procedure: Procedure) {
    let command = match procedure {
        Procedure::MotorCalibration => CalibrationCommand::Motor,
        Procedure::MagCalibration => CalibrationCommand::Mag,
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
/// A spawn that fails releases the claim so a dead task does not leave the slot
/// held; the task itself releases only through its terminal call.
#[embassy_executor::task]
async fn calibration_controller(spawner: Spawner) {
    loop {
        let (command, claim) = CALIBRATION_COMMAND.receive().await;
        match command {
            CalibrationCommand::Motor => {
                if let Ok(token) = motor_calibration_task(claim) {
                    spawner.spawn(token);
                } else {
                    error!("calibration: motor spawn failed");
                    claim.release();
                }
            }
            CalibrationCommand::Mag => {
                if let Ok(token) = mag_calibration_task(claim) {
                    spawner.spawn(token);
                } else {
                    error!("calibration: mag spawn failed");
                    claim.release();
                }
            }
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
