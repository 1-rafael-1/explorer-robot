//! Testing task modules.
//!
//! Provides on-demand test mode tasks spawned via a controller task.
//!
//! Tests cover motor, encoder, drive, and IMU subsystems. Each test publishes
//! what it is doing — phase, percent, and its end state — to
//! [`crate::system::state::activity`] instead of formatting display text; the
//! touch UI renders that and offers the Stop action.
//!
//! One test runs at a time because the caller takes the one Active Claim
//! (ADR-0034) before calling [`start`]. The claim travels with the
//! [`TestCommand`] to the controller, which spawns the test task with it; the
//! task reports its phases and terminal outcome through that claim, and the test
//! family keeps no latch or slot of its own.

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use touch_ui::Procedure;

use crate::task::{
    drive::{
        CompletionStatus, DriveQueueBuilder, DriveQueueCompletion, DriveQueueSubmitError, types::DriveQueueBuildError,
    },
    procedure::Claim,
};

pub mod arc_drive;
pub mod basic_motor;
pub mod imu_6axis;
pub mod imu_9axis;
pub mod straight_drive;
pub mod turns;

/// Command sent to the testmode controller, paired with the claim it runs under.
#[derive(Clone, Copy)]
pub(super) enum TestCommand {
    /// Spawn the turns test.
    Turns,
    /// Spawn the straight drive test.
    StraightDrive,
    /// Spawn the arc drive test.
    ArcDrive,
    /// Spawn the IMU 9-axis telemetry test.
    Imu,
    /// Spawn the IMU 6-axis telemetry test.
    Imu6,
    /// Spawn the basic motor test.
    BasicMotor,
}

/// Command channel for testmode spawn requests, each carrying its claim.
static TESTMODE_COMMAND: Channel<CriticalSectionRawMutex, (TestCommand, Claim), 4> = Channel::new();

/// Initialize testmode support (spawns the controller task).
#[allow(clippy::unwrap_used)]
pub fn init_testing(spawner: Spawner) {
    spawner.spawn(testmode_controller(spawner).unwrap());
}

/// Start the test-mode `procedure`, routing the caller's `claim` to its task.
///
/// The dispatch seam took the claim with [`crate::task::procedure::start`], which
/// already claimed the one slot and published the starting Activity. This maps the
/// Procedure to its [`TestCommand`] and hands `(command, claim)` to the controller.
/// A Procedure that is not a test entry starts nothing and abandons the claim
/// defensively — the panel never routes one here.
pub async fn start(claim: Claim, procedure: Procedure) {
    let command = match procedure {
        Procedure::BasicMotor => TestCommand::BasicMotor,
        Procedure::Turns => TestCommand::Turns,
        Procedure::StraightDrive => TestCommand::StraightDrive,
        Procedure::ArcDrive => TestCommand::ArcDrive,
        Procedure::Imu6Axis => TestCommand::Imu6,
        Procedure::Imu9Axis => TestCommand::Imu,
        Procedure::MotorCalibration
        | Procedure::MagCalibration
        | Procedure::DistanceCalibration
        | Procedure::CoastAndAvoid
        | Procedure::AttemptStraight => {
            defmt::warn!("testmode: {} is not a test", procedure.label());
            claim.abandon().await;
            return;
        }
    };

    TESTMODE_COMMAND.send((command, claim)).await;
}

/// Submit a built queue, mapping a build or submit failure to a short reason.
///
/// The reason is meant for the running screen, so it is short and static; the
/// caller records it through [`crate::task::procedure::Claim::fail`].
pub(super) async fn submit(
    queue: Result<DriveQueueBuilder, DriveQueueBuildError>,
) -> Result<DriveQueueCompletion, &'static str> {
    let queue = queue.map_err(DriveQueueBuildError::label)?;
    queue.submit().await.map_err(DriveQueueSubmitError::label)
}

/// A short label for a drive completion status.
pub(super) const fn status_label(status: &CompletionStatus) -> &'static str {
    match status {
        CompletionStatus::Success => "Success",
        CompletionStatus::Cancelled => "Cancelled",
        CompletionStatus::Failed(_) => "Failed",
    }
}

/// Controller task that spawns test tasks on demand.
#[embassy_executor::task]
async fn testmode_controller(spawner: Spawner) {
    loop {
        let (command, claim) = TESTMODE_COMMAND.receive().await;
        let task_started = match command {
            TestCommand::Turns => turns::spawn(spawner, claim),
            TestCommand::StraightDrive => straight_drive::spawn(spawner, claim),
            TestCommand::ArcDrive => arc_drive::spawn(spawner, claim),
            TestCommand::Imu => imu_9axis::spawn(spawner, claim),
            TestCommand::Imu6 => imu_6axis::spawn(spawner, claim),
            TestCommand::BasicMotor => basic_motor::spawn(spawner, claim),
        };

        if !task_started {
            // A task slot was unavailable: the test will never run, so clear the
            // starting Activity and release the claim instead of leaving the
            // running screen up with no owner.
            claim.abandon().await;
        }
    }
}
