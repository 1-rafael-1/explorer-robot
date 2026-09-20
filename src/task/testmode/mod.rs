//! Testing task modules.
//!
//! Provides on-demand test mode tasks spawned via a controller task.
//!
//! Tests cover motor, encoder, drive, and IMU subsystems. Each test publishes
//! what it is doing — phase, percent, and its end state — to
//! [`crate::system::state::activity`] instead of formatting display text; the
//! touch UI renders that and offers the Stop action.
//!
//! One test runs at a time. [`start`] is refused while another test's task is
//! live; the panel is modal, so the guard is a formality rather than a contention
//! point. The test family's stop latch and the lifecycle every test runs under
//! live in [`crate::task::procedure`].

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use touch_ui::Procedure;

use crate::{
    system::event::Events,
    task::{
        drive::{
            CompletionStatus, DriveQueueBuilder, DriveQueueCompletion, DriveQueueSubmitError,
            types::DriveQueueBuildError,
        },
        procedure::{Completion, Lifecycle, Slot, StopLatch},
    },
};

pub mod arc_drive;
pub mod basic_motor;
pub mod imu_6axis;
pub mod imu_9axis;
pub mod straight_drive;
pub mod turns;

/// Command sent to the testmode controller.
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

/// Tracks whether any testmode is currently active.
static TESTMODE_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Command channel for testmode spawn requests.
static TESTMODE_COMMAND: Channel<CriticalSectionRawMutex, TestCommand, 4> = Channel::new();

/// The test family's stop latch. A stop aimed at a test cannot reach a
/// calibration, which keeps its own.
static TEST_STOP: StopLatch = StopLatch::new();

/// Initialize testmode support (spawns the controller task).
#[allow(clippy::unwrap_used)]
pub fn init_testing(spawner: Spawner) {
    spawner.spawn(testmode_controller(spawner).unwrap());
}

/// Start the test-mode `procedure`, reporting whether a test was started.
///
/// `false` for a Procedure that is not a test-mode entry, and `false` while
/// another test is live: the panel's modal menus should make contention
/// impossible, but a refusal is logged and nothing runs, so the caller can leave
/// the panel where it is instead of opening a running screen over nothing. `true`
/// once the spawned test has been queued.
#[must_use]
pub async fn start(procedure: Procedure) -> bool {
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
        | Procedure::AttemptStraight => return false,
    };

    if test_lifecycle(procedure).start("Starting").await {
        request_start(command).await;
        true
    } else {
        defmt::warn!("testmode: {} refused — another test is active", procedure.label());
        false
    }
}

/// The lifecycle the test family runs `procedure` under.
///
/// Every test shares the family's stop latch and single-active slot. The
/// run-to-completion tests raise `TestingCompleted` on success; the interactive
/// tests raise nothing, because the operator stops them rather than them
/// finishing.
const fn test_lifecycle(procedure: Procedure) -> Lifecycle {
    Lifecycle::new(
        procedure,
        &TEST_STOP,
        Some(Slot::guarded(claim_testmode, release_testmode)),
        match procedure {
            Procedure::Turns | Procedure::StraightDrive | Procedure::ArcDrive => {
                Completion::OnSuccess(testing_completed)
            }
            _ => Completion::Silent,
        },
    )
}

/// The completion event the run-to-completion tests raise.
const fn testing_completed() -> Events {
    Events::TestingCompleted
}

/// Request that the running test stop.
///
/// Latches the test family's stop, which every test polls, and interrupts any
/// drive in flight — the turns, straight-drive and arc-drive tests are mid-queue
/// when the operator taps Stop. A test that is not driving ignores the interrupt;
/// the drive loop drains it and coasts.
pub fn stop() {
    TEST_STOP.request();
}

/// Queue a spawn request for a test whose slot was claimed.
///
/// The claim itself is the test family's [`Slot`], and the lifecycle makes it
/// before publishing the starting activity; this only hands the controller the
/// command to spawn.
async fn request_start(command: TestCommand) {
    TESTMODE_COMMAND.send(command).await;
}

/// Claim the test family's single-active slot, reporting whether it was free.
fn claim_testmode() -> bool {
    TESTMODE_ACTIVE
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
        .is_ok()
}

/// Release the test family's single-active slot.
fn release_testmode() {
    TESTMODE_ACTIVE.store(false, Ordering::Release);
}

/// Claim the single-active test-mode slot for Room Scan.
///
/// Room Scan is not a test-mode task, but it holds the same slot so a test
/// cannot start while its radar screen is open and its radar screen cannot open
/// while a test runs. Reports whether the slot was free.
#[must_use]
pub fn claim_room_scan() -> bool {
    claim_testmode()
}

/// Release the slot [`claim_room_scan`] took.
pub fn release_room_scan() {
    release_testmode();
}

/// Submit a built queue, mapping a build or submit failure to a short reason.
///
/// The reason is meant for the running screen, so it is short and static; the
/// caller records it through [`crate::task::procedure::Lifecycle::fail`].
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
        match TESTMODE_COMMAND.receive().await {
            TestCommand::Turns => turns::spawn(spawner),
            TestCommand::StraightDrive => straight_drive::spawn(spawner),
            TestCommand::ArcDrive => arc_drive::spawn(spawner),
            TestCommand::Imu => imu_9axis::spawn(spawner),
            TestCommand::Imu6 => imu_6axis::spawn(spawner),
            TestCommand::BasicMotor => basic_motor::spawn(spawner),
        }
    }
}
