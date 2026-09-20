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
//! point, and the stop path is a single shared latch any running test polls.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};

use crate::{
    system::state::activity::{self, StopRequest, TestKind},
    task::drive::{
        self, CompletionStatus, DriveQueueBuilder, DriveQueueCompletion, DriveQueueSubmitError, InterruptKind,
        types::DriveQueueBuildError,
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

/// The stop request every running test polls between steps.
static STOP: StopRequest = StopRequest::new();

/// Initialize testmode support (spawns the controller task).
#[allow(clippy::unwrap_used)]
pub fn init_testing(spawner: Spawner) {
    spawner.spawn(testmode_controller(spawner).unwrap());
}

/// Start the test-mode procedure `kind`.
///
/// Refused while another test is live, which the panel's modal menus should make
/// impossible; a refusal is logged and nothing runs.
pub async fn start(kind: TestKind) {
    let command = match kind {
        TestKind::BasicMotor => TestCommand::BasicMotor,
        TestKind::Turns => TestCommand::Turns,
        TestKind::StraightDrive => TestCommand::StraightDrive,
        TestKind::ArcDrive => TestCommand::ArcDrive,
        TestKind::Imu6Axis => TestCommand::Imu6,
        TestKind::Imu9Axis => TestCommand::Imu,
    };

    if request_start(command).await {
        activity::begin(activity::Activity::Test(kind), "Starting", kind.is_interactive()).await;
    } else {
        defmt::warn!("testmode: {:?} refused — another test is active", kind);
    }
}

/// Request that the running test stop.
///
/// Latches the stop every test polls, and interrupts any drive in flight — the
/// turns, straight-drive and arc-drive tests are mid-queue when the operator taps
/// Stop. A test that is not driving ignores the interrupt; the drive loop drains
/// it and coasts.
pub fn stop() {
    STOP.request();
    drive::send_drive_interrupt(InterruptKind::Stop);
}

/// Request that a test be spawned on demand.
/// Returns true if the request was accepted.
pub(super) async fn request_start(command: TestCommand) -> bool {
    if TESTMODE_ACTIVE
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
        .is_err()
    {
        return false;
    }

    TESTMODE_COMMAND.send(command).await;
    true
}

/// Mark the testmode controller as idle again.
pub(super) fn release_testmode() {
    TESTMODE_ACTIVE.store(false, Ordering::Release);
}

/// Claim the single-active test-mode slot for Room Scan.
///
/// Room Scan is not a test-mode task, but it holds the same slot so a test
/// cannot start while its radar screen is open and its radar screen cannot open
/// while a test runs. Reports whether the slot was free.
#[must_use]
pub fn claim_room_scan() -> bool {
    TESTMODE_ACTIVE
        .compare_exchange(false, true, Ordering::AcqRel, Ordering::Relaxed)
        .is_ok()
}

/// Release the slot [`claim_room_scan`] took.
pub fn release_room_scan() {
    release_testmode();
}

/// Re-arm the shared stop latch at the start of a run.
pub(super) async fn arm_stop() {
    STOP.rearm().await;
}

/// Whether the operator has asked the running test to stop.
pub(super) fn is_stop_requested() -> bool {
    STOP.is_requested()
}

/// Wait for `duration_ms`, returning `true` early if a stop is requested.
pub(super) async fn wait_or_stop(duration_ms: u64) -> bool {
    STOP.wait_or(duration_ms).await
}

/// Submit a built queue, mapping a build or submit failure to a short reason.
///
/// The reason is meant for the running screen, so it is short and static; the
/// caller records it through [`crate::system::state::activity::fail`].
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
