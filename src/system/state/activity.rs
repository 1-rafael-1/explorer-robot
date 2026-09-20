//! Activity state module.
//!
//! Records what long-running procedure the robot is currently performing and a
//! small progress snapshot for the panel: which procedure, how far it has got, a
//! short phase line, an optional percent, and whether it only ends on an explicit
//! stop. The test-mode tasks, the boot/initialisation flow and the calibration
//! flows write it; the touch UI reads it on its tick and is the only thing that
//! draws.
//!
//! # Lifecycle
//!
//! [`begin`] is the only way in and [`clear`] the only way out; the boot flow uses
//! the `_if_idle`/`_for` variants, which act only while the activity is the one it
//! expects. While an activity is recorded, the producer moves it along with
//! [`set_running`], [`complete`] or [`fail`]; those are no-ops once the state is
//! idle, so a producer that outlives the operator's Stop cannot resurrect a
//! finished procedure. A producer records its terminal state and leaves it for the
//! UI to show; the UI clears it when it leaves the running screen.
//!
//! # Lock order
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) power state mutex (use power module accessors)
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (use perception accessors)
//! 4) `MOTION_STATE`
//! 5) `ACTIVITY_STATE` — the innermost lock. Take it last, and never call another
//!    state accessor while holding it, so it can never close a lock cycle.

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Duration, Timer};

/// Which test-mode procedure is running.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum TestKind {
    /// The basic motor test.
    BasicMotor,
    /// The in-place turns test.
    Turns,
    /// The straight-drive test.
    StraightDrive,
    /// The arc-drive test.
    ArcDrive,
    /// The six-axis IMU telemetry test.
    Imu6Axis,
    /// The nine-axis IMU telemetry test.
    Imu9Axis,
}

impl TestKind {
    /// The title the running screen shows for this test.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::BasicMotor => "Basic Motor Test",
            Self::Turns => "Turns Test",
            Self::StraightDrive => "Straight Drive",
            Self::ArcDrive => "Arc Drive",
            Self::Imu6Axis => "IMU Test (6-axis)",
            Self::Imu9Axis => "IMU Test (9-axis)",
        }
    }

    /// Whether the test runs until the operator stops it.
    ///
    /// An interactive test streams sensor telemetry and never ends by itself, so
    /// leaving it returns to the test menu; a run-to-completion test drives a
    /// fixed sequence, raises [`crate::system::event::Events::TestingCompleted`],
    /// and returns to the main menu.
    #[must_use]
    pub const fn is_interactive(self) -> bool {
        matches!(self, Self::BasicMotor | Self::Imu6Axis | Self::Imu9Axis)
    }
}

/// Which autonomous drive mode is running.
///
/// Coast-and-avoid is the only mode the panel can start today; the attempt-straight
/// entry only records its value, so it has no running state yet.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum DriveModeKind {
    /// Coast-and-avoid obstacle avoidance, driven from the panel.
    CoastAndAvoid,
}

impl DriveModeKind {
    /// The title the running screen shows for this drive mode.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::CoastAndAvoid => "Coast & Avoid",
        }
    }
}

/// Which calibration procedure is running.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum CalibrationKind {
    /// The motor (track balance) calibration.
    Motor,
    /// The magnetometer calibration.
    Mag,
    /// The distance calibration.
    Distance,
}

impl CalibrationKind {
    /// The title the running screen shows for this calibration.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Motor => "Motor",
            Self::Mag => "Mag",
            Self::Distance => "Distance",
        }
    }
}

/// What the robot is currently doing.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Activity {
    /// Nothing long-running is in progress.
    Idle,
    /// The boot/initialisation flow is loading calibration data.
    Booting,
    /// A test-mode procedure is running.
    Test(TestKind),
    /// A calibration procedure is running.
    Calibration(CalibrationKind),
    /// An autonomous drive mode is running.
    DriveMode(DriveModeKind),
}

impl Activity {
    /// The title the running screen shows for this activity.
    #[must_use]
    pub const fn title(self) -> &'static str {
        match self {
            Self::Idle => "Running",
            Self::Booting => "Boot",
            Self::Test(kind) => kind.title(),
            Self::Calibration(kind) => kind.title(),
            Self::DriveMode(kind) => kind.title(),
        }
    }

    /// Whether this activity is a calibration.
    #[must_use]
    pub const fn is_calibration(self) -> bool {
        matches!(self, Self::Calibration(_))
    }
}

/// How far the recorded activity has got.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
pub enum Stage {
    /// Under way.
    Running,
    /// Reached its end successfully.
    Complete,
    /// Could not run, or failed part-way, with the reason in the phase line.
    Failed,
}

/// A consistent snapshot of the activity state, read under a single lock.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Snapshot {
    /// What is running.
    pub activity: Activity,
    /// How far it has got.
    pub stage: Stage,
    /// A short, operator-facing line: the current phase while running, or the
    /// result/reason once complete or failed.
    pub detail: &'static str,
    /// Percent progress (0–100), when the procedure can report one.
    pub percent: Option<u8>,
    /// Whether the procedure only ends on an explicit stop.
    pub interactive: bool,
}

impl Snapshot {
    /// The title the running screen shows.
    #[must_use]
    pub const fn title(&self) -> &'static str {
        self.activity.title()
    }

    /// Whether a calibration is under way — the panel's
    /// [`crate::task::ui::ui_is_calibrating`] test.
    ///
    /// A calibration that has finished but has not been dismissed yet is not
    /// "under way".
    #[must_use]
    pub const fn is_calibrating(&self) -> bool {
        self.activity.is_calibration() && matches!(self.stage, Stage::Running)
    }
}

/// Global activity state protected by a mutex.
static ACTIVITY_STATE: Mutex<CriticalSectionRawMutex, Snapshot> = Mutex::new(Snapshot {
    activity: Activity::Idle,
    stage: Stage::Running,
    detail: "",
    percent: None,
    interactive: false,
});

/// Start `activity`, replacing whatever was recorded before.
///
/// The stage resets to [`Stage::Running`], any stale percent is dropped, and
/// `interactive` records whether the procedure ends only on an explicit stop.
pub async fn begin(activity: Activity, detail: &'static str, interactive: bool) {
    let mut state = ACTIVITY_STATE.lock().await;
    *state = Snapshot {
        activity,
        stage: Stage::Running,
        detail,
        percent: None,
        interactive,
    };
}

/// Start `activity` only if nothing else is running, reporting whether it did.
///
/// The boot flow uses this so a test or calibration already on screen keeps it:
/// a background load has no business replacing what the operator is watching.
pub async fn begin_if_idle(activity: Activity, detail: &'static str, interactive: bool) -> bool {
    let mut state = ACTIVITY_STATE.lock().await;
    if state.activity != Activity::Idle {
        return false;
    }
    *state = Snapshot {
        activity,
        stage: Stage::Running,
        detail,
        percent: None,
        interactive,
    };
    true
}

/// Advance the phase line and percent of the running activity.
///
/// A no-op when nothing is running, so a stopped producer's late updates are
/// discarded rather than resurrecting a cleared activity.
pub async fn set_running(detail: &'static str, percent: Option<u8>) {
    let mut state = ACTIVITY_STATE.lock().await;
    if state.activity == Activity::Idle {
        return;
    }
    state.stage = Stage::Running;
    state.detail = detail;
    state.percent = percent;
}

/// Advance the phase line and percent, but only while `activity` is the one
/// running.
///
/// The boot flow reports through this so a test or calibration that claimed the
/// panel after boot started is never overwritten. Reports whether it updated.
pub async fn set_running_for(activity: Activity, detail: &'static str, percent: Option<u8>) -> bool {
    let mut state = ACTIVITY_STATE.lock().await;
    if state.activity != activity {
        return false;
    }
    state.stage = Stage::Running;
    state.detail = detail;
    state.percent = percent;
    true
}

/// Record that the running activity finished successfully.
///
/// The percent goes to 100 and the phase line carries the result. A no-op when
/// nothing is running.
pub async fn complete(detail: &'static str) {
    let mut state = ACTIVITY_STATE.lock().await;
    if state.activity == Activity::Idle {
        return;
    }
    state.stage = Stage::Complete;
    state.detail = detail;
    state.percent = Some(100);
}

/// Record that the running activity could not run or failed, with the reason.
///
/// The reason is the phase line, so the UI shows it on the running screen rather
/// than formatting text of its own. A no-op when nothing is running.
pub async fn fail(detail: &'static str) {
    let mut state = ACTIVITY_STATE.lock().await;
    if state.activity == Activity::Idle {
        return;
    }
    state.stage = Stage::Failed;
    state.detail = detail;
    state.percent = None;
}

/// Forget the running activity.
///
/// Called when the UI leaves a running screen, and by a producer whose procedure
/// was stopped before it reached an end state.
pub async fn clear() {
    let mut state = ACTIVITY_STATE.lock().await;
    *state = Snapshot {
        activity: Activity::Idle,
        stage: Stage::Running,
        detail: "",
        percent: None,
        interactive: false,
    };
}

/// Forget the running activity, but only while `activity` is the one running.
///
/// The boot flow uses this so it cannot clear a test or calibration that claimed
/// the panel after boot started. Reports whether it cleared.
pub async fn clear_for(activity: Activity) -> bool {
    let mut state = ACTIVITY_STATE.lock().await;
    if state.activity != activity {
        return false;
    }
    *state = Snapshot {
        activity: Activity::Idle,
        stage: Stage::Running,
        detail: "",
        percent: None,
        interactive: false,
    };
    true
}

/// Read a consistent snapshot under a single lock.
pub async fn snapshot() -> Snapshot {
    *ACTIVITY_STATE.lock().await
}

/// Whether a calibration is under way.
pub async fn is_calibrating() -> bool {
    ACTIVITY_STATE.lock().await.is_calibrating()
}

/// Percent of a staged procedure completed before stage `index` of `total`.
///
/// Producers publish this as the running screen's percent, so a staged procedure
/// reports where it is without formatting a number.
#[must_use]
pub fn percent_done(index: usize, total: usize) -> u8 {
    u8::try_from(index.saturating_mul(100) / total.max(1)).unwrap_or(100)
}

// ── Stop requests ──────────────────────────────────────────────────────────────

/// A latched stop request for the activity one producer owns.
///
/// The latch is sticky for the lifetime of a run: once the operator asks a
/// procedure to stop, every later check in that procedure sees the request, so a
/// `select` that drops the waiting future cannot lose it. A producer re-arms the
/// latch when its run starts, which is what makes a stale request from the
/// previous run harmless.
///
/// Each producer holds its own instance, so a stop aimed at a test cannot leak
/// into a calibration and vice versa.
pub struct StopRequest {
    /// Latched flag, checked without awaiting.
    requested: AtomicBool,
    /// Waker for a producer waiting between steps.
    signal: Signal<CriticalSectionRawMutex, ()>,
}

impl StopRequest {
    /// Create an un-requested latch.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            requested: AtomicBool::new(false),
            signal: Signal::new(),
        }
    }

    /// Latch a stop request and wake any producer waiting on it.
    pub fn request(&self) {
        self.requested.store(true, Ordering::Release);
        self.signal.signal(());
    }

    /// Discard a request left over from a previous run and start waiting fresh.
    pub async fn rearm(&self) {
        self.requested.store(false, Ordering::Release);
        while self.signal.signaled() {
            self.signal.wait().await;
        }
    }

    /// Whether a stop has been requested since the last re-arm.
    #[must_use]
    pub fn is_requested(&self) -> bool {
        self.requested.load(Ordering::Acquire)
    }

    /// Resolve as soon as a stop has been requested.
    pub async fn wait(&self) {
        if self.is_requested() {
            return;
        }
        self.signal.wait().await;
    }

    /// Wait for `duration_ms`, returning `true` early if a stop is requested.
    pub async fn wait_or(&self, duration_ms: u64) -> bool {
        match select(self.wait(), Timer::after(Duration::from_millis(duration_ms))).await {
            Either::First(()) => true,
            Either::Second(()) => false,
        }
    }
}

impl Default for StopRequest {
    /// Equivalent to [`StopRequest::new`].
    fn default() -> Self {
        Self::new()
    }
}
