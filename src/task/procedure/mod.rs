//! The one Active Claim every motion producer takes.
//!
//! A Procedure repeats the same steps around its own body: take the claim, publish
//! the starting activity, publish each phase and percent, publish the terminal
//! outcome, release the claim, and raise the completion event. This module owns
//! those steps once, so a new Procedure is its body and its phase words.
//!
//! It is deliberately not a runner. A producer keeps its own body and calls a
//! [`Claim`] around it; handing the body to a generic executor was rejected in
//! ADR-0014, because the producers vary too much for one runner to absorb without
//! growing a parameter per difference.
//!
//! # The Active Claim
//!
//! This module owns the one Active Claim (ADR-0034): a single [`ClaimSlot`] and one
//! stop-wake [`Signal`] beside it. A producer takes a [`Claim`] with [`claim`] or
//! [`start`], reports its phases and its terminal outcome through it, and [`stop`]
//! latches the stop for whichever claim holds the slot. The slot is the only
//! authority — there are no per-family flags.
//!
//! Every [`Claim`] method first checks its own generation, so a claim that lost the
//! slot is inert: a stale `phase`, `complete`, `fail` or `abandon` cannot write
//! over the next claim's Activity, and a stale claim never observes a stop or
//! releases a newer claim. `stop` itself writes no Activity: the owner observes the
//! latch, cleans up, and then records the terminal outcome.

use claim_slot::{ClaimSlot, Generation};
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use touch_ui::Procedure;

use crate::{
    system::{
        event::{Events, raise_event},
        state::activity::{self, Activity},
    },
    task::drive::{self, InterruptKind},
};

/// The completion event a Procedure raises.
///
/// Which outcomes raise an event is the policy in [`completion_for`]. The
/// run-to-completion tests raise only on success, and the interactive tests raise
/// nothing. The motor and magnetometer calibrations raise on both success and
/// failure, so a finished calibration never looks hung; distance calibration,
/// whose flow crosses the value-entry screen, raises nothing. The event is a
/// function rather than a value so the table can name it in a `const`.
#[derive(Clone, Copy)]
pub enum Completion {
    /// Raise the event when the Procedure finishes successfully.
    ///
    /// A failure or a stop raises nothing.
    OnSuccess(fn() -> Events),
    /// Raise the event when the Procedure reaches either terminal outcome.
    ///
    /// A stop still raises nothing: the operator ended it, and the UI is already
    /// leaving the running screen.
    OnBoth(fn() -> Events),
    /// Raise no completion event at all.
    Silent,
}

// ── The Active Claim ──────────────────────────────────────────────────────────

/// What a [`Claim`] is held for.
///
/// The kinds model today's producers: the boot flow, a Procedure the Panel
/// started, and a screen that needs exclusivity without an Activity write.
///
/// `ClaimOrigin` — whether the Panel or the Remote asked — and a `ManualDrive`
/// kind are deliberately absent: they arrive with ADR-0035 and the remote link,
/// and nothing produces them yet.
#[derive(Clone, Copy)]
pub enum ClaimKind {
    /// The boot/initialisation flow is loading calibration data.
    Boot,
    /// One Procedure started from the Panel is running.
    Procedure(Procedure),
    /// A screen holds the claim without publishing an Activity.
    Screen(ScreenKind),
}

/// A screen that holds the claim but writes no Activity.
#[derive(Clone, Copy)]
pub enum ScreenKind {
    /// The radar screen, which blocks Procedures and is blocked by them.
    RoomScan,
}

/// One held claim: the slot generation it owns and what it is for.
///
/// A `Claim` is a capability, not a handle to the slot. Every method first checks
/// that its generation still holds the slot; a claim that outlived its run because
/// it was stopped, or because it was released and another claim took over, is
/// inert. It cannot write Activity over its successor, observe its stop, or release
/// it.
#[derive(Clone, Copy)]
pub struct Claim {
    /// The generation that held the slot when this claim was minted.
    generation: Generation,
    /// What the claim is for.
    kind: ClaimKind,
}

impl Claim {
    /// Whether this claim's generation still holds the slot.
    fn is_current(&self) -> bool {
        CLAIM_SLOT.generation() == Some(self.generation)
    }

    /// Publish the current phase and percent to the Activity State.
    ///
    /// A no-op once the claim is stale.
    pub async fn phase(&self, detail: &'static str, percent: Option<u8>) {
        if !self.is_current() {
            return;
        }
        activity::set_running(detail, percent).await;
    }

    /// Record success, release the slot, and raise the completion event.
    ///
    /// A no-op once the claim is stale. The event follows [`completion_for`]: a
    /// Procedure that raises on success or on both outcomes raises here, and boot
    /// or a screen raises nothing.
    pub async fn complete(&self, detail: &'static str) {
        if !self.is_current() {
            return;
        }
        activity::complete(detail).await;
        self.release();
        let ClaimKind::Procedure(procedure) = self.kind else {
            return;
        };
        match completion_for(procedure) {
            Completion::OnSuccess(event) | Completion::OnBoth(event) => raise_event(event()).await,
            Completion::Silent => {}
        }
    }

    /// Record a failure with `reason`, release the slot, and raise an event when
    /// the completion table asks for one on both outcomes.
    ///
    /// A no-op once the claim is stale. A stop still raises nothing: the operator
    /// ended it, and the UI is already leaving the running screen.
    pub async fn fail(&self, reason: &'static str) {
        if !self.is_current() {
            return;
        }
        activity::fail(reason).await;
        self.release();
        let ClaimKind::Procedure(procedure) = self.kind else {
            return;
        };
        if let Completion::OnBoth(event) = completion_for(procedure) {
            raise_event(event()).await;
        }
    }

    /// Forget the running activity and release the slot.
    ///
    /// The terminal call for work the operator stopped: it records nothing and
    /// raises no event, because the UI is already leaving the running screen. A
    /// no-op once the claim is stale.
    pub async fn abandon(&self) {
        if !self.is_current() {
            return;
        }
        activity::clear().await;
        self.release();
    }

    /// Release the slot.
    ///
    /// Harmless on a stale generation: the slot only frees when this claim's
    /// generation is still the one holding it, so a late release cannot free a
    /// newer claim.
    pub fn release(&self) {
        // The slot reports whether it freed; a stale generation is expected here
        // and is not an error.
        let _released = CLAIM_SLOT.release(self.generation);
    }

    /// Whether the operator has asked the running work to stop.
    ///
    /// `false` for a stale claim, so a displaced owner never observes a stop
    /// aimed at its successor.
    #[must_use]
    pub fn is_stop_requested(&self) -> bool {
        self.is_current() && CLAIM_SLOT.is_stop_requested()
    }

    /// Wait for `duration_ms`, returning `true` early if a stop is requested.
    ///
    /// Returns `false` without waiting for a stale claim. The latch is re-checked
    /// before waiting, so a stop that landed since the last check is not lost when
    /// a `select` drops the waiting future.
    pub async fn wait_or_stop(&self, duration_ms: u64) -> bool {
        if !self.is_current() {
            return false;
        }
        if CLAIM_SLOT.is_stop_requested() {
            return true;
        }
        match select(STOP_WAKE.wait(), Timer::after(Duration::from_millis(duration_ms))).await {
            Either::First(()) => true,
            Either::Second(()) => false,
        }
    }
}

/// What [`start`] did with a Procedure.
#[derive(Clone, Copy)]
pub enum StartOutcome {
    /// The slot was free; the caller owns the claim and routes it to the task.
    Started(Claim),
    /// Another claim held the slot; nothing was claimed and nothing was published.
    Busy,
    /// The request was refused before claiming.
    ///
    /// Defined for the remote layer (ADR-0035); the panel path never produces it, so
    /// this variant stays unused until that layer lands.
    #[allow(dead_code)]
    Rejected,
}

/// The one Active Claim slot: the only authority over what is running.
static CLAIM_SLOT: ClaimSlot = ClaimSlot::new();

/// Wakes a [`Claim::wait_or_stop`] when [`stop`] latches a stop.
static STOP_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Claim the slot for `kind` and publish the Activity that kind stands for.
///
/// Returns `None` when another claim holds the slot; nothing is published then. On
/// success the latch is armed and any wake left over from a previous stop is
/// drained before the Activity is published — that is the moment the running
/// screen and its Stop become reachable, so a Stop that lands after it is kept.
pub async fn claim(kind: ClaimKind) -> Option<Claim> {
    let generation = CLAIM_SLOT.try_claim()?;
    let claim = Claim { generation, kind };
    CLAIM_SLOT.arm();
    while STOP_WAKE.signaled() {
        STOP_WAKE.wait().await;
    }
    match kind {
        ClaimKind::Boot => activity::begin(Activity::Booting, "Initializing").await,
        ClaimKind::Procedure(procedure) => activity::begin(Activity::Procedure(procedure), "Starting").await,
        ClaimKind::Screen(_) => {}
    }
    Some(claim)
}

/// Claim the slot for `procedure`.
///
/// The caller owns the `Claim` and routes it to the task that runs the Procedure;
/// this does not spawn a task and does not dispatch. The generic "Starting"
/// detail is published here and the task's first [`Claim::phase`] call replaces it.
pub async fn start(procedure: Procedure) -> StartOutcome {
    claim(ClaimKind::Procedure(procedure))
        .await
        .map_or(StartOutcome::Busy, StartOutcome::Started)
}

/// Latch a stop for whichever claim holds the slot.
///
/// Wakes a waiting owner and interrupts any drive in flight. It writes no Activity
/// and does not release the slot: the owner observes the latch, cleans up, then
/// records the terminal outcome — [`Claim::abandon`] for a normal stop or
/// [`Claim::fail`] for a failsafe.
pub fn stop() {
    CLAIM_SLOT.request_stop();
    STOP_WAKE.signal(());
    drive::send_drive_interrupt(InterruptKind::Stop);
}

/// The completion-event policy for `procedure`.
///
/// Today's policy: the run-to-completion tests raise [`Events::TestingCompleted`]
/// on success, the motor and magnetometer calibrations raise
/// [`Events::CalibrationCompleted`] on both outcomes so a finished calibration
/// never looks hung, and distance plus the interactive and autonomous Procedures
/// raise nothing — distance because its flow crosses the value-entry screen, the
/// interactive tests because the operator stops them rather than them finishing.
const fn completion_for(procedure: Procedure) -> Completion {
    match procedure {
        Procedure::Turns | Procedure::StraightDrive | Procedure::ArcDrive => Completion::OnSuccess(testing_completed),
        Procedure::MotorCalibration | Procedure::MagCalibration => Completion::OnBoth(calibration_completed),
        Procedure::DistanceCalibration
        | Procedure::CoastAndAvoid
        | Procedure::AttemptStraight
        | Procedure::BasicMotor
        | Procedure::Imu6Axis
        | Procedure::Imu9Axis => Completion::Silent,
    }
}

/// The event the run-to-completion tests raise.
const fn testing_completed() -> Events {
    Events::TestingCompleted
}

/// The event the motor and magnetometer calibrations raise.
const fn calibration_completed() -> Events {
    Events::CalibrationCompleted
}
