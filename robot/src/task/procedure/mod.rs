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
//! slot is inert: a stale `phase`, `complete`, `fail`, `finish` or `abandon`
//! cannot write over the next claim's Activity, and a stale claim never observes a
//! stop or releases a newer claim. `stop` itself writes no Activity: the owner
//! observes the latch, cleans up, and then records the terminal outcome.
//!
//! # Origin and the stop outcome
//!
//! A claim carries a transient [`ClaimOrigin`] tag — `Panel` or `Remote` — so the
//! Deadman can stop only Remote-originated work (ADR-0035). The tag lives on the
//! claim, never in Activity State or on the wire, and is cleared with it.
//!
//! [`stop`] takes a [`StopOutcome`] and an [`InterruptKind`]: it latches the
//! outcome, wakes the owner and interrupts the drive, but writes no Activity. The
//! owner's terminal call is [`Claim::finish`], which applies the latched outcome —
//! `Operator` clears to Idle, `Failsafe(reason)` records a held failure. The
//! actual-failure call remains [`Claim::fail`].

use core::{
    cell::Cell,
    sync::atomic::{AtomicU8, Ordering},
};

use claim_slot::{ClaimSlot, Generation};
use embassy_futures::select::{Either, select};
use embassy_sync::{
    blocking_mutex::{Mutex as BlockingMutex, raw::CriticalSectionRawMutex},
    signal::Signal,
};
use embassy_time::{Duration, Timer};
use touch_ui::Procedure;

use crate::{
    system::{
        event::{Events, raise_event},
        state::activity::{self, Activity, ManualDrive, Stage},
    },
    task::{
        drive::{self, InterruptKind},
        testmode, ui,
    },
};

// ── The Active Claim ──────────────────────────────────────────────────────────

/// What a [`Claim`] is held for.
///
/// The kinds model today's producers: the boot flow, a Procedure, and a screen
/// that needs exclusivity without an Activity write. The transient origin of a
/// Procedure or screen is carried beside the kind, not inside it.
#[derive(Clone, Copy)]
pub enum ClaimKind {
    /// The boot/initialisation flow is loading calibration data.
    Boot,
    /// One Procedure is running.
    Procedure(Procedure),
    /// One Manual Drive is running: an Activity-only identity, never a Procedure
    /// and never a Catalogue entry.
    ManualDrive(ManualDrive),
    /// A screen holds the claim without publishing an Activity.
    Screen(ScreenKind),
}

/// Who started the work a claim holds.
///
/// A transient tag on the claim, cleared with it: it is not authority, not on the
/// wire and not in Activity State. The Deadman uses it only to stop
/// Remote-originated work without touching the panel's (ADR-0035).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimOrigin {
    /// The Touch Panel started the work.
    Panel,
    /// The Remote started the work.
    Remote,
}

/// The outcome a stop carries into the owner's terminal record.
///
/// A human Stop and a link-loss failsafe both latch the one stop, but they leave
/// different records: an operator stop clears to Idle, while a failsafe holds a
/// failed Result Report with its reason (ADR-0035).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum StopOutcome {
    /// An operator stopped the work; it clears to Idle.
    Operator,
    /// A failsafe stopped the work; it records a failed Result Report.
    Failsafe(&'static str),
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
    /// Who started it; transient and never written to Activity or the wire.
    ///
    /// [`LIVE_ORIGIN`] mirrors this for the lock-free failsafe observation.
    #[allow(dead_code)] // The capability keeps the tag; `live_origin` is the current reader.
    origin: ClaimOrigin,
}

impl Claim {
    /// Whether this claim's generation still holds the slot.
    fn is_current(&self) -> bool {
        CLAIM_SLOT.generation() == Some(self.generation)
    }

    /// Who started the work this claim holds.
    ///
    /// The tag is on the claim only, so it is cleared when the claim is released.
    /// The Deadman reads the lock-free [`live_origin`] instead, so this is the
    /// owner-facing copy.
    #[must_use]
    #[allow(dead_code)] // Kept for the claim owner's diagnostics; the failsafe uses `live_origin`.
    pub const fn origin(&self) -> ClaimOrigin {
        self.origin
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

    /// Record success, release the slot, and raise the terminal Procedure event.
    ///
    /// A no-op once the claim is stale. The event carries the claim's own
    /// Procedure identity, so it does not race a later Activity read: boot and a
    /// screen raise nothing.
    pub async fn complete(&self, detail: &'static str) {
        if !self.is_current() {
            return;
        }
        activity::complete(detail).await;
        self.release();
        if let ClaimKind::Procedure(procedure) = self.kind {
            raise_event(Events::ProcedureFinished {
                procedure,
                stage: Stage::Complete,
                detail,
                generation: self.generation.get(),
            })
            .await;
        }
    }

    /// Record a failure with `reason`, release the slot, and raise the terminal
    /// Procedure event.
    ///
    /// A no-op once the claim is stale. A stop does not use this: the operator
    /// ended it, and the UI is already leaving the running screen.
    pub async fn fail(&self, reason: &'static str) {
        if !self.is_current() {
            return;
        }
        activity::fail(reason).await;
        self.release();
        if let ClaimKind::Procedure(procedure) = self.kind {
            raise_event(Events::ProcedureFinished {
                procedure,
                stage: Stage::Failed,
                detail: reason,
                generation: self.generation.get(),
            })
            .await;
        }
    }

    /// Apply the latched stop outcome, then release the slot.
    ///
    /// The terminal call for work the operator or a failsafe stopped. It records
    /// the outcome latched by [`stop`]: [`StopOutcome::Operator`] clears to Idle,
    /// [`StopOutcome::Failsafe`] records a failed Result Report with its reason.
    /// It raises no completion event, because a stop is not a finish. No outcome
    /// latched means an operator stop, which is also the non-failsafe default.
    ///
    /// A no-op once the claim is stale.
    pub async fn finish(&self) {
        if !self.is_current() {
            return;
        }
        let outcome = STOP_OUTCOME
            .lock(|cell| cell.replace(None))
            .unwrap_or(StopOutcome::Operator);
        match self.kind {
            // A Manual Drive leaves no Result Report (ADR-0035): its outcome is
            // the drive completion Event, so a stop or failsafe only clears the
            // Activity. The latched outcome is still consumed above.
            ClaimKind::ManualDrive(_) => activity::clear().await,
            ClaimKind::Boot | ClaimKind::Procedure(_) | ClaimKind::Screen(_) => match outcome {
                StopOutcome::Operator => activity::clear().await,
                StopOutcome::Failsafe(reason) => activity::fail(reason).await,
            },
        }
        self.release();
    }

    /// Forget the running activity and release the slot.
    ///
    /// The terminal call for teardown that is not an observed stop: a spawn
    /// failure, a screen leaving, or a Procedure that never started. It records
    /// nothing and raises no event. A no-op once the claim is stale.
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
        // and is not an error. Only a release that actually freed the slot may
        // clear the lock-free origin, so a late release cannot hide a newer claim.
        if CLAIM_SLOT.release(self.generation) {
            LIVE_ORIGIN.store(ORIGIN_NONE, Ordering::Release);
        }
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
    /// The request was refused before claiming, or a controller could not take
    /// the claim it was given (and released it first).
    Rejected,
}

/// The one Active Claim slot: the only authority over what is running.
static CLAIM_SLOT: ClaimSlot = ClaimSlot::new();

/// [`LIVE_ORIGIN`] value meaning no claim is live.
const ORIGIN_NONE: u8 = 0;
/// [`LIVE_ORIGIN`] value meaning the live claim was started by the panel.
const ORIGIN_PANEL: u8 = 1;
/// [`LIVE_ORIGIN`] value meaning the live claim was started by the Remote.
const ORIGIN_REMOTE: u8 = 2;

/// Lock-free observation of the live claim's origin for the Deadman.
///
/// A [`Claim`]'s own `origin` field is the owner's copy; this static is the
/// cross-task copy the Link Task's failsafe reads without taking a lock. It is
/// set by [`claim`] and cleared by [`Claim::release`] only when that release
/// actually frees the slot, so a stale owner cannot hide a newer claim's origin.
/// It is an observation, never authority: stops still go through [`stop`].
static LIVE_ORIGIN: AtomicU8 = AtomicU8::new(ORIGIN_NONE);

/// The live claim's generation, or `None` when the slot is free.
///
/// A terminal event carries the generation that owned the run; the UI uses this
/// to avoid dismissing a newer run that claimed the slot while the event was in
/// flight.
#[must_use]
pub fn live_generation() -> Option<Generation> {
    CLAIM_SLOT.generation()
}

/// The live claim's origin, or `None` when the slot is free.
///
/// This is the Deadman's only input: it lets [`stop`] be conditional on the claim
/// having been started by the Remote (ADR-0035).
#[must_use]
pub fn live_origin() -> Option<ClaimOrigin> {
    match LIVE_ORIGIN.load(Ordering::Acquire) {
        ORIGIN_PANEL => Some(ClaimOrigin::Panel),
        ORIGIN_REMOTE => Some(ClaimOrigin::Remote),
        _ => None,
    }
}

/// The [`LIVE_ORIGIN`] encoding of `origin`.
const fn encode_origin(origin: ClaimOrigin) -> u8 {
    match origin {
        ClaimOrigin::Panel => ORIGIN_PANEL,
        ClaimOrigin::Remote => ORIGIN_REMOTE,
    }
}

/// Wakes a [`Claim::wait_or_stop`] when [`stop`] latches a stop.
static STOP_WAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// The outcome the next [`Claim::finish`] should apply, latched by [`stop`].
///
/// It is cleared with the stop latch when a new claim is armed, so it lives no
/// longer than the claim it belongs to. A cell behind a critical-section mutex is
/// enough: the value is `Copy` and carries only a `&'static str`.
static STOP_OUTCOME: BlockingMutex<CriticalSectionRawMutex, Cell<Option<StopOutcome>>> =
    BlockingMutex::new(Cell::new(None));

/// Claim the slot for `kind` started by `origin` and publish the Activity that
/// kind stands for.
///
/// Returns `None` when another claim holds the slot; nothing is published then. On
/// success the stop latch and any outcome left over from the previous claim are
/// cleared, and any wake left over from a previous stop is drained before the
/// Activity is published — that is the moment the running screen and its Stop
/// become reachable, so a Stop that lands after it is kept.
pub async fn claim(kind: ClaimKind, origin: ClaimOrigin) -> Option<Claim> {
    let generation = CLAIM_SLOT.try_claim()?;
    // A successful claim owns the lock-free origin until it is released; a failed
    // attempt leaves the live claim's origin alone.
    LIVE_ORIGIN.store(encode_origin(origin), Ordering::Release);
    let claim = Claim {
        generation,
        kind,
        origin,
    };
    CLAIM_SLOT.arm();
    STOP_OUTCOME.lock(|cell| cell.set(None));
    while STOP_WAKE.signaled() {
        STOP_WAKE.wait().await;
    }
    match kind {
        ClaimKind::Boot => activity::begin(Activity::Booting, "Initializing").await,
        ClaimKind::Procedure(procedure) => activity::begin(Activity::Procedure(procedure), "Starting").await,
        ClaimKind::ManualDrive(drive) => activity::begin(Activity::ManualDrive(drive), "Starting").await,
        ClaimKind::Screen(_) => {}
    }
    Some(claim)
}

/// Claim the slot for `procedure` started by `origin`.
///
/// The caller owns the `Claim` and routes it to the task that runs the Procedure;
/// this does not spawn a task and does not dispatch. The generic "Starting"
/// detail is published here and the task's first [`Claim::phase`] call replaces it.
pub async fn start(procedure: Procedure, origin: ClaimOrigin) -> StartOutcome {
    claim(ClaimKind::Procedure(procedure), origin)
        .await
        .map_or(StartOutcome::Busy, StartOutcome::Started)
}

/// Start `procedure` for `origin` through the one routing seam.
///
/// Availability is decided before the claim is taken, so a refusal never
/// disturbs a live claim. Once the claim is held it is routed to exactly one
/// controller: the test-mode controller, the calibration controller, or the
/// coast-and-avoid warm-up helper. Each family owns its own Procedure-to-command
/// sub-match; this function is the only top-level routing decision, shared by the
/// panel now and the Link Task later (ADR-0032). A controller that refuses the
/// claim releases it itself, and dispatch reports `Rejected` because the work
/// did not start.
pub async fn dispatch(procedure: Procedure, origin: ClaimOrigin) -> StartOutcome {
    if !is_startable(procedure, origin) {
        return StartOutcome::Rejected;
    }

    let claim = match start(procedure, origin).await {
        StartOutcome::Started(claim) => claim,
        StartOutcome::Busy => return StartOutcome::Busy,
        StartOutcome::Rejected => return StartOutcome::Rejected,
    };

    let routed = match procedure {
        Procedure::BasicMotor
        | Procedure::Turns
        | Procedure::StraightDrive
        | Procedure::ArcDrive
        | Procedure::Imu6Axis
        | Procedure::Imu9Axis => {
            testmode::start(claim, procedure).await;
            true
        }
        Procedure::MotorCalibration | Procedure::MagCalibration | Procedure::DistanceCalibration => {
            drive::calibration::start(claim, procedure).await;
            true
        }
        Procedure::CoastAndAvoid => ui::request_coast_and_avoid(claim).await,
        // Never reached: `is_startable` refuses the value-entry flow first.
        Procedure::AttemptStraight => false,
    };

    if routed {
        StartOutcome::Started(claim)
    } else {
        StartOutcome::Rejected
    }
}

/// Whether `procedure` may be started by `origin`.
///
/// Attempt Straight is a value-entry flow, not a startable Procedure, so it is
/// refused without taking the claim. The Remote's hands-on refusals belong here,
/// in the one availability rule, rather than in a second routing table: motor,
/// magnetometer and distance calibration, the basic-motor test and attempt
/// straight need an operator at the robot. The Panel's behaviour is unchanged.
pub const fn is_startable(procedure: Procedure, origin: ClaimOrigin) -> bool {
    match origin {
        ClaimOrigin::Panel => !matches!(procedure, Procedure::AttemptStraight),
        ClaimOrigin::Remote => !matches!(
            procedure,
            Procedure::MotorCalibration
                | Procedure::MagCalibration
                | Procedure::DistanceCalibration
                | Procedure::BasicMotor
                | Procedure::AttemptStraight
        ),
    }
}

/// Latch a stop for whichever claim holds the slot.
///
/// Wakes a waiting owner, interrupts the drive with `interrupt`, and latches
/// `outcome` for the owner's [`Claim::finish`]. It writes no Activity and does not
/// release the slot: the owner observes the latch, cleans up, then records the
/// terminal outcome. The first outcome latched while a claim is live wins, so a
/// human Stop and a link-loss failsafe racing both leave a truthful record.
pub fn stop(outcome: StopOutcome, interrupt: InterruptKind) {
    STOP_OUTCOME.lock(|cell| {
        if cell.get().is_none() {
            cell.set(Some(outcome));
        }
    });
    CLAIM_SLOT.request_stop();
    STOP_WAKE.signal(());
    drive::send_drive_interrupt(interrupt);
}
