//! The Procedure lifecycle every producer family calls.
//!
//! A Procedure — a test mode or a calibration — repeats the same steps around its
//! own body: claim its family's slot, arm its family's stop latch, publish the
//! starting activity, publish each phase and percent, publish the terminal
//! outcome, release the family's single-active slot, and raise the family's
//! completion event. This module owns those steps once, so a new Procedure is its
//! body and its phase words.
//!
//! It is deliberately not a runner. A producer keeps its own body and calls a
//! [`Lifecycle`] around it; handing the body to a generic executor was rejected
//! in ADR-0014, because the two families vary too much for one runner to absorb
//! without growing a parameter per difference.
//!
//! Each family keeps its own [`StopLatch`] instance and its own single-active
//! slot, so a stop aimed at a test mode cannot reach a calibration.

use touch_ui::Procedure;

use crate::{
    system::{
        event::{Events, raise_event},
        state::activity::{self, Activity, StopRequest},
    },
    task::drive::{self, InterruptKind},
};

/// A latched stop request for one producer family.
///
/// The latch is sticky for a run: once the operator asks a Procedure to stop,
/// every later check in that Procedure sees the request, so a `select` that drops
/// the waiting future cannot lose it. [`Lifecycle::start`] arms the latch — after
/// claiming the family's slot and just before publishing the starting activity —
/// which is what makes a stale request from the previous run harmless.
///
/// Each family holds exactly one instance, so a stop aimed at a test mode cannot
/// leak into a calibration and vice versa.
pub struct StopLatch {
    /// The family's underlying request, shared by every Procedure in the family.
    stop: StopRequest,
}

impl StopLatch {
    /// Create an un-requested latch.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            stop: StopRequest::new(),
        }
    }

    /// Discard a request left over from a previous run and start waiting fresh.
    pub async fn arm(&self) {
        self.stop.rearm().await;
    }

    /// Whether the operator has asked the running Procedure to stop.
    #[must_use]
    pub fn is_requested(&self) -> bool {
        self.stop.is_requested()
    }

    /// Wait for `duration_ms`, returning `true` early if a stop is requested.
    pub async fn wait_or(&self, duration_ms: u64) -> bool {
        self.stop.wait_or(duration_ms).await
    }

    /// Latch a stop request and interrupt any drive in flight.
    ///
    /// A Procedure that is not driving ignores the interrupt; the drive loop
    /// drains it and coasts.
    pub fn request(&self) {
        self.stop.request();
        drive::send_drive_interrupt(InterruptKind::Stop);
    }
}

impl Default for StopLatch {
    /// Equivalent to [`StopLatch::new`].
    fn default() -> Self {
        Self::new()
    }
}

/// The completion event a family raises for its Procedures.
///
/// Which outcomes raise an event is the family's policy. The test family raises
/// only when a run-to-completion test succeeds, and the interactive tests raise
/// nothing. The motor and magnetometer calibrations raise on both success and
/// failure, so a finished calibration never looks hung; distance calibration,
/// whose flow crosses the value-entry screen, raises nothing. The event is a
/// function rather than a value so a family can name it in a `const` [`Lifecycle`].
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

/// A family's single-active slot, claimed while one of its Procedures is live.
///
/// A family with a slot refuses a second Procedure while the first holds it, and
/// releases the slot when the first ends. A family without a slot stores `None`
/// instead, and both the claim and the release are no-ops.
#[derive(Clone, Copy)]
pub struct Slot {
    /// Claims the slot, reporting whether it was free.
    claim: fn() -> bool,
    /// Releases a slot this family's Procedure claimed.
    release: fn(),
}

impl Slot {
    /// A slot backed by `claim` and `release`.
    #[must_use]
    pub const fn guarded(claim: fn() -> bool, release: fn()) -> Self {
        Self { claim, release }
    }
}

/// The shared lifecycle of one running Procedure.
///
/// A producer keeps one of these for its identity — its family's stop latch, its
/// family's single-active slot, and its family's completion event — and calls it
/// around its own body. It is not a runner: the producer still owns the body and
/// its phase words.
#[derive(Clone, Copy)]
pub struct Lifecycle {
    /// The Procedure identity being run.
    procedure: Procedure,
    /// The family's stop latch.
    latch: &'static StopLatch,
    /// The family's single-active slot, or `None` when the family has none.
    slot: Option<Slot>,
    /// The family's completion-event policy.
    completion: Completion,
}

impl Lifecycle {
    /// Build the lifecycle for `procedure`.
    #[must_use]
    pub const fn new(
        procedure: Procedure,
        latch: &'static StopLatch,
        slot: Option<Slot>,
        completion: Completion,
    ) -> Self {
        Self {
            procedure,
            latch,
            slot,
            completion,
        }
    }

    /// Whether the operator has asked this Procedure to stop.
    #[must_use]
    pub fn is_stop_requested(&self) -> bool {
        self.latch.is_requested()
    }

    /// Wait for `duration_ms`, returning `true` early if a stop is requested.
    pub async fn wait_or_stop(&self, duration_ms: u64) -> bool {
        self.latch.wait_or(duration_ms).await
    }

    /// Publish the current phase and percent to the Activity State.
    pub async fn phase(&self, detail: &'static str, percent: Option<u8>) {
        activity::set_running(detail, percent).await;
    }

    /// Claim the family's slot, arm the family's stop latch, and publish the
    /// starting activity.
    ///
    /// Reports whether it started. A guarded slot refuses while another Procedure
    /// of the family is live, and then nothing is recorded and the latch is left
    /// alone, so a refused start cannot disturb a sibling Procedure's latch.
    #[must_use]
    pub async fn start(&self, detail: &'static str) -> bool {
        if self.slot.is_some_and(|slot| !(slot.claim)()) {
            return false;
        }
        // Arm before the starting activity is published — the moment the running
        // screen and its Stop become reachable — and never after, so a Stop that
        // lands once the Procedure is exposed is preserved.
        self.latch.arm().await;
        activity::begin(Activity::Procedure(self.procedure), detail).await;
        true
    }

    /// Record success, release the family's slot, and raise the family's
    /// completion event.
    ///
    /// This is the whole run-to-completion success sequence, in the order the
    /// producers used before the lifecycle owned it. A family that raises only on
    /// success raises nothing, since a stop never reaches here.
    pub async fn complete(&self, detail: &'static str) {
        activity::complete(detail).await;
        self.release();
        match self.completion {
            Completion::OnSuccess(event) | Completion::OnBoth(event) => raise_event(event()).await,
            Completion::Silent => {}
        }
    }

    /// Record a failure with `reason`.
    ///
    /// The slot is left to the caller's stop path. A family that raises only on
    /// success raises nothing here; a calibration family that raises on both
    /// outcomes raises here too.
    pub async fn fail(&self, reason: &'static str) {
        activity::fail(reason).await;
        if let Completion::OnBoth(event) = self.completion {
            raise_event(event()).await;
        }
    }

    /// Forget the running activity, releasing the family's slot.
    ///
    /// The terminal outcome of a Procedure the operator stopped: it records
    /// nothing and raises no event, because the UI is already leaving the running
    /// screen. A producer calls this from its stop path, after its own hardware
    /// cleanup.
    pub async fn abandon(&self) {
        activity::clear().await;
        self.release();
    }

    /// Release the family's single-active slot.
    pub fn release(&self) {
        if let Some(slot) = self.slot {
            (slot.release)();
        }
    }
}
