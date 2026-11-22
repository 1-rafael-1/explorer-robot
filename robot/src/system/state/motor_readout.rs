//! Panel-only motor readout leaf.
//!
//! The single panel-only snapshot of the neutral [`touch_ui::BasicMotorReadout`]
//! a Basic Motor Test publishes as its Power Steps run, plus the
//! [`touch_ui::BasicMotorSummary`] published once a run completes. It is
//! deliberately not part of Activity State: the `ACTIVITY` stream and the Link
//! Protocol are unchanged, so per-step encoder counts never reach the wire.
//!
//! Two Procedures share the leaf. The Basic Motor Test publishes the live
//! readout as its samples arrive and the summary when its cycle completes. The
//! Motor Calibration publishes only the summary, for its Result Report
//! (including the optional saved/not-saved footer); its running screen carries
//! the Procedure's phases and progress instead. Both end in the same two tables,
//! so one leaf keeps them consistent instead of adding a second readout leaf.
//!
//! The task owns the readout's rules and writes typed copies here; the touch UI
//! formats the rows and is the only thing that draws, so the task never formats
//! text. Both values live in one leaf so the running view reads the live readout
//! and the finished view reads the summary from one lock, and [`clear`] empties
//! both so a new run never shows the previous run's numbers.
//!
//! # Lock order
//!
//! `MOTOR_READOUT_STATE` is an independent leaf: it is never taken while
//! another state mutex is held. See [`crate::system::state::activity`] for the
//! full order.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use touch_ui::{BasicMotorReadout, BasicMotorSummary};

/// The panel-only motor values a Procedure publishes.
#[derive(Clone, Copy)]
struct MotorReadoutSnapshot {
    /// The live readout, present from the first Power Step until a clear.
    readout: Option<BasicMotorReadout>,
    /// The Result Report summary, present only after a successful completion.
    summary: Option<BasicMotorSummary>,
}

impl MotorReadoutSnapshot {
    /// Nothing published yet.
    const EMPTY: Self = Self {
        readout: None,
        summary: None,
    };
}

/// The latest panel-only motor values, or [`MotorReadoutSnapshot::EMPTY`] before
/// the first publish and after a clear.
static MOTOR_READOUT_STATE: Mutex<CriticalSectionRawMutex, MotorReadoutSnapshot> =
    Mutex::new(MotorReadoutSnapshot::EMPTY);

/// Publish the latest live readout, replacing any previous snapshot.
///
/// The completed summary is left alone: the running view reads the readout and
/// the finished view reads the summary, so a late sample cannot erase the
/// Result Report.
pub async fn publish(readout: BasicMotorReadout) {
    MOTOR_READOUT_STATE.lock().await.readout = Some(readout);
}

/// Publish the completed Result Report summary.
///
/// Called once by the Basic Motor Test or Motor Calibration before the
/// Procedure's claim completes or fails, so the finished view has its tables on
/// the very first draw. The live readout is left in place beneath it.
pub async fn publish_summary(summary: BasicMotorSummary) {
    MOTOR_READOUT_STATE.lock().await.summary = Some(summary);
}

/// Read the latest live readout, if the running test has published one.
pub async fn snapshot() -> Option<BasicMotorReadout> {
    MOTOR_READOUT_STATE.lock().await.readout
}

/// Read the completed summary, if the run finished successfully.
pub async fn summary() -> Option<BasicMotorSummary> {
    MOTOR_READOUT_STATE.lock().await.summary
}

/// Forget the live readout and the completed summary.
///
/// Called when a run starts and when it stops or fails, so a previous run's
/// values never flash on the next screen.
pub async fn clear() {
    *MOTOR_READOUT_STATE.lock().await = MotorReadoutSnapshot::EMPTY;
}
