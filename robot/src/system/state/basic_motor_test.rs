//! Basic Motor Test UI leaf.
//!
//! A panel-only snapshot of the neutral [`touch_ui::BasicMotorReadout`] the
//! Basic Motor Test task publishes as its Power Steps run. It is deliberately
//! not part of Activity State: the `ACTIVITY` stream and the Link Protocol are
//! unchanged, so per-step encoder counts never reach the wire.
//!
//! The task owns the readout's rules and writes a typed copy here; the touch UI
//! formats the rows and is the only thing that draws, so the task never formats
//! text.
//!
//! # Lock order
//!
//! `BASIC_MOTOR_TEST_STATE` is an independent leaf: it is never taken while
//! another state mutex is held. See [`crate::system::state::activity`] for the
//! full order.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use touch_ui::BasicMotorReadout;

/// The latest Basic Motor Test Readout, or `None` before the first publish and
/// after a clear.
static BASIC_MOTOR_TEST_STATE: Mutex<CriticalSectionRawMutex, Option<BasicMotorReadout>> = Mutex::new(None);

/// Publish the latest readout, replacing any previous snapshot.
pub async fn publish(readout: BasicMotorReadout) {
    *BASIC_MOTOR_TEST_STATE.lock().await = Some(readout);
}

/// Read the latest readout, if the running test has published one.
pub async fn snapshot() -> Option<BasicMotorReadout> {
    *BASIC_MOTOR_TEST_STATE.lock().await
}

/// Forget the latest readout.
///
/// Called when the test starts and when it stops, so a previous run's values
/// never flash on the next screen.
pub async fn clear() {
    *BASIC_MOTOR_TEST_STATE.lock().await = None;
}
