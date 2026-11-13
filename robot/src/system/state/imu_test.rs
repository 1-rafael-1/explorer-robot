//! IMU Test Telemetry snapshot module.
//!
//! A panel-only 10 Hz snapshot of the typed [`touch_ui::ImuReadout`] values the
//! IMU test tasks publish. It is deliberately not part of Activity State:
//! Activity State stays phase plus percent, and the `ACTIVITY` stream and Link
//! Protocol are unchanged, so live sensor data never enters either.
//!
//! The touch UI is the only reader and formatter. The test task publishes typed
//! values, never preformatted text, and builds the snapshot from the latest
//! sensor readings on each publish tick.
//!
//! # Lock order
//!
//! `IMU_TEST_STATE` is an independent leaf: it is never taken while another
//! state mutex is held. See [`crate::system::state::activity`] for the full
//! order.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use touch_ui::ImuReadout;

/// The latest IMU Test Readout, or `None` before the first publish and after a
/// clear.
static IMU_TEST_STATE: Mutex<CriticalSectionRawMutex, Option<ImuReadout>> = Mutex::new(None);

/// Publish the latest readout, replacing any previous snapshot.
pub async fn publish(readout: ImuReadout) {
    *IMU_TEST_STATE.lock().await = Some(readout);
}

/// Read the latest readout, if the running test has published one.
pub async fn snapshot() -> Option<ImuReadout> {
    *IMU_TEST_STATE.lock().await
}

/// Forget the latest readout.
///
/// Called when a test starts and when it stops, so a previous run's values never
/// flash on the next screen.
pub async fn clear() {
    *IMU_TEST_STATE.lock().await = None;
}
