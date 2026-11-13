//! DMP Compass Calibration snapshot module.
//!
//! A panel-only snapshot of the typed [`touch_ui::DmpCalibrationSnapshot`] the
//! DMP compass calibration runner publishes at the panel's telemetry rate. It is
//! deliberately not part of Activity State: Activity State stays a lifecycle
//! record (phase plus percent), and the `ACTIVITY` stream and Link Protocol are
//! unchanged, so calibration telemetry never enters either.
//!
//! The touch UI is the only reader and formatter. The runner publishes typed
//! values, never preformatted text.
//!
//! # Lock order
//!
//! `DMP_CAL_STATE` is an independent leaf: it is never taken while another
//! state mutex is held. See [`crate::system::state::activity`] for the full
//! order.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use touch_ui::DmpCalibrationSnapshot;

/// The latest DMP calibration snapshot, or `None` before the first publish and
/// after a clear.
static DMP_CAL_STATE: Mutex<CriticalSectionRawMutex, Option<DmpCalibrationSnapshot>> = Mutex::new(None);

/// Publish the latest snapshot, replacing any previous one.
pub async fn publish(snapshot: DmpCalibrationSnapshot) {
    *DMP_CAL_STATE.lock().await = Some(snapshot);
}

/// Read the latest snapshot, if the running calibration has published one.
pub async fn snapshot() -> Option<DmpCalibrationSnapshot> {
    *DMP_CAL_STATE.lock().await
}

/// Forget the latest snapshot.
///
/// Called when a calibration starts and on every exit path, so a previous run's
/// step or heading never flashes on the next screen.
pub async fn clear() {
    *DMP_CAL_STATE.lock().await = None;
}
