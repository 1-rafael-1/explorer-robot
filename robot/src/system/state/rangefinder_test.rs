//! Rangefinder Test UI leaf.
//!
//! A panel-only 10 Hz snapshot of the neutral
//! [`touch_ui::RangefinderTestReadout`] the Rangefinder Test task builds from the
//! [`rangefinder`](super::rangefinder) telemetry. It is deliberately not part of
//! Activity State: the `ACTIVITY` stream and the Link Protocol are unchanged, so
//! per-sensor detail never reaches the wire.
//!
//! # Lock order
//!
//! `RANGEFINDER_TEST_STATE` is an independent leaf: it is never taken while
//! another state mutex is held.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use touch_ui::RangefinderTestReadout;

/// The latest Rangefinder Test Readout, or `None` before the first publish and
/// after a clear.
static RANGEFINDER_TEST_STATE: Mutex<CriticalSectionRawMutex, Option<RangefinderTestReadout>> = Mutex::new(None);

/// Publish the latest readout, replacing any previous snapshot.
pub async fn publish(readout: RangefinderTestReadout) {
    *RANGEFINDER_TEST_STATE.lock().await = Some(readout);
}

/// Read the latest readout, if the running test has published one.
pub async fn snapshot() -> Option<RangefinderTestReadout> {
    *RANGEFINDER_TEST_STATE.lock().await
}

/// Forget the latest readout.
///
/// Called when the test starts and when it stops, so a previous run's values
/// never flash on the next screen.
pub async fn clear() {
    *RANGEFINDER_TEST_STATE.lock().await = None;
}
