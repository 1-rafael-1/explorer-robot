//! Rangefinder telemetry snapshot.
//!
//! The pair task publishes the latest per-sensor reading, decoded status, raw
//! code, diagnostics, floor state, age and pair state after every reading. The
//! snapshot is panel-only: it is not part of Activity State and never enters the
//! `ACTIVITY` stream or the Link Protocol.
//!
//! The Rangefinder Test task reads this snapshot and maps it onto the neutral
//! [`touch_ui::RangefinderTestReadout`]; the UI is the only formatter.
//!
//! # Lock order
//!
//! `RANGEFINDER_STATE` is an independent leaf: it is never taken while another
//! state mutex is held.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use rangefinder::{FloorState, PairState, RangeDiagnostics, RangeStatus, SensorState};

/// One sensor's telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SensorTelemetry {
    /// The sensor's state within the pair.
    pub state: SensorState,
    /// The last valid range in millimetres, or `None` when the last reading was
    /// invalid or the sensor has not produced one yet.
    pub range_mm: Option<u16>,
    /// The last decoded Range Status, or `None` before the first reading.
    pub status: Option<RangeStatus>,
    /// The last raw 8-bit Range Status code, or `None` before the first reading.
    pub raw_status: Option<u8>,
    /// The raw signal and ambient rates and the effective SPAD count of the last
    /// reading, or `None` before the first reading.
    pub diagnostics: Option<RangeDiagnostics>,
    /// The detector's floor state.
    pub floor: FloorState,
    /// Age of the last reading in milliseconds, or `None` before the first
    /// reading.
    pub age_ms: Option<u32>,
}

/// The pair's latest telemetry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RangefinderTelemetry {
    /// The front sensor.
    pub front: SensorTelemetry,
    /// The rear sensor.
    pub rear: SensorTelemetry,
    /// The pair's Floor-Drop Protection state.
    pub pair: PairState,
    /// The aggregate floor-drop decision: the OR of the two sensor states.
    pub floor_drop: bool,
    /// The distance above which a valid reading means floor drop.
    pub threshold_mm: u16,
}

/// The latest snapshot, or `None` before the first publish and after a clear.
static RANGEFINDER_STATE: Mutex<CriticalSectionRawMutex, Option<RangefinderTelemetry>> = Mutex::new(None);

/// Publish the latest snapshot, replacing any previous one.
pub async fn publish(telemetry: RangefinderTelemetry) {
    *RANGEFINDER_STATE.lock().await = Some(telemetry);
}

/// Read the latest snapshot, if the pair task has published one.
pub async fn snapshot() -> Option<RangefinderTelemetry> {
    *RANGEFINDER_STATE.lock().await
}

/// Forget the latest snapshot.
///
/// Called when the pair powers down, so a previous run's values never flash on
/// the next screen.
pub async fn clear() {
    *RANGEFINDER_STATE.lock().await = None;
}
