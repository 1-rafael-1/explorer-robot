//! Calibration state module.
//!
//! Holds calibration status flags and exposes helpers for initialization checks.
//!
//! Lock order (when multiple state mutexes are needed):
//! 1) power state mutex (use power module accessors)
//! 2) `CALIBRATION_STATE`
//! 3) perception mutex (use accessor functions, see `perception` module)
//! 4) `MOTION_STATE`
//! 5) `ACTIVITY_STATE` (use accessor functions, see `activity` module)
//! 6) `IMU_TEST_STATE` — independent leaf; never taken while another state mutex is held
//!
//! Every accessor here takes only `CALIBRATION_STATE`.

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};

use crate::system::state::CalibrationStatus;

/// Global calibration state protected by a mutex.
pub static CALIBRATION_STATE: Mutex<CriticalSectionRawMutex, CalibrationState> = Mutex::new(CalibrationState {
    motor_cal_status: CalibrationStatus::NotLoaded,
    distance_cal_status: CalibrationStatus::NotLoaded,
    dmp_cal_status: CalibrationStatus::NotLoaded,
    distance_factor: 1.0,
});

/// Calibration state shared across the system.
pub struct CalibrationState {
    /// Motor calibration status.
    pub motor_cal_status: CalibrationStatus,
    /// Distance calibration status.
    pub distance_cal_status: CalibrationStatus,
    /// DMP Bias Snapshot status.
    ///
    /// `Loaded` means the snapshot was actually written to the DMP, not merely
    /// found in flash; `NotAvailable` covers an absent, corrupt or rejected
    /// snapshot.
    pub dmp_cal_status: CalibrationStatus,
    /// Distance calibration factor (1.0 = no correction).
    pub distance_factor: f32,
}

impl CalibrationState {
    /// Checks if system initialization is complete.
    pub fn is_initialized(&self) -> bool {
        self.motor_cal_status != CalibrationStatus::NotLoaded
            && self.distance_cal_status != CalibrationStatus::NotLoaded
            && self.dmp_cal_status != CalibrationStatus::NotLoaded
    }
}

/// Record the DMP Bias Snapshot status after the boot apply attempt.
///
/// The caller must not hold [`CALIBRATION_STATE`] across the DMP write; this
/// helper takes the lock only to store the settled result.
pub async fn set_dmp_cal_status(status: CalibrationStatus) {
    CALIBRATION_STATE.lock().await.dmp_cal_status = status;
}

/// Checks if system initialization is complete (async helper).
pub async fn is_initialized() -> bool {
    let state = CALIBRATION_STATE.lock().await;
    state.is_initialized()
}

/// Map the firmware's calibration status onto the UI's neutral one.
///
/// A never-queried status is *unknown*, not missing: only flash reporting that
/// no calibration exists is *missing*.
#[must_use]
pub const fn ui_status(status: CalibrationStatus) -> touch_ui::CalibrationStatus {
    match status {
        CalibrationStatus::Loaded => touch_ui::CalibrationStatus::Loaded,
        CalibrationStatus::NotLoaded => touch_ui::CalibrationStatus::Unknown,
        CalibrationStatus::NotAvailable => touch_ui::CalibrationStatus::Missing,
    }
}

/// Return the distance calibration factor from the global state.
pub async fn get_distance_factor() -> f32 {
    let state = CALIBRATION_STATE.lock().await;
    state.distance_factor
}
