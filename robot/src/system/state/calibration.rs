//! Calibration state module.
//!
//! Holds calibration status flags, the applied Motor Calibration Curve and the
//! encoder scale that came with it, and exposes helpers for initialization
//! checks.
//!
//! The curve and the encoder scale are stored here once and read by both the
//! motor driver (when it resolves `SetTracks`) and the drive path (when it turns
//! a distance into counts). There is no second copy of "what is applied". Both
//! values are `Copy`, so an accessor returns them by value and the lock is never
//! held across another state access.
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
use motor_cal::{Curve, DistanceScale};

use crate::system::state::CalibrationStatus;

/// Global calibration state protected by a mutex.
pub static CALIBRATION_STATE: Mutex<CriticalSectionRawMutex, CalibrationState> = Mutex::new(CalibrationState {
    motor_cal_status: CalibrationStatus::NotLoaded,
    distance_cal_status: CalibrationStatus::NotLoaded,
    dmp_cal_status: CalibrationStatus::NotLoaded,
    motor_curve: None,
    distance_scale: None,
    motor_run_id: 0,
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
    /// The loaded Motor Calibration Curve.
    ///
    /// `None` is the identity curve: `SetTracks` passes the requested magnitude
    /// through unchanged, so an uncalibrated robot still drives.
    pub motor_curve: Option<Curve>,
    /// The measured encoder scale carried by the same Motor Calibration record.
    ///
    /// Stored beside the curve because the record is one unit. The drive path
    /// reads this leaf for every distance drive and falls back to the inferred
    /// nominal scale when it is `None`, so a missing calibration never blocks
    /// driving. `None` also covers a record that carries only that inferred
    /// nominal scale: a motor sweep substitutes the fallback to measure, and boot
    /// leaves it here rather than reporting a scale Distance Calibration never
    /// produced.
    pub distance_scale: Option<DistanceScale>,
    /// The monotonic id of the run that produced the applied motor record.
    ///
    /// `0` means no sweep has produced the record: nothing is loaded, or flash
    /// holds a scale-only record. A Motor Calibration takes the current id, adds
    /// one (never wrapping to `0`), and stores it in the record; the flash task
    /// publishes the stored id after a successful write so this leaf names the
    /// durable record. The save confirmation itself is the flash task's reply, not
    /// this id.
    pub motor_run_id: u32,
}

impl CalibrationState {
    /// Checks if system initialization is complete.
    pub fn is_initialized(&self) -> bool {
        self.motor_cal_status != CalibrationStatus::NotLoaded
            && self.distance_cal_status != CalibrationStatus::NotLoaded
            && self.dmp_cal_status != CalibrationStatus::NotLoaded
    }
}

/// Publish a loaded Motor Calibration Curve and its encoder scale.
///
/// The two values travel together because they come from one record. Passing
/// `None` for the curve retires the applied calibration (a missing, corrupt,
/// wrong-version or truncated record) and reports `NotAvailable`, leaving the
/// identity curve in force. A `Some(curve)` reports `Loaded`.
pub async fn set_motor_calibration(curve: Option<&Curve>, scale: Option<DistanceScale>) {
    let mut state = CALIBRATION_STATE.lock().await;
    state.motor_curve = curve.copied();
    state.distance_scale = scale;
    state.motor_cal_status = if curve.is_some() {
        CalibrationStatus::Loaded
    } else {
        CalibrationStatus::NotAvailable
    };
}

/// Read the applied Motor Calibration Curve.
///
/// `None` means the robot is not calibrated and the request must be used
/// unchanged. The returned `Curve` is a copy, so no lock outlives this call.
pub async fn get_motor_curve() -> Option<Curve> {
    CALIBRATION_STATE.lock().await.motor_curve
}

/// Read the encoder scale stored with the applied Motor Calibration record.
///
/// `None` means Distance Calibration has not run (or its record was absent,
/// corrupt or from another schema version), or the record carries only the
/// inferred nominal scale a motor sweep substituted; the drive path then uses
/// [`DistanceScale::nominal`](motor_cal::DistanceScale::nominal). The returned
/// value is a copy, so no lock outlives this call.
pub async fn get_distance_scale() -> Option<DistanceScale> {
    CALIBRATION_STATE.lock().await.distance_scale
}

/// Read the run id of the applied Motor Calibration record.
///
/// `0` means no sweep has produced the record (nothing loaded, or a scale-only
/// record). A calibration adds one to this value before it builds the record; the
/// flash task publishes the stored id once the record is durable, so the leaf
/// names what is applied. It is not the save confirmation — the flash task's
/// reply is. The returned value is a copy, so no lock outlives this call.
pub async fn get_motor_run_id() -> u32 {
    CALIBRATION_STATE.lock().await.motor_run_id
}

/// Publish the run id of the applied Motor Calibration record.
///
/// Boot calls this with the id decoded from flash (`0` for a scale-only or
/// absent record), and the flash save arm calls it only after the record is
/// durable. Publishing it after the curve keeps a failed write from inventing a
/// run that the curve leaf does not match.
pub async fn set_motor_run_id(run_id: u32) {
    CALIBRATION_STATE.lock().await.motor_run_id = run_id;
}

/// Publish the measured encoder scale, or retire it when it is `None`.
///
/// The status travels with the value so `Loaded` always means "a scale is in
/// force". A calibration's commit replaces the scale here and in flash; an
/// abort restores the value that was backed up before the run.
pub async fn set_distance_scale(scale: Option<DistanceScale>) {
    let mut state = CALIBRATION_STATE.lock().await;
    state.distance_scale = scale;
    state.distance_cal_status = if scale.is_some() {
        CalibrationStatus::Loaded
    } else {
        CalibrationStatus::NotAvailable
    };
}

/// Record the DMP Bias Snapshot status after the boot apply attempt.
///
/// The caller must not hold [`CALIBRATION_STATE`] across the DMP write; this
/// helper takes the lock only to store the settled result.
pub async fn set_dmp_cal_status(status: CalibrationStatus) {
    CALIBRATION_STATE.lock().await.dmp_cal_status = status;
}

/// Read the DMP Bias Snapshot status.
///
/// A ceremony reads this once at start, so its first Prompt can say whether the
/// operator is working from a saved calibration.
pub async fn get_dmp_cal_status() -> CalibrationStatus {
    CALIBRATION_STATE.lock().await.dmp_cal_status
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
