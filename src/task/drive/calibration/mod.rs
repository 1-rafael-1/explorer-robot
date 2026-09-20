//! Calibration procedures for the drive system.
//!
//! Holds the motor (track balance), magnetometer and distance calibration
//! routines. Each procedure publishes its phase, percent and result to
//! [`crate::system::state::activity`]; nothing here formats display text.
//!
//! # Stopping
//!
//! Every procedure is interruptible from the panel. [`request_stop`] latches the
//! shared stop — which each procedure polls at its wait points — and interrupts
//! any drive in flight, so the distance calibration's fixed-distance drive stops
//! mid-leg and the motor procedure's track stops mid-measurement.

pub mod distance;
pub mod imu;
pub mod motor;

use crate::{
    system::state::activity::StopRequest,
    task::drive::{self, InterruptKind},
};

/// The stop request every running calibration procedure polls.
static STOP: StopRequest = StopRequest::new();

// Re-export calibration functions.
pub use imu::run_imu_calibration;
pub use motor::run_motor_calibration;

/// Request that the running calibration stop.
///
/// Latches the stop the procedures poll between steps, and interrupts any drive
/// in flight — only the distance calibration drives, and stopping its leg is not
/// a step boundary.
pub fn request_stop() {
    STOP.request();
    drive::send_drive_interrupt(InterruptKind::Stop);
}

/// Re-arm the shared stop latch at the start of a run.
async fn arm_stop() {
    STOP.rearm().await;
}

/// Whether the operator has asked the running calibration to stop.
fn is_stop_requested() -> bool {
    STOP.is_requested()
}

/// Wait for `duration_ms`, returning `true` early if a stop is requested.
async fn wait_or_stop(duration_ms: u64) -> bool {
    STOP.wait_or(duration_ms).await
}
