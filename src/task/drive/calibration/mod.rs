//! Calibration procedures for the drive system.
//!
//! Holds the motor (track balance), magnetometer and distance calibration
//! routines. Each procedure publishes its phase, percent and result to
//! [`crate::system::state::activity`]; nothing here formats display text. The
//! shared lifecycle — arming the stop, publishing the activity and phases, the
//! terminal outcome and the completion event — lives in
//! [`crate::task::procedure`]; each procedure supplies its own body.
//!
//! # Stopping
//!
//! Every procedure is interruptible from the panel. [`request_stop`] latches the
//! calibration family's stop — which each procedure polls at its wait points —
//! and interrupts any drive in flight, so the distance calibration's fixed-distance
//! drive stops mid-leg and the motor procedure's track stops mid-measurement. The
//! test family keeps its own latch, so a stop aimed at a test cannot reach a
//! calibration and vice versa.

pub mod distance;
pub mod imu;
pub mod motor;

use touch_ui::Procedure;

use crate::{
    system::event::Events,
    task::procedure::{Completion, Lifecycle, StopLatch},
};

/// The stop request every running calibration procedure polls.
static CALIBRATION_STOP: StopLatch = StopLatch::new();

// Re-export calibration functions.
pub use imu::run_imu_calibration;
pub use motor::run_motor_calibration;

/// Request that the running calibration stop.
///
/// Latches the calibration family's stop the procedures poll between steps, and
/// interrupts any drive in flight — only the distance calibration drives, and
/// stopping its leg is not a step boundary.
pub fn request_stop() {
    CALIBRATION_STOP.request();
}

/// The lifecycle the calibration family runs `procedure` under.
///
/// The calibrations have no single-active slot: they run inline in the drive
/// dispatch loop, which already serialises them, so the slot is `None`. The motor
/// and magnetometer calibrations raise `CalibrationCompleted` on both outcomes;
/// distance calibration, whose flow crosses the value-entry screen, raises
/// nothing.
const fn calibration_lifecycle(procedure: Procedure) -> Lifecycle {
    Lifecycle::new(
        procedure,
        &CALIBRATION_STOP,
        None,
        match procedure {
            Procedure::DistanceCalibration => Completion::Silent,
            _ => Completion::OnBoth(calibration_completed),
        },
    )
}

/// The completion event the motor and magnetometer calibrations raise.
const fn calibration_completed() -> Events {
    Events::CalibrationCompleted
}
