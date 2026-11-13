//! Plumbing shared by the six-axis and nine-axis IMU test tasks.
//!
//! Both tests sample at the same cadence, map the driver's latest readings into
//! the neutral [`ImuReadout`], and derive the same health state. Only the
//! nine-axis test adds the magnetometer rows, calibration and diagnostics.

use touch_ui::{Euler, ImuHealth, ImuReadout, ImuReadoutKind, Vector3};

use crate::task::sensors::imu::{ImuReadings, Orientation};

/// Sampling period, in milliseconds.
pub(super) const SAMPLE_INTERVAL_MS: u64 = 20;

/// Ticks between log reports (50 ticks ≈ 1 s).
pub(super) const LOG_EVERY_TICKS: u32 = 50;

/// Ticks between telemetry publishes (5 ticks ≈ 10 Hz).
pub(super) const PUBLISH_EVERY_TICKS: u32 = 5;

/// Consecutive missing samples that mean the IMU is not responding (100 ticks
/// ≈ 2 s).
pub(super) const NOT_RESPONDING_TICKS: u32 = 100;

/// The health state for the current sample run.
pub(super) const fn health(received: bool, missing: u32) -> ImuHealth {
    if missing >= NOT_RESPONDING_TICKS {
        ImuHealth::NotResponding
    } else if received {
        ImuHealth::Live
    } else {
        ImuHealth::Waiting
    }
}

/// Map the driver's fused orientation onto the UI's neutral Euler angles.
pub(super) const fn to_euler(orientation: Orientation) -> Euler {
    Euler {
        yaw: orientation.yaw,
        pitch: orientation.pitch,
        roll: orientation.roll,
    }
}

/// Map a driver vector onto the UI's neutral three-component vector.
pub(super) fn to_vector(vector: nalgebra::Vector3<f32>) -> Vector3 {
    Vector3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}

/// Build the readout rows both IMU tests share: health, orientation, the raw
/// accelerometer, the raw gyroscope and the calibrated gyroscope.
pub(super) fn base_readout(kind: ImuReadoutKind, health: ImuHealth, missing: u32, readings: ImuReadings) -> ImuReadout {
    let mut readout = ImuReadout::waiting(kind).with_health(health, missing);
    if let Some(reading) = readings.orientation {
        readout = readout.with_orientation(to_euler(reading.value));
    }
    if let Some(reading) = readings.raw_accel {
        readout = readout.with_raw_accel(to_vector(reading.value));
    }
    if let Some(reading) = readings.raw_gyro {
        readout = readout.with_raw_gyro(to_vector(reading.value));
    }
    if let Some(reading) = readings.calibrated_gyro {
        readout = readout.with_calibrated_gyro(to_vector(reading.value));
    }
    readout
}
