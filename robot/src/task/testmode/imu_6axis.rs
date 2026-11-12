//! On-demand IMU test mode task (6-axis: accel + gyro).
//!
//! Streams IMU telemetry until the operator stops it from the running screen.
//! The panel is the primary view: the task publishes a typed
//! [`touch_ui::ImuReadout`] into the IMU Test Telemetry snapshot at 10 Hz, and
//! the touch UI formats it into the running screen's rows. The periodic log is
//! diagnostics only, so its reports are `debug`.
//!
//! The screen still reads its phase from the activity state. ICM-20948
//! communicates over SPI.

use defmt::{debug, error};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use touch_ui::{Euler, ImuHealth, ImuReadout, ImuReadoutKind, Vector3};

use crate::{
    system::state::imu_test,
    task::{
        procedure::Claim,
        sensors::imu::{
            DmpFusionMode, Orientation, get_latest_readings, set_dmp_fusion_mode, start_imu_readings, stop_imu_readings,
        },
    },
};

/// Sampling period, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 20;

/// Ticks between log reports (50 ticks ≈ 1 s).
const LOG_EVERY_TICKS: u32 = 50;

/// Ticks between telemetry publishes (5 ticks ≈ 10 Hz).
const PUBLISH_EVERY_TICKS: u32 = 5;

/// Consecutive missing samples that mean the IMU is not responding (100 ticks
/// ≈ 2 s).
const NOT_RESPONDING_TICKS: u32 = 100;

/// Spawn the IMU 6-axis test task via the controller.
///
/// The caller's claim travels with the task; a spawn that fails releases it so a
/// dead task does not hold the slot.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    imu6_test_task(claim).map_or_else(
        |_| {
            error!("imu6: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// IMU 6-axis test task: streams readings until stopped.
#[embassy_executor::task]
async fn imu6_test_task(claim: Claim) {
    imu_test::clear().await;
    start_imu_readings();
    Timer::after(Duration::from_millis(30)).await;
    set_dmp_fusion_mode(DmpFusionMode::Axis6);
    claim.phase("6-axis streaming", None).await;

    let mut tick: u32 = 0;
    let mut missing: u32 = 0;
    let mut received = false;

    while !claim.wait_or_stop(SAMPLE_INTERVAL_MS).await {
        let r = get_latest_readings().await;
        let orientation = r.orientation;
        let gyro = r.calibrated_gyro;
        let raw_accel = r.raw_accel;
        let raw_gyro = r.raw_gyro;

        if orientation.is_none() && gyro.is_none() {
            missing = missing.saturating_add(1);
        } else {
            received = true;
            missing = 0;
        }

        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(PUBLISH_EVERY_TICKS) {
            imu_test::publish(build_readout(
                health(received, missing),
                missing,
                orientation,
                raw_accel,
                raw_gyro,
                gyro,
            ))
            .await;
        }
        if tick.is_multiple_of(LOG_EVERY_TICKS) {
            debug!(
                "imu6: ori={} gyro={} missing={=u32}",
                orientation.is_some(),
                gyro.is_some(),
                missing
            );
            log_orientation(orientation);
            log_raw(raw_accel, raw_gyro);
        }
    }

    stop_imu_readings();
    imu_test::clear().await;
    claim.finish().await;
}

/// The health state for the current sample run.
const fn health(received: bool, missing: u32) -> ImuHealth {
    if missing >= NOT_RESPONDING_TICKS {
        ImuHealth::NotResponding
    } else if received {
        ImuHealth::Live
    } else {
        ImuHealth::Waiting
    }
}

/// Build the typed readout the touch UI formats.
fn build_readout(
    health: ImuHealth,
    missing: u32,
    orientation: Option<Orientation>,
    raw_accel: Option<nalgebra::Vector3<f32>>,
    raw_gyro: Option<nalgebra::Vector3<f32>>,
    calibrated_gyro: Option<nalgebra::Vector3<f32>>,
) -> ImuReadout {
    let mut readout = ImuReadout::waiting(ImuReadoutKind::SixAxis).with_health(health, missing);
    if let Some(orientation) = orientation {
        readout = readout.with_orientation(to_euler(orientation));
    }
    if let Some(accel) = raw_accel {
        readout = readout.with_raw_accel(to_vector(accel));
    }
    if let Some(gyro) = raw_gyro {
        readout = readout.with_raw_gyro(to_vector(gyro));
    }
    if let Some(gyro) = calibrated_gyro {
        readout = readout.with_calibrated_gyro(to_vector(gyro));
    }
    readout
}

/// Map the driver's fused orientation onto the UI's neutral Euler angles.
const fn to_euler(orientation: Orientation) -> Euler {
    Euler {
        yaw: orientation.yaw,
        pitch: orientation.pitch,
        roll: orientation.roll,
    }
}

/// Map a driver vector onto the UI's neutral three-component vector.
fn to_vector(vector: nalgebra::Vector3<f32>) -> Vector3 {
    Vector3 {
        x: vector.x,
        y: vector.y,
        z: vector.z,
    }
}

/// Log the fused orientation in Euler angles, when there is one.
fn log_orientation(orientation: Option<Orientation>) {
    if let Some(ori) = orientation {
        debug!(
            "imu6 euler: yaw={=f32} pitch={=f32} roll={=f32}",
            ori.yaw, ori.pitch, ori.roll
        );
    }
}

/// Log the raw accelerometer and gyroscope vectors, or which one is missing.
fn log_raw(accel: Option<nalgebra::Vector3<f32>>, gyro: Option<nalgebra::Vector3<f32>>) {
    if let (Some(a), Some(g)) = (accel, gyro) {
        debug!(
            "imu6 raw: a=({=f32},{=f32},{=f32}) g=({=f32},{=f32},{=f32})",
            a.x, a.y, a.z, g.x, g.y, g.z
        );
    } else {
        debug!("imu6 raw: accel={} gyro={}", accel.is_some(), gyro.is_some());
    }
}
