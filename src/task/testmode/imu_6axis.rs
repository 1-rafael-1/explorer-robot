//! On-demand IMU test mode task (6-axis: accel + gyro).
//!
//! Streams IMU telemetry until the operator stops it from the running screen.
//! The screen reads its phase from the activity state; the sampled orientation
//! and raw vectors go to the log.
//!
//! ICM-20948 communicates over SPI.

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use super::{arm_stop, release_testmode, wait_or_stop};
use crate::{
    system::state::activity,
    task::sensors::imu::{
        DmpFusionMode, Orientation, get_latest_readings, set_dmp_fusion_mode, start_imu_readings, stop_imu_readings,
    },
};

/// Sampling period, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 20;

/// Ticks between log reports (50 ticks ≈ 1 s).
const LOG_EVERY_TICKS: u32 = 50;

/// Spawn the IMU 6-axis test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(imu6_test_task().unwrap());
}

/// IMU 6-axis test task: streams readings until stopped.
#[embassy_executor::task]
async fn imu6_test_task() {
    arm_stop().await;

    start_imu_readings();
    Timer::after(Duration::from_millis(30)).await;
    set_dmp_fusion_mode(DmpFusionMode::Axis6);
    activity::set_running("6-axis streaming", None).await;

    let mut tick: u32 = 0;
    let mut missing: u32 = 0;

    while !wait_or_stop(SAMPLE_INTERVAL_MS).await {
        let r = get_latest_readings().await;
        let orientation = r.orientation;
        let gyro = r.calibrated_gyro;
        let raw_accel = r.raw_accel;
        let raw_gyro = r.raw_gyro;

        if orientation.is_none() && gyro.is_none() {
            missing = missing.saturating_add(1);
        } else {
            missing = 0;
        }

        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(LOG_EVERY_TICKS) {
            info!(
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
    release_testmode();
}

/// Log the fused orientation in Euler angles, when there is one.
fn log_orientation(orientation: Option<Orientation>) {
    if let Some(ori) = orientation {
        info!(
            "imu6 euler: yaw={=f32} pitch={=f32} roll={=f32}",
            ori.yaw, ori.pitch, ori.roll
        );
    }
}

/// Log the raw accelerometer and gyroscope vectors, or which one is missing.
fn log_raw(accel: Option<nalgebra::Vector3<f32>>, gyro: Option<nalgebra::Vector3<f32>>) {
    if let (Some(a), Some(g)) = (accel, gyro) {
        info!(
            "imu6 raw: a=({=f32},{=f32},{=f32}) g=({=f32},{=f32},{=f32})",
            a.x, a.y, a.z, g.x, g.y, g.z
        );
    } else {
        info!("imu6 raw: accel={} gyro={}", accel.is_some(), gyro.is_some());
    }
}
