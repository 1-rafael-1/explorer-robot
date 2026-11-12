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
use touch_ui::ImuReadoutKind;

use super::imu_common::{LOG_EVERY_TICKS, PUBLISH_EVERY_TICKS, SAMPLE_INTERVAL_MS, base_readout, health};
use crate::{
    system::state::imu_test,
    task::{
        procedure::Claim,
        sensors::imu::{
            DmpFusionMode, Orientation, get_latest_readings, set_dmp_fusion_mode, start_imu_readings, stop_imu_readings,
        },
    },
};

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
        let readings = get_latest_readings().await;

        if readings.orientation.is_none() && readings.calibrated_gyro.is_none() {
            missing = missing.saturating_add(1);
        } else {
            received = true;
            missing = 0;
        }

        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(PUBLISH_EVERY_TICKS) {
            imu_test::publish(base_readout(
                ImuReadoutKind::SixAxis,
                health(received, missing),
                missing,
                readings,
            ))
            .await;
        }
        if tick.is_multiple_of(LOG_EVERY_TICKS) {
            debug!(
                "imu6: ori={} gyro={} missing={=u32}",
                readings.orientation.is_some(),
                readings.calibrated_gyro.is_some(),
                missing
            );
            log_orientation(readings.orientation);
            log_raw(readings.raw_accel, readings.raw_gyro);
        }
    }

    stop_imu_readings();
    imu_test::clear().await;
    claim.finish().await;
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
