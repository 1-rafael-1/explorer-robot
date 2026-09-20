//! On-demand IMU test mode task (9-axis: accel + gyro + mag).
//!
//! Streams IMU telemetry until the operator stops it from the running screen.
//! The screen reads its phase from the activity state; the sampled orientation,
//! raw vectors, and magnetometer remap diagnostics go to the log.
//!
//! ICM-20948 communicates over SPI.

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use micromath::F32Ext;
use touch_ui::Procedure;

use super::test_lifecycle;
use crate::task::{
    procedure::Lifecycle,
    sensors::imu::{
        DmpFusionMode, Orientation, get_latest_readings, set_dmp_fusion_mode, start_imu_readings, stop_imu_readings,
    },
};

/// Sampling period, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 20;

/// Ticks between log reports (50 ticks ≈ 1 s).
const LOG_EVERY_TICKS: u32 = 50;

/// The IMU 9-axis test's lifecycle: the test family's stop latch and slot, with
/// no completion event — the test streams until the operator stops it.
const LIFECYCLE: Lifecycle = test_lifecycle(Procedure::Imu9Axis);

/// Spawn the IMU 9-axis test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(imu_test_task().unwrap());
}

/// IMU 9-axis test task: streams readings until stopped.
#[embassy_executor::task]
async fn imu_test_task() {
    start_imu_readings();
    Timer::after(Duration::from_millis(30)).await;
    set_dmp_fusion_mode(DmpFusionMode::Axis9);
    LIFECYCLE.phase("9-axis streaming", None).await;

    let mut tick: u32 = 0;
    let mut missing: u32 = 0;

    while !LIFECYCLE.wait_or_stop(SAMPLE_INTERVAL_MS).await {
        let r = get_latest_readings().await;
        let orientation = r.orientation;
        let gyro = r.calibrated_gyro;
        let mag = r.calibrated_mag;
        let raw_accel = r.raw_accel;
        let raw_gyro = r.raw_gyro;
        let raw_mag = r.raw_mag;

        if orientation.is_none() && gyro.is_none() && mag.is_none() {
            missing = missing.saturating_add(1);
        } else {
            missing = 0;
        }

        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(LOG_EVERY_TICKS) {
            info!(
                "imu9: ori={} gyro={} mag={} missing={=u32}",
                orientation.is_some(),
                gyro.is_some(),
                mag.is_some(),
                missing
            );

            if let (Some(a), Some(g), Some(m)) = (raw_accel, raw_gyro, raw_mag) {
                log_orientation(orientation);
                log_mag_diagnostics(a, g, m, orientation);
            } else {
                info!(
                    "imu9 raw: accel={} gyro={} mag={}",
                    raw_accel.is_some(),
                    raw_gyro.is_some(),
                    raw_mag.is_some()
                );
                info!("imu9 mag gate: |m|=n/a use_mag=false");
            }
        }
    }

    stop_imu_readings();
    LIFECYCLE.release();
}

/// Log the fused orientation in Euler angles, when there is one.
fn log_orientation(orientation: Option<Orientation>) {
    if let Some(ori) = orientation {
        info!(
            "imu9 euler: yaw={=f32} pitch={=f32} roll={=f32}",
            ori.yaw, ori.pitch, ori.roll
        );
    }
}

/// Log raw IMU vectors and heading remap diagnostics.
#[allow(clippy::similar_names)]
fn log_mag_diagnostics(
    accel: nalgebra::Vector3<f32>,
    gyro: nalgebra::Vector3<f32>,
    mag: nalgebra::Vector3<f32>,
    orientation: Option<Orientation>,
) {
    info!(
        "imu9 raw: a=({=f32},{=f32},{=f32}) g=({=f32},{=f32},{=f32}) m=({=f32},{=f32},{=f32})",
        accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z
    );
    let mag_norm = mag.norm();
    let use_mag = mag_norm > 20.0 && mag_norm < 200.0;
    info!("imu9 mag gate: |m|={=f32} use_mag={}", mag_norm, use_mag);

    let headings_xy = [
        mag.y.atan2(mag.x).to_degrees(),
        mag.x.atan2(mag.y).to_degrees(),
        mag.y.atan2(-mag.x).to_degrees(),
        (-mag.y).atan2(mag.x).to_degrees(),
        (-mag.y).atan2(-mag.x).to_degrees(),
        mag.x.atan2(-mag.y).to_degrees(),
        (-mag.x).atan2(mag.y).to_degrees(),
        (-mag.x).atan2(-mag.y).to_degrees(),
    ];

    info!(
        "imu9 mag remap headings: xy={=f32} yx={=f32} -x,y={=f32} x,-y={=f32} -x,-y={=f32} -y,x={=f32} y,-x={=f32} -y,-x={=f32}",
        headings_xy[0],
        headings_xy[1],
        headings_xy[2],
        headings_xy[3],
        headings_xy[4],
        headings_xy[5],
        headings_xy[6],
        headings_xy[7]
    );

    let headings_xz = [
        mag.z.atan2(mag.x).to_degrees(),
        mag.x.atan2(mag.z).to_degrees(),
        mag.z.atan2(-mag.x).to_degrees(),
        (-mag.z).atan2(mag.x).to_degrees(),
        (-mag.z).atan2(-mag.x).to_degrees(),
        mag.x.atan2(-mag.z).to_degrees(),
        (-mag.x).atan2(mag.z).to_degrees(),
        (-mag.x).atan2(-mag.z).to_degrees(),
    ];

    info!(
        "imu9 mag remap headings xz-plane: xz={=f32} zx={=f32} -x,z={=f32} x,-z={=f32} -x,-z={=f32} -z,x={=f32} z,-x={=f32} -z,-x={=f32}",
        headings_xz[0],
        headings_xz[1],
        headings_xz[2],
        headings_xz[3],
        headings_xz[4],
        headings_xz[5],
        headings_xz[6],
        headings_xz[7]
    );

    let headings_yz = [
        mag.z.atan2(mag.y).to_degrees(),
        mag.y.atan2(mag.z).to_degrees(),
        mag.z.atan2(-mag.y).to_degrees(),
        (-mag.z).atan2(mag.y).to_degrees(),
        (-mag.z).atan2(-mag.y).to_degrees(),
        mag.y.atan2(-mag.z).to_degrees(),
        (-mag.y).atan2(mag.z).to_degrees(),
        (-mag.y).atan2(-mag.z).to_degrees(),
    ];

    info!(
        "imu9 mag remap headings yz-plane: yz={=f32} zy={=f32} -y,z={=f32} y,-z={=f32} -y,-z={=f32} -z,y={=f32} z,-y={=f32} -z,-y={=f32}",
        headings_yz[0],
        headings_yz[1],
        headings_yz[2],
        headings_yz[3],
        headings_yz[4],
        headings_yz[5],
        headings_yz[6],
        headings_yz[7]
    );

    let raw_heading = headings_xy[0];

    let roll = accel.y.atan2(accel.z);
    let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
    let mx2 = mag.x * pitch.cos() + mag.z * pitch.sin();
    let my2 = mag.x * roll.sin() * pitch.sin() + mag.y * roll.cos() - mag.z * roll.sin() * pitch.cos();
    let tilt_heading = my2.atan2(mx2).to_degrees();

    if let Some(ori) = orientation {
        info!(
            "imu9 heading: raw={=f32} tilt={=f32} dmp_yaw={=f32}",
            raw_heading, tilt_heading, ori.yaw
        );
    } else {
        info!(
            "imu9 heading: raw={=f32} tilt={=f32} dmp=n/a",
            raw_heading, tilt_heading
        );
    }
}
