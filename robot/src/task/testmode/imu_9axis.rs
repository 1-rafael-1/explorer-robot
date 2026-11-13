//! On-demand IMU test mode task (9-axis: accel + gyro + mag).
//!
//! Streams IMU telemetry until the operator stops it from the running screen.
//! The panel is the primary view: the task publishes a typed
//! [`touch_ui::ImuReadout`] into the IMU Test Telemetry snapshot at 10 Hz, and
//! the touch UI formats it into the running screen's rows — the raw and
//! DMP-calibrated magnetometer vectors, the DMP calibration status and the
//! `use_mag` gate included. The periodic log is diagnostics only, so its reports
//! are `debug`.
//!
//! The screen still reads its phase from the activity state. ICM-20948
//! communicates over SPI.

use defmt::{debug, error};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use micromath::F32Ext;
use touch_ui::{CalibrationStatus as UiCalibrationStatus, ImuHealth, ImuReadout, ImuReadoutKind, MagDiagnostics};

use super::imu_common::{LOG_EVERY_TICKS, PUBLISH_EVERY_TICKS, SAMPLE_INTERVAL_MS, base_readout, health, to_vector};
use crate::{
    system::state::{calibration, imu_test},
    task::{
        procedure::Claim,
        sensors::imu::{
            DmpFusionMode, ImuReadings, Orientation, READING_STALE_MS, get_latest_readings, is_mag_available,
            mag_gate_passes, set_dmp_fusion_mode, start_imu_readings, stop_imu_readings,
        },
    },
};

/// Spawn the IMU 9-axis test task via the controller.
///
/// The caller's claim travels with the task; a spawn that fails releases it so a
/// dead task does not hold the slot.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    imu_test_task(claim).map_or_else(
        |_| {
            error!("imu9: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// IMU 9-axis test task: streams readings until stopped.
#[embassy_executor::task]
async fn imu_test_task(claim: Claim) {
    imu_test::clear().await;
    start_imu_readings();
    Timer::after(Duration::from_millis(30)).await;
    set_dmp_fusion_mode(DmpFusionMode::Axis9);
    claim.phase("9-axis streaming", None).await;

    // The stored DMP calibration is start-up state and is read once so the
    // publish path takes no other lock. Magnetometer availability is refreshed
    // every tick instead: boot can open this test before the IMU task has
    // initialised the magnetometer, and a flag captured then would hide every
    // later reading for the whole run.
    let dmp_calibration = dmp_calibration_status().await;

    let mut tick: u32 = 0;
    let mut missing: u32 = 0;
    let mut received = false;
    let mut diagnostics: Option<MagDiagnostics> = None;

    while !claim.wait_or_stop(SAMPLE_INTERVAL_MS).await {
        // Drop values the driver stopped updating, so a stalled IMU reads as
        // missing instead of replaying its last sample as live data.
        let readings = get_latest_readings().await.fresh(READING_STALE_MS);
        let mag_available = is_mag_available();

        if readings.orientation.is_none() && readings.calibrated_gyro.is_none() && readings.calibrated_mag.is_none() {
            missing = missing.saturating_add(1);
        } else {
            received = true;
            missing = 0;
        }

        tick = tick.wrapping_add(1);
        if tick.is_multiple_of(PUBLISH_EVERY_TICKS) {
            // The magnetometer diagnostics are a UI concern at 10 Hz; computing
            // them at the 50 Hz sample rate wasted work, and the 1 s log tick is
            // also a publish tick, so it reuses this value.
            diagnostics = if mag_available {
                readings
                    .raw_mag
                    .map(|mag| mag_diagnostics(readings.raw_accel.map(|accel| accel.value), mag.value))
            } else {
                None
            };
            imu_test::publish(build_readout(
                health(received, missing),
                missing,
                readings,
                mag_available,
                dmp_calibration,
                diagnostics,
            ))
            .await;
        }
        if tick.is_multiple_of(LOG_EVERY_TICKS) {
            debug!(
                "imu9: ori={} gyro={} mag={} missing={=u32}",
                readings.orientation.is_some(),
                readings.calibrated_gyro.is_some(),
                readings.calibrated_mag.is_some(),
                missing
            );

            if let (Some(a), Some(g), Some(m), Some(diagnostics)) = (
                readings.raw_accel.map(|r| r.value),
                readings.raw_gyro.map(|r| r.value),
                readings.raw_mag.map(|r| r.value),
                diagnostics,
            ) {
                log_orientation(readings.orientation.map(|reading| reading.value));
                log_mag_diagnostics(a, g, m, diagnostics, readings.orientation.map(|reading| reading.value));
            } else {
                debug!(
                    "imu9 raw: accel={} gyro={} mag={}",
                    readings.raw_accel.is_some(),
                    readings.raw_gyro.is_some(),
                    readings.raw_mag.is_some()
                );
                debug!("imu9 mag gate: |m|=n/a use_mag=false");
            }
        }
    }

    stop_imu_readings();
    imu_test::clear().await;
    claim.finish().await;
}

/// Read the stored DMP calibration status once for the whole run.
///
/// Only `CALIBRATION_STATE` is locked, and the guard is released before the task
/// takes any other lock, so the documented lock order is kept.
async fn dmp_calibration_status() -> UiCalibrationStatus {
    let status = {
        let state = calibration::CALIBRATION_STATE.lock().await;
        state.dmp_cal_status
    };
    calibration::ui_status(status)
}

/// Build the typed readout the touch UI formats.
///
/// An unavailable magnetometer publishes no magnetometer vectors and no gate
/// verdict, so the panel shows `--` and the six-axis fallback line instead of
/// values the driver will never produce.
fn build_readout(
    health: ImuHealth,
    missing: u32,
    readings: ImuReadings,
    mag_available: bool,
    dmp_calibration: UiCalibrationStatus,
    diagnostics: Option<MagDiagnostics>,
) -> ImuReadout {
    let mut readout = base_readout(ImuReadoutKind::NineAxis, health, missing, readings)
        .with_dmp_calibration(dmp_calibration)
        .with_mag_available(mag_available);
    if !mag_available {
        return readout;
    }
    if let Some(raw_mag) = readings.raw_mag {
        readout = readout.with_raw_mag(to_vector(raw_mag.value));
    }
    if let Some(calibrated_mag) = readings.calibrated_mag {
        readout = readout.with_calibrated_mag(to_vector(calibrated_mag.value));
    }
    if let Some(diagnostics) = diagnostics {
        readout = readout.with_mag(diagnostics);
    }
    readout
}

/// Build the magnetometer diagnostics from the raw magnetometer, and the raw
/// accelerometer when it is available for tilt compensation.
///
/// This is the one place the test task computes the derived magnetometer values:
/// the norm and gate, the eight axis-sign permutations for each plane, the raw
/// XY heading, and — only when an accelerometer sample exists — the
/// tilt-compensated heading from the accelerometer's roll and pitch.
#[allow(clippy::similar_names)]
fn mag_diagnostics(accel: Option<nalgebra::Vector3<f32>>, mag: nalgebra::Vector3<f32>) -> MagDiagnostics {
    let norm = mag.norm();

    let remap_xy = [
        mag.y.atan2(mag.x).to_degrees(),
        mag.x.atan2(mag.y).to_degrees(),
        mag.y.atan2(-mag.x).to_degrees(),
        (-mag.y).atan2(mag.x).to_degrees(),
        (-mag.y).atan2(-mag.x).to_degrees(),
        mag.x.atan2(-mag.y).to_degrees(),
        (-mag.x).atan2(mag.y).to_degrees(),
        (-mag.x).atan2(-mag.y).to_degrees(),
    ];
    let remap_xz = [
        mag.z.atan2(mag.x).to_degrees(),
        mag.x.atan2(mag.z).to_degrees(),
        mag.z.atan2(-mag.x).to_degrees(),
        (-mag.z).atan2(mag.x).to_degrees(),
        (-mag.z).atan2(-mag.x).to_degrees(),
        mag.x.atan2(-mag.z).to_degrees(),
        (-mag.x).atan2(mag.z).to_degrees(),
        (-mag.x).atan2(-mag.z).to_degrees(),
    ];
    let remap_yz = [
        mag.z.atan2(mag.y).to_degrees(),
        mag.y.atan2(mag.z).to_degrees(),
        mag.z.atan2(-mag.y).to_degrees(),
        (-mag.z).atan2(mag.y).to_degrees(),
        (-mag.z).atan2(-mag.y).to_degrees(),
        mag.y.atan2(-mag.z).to_degrees(),
        (-mag.y).atan2(mag.z).to_degrees(),
        (-mag.y).atan2(-mag.z).to_degrees(),
    ];

    let tilt_heading = accel.map(|accel| {
        let roll = accel.y.atan2(accel.z);
        let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
        let mx2 = mag.x * pitch.cos() + mag.z * pitch.sin();
        let my2 = mag.x * roll.sin() * pitch.sin() + mag.y * roll.cos() - mag.z * roll.sin() * pitch.cos();
        my2.atan2(mx2).to_degrees()
    });

    MagDiagnostics {
        norm,
        use_mag: mag_gate_passes(norm),
        raw_heading: mag.y.atan2(mag.x).to_degrees(),
        tilt_heading,
        remap_xy,
        remap_xz,
        remap_yz,
    }
}

/// Log the fused orientation in Euler angles, when there is one.
fn log_orientation(orientation: Option<Orientation>) {
    if let Some(ori) = orientation {
        debug!(
            "imu9 euler: yaw={=f32} pitch={=f32} roll={=f32}",
            ori.yaw, ori.pitch, ori.roll
        );
    }
}

/// Log raw IMU vectors and the precomputed magnetometer heading diagnostics.
#[allow(clippy::similar_names)]
fn log_mag_diagnostics(
    accel: nalgebra::Vector3<f32>,
    gyro: nalgebra::Vector3<f32>,
    mag: nalgebra::Vector3<f32>,
    diagnostics: MagDiagnostics,
    orientation: Option<Orientation>,
) {
    debug!(
        "imu9 raw: a=({=f32},{=f32},{=f32}) g=({=f32},{=f32},{=f32}) m=({=f32},{=f32},{=f32})",
        accel.x, accel.y, accel.z, gyro.x, gyro.y, gyro.z, mag.x, mag.y, mag.z
    );
    debug!(
        "imu9 mag gate: |m|={=f32} use_mag={}",
        diagnostics.norm, diagnostics.use_mag
    );

    let headings_xy = diagnostics.remap_xy;
    debug!(
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

    let headings_xz = diagnostics.remap_xz;
    debug!(
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

    let headings_yz = diagnostics.remap_yz;
    debug!(
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

    match (diagnostics.tilt_heading, orientation) {
        (Some(tilt), Some(ori)) => debug!(
            "imu9 heading: raw={=f32} tilt={=f32} dmp_yaw={=f32}",
            diagnostics.raw_heading, tilt, ori.yaw
        ),
        (Some(tilt), None) => debug!(
            "imu9 heading: raw={=f32} tilt={=f32} dmp=n/a",
            diagnostics.raw_heading, tilt
        ),
        (None, Some(ori)) => debug!(
            "imu9 heading: raw={=f32} tilt=n/a dmp_yaw={=f32}",
            diagnostics.raw_heading, ori.yaw
        ),
        (None, None) => debug!("imu9 heading: raw={=f32} tilt=n/a dmp=n/a", diagnostics.raw_heading),
    }
}
