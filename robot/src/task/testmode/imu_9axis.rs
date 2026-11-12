//! On-demand IMU test mode task (9-axis: accel + gyro + mag).
//!
//! Streams IMU telemetry until the operator stops it from the running screen.
//! The panel is the primary view: the task publishes a typed
//! [`touch_ui::ImuReadout`] into the IMU Test Telemetry snapshot at 10 Hz, and
//! the touch UI formats it into the running screen's rows — the raw and
//! calibrated magnetometer vectors, the calibration status and the `use_mag`
//! gate included. The periodic log is diagnostics only, so its reports are
//! `debug`.
//!
//! The screen still reads its phase from the activity state. ICM-20948
//! communicates over SPI.

use defmt::{debug, error};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use micromath::F32Ext;
use touch_ui::{
    CalibrationStatus as UiCalibrationStatus, Euler, ImuHealth, ImuReadout, ImuReadoutKind, MagDiagnostics, Vector3,
};

use crate::{
    system::state::{CalibrationStatus, calibration, imu_test},
    task::{
        procedure::Claim,
        sensors::imu::{
            DmpFusionMode, ImuReadings, Orientation, get_latest_readings, is_mag_available, set_dmp_fusion_mode,
            start_imu_readings, stop_imu_readings,
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

/// Lower bound of the driver's magnetometer-norm gate, in uT.
const MAG_GATE_MIN_UT: f32 = 20.0;

/// Upper bound of the driver's magnetometer-norm gate, in uT.
const MAG_GATE_MAX_UT: f32 = 200.0;

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

    // Both magnetometer facts are start-up state: whether the driver initialised
    // the magnetometer, and the stored calibration. They are read once so the
    // publish path takes no other lock.
    let mag_available = is_mag_available();
    let mag_calibration = mag_calibration_status().await;

    let mut tick: u32 = 0;
    let mut missing: u32 = 0;
    let mut received = false;

    while !claim.wait_or_stop(SAMPLE_INTERVAL_MS).await {
        let readings = get_latest_readings().await;

        if readings.orientation.is_none() && readings.calibrated_gyro.is_none() && readings.calibrated_mag.is_none() {
            missing = missing.saturating_add(1);
        } else {
            received = true;
            missing = 0;
        }

        tick = tick.wrapping_add(1);
        let diagnostics = if mag_available {
            match (readings.raw_accel, readings.raw_mag) {
                (Some(accel), Some(mag)) => Some(mag_diagnostics(accel, mag)),
                _ => None,
            }
        } else {
            None
        };
        if tick.is_multiple_of(PUBLISH_EVERY_TICKS) {
            imu_test::publish(build_readout(
                health(received, missing),
                missing,
                readings,
                mag_available,
                mag_calibration,
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

            if let (Some(a), Some(g), Some(m), Some(diagnostics)) =
                (readings.raw_accel, readings.raw_gyro, readings.raw_mag, diagnostics)
            {
                log_orientation(readings.orientation);
                log_mag_diagnostics(a, g, m, diagnostics, readings.orientation);
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

/// Read the stored magnetometer calibration status once for the whole run.
///
/// Only `CALIBRATION_STATE` is locked, and the guard is released before the task
/// takes any other lock, so the documented lock order is kept.
async fn mag_calibration_status() -> UiCalibrationStatus {
    let status = {
        let state = calibration::CALIBRATION_STATE.lock().await;
        state.mag_cal_status
    };
    map_mag_calibration(status)
}

/// Map the firmware's calibration status onto the UI's neutral one.
const fn map_mag_calibration(status: CalibrationStatus) -> UiCalibrationStatus {
    match status {
        CalibrationStatus::Loaded => UiCalibrationStatus::Loaded,
        CalibrationStatus::NotLoaded => UiCalibrationStatus::Unknown,
        CalibrationStatus::NotAvailable => UiCalibrationStatus::Missing,
    }
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
    mag_calibration: UiCalibrationStatus,
    diagnostics: Option<MagDiagnostics>,
) -> ImuReadout {
    let mut readout = ImuReadout::waiting(ImuReadoutKind::NineAxis)
        .with_health(health, missing)
        .with_mag_calibration(mag_calibration)
        .with_mag_available(mag_available);
    if let Some(orientation) = readings.orientation {
        readout = readout.with_orientation(to_euler(orientation));
    }
    if let Some(accel) = readings.raw_accel {
        readout = readout.with_raw_accel(to_vector(accel));
    }
    if let Some(gyro) = readings.raw_gyro {
        readout = readout.with_raw_gyro(to_vector(gyro));
    }
    if let Some(gyro) = readings.calibrated_gyro {
        readout = readout.with_calibrated_gyro(to_vector(gyro));
    }
    if !mag_available {
        return readout;
    }
    if let Some(raw_mag) = readings.raw_mag {
        readout = readout.with_raw_mag(to_vector(raw_mag));
    }
    if let Some(calibrated_mag) = readings.calibrated_mag {
        readout = readout.with_calibrated_mag(to_vector(calibrated_mag));
    }
    if let Some(diagnostics) = diagnostics {
        readout = readout.with_mag(diagnostics);
    }
    readout
}

/// Whether a magnetometer norm passes the driver's gate.
fn mag_gate_passes(norm: f32) -> bool {
    norm > MAG_GATE_MIN_UT && norm < MAG_GATE_MAX_UT
}

/// Build the magnetometer diagnostics from the raw accelerometer and raw
/// magnetometer vectors.
///
/// This is the one place the test task computes the derived magnetometer values:
/// the norm and gate, the eight axis-sign permutations for each plane as whole
/// degrees, the raw XY heading and the tilt-compensated heading from the
/// accelerometer's roll and pitch.
#[allow(clippy::similar_names, clippy::cast_precision_loss)]
fn mag_diagnostics(accel: nalgebra::Vector3<f32>, mag: nalgebra::Vector3<f32>) -> MagDiagnostics {
    let norm = mag.norm();

    let remap_xy = [
        whole_degrees(mag.y.atan2(mag.x)),
        whole_degrees(mag.x.atan2(mag.y)),
        whole_degrees(mag.y.atan2(-mag.x)),
        whole_degrees((-mag.y).atan2(mag.x)),
        whole_degrees((-mag.y).atan2(-mag.x)),
        whole_degrees(mag.x.atan2(-mag.y)),
        whole_degrees((-mag.x).atan2(mag.y)),
        whole_degrees((-mag.x).atan2(-mag.y)),
    ];
    let remap_xz = [
        whole_degrees(mag.z.atan2(mag.x)),
        whole_degrees(mag.x.atan2(mag.z)),
        whole_degrees(mag.z.atan2(-mag.x)),
        whole_degrees((-mag.z).atan2(mag.x)),
        whole_degrees((-mag.z).atan2(-mag.x)),
        whole_degrees(mag.x.atan2(-mag.z)),
        whole_degrees((-mag.x).atan2(mag.z)),
        whole_degrees((-mag.x).atan2(-mag.z)),
    ];
    let remap_yz = [
        whole_degrees(mag.z.atan2(mag.y)),
        whole_degrees(mag.y.atan2(mag.z)),
        whole_degrees(mag.z.atan2(-mag.y)),
        whole_degrees((-mag.z).atan2(mag.y)),
        whole_degrees((-mag.z).atan2(-mag.y)),
        whole_degrees(mag.y.atan2(-mag.z)),
        whole_degrees((-mag.y).atan2(mag.z)),
        whole_degrees((-mag.y).atan2(-mag.z)),
    ];

    let roll = accel.y.atan2(accel.z);
    let pitch = (-accel.x).atan2((accel.y * accel.y + accel.z * accel.z).sqrt());
    let mx2 = mag.x * pitch.cos() + mag.z * pitch.sin();
    let my2 = mag.x * roll.sin() * pitch.sin() + mag.y * roll.cos() - mag.z * roll.sin() * pitch.cos();

    MagDiagnostics {
        norm,
        use_mag: mag_gate_passes(norm),
        raw_heading: mag.y.atan2(mag.x).to_degrees(),
        tilt_heading: my2.atan2(mx2).to_degrees(),
        remap_xy,
        remap_xz,
        remap_yz,
    }
}

/// Convert a radian heading to whole degrees.
#[allow(clippy::cast_possible_truncation)]
const fn whole_degrees(radians: f32) -> i32 {
    radians.to_degrees() as i32
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
        "imu9 mag remap headings: xy={} yx={} -x,y={} x,-y={} -x,-y={} -y,x={} y,-x={} -y,-x={}",
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
        "imu9 mag remap headings xz-plane: xz={} zx={} -x,z={} x,-z={} -x,-z={} -z,x={} z,-x={} -z,-x={}",
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
        "imu9 mag remap headings yz-plane: yz={} zy={} -y,z={} y,-z={} -y,-z={} -z,y={} z,-y={} -z,-y={}",
        headings_yz[0],
        headings_yz[1],
        headings_yz[2],
        headings_yz[3],
        headings_yz[4],
        headings_yz[5],
        headings_yz[6],
        headings_yz[7]
    );

    if let Some(ori) = orientation {
        debug!(
            "imu9 heading: raw={=f32} tilt={=f32} dmp_yaw={=f32}",
            diagnostics.raw_heading, diagnostics.tilt_heading, ori.yaw
        );
    } else {
        debug!(
            "imu9 heading: raw={=f32} tilt={=f32} dmp=n/a",
            diagnostics.raw_heading, diagnostics.tilt_heading
        );
    }
}
