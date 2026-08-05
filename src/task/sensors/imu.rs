//! IMU (Inertial Measurement Unit) reading functionality using the ICM-20948 DMP.
//!
//! This module drives the ICM-20948's on-chip Digital Motion Processor (DMP) to
//! produce quaternion-derived orientation at a configurable output rate.
//! DMP packets are polled from the FIFO on a fixed timer — no INT pin is required.
//!
//! # Architecture
//!
//! The module operates in two states:
//! 1. Standby — not reading
//! 2. Active — DMP FIFO is polled every `POLL_INTERVAL`; each packet that contains
//!    a quaternion updates shared state and forwards the measurement to the
//!    drive task via `try_send_imu_measurement`.
//!
//! # Fusion modes
//!
//! - `Axis6`: 6-axis DMP fusion (gyro + accel). Yaw is relative and drifts, but is
//!   magnetically robust and well-suited to short precise turns. This is the default.
//! - `Axis9`: 9-axis DMP fusion (gyro + accel + mag). Yaw is stabilized to magnetic
//!   north. Falls back to `Axis6` automatically if the magnetometer could not be
//!   initialized at start-up.
//!
//! # Mode switching
//!
//! Runtime mode switches signal the task via `set_dmp_fusion_mode`. The running DMP
//! is stopped (`dmp_enable(false)`), reconfigured, and restarted without reloading
//! firmware. The magnetometer is always initialized at start-up so that switching to
//! `Axis9` later is possible without a full re-init.
//!
//! # Calibration
//!
//! Magnetometer hard/soft-iron and motor-interference correction are applied in
//! software; the corrected result is exposed via `ImuReadings::calibrated_mag`
//! (raw readings are in `raw_mag`). The DMP's
//! own internal calibration engines handle gyroscope and accelerometer bias correction
//! automatically — no host-injected bias values are needed for those axes.
//!
//! # Orientation reference frame
//!
//! Euler angles are derived directly from the DMP quaternion. In `Axis9` mode yaw is
//! an absolute compass heading; in `Axis6` mode yaw is relative to start-up heading.
//!
//! # Note on telemetry
//!
//! High-frequency diagnostics are gated behind the `telemetry_logs` feature to keep
//! production builds free of formatting overhead when no log consumer is attached.
//!
//! # Architecture
//!
//! - Dedicated SPI bus for the ICM-20948 via `SpiDevice`.
//! - `inertial_measurement_read` takes the bus `Mutex` + CS `Output` pin.
//! - `MagCalibration` is defined locally (`flash_storage` integration TBD).
//! - Chip initialisation retries on transient failures.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{info, warn};
use embassy_embedded_hal::shared_bus::asynch::spi::SpiDevice as SpiDev;
use embassy_futures::select::{Either, select};
use embassy_rp::{
    gpio::Output,
    spi::{Async as SpiAsync, Spi},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex, signal::Signal};
use embassy_time::{Delay, Duration, Instant, Timer};
use icm20948::{
    Icm20948Driver, SpiInterface,
    dmp::DmpConfig,
    sensors::{GyroConfig, GyroDlpf, GyroFullScale},
};
use nalgebra::Vector3;

use crate::{system::state::motion, task::drive};

// ── Type aliases ─────────────────────────────────────────────────────────────

/// SPI peripheral type for the dedicated ICM-20948 bus.
type SpiPeripheral = Spi<'static, embassy_rp::peripherals::SPI0, SpiAsync>;

/// CS pin type.
type CsOutput = Output<'static>;

/// Mutex-protected SPI bus (dedicated, single-user — mutex is never contended).
type SpiMutex = Mutex<CriticalSectionRawMutex, SpiPeripheral>;

/// `SpiDevice` wrapper for the dedicated bus + CS pin.
type SpiDeviceType<'a> = SpiDev<'a, CriticalSectionRawMutex, SpiPeripheral, CsOutput>;

/// ICM-20948 driver instance using a dedicated async SPI bus (no shared-bus contention).
type ImuSensor<'a> = Icm20948Driver<SpiInterface<SpiDeviceType<'a>>>;

// ── Timing constants ──────────────────────────────────────────────────────────

/// DMP output rate in Hz. The DMP firmware emits packets at this rate.
const DMP_SAMPLE_RATE_HZ: u16 = 100;

/// FIFO poll interval. Polling faster than the DMP rate ensures packets are
/// drained promptly; `10 ms` (100 Hz) matches `DMP_SAMPLE_RATE_HZ`.
const POLL_INTERVAL: Duration = Duration::from_millis(10);

/// Set to `true` after the first successful DMP FIFO sample, cleared when the
/// IMU is stopped. The drive dispatch uses this to gate movement commands and
/// trigger DMP filter stabilisation on each fresh start.
pub static IMU_READY: AtomicBool = AtomicBool::new(false);

/// Initial delay after power-up before starting IMU initialisation.
const IMU_BOOT_DELAY_MS: u64 = 200;

/// Delay between consecutive initialisation attempts.
const IMU_INIT_RETRY_DELAY_MS: u64 = 50;

/// Maximum number of initialisation attempts before the task halts permanently.
const IMU_INIT_MAX_ATTEMPTS: u32 = 60; // ≈ 3 s total

/// Maximum consecutive FIFO-read failures before the IMU task terminates.
const MAX_FIFO_FAILURES: u32 = 10;

// ── Sensor conversion constants ───────────────────────────────────────────────

/// Accelerometer LSB → g for the DMP's ±4 g internal full-scale (8192 LSB/g).
const ACCEL_SCALE_G: f32 = 1.0 / 8192.0;

/// Gyroscope LSB → deg/s for ±2000 dps full-scale (16.384 LSB/dps).
/// The icm20948-rs DMP initialisation explicitly programs `GYRO_FS_SEL = 0b11`
/// (±2000 dps) in its DMP-enable sequence (see `device.rs`, `dmp_enable`).
/// Exact conversion: 32768 LSB / 2000 dps = 16.384 LSB/dps.
const GYRO_SCALE_DPS: f32 = 1.0 / 16.384;

/// AK09916 magnetometer sensitivity: 0.15 µT per LSB.
const MAG_SCALE_UT: f32 = 0.15;

// ── Telemetry ─────────────────────────────────────────────────────────────────

/// Rate-limiting interval for IMU loop diagnostics when `telemetry_logs` is on.
#[cfg(feature = "telemetry_logs")]
const IMU_LOOP_DIAG_LOG_INTERVAL_MS: u32 = 500;

// ── Public types ──────────────────────────────────────────────────────────────

/// Complete IMU measurement data raised as an event.
#[derive(Debug, Clone, Copy)]
pub struct ImuMeasurement {
    /// Sensor orientation derived from the DMP quaternion.
    pub orientation: Orientation,
    /// Timestamp of this measurement in milliseconds since boot.
    pub timestamp_ms: u64,
}

/// 3D orientation expressed as Euler angles derived from the DMP quaternion.
///
/// In `Axis9` mode `yaw` is an absolute compass heading.
/// In `Axis6` mode `yaw` is relative to the heading at start-up and drifts.
#[derive(Debug, Clone, Copy)]
pub struct Orientation {
    /// Heading / compass direction in degrees.
    pub yaw: f32,
    /// Forward / backward tilt relative to gravity in degrees.
    pub pitch: f32,
    /// Left / right tilt relative to gravity in degrees.
    pub roll: f32,
}

/// DMP fusion-mode selection.
#[derive(Debug, Clone, Copy, Eq, PartialEq, defmt::Format)]
pub enum DmpFusionMode {
    /// 6-axis fusion (gyro + accel). Yaw is relative; no magnetometer dependency.
    Axis6,
    /// 9-axis fusion (gyro + accel + mag). Yaw is absolute; degrades gracefully to
    /// `Axis6` if the magnetometer was unavailable at start-up.
    Axis9,
}

/// System-wide default fusion mode used at start-up.
pub const DEFAULT_FUSION_MODE: DmpFusionMode = DmpFusionMode::Axis6;

// ── Magnetometer calibration data (local; flash_storage integration TBD) ──────

/// Host-side magnetometer calibration data (hard/soft-iron + motor interference).
///
/// Defined locally until flash storage integration is completed.
#[derive(Debug, Clone, Copy)]
pub struct MagCalibration {
    /// Hard-iron bias correction (µT).
    pub x_bias: f32,
    /// Hard-iron bias correction (µT).
    pub y_bias: f32,
    /// Hard-iron bias correction (µT).
    pub z_bias: f32,
    /// Soft-iron scale correction.
    pub x_scale: f32,
    /// Soft-iron scale correction.
    pub y_scale: f32,
    /// Soft-iron scale correction.
    pub z_scale: f32,
    /// Motor interference at 50% speed — index [0]=both, [1]=left, [2]=right (µT).
    pub x_interference_50: [f32; 3],
    /// Motor interference at 50% speed (µT).
    pub y_interference_50: [f32; 3],
    /// Motor interference at 50% speed (µT).
    pub z_interference_50: [f32; 3],
    /// Motor interference at 100% speed — index [0]=both, [1]=left, [2]=right (µT).
    pub x_interference_100: [f32; 3],
    /// Motor interference at 100% speed (µT).
    pub y_interference_100: [f32; 3],
    /// Motor interference at 100% speed (µT).
    pub z_interference_100: [f32; 3],
}

impl Default for MagCalibration {
    fn default() -> Self {
        Self {
            x_bias: 0.0,
            y_bias: 0.0,
            z_bias: 0.0,
            x_scale: 1.0,
            y_scale: 1.0,
            z_scale: 1.0,
            x_interference_50: [0.0; 3],
            y_interference_50: [0.0; 3],
            z_interference_50: [0.0; 3],
            x_interference_100: [0.0; 3],
            y_interference_100: [0.0; 3],
            z_interference_100: [0.0; 3],
        }
    }
}

// ── Internal command type ─────────────────────────────────────────────────────

/// Commands delivered to the IMU task via [`IMU_CONTROL`].
enum ImuCommand {
    /// Begin reading and publishing measurements.
    Start,
    /// Stop reading; enter standby.
    Stop,
    /// Replace the active calibration data (mag hard/soft-iron + interference).
    LoadCalibration(MagCalibration),
    /// Switch DMP fusion mode at runtime.
    SetFusionMode(DmpFusionMode),
}

// ── Shared statics ────────────────────────────────────────────────────────────

/// Control signal carrying [`ImuCommand`]s into the IMU task.
static IMU_CONTROL: Signal<CriticalSectionRawMutex, ImuCommand> = Signal::new();

/// `true` when `dmp_init_magnetometer` succeeded at start-up; gates `Axis9` usage.
static MAG_AVAILABLE: AtomicBool = AtomicBool::new(false);

/// Latest IMU readings snapshot — one mutex, one accessor.
static LATEST_READINGS: Mutex<CriticalSectionRawMutex, ImuReadings> = Mutex::new(ImuReadings {
    orientation: None,
    calibrated_gyro: None,
    calibrated_mag: None,
    raw_accel: None,
    raw_gyro: None,
    raw_mag: None,
});

/// Snapshot of all IMU sensor data updated on every valid DMP packet.
///
/// Fields are `None` before the first packet arrives. Magnetometer fields
/// (`calibrated_mag`, `raw_mag`) are `None` before the first `Axis9` packet
/// arrives and are cleared when switching to `Axis6` fusion mode.
#[derive(Debug, Clone, Copy, Default)]
pub struct ImuReadings {
    /// DMP-derived orientation.
    pub orientation: Option<Orientation>,
    /// DMP-calibrated gyroscope (deg/s, internal bias subtracted).
    pub calibrated_gyro: Option<Vector3<f32>>,
    /// Magnetometer reading (µT). Host-corrected when calibration is loaded;
    /// otherwise the raw value.
    pub calibrated_mag: Option<Vector3<f32>>,
    /// Raw accelerometer (g, before DMP correction).
    pub raw_accel: Option<Vector3<f32>>,
    /// Raw gyroscope (deg/s, before DMP correction).
    pub raw_gyro: Option<Vector3<f32>>,
    /// Raw magnetometer (µT, before any host correction).
    pub raw_mag: Option<Vector3<f32>>,
}

// ── Public API ────────────────────────────────────────────────────────────────

/// Signal the IMU task to start continuous DMP measurements.
pub fn start_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Start);
}

/// Signal the IMU task to stop measurements and enter standby.
pub fn stop_imu_readings() {
    IMU_CONTROL.signal(ImuCommand::Stop);
}

/// Select DMP fusion mode.
///
/// `Axis9` is silently downgraded to `Axis6` if the magnetometer was not
/// available at start-up, preserving the same behaviour as the previous
/// software-fusion implementation.
pub fn set_dmp_fusion_mode(mode: DmpFusionMode) {
    IMU_CONTROL.signal(ImuCommand::SetFusionMode(mode));
}

/// Load magnetometer calibration data (hard/soft-iron + motor-interference).
///
/// Applied to `ImuReadings::calibrated_mag` on every subsequent DMP packet.
pub fn load_mag_calibration(calibration: MagCalibration) {
    IMU_CONTROL.signal(ImuCommand::LoadCalibration(calibration));
}

/// Return a snapshot of all latest IMU readings from a single lock.
pub async fn get_latest_readings() -> ImuReadings {
    *LATEST_READINGS.lock().await
}

// ── Internal helpers ──────────────────────────────────────────────────────────

/// Convert a DMP quaternion to [`Orientation`] (Euler angles in degrees).
///
/// The ICM-20948 is mounted with its sensor X-axis aligned with the robot's
/// lateral (side-to-side) axis and its sensor Y-axis aligned with the robot's
/// forward axis.  The DMP therefore emits:
///   - `roll`  (rotation around sensor X) → robot **pitch** (nose-up/down)
///   - `pitch` (rotation around sensor Y) → robot **roll**  (lean left/right)
///   - `yaw`   (rotation around sensor Z) → robot **yaw**   (heading) — unchanged
fn dmp_quat_to_orientation(quat: &icm20948::dmp::Quaternion) -> Orientation {
    let angles = quat.to_euler_angles();
    let (roll_deg, pitch_deg, yaw_deg) = angles.to_degrees();
    Orientation {
        // Swap sensor roll↔pitch to match the robot body frame.
        roll: pitch_deg,
        pitch: roll_deg,
        yaw: yaw_deg,
    }
}

/// Interpolate motor-interference correction for a single track.
fn interpolate_single_track(calibration: &MagCalibration, speed: f32, is_left: bool) -> (f32, f32, f32) {
    let idx = if is_left { 1 } else { 2 };

    if speed <= 50.0 {
        let factor = speed / 50.0;
        (
            calibration.x_interference_50[idx] * factor,
            calibration.y_interference_50[idx] * factor,
            calibration.z_interference_50[idx] * factor,
        )
    } else {
        let factor = (speed - 50.0) / 50.0;
        (
            calibration.x_interference_50[idx]
                + (calibration.x_interference_100[idx] - calibration.x_interference_50[idx]) * factor,
            calibration.y_interference_50[idx]
                + (calibration.y_interference_100[idx] - calibration.y_interference_50[idx]) * factor,
            calibration.z_interference_50[idx]
                + (calibration.z_interference_100[idx] - calibration.z_interference_50[idx]) * factor,
        )
    }
}

/// Interpolate motor-interference correction when both tracks run at equal speed.
fn interpolate_equal_motors(calibration: &MagCalibration, speed: f32) -> (f32, f32, f32) {
    const ALL: usize = 0;

    if speed <= 50.0 {
        let factor = speed / 50.0;
        (
            calibration.x_interference_50[ALL] * factor,
            calibration.y_interference_50[ALL] * factor,
            calibration.z_interference_50[ALL] * factor,
        )
    } else {
        let factor = (speed - 50.0) / 50.0;
        (
            calibration.x_interference_50[ALL]
                + (calibration.x_interference_100[ALL] - calibration.x_interference_50[ALL]) * factor,
            calibration.y_interference_50[ALL]
                + (calibration.y_interference_100[ALL] - calibration.y_interference_50[ALL]) * factor,
            calibration.z_interference_50[ALL]
                + (calibration.z_interference_100[ALL] - calibration.z_interference_50[ALL]) * factor,
        )
    }
}

/// Compute the total motor-interference correction vector given current track speeds.
fn interpolate_interference(calibration: &MagCalibration, left_speed: i8, right_speed: i8) -> (f32, f32, f32) {
    let left_abs = f32::from(left_speed.abs());
    let right_abs = f32::from(right_speed.abs());

    if left_abs == 0.0 && right_abs == 0.0 {
        (0.0, 0.0, 0.0)
    } else if (left_abs - right_abs).abs() < 10.0 {
        interpolate_equal_motors(calibration, f32::midpoint(left_abs, right_abs))
    } else {
        let (lx, ly, lz) = interpolate_single_track(calibration, left_abs, true);
        let (rx, ry, rz) = interpolate_single_track(calibration, right_abs, false);
        let total = left_abs + right_abs;
        let lw = left_abs / total;
        let rw = right_abs / total;
        (lx * lw + rx * rw, ly * lw + ry * rw, lz * lw + rz * rw)
    }
}

/// Update the shared `LATEST_READINGS` and drive-task sensor channels from a DMP packet.
async fn update_statics_from_dmp(
    packet: &icm20948::dmp::DmpData,
    orientation: Option<Orientation>,
    calibration: Option<&MagCalibration>,
) {
    // ── Precompute all scaled vectors outside the critical section ──────────
    let raw_accel_opt: Option<Vector3<f32>> = packet.raw_accel.map(|(ax, ay, az)| {
        Vector3::new(
            f32::from(ax) * ACCEL_SCALE_G,
            f32::from(ay) * ACCEL_SCALE_G,
            f32::from(az) * ACCEL_SCALE_G,
        )
    });

    let raw_gyro_opt: Option<Vector3<f32>> = packet.raw_gyro.map(|(gx, gy, gz)| {
        Vector3::new(
            f32::from(gx) * GYRO_SCALE_DPS,
            f32::from(gy) * GYRO_SCALE_DPS,
            f32::from(gz) * GYRO_SCALE_DPS,
        )
    });

    #[allow(clippy::cast_precision_loss)]
    // raw values are i32 but unlikely that f32 conversion loses precision at nominal values to be expected here
    let calibrated_gyro_opt: Option<Vector3<f32>> = packet.calibrated_gyro.map(|(gx, gy, gz)| {
        Vector3::new(
            gx as f32 * GYRO_SCALE_DPS,
            gy as f32 * GYRO_SCALE_DPS,
            gz as f32 * GYRO_SCALE_DPS,
        )
    });

    // Raw magnetometer with optional host-side calibration ------------------
    let mag_data: Option<(Vector3<f32>, Vector3<f32>)> = packet.raw_mag.map(|(mx, my, mz)| {
        let v = Vector3::new(
            f32::from(mx) * MAG_SCALE_UT,
            f32::from(my) * MAG_SCALE_UT,
            f32::from(mz) * MAG_SCALE_UT,
        );
        let mut mag_cal = v;
        if let Some(cal) = calibration {
            let (left_speed, right_speed) = motion::get_track_speeds_atomic();
            let (ix, iy, iz) = interpolate_interference(cal, left_speed, right_speed);
            mag_cal.x = (v.x - ix - cal.x_bias) * cal.x_scale;
            mag_cal.y = (v.y - iy - cal.y_bias) * cal.y_scale;
            mag_cal.z = (v.z - iz - cal.z_bias) * cal.z_scale;
        }
        (v, mag_cal)
    });

    // ── Commit snapshot under one lock, then release for the async send ────
    let mut readings = LATEST_READINGS.lock().await;

    if let Some(o) = orientation {
        readings.orientation = Some(o);
    }
    if let Some(v) = raw_accel_opt {
        readings.raw_accel = Some(v);
    }
    if let Some(v) = raw_gyro_opt {
        readings.raw_gyro = Some(v);
    }
    if let Some(v) = calibrated_gyro_opt {
        readings.calibrated_gyro = Some(v);
    }
    if let Some((raw, cal)) = mag_data {
        readings.raw_mag = Some(raw);
        readings.calibrated_mag = Some(cal);
    }
    drop(readings);

    // Forward raw mag to the drive task for the magnetometer calibration procedure.
    if let Some((raw, _cal)) = mag_data {
        drive::send_mag_measurement(raw).await;
    }
}

/// Power-up, detect, and fully initialise the ICM-20948, retrying on failure.
///
/// Uses `SpiInterface` + `SpiDevice` on the dedicated SPI bus. The SPI
/// interface owns the bus by value. Chip initialisation (`init`) retries on
/// transient failures.
async fn init_imu_sensor(spi_bus: &'static SpiMutex, cs: CsOutput) -> ImuSensor<'static> {
    info!("Waiting {}ms for IMU power-up...", IMU_BOOT_DELAY_MS);
    Timer::after(Duration::from_millis(IMU_BOOT_DELAY_MS)).await;

    let spi_device = SpiDev::new(spi_bus, cs);
    let interface = SpiInterface::new(spi_device);

    let mut imu = match Icm20948Driver::try_new(interface).await {
        Ok(imu) => imu,
        Err(e) => {
            warn!("ICM-20948 SPI detection failed: {:?} — task halted", e);
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let mut delay = Delay;
    let mut attempt: u32 = 0;

    loop {
        attempt += 1;
        info!(
            "Initializing ICM-20948 via SPI (attempt {}/{})",
            attempt, IMU_INIT_MAX_ATTEMPTS
        );

        match imu.init(&mut delay).await {
            Ok(()) => {
                info!("ICM-20948 initialized successfully via SPI");
                Timer::after(Duration::from_millis(100)).await;
                return imu;
            }
            Err(e) => warn!("ICM-20948 init failed (attempt {}): {:?}", attempt, e),
        }

        if attempt >= IMU_INIT_MAX_ATTEMPTS {
            warn!(
                "ICM-20948 init gave up after {} attempts — task halted",
                IMU_INIT_MAX_ATTEMPTS
            );
            loop {
                Timer::after(Duration::from_secs(1)).await;
            }
        }

        Timer::after(Duration::from_millis(IMU_INIT_RETRY_DELAY_MS)).await;
    }
}

/// Load DMP firmware and initialise the magnetometer.
///
/// Sets [`MAG_AVAILABLE`] based on whether `dmp_init_magnetometer` succeeds.
/// Returns `false` if firmware loading fails (caller should terminate the task).
async fn init_dmp(sensor: &mut ImuSensor<'_>) -> bool {
    let mut delay = Delay;

    if let Err(e) = sensor.dmp_init(&mut delay).await {
        warn!("DMP firmware load failed: {:?}", e);
        return false;
    }
    info!("DMP firmware loaded");

    // Always attempt magnetometer init so Axis9 is available for later mode switches.
    match sensor.dmp_init_magnetometer(&mut delay).await {
        Ok(()) => {
            info!("DMP magnetometer initialized — Axis9 fusion available");
            MAG_AVAILABLE.store(true, Ordering::Relaxed);
        }
        Err(e) => {
            warn!("DMP magnetometer init failed: {:?} — Axis9 unavailable, Axis6 only", e);
            MAG_AVAILABLE.store(false, Ordering::Relaxed);
        }
    }

    true
}

/// Build a [`DmpConfig`] for the given fusion mode.
const fn build_dmp_config(mode: DmpFusionMode) -> DmpConfig {
    match mode {
        DmpFusionMode::Axis6 => DmpConfig::six_axis()
            .with_raw_accel()
            .with_raw_gyro()
            .with_calibrated_gyro()
            .with_sample_rate(DMP_SAMPLE_RATE_HZ),
        DmpFusionMode::Axis9 => DmpConfig::nine_axis()
            .with_raw_accel()
            .with_raw_gyro()
            .with_calibrated_gyro()
            .with_raw_mag()
            .with_sample_rate(DMP_SAMPLE_RATE_HZ),
    }
}

/// Apply a DMP configuration, enable the DMP, and flush the FIFO.
///
/// Returns `false` if any step fails.
async fn apply_dmp_config(sensor: &mut ImuSensor<'_>, config: &DmpConfig) -> bool {
    if let Err(e) = sensor.dmp_configure(config).await {
        warn!("DMP configure failed: {:?}", e);
        return false;
    }
    if let Err(e) = sensor.dmp_enable(true).await {
        warn!("DMP enable failed: {:?}", e);
        return false;
    }
    if let Err(e) = sensor.reset_fifo().await {
        warn!("FIFO reset failed: {:?}", e);
        return false;
    }
    true
}

/// Resolve the effective mode: downgrade `Axis9` to `Axis6` if mag is unavailable.
fn effective_mode(requested: DmpFusionMode) -> DmpFusionMode {
    if requested == DmpFusionMode::Axis9 && !MAG_AVAILABLE.load(Ordering::Relaxed) {
        info!("Axis9 requested but magnetometer unavailable — using Axis6");
        DmpFusionMode::Axis6
    } else {
        requested
    }
}

/// Core command/sampling loop.
///
/// Waits for [`ImuCommand::Start`], then polls the DMP FIFO on a fixed timer
/// while also handling in-band commands (stop, mode switch, calibration load).
#[allow(clippy::too_many_lines)]
async fn run_imu_command_loop(sensor: &mut ImuSensor<'_>) {
    let mut current_calibration: Option<MagCalibration> = None;
    let mut fusion_mode = effective_mode(DEFAULT_FUSION_MODE);

    #[cfg(feature = "telemetry_logs")]
    let mut last_loop_diag_ts_ms: u32 = 0;

    'command: loop {
        match IMU_CONTROL.wait().await {
            // ── Start command ─────────────────────────────────────────────────
            ImuCommand::Start => {
                info!("Starting IMU readings (DMP mode: {:?})", fusion_mode);

                let config = build_dmp_config(fusion_mode);
                if !apply_dmp_config(sensor, &config).await {
                    warn!("DMP start failed — waiting for next command");
                    continue 'command;
                }

                let mut fifo_failures: u32 = 0;

                loop {
                    match select(IMU_CONTROL.wait(), Timer::after(POLL_INTERVAL)).await {
                        // ── In-band: Stop ─────────────────────────────────────
                        Either::First(ImuCommand::Stop) => {
                            info!("IMU stopped");
                            let _ = sensor.dmp_enable(false).await;
                            IMU_READY.store(false, Ordering::Relaxed);
                            continue 'command;
                        }

                        // ── In-band: mode switch ──────────────────────────────
                        Either::First(ImuCommand::SetFusionMode(mode)) => {
                            let new_mode = effective_mode(mode);
                            if new_mode == fusion_mode {
                                info!("IMU fusion mode already {:?} — no change", fusion_mode);
                            } else {
                                let _ = sensor.dmp_enable(false).await;
                                let new_config = build_dmp_config(new_mode);
                                if apply_dmp_config(sensor, &new_config).await {
                                    fusion_mode = new_mode;
                                    info!("Switched DMP fusion mode to {:?}", fusion_mode);
                                    // Axis6 never produces magnetometer data — clear
                                    // any stale values left from a previous Axis9 session.
                                    if fusion_mode == DmpFusionMode::Axis6 {
                                        let mut readings = LATEST_READINGS.lock().await;
                                        readings.raw_mag = None;
                                        readings.calibrated_mag = None;
                                    }
                                } else {
                                    warn!("DMP mode switch failed — restoring previous {:?}", fusion_mode);
                                    let old_config = build_dmp_config(fusion_mode);
                                    if !apply_dmp_config(sensor, &old_config).await {
                                        warn!("DMP restore failed — returning to standby");
                                        continue 'command;
                                    }
                                }
                                fifo_failures = 0;
                            }
                        }

                        // ── In-band: calibration load ─────────────────────────
                        Either::First(ImuCommand::LoadCalibration(cal)) => {
                            current_calibration = Some(cal);
                            info!("IMU calibration data loaded");
                        }

                        // ── In-band: Start (already running — ignore) ─────────
                        Either::First(ImuCommand::Start) => {}

                        // ── Timer: poll FIFO ──────────────────────────────────
                        Either::Second(()) => {
                            match sensor.dmp_read_fifo().await {
                                Ok(Some(packet)) => {
                                    fifo_failures = 0;

                                    // Prefer 9-axis quaternion; fall back to 6-axis.
                                    let quat_opt =
                                        packet.quaternion_9axis.as_ref().or(packet.quaternion_6axis.as_ref());

                                    if let Some(quat) = quat_opt {
                                        let orientation = dmp_quat_to_orientation(quat);
                                        let timestamp_ms = Instant::now().as_millis();

                                        update_statics_from_dmp(
                                            &packet,
                                            Some(orientation),
                                            current_calibration.as_ref(),
                                        )
                                        .await;

                                        let measurement = ImuMeasurement {
                                            orientation,
                                            timestamp_ms,
                                        };

                                        #[cfg(feature = "telemetry_logs")]
                                        {
                                            #[allow(clippy::cast_possible_truncation)]
                                            let ts_ms = timestamp_ms as u32;
                                            if ts_ms.wrapping_sub(last_loop_diag_ts_ms) >= IMU_LOOP_DIAG_LOG_INTERVAL_MS
                                            {
                                                defmt::info!(
                                                    "IMU diag: mode={:?} yaw={=f32} pitch={=f32} roll={=f32}",
                                                    fusion_mode,
                                                    orientation.yaw,
                                                    orientation.pitch,
                                                    orientation.roll
                                                );
                                                last_loop_diag_ts_ms = ts_ms;
                                            }
                                        }

                                        // Forward to the drive task's IMU feedback channel (capacity 16).
                                        // The rotation control loop drains it each ~10 ms; dropping a
                                        // 100 Hz sample when the channel is full is lossy but harmless —
                                        // the next sample arrives in 10 ms.
                                        let _ = drive::try_send_imu_measurement(measurement);

                                        IMU_READY.store(true, Ordering::Relaxed);
                                    } else {
                                        // Packet present but no quaternion yet (DMP warming up).
                                        update_statics_from_dmp(&packet, None, current_calibration.as_ref()).await;
                                    }
                                }

                                Ok(None) => {
                                    // FIFO not ready — normal during DMP warm-up.
                                }

                                Err(e) => {
                                    fifo_failures += 1;
                                    warn!("DMP FIFO read error ({}/{}): {:?}", fifo_failures, MAX_FIFO_FAILURES, e);

                                    // Attempt a FIFO reset to clear any overflow.
                                    let _ = sensor.reset_fifo().await;

                                    if fifo_failures >= MAX_FIFO_FAILURES {
                                        warn!("Max FIFO failures reached — IMU task terminating");
                                        return;
                                    }
                                }
                            }
                        }
                    }
                }
            }

            // ── Standby: commands received before Start ────────────────────────
            ImuCommand::Stop => {
                info!("IMU stop received (already in standby)");
                IMU_READY.store(false, Ordering::Relaxed);
            }
            ImuCommand::LoadCalibration(cal) => {
                current_calibration = Some(cal);
                info!("IMU calibration loaded (standby)");
            }
            ImuCommand::SetFusionMode(mode) => {
                fusion_mode = effective_mode(mode);
                info!("IMU fusion mode set to {:?} (standby)", fusion_mode);
            }
        }
    }
}

// ── Embassy task ──────────────────────────────────────────────────────────────

/// Embassy task that drives DMP-based IMU measurements on the ICM-20948.
///
/// Initialises the sensor and DMP firmware, then enters the command/sampling
/// loop. The task terminates only on unrecoverable sensor failures.
///
/// Takes a reference to the SPI bus mutex and a CS output pin. The SPI bus
/// is exclusively owned by this task.
#[embassy_executor::task]
pub async fn inertial_measurement_read(spi_bus: &'static SpiMutex, cs: CsOutput) {
    let mut sensor = init_imu_sensor(spi_bus, cs).await;

    if !init_dmp(&mut sensor).await {
        warn!("DMP firmware load failed — IMU task terminating");
        return;
    }

    // Configure gyro DLPF to 51 Hz for anti-aliasing with the 100 Hz DMP
    // sample rate.  The reset-default 197 Hz DLPF passes motor vibration
    // straight through to the gyro, causing false yaw accumulation during
    // turns and false overshoot corrections after stopping.
    if let Err(e) = sensor
        .configure_gyroscope(GyroConfig {
            full_scale: GyroFullScale::Dps2000,
            dlpf: GyroDlpf::Hz51,
            dlpf_enable: true,
            sample_rate_div: 0,
        })
        .await
    {
        warn!("Gyro DLPF configure failed: {:?}", e);
    }

    info!(
        "IMU DMP ready. MAG_AVAILABLE={}. Default mode={:?}",
        MAG_AVAILABLE.load(Ordering::Relaxed),
        DEFAULT_FUSION_MODE
    );

    run_imu_command_loop(&mut sensor).await;
}
