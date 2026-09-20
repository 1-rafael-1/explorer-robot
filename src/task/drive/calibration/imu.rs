//! IMU calibration procedure (SPI).
//!
//! Calibrates gyroscope, accelerometer, and magnetometer, plus measures
//! motor interference effects on magnetometer for runtime compensation.
//!
//! The ICM-20948 is connected via a **dedicated SPI bus**. Driver
//! initialization uses `Interface::Spi(spi_device, cs_pin)`.
//!
//! The running screen reads the current phase and its percent from the activity
//! state; a phase that cannot be completed records why in the activity state
//! rather than drawing it. The operator's Stop is honoured at every phase
//! boundary and inside the guided rotation loop.

use defmt::info;
use embassy_time::{Duration, Instant, Timer};
use nalgebra::Vector3;
use touch_ui::Procedure;

use super::calibration_lifecycle;
use crate::{
    system::state::activity,
    task::{
        drive::{
            sensors::data::{clear_mag_measurement, measure_mag_average, subtract_mag, wait_for_mag_event_timeout},
            types::ImuCalibrationKind,
        },
        io::flash_storage,
        motor_driver::{self, MotorCommand},
        procedure::Lifecycle,
        sensors::imu as imu_read,
    },
};

#[derive(Copy, Clone)]
/// Thresholds and timing used by magnetometer calibration.
struct MagCalibrationConfig {
    /// Max consecutive mag read timeouts before failing.
    timeout_limit: u32,
    /// Max time allowed for the rotation coverage phase.
    max_seconds: u64,
    /// Minimum axis range (μT) required per axis.
    range_min_ut: f32,
    /// Minimum samples required before accepting coverage.
    min_samples: usize,
    /// Samples per averaging window (baseline + interference).
    avg_samples: u16,
    /// Minimum acceptable corrected field magnitude (μT).
    verify_min_ut: f32,
    /// Maximum acceptable corrected field magnitude (μT).
    verify_max_ut: f32,
    /// Max allowed delta from baseline magnitude (μT).
    verify_max_delta_ut: f32,
}

/// Default magnetometer calibration thresholds.
const MAG_CALIBRATION_CONFIG: MagCalibrationConfig = MagCalibrationConfig {
    timeout_limit: 25, // ~5s at 200ms
    max_seconds: 90,
    range_min_ut: 30.0,
    min_samples: 500,
    avg_samples: 250, // ~5s at 50Hz
    verify_min_ut: 20.0,
    verify_max_ut: 200.0,
    verify_max_delta_ut: 20.0,
};

/// Number of guided rotation phases, and the denominator of their percent.
const MANUAL_PHASES: usize = 3;

/// Countdown before the motor interference phase, in seconds.
const SETTLE_SECONDS: u64 = 20;

/// The magnetometer calibration's lifecycle: the calibration family's stop latch,
/// no slot, raising `CalibrationCompleted` on both outcomes. The phase machine,
/// settle and interference passes stay this module's own body.
const LIFECYCLE: Lifecycle = calibration_lifecycle(Procedure::MagCalibration);

/// Ensures IMU readings are stopped (and fusion mode restored) after calibration completes.
struct ImuReadingsGuard {
    /// Fusion mode to restore after calibration (if any).
    restore_fusion_mode: Option<crate::task::sensors::imu::DmpFusionMode>,
}

impl ImuReadingsGuard {
    /// Start IMU readings and set a temporary fusion mode.
    fn start_with_fusion_mode(
        target_mode: crate::task::sensors::imu::DmpFusionMode,
        restore_mode: crate::task::sensors::imu::DmpFusionMode,
    ) -> Self {
        crate::task::sensors::imu::start_imu_readings();
        crate::task::sensors::imu::set_dmp_fusion_mode(target_mode);
        Self {
            restore_fusion_mode: Some(restore_mode),
        }
    }
}

impl Drop for ImuReadingsGuard {
    fn drop(&mut self) {
        if let Some(mode) = self.restore_fusion_mode {
            crate::task::sensors::imu::set_dmp_fusion_mode(mode);
        }
        crate::task::sensors::imu::stop_imu_readings();
    }
}

#[derive(Copy, Clone)]
/// Motor command step used to measure or verify mag interference.
struct InterferenceStep {
    /// Operator-facing label for the step.
    label: &'static str,
    /// Left track command percentage.
    left: i8,
    /// Right track command percentage.
    right: i8,
    /// Nominal speed bucket (50 or 100).
    speed: u8,
    /// Slot index for storing results.
    slot: usize,
}

/// Sequence of steps for interference measurement and verification.
const INTERFERENCE_STEPS: [InterferenceStep; 6] = [
    InterferenceStep {
        label: "All 50%",
        left: 50,
        right: 50,
        speed: 50,
        slot: 0,
    },
    InterferenceStep {
        label: "All 100%",
        left: 100,
        right: 100,
        speed: 100,
        slot: 0,
    },
    InterferenceStep {
        label: "Left 50%",
        left: 50,
        right: 0,
        speed: 50,
        slot: 1,
    },
    InterferenceStep {
        label: "Left 100%",
        left: 100,
        right: 0,
        speed: 100,
        slot: 1,
    },
    InterferenceStep {
        label: "Right 50%",
        left: 0,
        right: 50,
        speed: 50,
        slot: 2,
    },
    InterferenceStep {
        label: "Right 100%",
        left: 0,
        right: 100,
        speed: 100,
        slot: 2,
    },
];

/// Tracks coverage of magnetometer readings during manual rotation.
struct MagCoverage {
    /// Minimum X reading observed.
    x_min: f32,
    /// Maximum X reading observed.
    x_max: f32,
    /// Minimum Y reading observed.
    y_min: f32,
    /// Maximum Y reading observed.
    y_max: f32,
    /// Minimum Z reading observed.
    z_min: f32,
    /// Maximum Z reading observed.
    z_max: f32,
    /// Total samples collected.
    samples: usize,
}

impl MagCoverage {
    /// Create an empty coverage tracker.
    const fn new() -> Self {
        Self {
            x_min: f32::MAX,
            x_max: f32::MIN,
            y_min: f32::MAX,
            y_max: f32::MIN,
            z_min: f32::MAX,
            z_max: f32::MIN,
            samples: 0,
        }
    }

    /// Update min/max ranges with a new magnetometer sample.
    fn update(&mut self, mag: Vector3<f32>) {
        self.samples += 1;
        self.x_min = self.x_min.min(mag.x);
        self.x_max = self.x_max.max(mag.x);
        self.y_min = self.y_min.min(mag.y);
        self.y_max = self.y_max.max(mag.y);
        self.z_min = self.z_min.min(mag.z);
        self.z_max = self.z_max.max(mag.z);
    }

    /// Return the span of each axis (max - min).
    const fn ranges(&self) -> (f32, f32, f32) {
        (
            self.x_max - self.x_min,
            self.y_max - self.y_min,
            self.z_max - self.z_min,
        )
    }

    /// Count how many axes meet the minimum coverage requirement.
    const fn axes_ok(&self, config: MagCalibrationConfig) -> u8 {
        let (x_range, y_range, z_range) = self.ranges();
        (if x_range >= config.range_min_ut { 1 } else { 0 })
            + (if y_range >= config.range_min_ut { 1 } else { 0 })
            + (if z_range >= config.range_min_ut { 1 } else { 0 })
    }
}

/// Rotation guidance step for magnetometer coverage.
struct MagRotationStep {
    /// Require X-axis coverage for this step.
    require_x: bool,
    /// Require Y-axis coverage for this step.
    require_y: bool,
    /// Require Z-axis coverage for this step.
    require_z: bool,
    /// Minimum yaw heading span in degrees for this step (None = not used).
    min_heading_span_deg: Option<f32>,
}

/// Magnetometer calibration phase machine.
#[derive(Copy, Clone, Eq, PartialEq)]
enum MagCalibrationPhase {
    /// Manual yaw rotation phase.
    ManualYaw,
    /// Manual pitch rotation phase.
    ManualPitch,
    /// Manual roll rotation phase.
    ManualRoll,
    /// Settling delay before motor phase.
    SettleDelay,
    /// Baseline magnetometer measurement with motors off.
    MotorBaseline,
    /// Motor interference measurement phase.
    MotorMeasure,
    /// Motor interference verification phase.
    MotorVerify,
    /// Save calibration results.
    Save,
}

impl MagCalibrationPhase {
    /// Human-readable phase name.
    const fn label(self) -> &'static str {
        match self {
            Self::ManualYaw => "P1 YAW",
            Self::ManualPitch => "P2 PITCH",
            Self::ManualRoll => "P3 ROLL",
            Self::SettleDelay => "P4 WAIT20",
            Self::MotorBaseline => "P5 BASE",
            Self::MotorMeasure => "P6 MOTOR",
            Self::MotorVerify => "P7 VERIFY",
            Self::Save => "P8 SAVE",
        }
    }

    /// 1-based phase number and total count for user-visible progress.
    const fn progress(self) -> Option<(u8, u8)> {
        match self {
            Self::ManualYaw => Some((1, 7)),
            Self::ManualPitch => Some((2, 7)),
            Self::ManualRoll => Some((3, 7)),
            Self::SettleDelay => Some((4, 7)),
            Self::MotorBaseline => Some((5, 7)),
            Self::MotorMeasure => Some((6, 7)),
            Self::MotorVerify => Some((7, 7)),
            Self::Save => None,
        }
    }

    /// The instruction the running screen shows while this phase needs the
    /// operator to do something.
    const fn prompt(self) -> &'static str {
        match self {
            Self::ManualYaw => "Keep flat, spin on table",
            Self::ManualPitch => "Tilt nose up and down",
            Self::ManualRoll => "Tilt left and right",
            _ => "Move slowly, follow the prompt",
        }
    }
}

/// Sequence of user-guided rotations for magnetometer coverage.
const MAG_ROTATION_STEPS: [MagRotationStep; MANUAL_PHASES] = [
    MagRotationStep {
        require_x: false,
        require_y: false,
        require_z: false,
        min_heading_span_deg: Some(50.0),
    },
    MagRotationStep {
        require_x: true,
        require_y: false,
        require_z: true,
        min_heading_span_deg: None,
    },
    MagRotationStep {
        require_x: false,
        require_y: true,
        require_z: true,
        min_heading_span_deg: None,
    },
];

/// Captured motor interference vectors at 50% and 100% commands.
struct InterferenceData {
    /// X-axis interference at 50% (all, left, right).
    x_50: [f32; 3],
    /// Y-axis interference at 50% (all, left, right).
    y_50: [f32; 3],
    /// Z-axis interference at 50% (all, left, right).
    z_50: [f32; 3],
    /// X-axis interference at 100% (all, left, right).
    x_100: [f32; 3],
    /// Y-axis interference at 100% (all, left, right).
    y_100: [f32; 3],
    /// Z-axis interference at 100% (all, left, right).
    z_100: [f32; 3],
}

impl InterferenceData {
    /// Create empty interference buffers.
    const fn new() -> Self {
        Self {
            x_50: [0.0; 3],
            y_50: [0.0; 3],
            z_50: [0.0; 3],
            x_100: [0.0; 3],
            y_100: [0.0; 3],
            z_100: [0.0; 3],
        }
    }

    /// Store the measured interference vector for a step.
    fn set(&mut self, step: &InterferenceStep, interference: Vector3<f32>) {
        if step.speed == 50 {
            self.x_50[step.slot] = interference.x;
            self.y_50[step.slot] = interference.y;
            self.z_50[step.slot] = interference.z;
        } else {
            self.x_100[step.slot] = interference.x;
            self.y_100[step.slot] = interference.y;
            self.z_100[step.slot] = interference.z;
        }
    }

    /// Retrieve the interference vector for a step.
    const fn vector_for(&self, step: &InterferenceStep) -> Vector3<f32> {
        if step.speed == 50 {
            Vector3::new(self.x_50[step.slot], self.y_50[step.slot], self.z_50[step.slot])
        } else {
            Vector3::new(self.x_100[step.slot], self.y_100[step.slot], self.z_100[step.slot])
        }
    }
}

/// Output of magnetometer calibration.
struct MagCalibrationResult {
    /// Hard-iron bias in μT.
    bias: Vector3<f32>,
    /// Soft-iron scale factors.
    scale: Vector3<f32>,
    /// Motor interference measurements.
    interference: InterferenceData,
}

/// How a guided phase ended.
#[derive(Copy, Clone, Eq, PartialEq)]
enum PhaseResult {
    /// The phase collected what it needed.
    Complete,
    /// The phase ran out of time or samples first.
    Incomplete,
    /// The operator stopped the calibration.
    Stopped,
}

/// How the manual rotation coverage attempt ended.
enum CoverageOutcome {
    /// Enough coverage on all three axes.
    Collected(MagCoverage),
    /// The attempt ended without usable coverage, and why.
    Incomplete(&'static str),
    /// The operator stopped the calibration.
    Stopped,
}

/// How the whole magnetometer calibration ended.
enum Outcome {
    /// A result ready to save.
    Success(MagCalibrationResult),
    /// The procedure could not produce a result, and why.
    Incomplete(&'static str),
    /// The operator stopped the calibration.
    Stopped,
}

/// Publish a phase transition: the percent from the phase's place in the
/// sequence, and `detail` if given, else the phase's own label.
async fn enter_mag_phase(phase: MagCalibrationPhase, detail: Option<&'static str>) {
    let percent = phase
        .progress()
        .map(|(index, total)| activity::percent_done(usize::from(index).saturating_sub(1), usize::from(total)));
    let line = detail.unwrap_or_else(|| phase.label());
    info!("Mag phase transition -> {} ({})", phase.label(), line);
    LIFECYCLE.phase(line, percent).await;
}

/// Guide one explicit manual rotation phase and collect coverage.
#[allow(clippy::too_many_lines)]
async fn measure_mag_rotation_phase(
    phase: MagCalibrationPhase,
    step: &MagRotationStep,
    config: MagCalibrationConfig,
    coverage: &mut MagCoverage,
) -> PhaseResult {
    enter_mag_phase(phase, Some(phase.prompt())).await;

    let step_timeout = (config.max_seconds / MAG_ROTATION_STEPS.len() as u64).max(10);
    let min_step_samples = (config.min_samples / MAG_ROTATION_STEPS.len()).max(1);

    let mut step_coverage = MagCoverage::new();
    let mut mag_timeout_streak: u32 = 0;
    let start_time = Instant::now();
    let mut heading_min: f32 = f32::MAX;
    let mut heading_max: f32 = f32::MIN;
    let mut last_heading: Option<f32> = None;
    let mut heading_offset: f32 = 0.0;
    let mut last_log_ms: u32 = 0;

    loop {
        if LIFECYCLE.is_stop_requested() {
            return PhaseResult::Stopped;
        }

        let elapsed_secs = Instant::now().duration_since(start_time).as_secs();
        if elapsed_secs >= step_timeout {
            info!("Mag phase timeout at {}", phase.label());
            return PhaseResult::Incomplete;
        }

        clear_mag_measurement().await;
        if let Some(mag_data) = wait_for_mag_event_timeout(200).await {
            coverage.update(mag_data);
            step_coverage.update(mag_data);
            mag_timeout_streak = 0;

            if step.min_heading_span_deg.is_some() {
                let heading_deg = libm::atan2f(mag_data.y, mag_data.x).to_degrees();
                if let Some(prev) = last_heading {
                    let delta = heading_deg - prev;
                    if delta > 180.0 {
                        heading_offset -= 360.0;
                    } else if delta < -180.0 {
                        heading_offset += 360.0;
                    }
                }
                last_heading = Some(heading_deg);
                let unwrapped = heading_deg + heading_offset;
                heading_min = heading_min.min(unwrapped);
                heading_max = heading_max.max(unwrapped);
            }
        } else {
            mag_timeout_streak += 1;
        }

        let (x_range, y_range, z_range) = step_coverage.ranges();
        let heading_span = if step.min_heading_span_deg.is_some() {
            heading_max - heading_min
        } else {
            0.0
        };
        let heading_span_ok = step
            .min_heading_span_deg
            .is_none_or(|min_span| heading_span >= min_span);

        let step_ok = (!step.require_x || x_range >= config.range_min_ut)
            && (!step.require_y || y_range >= config.range_min_ut)
            && (!step.require_z || z_range >= config.range_min_ut)
            && heading_span_ok;

        if step.min_heading_span_deg.is_some() {
            #[allow(clippy::cast_possible_truncation)]
            let now_ms = Instant::now().as_millis() as u32;
            if now_ms.wrapping_sub(last_log_ms) >= 1000 {
                last_log_ms = now_ms;
                info!(
                    "Yaw span {} deg (min={} max={}) samples={}",
                    heading_span, heading_min, heading_max, step_coverage.samples
                );
            }
        }

        if step_ok && step_coverage.samples >= min_step_samples {
            info!(
                "Mag phase complete: {} (samples={})",
                phase.label(),
                step_coverage.samples
            );
            LIFECYCLE.phase("Phase OK", None).await;
            if LIFECYCLE.wait_or_stop(500).await {
                return PhaseResult::Stopped;
            }
            return PhaseResult::Complete;
        }

        if mag_timeout_streak >= config.timeout_limit {
            info!("Mag phase timeout streak exceeded at {}", phase.label());
            return PhaseResult::Incomplete;
        }
    }
}

/// Guide the user through explicit axis-specific phases and collect coverage.
async fn measure_mag_coverage(config: MagCalibrationConfig) -> CoverageOutcome {
    let mut coverage = MagCoverage::new();

    let phases = [
        (MagCalibrationPhase::ManualYaw, "Yaw incomplete"),
        (MagCalibrationPhase::ManualPitch, "Pitch incomplete"),
        (MagCalibrationPhase::ManualRoll, "Roll incomplete"),
    ];

    for (index, (phase, reason)) in phases.iter().enumerate() {
        match measure_mag_rotation_phase(*phase, &MAG_ROTATION_STEPS[index], config, &mut coverage).await {
            PhaseResult::Complete => {}
            PhaseResult::Incomplete => return CoverageOutcome::Incomplete(reason),
            PhaseResult::Stopped => return CoverageOutcome::Stopped,
        }
    }

    let axes_ok = coverage.axes_ok(config);
    if coverage.samples == 0 || axes_ok < 3 {
        info!(
            "Mag calibration failed after manual phases (samples={}, axes_ok={})",
            coverage.samples, axes_ok
        );
        return CoverageOutcome::Incomplete("Rotate more — all axes");
    }

    CoverageOutcome::Collected(coverage)
}

/// Enable or disable motor drivers during mag calibration.
async fn set_motor_drivers_enabled(enabled: bool) {
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled }).await;
}

/// Measure motor-induced magnetometer interference across predefined steps.
///
/// Returns `None` when the operator stopped the calibration.
async fn measure_mag_interference(
    config: MagCalibrationConfig,
    baseline_mag: Vector3<f32>,
) -> Option<InterferenceData> {
    let mut data = InterferenceData::new();
    let total = INTERFERENCE_STEPS.len();

    for (index, step) in INTERFERENCE_STEPS.iter().enumerate() {
        if LIFECYCLE.is_stop_requested() {
            return None;
        }
        info!("Motor interference step start: {=str}", step.label);
        LIFECYCLE
            .phase(step.label, Some(activity::percent_done(index, total)))
            .await;

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: step.left,
            right_speed: step.right,
        })
        .await;
        if LIFECYCLE.wait_or_stop(1_000).await {
            return None;
        }
        clear_mag_measurement().await;
        let mag_avg = measure_mag_average(config.avg_samples).await;
        let interference = subtract_mag(mag_avg, baseline_mag);

        data.set(step, interference);
        info!("Motor interference step done: {=str}", step.label);

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        if LIFECYCLE.wait_or_stop(500).await {
            return None;
        }
    }

    Some(data)
}

/// Verify interference compensation keeps the field within expected limits.
///
/// Returns `None` when the operator stopped the calibration.
async fn verify_mag_interference(
    config: MagCalibrationConfig,
    bias: Vector3<f32>,
    scale: Vector3<f32>,
    baseline_mag: Vector3<f32>,
    interference: &InterferenceData,
) -> Option<bool> {
    let baseline_corrected = Vector3::new(
        (baseline_mag.x - bias.x) * scale.x,
        (baseline_mag.y - bias.y) * scale.y,
        (baseline_mag.z - bias.z) * scale.z,
    );
    let baseline_norm = baseline_corrected.norm();
    let total = INTERFERENCE_STEPS.len();

    for (index, step) in INTERFERENCE_STEPS.iter().enumerate() {
        if LIFECYCLE.is_stop_requested() {
            return None;
        }
        info!("Motor verify step {}/{}: {=str}", index + 1, total, step.label);
        LIFECYCLE
            .phase(step.label, Some(activity::percent_done(index, total)))
            .await;

        motor_driver::send_motor_command(MotorCommand::SetTracks {
            left_speed: step.left,
            right_speed: step.right,
        })
        .await;
        if LIFECYCLE.wait_or_stop(1_000).await {
            return None;
        }
        clear_mag_measurement().await;
        let mag_avg = measure_mag_average(config.avg_samples).await;

        let interference_vec = interference.vector_for(step);
        let mut corrected = subtract_mag(mag_avg, interference_vec);
        corrected.x = (corrected.x - bias.x) * scale.x;
        corrected.y = (corrected.y - bias.y) * scale.y;
        corrected.z = (corrected.z - bias.z) * scale.z;

        let mag_norm = corrected.norm();
        if mag_norm < config.verify_min_ut
            || mag_norm > config.verify_max_ut
            || (mag_norm - baseline_norm).abs() > config.verify_max_delta_ut
        {
            return Some(false);
        }

        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        if LIFECYCLE.wait_or_stop(500).await {
            return None;
        }
    }

    Some(true)
}

/// Wait out the settle countdown, publishing the progress toward the motor
/// phase.
///
/// Returns `true` when the operator stopped the calibration.
async fn run_settle_countdown() -> bool {
    LIFECYCLE.phase("Hold still, motors soon", Some(0)).await;
    let total = usize::try_from(SETTLE_SECONDS).unwrap_or(1);

    for second in 0..SETTLE_SECONDS {
        let elapsed = usize::try_from(second).unwrap_or(0);
        LIFECYCLE
            .phase("Hold still, motors soon", Some(activity::percent_done(elapsed, total)))
            .await;
        if LIFECYCLE.wait_or_stop(1_000).await {
            return true;
        }
    }

    false
}

/// Run the magnetometer calibration flow.
#[allow(clippy::too_many_lines)]
async fn run_mag_calibration_steps(config: MagCalibrationConfig) -> Outcome {
    info!("Step 2: Magnetometer Calibration (strict phase machine)");

    LIFECYCLE.phase("Prepare to move", None).await;
    if LIFECYCLE.wait_or_stop(3_000).await {
        return Outcome::Stopped;
    }

    let coverage = match measure_mag_coverage(config).await {
        CoverageOutcome::Collected(coverage) => coverage,
        CoverageOutcome::Incomplete(reason) => {
            info!("MAG CAL RESULT: reached_motor_phase=false, reason={}", reason);
            return Outcome::Incomplete(reason);
        }
        CoverageOutcome::Stopped => return Outcome::Stopped,
    };

    let (x_range, y_range, z_range) = coverage.ranges();

    let mag_bias = Vector3::new(
        f32::midpoint(coverage.x_max, coverage.x_min),
        f32::midpoint(coverage.y_max, coverage.y_min),
        f32::midpoint(coverage.z_max, coverage.z_min),
    );

    let x_radius = x_range / 2.0;
    let y_radius = y_range / 2.0;
    let z_radius = z_range / 2.0;
    let avg_radius = (x_radius + y_radius + z_radius) / 3.0;

    let mag_scale = Vector3::new(avg_radius / x_radius, avg_radius / y_radius, avg_radius / z_radius);

    info!("  ✓ Magnetometer coverage complete!");
    info!(
        "  Hard iron bias (μT): X={} Y={} Z={}",
        mag_bias.x, mag_bias.y, mag_bias.z
    );
    info!(
        "  Soft iron scale: X={} Y={} Z={}",
        mag_scale.x, mag_scale.y, mag_scale.z
    );

    enter_mag_phase(MagCalibrationPhase::SettleDelay, Some("Set robot down")).await;
    if run_settle_countdown().await {
        return Outcome::Stopped;
    }

    info!("Mag calibration entering motor phase");
    set_motor_drivers_enabled(true).await;
    Timer::after(Duration::from_millis(100)).await;

    enter_mag_phase(MagCalibrationPhase::MotorBaseline, Some("Baseline, motors off")).await;
    clear_mag_measurement().await;
    Timer::after(Duration::from_millis(500)).await;
    let baseline_mag = measure_mag_average(config.avg_samples).await;

    enter_mag_phase(MagCalibrationPhase::MotorMeasure, Some("Motor interference")).await;
    let Some(interference) = measure_mag_interference(config, baseline_mag).await else {
        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        set_motor_drivers_enabled(false).await;
        return Outcome::Stopped;
    };

    enter_mag_phase(MagCalibrationPhase::MotorVerify, Some("Verify compensation")).await;
    let Some(verify_ok) = verify_mag_interference(config, mag_bias, mag_scale, baseline_mag, &interference).await
    else {
        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        set_motor_drivers_enabled(false).await;
        return Outcome::Stopped;
    };

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    Timer::after(Duration::from_millis(500)).await;
    set_motor_drivers_enabled(false).await;

    if !verify_ok {
        info!("Mag calibration failed in motor verify phase");
        info!("MAG CAL RESULT: reached_motor_phase=true, reason=motor verify failed");
        return Outcome::Incomplete("Motor verify failed");
    }

    enter_mag_phase(MagCalibrationPhase::Save, Some("Saving")).await;
    info!("MAG CAL RESULT: reached_motor_phase=true, reason=success");

    Outcome::Success(MagCalibrationResult {
        bias: mag_bias,
        scale: mag_scale,
        interference,
    })
}

#[allow(clippy::too_many_lines)]
#[allow(clippy::similar_names)]
#[allow(clippy::cast_precision_loss)]
/// Calibrate the magnetometer and persist results.
///
/// The ICM-20948 is connected via a dedicated SPI bus. The `sensors::imu` module
/// (ticket 04) handles SPI initialization; this function drives the calibration
/// algorithm and records the outcome in the activity state.
async fn run_mag_calibration() {
    info!("=== Starting IMU Mag Calibration ===");

    LIFECYCLE.arm().await;
    let _ = LIFECYCLE.start("Initializing").await;

    let _imu_guard =
        ImuReadingsGuard::start_with_fusion_mode(imu_read::DmpFusionMode::Axis9, imu_read::DEFAULT_FUSION_MODE);
    Timer::after(Duration::from_millis(500)).await;

    let mut mag_cal = imu_read::MagCalibration::default();
    let mut imu_flags = flash_storage::get_cached_imu_flags().await.unwrap_or_default();

    let mag_result = match run_mag_calibration_steps(MAG_CALIBRATION_CONFIG).await {
        Outcome::Success(result) => result,
        Outcome::Incomplete(reason) => {
            info!("Mag calibration failed; keeping previous values");
            LIFECYCLE.fail(reason).await;
            return;
        }
        Outcome::Stopped => {
            info!("Mag calibration stopped by the operator");
            LIFECYCLE.abandon().await;
            return;
        }
    };

    mag_cal.x_bias = mag_result.bias.x;
    mag_cal.y_bias = mag_result.bias.y;
    mag_cal.z_bias = mag_result.bias.z;
    mag_cal.x_scale = mag_result.scale.x;
    mag_cal.y_scale = mag_result.scale.y;
    mag_cal.z_scale = mag_result.scale.z;
    mag_cal.x_interference_50 = mag_result.interference.x_50;
    mag_cal.y_interference_50 = mag_result.interference.y_50;
    mag_cal.z_interference_50 = mag_result.interference.z_50;
    mag_cal.x_interference_100 = mag_result.interference.x_100;
    mag_cal.y_interference_100 = mag_result.interference.y_100;
    mag_cal.z_interference_100 = mag_result.interference.z_100;
    imu_flags.mag = true;

    info!("Saving mag calibration to flash");
    info!(
        "  Magnetometer hard iron bias (μT): X={} Y={} Z={}",
        mag_cal.x_bias, mag_cal.y_bias, mag_cal.z_bias
    );
    info!(
        "  Magnetometer soft iron scale: X={} Y={} Z={}",
        mag_cal.x_scale, mag_cal.y_scale, mag_cal.z_scale
    );
    info!("  Motor interference patterns captured");

    LIFECYCLE.phase("Saving to flash", None).await;

    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveData(
        flash_storage::CalibrationDataKind::ImuFlags(imu_flags),
    ))
    .await;

    Timer::after(Duration::from_millis(500)).await;

    info!("Applying mag calibration to IMU task");
    imu_read::load_mag_calibration(mag_cal);

    LIFECYCLE.complete("Calibration saved").await;
    Timer::after(Duration::from_secs(2)).await;

    info!("=== IMU Mag Calibration Complete ===");
}

/// Dispatch the requested IMU calibration routine.
///
/// The ICM-20948 is connected via a dedicated SPI bus. The sensor module
/// (`crate::task::sensors::imu`) owns SPI initialization.
pub async fn run_imu_calibration(kind: ImuCalibrationKind) {
    match kind {
        ImuCalibrationKind::Mag => run_mag_calibration().await,
    }
}
