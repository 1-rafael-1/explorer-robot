//! The Link Task's 10 Hz telemetry sampler.
//!
//! One call to [`sample_and_publish`] reads the state modules and the `LiDAR`
//! cloud once, in the documented lock order (power → calibration → perception →
//! motion → activity), and hands each reading to [`LinkCore::publish_stream`].
//! The core owns the effective rate, the on-change rule and the heartbeat; the
//! sampler only reports the current value, so there is no second on-change rule
//! in the firmware (ADR-0032).
//!
//! No lock is held across an `await` and no two state locks are ever held at
//! once. The cloud is the one exception to "publish every tick": it lives under
//! the perception mutex and is encoded inside [`perception::with_lidar`], so its
//! 720-byte slot array is never cloned, and it is skipped while its sequence has
//! not advanced since the last published sample.

use core::sync::atomic::{AtomicU32, Ordering};

use link_core::LinkCore;
use link_protocol::{
    ProcedureId, Stream, StreamSample,
    payload::stream::{
        Activity as WireActivity, ActivityStage, Calibration as WireCalibration,
        CalibrationStatus as WireCalibrationStatus, LEVEL_UNKNOWN, LidarCloud, Motion, Perception, Power,
        VOLTAGE_UNKNOWN_MV,
    },
};
use touch_ui::{Procedure, buffer::TextBuf};

use crate::system::state::{CalibrationStatus, activity, calibration, motion, perception, power};

// ── Constants ─────────────────────────────────────────────────────────────────

/// The Activity-only wire id reserved for a Manual Drive (ADR-0031).
const MANUAL_DRIVE_PROCEDURE_ID: u16 = 0x000D;

/// The firmware's idle `MOTION` ceiling, in centi-Hz (1 Hz).
///
/// While a track is moving the firmware has no preference (`0`) and the core's
/// baseline (10 Hz) or a Session's Requested Rate applies. While both are still
/// it asks for 1 Hz, so an idle robot keeps a heartbeat without streaming at
/// 10 Hz.
const MOTION_IDLE_RATE_CHZ: u32 = 100;

/// Sequence sentinel meaning "no cloud has been published in this link period".
///
/// The `LiDAR` sequence is a monotonically increasing `u64`. The 32-bit target
/// has no 64-bit atomic, so only the low word is remembered: a collision needs
/// 2^32 scans (over 13 years at 10 Hz) and can only skip one publish.
const NO_SEQUENCE: u32 = u32::MAX;

/// The low word of the last `LiDAR` cloud sequence handed to the link core.
///
/// Reset by [`reset`] at the start of each enabled period so a fresh core always
/// receives the current cloud once, even if the sensor has not scanned since the
/// previous link period.
static LAST_CLOUD_SEQUENCE: AtomicU32 = AtomicU32::new(NO_SEQUENCE);

// ── Public surface ────────────────────────────────────────────────────────────

/// Forget the remembered cloud sequence between link periods.
pub fn reset() {
    LAST_CLOUD_SEQUENCE.store(NO_SEQUENCE, Ordering::Relaxed);
}

/// Sample one of each stream and publish the current value.
///
/// Runs on the 10 Hz tick immediately after [`LinkCore::tick`], so scheduling is
/// evaluated at the same instant as the rest of the link state. Every stream is
/// published each tick except `LIDAR_CLOUD`, which is published only when the
/// sensor's sequence has advanced.
pub async fn sample_and_publish(core: &mut LinkCore, now_ms: u64) {
    // Lock order: power → calibration → perception → motion → activity.
    sample_power(core, now_ms).await;
    sample_calibration(core, now_ms).await;
    sample_perception(core, now_ms);
    sample_cloud(core, now_ms).await;
    sample_motion(core, now_ms);
    sample_activity(core, now_ms).await;
}

/// Every Procedure, in the panel's menu and Catalogue order.
///
/// This is the one list: [`procedure_id`] is the one id table, and the command
/// seam searches this array for the inverse rather than restating either.
/// Manual Drive is an Activity, not a Procedure, so it is absent.
pub(super) const PROCEDURES: [Procedure; 11] = [
    Procedure::MotorCalibration,
    Procedure::MagCalibration,
    Procedure::DistanceCalibration,
    Procedure::CoastAndAvoid,
    Procedure::AttemptStraight,
    Procedure::BasicMotor,
    Procedure::Turns,
    Procedure::StraightDrive,
    Procedure::ArcDrive,
    Procedure::Imu6Axis,
    Procedure::Imu9Axis,
];

/// The wire `ProcedureId` for a firmware Procedure.
///
/// The one home for ADR-0031's id table; [`super::events`] and the command seam
/// reuse it rather than restating the mapping.
#[must_use]
pub(super) const fn procedure_id(procedure: Procedure) -> ProcedureId {
    match procedure {
        Procedure::MotorCalibration => ProcedureId::MOTOR_CALIBRATION,
        Procedure::MagCalibration => ProcedureId::MAG_CALIBRATION,
        Procedure::DistanceCalibration => ProcedureId::DISTANCE_CALIBRATION,
        Procedure::CoastAndAvoid => ProcedureId::COAST_AND_AVOID,
        Procedure::AttemptStraight => ProcedureId::ATTEMPT_STRAIGHT,
        Procedure::BasicMotor => ProcedureId::BASIC_MOTOR,
        Procedure::Turns => ProcedureId::TURNS,
        Procedure::StraightDrive => ProcedureId::STRAIGHT_DRIVE,
        Procedure::ArcDrive => ProcedureId::ARC_DRIVE,
        Procedure::Imu6Axis => ProcedureId::IMU_6_AXIS,
        Procedure::Imu9Axis => ProcedureId::IMU_9_AXIS,
    }
}

// ── Per-stream samplers ───────────────────────────────────────────────────────

/// Publish the battery snapshot, never inventing a missing value.
async fn sample_power(core: &mut LinkCore, now_ms: u64) {
    let sample = power_sample().await;
    core.publish_stream(&StreamSample::Power(sample), 0, now_ms);
}

/// Build the `POWER` sample, never inventing a missing value.
pub(super) async fn power_sample() -> Power {
    let snapshot = power::get_battery_snapshot().await;
    Power {
        level: snapshot.level.unwrap_or(LEVEL_UNKNOWN),
        voltage_mv: snapshot.voltage.map_or(VOLTAGE_UNKNOWN_MV, voltage_mv),
    }
}

/// Publish the four calibration statuses.
async fn sample_calibration(core: &mut LinkCore, now_ms: u64) {
    let sample = calibration_sample().await;
    core.publish_stream(&StreamSample::Calibration(sample), 0, now_ms);
}

/// Build the four calibration statuses.
pub(super) async fn calibration_sample() -> WireCalibration {
    let (motor, imu, mag, distance) = {
        let state = calibration::CALIBRATION_STATE.lock().await;
        (
            state.motor_cal_status,
            state.imu_cal_status,
            state.mag_cal_status,
            state.distance_cal_status,
        )
    };

    WireCalibration {
        motor: wire_calibration_status(motor),
        imu: wire_calibration_status(imu),
        mag: wire_calibration_status(mag),
        distance: wire_calibration_status(distance),
    }
}

/// Publish the lock-free perception flags.
fn sample_perception(core: &mut LinkCore, now_ms: u64) {
    let sample = perception_sample();
    core.publish_stream(&StreamSample::Perception(sample), 0, now_ms);
}

/// Build the lock-free perception flags.
pub(super) fn perception_sample() -> Perception {
    let mut flags = Perception::default();
    if perception::is_obstacle_detected() {
        flags |= Perception::OBSTACLE;
    }
    if perception::is_floor_drop_detected() {
        flags |= Perception::FLOOR_DROP;
    }
    flags
}

/// Publish the `LiDAR` cloud when its sequence has advanced.
///
/// The whole conversion and publish run inside [`perception::with_lidar`], so
/// the cloud's slot array is read under the perception lock and encoded
/// immediately; nothing is cloned and no other lock is taken.
#[allow(clippy::cast_possible_truncation)]
async fn sample_cloud(core: &mut LinkCore, now_ms: u64) {
    perception::with_lidar(|cloud| {
        let Some(cloud) = cloud else {
            return;
        };
        let sequence = cloud.sequence() as u32;
        if LAST_CLOUD_SEQUENCE.load(Ordering::Relaxed) == sequence {
            return;
        }

        let mut slots = [LidarCloud::NO_RETURN; lidar_cloud::SLOTS];
        for (out, distance) in slots.iter_mut().zip(cloud.slots()) {
            if let Some(distance_cm) = *distance {
                *out = whole_cm(distance_cm);
            }
        }

        let sample = LidarCloud { slots };
        core.publish_stream(&StreamSample::LidarCloud(sample), 0, now_ms);
        LAST_CLOUD_SEQUENCE.store(sequence, Ordering::Relaxed);
    })
    .await;
}

/// Publish the lock-free track speeds with the idle heartbeat preference.
fn sample_motion(core: &mut LinkCore, now_ms: u64) {
    let sample = motion_sample();
    let requested_rate = if sample.left == 0 && sample.right == 0 && core.requested_rate(Stream::Motion).is_none() {
        MOTION_IDLE_RATE_CHZ
    } else {
        0
    };
    core.publish_stream(&StreamSample::Motion(sample), requested_rate, now_ms);
}

/// Build the lock-free track speeds.
pub(super) fn motion_sample() -> Motion {
    let (left, right) = motion::get_track_speeds_atomic();
    Motion { left, right }
}

/// Publish the Activity snapshot, borrowing a Manual Drive label when needed.
async fn sample_activity(core: &mut LinkCore, now_ms: u64) {
    let snapshot = activity::snapshot().await;

    // A Manual Drive's detail is derived on the spot into a fixed-capacity
    // buffer, so it is borrowed for the duration of the synchronous publish
    // below and never needs to outlive this function.
    let manual_label = snapshot.manual_drive_label();
    let detail = activity_detail(&snapshot, manual_label.as_ref());

    let sample = WireActivity {
        procedure_id: activity_procedure_id(snapshot.activity),
        stage: activity_stage(snapshot.stage),
        percent: snapshot.percent,
        detail,
    };
    core.publish_stream(&StreamSample::Activity(sample), 0, now_ms);
}

/// The body line for an Activity snapshot, borrowing a Manual Drive label.
///
/// A Manual Drive's detail is derived into a fixed-capacity buffer, so the caller
/// keeps that buffer alive for as long as the returned borrow is used.
#[must_use]
pub(super) fn activity_detail<'a>(snapshot: &activity::Snapshot, manual_label: Option<&'a TextBuf>) -> &'a str {
    match snapshot.activity {
        activity::Activity::ManualDrive(_) => manual_label.map_or("", TextBuf::as_str),
        activity::Activity::Idle | activity::Activity::Booting | activity::Activity::Procedure(_) => snapshot.detail,
    }
}

// ── Mappings ──────────────────────────────────────────────────────────────────

/// The wire id for an Activity, including the Activity-only Idle/Boot ids.
#[must_use]
pub(super) const fn activity_procedure_id(activity: activity::Activity) -> ProcedureId {
    match activity {
        activity::Activity::Idle => ProcedureId::IDLE,
        activity::Activity::Booting => ProcedureId::BOOT,
        activity::Activity::Procedure(procedure) => procedure_id(procedure),
        activity::Activity::ManualDrive(_) => ProcedureId::new(MANUAL_DRIVE_PROCEDURE_ID),
    }
}

/// The wire stage for a firmware Activity stage.
#[must_use]
pub(super) const fn activity_stage(stage: activity::Stage) -> ActivityStage {
    match stage {
        activity::Stage::Running => ActivityStage::Running,
        activity::Stage::Complete => ActivityStage::Complete,
        activity::Stage::Failed => ActivityStage::Failed,
    }
}

/// The wire status for a firmware calibration status.
#[must_use]
const fn wire_calibration_status(status: CalibrationStatus) -> WireCalibrationStatus {
    match status {
        CalibrationStatus::Loaded => WireCalibrationStatus::Loaded,
        CalibrationStatus::NotLoaded => WireCalibrationStatus::Unknown,
        CalibrationStatus::NotAvailable => WireCalibrationStatus::Missing,
    }
}

// ── Unit conversion ───────────────────────────────────────────────────────────

/// Round pack voltage in volts to whole millivolts, below the unknown sentinel.
///
/// `f32::round` is unavailable in `no_std`, so `libm` supplies the rounding. A
/// NaN reading casts to zero rather than to the sentinel, so an unreadable
/// voltage is reported as a real (zero) value only if the power module stored
/// one; a missing reading never reaches here.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn voltage_mv(volts: f32) -> u16 {
    let rounded = libm::roundf(volts * 1000.0);
    rounded.clamp(0.0, f32::from(VOLTAGE_UNKNOWN_MV - 1)) as u16
}

/// Round a distance in centimetres to a whole centimetre below the no-return
/// sentinel.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn whole_cm(distance_cm: f32) -> u16 {
    let rounded = libm::roundf(distance_cm);
    rounded.clamp(0.0, f32::from(LidarCloud::NO_RETURN - 1)) as u16
}
