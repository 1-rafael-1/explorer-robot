//! The distance and continuous drive control law.
//!
//! This module owns the `DriveDistance` and `DriveContinuous` intents, which
//! share one controller: encoder progress, the stall/encoder-timeout/anomaly
//! checks, the speed ramp and the IMU heading or curve-yaw correction. Only the
//! end condition differs — a target drive completes inside a tolerance, a
//! continuous drive never does.
//!
//! # Geometry
//!
//! A curve arc's tracks follow different radii:
//!
//! - `left_arc = arc_length × left_radius / radius`
//! - `right_arc = arc_length × right_radius / radius`
//!
//! The target revolutions are `arc / sprocket_circumference`, and the speed
//! ratios are each target over the larger target, so the dominant track runs at
//! `1.0` and the inner track is scaled down.
//!
//! # Curves
//!
//! The expected yaw follows from the encoder distances with the
//! differential-drive approximation `expected_yaw_rad = ±(right_cm −
//! left_cm) / track_width_cm`, negated when driving in reverse. The IMU yaw is
//! accumulated (wrapped into `[-180, 180]` per sample) and compared against that
//! expectation; the proportional correction is clamped and applied
//! symmetrically to the two ratios.
//!
//! # Straight lines
//!
//! Straight drives capture a heading reference on the first yaw sample and hold
//! it with a proportional heading correction, clamped by the current ramp speed.
//! Without a fresh yaw sample the drive keeps running open-loop on the encoders;
//! there is no IMU-loss fault.
//!
//! It is a pure step machine. It never reads a clock, starts a sensor, commands
//! a motor or allocates: every timestamp comes from the [`Observation`], and
//! every action is returned to the adapter to execute. A sleep becomes
//! [`IntentAction::WaitUntil`].

use drive_geometry::{SPROCKET_CIRCUMFERENCE_CM, TRACK_WIDTH_CM};
use libm::roundf;
use motor_cal::DistanceScale;

use crate::{
    CompletionTelemetry, DriveDirection, DriveDistanceKind, DriveFault, DriveOutcome, EncoderSample, IntentAction,
    Observation, Step, TurnDirection, YawSample,
};

// ── Distance control constants ───────────────────────────────────────────────

/// Distance drive ramp-down begins when remaining revolutions drop below this value.
const RAMP_DOWN_START_REVS: f32 = 0.25;
/// Minimum speed during ramp-down (0-100).
const MIN_SPEED: u8 = 40;
/// Maximum speed clamp for distance driving (0-100).
const MAX_SPEED: u8 = 100;
/// Completion tolerance for distance driving (revolutions).
const TOLERANCE_REVS: f32 = 0.01;
/// Distance drive control interval, and the adapter's tick, in milliseconds.
const CONTROL_INTERVAL_MS: u64 = 20;
/// Encoder timeout during distance driving (milliseconds).
const ENCODER_TIMEOUT_MS: u64 = 300;
/// Curve yaw correction proportional gain (radians -> ratio).
const CURVE_YAW_KP: f32 = 0.5;
/// Maximum absolute curve yaw correction applied to speed ratio.
const CURVE_YAW_MAX_CORRECTION: f32 = 0.25;
/// Proportional gain for IMU heading correction on straight drives.
const STRAIGHT_IMU_KP: f32 = 2.0;
/// Maximum absolute heading correction (speed units) before scaling.
const STRAIGHT_IMU_MAX_CORRECTION: f32 = 8.0;
/// Correction clamp scales with current ramp speed: `min(MAX, speed * SCALE)`.
const STRAIGHT_IMU_CORRECTION_SCALE: f32 = 0.15;
/// Stall timeout during distance driving (milliseconds).
const STALL_TIMEOUT_MS: u64 = 1500;
/// Upper bound on the initial heading-reference wait (milliseconds).
const HEADING_REFERENCE_WAIT_MS: u64 = 50;
/// Poll interval inside the heading-reference wait (milliseconds).
const HEADING_REFERENCE_POLL_MS: u64 = 10;

// ── Pure geometry and encoder helpers ────────────────────────────────────────

/// Convert a travelled distance in centimetres into sprocket revolutions.
///
/// The scale cancels algebraically: `cm × counts/cm / (counts/cm × C) = cm / C`.
/// Keeping it in the expression ties this conversion to the same counts-per-rev
/// divisor that turns accumulated counts into revolutions, so both sides of a
/// drive use one figure.
const fn to_revs(cm: f32, counts_per_cm: f32, counts_per_rev: f32) -> f32 {
    cm * counts_per_cm / counts_per_rev
}

/// 16-bit counter delta with wraparound handling.
const fn calculate_delta_u16(current: u16, previous: u16) -> u16 {
    if current >= previous {
        current - previous
    } else {
        (u16::MAX - previous).wrapping_add(current).wrapping_add(1)
    }
}

/// Convert a speed magnitude and direction into a signed track speed.
#[allow(clippy::cast_possible_wrap)]
const fn signed_speed(speed: u8, direction: DriveDirection) -> i8 {
    let magnitude = if speed < MAX_SPEED { speed } else { MAX_SPEED } as i8;
    match direction {
        DriveDirection::Forward => magnitude,
        DriveDirection::Reverse => -magnitude,
    }
}

/// Compute the ramp-down speed for a drive approaching its target.
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
fn ramp_speed(base_speed: u8, remaining: f32) -> u8 {
    if remaining <= RAMP_DOWN_START_REVS {
        let factor = (remaining / RAMP_DOWN_START_REVS).clamp(0.0, 1.0);
        let scaled = roundf(f32::from(base_speed) * factor) as u8;
        scaled.clamp(MIN_SPEED, MAX_SPEED)
    } else {
        base_speed.min(MAX_SPEED)
    }
}

/// The next poll inside the heading-reference wait, never past its bound.
const fn heading_poll_deadline(now_ms: u64, deadline_ms: u64) -> u64 {
    let polled = now_ms.saturating_add(HEADING_REFERENCE_POLL_MS);
    if polled < deadline_ms { polled } else { deadline_ms }
}

// ── Step machine ─────────────────────────────────────────────────────────────

/// How a distance drive ends.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Mode {
    /// Complete when the encoder progress reaches the distance target.
    Target,
    /// Run until interrupted: no target, no completion check, no ramp-down.
    Continuous,
}

/// Where the step machine is between adapter actions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Phase {
    /// The initial heading capture and track command have not been emitted yet.
    Start,
    /// Waiting for a heading sample until `deadline_ms`, then starting anyway.
    HeadingWait {
        /// Absolute end of the heading-reference wait.
        deadline_ms: u64,
    },
    /// Waiting for the next control tick at `deadline_ms`.
    Control {
        /// Absolute time at which the next control tick may begin.
        deadline_ms: u64,
    },
    /// The completion stop has been emitted; the terminal `Finished` follows.
    Stopping,
    /// The terminal `Finished` action has been emitted.
    Finished,
}

/// What one encoder read produced.
enum EncoderRead {
    /// A fresh, non-zero delta on both tracks was accumulated.
    Progress,
    /// No fresh sample, a stale one, or a zero delta: no motor update this tick.
    NoUpdate,
    /// A terminal encoder fault was detected.
    Failed(DriveFault),
}

/// Private state of a distance or continuous intent.
pub struct DistanceState {
    /// How this drive ends: at its target, or only when interrupted.
    mode: Mode,
    /// Straight or curved distance specification.
    kind: DriveDistanceKind,
    /// Drive direction (forward or reverse).
    direction: DriveDirection,
    /// Base speed magnitude (0-100).
    base_speed: u8,
    /// Target revolutions for the left track sprocket.
    target_left_revs: f32,
    /// Target revolutions for the right track sprocket.
    target_right_revs: f32,
    /// Whether the left track is the inner (shorter) side for curves.
    inner_left: Option<bool>,
    /// Target revolutions for the inner track sprocket.
    target_inner_revs: f32,
    /// Left speed scale relative to the max target (1.0 for the dominant track).
    left_ratio: f32,
    /// Right speed scale relative to the max target (1.0 for the dominant track).
    right_ratio: f32,
    /// Accumulated left revolutions.
    accumulated_left_revs: f32,
    /// Accumulated right revolutions.
    accumulated_right_revs: f32,
    /// Raw encoder counts accumulated on the left track.
    accumulated_left_counts: u32,
    /// Raw encoder counts accumulated on the right track.
    accumulated_right_counts: u32,
    /// Encoder counts per sprocket revolution, from the measured counts/cm and
    /// the sprocket circumference.
    counts_per_rev: f32,
    /// Last IMU yaw sample for curve control (degrees).
    curve_last_yaw_deg: Option<f32>,
    /// Accumulated curve yaw delta (degrees).
    curve_accumulated_yaw_deg: f32,
    /// IMU reference yaw captured at the first valid sample for straight-line
    /// heading control.
    reference_yaw: Option<f32>,
    /// Newest yaw sample seen, for the feature-gated trace.
    #[cfg_attr(not(feature = "trace"), allow(dead_code))]
    last_yaw_deg: Option<f32>,
    /// Last time we observed forward progress (ms).
    last_progress_ms: u64,
    /// Timestamp of the last processed encoder measurement (ms).
    last_encoder_timestamp_ms: u64,
    /// Last time we saw a new encoder measurement (ms).
    last_encoder_seen_ms: u64,
    /// Previous encoder measurement for computing deltas.
    last_encoder_measurement: Option<EncoderSample>,
    /// Start time (ms) used for duration telemetry.
    started_at_ms: u64,
    /// Where the step machine is between adapter actions.
    phase: Phase,
    /// The terminal outcome, recorded when the completion stop is emitted and
    /// published when `Finished` is emitted.
    pending_outcome: Option<DriveOutcome>,
    /// The terminal outcome, present only once `Finished` has been emitted.
    outcome: Option<DriveOutcome>,
}

impl DistanceState {
    /// Create a target drive and precompute its targets and speed ratios.
    pub const fn new_target(
        kind: DriveDistanceKind,
        direction: DriveDirection,
        base_speed: u8,
        scale: DistanceScale,
        started_at_ms: u64,
    ) -> Self {
        let counts_per_cm = scale.counts_per_cm();
        // The target is `cm × counts/cm`; converting it to sprocket revolutions
        // divides by the counts per revolution the same scale implies. The
        // accumulated counts are divided by that same figure, so a wrong scale
        // still moves the robot the wrong distance, and no hard-coded
        // pulses-per-revolution constant remains in the drive path.
        let counts_per_rev = counts_per_cm * SPROCKET_CIRCUMFERENCE_CM;

        let (target_left_revs, target_right_revs, inner_left, target_inner_revs) = match kind {
            DriveDistanceKind::Straight { distance_cm } => {
                let revolutions = to_revs(distance_cm, counts_per_cm, counts_per_rev);
                (revolutions, revolutions, None, revolutions)
            }
            DriveDistanceKind::CurveArc {
                radius_cm,
                arc_length_cm,
                direction: turn,
            } => {
                let half_width = TRACK_WIDTH_CM * 0.5;
                let inner_radius = (radius_cm - half_width).max(0.0);
                let outer_radius = radius_cm + half_width;
                let (left_radius, right_radius, inner_left) = match turn {
                    TurnDirection::Left => (inner_radius, outer_radius, true),
                    TurnDirection::Right => (outer_radius, inner_radius, false),
                };
                let safe_radius = radius_cm.max(0.001);
                let left_arc_cm = arc_length_cm * (left_radius / safe_radius);
                let right_arc_cm = arc_length_cm * (right_radius / safe_radius);
                let left_revs = to_revs(left_arc_cm, counts_per_cm, counts_per_rev);
                let right_revs = to_revs(right_arc_cm, counts_per_cm, counts_per_rev);
                let inner_revs = if inner_left { left_revs } else { right_revs };
                (left_revs, right_revs, Some(inner_left), inner_revs)
            }
        };

        let max_target = target_left_revs.max(target_right_revs);
        let left_ratio = if max_target > 0.0 {
            target_left_revs / max_target
        } else {
            1.0
        };
        let right_ratio = if max_target > 0.0 {
            target_right_revs / max_target
        } else {
            1.0
        };

        Self {
            mode: Mode::Target,
            kind,
            direction,
            base_speed,
            target_left_revs,
            target_right_revs,
            inner_left,
            target_inner_revs,
            left_ratio,
            right_ratio,
            accumulated_left_revs: 0.0,
            accumulated_right_revs: 0.0,
            accumulated_left_counts: 0,
            accumulated_right_counts: 0,
            counts_per_rev,
            curve_last_yaw_deg: None,
            curve_accumulated_yaw_deg: 0.0,
            reference_yaw: None,
            last_yaw_deg: None,
            last_progress_ms: started_at_ms,
            last_encoder_timestamp_ms: 0,
            last_encoder_seen_ms: started_at_ms,
            last_encoder_measurement: None,
            started_at_ms,
            phase: Phase::Start,
            pending_outcome: None,
            outcome: None,
        }
    }

    /// Create a continuous drive: the straight controller with no target.
    ///
    /// Built through [`Self::new_target`] so it shares the straight controller's
    /// geometry. A continuous drive never consults the target, the progress
    /// ratios or the telemetry, so the nominal scale cannot affect any
    /// observable behaviour.
    pub const fn new_continuous(direction: DriveDirection, base_speed: u8, started_at_ms: u64) -> Self {
        let mut state = Self::new_target(
            DriveDistanceKind::Straight { distance_cm: 0.0 },
            direction,
            base_speed,
            DistanceScale::nominal(),
            started_at_ms,
        );
        state.mode = Mode::Continuous;
        state
    }

    /// Advance the intent one step and return the action the adapter executes.
    pub fn step(&mut self, obs: &Observation) -> Step {
        match self.phase {
            Phase::Start => self.start(obs),
            Phase::HeadingWait { deadline_ms } => self.heading_wait(obs, deadline_ms),
            Phase::Control { deadline_ms } => {
                if obs.now_ms < deadline_ms {
                    self.make_step(IntentAction::WaitUntil { deadline_ms })
                } else {
                    self.control_tick(obs)
                }
            }
            Phase::Stopping => {
                self.outcome = self.pending_outcome.take();
                self.phase = Phase::Finished;
                self.make_step(IntentAction::Finished)
            }
            Phase::Finished => self.make_step(IntentAction::Finished),
        }
    }

    /// The terminal outcome, once the `Finished` action has been emitted.
    pub fn outcome(&self) -> Option<DriveOutcome> {
        self.outcome.clone()
    }

    /// Telemetry for an intent that the adapter cancels at `now_ms`.
    pub const fn cancellation_telemetry(&self, now_ms: u64) -> CompletionTelemetry {
        self.make_telemetry(now_ms)
    }

    /// Emit the initial action: finish a trivial target, or start the tracks
    /// after waiting up to the heading-reference bound for a yaw sample.
    fn start(&mut self, obs: &Observation) -> Step {
        // Early exit for a target inside the completion tolerance: finish with
        // zero progress before any motor command.
        if self.mode == Mode::Target && self.target_inner_revs <= TOLERANCE_REVS {
            self.outcome = Some(DriveOutcome::Completed {
                telemetry: CompletionTelemetry::DriveDistance {
                    achieved_left_revs: 0.0,
                    achieved_right_revs: 0.0,
                    achieved_left_counts: 0,
                    achieved_right_counts: 0,
                    target_left_revs: self.target_left_revs,
                    target_right_revs: self.target_right_revs,
                    duration_ms: 0,
                },
            });
            self.phase = Phase::Finished;
            return self.make_step(IntentAction::Finished);
        }

        if let Some(sample) = obs.yaw {
            self.capture_reference(sample);
            return self.apply_initial_tracks(obs.now_ms);
        }

        let deadline_ms = self.started_at_ms.saturating_add(HEADING_REFERENCE_WAIT_MS);
        if obs.now_ms >= deadline_ms {
            return self.apply_initial_tracks(obs.now_ms);
        }

        self.phase = Phase::HeadingWait { deadline_ms };
        self.make_step(IntentAction::WaitUntil {
            deadline_ms: heading_poll_deadline(obs.now_ms, deadline_ms),
        })
    }

    /// Drain one poll of the heading-reference wait and start when it ends.
    ///
    /// The deadline is checked first, so the wait ends without reading a sample
    /// that arrives at exactly the bound; that sample is left for the first
    /// control tick.
    fn heading_wait(&mut self, obs: &Observation, deadline_ms: u64) -> Step {
        if obs.now_ms >= deadline_ms {
            return self.apply_initial_tracks(obs.now_ms);
        }

        if let Some(sample) = obs.yaw {
            self.capture_reference(sample);
            return self.apply_initial_tracks(obs.now_ms);
        }

        self.make_step(IntentAction::WaitUntil {
            deadline_ms: heading_poll_deadline(obs.now_ms, deadline_ms),
        })
    }

    /// Record the heading reference from a yaw sample.
    const fn capture_reference(&mut self, sample: YawSample) {
        self.last_yaw_deg = Some(sample.yaw_deg);
        self.reference_yaw = Some(sample.yaw_deg);
    }

    /// Command the base speed scaled by the track ratios and wait one tick.
    #[allow(clippy::cast_possible_truncation)]
    fn apply_initial_tracks(&mut self, now_ms: u64) -> Step {
        let signed_base = signed_speed(self.base_speed, self.direction);
        let left = roundf(f32::from(signed_base) * self.left_ratio) as i8;
        let right = roundf(f32::from(signed_base) * self.right_ratio) as i8;

        self.phase = Phase::Control {
            deadline_ms: now_ms.saturating_add(CONTROL_INTERVAL_MS),
        };
        self.make_step(IntentAction::SetTracks { left, right })
    }

    /// Run one control tick: read encoders, finish or ramp, correct, command.
    fn control_tick(&mut self, obs: &Observation) -> Step {
        let now_ms = obs.now_ms;

        match self.read_encoder(now_ms, obs.encoder) {
            EncoderRead::Failed(fault) => return self.fail(fault, now_ms),
            EncoderRead::NoUpdate => {
                let deadline_ms = now_ms.saturating_add(CONTROL_INTERVAL_MS);
                self.phase = Phase::Control { deadline_ms };
                return self.make_step(IntentAction::WaitUntil { deadline_ms });
            }
            EncoderRead::Progress => {}
        }

        // Only a target drive can finish and ramp.
        let base_speed = match self.mode {
            Mode::Target => {
                let remaining = (self.target_inner_revs - self.inner_progress()).max(0.0);
                if remaining <= TOLERANCE_REVS {
                    let telemetry = self.make_telemetry(now_ms);
                    self.pending_outcome = Some(DriveOutcome::Completed { telemetry });
                    self.phase = Phase::Stopping;
                    return self.make_step(IntentAction::SetTracks { left: 0, right: 0 });
                }
                ramp_speed(self.base_speed, remaining)
            }
            // No target: run at the commanded speed until an interrupt arrives.
            Mode::Continuous => self.base_speed.min(MAX_SPEED),
        };

        let latest_imu = self.drain_imu(obs.yaw);
        let signed_base = signed_speed(base_speed, self.direction);
        let (left, right) = self.apply_imu_corrections(latest_imu, signed_base);

        let deadline_ms = now_ms.saturating_add(CONTROL_INTERVAL_MS);
        self.phase = Phase::Control { deadline_ms };
        self.make_step(IntentAction::SetTracks { left, right })
    }

    /// Read a fresh encoder sample, compute deltas and check stall and anomaly.
    fn read_encoder(&mut self, now_ms: u64, sample: Option<EncoderSample>) -> EncoderRead {
        if now_ms.saturating_sub(self.last_encoder_seen_ms) >= ENCODER_TIMEOUT_MS {
            return EncoderRead::Failed(DriveFault::EncoderTimeout);
        }

        let Some(measurement) = sample else {
            return EncoderRead::NoUpdate;
        };
        if measurement.timestamp_ms == 0 || measurement.timestamp_ms == self.last_encoder_timestamp_ms {
            return EncoderRead::NoUpdate;
        }
        self.last_encoder_timestamp_ms = measurement.timestamp_ms;
        self.last_encoder_seen_ms = now_ms;

        // Need two samples to compute deltas.
        let Some(previous) = self.last_encoder_measurement else {
            self.last_encoder_measurement = Some(measurement);
            return EncoderRead::NoUpdate;
        };

        let left_delta = calculate_delta_u16(measurement.left, previous.left);
        let right_delta = calculate_delta_u16(measurement.right, previous.right);
        self.last_encoder_measurement = Some(measurement);

        if left_delta == 0 && right_delta == 0 {
            if now_ms.saturating_sub(self.last_progress_ms) >= STALL_TIMEOUT_MS {
                return EncoderRead::Failed(DriveFault::Stall);
            }
            return EncoderRead::NoUpdate;
        }

        // Exactly one track still: a single-motor-zero anomaly.
        if left_delta == 0 || right_delta == 0 {
            return EncoderRead::Failed(DriveFault::EncoderAnomaly);
        }

        self.last_progress_ms = now_ms;
        self.accumulated_left_revs += f32::from(left_delta) / self.counts_per_rev;
        self.accumulated_right_revs += f32::from(right_delta) / self.counts_per_rev;
        self.accumulated_left_counts = self.accumulated_left_counts.saturating_add(u32::from(left_delta));
        self.accumulated_right_counts = self.accumulated_right_counts.saturating_add(u32::from(right_delta));

        EncoderRead::Progress
    }

    /// Fold the newest yaw sample into the heading or curve state.
    ///
    /// The observation carries only the newest sample, so there is nothing to
    /// drain. Returns that sample for the correction.
    fn drain_imu(&mut self, sample: Option<YawSample>) -> Option<YawSample> {
        let sample = sample?;
        self.last_yaw_deg = Some(sample.yaw_deg);

        match self.kind {
            DriveDistanceKind::Straight { .. } => {
                if self.reference_yaw.is_none() {
                    self.reference_yaw = Some(sample.yaw_deg);
                }
            }
            DriveDistanceKind::CurveArc { .. } => {
                if let Some(last_yaw) = self.curve_last_yaw_deg {
                    let mut delta = sample.yaw_deg - last_yaw;
                    if delta > 180.0 {
                        delta -= 360.0;
                    } else if delta < -180.0 {
                        delta += 360.0;
                    }
                    self.curve_accumulated_yaw_deg += delta;
                }
                self.curve_last_yaw_deg = Some(sample.yaw_deg);
            }
        }

        Some(sample)
    }

    /// Apply the curve-ratio correction or the straight heading correction.
    #[allow(clippy::cast_possible_truncation)]
    fn apply_imu_corrections(&self, latest_imu: Option<YawSample>, signed_base: i8) -> (i8, i8) {
        let mut left_ratio = self.left_ratio;
        let mut right_ratio = self.right_ratio;

        if matches!(self.kind, DriveDistanceKind::CurveArc { .. }) && self.curve_last_yaw_deg.is_some() {
            let left_cm = self.accumulated_left_revs * SPROCKET_CIRCUMFERENCE_CM;
            let right_cm = self.accumulated_right_revs * SPROCKET_CIRCUMFERENCE_CM;
            let direction_sign = match self.direction {
                DriveDirection::Forward => 1.0,
                DriveDirection::Reverse => -1.0,
            };
            let expected_yaw_rad = direction_sign * (right_cm - left_cm) / TRACK_WIDTH_CM;
            let actual_yaw_rad = self.curve_accumulated_yaw_deg.to_radians();
            let yaw_error = expected_yaw_rad - actual_yaw_rad;
            let correction = (CURVE_YAW_KP * yaw_error).clamp(-CURVE_YAW_MAX_CORRECTION, CURVE_YAW_MAX_CORRECTION);
            left_ratio = (left_ratio - correction).clamp(0.0, 1.0);
            right_ratio = (right_ratio + correction).clamp(0.0, 1.0);
        }

        let left_speed = roundf(f32::from(signed_base) * left_ratio) as i8;
        let right_speed = roundf(f32::from(signed_base) * right_ratio) as i8;

        if matches!(self.kind, DriveDistanceKind::Straight { .. })
            && let (Some(reference_yaw), Some(sample)) = (self.reference_yaw, latest_imu)
        {
            let mut heading_error = sample.yaw_deg - reference_yaw;
            if heading_error > 180.0 {
                heading_error -= 360.0;
            } else if heading_error <= -180.0 {
                heading_error += 360.0;
            }
            let ramp_speed = signed_base.unsigned_abs();
            let max_correction =
                (f32::from(ramp_speed) * STRAIGHT_IMU_CORRECTION_SCALE).min(STRAIGHT_IMU_MAX_CORRECTION);
            let correction = (STRAIGHT_IMU_KP * heading_error).clamp(-max_correction, max_correction);
            let corrected_left = (roundf(f32::from(left_speed) + correction) as i8).clamp(-100, 100);
            let corrected_right = (roundf(f32::from(right_speed) - correction) as i8).clamp(-100, 100);
            return (corrected_left, corrected_right);
        }

        (left_speed, right_speed)
    }

    /// Build the telemetry snapshot for a completion, failure or cancellation.
    const fn make_telemetry(&self, now_ms: u64) -> CompletionTelemetry {
        match self.mode {
            Mode::Target => CompletionTelemetry::DriveDistance {
                achieved_left_revs: self.accumulated_left_revs,
                achieved_right_revs: self.accumulated_right_revs,
                achieved_left_counts: self.accumulated_left_counts,
                achieved_right_counts: self.accumulated_right_counts,
                target_left_revs: self.target_left_revs,
                target_right_revs: self.target_right_revs,
                duration_ms: now_ms.saturating_sub(self.started_at_ms),
            },
            Mode::Continuous => CompletionTelemetry::None,
        }
    }

    /// Compute inner-track progress (the limiting track for completion).
    const fn inner_progress(&self) -> f32 {
        match self.kind {
            DriveDistanceKind::Straight { .. } => self.accumulated_left_revs.min(self.accumulated_right_revs),
            DriveDistanceKind::CurveArc { .. } => match self.inner_left {
                Some(true) => self.accumulated_left_revs,
                Some(false) => self.accumulated_right_revs,
                None => self.accumulated_left_revs.midpoint(self.accumulated_right_revs),
            },
        }
    }

    /// Record a terminal failure and emit `Finished`.
    ///
    /// The model emits no stop: the adapter's teardown stops the tracks after a
    /// terminal fault, so a fault needs no motor command here.
    fn fail(&mut self, fault: DriveFault, now_ms: u64) -> Step {
        self.outcome = Some(DriveOutcome::Failed {
            fault,
            telemetry: self.make_telemetry(now_ms),
        });
        self.phase = Phase::Finished;
        self.make_step(IntentAction::Finished)
    }

    /// Build a step, attaching a trace snapshot only when the `trace` feature is on.
    #[allow(clippy::unused_self, clippy::missing_const_for_fn)]
    fn make_step(&self, action: IntentAction) -> Step {
        #[cfg(feature = "trace")]
        {
            Step {
                action,
                trace: Some(self.trace(action)),
            }
        }
        #[cfg(not(feature = "trace"))]
        {
            Step { action, trace: None }
        }
    }

    /// A snapshot of the decision behind `action`.
    ///
    /// A distance intent has no remaining angle, so `remaining_deg` is zero; the
    /// latest yaw, the captured heading reference, the track speeds and the wait
    /// deadline are the useful fields.
    #[cfg(feature = "trace")]
    const fn trace(&self, action: IntentAction) -> crate::Trace {
        let (left, right) = match action {
            IntentAction::SetTracks { left, right } => (Some(left), Some(right)),
            _ => (None, None),
        };
        let deadline_ms = match action {
            IntentAction::WaitUntil { deadline_ms } => Some(deadline_ms),
            _ => None,
        };

        crate::Trace {
            yaw_deg: self.last_yaw_deg,
            // Only a straight drive has a heading target; a curve steers on the
            // encoder expectation and the accumulated IMU yaw instead.
            target_yaw_deg: match self.kind {
                DriveDistanceKind::Straight { .. } => self.reference_yaw,
                DriveDistanceKind::CurveArc { .. } => None,
            },
            remaining_deg: 0.0,
            left,
            right,
            deadline_ms,
        }
    }
}
