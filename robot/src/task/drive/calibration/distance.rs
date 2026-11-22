//! Distance calibration: the fixed-distance drive step and the encoder scale it
//! measures.
//!
//! # Flow
//!
//! 1. The calibration task backs up the scale in force and puts the nominal scale
//!    in force with [`begin`], so the commanded distance is the nominal 150 cm
//!    whatever the robot was calibrated with.
//! 2. [`run_drive_step`] counts down, drives the fixed distance, and brakes. It
//!    returns the raw encoder counts accumulated over the leg with
//!    [`DriveOutcome::Complete`].
//! 3. The operator measures how far the robot actually travelled and enters it on
//!    the panel's value screen; the UI sends the answer with [`send_decision`] and
//!    the calibration task's [`commit`] divides the accumulated counts by that
//!    measured distance, or [`abort`] restores the backed-up scale.
//!
//! The counts travel in the drive step's return value rather than a shared slot,
//! so the figure the commit uses belongs to the leg that just ran and cannot race
//! a later drive.
//!
//! A zero or negative measured distance, or a resulting scale outside the model's
//! plausibility band, fails the procedure and saves nothing; cancelling is the
//! screen's explicit Cancel action rather than a magic value.
//!
//! The running screen reads the drive step's phase from the activity state; the
//! step's failure reason goes there too.

use defmt::info;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex};
use motor_cal::{DistanceScale, ScaleError};

use crate::{
    system::state::{activity, calibration},
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder, DriveQueueCompletion, send_drive_command, types::DriveQueueBuildError,
        },
        io::flash_storage,
        procedure::Claim,
    },
};

/// Distance the calibration drives, in centimetres.
pub const DRIVE_DISTANCE_CM: f32 = 150.0;

/// Speed the calibration drives at (0-100).
const DRIVE_SPEED: u8 = 70;

/// Countdown before the drive, in seconds.
const COUNTDOWN_SECONDS: u64 = 3;

/// Settle time after the brake, in milliseconds.
const SETTLE_MS: u64 = 500;

/// The encoder scale that was in force before the calibration started.
///
/// [`PreviousScale::Unset`] means no calibration is in flight.
/// [`PreviousScale::Uncalibrated`] means the robot had no measured scale, so an
/// abort must leave it uncalibrated instead of inventing one. A mutex is enough:
/// begin, commit and abort all run on the one calibration task.
#[derive(Clone, Copy)]
enum PreviousScale {
    /// No run has backed up a scale.
    Unset,
    /// The robot was uncalibrated when the run began.
    Uncalibrated,
    /// This scale was in force when the run began.
    Calibrated(DistanceScale),
}

impl PreviousScale {
    /// The backup state for the scale that was in force.
    const fn of(scale: Option<DistanceScale>) -> Self {
        match scale {
            Some(scale) => Self::Calibrated(scale),
            None => Self::Uncalibrated,
        }
    }
}

/// The scale backup slot.
static PREVIOUS_SCALE: Mutex<CriticalSectionRawMutex, PreviousScale> = Mutex::new(PreviousScale::Unset);

/// What the distance calibration's drive step did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriveOutcome {
    /// The drive finished; the operator now measures and enters the distance.
    Complete {
        /// Raw encoder counts accumulated on the left track over the leg.
        left_counts: u32,
        /// Raw encoder counts accumulated on the right track over the leg.
        right_counts: u32,
    },
    /// The operator stopped the drive; the previous scale was restored.
    Stopped,
    /// The drive could not run; the reason is on the running screen.
    Failed,
}

/// The operator's answer to the value-entry screen.
///
/// The UI sends exactly one of these once the drive step has finished; the
/// calibration task takes it, makes the terminal call and releases the claim.
#[derive(Clone, Copy)]
pub enum DistanceDecision {
    /// Store the scale computed from this measured distance.
    Commit(i32),
    /// Restore the scale that was in force before the run.
    Cancel,
}

/// How long the task waits for a decision before re-checking the stop latch.
const DECISION_POLL_MS: u64 = 1_000;

/// The value-entry screen's decision for the running calibration.
///
/// Capacity one: the screen sends one decision per run, and the task that owns the
/// claim is the only receiver.
static DECISION: Channel<CriticalSectionRawMutex, DistanceDecision, 1> = Channel::new();

/// Send the value-entry screen's decision to the calibration task.
///
/// Called by the UI, which does not hold the claim; the task that does makes the
/// terminal call, so the decision is the only thing that crosses.
pub async fn send_decision(decision: DistanceDecision) {
    DECISION.send(decision).await;
}

/// Wait for the value-entry screen's decision or a stop.
///
/// Called by the calibration task, which holds the claim across the screen. A stop
/// returns `None` so the task can restore the previous scale and finish the
/// claim; the latch is re-checked between decision polls.
pub(super) async fn wait_for_decision_or_stop(claim: &Claim) -> Option<DistanceDecision> {
    loop {
        if claim.is_stop_requested() {
            return None;
        }
        match select(DECISION.receive(), claim.wait_or_stop(DECISION_POLL_MS)).await {
            Either::First(decision) => return Some(decision),
            Either::Second(true) => return None,
            Either::Second(false) => {}
        }
    }
}

/// Read the current scale into the backup slot and put the nominal scale in force.
///
/// The calibration task calls this before [`run_drive_step`], so the leg always
/// runs on the nominal commanded distance whatever scale was in force. The run is
/// measured, not tuned: the nominal scale is the inferred counts/cm, and the leg's
/// accumulated counts are what produce the replacement.
pub async fn begin() {
    let previous = calibration::get_distance_scale().await;
    *PREVIOUS_SCALE.lock().await = PreviousScale::of(previous);

    let nominal = DistanceScale::nominal();
    calibration::set_distance_scale(Some(nominal)).await;

    if let Some(previous) = previous {
        info!(
            "Distance calibration: nominal {=f32} counts/cm in force, was {=f32}",
            nominal.counts_per_cm(),
            previous.counts_per_cm()
        );
    } else {
        info!(
            "Distance calibration: nominal {=f32} counts/cm in force, was uncalibrated",
            nominal.counts_per_cm()
        );
    }
}

/// Commit the scale implied by the leg's accumulated counts and the measured
/// distance.
///
/// Both tracks ran the same leg under heading correction, so their mean count is
/// the centreline count. The scale is published to the calibration leaf and
/// handed to flash storage before the backup slot is cleared, so a Stop after the
/// commit cannot resurrect the old scale.
///
/// # Errors
///
/// Returns [`ScaleError`] when the measured distance is zero or negative, or when
/// the resulting scale is outside the model's plausibility band. Nothing is
/// published or saved in that case; the caller restores the backed-up scale with
/// [`abort`].
// `measured_cm` is a panel-entered whole number of centimetres, far below the
// exact-integer range of `f32`.
#[allow(clippy::cast_precision_loss)]
pub async fn commit(measured_cm: i32, left_counts: u32, right_counts: u32) -> Result<DistanceScale, ScaleError> {
    let counts = u32::midpoint(left_counts, right_counts);
    let scale = DistanceScale::from_measurement(counts, measured_cm as f32)?;

    calibration::set_distance_scale(Some(scale)).await;
    flash_storage::send_flash_command(flash_storage::FlashCommand::SaveDistanceScale(scale)).await;
    *PREVIOUS_SCALE.lock().await = PreviousScale::Unset;

    info!(
        "Distance calibration: {=f32} counts/cm saved from {=u32} counts over {=i32} cm",
        scale.counts_per_cm(),
        counts,
        measured_cm
    );
    Ok(scale)
}

/// Restore the scale that was in force before the run.
///
/// Used when the operator cancels the value screen, stops the drive, or when the
/// commit rejected the measurement, so a calibration that produced nothing does
/// not leave the robot on the nominal scale.
pub async fn abort() {
    let previous = core::mem::replace(&mut *PREVIOUS_SCALE.lock().await, PreviousScale::Unset);
    match previous {
        PreviousScale::Unset => {}
        PreviousScale::Uncalibrated => {
            calibration::set_distance_scale(None).await;
            info!("Distance calibration: restored the uncalibrated state");
        }
        PreviousScale::Calibrated(scale) => {
            calibration::set_distance_scale(Some(scale)).await;
            info!("Distance calibration: restored {=f32} counts/cm", scale.counts_per_cm());
        }
    }
}

/// Run the calibration's fixed-distance drive step under `claim`.
///
/// Counts down, drives [`DRIVE_DISTANCE_CM`] forward, and brakes, publishing each
/// phase to the activity state through the claim. The step is interruptible from
/// the panel: the Stop latches the claim and interrupts the drive, and a stopped
/// or failed step restores the previous scale and makes the claim's terminal
/// call. On success the raw counts are returned and the claim stays held, so the
/// value screen's decision can make the terminal call.
pub async fn run_drive_step(claim: &Claim) -> DriveOutcome {
    claim.phase("Driving 150 cm", None).await;

    for second in 0..COUNTDOWN_SECONDS {
        let elapsed = usize::try_from(second).unwrap_or(0);
        let total = usize::try_from(COUNTDOWN_SECONDS).unwrap_or(1);
        claim
            .phase("Get ready", Some(activity::percent_done(elapsed, total)))
            .await;
        if claim.wait_or_stop(1_000).await {
            return stopped(claim).await;
        }
    }

    claim.phase("Driving 150 cm", None).await;

    let queue = match build_drive_queue() {
        Ok(queue) => queue,
        Err(error) => return failed(claim, error.label()).await,
    };

    let completion = match queue.submit().await {
        Ok(completion) => completion,
        Err(error) => return failed(claim, error.label()).await,
    };

    match completion.status {
        CompletionStatus::Success => {}
        CompletionStatus::Cancelled => return stopped(claim).await,
        CompletionStatus::Failed(reason) => {
            // A leg that did not reach its target cannot calibrate anything, so
            // the reason is what the operator sees instead of the value screen.
            log_completion(&completion);
            return failed(claim, reason).await;
        }
    }

    // The queue executed exactly the one distance command, so a successful
    // completion must carry its counts. Without them there is nothing to divide
    // by the operator's measurement.
    let Some((left_counts, right_counts)) = leg_counts(&completion) else {
        return failed(claim, "No drive telemetry").await;
    };

    // Brake so the robot is still while the operator measures.
    send_drive_command(DriveCommand::Drive(DriveAction::Brake)).await;
    if claim.wait_or_stop(SETTLE_MS).await {
        return stopped(claim).await;
    }

    // The claim stays held: the task publishes the value screen's phase and makes
    // the terminal call once the operator has decided.
    log_completion(&completion);
    DriveOutcome::Complete {
        left_counts,
        right_counts,
    }
}

/// Restore the previous scale, finish the claim, and report the stop.
async fn stopped(claim: &Claim) -> DriveOutcome {
    info!("Distance calibration: drive step stopped");
    abort().await;
    claim.finish().await;
    DriveOutcome::Stopped
}

/// Restore the previous scale, fail the claim with `reason`, and report it.
///
/// Every failure after [`begin`] goes through here: a run that produced no scale
/// must not leave the nominal one in force.
async fn failed(claim: &Claim, reason: &'static str) -> DriveOutcome {
    abort().await;
    claim.fail(reason).await;
    DriveOutcome::Failed
}

/// Build the drive queue: the fixed distance alone, so its telemetry is the
/// queue's last step.
fn build_drive_queue() -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
        kind: DriveDistanceKind::Straight {
            distance_cm: DRIVE_DISTANCE_CM,
        },
        direction: DriveDirection::Forward,
        speed: DRIVE_SPEED,
    }))?;
    Ok(queue)
}

/// The raw encoder counts the completed leg accumulated, one per track.
///
/// `None` when the completion carries no distance telemetry, which a successful
/// one-step queue never does.
fn leg_counts(completion: &DriveQueueCompletion) -> Option<(u32, u32)> {
    let step = completion.last_step_completion.as_ref()?;
    match step.telemetry {
        CompletionTelemetry::DriveDistance {
            achieved_left_counts,
            achieved_right_counts,
            ..
        } => Some((achieved_left_counts, achieved_right_counts)),
        _ => None,
    }
}

/// Log the drive's achieved distance, for the record next to the operator's
/// measurement.
fn log_completion(completion: &DriveQueueCompletion) {
    let Some(step) = completion.last_step_completion.as_ref() else {
        return;
    };
    if let CompletionTelemetry::DriveDistance {
        achieved_left_revs,
        achieved_right_revs,
        achieved_left_counts,
        achieved_right_counts,
        duration_ms,
        ..
    } = step.telemetry
    {
        info!(
            "Distance calibration: drove L={=f32} R={=f32} revs (L={=u32} R={=u32} counts) in {=u64} ms",
            achieved_left_revs, achieved_right_revs, achieved_left_counts, achieved_right_counts, duration_ms
        );
    }
    if let CompletionStatus::Failed(reason) = step.status {
        info!("Distance calibration: drive failed: {=str}", reason);
    }
}
