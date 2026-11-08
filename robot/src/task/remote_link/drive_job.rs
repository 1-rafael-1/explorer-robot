//! The long-lived Remote drive job: one volatile drive command at a time.
//!
//! A Remote `DRIVE_DISTANCE` or `ROTATE_BY` becomes a visible Manual Drive at
//! the robot. [`start`] takes the one Active Claim (ADR-0034), maps the command
//! onto a [`ManualDrive`] Activity label, and hands `(claim, command_seq,
//! action)` to [`drive_job_task`] through a capacity-1 channel. The Link Task
//! returns from the hand-off immediately, so RX, the Event drain and the lease
//! keep running while the drive is in flight.
//!
//! The job is queue-shaped even though v1 has one step: it builds a
//! [`DriveQueueBuilder`], submits it, maps the queue completion onto the wire
//! completion Event with the originating command sequence, and releases the
//! claim. A stop or the Deadman reaches it through the shared drive interrupt,
//! which makes the queue resolve `Cancelled`.
//!
//! A Manual Drive leaves no Result Report (ADR-0035): its outcome is the
//! completion Event, and its terminal claim call is always [`Claim::finish`],
//! which clears the Activity.

use defmt::{info, warn};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use link_protocol::{Status, payload::event::CompletionStatus as WireCompletionStatus};

use super::events::{self, LinkedEvent};
use crate::{
    system::state::activity::ManualDrive,
    task::{
        drive::{
            CompletionStatus, CompletionTelemetry, DriveAction, DriveCommand, DriveDirection, DriveDistanceKind,
            DriveQueueBuilder,
            types::{DriveQueueBuildError, RotationDirection, RotationMotion, SPROCKET_CIRCUMFERENCE_CM},
        },
        procedure::{self, Claim, ClaimKind, ClaimOrigin},
    },
};

// ── Request channel ───────────────────────────────────────────────────────────

/// One volatile drive command the Remote asked for.
///
/// The units are the wire's: signed centimetres for a distance and signed
/// centidegrees for a rotation, with positive rotation clockwise.
#[derive(Clone, Copy, Debug)]
pub enum RemoteDriveAction {
    /// Travel `cm` centimetres at `speed` percent.
    Distance {
        /// Signed distance in centimetres; positive is forward.
        cm: i16,
        /// Speed magnitude (0-100).
        speed: u8,
    },
    /// Rotate by `cdeg` centidegrees at `speed` percent.
    Rotate {
        /// Signed angle in centidegrees; positive is clockwise.
        cdeg: i16,
        /// Speed magnitude (0-100).
        speed: u8,
    },
}

impl RemoteDriveAction {
    /// The Manual Drive Activity label this command publishes.
    #[must_use]
    fn manual_drive(self) -> ManualDrive {
        match self {
            Self::Distance { cm, .. } => ManualDrive::distance(i32::from(cm)),
            Self::Rotate { cdeg, .. } => ManualDrive::rotate_centidegrees(i32::from(cdeg)),
        }
    }
}

/// One hand-off from the Link Task to the drive job.
type DriveJobRequest = (Claim, u16, RemoteDriveAction);

/// Capacity-1 hand-off channel.
///
/// The claim is taken before the send, so a second drive command is `BUSY` at
/// the claim; the capacity is only a second line of defence for the window
/// between a `finish` and the job's next receive.
static DRIVE_JOB_REQUESTS: Channel<CriticalSectionRawMutex, DriveJobRequest, 1> = Channel::new();

/// A telemetry payload for a queue that produced none.
static NONE_TELEMETRY: CompletionTelemetry = CompletionTelemetry::None;

// ── Public surface ────────────────────────────────────────────────────────────

/// Take the Manual Drive claim and hand the command to the drive job.
///
/// Returns [`Status::Ok`] once the job owns the request and the Link Task may
/// acknowledge; [`Status::Busy`] when another claim is held or the hand-off
/// channel refuses. A refused hand-off abandons the claim it just took, so the
/// slot is never left held with no owner.
#[must_use]
pub async fn start(action: RemoteDriveAction, command_seq: u16) -> Status {
    let Some(claim) = procedure::claim(ClaimKind::ManualDrive(action.manual_drive()), ClaimOrigin::Remote).await else {
        return Status::Busy;
    };

    if DRIVE_JOB_REQUESTS.try_send((claim, command_seq, action)).is_err() {
        warn!("[link] drive job busy; refusing command");
        claim.abandon().await;
        return Status::Busy;
    }

    Status::Ok
}

// ── Task ──────────────────────────────────────────────────────────────────────

/// The one long-lived Remote drive job.
///
/// Receives one hand-off at a time, runs its queue to completion, publishes the
/// completion Event, and releases the claim. It never consults the link: the
/// job outlives Link Port enable/disable periods.
#[embassy_executor::task]
pub async fn drive_job_task() {
    loop {
        let (claim, command_seq, action) = DRIVE_JOB_REQUESTS.receive().await;
        run(command_seq, action).await;
        // Manual Drive has no Result Report; for every outcome the terminal
        // call clears the Activity and releases the slot.
        claim.finish().await;
    }
}

/// Run one command as a one-step queue and publish its completion Event.
///
/// A build or submit failure is reported as the queue's failed completion, so
/// the Remote still gets its originating sequence and a reason.
async fn run(command_seq: u16, action: RemoteDriveAction) {
    info!("[link] Manual Drive start seq {}", command_seq);

    let queue = match build_queue(action) {
        Ok(queue) => queue,
        Err(error) => {
            warn!("[link] drive queue build failed: {}", error.label());
            publish_outcome(
                command_seq,
                action,
                &CompletionStatus::Failed(error.label()),
                &NONE_TELEMETRY,
            );
            return;
        }
    };

    let completion = match queue.submit().await {
        Ok(completion) => completion,
        Err(error) => {
            warn!("[link] drive queue submit failed: {}", error.label());
            publish_outcome(
                command_seq,
                action,
                &CompletionStatus::Failed(error.label()),
                &NONE_TELEMETRY,
            );
            return;
        }
    };

    let telemetry = completion
        .last_step_completion
        .as_ref()
        .map_or(&NONE_TELEMETRY, |step| &step.telemetry);
    publish_outcome(command_seq, action, &completion.status, telemetry);
}

// ── Queue construction ────────────────────────────────────────────────────────

/// Build the one-step queue for `action`.
///
/// The queue is shaped like every other drive producer's: a single abort-on-fail
/// step whose completion telemetry is the queue's last step.
fn build_queue(action: RemoteDriveAction) -> Result<DriveQueueBuilder, DriveQueueBuildError> {
    let mut queue = DriveQueueBuilder::new();
    match action {
        RemoteDriveAction::Distance { cm, speed } => {
            queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::DriveDistance {
                kind: DriveDistanceKind::Straight {
                    distance_cm: f32::from(cm.unsigned_abs()),
                },
                direction: if cm >= 0 {
                    DriveDirection::Forward
                } else {
                    DriveDirection::Backward
                },
                speed,
            }))?;
        }
        RemoteDriveAction::Rotate { cdeg, speed } => {
            // The wire says positive is clockwise; the drive subsystem uses the
            // same convention, so the sign is passed through rather than inverted.
            queue.push_abort_on_fail(DriveCommand::Drive(DriveAction::RotateExact {
                degrees: f32::from(cdeg.unsigned_abs()) / 100.0,
                direction: if cdeg >= 0 {
                    RotationDirection::Clockwise
                } else {
                    RotationDirection::CounterClockwise
                },
                motion: RotationMotion::Stationary { speed },
            }))?;
        }
    }
    Ok(queue)
}

// ── Completion Event ──────────────────────────────────────────────────────────

/// Build and enqueue the wire completion Event for one queue outcome.
fn publish_outcome(
    command_seq: u16,
    action: RemoteDriveAction,
    status: &CompletionStatus,
    telemetry: &CompletionTelemetry,
) {
    let (wire_status, detail) = outcome(status);

    let event = match action {
        RemoteDriveAction::Distance { cm, .. } => LinkedEvent::DriveDistanceCompleted {
            command_seq,
            status: wire_status,
            target_cm: cm,
            distance_cm: achieved_distance_cm(cm, telemetry),
            duration_ms: duration_ms(telemetry),
            detail,
        },
        RemoteDriveAction::Rotate { cdeg, .. } => LinkedEvent::RotateCompleted {
            command_seq,
            status: wire_status,
            target_cdeg: cdeg,
            error_cdeg: rotate_error_cdeg(telemetry),
            duration_ms: duration_ms(telemetry),
            detail,
        },
    };

    events::publish(event);
}

/// The wire status and operator detail for a drive outcome.
#[must_use]
const fn outcome(status: &CompletionStatus) -> (WireCompletionStatus, &'static str) {
    match status {
        CompletionStatus::Success => (WireCompletionStatus::Success, "Done"),
        CompletionStatus::Cancelled => (WireCompletionStatus::Cancelled, "Cancelled"),
        CompletionStatus::Failed(reason) => (WireCompletionStatus::Failed, *reason),
    }
}

/// The distance actually travelled, in signed centimetres.
///
/// The average of the two achieved track revolutions times the sprocket
/// circumference; the command's sign is applied, so a backward command reports a
/// negative distance. Missing telemetry reports zero.
#[must_use]
fn achieved_distance_cm(cm: i16, telemetry: &CompletionTelemetry) -> i32 {
    let CompletionTelemetry::DriveDistance {
        achieved_left_revs,
        achieved_right_revs,
        ..
    } = telemetry
    else {
        return 0;
    };

    let average_revs = (achieved_left_revs + achieved_right_revs) / 2.0;
    let travelled_cm = average_revs * SPROCKET_CIRCUMFERENCE_CM;
    let signed_cm = if cm >= 0 { travelled_cm } else { -travelled_cm };
    i32::from(round_i16(signed_cm))
}

/// The rotation error in clockwise centidegrees, `achieved - target`.
///
/// Missing telemetry reports zero.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn rotate_error_cdeg(telemetry: &CompletionTelemetry) -> i32 {
    let CompletionTelemetry::RotateExact { angle_error_deg, .. } = telemetry else {
        return 0;
    };

    libm::roundf(angle_error_deg * 100.0) as i32
}

/// The duration in milliseconds, saturated to the wire's `u32`.
#[must_use]
fn duration_ms(telemetry: &CompletionTelemetry) -> u32 {
    let millis = match telemetry {
        CompletionTelemetry::DriveDistance { duration_ms, .. }
        | CompletionTelemetry::RotateExact { duration_ms, .. } => *duration_ms,
        CompletionTelemetry::None => 0,
    };
    u32::try_from(millis).unwrap_or(u32::MAX)
}

/// Round a centimetre value and clamp it to the wire's `i16` target range.
#[must_use]
#[allow(clippy::cast_possible_truncation)]
fn round_i16(value: f32) -> i16 {
    let rounded = libm::roundf(value);
    rounded.clamp(f32::from(i16::MIN), f32::from(i16::MAX)) as i16
}
