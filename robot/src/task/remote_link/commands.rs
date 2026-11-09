//! The Link Task's command dispatcher.
//!
//! [`handle`] drains [`LinkCore::poll`] one item at a time. A command is copied out
//! of its borrowed [`Inbound`] into an owned [`OwnedCommand`], executed through the
//! short dispatch/hand-off seam, and acknowledged with [`LinkCore::ack_command`]
//! before the next poll. An event ACK is only logged. A Bridge Control reply or
//! timeout is copied into an owned [`link_state::ControlOutcome`] and written to
//! [`crate::system::state::link`] once; the core has already retired it. The loop
//! never awaits a Procedure body, so RX, the ACK path and the lease keep running.
//!
//! # Command and ACK table
//!
//! | command | outcome |
//! |---|---|
//! | `START_PROCEDURE` | `OK` (started), `BUSY` (claim held), `REJECTED` (not remote-startable), `BAD_ARGUMENT` (unknown/idle/boot/Manual Drive id) |
//! | `STOP_PROCEDURE` | `OK`, whatever id it carries and whether or not work is live |
//! | `STOP_MOTION` | `OK`; `brake` selects `EmergencyBrake`, otherwise `Stop` |
//! | `GET_CATALOGUE` | `OK` plus one encoded [`CataloguePage`] |
//! | `GET_STATUS` | `OK` plus one encoded [`StatusSnapshot`] |
//! | `DRIVE_DISTANCE`, `ROTATE_BY` | `OK` (handed to the drive job), `BUSY` (claim held or job refusing) |
//!
//! The wire vocabulary and the Procedure dispatch both stay where they already
//! live: [`streams::procedure_id`] owns the id table and
//! [`procedure::dispatch`] is the one seam shared with the panel.

use defmt::{Debug2Format, info, warn};
use link_core::{Inbound, LinkCore};
use link_protocol::{
    BridgeControlReply, CommandMessage, PayloadCodec, ProcedureId, Status,
    payload::command::{
        CATALOGUE_PAGE_SIZE, CatalogueEntry, CatalogueFlags, CataloguePage, StatusSnapshot, StopMotion,
        encode_catalogue_page,
    },
};
use link_state::{ControlOutcome, ControlReply as LinkControlReply, ControlStatus, PairingCode, SetupCredentials};
use touch_ui::Procedure;

use super::{
    drive_job::{self, RemoteDriveAction},
    streams,
};
use crate::{
    system::state::{activity, link},
    task::{
        drive::InterruptKind,
        procedure::{self, ClaimOrigin, StartOutcome, StopOutcome},
    },
};

// ── Result buffer ─────────────────────────────────────────────────────────────

/// The largest result block an ACK can carry: a full catalogue page.
const ACK_RESULT_MAX: usize = CataloguePage::ENCODED_LEN;

/// The firmware's answer to one polled command.
///
/// [`LinkCore::ack_command`] copies the result before the next poll, so the bytes
/// are owned here and stay valid until the ACK has been encoded.
struct CommandAck {
    /// The wire status.
    status: Status,
    /// The number of valid bytes in [`Self::bytes`].
    len: usize,
    /// The command-specific result bytes after the status byte.
    bytes: [u8; ACK_RESULT_MAX],
}

impl CommandAck {
    /// A status with no result bytes.
    const fn status_only(status: Status) -> Self {
        Self {
            status,
            len: 0,
            bytes: [0; ACK_RESULT_MAX],
        }
    }

    /// A status with the given result bytes, truncated to [`ACK_RESULT_MAX`].
    fn with_result(status: Status, result: &[u8]) -> Self {
        let mut bytes = [0u8; ACK_RESULT_MAX];
        let len = result.len().min(ACK_RESULT_MAX);
        bytes[..len].copy_from_slice(&result[..len]);
        Self { status, len, bytes }
    }

    /// The result bytes to hand to [`LinkCore::ack_command`].
    #[must_use]
    fn result(&self) -> &[u8] {
        &self.bytes[..self.len]
    }
}

// ── Owned command ─────────────────────────────────────────────────────────────

/// One command copied out of the link core before any `await`.
///
/// An inbound request borrows the core, so every field needed after a suspension
/// point is copied here first. The handler can then await the short dispatch seam
/// and answer without holding a borrow of the core.
enum OwnedCommand {
    /// `START_PROCEDURE`: the Procedure to route.
    StartProcedure {
        /// The Procedure id as it arrived on the wire.
        procedure_id: ProcedureId,
    },
    /// `STOP_PROCEDURE`: the carried id is deliberately not retained.
    StopProcedure,
    /// `DRIVE_DISTANCE`: handed to the drive job.
    DriveDistance {
        /// Signed distance in centimetres.
        distance_cm: i16,
        /// Drive speed percentage.
        speed: u8,
    },
    /// `ROTATE_BY`: handed to the drive job.
    RotateBy {
        /// Signed angle in centidegrees.
        angle_cdeg: i16,
        /// Drive speed percentage.
        speed: u8,
    },
    /// `STOP_MOTION`: whether to brake or coast.
    StopMotion {
        /// `0` coasts, anything else selects the brake.
        brake: u8,
    },
    /// `GET_CATALOGUE`: the requested zero-based page.
    GetCatalogue {
        /// The zero-based page index.
        page: u8,
    },
    /// `GET_STATUS`: no request payload.
    GetStatus,
    /// A message the poll surface should never yield as a request.
    Unsupported,
}

impl OwnedCommand {
    /// Copy the typed request fields out of a borrowed command message.
    const fn from_message(message: CommandMessage<'_>) -> Self {
        match message {
            CommandMessage::StartProcedure(request) => Self::StartProcedure {
                procedure_id: request.procedure_id,
            },
            CommandMessage::StopProcedure(_) => Self::StopProcedure,
            CommandMessage::DriveDistance(request) => Self::DriveDistance {
                distance_cm: request.distance_cm,
                speed: request.speed,
            },
            CommandMessage::RotateBy(request) => Self::RotateBy {
                angle_cdeg: request.angle_cdeg,
                speed: request.speed,
            },
            CommandMessage::StopMotion(request) => Self::StopMotion { brake: request.brake },
            CommandMessage::GetCatalogue(request) => Self::GetCatalogue { page: request.page },
            CommandMessage::GetStatus(_) => Self::GetStatus,
            CommandMessage::CataloguePage(_) | CommandMessage::StatusSnapshot(_) | CommandMessage::Ack(_) => {
                Self::Unsupported
            }
        }
    }
}

// ── Dispatcher ────────────────────────────────────────────────────────────────

/// One item copied out of the link core, free of any borrow of it.
///
/// The core is borrowed for the duration of one [`LinkCore::poll`], so every value
/// needed after that borrow ends is owned here. That is what keeps the link-state
/// await from holding a borrow of the core.
enum PolledItem {
    /// A command and its wire sequence number.
    Command {
        /// The command's wire sequence number.
        seq: u16,
        /// The owned command fields.
        command: OwnedCommand,
    },
    /// A Bridge Control reply or timeout to write into the link state.
    Control(ControlOutcome),
}

/// Answer every polled command before polling again.
///
/// Each command is copied out, executed through the short dispatch/hand-off seam,
/// acknowledged, and only then is [`LinkCore::poll`] called again. A Bridge Control
/// reply or timeout is copied out first and written to the link state after the
/// borrow of the core has ended. No Procedure body is ever awaited.
pub(super) async fn handle(core: &mut LinkCore, _now_ms: u64) {
    while let Some(item) = next_item(core) {
        match item {
            PolledItem::Command { seq, command } => {
                let ack = execute(command, seq).await;
                let _ = core.ack_command(seq, ack.status, ack.result());
            }
            PolledItem::Control(outcome) => link::set_control(outcome).await,
        }
    }
}

/// Poll one item, logging event ACKs on the way.
///
/// Returns owned data, so the caller can await without holding the core's borrow.
/// `None` means the core has nothing more to yield right now.
fn next_item(core: &mut LinkCore) -> Option<PolledItem> {
    loop {
        match core.poll()? {
            Inbound::Command(request) => {
                return Some(PolledItem::Command {
                    seq: request.seq.get(),
                    command: OwnedCommand::from_message(request.message),
                });
            }
            Inbound::EventAck(notice) => info!(
                "[link] event {} ack: {:?}",
                notice.seq.get(),
                Debug2Format(&notice.status)
            ),
            Inbound::ControlReply(reply) => return Some(PolledItem::Control(control_outcome(&reply))),
            Inbound::ControlTimeout { seq } => {
                warn!("[link] bridge control {} timed out", seq);
                return Some(PolledItem::Control(ControlOutcome::TimedOut));
            }
        }
    }
}

/// Copy a Bridge Control reply into the neutral outcome the panel reads.
///
/// The reply borrows the core, so the SSID, passphrase and code are copied into
/// fixed-capacity strings here. A string over its capacity is truncated on a
/// UTF-8 boundary rather than dropping the whole reply. A malformed or unusable
/// acknowledgement is an answered `Failed` outcome, not a timeout: the Bridge did
/// answer, and the action it answered is not echoed on the wire, so the setup shape
/// is the neutral fallback. The core has already retired the reply.
fn control_outcome(reply: &link_core::ControlReply<'_>) -> ControlOutcome {
    match &reply.result {
        Ok(BridgeControlReply::Setup(setup)) => ControlOutcome::Answered(LinkControlReply::Setup {
            status: map_control_status(setup.status),
            credentials: SetupCredentials {
                ssid: copy_text(setup.ssid),
                passphrase: copy_text(setup.passphrase),
            },
        }),
        Ok(BridgeControlReply::PairingCode(code)) => ControlOutcome::Answered(LinkControlReply::PairingCode {
            status: map_control_status(code.status),
            code: PairingCode {
                code: copy_text(code.code),
            },
        }),
        Err(_) => ControlOutcome::Answered(LinkControlReply::Setup {
            status: ControlStatus::Failed,
            credentials: SetupCredentials {
                ssid: heapless::String::new(),
                passphrase: heapless::String::new(),
            },
        }),
    }
}

/// Map a wire status onto the three neutral outcomes the panel shows.
const fn map_control_status(status: Status) -> ControlStatus {
    match status {
        Status::Ok => ControlStatus::Ok,
        Status::Rejected | Status::Busy => ControlStatus::Refused,
        Status::UnknownId
        | Status::BadArgument
        | Status::VersionMismatch
        | Status::BadLength
        | Status::BadFlags
        | Status::Stale
        | Status::Internal => ControlStatus::Failed,
    }
}

/// Copy `text` into a fixed-capacity string, truncating on a character boundary.
///
/// Pushing character by character cannot split a multi-byte UTF-8 sequence, so a
/// too-long value is shortened rather than rejected.
fn copy_text<const N: usize>(text: &str) -> heapless::String<N> {
    let mut out = heapless::String::new();
    for ch in text.chars() {
        if out.push(ch).is_err() {
            break;
        }
    }
    out
}

/// Run one owned command and build its answer.
///
/// `seq` is the command's sequence number; the volatile drive commands carry it
/// into the completion Event so the Remote can match the outcome to the request.
async fn execute(command: OwnedCommand, seq: u16) -> CommandAck {
    match command {
        OwnedCommand::StartProcedure { procedure_id } => start_procedure(procedure_id).await,
        OwnedCommand::StopProcedure => {
            // The carried id is ignored: a stop stops the holder regardless of it.
            procedure::stop(StopOutcome::Operator, InterruptKind::Stop);
            CommandAck::status_only(Status::Ok)
        }
        OwnedCommand::StopMotion { brake } => {
            let interrupt = if brake == StopMotion::BRAKE {
                InterruptKind::EmergencyBrake
            } else {
                InterruptKind::Stop
            };
            procedure::stop(StopOutcome::Operator, interrupt);
            CommandAck::status_only(Status::Ok)
        }
        OwnedCommand::GetCatalogue { page } => catalogue_ack(page),
        OwnedCommand::GetStatus => status_ack().await,
        OwnedCommand::DriveDistance { distance_cm, speed } => {
            // Hand off to the long-lived drive job and acknowledge immediately.
            // The job owns the claim and the queue; this loop never awaits the drive.
            let status = drive_job::start(RemoteDriveAction::Distance { cm: distance_cm, speed }, seq).await;
            CommandAck::status_only(status)
        }
        OwnedCommand::RotateBy { angle_cdeg, speed } => {
            let status = drive_job::start(
                RemoteDriveAction::Rotate {
                    cdeg: angle_cdeg,
                    speed,
                },
                seq,
            )
            .await;
            CommandAck::status_only(status)
        }
        OwnedCommand::Unsupported => CommandAck::status_only(Status::Internal),
    }
}

/// Start one Procedure through the same dispatch seam the panel uses.
async fn start_procedure(procedure_id: ProcedureId) -> CommandAck {
    let Some(procedure) = procedure_from_id(procedure_id) else {
        return CommandAck::status_only(Status::BadArgument);
    };
    let status = match procedure::dispatch(procedure, ClaimOrigin::Remote).await {
        StartOutcome::Started(_) => Status::Ok,
        StartOutcome::Busy => Status::Busy,
        StartOutcome::Rejected => Status::Rejected,
    };
    CommandAck::status_only(status)
}

/// Map the wire id back to the firmware Procedure, the inverse of
/// [`streams::procedure_id`].
///
/// `IDLE`, `BOOT`, the Activity-only Manual Drive id and any id this version does
/// not know return `None`. The search uses the one Procedure list and the one id
/// table, so adding a Procedure cannot leave the inverse out of step.
fn procedure_from_id(id: ProcedureId) -> Option<Procedure> {
    streams::PROCEDURES
        .into_iter()
        .find(|procedure| streams::procedure_id(*procedure) == id)
}

// ── Catalogue ─────────────────────────────────────────────────────────────────

/// Encode one catalogue page and answer `OK` with it.
///
/// A page beyond the end is an empty page carrying the requested index and the
/// full total, not an error.
fn catalogue_ack(page: u8) -> CommandAck {
    let start = usize::from(page) * CATALOGUE_PAGE_SIZE;
    let entries = streams::PROCEDURES
        .iter()
        .copied()
        .skip(start)
        .take(CATALOGUE_PAGE_SIZE)
        .map(catalogue_entry);
    let total = u8::try_from(streams::PROCEDURES.len()).unwrap_or(u8::MAX);
    let mut encoded = [0u8; CataloguePage::ENCODED_LEN];
    encode_catalogue_page(total, page, entries, &mut encoded).map_or_else(
        |_| CommandAck::status_only(Status::Internal),
        |len| CommandAck::with_result(Status::Ok, &encoded[..len]),
    )
}

/// Build one catalogue entry from the robot's own Procedure metadata.
fn catalogue_entry(procedure: Procedure) -> CatalogueEntry<'static> {
    CatalogueEntry {
        procedure_id: streams::procedure_id(procedure),
        flags: catalogue_flags(procedure),
        label: procedure.label(),
    }
}

/// The catalogue flags for one Procedure.
///
/// Availability is the one [`procedure::is_startable`] rule, not a second copy of
/// the Remote hands-on set.
fn catalogue_flags(procedure: Procedure) -> CatalogueFlags {
    let mut flags = CatalogueFlags::default();
    if procedure.interactive() {
        flags |= CatalogueFlags::INTERACTIVE;
    }
    if procedure.is_calibration() {
        flags |= CatalogueFlags::CALIBRATION;
    }
    if matches!(procedure, Procedure::AttemptStraight) {
        flags |= CatalogueFlags::TAKES_PARAMS;
    }
    if procedure::is_startable(procedure, ClaimOrigin::Remote) {
        flags |= CatalogueFlags::AVAILABLE;
    }
    flags
}

// ── Status ────────────────────────────────────────────────────────────────────

/// Encode the current state snapshot and answer `OK` with it.
///
/// Reuses the Streams sampler's mappings, in the documented lock order
/// (power → calibration → perception → motion → activity). No lock is held across
/// an `await` beyond the module accessor that owns it.
async fn status_ack() -> CommandAck {
    let power = streams::power_sample().await;
    let calibration = streams::calibration_sample().await;
    let flags = streams::perception_sample();
    let motion = streams::motion_sample();
    let snapshot = activity::snapshot().await;

    // A Manual Drive's detail is derived into a fixed-capacity buffer, kept alive
    // here until `encode_into` has copied it.
    let manual_label = snapshot.manual_drive_label();
    let detail = streams::activity_detail(&snapshot, manual_label.as_ref());

    let status = StatusSnapshot {
        level: power.level,
        voltage_mv: power.voltage_mv,
        left: motion.left,
        right: motion.right,
        procedure_id: streams::activity_procedure_id(snapshot.activity),
        stage: streams::activity_stage(snapshot.stage),
        percent: snapshot.percent,
        detail,
        flags,
        calibration,
    };

    let mut encoded = [0u8; StatusSnapshot::ENCODED_LEN];
    status.encode_into(&mut encoded).map_or_else(
        |_| CommandAck::status_only(Status::Internal),
        |len| CommandAck::with_result(Status::Ok, &encoded[..len]),
    )
}
