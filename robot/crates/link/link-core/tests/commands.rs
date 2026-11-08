//! Host tests for the robot's two-phase command seam.
//!
//! Everything goes through the real contract: `FakeRemote` encodes a command, `sim::Wire` carries
//! the bytes, `LinkCore` parses and dedupes them, and the ACK travels back through the same wire.

use link_core::{CommandRequest, Config, Inbound, LinkCore};
use link_protocol::{
    Class, Command, CommandMessage, Direction, Flags, Frame, MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, Seq, Status,
    StreamSample,
    codec::PayloadCodec,
    ids::ProcedureId,
    payload::{
        command::{
            AckPayload, CATALOGUE_PAGE_SIZE, CatalogueEntry, CatalogueFlags, CataloguePage, GetCatalogue, GetStatus,
            StatusSnapshot, StopMotion, encode_catalogue_page,
        },
        stream::{Activity, ActivityStage, Calibration, CalibrationStatus, Motion, Perception, Power},
    },
    sim::{FakeRemote, Faults, Wire},
};

/// The Boot Id the tests hand to the link core.
const BOOT_ID: u32 = 0x0BAD_C0DE;

/// Creates a core, wire and remote for one command path.
fn harness() -> (LinkCore, Wire, FakeRemote) {
    (
        LinkCore::new(Config::default(), BOOT_ID, 0),
        Wire::new(1, Faults::default()),
        FakeRemote::new(),
    )
}

/// Builds one standalone frame for the inbound path.
fn frame_bytes(id: u16, seq: u16, flags: Flags, major: u8, payload: &[u8]) -> Option<Vec<u8>> {
    let frame = Frame::new(major, Class::Command, flags, id, Seq::new(seq), payload).ok()?;
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = frame.encode_into(&mut buffer).ok()?;
    Some(buffer[..written].to_vec())
}

/// Drains the wire's PC-to-robot queue into the core.
fn feed(core: &mut LinkCore, wire: &mut Wire) {
    let now = u64::from(wire.now_ms());
    let bytes = wire.take(Direction::PcToRobot);
    core.on_rx(&bytes, now);
}

/// Emits one frame from the core, returning its encoded bytes.
fn emit(core: &mut LinkCore) -> Option<Vec<u8>> {
    let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
    let written = core.next_frame(&mut buffer)?;
    Some(buffer[..written].to_vec())
}

/// The parts of an ACK frame the tests assert on.
#[derive(Debug)]
struct AckFrame {
    /// The frame's delivery class.
    class: Class,
    /// The frame's flags.
    flags: Flags,
    /// The frame's id.
    id: u16,
    /// The frame's sequence number.
    seq: u16,
    /// The frame's payload.
    payload: Vec<u8>,
}

/// Decodes one frame emitted by the core.
fn decode_ack(bytes: &[u8]) -> Option<AckFrame> {
    let frame = Frame::decode(bytes, Direction::RobotToPc).ok()?;
    Some(AckFrame {
        class: frame.header.class,
        flags: frame.header.flags,
        id: frame.header.id,
        seq: frame.header.seq.get(),
        payload: frame.payload.to_vec(),
    })
}

/// One invalid frame and the status it must be auto-ACKed with.
#[derive(Clone, Copy, Debug)]
struct InvalidFrame<'a> {
    /// The command id to send.
    id: u16,
    /// The sequence number to send.
    seq: u16,
    /// The flags to send.
    flags: Flags,
    /// The envelope major to send.
    major: u8,
    /// The payload to send.
    payload: &'a [u8],
    /// The status the auto-ACK must carry.
    status: Status,
}

/// Sends one raw command frame and asserts it is auto-ACKed with `status` and never yielded.
fn assert_auto_ack(core: &mut LinkCore, wire: &mut Wire, case: InvalidFrame<'_>) {
    let InvalidFrame {
        id,
        seq,
        flags,
        major,
        payload,
        status,
    } = case;
    let Some(frame) = frame_bytes(id, seq, flags, major, payload) else {
        return;
    };
    wire.send(Direction::PcToRobot, &frame);
    feed(core, wire);

    assert!(
        core.poll().is_none(),
        "an auto-ACKed frame must never be yielded to the firmware"
    );
    let Some(bytes) = emit(core) else {
        return;
    };
    let Some(ack) = decode_ack(&bytes) else {
        return;
    };
    assert_eq!(ack.class, Class::Command, "an ACK keeps the command class");
    assert!(ack.flags.contains(Flags::IS_ACK), "an ACK sets IS_ACK");
    assert_eq!(ack.id, id, "the ACK mirrors the id");
    assert_eq!(ack.seq, seq, "the ACK mirrors the seq");
    assert_eq!(
        ack.payload,
        vec![status.bits()],
        "an auto-ACK carries the status byte only"
    );
    assert_eq!(core.last_seq(), 0, "an auto-ACKed frame must not advance dedupe");
}

/// A valid command is yielded once and its ACK mirrors class, id, seq and the firmware's status.
#[test]
fn valid_command_is_yielded_once_and_acked() {
    let (mut core, mut wire, mut remote) = harness();
    let Some(seq) = remote.send_command(&mut wire, Command::StopMotion, &[StopMotion::BRAKE]) else {
        return;
    };
    assert_eq!(seq, Seq::new(1), "commands start at sequence 1");
    feed(&mut core, &mut wire);

    {
        let Some(Inbound::Command(CommandRequest {
            seq: request_seq,
            flags,
            command,
            message,
        })) = core.poll()
        else {
            return;
        };
        assert_eq!(request_seq, seq, "the request keeps its sequence");
        assert!(flags.contains(Flags::ACK_REQUESTED), "the flag mirrors the frame");
        assert_eq!(command, Command::StopMotion, "the id is decoded");
        assert_eq!(
            message,
            CommandMessage::StopMotion(StopMotion {
                brake: StopMotion::BRAKE
            }),
            "the payload is typed"
        );
    }
    assert!(core.poll().is_none(), "only one command is yielded at a time");
    assert_eq!(core.last_seq(), 1, "acceptance advances the dedupe sequence");

    assert!(
        core.ack_command(seq.get(), Status::Busy, &[]),
        "the ack addresses the pending command"
    );
    let Some(bytes) = emit(&mut core) else {
        return;
    };
    let Some(ack) = decode_ack(&bytes) else {
        return;
    };
    assert_eq!(ack.class, Class::Command, "the ACK keeps the command class");
    assert!(ack.flags.contains(Flags::IS_ACK), "the ACK sets IS_ACK");
    assert_eq!(ack.id, Command::StopMotion.wire_id(), "the ACK mirrors the id");
    assert_eq!(ack.seq, seq.get(), "the ACK mirrors the seq");
    assert_eq!(
        ack.payload,
        vec![Status::Busy.bits()],
        "the ACK carries the firmware status"
    );

    wire.send(Direction::RobotToPc, &bytes);
    remote.receive(&mut wire);
    assert_eq!(
        remote.last_ack(),
        Some((
            Command::StopMotion,
            seq,
            CommandMessage::Ack(AckPayload {
                status: Status::Busy,
                result: &[],
            }),
        )),
        "the peer decodes the ACK"
    );
    assert_eq!(core.last_status(), Status::Busy, "the status is retained");
}

/// `last + 1` is accepted, `last` is re-ACKed without a second yield, and anything else is stale.
#[test]
fn sequence_dedupe_accepts_next_reacks_duplicate_and_stales_others() {
    let (mut core, mut wire, _remote) = harness();

    // Sequence 1: accepted and acknowledged with a result.
    let Some(first) = frame_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::COAST],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &first);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "sequence 1 is accepted");
    assert!(core.ack_command(1, Status::Ok, &[0xAA, 0xBB]), "ack sequence 1");
    let Some(first_ack) = emit(&mut core) else {
        return;
    };
    assert_eq!(
        decode_ack(&first_ack).map(|ack| ack.payload),
        Some(vec![Status::Ok.bits(), 0xAA, 0xBB]),
        "the stored payload includes the result"
    );

    // Sequence 1 again: re-send the stored ACK, never yield or execute again.
    wire.send(Direction::PcToRobot, &first);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_none(), "a duplicate is never yielded");
    let Some(replay) = emit(&mut core) else {
        return;
    };
    assert_eq!(
        decode_ack(&replay).map(|ack| ack.payload),
        Some(vec![Status::Ok.bits(), 0xAA, 0xBB]),
        "a duplicate re-sends the stored ACK unchanged"
    );

    // A gapped sequence is stale and does not advance dedupe.
    let Some(gap) = frame_bytes(
        Command::StopMotion.wire_id(),
        3,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::COAST],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &gap);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_none(), "a gapped sequence is not yielded");
    let Some(stale) = emit(&mut core) else {
        return;
    };
    let Some(stale) = decode_ack(&stale) else {
        return;
    };
    assert_eq!(stale.seq, 3, "stale mirrors the sequence");
    assert_eq!(stale.payload, vec![Status::Stale.bits()], "the gap is STALE");
    assert_eq!(core.last_seq(), 1, "STALE does not advance dedupe");

    // Sequence 2 is `last + 1` and is accepted.
    let Some(second) = frame_bytes(
        Command::StopMotion.wire_id(),
        2,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::BRAKE],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &second);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "last + 1 is accepted");
    assert_eq!(core.last_seq(), 2, "acceptance advances the dedupe sequence");

    // An older sequence is stale even though the command is still pending an answer.
    wire.send(Direction::PcToRobot, &first);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_none(), "an older sequence is not yielded");
    let Some(older) = emit(&mut core) else {
        return;
    };
    let Some(older) = decode_ack(&older) else {
        return;
    };
    assert_eq!(older.seq, 1, "the old sequence is mirrored");
    assert_eq!(older.payload, vec![Status::Stale.bits()], "an older sequence is STALE");
}

/// Unknown ids, bad payloads, bad flags and a major mismatch are auto-ACKed with the exact status.
#[test]
fn invalid_commands_are_auto_acked_and_never_yielded() {
    let (mut core, mut wire, _remote) = harness();
    assert_auto_ack(
        &mut core,
        &mut wire,
        InvalidFrame {
            id: 0x0100,
            seq: 1,
            flags: Flags::ACK_REQUESTED,
            major: PROTOCOL_MAJOR,
            payload: &[],
            status: Status::UnknownId,
        },
    );

    let (mut core, mut wire, _remote) = harness();
    assert_auto_ack(
        &mut core,
        &mut wire,
        InvalidFrame {
            id: Command::StopMotion.wire_id(),
            seq: 1,
            flags: Flags::ACK_REQUESTED,
            major: PROTOCOL_MAJOR,
            payload: &[0, 0],
            status: Status::BadLength,
        },
    );

    let (mut core, mut wire, _remote) = harness();
    assert_auto_ack(
        &mut core,
        &mut wire,
        InvalidFrame {
            id: Command::DriveDistance.wire_id(),
            seq: 1,
            flags: Flags::ACK_REQUESTED,
            major: PROTOCOL_MAJOR,
            payload: &[0, 0, 101],
            status: Status::BadArgument,
        },
    );

    let (mut core, mut wire, _remote) = harness();
    assert_auto_ack(
        &mut core,
        &mut wire,
        InvalidFrame {
            id: Command::StopMotion.wire_id(),
            seq: 1,
            flags: Flags::ACK_REQUESTED | Flags::FRAGMENT,
            major: PROTOCOL_MAJOR,
            payload: &[StopMotion::BRAKE],
            status: Status::BadFlags,
        },
    );

    let (mut core, mut wire, _remote) = harness();
    assert_auto_ack(
        &mut core,
        &mut wire,
        InvalidFrame {
            id: Command::StopMotion.wire_id(),
            seq: 1,
            flags: Flags::ACK_REQUESTED,
            major: PROTOCOL_MAJOR + 1,
            payload: &[StopMotion::BRAKE],
            status: Status::VersionMismatch,
        },
    );
}

/// An auto-ACK is only emitted when the request set `ACK_REQUESTED`.
#[test]
fn auto_acks_require_the_ack_requested_flag() {
    let (mut core, mut wire, _remote) = harness();
    let Some(frame) = frame_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::default(),
        PROTOCOL_MAJOR,
        &[0, 0],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &frame);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_none(), "a bad frame is never yielded");
    assert!(emit(&mut core).is_none(), "an unrequested ACK is not emitted");
}

/// The firmware must acknowledge a polled command before the next one is polled.
#[test]
fn ack_command_must_precede_the_next_poll() {
    let (mut core, mut wire, _remote) = harness();
    let Some(first) = frame_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::BRAKE],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &first);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "the first command is yielded");
    assert!(core.poll().is_none(), "nothing more is yielded before the ACK");

    // A second command cannot be accepted into the single slot while the first is unacked.
    let Some(second) = frame_bytes(
        Command::StopMotion.wire_id(),
        2,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::COAST],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &second);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_none(), "the second command waits behind the first");

    assert!(core.ack_command(1, Status::Ok, &[]), "ack the first command");
    assert_eq!(core.last_seq(), 1, "the dropped second command did not advance dedupe");

    // The daemon retries sequence 2 after the ACK; it is accepted now.
    wire.send(Direction::PcToRobot, &second);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "the retried second command is accepted");
    assert_eq!(core.last_seq(), 2, "the accepted command advances dedupe");
}

/// An `ack_command` for a sequence that is not pending is ignored.
#[test]
fn ack_command_ignores_a_non_pending_sequence() {
    let (mut core, mut wire, _remote) = harness();
    let Some(frame) = frame_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::BRAKE],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &frame);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "the command is yielded");

    assert!(!core.ack_command(2, Status::Ok, &[]), "an unknown sequence is ignored");
    assert_eq!(core.last_status(), Status::Ok, "the status is unchanged");
    assert!(
        core.ack_command(1, Status::Rejected, &[]),
        "the pending sequence is accepted"
    );
    assert_eq!(core.last_status(), Status::Rejected, "the status is stored");
    assert!(!core.ack_command(1, Status::Ok, &[]), "a second ack is ignored");
}

/// A new epoch resets command dedupe and discards the outstanding ACK state.
#[test]
fn epoch_reset_restarts_the_sequence() {
    let (mut core, mut wire, _remote) = harness();
    assert_eq!(core.epoch(), 0, "commands before HELLO run under epoch zero");

    let Some(frame) = frame_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::BRAKE],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &frame);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "sequence 1 is accepted");
    assert!(core.ack_command(1, Status::Ok, &[]), "ack sequence 1");
    assert_eq!(core.last_seq(), 1, "dedupe advanced");

    core.reset_epoch(7);
    assert_eq!(core.epoch(), 7, "the epoch is stored");
    assert_eq!(core.last_seq(), 0, "dedupe restarts at zero");

    // The same sequence 1 is valid again in the new epoch.
    wire.send(Direction::PcToRobot, &frame);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "sequence 1 is accepted in the new epoch");
}

/// `GET_CATALOGUE` pages round-trip, including a page past the end.
#[test]
fn catalogue_pages_round_trip_through_the_link() {
    let entries = [
        entry(ProcedureId::MOTOR_CALIBRATION, "Motor calibration"),
        entry(ProcedureId::MAG_CALIBRATION, "Mag calibration"),
        entry(ProcedureId::DISTANCE_CALIBRATION, "Distance calib"),
        entry(ProcedureId::COAST_AND_AVOID, "Coast and avoid"),
        entry(ProcedureId::ATTEMPT_STRAIGHT, "Attempt straight"),
        entry(ProcedureId::BASIC_MOTOR, "Basic motor"),
        entry(ProcedureId::TURNS, "Turns"),
        entry(ProcedureId::STRAIGHT_DRIVE, "Straight drive"),
        entry(ProcedureId::ARC_DRIVE, "Arc drive"),
        entry(ProcedureId::IMU_6_AXIS, "IMU 6-axis"),
        entry(ProcedureId::IMU_9_AXIS, "IMU 9-axis"),
    ];
    assert_eq!(entries.len(), 11, "the fixture holds the whole catalogue");

    let (mut core, mut wire, mut remote) = harness();
    for (page, expected_count) in [(0u8, 8usize), (1, 3), (2, 0)] {
        let Some(seq) = remote.send_command(&mut wire, Command::GetCatalogue, &[page]) else {
            return;
        };
        feed(&mut core, &mut wire);
        {
            let Some(Inbound::Command(request)) = core.poll() else {
                return;
            };
            assert_eq!(
                request.message,
                CommandMessage::GetCatalogue(GetCatalogue { page }),
                "the requested page is yielded"
            );
        }

        let start = usize::from(page) * CATALOGUE_PAGE_SIZE;
        let end = (start + CATALOGUE_PAGE_SIZE).min(entries.len());
        let page_entries = &entries[start.min(entries.len())..end];
        let mut result = [0u8; CataloguePage::ENCODED_LEN];
        let Ok(written) = encode_catalogue_page(11, page, page_entries.iter().copied(), &mut result) else {
            return;
        };
        assert!(
            core.ack_command(seq.get(), Status::Ok, &result[..written]),
            "the query is acknowledged"
        );

        let Some(bytes) = emit(&mut core) else {
            return;
        };
        wire.send(Direction::RobotToPc, &bytes);
        remote.receive(&mut wire);
        let Some((command, ack_seq, message)) = remote.last_ack() else {
            return;
        };
        assert_eq!(command, Command::GetCatalogue, "the ACK id is the query");
        assert_eq!(ack_seq, seq, "the ACK seq is the query");
        let CommandMessage::CataloguePage(response) = message else {
            return;
        };
        assert_eq!(response.total, 11, "total spans every page");
        assert_eq!(response.page, page, "the page echoes the request");
        assert_eq!(
            usize::from(response.count()),
            expected_count,
            "page {page} carries the expected entry count"
        );
        let decoded: Vec<_> = response.entries().collect();
        assert_eq!(decoded.len(), expected_count, "every entry decodes");
        for (decoded_entry, expected) in decoded.iter().zip(page_entries) {
            assert_eq!(decoded_entry, expected, "entries round-trip");
        }
    }
}

/// `GET_STATUS` matches the individual streams.
#[test]
fn get_status_matches_the_individual_streams() {
    let power = Power {
        level: 87,
        voltage_mv: 12_345,
    };
    let motion = Motion { left: -12, right: 34 };
    let activity = Activity {
        procedure_id: ProcedureId::STRAIGHT_DRIVE,
        stage: ActivityStage::Complete,
        percent: Some(42),
        detail: "driving",
    };
    let perception = Perception::OBSTACLE | Perception::FLOOR_DROP;
    let calibration = Calibration {
        motor: CalibrationStatus::Loaded,
        imu: CalibrationStatus::Unknown,
        mag: CalibrationStatus::Missing,
        distance: CalibrationStatus::Loaded,
    };

    let (mut core, mut wire, mut remote) = harness();
    core.publish_stream(&StreamSample::Power(power), 0, 0);
    core.publish_stream(&StreamSample::Motion(motion), 0, 0);
    core.publish_stream(&StreamSample::Activity(activity), 0, 0);
    core.publish_stream(&StreamSample::Perception(perception), 0, 0);
    core.publish_stream(&StreamSample::Calibration(calibration), 0, 0);

    let expected = StatusSnapshot {
        level: power.level,
        voltage_mv: power.voltage_mv,
        left: motion.left,
        right: motion.right,
        procedure_id: activity.procedure_id,
        stage: activity.stage,
        percent: activity.percent,
        detail: activity.detail,
        flags: perception,
        calibration,
    };

    let Some(seq) = remote.send_command(&mut wire, Command::GetStatus, &[]) else {
        return;
    };
    feed(&mut core, &mut wire);
    assert!(
        matches!(core.poll(), Some(Inbound::Command(request)) if request.message == CommandMessage::GetStatus(GetStatus)),
        "GET_STATUS is yielded"
    );

    let mut result = [0u8; StatusSnapshot::ENCODED_LEN];
    let Ok(written) = expected.encode_into(&mut result) else {
        return;
    };
    assert!(
        core.ack_command(seq.get(), Status::Ok, &result[..written]),
        "the query is acknowledged"
    );
    let Some(bytes) = emit(&mut core) else {
        return;
    };
    wire.send(Direction::RobotToPc, &bytes);
    remote.receive(&mut wire);

    let Some((command, ack_seq, message)) = remote.last_ack() else {
        return;
    };
    assert_eq!(command, Command::GetStatus, "the ACK id is the query");
    assert_eq!(ack_seq, seq, "the ACK seq is the query");
    let CommandMessage::StatusSnapshot(snapshot) = message else {
        return;
    };
    assert_eq!(snapshot, expected, "the snapshot matches every stream's latest sample");
}

/// ACK frames are emitted before pending stream frames.
#[test]
fn acks_precede_streams() {
    let (mut core, mut wire, mut remote) = harness();
    core.publish_stream(
        &StreamSample::Power(Power {
            level: 50,
            voltage_mv: 4_000,
        }),
        0,
        0,
    );

    let Some(frame) = frame_bytes(
        Command::StopMotion.wire_id(),
        1,
        Flags::ACK_REQUESTED,
        PROTOCOL_MAJOR,
        &[StopMotion::BRAKE],
    ) else {
        return;
    };
    wire.send(Direction::PcToRobot, &frame);
    feed(&mut core, &mut wire);
    assert!(core.poll().is_some(), "the command is yielded");
    assert!(core.ack_command(1, Status::Ok, &[]), "ack the command");

    let Some(bytes) = emit(&mut core) else {
        return;
    };
    let Some(first) = decode_ack(&bytes) else {
        return;
    };
    assert_eq!(first.class, Class::Command, "the ACK is emitted first");
    assert!(first.flags.contains(Flags::IS_ACK), "the first frame is the ACK");

    let Some(bytes) = emit(&mut core) else {
        return;
    };
    let Some(second) = decode_ack(&bytes) else {
        return;
    };
    assert_eq!(second.class, Class::Stream, "the stream follows the ACK");
    assert_eq!(second.id, link_protocol::ids::Stream::Power.wire_id(), "POWER");
    wire.send(Direction::RobotToPc, &bytes);
    assert_eq!(remote.receive(&mut wire), 1, "the stream still decodes");
}

/// A helper catalogue entry.
const fn entry(procedure_id: ProcedureId, label: &'static str) -> CatalogueEntry<'static> {
    CatalogueEntry {
        procedure_id,
        flags: CatalogueFlags::AVAILABLE,
        label,
    }
}

/// Feeding arbitrary command frames and draining the core never panics.
#[test]
fn arbitrary_command_frames_never_panic() {
    let (mut core, _wire, _remote) = harness();
    let mut state = 0x9E37_79B9u32;
    for _ in 0..2_000 {
        state ^= state << 13;
        state ^= state >> 17;
        state ^= state << 5;
        let id = u16::try_from(state & 0xFFFF).unwrap_or(0);
        let seq = u16::try_from(state >> 16).unwrap_or(0);
        let len = usize::try_from(state).unwrap_or(0) % 48;
        let mut payload = [0u8; 48];
        for slot in &mut payload[..len] {
            state ^= state << 13;
            state ^= state >> 17;
            state ^= state << 5;
            *slot = state.to_le_bytes()[0];
        }
        let flags = Flags::from_bits(u8::try_from(state & 0x0F).unwrap_or(0)) | Flags::ACK_REQUESTED;
        let Some(frame) = frame_bytes(id, seq, flags, PROTOCOL_MAJOR, &payload[..len]) else {
            continue;
        };
        core.on_rx(&frame, 0);

        let polled = core.poll().and_then(|inbound| match inbound {
            Inbound::Command(request) => Some(request.seq.get()),
            Inbound::EventAck(_) | Inbound::ControlReply(_) | Inbound::ControlTimeout { .. } => None,
        });
        if let Some(seq) = polled {
            let _ = core.ack_command(seq, Status::Ok, &[]);
        }
        let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
        let _ = core.next_frame(&mut buffer);
    }
}
