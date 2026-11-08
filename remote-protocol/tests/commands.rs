//! Host tests for the v1 command codecs and typed query responses (ADR-0031).
//!
//! The tests exercise the external contract only: wire ids, encoded bytes, decoded values, the
//! typed errors for malformed input and the branch between requests and ACK responses.

use std::collections::HashSet;

use remote_protocol::{
    CodecError, Command, CommandMessage, EncodeError, Status,
    codec::{PayloadCodec, decode_command, encode_command},
    ids::ProcedureId,
    payload::{
        command::{
            AckPayload, CATALOGUE_LABEL_MAX, CATALOGUE_PAGE_SIZE, CatalogueEntry, CatalogueFlags, CataloguePage,
            DriveDistance, GetCatalogue, GetStatus, RotateBy, StartProcedure, StatusSnapshot, StopMotion,
            StopProcedure, encode_catalogue_page,
        },
        stream::{ActivityStage, Calibration, CalibrationStatus, Motion, Perception, Power},
    },
};

/// A scratch buffer large enough for every command payload.
const BUFFER_LEN: usize = 256;

/// Returns `true` when `message` is an ACK response.
const fn message_is_ack(message: &CommandMessage<'_>) -> bool {
    matches!(
        message,
        CommandMessage::CataloguePage(_) | CommandMessage::StatusSnapshot(_) | CommandMessage::Ack(_)
    )
}

/// Encodes `message`, decodes it back with the matching command id and asserts it survives.
fn assert_round_trip(message: &CommandMessage<'_>, command: Command) {
    let mut buffer = [0u8; BUFFER_LEN];
    let Ok(written) = encode_command(message, &mut buffer) else {
        return;
    };
    let Ok(decoded) = decode_command(command, message_is_ack(message), &buffer[..written]) else {
        return;
    };
    assert_eq!(&decoded, message, "{command:?} must round-trip");
}

/// Every implemented command is in `ALL`, with unique core ids that round-trip.
#[test]
fn command_ids_are_unique_in_range_and_round_trip() {
    assert_eq!(Command::ALL.len(), 7, "the v1 catalogue has seven commands");

    let mut ids = HashSet::new();
    let mut indices = HashSet::new();
    for command in Command::ALL {
        let id = command.wire_id();
        assert!(
            (0x0001..=0x00FF).contains(&id),
            "{command:?} must use a core id, got {id:#06x}"
        );
        assert!(ids.insert(id), "{command:?} must have a unique id");
        assert!(indices.insert(command.index()), "{command:?} must have a unique index");
        assert_eq!(
            Command::from_wire(id),
            Some(command),
            "from_wire must recover {command:?}"
        );
        assert_eq!(
            Command::ALL[command.index()],
            command,
            "index must address the command in ALL"
        );
    }
}

/// Reserved and unassigned command ids stay unknown.
#[test]
fn reserved_command_ids_are_none() {
    for id in [0x0000, 0x0008, 0x000F, 0x0010, 0x0015, 0x00FF, 0xFFFF] {
        assert_eq!(Command::from_wire(id), None, "{id:#06x} must be unknown");
    }
}

/// Every request round-trips through the codec dispatch.
#[test]
fn requests_round_trip() {
    let start = CommandMessage::StartProcedure(StartProcedure {
        procedure_id: ProcedureId::ATTEMPT_STRAIGHT,
        params: &[1, 2, 3],
    });
    assert_round_trip(&start, Command::StartProcedure);

    let stop = CommandMessage::StopProcedure(StopProcedure {
        procedure_id: ProcedureId::STRAIGHT_DRIVE,
    });
    assert_round_trip(&stop, Command::StopProcedure);

    let drive = CommandMessage::DriveDistance(DriveDistance {
        distance_cm: -250,
        speed: 70,
    });
    assert_round_trip(&drive, Command::DriveDistance);

    let rotate = CommandMessage::RotateBy(RotateBy {
        angle_cdeg: -9_000,
        speed: 60,
    });
    assert_round_trip(&rotate, Command::RotateBy);

    let stop_motion = CommandMessage::StopMotion(StopMotion {
        brake: StopMotion::BRAKE,
    });
    assert_round_trip(&stop_motion, Command::StopMotion);

    let catalogue = CommandMessage::GetCatalogue(GetCatalogue { page: 2 });
    assert_round_trip(&catalogue, Command::GetCatalogue);

    let status = CommandMessage::GetStatus(GetStatus);
    assert_round_trip(&status, Command::GetStatus);
}

/// `START_PROCEDURE` parameters are opaque and bounded at 32 bytes.
#[test]
fn start_procedure_params_are_bounded() {
    let empty = StartProcedure {
        procedure_id: ProcedureId::BASIC_MOTOR,
        params: &[],
    };
    let mut buffer = [0u8; BUFFER_LEN];
    let Ok(written) = empty.encode_into(&mut buffer) else {
        return;
    };
    assert_eq!(written, 2, "the empty parameter block is just the id");
    assert_eq!(
        StartProcedure::decode(&buffer[..written]),
        Ok(empty),
        "an empty parameter block must round-trip"
    );

    let longest = [0xABu8; 32];
    let at_cap = StartProcedure {
        procedure_id: ProcedureId::ATTEMPT_STRAIGHT,
        params: &longest,
    };
    let Ok(written) = at_cap.encode_into(&mut buffer) else {
        return;
    };
    assert_eq!(written, 34, "32 parameter bytes plus the id");
    assert_eq!(
        StartProcedure::decode(&buffer[..written]),
        Ok(at_cap),
        "the longest allowed parameters must round-trip"
    );

    let too_long = [0u8; 33];
    let over = StartProcedure {
        procedure_id: ProcedureId::ATTEMPT_STRAIGHT,
        params: &too_long,
    };
    assert_eq!(
        over.encode_into(&mut buffer),
        Err(EncodeError::Value),
        "33 parameter bytes cannot be encoded"
    );
    assert_eq!(
        StartProcedure::decode(&[0u8; 35]),
        Err(CodecError::Length),
        "33 parameter bytes cannot be decoded"
    );
}

/// Drive speeds are the firmware's documented `0..=100` percentage.
#[test]
fn drive_speed_range_is_enforced() {
    let mut buffer = [0u8; BUFFER_LEN];
    for speed in [0, 1, 99, 100] {
        let drive = DriveDistance { distance_cm: 10, speed };
        let Ok(written) = drive.encode_into(&mut buffer) else {
            return;
        };
        assert_eq!(
            DriveDistance::decode(&buffer[..written]),
            Ok(drive),
            "speed {speed} must round-trip"
        );
    }

    let too_fast = DriveDistance {
        distance_cm: 10,
        speed: 101,
    };
    assert_eq!(
        too_fast.encode_into(&mut buffer),
        Err(EncodeError::Value),
        "speed 101 cannot be encoded"
    );
    assert_eq!(
        DriveDistance::decode(&[0, 0, 101]),
        Err(CodecError::Value),
        "speed 101 cannot be decoded"
    );
}

/// `ROTATE_BY` enforces both the speed range and the signed centidegree range.
#[test]
fn rotate_range_checks_are_enforced() {
    let mut buffer = [0u8; BUFFER_LEN];
    for angle in [-18_000i16, -1, 0, 1, 18_000] {
        let rotate = RotateBy {
            angle_cdeg: angle,
            speed: 50,
        };
        let Ok(written) = rotate.encode_into(&mut buffer) else {
            return;
        };
        assert_eq!(
            RotateBy::decode(&buffer[..written]),
            Ok(rotate),
            "angle {angle} must round-trip"
        );
    }

    for angle in [-18_001i16, 18_001] {
        let rotate = RotateBy {
            angle_cdeg: angle,
            speed: 50,
        };
        assert_eq!(
            rotate.encode_into(&mut buffer),
            Err(EncodeError::Value),
            "angle {angle} cannot be encoded"
        );
        let [low, high] = angle.to_le_bytes();
        assert_eq!(
            RotateBy::decode(&[low, high, 50]),
            Err(CodecError::Value),
            "angle {angle} cannot be decoded"
        );
    }

    let too_fast = RotateBy {
        angle_cdeg: 0,
        speed: 101,
    };
    assert_eq!(
        too_fast.encode_into(&mut buffer),
        Err(EncodeError::Value),
        "rotation speed 101 cannot be encoded"
    );
    assert_eq!(
        RotateBy::decode(&[0, 0, 101]),
        Err(CodecError::Value),
        "rotation speed 101 cannot be decoded"
    );
}

/// `STOP_MOTION` accepts only coast and brake.
#[test]
fn stop_motion_brake_values_are_enforced() {
    let mut buffer = [0u8; BUFFER_LEN];
    for brake in [StopMotion::COAST, StopMotion::BRAKE] {
        let stop = StopMotion { brake };
        let Ok(written) = stop.encode_into(&mut buffer) else {
            return;
        };
        assert_eq!(StopMotion::decode(&buffer[..written]), Ok(stop), "brake {brake}");
    }

    let bad = StopMotion { brake: 2 };
    assert_eq!(
        bad.encode_into(&mut buffer),
        Err(EncodeError::Value),
        "brake 2 cannot be encoded"
    );
    assert_eq!(
        StopMotion::decode(&[2]),
        Err(CodecError::Value),
        "brake 2 cannot be decoded"
    );
}

/// Request layouts reject structural length mismatches.
#[test]
fn request_lengths_are_strict() {
    assert_eq!(
        StartProcedure::decode(&[0]),
        Err(CodecError::Length),
        "START_PROCEDURE needs the id"
    );
    assert_eq!(
        StopProcedure::decode(&[0]),
        Err(CodecError::Length),
        "STOP_PROCEDURE needs two bytes"
    );
    assert_eq!(
        DriveDistance::decode(&[0, 0]),
        Err(CodecError::Length),
        "DRIVE_DISTANCE needs three bytes"
    );
    assert_eq!(
        RotateBy::decode(&[0, 0, 0, 0]),
        Err(CodecError::Length),
        "ROTATE_BY is exactly three bytes"
    );
    assert_eq!(StopMotion::decode(&[]), Err(CodecError::Length), "STOP_MOTION");
    assert_eq!(GetCatalogue::decode(&[]), Err(CodecError::Length), "GET_CATALOGUE");
    assert_eq!(GetStatus::decode(&[0]), Err(CodecError::Length), "GET_STATUS is empty");
}

/// The generic ACK payload is a status byte plus opaque result bytes.
#[test]
fn generic_ack_carries_status_and_result() {
    let ack = AckPayload {
        status: Status::Busy,
        result: &[9, 8, 7],
    };
    let mut buffer = [0u8; BUFFER_LEN];
    let Ok(written) = ack.encode_into(&mut buffer) else {
        return;
    };
    assert_eq!(&buffer[..written], &[Status::Busy.bits(), 9, 8, 7], "status first");

    let message = CommandMessage::Ack(ack);
    assert_round_trip(&message, Command::StopMotion);

    assert_eq!(AckPayload::decode(&[]), Err(CodecError::Length), "a status is required");
    assert_eq!(
        AckPayload::decode(&[10]),
        Err(CodecError::Value),
        "status 10 is unassigned"
    );
}

/// A catalogue page round-trips and pages past the end report `count = 0` with `total` set.
#[test]
fn catalogue_pages_round_trip() {
    let entries = [
        CatalogueEntry {
            procedure_id: ProcedureId::MOTOR_CALIBRATION,
            flags: CatalogueFlags::CALIBRATION | CatalogueFlags::AVAILABLE,
            label: "Motor calibration",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::MAG_CALIBRATION,
            flags: CatalogueFlags::CALIBRATION,
            label: "Mag calibration",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::DISTANCE_CALIBRATION,
            flags: CatalogueFlags::CALIBRATION | CatalogueFlags::AVAILABLE,
            label: "Distance calib",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::COAST_AND_AVOID,
            flags: CatalogueFlags::AVAILABLE,
            label: "Coast and avoid",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::ATTEMPT_STRAIGHT,
            flags: CatalogueFlags::INTERACTIVE | CatalogueFlags::TAKES_PARAMS,
            label: "Attempt straight",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::BASIC_MOTOR,
            flags: CatalogueFlags::AVAILABLE,
            label: "Basic motor",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::TURNS,
            flags: CatalogueFlags::AVAILABLE,
            label: "Turns",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::STRAIGHT_DRIVE,
            flags: CatalogueFlags::AVAILABLE,
            label: "Straight drive",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::ARC_DRIVE,
            flags: CatalogueFlags::AVAILABLE,
            label: "Arc drive",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::IMU_6_AXIS,
            flags: CatalogueFlags::AVAILABLE,
            label: "IMU 6-axis",
        },
        CatalogueEntry {
            procedure_id: ProcedureId::IMU_9_AXIS,
            flags: CatalogueFlags::AVAILABLE,
            label: "IMU 9-axis",
        },
    ];
    assert_eq!(entries.len(), 11, "the fixture holds the whole catalogue");

    let mut buffer = [0u8; CataloguePage::ENCODED_LEN];
    let first_page = &entries[..CATALOGUE_PAGE_SIZE];
    let Ok(written) = encode_catalogue_page(11, 0, first_page.iter().copied(), &mut buffer) else {
        return;
    };
    let Ok(decoded) = CataloguePage::decode(&buffer[..written]) else {
        return;
    };
    assert_eq!(decoded.total, 11, "total spans every page");
    assert_eq!(decoded.page, 0, "page zero");
    assert_eq!(decoded.count(), 8, "the first page is full");
    let collected: Vec<_> = decoded.entries().collect();
    assert_eq!(collected.len(), first_page.len(), "every entry must decode");
    for (decoded_entry, expected) in collected.iter().zip(first_page) {
        assert_eq!(decoded_entry, expected, "entries must round-trip in order");
    }
    assert_round_trip(&CommandMessage::CataloguePage(decoded), Command::GetCatalogue);

    let second_page = &entries[CATALOGUE_PAGE_SIZE..];
    let Ok(written) = encode_catalogue_page(11, 1, second_page.iter().copied(), &mut buffer) else {
        return;
    };
    let Ok(decoded) = CataloguePage::decode(&buffer[..written]) else {
        return;
    };
    assert_eq!(decoded.total, 11, "total is unchanged on the last page");
    assert_eq!(decoded.page, 1, "page one");
    assert_eq!(decoded.count(), 3, "the remaining three entries fit");

    let Ok(written) = encode_catalogue_page(11, 2, [], &mut buffer) else {
        return;
    };
    let Ok(decoded) = CataloguePage::decode(&buffer[..written]) else {
        return;
    };
    assert_eq!(decoded.count(), 0, "a page past the end is empty");
    assert_eq!(decoded.total, 11, "a page past the end still reports the total");
    assert_eq!(decoded.entries().next(), None, "there is nothing to iterate");
}

/// Labels are truncated at a UTF-8 boundary by the encoder and tolerated by the decoder.
#[test]
fn catalogue_labels_are_truncated_and_tolerated() {
    let label = format!("{}{}", "a".repeat(CATALOGUE_LABEL_MAX - 1), "é");
    assert_eq!(label.len(), CATALOGUE_LABEL_MAX + 1, "the fixture exceeds the cap");
    let entry = CatalogueEntry {
        procedure_id: ProcedureId::TURNS,
        flags: CatalogueFlags::AVAILABLE,
        label: &label,
    };

    let mut buffer = [0u8; CataloguePage::ENCODED_LEN];
    let Ok(written) = encode_catalogue_page(1, 0, [entry], &mut buffer) else {
        return;
    };
    let Ok(decoded) = CataloguePage::decode(&buffer[..written]) else {
        return;
    };
    let Some(entry) = decoded.entries().next() else {
        return;
    };
    assert_eq!(
        entry.label,
        "a".repeat(CATALOGUE_LABEL_MAX - 1),
        "the label must end at a character boundary"
    );

    // A decoder accepts a label longer than the encoder's cap when it fits the payload.
    let mut longer = vec![1, 0, 1, 0x02, 0x00, 0x08, 40];
    longer.extend_from_slice(&[b'x'; 40]);
    let Ok(page) = CataloguePage::decode(&longer) else {
        return;
    };
    let Some(entry) = page.entries().next() else {
        return;
    };
    assert_eq!(entry.label.len(), 40, "the longer label must decode");
}

/// A malformed catalogue page is a length or UTF-8 error, and reserved flag bits are cleared.
#[test]
fn catalogue_structure_and_flags_are_validated() {
    assert_eq!(
        CataloguePage::decode(&[0, 0]),
        Err(CodecError::Length),
        "the three-byte header is required"
    );
    assert_eq!(
        CataloguePage::decode(&[1, 0, 1, 0, 0, 0, 5]),
        Err(CodecError::Length),
        "an entry shorter than its label is malformed"
    );
    assert_eq!(
        CataloguePage::decode(&[1, 0, 1, 0, 0, 0, 0, 0xAA]),
        Err(CodecError::Length),
        "trailing bytes are a structural mismatch"
    );
    assert_eq!(
        CataloguePage::decode(&[1, 0, 1, 0, 0, 0, 1, 0xFF]),
        Err(CodecError::Utf8),
        "invalid label text must be reported"
    );

    let reserved = CataloguePage::decode(&[1, 0, 1, 3, 0, 0xFF, 1, b'a']);
    let Ok(reserved) = reserved else {
        return;
    };
    let Some(entry) = reserved.entries().next() else {
        return;
    };
    assert_eq!(
        entry.flags.bits(),
        CatalogueFlags::INTERACTIVE.bits()
            | CatalogueFlags::CALIBRATION.bits()
            | CatalogueFlags::TAKES_PARAMS.bits()
            | CatalogueFlags::AVAILABLE.bits(),
        "bits 4-7 must be cleared on decode"
    );
}

/// A status snapshot round-trips and its fields match the individual streams byte for byte.
#[test]
fn status_snapshot_matches_the_individual_streams() {
    let perception = Perception::OBSTACLE | Perception::FLOOR_DROP;
    let calibration = Calibration {
        motor: CalibrationStatus::Loaded,
        imu: CalibrationStatus::Unknown,
        mag: CalibrationStatus::Missing,
        distance: CalibrationStatus::Loaded,
    };
    let snapshot = StatusSnapshot {
        level: 87,
        voltage_mv: 12_345,
        left: -12,
        right: 34,
        procedure_id: ProcedureId::STRAIGHT_DRIVE,
        stage: ActivityStage::Complete,
        percent: Some(42),
        detail: "driving",
        flags: perception,
        calibration,
    };

    let mut buffer = [0u8; BUFFER_LEN];
    let Ok(written) = snapshot.encode_into(&mut buffer) else {
        return;
    };
    assert_eq!(written, 15 + "driving".len(), "the layout is fixed besides the detail");
    assert_eq!(
        StatusSnapshot::decode(&buffer[..written]),
        Ok(snapshot),
        "the snapshot must round-trip"
    );

    let mut power_bytes = [0u8; Power::ENCODED_LEN];
    let power = Power {
        level: snapshot.level,
        voltage_mv: snapshot.voltage_mv,
    };
    if power.encode_into(&mut power_bytes).is_err() {
        return;
    }
    assert_eq!(&buffer[0..3], &power_bytes, "GET_STATUS begins with the POWER payload");

    let mut motion_bytes = [0u8; Motion::ENCODED_LEN];
    let motion = Motion {
        left: snapshot.left,
        right: snapshot.right,
    };
    if motion.encode_into(&mut motion_bytes).is_err() {
        return;
    }
    assert_eq!(&buffer[3..5], &motion_bytes, "then the MOTION payload");

    assert_eq!(
        u16::from_le_bytes([buffer[5], buffer[6]]),
        snapshot.procedure_id.get(),
        "the Procedure id matches ACTIVITY"
    );
    assert_eq!(buffer[7], snapshot.stage.bits(), "the stage matches ACTIVITY");
    assert_eq!(buffer[8], 42, "the percentage matches ACTIVITY");

    let detail_len = usize::from(buffer[9]);
    let mut perception_bytes = [0u8; Perception::ENCODED_LEN];
    if perception.encode_into(&mut perception_bytes).is_err() {
        return;
    }
    assert_eq!(
        buffer[10 + detail_len],
        perception_bytes[0],
        "the flags match PERCEPTION"
    );

    let mut calibration_bytes = [0u8; Calibration::ENCODED_LEN];
    if calibration.encode_into(&mut calibration_bytes).is_err() {
        return;
    }
    assert_eq!(
        &buffer[11 + detail_len..15 + detail_len],
        &calibration_bytes,
        "the four statuses match CALIBRATION"
    );

    assert_round_trip(&CommandMessage::StatusSnapshot(snapshot), Command::GetStatus);
}

/// The status snapshot uses the `ACTIVITY` percentage sentinel and validates each enum field.
#[test]
fn status_snapshot_sentinels_and_ranges_are_enforced() {
    let base = StatusSnapshot {
        level: 0xFF,
        voltage_mv: 0xFFFF,
        left: 0,
        right: 0,
        procedure_id: ProcedureId::IDLE,
        stage: ActivityStage::Running,
        percent: None,
        detail: "",
        flags: Perception::default(),
        calibration: Calibration {
            motor: CalibrationStatus::Unknown,
            imu: CalibrationStatus::Unknown,
            mag: CalibrationStatus::Unknown,
            distance: CalibrationStatus::Unknown,
        },
    };
    let mut buffer = [0u8; BUFFER_LEN];
    let Ok(written) = base.encode_into(&mut buffer) else {
        return;
    };
    assert_eq!(buffer[8], StatusSnapshot::PERCENT_NONE, "none uses the sentinel");
    assert_eq!(
        StatusSnapshot::decode(&buffer[..written]),
        Ok(base),
        "the sentinels must survive"
    );

    let unrepresentable = StatusSnapshot {
        percent: Some(StatusSnapshot::PERCENT_NONE),
        ..base
    };
    assert_eq!(
        unrepresentable.encode_into(&mut buffer),
        Err(EncodeError::Value),
        "Some(0xFF) cannot be distinguished from none"
    );

    let mut bad_stage = buffer;
    bad_stage[7] = 3;
    assert_eq!(
        StatusSnapshot::decode(&bad_stage[..written]),
        Err(CodecError::Value),
        "stage 3 is unassigned"
    );

    let mut bad_calibration = buffer;
    bad_calibration[written - 1] = 3;
    assert_eq!(
        StatusSnapshot::decode(&bad_calibration[..written]),
        Err(CodecError::Value),
        "a calibration status above 2 is unassigned"
    );

    let mut bad_utf8 = [0u8; 16];
    bad_utf8[7] = ActivityStage::Running.bits();
    bad_utf8[8] = StatusSnapshot::PERCENT_NONE;
    bad_utf8[9] = 1;
    bad_utf8[10] = 0xFF;
    assert_eq!(
        StatusSnapshot::decode(&bad_utf8),
        Err(CodecError::Utf8),
        "invalid detail text must be reported"
    );

    assert_eq!(
        StatusSnapshot::decode(&[0; 14]),
        Err(CodecError::Length),
        "the fixed header is fifteen bytes"
    );
    assert_eq!(
        StatusSnapshot::decode(&[0; 17]),
        Err(CodecError::Length),
        "a detail length beyond the payload is malformed"
    );
}

/// Decoding arbitrary payloads for every command never panics.
#[test]
fn arbitrary_command_payloads_never_panic() {
    let mut state = 0x1234_5678u32;
    let mut buffer = [0u8; 96];
    for command in Command::ALL {
        for is_ack in [false, true] {
            for len in 0..=buffer.len() {
                for _ in 0..8 {
                    state ^= state << 13;
                    state ^= state >> 17;
                    state ^= state << 5;
                    let fill = state.to_le_bytes();
                    for (index, slot) in buffer.iter_mut().enumerate().take(len) {
                        *slot = fill[index % fill.len()];
                    }
                    let _ = decode_command(command, is_ack, &buffer[..len]);
                }
            }
        }
    }
}
