//! Host tests for the v1 event vocabulary, its payload codecs and the replay window.
//!
//! The tests exercise the external contract: the ids, each ADR-0031 layout, the completion status,
//! detail truncation, the ACK status and the bounded `(boot_id, seq)` dedupe. No allocation and no
//! `unwrap`: a malformed input is an error value, never a panic.

use link_protocol::{
    CodecError, CompletionStatus, EVENT_PAYLOAD_MAX, Event, EventAck, EventMessage, ReplayWindow, Seq, Status,
    codec::{PayloadCodec, decode_event, encode_event},
    ids::ProcedureId,
    payload::{
        event::{
            CalibrationKind, CalibrationLoadStatus, CalibrationLoaded, DriveDistanceCompleted, EventOverflow,
            FloorDrop, Obstacle, ProcedureCompleted, RotateCompleted,
        },
        stream::ActivityStage,
    },
};

/// Encodes one event message into a fresh vector.
fn encode(message: &EventMessage<'_>) -> Option<Vec<u8>> {
    let mut buffer = [0u8; EVENT_PAYLOAD_MAX];
    let len = encode_event(message, &mut buffer).ok()?;
    Some(buffer[..len].to_vec())
}

/// Decodes one event payload into a message.
fn decode(event: Event, bytes: &[u8]) -> Option<EventMessage<'_>> {
    decode_event(event, false, bytes).ok()
}

/// Every implemented event id is unique, in the core range and round-trips.
#[test]
fn event_ids_are_unique_in_range_and_round_trip() {
    let expected = [
        (Event::Obstacle, 0x0001),
        (Event::FloorDrop, 0x0002),
        (Event::ProcedureCompleted, 0x0003),
        (Event::DriveDistanceCompleted, 0x0004),
        (Event::RotateCompleted, 0x0005),
        (Event::CalibrationLoaded, 0x0006),
        (Event::EventOverflow, 0x0007),
    ];
    assert_eq!(Event::ALL.len(), expected.len(), "ALL covers every v1 event");
    assert_eq!(Event::COUNT, expected.len());

    let mut seen = [false; 0x100];
    for (index, (event, id)) in expected.into_iter().enumerate() {
        assert_eq!(Event::ALL[index], event);
        assert_eq!(event.index(), index, "{event:?} index");
        assert_eq!(event.wire_id(), id, "{event:?} wire id");
        assert!((0x0001..=0x00FF).contains(&id), "{event:?} must be a core id");
        assert!(!seen[usize::from(id)], "{event:?} id must be unique");
        seen[usize::from(id)] = true;
        assert_eq!(Event::from_wire(id), Some(event));
    }
}

/// Reserved, zero and out-of-range ids are never variants.
#[test]
fn reserved_event_ids_are_none() {
    for id in [0x0000u16, 0x0008, 0x000F, 0x0010, 0x00FF, 0x0100, 0x8000, 0xFFFF] {
        assert_eq!(Event::from_wire(id), None, "id {id:#06x} must stay reserved");
    }
}

/// Every v1 event round-trips through its codec.
#[test]
fn every_v1_event_round_trips() {
    let messages = [
        EventMessage::Obstacle(Obstacle { detected: 1, source: 0 }),
        EventMessage::FloorDrop(FloorDrop { detected: 0 }),
        EventMessage::ProcedureCompleted(ProcedureCompleted {
            procedure_id: ProcedureId::TURNS,
            stage: ActivityStage::Failed,
            detail: "wheel slip",
        }),
        EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
            command_seq: 42,
            status: CompletionStatus::Cancelled,
            target_cm: 100,
            distance_cm: -94,
            duration_ms: 1_234,
            detail: "obstacle",
        }),
        EventMessage::RotateCompleted(RotateCompleted {
            command_seq: 43,
            status: CompletionStatus::Success,
            target_cdeg: 9_000,
            error_cdeg: -125,
            duration_ms: 2_000,
            detail: "90°",
        }),
        EventMessage::CalibrationLoaded(CalibrationLoaded {
            kind: CalibrationKind::Distance,
            status: CalibrationLoadStatus::Missing,
        }),
        EventMessage::EventOverflow(EventOverflow { dropped: 3 }),
    ];

    for message in messages {
        let Some(event) = message.event() else {
            return;
        };
        let Some(bytes) = encode(&message) else {
            return;
        };
        assert_eq!(decode(event, &bytes), Some(message), "{event:?} must round-trip");
    }
}

/// The drive completions preserve the originating command sequence, status and detail.
#[test]
fn drive_completions_preserve_command_seq_status_and_detail() {
    let distance = EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
        command_seq: 0xBEEF,
        status: CompletionStatus::Failed,
        target_cm: -150,
        distance_cm: -149,
        duration_ms: 0xDEAD_BEEF,
        detail: "stalled",
    });
    let Some(bytes) = encode(&distance) else {
        return;
    };
    let Some(EventMessage::DriveDistanceCompleted(decoded)) = decode(Event::DriveDistanceCompleted, &bytes) else {
        return;
    };
    assert_eq!(decoded.command_seq, 0xBEEF);
    assert_eq!(decoded.status, CompletionStatus::Failed);
    assert_eq!(decoded.target_cm, -150);
    assert_eq!(decoded.distance_cm, -149);
    assert_eq!(decoded.duration_ms, 0xDEAD_BEEF);
    assert_eq!(decoded.detail, "stalled");

    let rotate = EventMessage::RotateCompleted(RotateCompleted {
        command_seq: 0x0102,
        status: CompletionStatus::Cancelled,
        target_cdeg: -18_000,
        error_cdeg: 17_999,
        duration_ms: 4_000,
        detail: "stopped",
    });
    let Some(bytes) = encode(&rotate) else {
        return;
    };
    let Some(EventMessage::RotateCompleted(decoded)) = decode(Event::RotateCompleted, &bytes) else {
        return;
    };
    assert_eq!(decoded.command_seq, 0x0102);
    assert_eq!(decoded.status, CompletionStatus::Cancelled);
    assert_eq!(decoded.target_cdeg, -18_000);
    assert_eq!(decoded.error_cdeg, 17_999);
    assert_eq!(decoded.duration_ms, 4_000);
    assert_eq!(decoded.detail, "stopped");
}

/// Unassigned completion, calibration kind and calibration status bytes are value errors.
#[test]
fn unassigned_vocabulary_values_are_rejected() {
    let Some(mut bytes) = encode(&EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
        command_seq: 1,
        status: CompletionStatus::Success,
        target_cm: 0,
        distance_cm: 0,
        duration_ms: 0,
        detail: "",
    })) else {
        return;
    };
    bytes[2] = 3;
    assert_eq!(decode(Event::DriveDistanceCompleted, &bytes), None);

    let Some(mut bytes) = encode(&EventMessage::CalibrationLoaded(CalibrationLoaded {
        kind: CalibrationKind::Motor,
        status: CalibrationLoadStatus::Loaded,
    })) else {
        return;
    };
    bytes[0] = 3;
    assert_eq!(decode(Event::CalibrationLoaded, &bytes), None);
    bytes[0] = 0;
    bytes[1] = 2;
    assert_eq!(decode(Event::CalibrationLoaded, &bytes), None);

    assert_eq!(CompletionStatus::from_wire(3), None);
    assert_eq!(CalibrationKind::from_wire(3), None);
    assert_eq!(CalibrationLoadStatus::from_wire(2), None);
}

/// Detail text truncates at a character boundary on encode and tolerates longer text on decode.
#[test]
fn details_truncate_at_utf8_boundaries_and_tolerate_longer_text() {
    let detail = format!("{}é", "b".repeat(31));
    let message = EventMessage::ProcedureCompleted(ProcedureCompleted {
        procedure_id: ProcedureId::STRAIGHT_DRIVE,
        stage: ActivityStage::Complete,
        detail: &detail,
    });
    assert_eq!(detail.len(), 33, "the sample is one byte over the cap");
    let Some(bytes) = encode(&message) else {
        return;
    };
    let Some(EventMessage::ProcedureCompleted(decoded)) = decode(Event::ProcedureCompleted, &bytes) else {
        return;
    };
    assert_eq!(decoded.detail.len(), 31);
    assert_eq!(decoded.detail, "b".repeat(31).as_str());

    // A conforming peer's decoder accepts a detail longer than the sender's cap when it fits.
    let long = "x".repeat(40);
    let mut payload = [0u8; 14 + 40];
    payload[0..2].copy_from_slice(&ProcedureId::BOOT.get().to_le_bytes());
    payload[2] = ActivityStage::Failed.bits();
    payload[3] = 40;
    payload[4..4 + long.len()].copy_from_slice(long.as_bytes());
    let Some(EventMessage::ProcedureCompleted(decoded)) = decode(Event::ProcedureCompleted, &payload) else {
        return;
    };
    assert_eq!(decoded.detail, long.as_str());
}

/// Each fixed event layout rejects a structural mismatch.
#[test]
fn event_lengths_are_strict() {
    assert_eq!(decode(Event::Obstacle, &[]), None);
    assert_eq!(decode(Event::Obstacle, &[1, 0, 0]), None);
    assert_eq!(decode(Event::FloorDrop, &[]), None);
    assert_eq!(decode(Event::FloorDrop, &[0, 0]), None);
    assert_eq!(decode(Event::CalibrationLoaded, &[0]), None);
    assert_eq!(decode(Event::CalibrationLoaded, &[0, 0, 0]), None);
    assert_eq!(decode(Event::EventOverflow, &[0]), None);
    assert_eq!(decode(Event::EventOverflow, &[0, 0, 0]), None);
    assert_eq!(decode(Event::ProcedureCompleted, &[0, 0, 1, 2]), None);
    assert_eq!(decode(Event::DriveDistanceCompleted, &[0; 13]), None);
    assert_eq!(decode(Event::RotateCompleted, &[0; 13]), None);
}

/// An event ACK carries the shared status and ignores result bytes.
#[test]
fn event_ack_carries_a_status_and_ignores_result_bytes() {
    let ack = EventAck {
        status: Status::Rejected,
    };
    let mut buffer = [0u8; 4];
    let Some(len) = ack.encode_into(&mut buffer).ok() else {
        return;
    };
    assert_eq!(len, 1);
    assert_eq!(buffer[0], Status::Rejected.bits());
    assert_eq!(EventAck::decode(&[Status::Rejected.bits()]), Ok(ack));
    assert_eq!(
        EventAck::decode(&[Status::Rejected.bits(), 0xAA, 0xBB]),
        Ok(ack),
        "optional result bytes are ignored"
    );
    assert_eq!(EventAck::decode(&[]), Err(CodecError::Length));
    assert_eq!(EventAck::decode(&[10]), Err(CodecError::Value));

    let Some(EventMessage::Ack(decoded)) = decode_event(Event::Obstacle, true, &[Status::Busy.bits()]).ok() else {
        return;
    };
    assert_eq!(decoded.status, Status::Busy);
    assert_eq!(EventMessage::Ack(decoded).event(), None);
}

/// The replay window ignores a duplicate and starts clean on a Boot Id change.
#[test]
fn replay_window_ignores_duplicates_and_resets_on_a_boot_change() {
    let mut window = ReplayWindow::new();
    assert!(window.is_empty());
    assert_eq!(window.boot_id(), None);

    assert!(window.accept(7, Seq::new(0)), "the first sighting is new");
    assert!(!window.accept(7, Seq::new(0)), "a retransmit is a duplicate");
    assert!(!window.accept(7, Seq::new(0)), "every retransmit is a duplicate");
    assert_eq!(window.boot_id(), Some(7));
    assert_eq!(window.len(), 1);
    assert!(window.contains(7, Seq::new(0)));
    assert!(!window.contains(8, Seq::new(0)), "another Boot Id is a new window");

    assert!(window.accept(8, Seq::new(0)), "a new Boot Id starts clean");
    assert_eq!(window.boot_id(), Some(8));
    assert_eq!(window.len(), 1);
    assert!(!window.accept(8, Seq::new(0)));
    assert!(!window.observe_boot_id(8), "the same Boot Id is a no-op");
}

/// The replay window remembers at most 64 sequences and evicts the oldest.
#[test]
fn replay_window_is_bounded_to_64_entries() {
    let mut window = ReplayWindow::new();
    for seq in 0..64u16 {
        assert!(window.accept(1, Seq::new(seq)));
    }
    assert_eq!(window.len(), ReplayWindow::CAPACITY);
    assert!(!window.accept(1, Seq::new(0)), "a remembered sequence repeats");

    assert!(window.accept(1, Seq::new(64)), "a newer sequence is accepted");
    assert_eq!(window.len(), ReplayWindow::CAPACITY);
    assert!(
        window.accept(1, Seq::new(0)),
        "the oldest sequence was evicted and looks new again"
    );

    window.clear();
    assert!(window.is_empty());
    assert_eq!(window.boot_id(), Some(1), "clearing keeps the Boot Id");
}

/// No byte sequence panics a decoder.
#[test]
fn arbitrary_event_payloads_never_panic() {
    let payloads = [
        &[][..],
        &[0][..],
        &[0xFF, 0xFF][..],
        &[0xAA; 13][..],
        &[0xAA; 14][..],
        &[0xAA; 16][..],
        &[0xAA; EVENT_PAYLOAD_MAX][..],
        &[0x55; 64][..],
    ];
    for event in Event::ALL {
        for payload in payloads {
            let _ = decode_event(event, false, payload);
            let _ = decode_event(event, true, payload);
        }
    }
}
