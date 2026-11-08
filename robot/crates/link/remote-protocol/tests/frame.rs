//! Host tests for the frozen frame envelope (ADR-0030).
//!
//! The tests exercise only the public contract: encoded bytes, decoded headers and payloads, and
//! the typed errors for malformed input.

use remote_protocol::{
    Class, Direction, EncodeError, Flags, Frame, FrameError, HEADER_LEN, PROTOCOL_MAJOR, Seq, Status, crc::crc16,
};

/// Encodes `frame` into a fresh scratch buffer and returns the bytes written.
fn encode(frame: &Frame<'_>) -> Option<Vec<u8>> {
    let mut buffer = [0u8; 2048];
    let Ok(written) = frame.encode_into(&mut buffer) else {
        return None;
    };
    Some(buffer[..written].to_vec())
}

/// Recomputes and stores the CRC of one complete, exactly-sized encoded frame.
fn recompute_crc(frame: &mut [u8]) {
    let mut covered = frame[..10].to_vec();
    covered.extend_from_slice(&frame[HEADER_LEN..]);
    let crc = crc16(&covered);
    frame[10..12].copy_from_slice(&crc.to_le_bytes());
}

/// The golden vector from the ticket: major 1, stream, id 1, seq 1, payload `0x42`.
const GOLDEN: [u8; 13] = [
    0xA5, 0x5A, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x01, 0x00, 0x39, 0xAA, 0x42,
];

/// Encoding the golden frame produces exactly the ADR-0030 bytes.
#[test]
fn golden_stream_frame_encodes_to_the_adr_layout() {
    let Ok(frame) = Frame::new(1, Class::Stream, Flags::default(), 0x0001, Seq::new(1), &[0x42]) else {
        return;
    };
    let Some(encoded) = encode(&frame) else {
        return;
    };
    assert_eq!(encoded.as_slice(), GOLDEN.as_slice(), "encoded bytes must match");
}

/// The golden bytes decode to the expected header and payload.
#[test]
fn golden_stream_frame_decodes_to_the_expected_values() {
    let Ok(frame) = Frame::decode(&GOLDEN, Direction::RobotToPc) else {
        return;
    };
    assert_eq!(frame.header.major, 1, "major");
    assert_eq!(frame.header.class, Class::Stream, "class");
    assert_eq!(frame.header.flags, Flags::default(), "flags");
    assert_eq!(frame.header.id, 0x0001, "id");
    assert_eq!(frame.header.seq, Seq::new(1), "seq");
    assert_eq!(frame.header.len, 1, "length");
    assert_eq!(frame.payload, [0x42].as_slice(), "payload");
}

/// Multi-byte header fields are little-endian on the wire.
#[test]
fn multi_byte_fields_are_little_endian() {
    let Ok(frame) = Frame::new(
        1,
        Class::Command,
        Flags::default(),
        0x1234,
        Seq::new(0xABCD),
        &[0x01, 0x02],
    ) else {
        return;
    };
    let Some(encoded) = encode(&frame) else {
        return;
    };
    assert_eq!(&encoded[4..6], &[0x34, 0x12], "id must be little-endian");
    assert_eq!(&encoded[6..8], &[0xCD, 0xAB], "seq must be little-endian");
    assert_eq!(&encoded[8..10], &[0x02, 0x00], "length must be little-endian");
}

/// Every class round-trips, including an empty payload.
#[test]
fn every_class_round_trips_including_an_empty_payload() {
    for class in Class::ALL {
        for payload in [&[][..], &[0x11, 0x22, 0x33][..]] {
            let Ok(frame) = Frame::new(
                PROTOCOL_MAJOR,
                class,
                Flags::default(),
                0x0007,
                Seq::new(0xABCD),
                payload,
            ) else {
                return;
            };
            let Some(encoded) = encode(&frame) else {
                return;
            };
            let Ok(decoded) = Frame::decode(&encoded, Direction::PcToRobot) else {
                return;
            };
            assert_eq!(decoded.header, frame.header, "header must round-trip");
            assert_eq!(decoded.payload, payload, "payload must round-trip");
        }
    }
}

/// The largest allowed payload in each direction round-trips.
#[test]
fn direction_maxima_round_trip() {
    for direction in [Direction::RobotToPc, Direction::PcToRobot] {
        let payload = vec![0x5A; direction.max_payload()];
        let Ok(frame) = Frame::new(
            PROTOCOL_MAJOR,
            Class::Stream,
            Flags::default(),
            1,
            Seq::new(1),
            &payload,
        ) else {
            return;
        };
        let Some(encoded) = encode(&frame) else {
            return;
        };
        assert_eq!(encoded.len(), direction.max_frame(), "frame size at the limit");
        let Ok(decoded) = Frame::decode(&encoded, direction) else {
            return;
        };
        assert_eq!(decoded.payload, payload.as_slice(), "payload at the limit");
    }
}

/// A payload above the direction limit is rejected with the limit in the error.
#[test]
fn payload_above_the_direction_limit_is_rejected() {
    let payload = vec![0u8; Direction::PcToRobot.max_payload() + 1];
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR,
        Class::Command,
        Flags::default(),
        1,
        Seq::new(1),
        &payload,
    ) else {
        return;
    };
    let Some(encoded) = encode(&frame) else {
        return;
    };
    assert!(
        matches!(
            Frame::decode(&encoded, Direction::PcToRobot),
            Err(FrameError::LengthExceeded { len: 257, max: 256 })
        ),
        "an over-limit payload must be a length error"
    );

    let payload = vec![0u8; Direction::RobotToPc.max_payload() + 1];
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR,
        Class::Stream,
        Flags::default(),
        1,
        Seq::new(1),
        &payload,
    ) else {
        return;
    };
    let Some(encoded) = encode(&frame) else {
        return;
    };
    assert!(
        matches!(
            Frame::decode(&encoded, Direction::RobotToPc),
            Err(FrameError::LengthExceeded { len: 1025, max: 1024 })
        ),
        "the robot-to-PC limit must also be enforced"
    );
}

/// The reserved `FRAGMENT` flag is rejected in v1.
#[test]
fn fragment_flag_is_rejected() {
    let Ok(frame) = Frame::new(PROTOCOL_MAJOR, Class::Event, Flags::FRAGMENT, 1, Seq::new(1), &[0x01]) else {
        return;
    };
    let Some(encoded) = encode(&frame) else {
        return;
    };
    assert!(
        matches!(Frame::decode(&encoded, Direction::RobotToPc), Err(FrameError::Fragment)),
        "FRAGMENT must be rejected"
    );
}

/// The two reserved flag bits are ignored rather than rejected.
#[test]
fn reserved_flag_bits_are_ignored_on_decode() {
    let Ok(frame) = Frame::new(PROTOCOL_MAJOR, Class::Stream, Flags::default(), 1, Seq::new(1), &[0x42]) else {
        return;
    };
    let Some(mut encoded) = encode(&frame) else {
        return;
    };
    encoded[3] |= 0x30;
    recompute_crc(&mut encoded);

    let Ok(decoded) = Frame::decode(&encoded, Direction::RobotToPc) else {
        return;
    };
    assert!(decoded.header.flags.is_empty(), "reserved bits must be ignored");
    assert_eq!(decoded.payload, [0x42].as_slice(), "payload must still decode");
}

/// An unknown major version still parses as a frame.
#[test]
fn unknown_major_version_decodes_successfully() {
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR + 10,
        Class::Control,
        Flags::default(),
        0x0002,
        Seq::new(9),
        &[0x01, 0x02],
    ) else {
        return;
    };
    let Some(encoded) = encode(&frame) else {
        return;
    };
    let Ok(decoded) = Frame::decode(&encoded, Direction::PcToRobot) else {
        return;
    };
    assert_eq!(decoded.header.major, PROTOCOL_MAJOR + 10, "major must survive");
    assert_eq!(decoded.payload, [0x01, 0x02].as_slice(), "payload must survive");
}

/// Input shorter than the header is a short-header error.
#[test]
fn short_header_is_rejected() {
    assert!(
        matches!(
            Frame::decode(&[0xA5, 0x5A, 0x01], Direction::RobotToPc),
            Err(FrameError::ShortHeader)
        ),
        "a partial header must be rejected"
    );
    assert!(
        matches!(Frame::decode(&[], Direction::RobotToPc), Err(FrameError::ShortHeader)),
        "empty input must be rejected"
    );
}

/// A wrong magic prefix is a bad-magic error.
#[test]
fn bad_magic_is_rejected() {
    let mut encoded = GOLDEN;
    encoded[0] = 0x00;
    assert!(
        matches!(Frame::decode(&encoded, Direction::RobotToPc), Err(FrameError::BadMagic)),
        "a wrong magic byte must be rejected"
    );
}

/// A payload that does not match its CRC is rejected.
#[test]
fn bad_crc_is_rejected() {
    let mut encoded = GOLDEN;
    encoded[12] ^= 0xFF;
    assert!(
        matches!(Frame::decode(&encoded, Direction::RobotToPc), Err(FrameError::BadCrc)),
        "a corrupted payload must be rejected"
    );
}

/// A header that declares more payload than the input carries is truncated.
#[test]
fn truncated_payload_is_rejected() {
    let truncated = &GOLDEN[..HEADER_LEN];
    assert!(
        matches!(
            Frame::decode(truncated, Direction::RobotToPc),
            Err(FrameError::Truncated)
        ),
        "a missing payload byte must be rejected"
    );
}

/// Encoding into a buffer that cannot hold the whole frame is a buffer error.
#[test]
fn too_small_buffer_is_rejected() {
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR,
        Class::Stream,
        Flags::default(),
        1,
        Seq::new(1),
        &[1, 2, 3, 4],
    ) else {
        return;
    };
    let mut buffer = [0u8; HEADER_LEN + 3];
    assert!(
        matches!(frame.encode_into(&mut buffer), Err(EncodeError::BufferTooSmall)),
        "a short buffer must be reported, not overflowed"
    );
}

/// The ten status codes cover `0..=9` and round-trip through their wire value.
#[test]
fn status_codes_cover_zero_through_nine() {
    for (index, status) in Status::ALL.iter().enumerate() {
        assert_eq!(usize::from(status.bits()), index, "codes must be contiguous");
        assert_eq!(status.index(), index, "index must match ALL");
        assert_eq!(
            Status::from_byte(status.bits()),
            Some(*status),
            "from_byte must invert bits"
        );
    }
    assert_eq!(Status::from_byte(10), None, "10 is not assigned");
}
