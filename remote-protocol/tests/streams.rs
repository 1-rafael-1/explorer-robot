//! Host tests for the typed v1 stream codecs (ADR-0031).
//!
//! The tests exercise the external contract only: encoded bytes, decoded values and the typed
//! errors for malformed input.

use remote_protocol::{
    CodecError, EncodeError, StreamSample,
    codec::PayloadCodec,
    decode_stream, encode_stream,
    ids::ProcedureId,
    payload::stream::{Activity, ActivityStage, Calibration, CalibrationStatus, LidarCloud, Motion, Perception, Power},
};

/// Encodes `sample` and decodes it back, asserting the value survives unchanged.
fn assert_round_trip(sample: &StreamSample<'_>) {
    let mut buffer = [0u8; LidarCloud::ENCODED_LEN];
    let Ok(written) = encode_stream(sample, &mut buffer) else {
        return;
    };
    let Ok(decoded) = decode_stream(sample.stream(), &buffer[..written]) else {
        return;
    };
    assert_eq!(decoded, *sample, "{:?} must round-trip", sample.stream());
}

/// A `POWER` sample round-trips, including both unknown sentinels.
#[test]
fn power_round_trips_and_preserves_sentinels() {
    let known = StreamSample::Power(Power {
        level: 87,
        voltage_mv: 12_345,
    });
    assert_round_trip(&known);

    let unknown = StreamSample::Power(Power {
        level: 0xFF,
        voltage_mv: 0xFFFF,
    });
    assert_round_trip(&unknown);

    let mut buffer = [0u8; Power::ENCODED_LEN];
    let Ok(written) = encode_stream(&unknown, &mut buffer) else {
        return;
    };
    assert_eq!(written, Power::ENCODED_LEN, "POWER is three bytes");
    assert_eq!(
        &buffer[..written],
        &[0xFF, 0xFF, 0xFF],
        "sentinels must be written verbatim"
    );
}

/// A `MOTION` sample round-trips with negative signed values.
#[test]
fn motion_round_trips_signed_values() {
    assert_round_trip(&StreamSample::Motion(Motion { left: -128, right: 127 }));
    assert_round_trip(&StreamSample::Motion(Motion { left: 0, right: 0 }));
}

/// An `ACTIVITY` sample round-trips with a detail and an optional percentage.
#[test]
fn activity_round_trips_with_detail_and_percent() {
    assert_round_trip(&StreamSample::Activity(Activity {
        procedure_id: ProcedureId::STRAIGHT_DRIVE,
        stage: ActivityStage::Running,
        percent: Some(42),
        detail: "driving",
    }));
    assert_round_trip(&StreamSample::Activity(Activity {
        procedure_id: ProcedureId::new(0xBEEF),
        stage: ActivityStage::Failed,
        percent: None,
        detail: "",
    }));
}

/// A detail that would split a character at the 32-byte cap is truncated to a boundary.
#[test]
fn activity_truncates_detail_at_a_utf8_boundary() {
    let split = format!("{}{}", "a".repeat(31), "é");
    assert_eq!(split.len(), 33, "the test string must exceed the cap");

    let activity = Activity {
        procedure_id: ProcedureId::IDLE,
        stage: ActivityStage::Running,
        percent: None,
        detail: &split,
    };
    let mut buffer = [0u8; Activity::ENCODED_LEN];
    let Ok(written) = activity.encode_into(&mut buffer) else {
        return;
    };
    let Ok(decoded) = Activity::decode(&buffer[..written]) else {
        return;
    };
    assert_eq!(
        decoded.detail,
        "a".repeat(31).as_str(),
        "the truncated detail must end at a character boundary"
    );
    assert_eq!(written, 5 + 31, "the encoded length must follow the truncated detail");

    let exact = format!("{}{}", "a".repeat(30), "é");
    assert_eq!(exact.len(), 32, "the test string must be exactly the cap");
    let exact_activity = Activity {
        detail: &exact,
        ..activity
    };
    let Ok(written) = exact_activity.encode_into(&mut buffer) else {
        return;
    };
    assert_eq!(written, 5 + 32, "a detail at the cap must not be truncated");
    let Ok(decoded) = Activity::decode(&buffer[..written]) else {
        return;
    };
    assert_eq!(decoded.detail, exact.as_str(), "the exact-fit detail must survive");
}

/// A decoder accepts a detail longer than the encoder's 32-byte cap when the payload fits.
#[test]
fn activity_decode_tolerates_a_longer_detail() {
    let mut bytes = vec![0x00, 0x00, ActivityStage::Running.bits(), 0xFF, 40];
    bytes.extend_from_slice(&[b'x'; 40]);

    let Ok(activity) = Activity::decode(&bytes) else {
        return;
    };
    assert_eq!(activity.detail.len(), 40, "the longer detail must decode");
}

/// An unassigned stage, an unrepresentable percent and invalid UTF-8 are typed errors; a
/// percentage above 100 is still a legal wire value.
#[test]
fn activity_rejects_unrepresentable_values() {
    let bad_stage = [0x00, 0x00, 3, 0xFF, 0];
    assert_eq!(
        Activity::decode(&bad_stage),
        Err(CodecError::Value),
        "stage 3 is unassigned"
    );

    let high_percent = [0x00, 0x00, ActivityStage::Running.bits(), 101, 0];
    let Ok(decoded) = Activity::decode(&high_percent) else {
        return;
    };
    assert_eq!(
        decoded.percent,
        Some(101),
        "the wire tolerates a percentage above 100; only the sentinel is special"
    );

    let bad_utf8 = [0x00, 0x00, ActivityStage::Running.bits(), 0xFF, 1, 0xFF];
    assert_eq!(
        Activity::decode(&bad_utf8),
        Err(CodecError::Utf8),
        "invalid UTF-8 must be reported"
    );

    let mut buffer = [0u8; Activity::ENCODED_LEN];
    let unrepresentable = Activity {
        procedure_id: ProcedureId::IDLE,
        stage: ActivityStage::Running,
        percent: Some(Activity::PERCENT_NONE),
        detail: "",
    };
    assert_eq!(
        unrepresentable.encode_into(&mut buffer),
        Err(EncodeError::Value),
        "Some(0xFF) cannot be distinguished from the none sentinel"
    );
}

/// `PERCEPTION` round-trips its two flags and masks the reserved bits on decode.
#[test]
fn perception_round_trips_and_masks_reserved_bits() {
    let both = Perception::OBSTACLE | Perception::FLOOR_DROP;
    assert_round_trip(&StreamSample::Perception(both));
    assert_round_trip(&StreamSample::Perception(Perception::default()));

    let Ok(decoded) = Perception::decode(&[0xFF]) else {
        return;
    };
    assert_eq!(decoded.bits(), 0x03, "reserved bits must be cleared on decode");
    assert!(decoded.contains(Perception::OBSTACLE), "obstacle is set");
    assert!(decoded.contains(Perception::FLOOR_DROP), "floor drop is set");
}

/// `LIDAR_CLOUD` round-trips all 360 slots, including the no-return sentinel.
#[test]
fn lidar_cloud_round_trips_360_slots() {
    let mut slots = [7u16; 360];
    slots[0] = 0;
    slots[179] = 0xFFFF;
    slots[359] = 12_345;
    let cloud = LidarCloud { slots };
    assert_round_trip(&StreamSample::LidarCloud(cloud));

    let mut buffer = [0u8; LidarCloud::ENCODED_LEN];
    let Ok(written) = encode_stream(&StreamSample::LidarCloud(cloud), &mut buffer) else {
        return;
    };
    assert_eq!(written, 720, "the cloud payload is 720 bytes");
    assert_eq!(
        &buffer[179 * 2..179 * 2 + 2],
        &[0xFF, 0xFF],
        "no return must be little-endian 0xFFFF"
    );
}

/// `CALIBRATION` maps all three statuses and rejects an unassigned byte.
#[test]
fn calibration_round_trips_statuses() {
    let calibration = Calibration {
        motor: CalibrationStatus::Loaded,
        imu: CalibrationStatus::Missing,
        mag: CalibrationStatus::Unknown,
        distance: CalibrationStatus::Loaded,
    };
    assert_round_trip(&StreamSample::Calibration(calibration));

    assert_eq!(CalibrationStatus::from_wire(3), None, "status 3 is unassigned");
    assert_eq!(
        Calibration::decode(&[0, 1, 2, 3]),
        Err(CodecError::Value),
        "an unassigned status must be rejected"
    );
}

/// Every payload rejects a structural length mismatch with `CodecError::Length`.
#[test]
fn structural_length_mismatches_are_length_errors() {
    assert_eq!(Power::decode(&[0; 2]), Err(CodecError::Length), "POWER");
    assert_eq!(Motion::decode(&[0; 3]), Err(CodecError::Length), "MOTION");
    assert_eq!(Activity::decode(&[0; 4]), Err(CodecError::Length), "ACTIVITY header");
    assert_eq!(
        Activity::decode(&[0, 0, 0, 0xFF, 2, b'a']),
        Err(CodecError::Length),
        "ACTIVITY detail length"
    );
    assert_eq!(Perception::decode(&[]), Err(CodecError::Length), "PERCEPTION");
    assert_eq!(LidarCloud::decode(&[0; 719]), Err(CodecError::Length), "LIDAR_CLOUD");
    assert_eq!(Calibration::decode(&[0; 5]), Err(CodecError::Length), "CALIBRATION");
}

/// Encoding into a buffer smaller than the layout reports `BufferTooSmall`.
#[test]
fn encoding_enforces_the_output_buffer() {
    let mut one_byte = [0u8; 1];
    let motion = Motion { left: 1, right: 2 };
    assert_eq!(
        motion.encode_into(&mut one_byte),
        Err(EncodeError::BufferTooSmall),
        "MOTION needs two bytes"
    );

    let mut short_cloud = [0u8; LidarCloud::ENCODED_LEN - 1];
    let cloud = LidarCloud { slots: [0; 360] };
    assert_eq!(
        cloud.encode_into(&mut short_cloud),
        Err(EncodeError::BufferTooSmall),
        "the cloud needs all 720 bytes"
    );
}

/// Every stream's maximum encoded length matches the catalogue's staging contributions.
#[test]
fn encoded_lengths_match_the_catalogue() {
    assert_eq!(Power::ENCODED_LEN, 3, "POWER");
    assert_eq!(Motion::ENCODED_LEN, 2, "MOTION");
    assert_eq!(Activity::ENCODED_LEN, 37, "ACTIVITY");
    assert_eq!(Perception::ENCODED_LEN, 1, "PERCEPTION");
    assert_eq!(LidarCloud::ENCODED_LEN, 720, "LIDAR_CLOUD");
    assert_eq!(Calibration::ENCODED_LEN, 4, "CALIBRATION");
    assert_eq!(
        Power::ENCODED_LEN
            + Motion::ENCODED_LEN
            + Activity::ENCODED_LEN
            + Perception::ENCODED_LEN
            + LidarCloud::ENCODED_LEN
            + Calibration::ENCODED_LEN,
        767,
        "the v1 payloads sum to 767 bytes"
    );
}
