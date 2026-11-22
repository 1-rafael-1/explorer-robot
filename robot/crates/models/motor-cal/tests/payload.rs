//! Host-side integration tests for the `motor-cal` stored Motor Calibration
//! record.
//!
//! Every assertion is made through the crate's public API. The expected values
//! are worked examples: literal byte offsets from the documented layout, literal
//! field values, and the curve that `derive_curve` produces from the same raw
//! measurements. The decoder's own output is never used to derive an
//! expectation.

use motor_cal::{
    Curve, CurveError, DecodeError, Direction, DistanceScale, EncodeError, LegPoint, MotorCalibrationRecord,
    POWER_STEPS, RECORD_ENCODED_LEN, RECORD_FORMAT_VERSION, StepMeasurement, Sweep, TEST_LEGS, TestLeg, Track,
    derive_curve,
};

/// The Measurement Window stored with every worked-example step, in
/// milliseconds.
const WINDOW_MS: u32 = 2_000;

/// The motor rail voltage stored with every worked-example step, in volts.
const RAIL_VOLTS: f32 = 7.4;

/// The voltage compensation stored with every worked-example step.
const COMPENSATION: f32 = 0.81;

/// The ten Power Steps of a Test Leg, in percent.
const PERCENTS: [u8; POWER_STEPS] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

/// The four Test Legs in cycle order.
const ALL_LEGS: [TestLeg; TEST_LEGS] = [
    TestLeg::new(Track::Left, Direction::Forward),
    TestLeg::new(Track::Left, Direction::Reverse),
    TestLeg::new(Track::Right, Direction::Forward),
    TestLeg::new(Track::Right, Direction::Reverse),
];

/// The offset of the four Breakaway cache bytes in the encoded record.
const BREAKAWAYS_OFFSET: usize = 10;

/// The offset of the first Power Step in the encoded record.
const STEPS_OFFSET: usize = 14;

/// The encoded length of one Power Step, in bytes.
const STEP_LEN: usize = 16;

/// The offset of a step's flags byte within the step.
const STEP_FLAGS_OFFSET: usize = 15;

/// The offset of the derived-corrections cache in the encoded record.
const CORRECTIONS_OFFSET: usize = 654;

/// The header flag that says the record carries raw sweep measurements and a
/// derivable curve.
const RECORD_HAS_CURVE_FLAG: u8 = 0b0000_0001;

/// The header flag that says the record's scale came from Distance Calibration.
const RECORD_SCALE_MEASURED_FLAG: u8 = 0b0000_0010;

/// The counts of a Test Leg that moves from its first Power Step.
const BASE_COUNTS: [u16; POWER_STEPS] = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];

/// The counts of the slower right-forward Test Leg.
const SLOW_COUNTS: [u16; POWER_STEPS] = [80, 160, 240, 320, 400, 480, 560, 640, 720, 800];

/// One Test Leg of raw measurements that all showed sustained motion.
fn leg_with_counts(counts: [u16; POWER_STEPS]) -> [StepMeasurement; POWER_STEPS] {
    leg_with_breakaway(counts, PERCENTS[0])
}

/// One Test Leg whose Measurement Windows showed no sustained motion below
/// `first_moving_percent`, so its Breakaway is that Power Step.
///
/// The steps below the Breakaway still carry a real count and window: the window
/// ran, it simply did not show sustained motion.
fn leg_with_breakaway(counts: [u16; POWER_STEPS], first_moving_percent: u8) -> [StepMeasurement; POWER_STEPS] {
    let mut measurements = [StepMeasurement::new(0, 0, WINDOW_MS, RAIL_VOLTS, COMPENSATION, false); POWER_STEPS];
    for (index, percent) in PERCENTS.iter().enumerate() {
        measurements[index] = StepMeasurement::new(
            *percent,
            counts[index],
            WINDOW_MS,
            RAIL_VOLTS,
            COMPENSATION,
            *percent >= first_moving_percent,
        );
    }
    measurements
}

/// The four worked-example Test Legs: right forward is the slower leg.
fn sample_legs() -> [[StepMeasurement; POWER_STEPS]; TEST_LEGS] {
    [
        leg_with_counts(BASE_COUNTS),
        leg_with_counts(BASE_COUNTS),
        leg_with_counts(SLOW_COUNTS),
        leg_with_counts(BASE_COUNTS),
    ]
}

/// The scale every worked example was measured with.
fn scale() -> DistanceScale {
    match DistanceScale::from_counts_per_cm(17.36) {
        Ok(scale) => scale,
        Err(error) => unreachable!("17.36 counts/cm is inside the plausibility band: {error:?}"),
    }
}

/// A valid record of the worked-example sweep under run id 7.
///
/// The worked-example scale stands in for a nominal fallback, so the record is
/// unmeasured unless a test says otherwise.
fn sample_record() -> MotorCalibrationRecord {
    match MotorCalibrationRecord::new(7, scale(), sample_legs(), false) {
        Ok(record) => record,
        Err(error) => unreachable!("the worked-example sweep derives a curve: {error:?}"),
    }
}

/// Encode `record` into a fresh buffer of exactly the documented length.
fn encoded(record: &MotorCalibrationRecord) -> [u8; RECORD_ENCODED_LEN] {
    let mut buffer = [0_u8; RECORD_ENCODED_LEN];
    match record.encode_into(&mut buffer) {
        Ok(written) => assert_eq!(written, RECORD_ENCODED_LEN),
        Err(_) => unreachable!("the buffer is exactly the encoded length"),
    }
    buffer
}

/// Decode `bytes`, which must be a valid record.
fn decoded(bytes: &[u8]) -> MotorCalibrationRecord {
    match MotorCalibrationRecord::decode(bytes) {
        Ok(record) => record,
        Err(error) => unreachable!("expected a decodable record, got {error:?}"),
    }
}

/// Recompute the checksum after a deliberate byte edit, so a tampered record is
/// still integrity-valid and only its content is under test.
fn fix_checksum(bytes: &mut [u8; RECORD_ENCODED_LEN]) {
    let checksum_end = RECORD_ENCODED_LEN - 2;
    let checksum = motor_cal::payload::checksum(&bytes[..checksum_end]);
    bytes[checksum_end..].copy_from_slice(&checksum.to_le_bytes());
}

/// The curve `derive_curve` implies for the same raw measurements.
fn expected_curve(legs: &[[StepMeasurement; POWER_STEPS]; TEST_LEGS]) -> Curve {
    let mut points = [[LegPoint::new(0, 0, 0, false); POWER_STEPS]; TEST_LEGS];
    for (leg_index, measurements) in legs.iter().enumerate() {
        for (step_index, measurement) in measurements.iter().enumerate() {
            points[leg_index][step_index] = LegPoint::new(
                measurement.command_percent(),
                measurement.count(),
                u64::from(measurement.window_ms()),
                measurement.sustained_motion(),
            );
        }
    }
    match derive_curve(Sweep::new(&points[0], &points[1], &points[2], &points[3])) {
        Ok(curve) => curve,
        Err(error) => unreachable!("the worked-example sweep derives a curve: {error:?}"),
    }
}

#[test]
fn the_encoded_record_has_a_fixed_documented_length() {
    assert_eq!(RECORD_FORMAT_VERSION, 4, "the pair-measured record is version 4");
    assert_eq!(RECORD_ENCODED_LEN, 856);

    let mut buffer = [0_u8; RECORD_ENCODED_LEN];
    assert_eq!(sample_record().encode_into(&mut buffer), Ok(RECORD_ENCODED_LEN));
}

#[test]
fn the_header_is_the_version_flags_run_id_scale_and_breakaways() {
    let bytes = encoded(&sample_record());

    assert_eq!(bytes[0], RECORD_FORMAT_VERSION);
    assert_eq!(bytes[1], RECORD_HAS_CURVE_FLAG);
    assert_eq!(u32::from_le_bytes([bytes[2], bytes[3], bytes[4], bytes[5]]), 7);
    assert_eq!(f32::from_le_bytes([bytes[6], bytes[7], bytes[8], bytes[9]]), 17.36);
    assert_eq!(
        &bytes[BREAKAWAYS_OFFSET..BREAKAWAYS_OFFSET + TEST_LEGS],
        &[10, 10, 10, 10],
        "every worked-example leg moves from its first Power Step"
    );
}

#[test]
fn a_step_is_the_command_count_window_rail_compensation_and_flags() {
    let bytes = encoded(&sample_record());
    let step = STEPS_OFFSET;

    assert_eq!(bytes[step], 10);
    assert_eq!(u16::from_le_bytes([bytes[step + 1], bytes[step + 2]]), 100);
    assert_eq!(
        u32::from_le_bytes([bytes[step + 3], bytes[step + 4], bytes[step + 5], bytes[step + 6]]),
        WINDOW_MS
    );
    assert_eq!(
        f32::from_le_bytes([bytes[step + 7], bytes[step + 8], bytes[step + 9], bytes[step + 10]]),
        RAIL_VOLTS
    );
    assert_eq!(
        f32::from_le_bytes([bytes[step + 11], bytes[step + 12], bytes[step + 13], bytes[step + 14],]),
        COMPENSATION
    );
    assert_eq!(bytes[step + STEP_FLAGS_OFFSET], 1);
}

#[test]
fn a_record_round_trips_through_its_encoded_bytes() {
    let expected_legs = sample_legs();
    let record = sample_record();
    let bytes = encoded(&record);
    let record = decoded(&bytes);

    assert_eq!(record.run_id(), 7);
    assert_eq!(record.scale(), scale());
    assert!(
        !record.scale_measured(),
        "the worked example treats the scale as inferred"
    );

    for (leg_index, leg) in ALL_LEGS.iter().enumerate() {
        assert_eq!(record.leg(*leg), &expected_legs[leg_index]);
    }

    let first = record.leg(ALL_LEGS[0])[0];
    assert_eq!(first.command_percent(), 10);
    assert_eq!(first.count(), 100);
    assert_eq!(first.window_ms(), WINDOW_MS);
    assert_eq!(first.rail_volts(), RAIL_VOLTS);
    assert_eq!(first.compensation(), COMPENSATION);
    assert!(first.sustained_motion());

    assert_eq!(record.curve(), Some(&expected_curve(&expected_legs)));

    for leg in ALL_LEGS {
        assert_eq!(record.breakaway(leg), Some(10));
    }
}

#[test]
fn the_breakaway_is_the_lowest_power_step_that_showed_sustained_motion() {
    let legs = [
        leg_with_breakaway(BASE_COUNTS, 30),
        leg_with_breakaway(BASE_COUNTS, 10),
        leg_with_breakaway(SLOW_COUNTS, 50),
        leg_with_breakaway(BASE_COUNTS, 20),
    ];
    let record = match MotorCalibrationRecord::new(7, scale(), legs, false) {
        Ok(record) => record,
        Err(error) => unreachable!("the legs derive a curve: {error:?}"),
    };

    assert_eq!(record.breakaway(ALL_LEGS[0]), Some(30));
    assert_eq!(record.breakaway(ALL_LEGS[1]), Some(10));
    assert_eq!(record.breakaway(ALL_LEGS[2]), Some(50));
    assert_eq!(record.breakaway(ALL_LEGS[3]), Some(20));

    let bytes = encoded(&record);
    assert_eq!(
        &bytes[BREAKAWAYS_OFFSET..BREAKAWAYS_OFFSET + TEST_LEGS],
        &[30, 10, 50, 20]
    );

    let decoded = decoded(&bytes);
    assert_eq!(decoded.breakaway(ALL_LEGS[0]), Some(30));
    assert_eq!(decoded.breakaway(ALL_LEGS[1]), Some(10));
    assert_eq!(decoded.breakaway(ALL_LEGS[2]), Some(50));
    assert_eq!(decoded.breakaway(ALL_LEGS[3]), Some(20));
}

#[test]
fn a_tampered_breakaway_cache_is_ignored_in_favour_of_the_raw_measurements() {
    let mut bytes = encoded(&sample_record());
    // The stored Breakaways are a cache: the raw sustained-motion flags are the
    // source of truth, so a rewritten copy must not change what decode reports.
    for (index, byte) in bytes[BREAKAWAYS_OFFSET..BREAKAWAYS_OFFSET + TEST_LEGS]
        .iter_mut()
        .enumerate()
    {
        *byte = if index % 2 == 0 { 0xFF } else { 0 };
    }
    fix_checksum(&mut bytes);

    let record = decoded(&bytes);
    for leg in ALL_LEGS {
        assert_eq!(record.breakaway(leg), Some(10));
    }
}

#[test]
fn a_scale_only_record_has_no_breakaway() {
    let record = MotorCalibrationRecord::scale_only(9, scale());
    let bytes = encoded(&record);

    assert_eq!(&bytes[BREAKAWAYS_OFFSET..BREAKAWAYS_OFFSET + TEST_LEGS], &[0, 0, 0, 0]);
    for leg in ALL_LEGS {
        assert_eq!(record.breakaway(leg), None);
        assert_eq!(decoded(&bytes).breakaway(leg), None);
    }
}

#[test]
fn a_scale_only_record_round_trips_with_its_scale_and_no_curve() {
    let record = MotorCalibrationRecord::scale_only(9, scale());
    let bytes = encoded(&record);

    assert_eq!(
        bytes[1], RECORD_SCALE_MEASURED_FLAG,
        "a scale-only record clears the has-curve flag but marks the scale measured"
    );

    let record = decoded(&bytes);
    assert_eq!(record.run_id(), 9);
    assert_eq!(record.scale(), scale());
    assert!(record.scale_measured());
    assert_eq!(record.curve(), None);
}

#[test]
fn a_scale_only_record_ignores_the_step_region_it_cannot_vouch_for() {
    // The fixed layout reserves the step and cache regions. A scale-only record
    // says they mean nothing, so even bytes that would be invalid step flags
    // cannot invent a curve.
    let mut bytes = encoded(&MotorCalibrationRecord::scale_only(1, scale()));
    for byte in &mut bytes[STEPS_OFFSET..RECORD_ENCODED_LEN - 2] {
        *byte = 0xFF;
    }
    fix_checksum(&mut bytes);

    let record = decoded(&bytes);
    assert_eq!(record.scale(), scale());
    assert_eq!(record.curve(), None);
}

#[test]
fn an_unknown_header_flag_is_rejected() {
    let mut bytes = encoded(&MotorCalibrationRecord::scale_only(1, scale()));
    bytes[1] = 0b1000_0000;
    fix_checksum(&mut bytes);

    assert_eq!(
        MotorCalibrationRecord::decode(&bytes).err(),
        Some(DecodeError::InvalidHeaderFlags { found: 0b1000_0000 })
    );
}

#[test]
fn a_full_record_takes_a_new_scale_without_losing_its_measurements() {
    let new_scale = match DistanceScale::from_counts_per_cm(18.5) {
        Ok(scale) => scale,
        Err(error) => unreachable!("18.5 counts/cm is inside the plausibility band: {error:?}"),
    };

    let record = sample_record().with_scale(new_scale);
    let record = decoded(&encoded(&record));

    assert_eq!(record.scale(), new_scale);
    assert!(record.scale_measured(), "a new Distance Calibration scale is measured");
    assert_eq!(record.curve(), Some(&expected_curve(&sample_legs())));
}

#[test]
fn a_measured_scale_round_trips_as_measured() {
    let record = match MotorCalibrationRecord::new(7, scale(), sample_legs(), true) {
        Ok(record) => record,
        Err(error) => unreachable!("the worked-example sweep derives a curve: {error:?}"),
    };
    let bytes = encoded(&record);

    assert_eq!(bytes[1], RECORD_HAS_CURVE_FLAG | RECORD_SCALE_MEASURED_FLAG);
    assert!(decoded(&bytes).scale_measured());
}

#[test]
fn an_unmeasured_scale_round_trips_as_unmeasured() {
    let record = match MotorCalibrationRecord::new(7, scale(), sample_legs(), false) {
        Ok(record) => record,
        Err(error) => unreachable!("the worked-example sweep derives a curve: {error:?}"),
    };
    let bytes = encoded(&record);

    assert_eq!(bytes[1], RECORD_HAS_CURVE_FLAG);
    assert!(!decoded(&bytes).scale_measured());
}

#[test]
fn a_scale_only_record_marks_its_scale_as_measured() {
    let record = MotorCalibrationRecord::scale_only(9, scale());
    assert!(record.scale_measured());

    let bytes = encoded(&record);
    assert_eq!(bytes[1], RECORD_SCALE_MEASURED_FLAG);
    assert!(decoded(&bytes).scale_measured());
}

#[test]
fn taking_a_new_scale_marks_it_as_measured() {
    let unmeasured = match MotorCalibrationRecord::new(7, scale(), sample_legs(), false) {
        Ok(record) => record,
        Err(error) => unreachable!("the worked-example sweep derives a curve: {error:?}"),
    };
    assert!(!unmeasured.scale_measured());

    assert!(
        unmeasured.with_scale(scale()).scale_measured(),
        "Distance Calibration always supplies a measured scale"
    );
}

#[test]
fn encoding_into_a_too_small_buffer_is_rejected() {
    let record = sample_record();
    let mut buffer = [0_u8; RECORD_ENCODED_LEN - 1];

    assert_eq!(
        record.encode_into(&mut buffer),
        Err(EncodeError::BufferTooSmall {
            needed: RECORD_ENCODED_LEN
        })
    );
}

#[test]
fn a_record_from_an_unknown_schema_version_is_rejected() {
    let mut bytes = encoded(&sample_record());
    let unknown = RECORD_FORMAT_VERSION.wrapping_add(1);
    bytes[0] = unknown;
    fix_checksum(&mut bytes);

    assert_eq!(
        MotorCalibrationRecord::decode(&bytes).err(),
        Some(DecodeError::UnsupportedVersion { found: unknown })
    );
}

#[test]
fn a_version_3_single_track_record_is_not_calibrated() {
    // Version 3 was measured one track at a time. It is rejected rather than
    // migrated: the simultaneous-pair curve model needs pair measurements.
    let mut bytes = encoded(&sample_record());
    bytes[0] = 3;
    fix_checksum(&mut bytes);

    assert_eq!(
        MotorCalibrationRecord::decode(&bytes).err(),
        Some(DecodeError::UnsupportedVersion { found: 3 })
    );
}

#[test]
fn a_truncated_record_is_rejected() {
    let bytes = encoded(&sample_record());

    assert_eq!(
        MotorCalibrationRecord::decode(&bytes[..RECORD_ENCODED_LEN - 1]).err(),
        Some(DecodeError::Length {
            actual: RECORD_ENCODED_LEN - 1
        })
    );
}

#[test]
fn a_corrupt_record_fails_the_checksum() {
    let mut bytes = encoded(&sample_record());
    // One flipped bit is enough: the stale checksum no longer matches.
    bytes[2] ^= 0x01;

    assert!(matches!(
        MotorCalibrationRecord::decode(&bytes).err(),
        Some(DecodeError::ChecksumMismatch { .. })
    ));
}

#[test]
fn an_unknown_step_flag_is_rejected() {
    let mut bytes = encoded(&sample_record());
    bytes[STEPS_OFFSET + STEP_FLAGS_OFFSET] = 0b1000_0000;
    fix_checksum(&mut bytes);

    assert_eq!(
        MotorCalibrationRecord::decode(&bytes).err(),
        Some(DecodeError::InvalidFlags {
            leg: ALL_LEGS[0],
            command_percent: 10,
            found: 0b1000_0000,
        })
    );
}

#[test]
fn a_tampered_corrections_cache_still_decodes_to_the_curve_the_raw_measurements_imply() {
    let mut bytes = encoded(&sample_record());
    for byte in &mut bytes[CORRECTIONS_OFFSET..RECORD_ENCODED_LEN - 2] {
        *byte = 0xA5;
    }
    fix_checksum(&mut bytes);

    let record = decoded(&bytes);
    assert_eq!(record.curve(), Some(&expected_curve(&sample_legs())));
    assert_eq!(record.run_id(), 7);
}

#[test]
fn a_record_whose_raw_measurements_do_not_derive_a_curve_is_rejected() {
    let mut bytes = encoded(&sample_record());
    // Clear the sustained-motion flag of the last leg's 100 % step: the leg then
    // did not move at 100 %, so no curve can be derived even though the cached
    // corrections are still intact.
    let last_step = STEPS_OFFSET + (TEST_LEGS * POWER_STEPS - 1) * STEP_LEN;
    bytes[last_step + STEP_FLAGS_OFFSET] = 0;
    fix_checksum(&mut bytes);

    assert_eq!(
        MotorCalibrationRecord::decode(&bytes).err(),
        Some(DecodeError::Curve(CurveError::LegDidNotMove {
            leg: ALL_LEGS[TEST_LEGS - 1],
        }))
    );
}

#[test]
fn the_checksum_is_crc_16_ccitt_false() {
    // The standard check value for "123456789".
    assert_eq!(motor_cal::payload::checksum(b"123456789"), 0x29B1);
    assert_eq!(motor_cal::payload::checksum(b""), 0xFFFF);
}

#[test]
fn failure_reasons_name_themselves_for_the_log() {
    assert_eq!(
        EncodeError::BufferTooSmall { needed: 856 }.label(),
        "record buffer too small"
    );
    assert_eq!(DecodeError::Length { actual: 850 }.label(), "wrong record length");
    assert_eq!(
        DecodeError::UnsupportedVersion { found: 99 }.label(),
        "unsupported record version"
    );
    assert_eq!(
        DecodeError::InvalidHeaderFlags { found: 0x80 }.label(),
        "invalid record header flags"
    );
    assert_eq!(
        DecodeError::ChecksumMismatch {
            expected: 0x1234,
            found: 0x5678,
        }
        .label(),
        "record checksum mismatch"
    );
    assert_eq!(
        DecodeError::InvalidFlags {
            leg: ALL_LEGS[0],
            command_percent: 10,
            found: 0xFF,
        }
        .label(),
        "invalid record flags"
    );
    assert_eq!(
        DecodeError::Curve(CurveError::LegDidNotMove { leg: ALL_LEGS[0] }).label(),
        "record curve is invalid"
    );
}
