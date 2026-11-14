//! Host-side tests for the raw range-status decode.

use rangefinder::{RangeStatus, RangefinderReading, Reading};

#[test]
fn every_raw_code_decodes_to_its_status() {
    let expected = [
        RangeStatus::NoRange,
        RangeStatus::HardwareFail,
        RangeStatus::HardwareFail,
        RangeStatus::HardwareFail,
        RangeStatus::SignalFail,
        RangeStatus::Unknown(5),
        RangeStatus::PhaseFail,
        RangeStatus::Unknown(7),
        RangeStatus::MinRangeFail,
        RangeStatus::PhaseFail,
        RangeStatus::MinRangeFail,
        RangeStatus::Valid,
        RangeStatus::Unknown(12),
        RangeStatus::Unknown(13),
        RangeStatus::Unknown(14),
        RangeStatus::Unknown(15),
    ];

    for (raw, status) in expected.into_iter().enumerate() {
        let raw = u8::try_from(raw).unwrap_or(u8::MAX);
        assert_eq!(RangeStatus::from_raw(raw), status, "raw {raw}");
    }
}

#[test]
fn out_of_range_is_no_target_even_when_the_status_is_valid() {
    let sample = RangefinderReading::decode(11 << 3, 8190);

    assert_eq!(sample.reading, Reading::NoTarget);
    assert_eq!(sample.raw_status, 11);
}

#[test]
fn valid_status_with_a_range_is_a_valid_reading() {
    let sample = RangefinderReading::decode(11 << 3, 1234);

    assert_eq!(sample.reading, Reading::Valid(1234));
}

#[test]
fn raw_status_is_preserved_as_the_four_bit_code() {
    // Bits 0, 1 and 7 are not part of the status field and must be masked off.
    let sample = RangefinderReading::decode(0b1100_1011, 500);

    assert_eq!(sample.raw_status, 9);
    assert_eq!(sample.reading, Reading::Invalid(RangeStatus::PhaseFail));
}

#[test]
fn no_range_status_that_is_not_out_of_range_is_no_target() {
    let sample = RangefinderReading::decode(0, 1234);

    assert_eq!(sample.reading, Reading::NoTarget);
}

#[test]
fn hardware_failure_with_a_real_range_stays_invalid() {
    let sample = RangefinderReading::decode(1 << 3, 1234);

    assert_eq!(sample.reading, Reading::Invalid(RangeStatus::HardwareFail));
    assert_eq!(sample.raw_status, 1);
}

#[test]
fn out_of_range_takes_precedence_over_a_failure_status() {
    let sample = RangefinderReading::decode(1 << 3, 8190);

    assert_eq!(sample.reading, Reading::NoTarget);
    assert_eq!(sample.raw_status, 1);
}

#[test]
fn unknown_codes_are_preserved() {
    let sample = RangefinderReading::decode(14 << 3, 700);

    assert_eq!(sample.reading, Reading::Invalid(RangeStatus::Unknown(14)));
    assert_eq!(sample.raw_status, 14);
}
