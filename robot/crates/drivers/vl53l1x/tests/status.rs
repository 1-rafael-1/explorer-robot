//! Host-side tests for the pure status decode and reading classification.

use vl53l1x::{RangeStatus, Reading};

#[test]
fn decodes_every_status_code() {
    assert_eq!(RangeStatus::from_raw(8, 1), RangeStatus::MinRangeClipped);
    assert_eq!(RangeStatus::from_raw(9, 1), RangeStatus::Valid);
    assert_eq!(RangeStatus::from_raw(9, 0), RangeStatus::NoWrapCheck);
    assert_eq!(RangeStatus::from_raw(4, 1), RangeStatus::SignalFail);
    assert_eq!(RangeStatus::from_raw(5, 1), RangeStatus::OutOfBounds);
    assert_eq!(RangeStatus::from_raw(13, 1), RangeStatus::MinRangeFail);
    assert_eq!(RangeStatus::from_raw(6, 1), RangeStatus::SigmaFail);
    assert_eq!(RangeStatus::from_raw(7, 1), RangeStatus::WrapFail);
    assert_eq!(RangeStatus::from_raw(12, 1), RangeStatus::XtalkFail);
    assert_eq!(RangeStatus::from_raw(1, 1), RangeStatus::HardwareFail);
    assert_eq!(RangeStatus::from_raw(2, 1), RangeStatus::HardwareFail);
    assert_eq!(RangeStatus::from_raw(3, 1), RangeStatus::HardwareFail);
    assert_eq!(RangeStatus::from_raw(17, 1), RangeStatus::HardwareFail);
    assert_eq!(RangeStatus::from_raw(255, 1), RangeStatus::NoUpdate);
    assert_eq!(RangeStatus::from_raw(18, 1), RangeStatus::Synchronization);
}

#[test]
fn decodes_unknown_codes_verbatim() {
    for raw in [0, 10, 11, 14, 15, 16, 19, 254] {
        assert_eq!(RangeStatus::from_raw(raw, 1), RangeStatus::Unknown(raw));
    }
}

#[test]
fn faults_are_only_the_recovery_set() {
    assert!(RangeStatus::HardwareFail.is_fault());
    assert!(RangeStatus::NoUpdate.is_fault());
    assert!(RangeStatus::Unknown(0).is_fault());

    assert!(!RangeStatus::Valid.is_fault());
    assert!(!RangeStatus::MinRangeClipped.is_fault());
    assert!(!RangeStatus::NoWrapCheck.is_fault());
    assert!(!RangeStatus::SignalFail.is_fault());
    assert!(!RangeStatus::OutOfBounds.is_fault());
    assert!(!RangeStatus::MinRangeFail.is_fault());
    assert!(!RangeStatus::Synchronization.is_fault());
    assert!(
        !RangeStatus::SigmaFail.is_fault(),
        "sigma is a scene condition, not a wedged sensor"
    );
    assert!(
        !RangeStatus::WrapFail.is_fault(),
        "a wrapped far target is a scene condition, not a wedged sensor"
    );
    assert!(
        !RangeStatus::XtalkFail.is_fault(),
        "crosstalk is a scene condition, not a wedged sensor"
    );
}

#[test]
fn valid_and_min_clip_produce_valid_readings() {
    assert_eq!(Reading::from_status(RangeStatus::Valid, 1234), Reading::Valid(1234));
    assert_eq!(
        Reading::from_status(RangeStatus::MinRangeClipped, 95),
        Reading::Valid(95)
    );
}

#[test]
fn no_target_statuses_produce_no_target() {
    assert_eq!(Reading::from_status(RangeStatus::SignalFail, 0), Reading::NoTarget);
    assert_eq!(Reading::from_status(RangeStatus::OutOfBounds, 0), Reading::NoTarget);
    assert_eq!(Reading::from_status(RangeStatus::MinRangeFail, 0), Reading::NoTarget);
}

#[test]
fn no_wrap_check_is_deliberately_no_target() {
    assert_eq!(Reading::from_status(RangeStatus::NoWrapCheck, 900), Reading::NoTarget);
}

#[test]
fn invalid_statuses_produce_invalid_readings() {
    for status in [
        RangeStatus::Synchronization,
        RangeStatus::NoUpdate,
        RangeStatus::HardwareFail,
        RangeStatus::SigmaFail,
        RangeStatus::WrapFail,
        RangeStatus::XtalkFail,
        RangeStatus::Unknown(0),
    ] {
        assert_eq!(Reading::from_status(status, 500), Reading::Invalid(status));
    }
}
