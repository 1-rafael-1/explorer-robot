//! Host-side tests for the floor-drop detector transitions.
//!
//! Time is supplied by the test, so each transition is checked at an exact
//! millisecond boundary.

use rangefinder::{
    DROP_THRESHOLD_MM, FloorDropDetector, FloorState, RangeStatus, RangefinderReading, Reading, ReadingHealth,
};

/// A valid floor-level reading.
const fn floor(mm: u16) -> RangefinderReading {
    RangefinderReading {
        reading: Reading::Valid(mm),
        raw_status: 11,
    }
}

/// A reading with no target.
const fn no_target() -> RangefinderReading {
    RangefinderReading {
        reading: Reading::NoTarget,
        raw_status: 0,
    }
}

/// A reading whose status reports a hardware failure.
const fn hardware_fail() -> RangefinderReading {
    RangefinderReading {
        reading: Reading::Invalid(RangeStatus::HardwareFail),
        raw_status: 1,
    }
}

#[test]
fn valid_reading_above_threshold_drops_immediately() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);

    let health = detector.update(floor(500), 50);

    assert_eq!(detector.state(), FloorState::Drop);
    assert_eq!(health, ReadingHealth::Valid);
    assert!(detector.is_drop());
}

#[test]
fn exact_threshold_is_still_floor() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);

    detector.update(floor(DROP_THRESHOLD_MM), 50);

    assert_eq!(detector.state(), FloorState::Floor);
    assert!(!detector.is_drop());
}

#[test]
fn first_valid_floor_from_unknown_sets_floor() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);

    let health = detector.update(floor(200), 50);

    assert_eq!(detector.state(), FloorState::Floor);
    assert_eq!(health, ReadingHealth::Valid);
}

#[test]
fn clearing_a_drop_requires_two_valid_floor_readings() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);
    detector.update(floor(500), 50);

    detector.update(floor(300), 100);
    assert_eq!(detector.state(), FloorState::Drop);
    assert!(detector.is_drop());

    detector.update(floor(300), 150);
    assert_eq!(detector.state(), FloorState::Floor);
    assert!(!detector.is_drop());
}

#[test]
fn invalid_and_no_target_report_unknown_then_fail_safe() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);
    detector.update(floor(200), 50);
    assert_eq!(detector.state(), FloorState::Floor);

    let health = detector.update(no_target(), 100);
    assert_eq!(detector.state(), FloorState::Unknown);
    assert_eq!(health, ReadingHealth::NoTarget);
    assert!(!detector.is_drop(), "a floor decision is held, not a drop");

    let health = detector.update(hardware_fail(), 299);
    assert_eq!(detector.state(), FloorState::Unknown);
    assert_eq!(health, ReadingHealth::Invalid);
    assert!(!detector.is_drop(), "the held floor decision survives the hold");

    detector.update(hardware_fail(), 300);
    assert_eq!(detector.state(), FloorState::Drop);
    assert!(detector.is_drop());
}

#[test]
fn unknown_is_held_for_two_hundred_ms_before_failing_safe() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);

    detector.update(no_target(), 50);
    assert_eq!(detector.state(), FloorState::Unknown);
    assert!(!detector.is_drop());

    detector.update(no_target(), 249);
    assert_eq!(detector.state(), FloorState::Unknown);

    detector.update(no_target(), 250);
    assert_eq!(detector.state(), FloorState::Drop);
    assert!(detector.is_drop());
}

#[test]
fn a_single_invalid_reading_does_not_clear_a_drop() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);
    detector.update(floor(500), 50);

    detector.update(no_target(), 100);
    assert_eq!(detector.state(), FloorState::Unknown);
    assert!(detector.is_drop(), "the held drop survives the invalid streak");

    detector.update(floor(300), 150);
    assert_eq!(detector.state(), FloorState::Drop);
    assert!(detector.is_drop());

    detector.update(floor(300), 200);
    assert_eq!(detector.state(), FloorState::Floor);
    assert!(!detector.is_drop());
}

#[test]
fn invalid_reading_from_unknown_does_not_invent_a_drop() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);

    detector.update(no_target(), 100);

    assert_eq!(detector.state(), FloorState::Unknown);
    assert!(!detector.is_drop());
}

#[test]
fn fail_safe_latches_drop_until_a_fresh_arm() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);
    detector.update(floor(200), 50);

    detector.fail_safe();
    assert_eq!(detector.state(), FloorState::Drop);
    assert!(detector.is_drop());

    detector.arm(100);
    assert_eq!(detector.state(), FloorState::Unknown);
    assert!(!detector.is_drop());
}

#[test]
fn stale_fires_after_one_hundred_fifty_ms_without_a_reading() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(1_000);

    assert!(!detector.is_stale(1_149));
    assert!(detector.is_stale(1_150));
}

#[test]
fn stale_is_false_before_the_first_reading() {
    let detector = FloorDropDetector::new(DROP_THRESHOLD_MM);

    assert!(!detector.is_stale(10_000));
}

#[test]
fn update_refreshes_the_stale_watchdog() {
    let mut detector = FloorDropDetector::new(DROP_THRESHOLD_MM);
    detector.arm(0);

    detector.update(floor(200), 100);

    assert!(!detector.is_stale(249));
    assert!(detector.is_stale(250));
}
