//! Host-side tests for the per-sensor lifecycle and recovery budget.

use rangefinder::{
    DROP_THRESHOLD_MM, FloorState, RangefinderReading, Reading, ReadingHealth, RecoveryCause, SensorLifecycle,
    SensorState,
};

/// A valid floor-level reading.
const fn floor(mm: u16) -> RangefinderReading {
    RangefinderReading {
        reading: Reading::Valid(mm),
        raw_status: 11,
    }
}

/// A reading whose status reports a hardware failure.
const fn hardware_fail() -> RangefinderReading {
    RangefinderReading {
        reading: Reading::Invalid(rangefinder::RangeStatus::HardwareFail),
        raw_status: 1,
    }
}

#[test]
fn a_fresh_enable_waits_for_the_first_valid_reading() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    assert_eq!(lifecycle.state(), SensorState::Waiting);
    assert_eq!(lifecycle.floor(), FloorState::Unknown);

    let _ = lifecycle.on_reading(floor(200), 50);
    assert_eq!(lifecycle.state(), SensorState::Armed);
    assert_eq!(lifecycle.floor(), FloorState::Floor);
}

#[test]
fn an_invalid_reading_does_not_arm_a_waiting_sensor() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    lifecycle.on_reading(hardware_fail(), 50);

    assert_eq!(lifecycle.state(), SensorState::Waiting);
    assert_eq!(lifecycle.floor(), FloorState::Unknown);
}

#[test]
fn the_attempt_budget_is_three_per_enable() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    assert_eq!(lifecycle.begin_attempt(), Some(1));
    assert_eq!(lifecycle.begin_attempt(), Some(2));
    assert_eq!(lifecycle.begin_attempt(), Some(3));
    assert_eq!(lifecycle.begin_attempt(), None);

    lifecycle.begin_enable(100);
    assert_eq!(lifecycle.begin_attempt(), Some(1), "a fresh enable resets the budget");
}

#[test]
fn a_successful_attempt_replenishes_the_budget() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    assert_eq!(lifecycle.begin_attempt(), Some(1));
    assert_eq!(lifecycle.begin_attempt(), Some(2));
    lifecycle.attempt_succeeded(100);

    assert_eq!(
        lifecycle.begin_attempt(),
        Some(1),
        "a success starts a fresh run of consecutive failures"
    );
}

#[test]
fn a_failed_sensor_has_no_recovery_cause() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);
    lifecycle.mark_failed();

    assert_eq!(lifecycle.state(), SensorState::Failed);
    assert_eq!(lifecycle.recovery_cause(10_000), None);
}

#[test]
fn starting_recovery_from_armed_fails_safe() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);
    lifecycle.on_reading(floor(200), 50);

    lifecycle.begin_recovery();

    assert_eq!(lifecycle.state(), SensorState::Recovering);
    assert!(lifecycle.is_drop(), "an armed sensor fails safe while it resets");
}

#[test]
fn starting_recovery_from_waiting_does_not_invent_a_drop() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    lifecycle.begin_recovery();

    assert_eq!(lifecycle.state(), SensorState::Recovering);
    assert!(!lifecycle.is_drop());
}

#[test]
fn a_successful_attempt_retains_a_held_drop_until_two_valid_floors() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);
    lifecycle.on_reading(floor(200), 50);
    lifecycle.begin_recovery();
    assert!(lifecycle.is_drop());

    lifecycle.attempt_succeeded(100);

    assert_eq!(lifecycle.state(), SensorState::Waiting);
    assert!(
        lifecycle.is_drop(),
        "a reset must not release protection before fresh readings"
    );

    lifecycle.on_reading(floor(200), 150);
    assert!(lifecycle.is_drop(), "one valid floor reading is not enough");

    lifecycle.on_reading(floor(200), 200);
    assert!(!lifecycle.is_drop(), "two consecutive valid floors clear the drop");
}

#[test]
fn a_stale_sensor_is_a_recovery_cause_waiting_or_armed() {
    for armed in [false, true] {
        let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
        lifecycle.begin_enable(0);
        if armed {
            lifecycle.on_reading(floor(200), 50);
        }

        let last = if armed { 50 } else { 0 };
        assert_eq!(lifecycle.recovery_cause(last + 149), None);
        assert_eq!(
            lifecycle.recovery_cause(last + 150),
            Some(RecoveryCause::Stale),
            "armed={armed}"
        );
    }
}

#[test]
fn a_sustained_invalid_status_is_a_recovery_cause_once_armed() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);
    lifecycle.on_reading(floor(200), 50);

    lifecycle.on_reading(hardware_fail(), 100);
    assert_eq!(lifecycle.recovery_cause(150), None, "the invalid hold is still running");

    lifecycle.on_reading(hardware_fail(), 300);
    assert_eq!(lifecycle.recovery_cause(300), Some(RecoveryCause::InvalidStreak));
}

#[test]
fn a_disable_resets_the_lifecycle_and_the_detector() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);
    lifecycle.on_reading(floor(500), 50);
    assert!(lifecycle.is_drop());

    lifecycle.begin_disable();

    assert_eq!(lifecycle.state(), SensorState::Waiting);
    assert_eq!(lifecycle.floor(), FloorState::Unknown);
    assert!(!lifecycle.is_drop());
}
