//! Host-side tests for the per-sensor lifecycle and recovery budget.

use rangefinder::{
    DROP_THRESHOLD_MM, FloorState, RangeDiagnostics, RangeStatus, RangefinderReading, Reading, RecoveryCause,
    SensorLifecycle, SensorState,
};

/// Build a reading from its decoded parts.
const fn reading(reading: Reading, status: RangeStatus, raw_status: u8) -> RangefinderReading {
    RangefinderReading {
        reading,
        status,
        raw_status,
        diagnostics: RangeDiagnostics {
            signal_rate: 0,
            ambient_rate: 0,
            effective_spads: 0,
        },
    }
}

/// A valid floor-level reading.
const fn floor(mm: u16) -> RangefinderReading {
    reading(Reading::Valid(mm), RangeStatus::Valid, 9)
}

/// A reading whose status reports a hardware failure.
const fn hardware_fail() -> RangefinderReading {
    reading(
        Reading::Invalid(RangeStatus::HardwareFail),
        RangeStatus::HardwareFail,
        1,
    )
}

/// Arm a lifecycle with one valid floor reading at 50 ms.
fn armed_lifecycle() -> SensorLifecycle {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);
    lifecycle.on_reading(floor(200), 50);
    lifecycle
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
fn a_successful_attempt_does_not_replenish_the_budget() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    assert_eq!(lifecycle.begin_attempt(), Some(1));
    lifecycle.attempt_succeeded(100);

    assert_eq!(
        lifecycle.begin_attempt(),
        Some(2),
        "an init that never produces a valid reading must still exhaust the budget"
    );
    assert_eq!(lifecycle.begin_attempt(), Some(3));
    assert_eq!(lifecycle.begin_attempt(), None);
}

#[test]
fn a_valid_reading_replenishes_the_budget() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    assert_eq!(lifecycle.begin_attempt(), Some(1));
    assert_eq!(lifecycle.begin_attempt(), Some(2));
    lifecycle.attempt_succeeded(100);
    assert_eq!(lifecycle.begin_attempt(), Some(3));

    lifecycle.on_reading(floor(200), 150);

    assert_eq!(
        lifecycle.begin_attempt(),
        Some(1),
        "a valid reading proves recovery and starts a fresh run of attempts"
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
fn a_sustained_no_wrap_check_holds_and_fails_safe_without_recovery() {
    let mut lifecycle = armed_lifecycle();

    lifecycle.on_reading(reading(Reading::NoTarget, RangeStatus::NoWrapCheck, 9), 100);
    assert_eq!(lifecycle.recovery_cause(150), None, "the invalid hold is still running");
    assert_eq!(lifecycle.floor(), FloorState::Unknown);

    lifecycle.on_reading(reading(Reading::NoTarget, RangeStatus::NoWrapCheck, 9), 300);
    assert_eq!(lifecycle.floor(), FloorState::Drop, "the streak fails safe");
    assert_eq!(
        lifecycle.recovery_cause(300),
        None,
        "a no-wrap-check reading must not reset a healthy sensor"
    );
}

#[test]
fn a_sustained_synchronization_holds_and_fails_safe_without_recovery() {
    let mut lifecycle = armed_lifecycle();

    lifecycle.on_reading(
        reading(
            Reading::Invalid(RangeStatus::Synchronization),
            RangeStatus::Synchronization,
            18,
        ),
        100,
    );
    assert_eq!(lifecycle.recovery_cause(150), None, "the invalid hold is still running");
    assert_eq!(lifecycle.floor(), FloorState::Unknown);

    lifecycle.on_reading(
        reading(
            Reading::Invalid(RangeStatus::Synchronization),
            RangeStatus::Synchronization,
            18,
        ),
        300,
    );
    assert_eq!(lifecycle.floor(), FloorState::Drop, "the streak fails safe");
    assert_eq!(
        lifecycle.recovery_cause(300),
        None,
        "a synchronization reading must not reset a healthy sensor"
    );
}

#[test]
fn a_sustained_no_target_status_never_starts_recovery() {
    for (status, raw_status) in [
        (RangeStatus::SignalFail, 4),
        (RangeStatus::OutOfBounds, 5),
        (RangeStatus::MinRangeFail, 13),
    ] {
        let mut lifecycle = armed_lifecycle();

        lifecycle.on_reading(reading(Reading::NoTarget, status, raw_status), 100);
        lifecycle.on_reading(reading(Reading::NoTarget, status, raw_status), 300);

        assert_eq!(lifecycle.floor(), FloorState::Drop, "{status:?} fails safe");
        assert_eq!(
            lifecycle.recovery_cause(300),
            None,
            "{status:?} must not reset a healthy sensor"
        );
    }
}

#[test]
fn a_sustained_fault_status_starts_recovery() {
    for (status, raw_status) in [
        (RangeStatus::HardwareFail, 1),
        (RangeStatus::NoUpdate, 255),
        (RangeStatus::Unknown(0), 0),
    ] {
        assert!(status.is_fault(), "{status:?} is a Rangefinder Fault");

        let mut lifecycle = armed_lifecycle();

        lifecycle.on_reading(reading(Reading::Invalid(status), status, raw_status), 100);
        assert_eq!(lifecycle.recovery_cause(150), None, "the invalid hold is still running");

        lifecycle.on_reading(reading(Reading::Invalid(status), status, raw_status), 300);
        assert_eq!(
            lifecycle.recovery_cause(300),
            Some(RecoveryCause::InvalidStreak),
            "{status:?} must drive recovery"
        );
    }
}

#[test]
fn a_sustained_scene_status_holds_and_fails_safe_without_recovery() {
    for (status, raw_status) in [
        (RangeStatus::SigmaFail, 6),
        (RangeStatus::WrapFail, 7),
        (RangeStatus::XtalkFail, 12),
    ] {
        assert!(!status.is_fault(), "{status:?} is a scene condition, not a fault");

        let mut lifecycle = armed_lifecycle();

        lifecycle.on_reading(reading(Reading::Invalid(status), status, raw_status), 100);
        assert_eq!(lifecycle.recovery_cause(150), None, "the invalid hold is still running");
        assert_eq!(lifecycle.floor(), FloorState::Unknown);

        lifecycle.on_reading(reading(Reading::Invalid(status), status, raw_status), 300);
        assert_eq!(lifecycle.floor(), FloorState::Drop, "{status:?} fails safe");
        assert_eq!(
            lifecycle.recovery_cause(300),
            None,
            "{status:?} must not reset a healthy sensor"
        );
    }
}

#[test]
fn a_waiting_sensor_recovers_from_a_sustained_fault() {
    let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
    lifecycle.begin_enable(0);

    lifecycle.on_reading(hardware_fail(), 100);
    assert_eq!(lifecycle.state(), SensorState::Waiting);
    assert_eq!(lifecycle.recovery_cause(150), None, "the invalid hold is still running");

    lifecycle.on_reading(hardware_fail(), 300);
    assert_eq!(lifecycle.floor(), FloorState::Drop, "the fault streak fails safe");
    assert_eq!(
        lifecycle.recovery_cause(300),
        Some(RecoveryCause::InvalidStreak),
        "a sensor that never arms on a fault must still be reset"
    );
}

#[test]
fn a_waiting_sensor_does_not_recover_from_non_fault_invalids() {
    for (status, raw_status) in [
        (RangeStatus::NoWrapCheck, 9),
        (RangeStatus::Synchronization, 18),
        (RangeStatus::SignalFail, 4),
        (RangeStatus::OutOfBounds, 5),
        (RangeStatus::MinRangeFail, 13),
    ] {
        let mut lifecycle = SensorLifecycle::new(DROP_THRESHOLD_MM);
        lifecycle.begin_enable(0);

        for now in [100, 300] {
            let reading = match status {
                RangeStatus::Synchronization => reading(Reading::Invalid(status), status, raw_status),
                _ => reading(Reading::NoTarget, status, raw_status),
            };
            lifecycle.on_reading(reading, now);
        }

        assert_eq!(lifecycle.floor(), FloorState::Drop, "{status:?} fails safe");
        assert_eq!(
            lifecycle.recovery_cause(300),
            None,
            "{status:?} must not reset a waiting sensor"
        );
    }
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
