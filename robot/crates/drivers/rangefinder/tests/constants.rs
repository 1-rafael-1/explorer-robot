//! Host-side tests that pin the documented constants.

use rangefinder::{
    CLEAR_SAMPLES, DROP_THRESHOLD_MM, GATE_TIMEOUT_MS, INVALID_HOLD_MS, OUT_OF_RANGE_MILLIMETERS, RANGING_PERIOD_MS,
    RESET_ATTEMPTS, STALE_TIMEOUT_MS, TEST_PUBLISH_HZ, TIMING_BUDGET_US,
};

#[test]
fn constants_match_the_adr() {
    assert_eq!(DROP_THRESHOLD_MM, 400);
    assert_eq!(RANGING_PERIOD_MS, 50);
    assert_eq!(TIMING_BUDGET_US, 33_000);
    assert_eq!(INVALID_HOLD_MS, 200);
    assert_eq!(STALE_TIMEOUT_MS, 150);
    assert_eq!(CLEAR_SAMPLES, 2);
    assert_eq!(RESET_ATTEMPTS, 3);
    assert_eq!(GATE_TIMEOUT_MS, 1_000);
    assert_eq!(TEST_PUBLISH_HZ, 10);
    assert_eq!(OUT_OF_RANGE_MILLIMETERS, 8190);
}
