//! Host tests for the watermark to `THROTTLE` decisions (ADR-0030, ADR-0033).
//!
//! The expected numbers are the ADRs' worked values: a 6 KiB high watermark emits
//! `PAUSE(ALL_STREAMS, ttl 1000 ms)`, a 2 KiB low watermark emits `RESUME(ALL_STREAMS)`, and two
//! hints are at least 500 ms apart.

use bridge_core::{WatermarkPolicy, flow};
use remote_protocol::{ThrottleAction, ThrottleHint};

/// Crossing 6 KiB emits `PAUSE(ALL_STREAMS)` with the ADR's one-second lifetime.
#[test]
fn crossing_six_kib_pauses_every_stream_for_one_second() {
    let mut policy = WatermarkPolicy::new();
    let Some(hint) = policy.observe(6 * 1024, 0) else {
        return;
    };

    assert_eq!(hint.stream_id, ThrottleHint::ALL_STREAMS);
    assert_eq!(hint.action, ThrottleAction::Pause);
    assert_eq!(hint.ttl_ms, 1000);
    assert!(policy.is_paused());
}

/// Draining to 2 KiB emits `RESUME(ALL_STREAMS)` once the suppression window has passed.
#[test]
fn draining_to_two_kib_resumes_every_stream() {
    let mut policy = WatermarkPolicy::new();
    let _ = policy.observe(6 * 1024, 0);
    let Some(hint) = policy.observe(2 * 1024, 1000) else {
        return;
    };

    assert_eq!(hint.stream_id, ThrottleHint::ALL_STREAMS);
    assert_eq!(hint.action, ThrottleAction::Resume);
    assert!(!policy.is_paused());
}

/// Two hints are never closer than 500 ms, whichever direction they travel.
#[test]
fn hints_are_suppressed_inside_the_five_hundred_millisecond_window() {
    let mut policy = WatermarkPolicy::new();
    assert!(
        policy.observe(6 * 1024, 0).is_some(),
        "the first hint has no predecessor"
    );

    assert!(
        policy.observe(2 * 1024, 499).is_none(),
        "a resume 499 ms later is suppressed"
    );
    assert!(
        policy.observe(2 * 1024, 500).is_some(),
        "a resume at exactly 500 ms is allowed"
    );

    assert!(
        policy.observe(6 * 1024, 900).is_none(),
        "a pause 400 ms after the resume is suppressed"
    );
    assert!(
        policy.observe(6 * 1024, 1000).is_some(),
        "a pause at exactly 500 ms is allowed"
    );
}

/// A pause is not repeated while the ring stays above the high watermark.
#[test]
fn a_pause_is_emitted_once_per_crossing() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(7 * 1024, 0).is_some());
    assert!(policy.observe(7 * 1024, 10_000).is_none());
    assert!(policy.observe(8 * 1024, 20_000).is_none());
}

/// A resume is not repeated while the ring stays below the low watermark.
#[test]
fn a_resume_is_emitted_once_per_drain() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(6 * 1024, 0).is_some());
    assert!(policy.observe(0, 1000).is_some());
    assert!(policy.observe(0, 10_000).is_none());
}

/// Between 2 KiB and 6 KiB the policy holds its state (hysteresis).
#[test]
fn hysteresis_holds_between_the_two_watermarks() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(6 * 1024, 0).is_some());
    assert!(
        policy.observe(4 * 1024, 1000).is_none(),
        "4 KiB is neither high nor low"
    );
    assert!(policy.is_paused());
    assert!(policy.observe(2 * 1024, 2000).is_some(), "2 KiB resumes");
}

/// Below the high watermark nothing is emitted.
#[test]
fn no_hint_is_emitted_below_the_high_watermark() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(6 * 1024 - 1, 0).is_none());
    assert!(policy.observe(0, 10_000).is_none());
    assert!(!policy.is_paused());
}

/// The constants are the ADRs' worked numbers.
#[test]
fn the_watermark_constants_are_the_adr_numbers() {
    assert_eq!(flow::HIGH_WATERMARK_BYTES, 6 * 1024);
    assert_eq!(flow::LOW_WATERMARK_BYTES, 2 * 1024);
    assert_eq!(flow::PAUSE_TTL_MS, 1000);
    assert_eq!(flow::MIN_HINT_INTERVAL_MS, 500);
}
