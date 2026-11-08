//! Host tests for the watermark to `THROTTLE` decisions (ADR-0030, ADR-0033).
//!
//! The expected numbers are the ADRs' worked values: a 6 KiB high watermark emits
//! `PAUSE(ALL_STREAMS, ttl 1000 ms)`, a 2 KiB low watermark emits `RESUME(ALL_STREAMS)`, two hints
//! are at least 500 ms apart, and a pause is refreshed while the ring stays above the low marks.

use bridge_core::{WatermarkPolicy, flow};
use link_protocol::{ThrottleAction, ThrottleHint};

/// Builds an occupancy from bytes and queued entries.
const fn occupancy(bytes: usize, entries: usize) -> flow::Occupancy {
    flow::Occupancy { bytes, entries }
}

/// Crossing 6 KiB emits `PAUSE(ALL_STREAMS)` with the ADR's one-second lifetime.
#[test]
fn crossing_six_kib_pauses_every_stream_for_one_second() {
    let mut policy = WatermarkPolicy::new();
    let Some(hint) = policy.observe(occupancy(6 * 1024, 0), 0) else {
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
    let _ = policy.observe(occupancy(6 * 1024, 0), 0);
    let Some(hint) = policy.observe(occupancy(2 * 1024, 0), 1000) else {
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
        policy.observe(occupancy(6 * 1024, 0), 0).is_some(),
        "the first hint has no predecessor"
    );

    assert!(
        policy.observe(occupancy(2 * 1024, 0), 499).is_none(),
        "a resume 499 ms later is suppressed"
    );
    assert!(
        policy.observe(occupancy(2 * 1024, 0), 500).is_some(),
        "a resume at exactly 500 ms is allowed"
    );

    assert!(
        policy.observe(occupancy(6 * 1024, 0), 900).is_none(),
        "a pause 400 ms after the resume is suppressed"
    );
    assert!(
        policy.observe(occupancy(6 * 1024, 0), 1000).is_some(),
        "a pause at exactly 500 ms is allowed"
    );
}

/// A stall longer than the pause TTL is refreshed: the robot must not resume while the ring stays
/// above the low marks, and a resume still needs both of them.
#[test]
fn a_pause_is_refreshed_while_the_ring_stays_above_the_low_marks() {
    let mut policy = WatermarkPolicy::new();
    let high = occupancy(7 * 1024, 8);
    assert_eq!(
        policy.observe(high, 0).map(|hint| hint.action),
        Some(ThrottleAction::Pause)
    );

    assert!(
        policy.observe(high, 499).is_none(),
        "a refresh 499 ms later is suppressed"
    );
    let Some(refresh) = policy.observe(high, 500) else {
        return;
    };
    assert_eq!(refresh.stream_id, ThrottleHint::ALL_STREAMS);
    assert_eq!(refresh.action, ThrottleAction::Pause);
    assert_eq!(refresh.ttl_ms, 1000, "the refresh renews the full TTL");
    assert!(policy.is_paused());

    // Bytes are below the low mark but entries are not: the pause is still refreshed.
    let Some(refresh) = policy.observe(occupancy(1024, flow::LOW_WATERMARK_ENTRIES + 1), 1000) else {
        return;
    };
    assert_eq!(
        refresh.action,
        ThrottleAction::Pause,
        "entries above the low mark keep the pause alive"
    );

    // Entries are low but bytes are not: the pause is still refreshed.
    let Some(refresh) = policy.observe(occupancy(3 * 1024, flow::LOW_WATERMARK_ENTRIES), 1500) else {
        return;
    };
    assert_eq!(
        refresh.action,
        ThrottleAction::Pause,
        "bytes above the low mark keep the pause alive"
    );

    // Only both low marks resume.
    let Some(resume) = policy.observe(occupancy(2 * 1024, flow::LOW_WATERMARK_ENTRIES), 2000) else {
        return;
    };
    assert_eq!(resume.action, ThrottleAction::Resume);
    assert!(!policy.is_paused());
}

/// A resume is not repeated while the ring stays below the low watermark.
#[test]
fn a_resume_is_emitted_once_per_drain() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(occupancy(6 * 1024, 0), 0).is_some());
    assert!(policy.observe(occupancy(0, 0), 1000).is_some());
    assert!(policy.observe(occupancy(0, 0), 10_000).is_none());
}

/// Between 2 KiB and 6 KiB the policy holds its state (hysteresis).
#[test]
fn hysteresis_holds_between_the_two_watermarks() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(occupancy(6 * 1024, 0), 0).is_some());
    assert!(
        policy.observe(occupancy(4 * 1024, 0), 100).is_none(),
        "4 KiB is neither high nor low, and the hint window still holds"
    );
    assert!(policy.is_paused());
    assert!(policy.observe(occupancy(2 * 1024, 0), 600).is_some(), "2 KiB resumes");
}

/// Below the high watermark nothing is emitted.
#[test]
fn no_hint_is_emitted_below_the_high_watermark() {
    let mut policy = WatermarkPolicy::new();
    assert!(policy.observe(occupancy(6 * 1024 - 1, 0), 0).is_none());
    assert!(policy.observe(occupancy(0, 0), 10_000).is_none());
    assert!(!policy.is_paused());
}

/// The 16-entry descriptor table fills long before 6 KiB under the worst-case robot mix (six
/// coalesced streams plus many small reliable events), so entry pressure must pause the robot.
#[test]
fn entry_pressure_pauses_below_the_byte_high_watermark() {
    let mut policy = WatermarkPolicy::new();
    let Some(hint) = policy.observe(occupancy(2 * 1024, flow::HIGH_WATERMARK_ENTRIES), 0) else {
        return;
    };
    assert_eq!(hint.action, ThrottleAction::Pause);
    assert!(policy.is_paused());

    assert!(
        policy
            .observe(occupancy(1024, flow::HIGH_WATERMARK_ENTRIES - 1), 100)
            .is_none(),
        "bytes are low but entries are above the low mark: hold"
    );
    assert!(
        policy
            .observe(occupancy(4 * 1024, flow::LOW_WATERMARK_ENTRIES), 200)
            .is_none(),
        "entries are low but bytes are above the low mark: hold"
    );

    let Some(hint) = policy.observe(occupancy(1024, flow::LOW_WATERMARK_ENTRIES), 600) else {
        return;
    };
    assert_eq!(hint.action, ThrottleAction::Resume);
    assert!(!policy.is_paused());
}

/// The constants are the ADRs' worked numbers plus the descriptor table's watermarks.
#[test]
fn the_watermark_constants_are_the_adr_numbers() {
    assert_eq!(flow::HIGH_WATERMARK_BYTES, 6 * 1024);
    assert_eq!(flow::LOW_WATERMARK_BYTES, 2 * 1024);
    assert_eq!(flow::HIGH_WATERMARK_ENTRIES, 12);
    assert_eq!(flow::LOW_WATERMARK_ENTRIES, 4);
    assert_eq!(flow::PAUSE_TTL_MS, 1000);
    assert_eq!(flow::MIN_HINT_INTERVAL_MS, 500);
}
