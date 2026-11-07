//! The link's pure policy: the DENY mapping, the backoff schedule and the jitter (ADR-0039).

use std::time::Duration;

use bridge_net::preamble::DenyReason;
use remote_daemon::link::{
    BACKOFF_BASE_MS, BACKOFF_CAP_MS, BUSY_RETRY_DEFAULT_MS, DenyDisposition, LinkState, backoff_delay,
    deny_disposition, jittered,
};

/// The reconnect delays are ADR-0039's 500 ms → 1, 2, 4, 8 s cap.
#[test]
fn the_backoff_schedule_is_the_adr_sequence() {
    let expected = [
        Duration::from_millis(500),
        Duration::from_secs(1),
        Duration::from_secs(2),
        Duration::from_secs(4),
        Duration::from_secs(8),
        Duration::from_secs(8),
    ];
    for (attempt, expected) in expected.into_iter().enumerate() {
        assert_eq!(
            backoff_delay(u8::try_from(attempt).unwrap_or(u8::MAX)),
            expected,
            "attempt {attempt}"
        );
    }
    assert_eq!(BACKOFF_BASE_MS, 500);
    assert_eq!(BACKOFF_CAP_MS, 8_000);
}

/// The jitter spans ±20 % of the base delay.
#[test]
fn the_jitter_spans_plus_or_minus_twenty_percent() {
    let delay = Duration::from_secs(1);
    assert_eq!(jittered(delay, 0.20, 0), Duration::from_millis(800));
    assert_eq!(jittered(delay, 0.20, u16::MAX), Duration::from_millis(1_200));
    let middle = jittered(delay, 0.20, u16::MAX / 2).as_millis();
    assert!(middle.abs_diff(1_000) <= 1, "the midpoint is the base delay");
    assert_eq!(jittered(delay, 0.0, 0), delay, "zero jitter leaves the delay exact");
}

/// `unauthorized` and `setup_mode` are fatal and never auto-retried; `busy` retries.
#[test]
fn the_deny_reasons_map_to_the_documented_actions() {
    assert!(matches!(
        deny_disposition(DenyReason::Unauthorized, None),
        DenyDisposition::Fatal(LinkState::Unauthorized { .. })
    ));
    assert!(matches!(
        deny_disposition(DenyReason::SetupMode, None),
        DenyDisposition::Fatal(LinkState::SetupMode { .. })
    ));
    assert!(matches!(
        deny_disposition(DenyReason::Version, None),
        DenyDisposition::Fatal(LinkState::VersionMismatch { .. })
    ));
    assert_eq!(
        deny_disposition(DenyReason::Busy, None),
        DenyDisposition::RetryAfter(Duration::from_millis(u64::from(BUSY_RETRY_DEFAULT_MS)))
    );
    assert_eq!(
        deny_disposition(DenyReason::Busy, Some(1_500)),
        DenyDisposition::RetryAfter(Duration::from_millis(1_500)),
        "a busy hint wins over the 4 s default"
    );
}
