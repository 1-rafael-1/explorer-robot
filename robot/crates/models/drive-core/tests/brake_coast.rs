//! Host-side tests for brake, coast and idle behind `Intent::step`.
//!
//! Brake and coast emit their initial motor action and then watch encoder deltas
//! at the firmware's 100 ms settle cadence: three consecutive zero deltas on both
//! tracks settle, any motion resets the count, and two seconds without settling
//! fails with the exact lowercase `encoder settle timeout` detail. Idle coasts
//! and completes on its duration, or immediately at zero.
//!
//! Every assertion goes through the public `Intent` surface and uses the
//! firmware's adapter contract: execute a non-blocking action and step again
//! immediately, and sleep to the absolute deadline of a `WaitUntil`.

use drive_core::{
    CompletionTelemetry, DriveFault, DriveOutcome, EncoderSample, Intent, IntentAction, IntentParams, Observation,
};

/// Every scenario starts at this instant.
const START_MS: u64 = 1_000;
/// The settle tick the model emits first.
const FIRST_TICK_MS: u64 = START_MS + 100;

/// A brake parameter set.
const fn brake() -> IntentParams {
    IntentParams::Brake
}

/// A coast parameter set.
const fn coast() -> IntentParams {
    IntentParams::Coast
}

/// An idle parameter set.
const fn idle(duration_ms: u64) -> IntentParams {
    IntentParams::Idle { duration_ms }
}

/// An absolute encoder sample.
const fn enc(left: u16, right: u16, timestamp_ms: u64) -> Option<EncoderSample> {
    Some(EncoderSample {
        left,
        right,
        timestamp_ms,
    })
}

/// An observation carrying only the clock and an optional encoder sample.
const fn observation(now_ms: u64, encoder: Option<EncoderSample>) -> Observation {
    Observation {
        now_ms,
        encoder,
        yaw: None,
    }
}

/// One step of the adapter contract.
fn step(intent: &mut Intent, now_ms: u64, encoder: Option<EncoderSample>) -> IntentAction {
    intent.step(&observation(now_ms, encoder)).action
}

/// The telemetry of a completed intent, or a failed assertion.
fn completed(outcome: Option<DriveOutcome>) -> CompletionTelemetry {
    match outcome {
        Some(DriveOutcome::Completed { telemetry }) => telemetry,
        other => panic!("expected a completed intent, got {other:?}"),
    }
}

/// The fault and telemetry of a failed intent, or a failed assertion.
fn failed(outcome: Option<DriveOutcome>) -> (DriveFault, CompletionTelemetry) {
    match outcome {
        Some(DriveOutcome::Failed { fault, telemetry }) => (fault, telemetry),
        other => panic!("expected a failed intent, got {other:?}"),
    }
}

// ── Initial actions ──────────────────────────────────────────────────────────

#[test]
fn brake_emits_the_initial_brake_action_then_waits_one_tick() {
    let mut intent = Intent::new(brake(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::BrakeAll);
    assert_eq!(
        step(&mut intent, START_MS, None),
        IntentAction::WaitUntil {
            deadline_ms: FIRST_TICK_MS
        }
    );
}

#[test]
fn coast_emits_the_initial_coast_action_then_waits_one_tick() {
    let mut intent = Intent::new(coast(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::CoastAll);
    assert_eq!(
        step(&mut intent, START_MS, None),
        IntentAction::WaitUntil {
            deadline_ms: FIRST_TICK_MS
        }
    );
}

// ── Settle detection ─────────────────────────────────────────────────────────

#[test]
fn brake_settles_after_three_consecutive_zero_deltas() {
    let mut intent = Intent::new(brake(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::BrakeAll);
    assert_eq!(
        step(&mut intent, START_MS, None),
        IntentAction::WaitUntil {
            deadline_ms: FIRST_TICK_MS
        }
    );

    // The first fresh sample only seeds the delta baseline.
    assert_eq!(
        step(&mut intent, 1_100, enc(100, 100, 1_100)),
        IntentAction::WaitUntil { deadline_ms: 1_200 }
    );
    // Three consecutive zero deltas on both tracks.
    assert_eq!(
        step(&mut intent, 1_200, enc(100, 100, 1_200)),
        IntentAction::WaitUntil { deadline_ms: 1_300 }
    );
    assert_eq!(
        step(&mut intent, 1_300, enc(100, 100, 1_300)),
        IntentAction::WaitUntil { deadline_ms: 1_400 }
    );
    assert_eq!(step(&mut intent, 1_400, enc(100, 100, 1_400)), IntentAction::Finished);

    assert_eq!(completed(intent.outcome()), CompletionTelemetry::None);
}

#[test]
fn a_non_zero_delta_resets_the_settle_count() {
    let mut intent = Intent::new(coast(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::CoastAll);

    assert_eq!(
        step(&mut intent, 1_100, enc(100, 100, 1_100)),
        IntentAction::WaitUntil { deadline_ms: 1_200 }
    );
    // Motion on both tracks resets the count.
    assert_eq!(
        step(&mut intent, 1_200, enc(110, 90, 1_200)),
        IntentAction::WaitUntil { deadline_ms: 1_300 }
    );
    // The count restarts from one.
    assert_eq!(
        step(&mut intent, 1_300, enc(110, 90, 1_300)),
        IntentAction::WaitUntil { deadline_ms: 1_400 }
    );
    assert_eq!(
        step(&mut intent, 1_400, enc(110, 90, 1_400)),
        IntentAction::WaitUntil { deadline_ms: 1_500 }
    );
    assert_eq!(step(&mut intent, 1_500, enc(110, 90, 1_500)), IntentAction::Finished);

    assert_eq!(completed(intent.outcome()), CompletionTelemetry::None);
}

#[test]
fn a_wrapped_counter_delta_is_motion_not_a_huge_delta() {
    let mut intent = Intent::new(brake(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::BrakeAll);

    assert_eq!(
        step(&mut intent, 1_100, enc(65_500, 65_500, 1_100)),
        IntentAction::WaitUntil { deadline_ms: 1_200 }
    );
    // 65_500 -> 100 wraps; the wrap-safe delta is 136 counts of motion, so the
    // settle count resets instead of completing on a bogus zero.
    assert_eq!(
        step(&mut intent, 1_200, enc(100, 100, 1_200)),
        IntentAction::WaitUntil { deadline_ms: 1_300 }
    );
    // Equal counts after the wrap produce a true zero delta.
    assert_eq!(
        step(&mut intent, 1_300, enc(100, 100, 1_300)),
        IntentAction::WaitUntil { deadline_ms: 1_400 }
    );
    assert_eq!(
        step(&mut intent, 1_400, enc(100, 100, 1_400)),
        IntentAction::WaitUntil { deadline_ms: 1_500 }
    );
    assert_eq!(step(&mut intent, 1_500, enc(100, 100, 1_500)), IntentAction::Finished);

    assert_eq!(completed(intent.outcome()), CompletionTelemetry::None);
}

// ── Timeout ──────────────────────────────────────────────────────────────────

#[test]
fn brake_without_a_fresh_sample_times_out_with_the_exact_label() {
    let mut intent = Intent::new(brake(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::BrakeAll);

    // No encoder sample ever arrives; every tick re-arms the settle wait.
    let mut now_ms = 1_100;
    while now_ms <= 2_900 {
        assert_eq!(
            step(&mut intent, now_ms, None),
            IntentAction::WaitUntil {
                deadline_ms: now_ms + 100
            }
        );
        now_ms += 100;
    }

    // The two-second deadline is 3 000 ms; the tick at the deadline fails before
    // reading a sample.
    assert_eq!(step(&mut intent, 3_000, None), IntentAction::Finished);

    let (fault, telemetry) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::SettleTimeout);
    assert_eq!(fault.label(), "encoder settle timeout");
    assert_eq!(telemetry, CompletionTelemetry::None);
}

#[test]
fn the_timeout_is_checked_before_a_settling_sample() {
    let mut intent = Intent::new(coast(), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::CoastAll);

    // No samples until late in the window, so the settle count stays at zero.
    let mut now_ms = 1_100;
    while now_ms <= 2_600 {
        assert_eq!(
            step(&mut intent, now_ms, None),
            IntentAction::WaitUntil {
                deadline_ms: now_ms + 100
            }
        );
        now_ms += 100;
    }

    // Seed the baseline at 2 700, then two zero deltas.
    assert_eq!(
        step(&mut intent, 2_700, enc(100, 100, 2_700)),
        IntentAction::WaitUntil { deadline_ms: 2_800 }
    );
    assert_eq!(
        step(&mut intent, 2_800, enc(100, 100, 2_800)),
        IntentAction::WaitUntil { deadline_ms: 2_900 }
    );
    assert_eq!(
        step(&mut intent, 2_900, enc(100, 100, 2_900)),
        IntentAction::WaitUntil { deadline_ms: 3_000 }
    );

    // The sample at 3 000 would be the third zero delta, but the two-second
    // deadline is checked first and wins.
    assert_eq!(step(&mut intent, 3_000, enc(100, 100, 3_000)), IntentAction::Finished);

    let (fault, telemetry) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::SettleTimeout);
    assert_eq!(telemetry, CompletionTelemetry::None);
}

// ── Idle ─────────────────────────────────────────────────────────────────────

#[test]
fn idle_at_zero_duration_coasts_then_finishes_immediately() {
    let mut intent = Intent::new(idle(0), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::CoastAll);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::Finished);

    assert_eq!(completed(intent.outcome()), CompletionTelemetry::None);
}

#[test]
fn idle_waits_for_its_duration_then_finishes() {
    let mut intent = Intent::new(idle(500), START_MS);
    assert_eq!(step(&mut intent, START_MS, None), IntentAction::CoastAll);
    assert_eq!(
        step(&mut intent, START_MS, None),
        IntentAction::WaitUntil { deadline_ms: 1_500 }
    );

    // A wait re-arms the same absolute deadline until it passes.
    assert_eq!(
        step(&mut intent, 1_400, None),
        IntentAction::WaitUntil { deadline_ms: 1_500 }
    );
    assert_eq!(step(&mut intent, 1_500, None), IntentAction::Finished);

    assert_eq!(completed(intent.outcome()), CompletionTelemetry::None);
}

// ── Cancellation telemetry ───────────────────────────────────────────────────

#[test]
fn cancelling_brake_coast_or_idle_reports_no_telemetry() {
    let mut brake_intent = Intent::new(brake(), START_MS);
    let _ = step(&mut brake_intent, START_MS, None);
    let _ = step(&mut brake_intent, 1_100, enc(100, 100, 1_100));
    assert_eq!(brake_intent.cancellation_telemetry(1_250), CompletionTelemetry::None);

    let mut coast_intent = Intent::new(coast(), START_MS);
    let _ = step(&mut coast_intent, START_MS, None);
    assert_eq!(coast_intent.cancellation_telemetry(1_250), CompletionTelemetry::None);

    let mut idle_intent = Intent::new(idle(500), START_MS);
    let _ = step(&mut idle_intent, START_MS, None);
    assert_eq!(idle_intent.cancellation_telemetry(1_250), CompletionTelemetry::None);
}
