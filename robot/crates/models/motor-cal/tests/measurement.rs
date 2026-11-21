//! Host-side integration tests for the `motor-cal` measurement engine.
//!
//! Every assertion is made through the crate's public API. The expected values
//! come from the measurement contract in `.scratch/motor-calibration/spec.md` and
//! from worked encoder timelines, never from re-deriving the result the way the
//! engine does.

use motor_cal::{
    EncoderSample, InvalidReason, MEASUREMENT_WINDOW_MS, Measurement, MeasurementEngine, Phase, PointResult,
    QUIESCENCE_HOLD_MS, QUIESCENCE_TIMEOUT_MS, SETTLE_MS,
};

/// Spacing of the worked sample timelines, in milliseconds.
const INTERVAL_MS: u64 = 20;

/// A sample at `timestamp_ms` reporting `count`.
const fn sample(timestamp_ms: u64, count: u16) -> EncoderSample {
    EncoderSample { timestamp_ms, count }
}

/// Drive a fresh engine through Quiescence, the reset and the command, and
/// return it left in [`Phase::Settling`] with the command instant.
fn armed() -> (MeasurementEngine, u64) {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);

    let mut quiescence_ms = 0;
    loop {
        engine.observe(sample(quiescence_ms, 0));
        if engine.phase() == Phase::Quiescent {
            break;
        }
        quiescence_ms += INTERVAL_MS;
    }

    engine.reset_now(quiescence_ms);
    let command_ms = quiescence_ms + INTERVAL_MS;
    engine.observe(sample(command_ms, 0));
    assert_eq!(engine.phase(), Phase::ReadyToCommand);
    engine.command_applied(command_ms);
    assert_eq!(engine.phase(), Phase::Settling);

    (engine, command_ms)
}

/// The valid measurement `engine` finished with, or `None`.
fn measurement(engine: &MeasurementEngine) -> Option<Measurement> {
    engine.result().and_then(PointResult::measurement)
}

#[test]
fn quiescence_requires_a_full_hold_window() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);

    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS - 1, 0));
    assert_eq!(engine.phase(), Phase::SeekingQuiescence);

    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    assert_eq!(engine.phase(), Phase::Quiescent);
}

#[test]
fn a_pulse_restarts_the_hold_window() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);

    engine.observe(sample(0, 0));
    engine.observe(sample(200, 1));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 1));
    assert_eq!(engine.phase(), Phase::SeekingQuiescence);

    engine.observe(sample(450, 1));
    assert_eq!(engine.phase(), Phase::Quiescent);
}

#[test]
fn reset_is_confirmed_by_a_fresh_zero_sample() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    assert_eq!(engine.phase(), Phase::Quiescent);

    engine.reset_now(300);
    assert_eq!(engine.phase(), Phase::ConfirmingReset);

    engine.observe(sample(320, 0));
    assert_eq!(engine.phase(), Phase::ReadyToCommand);
    assert_eq!(engine.result(), None);
}

#[test]
fn a_sample_from_before_the_reset_is_stale() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    engine.reset_now(300);

    engine.observe(sample(260, 0));

    assert_eq!(engine.result(), Some(PointResult::Invalid(InvalidReason::StaleSample)));
}

#[test]
fn a_sample_at_the_reset_instant_confirms_it() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    engine.reset_now(300);

    engine.observe(sample(300, 0));

    assert_eq!(engine.phase(), Phase::ReadyToCommand);
    assert_eq!(engine.result(), None);
}

#[test]
fn a_nonzero_fresh_sample_leaves_the_reset_unconfirmed() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    engine.reset_now(300);

    engine.observe(sample(320, 5));

    assert_eq!(
        engine.result(),
        Some(PointResult::Invalid(InvalidReason::ResetUnconfirmed))
    );
}

#[test]
fn quiescence_times_out_when_pulses_keep_arriving() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);

    let mut timestamp_ms = 0;
    let mut count = 0;
    while engine.phase() != Phase::Finished {
        assert!(
            timestamp_ms <= QUIESCENCE_TIMEOUT_MS + 20,
            "the point did not time out by {QUIESCENCE_TIMEOUT_MS} ms"
        );
        engine.observe(sample(timestamp_ms, count));
        timestamp_ms += 20;
        count += 1;
    }

    assert_eq!(
        engine.result(),
        Some(PointResult::Invalid(InvalidReason::QuiescenceTimeout))
    );
}

#[test]
fn a_sustained_step_reports_the_window_delta_and_its_speed() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    // Settle: zero counts, discarded.
    let mut timestamp_ms = command_ms + INTERVAL_MS;
    while timestamp_ms < window_start_ms {
        engine.observe(sample(timestamp_ms, 0));
        timestamp_ms += INTERVAL_MS;
    }

    // Window: 4 counts every 20 ms for 2 000 ms, i.e. 400 counts.
    let mut count = 0;
    while timestamp_ms <= window_start_ms + MEASUREMENT_WINDOW_MS {
        engine.observe(sample(timestamp_ms, count));
        count += 4;
        timestamp_ms += INTERVAL_MS;
    }

    assert_eq!(engine.phase(), Phase::Finished);
    let measurement = measurement(&engine);
    assert!(measurement.is_some());
    if let Some(measurement) = measurement {
        assert_eq!(measurement.count(), 400);
        assert_eq!(measurement.window_ms(), MEASUREMENT_WINDOW_MS);
        assert_eq!(measurement.speed_counts_per_s(), 200);
        assert!(measurement.sustained_motion());
    }
}

#[test]
fn the_window_length_is_the_timestamped_span() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    // The end sample arrives 40 ms after the nominal 2 000 ms boundary. The
    // measurement reports the real 2 040 ms span and the speed over it, so a
    // stored measurement never claims a window it did not take: 408 counts over
    // 2 040 ms is 200 counts per second.
    engine.observe(sample(window_start_ms, 100));
    engine.observe(sample(window_start_ms + MEASUREMENT_WINDOW_MS + 40, 508));

    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(408));
    assert_eq!(measurement.map(|m| m.window_ms()), Some(MEASUREMENT_WINDOW_MS + 40));
    assert_eq!(measurement.map(|m| m.speed_counts_per_s()), Some(200));
}

#[test]
fn the_settle_interval_is_discarded() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    // A sample inside the settle interval must not become the window start.
    engine.observe(sample(window_start_ms - 1, 5));
    assert_eq!(engine.phase(), Phase::Settling);

    // The first sample at the settle boundary starts the window at 7 counts.
    engine.observe(sample(window_start_ms, 7));
    assert_eq!(engine.phase(), Phase::Measuring);

    let mut count = 11;
    let mut timestamp_ms = window_start_ms + INTERVAL_MS;
    while timestamp_ms <= window_start_ms + MEASUREMENT_WINDOW_MS {
        engine.observe(sample(timestamp_ms, count));
        count += 4;
        timestamp_ms += INTERVAL_MS;
    }

    // 407 - 7 = 400. Had the settle sample's 5 been the start it would be 402.
    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(400));
    assert_eq!(measurement.map(|m| m.speed_counts_per_s()), Some(200));
}

#[test]
fn the_window_completes_only_at_its_end_sample() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    engine.observe(sample(window_start_ms, 0));
    assert_eq!(engine.phase(), Phase::Measuring);

    engine.observe(sample(window_start_ms + MEASUREMENT_WINDOW_MS - INTERVAL_MS, 100));
    assert_eq!(engine.phase(), Phase::Measuring);
    assert_eq!(engine.result(), None);

    engine.observe(sample(window_start_ms + MEASUREMENT_WINDOW_MS, 200));
    assert_eq!(engine.phase(), Phase::Finished);
    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(200));
    assert_eq!(measurement.map(|m| m.speed_counts_per_s()), Some(100));
}

#[test]
fn the_window_delta_wraps_with_the_16_bit_counter() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    // Start 36 counts below the u16 wrap and add 400 counts across the window.
    engine.observe(sample(window_start_ms, 65_500));
    let mut count = 65_504;
    let mut timestamp_ms = window_start_ms + INTERVAL_MS;
    while timestamp_ms <= window_start_ms + MEASUREMENT_WINDOW_MS {
        engine.observe(sample(timestamp_ms, count));
        count = count.wrapping_add(4);
        timestamp_ms += INTERVAL_MS;
    }

    // The counter ended at 364 after wrapping; the delta is still 400.
    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(400));
    assert_eq!(measurement.map(|m| m.speed_counts_per_s()), Some(200));
    assert_eq!(measurement.map(|m| m.sustained_motion()), Some(true));
}

#[test]
fn a_single_stray_pulse_is_not_sustained_motion() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    // The counter ticks once, half-way through the window, and never again.
    let mut timestamp_ms = window_start_ms;
    while timestamp_ms <= window_start_ms + MEASUREMENT_WINDOW_MS {
        let offset = timestamp_ms - window_start_ms;
        let count = u16::from(offset >= 1_000);
        engine.observe(sample(timestamp_ms, count));
        timestamp_ms += INTERVAL_MS;
    }

    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(1));
    assert_eq!(measurement.map(|m| m.sustained_motion()), Some(false));
}

#[test]
fn motion_in_only_one_sub_window_is_not_sustained() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;

    // 200 counts inside the first 500 ms sub-window, then nothing at all.
    let mut count = 0;
    let mut timestamp_ms = window_start_ms;
    while timestamp_ms <= window_start_ms + MEASUREMENT_WINDOW_MS {
        engine.observe(sample(timestamp_ms, count));
        if timestamp_ms - window_start_ms < 400 {
            count += 10;
        }
        timestamp_ms += INTERVAL_MS;
    }

    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(200));
    assert_eq!(measurement.map(|m| m.sustained_motion()), Some(false));
}

#[test]
fn finish_before_quiescence_reports_a_quiescence_timeout() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));

    engine.finish();

    assert_eq!(engine.phase(), Phase::Finished);
    assert_eq!(
        engine.result(),
        Some(PointResult::Invalid(InvalidReason::QuiescenceTimeout))
    );
}

#[test]
fn finish_after_quiescence_without_a_reset_reports_reset_unconfirmed() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    assert_eq!(engine.phase(), Phase::Quiescent);

    engine.finish();

    assert_eq!(
        engine.result(),
        Some(PointResult::Invalid(InvalidReason::ResetUnconfirmed))
    );
}

#[test]
fn finish_after_the_reset_without_confirmation_reports_reset_unconfirmed() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    engine.reset_now(300);
    assert_eq!(engine.phase(), Phase::ConfirmingReset);

    engine.finish();

    assert_eq!(
        engine.result(),
        Some(PointResult::Invalid(InvalidReason::ResetUnconfirmed))
    );
}

#[test]
fn finish_before_the_window_ends_reports_no_window() {
    let (mut engine, command_ms) = armed();
    engine.observe(sample(command_ms + SETTLE_MS, 0));
    assert_eq!(engine.phase(), Phase::Measuring);

    engine.finish();

    assert_eq!(engine.phase(), Phase::Finished);
    assert_eq!(engine.result(), Some(PointResult::Invalid(InvalidReason::NoWindow)));
}

#[test]
fn finish_after_a_completed_window_keeps_the_measurement() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;
    engine.observe(sample(window_start_ms, 0));
    engine.observe(sample(window_start_ms + MEASUREMENT_WINDOW_MS, 400));

    engine.finish();

    let measurement = measurement(&engine);
    assert_eq!(measurement.map(|m| m.count()), Some(400));
}

#[test]
fn abort_finishes_the_point_with_the_given_reason() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.observe(sample(0, 0));
    engine.observe(sample(QUIESCENCE_HOLD_MS, 0));
    assert_eq!(engine.phase(), Phase::Quiescent);

    // The runner aborts when a condition the engine cannot see — the untested
    // track still moving — outlasts the quiescence timeout.
    engine.abort(InvalidReason::QuiescenceTimeout);

    assert_eq!(engine.phase(), Phase::Finished);
    assert_eq!(
        engine.result(),
        Some(PointResult::Invalid(InvalidReason::QuiescenceTimeout))
    );
}

#[test]
fn abort_leaves_a_finished_point_unchanged() {
    let (mut engine, command_ms) = armed();
    let window_start_ms = command_ms + SETTLE_MS;
    engine.observe(sample(window_start_ms, 0));
    engine.observe(sample(window_start_ms + MEASUREMENT_WINDOW_MS, 400));
    assert_eq!(engine.phase(), Phase::Finished);

    engine.abort(InvalidReason::StaleSample);

    assert_eq!(measurement(&engine).map(|m| m.count()), Some(400));
}

#[test]
fn abort_before_a_step_begins_does_nothing() {
    let mut engine = MeasurementEngine::new();

    engine.abort(InvalidReason::NoWindow);

    assert_eq!(engine.phase(), Phase::Idle);
    assert_eq!(engine.result(), None);
}

#[test]
fn invalid_reasons_name_themselves_for_the_log() {
    assert_eq!(InvalidReason::QuiescenceTimeout.label(), "quiescence timeout");
    assert_eq!(InvalidReason::ResetUnconfirmed.label(), "reset unconfirmed");
    assert_eq!(InvalidReason::StaleSample.label(), "stale sample");
    assert_eq!(InvalidReason::NoWindow.label(), "no window");
}
