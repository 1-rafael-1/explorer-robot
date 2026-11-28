//! Host-side tests for the distance control law behind `Intent::step`.
//!
//! Every assertion goes through the public `Intent` surface: the action
//! sequence, the terminal outcome and the telemetry. The expected speeds,
//! revolutions and counts are literal worked examples, never a re-run of the
//! model's arithmetic.
//!
//! Every scenario uses a ten-counts-per-centimetre scale, so one centimetre is
//! ten counts and one sprocket revolution (19.01 cm) is 190.1 counts.
//!
//! The adapter contract these tests follow is the same one the firmware uses:
//! execute a non-blocking action and step again immediately, and sleep to the
//! absolute deadline of a `WaitUntil` before the next observation.

use drive_core::{
    CompletionTelemetry, DistanceScale, DriveDirection, DriveDistanceKind, DriveFault, DriveOutcome, EncoderSample,
    Intent, IntentAction, IntentParams, Observation, TurnDirection, YawSample,
};

/// Every scenario starts at this instant.
const START_MS: u64 = 1_000;
/// Forward motion.
const FORWARD: DriveDirection = DriveDirection::Forward;
/// Reverse motion.
const REVERSE: DriveDirection = DriveDirection::Reverse;

/// Ten counts per centimetre: one centimetre is ten counts, one revolution is
/// 190.1 counts, so 100 cm is 5.2604 revolutions and 1000 counts.
fn scale() -> DistanceScale {
    match DistanceScale::from_counts_per_cm(10.0) {
        Ok(scale) => scale,
        Err(error) => panic!("10 counts/cm must be inside the plausibility band: {error:?}"),
    }
}

/// An absolute encoder sample.
const fn enc(left: u16, right: u16, timestamp_ms: u64) -> Option<EncoderSample> {
    Some(EncoderSample {
        left,
        right,
        timestamp_ms,
    })
}

/// A yaw sample.
const fn yaw(yaw_deg: f32, timestamp_ms: u64) -> Option<YawSample> {
    Some(YawSample { yaw_deg, timestamp_ms })
}

/// A straight distance parameter set.
fn straight(distance_cm: f32, direction: DriveDirection, speed: u8) -> IntentParams {
    IntentParams::DriveDistance {
        kind: DriveDistanceKind::Straight { distance_cm },
        direction,
        speed,
        scale: scale(),
    }
}

/// A curve-arc parameter set.
fn curve(
    radius_cm: f32,
    arc_length_cm: f32,
    turn: TurnDirection,
    direction: DriveDirection,
    speed: u8,
) -> IntentParams {
    IntentParams::DriveDistance {
        kind: DriveDistanceKind::CurveArc {
            radius_cm,
            arc_length_cm,
            direction: turn,
        },
        direction,
        speed,
        scale: scale(),
    }
}

/// One step of the adapter contract.
fn step(intent: &mut Intent, now_ms: u64, encoder: Option<EncoderSample>, yaw: Option<YawSample>) -> IntentAction {
    intent.step(&Observation { now_ms, encoder, yaw }).action
}

/// The deadline of a `WaitUntil`, or a failed assertion.
fn wait_deadline(action: IntentAction) -> u64 {
    match action {
        IntentAction::WaitUntil { deadline_ms } => deadline_ms,
        other => {
            assert!(
                matches!(other, IntentAction::WaitUntil { .. }),
                "expected WaitUntil, got {other:?}"
            );
            0
        }
    }
}

/// Start a drive whose first observation already carries a yaw sample.
///
/// Returns the initial track command and checks that the next step waits one
/// control interval (`START_MS + 20`).
fn start(intent: &mut Intent, yaw_deg: f32) -> IntentAction {
    let first = step(intent, START_MS, None, yaw(yaw_deg, START_MS));
    assert_eq!(wait_deadline(step(intent, START_MS, None, None)), START_MS + 20);
    first
}

/// Assert two floats are the same worked-example value.
fn assert_close(actual: f32, expected: f32) {
    assert!((actual - expected).abs() < 1e-3, "expected {expected}, got {actual}");
}

/// The telemetry of a completed drive, or a failed assertion.
fn completed(outcome: Option<DriveOutcome>) -> CompletionTelemetry {
    match outcome {
        Some(DriveOutcome::Completed { telemetry }) => telemetry,
        other => panic!("expected a completed drive, got {other:?}"),
    }
}

/// The fault and telemetry of a failed drive, or a failed assertion.
fn failed(outcome: Option<DriveOutcome>) -> (DriveFault, CompletionTelemetry) {
    match outcome {
        Some(DriveOutcome::Failed { fault, telemetry }) => (fault, telemetry),
        other => panic!("expected a failed drive, got {other:?}"),
    }
}

/// Assert the distance telemetry is the worked example.
fn assert_distance_telemetry(
    telemetry: CompletionTelemetry,
    achieved: (f32, f32),
    counts: (u32, u32),
    targets: (f32, f32),
    duration_ms: u64,
) {
    match telemetry {
        CompletionTelemetry::DriveDistance {
            achieved_left_revs,
            achieved_right_revs,
            achieved_left_counts,
            achieved_right_counts,
            target_left_revs,
            target_right_revs,
            duration_ms: actual_duration,
        } => {
            assert_close(achieved_left_revs, achieved.0);
            assert_close(achieved_right_revs, achieved.1);
            assert_eq!(achieved_left_counts, counts.0);
            assert_eq!(achieved_right_counts, counts.1);
            assert_close(target_left_revs, targets.0);
            assert_close(target_right_revs, targets.1);
            assert_eq!(actual_duration, duration_ms);
        }
        other => panic!("expected distance telemetry, got {other:?}"),
    }
}

// ── Target drives: progress, ramp, tolerance and telemetry ────────────────────

#[test]
fn a_straight_forward_drive_completes_on_its_target() {
    // 100 cm at 60 %: 5.2604 revolutions and 1000 counts on both tracks.
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });

    // The first fresh sample only sets the delta baseline.
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 500 counts is 2.6302 revolutions: half way, so the speed is unchanged.
    assert_eq!(
        step(&mut intent, 1_040, enc(1_000, 1_000, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    // The next 500 counts reach 5.2604 revolutions: stop, then finish.
    assert_eq!(
        step(&mut intent, 1_060, enc(1_500, 1_500, 1_060), yaw(0.0, 1_060)),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(
        step(&mut intent, 1_060, enc(1_500, 1_500, 1_060), yaw(0.0, 1_060)),
        IntentAction::Finished
    );

    let telemetry = completed(intent.outcome());
    assert_distance_telemetry(telemetry, (5.2604, 5.2604), (1_000, 1_000), (5.2604, 5.2604), 60);
}

#[test]
fn a_straight_reverse_drive_commands_negative_tracks() {
    let mut intent = Intent::new(straight(100.0, REVERSE, 60), START_MS);
    assert_eq!(
        start(&mut intent, 0.0),
        IntentAction::SetTracks { left: -60, right: -60 }
    );

    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut intent, 1_040, enc(1_000, 1_000, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: -60, right: -60 }
    );
    assert_eq!(
        step(&mut intent, 1_060, enc(1_500, 1_500, 1_060), yaw(0.0, 1_060)),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(
        step(&mut intent, 1_060, enc(1_500, 1_500, 1_060), yaw(0.0, 1_060)),
        IntentAction::Finished
    );

    // Progress is always positive; only the command direction carries the sign.
    let telemetry = completed(intent.outcome());
    assert_distance_telemetry(telemetry, (5.2604, 5.2604), (1_000, 1_000), (5.2604, 5.2604), 60);
}

#[test]
fn the_speed_ramps_down_inside_a_quarter_revolution() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 960 counts leave 0.2104 revolutions: round(60 × 0.2104 / 0.25) = 50.
    assert_eq!(
        step(&mut intent, 1_040, enc(1_460, 1_460, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 50, right: 50 }
    );
}

#[test]
fn the_ramp_holds_the_minimum_speed_floor() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 100), START_MS);
    assert_eq!(
        start(&mut intent, 0.0),
        IntentAction::SetTracks { left: 100, right: 100 }
    );
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 995 counts leave 0.0263 revolutions: round(100 × 0.1052) = 11, floored
    // to the minimum ramp speed of 40.
    assert_eq!(
        step(&mut intent, 1_040, enc(1_495, 1_495, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 40, right: 40 }
    );
}

#[test]
fn completion_fires_inside_the_tolerance_band() {
    // 19.01 cm is one revolution.
    let mut intent = Intent::new(straight(19.01, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 188 counts is 0.9890 revolutions: 0.0110 remaining, outside the 0.01 band.
    assert_eq!(
        step(&mut intent, 1_040, enc(288, 288, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 40, right: 40 }
    );

    // One more count is 0.9942 revolutions: 0.0058 remaining, inside the band.
    assert_eq!(
        step(&mut intent, 1_060, enc(289, 289, 1_060), yaw(0.0, 1_060)),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(
        step(&mut intent, 1_060, enc(289, 289, 1_060), yaw(0.0, 1_060)),
        IntentAction::Finished
    );

    let telemetry = completed(intent.outcome());
    assert_distance_telemetry(telemetry, (0.9942, 0.9942), (189, 189), (1.0, 1.0), 60);
}

#[test]
fn a_distance_inside_the_tolerance_finishes_on_the_first_step() {
    // 0.1 cm is 0.0053 revolutions, inside the 0.01 tolerance: no motor command.
    let mut intent = Intent::new(straight(0.1, FORWARD, 60), START_MS);
    assert_eq!(step(&mut intent, START_MS, None, None), IntentAction::Finished);

    let telemetry = completed(intent.outcome());
    assert_distance_telemetry(telemetry, (0.0, 0.0), (0, 0), (0.0053, 0.0053), 0);
}

#[test]
fn cancelling_a_distance_drive_reports_its_progress() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut intent, 1_040, enc(600, 600, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    assert_distance_telemetry(
        intent.cancellation_telemetry(2_500),
        (2.6302, 2.6302),
        (500, 500),
        (5.2604, 5.2604),
        1_500,
    );
}

// ── Encoder deltas, freshness, timeouts and anomalies ────────────────────────

#[test]
fn a_wrapping_counter_delta_is_counted() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(65_500, 65_500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 65 500 → 100 wraps the 16-bit counter: the delta is 136 counts (0.7154 rev).
    assert_eq!(
        step(&mut intent, 1_040, enc(100, 100, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    assert_distance_telemetry(
        intent.cancellation_telemetry(1_040),
        (0.7154, 0.7154),
        (136, 136),
        (5.2604, 5.2604),
        40,
    );
}

#[test]
fn a_stale_sample_does_not_command_the_tracks() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut intent, 1_040, enc(200, 200, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    // The same timestamp as the last processed sample is ignored: no motor
    // command this tick, and no additional progress.
    assert_eq!(
        step(&mut intent, 1_060, enc(200, 200, 1_040), yaw(0.0, 1_060)),
        IntentAction::WaitUntil { deadline_ms: 1_080 }
    );

    assert_distance_telemetry(
        intent.cancellation_telemetry(1_060),
        (0.5260, 0.5260),
        (100, 100),
        (5.2604, 5.2604),
        60,
    );
}

#[test]
fn encoder_timeout_fails_after_three_hundred_milliseconds() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut intent, 1_040, enc(200, 200, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    // The last fresh sample was seen at 1 040; 300 ms later is 1 340.
    let mut now_ms = 1_060;
    while now_ms <= 1_320 {
        assert_eq!(
            step(&mut intent, now_ms, enc(200, 200, 1_040), None),
            IntentAction::WaitUntil {
                deadline_ms: now_ms + 20
            }
        );
        now_ms += 20;
    }

    assert_eq!(
        step(&mut intent, 1_340, enc(200, 200, 1_040), None),
        IntentAction::Finished
    );

    let (fault, telemetry) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::EncoderTimeout);
    assert_eq!(fault.label(), "EncoderTimeout");
    assert_distance_telemetry(telemetry, (0.5260, 0.5260), (100, 100), (5.2604, 5.2604), 340);
}

#[test]
fn a_stalled_encoder_fails_after_the_stall_timeout() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });

    // The first fresh sample sets the baseline; every later sample is a fresh
    // timestamp with unchanged counts, so the deltas are all zero.
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // The stall window starts at the intent start: 1 500 ms later is 2 500.
    let mut now_ms = 1_040;
    while now_ms <= 2_480 {
        assert_eq!(
            step(&mut intent, now_ms, enc(100, 100, now_ms), None),
            IntentAction::WaitUntil {
                deadline_ms: now_ms + 20
            }
        );
        now_ms += 20;
    }

    assert_eq!(
        step(&mut intent, 2_500, enc(100, 100, 2_500), None),
        IntentAction::Finished
    );

    let (fault, telemetry) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::Stall);
    assert_eq!(fault.label(), "StallTimeout");
    assert_distance_telemetry(telemetry, (0.0, 0.0), (0, 0), (5.2604, 5.2604), 1_500);
}

#[test]
fn a_single_motor_zero_fails_as_an_encoder_anomaly() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // Left stood still while the right moved 100 counts.
    assert_eq!(
        step(&mut intent, 1_040, enc(100, 200, 1_040), yaw(0.0, 1_040)),
        IntentAction::Finished
    );

    let (fault, telemetry) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::EncoderAnomaly);
    assert_eq!(fault.label(), "EncoderAnomaly");
    assert_distance_telemetry(telemetry, (0.0, 0.0), (0, 0), (5.2604, 5.2604), 40);
}

#[test]
fn progress_restarts_the_stall_window() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // Real progress at 1 040 moves the stall window from 1 000 to 1 040, so it
    // now ends at 2 540, not 2 500.
    assert_eq!(
        step(&mut intent, 1_040, enc(200, 200, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    let mut now_ms = 1_060;
    while now_ms <= 2_520 {
        assert_eq!(
            step(&mut intent, now_ms, enc(200, 200, now_ms), None),
            IntentAction::WaitUntil {
                deadline_ms: now_ms + 20
            }
        );
        now_ms += 20;
    }

    assert_eq!(
        step(&mut intent, 2_540, enc(200, 200, 2_540), None),
        IntentAction::Finished
    );

    let (fault, _) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::Stall);
}

// ── Curve geometry, ratios and inner-track progress ──────────────────────────

#[test]
fn a_curve_commands_the_inner_track_slower() {
    // 20 cm radius, 50 cm arc: the inner track travels 31.875 cm (1.6767 rev)
    // and the outer 68.125 cm (3.5836 rev), so the inner ratio is 0.4679.
    let mut left = Intent::new(curve(20.0, 50.0, TurnDirection::Left, FORWARD, 60), START_MS);
    assert_eq!(start(&mut left, 0.0), IntentAction::SetTracks { left: 28, right: 60 });
    assert_distance_telemetry(
        left.cancellation_telemetry(START_MS),
        (0.0, 0.0),
        (0, 0),
        (1.6767, 3.5836),
        0,
    );

    let mut right = Intent::new(curve(20.0, 50.0, TurnDirection::Right, FORWARD, 60), START_MS);
    assert_eq!(start(&mut right, 0.0), IntentAction::SetTracks { left: 60, right: 28 });

    // Reverse the same geometry runs at the same ratios with negative signs.
    let mut reverse = Intent::new(curve(20.0, 50.0, TurnDirection::Left, REVERSE, 60), START_MS);
    assert_eq!(
        start(&mut reverse, 0.0),
        IntentAction::SetTracks { left: -28, right: -60 }
    );
}

#[test]
fn a_curve_completes_on_its_inner_track() {
    let mut intent = Intent::new(curve(20.0, 50.0, TurnDirection::Left, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 28, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 50 inner and 700 outer counts: the outer is already past its 3.5836 rev
    // target, but the inner is at 0.2630 rev, so the drive keeps going. The
    // expected yaw (65 cm / 14.5 cm = 4.4828 rad) clamps the correction to 0.25.
    assert_eq!(
        step(&mut intent, 1_040, enc(150, 800, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 13, right: 60 }
    );

    // 270 more inner counts reach 1.6833 revolutions, past the inner target.
    assert_eq!(
        step(&mut intent, 1_060, enc(420, 830, 1_060), yaw(0.0, 1_060)),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(
        step(&mut intent, 1_060, enc(420, 830, 1_060), yaw(0.0, 1_060)),
        IntentAction::Finished
    );

    let telemetry = completed(intent.outcome());
    assert_distance_telemetry(telemetry, (1.6833, 3.8401), (320, 730), (1.6767, 3.5836), 60);
}

#[test]
fn the_curve_correction_uses_the_encoder_yaw_expectation() {
    let mut intent = Intent::new(curve(20.0, 50.0, TurnDirection::Left, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 28, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 20 inner and 40 outer counts (2 cm / 4 cm) expect (4 − 2) / 14.5 =
    // 0.1379 rad of yaw; the IMU still reads zero, so the outer is 0.25 too fast.
    assert_eq!(
        step(&mut intent, 1_040, enc(120, 140, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 24, right: 60 }
    );
}

#[test]
fn accumulated_curve_yaw_slows_the_outer_track() {
    let mut intent = Intent::new(curve(20.0, 50.0, TurnDirection::Left, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 28, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // The first progress tick seeds the yaw accumulator at 0°.
    assert_eq!(
        step(&mut intent, 1_040, enc(120, 140, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 24, right: 60 }
    );

    // Ten degrees of real yaw overrun the 0.1379 rad expectation, so the outer
    // ratio drops to 0.9817 while the inner rises to 0.4862.
    assert_eq!(
        step(&mut intent, 1_060, enc(140, 160, 1_060), yaw(10.0, 1_060)),
        IntentAction::SetTracks { left: 29, right: 59 }
    );
}

#[test]
fn accumulated_curve_yaw_wraps_across_the_seam() {
    let mut intent = Intent::new(curve(20.0, 50.0, TurnDirection::Left, FORWARD, 60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 28, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // 179° seeds the accumulator; equal deltas expect no yaw, so no correction.
    assert_eq!(
        step(&mut intent, 1_040, enc(120, 120, 1_040), yaw(179.0, 1_040)),
        IntentAction::SetTracks { left: 28, right: 60 }
    );

    // −170° is an 11° step across the seam, not −349°: the inner rises to
    // 0.5639 and the outer drops to 0.9040 of base.
    assert_eq!(
        step(&mut intent, 1_060, enc(140, 140, 1_060), yaw(-170.0, 1_060)),
        IntentAction::SetTracks { left: 34, right: 54 }
    );
}

#[test]
fn a_reverse_curve_negates_the_expected_yaw() {
    let mut intent = Intent::new(curve(20.0, 50.0, TurnDirection::Left, REVERSE, 60), START_MS);
    assert_eq!(
        start(&mut intent, 0.0),
        IntentAction::SetTracks { left: -28, right: -60 }
    );
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // The same 20/40 counts as the forward curve, but the expectation is
    // negative, so the correction flips: 0.5369 / 0.9310 of −60.
    assert_eq!(
        step(&mut intent, 1_040, enc(120, 140, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: -32, right: -56 }
    );
}

// ── Straight-line IMU heading hold ──────────────────────────────────────────

#[test]
fn a_straight_drive_holds_the_captured_heading() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    // The first observation carries the reference heading of 10°.
    assert_eq!(
        start(&mut intent, 10.0),
        IntentAction::SetTracks { left: 60, right: 60 }
    );
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(10.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // Five degrees off the captured 10°: clamped to 8 speed units.
    assert_eq!(
        step(&mut intent, 1_040, enc(1_000, 1_000, 1_040), yaw(15.0, 1_040)),
        IntentAction::SetTracks { left: 68, right: 52 }
    );
}

#[test]
fn heading_error_wraps_across_the_seam() {
    // 179° → −179° is +2° off the reference, not −358°.
    let mut plus = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut plus, 179.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut plus, 1_020, enc(500, 500, 1_020), yaw(179.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut plus, 1_040, enc(1_000, 1_000, 1_040), yaw(-179.0, 1_040)),
        IntentAction::SetTracks { left: 64, right: 56 }
    );

    // −179° → +179° is −2°, the mirrored correction.
    let mut minus = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(
        start(&mut minus, -179.0),
        IntentAction::SetTracks { left: 60, right: 60 }
    );
    assert_eq!(
        step(&mut minus, 1_020, enc(500, 500, 1_020), yaw(-179.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut minus, 1_040, enc(1_000, 1_000, 1_040), yaw(179.0, 1_040)),
        IntentAction::SetTracks { left: 56, right: 64 }
    );
}

#[test]
fn the_heading_correction_clamps_with_the_current_speed() {
    // 5° at 60 %: 2 × 5 = 10 clamped to min(60 × 0.15, 8) = 8 → 68 / 52.
    let mut fast = Intent::new(straight(100.0, FORWARD, 60), START_MS);
    assert_eq!(start(&mut fast, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut fast, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut fast, 1_040, enc(1_000, 1_000, 1_040), yaw(5.0, 1_040)),
        IntentAction::SetTracks { left: 68, right: 52 }
    );

    // 5° at 40 %: the clamp scales down to 6 → 46 / 34.
    let mut slow = Intent::new(straight(100.0, FORWARD, 40), START_MS);
    assert_eq!(start(&mut slow, 0.0), IntentAction::SetTracks { left: 40, right: 40 });
    assert_eq!(
        step(&mut slow, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut slow, 1_040, enc(1_000, 1_000, 1_040), yaw(5.0, 1_040)),
        IntentAction::SetTracks { left: 46, right: 34 }
    );

    // 5° at 90 %: the clamp is the absolute maximum of 8 → 98 / 82.
    let mut full = Intent::new(straight(100.0, FORWARD, 90), START_MS);
    assert_eq!(start(&mut full, 0.0), IntentAction::SetTracks { left: 90, right: 90 });
    assert_eq!(
        step(&mut full, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut full, 1_040, enc(1_000, 1_000, 1_040), yaw(5.0, 1_040)),
        IntentAction::SetTracks { left: 98, right: 82 }
    );
}

#[test]
fn a_reverse_straight_drive_keeps_the_heading_sign() {
    let mut intent = Intent::new(straight(100.0, REVERSE, 60), START_MS);
    assert_eq!(
        start(&mut intent, 0.0),
        IntentAction::SetTracks { left: -60, right: -60 }
    );
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // The same +5° error as the forward case, added to the negative base:
    // round(−60 + 8) = −52 and round(−60 − 8) = −68.
    assert_eq!(
        step(&mut intent, 1_040, enc(1_000, 1_000, 1_040), yaw(5.0, 1_040)),
        IntentAction::SetTracks { left: -52, right: -68 }
    );
}

#[test]
fn a_drive_without_yaw_runs_open_loop() {
    let mut intent = Intent::new(straight(100.0, FORWARD, 60), START_MS);

    // No heading sample: poll every 10 ms, up to the 50 ms bound.
    assert_eq!(
        step(&mut intent, 1_000, None, None),
        IntentAction::WaitUntil { deadline_ms: 1_010 }
    );
    assert_eq!(
        step(&mut intent, 1_010, None, None),
        IntentAction::WaitUntil { deadline_ms: 1_020 }
    );
    assert_eq!(
        step(&mut intent, 1_020, None, None),
        IntentAction::WaitUntil { deadline_ms: 1_030 }
    );
    assert_eq!(
        step(&mut intent, 1_030, None, None),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut intent, 1_040, None, None),
        IntentAction::WaitUntil { deadline_ms: 1_050 }
    );
    assert_eq!(
        step(&mut intent, 1_050, None, None),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    // Encoder samples without yaw keep the drive running, uncorrected.
    assert_eq!(
        step(&mut intent, 1_070, enc(500, 500, 1_070), None),
        IntentAction::WaitUntil { deadline_ms: 1_090 }
    );
    assert_eq!(
        step(&mut intent, 1_090, enc(1_000, 1_000, 1_090), None),
        IntentAction::SetTracks { left: 60, right: 60 }
    );
    assert_eq!(intent.outcome(), None);
}
