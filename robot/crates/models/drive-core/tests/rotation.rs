//! Host-side tests for the rotation turn control law behind `Intent::step`.
//!
//! Every assertion goes through the public `Intent` surface: the action
//! sequence, the terminal outcome and the telemetry. The expected speeds are
//! literal worked examples (start yaw, sample yaw and the resulting track
//! speeds), never a re-run of the model's arithmetic.
//!
//! The adapter contract these tests follow is the same one the firmware uses:
//! execute a non-blocking action and step again immediately, and sleep to the
//! absolute deadline of a `WaitUntil` before the next observation.

use drive_core::{
    CompletionTelemetry, DriveFault, DriveOutcome, Intent, IntentAction, IntentParams, Observation, RotationDirection,
    RotationMotion, YawSample,
};

/// Clockwise rotation.
const CW: RotationDirection = RotationDirection::Clockwise;
/// Counter-clockwise rotation.
const CCW: RotationDirection = RotationDirection::CounterClockwise;
/// A stationary turn at 60 % rotation speed.
const SPEED: RotationMotion = RotationMotion::Stationary { speed: 60 };
/// A stationary turn at 70 % rotation speed.
const SPEED_70: RotationMotion = RotationMotion::Stationary { speed: 70 };
/// A forward turn while moving at 50 %.
const FORWARD: RotationMotion = RotationMotion::WhileMoving(50);
/// A reverse turn while moving at −50 %.
const REVERSE: RotationMotion = RotationMotion::WhileMoving(-50);

/// An observation carrying only the clock and an optional yaw sample.
const fn observation(now_ms: u64, yaw: Option<YawSample>) -> Observation {
    Observation {
        now_ms,
        encoder: None,
        yaw,
    }
}

/// A yaw sample stamped at `timestamp_ms`.
const fn yaw(yaw_deg: f32, timestamp_ms: u64) -> Option<YawSample> {
    Some(YawSample { yaw_deg, timestamp_ms })
}

/// Rotation parameters.
const fn rotate(degrees: f32, direction: RotationDirection, motion: RotationMotion) -> IntentParams {
    IntentParams::RotateExact {
        degrees,
        direction,
        motion,
    }
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

/// Drive the initial zero command and the wait that follows it.
///
/// Returns the first control-tick deadline, which is `start_ms + 20`.
fn start(intent: &mut Intent, start_ms: u64) -> u64 {
    let first = intent.step(&observation(start_ms, None));
    assert_eq!(first.action, IntentAction::SetTracks { left: 0, right: 0 });
    #[cfg(not(feature = "trace"))]
    assert!(first.trace.is_none(), "trace must be off in a `std`-only build");

    wait_deadline(intent.step(&observation(start_ms, None)).action)
}

/// Feed one control tick at `now_ms` with a yaw sample stamped at `now_ms`.
fn tick(intent: &mut Intent, now_ms: u64, yaw_deg: f32) -> IntentAction {
    intent.step(&observation(now_ms, yaw(yaw_deg, now_ms))).action
}

/// The `WaitUntil` emitted immediately after a non-terminal `SetTracks`.
fn follow(intent: &mut Intent, now_ms: u64) -> u64 {
    wait_deadline(intent.step(&observation(now_ms, None)).action)
}

/// Run the adapter contract until `Finished`, returning the actions and the
/// clock value at which the intent finished.
fn run(
    intent: &mut Intent,
    mut now_ms: u64,
    mut sample_at: impl FnMut(u64) -> Option<YawSample>,
) -> (Vec<IntentAction>, u64) {
    let mut actions = Vec::new();
    loop {
        let step = intent.step(&observation(now_ms, sample_at(now_ms)));
        actions.push(step.action);
        match step.action {
            IntentAction::WaitUntil { deadline_ms } => now_ms = deadline_ms,
            IntentAction::Finished => return (actions, now_ms),
            IntentAction::SetTracks { .. } | IntentAction::BrakeAll | IntentAction::CoastAll => {}
        }
        assert!(actions.len() < 5_000, "intent did not reach Finished");
    }
}

/// Assert two floats are the same worked-example value.
fn assert_close(actual: f32, expected: f32) {
    assert!((actual - expected).abs() < 1e-3, "expected {expected}, got {actual}");
}

/// Assert the rotate telemetry is the worked example.
fn assert_rotate_telemetry(telemetry: CompletionTelemetry, final_yaw_deg: f32, angle_error_deg: f32, duration_ms: u64) {
    match telemetry {
        CompletionTelemetry::RotateExact {
            final_yaw_deg: actual_yaw,
            angle_error_deg: actual_error,
            duration_ms: actual_duration,
        } => {
            assert_close(actual_yaw, final_yaw_deg);
            assert_close(actual_error, angle_error_deg);
            assert_eq!(actual_duration, duration_ms);
        }
        other => assert!(
            matches!(other, CompletionTelemetry::RotateExact { .. }),
            "expected RotateExact telemetry, got {other:?}"
        ),
    }
}

/// Assert the intent completed with the worked-example telemetry.
fn assert_completed(intent: &Intent, final_yaw_deg: f32, angle_error_deg: f32, duration_ms: u64) {
    match intent.outcome() {
        Some(DriveOutcome::Completed { telemetry }) => {
            assert_rotate_telemetry(telemetry, final_yaw_deg, angle_error_deg, duration_ms);
        }
        other => assert!(
            matches!(other, Some(DriveOutcome::Completed { .. })),
            "expected Completed, got {other:?}"
        ),
    }
}

/// Assert the intent failed with `fault` and the worked-example telemetry.
fn assert_failed(intent: &Intent, fault: DriveFault, final_yaw_deg: f32, angle_error_deg: f32, duration_ms: u64) {
    match intent.outcome() {
        Some(DriveOutcome::Failed {
            fault: actual_fault,
            telemetry,
        }) => {
            assert_eq!(actual_fault, fault);
            assert_rotate_telemetry(telemetry, final_yaw_deg, angle_error_deg, duration_ms);
        }
        other => assert!(
            matches!(other, Some(DriveOutcome::Failed { .. })),
            "expected Failed {fault:?}, got {other:?}"
        ),
    }
}

#[test]
fn the_turn_zeroes_the_tracks_then_waits_for_the_first_tick() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);

    let first = intent.step(&observation(1_000, None));
    assert_eq!(first.action, IntentAction::SetTracks { left: 0, right: 0 });
    assert!(intent.outcome().is_none(), "the intent is not finished yet");
    #[cfg(not(feature = "trace"))]
    assert!(first.trace.is_none(), "trace must be off in a `std`-only build");

    let wait = intent.step(&observation(1_000, None));
    assert_eq!(wait.action, IntentAction::WaitUntil { deadline_ms: 1_020 });

    // A processed tick emits the speeds and then the wait for the next tick.
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );
    assert_eq!(follow(&mut intent, 1_020), 1_040);
}

#[test]
fn a_missing_sample_waits_the_sample_bound_then_keeps_the_previous_command() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    let first_deadline = start(&mut intent, 1_000);
    assert_eq!(first_deadline, 1_020);

    // Tick start, no sample: wait the 20 ms IMU bound, then retry the drain.
    let retry = intent.step(&observation(1_020, None));
    assert_eq!(retry.action, IntentAction::WaitUntil { deadline_ms: 1_040 });

    // Retry still empty: the tick ends and the next control tick is 20 ms later.
    let next_tick = intent.step(&observation(1_040, None));
    assert_eq!(next_tick.action, IntentAction::WaitUntil { deadline_ms: 1_060 });

    // The next tick starts and again waits for a sample.
    let again = intent.step(&observation(1_060, None));
    assert_eq!(again.action, IntentAction::WaitUntil { deadline_ms: 1_080 });

    // A sample older than the intent start is ignored, just like the firmware drain.
    let stale = intent.step(&observation(1_080, yaw(0.0, 999)));
    assert_eq!(stale.action, IntentAction::WaitUntil { deadline_ms: 1_100 });
}

#[test]
fn clockwise_stationary_turn_uses_the_ramp_law() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);

    // First sample anchors the target at −90° and leaves 90° to go.
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );
    // 60° remaining sits on the ramp threshold, so the requested speed holds.
    assert_eq!(
        tick(&mut intent, 1_040, -30.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );
    // 45° remaining ramps to 45 + 15 × 45/60 = 56.25, truncated to 56.
    assert_eq!(
        tick(&mut intent, 1_060, -45.0),
        IntentAction::SetTracks { left: 56, right: -56 }
    );
    // 15° remaining ramps to 45 + 15 × 15/60 = 48.75, truncated to 48.
    assert_eq!(
        tick(&mut intent, 1_080, -75.0),
        IntentAction::SetTracks { left: 48, right: -48 }
    );
}

#[test]
fn counter_clockwise_stationary_turn_mirrors_the_speeds() {
    let mut intent = Intent::new(rotate(90.0, CCW, SPEED), 1_000);
    start(&mut intent, 1_000);

    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: -60, right: 60 }
    );
    assert_eq!(
        tick(&mut intent, 1_040, 30.0),
        IntentAction::SetTracks { left: -60, right: 60 }
    );
    assert_eq!(
        tick(&mut intent, 1_060, 45.0),
        IntentAction::SetTracks { left: -56, right: 56 }
    );
    assert_eq!(
        tick(&mut intent, 1_080, 75.0),
        IntentAction::SetTracks { left: -48, right: 48 }
    );
}

#[test]
fn while_moving_forward_turns_on_the_inner_track() {
    let mut forward_cw = Intent::new(rotate(90.0, CW, FORWARD), 1_000);
    start(&mut forward_cw, 1_000);
    // CW while moving: base 50 on the left, 50 − min(speed, 30) on the right.
    assert_eq!(
        tick(&mut forward_cw, 1_020, 0.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );
    assert_eq!(
        tick(&mut forward_cw, 1_040, -30.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );
    assert_eq!(
        tick(&mut forward_cw, 1_060, -75.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );

    let mut forward_ccw = Intent::new(rotate(90.0, CCW, FORWARD), 1_000);
    start(&mut forward_ccw, 1_000);
    // CCW while moving: the differential moves to the left track.
    assert_eq!(
        tick(&mut forward_ccw, 1_020, 0.0),
        IntentAction::SetTracks { left: 20, right: 50 }
    );
    assert_eq!(
        tick(&mut forward_ccw, 1_040, 30.0),
        IntentAction::SetTracks { left: 20, right: 50 }
    );
}

#[test]
fn while_moving_reverse_turns_on_the_inner_track() {
    let mut reverse_cw = Intent::new(rotate(90.0, CW, REVERSE), 1_000);
    start(&mut reverse_cw, 1_000);
    // CW while reversing: base −50 on the left, −50 − 30 on the right.
    assert_eq!(
        tick(&mut reverse_cw, 1_020, 0.0),
        IntentAction::SetTracks { left: -50, right: -80 }
    );
    assert_eq!(
        tick(&mut reverse_cw, 1_040, -30.0),
        IntentAction::SetTracks { left: -50, right: -80 }
    );

    let mut reverse_ccw = Intent::new(rotate(90.0, CCW, REVERSE), 1_000);
    start(&mut reverse_ccw, 1_000);
    assert_eq!(
        tick(&mut reverse_ccw, 1_020, 0.0),
        IntentAction::SetTracks { left: -80, right: -50 }
    );
}

#[test]
fn the_first_sample_anchors_the_absolute_target() {
    // Clockwise 20° from a yaw of 170°: target yaw 150°, not 20° off zero.
    let mut cw = Intent::new(rotate(20.0, CW, SPEED), 1_000);
    start(&mut cw, 1_000);
    // 20° remaining ramps to 45 + 15 × 20/60 = 50.
    assert_eq!(
        tick(&mut cw, 1_020, 170.0),
        IntentAction::SetTracks { left: 50, right: -50 }
    );
    // 10° remaining ramps to 45 + 15 × 10/60 = 47.5, truncated to 47.
    assert_eq!(
        tick(&mut cw, 1_040, 160.0),
        IntentAction::SetTracks { left: 47, right: -47 }
    );
    // Reaching 150° completes the turn.
    assert_eq!(
        tick(&mut cw, 1_060, 150.0),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(cw.step(&observation(1_060, None)).action, IntentAction::Finished);
    assert_completed(&cw, 150.0, 0.0, 60);

    // Counter-clockwise 20° from 170°: target yaw wraps to −170°.
    let mut ccw = Intent::new(rotate(20.0, CCW, SPEED), 1_000);
    start(&mut ccw, 1_000);
    assert_eq!(
        tick(&mut ccw, 1_020, 170.0),
        IntentAction::SetTracks { left: -50, right: 50 }
    );
    assert_eq!(
        tick(&mut ccw, 1_040, 180.0),
        IntentAction::SetTracks { left: -47, right: 47 }
    );
    assert_eq!(
        tick(&mut ccw, 1_060, -170.0),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(ccw.step(&observation(1_060, None)).action, IntentAction::Finished);
    assert_completed(&ccw, -170.0, 0.0, 60);
}

#[test]
fn the_error_wraps_across_the_plus_minus_180_seam() {
    // Clockwise 20° from −170° wraps the target to +170°.
    let mut intent = Intent::new(rotate(20.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);

    assert_eq!(
        tick(&mut intent, 1_020, -170.0),
        IntentAction::SetTracks { left: 50, right: -50 }
    );
    // A sample at −180° is 10° short of +170°, so the error is +10, not +350.
    assert_eq!(
        tick(&mut intent, 1_040, -180.0),
        IntentAction::SetTracks { left: 47, right: -47 }
    );
}

#[test]
fn ramp_down_above_at_and_below_the_threshold() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED_70), 1_000);
    start(&mut intent, 1_000);

    // 90° remaining: the full requested 70 %.
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 70, right: -70 }
    );
    // 61° remaining, just above the ramp threshold: still 70 %.
    assert_eq!(
        tick(&mut intent, 1_040, -29.0),
        IntentAction::SetTracks { left: 70, right: -70 }
    );
    // 60° remaining, exactly on the threshold: the ramp has not started.
    assert_eq!(
        tick(&mut intent, 1_060, -30.0),
        IntentAction::SetTracks { left: 70, right: -70 }
    );
    // 45° remaining: 45 + (70 − 45) × 45/60 = 63.75, truncated to 63.
    assert_eq!(
        tick(&mut intent, 1_080, -45.0),
        IntentAction::SetTracks { left: 63, right: -63 }
    );
    // 30° remaining: 45 + 25 × 30/60 = 57.5, truncated to 57.
    assert_eq!(
        tick(&mut intent, 1_100, -60.0),
        IntentAction::SetTracks { left: 57, right: -57 }
    );
    // 5° remaining: 45 + 25 × 5/60 = 47.08, truncated to 47.
    assert_eq!(
        tick(&mut intent, 1_120, -85.0),
        IntentAction::SetTracks { left: 47, right: -47 }
    );
}

#[test]
fn a_sample_within_tolerance_completes_the_turn() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );

    // −0.5° error is exactly inside the tolerance.
    assert_eq!(
        tick(&mut intent, 1_040, -90.5),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert!(
        intent.outcome().is_none(),
        "the outcome is published only with Finished"
    );

    assert_eq!(intent.step(&observation(1_040, None)).action, IntentAction::Finished);
    // final yaw −90.5, error 0.5, duration 40 ms.
    assert_completed(&intent, -90.5, 0.5, 40);
    assert_eq!(
        intent.step(&observation(1_999, None)).action,
        IntentAction::Finished,
        "Finished is idempotent"
    );
}

#[test]
fn an_overshoot_reverses_the_turn() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );

    // −5° remaining is a 5° overshoot: the effective direction flips to CCW
    // and the ramp gives 45 + 15 × 5/60 = 46.25, truncated to 46.
    assert_eq!(
        tick(&mut intent, 1_040, -95.0),
        IntentAction::SetTracks { left: -46, right: 46 }
    );
}

#[test]
fn the_turn_fails_after_the_flip_limit() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);
    // Anchor on the positive side, so every following sign change is a flip.
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );

    let mut now = 1_020;
    let mut last = IntentAction::WaitUntil { deadline_ms: 0 };
    for step in 0..12 {
        now += 20;
        let sample = if step % 2 == 0 { -95.0 } else { 0.0 };
        last = tick(&mut intent, now, sample);
    }

    // The twelfth flip fails the turn with the stop command.
    assert_eq!(now, 1_260);
    assert_eq!(last, IntentAction::SetTracks { left: 0, right: 0 });
    assert_eq!(intent.step(&observation(now, None)).action, IntentAction::Finished);
    // final yaw 0°, error −90°, duration 260 ms.
    assert_failed(&intent, DriveFault::CorrectionLimit, 0.0, -90.0, 260);
}

#[test]
fn the_turn_fails_when_the_overshoot_correction_runs_too_long() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );
    // First flip starts the 4 000 ms correction clock at t = 1 040.
    assert_eq!(
        tick(&mut intent, 1_040, -95.0),
        IntentAction::SetTracks { left: -46, right: 46 }
    );

    let mut now = 1_040;
    let mut last = IntentAction::WaitUntil { deadline_ms: 0 };
    while now < 5_060 {
        now += 20;
        last = tick(&mut intent, now, -95.0);
    }

    // 1 040 + 4 020 ms: the first tick strictly past the 4 000 ms bound.
    assert_eq!(now, 5_060);
    assert_eq!(last, IntentAction::SetTracks { left: 0, right: 0 });
    assert_eq!(intent.step(&observation(now, None)).action, IntentAction::Finished);
    // final yaw −95°, error +5°, duration 4 060 ms.
    assert_failed(&intent, DriveFault::CorrectionTimeout, -95.0, 5.0, 4_060);
}

#[test]
fn the_turn_times_out_with_samples_present() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );

    let mut now = 1_020;
    let mut last = IntentAction::WaitUntil { deadline_ms: 0 };
    while now < 9_020 {
        now += 20;
        last = tick(&mut intent, now, 0.0);
    }

    // 8 000 ms is a strict bound: the first tick at 8 020 ms fails.
    assert_eq!(now, 9_020);
    assert_eq!(last, IntentAction::SetTracks { left: 0, right: 0 });
    assert_eq!(intent.step(&observation(now, None)).action, IntentAction::Finished);
    // No sample ever moved the turn, so the final yaw is still 0°.
    assert_failed(&intent, DriveFault::RotationTimeout, 0.0, -90.0, 8_020);
}

#[test]
fn the_turn_times_out_when_no_sample_ever_arrives() {
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    let first_deadline = start(&mut intent, 1_000);
    assert_eq!(first_deadline, 1_020);

    let (actions, finished_at) = run(&mut intent, first_deadline, |_| None);

    // The last two actions are always the stop command and Finished.
    assert_eq!(
        actions[actions.len() - 2],
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(actions[actions.len() - 1], IntentAction::Finished);
    // The 8 000 ms bound is reached on a 40 ms no-sample tick cadence.
    assert_eq!(finished_at, 9_020);
    assert_failed(&intent, DriveFault::RotationTimeout, 0.0, -90.0, 8_020);
}

#[test]
fn a_while_moving_turn_completes_with_its_elapsed_duration() {
    let mut intent = Intent::new(rotate(90.0, CW, FORWARD), 1_000);
    start(&mut intent, 1_000);
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );
    assert_eq!(
        tick(&mut intent, 1_040, -30.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );
    assert_eq!(
        tick(&mut intent, 1_060, -70.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );
    assert_eq!(
        tick(&mut intent, 1_080, -89.0),
        IntentAction::SetTracks { left: 50, right: 20 }
    );
    assert_eq!(
        tick(&mut intent, 1_100, -90.0),
        IntentAction::SetTracks { left: 0, right: 0 }
    );
    assert_eq!(intent.step(&observation(1_100, None)).action, IntentAction::Finished);

    // 1 100 − 1 000 = 100 ms.
    assert_completed(&intent, -90.0, 0.0, 100);
}

#[test]
fn cancellation_telemetry_reports_the_state_at_the_interrupt() {
    // Before the first sample the yaw is 0 and the error is the raw target.
    let intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    assert_rotate_telemetry(intent.cancellation_telemetry(1_500), 0.0, -90.0, 500);

    // After two samples the last yaw and the live remaining error are reported.
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), 1_000);
    start(&mut intent, 1_000);
    assert_eq!(
        tick(&mut intent, 1_020, 0.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );
    assert_eq!(
        tick(&mut intent, 1_040, -30.0),
        IntentAction::SetTracks { left: 60, right: -60 }
    );
    assert_rotate_telemetry(intent.cancellation_telemetry(1_120), -30.0, -60.0, 120);
}
