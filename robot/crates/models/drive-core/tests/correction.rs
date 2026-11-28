//! Host-side tests for the provisional post-stop rotation correction phase.
//!
//! Every assertion goes through the public `Intent` surface: the action
//! sequence, the terminal outcome and the telemetry. The expected actions are
//! literal worked examples (the turn, the settle wait and the torque-ladder
//! pulses), never a re-run of the model's arithmetic.
//!
//! Samples exist only at the timestamps listed in each scenario; every other
//! observation carries no yaw, which is what a poll every [`POLL_MS`] sees
//! between IMU samples. That keeps the per-iteration timing explicit:
//!
//! - a pulse starts at `S`, polls every 5 ms, and times out strictly after
//!   `S + 400`;
//! - the settle after it gives up at `S + 405 + 400`.
//!
//! The tests drive the model the way the drive task does: execute a
//! non-blocking action and step again immediately, and sleep to the absolute
//! deadline of a `WaitUntil` before the next observation.

use drive_core::{
    CompletionTelemetry, DriveFault, DriveOutcome, Intent, IntentAction, IntentParams, Observation, RotationDirection,
    RotationMotion, YawSample,
};

/// Clockwise rotation.
const CW: RotationDirection = RotationDirection::Clockwise;
/// A stationary turn at 60 % rotation speed.
const SPEED: RotationMotion = RotationMotion::Stationary { speed: 60 };
/// A turn while moving forward at 50 %; it never enters the correction phase.
const FORWARD: RotationMotion = RotationMotion::WhileMoving(50);
/// The turn starts at this instant in every scenario.
const START_MS: u64 = 1_000;

/// An observation carrying only the clock and an optional yaw sample.
const fn observation(now_ms: u64, yaw: Option<YawSample>) -> Observation {
    Observation {
        now_ms,
        encoder: None,
        yaw,
    }
}

/// Rotation parameters.
const fn rotate(degrees: f32, direction: RotationDirection, motion: RotationMotion) -> IntentParams {
    IntentParams::RotateExact {
        degrees,
        direction,
        motion,
    }
}

/// Samples that exist only at exactly these timestamps.
fn samples_at(entries: &[(u64, f32)]) -> impl FnMut(u64) -> Option<YawSample> + '_ {
    move |now_ms| {
        entries
            .iter()
            .find(|(timestamp_ms, _)| *timestamp_ms == now_ms)
            .map(|&(timestamp_ms, yaw_deg)| YawSample { yaw_deg, timestamp_ms })
    }
}

/// Run the adapter contract until `Finished`, returning the actions and the
/// clock value at which the intent finished.
fn run(
    intent: &mut Intent,
    start_ms: u64,
    mut sample_at: impl FnMut(u64) -> Option<YawSample>,
) -> (Vec<IntentAction>, u64) {
    let mut now_ms = start_ms;
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

/// The non-zero `SetTracks` commands: the turn ramps and the correction pulses.
fn pulses(actions: &[IntentAction]) -> Vec<(i8, i8)> {
    actions
        .iter()
        .filter_map(|action| match action {
            IntentAction::SetTracks { left, right } if *left != 0 || *right != 0 => Some((*left, *right)),
            _ => None,
        })
        .collect()
}

/// The action the adapter stops on.
const fn tracks(left: i8, right: i8) -> IntentAction {
    IntentAction::SetTracks { left, right }
}

/// A `WaitUntil` action.
const fn wait(deadline_ms: u64) -> IntentAction {
    IntentAction::WaitUntil { deadline_ms }
}

/// The terminal `Finished` action.
const FINISHED: IntentAction = IntentAction::Finished;

// ── Convergence ───────────────────────────────────────────────────────────────

#[test]
fn convergence_at_entry_finishes_without_a_pulse() {
    // The turn stops at 1 040 (yaw −90). The settle sample at 1 440 is still
    // inside tolerance, so the loop-entry check finishes before any pulse.
    let samples = &[(1_020, 0.0), (1_040, -90.0), (1_440, -90.0)];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(
        actions,
        vec![
            tracks(0, 0),
            wait(1_020),
            tracks(60, -60),
            wait(1_040),
            tracks(0, 0),
            wait(1_440),
            FINISHED,
        ]
    );
    assert_eq!(finished_at, 1_440);
    // Correction-inclusive telemetry: the stop's final yaw, zero error, the
    // duration through the settle.
    assert_completed(&intent, -90.0, 0.0, 440);
}

#[test]
fn convergence_mid_pulse_stops_and_finishes() {
    // The settle sample at 1 440 is 10° short. The first pulse runs; the sample
    // at 1 460 lands inside tolerance, so the pulse stops and the intent ends.
    let samples = &[(1_020, 0.0), (1_040, -90.0), (1_440, -80.0), (1_460, -90.0)];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(
        actions,
        vec![
            tracks(0, 0),
            wait(1_020),
            tracks(60, -60),
            wait(1_040),
            tracks(0, 0),
            wait(1_440),
            // 10° of error: round(10 × 6) = 60, clamped to the 35 start speed.
            tracks(35, -35),
            wait(1_445),
            wait(1_450),
            wait(1_455),
            wait(1_460),
            tracks(0, 0),
            FINISHED,
        ]
    );
    assert_eq!(finished_at, 1_460);
    assert_completed(&intent, -90.0, 0.0, 460);
}

#[test]
fn convergence_after_the_post_pulse_settle_finishes_without_another_stop() {
    // The pulse overshoots at 1 450 (−90.6°), so it stops and settles. The
    // settle sample at 1 850 is back inside tolerance: the correction finishes
    // without a second stop command.
    let samples = &[
        (1_020, 0.0),
        (1_040, -90.0),
        (1_440, -80.0),
        (1_450, -90.6),
        (1_850, -90.2),
    ];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(
        actions,
        vec![
            tracks(0, 0),
            wait(1_020),
            tracks(60, -60),
            wait(1_040),
            tracks(0, 0),
            wait(1_440),
            tracks(35, -35),
            wait(1_445),
            wait(1_450),
            // −90.6° is a sign flip: the overshoot stops the pulse.
            tracks(0, 0),
            wait(1_850),
            FINISHED,
        ]
    );
    assert_eq!(finished_at, 1_850);
    assert_completed(&intent, -90.2, 0.2, 850);
}

#[test]
fn a_missing_settle_sample_keeps_the_stops_final_yaw() {
    // No sample arrives after the stop. The settle gives up at 1 440 and keeps
    // the turn's final yaw, which is inside tolerance.
    let samples = &[(1_020, 0.0), (1_040, -90.0)];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(
        actions,
        vec![
            tracks(0, 0),
            wait(1_020),
            tracks(60, -60),
            wait(1_040),
            tracks(0, 0),
            wait(1_440),
            FINISHED,
        ]
    );
    assert_eq!(finished_at, 1_440);
    assert_completed(&intent, -90.0, 0.0, 440);
}

// ── Torque ladder, timeout and overshoot ──────────────────────────────────────

#[test]
fn the_torque_ladder_steps_down_each_iteration() {
    // Ten degrees of error: the 35 start speed steps down by two per iteration.
    // Each pulse times out at `S + 405`; the settle then finds the same error at
    // `S + 805`, so the next iteration starts there. The fourth pulse converges
    // at 3 860.
    let samples = &[
        (1_020, 0.0),
        (1_040, -90.0),
        (1_440, -80.0),
        (2_245, -80.0),
        (3_050, -80.0),
        (3_855, -80.0),
        (3_860, -90.0),
    ];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(
        pulses(&actions),
        vec![(60, -60), (35, -35), (33, -33), (31, -31), (29, -29)]
    );
    // The turn ramp is the first pulse; the correction ladder follows it.
    assert_eq!(&actions[actions.len() - 2..], &[tracks(0, 0), FINISHED]);
    assert_eq!(finished_at, 3_860);
    assert_completed(&intent, -90.0, 0.0, 2_860);
}

#[test]
fn a_sign_flip_after_the_settle_reverses_the_next_pulse() {
    // The first pulse is an undershoot (+10°). Its per-iteration timeout stops
    // it, and the settle sample at 1 845 has flipped to −5°: the second pulse
    // reverses direction and steps the ladder down to 28.
    let samples = &[
        (1_020, 0.0),
        (1_040, -90.0),
        (1_440, -80.0),
        (1_845, -95.0),
        (1_850, -90.0),
    ];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, _) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(pulses(&actions), vec![(60, -60), (35, -35), (-28, 28)]);
    assert_completed(&intent, -90.0, 0.0, 850);
}

#[test]
fn the_correction_exhausts_after_fifteen_pulses() {
    // The error never returns to tolerance. Every pulse times out at `S + 405`
    // and every settle gives up at `S + 805`; the fifteenth settle exhausts it.
    let samples = &[(1_020, 0.0), (1_040, -90.0), (1_440, -80.0)];
    let mut intent = Intent::new(rotate(90.0, CW, SPEED), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    // Pulses land at 1 440, 2 245, 3 050, … and step down to the 25 floor.
    assert_eq!(
        pulses(&actions),
        vec![
            (60, -60),
            (35, -35),
            (33, -33),
            (31, -31),
            (29, -29),
            (27, -27),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
            (25, -25),
        ]
    );
    // 1 440 + 14 × 805 + 405 + 400 = 13 515.
    assert_eq!(finished_at, 13_515);
    assert_failed(&intent, DriveFault::CorrectionExhausted, -80.0, -10.0, 12_515);
}

// ── No correction for a while-moving turn ─────────────────────────────────────

#[test]
fn a_while_moving_turn_never_enters_the_correction_phase() {
    // Same completion sample as the stationary turn, but the while-moving path
    // stops and finishes at once: no settle, no pulse.
    let samples = &[
        (1_020, 0.0),
        (1_040, -30.0),
        (1_060, -70.0),
        (1_080, -89.0),
        (1_100, -90.0),
    ];
    let mut intent = Intent::new(rotate(90.0, CW, FORWARD), START_MS);
    let (actions, finished_at) = run(&mut intent, START_MS, samples_at(samples));

    assert_eq!(
        actions,
        vec![
            tracks(0, 0),
            wait(1_020),
            tracks(50, 20),
            wait(1_040),
            tracks(50, 20),
            wait(1_060),
            tracks(50, 20),
            wait(1_080),
            tracks(50, 20),
            wait(1_100),
            tracks(0, 0),
            FINISHED,
        ]
    );
    assert_eq!(finished_at, 1_100);
    assert_completed(&intent, -90.0, 0.0, 100);
}
