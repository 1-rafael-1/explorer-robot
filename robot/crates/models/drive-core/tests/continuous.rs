//! Host-side tests for the continuous drive behind `Intent::step`.
//!
//! A continuous drive is the straight controller with no target: it holds the
//! commanded speed and heading, never ramps, never completes and reports no
//! telemetry. It ends only on a fault (which the adapter turns into a
//! completion) or on an interrupt (which the adapter cancels).
//!
//! Every assertion goes through the public `Intent` surface and uses literal
//! worked examples, driven the way the drive task drives the model.

use drive_core::{
    CompletionTelemetry, DriveDirection, DriveFault, DriveOutcome, EncoderSample, Intent, IntentAction, IntentParams,
    Observation, YawSample,
};

/// Every scenario starts at this instant.
const START_MS: u64 = 1_000;
/// Forward motion.
const FORWARD: DriveDirection = DriveDirection::Forward;

/// A continuous drive parameter set.
const fn continuous(speed: u8) -> IntentParams {
    IntentParams::DriveContinuous {
        direction: FORWARD,
        speed,
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
fn start(intent: &mut Intent, yaw_deg: f32) -> IntentAction {
    let first = step(intent, START_MS, None, yaw(yaw_deg, START_MS));
    assert_eq!(wait_deadline(step(intent, START_MS, None, None)), START_MS + 20);
    first
}

/// The fault and telemetry of a failed drive, or a failed assertion.
fn failed(outcome: Option<DriveOutcome>) -> (DriveFault, CompletionTelemetry) {
    match outcome {
        Some(DriveOutcome::Failed { fault, telemetry }) => (fault, telemetry),
        other => panic!("expected a failed drive, got {other:?}"),
    }
}

// ── Holding speed, never ramping, never completing ───────────────────────────

#[test]
fn a_continuous_drive_holds_its_speed_and_never_completes() {
    let mut intent = Intent::new(continuous(60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // Ten more ticks: 10 000 counts is 52.6 revolutions, far past any target a
    // distance drive would have stopped at, yet the speed never ramps and the
    // drive never finishes.
    let mut now_ms = 1_040;
    let mut counts = 500_u16;
    while now_ms <= 1_220 {
        counts += 1_000;
        assert_eq!(
            step(&mut intent, now_ms, enc(counts, counts, now_ms), yaw(0.0, now_ms)),
            IntentAction::SetTracks { left: 60, right: 60 }
        );
        assert_eq!(
            wait_deadline(step(&mut intent, now_ms, enc(counts, counts, now_ms), None)),
            now_ms + 20
        );
        now_ms += 20;
    }

    assert_eq!(intent.outcome(), None);
}

#[test]
fn a_continuous_drive_holds_the_captured_heading() {
    let mut intent = Intent::new(continuous(60), START_MS);
    assert_eq!(
        start(&mut intent, 10.0),
        IntentAction::SetTracks { left: 60, right: 60 }
    );
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(10.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // Five degrees off the captured heading clamp to 8 speed units, exactly as
    // on a target straight drive.
    assert_eq!(
        step(&mut intent, 1_040, enc(1_000, 1_000, 1_040), yaw(15.0, 1_040)),
        IntentAction::SetTracks { left: 68, right: 52 }
    );
}

// ── Faults: stall, encoder timeout and anomaly ───────────────────────────────

#[test]
fn a_continuous_drive_fails_on_a_stall_without_telemetry() {
    let mut intent = Intent::new(continuous(60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    // Fresh timestamps with unchanged counts: zero deltas until the 1 500 ms
    // stall window that started at 1 000 runs out.
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
    assert_eq!(telemetry, CompletionTelemetry::None);
}

#[test]
fn a_continuous_drive_fails_on_encoder_timeout_without_telemetry() {
    let mut intent = Intent::new(continuous(60), START_MS);
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
    assert_eq!(telemetry, CompletionTelemetry::None);
}

#[test]
fn a_continuous_drive_fails_on_a_single_motor_zero_without_telemetry() {
    let mut intent = Intent::new(continuous(60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(100, 100, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );

    assert_eq!(
        step(&mut intent, 1_040, enc(100, 200, 1_040), yaw(0.0, 1_040)),
        IntentAction::Finished
    );

    let (fault, telemetry) = failed(intent.outcome());
    assert_eq!(fault, DriveFault::EncoderAnomaly);
    assert_eq!(fault.label(), "EncoderAnomaly");
    assert_eq!(telemetry, CompletionTelemetry::None);
}

// ── Cancellation and the heading wait ────────────────────────────────────────

#[test]
fn cancelling_a_continuous_drive_reports_no_telemetry() {
    let mut intent = Intent::new(continuous(60), START_MS);
    assert_eq!(start(&mut intent, 0.0), IntentAction::SetTracks { left: 60, right: 60 });
    assert_eq!(
        step(&mut intent, 1_020, enc(500, 500, 1_020), yaw(0.0, 1_020)),
        IntentAction::WaitUntil { deadline_ms: 1_040 }
    );
    assert_eq!(
        step(&mut intent, 1_040, enc(1_000, 1_000, 1_040), yaw(0.0, 1_040)),
        IntentAction::SetTracks { left: 60, right: 60 }
    );

    assert_eq!(intent.cancellation_telemetry(2_500), CompletionTelemetry::None);
}

#[test]
fn a_continuous_drive_without_yaw_waits_then_runs_open_loop() {
    let mut intent = Intent::new(continuous(60), START_MS);

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
