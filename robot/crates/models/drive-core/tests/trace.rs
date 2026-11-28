//! Host tests for the feature-gated `Step::trace` decision snapshot.
//!
//! These run only with the `trace` feature on. The `std`-only rotation tests
//! assert the other half of the contract: `Step::trace` is `None`.

use drive_core::{
    Intent, IntentAction, IntentParams, Observation, RotationDirection, RotationMotion, Trace, YawSample,
};

/// An observation carrying only the clock and an optional yaw sample.
const fn observation(now_ms: u64, yaw: Option<YawSample>) -> Observation {
    Observation {
        now_ms,
        encoder: None,
        yaw,
    }
}

/// The snapshot carried by `step`, or a failed assertion.
fn trace_of(step: drive_core::Step) -> Trace {
    match step.trace {
        Some(trace) => trace,
        None => {
            assert!(step.trace.is_some(), "the trace feature is on, so trace must be Some");
            Trace {
                yaw_deg: None,
                target_yaw_deg: None,
                remaining_deg: 0.0,
                left: None,
                right: None,
                deadline_ms: None,
            }
        }
    }
}

#[test]
fn a_wait_step_traces_its_deadline_and_no_speeds() {
    let mut intent = Intent::new(
        IntentParams::RotateExact {
            degrees: 90.0,
            direction: RotationDirection::Clockwise,
            motion: RotationMotion::Stationary { speed: 60 },
        },
        1_000,
    );

    let start = intent.step(&observation(1_000, None));
    assert_eq!(start.action, IntentAction::SetTracks { left: 0, right: 0 });
    let start_trace = trace_of(start);
    assert_eq!(start_trace.yaw_deg, None);
    assert_eq!(start_trace.target_yaw_deg, None);
    assert_eq!(start_trace.left, Some(0));
    assert_eq!(start_trace.right, Some(0));
    assert_eq!(start_trace.deadline_ms, None);

    let wait = trace_of(intent.step(&observation(1_000, None)));
    assert_eq!(wait.deadline_ms, Some(1_020));
    assert_eq!(wait.left, None);
    assert_eq!(wait.right, None);
}

#[test]
fn a_set_tracks_step_traces_the_anchored_decision() {
    let mut intent = Intent::new(
        IntentParams::RotateExact {
            degrees: 90.0,
            direction: RotationDirection::Clockwise,
            motion: RotationMotion::Stationary { speed: 60 },
        },
        1_000,
    );
    let _ = intent.step(&observation(1_000, None));
    let _ = intent.step(&observation(1_000, None));

    let step = intent.step(&observation(
        1_020,
        Some(YawSample {
            yaw_deg: 0.0,
            timestamp_ms: 1_020,
        }),
    ));
    assert_eq!(step.action, IntentAction::SetTracks { left: 60, right: -60 });

    let trace = trace_of(step);
    assert_eq!(trace.yaw_deg, Some(0.0));
    assert_eq!(trace.target_yaw_deg, Some(-90.0));
    assert!((trace.remaining_deg - 90.0).abs() < 1e-3);
    assert_eq!(trace.left, Some(60));
    assert_eq!(trace.right, Some(-60));
    assert_eq!(trace.deadline_ms, None);
}
