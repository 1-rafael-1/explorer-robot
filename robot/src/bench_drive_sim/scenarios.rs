//! Rotation scenarios that mirror the `drive-core` host worked examples.
//!
//! Each scenario runs the compiled model through [`run_sim`], which substitutes
//! a synthetic clock and observation feed for the adapter's sensors. The
//! expected actions and terminal telemetry are the literal worked examples from
//! `robot/crates/models/drive-core/tests/rotation.rs`, never a re-run of the
//! model's arithmetic.

use drive_core::{
    CompletionTelemetry, DriveFault, DriveOutcome, Intent, IntentAction, IntentParams, Observation, RotationDirection,
    RotationMotion, YawSample,
};
use heapless::Vec;

/// Clockwise rotation.
const CW: RotationDirection = RotationDirection::Clockwise;
/// Counter-clockwise rotation.
const CCW: RotationDirection = RotationDirection::CounterClockwise;
/// A stationary turn at 60 % rotation speed.
const SPEED_60: RotationMotion = RotationMotion::Stationary { speed: 60 };
/// A stationary turn at 70 % rotation speed.
const SPEED_70: RotationMotion = RotationMotion::Stationary { speed: 70 };
/// A forward turn while moving at 50 %.
const FORWARD: RotationMotion = RotationMotion::WhileMoving(50);
/// A reverse turn while moving at −50 %.
const REVERSE: RotationMotion = RotationMotion::WhileMoving(-50);
/// The synthetic start time shared by every scenario.
const START_MS: u64 = 1_000;
/// How many leading actions each run keeps for sequence assertions.
const ACTION_PREFIX: usize = 32;
/// Upper bound on emitted actions before a scenario is declared stuck.
const ACTION_GUARD: u32 = 20_000;

/// One named simulation scenario.
pub(super) struct Scenario {
    /// The name reported in the `drive-sim:` line.
    pub(super) name: &'static str,
    /// The scenario body; `Err` carries the failure detail.
    pub(super) run: fn() -> Result<(), &'static str>,
}

/// The scenarios run in order, each mirroring a host worked example.
pub(super) const SCENARIOS: &[Scenario] = &[
    Scenario {
        name: "rotate_stationary_cw",
        run: rotate_stationary_cw,
    },
    Scenario {
        name: "rotate_stationary_ccw",
        run: rotate_stationary_ccw,
    },
    Scenario {
        name: "rotate_while_moving_forward",
        run: rotate_while_moving_forward,
    },
    Scenario {
        name: "rotate_while_moving_reverse",
        run: rotate_while_moving_reverse,
    },
    Scenario {
        name: "rotate_first_sample_anchoring",
        run: rotate_first_sample_anchoring,
    },
    Scenario {
        name: "rotate_yaw_wrap",
        run: rotate_yaw_wrap,
    },
    Scenario {
        name: "rotate_ramp_down",
        run: rotate_ramp_down,
    },
    Scenario {
        name: "rotate_tolerance_completion",
        run: rotate_tolerance_completion,
    },
    Scenario {
        name: "rotate_overshoot_reverse",
        run: rotate_overshoot_reverse,
    },
    Scenario {
        name: "rotate_flip_limit",
        run: rotate_flip_limit,
    },
    Scenario {
        name: "rotate_timeout",
        run: rotate_timeout,
    },
    Scenario {
        name: "rotate_no_sample_wait",
        run: rotate_no_sample_wait,
    },
];

// ── Simulation harness ────────────────────────────────────────────────────────

/// The summary of one simulated run.
struct RunResult {
    /// The first [`ACTION_PREFIX`] actions, for sequence assertions.
    actions: Vec<IntentAction, ACTION_PREFIX>,
    /// The total number of actions, including any beyond the prefix.
    count: usize,
    /// The last two actions, so long runs can still check their teardown.
    last_two: [Option<IntentAction>; 2],
    /// The synthetic clock when `Finished` arrived.
    end_ms: u64,
    /// The terminal model outcome.
    outcome: DriveOutcome,
}

/// Run one intent with a scripted observation feed and a recording executor.
///
/// `SetTracks`, `BrakeAll` and `CoastAll` are recorded and execution continues
/// immediately. `WaitUntil` advances the synthetic clock to its deadline; no
/// real sleep happens. `Finished` ends the scenario.
fn run_sim(
    params: IntentParams,
    start_ms: u64,
    mut sample_at: impl FnMut(u64) -> Option<YawSample>,
) -> Result<RunResult, &'static str> {
    let mut intent = Intent::new(params, start_ms);
    let mut now_ms = start_ms;
    // The start step must see an empty observation, exactly like the host
    // `start` helper; samples begin at the first control tick.
    let mut observation = Observation {
        now_ms,
        encoder: None,
        yaw: None,
    };
    let mut actions = Vec::new();
    let mut count = 0_usize;
    let mut last_two = [None, None];
    let mut guard = 0_u32;

    loop {
        let step = intent.step(&observation);
        if actions.len() < ACTION_PREFIX {
            actions.push(step.action).map_err(|_| "action prefix overflow")?;
        }
        count += 1;
        last_two[0] = last_two[1];
        last_two[1] = Some(step.action);

        match step.action {
            IntentAction::SetTracks { .. } | IntentAction::BrakeAll | IntentAction::CoastAll => {}
            IntentAction::WaitUntil { deadline_ms } => {
                if deadline_ms < now_ms {
                    return Err("wait deadline moved backwards");
                }
                now_ms = deadline_ms;
                observation = Observation {
                    now_ms,
                    encoder: None,
                    yaw: sample_at(now_ms),
                };
            }
            IntentAction::Finished => {
                let Some(outcome) = intent.outcome() else {
                    return Err("finished without an outcome");
                };
                return Ok(RunResult {
                    actions,
                    count,
                    last_two,
                    end_ms: now_ms,
                    outcome,
                });
            }
        }

        guard = guard.saturating_add(1);
        if guard > ACTION_GUARD {
            return Err("intent did not finish");
        }
    }
}

// ── Assertion helpers ─────────────────────────────────────────────────────────

/// A `SetTracks` action.
const fn set_tracks(left: i8, right: i8) -> IntentAction {
    IntentAction::SetTracks { left, right }
}

/// A `WaitUntil` action.
const fn wait(deadline_ms: u64) -> IntentAction {
    IntentAction::WaitUntil { deadline_ms }
}

/// The terminal `Finished` action.
const FINISHED: IntentAction = IntentAction::Finished;

/// Rotation parameters for a scenario.
const fn rotate(degrees: f32, direction: RotationDirection, motion: RotationMotion) -> IntentParams {
    IntentParams::RotateExact {
        degrees,
        direction,
        motion,
    }
}

/// A yaw sample stamped at the observation time.
const fn sample(now_ms: u64, yaw_deg: f32) -> YawSample {
    YawSample {
        yaw_deg,
        timestamp_ms: now_ms,
    }
}

/// A step table: the yaw holds at the last entry at or before `now_ms`.
fn table_sample(now_ms: u64, entries: &[(u64, f32)]) -> Option<YawSample> {
    let mut yaw_deg = None;
    for &(time_ms, value) in entries {
        if time_ms <= now_ms {
            yaw_deg = Some(value);
        } else {
            break;
        }
    }
    yaw_deg.map(|value| YawSample {
        yaw_deg: value,
        timestamp_ms: now_ms,
    })
}

/// Assert the run emitted exactly `expected` actions.
fn expect_actions(run: &RunResult, expected: &[IntentAction]) -> Result<(), &'static str> {
    if run.count != expected.len() {
        return Err("unexpected action count");
    }
    expect_prefix(run, expected)
}

/// Assert the run starts with `expected` actions.
fn expect_prefix(run: &RunResult, expected: &[IntentAction]) -> Result<(), &'static str> {
    for (index, expected_action) in expected.iter().enumerate() {
        match run.actions.get(index) {
            Some(action) if action == expected_action => {}
            _ => return Err("unexpected action"),
        }
    }
    Ok(())
}

/// Assert the last two actions are a zero track command and `Finished`.
const fn expect_teardown(run: &RunResult) -> Result<(), &'static str> {
    match run.last_two {
        [
            Some(IntentAction::SetTracks { left: 0, right: 0 }),
            Some(IntentAction::Finished),
        ] => Ok(()),
        _ => Err("unexpected teardown"),
    }
}

/// Assert the run completed with the worked-example telemetry.
fn expect_completed(
    run: &RunResult,
    final_yaw_deg: f32,
    angle_error_deg: f32,
    duration_ms: u64,
) -> Result<(), &'static str> {
    match &run.outcome {
        DriveOutcome::Completed { telemetry } => {
            expect_rotate_telemetry(telemetry, final_yaw_deg, angle_error_deg, duration_ms)
        }
        DriveOutcome::Failed { .. } => Err("expected Completed"),
    }
}

/// Assert the run failed with `fault` and the worked-example telemetry.
fn expect_failed(
    run: &RunResult,
    fault: DriveFault,
    final_yaw_deg: f32,
    angle_error_deg: f32,
    duration_ms: u64,
) -> Result<(), &'static str> {
    match &run.outcome {
        DriveOutcome::Failed {
            fault: actual,
            telemetry,
        } => {
            if *actual != fault {
                return Err("unexpected fault");
            }
            expect_rotate_telemetry(telemetry, final_yaw_deg, angle_error_deg, duration_ms)
        }
        DriveOutcome::Completed { .. } => Err("expected Failed"),
    }
}

/// Assert rotate telemetry matches the worked example.
fn expect_rotate_telemetry(
    telemetry: &CompletionTelemetry,
    final_yaw_deg: f32,
    angle_error_deg: f32,
    duration_ms: u64,
) -> Result<(), &'static str> {
    match telemetry {
        CompletionTelemetry::RotateExact {
            final_yaw_deg: yaw,
            angle_error_deg: error,
            duration_ms: duration,
        } => {
            if !close(*yaw, final_yaw_deg) {
                return Err("unexpected final yaw");
            }
            if !close(*error, angle_error_deg) {
                return Err("unexpected angle error");
            }
            if *duration != duration_ms {
                return Err("unexpected duration");
            }
            Ok(())
        }
        CompletionTelemetry::None | CompletionTelemetry::DriveDistance { .. } => Err("expected RotateExact telemetry"),
    }
}

/// Float equality for worked-example values.
fn close(actual: f32, expected: f32) -> bool {
    (actual - expected).abs() < 1e-3
}

// ── Rotation scenarios ────────────────────────────────────────────────────────

/// Clockwise stationary turn: the ramp law and a clean completion.
fn rotate_stationary_cw() -> Result<(), &'static str> {
    const TABLE: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -30.0),
        (1_060, -45.0),
        (1_080, -75.0),
        (1_100, -90.0),
    ];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| table_sample(now, TABLE))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(60, -60),
            wait(1_040),
            set_tracks(60, -60),
            wait(1_060),
            set_tracks(56, -56),
            wait(1_080),
            set_tracks(48, -48),
            wait(1_100),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 100)
}

/// Counter-clockwise stationary turn: the mirrored ramp law.
fn rotate_stationary_ccw() -> Result<(), &'static str> {
    const TABLE: &[(u64, f32)] = &[(1_020, 0.0), (1_040, 30.0), (1_060, 45.0), (1_080, 75.0), (1_100, 90.0)];
    let run = run_sim(rotate(90.0, CCW, SPEED_60), START_MS, |now| table_sample(now, TABLE))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(-60, 60),
            wait(1_040),
            set_tracks(-60, 60),
            wait(1_060),
            set_tracks(-56, 56),
            wait(1_080),
            set_tracks(-48, 48),
            wait(1_100),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, 90.0, 0.0, 100)
}

/// Forward while-moving turn: the differential stays on the inner track.
fn rotate_while_moving_forward() -> Result<(), &'static str> {
    const CW_TABLE: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -30.0),
        (1_060, -70.0),
        (1_080, -89.0),
        (1_100, -90.0),
    ];
    const CCW_TABLE: &[(u64, f32)] = &[(1_020, 0.0), (1_040, 30.0), (1_060, 70.0), (1_080, 89.0), (1_100, 90.0)];
    let cw = run_sim(rotate(90.0, CW, FORWARD), START_MS, |now| table_sample(now, CW_TABLE))?;
    expect_actions(
        &cw,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(50, 20),
            wait(1_040),
            set_tracks(50, 20),
            wait(1_060),
            set_tracks(50, 20),
            wait(1_080),
            set_tracks(50, 20),
            wait(1_100),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&cw, -90.0, 0.0, 100)?;

    let ccw = run_sim(rotate(90.0, CCW, FORWARD), START_MS, |now| table_sample(now, CCW_TABLE))?;
    expect_actions(
        &ccw,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(20, 50),
            wait(1_040),
            set_tracks(20, 50),
            wait(1_060),
            set_tracks(20, 50),
            wait(1_080),
            set_tracks(20, 50),
            wait(1_100),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&ccw, 90.0, 0.0, 100)
}

/// Reverse while-moving turn: the differential stays on the inner track.
fn rotate_while_moving_reverse() -> Result<(), &'static str> {
    const CW_TABLE: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -30.0),
        (1_060, -70.0),
        (1_080, -89.0),
        (1_100, -90.0),
    ];
    const CCW_TABLE: &[(u64, f32)] = &[(1_020, 0.0), (1_040, 30.0), (1_060, 70.0), (1_080, 89.0), (1_100, 90.0)];
    let cw = run_sim(rotate(90.0, CW, REVERSE), START_MS, |now| table_sample(now, CW_TABLE))?;
    expect_actions(
        &cw,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(-50, -80),
            wait(1_040),
            set_tracks(-50, -80),
            wait(1_060),
            set_tracks(-50, -80),
            wait(1_080),
            set_tracks(-50, -80),
            wait(1_100),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&cw, -90.0, 0.0, 100)?;

    let ccw = run_sim(rotate(90.0, CCW, REVERSE), START_MS, |now| table_sample(now, CCW_TABLE))?;
    expect_actions(
        &ccw,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(-80, -50),
            wait(1_040),
            set_tracks(-80, -50),
            wait(1_060),
            set_tracks(-80, -50),
            wait(1_080),
            set_tracks(-80, -50),
            wait(1_100),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&ccw, 90.0, 0.0, 100)
}

/// The first sample anchors an absolute target instead of a zero-relative one.
fn rotate_first_sample_anchoring() -> Result<(), &'static str> {
    // Clockwise 20° from 170°: target yaw 150°.
    const CW_TABLE: &[(u64, f32)] = &[(1_020, 170.0), (1_040, 160.0), (1_060, 150.0)];
    // Counter-clockwise 20° from 170°: target yaw wraps to −170°.
    const CCW_TABLE: &[(u64, f32)] = &[(1_020, 170.0), (1_040, 180.0), (1_060, -170.0)];
    let cw = run_sim(rotate(20.0, CW, SPEED_60), START_MS, |now| table_sample(now, CW_TABLE))?;
    expect_actions(
        &cw,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(50, -50),
            wait(1_040),
            set_tracks(47, -47),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&cw, 150.0, 0.0, 60)?;

    let ccw = run_sim(rotate(20.0, CCW, SPEED_60), START_MS, |now| {
        table_sample(now, CCW_TABLE)
    })?;
    expect_actions(
        &ccw,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(-50, 50),
            wait(1_040),
            set_tracks(-47, 47),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&ccw, -170.0, 0.0, 60)
}

/// The remaining error wraps across the ±180° seam.
fn rotate_yaw_wrap() -> Result<(), &'static str> {
    // Clockwise 20° from −170°: target yaw wraps to +170°.
    const TABLE: &[(u64, f32)] = &[(1_020, -170.0), (1_040, -180.0), (1_060, 170.0)];
    let run = run_sim(rotate(20.0, CW, SPEED_60), START_MS, |now| table_sample(now, TABLE))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(50, -50),
            wait(1_040),
            set_tracks(47, -47),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, 170.0, 0.0, 60)
}

/// Ramp-down above, exactly at and below the 60° threshold.
fn rotate_ramp_down() -> Result<(), &'static str> {
    const TABLE: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -29.0),
        (1_060, -30.0),
        (1_080, -45.0),
        (1_100, -60.0),
        (1_120, -85.0),
        (1_140, -90.0),
    ];
    let run = run_sim(rotate(90.0, CW, SPEED_70), START_MS, |now| table_sample(now, TABLE))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(70, -70),
            wait(1_040),
            set_tracks(70, -70),
            wait(1_060),
            set_tracks(70, -70),
            wait(1_080),
            set_tracks(63, -63),
            wait(1_100),
            set_tracks(57, -57),
            wait(1_120),
            set_tracks(47, -47),
            wait(1_140),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 140)
}

/// A sample inside the tolerance completes the turn.
fn rotate_tolerance_completion() -> Result<(), &'static str> {
    const TABLE: &[(u64, f32)] = &[(1_020, 0.0), (1_040, -90.5)];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| table_sample(now, TABLE))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(60, -60),
            wait(1_040),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.5, 0.5, 40)
}

/// An overshoot reverses the effective rotation direction.
fn rotate_overshoot_reverse() -> Result<(), &'static str> {
    const TABLE: &[(u64, f32)] = &[(1_020, 0.0), (1_040, -95.0), (1_060, -90.0)];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| table_sample(now, TABLE))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(60, -60),
            wait(1_040),
            set_tracks(-46, 46),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 60)
}

/// The turn fails after the twelfth corrective direction flip.
fn rotate_flip_limit() -> Result<(), &'static str> {
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| {
        if now == 1_020 {
            return Some(sample(now, 0.0));
        }
        if (1_040..=1_260).contains(&now) {
            let step = (now - 1_020) / 20;
            let yaw_deg = if step % 2 == 1 { -95.0 } else { 0.0 };
            return Some(sample(now, yaw_deg));
        }
        None
    })?;

    if run.end_ms != 1_260 {
        return Err("unexpected flip-limit end time");
    }
    expect_teardown(&run)?;
    expect_failed(&run, DriveFault::CorrectionLimit, 0.0, -90.0, 260)
}

/// The turn fails when the overall rotation timeout expires with samples present.
fn rotate_timeout() -> Result<(), &'static str> {
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| Some(sample(now, 0.0)))?;

    if run.end_ms != 9_020 {
        return Err("unexpected timeout end time");
    }
    expect_teardown(&run)?;
    expect_failed(&run, DriveFault::RotationTimeout, 0.0, -90.0, 8_020)
}

/// No fresh sample: the wait cadence, the stale-sample filter and the timeout.
fn rotate_no_sample_wait() -> Result<(), &'static str> {
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| {
        if now == 1_080 {
            Some(YawSample {
                yaw_deg: 0.0,
                timestamp_ms: 999,
            })
        } else {
            None
        }
    })?;

    expect_prefix(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            wait(1_040),
            wait(1_060),
            wait(1_080),
            wait(1_100),
        ],
    )?;
    expect_teardown(&run)?;
    expect_failed(&run, DriveFault::RotationTimeout, 0.0, -90.0, 8_020)
}
