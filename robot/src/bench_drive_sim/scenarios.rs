//! Drive-intent scenarios that mirror the `drive-core` host worked examples.
//!
//! Each scenario runs the compiled model through [`run_sim`], which substitutes
//! a synthetic clock and observation feed for the adapter's sensors. The
//! expected actions and terminal telemetry are the literal worked examples from
//! the matching `robot/crates/models/drive-core/tests/*.rs` files, never a
//! re-run of the model's arithmetic.

use drive_core::{
    CompletionTelemetry, DistanceScale, DriveDirection, DriveDistanceKind, DriveFault, DriveOutcome, EncoderSample,
    Intent, IntentAction, IntentParams, Observation, RotationDirection, RotationMotion, TurnDirection, YawSample,
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
/// Forward distance motion.
const DRIVE_FORWARD: DriveDirection = DriveDirection::Forward;
/// Backward distance motion.
const DRIVE_BACKWARD: DriveDirection = DriveDirection::Backward;
/// The synthetic start time shared by every scenario.
const START_MS: u64 = 1_000;
/// How many leading actions each run keeps for sequence assertions.
const ACTION_PREFIX: usize = 32;
/// How many non-zero track commands each run keeps for torque-ladder assertions.
const PULSE_LOG: usize = 32;
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
    Scenario {
        name: "rotate_correction_mid_pulse",
        run: rotate_correction_mid_pulse,
    },
    Scenario {
        name: "rotate_correction_after_settle",
        run: rotate_correction_after_settle,
    },
    Scenario {
        name: "rotate_correction_ladder",
        run: rotate_correction_ladder,
    },
    Scenario {
        name: "rotate_correction_sign_flip",
        run: rotate_correction_sign_flip,
    },
    Scenario {
        name: "rotate_correction_exhausted",
        run: rotate_correction_exhausted,
    },
    Scenario {
        name: "distance_straight_forward",
        run: distance_straight_forward,
    },
    Scenario {
        name: "distance_straight_reverse",
        run: distance_straight_reverse,
    },
    Scenario {
        name: "distance_short_early_finish",
        run: distance_short_early_finish,
    },
    Scenario {
        name: "distance_ramp_down",
        run: distance_ramp_down,
    },
    Scenario {
        name: "distance_tolerance_completion",
        run: distance_tolerance_completion,
    },
    Scenario {
        name: "distance_wrap",
        run: distance_wrap,
    },
    Scenario {
        name: "distance_stale_sample",
        run: distance_stale_sample,
    },
    Scenario {
        name: "distance_encoder_timeout",
        run: distance_encoder_timeout,
    },
    Scenario {
        name: "distance_stall",
        run: distance_stall,
    },
    Scenario {
        name: "distance_encoder_anomaly",
        run: distance_encoder_anomaly,
    },
    Scenario {
        name: "distance_curve_left",
        run: distance_curve_left,
    },
    Scenario {
        name: "distance_curve_right",
        run: distance_curve_right,
    },
    Scenario {
        name: "distance_heading_correction",
        run: distance_heading_correction,
    },
    Scenario {
        name: "distance_no_imu",
        run: distance_no_imu,
    },
    Scenario {
        name: "continuous_hold",
        run: continuous_hold,
    },
    Scenario {
        name: "continuous_stall",
        run: continuous_stall,
    },
    Scenario {
        name: "continuous_encoder_timeout",
        run: continuous_encoder_timeout,
    },
    Scenario {
        name: "continuous_anomaly",
        run: continuous_anomaly,
    },
    Scenario {
        name: "brake_settle",
        run: brake_settle,
    },
    Scenario {
        name: "coast_non_zero_reset",
        run: coast_non_zero_reset,
    },
    Scenario {
        name: "brake_wrap",
        run: brake_wrap,
    },
    Scenario {
        name: "brake_timeout",
        run: brake_timeout,
    },
    Scenario {
        name: "idle_zero",
        run: idle_zero,
    },
    Scenario {
        name: "idle_duration",
        run: idle_duration,
    },
];

// ── Simulation harness ────────────────────────────────────────────────────────

/// The summary of one simulated run.
struct RunResult {
    /// The first [`ACTION_PREFIX`] actions, for sequence assertions.
    actions: Vec<IntentAction, ACTION_PREFIX>,
    /// Every non-zero `SetTracks`, in order: the turn ramps and correction pulses.
    pulses: Vec<(i8, i8), PULSE_LOG>,
    /// The total number of actions, including any beyond the prefix.
    count: usize,
    /// The last two actions, so long runs can still check their teardown.
    last_two: [Option<IntentAction>; 2],
    /// The synthetic clock when `Finished` arrived or the run was bounded.
    end_ms: u64,
    /// The terminal model outcome, absent for a bounded run that never finished.
    outcome: Option<DriveOutcome>,
}

/// Run one rotation intent with a scripted yaw feed and a recording executor.
///
/// The start step sees an empty observation, exactly like the rotation host
/// `start` helper.
fn run_sim(
    params: IntentParams,
    start_ms: u64,
    mut sample_at: impl FnMut(u64) -> Option<YawSample>,
) -> Result<RunResult, &'static str> {
    let initial = Observation {
        now_ms: start_ms,
        encoder: None,
        yaw: None,
    };
    run_sim_inner(
        params,
        start_ms,
        initial,
        move |now_ms| Observation {
            now_ms,
            encoder: None,
            yaw: sample_at(now_ms),
        },
        None,
    )
}

/// Run one distance or continuous intent with scripted encoder and yaw feeds.
///
/// The start observation is the feed's sample at `start_ms`, exactly like the
/// distance host `start` helper.
fn run_drive_sim(
    params: IntentParams,
    start_ms: u64,
    observe: impl FnMut(u64) -> (Option<EncoderSample>, Option<YawSample>),
) -> Result<RunResult, &'static str> {
    run_drive_sim_limited(params, start_ms, None, observe)
}

/// Run a distance or continuous intent, optionally stopping after `action_limit`
/// actions.
///
/// A bounded run that never reaches `Finished` reports `outcome: None`; it is how
/// the bench checks a continuous drive that correctly never completes.
fn run_drive_sim_limited(
    params: IntentParams,
    start_ms: u64,
    action_limit: Option<usize>,
    mut observe: impl FnMut(u64) -> (Option<EncoderSample>, Option<YawSample>),
) -> Result<RunResult, &'static str> {
    let (encoder, yaw) = observe(start_ms);
    let initial = Observation {
        now_ms: start_ms,
        encoder,
        yaw,
    };
    run_sim_inner(
        params,
        start_ms,
        initial,
        move |now_ms| {
            let (encoder, yaw) = observe(now_ms);
            Observation { now_ms, encoder, yaw }
        },
        action_limit,
    )
}

/// Run one intent with a scripted observation feed and a recording executor.
///
/// `SetTracks`, `BrakeAll` and `CoastAll` are recorded and execution continues
/// immediately. `WaitUntil` advances the synthetic clock to its deadline; no
/// real sleep happens. `Finished` ends the scenario.
fn run_sim_inner(
    params: IntentParams,
    start_ms: u64,
    mut observation: Observation,
    mut observe: impl FnMut(u64) -> Observation,
    action_limit: Option<usize>,
) -> Result<RunResult, &'static str> {
    let mut intent = Intent::new(params, start_ms);
    let mut actions = Vec::new();
    let mut pulses = Vec::new();
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
            IntentAction::SetTracks { left, right } => {
                if left != 0 || right != 0 {
                    let _ = pulses.push((left, right));
                }
            }
            IntentAction::BrakeAll | IntentAction::CoastAll => {}
            IntentAction::WaitUntil { deadline_ms } => {
                if deadline_ms < observation.now_ms {
                    return Err("wait deadline moved backwards");
                }
                observation = observe(deadline_ms);
            }
            IntentAction::Finished => {
                let Some(outcome) = intent.outcome() else {
                    return Err("finished without an outcome");
                };
                return Ok(RunResult {
                    actions,
                    pulses,
                    count,
                    last_two,
                    end_ms: observation.now_ms,
                    outcome: Some(outcome),
                });
            }
        }

        if let Some(limit) = action_limit
            && count >= limit
        {
            return Ok(RunResult {
                actions,
                pulses,
                count,
                last_two,
                end_ms: observation.now_ms,
                outcome: intent.outcome(),
            });
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

/// An absolute encoder sample stamped at the observation time.
///
/// The `Option` mirrors the host tests' `enc` helper: a feed composes this with
/// `None` for ticks without a sample.
#[allow(clippy::unnecessary_wraps)]
const fn encoder(left: u16, right: u16, timestamp_ms: u64) -> Option<EncoderSample> {
    Some(EncoderSample {
        left,
        right,
        timestamp_ms,
    })
}

/// The ten-counts-per-centimetre scale the distance host tests use.
fn scale() -> Result<DistanceScale, &'static str> {
    DistanceScale::from_counts_per_cm(10.0).map_err(|_| "10 counts/cm outside the plausibility band")
}

/// A straight distance parameter set.
fn straight(distance_cm: f32, direction: DriveDirection, speed: u8) -> Result<IntentParams, &'static str> {
    Ok(IntentParams::DriveDistance {
        kind: DriveDistanceKind::Straight { distance_cm },
        direction,
        speed,
        scale: scale()?,
    })
}

/// A curve-arc parameter set.
fn curve(
    radius_cm: f32,
    arc_length_cm: f32,
    turn: TurnDirection,
    direction: DriveDirection,
    speed: u8,
) -> Result<IntentParams, &'static str> {
    Ok(IntentParams::DriveDistance {
        kind: DriveDistanceKind::CurveArc {
            radius_cm,
            arc_length_cm,
            direction: turn,
        },
        direction,
        speed,
        scale: scale()?,
    })
}

/// A continuous drive parameter set.
const fn continuous(speed: u8) -> IntentParams {
    IntentParams::DriveContinuous {
        direction: DRIVE_FORWARD,
        speed,
    }
}

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

/// A sample that exists only at exactly `time_ms`.
///
/// The correction scenarios use this so the poll cadence and the settle
/// deadlines are explicit, mirroring the host worked examples.
fn exact_sample(entries: &[(u64, f32)], now_ms: u64) -> Option<YawSample> {
    entries
        .iter()
        .find(|(timestamp_ms, _)| *timestamp_ms == now_ms)
        .map(|&(timestamp_ms, yaw_deg)| YawSample { yaw_deg, timestamp_ms })
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

/// Assert the run's non-zero track commands are exactly `expected`.
fn expect_pulses(run: &RunResult, expected: &[(i8, i8)]) -> Result<(), &'static str> {
    if run.pulses.len() != expected.len() {
        return Err("unexpected pulse count");
    }
    for (actual, &want) in run.pulses.iter().zip(expected) {
        if *actual != want {
            return Err("unexpected pulse");
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
    match run.outcome.as_ref() {
        Some(DriveOutcome::Completed { telemetry }) => {
            expect_rotate_telemetry(telemetry, final_yaw_deg, angle_error_deg, duration_ms)
        }
        _ => Err("expected Completed"),
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
    match run.outcome.as_ref() {
        Some(DriveOutcome::Failed {
            fault: actual,
            telemetry,
        }) => {
            if *actual != fault {
                return Err("unexpected fault");
            }
            expect_rotate_telemetry(telemetry, final_yaw_deg, angle_error_deg, duration_ms)
        }
        _ => Err("expected Failed"),
    }
}

/// Assert a distance run completed with the worked-example telemetry.
fn expect_distance_completed(
    run: &RunResult,
    achieved: (f32, f32),
    counts: (u32, u32),
    targets: (f32, f32),
    duration_ms: u64,
) -> Result<(), &'static str> {
    match run.outcome.as_ref() {
        Some(DriveOutcome::Completed { telemetry }) => {
            expect_distance_telemetry(telemetry, achieved, counts, targets, duration_ms)
        }
        _ => Err("expected Completed"),
    }
}

/// Assert a distance run failed with `fault` and the worked-example telemetry.
fn expect_distance_failed(
    run: &RunResult,
    fault: DriveFault,
    achieved: (f32, f32),
    counts: (u32, u32),
    targets: (f32, f32),
    duration_ms: u64,
) -> Result<(), &'static str> {
    match run.outcome.as_ref() {
        Some(DriveOutcome::Failed {
            fault: actual,
            telemetry,
        }) => {
            if *actual != fault {
                return Err("unexpected fault");
            }
            expect_distance_telemetry(telemetry, achieved, counts, targets, duration_ms)
        }
        _ => Err("expected Failed"),
    }
}

/// Assert a continuous run failed with `fault` and no telemetry.
fn expect_failed_without_telemetry(run: &RunResult, fault: DriveFault) -> Result<(), &'static str> {
    match run.outcome.as_ref() {
        Some(DriveOutcome::Failed {
            fault: actual,
            telemetry,
        }) if *actual == fault && *telemetry == CompletionTelemetry::None => Ok(()),
        _ => Err("unexpected failure"),
    }
}

/// Assert a run completed with no telemetry.
fn expect_completed_without_telemetry(run: &RunResult) -> Result<(), &'static str> {
    match run.outcome.as_ref() {
        Some(DriveOutcome::Completed { telemetry }) if *telemetry == CompletionTelemetry::None => Ok(()),
        _ => Err("unexpected completion"),
    }
}

/// Assert distance telemetry matches the worked example.
fn expect_distance_telemetry(
    telemetry: &CompletionTelemetry,
    achieved: (f32, f32),
    counts: (u32, u32),
    targets: (f32, f32),
    duration_ms: u64,
) -> Result<(), &'static str> {
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
            if !close(*achieved_left_revs, achieved.0) || !close(*achieved_right_revs, achieved.1) {
                return Err("unexpected achieved revolutions");
            }
            if (*achieved_left_counts, *achieved_right_counts) != counts {
                return Err("unexpected achieved counts");
            }
            if !close(*target_left_revs, targets.0) || !close(*target_right_revs, targets.1) {
                return Err("unexpected target revolutions");
            }
            if *actual_duration != duration_ms {
                return Err("unexpected duration");
            }
            Ok(())
        }
        _ => Err("expected DriveDistance telemetry"),
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
            wait(1_500),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 500)
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
            wait(1_500),
            FINISHED,
        ],
    )?;
    expect_completed(&run, 90.0, 0.0, 500)
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
            wait(1_460),
            FINISHED,
        ],
    )?;
    expect_completed(&cw, 150.0, 0.0, 460)?;

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
            wait(1_460),
            FINISHED,
        ],
    )?;
    expect_completed(&ccw, -170.0, 0.0, 460)
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
            wait(1_460),
            FINISHED,
        ],
    )?;
    expect_completed(&run, 170.0, 0.0, 460)
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
            wait(1_540),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 540)
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
            wait(1_440),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.5, 0.5, 440)
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
            wait(1_460),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 460)
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

// ── Correction scenarios ──────────────────────────────────────────────────────

/// The first correction pulse converges mid-pulse and stops.
fn rotate_correction_mid_pulse() -> Result<(), &'static str> {
    // The turn stops at 1 040. The settle at 1 440 leaves 10° of error; the
    // first pulse runs until the 1 460 sample lands inside tolerance.
    const SAMPLES: &[(u64, f32)] = &[(1_020, 0.0), (1_040, -90.0), (1_440, -80.0), (1_460, -90.0)];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| exact_sample(SAMPLES, now))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(60, -60),
            wait(1_040),
            set_tracks(0, 0),
            wait(1_440),
            set_tracks(35, -35),
            wait(1_445),
            wait(1_450),
            wait(1_455),
            wait(1_460),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.0, 0.0, 460)
}

/// The post-pulse settle converges without a second stop command.
fn rotate_correction_after_settle() -> Result<(), &'static str> {
    // The pulse overshoots at 1 450 and stops; the settle at 1 850 lands back
    // inside tolerance.
    const SAMPLES: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -90.0),
        (1_440, -80.0),
        (1_450, -90.6),
        (1_850, -90.2),
    ];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| exact_sample(SAMPLES, now))?;

    expect_actions(
        &run,
        &[
            set_tracks(0, 0),
            wait(1_020),
            set_tracks(60, -60),
            wait(1_040),
            set_tracks(0, 0),
            wait(1_440),
            set_tracks(35, -35),
            wait(1_445),
            wait(1_450),
            set_tracks(0, 0),
            wait(1_850),
            FINISHED,
        ],
    )?;
    expect_completed(&run, -90.2, 0.2, 850)
}

/// The torque ladder steps down while each pulse times out.
fn rotate_correction_ladder() -> Result<(), &'static str> {
    // Each pulse times out at `S + 405`; its settle finds the same 10° error at
    // `S + 805`, so the next pulse starts there. The fourth pulse converges.
    const SAMPLES: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -90.0),
        (1_440, -80.0),
        (2_245, -80.0),
        (3_050, -80.0),
        (3_855, -80.0),
        (3_860, -90.0),
    ];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| exact_sample(SAMPLES, now))?;

    expect_pulses(&run, &[(60, -60), (35, -35), (33, -33), (31, -31), (29, -29)])?;
    if run.end_ms != 3_860 {
        return Err("unexpected ladder end time");
    }
    expect_completed(&run, -90.0, 0.0, 2_860)
}

/// A sign flip after the settle reverses the next pulse.
fn rotate_correction_sign_flip() -> Result<(), &'static str> {
    // The first pulse runs an undershoot (+10°) and times out. The settle
    // sample at 1 845 is −5°, so the second pulse reverses direction and steps
    // the ladder down to 28.
    const SAMPLES: &[(u64, f32)] = &[
        (1_020, 0.0),
        (1_040, -90.0),
        (1_440, -80.0),
        (1_845, -95.0),
        (1_850, -90.0),
    ];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| exact_sample(SAMPLES, now))?;

    expect_pulses(&run, &[(60, -60), (35, -35), (-28, 28)])?;
    expect_completed(&run, -90.0, 0.0, 850)
}

/// Fifteen failed pulses exhaust the correction.
fn rotate_correction_exhausted() -> Result<(), &'static str> {
    // The error never returns to tolerance: 15 pulses at `S + 805` spacing, and
    // the fifteenth settle exhausts it at 13 515.
    const SAMPLES: &[(u64, f32)] = &[(1_020, 0.0), (1_040, -90.0), (1_440, -80.0)];
    let run = run_sim(rotate(90.0, CW, SPEED_60), START_MS, |now| exact_sample(SAMPLES, now))?;

    expect_pulses(
        &run,
        &[
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
        ],
    )?;
    if run.end_ms != 13_515 {
        return Err("unexpected exhaustion end time");
    }
    expect_failed(&run, DriveFault::CorrectionExhausted, -80.0, -10.0, 12_515)
}

// ── Distance scenarios ────────────────────────────────────────────────────────

/// Straight forward: baseline, two progress ticks and a completed target.
fn distance_straight_forward() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(500, 500, now),
            1_040 => encoder(1_000, 1_000, now),
            1_060 => encoder(1_500, 1_500, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[
            set_tracks(60, 60),
            wait(1_020),
            wait(1_040),
            set_tracks(60, 60),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_distance_completed(&run, (5.2604, 5.2604), (1_000, 1_000), (5.2604, 5.2604), 60)
}

/// Straight reverse: the same progress with negative track commands.
fn distance_straight_reverse() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_BACKWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(500, 500, now),
            1_040 => encoder(1_000, 1_000, now),
            1_060 => encoder(1_500, 1_500, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[
            set_tracks(-60, -60),
            wait(1_020),
            wait(1_040),
            set_tracks(-60, -60),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_distance_completed(&run, (5.2604, 5.2604), (1_000, 1_000), (5.2604, 5.2604), 60)
}

/// A target inside the tolerance finishes on the first step, with no motor command.
fn distance_short_early_finish() -> Result<(), &'static str> {
    let params = straight(0.1, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| (None, Some(sample(now, 0.0))))?;

    expect_actions(&run, &[FINISHED])?;
    expect_distance_completed(&run, (0.0, 0.0), (0, 0), (0.0053, 0.0053), 0)
}

/// The speed ramps down inside the final quarter revolution.
fn distance_ramp_down() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim_limited(params, START_MS, Some(4), |now| {
        let encoder = match now {
            1_020 => encoder(500, 500, now),
            1_040 => encoder(1_460, 1_460, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[set_tracks(60, 60), wait(1_020), wait(1_040), set_tracks(50, 50)],
    )
}

/// One more count inside the tolerance completes the drive.
fn distance_tolerance_completion() -> Result<(), &'static str> {
    let params = straight(19.01, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(288, 288, now),
            1_060 => encoder(289, 289, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[
            set_tracks(60, 60),
            wait(1_020),
            wait(1_040),
            set_tracks(40, 40),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_distance_completed(&run, (0.9942, 0.9942), (189, 189), (1.0, 1.0), 60)
}

/// A 16-bit counter wrap counts the wrapped delta and completes the drive.
fn distance_wrap() -> Result<(), &'static str> {
    let params = straight(10.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(65_500, 65_500, now),
            1_040 => encoder(100, 100, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[set_tracks(60, 60), wait(1_020), wait(1_040), set_tracks(0, 0), FINISHED],
    )?;
    expect_distance_completed(&run, (0.7154, 0.7154), (136, 136), (0.5260, 0.5260), 40)
}

/// A repeated sample timestamp never commands the tracks again.
fn distance_stale_sample() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim_limited(params, START_MS, Some(7), |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(200, 200, now),
            1_060 | 1_080 | 1_100 => encoder(200, 200, 1_040),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[
            set_tracks(60, 60),
            wait(1_020),
            wait(1_040),
            set_tracks(60, 60),
            wait(1_060),
            wait(1_080),
            wait(1_100),
        ],
    )?;
    expect_pulses(&run, &[(60, 60), (60, 60)])
}

/// No fresh encoder sample fails with the encoder-timeout label.
fn distance_encoder_timeout() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(200, 200, now),
            _ if now >= 1_060 => encoder(200, 200, 1_040),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_distance_failed(
        &run,
        DriveFault::EncoderTimeout,
        (0.5260, 0.5260),
        (100, 100),
        (5.2604, 5.2604),
        340,
    )
}

/// Fresh timestamps with unchanged counts fail at the stall timeout.
fn distance_stall() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            _ if now >= 1_040 => encoder(100, 100, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_distance_failed(&run, DriveFault::Stall, (0.0, 0.0), (0, 0), (5.2604, 5.2604), 1_500)
}

/// One moving track while the other stands still fails as an encoder anomaly.
fn distance_encoder_anomaly() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(100, 200, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_distance_failed(
        &run,
        DriveFault::EncoderAnomaly,
        (0.0, 0.0),
        (0, 0),
        (5.2604, 5.2604),
        40,
    )
}

/// A left curve slows the inner track and completes on its progress.
fn distance_curve_left() -> Result<(), &'static str> {
    let params = curve(20.0, 50.0, TurnDirection::Left, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(150, 800, now),
            1_060 => encoder(420, 830, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[
            set_tracks(28, 60),
            wait(1_020),
            wait(1_040),
            set_tracks(13, 60),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_distance_completed(&run, (1.6833, 3.8401), (320, 730), (1.6767, 3.5836), 60)
}

/// A right curve mirrors the left geometry and completes on the right track.
fn distance_curve_right() -> Result<(), &'static str> {
    let params = curve(20.0, 50.0, TurnDirection::Right, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim(params, START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(800, 150, now),
            1_060 => encoder(830, 420, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_actions(
        &run,
        &[
            set_tracks(60, 28),
            wait(1_020),
            wait(1_040),
            set_tracks(60, 13),
            wait(1_060),
            set_tracks(0, 0),
            FINISHED,
        ],
    )?;
    expect_distance_completed(&run, (3.8401, 1.6833), (730, 320), (3.5836, 1.6767), 60)
}

/// A straight drive holds the heading captured at start.
fn distance_heading_correction() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim_limited(params, START_MS, Some(4), |now| {
        let encoder = match now {
            1_020 => encoder(500, 500, now),
            1_040 => encoder(1_000, 1_000, now),
            _ => None,
        };
        let yaw_deg = if now == 1_040 { 15.0 } else { 10.0 };
        (encoder, Some(sample(now, yaw_deg)))
    })?;

    expect_actions(
        &run,
        &[set_tracks(60, 60), wait(1_020), wait(1_040), set_tracks(68, 52)],
    )
}

/// Without a yaw sample the drive waits its 50 ms bound then runs open loop.
fn distance_no_imu() -> Result<(), &'static str> {
    let params = straight(100.0, DRIVE_FORWARD, 60)?;
    let run = run_drive_sim_limited(params, START_MS, Some(9), |now| {
        let encoder = match now {
            1_070 => encoder(500, 500, now),
            1_090 => encoder(1_000, 1_000, now),
            _ => None,
        };
        (encoder, None)
    })?;

    expect_actions(
        &run,
        &[
            wait(1_010),
            wait(1_020),
            wait(1_030),
            wait(1_040),
            wait(1_050),
            set_tracks(60, 60),
            wait(1_070),
            wait(1_090),
            set_tracks(60, 60),
        ],
    )
}

// ── Continuous scenarios ──────────────────────────────────────────────────────

/// A continuous drive holds its speed and never ramps or completes.
fn continuous_hold() -> Result<(), &'static str> {
    let run = run_drive_sim_limited(continuous(60), START_MS, Some(8), |now| {
        let encoder = match now {
            1_020 => encoder(500, 500, now),
            1_040 => encoder(1_500, 1_500, now),
            1_060 => encoder(2_500, 2_500, now),
            1_080 => encoder(3_500, 3_500, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_prefix(
        &run,
        &[
            set_tracks(60, 60),
            wait(1_020),
            wait(1_040),
            set_tracks(60, 60),
            wait(1_060),
            set_tracks(60, 60),
            wait(1_080),
            set_tracks(60, 60),
        ],
    )?;
    expect_pulses(&run, &[(60, 60), (60, 60), (60, 60), (60, 60)]).and_then(|()| {
        if run.outcome.is_none() {
            Ok(())
        } else {
            Err("continuous drive completed")
        }
    })
}

/// A continuous drive fails on a stall without telemetry.
fn continuous_stall() -> Result<(), &'static str> {
    let run = run_drive_sim(continuous(60), START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            _ if now >= 1_040 => encoder(100, 100, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_failed_without_telemetry(&run, DriveFault::Stall)
}

/// A continuous drive fails on an encoder timeout without telemetry.
fn continuous_encoder_timeout() -> Result<(), &'static str> {
    let run = run_drive_sim(continuous(60), START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(200, 200, now),
            _ if now >= 1_060 => encoder(200, 200, 1_040),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_failed_without_telemetry(&run, DriveFault::EncoderTimeout)
}

/// A continuous drive fails on a single-motor-zero anomaly without telemetry.
fn continuous_anomaly() -> Result<(), &'static str> {
    let run = run_drive_sim(continuous(60), START_MS, |now| {
        let encoder = match now {
            1_020 => encoder(100, 100, now),
            1_040 => encoder(100, 200, now),
            _ => None,
        };
        (encoder, Some(sample(now, 0.0)))
    })?;

    expect_failed_without_telemetry(&run, DriveFault::EncoderAnomaly)
}

// ── Brake, coast and idle scenarios ──────────────────────────────────────────

/// Brake emits `BrakeAll` and settles after three consecutive zero deltas.
fn brake_settle() -> Result<(), &'static str> {
    let run = run_drive_sim(brake(), START_MS, |now| {
        let encoder = match now {
            1_100 | 1_200 | 1_300 | 1_400 => encoder(100, 100, now),
            _ => None,
        };
        (encoder, None)
    })?;

    expect_actions(
        &run,
        &[
            IntentAction::BrakeAll,
            wait(1_100),
            wait(1_200),
            wait(1_300),
            wait(1_400),
            FINISHED,
        ],
    )?;
    expect_completed_without_telemetry(&run)
}

/// Coast resets the zero-delta count on motion before settling.
fn coast_non_zero_reset() -> Result<(), &'static str> {
    let run = run_drive_sim(coast(), START_MS, |now| {
        let encoder = match now {
            1_100 => encoder(100, 100, now),
            1_200 | 1_300 | 1_400 | 1_500 => encoder(110, 90, now),
            _ => None,
        };
        (encoder, None)
    })?;

    expect_actions(
        &run,
        &[
            IntentAction::CoastAll,
            wait(1_100),
            wait(1_200),
            wait(1_300),
            wait(1_400),
            wait(1_500),
            FINISHED,
        ],
    )?;
    expect_completed_without_telemetry(&run)
}

/// A wrapped counter delta is treated as motion, then equal counts settle.
fn brake_wrap() -> Result<(), &'static str> {
    let run = run_drive_sim(brake(), START_MS, |now| {
        let encoder = match now {
            1_100 => encoder(65_500, 65_500, now),
            1_200 | 1_300 | 1_400 | 1_500 => encoder(100, 100, now),
            _ => None,
        };
        (encoder, None)
    })?;

    expect_actions(
        &run,
        &[
            IntentAction::BrakeAll,
            wait(1_100),
            wait(1_200),
            wait(1_300),
            wait(1_400),
            wait(1_500),
            FINISHED,
        ],
    )?;
    expect_completed_without_telemetry(&run)
}

/// No encoder sample ever arrives: the settle times out with the exact label.
fn brake_timeout() -> Result<(), &'static str> {
    let run = run_drive_sim(brake(), START_MS, |_now| (None, None))?;

    expect_actions(
        &run,
        &[
            IntentAction::BrakeAll,
            wait(1_100),
            wait(1_200),
            wait(1_300),
            wait(1_400),
            wait(1_500),
            wait(1_600),
            wait(1_700),
            wait(1_800),
            wait(1_900),
            wait(2_000),
            wait(2_100),
            wait(2_200),
            wait(2_300),
            wait(2_400),
            wait(2_500),
            wait(2_600),
            wait(2_700),
            wait(2_800),
            wait(2_900),
            wait(3_000),
            FINISHED,
        ],
    )?;
    expect_failed_without_telemetry(&run, DriveFault::SettleTimeout)?;
    if DriveFault::SettleTimeout.label() != "encoder settle timeout" {
        return Err("unexpected settle fault label");
    }
    Ok(())
}

/// Zero-duration idle coasts and finishes on the next step.
fn idle_zero() -> Result<(), &'static str> {
    let run = run_drive_sim(idle(0), START_MS, |_now| (None, None))?;

    expect_actions(&run, &[IntentAction::CoastAll, FINISHED])?;
    expect_completed_without_telemetry(&run)
}

/// A non-zero idle coasts, waits its absolute deadline, then finishes.
fn idle_duration() -> Result<(), &'static str> {
    let run = run_drive_sim(idle(500), START_MS, |_now| (None, None))?;

    expect_actions(&run, &[IntentAction::CoastAll, wait(1_500), FINISHED])?;
    expect_completed_without_telemetry(&run)
}
