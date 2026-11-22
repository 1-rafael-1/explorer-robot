//! Host-side integration tests for the `motor-cal` Motor Calibration Curve.
//!
//! Every assertion is made through the crate's public API. The expected values
//! are worked examples from `.scratch/motor-calibration/precision-diagnosis.md`
//! and `.scratch/motor-calibration/spec.md` — literal commands and corrections —
//! never a re-run of the derivation the crate does.

use motor_cal::{
    CurveCommand, CurveError, Direction, EncoderSample, InvalidReason, LegPoint, MEASUREMENT_WINDOW_MS,
    MeasurementEngine, POWER_STEPS, Phase, PointResult, SETTLE_MS, Sweep, TestLeg, Track, derive_curve,
};

/// The Measurement Window length used by the worked examples, in milliseconds.
const WINDOW_MS: u64 = 2_000;

/// The ten Power Steps of a Test Leg, in percent.
const PERCENTS: [u8; POWER_STEPS] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

/// The four Test Legs in cycle order.
const ALL_LEGS: [TestLeg; 4] = [
    TestLeg::new(Track::Left, Direction::Forward),
    TestLeg::new(Track::Left, Direction::Reverse),
    TestLeg::new(Track::Right, Direction::Forward),
    TestLeg::new(Track::Right, Direction::Reverse),
];

/// A linear leg: 10 counts per second per percent.
const LINEAR: [u32; POWER_STEPS] = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];

/// A leg that is 80 % of [`LINEAR`].
const SLOW: [u32; POWER_STEPS] = [80, 160, 240, 320, 400, 480, 560, 640, 720, 800];

/// A leg that is twice [`LINEAR`].
const FAST: [u32; POWER_STEPS] = [200, 400, 600, 800, 1000, 1200, 1400, 1600, 1800, 2000];

/// The simultaneous-pair forward left series from the 2026-11-21 bench run.
const PAIR_LEFT: [u32; POWER_STEPS] = [130, 259, 435, 583, 720, 860, 1000, 1130, 1260, 1400];

/// The simultaneous-pair forward right series from the 2026-11-21 bench run.
const PAIR_RIGHT: [u32; POWER_STEPS] = [120, 231, 397, 568, 710, 850, 990, 1120, 1250, 1390];

/// The raw count that reports `speed_counts_per_s` over [`WINDOW_MS`].
///
/// A 2 000 ms window makes one encoder count worth half a count per second.
fn count_for(speed_counts_per_s: u32) -> u16 {
    u16::try_from(speed_counts_per_s * 2).unwrap_or(u16::MAX)
}

/// One Power Step that moved, reporting `speed_counts_per_s`.
fn moving(percent: u8, speed_counts_per_s: u32) -> LegPoint {
    LegPoint::new(percent, count_for(speed_counts_per_s), WINDOW_MS, true)
}

/// One Power Step that showed no sustained motion, with `stray` raw counts.
const fn stalled(percent: u8, stray: u16) -> LegPoint {
    LegPoint::new(percent, stray, WINDOW_MS, false)
}

/// Build a Test Leg from ten speeds; every Power Step moved.
fn moving_leg(speeds: [u32; POWER_STEPS]) -> Vec<LegPoint> {
    PERCENTS
        .iter()
        .copied()
        .zip(speeds)
        .map(|(percent, speed)| moving(percent, speed))
        .collect()
}

/// Build a Test Leg from ten `(speed, moved)` pairs.
fn leg_from(steps: [(u32, bool); POWER_STEPS]) -> Vec<LegPoint> {
    PERCENTS
        .iter()
        .copied()
        .zip(steps)
        .map(|(percent, (speed, moved))| {
            if moved {
                moving(percent, speed)
            } else {
                stalled(percent, 0)
            }
        })
        .collect()
}

/// A sweep of the four Test Legs.
const fn sweep<'a>(
    left_forward: &'a [LegPoint],
    left_reverse: &'a [LegPoint],
    right_forward: &'a [LegPoint],
    right_reverse: &'a [LegPoint],
) -> Sweep<'a> {
    Sweep::new(left_forward, left_reverse, right_forward, right_reverse)
}

/// Assert a correction is the worked-example `expected` value.
fn assert_correction(actual: Option<f32>, expected: f32) {
    assert!(actual.is_some(), "expected a correction at or above the floor");
    if let Some(actual) = actual {
        assert!((actual - expected).abs() < 1e-5, "expected {expected}, got {actual}");
    }
}

/// Drive a fresh Measurement Engine to a 400-count, 2 000 ms window result.
fn measured_400_counts() -> PointResult {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);

    let mut quiescence_ms = 0;
    loop {
        engine.observe(EncoderSample {
            timestamp_ms: quiescence_ms,
            count: 0,
        });
        if engine.phase() == Phase::Quiescent {
            break;
        }
        quiescence_ms += 20;
    }

    engine.reset_now(quiescence_ms);
    let command_ms = quiescence_ms + 20;
    engine.observe(EncoderSample {
        timestamp_ms: command_ms,
        count: 0,
    });
    engine.command_applied(command_ms);

    let window_start_ms = command_ms + SETTLE_MS;
    let mut timestamp_ms = command_ms + 20;
    while timestamp_ms < window_start_ms {
        engine.observe(EncoderSample { timestamp_ms, count: 0 });
        timestamp_ms += 20;
    }

    let mut count = 0;
    while timestamp_ms <= window_start_ms + MEASUREMENT_WINDOW_MS {
        engine.observe(EncoderSample { timestamp_ms, count });
        count += 4;
        timestamp_ms += 20;
    }

    engine.result().unwrap_or(PointResult::Invalid(InvalidReason::NoWindow))
}

/// The simultaneous-pair sweep from the bench run: both forward legs are the
/// measured pair, both reverse legs are the linear identity.
fn pair_curve() -> Result<motor_cal::Curve, CurveError> {
    let left_forward = moving_leg(PAIR_LEFT);
    let right_forward = moving_leg(PAIR_RIGHT);
    let left_reverse = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);
    derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse))
}

#[test]
fn a_leg_point_reports_its_window_speed() {
    let point = LegPoint::new(40, 400, WINDOW_MS, true);

    assert_eq!(point.command_percent(), 40);
    assert_eq!(point.count(), 400);
    assert_eq!(point.window_ms(), WINDOW_MS);
    assert!(point.sustained_motion());
    assert_eq!(point.speed_counts_per_s(), 200);
}

#[test]
fn a_measurement_becomes_a_leg_point() {
    let result = measured_400_counts();
    assert!(result.measurement().is_some());

    if let Some(measurement) = result.measurement() {
        let point = LegPoint::from_measurement(40, measurement, MEASUREMENT_WINDOW_MS);

        assert_eq!(point.command_percent(), 40);
        assert_eq!(point.count(), 400);
        assert_eq!(point.window_ms(), MEASUREMENT_WINDOW_MS);
        assert!(point.sustained_motion());
        assert_eq!(point.speed_counts_per_s(), 200);
    }
}

#[test]
fn an_invalid_point_result_has_no_leg_point() {
    let mut engine = MeasurementEngine::new();
    engine.begin_step(0);
    engine.finish();

    assert!(engine.result().is_some());
    if let Some(result) = engine.result() {
        assert!(matches!(result, PointResult::Invalid(_)));
        assert_eq!(LegPoint::from_point_result(10, result, WINDOW_MS), None);
    }
}

#[test]
fn identical_legs_produce_identity_corrections() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 10);
        for leg in ALL_LEGS {
            for percent in PERCENTS {
                assert_correction(curve.correction(leg.track, leg.direction, percent), 1.0);
            }
        }
        // The inverse curve reproduces a request between measured points exactly.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 55),
            CurveCommand::Corrected(550)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Reverse, 55),
            CurveCommand::Corrected(550)
        );
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 0), CurveCommand::Raw(0));
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 9), CurveCommand::Raw(90));
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 10),
            CurveCommand::Corrected(100)
        );
    }
}

#[test]
fn the_slower_track_is_the_target_and_the_faster_is_attenuated() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(SLOW);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // At the 10 % Calibration Floor the faster leg's ideal 8 % command would be
        // below the floor, so the applied command is floored and the correction in
        // force is the identity. Above the floor it is attenuated to 0.8.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 10), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 10), 1.0);
        assert_correction(curve.correction(Track::Left, Direction::Reverse, 10), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Reverse, 10), 1.0);
        for percent in PERCENTS.iter().copied().skip(1) {
            assert_correction(curve.correction(Track::Left, Direction::Forward, percent), 0.8);
            assert_correction(curve.correction(Track::Right, Direction::Forward, percent), 1.0);
            assert_correction(curve.correction(Track::Left, Direction::Reverse, percent), 1.0);
            assert_correction(curve.correction(Track::Right, Direction::Reverse, percent), 1.0);
        }
        for leg in ALL_LEGS {
            for percent in PERCENTS {
                if let Some(correction) = curve.correction(leg.track, leg.direction, percent) {
                    assert!(correction <= 1.0, "correction {correction} exceeds 1.00");
                }
            }
        }
        // The faster track is commanded to the slower track's speed.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 50),
            CurveCommand::Corrected(400)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 50),
            CurveCommand::Corrected(500)
        );
    }
}

#[test]
fn corrections_are_independent_per_direction() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(SLOW);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_correction(curve.correction(Track::Left, Direction::Forward, 50), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 50), 1.0);
        assert_correction(curve.correction(Track::Left, Direction::Reverse, 50), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Reverse, 50), 0.8);
    }
}

#[test]
fn the_floor_is_the_highest_breakaway_and_below_it_the_raw_command_is_used() {
    let left_forward = leg_from([
        (0, false),
        (0, false),
        (300, true),
        (400, true),
        (500, true),
        (600, true),
        (700, true),
        (800, true),
        (900, true),
        (1000, true),
    ]);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 30);
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 0), CurveCommand::Raw(0));
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 10),
            CurveCommand::Raw(100)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 20),
            CurveCommand::Raw(200)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 29),
            CurveCommand::Raw(290)
        );
        assert!(curve.correction(Track::Right, Direction::Forward, 20).is_none());
        assert_correction(curve.correction(Track::Right, Direction::Forward, 30), 1.0);
        assert!(curve.resolve(Track::Right, Direction::Forward, 30).is_corrected());
        assert!(curve.resolve(Track::Left, Direction::Forward, 30).is_corrected());
    }
}

#[test]
fn a_floor_at_one_hundred_percent_leaves_every_lower_command_raw() {
    let left_forward = leg_from([
        (0, false),
        (0, false),
        (0, false),
        (0, false),
        (0, false),
        (0, false),
        (0, false),
        (0, false),
        (0, false),
        (1000, true),
    ]);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 100);
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 99),
            CurveCommand::Raw(990)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 99),
            CurveCommand::Raw(990)
        );
        assert!(curve.resolve(Track::Left, Direction::Forward, 100).is_corrected());
        assert_correction(curve.correction(Track::Left, Direction::Forward, 100), 1.0);
    }
}

#[test]
fn the_measured_speed_series_is_made_monotone_before_inversion() {
    let left = [100, 200, 180, 400, 500, 600, 700, 800, 900, 1000];
    let right = [100, 200, 250, 400, 500, 600, 700, 800, 900, 1000];
    let left_forward = moving_leg(left);
    let left_reverse = moving_leg(right);
    let right_forward = moving_leg(right);
    let right_reverse = moving_leg(left);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_correction(curve.correction(Track::Left, Direction::Forward, 20), 1.0);
        // The dip at 30 % is replaced by the running maximum of 200 counts/s,
        // which is also the target. The left leg is on a plateau from 20 % to
        // 30 %, so it inverts to the highest command that still produces 200:
        // 30 % is the identity. The right leg's 30 % speed (250 counts/s) is above
        // the target, so it still inverts to 20 %.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 30), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 30), 2.0 / 3.0);
        assert_correction(curve.correction(Track::Left, Direction::Forward, 40), 1.0);
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 30),
            CurveCommand::Corrected(300)
        );
    }
}

#[test]
fn the_target_speed_is_interpolated_between_measured_steps() {
    // The right leg reaches 300 counts/s at 20 % and sits there at 30 %, so the
    // slower track is the linear left leg at every measured step. Half-way
    // between 20 % and 30 % the target is therefore 250 counts/s.
    let right = [150, 300, 300, 400, 500, 600, 700, 800, 900, 1000];
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(right);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // Target 150 at 15 %: the linear left leg is identity.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 15),
            CurveCommand::Corrected(150)
        );
        // Target 250 at 25 %: the right leg needs command 16.67 % = 167 per-mille.
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 25),
            CurveCommand::Corrected(167)
        );
        assert_correction(curve.correction(Track::Right, Direction::Forward, 25), 2.0 / 3.0);
    }
}

#[test]
fn the_top_target_holds_at_and_above_one_hundred_percent() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(SLOW);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_correction(curve.correction(Track::Left, Direction::Forward, 100), 0.8);
        // 80 / 150: the top command holds but the factor is reported against the
        // requested magnitude.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 150), 80.0 / 150.0);
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 100),
            CurveCommand::Corrected(800)
        );
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 150),
            CurveCommand::Corrected(800)
        );
    }
}

#[test]
fn zero_resolves_to_zero() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        for leg in ALL_LEGS {
            let command = curve.resolve(leg.track, leg.direction, 0);
            assert_eq!(command, CurveCommand::Raw(0));
            assert_eq!(command.per_mille(), 0);
            assert!(!command.is_corrected());
        }
    }
}

#[test]
fn a_point_that_never_moved_is_excluded_from_the_target() {
    let right_forward = leg_from([
        (100, true),
        (200, true),
        (300, true),
        (400, true),
        (0, false),
        (600, true),
        (700, true),
        (800, true),
        (900, true),
        (1000, true),
    ]);
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // The stalled point is excluded, so the target at 50 % is interpolated
        // between the measured 40 % and 60 % points (500 counts/s), not dragged
        // down to the stalled point's running maximum (400 counts/s).
        assert_correction(curve.correction(Track::Left, Direction::Forward, 50), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 50), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 40), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 60), 1.0);
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 50),
            CurveCommand::Corrected(500)
        );
    }
}

#[test]
fn a_leg_that_does_not_move_at_one_hundred_percent_fails() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = leg_from([
        (100, true),
        (200, true),
        (300, true),
        (400, true),
        (500, true),
        (600, true),
        (700, true),
        (800, true),
        (900, true),
        (0, false),
    ]);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    assert_eq!(
        derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::LegDidNotMove {
            leg: TestLeg::new(Track::Left, Direction::Reverse),
        })
    );
}

#[test]
fn a_target_below_the_first_moved_point_uses_the_zero_anchor() {
    // The left leg produces 200 counts/s at its first moved point (10 %), while
    // the target there is 100 counts/s. The inverse curve needs the (0, 0)
    // anchor to reach a command below 10 %: it interpolates to 5 %, an ideal
    // correction of 0.50 at the plausibility band, so the curve is accepted.
    let left_forward = moving_leg(FAST);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // The applied command is never below the 10 % Calibration Floor, so the
        // 5 % ideal is lifted to 10 %: the correction in force is the identity.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 10),
            CurveCommand::Corrected(100)
        );
        assert_correction(curve.correction(Track::Left, Direction::Forward, 10), 1.0);
    }
}

#[test]
fn a_correction_at_the_plausibility_band_is_accepted() {
    let left_forward = moving_leg(FAST);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // The faster leg needs half the command to match the slower leg.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 50), 0.5);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 50), 1.0);
    }
}

#[test]
fn an_implausible_correction_fails_rather_than_being_clamped() {
    let left = [250, 500, 750, 1000, 1250, 1500, 1750, 2000, 2250, 2500];
    let left_forward = moving_leg(left);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    assert_eq!(
        derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::ImplausibleCorrection {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 10,
            correction: 0.4,
        })
    );
}

#[test]
fn a_missing_point_fails_with_the_reason() {
    let mut left_forward = moving_leg(LINEAR);
    left_forward.remove(4);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    assert_eq!(
        derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::MissingPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 50,
        })
    );
}

#[test]
fn an_invalid_point_fails_with_the_reason() {
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let mut wrong_percent = moving_leg(LINEAR);
    wrong_percent[1] = LegPoint::new(15, count_for(200), WINDOW_MS, true);
    assert_eq!(
        derive_curve(sweep(&wrong_percent, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 15,
        })
    );

    let mut zero_window = moving_leg(LINEAR);
    zero_window[1] = LegPoint::new(20, count_for(200), 0, true);
    assert_eq!(
        derive_curve(sweep(&zero_window, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 20,
        })
    );

    let mut zero_speed = moving_leg(LINEAR);
    zero_speed[1] = LegPoint::new(20, 0, WINDOW_MS, true);
    assert_eq!(
        derive_curve(sweep(&zero_speed, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 20,
        })
    );

    let mut duplicate = moving_leg(LINEAR);
    duplicate[1] = LegPoint::new(10, count_for(200), WINDOW_MS, true);
    assert_eq!(
        derive_curve(sweep(&duplicate, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 10,
        })
    );
}

#[test]
fn failure_reasons_name_themselves_for_the_log() {
    let leg = TestLeg::new(Track::Left, Direction::Forward);

    assert_eq!(
        CurveError::MissingPoint {
            leg,
            command_percent: 50,
        }
        .label(),
        "missing point"
    );
    assert_eq!(
        CurveError::InvalidPoint {
            leg,
            command_percent: 50,
        }
        .label(),
        "invalid point"
    );
    assert_eq!(CurveError::LegDidNotMove { leg }.label(), "leg did not move");
    assert_eq!(
        CurveError::ImplausibleCorrection {
            leg,
            command_percent: 50,
            correction: 0.4,
        }
        .label(),
        "implausible correction"
    );
}

#[test]
fn inverse_interpolation_beats_the_linear_ratio() {
    let curve = pair_curve();

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // Target at 30 % forward is the slower right leg's 397 counts/s.
        // The linear ratio would command `30 * 397 / 435 = 27.4 %` = 274
        // per-mille; the inverse interpolation over left (20, 259) and (30, 435)
        // commands 27.84 % = 278 per-mille.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 30),
            CurveCommand::Corrected(278)
        );
        assert_ne!(
            curve.resolve(Track::Left, Direction::Forward, 30),
            CurveCommand::Corrected(270)
        );
        // The slower leg is the target, so its own command is unchanged.
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 30),
            CurveCommand::Corrected(300)
        );
    }
}

#[test]
fn the_simultaneous_pair_at_forty_percent() {
    let curve = pair_curve();

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // Raw pair: left 583, right 568. The needed left command is 38.99 %
        // = 390 per-mille, not the linear ratio's 36 %.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 40),
            CurveCommand::Corrected(390)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 40),
            CurveCommand::Corrected(400)
        );
        assert_correction(curve.correction(Track::Left, Direction::Forward, 40), 0.974_662);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 40), 1.0);
    }
}

#[test]
fn a_low_speed_plateau_inverts_to_its_highest_command() {
    // Speeds of 20 counts/s at the first three Power Steps (40 and 41 counts over
    // the 2 000 ms window both report 20) form a plateau. Identical legs make
    // 20 counts/s the target at 30 %, and the inverse must keep the highest
    // command that still produces it rather than the lowest.
    let plateau = [20, 20, 20, 40, 60, 80, 100, 120, 140, 160];
    let left_forward = moving_leg(plateau);
    let left_reverse = moving_leg(plateau);
    let right_forward = moving_leg(plateau);
    let right_reverse = moving_leg(plateau);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    // The old lowest-command inversion produced 1/3 at 30 % and failed.
    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_correction(curve.correction(Track::Left, Direction::Forward, 30), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 30), 1.0);
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 30),
            CurveCommand::Corrected(300)
        );
    }
}

#[test]
fn a_target_below_the_faster_legs_first_point_is_floored_to_the_floor() {
    // The floor is 30 %: neither forward leg moves at 10 % or 20 %. At 30 % the
    // slow leg produces 170 counts/s and the fast leg 340 counts/s, so the target
    // is 170 and the fast leg's ideal inverse (from the zero anchor) is 15 %,
    // below its own breakaway. The applied command is lifted to the floor.
    let slow = leg_from([
        (0, false),
        (0, false),
        (170, true),
        (340, true),
        (425, true),
        (510, true),
        (595, true),
        (680, true),
        (765, true),
        (850, true),
    ]);
    let fast = leg_from([
        (0, false),
        (0, false),
        (340, true),
        (400, true),
        (480, true),
        (560, true),
        (640, true),
        (720, true),
        (800, true),
        (880, true),
    ]);
    let left_reverse = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&slow, &left_reverse, &fast, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 30);
        // The unclamped inverse is 15 % = 150 per-mille; the floor lifts it to 30 %.
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 30),
            CurveCommand::Corrected(300)
        );
        assert_ne!(
            curve.resolve(Track::Right, Direction::Forward, 30),
            CurveCommand::Corrected(150)
        );
        // The correction in force reports the applied (floored) command.
        assert_correction(curve.correction(Track::Right, Direction::Forward, 30), 1.0);
        // The slow leg is the target, so its own command is unchanged.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 30),
            CurveCommand::Corrected(300)
        );
        assert_correction(curve.correction(Track::Left, Direction::Forward, 30), 1.0);
    }
}

#[test]
fn a_floored_command_does_not_hide_an_implausible_ideal_correction() {
    // The floor keeps the applied command at 30 %, but the measurement implies a
    // 10 % ideal command: the fast leg produces three times the slow leg's speed
    // at the first moved Power Step, so its ideal correction is 1/3, below the
    // plausibility band. Calibration must still fail.
    let slow = leg_from([
        (0, false),
        (0, false),
        (170, true),
        (340, true),
        (425, true),
        (510, true),
        (595, true),
        (680, true),
        (765, true),
        (850, true),
    ]);
    let fast = leg_from([
        (0, false),
        (0, false),
        (510, true),
        (600, true),
        (700, true),
        (800, true),
        (900, true),
        (1000, true),
        (1100, true),
        (1200, true),
    ]);
    let left_reverse = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    assert_eq!(
        derive_curve(sweep(&slow, &left_reverse, &fast, &right_reverse)),
        Err(CurveError::ImplausibleCorrection {
            leg: TestLeg::new(Track::Right, Direction::Forward),
            command_percent: 30,
            correction: 10.0 / 30.0,
        })
    );
}

#[test]
fn exact_and_between_point_inversions_stay_identity() {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(LINEAR);
    let right_reverse = moving_leg(LINEAR);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // An exact measured point is the identity.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 30),
            CurveCommand::Corrected(300)
        );
        // A between-point request interpolates linearly and is also the identity.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 45),
            CurveCommand::Corrected(450)
        );
        assert_correction(curve.correction(Track::Left, Direction::Forward, 45), 1.0);
    }
}
