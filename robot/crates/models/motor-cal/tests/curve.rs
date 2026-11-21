//! Host-side integration tests for the `motor-cal` Motor Calibration Curve.
//!
//! Every assertion is made through the crate's public API. The expected values
//! are worked examples from `.scratch/motor-calibration/spec.md` — literal
//! corrections and commands — never a re-run of the derivation the crate does.

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
    let speeds = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let left_forward = moving_leg(speeds);
    let left_reverse = moving_leg(speeds);
    let right_forward = moving_leg(speeds);
    let right_reverse = moving_leg(speeds);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 10);
        for leg in ALL_LEGS {
            for percent in PERCENTS {
                assert_correction(curve.correction(leg.track, leg.direction, percent), 1.0);
            }
        }
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 55),
            CurveCommand::Corrected(55)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Reverse, 55),
            CurveCommand::Corrected(55)
        );
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 0), CurveCommand::Raw(0));
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 9), CurveCommand::Raw(9));
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 10),
            CurveCommand::Corrected(10)
        );
    }
}

#[test]
fn the_slower_track_is_the_target_and_the_faster_is_attenuated() {
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let slow = [80, 160, 240, 320, 400, 480, 560, 640, 720, 800];
    let left_forward = moving_leg(base);
    let left_reverse = moving_leg(base);
    let right_forward = moving_leg(slow);
    let right_reverse = moving_leg(base);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        for percent in PERCENTS {
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
    }
}

#[test]
fn corrections_are_independent_per_direction() {
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let slow_reverse = [80, 160, 240, 320, 400, 480, 560, 640, 720, 800];
    let left_forward = moving_leg(base);
    let left_reverse = moving_leg(slow_reverse);
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

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
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
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
    let left_reverse = moving_leg(base);
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 30);
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 0), CurveCommand::Raw(0));
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 10),
            CurveCommand::Raw(10)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 20),
            CurveCommand::Raw(20)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 29),
            CurveCommand::Raw(29)
        );
        assert!(curve.correction(Track::Right, Direction::Forward, 20).is_none());
        assert_correction(curve.correction(Track::Right, Direction::Forward, 30), 1.0);
        assert!(curve.resolve(Track::Right, Direction::Forward, 30).is_corrected());
        assert!(curve.resolve(Track::Left, Direction::Forward, 30).is_corrected());
    }
}

#[test]
fn a_floor_at_one_hundred_percent_leaves_every_lower_command_raw() {
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
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
    let left_reverse = moving_leg(base);
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.floor_percent(), 100);
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 99),
            CurveCommand::Raw(99)
        );
        assert_eq!(
            curve.resolve(Track::Right, Direction::Forward, 99),
            CurveCommand::Raw(99)
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
        // so the left correction is 1.00 and the right is 200/250 = 0.8.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 30), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 30), 0.8);
        assert_correction(curve.correction(Track::Left, Direction::Forward, 40), 1.0);
    }
}

#[test]
fn interpolation_is_piecewise_linear_between_measured_points() {
    let target = [100; POWER_STEPS];
    let left = [100, 125, 160, 160, 160, 160, 160, 160, 160, 160];
    let left_forward = moving_leg(left);
    let left_reverse = moving_leg(target);
    let right_forward = moving_leg(target);
    let right_reverse = moving_leg(target);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_correction(curve.correction(Track::Left, Direction::Forward, 10), 1.0);
        assert_correction(curve.correction(Track::Left, Direction::Forward, 20), 0.8);
        assert_correction(curve.correction(Track::Left, Direction::Forward, 30), 0.625);
        // Half-way between 10 % (1.00) and 20 % (0.80).
        assert_correction(curve.correction(Track::Left, Direction::Forward, 15), 0.9);
        // Half-way between 20 % (0.80) and 30 % (0.625).
        assert_correction(curve.correction(Track::Left, Direction::Forward, 25), 0.7125);
        // 14 * 0.9 = 12.6, which rounds to 13.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 14),
            CurveCommand::Corrected(13)
        );
        // 25 * 0.7125 = 17.8125, which rounds to 18.
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 25),
            CurveCommand::Corrected(18)
        );
    }
}

#[test]
fn the_top_point_holds_at_and_above_one_hundred_percent() {
    let target = [100; POWER_STEPS];
    let left = [100, 125, 125, 125, 125, 125, 125, 125, 125, 125];
    let left_forward = moving_leg(left);
    let left_reverse = moving_leg(target);
    let right_forward = moving_leg(target);
    let right_reverse = moving_leg(target);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_correction(curve.correction(Track::Left, Direction::Forward, 100), 0.8);
        assert_correction(curve.correction(Track::Left, Direction::Forward, 150), 0.8);
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 100),
            CurveCommand::Corrected(80)
        );
        assert_eq!(
            curve.resolve(Track::Left, Direction::Forward, 150),
            CurveCommand::Corrected(100)
        );
    }
}

#[test]
fn zero_resolves_to_zero() {
    let base = [100; POWER_STEPS];
    let left_forward = moving_leg(base);
    let left_reverse = moving_leg(base);
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        for leg in ALL_LEGS {
            let command = curve.resolve(leg.track, leg.direction, 0);
            assert_eq!(command, CurveCommand::Raw(0));
            assert_eq!(command.command(), 0);
            assert!(!command.is_corrected());
        }
    }
}

#[test]
fn a_point_that_never_moved_is_excluded_from_the_target() {
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let left_forward = moving_leg(base);
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
    let left_reverse = moving_leg(base);
    let right_reverse = moving_leg(base);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // If the stalled right point were in the target it would drag the
        // target down to its running maximum (400), making this 0.8.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 50), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 40), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 50), 1.0);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 60), 1.0);
    }
}

#[test]
fn a_leg_that_does_not_move_at_one_hundred_percent_fails() {
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let left_forward = moving_leg(base);
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
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

    assert_eq!(
        derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::LegDidNotMove {
            leg: TestLeg::new(Track::Left, Direction::Reverse),
        })
    );
}

#[test]
fn a_correction_at_the_plausibility_band_is_accepted() {
    let left = [1000; POWER_STEPS];
    let right = [500; POWER_STEPS];
    let left_forward = moving_leg(left);
    let left_reverse = moving_leg(right);
    let right_forward = moving_leg(right);
    let right_reverse = moving_leg(right);

    let curve = derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse));

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        // 500 / 1000 = 0.50 is inside the inclusive band.
        assert_correction(curve.correction(Track::Left, Direction::Forward, 50), 0.5);
        assert_correction(curve.correction(Track::Right, Direction::Forward, 50), 1.0);
    }
}

#[test]
fn an_implausible_correction_fails_rather_than_being_clamped() {
    let left = [1000; POWER_STEPS];
    let right = [400; POWER_STEPS];
    let left_forward = moving_leg(left);
    let left_reverse = moving_leg(right);
    let right_forward = moving_leg(right);
    let right_reverse = moving_leg(right);

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
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let mut left_forward = moving_leg(base);
    left_forward.remove(4);
    let left_reverse = moving_leg(base);
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

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
    let base = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];
    let left_reverse = moving_leg(base);
    let right_forward = moving_leg(base);
    let right_reverse = moving_leg(base);

    let mut wrong_percent = moving_leg(base);
    wrong_percent[1] = LegPoint::new(15, count_for(200), WINDOW_MS, true);
    assert_eq!(
        derive_curve(sweep(&wrong_percent, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 15,
        })
    );

    let mut zero_window = moving_leg(base);
    zero_window[1] = LegPoint::new(20, count_for(200), 0, true);
    assert_eq!(
        derive_curve(sweep(&zero_window, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 20,
        })
    );

    let mut zero_speed = moving_leg(base);
    zero_speed[1] = LegPoint::new(20, 0, WINDOW_MS, true);
    assert_eq!(
        derive_curve(sweep(&zero_speed, &left_reverse, &right_forward, &right_reverse)),
        Err(CurveError::InvalidPoint {
            leg: TestLeg::new(Track::Left, Direction::Forward),
            command_percent: 20,
        })
    );

    let mut duplicate = moving_leg(base);
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
