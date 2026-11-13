//! Host-side integration tests for the `dmp-cal` crate.
//!
//! These exercise the eight-step order and actions, Done/Redo advancement and
//! completion, the Compass Heading conversion, the per-step motion hints, the
//! two-target Compass Check, and the DMP Bias Snapshot encoding and
//! validation. Every assertion is made through the public API.

use dmp_cal::{
    CALIBRATED_COMPASS_ACCURACY, CHECK_STABILITY_SPAN_DEG, CHECK_STABILITY_WINDOW_MS, CHECK_TOLERANCE_DEG,
    CheckFailure, CheckOutcome, CompassCalibration, CompassCheck, CompassSample, CompassStep, CompassStepKind,
    CompassTarget, DMP_BIAS_ENCODED_LEN, DMP_BIAS_FORMAT_VERSION, DmpBiasSnapshot, MAGNETIC_EAST_DEG,
    MAGNETIC_NORTH_DEG, MOUNTING_OFFSET_DEG, MotionHint, MotionSample, SnapshotError, compass_heading,
};

/// Comparison tolerance for angles in degrees.
const ANGLE_EPS: f32 = 1e-4;

/// Spacing of the synthetic samples the check tests feed, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 100;

/// Number of samples that exactly covers the stability window.
const WINDOW_SAMPLES: u64 = (CHECK_STABILITY_WINDOW_MS / SAMPLE_INTERVAL_MS) + 1;

/// Assert that two angles agree within [`ANGLE_EPS`].
fn assert_angle(actual: f32, expected: f32) {
    assert!(
        (actual - expected).abs() <= ANGLE_EPS,
        "expected {expected}°, got {actual}°"
    );
}

/// Build an attitude sample.
const fn motion(timestamp_ms: u64, yaw_deg: f32, pitch_deg: f32, roll_deg: f32) -> MotionSample {
    MotionSample {
        timestamp_ms,
        yaw_deg,
        pitch_deg,
        roll_deg,
    }
}

/// A fresh calibration advanced to `step`.
fn calibration_at(step: CompassStep) -> CompassCalibration {
    let mut calibration = CompassCalibration::new();
    while calibration.step() != Some(step) {
        assert!(calibration.done());
    }
    calibration
}

/// Feed `count` identical compass samples at [`SAMPLE_INTERVAL_MS`] spacing.
fn feed(check: &mut CompassCheck, heading_deg: f32, accuracy: u8, start_ms: u64, count: u64) -> CheckOutcome {
    let mut outcome = check.outcome();
    for index in 0..count {
        outcome = check.sample(CompassSample {
            timestamp_ms: start_ms + (index * SAMPLE_INTERVAL_MS),
            heading_deg,
            accuracy,
        });
    }
    outcome
}

#[test]
fn steps_are_in_the_agreed_order_with_their_metadata() {
    let expected = [
        (
            CompassStep::FlatHold,
            CompassStepKind::Hold,
            "Place flat. Hold still.",
            false,
        ),
        (
            CompassStep::FlatSpin,
            CompassStepKind::Spin,
            "On the table, spin 360 deg both ways.",
            false,
        ),
        (
            CompassStep::NoseTilt,
            CompassStepKind::Tilt,
            "Tilt nose up and down 90 deg.",
            false,
        ),
        (
            CompassStep::SideTilt,
            CompassStepKind::Tilt,
            "Tilt left and right 90 deg.",
            false,
        ),
        (
            CompassStep::InvertedSpin,
            CompassStepKind::Spin,
            "Flip wheels up, spin 360 deg both ways.",
            false,
        ),
        (
            CompassStep::FlatHoldBeforeCheck,
            CompassStepKind::Hold,
            "Place flat. Hold still.",
            false,
        ),
        (
            CompassStep::NorthCheck,
            CompassStepKind::Check,
            "Point front at magnetic north (0 deg). Hold still.",
            true,
        ),
        (
            CompassStep::EastCheck,
            CompassStepKind::Check,
            "Point front at magnetic east (90 deg). Hold still.",
            true,
        ),
    ];

    assert_eq!(CompassStep::ALL.len(), expected.len());
    for (index, (step, (expected_step, kind, instruction, redo_allowed))) in
        CompassStep::ALL.iter().zip(expected).enumerate()
    {
        assert_eq!(*step, expected_step);
        assert_eq!(step.index(), index);
        assert_eq!(step.kind(), kind);
        assert_eq!(step.instruction(), instruction);
        assert_eq!(step.redo_allowed(), redo_allowed);
    }
}

#[test]
fn check_steps_name_their_magnetic_targets() {
    assert!(matches!(
        CompassStep::NorthCheck.check_target_deg(),
        Some(target) if (target - MAGNETIC_NORTH_DEG).abs() <= ANGLE_EPS
    ));
    assert!(matches!(
        CompassStep::EastCheck.check_target_deg(),
        Some(target) if (target - MAGNETIC_EAST_DEG).abs() <= ANGLE_EPS
    ));
    for step in &CompassStep::ALL[..6] {
        assert!(step.check_target_deg().is_none());
    }
}

#[test]
fn done_advances_through_every_step_and_completes() {
    let mut calibration = CompassCalibration::new();
    assert_eq!(calibration.step(), Some(CompassStep::FlatHold));
    assert!(!calibration.is_complete());

    for expected in &CompassStep::ALL[1..] {
        assert!(calibration.done());
        assert_eq!(calibration.step(), Some(*expected));
    }
    assert!(!calibration.is_complete());

    assert!(calibration.done());
    assert!(calibration.is_complete());
    assert_eq!(calibration.step(), None);

    // Completion is explicit: there is no step to advance off the end of.
    assert!(!calibration.done());
    assert_eq!(calibration.step(), None);
    assert!(matches!(calibration.hint(), MotionHint::None));
}

#[test]
fn redo_restarts_from_the_first_step_at_each_check() {
    let mut calibration = calibration_at(CompassStep::NorthCheck);
    assert!(calibration.redo());
    assert_eq!(calibration.step(), Some(CompassStep::FlatHold));

    // A movement step does not offer Redo.
    assert!(!calibration.redo());
    assert_eq!(calibration.step(), Some(CompassStep::FlatHold));

    let mut calibration = calibration_at(CompassStep::EastCheck);
    assert!(calibration.redo());
    assert_eq!(calibration.step(), Some(CompassStep::FlatHold));

    for _ in 0..CompassStep::ALL.len() {
        assert!(calibration.done());
    }
    assert!(calibration.is_complete());
    assert!(!calibration.redo());
}

#[test]
fn motion_hints_never_gate_done() {
    let mut calibration = calibration_at(CompassStep::FlatSpin);
    assert!(matches!(
        calibration.hint(),
        MotionHint::YawSpanDeg(span) if span.abs() <= ANGLE_EPS
    ));
    assert!(calibration.done());
    assert_eq!(calibration.step(), Some(CompassStep::NoseTilt));
}

#[test]
fn spin_hint_is_a_wrap_aware_yaw_span() {
    let mut calibration = calibration_at(CompassStep::FlatSpin);
    calibration.observe(motion(0, 350.0, 0.0, 0.0));
    calibration.observe(motion(100, 10.0, 0.0, 0.0));
    assert!(matches!(
        calibration.hint(),
        MotionHint::YawSpanDeg(span) if (span - 20.0).abs() <= ANGLE_EPS
    ));

    let mut calibration = calibration_at(CompassStep::InvertedSpin);
    for (index, yaw) in [0.0, 90.0, 180.0, 270.0, 360.0].iter().enumerate() {
        let timestamp_ms = u64::try_from(index).unwrap_or(0) * SAMPLE_INTERVAL_MS;
        calibration.observe(motion(timestamp_ms, *yaw, 0.0, 0.0));
    }
    assert!(matches!(
        calibration.hint(),
        MotionHint::YawSpanDeg(span) if (span - 360.0).abs() <= ANGLE_EPS
    ));
}

#[test]
fn tilt_hints_track_pitch_and_roll() {
    let mut calibration = calibration_at(CompassStep::NoseTilt);
    calibration.observe(motion(0, 0.0, -90.0, 0.0));
    calibration.observe(motion(100, 500.0, 90.0, 0.0));
    assert!(matches!(
        calibration.hint(),
        MotionHint::PitchSpanDeg(span) if (span - 180.0).abs() <= ANGLE_EPS
    ));

    let mut calibration = calibration_at(CompassStep::SideTilt);
    calibration.observe(motion(0, 0.0, 0.0, -45.0));
    calibration.observe(motion(100, 500.0, 0.0, 45.0));
    assert!(matches!(
        calibration.hint(),
        MotionHint::RollSpanDeg(span) if (span - 90.0).abs() <= ANGLE_EPS
    ));
}

#[test]
fn hold_hint_tracks_elapsed_stillness_and_ignores_jitter() {
    let mut calibration = CompassCalibration::new();
    assert!(matches!(calibration.hint(), MotionHint::StillMillis(0)));

    calibration.observe(motion(0, 0.0, 0.0, 0.0));
    calibration.observe(motion(500, 0.2, 0.0, 0.0));
    assert!(matches!(calibration.hint(), MotionHint::StillMillis(500)));

    calibration.observe(motion(800, 3.0, 0.0, 0.0));
    assert!(matches!(calibration.hint(), MotionHint::StillMillis(0)));

    calibration.observe(motion(1800, 3.2, 0.0, 0.0));
    assert!(matches!(calibration.hint(), MotionHint::StillMillis(1000)));
}

#[test]
fn hints_reset_on_step_change_and_on_redo() {
    let mut calibration = calibration_at(CompassStep::FlatSpin);
    calibration.observe(motion(0, 0.0, 0.0, 0.0));
    calibration.observe(motion(100, 90.0, 0.0, 0.0));
    assert!(matches!(
        calibration.hint(),
        MotionHint::YawSpanDeg(span) if (span - 90.0).abs() <= ANGLE_EPS
    ));

    // Done resets the tracker for the next step, so the pitch span starts from
    // the next sample rather than inheriting the yaw travel.
    assert!(calibration.done());
    assert!(matches!(
        calibration.hint(),
        MotionHint::PitchSpanDeg(span) if span.abs() <= ANGLE_EPS
    ));
    calibration.observe(motion(200, 700.0, 10.0, 0.0));
    calibration.observe(motion(300, 800.0, 20.0, 0.0));
    assert!(matches!(
        calibration.hint(),
        MotionHint::PitchSpanDeg(span) if (span - 10.0).abs() <= ANGLE_EPS
    ));

    // Redo clears the tracker even when a check step was accumulating samples.
    let mut calibration = calibration_at(CompassStep::NorthCheck);
    calibration.observe(motion(0, 0.0, 0.0, 0.0));
    calibration.observe(motion(100, 90.0, 0.0, 0.0));
    assert!(calibration.redo());
    assert!(matches!(calibration.hint(), MotionHint::StillMillis(0)));
    assert!(calibration.done());
    assert!(matches!(
        calibration.hint(),
        MotionHint::YawSpanDeg(span) if span.abs() <= ANGLE_EPS
    ));
}

#[test]
fn compass_heading_is_magnetic_and_wrapped() {
    assert_angle(compass_heading(0.0), 0.0);
    assert_angle(compass_heading(90.0), 90.0);
    assert_angle(compass_heading(360.0), 0.0);
    assert_angle(compass_heading(450.0), 90.0);
    assert_angle(compass_heading(-90.0), 270.0);
    assert_angle(compass_heading(-450.0), 270.0);

    // The mounting/sign offset is applied by subtraction: yaw at the offset is
    // north, and ±90° about that offset stays on the east/west axes.
    assert_angle(compass_heading(MOUNTING_OFFSET_DEG), MAGNETIC_NORTH_DEG);
    assert_angle(compass_heading(MOUNTING_OFFSET_DEG + 90.0), MAGNETIC_EAST_DEG);
    assert_angle(compass_heading(MOUNTING_OFFSET_DEG - 90.0), 270.0);
    assert_angle(compass_heading(MOUNTING_OFFSET_DEG + 180.0), 180.0);

    for yaw in [-720.0, -359.0, -1.0, 0.0, 123.4, 359.9] {
        let heading = compass_heading(yaw);
        assert!(
            (0.0..360.0).contains(&heading),
            "heading {heading} out of range for yaw {yaw}"
        );
        assert_angle(compass_heading(yaw + 360.0), heading);
    }
}

#[test]
fn check_waits_for_a_full_window_before_passing() {
    let mut check = CompassCheck::new();
    assert_eq!(check.target(), CompassTarget::MagneticNorth);
    assert_eq!(check.outcome(), CheckOutcome::Waiting);

    assert_eq!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY, 0, 1),
        CheckOutcome::Waiting
    );
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            100,
            WINDOW_SAMPLES - 2,
        ),
        CheckOutcome::Waiting
    );
    assert_eq!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY, 2000, 1),
        CheckOutcome::Pass
    );
}

#[test]
fn a_window_passes_with_sample_spacing_that_does_not_divide_it() {
    // The calibration runner polls every 100 ms plus its own work, so samples
    // arrive slightly more than 100 ms apart. The retained window must still
    // reach the full stability period instead of stopping one sample short.
    let mut check = CompassCheck::new();
    let mut outcome = check.outcome();
    for index in 0..=25_u64 {
        outcome = check.sample(CompassSample {
            timestamp_ms: index * 101,
            heading_deg: MAGNETIC_NORTH_DEG,
            accuracy: CALIBRATED_COMPASS_ACCURACY,
        });
    }
    assert_eq!(outcome, CheckOutcome::Pass);
}

#[test]
fn expiring_clears_the_window_but_keeps_the_target() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Pass
    );

    check.expire();
    assert_eq!(check.outcome(), CheckOutcome::Waiting);
    assert_eq!(check.target(), CompassTarget::MagneticNorth);
    assert!(!check.accept());

    // New evidence starts a fresh window rather than reusing the old one.
    assert_eq!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY, 3000, 1),
        CheckOutcome::Waiting
    );
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            3100,
            WINDOW_SAMPLES - 1
        ),
        CheckOutcome::Pass
    );
}

#[test]
fn a_gap_longer_than_the_window_restarts_the_evidence() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Pass
    );

    // The readings stopped for five seconds. The heading before the gap is not
    // evidence that it is held now, so this sample starts a new window.
    assert_eq!(
        check.sample(CompassSample {
            timestamp_ms: 5000,
            heading_deg: MAGNETIC_NORTH_DEG,
            accuracy: CALIBRATED_COMPASS_ACCURACY,
        }),
        CheckOutcome::Waiting
    );
}

#[test]
fn north_then_east_check_passes_and_completes() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Pass
    );
    assert!(check.accept());
    assert_eq!(check.target(), CompassTarget::MagneticEast);
    assert_eq!(check.outcome(), CheckOutcome::Waiting);

    // The window was cleared for east: one sample is not yet a full window.
    assert_eq!(
        feed(&mut check, MAGNETIC_EAST_DEG, CALIBRATED_COMPASS_ACCURACY, 3000, 1),
        CheckOutcome::Waiting
    );
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_EAST_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            3100,
            WINDOW_SAMPLES - 1,
        ),
        CheckOutcome::Pass
    );
    assert!(check.accept());
    assert!(check.is_complete());
    assert!(!check.accept());
}

#[test]
fn a_check_cannot_be_accepted_before_it_passes() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_EAST_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Fail(CheckFailure::Heading {
            measured_deg: MAGNETIC_EAST_DEG,
            target_deg: MAGNETIC_NORTH_DEG,
        })
    );
    assert!(!check.accept());
    assert_eq!(check.target(), CompassTarget::MagneticNorth);
}

#[test]
fn targets_are_evaluated_in_order_north_then_east() {
    let mut check = CompassCheck::new();
    assert!(matches!(
        feed(&mut check, MAGNETIC_EAST_DEG, CALIBRATED_COMPASS_ACCURACY, 0, WINDOW_SAMPLES),
        CheckOutcome::Fail(CheckFailure::Heading {
            measured_deg,
            target_deg,
        }) if (measured_deg - MAGNETIC_EAST_DEG).abs() <= ANGLE_EPS
            && (target_deg - MAGNETIC_NORTH_DEG).abs() <= ANGLE_EPS
    ));

    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            2100,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Pass
    );
    assert!(check.accept());
    assert_eq!(check.target(), CompassTarget::MagneticEast);

    assert!(matches!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY, 4200, WINDOW_SAMPLES),
        CheckOutcome::Fail(CheckFailure::Heading {
            measured_deg,
            target_deg,
        }) if (measured_deg - MAGNETIC_NORTH_DEG).abs() <= ANGLE_EPS
            && (target_deg - MAGNETIC_EAST_DEG).abs() <= ANGLE_EPS
    ));
    assert!(!check.accept());
    assert!(!check.is_complete());
}

#[test]
fn the_2_to_1_gain_case_passes_north_and_fails_east() {
    let mut check = CompassCheck::new();

    // Measured heading = 2 × true heading. At true north it reads 0°.
    assert_eq!(
        feed(&mut check, 0.0, CALIBRATED_COMPASS_ACCURACY, 0, WINDOW_SAMPLES),
        CheckOutcome::Pass
    );
    assert!(check.accept());

    // At true east it reads 180°, which must fail the 90° target.
    assert!(matches!(
        feed(&mut check, 180.0, CALIBRATED_COMPASS_ACCURACY, 3000, WINDOW_SAMPLES),
        CheckOutcome::Fail(CheckFailure::Heading {
            measured_deg,
            target_deg,
        }) if (measured_deg - 180.0).abs() <= ANGLE_EPS
            && (target_deg - MAGNETIC_EAST_DEG).abs() <= ANGLE_EPS
    ));
    assert!(!check.accept());
    assert!(!check.is_complete());
}

#[test]
fn tolerance_is_inclusive_at_ten_degrees() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG + CHECK_TOLERANCE_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES,
        ),
        CheckOutcome::Pass
    );

    let mut check = CompassCheck::new();
    assert!(matches!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG + CHECK_TOLERANCE_DEG + 0.01,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES,
        ),
        CheckOutcome::Fail(CheckFailure::Heading { .. })
    ));
}

#[test]
fn accuracy_below_three_fails_until_the_bad_sample_leaves_the_window() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY - 1, 0, 1),
        CheckOutcome::Fail(CheckFailure::Accuracy { measured: 2 })
    );

    // At t = 2000 ms the accuracy-2 sample is exactly one window old, so it is
    // still evidence against a converged engine.
    assert!(matches!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY, 100, 20),
        CheckOutcome::Fail(CheckFailure::Accuracy { measured: 2 })
    ));

    // Once it falls outside the window, the all-accuracy-3 window can pass.
    assert_eq!(
        feed(&mut check, MAGNETIC_NORTH_DEG, CALIBRATED_COMPASS_ACCURACY, 2100, 6),
        CheckOutcome::Pass
    );
}

#[test]
fn stability_requires_the_span_to_stay_within_two_degrees() {
    // Exactly 2° of span is accepted.
    let mut check = CompassCheck::new();
    for index in 0..=20_u64 {
        let heading = if index % 2 == 0 { 0.0 } else { CHECK_STABILITY_SPAN_DEG };
        let _ = check.sample(CompassSample {
            timestamp_ms: index * SAMPLE_INTERVAL_MS,
            heading_deg: heading,
            accuracy: CALIBRATED_COMPASS_ACCURACY,
        });
    }
    assert_eq!(check.outcome(), CheckOutcome::Pass);

    // A 2.5° swing over the window is unstable.
    let mut check = CompassCheck::new();
    let _ = feed(&mut check, 0.0, CALIBRATED_COMPASS_ACCURACY, 0, 11);
    assert_eq!(
        feed(&mut check, 2.5, CALIBRATED_COMPASS_ACCURACY, 1100, 10),
        CheckOutcome::Unstable
    );

    // Once the 0° samples age out, the steady 2.5° window passes.
    assert_eq!(
        feed(&mut check, 2.5, CALIBRATED_COMPASS_ACCURACY, 2100, 11),
        CheckOutcome::Pass
    );
}

#[test]
fn stability_span_wraps_across_north() {
    let mut check = CompassCheck::new();
    for index in 0..=20_u64 {
        let heading = if index % 2 == 0 { 359.5 } else { 0.5 };
        let _ = check.sample(CompassSample {
            timestamp_ms: index * SAMPLE_INTERVAL_MS,
            heading_deg: heading,
            accuracy: CALIBRATED_COMPASS_ACCURACY,
        });
    }
    assert_eq!(check.outcome(), CheckOutcome::Pass);
}

#[test]
fn out_of_order_and_post_completion_samples_are_ignored() {
    let mut check = CompassCheck::new();
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_NORTH_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            0,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Pass
    );

    assert_eq!(
        check.sample(CompassSample {
            timestamp_ms: 500,
            heading_deg: MAGNETIC_EAST_DEG,
            accuracy: 0,
        }),
        CheckOutcome::Pass
    );

    assert!(check.accept());
    assert_eq!(
        feed(
            &mut check,
            MAGNETIC_EAST_DEG,
            CALIBRATED_COMPASS_ACCURACY,
            2100,
            WINDOW_SAMPLES
        ),
        CheckOutcome::Pass
    );
    assert!(check.accept());
    assert!(check.is_complete());

    assert_eq!(
        check.sample(CompassSample {
            timestamp_ms: 9000,
            heading_deg: 180.0,
            accuracy: 0,
        }),
        CheckOutcome::Pass
    );
}

#[test]
fn snapshot_round_trips_nine_words_version_and_checksum() {
    let snapshot = DmpBiasSnapshot::new([1, -2, 3], [4, -5, 6], [7, -8, 9]);
    let encoded = snapshot.encode();

    assert_eq!(encoded.len(), DMP_BIAS_ENCODED_LEN);
    assert_eq!(encoded[0], DMP_BIAS_FORMAT_VERSION);

    let decoded = DmpBiasSnapshot::decode(&encoded);
    assert!(decoded.is_ok());
    if let Ok(decoded) = decoded {
        assert_eq!(decoded, snapshot);
        assert_eq!(decoded.gyro_bias(), [1, -2, 3]);
        assert_eq!(decoded.accel_bias(), [4, -5, 6]);
        assert_eq!(decoded.compass_bias(), [7, -8, 9]);
        assert_eq!(decoded.version(), DMP_BIAS_FORMAT_VERSION);
        assert_eq!(decoded.checksum(), snapshot.checksum());
    }
}

#[test]
fn snapshot_preserves_boundary_words() {
    let snapshot = DmpBiasSnapshot::new([i32::MIN, i32::MAX, 0], [i32::MIN, 0, i32::MAX], [-1, 1, i32::MIN]);
    let encoded = snapshot.encode();

    assert_eq!(DmpBiasSnapshot::decode(&encoded), Ok(snapshot));
}

#[test]
fn snapshot_words_are_big_endian_in_group_order() {
    let snapshot = DmpBiasSnapshot::new([0x0102_0304, 5, 0], [0, 0, 0], [0, 0, 0]);
    let encoded = snapshot.encode();

    assert_eq!(&encoded[1..5], &[0x01_u8, 0x02, 0x03, 0x04]);
    assert_eq!(&encoded[5..9], &[0x00_u8, 0x00, 0x00, 0x05]);
}

#[test]
fn snapshot_rejects_a_wrong_version_before_any_decode() {
    let encoded = DmpBiasSnapshot::new([0; 3], [0; 3], [0; 3]).encode();
    let mut corrupt = encoded;
    corrupt[0] = DMP_BIAS_FORMAT_VERSION + 1;

    assert!(matches!(
        DmpBiasSnapshot::decode(&corrupt),
        Err(SnapshotError::UnsupportedVersion { found }) if found == DMP_BIAS_FORMAT_VERSION + 1
    ));
}

#[test]
fn snapshot_rejects_a_corrupt_checksum() {
    let encoded = DmpBiasSnapshot::new([1, 2, 3], [4, 5, 6], [7, 8, 9]).encode();
    let mut corrupt = encoded;
    corrupt[10] ^= 0xFF;

    assert!(matches!(
        DmpBiasSnapshot::decode(&corrupt),
        Err(SnapshotError::ChecksumMismatch { .. })
    ));
}

#[test]
fn snapshot_rejects_truncated_and_oversized_input() {
    let encoded = DmpBiasSnapshot::new([1, 2, 3], [4, 5, 6], [7, 8, 9]).encode();

    let truncated = DMP_BIAS_ENCODED_LEN - 1;
    assert!(matches!(
        DmpBiasSnapshot::decode(&encoded[..truncated]),
        Err(SnapshotError::Length { actual }) if actual == truncated
    ));
    assert!(matches!(
        DmpBiasSnapshot::decode(&[]),
        Err(SnapshotError::Length { actual: 0 })
    ));

    let mut oversized = [0_u8; DMP_BIAS_ENCODED_LEN + 1];
    oversized[..DMP_BIAS_ENCODED_LEN].copy_from_slice(&encoded);
    assert!(matches!(
        DmpBiasSnapshot::decode(&oversized),
        Err(SnapshotError::Length { actual }) if actual == DMP_BIAS_ENCODED_LEN + 1
    ));
}
