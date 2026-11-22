//! Host-side integration tests for the pure `SetTracks` resolution.
//!
//! Every assertion is made through the crate's public API. The derived-curve
//! case builds its curve from literal measured series and then checks the
//! resolution that the same API promises, so the test never re-runs the
//! derivation it is testing.

use motor_cal::{Curve, CurveError, Direction, LegPoint, POWER_STEPS, Sweep, Track, derive_curve, resolve_track_speed};

/// The Measurement Window length used by the derived curve, in milliseconds.
const WINDOW_MS: u64 = 2_000;

/// The ten Power Steps of a Test Leg, in percent.
const PERCENTS: [u8; POWER_STEPS] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

/// A linear leg: 10 counts per second per percent.
const LINEAR: [u32; POWER_STEPS] = [100, 200, 300, 400, 500, 600, 700, 800, 900, 1000];

/// A leg that reaches 555 counts per second at 60 %, so the faster forward left
/// leg resolves that request to 555 per-mille.
const ATTENUATED: [u32; POWER_STEPS] = [92, 185, 277, 370, 462, 555, 647, 740, 832, 925];

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

/// Build a Test Leg from ten speeds; every Power Step moved.
fn moving_leg(speeds: [u32; POWER_STEPS]) -> Vec<LegPoint> {
    PERCENTS
        .iter()
        .copied()
        .zip(speeds)
        .map(|(percent, speed)| moving(percent, speed))
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

/// A curve whose forward right leg is attenuated, so the forward left leg
/// resolves a 60 % request to 555 per-mille.
fn attenuated_curve() -> Result<Curve, CurveError> {
    let left_forward = moving_leg(LINEAR);
    let left_reverse = moving_leg(LINEAR);
    let right_forward = moving_leg(ATTENUATED);
    let right_reverse = moving_leg(LINEAR);
    derive_curve(sweep(&left_forward, &left_reverse, &right_forward, &right_reverse))
}

#[test]
fn a_missing_curve_is_the_identity() {
    assert_eq!(resolve_track_speed(None, Track::Left, 100, 1.0), 1000);
    assert_eq!(resolve_track_speed(None, Track::Right, -50, 1.0), -500);
    assert_eq!(resolve_track_speed(None, Track::Left, 0, 1.0), 0);
}

#[test]
fn voltage_compensation_scales_the_resolved_per_mille_and_clamps() {
    assert_eq!(resolve_track_speed(None, Track::Left, 50, 0.5), 250);
    assert_eq!(resolve_track_speed(None, Track::Left, 50, 2.0), 1000);
    assert_eq!(resolve_track_speed(None, Track::Left, 50, 0.25), 125);
}

#[test]
fn a_derived_curve_resolves_a_known_point() {
    let curve = attenuated_curve();

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 60).per_mille(), 555);
        assert_eq!(resolve_track_speed(Some(&curve), Track::Left, 60, 1.0), 555);
    }
}

#[test]
fn the_compensated_duty_is_rounded_once_away_from_zero() {
    let curve = attenuated_curve();

    assert!(curve.is_ok());
    if let Ok(curve) = curve {
        assert_eq!(curve.resolve(Track::Left, Direction::Forward, 60).per_mille(), 555);
        // 555 × 1.5 = 832.5, rounded once half away from zero to 833.
        assert_eq!(resolve_track_speed(Some(&curve), Track::Left, 60, 1.5), 833);
    }
}

#[test]
fn a_negative_request_scales_the_magnitude_then_negates() {
    assert_eq!(resolve_track_speed(None, Track::Right, -50, 0.5), -250);
    assert_eq!(resolve_track_speed(None, Track::Left, -100, 2.0), -1000);
}
