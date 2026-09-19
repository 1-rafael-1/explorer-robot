//! Host-side integration tests for the `lidar-cloud` crate.
//!
//! These exercise the driver-scan-to-cloud mapping (millimetre conversion and
//! mounting offset), the no-return representation, the Front Sector stop test at
//! its boundary angles and threshold, and the deliberately opposite readings the
//! two consumers give to a missing return.

use core::num::NonZeroU16;

use coin_d6::{Point, Scan};
use lidar_cloud::{
    Cloud, FRONT_SECTOR_HALF_ANGLE_DEG, FRONT_SECTOR_THRESHOLD_CM, SLOTS, front_sector_obstacle, is_clear,
};

/// Build a [`Scan`] from `(angle_deg, distance_mm)` pairs; a distance of `0`
/// becomes a no-return.
fn build_scan(points: &[(f32, u16)]) -> Scan<400> {
    let mut scan = Scan::<400>::new();
    scan.len = points.len();
    for (i, &(angle_deg, distance_mm)) in points.iter().enumerate() {
        scan.points[i] = Point {
            angle_deg,
            distance_mm: NonZeroU16::new(distance_mm),
            intensity: 1,
        };
    }
    scan
}

/// A cloud with every slot a no-return.
const fn empty_cloud() -> Cloud {
    Cloud::from_slots([None; SLOTS], 0)
}

/// A cloud with `distance_cm` at each listed slot and no-return elsewhere.
fn cloud_with(slots: &[(usize, f32)]) -> Cloud {
    let mut distances = [None; SLOTS];
    for &(slot, distance_cm) in slots {
        distances[slot] = Some(distance_cm);
    }
    Cloud::from_slots(distances, 0)
}

#[test]
fn mapping_from_a_synthetic_scan_places_returns_in_their_slots() {
    // Native bearing 180 is dead ahead after the 180° mounting offset.
    let cloud = Cloud::from_spin(&build_scan(&[(180.0, 1000)]), 7);

    assert_eq!(cloud.sequence(), 7);
    assert_eq!(cloud.distance_cm(0), Some(100.0));
    // Every other slot is a no-return, not a distance.
    assert_eq!(cloud.distance_cm(1), None);
    assert_eq!(cloud.distance_cm(SLOTS - 1), None);
}

#[test]
fn millimetres_become_centimetres_at_the_one_boundary() {
    let cloud = Cloud::from_spin(&build_scan(&[(180.0, 1234)]), 0);

    assert_eq!(cloud.distance_cm(0), Some(123.4));
}

#[test]
fn mounting_offset_rotates_native_bearings() {
    // Slot = native bearing - 180°, modulo 360.
    let cloud = Cloud::from_spin(&build_scan(&[(0.0, 500), (225.0, 800), (270.0, 900)]), 0);

    assert_eq!(cloud.distance_cm(180), Some(50.0));
    assert_eq!(cloud.distance_cm(45), Some(80.0));
    assert_eq!(cloud.distance_cm(90), Some(90.0));
}

#[test]
fn a_bucket_with_no_valid_return_is_none_not_zero() {
    // The scan has no point near dead ahead, so slot 0 must be `None`.
    let cloud = Cloud::from_spin(&build_scan(&[(45.0, 1000)]), 0);

    assert_eq!(cloud.distance_cm(0), None);
    assert_eq!(cloud.distance_cm(225), Some(100.0));
}

#[test]
fn front_sector_includes_its_boundary_angles() {
    // A return exactly at +45° (slot 45) and exactly at -45° (slot 315) count.
    let half = 45usize;

    // A return exactly at +45° (slot 45) and exactly at -45° (slot 315) count.
    assert!(front_sector_obstacle(&cloud_with(&[(half, 10.0)]), 0.0, 45.0, 30.0));
    assert!(front_sector_obstacle(
        &cloud_with(&[(SLOTS - half, 10.0)]),
        0.0,
        45.0,
        30.0
    ));
}

#[test]
fn front_sector_excludes_just_outside_the_boundary() {
    // One degree outside ±45° is not in the sector.
    assert!(!front_sector_obstacle(&cloud_with(&[(46, 10.0)]), 0.0, 45.0, 30.0));
    assert!(!front_sector_obstacle(
        &cloud_with(&[(SLOTS - 46, 10.0)]),
        0.0,
        45.0,
        30.0
    ));
}

#[test]
fn front_sector_threshold_is_inclusive_and_ignores_returns_beyond() {
    assert!(front_sector_obstacle(&cloud_with(&[(0, 30.0)]), 0.0, 45.0, 30.0));
    assert!(!front_sector_obstacle(&cloud_with(&[(0, 30.1)]), 0.0, 45.0, 30.0));
    assert!(!front_sector_obstacle(&cloud_with(&[(0, 200.0)]), 0.0, 45.0, 30.0));
}

#[test]
fn a_missing_return_cannot_trigger_the_stop_test() {
    let cloud = empty_cloud();

    assert!(!cloud.front_sector_obstacle());
    assert!(!front_sector_obstacle(
        &cloud,
        0.0,
        FRONT_SECTOR_HALF_ANGLE_DEG,
        FRONT_SECTOR_THRESHOLD_CM
    ));
}

#[test]
fn gap_selection_treats_a_missing_return_as_not_clear() {
    // The two consumers deliberately disagree: no return is neither an obstacle
    // (the stop test above) nor a clear angle (here).
    assert!(!is_clear(None, 40.0));
    assert!(is_clear(Some(40.1), 40.0));
    assert!(!is_clear(Some(40.0), 40.0));
    assert!(!is_clear(Some(5.0), 40.0));
}

#[test]
fn the_named_constants_bind_the_stop_rule() {
    // A return inside the named sector at the named threshold reports an
    // obstacle through the convenience method.
    assert!(cloud_with(&[(0, FRONT_SECTOR_THRESHOLD_CM)]).front_sector_obstacle());
    assert!(!cloud_with(&[(0, FRONT_SECTOR_THRESHOLD_CM + 1.0)]).front_sector_obstacle());
}
