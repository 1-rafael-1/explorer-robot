//! Firmware-side `LiDAR` cloud boundary.
//!
//! This crate is the one place the COIN-D6 driver's millimetre scan becomes the
//! firmware's centimetre point cloud, and the one place the obstacle rule is
//! defined. It is `#![no_std]` and HAL-free so it can run on the robot's second
//! core and be exercised by host tests.
//!
//! # The cloud
//!
//! A [`Cloud`] is [`SLOTS`] one-degree bins of an optional distance in
//! centimetres. Slot `0` is dead ahead and increasing slots run clockwise,
//! matching the bench radar example (ADR-0012). A bin
//! with no valid return is `None` — never a distance.
//!
//! The angular reduction itself is **not new code**: [`Cloud::from_reduction`]
//! maps the driver's by-angle [`Reduction`] — produced by the firmware at a
//! one-degree bucket — into the grid, so the driver's nearest-valid-return rule
//! *is* the grid. The native scan type never reaches this crate.
//!
//! # No return is not clear
//!
//! A missing return is not an obstacle (it cannot phantom-brake the robot) and
//! it is not a clear angle either (a blind spot must never extend a navigable
//! gap). [`front_sector_obstacle`] ignores `None`; [`is_clear`] treats it as
//! **not** clear. The two give deliberately opposite answers to the same input.

#![no_std]
#![warn(missing_docs)]

use coin_d6::Reduction;

/// Number of one-degree slots in a [`Cloud`].
pub const SLOTS: usize = 360;

/// Angular width of one [`Cloud`] slot, in degrees.
///
/// The cloud is a one-degree grid, so this is also the reduction resolution a
/// caller must reduce scans onto before [`Cloud::from_reduction`]. Reducing onto
/// any other grid would place returns in buckets that do not match the slot
/// convention, so the grid has this one home rather than being restated at each
/// call site. [`SLOTS`] slots of this width cover the full circle.
pub const SLOT_WIDTH_DEG: f32 = 1.0;

/// The COIN-D6's mounting rotation, in degrees.
///
/// The sensor's native bearings are rotated by this to bring slot `0` to dead
/// ahead. This is the single place the mounting rotation is applied: the radar
/// widget plots the neutral slot order and carries no correction of its own
/// (ADR-0012). Confirm on the robot when the `LiDAR` is mounted.
pub const MOUNTING_OFFSET_DEG: f32 = 180.0;

/// Centre of the Front Sector, in degrees relative to dead ahead.
pub const FRONT_SECTOR_CENTRE_DEG: f32 = 0.0;

/// Half-angle of the Front Sector, in degrees. The stop test sweeps
/// ±this about [`FRONT_SECTOR_CENTRE_DEG`].
pub const FRONT_SECTOR_HALF_ANGLE_DEG: f32 = 45.0;

/// Distance at or below which a return inside the Front Sector is an obstacle,
/// in centimetres.
pub const FRONT_SECTOR_THRESHOLD_CM: f32 = 30.0;

/// Millimetres in one centimetre.
const MM_PER_CM: f32 = 10.0;

/// A 360-slot, one-degree `LiDAR` point cloud in centimetres.
///
/// Slot `0` is dead ahead; increasing slots run clockwise. A slot is
/// `None` when the sensor measured no valid return at that bearing.
#[derive(Debug, Clone)]
pub struct Cloud {
    /// Distance per slot in centimetres; `None` is no return.
    distances_cm: [Option<f32>; SLOTS],
    /// Monotonically increasing scan sequence number.
    sequence: u64,
}

impl Cloud {
    /// Build a cloud directly from per-slot distances, in centimetres.
    ///
    /// This is for known-geometry clouds: the crate's host tests build fixed
    /// scenes with it, since [`Cloud::from_reduction`] only accepts a real driver
    /// reduction. Real scans go through [`Cloud::from_reduction`].
    #[must_use]
    pub const fn from_slots(distances_cm: [Option<f32>; SLOTS], sequence: u64) -> Self {
        Self { distances_cm, sequence }
    }

    /// Map a by-angle reduction into a cloud, applying the mounting offset.
    ///
    /// `reduction` is the driver's [`Reduction`] over a one-degree bucket,
    /// produced by the firmware as the grid the robot reasons in. Each bucket's
    /// millimetres become centimetres at this single boundary. Native bearing `b`
    /// lands in slot `(b - MOUNTING_OFFSET_DEG)` so slot zero is dead ahead; read
    /// the other way, slot `slot` reads native bucket
    /// `(slot + MOUNTING_OFFSET_DEG) mod SLOTS`.
    ///
    /// The bucket count is [`SLOTS`] by construction, so a mismatch is a compile
    /// error rather than a panic. A bucket past the reduction's emitted grid
    /// (`len`) degrades to a no-return rather than being read as default data.
    #[must_use]
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    pub fn from_reduction(reduction: &Reduction<SLOTS>, sequence: u64) -> Self {
        let mut distances_cm = [None; SLOTS];
        for (slot, out) in distances_cm.iter_mut().enumerate() {
            // `slot` and the offset are both within a few hundred, so the cast
            // to `f32` is exact. `f32::round` is unavailable in `no_std`, so add
            // a half and truncate; the result is in `0..360`.
            let native = normalise_angle(slot as f32 + MOUNTING_OFFSET_DEG) + 0.5;
            let native = native as usize % SLOTS;
            if native < reduction.len
                && let Some(distance_mm) = reduction.points[native].distance_mm
            {
                *out = Some(f32::from(distance_mm.get()) / MM_PER_CM);
            }
        }

        Self { distances_cm, sequence }
    }

    /// The distance in slot `slot`, in centimetres, or `None` for no return.
    ///
    /// Returns `None` for an out-of-range slot as well as for a missing return.
    #[must_use]
    pub fn distance_cm(&self, slot: usize) -> Option<f32> {
        self.distances_cm.get(slot).copied().flatten()
    }

    /// All slots, index `0`..[`SLOTS`], in centimetres; `None` is no return.
    #[must_use]
    pub const fn slots(&self) -> &[Option<f32>; SLOTS] {
        &self.distances_cm
    }

    /// The scan sequence number this cloud was published with.
    #[must_use]
    pub const fn sequence(&self) -> u64 {
        self.sequence
    }

    /// Whether this cloud reports an obstacle in the robot's Front Sector.
    ///
    /// Uses the named [`FRONT_SECTOR_CENTRE_DEG`],
    /// [`FRONT_SECTOR_HALF_ANGLE_DEG`] and [`FRONT_SECTOR_THRESHOLD_CM`]; the
    /// rule itself is [`front_sector_obstacle`].
    #[must_use]
    pub fn front_sector_obstacle(&self) -> bool {
        front_sector_obstacle(
            self,
            FRONT_SECTOR_CENTRE_DEG,
            FRONT_SECTOR_HALF_ANGLE_DEG,
            FRONT_SECTOR_THRESHOLD_CM,
        )
    }
}

impl Default for Cloud {
    /// A cloud with every slot a no-return and sequence `0`.
    fn default() -> Self {
        Self {
            distances_cm: [None; SLOTS],
            sequence: 0,
        }
    }
}

/// Whether a forward cone about `centre_deg` contains a return at or below
/// `threshold_cm`.
///
/// This is the single source of truth for the Coast-and-Avoid stop rule. It
/// sweeps the closed angular window `centre_deg ± half_angle_deg`, including
/// both boundary angles, and reports an obstacle for any slot whose distance is
/// `Some(d)` with `d <= threshold_cm`.
///
/// A missing return (`None`) is **not** an obstacle: a dropout cannot
/// phantom-brake the robot. See [`is_clear`] for the deliberately opposite
/// reading gap selection uses.
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn front_sector_obstacle(cloud: &Cloud, centre_deg: f32, half_angle_deg: f32, threshold_cm: f32) -> bool {
    let centre = normalise_angle(centre_deg);
    let half = half_angle_deg.abs();

    for (slot, distance) in cloud.distances_cm.iter().enumerate() {
        // Signed angular offset from the centre, in (-180, 180], so a cone that
        // straddles 0° needs no special case.
        let bearing = slot as f32;
        let offset = normalise_angle(bearing - centre + 180.0) - 180.0;
        if offset.abs() > half {
            continue;
        }
        if let Some(distance) = distance
            && *distance <= threshold_cm
        {
            return true;
        }
    }

    false
}

/// Coerce an angle into `[0, 360)`.
///
/// `f32` has no `rem_euclid` in `no_std`, so the wrap is done explicitly.
fn normalise_angle(angle_deg: f32) -> f32 {
    let wrapped = angle_deg % 360.0;
    if wrapped < 0.0 { wrapped + 360.0 } else { wrapped }
}

/// Whether a slot's distance is a clear angle at `threshold_cm`.
///
/// A return farther than the threshold is clear; a return at or below it is an
/// obstacle; a missing return (`None`) is **not** clear. This deliberately
/// inverts the old zero-sentinel reading, which treated a dropout as drivable:
/// the `LiDAR` returns no reading for a black or specular surface as readily as
/// for open space, so "clear" must mean "we actually measured past it".
#[must_use]
pub fn is_clear(distance_cm: Option<f32>, threshold_cm: f32) -> bool {
    matches!(distance_cm, Some(distance) if distance > threshold_cm)
}
