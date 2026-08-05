//! Gap analysis for `LiDAR` point cloud.
//!
//! Pure, stateless algorithm that analyzes a 360-point `LiDAR` scan and
//! returns the best gap to drive through within the forward cone.
//!
//! Operates on a full 360° point cloud (`distances: [f32; 360]`) with
//! world-relative angles (0° = forward). Uses widest-gap selection within
//! the forward cone (±60°).

use crate::system::state::perception::LidarPointCloud;

// ── Constants ───────────────────────────────────────────────────────────────

/// Distance threshold (cm). Points closer than this are obstacles;
/// points farther or zero (no return) are "clear".
const OBSTACLE_THRESHOLD_CM: f32 = 40.0;

/// Half-angle of the forward cone (± degrees from heading 0°).
const FORWARD_CONE_HALF_DEG: f32 = 60.0;

/// Minimum angular width (degrees) for a gap to be considered viable.
const MIN_GAP_WIDTH_DEG: f32 = 10.0;

/// Default driving distance per leg (cm).
const DEFAULT_LEG_DISTANCE_CM: f32 = 40.0;

/// Safety margin subtracted from constriction depth (cm).
const SAFETY_MARGIN_CM: f32 = 10.0;

// ── Types ───────────────────────────────────────────────────────────────────

/// Result of gap analysis: a chosen gap to navigate through.
#[derive(Debug, Clone, Copy)]
pub struct GapDecision {
    /// Center angle of the chosen gap (degrees, 0 = forward, positive = left/CCW).
    pub gap_center_deg: f32,
    /// Absolute rotation needed to face the gap center (degrees).
    pub rotation_degrees: f32,
    /// Whether to rotate clockwise to align with the gap.
    pub clockwise: bool,
    /// Distance to drive through this gap (cm), capped at remaining target.
    pub drive_distance_cm: f32,
}

/// Internal representation of a contiguous clear arc (gap).
#[derive(Debug, Clone, Copy)]
struct Gap {
    /// Start angle in degrees (inclusive, 0–359).
    start_deg: u16,
    /// End angle in degrees (inclusive, 0–359).
    end_deg: u16,
    /// Minimum distance reading within the gap (cm).
    min_distance_cm: f32,
}

impl Gap {
    /// Angular span in degrees.
    fn span_deg(&self) -> f32 {
        let start = f32::from(self.start_deg);
        let end = f32::from(self.end_deg);
        if end >= start {
            end - start + 1.0
        } else {
            // Wrap-around gap.
            (360.0 - start) + end + 1.0
        }
    }

    /// Center angle in degrees (0–360, 0 = forward).
    fn center_deg(&self) -> f32 {
        let span = self.span_deg();
        let half = span / 2.0;
        let raw = f32::from(self.start_deg) + half;
        if raw >= 360.0 { raw - 360.0 } else { raw }
    }

    /// Returns true if the gap center is within the forward cone (± `cone_half_deg`
    /// from 0° heading), handling wrap-around.
    fn is_in_forward_cone(&self, cone_half_deg: f32) -> bool {
        let center = self.center_deg();
        let forward_diff = if center > 180.0 { 360.0 - center } else { center };
        forward_diff <= cone_half_deg
    }
}

// ── Main algorithm ──────────────────────────────────────────────────────────

/// Analyze the `LiDAR` point cloud and return the best gap within the forward
/// cone, or `None` if no viable gap exists.
///
/// # Arguments
/// * `cloud` — The `LiDAR` point cloud (360 distances, index = angle in degrees).
/// * `remaining_target_cm` — Remaining distance to the target in cm.
///
/// # Returns
/// * `Some(GapDecision)` if a viable gap exists within the forward cone.
/// * `None` if no gap is wide enough or within the forward cone.
#[must_use]
pub fn analyze_gaps(cloud: &LidarPointCloud, remaining_target_cm: f32) -> Option<GapDecision> {
    // ── Extract contiguous clear gaps ───────────────────────────────────
    // A point is "clear" when distance > threshold or distance == 0 (no return).
    let mut gaps: heapless::Vec<Gap, 64> = heapless::Vec::new();

    let mut i: usize = 0;
    while i < 360 {
        if !is_clear(cloud.distances[i]) {
            i += 1;
            continue;
        }

        #[allow(clippy::cast_possible_truncation)]
        let start = i as u16;
        let mut min_distance = f32::MAX;

        // Walk the clear run.
        while i < 360 && is_clear(cloud.distances[i]) {
            let d = cloud.distances[i];
            if d > 0.0 {
                min_distance = min_distance.min(d);
            }
            i += 1;
        }

        #[allow(clippy::cast_possible_truncation)]
        let end = (i - 1) as u16;

        let _ = gaps.push(Gap {
            start_deg: start,
            end_deg: end,
            min_distance_cm: if (min_distance - f32::MAX).abs() < f32::EPSILON {
                f32::MAX
            } else {
                min_distance
            },
        });
    }

    // Merge first and last if they both touch boundaries (wrap-around).
    if gaps.len() >= 2 {
        let first_touches_0 = gaps[0].start_deg == 0;
        let last_touches_359 = gaps.last().is_some_and(|g| g.end_deg == 359);

        if first_touches_0 && last_touches_359 {
            // Remove last, extend first to wrap around.
            if let Some(last) = gaps.pop() {
                gaps[0].start_deg = last.start_deg;
                gaps[0].min_distance_cm = gaps[0].min_distance_cm.min(last.min_distance_cm);
            }
        }
    }

    if gaps.is_empty() {
        return None;
    }

    // ── Select widest gap within forward cone ───────────────────────────
    let mut best_gap: Option<&Gap> = None;
    let mut best_span: f32 = 0.0;

    for gap in &gaps {
        if !gap.is_in_forward_cone(FORWARD_CONE_HALF_DEG) {
            continue;
        }

        let span = gap.span_deg();
        if span < MIN_GAP_WIDTH_DEG {
            continue;
        }

        if span > best_span {
            best_span = span;
            best_gap = Some(gap);
        }
    }

    let gap = best_gap?;

    // ── Build decision ─────────────────────────────────────────────────
    let center = gap.center_deg();

    // Determine rotation: center in [0, 360] with 0 = forward.
    // - center 0–180: gap is on the left side → turn CCW (clockwise = false)
    // - center 180–360: gap is on the right side → turn CW (clockwise = true)
    let (rotation_degrees, clockwise) = if center <= 180.0 {
        (center, false) // Turn CCW (left)
    } else {
        (360.0 - center, true) // Turn CW (right)
    };

    // Cap drive distance at the constriction depth minus a safety margin.
    let available = if (gap.min_distance_cm - f32::MAX).abs() < f32::EPSILON {
        DEFAULT_LEG_DISTANCE_CM
    } else {
        (gap.min_distance_cm - SAFETY_MARGIN_CM).max(0.0)
    };
    let drive_distance_cm = available.min(DEFAULT_LEG_DISTANCE_CM).min(remaining_target_cm).max(0.0);

    Some(GapDecision {
        gap_center_deg: center,
        rotation_degrees,
        clockwise,
        drive_distance_cm,
    })
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// A point is "clear" when no obstacle is within the threshold distance,
/// or when no return was measured (distance == 0.0).
fn is_clear(distance_cm: f32) -> bool {
    distance_cm == 0.0 || distance_cm > OBSTACLE_THRESHOLD_CM
}
