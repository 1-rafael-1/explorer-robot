// Pure world-frame geometry: bearings, distances, the camera transform and the
// robot footprint. No DOM, no clock, so the headless suite can test it directly.
//
// Conventions (ADR-0038): distances are centimetres, +x is forward, +y is left,
// yaw is counter-clockwise positive. A LiDAR slot `i` is `i` degrees clockwise
// from the heading (slot 0 ahead); `first_deg`, `step_deg` and `clockwise` are
// honoured even though v1 always sends 0/1/true.

import { centidegrees, needsRotate } from './protocol.js';

/** The one degree/radian conversion the geometry and the renderer share. */
export const RAD = Math.PI / 180;

/** The robot's provisional world footprint, in centimetres (ADR-0043). */
export const FOOTPRINT = Object.freeze({ halfLengthCm: 9, halfWidthCm: 7 });

/** The minimum on-screen size of the robot marker, in CSS pixels. */
export const MIN_ICON_PX = 22;

/** The scan age after which a scan is drawn dimmed; display only, never a gate (ADR-0043). */
export const STALE_MS = 350;

/** The range step that marks an Occlusion Edge, in centimetres (ADR-0046). */
export const OCCLUSION_STEP_CM = 50;

/** The identity pose, used until odometry lands. */
export const IDENTITY_POSE = Object.freeze({ x: 0, y: 0, yaw: 0 });

/**
 * The robot icon's half extents in CSS pixels at `scale`.
 *
 * The icon is drawn at the true world footprint, grown uniformly only until its
 * longer half reaches `MIN_ICON_PX / 2`. The tap guard uses the same extents, so
 * what the operator sees is exactly what they cannot tap through (ADR-0043).
 */
export function iconFootprintPx(scale) {
  const safe = Math.max(scale, 1e-6);
  const trueHalfLength = FOOTPRINT.halfLengthCm * safe;
  const trueHalfWidth = FOOTPRINT.halfWidthCm * safe;
  const grow = Math.max(1, MIN_ICON_PX / 2 / Math.max(trueHalfLength, trueHalfWidth, 1e-6));
  return { halfLengthPx: trueHalfLength * grow, halfWidthPx: trueHalfWidth * grow };
}

/** Returns a scan's age in milliseconds, or `null` when `ts` is unknown. */
export function scanAgeMs(ts, now) {
  if (ts == null || !Number.isFinite(ts)) return null;
  return Math.max(0, now - ts);
}

/** Returns whether a scan is stale; the age is display truth and never a Go gate. */
export function isStaleScan(ts, now, staleMs = STALE_MS) {
  const age = scanAgeMs(ts, now);
  return age == null || age > staleMs;
}

/** Returns a point's distance from the pose, in centimetres. */
export function distanceCm(pose, point) {
  return Math.hypot(point.x - pose.x, point.y - pose.y);
}

/** Returns the world bearing to a point, in `[0, 360)` degrees. */
export function bearingDeg(pose, point) {
  const degrees = Math.atan2(point.y - pose.y, point.x - pose.x) / RAD;
  let value = degrees % 360;
  if (value < 0) value += 360;
  return value;
}

/** Returns the shortest turn from the pose's heading to a point, in degrees. */
export function relativeBearingDeg(pose, point) {
  const delta = bearingDeg(pose, point) - pose.yaw;
  let value = (delta + 180) % 360;
  if (value < 0) value += 360;
  return value - 180;
}

/** Returns the two-leg plan from `pose` to `target`, in world units. */
export function planLeg(pose, target) {
  const distance = distanceCm(pose, target);
  const bearing = bearingDeg(pose, target);
  const angle = relativeBearingDeg(pose, target);
  return {
    target: { x: target.x, y: target.y },
    pose: { x: pose.x, y: pose.y, yaw: pose.yaw },
    distance_cm: distance,
    bearing_deg: bearing,
    angle_deg: angle,
    rotate_centideg: centidegrees(angle),
    needs_rotate: needsRotate(angle),
  };
}

/** Returns whether a world point is inside the robot's drawn footprint. */
export function pointInFootprint(pose, point, scale = 1) {
  const dx = point.x - pose.x;
  const dy = point.y - pose.y;
  const cos = Math.cos(pose.yaw * RAD);
  const sin = Math.sin(pose.yaw * RAD);
  const forward = dx * cos + dy * sin;
  const left = -dx * sin + dy * cos;
  const safe = Math.max(scale, 1e-6);
  const { halfLengthPx, halfWidthPx } = iconFootprintPx(safe);
  return Math.abs(forward) <= halfLengthPx / safe && Math.abs(left) <= halfWidthPx / safe;
}

/** Builds a camera view over a canvas of `width` by `height` CSS pixels. */
export function cameraView({ width, height, scale, centerX, centerY }) {
  return {
    width,
    height,
    scale,
    centerX,
    centerY,
    cx: width / 2,
    cy: height / 2,
  };
}

/** Projects a world point into canvas CSS pixels. */
export function worldToScreen(view, point) {
  return {
    x: view.cx + (point.x - view.centerX) * view.scale,
    y: view.cy - (point.y - view.centerY) * view.scale,
  };
}

/** Unprojects a canvas CSS pixel into the world frame. */
export function screenToWorld(view, screenX, screenY) {
  return {
    x: view.centerX + (screenX - view.cx) / view.scale,
    y: view.centerY - (screenY - view.cy) / view.scale,
  };
}

/** Returns a camera scale that fits `rangeCm` into the canvas' shorter axis. */
export function fitScale(width, height, rangeCm = 460) {
  return (Math.min(width, height) * 0.94) / 2 / Math.max(rangeCm, 1);
}

/** Returns the world angle of a LiDAR slot in degrees. */
export function slotAngleDeg(pose, firstDeg, stepDeg, clockwise, slot) {
  const direction = clockwise ? -1 : 1;
  return pose.yaw + direction * (firstDeg + slot * stepDeg);
}

/**
 * Returns the newest pose sample at or before `ts`, or `null`.
 *
 * The World Frame rule is that a scan is paired with the pose at its own
 * timestamp, never a newer pose (ADR-0038). v1 has no pose history and the
 * reserved `pose` stream is absent, so this returns `null` and the renderer
 * honestly falls back to the identity pose. The seam exists so a future pose
 * stream can supply the sample without a rewrite.
 */
export function poseAt(poses, ts) {
  if (!Array.isArray(poses) || ts == null) return null;
  let best = null;
  for (const sample of poses) {
    if (sample?.ts == null || sample.ts > ts) continue;
    if (!best || sample.ts > best.ts) best = sample;
  }
  return best;
}

/**
 * Projects a `lidar` stream value into world points paired with `pose`.
 *
 * A `null` range is no return and produces no point, so the caller draws an
 * honest gap rather than a guessed line.
 */
export function scanPoints(pose, value) {
  const points = [];
  const ranges = value?.ranges_cm ?? [];
  const first = value?.first_deg ?? 0;
  const step = value?.step_deg ?? 1;
  const clockwise = value?.clockwise ?? true;
  for (let slot = 0; slot < ranges.length; slot += 1) {
    const range = ranges[slot];
    if (range == null) continue;
    const angle = slotAngleDeg(pose, first, step, clockwise, slot) * RAD;
    points.push({
      slot,
      range,
      x: pose.x + range * Math.cos(angle),
      y: pose.y + range * Math.sin(angle),
    });
  }
  return points;
}

/**
 * Classifies a `lidar` stream value into Occlusion Shadows (ADR-0046).
 *
 * A shadow is a near segment: a run of valid returns bounded in slot order by
 * a fall and the next rise, where each step is at least `OCCLUSION_STEP_CM`,
 * the run holds at least two adjacent returns, and the run is shorter than
 * half the scan. A `null` range is a No Return and disqualifies a candidate:
 * it is unknown, not far. The scan is walked cyclically, so a protrusion
 * across slot 0 is found and kept in slot order.
 *
 * Each shadow carries the segment's measured contour in world centimetres and
 * the cap range: the greater of the two flanking far returns. This is display
 * geometry only; the caller recomputes it on every render.
 */
export function scanShadows(pose, value) {
  const ranges = value?.ranges_cm ?? [];
  const count = ranges.length;
  if (count === 0) return [];

  const first = value?.first_deg ?? 0;
  const step = value?.step_deg ?? 1;
  const clockwise = value?.clockwise ?? true;
  const valid = (slot) => ranges[slot] != null;

  // The slot distance forward from `from` to `to` around the closed scan.
  const advance = (from, to) => (to - from + count) % count;

  // Walks forward from `start` to the first rise, bounded by the fall that
  // started the segment. A No Return on the way breaks the candidate instead
  // of being skipped, and the rise needs a valid far return after it.
  const firstRise = (start) => {
    let slot = start;
    for (let edge = 0; edge < count - 1; edge += 1) {
      if (!valid(slot)) return null;
      const next = (slot + 1) % count;
      if (!valid(next)) return null;
      if (ranges[next] - ranges[slot] >= OCCLUSION_STEP_CM) return slot;
      slot = next;
    }
    return null;
  };

  const candidates = [];
  for (let slot = 0; slot < count; slot += 1) {
    const next = (slot + 1) % count;
    if (!valid(slot) || !valid(next)) continue;
    if (ranges[slot] - ranges[next] < OCCLUSION_STEP_CM) continue;
    const rise = firstRise(next);
    if (rise == null) continue;
    const span = advance(slot, rise);
    // A one-slot segment has coincident Occlusion Edges; a segment at least
    // half the scan leaves the far recess as the near background, not a
    // protrusion.
    if (span < 2 || span * 2 >= count) continue;
    candidates.push({ fall: slot, rise, span });
  }

  // A fall inside another candidate's near segment is the same protrusion, so
  // only the outermost fall of each run emits a shadow.
  const outer = candidates.filter(
    (candidate) =>
      !candidates.some(
        (other) => other !== candidate && advance(other.fall, candidate.fall) <= other.span,
      ),
  );

  const worldPoint = (slot) => {
    const angle = slotAngleDeg(pose, first, step, clockwise, slot) * RAD;
    const range = ranges[slot];
    return {
      slot,
      range,
      x: pose.x + range * Math.cos(angle),
      y: pose.y + range * Math.sin(angle),
    };
  };

  return outer.map((candidate) => {
    const silhouette = [];
    for (let offset = 1; offset <= candidate.span; offset += 1) {
      silhouette.push(worldPoint((candidate.fall + offset) % count));
    }
    const farRight = ranges[(candidate.rise + 1) % count];
    return {
      silhouette,
      capCm: Math.max(ranges[candidate.fall], farRight),
    };
  });
}
