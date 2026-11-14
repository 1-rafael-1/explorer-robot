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

/** The minimum range step that marks an Occlusion Edge, in centimetres (ADR-0047). */
export const OCCLUSION_EDGE_MIN_CM = 20;

/** The relative range step that marks an Occlusion Edge, against the nearer return. */
export const OCCLUSION_EDGE_RATIO = 0.25;

/** Returns at or under this range are drawn as bright near-return dots. */
export const NEAR_RETURN_CM = 45;

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
 * Projects one slot and range into the World Frame.
 *
 * The one place the slot convention becomes a world point, shared by the scan
 * and the Occlusion Shadow contour so the two cannot drift apart.
 */
function scanPoint(pose, firstDeg, stepDeg, clockwise, slot, range) {
  const angle = slotAngleDeg(pose, firstDeg, stepDeg, clockwise, slot) * RAD;
  return {
    slot,
    range,
    x: pose.x + range * Math.cos(angle),
    y: pose.y + range * Math.sin(angle),
  };
}

/**
 * Builds the render model for one scan (ADR-0047): the projected returns, the
 * Seen Polygon, the significant Occlusion Edges and the No Return notches.
 *
 * The Seen Polygon is the star-shaped region from the scan pose out to each
 * measured return, in slot order. A No Return is not free space: it contributes
 * a radial notch to the pose, so that sector stays unseen. The Occlusion Shadow
 * is the complement of this polygon; the renderer fills it. Each notch is
 * returned as the two measured returns that flank it, so the renderer can dot
 * the closing chord.
 *
 * An Occlusion Edge is an adjacent valid pair whose range step is at least the
 * greater of `OCCLUSION_EDGE_MIN_CM` and `OCCLUSION_EDGE_RATIO` of the nearer
 * return. The chord between those returns is not a measured surface, so the
 * renderer dots it and never draws it as a solid return.
 */
export function buildScan(pose, value) {
  const ranges = value?.ranges_cm ?? [];
  const count = ranges.length;
  if (count === 0) return { points: [], seen: [], edges: [], notches: [] };

  const first = value?.first_deg ?? 0;
  const step = value?.step_deg ?? 1;
  const clockwise = value?.clockwise ?? true;
  const posePoint = { x: pose.x, y: pose.y };
  const points = [];
  const seen = [posePoint];
  let previousValid = false;
  for (let slot = 0; slot < count; slot += 1) {
    const range = ranges[slot];
    if (range != null) {
      const point = scanPoint(pose, first, step, clockwise, slot, range);
      points.push(point);
      seen.push(point);
      previousValid = true;
    } else if (previousValid) {
      // A No Return run notches the Seen Polygon back to the pose, so the
      // sector is part of the unseen complement rather than a guessed surface.
      seen.push(posePoint);
      previousValid = false;
    }
  }
  if (previousValid) seen.push(posePoint);

  const edges = [];
  for (let slot = 0; slot < count; slot += 1) {
    const next = (slot + 1) % count;
    if (ranges[slot] == null || ranges[next] == null) continue;
    const near = Math.min(ranges[slot], ranges[next]);
    const far = Math.max(ranges[slot], ranges[next]);
    if (far - near < Math.max(OCCLUSION_EDGE_MIN_CM, OCCLUSION_EDGE_RATIO * near)) continue;
    const nearSlot = ranges[slot] <= ranges[next] ? slot : next;
    const farSlot = nearSlot === slot ? next : slot;
    edges.push({
      seamSlot: slot,
      nearPoint: scanPoint(pose, first, step, clockwise, nearSlot, ranges[nearSlot]),
      farPoint: scanPoint(pose, first, step, clockwise, farSlot, ranges[farSlot]),
    });
  }

  // A No Return run has no measured surface, so it is closed by the chord
  // between the valid returns on either side. The pair is returned for the
  // renderer to dot; the notch's radial sides stay part of the unseen fill.
  const notches = [];
  const firstValid = ranges.findIndex((range) => range != null);
  if (firstValid >= 0) {
    const firstPoint = scanPoint(pose, first, step, clockwise, firstValid, ranges[firstValid]);
    let previous = null;
    let inNull = false;
    for (let offset = 0; offset < count; offset += 1) {
      const slot = (firstValid + offset) % count;
      const range = ranges[slot];
      if (range != null) {
        const point = scanPoint(pose, first, step, clockwise, slot, range);
        if (inNull && previous) notches.push({ before: previous, after: point });
        previous = point;
        inNull = false;
      } else {
        inNull = true;
      }
    }
    if (inNull && previous) notches.push({ before: previous, after: firstPoint });
  }

  return { points, seen, edges, notches };
}
