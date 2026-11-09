// Pure world-frame geometry: bearings, distances, the camera transform and the
// robot footprint. No DOM, no clock, so the headless suite can test it directly.
//
// Conventions (ADR-0038): distances are centimetres, +x is forward, +y is left,
// yaw is counter-clockwise positive. A LiDAR slot `i` is `i` degrees clockwise
// from the heading (slot 0 ahead); `first_deg`, `step_deg` and `clockwise` are
// honoured even though v1 always sends 0/1/true.

const RAD = Math.PI / 180;

/** The robot's provisional world footprint, in centimetres (ADR-0043). */
export const FOOTPRINT = Object.freeze({ halfLengthCm: 9, halfWidthCm: 7 });

/** The minimum on-screen size of the robot marker, in CSS pixels. */
export const MIN_ICON_PX = 22;

/** The identity pose, used until odometry lands. */
export const IDENTITY_POSE = Object.freeze({ x: 0, y: 0, yaw: 0 });

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
  const centi = Math.round(angle * 100);
  return {
    target: { x: target.x, y: target.y },
    pose: { x: pose.x, y: pose.y, yaw: pose.yaw },
    distance_cm: distance,
    bearing_deg: bearing,
    angle_deg: angle,
    rotate_centideg: centi,
    needs_rotate: centi !== 0,
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
  const minimumHalf = MIN_ICON_PX / 2 / Math.max(scale, 1e-6);
  const halfLength = Math.max(FOOTPRINT.halfLengthCm, minimumHalf);
  const halfWidth = Math.max(FOOTPRINT.halfWidthCm, minimumHalf);
  return Math.abs(forward) <= halfLength && Math.abs(left) <= halfWidth;
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
