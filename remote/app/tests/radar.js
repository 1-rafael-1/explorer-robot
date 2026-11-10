// The World Frame radar suite: the slot convention, the camera transform, the
// icon tap guard, the scan/pose timestamp seam, and a live scan drawn onto a
// real canvas (fresh versus stale). It does not mount the app's full DOM.

import {
  FOOTPRINT,
  MIN_ICON_PX,
  STALE_MS,
  cameraView,
  fitScale,
  iconFootprintPx,
  isStaleScan,
  pointInFootprint,
  poseAt,
  scanAgeMs,
  scanPoints,
  screenToWorld,
  slotAngleDeg,
  worldToScreen,
} from '../js/geometry.js';
import { createRadar } from '../js/radar.js';
import { createStore } from '../js/store.js';
import { assert, assertClose, assertEqual, autorun, test } from './harness.js';

const RAD = Math.PI / 180;

/** Creates a laid-out canvas so `clientWidth`/`clientHeight` are real. */
function makeCanvas(width = 320, height = 240) {
  const canvas = document.createElement('canvas');
  canvas.style.width = `${width}px`;
  canvas.style.height = `${height}px`;
  canvas.style.display = 'block';
  document.body.append(canvas);
  return canvas;
}

/** Counts pixels that differ from the background and pixels in the live blue. */
function pixelStats(canvas) {
  const context = canvas.getContext('2d');
  const { data } = context.getImageData(0, 0, canvas.width, canvas.height);
  let nonBackground = 0;
  let freshCloud = 0;
  for (let index = 0; index < data.length; index += 4) {
    const red = data[index];
    const green = data[index + 1];
    const blue = data[index + 2];
    if (Math.abs(red - 11) > 8 || Math.abs(green - 15) > 8 || Math.abs(blue - 20) > 8) {
      nonBackground += 1;
    }
    // #46c2ff is the live cloud/robot white; #7f96aa (stale) is below this.
    if (green > 160 && blue > 200) freshCloud += 1;
  }
  return { nonBackground, freshCloud };
}

test('LiDAR slot 0 is ahead and the slots run clockwise', () => {
  assertClose(slotAngleDeg({ x: 0, y: 0, yaw: 0 }, 0, 1, true, 90), -90, 1e-9);
  assertClose(slotAngleDeg({ x: 0, y: 0, yaw: 0 }, 0, 1, false, 90), 90, 1e-9);
  assertClose(slotAngleDeg({ x: 0, y: 0, yaw: 90 }, 0, 1, true, 0), 90, 1e-9);
  // `first_deg` and `step_deg` are honoured, not assumed to be 0 and 1.
  assertClose(slotAngleDeg({ x: 0, y: 0, yaw: 0 }, 10, 2, true, 5), -20, 1e-9);
});

test('a null return produces no point, so the cloud draws a gap', () => {
  const pose = { x: 0, y: 0, yaw: 0 };
  const points = scanPoints(pose, {
    ranges_cm: [100, null, 200, null, 300],
    first_deg: 0,
    step_deg: 1,
    clockwise: true,
  });
  assertEqual(
    points.map((point) => point.slot),
    [0, 2, 4],
    'only slots with a return survive',
  );
  assertClose(points[0].x, 100, 1e-9);
  assertClose(points[0].y, 0, 1e-9);
  assertClose(points[1].x, 200 * Math.cos(-2 * RAD), 1e-9);
  assertClose(points[1].y, 200 * Math.sin(-2 * RAD), 1e-9);

  const only = scanPoints(pose, {
    ranges_cm: [10],
    first_deg: 90,
    step_deg: 5,
    clockwise: false,
  });
  assertClose(only[0].x, 0, 1e-9);
  assertClose(only[0].y, 10, 1e-9);
});

/** Returns whether the fresh-cloud colour is present near a world point. */
function cloudNear(canvas, radar, x, y) {
  const dpr = canvas.width / Math.max(1, canvas.clientWidth);
  const scale = radar.scale;
  const px = Math.round((canvas.clientWidth / 2 + x * scale) * dpr);
  const py = Math.round((canvas.clientHeight / 2 - y * scale) * dpr);
  const size = 7;
  const { data } = canvas
    .getContext('2d')
    .getImageData(px - (size - 1) / 2, py - (size - 1) / 2, size, size);
  for (let index = 0; index < data.length; index += 4) {
    if (data[index + 1] > 160 && data[index + 2] > 200) return true;
  }
  return false;
}

test('an isolated LiDAR return is drawn as a point', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  const ranges = Array.from({ length: 360 }, () => null);
  ranges[10] = 123;
  ranges[30] = 187;
  store.dispatch({
    type: 'stream',
    name: 'lidar',
    seq: 1,
    ts: 1000,
    v: { ranges_cm: ranges, first_deg: 0, step_deg: 1, clockwise: true },
  });
  radar.render(store.state, 1100);

  const first = scanPoints({ x: 0, y: 0, yaw: 0 }, store.state.streams.lidar.v)[0];
  const second = scanPoints({ x: 0, y: 0, yaw: 0 }, store.state.streams.lidar.v)[1];
  assert(cloudNear(canvas, radar, first.x, first.y), 'the first isolated return is visible');
  assert(cloudNear(canvas, radar, second.x, second.y), 'the second isolated return is visible');
});

test('the camera transform round-trips and fitScale fits the shorter axis', () => {
  const view = cameraView({ width: 800, height: 600, scale: 2, centerX: 50, centerY: -20 });
  assertEqual(worldToScreen(view, { x: 50, y: -20 }), { x: 400, y: 300 });
  assertEqual(worldToScreen(view, { x: 60, y: -10 }), { x: 420, y: 280 });

  const screen = worldToScreen(view, { x: 120, y: 30 });
  const world = screenToWorld(view, screen.x, screen.y);
  assertClose(world.x, 120, 1e-9);
  assertClose(world.y, 30, 1e-9);

  const scale = fitScale(800, 600, 460);
  assert(scale > 0);
  assert(scale * 460 <= 300, 'the range fits the shorter half-axis');
  assert(scale * 460 > 240, 'and it is not needlessly small');
});

test('the tap guard is the drawn footprint, including a rotated pose', () => {
  const { halfLengthPx, halfWidthPx } = iconFootprintPx(1);
  const pose = { x: 0, y: 0, yaw: 0 };
  assert(pointInFootprint(pose, { x: halfLengthPx * 0.9, y: 0 }, 1));
  assert(!pointInFootprint(pose, { x: halfLengthPx * 1.1, y: 0 }, 1));
  assert(pointInFootprint(pose, { x: 0, y: halfWidthPx * 0.9 }, 1));
  assert(!pointInFootprint(pose, { x: 0, y: halfWidthPx * 1.1 }, 1));

  // Rotated 90° counter-clockwise, the robot's length lies along world +y.
  const rotated = { x: 0, y: 0, yaw: 90 };
  assert(pointInFootprint(rotated, { x: 0, y: halfLengthPx * 0.9 }, 1));
  assert(!pointInFootprint(rotated, { x: halfLengthPx * 0.9, y: 0 }, 1));

  // The icon grows uniformly only below the minimum screen size.
  const big = iconFootprintPx(4);
  assertClose(big.halfLengthPx, FOOTPRINT.halfLengthCm * 4, 1e-9);
  assertClose(big.halfWidthPx, FOOTPRINT.halfWidthCm * 4, 1e-9);
  const small = iconFootprintPx(0.1);
  assertClose(small.halfLengthPx, MIN_ICON_PX / 2, 1e-9);
  assert(small.halfWidthPx < small.halfLengthPx);
});

test('a scan is paired with the pose at its own timestamp, never a newer one', () => {
  const history = [
    { ts: 1000, x_cm: 1, y_cm: 2, yaw_deg: 3 },
    { ts: 2000, x_cm: 10, y_cm: 20, yaw_deg: 30 },
  ];
  assertEqual(poseAt(history, 1500).x_cm, 1);
  assertEqual(poseAt(history, 2000).x_cm, 10);
  assertEqual(poseAt(history, 2500).x_cm, 10);
  assertEqual(poseAt(history, 500), null);
  assertEqual(poseAt(null, 2500), null);
});

test('the scan age helper keeps the badge honest and never gates', () => {
  assertEqual(scanAgeMs(1000, 500), 0, 'a clock skew is not a negative age');
  assertEqual(scanAgeMs(null, 500), null);
  assertEqual(isStaleScan(1000, 1000 + STALE_MS), false);
  assertEqual(isStaleScan(1000, 1000 + STALE_MS + 1), true);
  assertEqual(isStaleScan(null, 1000), true, 'no scan is stale');
});

test('a live scan renders onto the canvas and a stale scan dims', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const age = document.getElementById('scan-age');

  // With no scan the badge is explicit and the canvas still draws the grid/robot.
  radar.render(store.state, 1000);
  const empty = pixelStats(canvas);
  assertEqual(age.textContent, 'No scan');
  assertEqual(age.dataset.stale, 'true');

  const ranges = Array.from({ length: 360 }, (_, slot) => (slot % 17 === 0 ? null : 100));
  store.dispatch({
    type: 'stream',
    name: 'lidar',
    seq: 1,
    ts: 1000,
    v: { ranges_cm: ranges, first_deg: 0, step_deg: 1, clockwise: true },
  });

  radar.render(store.state, 1100);
  const fresh = pixelStats(canvas);
  assertEqual(age.dataset.stale, 'false');
  assert(fresh.nonBackground > empty.nonBackground + 50, 'the scan drew pixels');
  assert(fresh.freshCloud > 0, 'the fresh scan uses its live colour');

  radar.render(store.state, 1000 + STALE_MS + 1000);
  const stale = pixelStats(canvas);
  assertEqual(age.dataset.stale, 'true');
  assert(stale.nonBackground > empty.nonBackground, 'the stale scan still draws');
  assert(stale.freshCloud < fresh.freshCloud, 'the stale scan is dimmed');
});

test('center returns to the robot and keeps the zoom', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  radar.render(store.state, 0);
  const base = radar.scale;
  radar.zoomBy(2);
  radar.render(store.state, 0);
  const zoomed = radar.scale;
  assertClose(zoomed, base * 2, 1e-9, 'zoom doubles the camera scale');

  radar.panBy(500, 0);
  radar.render(store.state, 0);
  radar.center();
  radar.render(store.state, 0);
  assertClose(radar.scale, zoomed, 1e-9, 'center keeps the zoom');
});

autorun();
