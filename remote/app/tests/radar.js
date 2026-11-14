// The World Frame radar canvas suite: the camera transform, the icon tap guard,
// the scan/pose timestamp seam, the stale gate, and a live scan drawn onto a
// real canvas (unseen fill, Occlusion Edges, notch chords, fresh versus stale).
// Pure scan-model cases live in pure.js. It does not mount the app's full DOM.

import {
  FOOTPRINT,
  MIN_ICON_PX,
  STALE_MS,
  buildScan,
  cameraView,
  fitScale,
  iconFootprintPx,
  isStaleScan,
  pointInFootprint,
  poseAt,
  scanAgeMs,
  screenToWorld,
  slotAngleDeg,
  worldToScreen,
} from '../js/geometry.js';
import { COLOR, createRadar } from '../js/radar.js';
import { createStore } from '../js/store.js';
import { assert, assertClose, assertEqual, autorun, test } from './harness.js';

const RAD = Math.PI / 180;
const ORIGIN = { x: 0, y: 0, yaw: 0 };

/** Parses a `#rrggbb` canvas colour into channel values. */
function rgb(hex) {
  return {
    red: parseInt(hex.slice(1, 3), 16),
    green: parseInt(hex.slice(3, 5), 16),
    blue: parseInt(hex.slice(5, 7), 16),
  };
}

const BACKGROUND = rgb(COLOR.bg);

/** Builds a one-degree scan value with the v1 conventions. */
function scanValue(ranges) {
  return { ranges_cm: ranges, first_deg: 0, step_deg: 1, clockwise: true };
}

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
    if (
      Math.abs(red - BACKGROUND.red) > 8 ||
      Math.abs(green - BACKGROUND.green) > 8 ||
      Math.abs(blue - BACKGROUND.blue) > 8
    ) {
      nonBackground += 1;
    }
    // #46c2ff is the live cloud; #7f96aa (stale) is below this.
    if (green > 160 && blue > 200) freshCloud += 1;
  }
  return { nonBackground, freshCloud };
}

/** Returns a square ImageData window centred on a world point. */
function windowNear(canvas, radar, x, y, size) {
  const dpr = canvas.width / Math.max(1, canvas.clientWidth);
  const scale = radar.scale;
  const px = Math.round((canvas.clientWidth / 2 + x * scale) * dpr);
  const py = Math.round((canvas.clientHeight / 2 - y * scale) * dpr);
  const half = (size - 1) / 2;
  return canvas.getContext('2d').getImageData(px - half, py - half, size, size);
}

/**
 * The live unseen-fill red. `green - blue < 40` keeps the amber obstacle and
 * target colour out of the match.
 */
function isLiveRed(red, green, blue) {
  return red > 24 && red > green && red > blue - 12 && green - blue < 40;
}

/** Counts translucent soft-red fill pixels near a world point. */
function redNear(canvas, radar, x, y, size = 7) {
  const { data } = windowNear(canvas, radar, x, y, size);
  let count = 0;
  for (let index = 0; index < data.length; index += 4) {
    if (isLiveRed(data[index], data[index + 1], data[index + 2])) count += 1;
  }
  return count;
}

/** Returns whether the fresh-cloud colour is present near a world point. */
function cloudNear(canvas, radar, x, y) {
  const { data } = windowNear(canvas, radar, x, y, 7);
  for (let index = 0; index < data.length; index += 4) {
    if (data[index + 1] > 160 && data[index + 2] > 200) return true;
  }
  return false;
}

/** Returns whether the bright soft-red Occlusion Edge stroke is near a world point. */
function edgeRedNear(canvas, radar, x, y, size = 9) {
  const { data } = windowNear(canvas, radar, x, y, size);
  for (let index = 0; index < data.length; index += 4) {
    const red = data[index];
    const green = data[index + 1];
    const blue = data[index + 2];
    // #f87171 is opaque; the 0.12 fill over the dark grid stays near (39,27,31).
    // `green < 150` keeps the amber near-obstacle dots out.
    if (red > 120 && red - green > 50 && red - blue > 50 && green < 150) return true;
  }
  return false;
}

/** Counts live soft-red pixels — the unseen fill and its dotted edges — canvas-wide. */
function liveRedPixels(canvas) {
  const { data } = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height);
  let count = 0;
  for (let index = 0; index < data.length; index += 4) {
    if (isLiveRed(data[index], data[index + 1], data[index + 2])) count += 1;
  }
  return count;
}

/** Returns the mean colour of a square sample window centred on a world point. */
function tintNear(canvas, radar, x, y, size = 9) {
  const { data } = windowNear(canvas, radar, x, y, size);
  const total = { red: 0, green: 0, blue: 0 };
  for (let index = 0; index < data.length; index += 4) {
    total.red += data[index];
    total.green += data[index + 1];
    total.blue += data[index + 2];
  }
  const pixels = data.length / 4;
  return { red: total.red / pixels, green: total.green / pixels, blue: total.blue / pixels };
}

/** Returns the world midpoint between two returns. */
function midpoint(a, b) {
  return { x: (a.x + b.x) / 2, y: (a.y + b.y) / 2 };
}

test('an isolated LiDAR return is drawn as a point', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  const ranges = Array.from({ length: 360 }, () => null);
  ranges[10] = 123;
  ranges[30] = 187;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });
  radar.render(store.state, 1100);

  const first = buildScan(ORIGIN, store.state.streams.lidar.v).points[0];
  const second = buildScan(ORIGIN, store.state.streams.lidar.v).points[1];
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
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

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

test('the unseen complement is red and the seen region is clear', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 300);
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  assertEqual(redNear(canvas, radar, 150, 0), 0, 'inside the wall is measured and clear');
  assert(redNear(canvas, radar, 350, 0) > 0, 'beyond the wall is unseen and red');
});

test('a fully valid seam does not leave a false unseen wedge', () => {
  const canvas = makeCanvas(1200, 1200);
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 300);
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  // Slot 0 is 0° and slot 359 is +1° clockwise; the old pose-closed polygon
  // painted the half-degree between them as unseen even though both measured.
  // A larger canvas makes the one-degree wedge wide enough to sample.
  const seam = 0.5 * RAD;
  assertEqual(
    redNear(canvas, radar, 280 * Math.cos(seam), 280 * Math.sin(seam), 11),
    0,
    'the seam between slots 359 and 0 stays measured',
  );
});

test('a No Return sector is part of the unseen complement', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 350; slot <= 357; slot += 1) ranges[slot] = 100;
  for (let slot = 358; slot < 360; slot += 1) ranges[slot] = null;
  for (let slot = 0; slot <= 2; slot += 1) ranges[slot] = null;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  assert(redNear(canvas, radar, 200, 0) > 0, 'the null sector is unknown, so it is unseen');
  const points = buildScan(ORIGIN, scanValue(ranges)).points;
  const at = (slot) => points.find((point) => point.slot === slot);
  const start = at(357);
  const end = at(3);
  const hit = [0.25, 0.5, 0.75].some((ratio) =>
    edgeRedNear(
      canvas,
      radar,
      start.x + (end.x - start.x) * ratio,
      start.y + (end.y - start.y) * ratio,
    ),
  );
  assert(hit, 'the No Return edge is the chord between the two returns');
  const angle = slotAngleDeg(ORIGIN, 0, 1, true, 357) * RAD;
  const beyond = [350, 400].some((radius) =>
    edgeRedNear(canvas, radar, radius * Math.cos(angle), radius * Math.sin(angle)),
  );
  assert(!beyond, 'the No Return edge stops at the returns');
});

test('a near object leaves the space before it seen and behind it unseen', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 350; slot < 360; slot += 1) ranges[slot] = 60;
  for (let slot = 0; slot <= 10; slot += 1) ranges[slot] = 60;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  assertEqual(redNear(canvas, radar, 30, 0), 0, 'between the robot and the object is seen');
  assert(redNear(canvas, radar, 150, 0) > 0, 'behind the object is unseen');
});

test('a significant step is dotted and its solid chord is gone', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 60;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  const points = buildScan(ORIGIN, scanValue(ranges)).points;
  const at = (slot) => points.find((point) => point.slot === slot);
  const start = at(169);
  const end = at(170);
  const hit = [0.2, 0.4, 0.6, 0.8].some((ratio) =>
    edgeRedNear(
      canvas,
      radar,
      start.x + (end.x - start.x) * ratio,
      start.y + (end.y - start.y) * ratio,
    ),
  );
  assert(hit, 'the Occlusion Edge is dotted soft red');
  const chord = midpoint(start, end);
  assertEqual(cloudNear(canvas, radar, chord.x, chord.y), false, 'no solid chord spans the edge');
});

test('a gentle range step keeps its solid surface chord', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 200);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 190;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  const points = buildScan(ORIGIN, scanValue(ranges)).points;
  const at = (slot) => points.find((point) => point.slot === slot);
  const chord = midpoint(at(169), at(170));
  assert(cloudNear(canvas, radar, chord.x, chord.y), 'a sub-threshold step stays a solid return');
});

test('a fresh scan paints live red; a stale scan mutes the unseen fill', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => 300);
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  assert(redNear(canvas, radar, 350, 0) > 0, 'the unseen complement is live red');
  assert(liveRedPixels(canvas) > 0, 'the fresh scan paints live-red pixels');

  radar.render(store.state, 1000 + STALE_MS + 1);
  assertEqual(liveRedPixels(canvas), 0, 'a stale scan paints no live-red pixels');
  const stale = tintNear(canvas, radar, 340, 12);
  assert(stale.blue > BACKGROUND.blue + 4, 'the stale unseen fill is painted, not just absent');
  assert(stale.red <= stale.green && stale.red <= stale.blue, 'the stale unseen fill reads grey');
});

test('a scan with no measured return paints no unseen fill', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();
  const ranges = Array.from({ length: 360 }, () => null);
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  radar.render(store.state, 1100);
  assertEqual(liveRedPixels(canvas), 0, 'no measured return means no unseen inference');
});

test('no scan means no unseen fill', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  radar.render(store.state, 1100);
  assertEqual(liveRedPixels(canvas), 0, 'a radar with no lidar stream paints no red');
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
