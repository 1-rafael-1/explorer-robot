// The World Frame radar suite: the slot convention, the camera transform, the
// icon tap guard, the scan/pose timestamp seam, the Occlusion Shadow
// classifier, and a live scan drawn onto a real canvas (fresh versus stale).
// It does not mount the app's full DOM.

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
  scanShadows,
  screenToWorld,
  slotAngleDeg,
  worldToScreen,
} from '../js/geometry.js';
import { createRadar } from '../js/radar.js';
import { createStore } from '../js/store.js';
import { assert, assertClose, assertEqual, autorun, test } from './harness.js';

const RAD = Math.PI / 180;
const ORIGIN = { x: 0, y: 0, yaw: 0 };

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

test('a near protrusion casts one Occlusion Shadow on its own contour', () => {
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 60;

  const shadows = scanShadows(ORIGIN, scanValue(ranges));
  assertEqual(shadows.length, 1, 'one protrusion, one shadow');
  const [shadow] = shadows;
  assertEqual(shadow.capCm, 300, 'the cap is the local background');
  assertEqual(
    shadow.silhouette.map((point) => point.slot),
    Array.from({ length: 21 }, (_, index) => 170 + index),
    'the silhouette is the measured contour in slot order',
  );
  assertEqual(
    shadow.silhouette.map((point) => point.range),
    Array.from({ length: 21 }, () => 60),
    'every contour point keeps its return',
  );
  // The world points use the slot convention: slot 0 ahead, slots clockwise.
  const first = shadow.silhouette[0];
  assertClose(first.x, 60 * Math.cos(-170 * RAD), 1e-9);
  assertClose(first.y, 60 * Math.sin(-170 * RAD), 1e-9);
});

test('a flat or slanted scan casts no Occlusion Shadow', () => {
  const flat = Array.from({ length: 360 }, () => 240);
  assertEqual(scanShadows(ORIGIN, scanValue(flat)).length, 0, 'a flat wall is not a protrusion');

  const slanted = Array.from({ length: 360 }, (_, slot) => 250 + 30 * Math.cos(slot * RAD));
  assertEqual(
    scanShadows(ORIGIN, scanValue(slanted)).length,
    0,
    'a gentle range change is not an edge',
  );
});

test('a far recess whose complement is the near background is not a protrusion', () => {
  const ranges = Array.from({ length: 360 }, () => 100);
  for (let slot = 100; slot <= 200; slot += 1) ranges[slot] = 300;
  assertEqual(
    scanShadows(ORIGIN, scanValue(ranges)).length,
    0,
    'the longer complement of a wide doorway is the near background',
  );
});

test('a step under the threshold is not an Occlusion Edge', () => {
  // A 40 cm rise on the right extends the near run; the first real rise is
  // then more than half the scan away, so nothing is shaded.
  const weakRise = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) weakRise[slot] = 60;
  for (let slot = 191; slot < 360; slot += 1) weakRise[slot] = 100;
  assertEqual(scanShadows(ORIGIN, scanValue(weakRise)).length, 0);

  // A 40 cm fall cannot start a near segment at all.
  const weakFall = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) weakFall[slot] = 260;
  assertEqual(scanShadows(ORIGIN, scanValue(weakFall)).length, 0);
});

test('a one-slot near spike has coincident edges, so it casts no shadow', () => {
  const ranges = Array.from({ length: 360 }, () => 300);
  ranges[180] = 60;
  assertEqual(scanShadows(ORIGIN, scanValue(ranges)).length, 0);
});

test('a No Return breaks the near segment, so no shadow is cast', () => {
  const inner = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) inner[slot] = 60;
  inner[180] = null;
  assertEqual(
    scanShadows(ORIGIN, scanValue(inner)).length,
    0,
    'a null inside the span disqualifies',
  );

  const flank = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) flank[slot] = 60;
  flank[191] = null;
  assertEqual(scanShadows(ORIGIN, scanValue(flank)).length, 0, 'a null neighbour disqualifies');
});

test('a protrusion across slot 0 is found once in cyclic slot order', () => {
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 355; slot < 360; slot += 1) ranges[slot] = 60;
  for (let slot = 0; slot <= 5; slot += 1) ranges[slot] = 60;

  const shadows = scanShadows(ORIGIN, scanValue(ranges));
  assertEqual(shadows.length, 1, 'the seam does not split the protrusion');
  assertEqual(
    shadows[0].silhouette.map((point) => point.slot),
    [355, 356, 357, 358, 359, 0, 1, 2, 3, 4, 5],
    'the contour runs forward through the seam',
  );
  assertEqual(shadows[0].capCm, 300);
});

test('the cap is the greater of the two flanking returns', () => {
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 60;
  ranges[191] = 500;

  const shadows = scanShadows(ORIGIN, scanValue(ranges));
  assertEqual(shadows.length, 1);
  assertEqual(shadows[0].capCm, 500, 'the far flank wins over the near flank');
});

test('consecutive falls in one protrusion emit a single shadow', () => {
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 185; slot += 1) ranges[slot] = 200;
  ranges[186] = 140;
  for (let slot = 187; slot <= 190; slot += 1) ranges[slot] = 80;

  const shadows = scanShadows(ORIGIN, scanValue(ranges));
  assertEqual(shadows.length, 1, 'an inner fall does not start a second shadow');
  assertEqual(
    shadows[0].silhouette.map((point) => point.slot),
    Array.from({ length: 21 }, (_, index) => 170 + index),
    'the outer fall owns the whole near run',
  );
});

test('the shadow is measured from the pose it is given', () => {
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 60;

  const pose = { x: 100, y: -50, yaw: 20 };
  const first = scanShadows(pose, scanValue(ranges))[0].silhouette[0];
  const angle = (pose.yaw - 170) * RAD;
  assertClose(first.x, pose.x + 60 * Math.cos(angle), 1e-9);
  assertClose(first.y, pose.y + 60 * Math.sin(angle), 1e-9);
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

/** Counts translucent soft-red fill pixels near a world point. */
function redNear(canvas, radar, x, y, size = 7) {
  const dpr = canvas.width / Math.max(1, canvas.clientWidth);
  const scale = radar.scale;
  const px = Math.round((canvas.clientWidth / 2 + x * scale) * dpr);
  const py = Math.round((canvas.clientHeight / 2 - y * scale) * dpr);
  const { data } = canvas
    .getContext('2d')
    .getImageData(px - (size - 1) / 2, py - (size - 1) / 2, size, size);
  let count = 0;
  for (let index = 0; index < data.length; index += 4) {
    const red = data[index];
    const green = data[index + 1];
    const blue = data[index + 2];
    // rgba(248,113,113,0.12) over the dark grid keeps red ahead of green and
    // close to blue; the blue cloud, white robot and grid never do.
    if (red > 24 && red > green && red > blue - 12) count += 1;
  }
  return count;
}

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

/** Returns whether the bright soft-red Occlusion Edge stroke is near a world point. */
function edgeRedNear(canvas, radar, x, y, size = 9) {
  const dpr = canvas.width / Math.max(1, canvas.clientWidth);
  const scale = radar.scale;
  const px = Math.round((canvas.clientWidth / 2 + x * scale) * dpr);
  const py = Math.round((canvas.clientHeight / 2 - y * scale) * dpr);
  const half = (size - 1) / 2;
  const { data } = canvas
    .getContext('2d')
    .getImageData(px - half, py - half, size, size);
  for (let index = 0; index < data.length; index += 4) {
    const red = data[index];
    const green = data[index + 1];
    const blue = data[index + 2];
    // #f87171 is opaque; the 0.12 fill over the dark grid stays near (39,27,31),
    // so a bright red can only be the dotted stroke.
    if (red > 120 && red - green > 50 && red - blue > 50) return true;
  }
  return false;
}

/** Counts live soft-red pixels — the Occlusion fill and its dotted edges — canvas-wide. */
function liveRedPixels(canvas) {
  const { data } = canvas.getContext('2d').getImageData(0, 0, canvas.width, canvas.height);
  let count = 0;
  for (let index = 0; index < data.length; index += 4) {
    const red = data[index];
    const green = data[index + 1];
    const blue = data[index + 2];
    // The same test as `redNear`: the 0.12 fill and the #f87171 dash both pass;
    // the dark background, the blue cloud and the grey stale palette do not.
    if (red > 24 && red > green && red > blue - 12) count += 1;
  }
  return count;
}

/** Returns the mean colour of a square sample window centred on a world point. */
function tintNear(canvas, radar, x, y, size = 9) {
  const dpr = canvas.width / Math.max(1, canvas.clientWidth);
  const scale = radar.scale;
  const px = Math.round((canvas.clientWidth / 2 + x * scale) * dpr);
  const py = Math.round((canvas.clientHeight / 2 - y * scale) * dpr);
  const half = (size - 1) / 2;
  const { data } = canvas
    .getContext('2d')
    .getImageData(px - half, py - half, size, size);
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

test('a protrusion paints a red fan and a flat scan paints none', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  // A 60 cm protrusion across the seam, against a 300 cm background. The
  // near range stays above the 45 cm amber-dot threshold.
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 350; slot < 360; slot += 1) ranges[slot] = 60;
  for (let slot = 0; slot <= 10; slot += 1) ranges[slot] = 60;
  store.dispatch({
    type: 'stream',
    name: 'lidar',
    seq: 1,
    ts: 1000,
    v: { ranges_cm: ranges, first_deg: 0, step_deg: 1, clockwise: true },
  });
  radar.render(store.state, 1100);
  assert(redNear(canvas, radar, 140, 0) > 0, 'the wedge behind the protrusion is shaded');

  // A flat wall at the same range removes the inference and its fill.
  const flat = Array.from({ length: 360 }, () => 300);
  store.dispatch({
    type: 'stream',
    name: 'lidar',
    seq: 2,
    ts: 1000,
    v: { ranges_cm: flat, first_deg: 0, step_deg: 1, clockwise: true },
  });
  radar.render(store.state, 1100);
  assertEqual(redNear(canvas, radar, 140, 0), 0, 'a flat wall paints no shadow');
});

test('a fresh protrusion paints live red and a stale one is muted to grey', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  // A 60 cm protrusion across the scan seam against a 300 cm background, wide
  // enough that a sample well inside the fan is unambiguous. The near range
  // stays above the 45 cm amber-dot threshold.
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 340; slot < 360; slot += 1) ranges[slot] = 60;
  for (let slot = 0; slot <= 20; slot += 1) ranges[slot] = 60;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });

  // Fresh: the live soft red fills the wedge and strokes both Occlusion Edges.
  radar.render(store.state, 1100);
  assert(redNear(canvas, radar, 200, 0) > 0, 'the fresh wedge is shaded live red');
  assert(liveRedPixels(canvas) > 0, 'the fresh scan paints live-red pixels');
  for (const slot of [340, 20]) {
    const angle = slotAngleDeg(ORIGIN, 0, 1, true, slot) * RAD;
    const hit = [200, 230, 260, 290].some((radius) =>
      edgeRedNear(canvas, radar, radius * Math.cos(angle), radius * Math.sin(angle)),
    );
    assert(hit, `slot ${slot} strokes a live-red Occlusion Edge`);
  }

  // Past the stale gate the geometry stays; the live red does not.
  radar.render(store.state, 1000 + STALE_MS + 1);
  assertEqual(liveRedPixels(canvas), 0, 'a stale scan paints no live-red pixels');
  const wedge = tintNear(canvas, radar, 200, 0);
  assert(wedge.red <= wedge.green && wedge.red <= wedge.blue, 'the stale tint reads grey, not red');

  // The same stale grid with no shadow is the control: the fan still tints it.
  const flat = Array.from({ length: 360 }, () => 300);
  store.dispatch({ type: 'stream', name: 'lidar', seq: 2, ts: 1000, v: scanValue(flat) });
  radar.render(store.state, 1000 + STALE_MS + 1);
  const plain = tintNear(canvas, radar, 200, 0);
  assert(
    wedge.blue >= plain.blue + 3,
    `the stale wedge keeps a grey tint: ${wedge.blue} over ${plain.blue}`,
  );
});

test('no scan means no shadow, so no remembered shadow is drawn', () => {
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

test('each Occlusion Shadow strokes dotted soft-red edges to the cap', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  // A 60 cm protrusion across slots 170..190 against a 300 cm background: its
  // Occlusion Edges run outward along the slots 170 and 190 rays.
  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 60;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });
  radar.render(store.state, 1100);

  // The [2, 4] dash leaves gaps, so several radii along each ray must find at
  // least one stroke sample.
  const radii = [80, 120, 160, 200, 240];
  for (const slot of [170, 190]) {
    const angle = slotAngleDeg(ORIGIN, 0, 1, true, slot) * RAD;
    const hit = radii.some((radius) =>
      edgeRedNear(canvas, radar, radius * Math.cos(angle), radius * Math.sin(angle)),
    );
    assert(hit, `slot ${slot} has a dotted Occlusion Edge along its ray`);
  }
});

test('the cloud does not connect across an Occlusion Edge', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  const ranges = Array.from({ length: 360 }, () => 300);
  for (let slot = 170; slot <= 190; slot += 1) ranges[slot] = 60;
  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });
  radar.render(store.state, 1100);

  const points = scanPoints(ORIGIN, scanValue(ranges));
  const at = (slot) => points.find((point) => point.slot === slot);

  const fall = midpoint(at(169), at(170));
  assertEqual(cloudNear(canvas, radar, fall.x, fall.y), false, 'no chord across the fall edge');

  const rise = midpoint(at(190), at(191));
  assertEqual(cloudNear(canvas, radar, rise.x, rise.y), false, 'no chord across the rise edge');
});

test('a far recess keeps its surface chord, because it is not an Occlusion Edge', () => {
  const canvas = makeCanvas();
  const radar = createRadar(canvas, {});
  const store = createStore();

  // The near background is everywhere else, so the complement of this narrow
  // far recess is the background and the classifier infers no shadow.
  const ranges = Array.from({ length: 360 }, () => 100);
  for (let slot = 100; slot <= 110; slot += 1) ranges[slot] = 300;
  assertEqual(scanShadows(ORIGIN, scanValue(ranges)).length, 0, 'a far recess is not a protrusion');

  store.dispatch({ type: 'stream', name: 'lidar', seq: 1, ts: 1000, v: scanValue(ranges) });
  radar.render(store.state, 1100);

  const points = scanPoints(ORIGIN, scanValue(ranges));
  const at = (slot) => points.find((point) => point.slot === slot);
  const chord = midpoint(at(99), at(100));
  assertEqual(cloudNear(canvas, radar, chord.x, chord.y), true, 'the surface chord is still drawn');
});

autorun();
