// The World Frame radar: one canvas, a camera view transform and layered
// drawing. ADR-0043 fixes the order; ADR-0047 inserts the unseen fill:
// grid, map layer, unseen fill, live scan, robot/target/preview, overlay.
// The camera is display state only: a tap picks a world point, a drag pans, and
// neither changes the frame or the meaning of a world point (ADR-0043).
//
// Each scan is paired with the pose at its own timestamp, never a newer one; the
// robot marker uses the newest pose. v1's pose is identity, so this is the same
// point today, but the seam is written honestly for odometry.

import {
  IDENTITY_POSE,
  RAD,
  STALE_MS,
  cameraView,
  fitScale,
  iconFootprintPx,
  isStaleScan,
  planLeg,
  pointInFootprint,
  poseAt,
  scanAgeMs,
  scanPoints,
  scanVisibility,
  screenToWorld,
  worldToScreen,
} from './geometry.js';
import { latestPose, relayNow } from './store.js';
import { byId, setData, setText } from './views/dom.js';

export { STALE_MS };

const COLOR = Object.freeze({
  bg: '#0b0f14',
  grid: '#16232f',
  gridStrong: '#22384c',
  cloud: '#46c2ff',
  cloudStale: '#7f96aa',
  obstacle: '#fbbf24',
  obstacleStale: 'rgba(127, 150, 170, 0.5)',
  robot: '#e6f2ff',
  track: '#070a0e',
  heading: '#4ade80',
  target: '#fbbf24',
  preview: '#46c2ff',
  occlusionFill: 'rgba(248, 113, 113, 0.12)',
  occlusionEdge: '#f87171',
  occlusionFillStale: 'rgba(127, 150, 170, 0.06)',
  occlusionEdgeStale: '#7f96aa',
});

/** Creates the radar over `canvas`; a missing canvas yields a no-op camera. */
export function createRadar(canvas, { onPick = () => {} } = {}) {
  if (!canvas) {
    return {
      render() {},
      invalidate() {},
      center() {},
      zoomBy() {},
      panBy() {},
      pointInFootprint: () => false,
      worldAt: () => ({ x: 0, y: 0 }),
      get scale() {
        return 1;
      },
      get pose() {
        return IDENTITY_POSE;
      },
      planTo: (target) => planLeg(IDENTITY_POSE, target),
    };
  }

  const context = canvas.getContext('2d');
  const camera = { zoom: 1, centerX: 0, centerY: 0, follow: true };
  let view = null;
  let pose = IDENTITY_POSE;
  let dirty = true;
  let drag = null;
  let pinch = null;
  const pointers = new Map();

  function resize() {
    const dpr = Math.min(globalThis.devicePixelRatio || 1, 2);
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    const backingWidth = Math.round(width * dpr);
    const backingHeight = Math.round(height * dpr);
    if (canvas.width !== backingWidth || canvas.height !== backingHeight) {
      canvas.width = backingWidth;
      canvas.height = backingHeight;
    }
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { width, height };
  }

  if (typeof globalThis.ResizeObserver === 'function') {
    const observer = new globalThis.ResizeObserver(() => {
      // Keep the backing store sharp immediately; `render` also calls resize.
      resize();
      dirty = true;
    });
    observer.observe(canvas);
  }

  /** Recomputes the view transform from the camera; also syncs taps after a gesture. */
  function syncView() {
    const { width, height } = resize();
    const scale = fitScale(width, height, 460) * camera.zoom;
    const centerX = camera.follow ? pose.x : camera.centerX;
    const centerY = camera.follow ? pose.y : camera.centerY;
    view = cameraView({ width, height, scale, centerX, centerY });
    return { width, height };
  }

  /** Marks the camera view dirty; the app's single rAF loop redraws it. */
  function invalidate() {
    dirty = true;
  }

  /** Returns the camera to the robot, keeping the current zoom (ADR-0038). */
  function center() {
    camera.follow = true;
    if (view) syncView();
    dirty = true;
  }

  function zoomBy(factor, screenX = null, screenY = null) {
    const before = view ?? syncView();
    const next = Math.min(12, Math.max(0.25, camera.zoom * factor));
    if (next === camera.zoom) return;
    if (before && screenX != null && screenY != null) {
      const world = screenToWorld(before, screenX, screenY);
      camera.zoom = next;
      const scale = fitScale(before.width, before.height) * camera.zoom;
      camera.centerX = world.x - (screenX - before.cx) / scale;
      camera.centerY = world.y + (screenY - before.cy) / scale;
      camera.follow = false;
    } else {
      camera.zoom = next;
    }
    syncView();
    dirty = true;
  }

  function panBy(dx, dy) {
    const current = view ?? syncView();
    camera.follow = false;
    camera.centerX -= dx / current.scale;
    camera.centerY += dy / current.scale;
    syncView();
    dirty = true;
  }

  function localPoint(event) {
    const rect = canvas.getBoundingClientRect();
    return { x: event.clientX - rect.left, y: event.clientY - rect.top };
  }

  function worldAt(screenX, screenY) {
    return view ? screenToWorld(view, screenX, screenY) : { x: 0, y: 0 };
  }

  canvas.addEventListener('pointerdown', (event) => {
    canvas.setPointerCapture?.(event.pointerId);
    const point = localPoint(event);
    pointers.set(event.pointerId, point);
    drag = { ...point, moved: false };
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      pinch = { distance: Math.hypot(a.x - b.x, a.y - b.y), zoom: camera.zoom };
      // A two-finger gesture is a camera gesture, never a pick.
      drag.moved = true;
    }
  });

  canvas.addEventListener('pointermove', (event) => {
    if (!pointers.has(event.pointerId) || !drag) return;
    const point = localPoint(event);
    const previous = pointers.get(event.pointerId);
    pointers.set(event.pointerId, point);
    const total = Math.hypot(point.x - drag.x, point.y - drag.y);
    if (total > 4) drag.moved = true;
    if (pointers.size === 2 && pinch) {
      const [a, b] = [...pointers.values()];
      const distance = Math.hypot(a.x - b.x, a.y - b.y);
      if (distance > 0 && pinch.distance > 0) {
        const target = pinch.zoom * (distance / pinch.distance);
        zoomBy(target / camera.zoom, (a.x + b.x) / 2, (a.y + b.y) / 2);
      }
      return;
    }
    if (drag.moved) panBy(point.x - previous.x, point.y - previous.y);
  });

  function release(event) {
    if (!pointers.has(event.pointerId)) return;
    const point = localPoint(event);
    const wasTap =
      pointers.size === 1 &&
      drag &&
      !drag.moved &&
      Math.hypot(point.x - drag.x, point.y - drag.y) <= 4;
    pointers.delete(event.pointerId);
    if (pointers.size < 2) pinch = null;
    if (wasTap) {
      const world = worldAt(point.x, point.y);
      const inside = pointInFootprint(pose, world, view?.scale ?? 1);
      onPick(world, { inside_robot: inside });
    }
    if (pointers.size === 0) drag = null;
  }

  canvas.addEventListener('pointerup', release);
  canvas.addEventListener('pointercancel', release);
  canvas.addEventListener(
    'wheel',
    (event) => {
      event.preventDefault();
      const point = localPoint(event);
      zoomBy(Math.exp(-event.deltaY * 0.0015), point.x, point.y);
    },
    { passive: false },
  );

  /**
   * Returns the pose a scan is drawn with: the newest sample at or before the
   * scan's `ts`. v1 has no pose history and no pose stream, so it falls back to
   * the identity pose; a newer pose is never applied to an older scan.
   */
  function poseForScan(state, lidar) {
    const history = state?.poses ?? state?.poseHistory ?? null;
    const sample = poseAt(history, lidar?.ts);
    if (sample) {
      return {
        x: sample.x_cm ?? sample.x ?? 0,
        y: sample.y_cm ?? sample.y ?? 0,
        yaw: sample.yaw_deg ?? sample.yaw ?? 0,
      };
    }
    const latest = latestPose(state ?? { streams: {} });
    if (!latest.identity && lidar?.ts != null && latest.ts <= lidar.ts) return latest;
    return IDENTITY_POSE;
  }

  function render(state, now = relayNow(state)) {
    const source = state ?? { streams: {} };
    const latest = latestPose(source);
    pose = { x: latest.x, y: latest.y, yaw: latest.yaw };
    const { width, height } = syncView();

    context.clearRect(0, 0, width, height);
    context.fillStyle = COLOR.bg;
    context.fillRect(0, 0, width, height);

    // The scan's seen polygon and significant edges are inferred once per
    // render, so the unseen fill, the dotted edges and the cloud's chord seams
    // all come from the same scan (ADR-0047). Its liveness is the scan's own: a
    // missing scan is stale, so the layer never outlives what it came from.
    const lidar = source.streams?.lidar;
    const scanPose = lidar ? poseForScan(source, lidar) : null;
    const visibility = lidar ? scanVisibility(scanPose, lidar.v) : null;
    const stale = isStaleScan(lidar?.ts, now);
    const scan = lidar ? { lidar, pose: scanPose, visibility, stale } : null;

    // Layer order (ADR-0043; unseen fill per ADR-0047): grid, map layer, unseen
    // fill, live scan, robot/target/preview, overlay.
    drawGrid();
    drawMapLayer();
    drawUnseen(scan);
    drawNotchEdges(scan);
    drawCloud(scan);
    drawOcclusionEdges(scan);
    drawPreview(source);
    drawRobot();
    updateOverlay(source, now);

    dirty = false;
  }

  function drawGrid() {
    if (!view) return;
    const { width, height, scale, centerX, centerY, cx, cy } = view;
    const minX = centerX - cx / scale;
    const maxX = centerX + (width - cx) / scale;
    const minY = centerY - (height - cy) / scale;
    const maxY = centerY + cy / scale;
    context.lineWidth = 1;
    for (let x = Math.ceil(minX / 50) * 50; x <= maxX; x += 50) {
      const sx = cx + (x - centerX) * scale;
      context.strokeStyle = x % 100 === 0 ? COLOR.gridStrong : COLOR.grid;
      context.beginPath();
      context.moveTo(sx, 0);
      context.lineTo(sx, height);
      context.stroke();
    }
    for (let y = Math.ceil(minY / 50) * 50; y <= maxY; y += 50) {
      const sy = cy - (y - centerY) * scale;
      context.strokeStyle = y % 100 === 0 ? COLOR.gridStrong : COLOR.grid;
      context.beginPath();
      context.moveTo(0, sy);
      context.lineTo(width, sy);
      context.stroke();
    }
  }

  function drawMapLayer() {
    // The future `pose` and `map` streams (the robot's Odometry Grid) render
    // here, sharing the camera. There is no app-side map in v1 (ADR-0038), so
    // this layer is deliberately empty.
  }

  /**
   * Fills the unseen complement of the Seen Polygon under the cloud (ADR-0047).
   * The polygon is the star-shaped region from the scan pose to every measured
   * return; the fill is the whole viewport outside it, with a radial notch at
   * every No Return. The fill follows the scan's age: live soft red, or the
   * muted stale grey.
   */
  function drawUnseen(scan) {
    if (!view || !scan || !scan.visibility) return;
    const { seen } = scan.visibility;

    context.save();
    context.fillStyle = scan.stale ? COLOR.occlusionFillStale : COLOR.occlusionFill;
    context.beginPath();
    context.rect(0, 0, view.width, view.height);
    if (seen.length > 1) {
      const start = worldToScreen(view, seen[0]);
      context.moveTo(start.x, start.y);
      for (let index = 1; index < seen.length; index += 1) {
        const screen = worldToScreen(view, seen[index]);
        context.lineTo(screen.x, screen.y);
      }
      context.closePath();
    }
    // Even-odd fills the viewport outside the Seen Polygon and leaves the
    // measured region clear.
    context.fill('evenodd');
    context.restore();
  }

  /**
   * Strokes the No Return notch boundaries under the cloud (ADR-0047). A notch
   * has no chord, so each ray starts at the seen return and runs outward; the
   * cloud stays on top, so a measured return at the notch edge is not painted
   * over.
   */
  function drawNotchEdges(scan) {
    if (!view || !scan || !scan.visibility) return;
    const { notches } = scan.visibility;
    if (notches.length === 0) return;

    context.save();
    context.strokeStyle = scan.stale ? COLOR.occlusionEdgeStale : COLOR.occlusionEdge;
    context.lineWidth = 1.2;
    context.setLineDash([2, 4]);
    context.lineCap = 'round';
    const pose = worldToScreen(view, scan.pose);
    const reach = Math.hypot(view.width, view.height);
    for (const notch of notches) {
      for (const point of [notch.before, notch.after]) {
        const start = worldToScreen(view, point);
        const dx = start.x - pose.x;
        const dy = start.y - pose.y;
        const length = Math.hypot(dx, dy);
        if (!(length > 0)) continue;
        const end = {
          x: start.x + (dx / length) * reach,
          y: start.y + (dy / length) * reach,
        };
        context.beginPath();
        context.moveTo(start.x, start.y);
        context.lineTo(end.x, end.y);
        context.stroke();
      }
    }
    context.restore();
  }

  /**
   * Strokes the significant Occlusion Edge chords above the cloud (ADR-0047).
   * Each edge is the chord between two adjacent returns whose range step is
   * large enough that the chord is not a measured surface. The stroke shares
   * the scan's age with the fill and the cloud.
   */
  function drawOcclusionEdges(scan) {
    if (!view || !scan || !scan.visibility) return;
    const { edges } = scan.visibility;
    if (edges.length === 0) return;

    context.save();
    context.strokeStyle = scan.stale ? COLOR.occlusionEdgeStale : COLOR.occlusionEdge;
    context.lineWidth = 1.2;
    context.setLineDash([2, 4]);
    context.lineCap = 'round';
    for (const edge of edges) {
      const near = worldToScreen(view, edge.near);
      const far = worldToScreen(view, edge.far);
      context.beginPath();
      context.moveTo(near.x, near.y);
      context.lineTo(far.x, far.y);
      context.stroke();
    }
    context.restore();
  }

  function drawCloud(scan) {
    if (!view || !scan) return;
    const { lidar, pose: cloudPose, visibility, stale } = scan;
    const points = scanPoints(cloudPose, lidar.v);
    const seamStarts = new Set(visibility.edges.map((edge) => edge.seamSlot));

    // Group adjacent slots into runs, breaking at an Occlusion Edge so no
    // surface chord spans unseen space. A lone slot has no line to join, so it
    // is drawn as a point rather than a `moveTo` that `stroke()` cannot show.
    const runs = [];
    let run = null;
    for (const point of points) {
      const screen = worldToScreen(view, point);
      const previous = run?.[run.length - 1].point;
      const joins =
        previous && point.slot === previous.slot + 1 && !seamStarts.has(previous.slot);
      if (joins) {
        run.push({ point, screen });
      } else {
        run = [{ point, screen }];
        runs.push(run);
      }
    }

    context.save();
    context.strokeStyle = stale ? COLOR.cloudStale : COLOR.cloud;
    context.lineWidth = 1.6;
    context.lineJoin = 'round';
    context.beginPath();
    for (const segment of runs) {
      if (segment.length < 2) continue;
      context.moveTo(segment[0].screen.x, segment[0].screen.y);
      for (let index = 1; index < segment.length; index += 1) {
        context.lineTo(segment[index].screen.x, segment[index].screen.y);
      }
    }
    context.stroke();

    // Isolated returns still count as obstacles.
    context.fillStyle = stale ? COLOR.cloudStale : COLOR.cloud;
    for (const segment of runs) {
      if (segment.length !== 1) continue;
      const { screen } = segment[0];
      context.beginPath();
      context.arc(screen.x, screen.y, 1.6, 0, Math.PI * 2);
      context.fill();
    }

    // Near returns are the interesting ones: bright dots keep an obstacle
    // legible even when the outline is a thin wall.
    context.fillStyle = stale ? COLOR.obstacleStale : COLOR.obstacle;
    for (const point of points) {
      if (point.range > 45) continue;
      const screen = worldToScreen(view, point);
      context.beginPath();
      context.arc(screen.x, screen.y, 2.2, 0, Math.PI * 2);
      context.fill();
    }
    context.restore();
  }

  function drawPreview(state) {
    const plan = state?.drive?.plan;
    if (!plan || !view) return;
    if (plan.distance_cm == null) return;
    const origin = { x: plan.pose.x, y: plan.pose.y, yaw: plan.pose.yaw };
    const end = {
      x: origin.x + plan.distance_cm * Math.cos((origin.yaw + plan.angle_deg) * RAD),
      y: origin.y + plan.distance_cm * Math.sin((origin.yaw + plan.angle_deg) * RAD),
    };
    const start = worldToScreen(view, origin);
    const target = worldToScreen(view, end);
    context.save();
    context.strokeStyle = COLOR.preview;
    context.setLineDash([6, 5]);
    context.lineWidth = 1.5;
    context.beginPath();
    context.moveTo(start.x, start.y);
    context.lineTo(target.x, target.y);
    context.stroke();
    context.setLineDash([]);

    context.strokeStyle = COLOR.target;
    context.lineWidth = 2;
    context.beginPath();
    context.arc(target.x, target.y, 7, 0, Math.PI * 2);
    context.moveTo(target.x - 12, target.y);
    context.lineTo(target.x + 12, target.y);
    context.moveTo(target.x, target.y - 12);
    context.lineTo(target.x, target.y + 12);
    context.stroke();
    context.restore();
  }

  function drawRobot() {
    if (!view) return;
    const screen = worldToScreen(view, pose);
    // The drawn rectangle is also the tap guard (pointInFootprint).
    const { halfLengthPx, halfWidthPx } = iconFootprintPx(view.scale);
    const length = halfLengthPx * 2;
    const width = halfWidthPx * 2;
    const trackWidth = Math.max(2, width * 0.3);
    context.save();
    context.translate(screen.x, screen.y);
    context.rotate(-pose.yaw * RAD);
    // Two black tracks down the sides and a white body between them.
    context.fillStyle = COLOR.track;
    context.fillRect(-length / 2, -width / 2, length, trackWidth);
    context.fillRect(-length / 2, width / 2 - trackWidth, length, trackWidth);
    context.fillStyle = COLOR.robot;
    context.fillRect(-length * 0.43, -width * 0.28, length * 0.86, width * 0.56);
    // A black nose stripe and a green heading mark, both inside the footprint.
    context.fillStyle = COLOR.track;
    context.fillRect(length * 0.34, -width * 0.2, length * 0.1, width * 0.4);
    context.strokeStyle = COLOR.heading;
    context.lineWidth = Math.max(1.5, length * 0.08);
    context.beginPath();
    context.moveTo(-length * 0.08, 0);
    context.lineTo(length * 0.4, 0);
    context.stroke();
    context.restore();
  }

  function updateOverlay(state, now) {
    const lidar = state?.streams?.lidar;
    const age = byId('scan-age');
    if (age) {
      const elapsed = scanAgeMs(lidar?.ts, now);
      if (elapsed == null) {
        setText(age, 'No scan');
        setData(age, 'stale', 'true');
      } else {
        setText(
          age,
          elapsed < 1000 ? `${Math.round(elapsed)} ms old` : `${(elapsed / 1000).toFixed(1)} s old`,
        );
        setData(age, 'stale', String(isStaleScan(lidar.ts, now)));
      }
    }
    setText(byId('zoom-level'), `${camera.zoom.toFixed(1)}×`);
  }

  return {
    render,
    invalidate,
    center,
    zoomBy,
    panBy,
    worldAt,
    get scale() {
      return view?.scale ?? 1;
    },
    get pose() {
      return pose;
    },
    planTo(target) {
      return planLeg(pose, target);
    },
  };
}

/** Wires the on-screen camera cluster. */
export function bindRadar(radar, { onCenter = () => {} } = {}) {
  byId('zoom-in')?.addEventListener('click', () => radar.zoomBy(1.25));
  byId('zoom-out')?.addEventListener('click', () => radar.zoomBy(0.8));
  byId('center-view')?.addEventListener('click', () => {
    radar.center();
    onCenter();
  });
  byId('pan-up')?.addEventListener('click', () => radar.panBy(0, 60));
  byId('pan-down')?.addEventListener('click', () => radar.panBy(0, -60));
  byId('pan-left')?.addEventListener('click', () => radar.panBy(60, 0));
  byId('pan-right')?.addEventListener('click', () => radar.panBy(-60, 0));
}
