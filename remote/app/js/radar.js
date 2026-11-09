// The World Frame radar: one canvas, a camera view transform and layered drawing
// (grid, future map layer, live scan, robot/target/preview, overlay). ADR-0038
// pairs each scan with the pose at its own timestamp; v1's pose is identity.
//
// The camera is display state only: a tap picks a world point, a drag pans, and
// neither changes the frame or the meaning of a world point (ADR-0043).

import {
  FOOTPRINT,
  MIN_ICON_PX,
  cameraView,
  fitScale,
  planLeg,
  pointInFootprint,
  scanPoints,
  screenToWorld,
  worldToScreen,
} from './geometry.js';
import { latestPose } from './store.js';
import { byId, setData, setText } from './views/dom.js';

const RAD = Math.PI / 180;

/** The age after which a scan is drawn dimmed; display only, never a Go gate. */
export const STALE_MS = 350;

const COLOR = Object.freeze({
  bg: '#0b0f14',
  grid: '#16232f',
  gridStrong: '#22384c',
  cloud: '#46c2ff',
  cloudStale: '#7f96aa',
  obstacle: '#fbbf24',
  robot: '#e6f2ff',
  track: '#070a0e',
  heading: '#4ade80',
  target: '#fbbf24',
  preview: '#46c2ff',
  text: '#8aa0b5',
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
    };
  }
  const context = canvas.getContext('2d');
  const camera = { zoom: 1, centerX: 0, centerY: 0, follow: true };
  let view = null;
  let pose = { x: 0, y: 0, yaw: 0 };
  let dirty = true;
  let drag = null;
  let pinch = null;
  const pointers = new Map();

  function resize() {
    const dpr = Math.min(globalThis.devicePixelRatio || 1, 2);
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
      canvas.width = Math.round(width * dpr);
      canvas.height = Math.round(height * dpr);
    }
    context.setTransform(dpr, 0, 0, dpr, 0, 0);
    return { width, height };
  }

  function invalidate() {
    dirty = true;
  }

  function center() {
    camera.follow = true;
    camera.zoom = 1;
    dirty = true;
  }

  function zoomBy(factor, screenX = null, screenY = null) {
    const before = view;
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
    dirty = true;
  }

  function panBy(dx, dy) {
    if (!view) return;
    camera.follow = false;
    camera.centerX -= dx / view.scale;
    camera.centerY += dy / view.scale;
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
    pointers.set(event.pointerId, localPoint(event));
    drag = { ...localPoint(event), moved: false };
    if (pointers.size === 2) {
      const [a, b] = [...pointers.values()];
      pinch = { distance: Math.hypot(a.x - b.x, a.y - b.y), zoom: camera.zoom };
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
      if (distance > 0) zoomBy((distance / pinch.distance) * (pinch.zoom / camera.zoom));
      return;
    }
    if (drag.moved) panBy(point.x - previous.x, point.y - previous.y);
  });

  function release(event) {
    if (!pointers.has(event.pointerId)) return;
    const point = localPoint(event);
    const wasTap = drag && !drag.moved && Math.hypot(point.x - drag.x, point.y - drag.y) <= 4;
    pointers.delete(event.pointerId);
    if (pointers.size < 2) pinch = null;
    if (wasTap) {
      const world = worldAt(point.x, point.y);
      const inside = pointInFootprint(pose, world, view?.scale ?? 1);
      onPick(world, { inside_robot: inside });
    }
    drag = null;
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

  function render(state, now = Date.now()) {
    const { width, height } = resize();
    const latest = latestPose(state);
    pose = { x: latest.x, y: latest.y, yaw: latest.yaw };
    const scale = fitScale(width, height, 460) * camera.zoom;
    const centerX = camera.follow ? pose.x : camera.centerX;
    const centerY = camera.follow ? pose.y : camera.centerY;
    view = cameraView({ width, height, scale, centerX, centerY });

    context.clearRect(0, 0, width, height);
    context.fillStyle = COLOR.bg;
    context.fillRect(0, 0, width, height);
    drawGrid();
    drawMapLayer();
    const lidar = state.streams.lidar;
    if (lidar) drawCloud(lidar, now);
    drawPreview(state);
    drawRobot();
    updateOverlay(state, now);
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
    // The future `pose` and `map` streams plug in here; there is no app-side map
    // in v1 (ADR-0038), so this layer is deliberately empty.
  }

  function drawCloud(lidar, now) {
    const points = scanPoints(pose, lidar.v);
    const stale = now - lidar.ts > STALE_MS;
    context.save();
    context.strokeStyle = stale ? COLOR.cloudStale : COLOR.cloud;
    context.lineWidth = 1.6;
    context.lineJoin = 'round';
    context.beginPath();
    let previousSlot = null;
    for (const point of points) {
      const screen = worldToScreen(view, point);
      if (previousSlot != null && point.slot === previousSlot + 1) context.lineTo(screen.x, screen.y);
      else context.moveTo(screen.x, screen.y);
      previousSlot = point.slot;
    }
    context.stroke();
    context.fillStyle = stale ? 'rgba(127, 150, 170, 0.5)' : COLOR.obstacle;
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
    const plan = state.drive?.plan;
    if (!plan || !view) return;
    const origin = { x: plan.pose.x, y: plan.pose.y, yaw: plan.pose.yaw };
    const leg = plan.distance_cm == null ? null : plan;
    if (!leg) return;
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
    const size = Math.max(MIN_ICON_PX, 2 * FOOTPRINT.halfLengthCm * view.scale);
    const width = size * (FOOTPRINT.halfWidthCm / FOOTPRINT.halfLengthCm);
    context.save();
    context.translate(screen.x, screen.y);
    context.rotate(-pose.yaw * RAD);
    // Two black tracks, a white body and a heading mark.
    context.fillStyle = COLOR.track;
    context.fillRect(-size / 2, -width / 2, size, width * 0.28);
    context.fillRect(-size / 2, width / 2 - width * 0.28, size, width * 0.28);
    context.fillStyle = COLOR.robot;
    context.fillRect(-size * 0.42, -width * 0.24, size * 0.84, width * 0.48);
    context.strokeStyle = COLOR.heading;
    context.lineWidth = Math.max(1.5, size * 0.06);
    context.beginPath();
    context.moveTo(size * 0.2, 0);
    context.lineTo(size * 0.52, 0);
    context.stroke();
    context.restore();
  }

  function updateOverlay(state, now) {
    const lidar = state.streams.lidar;
    const age = byId('scan-age');
    if (age) {
      if (!lidar) {
        setText(age, 'No scan');
        setData(age, 'stale', 'true');
      } else {
        const elapsed = Math.max(0, now - lidar.ts);
        setText(age, elapsed < 1000 ? `${Math.round(elapsed)} ms old` : `${(elapsed / 1000).toFixed(1)} s old`);
        setData(age, 'stale', String(elapsed > STALE_MS));
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
