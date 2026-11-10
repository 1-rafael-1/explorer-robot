import {
  CODE_ALPHABET,
  labelForActivity,
  normalise180,
  normalise360,
  needsRotate,
  centidegrees,
  formatCode,
  isCompleteCode,
  invalidCodeCharacters,
  codeHint,
  normaliseCode,
} from '../js/protocol.js';
import {
  FOOTPRINT,
  bearingDeg,
  distanceCm,
  fitScale,
  planLeg,
  pointInFootprint,
  relativeBearingDeg,
  scanPoints,
  screenToWorld,
  slotAngleDeg,
  worldToScreen,
} from '../js/geometry.js';
import { apply, createStore, initialState, latestPose, powerView, unreadEvents } from '../js/store.js';
import { assert, assertClose, assertEqual, autorun, test } from './harness.js';

test('the pairing code is normalised and validated against the panel alphabet', () => {
  assertEqual(normaliseCode('abcd-efgh jkmn'), 'ABCDEFGHJKMN');
  assertEqual(formatCode('abcdefghjkmn'), 'ABCD-EFGH-JKMN');
  assert(isCompleteCode('ABCD EFGH JKMN'));
  assert(!isCompleteCode('ABCD EFGH JKM'));
  assert(!isCompleteCode('ABCD EFGH JKMI'));
  assert(invalidCodeCharacters('ABCDILOU').includes('I'));
  assert(invalidCodeCharacters('ABCDILOU').includes('L'));
  assert(invalidCodeCharacters('ABCDILOU').includes('O'));
  assert(invalidCodeCharacters('ABCDILOU').includes('U'));
  assertEqual(invalidCodeCharacters(CODE_ALPHABET), []);
  assert(/I, L, O and U/.test(codeHint('ABCDILOU')));
});

test('Activity labels are local only for Idle, Booting and Manual Drive', () => {
  assertEqual(labelForActivity(0), 'Idle');
  assertEqual(labelForActivity(1), 'Booting');
  assertEqual(labelForActivity(0x000d), 'Manual Drive');
  assertEqual(labelForActivity(5, [{ id: 5, label: 'Coast and Avoid' }]), 'Coast and Avoid');
  assertEqual(labelForActivity(99, []), 'Activity 0x0063');
});

test('angles normalise and the rotate skip is a zero centidegree', () => {
  assertEqual(normalise180(190), -170);
  assertEqual(normalise360(-10), 350);
  assert(!needsRotate(0.004));
  assert(needsRotate(0.006));
  assertEqual(centidegrees(12.345), 1235);
});

test('bearing and distance follow the world frame', () => {
  const pose = { x: 0, y: 0, yaw: 0 };
  assertClose(distanceCm(pose, { x: 100, y: 0 }), 100, 1e-9);
  assertClose(bearingDeg(pose, { x: 100, y: 0 }), 0, 1e-9);
  assertClose(bearingDeg(pose, { x: 0, y: 100 }), 90, 1e-9);
  assertClose(relativeBearingDeg({ x: 0, y: 0, yaw: 90 }, { x: 0, y: 100 }), 0, 1e-9);
  assertClose(relativeBearingDeg({ x: 0, y: 0, yaw: 90 }, { x: 100, y: 0 }), -90, 1e-9);
});

test('the two-leg plan carries the target, bearing and centidegree turn', () => {
  const leg = planLeg({ x: 0, y: 0, yaw: 0 }, { x: 0, y: 100 });
  assertEqual(leg.target, { x: 0, y: 100 });
  assertClose(leg.distance_cm, 100, 1e-9);
  assertClose(leg.bearing_deg, 90, 1e-9);
  assertClose(leg.angle_deg, 90, 1e-9);
  assertEqual(leg.rotate_centideg, 9000);
  assert(leg.needs_rotate);
});

test('LiDAR slot 0 is ahead and the slots run clockwise', () => {
  assertClose(slotAngleDeg({ x: 0, y: 0, yaw: 0 }, 0, 1, true, 90), -90, 1e-9);
  assertClose(slotAngleDeg({ x: 0, y: 0, yaw: 0 }, 0, 1, false, 90), 90, 1e-9);
  const points = scanPoints({ x: 0, y: 0, yaw: 0 }, { ranges_cm: [100, null, 200], first_deg: 0, step_deg: 1, clockwise: true });
  assertEqual(points.length, 2);
  assertClose(points[0].x, 100, 1e-9);
  assertClose(points[0].y, 0, 1e-9);
  assertClose(points[1].x, 200 * Math.cos(-2 * (Math.PI / 180)), 1e-9);
});

test('the camera transform round-trips a world point', () => {
  const view = { width: 800, height: 600, scale: 2, centerX: 50, centerY: -20, cx: 400, cy: 300 };
  const screen = worldToScreen(view, { x: 120, y: 30 });
  const world = screenToWorld(view, screen.x, screen.y);
  assertClose(world.x, 120, 1e-9);
  assertClose(world.y, 30, 1e-9);
  assert(fitScale(800, 600) > 0);
});

test('the tap guard is the drawn robot footprint', () => {
  const pose = { x: 0, y: 0, yaw: 0 };
  assert(pointInFootprint(pose, { x: 5, y: 0 }, 1));
  assert(!pointInFootprint(pose, { x: 5 + FOOTPRINT.halfLengthCm * 4, y: 0 }, 1));
  assert(pointInFootprint({ x: 0, y: 0, yaw: 90 }, { x: 0, y: 5 }, 1));
  assert(!pointInFootprint({ x: 0, y: 0, yaw: 90 }, { x: 30, y: 0 }, 1));
});

test('the store merges a snapshot, dedupes streams and keeps unknown as null', () => {
  let state = initialState();
  state = apply(state, { type: 'hello', protocol: 1, server: { name: 'explorer-remote' }, robot: { model: null } });
  state = apply(state, {
    type: 'snapshot',
    protocol: 1,
    boot_id: 42,
    ts: 1000,
    streams: [
      { name: 'power', seq: 1, ts: 1000, v: { level: null, voltage_mv: 12000 } },
      { name: 'motion', seq: 1, ts: 1000, v: { left: 10, right: -10 } },
    ],
    last_terminal: null,
    events: [],
    event_cursor: 0,
    catalogue: { procedures: [{ id: 5, label: 'Coast and Avoid' }] },
    bridge: null,
  });
  assertEqual(state.bootId, 42);
  assertEqual(state.streams.power.v.level, null);
  assertEqual(state.streams.motion.v.left, 10);
  assertEqual(state.catalogue[0].label, 'Coast and Avoid');
  assertEqual(latestPose(state), { x: 0, y: 0, yaw: 0, ts: 0, identity: true });
  assertEqual(powerView(state), { level: null, voltage_mv: 12000, ts: 1000 });

  const after = apply(state, { type: 'stream', name: 'power', seq: 1, ts: 2000, v: { level: 50, voltage_mv: 12000 } });
  assertEqual(after, state, 'an older or equal stream sequence must be ignored');
});

test('Events dedupe by boot id and sequence; overflow is a notice', () => {
  let state = { ...initialState(), bootId: 7 };
  const event = { type: 'event', boot_id: 7, seq: 3, ts: 10, name: 'obstacle', v: { detected: true, source: 'lidar' } };
  state = apply(state, event);
  state = apply(state, event);
  assertEqual(state.events.length, 1);
  state = apply(state, { type: 'event', boot_id: 7, seq: 5, ts: 20, name: 'event_overflow', v: { dropped: 2 } });
  assertEqual(state.overflowAt, 20);
  assert(state.eventGap, 'a skipped sequence marks a gap');
});

test('a Boot Id change clears robot state; a same-Boot-Id snapshot keeps the plan', () => {
  const plan = { target: { x: 100, y: 0 }, distance_cm: 100, angle_deg: 0, pose: { x: 0, y: 0, yaw: 0 } };
  let state = {
    ...initialState(),
    bootId: 1,
    streams: { activity: { name: 'activity', seq: 1, ts: 1, v: { id: 0, stage: 'running', percent: null, detail: '' } } },
    drive: { plan, stage: 'idle', report: null, sequence: null },
  };
  state = apply(state, {
    type: 'snapshot',
    protocol: 1,
    boot_id: 1,
    ts: 2,
    streams: [],
    last_terminal: null,
    events: [],
    event_cursor: 0,
    catalogue: null,
    bridge: null,
  });
  assertEqual(state.drive.plan, plan, 'a same-Boot-Id snapshot preserves the plan');

  state = apply(state, {
    type: 'snapshot',
    protocol: 1,
    boot_id: 2,
    ts: 3,
    streams: [],
    last_terminal: null,
    events: [],
    event_cursor: 0,
    catalogue: null,
    bridge: null,
  });
  assertEqual(state.bootId, 2);
  assertEqual(state.streams, {});
  assertEqual(state.drive.plan, null);
  assertEqual(state.drive.report.kind, 'unknown');
});

test('a terminal Activity becomes the Result Report and an operator stop is a Stopped card', () => {
  let state = { ...initialState(), bootId: 1 };
  state = apply(state, { type: 'stream', name: 'activity', seq: 1, ts: 10, v: { id: 5, stage: 'running', percent: 20, detail: '' } });
  state = apply(state, { type: 'stream', name: 'activity', seq: 2, ts: 20, v: { id: 5, stage: 'complete', percent: 100, detail: 'done' } });
  assertEqual(state.lastTerminal.v.stage, 'complete');

  state = { ...initialState(), bootId: 1 };
  state = apply(state, { type: 'stream', name: 'activity', seq: 1, ts: 10, v: { id: 5, stage: 'running', percent: 20, detail: '' } });
  state = apply(state, { type: 'stream', name: 'activity', seq: 2, ts: 20, v: { id: 0, stage: 'running', percent: null, detail: '' } });
  assert(state.activityStopped, 'leaving running without a terminal event is a Stopped card');
});

test('the unread count follows the replay cursor', () => {
  let state = { ...initialState(), bootId: 1, eventCursor: 2, seenEventSeq: 2 };
  state = apply(state, { type: 'event', boot_id: 1, seq: 3, ts: 10, name: 'obstacle', v: { detected: true, source: 'lidar' } });
  assertEqual(unreadEvents(state), 1);
  state = apply(state, { type: '@seen' });
  assertEqual(unreadEvents(state), 0);
});

test('the store notifies subscribers and ignores unknown messages', () => {
  const store = createStore();
  let calls = 0;
  const off = store.subscribe(() => {
    calls += 1;
  });
  store.dispatch({ type: 'mystery' });
  assertEqual(calls, 0, 'an unknown message leaves the state alone');
  store.dispatch({ type: '@link', link: 'open' });
  assertEqual(calls, 1);
  assertEqual(store.state.link, 'open');
  off();
});

autorun();
