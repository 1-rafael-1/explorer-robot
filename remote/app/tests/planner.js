// The point-and-go planner suite: the preview, the icon guard, the numeric
// entry point, the zero-centidegree skip, the rotate leg and its wait endings.
// The sender and clock are injected, and the wait is ended by Store messages,
// never by a wall-clock sleep.

import { createPlanner } from '../js/planner.js';
import { MANUAL_DRIVE_ID, centidegrees } from '../js/protocol.js';
import { createStore } from '../js/store.js';
import { createFakeClock } from './fake-clock.js';
import { assert, assertClose, assertEqual, autorun, test } from './harness.js';

const RAD = Math.PI / 180;

/** Builds a connected store, a recording sender and a planner over a fake clock. */
function make({ pose = { x: 0, y: 0, yaw: 0 } } = {}) {
  const store = createStore();
  store.dispatch({ type: '@link', link: 'open', detail: '' });
  store.dispatch({ type: 'connection', state: 'connected', boot_id: 7, ts: 0 });
  store.dispatch({
    type: 'stream',
    name: 'pose',
    seq: 1,
    ts: 0,
    v: { x_cm: pose.x, y_cm: pose.y, yaw_deg: pose.yaw },
  });

  const clock = createFakeClock();
  const sent = [];
  let nextSeq = 1;
  const send = (name, args) => {
    const seq = nextSeq;
    nextSeq += 1;
    sent.push({ name, args, seq });
    return seq;
  };
  const announcements = [];
  const planner = createPlanner({
    store,
    send,
    now: clock.now,
    announce: (text, tone) => announcements.push({ text, tone }),
  });

  let eventSeq = 0;
  function ack(seq, overrides = {}) {
    store.dispatch({
      type: 'command_result',
      command_seq: seq,
      command: 'rotate_by',
      outcome: 'acked',
      status: 'ok',
      detail: null,
      ts: 0,
      ...overrides,
    });
  }

  function complete(seq, overrides = {}) {
    eventSeq += 1;
    store.dispatch({
      type: 'event',
      boot_id: store.state.bootId,
      seq: eventSeq,
      ts: 1000 + eventSeq,
      name: 'rotate_completed',
      v: {
        command_seq: seq,
        status: 'ok',
        target_deg: 0,
        error_deg: 0,
        duration_ms: 1,
        detail: 'done',
        ...overrides,
      },
    });
  }

  function activity(seq, stage) {
    store.dispatch({
      type: 'stream',
      name: 'activity',
      seq,
      ts: 1000 + seq,
      v: { id: MANUAL_DRIVE_ID, stage, percent: stage === 'running' ? null : 100, detail: '' },
    });
  }

  return { store, planner, clock, sent, announcements, ack, complete, activity };
}

test('a tap previews a target and a second tap replaces it; Cancel clears it', () => {
  const h = make();

  assertEqual(h.planner.pick({ x: 100, y: 0 }), true);
  const first = h.store.state.drive.plan;
  assertEqual(first.target, { x: 100, y: 0 });
  assertClose(first.distance_cm, 100, 1e-9);
  assertClose(first.bearing_deg, 0, 1e-9);
  assertClose(first.angle_deg, 0, 1e-9);

  assertEqual(h.planner.pick({ x: 0, y: 100 }), true);
  const second = h.store.state.drive.plan;
  assertEqual(second.target, { x: 0, y: 100 });
  assertClose(second.distance_cm, 100, 1e-9);
  assertClose(second.bearing_deg, 90, 1e-9);
  assertClose(second.angle_deg, 90, 1e-9);

  h.planner.cancel();
  assertEqual(h.store.state.drive.plan, null);
  assertEqual(h.store.state.drive.stage, 'idle');
  assertEqual(h.sent.length, 0);
});

test('a tap inside the drawn robot footprint produces no command', () => {
  const h = make();

  // The default drawn footprint is the conservative minimum icon.
  assertEqual(h.planner.pick({ x: 5, y: 0 }), false);
  assertEqual(h.store.state.drive.plan, null);
  assert(h.announcements.some((item) => /outside the robot footprint/.test(item.text)));

  // A radar tap already knows it is inside its own drawn icon.
  assertEqual(h.planner.pick({ x: 200, y: 200 }, { inside_robot: true }), false);
  assertEqual(h.store.state.drive.plan, null);

  // A small leg outside the icon is allowed.
  assertEqual(h.planner.pick({ x: 15, y: 10 }), true);
  assertClose(h.store.state.drive.plan.distance_cm, Math.hypot(15, 10), 1e-9);
  assertEqual(h.sent.length, 0);
});

test('the numeric form creates the same Target Point from the current pose', () => {
  const h = make({ pose: { x: 0, y: 0, yaw: 30 } });

  assertEqual(h.planner.numeric({ bearing_deg: 60, distance_cm: 100 }), true);
  const plan = h.store.state.drive.plan;
  assertClose(plan.target.x, 0, 1e-9);
  assertClose(plan.target.y, 100, 1e-9);
  assertClose(plan.bearing_deg, 90, 1e-9);
  assertClose(plan.angle_deg, 60, 1e-9);
  assertClose(plan.distance_cm, 100, 1e-9);
  assertEqual(h.sent.length, 0);
});

test('a bearing that rounds to zero centidegrees sends no rotate', async () => {
  const h = make();
  h.planner.preview({ x: 100, y: 0.004 });

  const report = await h.planner.go();
  assertEqual(h.sent.length, 0, 'no rotate and no drive');
  assertEqual(report.title, 'No rotation needed');
  assertEqual(h.store.state.drive.stage, 'idle');
});

test('a non-zero bearing sends rotate_by with the fixed speed and the rounded angle', async () => {
  const h = make();
  h.planner.preview({ x: 100, y: 100 });
  const running = h.planner.go();
  const seq = h.sent[0].seq;
  assertEqual(h.sent[0].name, 'rotate_by');
  assertEqual(h.sent[0].args, { angle_deg: 45, speed: 60 });
  h.ack(seq);
  h.complete(seq, { target_deg: 45, error_deg: 0, detail: 'done' });
  await running;

  const rounded = make();
  rounded.planner.preview({ x: 100, y: 0.1 });
  const angle = Math.atan2(0.1, 100) / RAD;
  const expected = centidegrees(angle) / 100;
  assert(expected !== 0, 'the tiny bearing still needs a rotate');
  const pending = rounded.planner.go();
  const roundedSeq = rounded.sent[0].seq;
  assertEqual(rounded.sent[0].args, { angle_deg: expected, speed: 60 });
  rounded.ack(roundedSeq);
  rounded.complete(roundedSeq, { target_deg: expected, error_deg: 0, detail: 'done' });
  await pending;
});

test('a very short and a very long leg both plan without a cap or refusal', async () => {
  const short = make();
  short.planner.preview({ x: 15, y: 10 });
  const shortPlan = short.store.state.drive.plan;
  assertClose(shortPlan.distance_cm, Math.hypot(15, 10), 1e-9);
  assert(shortPlan.distance_cm < 30, 'a short leg is not refused');
  assertEqual(short.announcements.length, 0, 'a short leg raises no warning');
  const shortRun = short.planner.go();
  assertEqual(short.sent[0].name, 'rotate_by', 'a short leg still sends its rotate');
  short.planner.stop();
  await shortRun;

  const long = make();
  long.planner.preview({ x: 4000, y: 3000 });
  const longPlan = long.store.state.drive.plan;
  assertClose(longPlan.distance_cm, 5000, 1e-9);
  assert(longPlan.distance_cm > 300, 'a leg beyond the old 3 m cap still plans');
  assertEqual(long.announcements.length, 0, 'a long leg raises no warning');
  const longRun = long.planner.go();
  assertEqual(long.sent[0].name, 'rotate_by', 'a long leg still sends its rotate');
  long.planner.stop();
  await longRun;
});

test('a successful rotate reports target, actual and the reason', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  const seq = h.sent[0].seq;
  h.ack(seq);
  h.complete(seq, { target_deg: 90, error_deg: 0.4, detail: 'done' });

  const report = await running;
  assertEqual(report.kind, 'success');
  assertEqual(report.title, 'Rotation complete');
  assertClose(report.target_deg, 90, 1e-9);
  assertClose(report.actual_deg, 90.4, 1e-9);
  assertEqual(report.reason, 'done');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
  assertEqual(h.store.state.drive.stage, 'idle');
});

test('a failed or cancelled rotate sends no drive leg', async () => {
  for (const status of ['failed', 'cancelled']) {
    const h = make();
    h.planner.preview({ x: 0, y: 100 });
    const running = h.planner.go();
    const seq = h.sent[0].seq;
    h.ack(seq);
    h.complete(seq, {
      status,
      target_deg: 90,
      error_deg: -1.5,
      detail: status === 'failed' ? 'Obstacle' : 'Stopped',
    });

    const report = await running;
    assertEqual(
      h.sent.filter((message) => message.name === 'drive_distance').length,
      0,
      `${status} rotates send no drive`,
    );
    assertEqual(report.status, status);
    assertClose(report.actual_deg, 88.5, 1e-9);
    assertEqual(report.reason, status === 'failed' ? 'Obstacle' : 'Stopped');
    assertEqual(report.kind, status === 'failed' ? 'bad' : 'warn');
    assertEqual(report.title, status === 'failed' ? 'Rotation failed' : 'Rotation cancelled');
  }
});

test('a rejected rotate ACK ends the wait without a completion', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  const seq = h.sent[0].seq;
  h.ack(seq, { outcome: 'rejected', status: 'busy', detail: 'Another Activity' });

  const report = await running;
  assertEqual(report.kind, 'bad');
  assertEqual(report.title, 'Rotation rejected');
  assertEqual(report.reason, 'Another Activity');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

test('a link loss ends the wait and reports the loss', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);
  h.store.dispatch({ type: '@link', link: 'reconnecting' });

  const report = await running;
  assertEqual(report.kind, 'warn');
  assertEqual(report.title, 'Link lost');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

test('a Boot Id change ends the wait as unknown', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);
  h.store.dispatch({ type: 'connection', state: 'connected', boot_id: 8, ts: 2000 });

  const report = await running;
  assertEqual(report.kind, 'unknown');
  assertEqual(report.title, 'Outcome unknown');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

test('Stop ends the wait as not confirmed', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);
  h.planner.stop();

  const report = await running;
  assertEqual(report.kind, 'warn');
  assertEqual(report.title, 'Stop not confirmed');
  assert(/link loss/.test(report.text));
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

test('the wait has no fixed completion timeout', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);

  let settled = false;
  running.then(() => {
    settled = true;
  });
  await Promise.resolve();
  h.clock.advance(24 * 60 * 60 * 1000);
  await Promise.resolve();
  assertEqual(settled, false, 'the wait survives any fixed timeout');

  h.planner.stop();
  await running;
});

test('the Activity leaving running ends the wait', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);
  h.activity(1, 'running');
  h.activity(2, 'complete');

  const report = await running;
  assertEqual(report.kind, 'warn');
  assertEqual(report.title, 'Rotation ended');
  assertEqual(report.reason, 'activity ended');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

autorun();
