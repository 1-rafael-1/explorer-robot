// The point-and-go planner suite: the preview, the icon guard, the numeric
// entry point, the zero-centidegree skip, the rotate and drive legs, their wait
// endings, the busy/unknown outcomes and the context-sensitive Stop.
//
// The sender and clock are injected, and every wait is ended by Store messages,
// never by a wall-clock sleep or a poll.

import { createPlanner } from '../js/planner.js';
import { DRIVE_SPEED, MANUAL_DRIVE_ID, centidegrees } from '../js/protocol.js';
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

  function driveComplete(seq, overrides = {}) {
    eventSeq += 1;
    const drive = sent.find((message) => message.name === 'drive_distance');
    const target = drive?.args?.distance_cm ?? 0;
    store.dispatch({
      type: 'event',
      boot_id: store.state.bootId,
      seq: eventSeq,
      ts: 1000 + eventSeq,
      name: 'drive_distance_completed',
      v: {
        command_seq: seq,
        status: 'ok',
        target_cm: target,
        actual_cm: target,
        duration_ms: 1,
        detail: 'done',
        ...overrides,
      },
    });
  }

  function activity(seq, stage, id = MANUAL_DRIVE_ID) {
    store.dispatch({
      type: 'stream',
      name: 'activity',
      seq,
      ts: 1000 + seq,
      v: { id, stage, percent: stage === 'running' ? null : 100, detail: '' },
    });
  }

  return { store, planner, clock, sent, announcements, ack, complete, driveComplete, activity };
}

/** Runs a plan through its rotate leg and returns the drive's command seq. */
function reachDrive(h, { angle = 90, distance = 100 } = {}) {
  h.planner.preview({ x: 0, y: distance });
  const running = h.planner.go();
  const rotateSeq = h.sent[0].seq;
  h.ack(rotateSeq);
  h.complete(rotateSeq, { target_deg: angle, error_deg: 0, detail: 'done' });
  const drive = h.sent.find((message) => message.name === 'drive_distance');
  return { running, rotateSeq, driveSeq: drive?.seq ?? null };
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

test('a bearing that rounds to zero centidegrees sends the drive directly', async () => {
  const h = make();
  h.planner.preview({ x: 100, y: 0.004 });

  const running = h.planner.go();
  assertEqual(h.sent.map((message) => message.name), ['drive_distance'], 'no rotate');
  const seq = h.sent[0].seq;
  assertEqual(h.sent[0].args, { distance_cm: 100, speed: DRIVE_SPEED });

  h.ack(seq, { command: 'drive_distance' });
  h.driveComplete(seq, { target_cm: 100, actual_cm: 100, detail: 'done' });
  const report = await running;
  assertEqual(report.title, 'Drive complete');
  assertEqual(h.store.state.drive.stage, 'idle');
});

test('a non-zero bearing sends rotate_by then drive_distance with the fixed speeds', async () => {
  const h = make();
  h.planner.preview({ x: 100, y: 100 });
  const running = h.planner.go();
  const rotateSeq = h.sent[0].seq;
  assertEqual(h.sent[0].name, 'rotate_by');
  assertEqual(h.sent[0].args, { angle_deg: 45, speed: 60 });

  h.ack(rotateSeq);
  assertEqual(
    h.sent.map((message) => message.name),
    ['rotate_by'],
    'the rotate ACK alone sends no drive; the completion is the outcome',
  );
  h.complete(rotateSeq, { target_deg: 45, error_deg: 0, detail: 'done' });
  assertEqual(h.sent.map((message) => message.name), ['rotate_by', 'drive_distance']);
  const drive = h.sent[1];
  assertEqual(drive.args, { distance_cm: Math.round(Math.hypot(100, 100)), speed: DRIVE_SPEED });

  h.ack(drive.seq, { command: 'drive_distance' });
  h.driveComplete(drive.seq, { detail: 'done' });
  await running;

  // The bearing is rounded to the Relay's centidegree precision before sending.
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
  const roundedDrive = rounded.sent[1].seq;
  rounded.ack(roundedDrive, { command: 'drive_distance' });
  rounded.driveComplete(roundedDrive);
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
  assertEqual(short.sent[1].name, 'stop_motion');
  assertEqual(short.sent[1].args, { brake: 1 });
  short.ack(short.sent[1].seq, { command: 'stop_motion' });
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
  long.ack(long.sent[1].seq, { command: 'stop_motion' });
  await longRun;
});

test('the drive completion reports target, actual and the reason', async () => {
  const cases = [
    { status: 'ok', detail: 'done', title: 'Drive complete', kind: 'success' },
    { status: 'cancelled', detail: 'Stopped', title: 'Drive stopped', kind: 'warn' },
    { status: 'failed', detail: 'Obstacle', title: 'Drive failed', kind: 'bad' },
  ];

  for (const item of cases) {
    const h = make();
    const { running, driveSeq } = reachDrive(h, { angle: 90, distance: 100 });
    h.ack(driveSeq, { command: 'drive_distance' });
    h.driveComplete(driveSeq, {
      status: item.status,
      target_cm: 100,
      actual_cm: 82,
      detail: item.detail,
    });

    const report = await running;
    assertEqual(report.title, item.title, `${item.status} title`);
    assertEqual(report.kind, item.kind, `${item.status} kind`);
    assertEqual(report.status, item.status);
    assertEqual(report.target_cm, 100);
    assertEqual(report.actual_cm, 82);
    assertEqual(report.reason, item.detail);
    assert(/Target 100 cm/.test(report.text), 'target in the copy');
    assert(/actual 82 cm/.test(report.text), 'actual in the copy');
    assertEqual(h.store.state.drive.stage, 'idle');
  }
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

test('a rejected rotate ACK ends the wait without a completion or a drive', async () => {
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

test('a busy drive leg reports rotation done, drive not sent, and does not retry', async () => {
  const h = make();
  const { running, driveSeq } = reachDrive(h, { angle: 90, distance: 100 });
  h.ack(driveSeq, {
    command: 'drive_distance',
    outcome: 'rejected',
    status: 'busy',
    detail: 'Another Activity',
  });

  const report = await running;
  assertEqual(report.kind, 'busy');
  assertEqual(report.title, 'Rotation done, drive not sent');
  assertEqual(report.status, 'busy');
  assertEqual(report.reason, 'Another Activity');
  assertEqual(
    h.sent.filter((message) => message.name === 'drive_distance').length,
    1,
    'the drive is not retried',
  );
  assertEqual(h.store.state.drive.stage, 'idle');
});

test('an indeterminate or timed-out drive leg is unknown and is not retried', async () => {
  for (const outcome of ['indeterminate', 'timeout']) {
    const h = make();
    const { running, driveSeq } = reachDrive(h);
    h.ack(driveSeq, {
      command: 'drive_distance',
      outcome,
      status: outcome === 'timeout' ? 'unknown' : 'stale',
      detail: null,
    });

    const report = await running;
    assertEqual(report.kind, 'unknown', `${outcome} is unknown, never success`);
    assertEqual(report.title, 'Outcome unknown');
    assert(
      report.text.includes(outcome === 'timeout' ? 'timed out' : 'outcome is unknown'),
      `${outcome} copy`,
    );
    assertEqual(
      h.sent.filter((message) => message.name === 'drive_distance').length,
      1,
      `${outcome} is not retried`,
    );
    // The plan stays, so the operator can re-issue with Go.
    assert(h.store.state.drive.plan != null, 'the target survives for a re-issue');
    assertEqual(h.store.state.drive.stage, 'idle');
  }
});

test('an indeterminate rotate is unknown and sends no drive', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq, { outcome: 'timeout', status: 'unknown', detail: null });

  const report = await running;
  assertEqual(report.kind, 'unknown');
  assertEqual(report.title, 'Outcome unknown');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

test('a link loss ends the rotate wait and reports the loss', async () => {
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

test('a link loss ends the drive wait and reports the loss', async () => {
  const h = make();
  const { running, driveSeq } = reachDrive(h);
  h.ack(driveSeq, { command: 'drive_distance' });
  h.store.dispatch({ type: '@link', link: 'reconnecting' });

  const report = await running;
  assertEqual(report.kind, 'warn');
  assertEqual(report.title, 'Link lost');
  assert(/drive outcome is unknown/.test(report.text));
  assertEqual(
    h.sent.filter((message) => message.name === 'drive_distance').length,
    1,
    'no retry on link loss',
  );
});

test('a Boot Id change ends the rotate wait as unknown and clears the plan', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);
  h.store.dispatch({ type: 'connection', state: 'connected', boot_id: 8, ts: 2000 });

  const report = await running;
  assertEqual(report.kind, 'unknown');
  assertEqual(report.title, 'Outcome unknown');
  assertEqual(h.store.state.drive.plan, null, 'a Boot Id change clears the plan');
  assertEqual(h.sent.map((message) => message.name), ['rotate_by']);
});

test('a Boot Id change ends the drive wait as unknown', async () => {
  const h = make();
  const { running, driveSeq } = reachDrive(h);
  h.ack(driveSeq, { command: 'drive_distance' });
  h.store.dispatch({ type: 'connection', state: 'connected', boot_id: 8, ts: 2000 });

  const report = await running;
  assertEqual(report.kind, 'unknown');
  assertEqual(report.title, 'Outcome unknown');
  assertEqual(h.store.state.drive.plan, null);
});

test('Stop ends the motion wait and is not confirmed without an ok ACK', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  h.ack(h.sent[0].seq);

  h.planner.stop();
  assertEqual(h.sent.map((message) => message.name), ['rotate_by', 'stop_motion']);
  assertEqual(h.sent[1].args, { brake: 1 }, 'Stop brakes a Manual Drive');

  h.ack(h.sent[1].seq, {
    command: 'stop_motion',
    outcome: 'rejected',
    status: 'stale',
    detail: 'Robot link lost',
  });

  const report = await running;
  assertEqual(report.kind, 'warn');
  assertEqual(report.title, 'Stop not confirmed');
  assert(/link loss/.test(report.text), 'the app does not overclaim');
});

test('Stop shows Stopped only on an acked/ok ACK', async () => {
  const h = make();
  const { running, driveSeq } = reachDrive(h);
  h.ack(driveSeq, { command: 'drive_distance' });

  const stopping = h.planner.stop();
  const stopSeq = h.sent[h.sent.length - 1].seq;
  assertEqual(h.sent[h.sent.length - 1].name, 'stop_motion');
  assertEqual(h.sent[h.sent.length - 1].args, { brake: 1 });

  let settled = false;
  running.then(() => {
    settled = true;
  });
  await Promise.resolve();
  assertEqual(settled, false, 'the wait is not resolved before the Stop ACK');

  h.ack(stopSeq, { command: 'stop_motion', outcome: 'acked', status: 'ok' });
  const report = await running;
  assertEqual(report.kind, 'success');
  assertEqual(report.title, 'Stopped');
  assertEqual(h.store.state.drive.stage, 'idle');
  await stopping;
});

test('Stop sends stop_motion(brake=1) for a live Manual Drive', async () => {
  const h = make();
  h.activity(1, 'running');

  h.planner.stop();
  assertEqual(h.sent.map((message) => message.name), ['stop_motion']);
  assertEqual(h.sent[0].args, { brake: 1 });

  h.ack(h.sent[0].seq, { command: 'stop_motion' });
  assertEqual(h.store.state.drive.report.title, 'Stopped');
});

test('Stop sends stop_procedure{id} for a live Procedure', async () => {
  const h = make();
  h.activity(1, 'running', 5);

  h.planner.stop();
  assertEqual(h.sent.map((message) => message.name), ['stop_procedure']);
  assertEqual(h.sent[0].args, { id: 5 });

  h.ack(h.sent[0].seq, { command: 'stop_procedure' });
  assertEqual(h.store.state.drive.report.title, 'Stopped');
});

test('Stop without an ok ACK does not claim Stopped for a foreign claim', async () => {
  const h = make();
  h.activity(1, 'running', 5);

  h.planner.stop();
  h.ack(h.sent[0].seq, {
    command: 'stop_procedure',
    outcome: 'timeout',
    status: 'unknown',
    detail: null,
  });

  assertEqual(h.store.state.drive.report.title, 'Stop not confirmed');
  assert(/link loss/.test(h.store.state.drive.report.text));
});

test('a long drive leg waits without a fixed timeout, then completes on the Event', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  const rotateSeq = h.sent[0].seq;
  h.ack(rotateSeq);
  h.complete(rotateSeq, { target_deg: 90 });
  const driveSeq = h.sent[1].seq;
  h.ack(driveSeq, { command: 'drive_distance' });
  assertEqual(h.store.state.drive.stage, 'drive');

  let settled = false;
  running.then(() => {
    settled = true;
  });
  await Promise.resolve();
  h.clock.advance(24 * 60 * 60 * 1000);
  await Promise.resolve();
  assertEqual(settled, false, 'the wait survives any fixed timeout');

  h.driveComplete(driveSeq, { target_cm: 100, actual_cm: 99, detail: 'done' });
  const report = await running;
  assertEqual(report.title, 'Drive complete');
  assertEqual(report.actual_cm, 99);
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

test('the Activity leaving running ends the drive wait', async () => {
  const h = make();
  const { running, driveSeq } = reachDrive(h);
  h.ack(driveSeq, { command: 'drive_distance' });
  h.activity(1, 'running');
  h.activity(2, 'complete');

  const report = await running;
  assertEqual(report.kind, 'warn');
  assertEqual(report.title, 'Drive ended');
  assertEqual(report.reason, 'activity ended');
});

test('a null command_seq completion is matched by leg and target, never the other leg', async () => {
  const h = make();
  h.planner.preview({ x: 0, y: 100 });
  const running = h.planner.go();
  const rotateSeq = h.sent[0].seq;
  h.ack(rotateSeq);

  // Waiting for the rotate: a null-attribution drive completion cannot end it.
  h.driveComplete(null, { target_cm: 100 });
  let settled = false;
  running.then(() => {
    settled = true;
  });
  await Promise.resolve();
  assertEqual(settled, false, 'the drive completion is not mistaken for the rotate');

  // The rotate itself arrives with a null attribution.
  h.complete(null, { target_deg: 90, status: 'ok' });
  const driveSeq = h.sent[1].seq;
  h.ack(driveSeq, { command: 'drive_distance' });

  // On the drive leg, a null-attribution rotate completion is never accepted,
  // and neither is a drive completion for another target.
  h.complete(null, { target_deg: 90, status: 'ok' });
  h.driveComplete(null, { target_cm: 999, actual_cm: 999, detail: 'other' });
  await Promise.resolve();
  assertEqual(settled, false, 'the other leg or another target is not this drive');

  h.driveComplete(null, { status: 'failed', target_cm: 100, actual_cm: 98, detail: 'Obstacle' });
  const report = await running;
  assertEqual(report.title, 'Drive failed');
  assertEqual(report.target_cm, 100);
  assertEqual(report.actual_cm, 98);
  assertEqual(report.reason, 'Obstacle');
});

autorun();
