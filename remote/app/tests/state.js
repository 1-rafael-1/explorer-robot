import {
  activityBusy,
  activityView,
  apply,
  controlEnabled,
  createStore,
  initialState,
  latestPose,
  lidarView,
  liveActivityId,
  markEventsSeen,
  motionView,
  outcomeUnknown,
  perceptionView,
  powerView,
  relayNow,
  unreadEvents,
} from '../js/store.js';
import { labelForActivity } from '../js/protocol.js';
import { assert, assertEqual, autorun, test } from './harness.js';

const CATALOGUE = {
  procedures: [
    { id: 5, label: 'Coast and Avoid', available: true, interactive: false, calibration: false, takes_params: false },
    { id: 6, label: 'Dock', available: true, interactive: true, calibration: false, takes_params: false },
  ],
};

/** Builds a daemon `snapshot` message with the fields this suite does not vary. */
function snapshot(overrides = {}) {
  return {
    type: 'snapshot',
    protocol: 1,
    boot_id: 1,
    ts: 1000,
    streams: [],
    last_terminal: null,
    events: [],
    event_cursor: 0,
    catalogue: null,
    bridge: null,
    ...overrides,
  };
}

/** Builds one wire Event. */
function event(seq, ts, name, v, bootId = 1) {
  return { type: 'event', boot_id: bootId, seq, ts, name, v };
}

test('a snapshot replaces every stream from the array and keeps unknown null', () => {
  const state = apply(
    initialState(),
    snapshot({
      streams: [
        { name: 'power', seq: 4, ts: 900, v: { level: null, voltage_mv: 12100 } },
        { name: 'motion', seq: 2, ts: 950, v: { left: 0, right: 35 } },
        { name: 'activity', seq: 1, ts: 940, v: { id: 0, stage: 'running', percent: null, detail: '' } },
        { name: 'perception', seq: 3, ts: 960, v: { obstacle: false, floor_drop: null } },
        {
          name: 'lidar',
          seq: 7,
          ts: 970,
          v: { ranges_cm: [null, 120], first_deg: 0, step_deg: 1, clockwise: true },
        },
        {
          name: 'calibration',
          seq: 1,
          ts: 800,
          v: { motor: 'loaded', imu: 'unknown', mag: 'missing', distance: 'loaded' },
        },
      ],
      catalogue: CATALOGUE,
    }),
  );

  assertEqual(state.bootId, 1);
  assertEqual(state.streams.motion.v.right, 35);
  assertEqual(state.streams.calibration.v.mag, 'missing');
  assertEqual(powerView(state), { level: null, voltage_mv: 12100, ts: 900 });
  assertEqual(motionView(state), { left: 0, right: 35, ts: 950 });
  assertEqual(perceptionView(state), { obstacle: false, floor_drop: null, ts: 960 });
  assertEqual(lidarView(state).present, true);
  assertEqual(lidarView(state).ranges_cm, [null, 120]);
  assertEqual(state.catalogue, CATALOGUE.procedures);
  assertEqual(latestPose(state), { x: 0, y: 0, yaw: 0, ts: 0, identity: true });
});

test('streams dedupe by name and sequence, never by name alone', () => {
  let state = apply(
    initialState(),
    snapshot({ streams: [{ name: 'motion', seq: 4, ts: 10, v: { left: 1, right: 2 } }] }),
  );

  const equalSeq = apply(state, {
    type: 'stream',
    name: 'motion',
    seq: 4,
    ts: 20,
    v: { left: 9, right: 9 },
  });
  assert(equalSeq === state, 'an equal sequence must be ignored');

  const olderSeq = apply(state, {
    type: 'stream',
    name: 'motion',
    seq: 3,
    ts: 20,
    v: { left: 9, right: 9 },
  });
  assert(olderSeq === state, 'an older sequence must be ignored');

  state = apply(state, { type: 'stream', name: 'motion', seq: 5, ts: 30, v: { left: 3, right: 4 } });
  assertEqual(motionView(state), { left: 3, right: 4, ts: 30 });

  const otherName = apply(state, {
    type: 'stream',
    name: 'perception',
    seq: 5,
    ts: 30,
    v: { obstacle: true, floor_drop: false },
  });
  assertEqual(perceptionView(otherName).obstacle, true, 'the same sequence on another stream is accepted');
});

test('Events dedupe by boot id and sequence; a notice dedupes by timestamp', () => {
  let state = apply(initialState(), snapshot({}));
  const obstacle = event(3, 10, 'obstacle', { detected: true, source: 'lidar' });

  state = apply(state, obstacle);
  const afterFirst = state;
  state = apply(state, obstacle);
  assert(state === afterFirst, 'a replayed Event must not be duplicated');
  assertEqual(state.events.length, 1);

  const notice = { type: 'event', boot_id: null, seq: null, ts: 50, name: 'event_overflow', v: { dropped: 2 } };
  state = apply(state, notice);
  state = apply(state, notice);
  assertEqual(state.events.filter((entry) => entry.name === 'event_overflow').length, 1);
  assertEqual(state.overflowAt, 50);
});

test('a snapshot merges Events by (boot_id, seq) and sets the scalar cursor', () => {
  let state = apply(
    initialState(),
    snapshot({
      events: [
        event(1, 10, 'obstacle', { detected: true, source: 'lidar' }),
        event(2, 20, 'floor_drop', { detected: false }),
      ],
      event_cursor: 2,
    }),
  );
  assertEqual(state.events.length, 2);
  assertEqual(state.eventCursor, 2);

  state = apply(
    state,
    snapshot({
      events: [
        event(2, 20, 'floor_drop', { detected: false }),
        event(3, 30, 'procedure_completed', { id: 5, stage: 'complete', detail: 'ok' }),
      ],
      event_cursor: 3,
    }),
  );
  assertEqual(state.events.map((entry) => entry.seq), [1, 2, 3], 'a replayed Event is merged once');
  assertEqual(state.eventCursor, 3);
});

test('the unread badge counts after the scalar replay cursor', () => {
  let state = apply(
    initialState(),
    snapshot({
      events: [
        event(1, 10, 'obstacle', { detected: true, source: 'lidar' }),
        event(2, 20, 'floor_drop', { detected: false }),
      ],
      event_cursor: 2,
    }),
  );
  assertEqual(unreadEvents(state), 0, 'the snapshot replay is at or before the cursor');

  state = apply(state, event(3, 30, 'obstacle', { detected: false, source: 'lidar' }));
  assertEqual(unreadEvents(state), 1, 'an Event after the cursor is unread');
  assertEqual(markEventsSeen(state).seenEventSeq, 3);

  const replayed = apply(
    markEventsSeen(state),
    snapshot({
      events: [
        event(3, 30, 'obstacle', { detected: false, source: 'lidar' }),
        event(4, 40, 'obstacle', { detected: true, source: 'lidar' }),
      ],
      event_cursor: 4,
    }),
  );
  assertEqual(unreadEvents(replayed), 0, 'a mid-session replay advances the cursor, not the badge');
  assertEqual(replayed.seenEventSeq, 4);
});

test('a same-Boot-Id snapshot keeps the plan; a Boot Id change marks it unknown', () => {
  const plan = {
    target: { x: 100, y: 0 },
    distance_cm: 100,
    angle_deg: 0,
    pose: { x: 0, y: 0, yaw: 0 },
  };
  let state = apply(
    initialState(),
    snapshot({
      streams: [
        { name: 'motion', seq: 1, ts: 10, v: { left: 1, right: 2 } },
        { name: 'activity', seq: 1, ts: 10, v: { id: 5, stage: 'running', percent: 20, detail: '' } },
      ],
      catalogue: CATALOGUE,
      events: [event(1, 10, 'obstacle', { detected: true, source: 'lidar' })],
      event_cursor: 1,
    }),
  );
  state = apply(state, { type: '@drive', drive: { plan, stage: 'idle', report: null, sequence: null } });

  const same = apply(
    state,
    snapshot({
      boot_id: 1,
      streams: [{ name: 'power', seq: 9, ts: 20, v: { level: 80, voltage_mv: 12000 } }],
      last_terminal: { name: 'activity', seq: 9, ts: 20, v: { id: 6, stage: 'complete', percent: 100, detail: '' } },
      catalogue: CATALOGUE,
      events: [],
      event_cursor: 1,
    }),
  );
  assertEqual(same.drive.plan, plan, 'a same-Boot-Id snapshot preserves the in-flight plan');
  assertEqual(same.streams.motion, undefined, 'streams are replaced, not accumulated');
  assertEqual(same.lastTerminal.v.id, 6, 'the last terminal is replaced');

  const sameConnection = apply(state, { type: 'connection', state: 'connected', boot_id: 1, ts: 30 });
  assertEqual(sameConnection.drive.plan, plan);

  const changed = apply(
    state,
    snapshot({
      boot_id: 2,
      streams: [],
      last_terminal: null,
      catalogue: null,
      events: [],
      event_cursor: null,
    }),
  );
  assertEqual(changed.bootId, 2);
  assertEqual(changed.streams, {});
  assertEqual(changed.events, []);
  assertEqual(changed.catalogue, []);
  assertEqual(changed.lastTerminal, null);
  assertEqual(changed.eventCursor, null);
  assertEqual(changed.seenEventSeq, 0);
  assertEqual(changed.eventGap, false);
  assertEqual(changed.overflowAt, null);
  assertEqual(changed.drive.plan, null);
  assertEqual(changed.drive.report.kind, 'unknown');

  const changedConnection = apply(state, { type: 'connection', state: 'connected', boot_id: 2, ts: 40 });
  assertEqual(changedConnection.bootId, 2);
  assertEqual(changedConnection.drive.plan, null);
  assertEqual(changedConnection.drive.report.kind, 'unknown');
});

test('the last terminal Activity is a Result Report after a reload', () => {
  const state = apply(
    initialState(),
    snapshot({
      streams: [{ name: 'activity', seq: 10, ts: 600, v: { id: 0, stage: 'running', percent: null, detail: '' } }],
      last_terminal: {
        name: 'activity',
        seq: 9,
        ts: 500,
        v: { id: 5, stage: 'failed', percent: null, detail: 'Link lost' },
      },
      catalogue: CATALOGUE,
    }),
  );
  const view = activityView(state);
  assertEqual(view.kind, 'result');
  assertEqual(view.data.stage, 'failed');
  assertEqual(view.data.detail, 'Link lost');
  assertEqual(labelForActivity(5, state.catalogue), 'Coast and Avoid');
});

test('a snapshot reconstructs the last terminal from a replayed completion Event', () => {
  const state = apply(
    initialState(),
    snapshot({
      streams: [],
      last_terminal: null,
      catalogue: CATALOGUE,
      events: [
        event(1, 10, 'obstacle', { detected: true, source: 'lidar' }),
        event(2, 20, 'procedure_completed', { id: 5, stage: 'complete', detail: 'done' }),
        event(3, 30, 'procedure_completed', { id: 6, stage: 'failed', detail: 'stalled' }),
      ],
      event_cursor: 3,
    }),
  );

  const view = activityView(state);
  assertEqual(view.kind, 'result', 'the replayed completion is a Result Report');
  assertEqual(view.data.id, 6, 'the newest replayed completion wins');
  assertEqual(view.data.stage, 'failed');
  assertEqual(view.data.detail, 'stalled');
  assertEqual(state.lastTerminal.seq, 0, 'the reconstructed entry uses the Event shape');

  const authoritative = apply(
    initialState(),
    snapshot({
      last_terminal: {
        name: 'activity',
        seq: 4,
        ts: 40,
        v: { id: 5, stage: 'complete', percent: 100, detail: 'from the Relay' },
      },
      events: [event(3, 30, 'procedure_completed', { id: 6, stage: 'failed', detail: 'stale' })],
      event_cursor: 4,
    }),
  );
  assertEqual(activityView(authoritative).data.id, 5, 'the Relay last_terminal stays authoritative');
  assertEqual(activityView(authoritative).data.detail, 'from the Relay');
});

test('a dismissed Result Report stays dismissed until a new Activity or terminal Event', () => {
  let state = apply(initialState(), snapshot({ catalogue: CATALOGUE }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 1,
    ts: 10,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 20,
    v: { id: 5, stage: 'complete', percent: 100, detail: 'done' },
  });
  assertEqual(activityView(state).kind, 'result');

  state = apply(state, { type: '@dismiss' });
  assertEqual(activityView(state).kind, 'idle', 'Dismiss clears the Result Report');

  state = apply(state, {
    type: 'stream',
    name: 'power',
    seq: 1,
    ts: 30,
    v: { level: 50, voltage_mv: 12000 },
  });
  assertEqual(activityView(state).kind, 'idle', 'an unrelated Stream does not resurrect it');

  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 3,
    ts: 40,
    v: { id: 6, stage: 'running', percent: 5, detail: '' },
  });
  assertEqual(activityView(state).kind, 'live', 'a new running Activity replaces it');

  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 4,
    ts: 50,
    v: { id: 6, stage: 'failed', percent: 100, detail: 'stalled' },
  });
  state = apply(state, { type: '@dismiss' });
  state = apply(state, event(1, 60, 'procedure_completed', { id: 6, stage: 'complete', detail: 'ok' }));
  assertEqual(activityView(state).kind, 'result', 'a new terminal Event ends the dismissal');
  assertEqual(activityView(state).data.stage, 'complete');
});

test('a terminal Activity stream or completion Event renders the Result Report', () => {
  let state = apply(initialState(), snapshot({ catalogue: CATALOGUE }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 1,
    ts: 10,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 20,
    v: { id: 5, stage: 'complete', percent: 100, detail: 'done' },
  });
  assertEqual(activityView(state).kind, 'result');
  assertEqual(activityView(state).data.percent, 100);

  let eventState = apply(initialState(), snapshot({ catalogue: CATALOGUE }));
  eventState = apply(eventState, {
    type: 'stream',
    name: 'activity',
    seq: 1,
    ts: 10,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  eventState = apply(eventState, event(1, 15, 'procedure_completed', { id: 5, stage: 'complete', detail: 'ok' }));
  eventState = apply(eventState, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 20,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });
  assertEqual(activityView(eventState).kind, 'result');
  assertEqual(activityView(eventState).data.stage, 'complete');
  assertEqual(eventState.activityStopped, null);
});

test('running to idle without a terminal stage or Event renders Stopped', () => {
  let state = apply(
    initialState(),
    snapshot({
      catalogue: CATALOGUE,
      last_terminal: {
        name: 'activity',
        seq: 1,
        ts: 5,
        v: { id: 6, stage: 'complete', percent: 100, detail: 'done' },
      },
      streams: [{ name: 'activity', seq: 2, ts: 10, v: { id: 5, stage: 'running', percent: 20, detail: '' } }],
    }),
  );
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 3,
    ts: 20,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });

  assert(state.activityStopped, 'leaving running without a terminal stage is Stopped');
  assertEqual(activityView(state).kind, 'stopped', 'the new Stopped card wins over the older Result Report');
  assertEqual(activityView(state).data.stage, 'stopped');
  assertEqual(labelForActivity(5, state.catalogue), 'Coast and Avoid');
});

test('a drive or rotate completion Event prevents a false Stopped card', () => {
  let state = apply(
    initialState(),
    snapshot({
      catalogue: CATALOGUE,
      streams: [
        { name: 'activity', seq: 1, ts: 10, v: { id: 0x000d, stage: 'running', percent: null, detail: '' } },
      ],
    }),
  );
  state = apply(state, event(1, 15, 'drive_distance_completed', { target_cm: 100, actual_cm: 100 }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 20,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });

  assertEqual(state.activityStopped, null, 'the drive completion is terminal');
  assertEqual(activityView(state).kind, 'idle', 'a successful Manual Drive never shows Stopped');
});

test('Event gaps and overflow notices make a live outcome unknown', () => {
  let state = apply(initialState(), snapshot({}));
  state = apply(state, event(1, 10, 'obstacle', { detected: true, source: 'lidar' }));
  assertEqual(outcomeUnknown(state), false);

  state = apply(state, event(4, 20, 'obstacle', { detected: false, source: 'lidar' }));
  assertEqual(state.eventGap, true);
  assertEqual(state.eventGapAt, 20);
  assertEqual(outcomeUnknown(state), true);
  assertEqual(outcomeUnknown(state, 21), false, 'a gap before a command does not taint it');

  const overflow = apply(
    initialState(),
    snapshot({
      events: [{ boot_id: null, seq: null, ts: 40, name: 'event_overflow', v: { dropped: 3 } }],
      event_cursor: null,
    }),
  );
  assertEqual(overflow.overflowAt, 40);
  assertEqual(overflow.eventGap, false);
  assertEqual(outcomeUnknown(overflow), true);
  assertEqual(outcomeUnknown(overflow, 41), false);
});

test('Activity labels come from the Catalogue except Idle, Booting and Manual Drive', () => {
  const state = apply(initialState(), snapshot({ catalogue: CATALOGUE }));
  assertEqual(labelForActivity(0, state.catalogue), 'Idle');
  assertEqual(labelForActivity(1, state.catalogue), 'Booting');
  assertEqual(labelForActivity(0x000d, state.catalogue), 'Manual Drive');
  assertEqual(labelForActivity(5, state.catalogue), 'Coast and Avoid');
  assertEqual(labelForActivity(6, state.catalogue), 'Dock');
  assertEqual(labelForActivity(99, state.catalogue), 'Activity 0x0063');
  assert(Array.isArray(state.catalogue), 'the app keeps the Catalogue array, not a Procedure table');
});

test('a live Activity exposes busy and the id for Stop without changing controlEnabled', () => {
  let state = apply(initialState(), snapshot({}));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 1,
    ts: 10,
    v: { id: 5, stage: 'running', percent: null, detail: 'patrolling' },
  });
  assertEqual(liveActivityId(state), 5);
  assertEqual(activityBusy(state), true);
  assertEqual(activityView(state).busy, true);

  state = apply(state, { type: '@link', link: 'open' });
  state = apply(state, { type: 'connection', state: 'connected', boot_id: 1, ts: 20 });
  assertEqual(controlEnabled(state), true, 'controlEnabled stays the connection truth');
  assertEqual(activityBusy(state), true);

  const idle = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 20,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });
  assertEqual(liveActivityId(idle), null);
  assertEqual(activityBusy(idle), false);

  const booting = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 3,
    ts: 30,
    v: { id: 1, stage: 'running', percent: null, detail: 'booting' },
  });
  assertEqual(activityBusy(booting), false, 'Booting is not a foreign claim');
});

test('a missing or omitted stream is unknown, never zero or a stale value', () => {
  let state = apply(
    initialState(),
    snapshot({
      streams: [
        { name: 'power', seq: 1, ts: 10, v: { level: null, voltage_mv: null } },
        { name: 'motion', seq: 1, ts: 10, v: { left: 0, right: -20 } },
        { name: 'perception', seq: 1, ts: 10, v: { obstacle: null, floor_drop: false } },
      ],
    }),
  );
  assertEqual(powerView(state), { level: null, voltage_mv: null, ts: 10 });
  assertEqual(motionView(state).left, 0, 'zero is a value, not unknown');
  assertEqual(perceptionView(state).obstacle, null);
  assertEqual(perceptionView(state).floor_drop, false, 'clear is a value, not unknown');

  state = apply(
    state,
    snapshot({ streams: [{ name: 'power', seq: 2, ts: 20, v: { level: 50, voltage_mv: 12000 } }] }),
  );
  assertEqual(motionView(state), { left: null, right: null, ts: null }, 'an omitted stream is unknown');
  assertEqual(lidarView(state), {
    present: false,
    ts: null,
    ranges_cm: null,
    first_deg: null,
    step_deg: null,
    clockwise: null,
  });

  const empty = initialState();
  assertEqual(powerView(empty), { level: null, voltage_mv: null, ts: null });
  assertEqual(motionView(empty), { left: null, right: null, ts: null });
  assertEqual(perceptionView(empty), { obstacle: null, floor_drop: null, ts: null });
});

test('apply never mutates the state it is given', () => {
  const start = apply(
    initialState(),
    snapshot({
      streams: [{ name: 'motion', seq: 1, ts: 10, v: { left: 1, right: 2 } }],
      catalogue: CATALOGUE,
      events: [event(1, 10, 'obstacle', { detected: true, source: 'lidar' })],
      event_cursor: 1,
    }),
  );
  const before = JSON.stringify(start);

  apply(start, { type: 'stream', name: 'motion', seq: 2, ts: 20, v: { left: 3, right: 4 } });
  apply(start, event(9, 30, 'obstacle', { detected: false, source: 'lidar' }));
  apply(start, snapshot({ boot_id: 2, streams: [], events: [], event_cursor: null }));

  assertEqual(JSON.stringify(start), before, 'apply must not mutate its argument');
  assertEqual(start.bootId, 1);
  assertEqual(motionView(start).left, 1);
});

test('the u16 Event sequence wraps without freezing the badge or hiding a gap', () => {
  let state = apply(initialState(), snapshot({ event_cursor: 65535 }));
  assertEqual(state.seenEventSeq, 65535);

  state = apply(state, event(0, 10, 'obstacle', { detected: true, source: 'lidar' }));
  assertEqual(unreadEvents(state), 1, 'sequence 0 after 65535 is a new Event');
  const seen = markEventsSeen(state);
  assertEqual(seen.seenEventSeq, 65536, 'the local cursor extends past the wire wrap');
  assertEqual(unreadEvents(seen), 0);

  const older = apply(seen, event(65535, 20, 'obstacle', { detected: false, source: 'lidar' }));
  assertEqual(unreadEvents(older), 0, 'a stale pre-wrap Event is not unread again');

  const withGap = apply(
    apply(initialState(), snapshot({ event_cursor: 65535 })),
    event(2, 30, 'obstacle', { detected: true, source: 'lidar' }),
  );
  assertEqual(withGap.eventGap, true, 'a dropped sequence after the wrap is still a gap');
});

test('the store derives one Relay-clock offset for planning and rendering', () => {
  const store = createStore();
  store.dispatch({ type: 'stream', name: 'power', seq: 1, ts: 5000, v: { level: 50, voltage_mv: 12000 } });
  const state = store.state;
  assert(Number.isFinite(state.clockOffset), 'the offset is a number');
  assert(
    Math.abs(relayNow(state) - 5000) < 1000,
    'relayNow reads the Relay clock, not the page clock',
  );
});

test('a reconnect snapshot keeps a replayed dismissal and reveals a new Result Report', () => {
  const dismissed = apply(
    apply(initialState(), {
      ...snapshot({
        catalogue: CATALOGUE,
        last_terminal: {
          name: 'activity',
          seq: 1,
          ts: 5,
          v: { id: 5, stage: 'complete', percent: 100, detail: 'done' },
        },
      }),
    }),
    { type: '@dismiss' },
  );
  assertEqual(activityView(dismissed).kind, 'idle');

  const replayed = apply(
    dismissed,
    snapshot({
      catalogue: CATALOGUE,
      last_terminal: {
        name: 'activity',
        seq: 2,
        ts: 6,
        v: { id: 5, stage: 'complete', percent: 100, detail: 'done' },
      },
    }),
  );
  assertEqual(activityView(replayed).kind, 'idle', 'a refresh of the same Result stays dismissed');

  const fresh = apply(
    dismissed,
    snapshot({
      catalogue: CATALOGUE,
      last_terminal: {
        name: 'activity',
        seq: 3,
        ts: 7,
        v: { id: 6, stage: 'failed', percent: 100, detail: 'stalled' },
      },
    }),
  );
  assertEqual(activityView(fresh).kind, 'result', 'a new Result clears the dismissal');
});

test('a refreshed terminal Activity sample does not restore a dismissed report', () => {
  let state = apply(initialState(), snapshot({ catalogue: CATALOGUE }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 1,
    ts: 10,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 20,
    v: { id: 5, stage: 'complete', percent: 100, detail: 'done' },
  });
  state = apply(state, { type: '@dismiss' });
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 3,
    ts: 30,
    v: { id: 5, stage: 'complete', percent: 100, detail: 'done' },
  });
  assertEqual(activityView(state).kind, 'idle', 'the same terminal sample is not a new Result');
});

test('a Stopped card needs a completion from the current run and the right Activity kind', () => {
  let state = apply(initialState(), snapshot({ catalogue: CATALOGUE }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 1,
    ts: 100,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  state = apply(state, event(1, 150, 'drive_distance_completed', { target_cm: 100, actual_cm: 100 }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 2,
    ts: 200,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });
  assert(state.activityStopped, 'a Manual Drive completion cannot end a Procedure run');

  state = apply(state, { type: '@dismiss' });
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 3,
    ts: 300,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  state = apply(state, event(2, 320, 'procedure_completed', { id: 5, stage: 'complete', detail: 'done' }));
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 4,
    ts: 340,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });
  assertEqual(state.activityStopped, null, 'the current run completed, so it is not Stopped');

  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 5,
    ts: 400,
    v: { id: 5, stage: 'running', percent: 10, detail: '' },
  });
  state = apply(state, {
    type: 'stream',
    name: 'activity',
    seq: 6,
    ts: 420,
    v: { id: 0, stage: 'running', percent: null, detail: '' },
  });
  assert(
    state.activityStopped,
    'a completion from the previous run cannot end the restarted run',
  );
});

autorun();
