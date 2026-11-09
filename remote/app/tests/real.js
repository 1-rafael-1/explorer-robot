// The real-chain suite: the app modules against the live Relay and Bridge Stub.
//
// The daemon serves this page and the app folder; the Relay discovers the
// Bridge Stub on loopback, opens the Session, and speaks the real App Boundary.
// Nothing here injects a socket: `createTransport` opens the same-origin `/ws`
// with a real `WebSocket`, and `store`, `planner`, `protocol` and `transport`
// are the modules the app ships. The page is a test, not a second app shell.
//
// The completion Events are not scripted (the stub ACKs commands but sends no
// drive/rotate completion), so the waits end on Stop, which is also the real
// contract: the planner is already covered by `planner.html`. If a completion
// Event does arrive, the report is asserted.

import { createPlanner } from '../js/planner.js';
import { createStore } from '../js/store.js';
import { createTransport } from '../js/transport.js';
import { assert, assertEqual, run, test } from './harness.js';

/** The longest any one predicate waits before the page fails. */
const WAIT_MS = 10_000;

/** How often a wait re-checks its predicate. */
const POLL_MS = 25;

/** Polls `predicate` in real time and fails with `description` after `timeout`. */
function waitFor(predicate, description, timeout = WAIT_MS) {
  const started = Date.now();
  return new Promise((resolve, reject) => {
    function poll() {
      let value;
      try {
        value = predicate();
      } catch (error) {
        reject(error);
        return;
      }
      if (value) {
        resolve(value);
        return;
      }
      if (Date.now() - started >= timeout) {
        reject(new Error(`timed out after ${timeout} ms waiting for ${description}`));
        return;
      }
      setTimeout(poll, POLL_MS);
    }
    poll();
  });
}

/** Opens one live store + transport pair on the Relay's `/ws`. */
function liveSession() {
  const store = createStore();
  const messages = [];
  const transport = createTransport({
    onLink: (link, detail) => store.dispatch({ type: '@link', link, detail }),
    onMessage: (message) => {
      messages.push(message);
      store.dispatch(message);
    },
  });
  try {
    transport.start();
  } catch {
    // A page loaded outside the daemon cannot open `/ws`; the waits below fail
    // with the connection state instead of an unhandled rejection.
  }
  return { store, transport, messages };
}

/** The one session the page drives; every test reads its store. */
const session = liveSession();

/** Builds a planner over the live session's store and real command sender. */
const planner = createPlanner({
  store: session.store,
  send: (name, args) => session.transport.command(name, args),
});

test('the Relay discovers the Bridge, opens the Session and sends the snapshot', async () => {
  const state = await waitFor(
    () => session.store.state.connection.state === 'connected' && session.store.state,
    'the Relay to open the Session',
  );
  assertEqual(state.link, 'open', 'the app socket is open');
  assertEqual(state.protocol, 1, 'the hello carries the App Boundary version');
  assertEqual(state.server?.name, 'explorer-remote', 'the hello names the Relay');
  assertEqual(state.connection.robot, 'Explorer-STUB', 'the Relay names the discovered robot');
  assertEqual(state.bootId, 1, 'the Bridge Stub reports Boot Id 1');
  assert(
    session.messages.some((message) => message.type === 'snapshot'),
    'the Relay sent an App Snapshot',
  );
});

test('the App Snapshot and live Streams carry motion and activity', async () => {
  const motion = await waitFor(() => session.store.state.streams.motion, 'the motion Stream');
  assert(motion && typeof motion.v === 'object' && motion.v != null, 'the motion sample has a value');
  assert(typeof motion.seq === 'number' && motion.seq > 0, 'the motion Stream has a sequence');

  const activity = await waitFor(() => session.store.state.streams.activity, 'the activity Stream');
  assert(activity && typeof activity.v === 'object' && activity.v != null, 'the activity sample has a value');
  assert(typeof activity.v.id === 'number', 'the activity sample has a Procedure id');
});

test('the scripted Events arrive once each and keep the Relay iterator', async () => {
  await waitFor(
    () => session.store.state.events.some((event) => event.name === 'obstacle'),
    'the obstacle Event',
  );
  await waitFor(
    () => session.store.state.events.some((event) => event.name === 'floor_drop'),
    'the floor_drop Event',
  );
  const events = session.store.state.events;
  const keys = events.map((event) => `${event.boot_id}:${event.seq}`);
  assertEqual(new Set(keys).size, keys.length, 'Events are deduped by (boot_id, seq)');
  assert(
    events.every((event) => event.boot_id === session.store.state.bootId),
    'every replayed Event belongs to the live Boot Id',
  );
});

test('a zero-bearing Target Point sends drive_distance and the Relay ACKs it', async () => {
  const { store } = session;
  assertEqual(planner.preview({ x: 100, y: 0 }), true, 'the Target Point is committed');
  assertEqual(store.state.drive.plan.target, { x: 100, y: 0 });

  const running = planner.go();
  const seq = store.state.drive.sequence;
  assert(seq != null, 'go sent the drive leg');
  const result = await waitFor(() => store.state.commands[seq], 'the drive_distance ACK');
  assertEqual(result.command, 'drive_distance');
  assertEqual(result.outcome, 'acked', `drive_distance outcome: ${JSON.stringify(result)}`);
  assertEqual(result.status, 'ok');

  // The stub sends no completion Event, so Stop ends the wait (the planner's
  // completion handling is covered by planner.html). A completion that does
  // arrive is reported and must not be a silent release.
  planner.stop();
  const stopSeq = store.state.drive.sequence;
  const stopped = await waitFor(() => store.state.commands[stopSeq], 'the Stop ACK');
  assertEqual(stopped.command, 'stop_motion');
  assertEqual(stopped.outcome, 'acked');
  await waitFor(() => store.state.drive.report?.title === 'Stopped', 'the Stopped report');
  await running;

  const completion = store.state.events.find(
    (event) => event.name === 'drive_distance_completed' && event.v?.command_seq === seq,
  );
  if (completion) {
    assert(store.state.drive.report, 'a completion Event produces a report');
  }
});

test('a bearing Target Point sends rotate_by, and Stop is Stopped only on an ACK', async () => {
  const { store } = session;
  assertEqual(planner.preview({ x: 100, y: 100 }), true, 'the Target Point is committed');
  assert(
    Math.abs(store.state.drive.plan.angle_deg - 45) < 1e-6,
    'the leg needs a 45° rotation from the identity Pose',
  );

  const running = planner.go();
  const seq = store.state.drive.sequence;
  assert(seq != null, 'go sent the rotate leg');
  const result = await waitFor(() => store.state.commands[seq], 'the rotate_by ACK');
  assertEqual(result.command, 'rotate_by');
  assertEqual(result.outcome, 'acked', `rotate_by outcome: ${JSON.stringify(result)}`);
  assertEqual(result.status, 'ok');

  // Stop is context-sensitive: a motion wait in flight sends stop_motion.
  const stopping = planner.stop();
  const stopSeq = store.state.drive.sequence;
  assert(stopSeq != null, 'Stop sent a command');
  assert(store.state.drive.report == null, 'the Stop report waits for the ACK');
  const stopped = await waitFor(() => store.state.commands[stopSeq], 'the stop_motion ACK');
  assertEqual(stopped.command, 'stop_motion');
  assertEqual(
    stopped.outcome,
    'acked',
    `stop_motion outcome: ${JSON.stringify(stopped)}`,
  );
  const report = await waitFor(() => store.state.drive.report, 'the Stop report');
  assertEqual(report.title, 'Stopped', '"Stopped" appears only because the ACK was accepted');
  await stopping;
  await running;

  const completion = store.state.events.find(
    (event) => event.name === 'rotate_completed' && event.v?.command_seq === seq,
  );
  if (completion) {
    assert(store.state.drive.report, 'a completion Event produces a report');
  }
});

test('a second connection reconstructs the snapshot, Streams and replayed Events', async () => {
  const reload = liveSession();
  try {
    const motion = await waitFor(
      () => reload.store.state.streams.motion,
      'the reload snapshot to carry the motion Stream',
    );
    assert(motion.seq > 0, 'the reconstructed Stream keeps its sequence');
    await waitFor(
      () => reload.store.state.events.some((event) => event.name === 'obstacle'),
      'the replayed obstacle Event',
    );
    await waitFor(
      () => reload.store.state.events.some((event) => event.name === 'floor_drop'),
      'the replayed floor_drop Event',
    );

    const state = reload.store.state;
    assert(
      reload.messages.some((message) => message.type === 'snapshot'),
      'the reload was served from an App Snapshot',
    );
    assert(state.eventCursor != null && state.eventCursor > 0, 'the snapshot carries the Event cursor');
    assert(state.events.length >= 2, 'the replayed Events survive the reload');
    // The stub's activity is Idle, so the real snapshot carries no terminal
    // Activity. This asserts the real value rather than inventing a report.
    assertEqual(state.lastTerminal, null, 'the stub reports no terminal Activity');
  } finally {
    reload.transport.stop();
  }
});

test('the daemon serves the app at / with the strict headers', async () => {
  const response = await fetch('/', { cache: 'no-store' });
  assertEqual(response.status, 200, 'GET / answers 200');
  assert(
    (response.headers.get('content-type') ?? '').includes('text/html'),
    'GET / is served as HTML',
  );
  const csp = response.headers.get('content-security-policy') ?? '';
  assert(csp.includes("script-src 'self'"), 'the static response carries the ADR-0039 CSP');
  assert(csp.includes("frame-ancestors 'none'"), 'the app page may not be framed');
  assertEqual(response.headers.get('cache-control'), 'no-store');
  const html = await response.text();
  assert(html.includes('./js/main.js'), 'the served page boots the app module graph');
  // ADR-0039's `frame-ancestors 'none'` blocks a same-origin iframe, so the
  // boot itself is asserted by the runner's top-level Chrome load of `/`.
});

await run();
