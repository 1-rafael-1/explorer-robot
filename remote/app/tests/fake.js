import { createTokenStorage, createTransport, defaultWsUrl } from '../js/transport.js';
import { assert, assertEqual, autorun, test } from './harness.js';
import { createFakeClock } from './fake-clock.js';
import { clientHello, createMemoryStorage, createSocketFactory, hello } from './fake-socket.js';

function setup({ authRequired = false, code = null, url = 'ws://test/ws' } = {}) {
  const clock = createFakeClock();
  const factory = createSocketFactory();
  const storage = createMemoryStorage(code);
  const links = [];
  const messages = [];
  const transport = createTransport({
    socketFactory: factory,
    url,
    storage,
    heartbeatMs: 1000,
    reconnectBaseMs: 500,
    reconnectMaxMs: 8000,
    random: () => 0.5,
    onLink: (link, detail) => links.push({ link, detail }),
    onMessage: (message) => messages.push(message),
    setTimeoutFn: clock.setTimeout,
    clearTimeoutFn: clock.clearTimeout,
    setIntervalFn: clock.setInterval,
    clearIntervalFn: clock.clearInterval,
  });
  return { clock, factory, storage, transport, links, messages, authRequired };
}

function start(session) {
  session.transport.start();
  assertEqual(session.factory.count(), 1, 'start opens one socket');
  const socket = session.factory.last();
  socket.open();
  return socket;
}

test('the socket is derived from the page origin', () => {
  assert(/^wss?:\/\//.test(defaultWsUrl()));
  assert(defaultWsUrl().endsWith('/ws'));
});

test('loopback connects, greets, sends its hello and a 1 Hz heartbeat', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello());
  assertEqual(socket.last('hello'), clientHello());
  assertEqual(session.transport.link, 'open');

  session.clock.advance(1000);
  assertEqual(socket.frames('heartbeat').length, 1);
  session.clock.advance(1000);
  assertEqual(socket.frames('heartbeat').length, 2);

  socket.receive({ type: 'connection', state: 'connected', robot: 'Explorer', detail: null, boot_id: 7, ts: 100 });
  socket.receive({
    type: 'snapshot',
    protocol: 1,
    boot_id: 7,
    ts: 101,
    streams: [{ name: 'power', seq: 1, ts: 101, v: { level: 80, voltage_mv: 12_000 } }],
    last_terminal: null,
    events: [],
    event_cursor: 0,
    catalogue: null,
    bridge: null,
  });
  assertEqual(session.messages.filter((message) => message.type === 'connection').length, 1);
  assertEqual(session.messages.filter((message) => message.type === 'snapshot').length, 1);
});

test('an unexpected close reconnects with backoff', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello());
  socket.peerClose(1006);
  assertEqual(session.transport.link, 'reconnecting');
  session.clock.advance(499);
  assertEqual(session.factory.count(), 1, 'the reconnect waits for the backoff');
  session.clock.advance(1);
  assertEqual(session.factory.count(), 2, 'the backoff elapses into a new socket');
});

test('a protocol mismatch is visible and does not reconnect', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello({ protocol: 2 }));
  assertEqual(session.transport.link, 'protocol_mismatch');
  assertEqual(socket.closed?.code, 4426);
  assertEqual(socket.last('hello'), undefined, 'no partial handshake after a mismatch');
  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 1);
});

test('a gated peer with no stored code probes and shows the pairing screen', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello({ authRequired: true }));
  assertEqual(session.transport.link, 'pairing_required');
  assertEqual(socket.last('hello'), undefined, 'the client hello waits for the code');
  assertEqual(socket.last('auth'), undefined);
  assertEqual(socket.closed?.code, 1000);
  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 1, 'the probe-close must not reconnect');
});

test('a stored code is sent as auth before the client hello', () => {
  const session = setup({ code: 'ABCD2345EFGH' });
  const socket = start(session);
  socket.receive(hello({ authRequired: true }));
  assertEqual(socket.last('auth'), { type: 'auth', code: 'ABCD2345EFGH' });
  assertEqual(socket.last('hello'), clientHello());
  assertEqual(session.transport.link, 'open');
});

test('submitting a code pairs, stores it and then greets', () => {
  const session = setup();
  const first = start(session);
  first.receive(hello({ authRequired: true }));
  assertEqual(session.transport.link, 'pairing_required');

  session.transport.pair('abcd-efgh-jkmn');
  assertEqual(session.storage.value, 'ABCDEFGHJKMN', 'the code is kept for the tab');
  assertEqual(session.factory.count(), 2);
  const second = session.factory.last();
  second.open();
  second.receive(hello({ authRequired: true }));
  assertEqual(second.last('pair'), { type: 'pair', code: 'ABCDEFGHJKMN' });
  assertEqual(second.last('hello'), clientHello());
});

test('4401 clears the stored code and shows unauthorized', () => {
  const session = setup({ code: 'ABCD2345EFGH' });
  const socket = start(session);
  socket.receive(hello({ authRequired: true }));
  socket.peerClose(4401);
  assertEqual(session.transport.link, 'unauthorized');
  assertEqual(session.storage.value, null, 'a rejected code is forgotten');
  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 1, '4401 is not retried');
});

test('a robot unauthorized message clears the stored code', () => {
  const session = setup({ code: 'ABCD2345EFGH' });
  const socket = start(session);
  socket.receive(hello({ authRequired: true }));
  socket.receive({ type: 'connection', state: 'unauthorized', detail: 'regenerate', boot_id: null, ts: 1 });
  assertEqual(session.storage.value, null);
});

test('a manual endpoint sends connect after the handshake', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello());
  session.transport.manual('192.168.1.20', 9000);
  assertEqual(socket.last('connect'), { type: 'connect', host: '192.168.1.20', port: 9000 });
});

test('1001 is a visible state and then a reconnect', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello());
  socket.peerClose(1001);
  assertEqual(session.transport.link, 'stopped');
  session.clock.advance(500);
  assertEqual(session.factory.count(), 2);
});

test('token storage falls back to memory when sessionStorage throws', () => {
  const refusing = {
    getItem() {
      throw new Error('denied');
    },
    setItem() {
      throw new Error('denied');
    },
    removeItem() {
      throw new Error('denied');
    },
  };
  const storage = createTokenStorage(refusing);
  storage.write('ABCD2345EFGH');
  assertEqual(storage.read(), 'ABCD2345EFGH');
  storage.clear();
  assertEqual(storage.read(), null);
});

autorun();
