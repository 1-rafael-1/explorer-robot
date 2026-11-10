// Pairing and recovery at the two seams the ticket owns: the transport's socket
// factory (handshake, auth/pair, close codes) and the pairing view (field
// normalisation and the manual endpoint). No wall-clock sleeps: the fake clock
// drives the hello deadline and the reconnect backoff.

import {
  createTransport,
  TOKEN_KEY,
} from '../js/transport.js';
import { bindPairing, renderPairing } from '../js/views/pairing.js';
import { assert, assertEqual, autorun, test } from './harness.js';
import { createFakeClock } from './fake-clock.js';
import { clientHello, createMemoryStorage, createSocketFactory, hello } from './fake-socket.js';

const els = {
  form: document.getElementById('pairing-form'),
  input: document.getElementById('pairing-code'),
  hint: document.getElementById('pairing-hint'),
  manual: document.getElementById('manual-endpoint'),
  manualForm: document.getElementById('manual-form'),
  manualHost: document.getElementById('manual-host'),
  manualPort: document.getElementById('manual-port'),
  manualHint: document.getElementById('manual-hint'),
  threat: document.getElementById('threat-note'),
};

const calls = { pair: [], manual: [] };

// Bind once: re-binding would attach a second listener for every call.
bindPairing({
  pair: (code) => calls.pair.push(code),
  manual: (host, port) => calls.manual.push({ host, port }),
});

function resetFixture() {
  els.input.value = '';
  els.hint.textContent = '';
  els.manualHost.value = '';
  els.manualPort.value = '';
  els.manualHint.textContent = '';
  els.form.hidden = true;
  els.manual.hidden = true;
  els.threat.hidden = true;
  calls.pair.length = 0;
  calls.manual.length = 0;
}

function setup({ code = null } = {}) {
  const clock = createFakeClock();
  const factory = createSocketFactory();
  const storage = createMemoryStorage(code);
  const links = [];
  const messages = [];
  const transport = createTransport({
    socketFactory: factory,
    url: 'ws://test/ws',
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
  return { clock, factory, storage, transport, links, messages };
}

function start(session) {
  session.transport.start();
  assertEqual(session.factory.count(), 1, 'start opens one socket');
  const socket = session.factory.last();
  socket.open();
  return socket;
}

function frameTypes(socket) {
  return socket.sent.map((frame) => frame.type);
}

test('a gated peer with no code probes for pairing without a greeting or reconnect', () => {
  const session = setup();
  const socket = start(session);

  socket.receive(hello({ authRequired: true }));

  assertEqual(session.transport.link, 'pairing_required');
  assertEqual(socket.last('hello'), undefined, 'the client hello waits for the code');
  assertEqual(socket.last('auth'), undefined);
  assertEqual(socket.closed?.code, 1000, 'the probe closes the socket');
  assertEqual(session.factory.count(), 1);

  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 1, 'a probe-close must not reconnect');
});

test('a stored code is sent as auth before the client hello', () => {
  const session = setup({ code: 'ABCD2345EFGH' });
  const socket = start(session);

  socket.receive(hello({ authRequired: true }));

  assertEqual(frameTypes(socket), ['auth', 'hello'], 'auth is the first frame');
  assertEqual(socket.last('auth'), { type: 'auth', code: 'ABCD2345EFGH' });
  assertEqual(socket.last('hello'), clientHello());
  assertEqual(session.transport.link, 'open');
});

test('submitting a code reopens, pairs before greeting and keeps the code for the tab', () => {
  const session = setup();
  const first = start(session);
  first.receive(hello({ authRequired: true }));
  assertEqual(session.transport.link, 'pairing_required');

  session.transport.pair('abcd-efgh-jkmn');
  assertEqual(session.storage.value, 'ABCDEFGHJKMN', 'the code is kept for the tab');
  assertEqual(session.factory.count(), 2, 'submit reopens the socket');

  const second = session.factory.last();
  second.open();
  session.clock.advance(4999);
  assertEqual(session.factory.count(), 2, 'the pair handshake waits inside the 5 s deadline');

  second.receive(hello({ authRequired: true }));
  assertEqual(frameTypes(second), ['pair', 'hello'], 'pair is the first frame, then the greeting');
  assertEqual(second.last('pair'), { type: 'pair', code: 'ABCDEFGHJKMN' });
  assertEqual(second.last('hello'), clientHello());
  assertEqual(session.transport.link, 'open');

  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 2, 'the hello deadline was cleared');
});

test('a loopback pair is accepted after the greeting and re-sends connect for a manual endpoint', () => {
  const session = setup();
  const socket = start(session);
  socket.receive(hello());
  assertEqual(socket.last('hello'), clientHello(), 'loopback greets without a code');

  session.transport.manual('192.168.1.20', 9000);
  assertEqual(socket.last('connect'), { type: 'connect', host: '192.168.1.20', port: 9000 });

  const before = socket.sent.length;
  session.transport.pair('abcd-efgh-jkmn');
  assertEqual(socket.sent.slice(before).map((frame) => frame.type), ['pair', 'connect']);
  assertEqual(socket.last('pair'), { type: 'pair', code: 'ABCDEFGHJKMN' });
  assertEqual(socket.last('connect'), { type: 'connect', host: '192.168.1.20', port: 9000 });
  assertEqual(session.storage.value, 'ABCDEFGHJKMN');
  assertEqual(session.factory.count(), 1, 'a loopback pair keeps the healthy socket');
});

test('4401 forgets the code and does not reconnect', () => {
  const session = setup({ code: 'ABCD2345EFGH' });
  const socket = start(session);
  socket.receive(hello({ authRequired: true }));
  assertEqual(session.storage.value, 'ABCD2345EFGH');

  socket.peerClose(4401);

  assertEqual(session.transport.link, 'unauthorized');
  assertEqual(session.storage.value, null, 'a rejected code is forgotten');
  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 1, '4401 is not retried');
});

test('a robot unauthorized message forgets the code', () => {
  const session = setup({ code: 'ABCD2345EFGH' });
  const socket = start(session);
  socket.receive(hello({ authRequired: true }));
  assertEqual(session.storage.value, 'ABCD2345EFGH');

  socket.receive({ type: 'connection', state: 'unauthorized', detail: 'regenerate', boot_id: null, ts: 1 });

  assertEqual(session.storage.value, null);
  assertEqual(session.messages.at(-1).state, 'unauthorized', 'the state reaches the app');
});

test('a protocol mismatch closes 4426 with no partial handshake', () => {
  const session = setup();
  const socket = start(session);

  socket.receive(hello({ protocol: 2 }));

  assertEqual(session.transport.link, 'protocol_mismatch');
  assertEqual(socket.closed?.code, 4426);
  assertEqual(socket.sent.length, 0, 'no client frames after a mismatch');
  session.clock.advance(60_000);
  assertEqual(session.factory.count(), 1, 'a mismatch is not retried');
});

test('the pairing field normalises case, strips separators and hints at I, L, O and U', () => {
  resetFixture();

  els.input.value = 'abcd efgh jkmn';
  els.input.dispatchEvent(new Event('input', { bubbles: true }));
  assertEqual(els.input.value, 'ABCD-EFGH-JKMN', 'uppercase groups of four');
  assertEqual(els.hint.textContent, '', 'a valid code shows no hint');

  els.input.value = 'abcd-ilou-efgh';
  els.input.dispatchEvent(new Event('input', { bubbles: true }));
  assertEqual(els.input.value, 'ABCD-ILOU-EFGH');
  assert(/I, L, O and U/.test(els.hint.textContent), `the hint names the excluded letters: ${els.hint.textContent}`);
});

test('the token never reaches the URL and the field is cleared after submit', () => {
  resetFixture();
  const before = globalThis.location.href;

  els.input.value = 'abcd-efgh-jkmn';
  els.input.dispatchEvent(new Event('input', { bubbles: true }));
  els.form.dispatchEvent(new Event('submit', { bubbles: true, cancelable: true }));

  assertEqual(calls.pair, ['ABCDEFGHJKMN'], 'the view hands the normalised code to the transport');
  assertEqual(els.input.value, '', 'the field is cleared after submit');
  assertEqual(els.hint.textContent, '', 'the hint is cleared after submit');
  assertEqual(globalThis.location.href, before, 'submit does not navigate');
  assert(!globalThis.location.href.toUpperCase().includes('ABCDEFGHJKMN'), 'the code is not in the URL');
  assert(!globalThis.location.href.toUpperCase().includes('ABCD-EFGH-JKMN'), 'the formatted code is not in the URL');
});

test('the pairing screen shows only where pairing is needed and defaults the manual port to 9000', () => {
  resetFixture();

  renderPairing({ link: 'open', connection: { state: 'no_robots' } });
  assertEqual(els.manual.hidden, false, 'no robots offers the manual endpoint');
  assertEqual(els.manualPort.value, '9000', 'the manual port defaults to 9000');
  assertEqual(els.form.hidden, true, 'no robots hides the pairing field');

  renderPairing({ link: 'open', connection: { state: 'pairing_required' } });
  assertEqual(els.form.hidden, false, 'pairing required shows the field');
  assertEqual(els.threat.hidden, false, 'pairing required shows the threat note');
  assertEqual(els.manual.hidden, true, 'pairing required hides the manual endpoint');

  renderPairing({ link: 'unauthorized', connection: { state: 'connecting' } });
  assertEqual(els.form.hidden, false, 'an unauthorized code offers another try');
  assertEqual(els.threat.hidden, false);

  for (const state of ['searching', 'connecting', 'connected', 'busy', 'setup_mode', 'protocol_mismatch', 'stopped']) {
    renderPairing({ link: 'open', connection: { state } });
    assertEqual(els.form.hidden, true, `${state} hides the pairing field`);
    assertEqual(els.manual.hidden, true, `${state} hides the manual endpoint`);
    assertEqual(els.threat.hidden, true, `${state} hides the threat note`);
  }

  renderPairing({ link: 'reconnecting', connection: { state: 'connected' } });
  assertEqual(els.form.hidden, true, 'a lost socket keeps the pairing field hidden');
  assertEqual(els.manual.hidden, true);

  renderPairing({ link: 'open', connection: { state: 'pairing_required' } });
  els.input.value = 'ABCD-EFGH-JKMN';
  renderPairing({ link: 'unauthorized', connection: { state: 'connecting' } });
  assertEqual(els.input.value, '', 'an unauthorized state clears a code left on screen');
});

test('the manual form sends the host and the default port', () => {
  resetFixture();
  renderPairing({ link: 'open', connection: { state: 'no_robots' } });

  els.manualHost.value = '192.168.1.20';
  els.manualForm.dispatchEvent(new Event('submit', { bubbles: true, cancelable: true }));
  assertEqual(calls.manual, [{ host: '192.168.1.20', port: 9000 }]);

  resetFixture();
  els.manualHost.value = '';
  els.manualForm.dispatchEvent(new Event('submit', { bubbles: true, cancelable: true }));
  assertEqual(calls.manual, [], 'a missing host draws a hint instead of connecting');
  assert(/host/i.test(els.manualHint.textContent), `the manual hint explains the host: ${els.manualHint.textContent}`);
});

// The token key is stable so a reload in the same tab finds the stored code.
test('the token key is the tab store’s stable name', () => {
  assertEqual(TOKEN_KEY, 'explorer-remote.pairing-token');
});

autorun();
