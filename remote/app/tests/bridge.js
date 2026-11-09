import { apply, initialState } from '../js/store.js';
import { renderBridge, rssiBucket, stationLabel } from '../js/views/bridge.js';
import { assert, assertEqual, autorun, test } from './harness.js';

const NOW = 1_700_000_000_000;

function bridgeMessage(overrides = {}) {
  return {
    type: 'bridge',
    ts: NOW - 1500,
    rssi_dbm: -61,
    channel: 6,
    station: 'associated',
    provisioned: true,
    setup_open: false,
    link_ready: true,
    uart_dropped_bytes: 12,
    dma_stalls: 1,
    crc_failures: 2,
    resyncs: 3,
    wdt_feeds: 40,
    wdt_withheld: 4,
    ...overrides,
  };
}

function snapshot(overrides = {}) {
  return {
    type: 'snapshot',
    protocol: 1,
    boot_id: null,
    ts: NOW,
    streams: [],
    last_terminal: null,
    events: [],
    event_cursor: null,
    catalogue: null,
    bridge: null,
    ...overrides,
  };
}

function text(id) {
  return document.getElementById(id)?.textContent ?? null;
}

function dataAttr(id, name) {
  const node = document.getElementById(id);
  return node ? node.getAttribute(`data-${name}`) : null;
}

test('apply stores the bridge message and snapshot.bridge', () => {
  const message = bridgeMessage();
  let state = apply(initialState(), message);
  assertEqual(state.bridge, message);

  const fromSnapshot = bridgeMessage({ station: 'connecting', rssi_dbm: null, channel: null });
  state = apply(state, snapshot({ bridge: fromSnapshot }));
  assertEqual(state.bridge, fromSnapshot);
});

test('a robot Boot Id change clears robot state but keeps the bridge status', () => {
  let state = apply(initialState(), snapshot({ boot_id: 1, bridge: bridgeMessage() }));
  state = apply(state, {
    type: 'snapshot',
    protocol: 1,
    boot_id: 1,
    ts: NOW + 10,
    streams: [{ name: 'pose', seq: 1, ts: NOW + 10, v: { x_cm: 10, y_cm: 20, yaw_deg: 0 } }],
    last_terminal: null,
    events: [],
    event_cursor: 0,
    catalogue: { procedures: [{ id: 5, label: 'Coast and Avoid' }] },
    bridge: bridgeMessage(),
  });
  const bridge = state.bridge;

  state = apply(state, snapshot({ boot_id: 2, streams: [], catalogue: null, bridge: null }));
  assertEqual(state.bootId, 2);
  assertEqual(state.streams, {}, 'the Boot Id change clears the robot streams');
  assertEqual(state.catalogue, [], 'the Boot Id change clears the Catalogue');
  assertEqual(state.bridge, bridge, 'the Bridge Status survives the Boot Id change');

  state = apply(state, { type: 'connection', state: 'connected', boot_id: 3, ts: NOW + 20 });
  assertEqual(state.bootId, 3);
  assertEqual(state.bridge, bridge, 'a connection Boot Id change also keeps the Bridge Status');
});

test('station and RSSI labels include the null and boundary values', () => {
  assertEqual(stationLabel('associated'), 'Associated');
  assertEqual(stationLabel('connecting'), 'Connecting');
  assertEqual(stationLabel('down'), 'Down');
  assertEqual(stationLabel(null), 'Unknown');
  assertEqual(stationLabel(undefined), 'Unknown');

  assertEqual(rssiBucket(null), '—');
  assertEqual(rssiBucket(undefined), '—');
  assertEqual(rssiBucket(-40), 'Strong');
  assertEqual(rssiBucket(-55), 'Strong');
  assertEqual(rssiBucket(-56), 'Good');
  assertEqual(rssiBucket(-67), 'Good');
  assertEqual(rssiBucket(-68), 'Fair');
  assertEqual(rssiBucket(-75), 'Fair');
  assertEqual(rssiBucket(-76), 'Weak');
  assertEqual(rssiBucket(-95), 'Weak');
});

test('renderBridge writes the station, RSSI and every counter', () => {
  renderBridge({ bridge: bridgeMessage() }, NOW);

  assertEqual(text('bridge-chip-station'), 'Associated');
  assertEqual(text('bridge-chip-rssi'), 'Good');
  assertEqual(text('bridge-rssi'), '-61 dBm');
  assertEqual(text('bridge-rssi-bucket'), 'Good');
  assertEqual(text('bridge-channel'), '6');
  assertEqual(text('bridge-provisioned'), 'Yes');
  assertEqual(text('bridge-setup'), 'No');
  assertEqual(text('bridge-link-ready'), 'Yes');
  assertEqual(text('bridge-uart'), '12');
  assertEqual(text('bridge-dma'), '1');
  assertEqual(text('bridge-crc'), '2');
  assertEqual(text('bridge-resyncs'), '3');
  assertEqual(text('bridge-wdt-feeds'), '40');
  assertEqual(text('bridge-wdt-withheld'), '4');
  assertEqual(text('bridge-age'), '1.5 s old');
});

test('renderBridge leaves "—" in every missing value slot', () => {
  renderBridge({ bridge: { ts: null, station: 'down' } }, NOW);

  assertEqual(text('bridge-chip-station'), 'Down');
  assertEqual(text('bridge-chip-rssi'), '—');
  assertEqual(text('bridge-rssi'), '—');
  assertEqual(text('bridge-rssi-bucket'), '—');
  assertEqual(text('bridge-channel'), '—');
  assertEqual(text('bridge-provisioned'), '—');
  assertEqual(text('bridge-setup'), '—');
  assertEqual(text('bridge-link-ready'), '—');
  for (const id of [
    'bridge-uart',
    'bridge-dma',
    'bridge-crc',
    'bridge-resyncs',
    'bridge-wdt-feeds',
    'bridge-wdt-withheld',
  ]) {
    assertEqual(text(id), '—', `${id} keeps a placeholder`);
  }
  assertEqual(text('bridge-age'), 'age unknown');

  renderBridge({ bridge: null }, NOW);
  assertEqual(text('bridge-chip-station'), 'Unknown');
  assertEqual(text('bridge-chip-rssi'), '—');
  assertEqual(text('bridge-rssi'), '—');
  assertEqual(text('bridge-age'), 'not reported');
});

test('the chip keeps a stable station and RSSI data attribute', () => {
  renderBridge({ bridge: bridgeMessage({ station: 'associated', rssi_dbm: -70 }) }, NOW);
  assertEqual(dataAttr('bridge-chip', 'station'), 'associated');
  assertEqual(dataAttr('bridge-chip', 'rssi'), 'fair');
  assertEqual(dataAttr('bridge-panel', 'station'), 'associated');

  renderBridge({ bridge: { ts: NOW, station: 'connecting' } }, NOW);
  assertEqual(dataAttr('bridge-chip', 'station'), 'connecting');
  assertEqual(dataAttr('bridge-chip', 'rssi'), 'unknown');

  renderBridge({ bridge: null }, NOW);
  assertEqual(dataAttr('bridge-chip', 'station'), 'unknown');
  assertEqual(dataAttr('bridge-chip', 'rssi'), 'unknown');
  assert(dataAttr('bridge-chip', 'station') !== null, 'the station attribute stays set');
  assert(dataAttr('bridge-chip', 'rssi') !== null, 'the RSSI attribute stays set');
});

test('the bridge view is status only and setup mode stays a connection state', () => {
  renderBridge({ bridge: bridgeMessage({ setup_open: true }) }, NOW);
  assertEqual(text('bridge-setup'), 'Yes');
  assertEqual(text('bridge-chip-station'), 'Associated', 'setup-open does not replace the station');
  assertEqual(dataAttr('bridge-chip', 'station'), 'associated');

  renderBridge({ bridge: bridgeMessage({ station: 'setup_mode' }) }, NOW);
  assertEqual(text('bridge-chip-station'), 'Unknown', 'setup mode is not a bridge station');
  assertEqual(dataAttr('bridge-chip', 'station'), 'unknown');

  const panel = document.getElementById('bridge-panel');
  assertEqual(panel.querySelectorAll('button, input, select, textarea').length, 0,
    'the panel shows status only: no Link screen or token action');
});

autorun();
