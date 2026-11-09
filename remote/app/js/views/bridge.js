// Bridge and link status: a fixed-width header chip and the Robot/Link panel.
// The app shows status only; it never duplicates the panel's Link screen or
// rotates the Pairing Token (ADR-0042, ADR-0043).

import { byId, formatAge, orUnknown, setData, setText } from './dom.js';

/** Returns a short station label. */
export function stationLabel(station) {
  if (station === 'associated') return 'Associated';
  if (station === 'connecting') return 'Connecting';
  if (station === 'down') return 'Down';
  return 'Unknown';
}

/** Returns a shape-and-word RSSI bucket, so hue is never the only signal. */
export function rssiBucket(rssi) {
  if (rssi == null) return '—';
  if (rssi >= -55) return 'Strong';
  if (rssi >= -67) return 'Good';
  if (rssi >= -75) return 'Fair';
  return 'Weak';
}

/** Renders the header chip and the Robot/Link panel. */
export function renderBridge(state, now = Date.now()) {
  const bridge = state.bridge;
  const station = bridge?.station ?? null;
  const chip = byId('bridge-chip');
  setData(chip, 'station', station ?? 'unknown');
  setData(chip, 'rssi', bridge?.rssi_dbm == null ? 'unknown' : rssiBucket(bridge.rssi_dbm).toLowerCase());
  setText(byId('bridge-chip-station'), stationLabel(station));
  setText(byId('bridge-chip-rssi'), rssiBucket(bridge?.rssi_dbm));

  setText(byId('bridge-rssi'), bridge?.rssi_dbm == null ? '—' : `${bridge.rssi_dbm} dBm`);
  setText(byId('bridge-rssi-bucket'), rssiBucket(bridge?.rssi_dbm));
  setText(byId('bridge-channel'), orUnknown(bridge?.channel));
  setText(byId('bridge-provisioned'), yesNo(bridge?.provisioned));
  setText(byId('bridge-setup'), yesNo(bridge?.setup_open));
  setText(byId('bridge-link-ready'), yesNo(bridge?.link_ready));
  setText(byId('bridge-uart'), orUnknown(bridge?.uart_dropped_bytes));
  setText(byId('bridge-dma'), orUnknown(bridge?.dma_stalls));
  setText(byId('bridge-crc'), orUnknown(bridge?.crc_failures));
  setText(byId('bridge-resyncs'), orUnknown(bridge?.resyncs));
  setText(byId('bridge-wdt-feeds'), orUnknown(bridge?.wdt_feeds));
  setText(byId('bridge-wdt-withheld'), orUnknown(bridge?.wdt_withheld));
  setText(byId('bridge-age'), bridge ? formatAge(bridge.ts, now) : 'not reported');
  setData(byId('bridge-panel'), 'station', station ?? 'unknown');
}

function yesNo(value) {
  if (value == null) return '—';
  return value ? 'Yes' : 'No';
}
