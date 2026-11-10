// Bridge and link status: a fixed-width header chip and the Robot/Link panel.
// The app shows status only; it never duplicates the panel's Link screen or
// rotates the Pairing Token (ADR-0042, ADR-0043).

import { relayNow } from '../store.js';
import { byId, formatAge, orUnknown, setData, setText } from './dom.js';

/** Returns a stable key for the station state; anything else is `unknown`. */
export function stationKey(station) {
  if (station === 'associated' || station === 'connecting' || station === 'down') return station;
  return 'unknown';
}

/** Returns a short station label. */
export function stationLabel(station) {
  const key = stationKey(station);
  if (key === 'associated') return 'Associated';
  if (key === 'connecting') return 'Connecting';
  if (key === 'down') return 'Down';
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
export function renderBridge(state, now = relayNow(state)) {
  const bridge = state.bridge;
  const station = stationKey(bridge?.station);
  const chip = byId('bridge-chip');
  setData(chip, 'station', station);
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
  setData(byId('bridge-panel'), 'station', station);
}

function yesNo(value) {
  if (value == null) return '—';
  return value ? 'Yes' : 'No';
}
