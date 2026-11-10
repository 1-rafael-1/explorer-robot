// Calibration status, on demand. A missing status is shown as unknown, never as
// loaded (ADR-0038's unknown-is-null rule).

import { byId, setText } from './dom.js';

const KEYS = ['motor', 'imu', 'mag', 'distance'];

/** Renders the four calibration statuses. */
export function renderCalibration(state) {
  const value = state.streams.calibration?.v;
  for (const key of KEYS) {
    const status = value?.[key] ?? 'unknown';
    setText(byId(`calibration-${key}`), statusLabel(status));
  }
}

function statusLabel(status) {
  if (status === 'loaded') return 'Loaded';
  if (status === 'missing') return 'Missing';
  return 'Unknown';
}
