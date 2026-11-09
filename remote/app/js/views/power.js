// Battery and pack voltage. An absent value keeps its slot and reads unknown,
// never zero (ADR-0038's unknown-is-null rule).

import { powerView } from '../store.js';
import { byId, setClass, setData, setText } from './dom.js';

/** Renders the header battery and voltage. */
export function renderPower(state) {
  const power = powerView(state);
  const level = byId('battery-level');
  setText(level, power.level == null ? '—' : `${power.level}%`);
  setClass(level, 'is-unknown', power.level == null);
  setData(level, 'unknown', String(power.level == null));
  setData(level, 'tone', power.level == null ? 'unknown' : power.level <= 20 ? 'bad' : power.level <= 40 ? 'warn' : 'good');

  const voltage = byId('battery-voltage');
  setText(voltage, power.voltage_mv == null ? '—' : `${(power.voltage_mv / 1000).toFixed(2)} V`);
  setClass(voltage, 'is-unknown', power.voltage_mv == null);
}
