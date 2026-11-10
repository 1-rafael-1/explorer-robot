// Motion, perception and the stream data ages. A missing value stays unknown;
// zero and false are values, never "unknown" (ADR-0038). Ages come from each
// sample's `ts` and are display only: they never gate an action.

import { motionView, perceptionView, powerView, relayNow } from '../store.js';
import { byId, formatAge, setClass, setData, setText } from './dom.js';

/** The age after which live stream data is shown stale. */
export const STREAM_STALE_MS = 1000;

/** Renders the motion slots, the perception row and the data ages. */
export function renderStreams(state, now = relayNow(state)) {
  const motion = motionView(state);
  setText(byId('motion-left'), motion.left == null ? '—' : String(motion.left));
  setText(byId('motion-right'), motion.right == null ? '—' : String(motion.right));
  setData(byId('motion-left'), 'unknown', String(motion.left == null));
  setData(byId('motion-right'), 'unknown', String(motion.right == null));
  renderAge(byId('motion-age'), motion.ts, motion.left == null && motion.right == null, now);

  const perception = perceptionView(state);
  renderFlag(byId('perception-obstacle'), perception.obstacle, 'Obstacle');
  renderFlag(byId('perception-floor-drop'), perception.floor_drop, 'Floor drop');
  renderAge(
    byId('perception-age'),
    perception.ts,
    perception.obstacle == null && perception.floor_drop == null,
    now,
  );

  const power = powerView(state);
  renderAge(byId('power-age'), power.ts, power.level == null && power.voltage_mv == null, now);
}

// The perception labels are words, so a colour-vision deficiency never hides the
// state; `false` reads Clear and `null` reads unknown.
function renderFlag(node, value, name) {
  if (!node) return;
  setText(node, value == null ? `${name} unknown` : value ? name : 'Clear');
  setData(node, 'detected', value == null ? 'unknown' : String(value));
  setClass(node, 'is-unknown', value == null);
}

function renderAge(node, ts, absent, now) {
  if (!node) return;
  if (absent || ts == null) {
    setText(node, 'no data');
    setData(node, 'stale', 'true');
    setClass(node, 'is-stale', true);
    return;
  }
  const age = Math.max(0, now - ts);
  const stale = age > STREAM_STALE_MS;
  setText(node, stale ? `${formatAge(ts, now)} · stale` : formatAge(ts, now));
  setData(node, 'stale', String(stale));
  setClass(node, 'is-stale', stale);
}
