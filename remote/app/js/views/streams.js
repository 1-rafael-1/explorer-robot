// Motion, perception and the stream data ages. A missing value stays unknown;
// zero and false are values, never "unknown" (ADR-0038). Ages come from each
// sample's `ts` and are display only: they never gate an action.

import { motionView, perceptionView, powerView, relayNow } from '../store.js';
import { byId, formatAge, setClass, setData, setHidden, setText } from './dom.js';

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
  renderObstacle(byId('perception-obstacle'), perception.obstacle);
  const aggregate = byId('perception-floor-drop');
  const front = byId('perception-floor-drop-front');
  const rear = byId('perception-floor-drop-rear');
  // A current robot sends the split; an older one sends only the aggregate.
  const splitKnown = perception.floor_drop_front != null || perception.floor_drop_rear != null;
  const showAggregate = !splitKnown && perception.floor_drop != null;
  setHidden(aggregate, !showAggregate);
  setHidden(front, showAggregate);
  setHidden(rear, showAggregate);
  if (showAggregate) {
    renderAggregate(aggregate, perception.floor_drop);
  } else {
    renderFloor(front, perception.floor_drop_front, 'Front');
    renderFloor(rear, perception.floor_drop_rear, 'Rear');
  }
  renderAge(
    byId('perception-age'),
    perception.ts,
    perception.obstacle == null &&
      perception.floor_drop == null &&
      perception.floor_drop_front == null &&
      perception.floor_drop_rear == null,
    now,
  );

  const power = powerView(state);
  renderAge(byId('power-age'), power.ts, power.level == null && power.voltage_mv == null, now);
}

// The perception labels are words, so a colour-vision deficiency never hides the
// state. Every value carries its own name, so a clear flag cannot render as a
// bare "Clear" that reads like a second sensor.
function renderObstacle(node, value) {
  renderState(
    node,
    value == null ? 'Obstacle: unknown' : value ? 'Obstacle: detected' : 'Obstacle: clear',
    value,
  );
}

// A rangefinder reads Floor when the floor is present, Drop when the sensor sees
// past the threshold, and unknown before the first sample.
function renderFloor(node, value, name) {
  renderState(node, value == null ? `${name}: unknown` : value ? `${name}: drop` : `${name}: floor`, value);
}

// An older robot sends only the aggregate, so the app falls back to one line.
function renderAggregate(node, value) {
  renderState(
    node,
    value == null ? 'Floor drop: unknown' : value ? 'Floor drop: detected' : 'Floor drop: clear',
    value,
  );
}

function renderState(node, text, value) {
  if (!node) return;
  setText(node, text);
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
