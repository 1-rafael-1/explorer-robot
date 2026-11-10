// The one Activity truth: a live card, a Result Report, a Stopped card or Idle.
// Labels come from the Catalogue except Idle, Booting and Manual Drive (ADR-0038).

import { labelForActivity } from '../protocol.js';
import { activityView, relayNow } from '../store.js';
import { byId, formatAge, setClass, setData, setHidden, setText } from './dom.js';
import { STREAM_STALE_MS } from './streams.js';

/** Wires the Result Report dismiss action. */
export function bindActivity({ dismiss = () => {} } = {}) {
  const button = byId('activity-dismiss');
  if (button) button.addEventListener('click', () => dismiss());
}

/** Renders the Activity card for `state`. */
export function renderActivity(state, now = relayNow(state)) {
  const view = activityView(state);
  const label = view.kind === 'idle' ? 'Idle' : labelForActivity(view.data.id, state.catalogue);
  const panel = byId('activity-panel');

  setData(panel, 'kind', view.kind);
  setData(panel, 'tone', view.kind === 'result' ? view.data.stage : '');
  setData(panel, 'busy', String(Boolean(view.busy)));
  setText(byId('activity-label'), label);
  setText(byId('activity-stage'), stageText(view));
  setText(byId('activity-detail'), detailText(view));
  setHidden(byId('activity-busy'), !view.busy);
  renderAge(byId('activity-age'), view.ts, now);

  const percent = view.data.percent;
  const progress = byId('activity-progress');
  const showProgress = view.kind === 'live' || percent != null;
  setHidden(progress, !showProgress);
  if (progress && showProgress) {
    if (percent == null) {
      // A live Activity with no reported percentage is indeterminate, not zero.
      progress.removeAttribute('value');
      setText(progress, '');
    } else {
      progress.value = percent;
      progress.textContent = `${percent}%`;
    }
  }

  setHidden(byId('activity-dismiss'), view.kind !== 'result');
  setHidden(byId('activity-panel'), false);
}

function stageText(view) {
  if (view.kind === 'live') {
    return view.data.percent == null ? 'Running' : `Running · ${view.data.percent}%`;
  }
  if (view.kind === 'stopped') return 'Stopped';
  if (view.kind === 'result') {
    if (view.data.stage === 'failed') return 'Failed';
    if (view.data.stage === 'complete') return 'Complete';
    return 'Finished';
  }
  return '';
}

function detailText(view) {
  const detail = view.data.detail || '';
  if (view.kind === 'live' && view.busy) {
    return detail ? `${detail} · Another client holds the robot.` : 'Another client holds the robot.';
  }
  return detail;
}

function renderAge(node, ts, now) {
  if (!node) return;
  if (ts == null) {
    setText(node, '');
    setClass(node, 'is-stale', false);
    setData(node, 'stale', 'false');
    return;
  }
  const stale = now - ts > STREAM_STALE_MS;
  setText(node, stale ? `${formatAge(ts, now)} · stale` : formatAge(ts, now));
  setClass(node, 'is-stale', stale);
  setData(node, 'stale', String(stale));
}
