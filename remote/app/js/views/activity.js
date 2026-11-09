// The one Activity truth: a live card, a Result Report, a Stopped card or Idle.
// Labels come from the Catalogue except Idle, Booting and Manual Drive (ADR-0038).

import { activityLabel, activityView } from '../store.js';
import { byId, setData, setHidden, setText } from './dom.js';

/** Wires the Result Report dismiss action. */
export function bindActivity({ dismiss = () => {} } = {}) {
  const button = byId('activity-dismiss');
  if (button) button.addEventListener('click', () => dismiss());
}

/** Renders the Activity card for `state`. */
export function renderActivity(state) {
  const view = activityView(state);
  const label = view.kind === 'idle' ? 'Idle' : activityLabel(state, view.data.id);
  const panel = byId('activity-panel');

  setData(panel, 'kind', view.kind);
  setText(byId('activity-label'), label);
  setText(byId('activity-stage'), stageText(view));
  setText(byId('activity-detail'), view.data.detail || '');

  const percent = view.data.percent;
  const progress = byId('activity-progress');
  setHidden(progress, percent == null);
  if (progress && percent != null) {
    progress.value = percent;
    progress.textContent = `${percent}%`;
  }

  setHidden(byId('activity-dismiss'), view.kind !== 'result');
  setHidden(byId('activity-panel'), false);
}

function stageText(view) {
  if (view.kind === 'live') return view.data.percent == null ? 'Running' : `Running · ${view.data.percent}%`;
  if (view.kind === 'stopped') return 'Stopped';
  if (view.kind === 'result') {
    if (view.data.stage === 'failed') return 'Failed';
    if (view.data.stage === 'complete') return 'Complete';
    return 'Finished';
  }
  return '';
}
