// The drive bar: the numeric bearing/distance form, the Target Point preview,
// the context-sensitive Stop and the outcome report (ADR-0043).
//
// The planner owns the legs and their timing; this module only renders its state
// and forwards the operator's intents.

import { activityBusy, controlEnabled } from '../store.js';
import { byId, setClass, setData, setHidden, setText } from './dom.js';

/** Wires the drive controls. */
export function bindDrive({
  stop = () => {},
  previewNumeric = () => {},
  go = () => {},
  cancel = () => {},
  dismiss = () => {},
} = {}) {
  const stopButton = byId('stop-button');
  if (stopButton) stopButton.addEventListener('click', () => stop());

  const form = byId('numeric-drive-form');
  if (form) {
    form.addEventListener('submit', (event) => {
      event.preventDefault();
      const bearing = Number.parseFloat(byId('numeric-bearing')?.value ?? '');
      const distance = Number.parseFloat(byId('numeric-distance')?.value ?? '');
      if (!Number.isFinite(bearing) || !Number.isFinite(distance)) {
        setText(byId('numeric-hint'), 'Enter a bearing in degrees and a distance in centimetres.');
        return;
      }
      setText(byId('numeric-hint'), '');
      previewNumeric({ bearing_deg: bearing, distance_cm: distance });
    });
  }

  byId('target-go')?.addEventListener('click', () => go());
  byId('target-cancel')?.addEventListener('click', () => cancel());
  byId('drive-dismiss')?.addEventListener('click', () => dismiss());
}

/** Renders the preview, the waiting state and the last outcome. */
export function renderDrive(state) {
  const enabled = controlEnabled(state);
  const plan = state.drive?.plan;
  const report = state.drive?.report;
  const stage = state.drive?.stage ?? 'idle';

  const stop = byId('stop-button');
  if (stop) {
    stop.disabled = !enabled && state.connection.state !== 'connected';
    setText(stop, stage === 'rotate' || stage === 'drive' ? 'Stop' : 'Stop');
  }

  const waitingStage = stage === 'rotate' || stage === 'drive';
  const card = byId('target-card');
  setHidden(card, !plan);
  if (plan) {
    setText(byId('target-bearing'), `${Number(plan.bearing_deg).toFixed(1)}°`);
    setText(byId('target-distance'), `${Number(plan.distance_cm).toFixed(0)} cm`);
    setText(byId('target-turn'), `${Number(plan.angle_deg).toFixed(1)}°`);
    setData(card, 'stage', stage);
    const go = byId('target-go');
    if (go) {
      go.disabled = !enabled || activityBusy(state) || waitingStage;
      setText(go, waitingStage ? 'Waiting…' : 'Go');
    }
    setHidden(byId('target-cancel'), stage !== 'idle');
    setText(
      byId('target-status'),
      stage === 'rotate'
        ? 'Waiting for the robot to rotate…'
        : stage === 'drive'
          ? 'Waiting for the robot to drive…'
          : '',
    );
  }

  setHidden(byId('drive-report'), !report);
  if (report) {
    setText(byId('drive-report-title'), report.title ?? '');
    setText(byId('drive-report-text'), report.text ?? '');
    setData(byId('drive-report'), 'kind', report.kind ?? 'info');
  }
  setData(byId('drive-bar'), 'stage', stage);
  setClass(byId('drive-bar'), 'is-waiting', stage === 'rotate' || stage === 'drive');
}
