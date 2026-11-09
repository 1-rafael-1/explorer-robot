// The view aggregator: binds the shell and forwards store changes to the panel
// renderers. It owns only ephemeral view state (the sheet tab and which on-demand
// panel is open); the domain state stays in `store.js`.

import { createRadar, bindRadar } from '../radar.js';
import { renderActivity, bindActivity } from './activity.js';
import { renderBridge } from './bridge.js';
import { renderCalibration } from './calibration.js';
import { renderDrive, bindDrive } from './drive.js';
import { renderEvents, bindEvents } from './events.js';
import { renderPairing, bindPairing } from './pairing.js';
import { renderPower } from './power.js';
import { bindShell, renderShell } from './shell.js';
import { byId, setHidden, setText } from './dom.js';

let toastTimer = null;

/** Shows a short status message, e.g. a drive outcome that has no panel. */
export function announce(text, tone = 'info') {
  const toast = byId('toast');
  if (!toast) return;
  setText(toast, text);
  toast.dataset.tone = tone;
  toast.hidden = false;
  if (toastTimer != null) globalThis.clearTimeout(toastTimer);
  toastTimer = globalThis.setTimeout(() => {
    toast.hidden = true;
    toastTimer = null;
  }, 4000);
}

/** Mounts the app: binds the controls and renders on every store change. */
export function mountApp({ store, transport, planner, radar = createRadar(byId('radar'), {}) }) {
  const ui = { tab: 'activity', eventsOpen: false, robotOpen: false, calibrationOpen: false };

  function renderPanels() {
    setHidden(byId('events-panel'), !ui.eventsOpen);
    setHidden(byId('bridge-panel'), !ui.robotOpen);
    setHidden(byId('calibration-panel'), !ui.calibrationOpen);
    byId('events-toggle')?.setAttribute('aria-expanded', String(ui.eventsOpen));
    byId('robot-toggle')?.setAttribute('aria-expanded', String(ui.robotOpen));
    byId('calibration-toggle')?.setAttribute('aria-expanded', String(ui.calibrationOpen));
  }

  function render() {
    const state = store.state;
    renderShell(state, ui);
    renderPower(state);
    renderPairing(state);
    renderActivity(state);
    renderEvents(state, ui);
    renderBridge(state);
    renderCalibration(state);
    renderDrive(state);
    renderPanels();
    radar.invalidate();
  }

  bindShell({
    retry: () => transport.reconnectNow(),
    onTab: (tab) => {
      ui.tab = tab;
      if (tab === 'events') ui.eventsOpen = true;
      if (tab === 'robot') ui.robotOpen = true;
      if (tab === 'events') store.dispatch({ type: '@seen' });
      render();
    },
  });

  bindPairing({
    pair: (code) => transport.pair(code),
    manual: (host, port) => transport.manual(host, port),
  });

  bindActivity({
    dismiss: () => store.dispatch({ type: '@dismiss' }),
  });

  bindEvents({
    toggle: () => {
      ui.eventsOpen = !ui.eventsOpen;
      if (ui.eventsOpen) store.dispatch({ type: '@seen' });
      render();
    },
    close: () => {
      ui.eventsOpen = false;
      render();
    },
  });

  byId('robot-toggle')?.addEventListener('click', () => {
    ui.robotOpen = !ui.robotOpen;
    render();
  });
  byId('calibration-toggle')?.addEventListener('click', () => {
    ui.calibrationOpen = !ui.calibrationOpen;
    render();
  });

  bindDrive({
    stop: () => planner.stop(),
    previewNumeric: (value) => planner.numeric(value),
    go: () => planner.go(),
    cancel: () => planner.cancel(),
    dismiss: () => planner.dismiss(),
  });

  if (radar) {
    bindRadar(radar, {
      onCenter: () => {
        radar.invalidate();
      },
    });
  }

  store.subscribe(render);
  render();
  return { render, ui, radar };
}

/** Starts the single requestAnimationFrame loop that redraws a dirty radar. */
export function startRenderLoop(store, radar, frame = globalThis.requestAnimationFrame) {
  let dirty = true;
  let lastDraw = 0;
  store.subscribe(() => {
    dirty = true;
  });
  const tick = (time) => {
    if (dirty || time - lastDraw > 250) {
      radar.render(store.state, Date.now());
      dirty = false;
      lastDraw = time;
    }
    frame(tick);
  };
  frame(tick);
}
