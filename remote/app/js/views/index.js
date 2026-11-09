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
import { renderStreams } from './streams.js';
import { bindShell, renderShell } from './shell.js';
import { byId, setHidden, setText } from './dom.js';

let toastTimer = null;

/** The one media query that switches the shell to its compact phone layout. */
export const COMPACT_QUERY = '(max-width: 900px), (max-height: 520px)';

/** Returns the first visibly focusable control in `root`, or `null`. */
function firstFocusable(root) {
  if (!root) return null;
  return root.querySelector(
    'button:not([disabled]), [href], input:not([disabled]), select:not([disabled]), ' +
      'textarea:not([disabled]), summary, [tabindex]:not([tabindex="-1"])',
  );
}

/** Moves focus into a just-opened panel, falling back to the panel itself. */
function focusInto(panel) {
  if (!panel) return;
  if (!panel.hasAttribute('tabindex')) panel.setAttribute('tabindex', '-1');
  const target = firstFocusable(panel) ?? panel;
  target.focus?.({ preventScroll: true });
}

/** Restores focus to an opener when it is still on the page. */
function restoreFocus(node) {
  if (node && node.isConnected) node.focus?.({ preventScroll: true });
}

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
  const openers = { events: null, robot: null, calibration: null };

  const isCompact = () => Boolean(globalThis.matchMedia?.(COMPACT_QUERY)?.matches);

  function renderPanels() {
    const compact = isCompact();
    // Compact is one tabbed sheet: the selected tab owns the whole sheet.
    if (compact) {
      ui.eventsOpen = ui.tab === 'events';
      ui.robotOpen = ui.tab === 'robot';
      if (ui.tab !== 'robot') ui.calibrationOpen = false;
    }

    const activity = byId('activity-panel');
    const events = byId('events-panel');
    const bridge = byId('bridge-panel');
    if (compact) {
      // The live Activity is always visible; the tabs switch the secondary panel.
      setHidden(activity, false);
      setHidden(events, ui.tab !== 'events');
      setHidden(bridge, ui.tab !== 'robot');
    } else {
      setHidden(activity, false);
      setHidden(events, !ui.eventsOpen);
      setHidden(bridge, !ui.robotOpen);
    }
    const calibrationShown = ui.calibrationOpen && (compact ? ui.tab === 'robot' : ui.robotOpen);
    setHidden(byId('calibration-panel'), !calibrationShown);

    // The tab semantics only hold while the compact sheet is on screen.
    const role = compact ? 'tabpanel' : 'region';
    for (const panel of [activity, events, bridge]) {
      if (panel) panel.setAttribute('role', role);
    }

    byId('events-toggle')?.setAttribute('aria-expanded', String(ui.eventsOpen));
    byId('robot-toggle')?.setAttribute('aria-expanded', String(ui.robotOpen));
    byId('calibration-toggle')?.setAttribute('aria-expanded', String(calibrationShown));
  }

  function render() {
    const state = store.state;
    renderShell(state, ui);
    renderPower(state);
    renderStreams(state, Date.now());
    renderPairing(state);
    renderActivity(state);
    renderEvents(state, ui);
    renderBridge(state);
    renderCalibration(state);
    renderDrive(state);
    renderPanels();
    radar.invalidate();
  }

  function showEvents(opener = byId('events-toggle'), { focus = true } = {}) {
    ui.eventsOpen = true;
    ui.tab = 'events';
    if (isCompact()) ui.robotOpen = false;
    openers.events = opener;
    store.dispatch({ type: '@seen' });
    render();
    if (focus) focusInto(byId('events-panel'));
  }

  function hideEvents({ focus = true } = {}) {
    ui.eventsOpen = false;
    if (ui.tab === 'events') ui.tab = ui.robotOpen ? 'robot' : 'activity';
    render();
    if (focus) restoreFocus(openers.events ?? byId('events-toggle'));
  }

  function showRobot(opener = byId('robot-toggle'), { focus = true } = {}) {
    ui.robotOpen = true;
    ui.tab = 'robot';
    if (isCompact()) ui.eventsOpen = false;
    openers.robot = opener;
    render();
    if (focus) focusInto(byId('bridge-panel'));
  }

  function hideRobot({ focus = true } = {}) {
    ui.robotOpen = false;
    ui.calibrationOpen = false;
    if (ui.tab === 'robot') ui.tab = 'activity';
    render();
    if (focus) restoreFocus(openers.robot ?? byId('robot-toggle'));
  }

  function toggleCalibration() {
    ui.calibrationOpen = !ui.calibrationOpen;
    openers.calibration = byId('calibration-toggle');
    render();
    if (ui.calibrationOpen) focusInto(byId('calibration-panel'));
    else restoreFocus(openers.calibration);
  }

  /** Selects a compact sheet tab; focus stays on the tab, as the tabs model asks. */
  function selectTab(tab, opener) {
    ui.tab = tab;
    ui.eventsOpen = tab === 'events';
    ui.robotOpen = tab === 'robot';
    if (tab !== 'robot') ui.calibrationOpen = false;
    if (tab === 'events') {
      openers.events = opener ?? byId('tab-events');
      store.dispatch({ type: '@seen' });
    }
    if (tab === 'robot') openers.robot = opener ?? byId('tab-robot');
    render();
  }

  // Escape closes the topmost overlay or the Target Point preview; it never
  // triggers Stop, and it returns focus to whatever opened the closed surface.
  function onKeydown(event) {
    if (event.key !== 'Escape' && event.key !== 'Esc') return;
    if (ui.calibrationOpen) {
      event.preventDefault();
      ui.calibrationOpen = false;
      render();
      restoreFocus(openers.calibration ?? byId('calibration-toggle'));
      return;
    }
    if (ui.robotOpen) {
      event.preventDefault();
      hideRobot();
      return;
    }
    if (ui.eventsOpen) {
      event.preventDefault();
      hideEvents();
      return;
    }
    if (store.state.drive?.plan) {
      event.preventDefault();
      planner.cancel();
      restoreFocus(byId('radar'));
    }
  }

  bindShell({
    retry: () => transport.reconnectNow(),
    onTab: (tab) => selectTab(tab, byId(`tab-${tab}`)),
  });

  bindPairing({
    pair: (code) => transport.pair(code),
    manual: (host, port) => transport.manual(host, port),
  });

  bindActivity({
    dismiss: () => store.dispatch({ type: '@dismiss' }),
  });

  bindEvents({
    toggle: () => (ui.eventsOpen ? hideEvents() : showEvents(byId('events-toggle'))),
    close: () => hideEvents(),
  });

  byId('robot-toggle')?.addEventListener('click', () => {
    if (ui.robotOpen) hideRobot();
    else showRobot(byId('robot-toggle'));
  });
  byId('calibration-toggle')?.addEventListener('click', () => toggleCalibration());

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

  document.addEventListener('keydown', onKeydown);
  // A resize or zoom across the breakpoint must re-run the sheet layout.
  globalThis.matchMedia?.(COMPACT_QUERY)?.addEventListener?.('change', () => render());

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
      renderStreams(store.state, Date.now());
      radar.render(store.state, Date.now());
      dirty = false;
      lastDraw = time;
    }
    frame(tick);
  };
  frame(tick);
}
