// The responsive shell: connection screen versus dimmed shell, the header strip
// and the compact sheet tabs (ADR-0038, ADR-0039).

import { connectionCopy } from '../protocol.js';
import { byId, setClass, setData, setHidden, setText } from './dom.js';

/** The app states that come from the socket rather than the Relay's `connection`. */
const LINK_STATES = new Set(['pairing_required', 'unauthorized', 'protocol_mismatch', 'stopped']);

/** The sheet tabs the compact layout exposes. */
export const SHEET_TABS = Object.freeze(['activity', 'events', 'robot']);

/** Returns the state the connection screen and chip should show. */
export function displayState(state) {
  if (LINK_STATES.has(state.link)) return state.link;
  if (state.link === 'reconnecting') return 'reconnecting';
  return state.connection.state;
}

/** Wires the shell's own controls (retry and the compact sheet tabs). */
export function bindShell({ retry = () => {}, onTab = () => {} } = {}) {
  const retryButton = byId('retry');
  if (retryButton) retryButton.addEventListener('click', () => retry());

  for (const tab of SHEET_TABS) {
    const button = byId(`tab-${tab}`);
    if (button) button.addEventListener('click', () => onTab(tab));
  }
}

/** Returns whether the shell stays visible with dimmed, last-known data. */
export function keepsShell(state) {
  return (
    (state.connection.state === 'connected' && state.link === 'open') ||
    (state.bootId != null && (state.link === 'reconnecting' || state.connection.state === 'reconnecting'))
  );
}

/** Renders the shell and the connection screen for `state`. */
export function renderShell(state, ui = { tab: 'activity' }) {
  const shown = displayState(state);
  const copy = connectionCopy(shown);
  const connected = state.connection.state === 'connected' && state.link === 'open';
  const keepShell = keepsShell(state);

  setHidden(byId('connection-screen'), keepShell);
  setHidden(byId('shell'), !keepShell);
  setHidden(byId('link-banner'), !keepShell || connected);
  setClass(byId('shell'), 'is-stale', keepShell && !connected);
  setClass(byId('connection-screen'), 'is-pairing', shown === 'pairing_required' || shown === 'unauthorized');

  setText(byId('connection-state'), copy.title);
  setText(byId('connection-detail'), state.linkDetail || state.connection.detail || copy.detail);
  setData(byId('connection-screen'), 'state', shown);

  setText(byId('connection-chip'), copy.chip);
  setData(byId('connection-chip'), 'state', shown);
  setText(byId('robot-label'), state.connection.robot || 'Robot');

  const retry = byId('retry');
  setHidden(retry, !(shown === 'reconnecting' || shown === 'stopped' || shown === 'busy'));
  setClass(byId('app'), 'is-connected', connected);

  const stop = byId('stop-button');
  if (stop) stop.disabled = !connected;
  setData(byId('shell'), 'state', shown);

  renderTabs(ui?.tab ?? 'activity');
}

/** Marks the selected compact tab; CSS shows the matching sheet panel. */
export function renderTabs(tab) {
  for (const name of SHEET_TABS) {
    const button = byId(`tab-${name}`);
    if (button) {
      const selected = name === tab;
      button.setAttribute('aria-selected', String(selected));
      setClass(button, 'is-selected', selected);
      setData(byId('app'), 'tab', tab);
    }
  }
}
