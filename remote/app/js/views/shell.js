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

  const tabs = SHEET_TABS.map((tab) => byId(`tab-${tab}`)).filter(Boolean);
  for (const [index, button] of tabs.entries()) {
    button.addEventListener('click', () => onTab(SHEET_TABS[index]));
  }
  // A tablist is one tab stop; arrow keys move between tabs (ARIA tabs model).
  for (const [index, button] of tabs.entries()) {
    button.addEventListener('keydown', (event) => {
      let next = null;
      if (event.key === 'ArrowRight') next = (index + 1) % tabs.length;
      else if (event.key === 'ArrowLeft') next = (index - 1 + tabs.length) % tabs.length;
      else if (event.key === 'Home') next = 0;
      else if (event.key === 'End') next = tabs.length - 1;
      if (next == null) return;
      event.preventDefault();
      tabs[next].focus();
      onTab(SHEET_TABS[next]);
    });
  }
}

/** Returns whether the shell stays visible with dimmed, last-known data. */
export function keepsShell(state) {
  return (
    (state.connection.state === 'connected' && state.link === 'open') ||
    (state.bootId != null && (state.link === 'reconnecting' || state.connection.state === 'reconnecting'))
  );
}

/** Returns the kept-shell banner: a lost socket and a lost robot link differ. */
export function linkBannerText(state) {
  const honesty = 'showing the last known state.';
  if (state.link !== 'open') return `Relay connection lost — ${honesty}`;
  if (state.connection.state === 'reconnecting') return `Robot link lost — ${honesty}`;
  return `Connection lost — ${honesty}`;
}

/** Formats the Relay's retry delay in seconds, keeping a half second when needed. */
function retrySeconds(ms) {
  return Number((Number(ms) / 1000).toFixed(1));
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
  setText(byId('link-banner'), linkBannerText(state));
  setClass(byId('shell'), 'is-stale', keepShell && !connected);
  setClass(byId('connection-screen'), 'is-pairing', shown === 'pairing_required' || shown === 'unauthorized');

  const detail = state.linkDetail || state.connection.detail || copy.detail;
  const retryCopy =
    shown === 'busy' && state.connection.retry_in_ms != null
      ? `The Relay will retry in ${retrySeconds(state.connection.retry_in_ms)} s.`
      : '';
  setText(byId('connection-state'), copy.title);
  setText(byId('connection-detail'), [detail, retryCopy].filter(Boolean).join(' '));
  setText(byId('connection-robot'), state.connection.robot || 'Robot');
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
      button.setAttribute('tabindex', selected ? '0' : '-1');
      setClass(button, 'is-selected', selected);
      setData(byId('app'), 'tab', tab);
    }
  }
}
