// The pairing screen and the manual `host:port` fallback (ADR-0038, ADR-0043).
//
// The Pairing Token is normalised to the panel's alphabet, kept for the tab
// only, and never rendered back: the field is cleared after submit and when the
// Relay rejects the code.

import {
  DEFAULT_LINK_PORT,
  codeHint,
  formatCode,
  isCompleteCode,
} from '../protocol.js';
import { byId, setHidden, setText } from './dom.js';
import { displayState } from './shell.js';

/** Wires the pairing and manual-endpoint forms. */
export function bindPairing({ pair = () => {}, manual = () => {} } = {}) {
  const form = byId('pairing-form');
  const input = byId('pairing-code');
  const hint = byId('pairing-hint');

  if (input) {
    input.addEventListener('input', () => {
      const formatted = formatCode(input.value);
      if (input.value !== formatted) input.value = formatted;
      setText(hint, codeHint(input.value) ?? '');
    });
  }

  if (form) {
    form.addEventListener('submit', (event) => {
      event.preventDefault();
      const code = input?.value ?? '';
      if (!isCompleteCode(code)) {
        setText(hint, codeHint(code) ?? 'Enter the twelve-character code from the panel.');
        return;
      }
      pair(code);
      if (input) input.value = '';
    });
  }

  const manualForm = byId('manual-form');
  if (manualForm) {
    manualForm.addEventListener('submit', (event) => {
      event.preventDefault();
      const host = byId('manual-host')?.value?.trim() ?? '';
      const port = Number.parseInt(byId('manual-port')?.value ?? '', 10);
      if (host.length === 0) {
        setText(byId('manual-hint'), 'Enter the robot’s host address.');
        return;
      }
      if (!Number.isInteger(port) || port < 1 || port > 65535) {
        setText(byId('manual-hint'), 'Enter a port from 1 to 65535.');
        return;
      }
      manual(host, port);
    });
  }
}

/** Shows the pairing field, the manual endpoint or neither, honestly. */
export function renderPairing(state) {
  const shown = displayState(state);
  const pairing = shown === 'pairing_required' || shown === 'unauthorized';
  setHidden(byId('pairing-form'), !pairing);
  setHidden(byId('manual-endpoint'), shown !== 'no_robots');
  setHidden(byId('threat-note'), shown !== 'pairing_required' && shown !== 'unauthorized');

  const input = byId('pairing-code');
  if (input) {
    if (!pairing) input.value = '';
    else if (document.activeElement !== input && input.value.length > 0 && shown === 'unauthorized') {
      input.value = '';
    }
  }
  const port = byId('manual-port');
  if (port && port.value.length === 0) port.value = String(DEFAULT_LINK_PORT);
}
