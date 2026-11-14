// The static and runtime accessibility audit of the real `index.html` markup.
//
// The page is fetched and parsed with DOMParser, so the audit fails when the
// shipped markup loses a name, a label, an id or a live region. The second half
// adopts the same markup into this page, mounts the view layer with a fake
// transport/planner, and checks the focus, Escape and target-size behaviour that
// a DOM parser alone cannot see. No daemon, socket or real clock is involved.

import { createStore } from '../js/store.js';
import { mountApp } from '../js/views/index.js';
import { assert, assertEqual, autorun, test } from './harness.js';

let documentPromise = null;
let stylesPromise = null;

/** Fetches and parses the real app page once. */
function realDocument() {
  documentPromise ??= fetch('../index.html').then(async (response) => {
    assert(response.ok, 'index.html must be fetchable');
    const text = await response.text();
    return new DOMParser().parseFromString(text, 'text/html');
  });
  return documentPromise;
}

/** Fetches the real stylesheet once. */
function stylesText() {
  stylesPromise ??= fetch('../styles.css').then(async (response) => {
    assert(response.ok, 'styles.css must be fetchable');
    return response.text();
  });
  return stylesPromise;
}

/** Resolves an accessible name from aria-labelledby, aria-label or the text. */
function accessibleName(doc, element) {
  const labelledBy = element.getAttribute('aria-labelledby');
  if (labelledBy) {
    const text = labelledBy
      .split(/\s+/)
      .map((id) => doc.getElementById(id)?.textContent ?? '')
      .join(' ')
      .trim();
    if (text) return text;
  }
  const label = element.getAttribute('aria-label');
  if (label && label.trim()) return label.trim();
  return (element.textContent ?? '').replace(/\s+/g, ' ').trim();
}

/** Returns whether an input is labelled by a `<label for>` or an ARIA name. */
function hasInputLabel(doc, input) {
  if (input.getAttribute('aria-label')?.trim()) return true;
  if (input.getAttribute('aria-labelledby')?.trim()) return true;
  const id = input.id;
  return Boolean(id && doc.querySelector(`label[for="${id}"]`));
}

let mounted = null;

/** Adopts the real body markup and mounts the view layer for runtime checks. */
async function mountRealMarkup() {
  if (mounted) return mounted;
  const doc = await realDocument();
  for (const child of [...doc.body.childNodes]) {
    if (child.nodeType === 1 && child.tagName === 'SCRIPT') continue;
    document.body.append(document.importNode(child, true));
  }

  const store = createStore();
  const calls = { stop: 0, cancel: 0 };
  const transport = { reconnectNow() {}, pair() {}, manual() {}, command() {}, start() {} };
  const planner = {
    stop() {
      calls.stop += 1;
    },
    numeric() {},
    go() {},
    cancel() {
      calls.cancel += 1;
    },
    dismiss() {},
    pick() {},
  };
  const radar = {
    invalidate() {},
    render() {},
    zoomBy() {},
    panBy() {},
    center() {},
    worldAt: () => ({ x: 0, y: 0 }),
    get scale() {
      return 1;
    },
    get pose() {
      return { x: 0, y: 0, yaw: 0 };
    },
  };

  const app = mountApp({ store, transport, planner, radar });
  store.dispatch({ type: '@link', link: 'open' });
  store.dispatch({ type: 'connection', state: 'connected', robot: 'Test Robot', ts: Date.now() });
  mounted = { app, store, calls, planner };
  return mounted;
}

test('every button in the real markup has an accessible name', async () => {
  const doc = await realDocument();
  const buttons = [...doc.querySelectorAll('button')];
  assert(buttons.length >= 10, 'the shipped page keeps its controls');
  for (const button of buttons) {
    const name = accessibleName(doc, button);
    assert(name.length > 0, `button #${button.id || '(no id)'} has no accessible name`);
  }
});

test('every input in the real markup has a label or an ARIA name', async () => {
  const doc = await realDocument();
  const inputs = [...doc.querySelectorAll('input')];
  assert(inputs.length >= 5, 'the shipped page keeps its inputs');
  for (const input of inputs) {
    assert(hasInputLabel(doc, input), `input #${input.id || '(no id)'} has no label`);
  }
});

test('the canvas is named, focusable and points at the numeric form', async () => {
  const doc = await realDocument();
  const canvas = doc.getElementById('radar');
  assert(canvas, 'the radar canvas exists');
  assertEqual(canvas.getAttribute('role'), 'img');
  assertEqual(canvas.getAttribute('tabindex'), '0');
  const name = accessibleName(doc, canvas);
  assert(name.length > 0, 'the canvas has an accessible name');
  assert(/bearing/i.test(name), 'the name mentions the bearing field');
  assert(/distance/i.test(name), 'the name mentions the distance field');
  assert(/form/i.test(name), 'the name points at the keyboard form');
  assert(/dotted/i.test(name), 'the name mentions the dotted fans');
  assert(/red/i.test(name), 'the name says the fans are red');
  assert(
    /cannot see|unseen/i.test(name),
    'the name says the fans mark space the scan cannot see',
  );
});

test('the sheet tabs expose aria-selected and control existing panels', async () => {
  const doc = await realDocument();
  const tablist = doc.querySelector('.sheet-tabs');
  assert(tablist, 'the compact sheet has a tab list');
  assertEqual(tablist.getAttribute('role'), 'tablist');
  for (const id of ['tab-activity', 'tab-events', 'tab-robot']) {
    const tab = doc.getElementById(id);
    assert(tab, `${id} exists`);
    assertEqual(tab.getAttribute('role'), 'tab');
    assert(tab.hasAttribute('aria-selected'), `${id} exposes aria-selected`);
    assert(['true', 'false'].includes(tab.getAttribute('aria-selected')));
    const controls = tab.getAttribute('aria-controls');
    assert(controls && doc.getElementById(controls), `${id} controls an existing panel`);
  }
});

test('Stop is a native button with its word, independent of the panels', async () => {
  const doc = await realDocument();
  const stop = doc.getElementById('stop-button');
  assert(stop, 'the Stop button exists');
  assertEqual(stop.tagName, 'BUTTON');
  assertEqual(stop.getAttribute('type'), 'button');
  assertEqual(stop.textContent.trim(), 'Stop');
  assert(!stop.hasAttribute('hidden'), 'Stop is never hidden by layout');
});

test('the ids and live regions the modules bind are present', async () => {
  const doc = await realDocument();
  const ids = [
    'app',
    'connection-screen',
    'connection-chip',
    'connection-robot',
    'shell',
    'link-banner',
    'battery-level',
    'battery-voltage',
    'motion-left',
    'motion-right',
    'scan-age',
    'zoom-level',
    'radar',
    'activity-panel',
    'activity-label',
    'activity-stage',
    'activity-busy',
    'activity-dismiss',
    'perception-obstacle',
    'perception-floor-drop',
    'events-panel',
    'events-heading',
    'events-list',
    'events-toggle',
    'events-close',
    'robot-toggle',
    'bridge-chip',
    'bridge-chip-station',
    'bridge-chip-rssi',
    'bridge-panel',
    'bridge-heading',
    'bridge-rssi',
    'bridge-channel',
    'bridge-provisioned',
    'bridge-setup',
    'bridge-link-ready',
    'bridge-uart',
    'calibration-toggle',
    'calibration-panel',
    'drive-bar',
    'numeric-drive-form',
    'numeric-bearing',
    'numeric-distance',
    'numeric-hint',
    'target-card',
    'target-go',
    'target-cancel',
    'toast',
  ];
  for (const id of ids) assert(doc.getElementById(id), `#${id} is missing`);
  for (const id of ['link-banner', 'toast', 'connection-chip', 'drive-report']) {
    const node = doc.getElementById(id);
    const live = node.getAttribute('role') === 'status' || node.hasAttribute('aria-live');
    assert(live, `#${id} is a live region`);
  }
});

test('the target card and the numeric form stay reachable', async () => {
  const doc = await realDocument();
  const card = doc.getElementById('target-card');
  const go = doc.getElementById('target-go');
  const cancel = doc.getElementById('target-cancel');
  assert(card.contains(go), 'Go lives in the Target Point card');
  assert(card.contains(cancel), 'Cancel lives in the Target Point card');
  assert(accessibleName(doc, go).length > 0, 'Go has an accessible name');
  assert(accessibleName(doc, cancel).length > 0, 'Cancel has an accessible name');

  const form = doc.getElementById('numeric-drive-form');
  assert(form.getAttribute('aria-label')?.trim(), 'the numeric form has a name');
  assert(form.contains(doc.getElementById('numeric-bearing')), 'bearing input is in the form');
  assert(form.contains(doc.getElementById('numeric-distance')), 'distance input is in the form');
  const preview = form.querySelector('button[type="submit"]');
  assert(preview && accessibleName(doc, preview).length > 0, 'Preview has a name');
});

test('overlays can take focus when opened', async () => {
  const doc = await realDocument();
  for (const id of ['events-panel', 'bridge-panel', 'calibration-panel']) {
    assertEqual(doc.getElementById(id).getAttribute('tabindex'), '-1', `${id} can receive focus`);
  }
  assert(accessibleName(doc, doc.getElementById('events-close')).length > 0);
});

test('status is carried by words, not colour alone', async () => {
  const doc = await realDocument();
  for (const id of [
    'connection-chip',
    'bridge-chip-station',
    'battery-level',
    'activity-label',
    'perception-obstacle',
    'perception-floor-drop',
    'tab-activity',
  ]) {
    assert(doc.getElementById(id).textContent.trim().length > 0, `#${id} has a text label`);
  }
  const battery = doc.getElementById('battery-level');
  const label = battery.closest('.slot')?.querySelector('.slot-label')?.textContent.trim();
  assertEqual(label, 'Battery');
});

test('no element asks for a positive tabindex', async () => {
  const doc = await realDocument();
  for (const node of doc.querySelectorAll('[tabindex]')) {
    const value = Number.parseInt(node.getAttribute('tabindex'), 10);
    assert(!(value > 0), `#${node.id || node.tagName} has tabindex ${value}`);
  }
});

test('the stylesheet keeps the focus ring, the 34 px token and ellipsis', async () => {
  const css = await stylesText();
  assert(/--target:\s*34px/.test(css), 'the target token is 34px');
  assert(/:focus-visible\s*\{[^}]*outline:\s*3px solid var\(--focus\)/s.test(css), 'a visible focus ring');
  assert(/\.small\s*\{[^}]*min-height:\s*var\(--target\)/s.test(css), '.small keeps the 34px target');
  assert(/--control-line/.test(css), 'controls have a 3:1 boundary token');
  assert(/\.panel-head h2\s*\{[^}]*text-overflow:\s*ellipsis/s.test(css), 'long labels ellipsise');
  assert(
    /@media \(max-width: 900px\), \(max-height: 520px\)/.test(css),
    'the compact breakpoint is the spec one',
  );
  const compact = css.slice(css.indexOf('@media (max-width: 900px)'));
  assert(/\.status-strip\s*\{[^}]*flex-wrap:\s*wrap/s.test(compact), 'the compact strip wraps');
});

test('the mounted shell opens Events, moves focus in and Escape restores it', async () => {
  await mountRealMarkup();
  const toggle = document.getElementById('events-toggle');
  const panel = document.getElementById('events-panel');
  toggle.focus();
  toggle.click();
  assert(!panel.hidden, 'Events is shown');
  assert(panel.contains(document.activeElement), 'focus moved into Events');
  document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true }));
  assert(panel.hidden, 'Escape closes Events');
  assert(document.activeElement === toggle, 'focus returns to the opener');
});

test('Escape closes Robot and never triggers Stop', async () => {
  const { calls } = await mountRealMarkup();
  const toggle = document.getElementById('robot-toggle');
  const panel = document.getElementById('bridge-panel');
  toggle.focus();
  toggle.click();
  assert(!panel.hidden, 'Robot / Link is shown');
  assert(panel.contains(document.activeElement), 'focus moved into Robot / Link');
  document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true }));
  assert(panel.hidden, 'Escape closes Robot / Link');
  assert(document.activeElement === toggle, 'focus returns to the opener');
  assertEqual(calls.stop, 0, 'Escape never sent Stop');
});

test('the compact tabs keep Activity visible and switch the secondary panel', async () => {
  await mountRealMarkup();
  const activity = document.getElementById('activity-panel');
  const events = document.getElementById('events-panel');
  const bridge = document.getElementById('bridge-panel');
  document.getElementById('tab-events').click();
  assert(events.hidden === false, 'Events is the secondary panel');
  assert(activity.hidden === false, 'Activity stays visible');
  assert(bridge.hidden, 'Robot / Link is hidden');
  document.getElementById('tab-robot').click();
  assert(bridge.hidden === false, 'Robot / Link is the secondary panel');
  assert(events.hidden, 'Events is hidden');
  assert(activity.hidden === false, 'Activity stays visible');
  document.getElementById('tab-activity').click();
  assert(activity.hidden === false, 'Activity stays visible');
  assert(bridge.hidden, 'Robot / Link is hidden again');
  assert(events.hidden, 'Events is hidden again');
});

test('every control keeps at least a 34x34 CSS px target', async () => {
  await mountRealMarkup();
  const controls = document.querySelectorAll(
    '#connection-screen button, #connection-screen input, #connection-screen summary, ' +
      '#shell button, #shell input, #shell summary',
  );
  assert(controls.length >= 15, 'the shell keeps its controls');
  for (const control of controls) {
    const style = getComputedStyle(control);
    const minWidth = Number.parseFloat(style.minWidth);
    const minHeight = Number.parseFloat(style.minHeight);
    const label = control.id || control.className || control.tagName;
    assert(minWidth >= 34, `${label} min-width is ${style.minWidth}`);
    assert(minHeight >= 34, `${label} min-height is ${style.minHeight}`);
    const rect = control.getBoundingClientRect();
    if (rect.width > 0 && rect.height > 0) {
      assert(rect.width >= 34 - 0.01, `${label} is ${rect.width}px wide`);
      assert(rect.height >= 34 - 0.01, `${label} is ${rect.height}px tall`);
    }
  }
});

test('Escape closes the Target Point preview and never triggers Stop', async () => {
  const { store, calls } = await mountRealMarkup();
  store.dispatch({
    type: '@drive',
    drive: {
      plan: {
        bearing_deg: 10,
        distance_cm: 100,
        angle_deg: 10,
        pose: { x: 0, y: 0, yaw: 0 },
      },
      stage: 'idle',
      report: null,
      sequence: null,
    },
  });
  const card = document.getElementById('target-card');
  assert(!card.hidden, 'the Target Point preview is shown');
  document.dispatchEvent(new KeyboardEvent('keydown', { key: 'Escape', bubbles: true }));
  assertEqual(calls.cancel, 1, 'Escape cancels the Target Point');
  assertEqual(calls.stop, 0, 'Escape never sent Stop');
  assert(document.activeElement === document.getElementById('radar'), 'focus lands on the radar');
});

autorun();
