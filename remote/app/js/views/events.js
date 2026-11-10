// The on-demand Events panel: a replayable list with an unread badge. The radar
// stays uncluttered; opening the panel marks the replay seen (ADR-0038).

import { unreadEvents } from '../store.js';
import { byId, element, setAttr, setData, setHidden, setText } from './dom.js';

/** Wires the Events toggle and close button. */
export function bindEvents({ toggle = () => {}, close = () => {} } = {}) {
  const open = byId('events-toggle');
  if (open) open.addEventListener('click', () => toggle());
  const shut = byId('events-close');
  if (shut) shut.addEventListener('click', () => close());
}

/** Renders the badge and, when open, the replay list. */
export function renderEvents(state, ui = {}) {
  const unread = unreadEvents(state);
  const badge = byId('events-badge');
  setText(badge, unread > 0 ? String(Math.min(unread, 99)) : '');
  setHidden(badge, unread === 0);
  setAttr(byId('events-toggle'), 'aria-label', unread === 0 ? 'Events' : `Events, ${unread} unread`);

  const panel = byId('events-panel');
  setHidden(panel, !ui.eventsOpen);
  setAttr(byId('events-toggle'), 'aria-expanded', String(Boolean(ui.eventsOpen)));
  const list = byId('events-list');
  if (!list) return;
  // Event timestamps are Relay-clock milliseconds, so they need the same offset
  // as every age and staleness check before they read as wall-clock time.
  const offset = state.clockOffset ?? 0;
  list.replaceChildren(...state.events.slice().reverse().map((event) => row(event, offset)));
  if (state.events.length === 0) {
    list.append(element('li', 'events-empty', 'No Events yet.'));
  }
  setData(byId('events-panel'), 'gap', String(state.eventGap));
}

/** Returns a short, human label for an Event name. */
export function eventTitle(event) {
  switch (event.name) {
    case 'obstacle':
      return event.v?.detected ? 'Obstacle detected' : 'Obstacle cleared';
    case 'floor_drop':
      return event.v?.detected ? 'Floor drop detected' : 'Floor drop cleared';
    case 'procedure_completed':
      return `Procedure ${event.v?.stage ?? 'finished'}`;
    case 'drive_distance_completed':
      return `Drive ${event.v?.status ?? 'finished'}`;
    case 'rotate_completed':
      return `Rotate ${event.v?.status ?? 'finished'}`;
    case 'calibration_loaded':
      return `Calibration ${event.v?.status ?? 'finished'}`;
    case 'event_overflow':
      return 'Events dropped';
    default:
      return event.name ?? 'Event';
  }
}

/** Returns the secondary line for an Event. */
export function eventDetail(event) {
  const value = event.v ?? {};
  switch (event.name) {
    case 'obstacle':
      return value.source ? `Source: ${value.source}` : '';
    case 'procedure_completed':
      return value.detail || '';
    case 'drive_distance_completed':
      return `${value.actual_cm ?? '—'} of ${value.target_cm ?? '—'} cm${value.detail ? ` · ${value.detail}` : ''}`;
    case 'rotate_completed':
      return `${value.target_deg ?? '—'}° requested${value.detail ? ` · ${value.detail}` : ''}`;
    case 'event_overflow':
      return value.dropped == null ? 'Some Events were not delivered.' : `${value.dropped} Event(s) not delivered.`;
    default:
      return '';
  }
}

function row(event, offset) {
  const item = element('li', 'event-row');
  setData(item, 'name', event.name ?? 'event');
  setData(item, 'tone', tone(event));
  item.append(element('span', 'event-title', eventTitle(event)));
  const detail = eventDetail(event);
  if (detail) item.append(element('span', 'event-detail', detail));
  item.append(element('time', 'event-time', timeText(event.ts, offset)));
  return item;
}

function tone(event) {
  if (event.name === 'event_overflow') return 'warn';
  if (event.name === 'obstacle' || event.name === 'floor_drop') {
    return event.v?.detected ? 'warn' : 'info';
  }
  if (event.name === 'drive_distance_completed' || event.name === 'rotate_completed') {
    if (event.v?.status === 'failed') return 'bad';
    if (event.v?.status === 'cancelled') return 'warn';
    return 'good';
  }
  if (event.name === 'procedure_completed') return event.v?.stage === 'failed' ? 'bad' : 'good';
  return 'info';
}

function timeText(ts, offset = 0) {
  if (ts == null) return '';
  try {
    return new Date(ts + offset).toLocaleTimeString();
  } catch {
    return '';
  }
}
