// The one domain store: a pure `apply(state, message)` plus selectors.
//
// The radar, Activity, Result Report, Events and Bridge Status all read this
// state, so there is one dedupe rule and one Boot Id rule (ADR-0038, ADR-0043).
// Ephemeral view state (overlay, tab, form inputs, camera) lives in the views,
// not here. `apply` never mutates its argument.

import { BOOTING_ID, EVENT_LIMIT, IDLE_ID, MANUAL_DRIVE_ID, labelForActivity } from './protocol.js';

/** Returns the key used to dedupe an Event: `(boot_id, seq)` or a notice's timestamp. */
export function eventKey(event) {
  return event.boot_id == null ? `notice:${event.ts}` : `${event.boot_id}:${event.seq}`;
}

/** The wire Event sequence is a `u16`, so ordering must wrap at 65536. */
export const EVENT_SEQ_WRAP = 65536;

/** Half the sequence space; the nearest timeline value wins inside this range. */
const EVENT_SEQ_HALF = EVENT_SEQ_WRAP / 2;

/**
 * Extends a wire `u16` sequence onto the local, monotonic sequence timeline.
 *
 * `reference` is the locally extended cursor (or `null` before any Event). The
 * wire value is placed in the wrap window nearest that cursor, so a live 0 after
 * 65535 becomes 65536 while a stale 65535 after 0 becomes -1.
 */
export function extendSeq(seq, reference = null, known = true) {
  if (seq == null) return null;
  if (!known || reference == null) return seq;
  const base = Math.floor(reference / EVENT_SEQ_WRAP) * EVENT_SEQ_WRAP;
  let value = base + seq;
  if (value - reference > EVENT_SEQ_HALF) value -= EVENT_SEQ_WRAP;
  else if (reference - value >= EVENT_SEQ_HALF) value += EVENT_SEQ_WRAP;
  return value;
}

/** Returns the identity of a Result Report entry, stable across a refresh. */
function resultIdentity(entry) {
  if (!entry) return null;
  const value = entry.v ?? entry;
  return `${value.id ?? ''}:${value.stage ?? ''}:${value.detail ?? ''}`;
}

/** Returns a fresh initial state. */
export function initialState() {
  return {
    protocol: null,
    server: null,
    bootId: null,
    link: 'searching',
    linkDetail: '',
    connection: {
      state: 'searching',
      robot: null,
      detail: '',
      retry_in_ms: null,
      boot_id: null,
      ts: 0,
    },
    streams: {},
    events: [],
    eventCursor: null,
    seenEventSeq: 0,
    eventSeqKnown: false,
    lastTerminal: null,
    resultDismissedKey: null,
    activityRun: null,
    clockOffset: 0,
    catalogue: [],
    bridge: null,
    commands: {},
    activityStopped: null,
    eventGap: false,
    eventGapAt: null,
    overflowAt: null,
    drive: driveState(),
  };
}

/** Builds one drive state from its parts. */
export function driveState(plan = null, stage = 'idle', report = null, sequence = null) {
  return { plan, stage, report, sequence };
}

/** An empty drive state. */
export function emptyDrive() {
  return driveState();
}

/** Returns the latest pose: identity until a reserved `pose` stream lands. */
export function latestPose(state) {
  const pose = state.streams.pose;
  if (!pose) return { x: 0, y: 0, yaw: 0, ts: 0, identity: true };
  return {
    x: pose.v.x_cm,
    y: pose.v.y_cm,
    yaw: pose.v.yaw_deg,
    ts: pose.ts,
    identity: false,
  };
}

/** Returns the id of a live, non-Idle Activity, or `null`. */
export function liveActivityId(state) {
  const live = state.streams.activity?.v;
  if (!live || live.id === IDLE_ID || live.stage !== 'running') return null;
  return live.id;
}

/**
 * Returns whether a live Activity the app did not start holds the robot. The
 * app's own drive legs are not a foreign claim; Idle and Booting are not claims.
 */
export function activityBusy(state) {
  const id = liveActivityId(state);
  if (id == null || id === BOOTING_ID) return false;
  return state.drive.stage !== 'rotate' && state.drive.stage !== 'drive';
}

/** Returns the live Activity, the last terminal, the Stopped card, or Idle. */
export function activityView(state) {
  const entry = state.streams.activity;
  const live = entry?.v;
  if (live && live.id !== IDLE_ID && live.stage === 'running') {
    return { kind: 'live', data: live, ts: entry.ts ?? null, busy: activityBusy(state) };
  }
  // A run that leaves `running` with no terminal stage and no completion Event is
  // a Stopped card; it is newer than any earlier Result Report.
  if (
    state.activityStopped &&
    (!state.lastTerminal || (state.activityStopped.ts ?? 0) >= (state.lastTerminal.ts ?? 0))
  ) {
    return {
      kind: 'stopped',
      data: {
        id: state.activityStopped.id,
        label: state.activityStopped.label,
        stage: 'stopped',
        percent: null,
        detail: 'Stopped by an operator',
      },
      ts: state.activityStopped.ts ?? null,
    };
  }
  // A dismissed Result Report stays dismissed, including the terminal-stream
  // fallback, until a new Activity or terminal Event arrives. Dismissal is keyed
  // to the result itself so a same-result refresh stays hidden.
  const result = state.lastTerminal ?? (live && live.id !== IDLE_ID ? entry : null);
  if (result && resultIdentity(result) !== state.resultDismissedKey) {
    return { kind: 'result', data: result.v, ts: result.ts ?? null };
  }
  return {
    kind: 'idle',
    data: { id: IDLE_ID, label: 'Idle', stage: 'running', percent: null, detail: '' },
    ts: null,
    busy: false,
  };
}

/** Returns the power values, preserving `null` as unknown. */
export function powerView(state) {
  const value = state.streams.power?.v ?? {};
  return {
    level: value.level ?? null,
    voltage_mv: value.voltage_mv ?? null,
    ts: state.streams.power?.ts ?? null,
  };
}

/** Returns the motor outputs, preserving `null` as unknown and `0` as a value. */
export function motionView(state) {
  const entry = state.streams.motion;
  const value = entry?.v ?? {};
  return { left: value.left ?? null, right: value.right ?? null, ts: entry?.ts ?? null };
}

/** Returns the perception flags; `null` is unknown and `false` is clear. */
export function perceptionView(state) {
  const entry = state.streams.perception;
  const value = entry?.v ?? {};
  return {
    obstacle: value.obstacle ?? null,
    floor_drop: value.floor_drop ?? null,
    ts: entry?.ts ?? null,
  };
}

/** Returns the latest LiDAR sample, or an all-unknown view when it is absent. */
export function lidarView(state) {
  const entry = state.streams.lidar;
  if (!entry?.v) {
    return {
      present: false,
      ts: null,
      ranges_cm: null,
      first_deg: null,
      step_deg: null,
      clockwise: null,
    };
  }
  return {
    present: true,
    ts: entry.ts ?? null,
    ranges_cm: entry.v.ranges_cm ?? null,
    first_deg: entry.v.first_deg ?? null,
    step_deg: entry.v.step_deg ?? null,
    clockwise: entry.v.clockwise ?? null,
  };
}

/** Returns whether the app holds a live robot Session. */
export function isConnected(state) {
  return state.link === 'open' && state.connection.state === 'connected';
}

/** Returns whether a drive may be issued. */
export function controlEnabled(state) {
  return isConnected(state) && state.connection.state === 'connected';
}

/** Returns the unread Event count after the operator's read cursor. */
export function unreadEvents(state) {
  const seen = state.seenEventSeq ?? 0;
  return state.events.filter(
    (event) =>
      event.boot_id === state.bootId &&
      event.seq != null &&
      (extendSeq(event.seq, seen, state.eventSeqKnown) ?? 0) > seen,
  ).length;
}

/**
 * Returns whether a live Volatile Command's outcome is unknown: an Event
 * sequence gap or an overflow notice at or after `since`. A gap or overflow
 * before the command does not taint it.
 */
export function outcomeUnknown(state, since = 0) {
  const gapAt = state.eventGap ? (state.eventGapAt ?? 0) : null;
  const overflowAt = state.overflowAt ?? null;
  return (gapAt != null && gapAt >= since) || (overflowAt != null && overflowAt >= since);
}

/** Marks every current Event as read. */
export function markEventsSeen(state) {
  let top = state.seenEventSeq ?? 0;
  let known = state.eventSeqKnown ?? false;
  for (const event of state.events) {
    if (event.boot_id === state.bootId && event.seq != null) {
      top = Math.max(top, extendSeq(event.seq, top, known) ?? top);
      known = true;
    }
  }
  return { ...state, seenEventSeq: top, eventSeqKnown: known };
}

/** Clears the Result Report and the Stopped card, and keeps them dismissed. */
export function dismissReport(state) {
  const view = activityView(state);
  // Dismissing a Stopped card also keeps the older Result Report it covered
  // dismissed, so a reconnect snapshot cannot resurrect it.
  const key =
    view.kind === 'result'
      ? resultIdentity(view.data)
      : state.lastTerminal
        ? resultIdentity(state.lastTerminal)
        : null;
  return { ...state, activityStopped: null, resultDismissedKey: key };
}

/** Returns the active drive stage, or `idle`. */
export function driveActive(state) {
  return state.drive.plan != null || state.drive.stage === 'rotate' || state.drive.stage === 'drive';
}

/** Rebuilds the robot-derived state after a Boot Id change. */
function resetForBoot(state, bootId, bridge) {
  const wasActive = driveActive(state);
  return {
    ...initialState(),
    protocol: state.protocol,
    server: state.server,
    link: state.link,
    linkDetail: state.linkDetail,
    clockOffset: state.clockOffset,
    bootId,
    connection: { ...state.connection, boot_id: bootId },
    bridge: bridge ?? state.bridge,
    drive: wasActive
      ? driveState(null, 'idle', {
          kind: 'unknown',
          title: 'Outcome unknown',
          text: 'The robot restarted (new Boot Id). Check it before re-issuing.',
        })
      : emptyDrive(),
  };
}

/** Merges Events by `(boot_id, seq)`, keeping a bounded, ordered replay. */
function mergeEvents(state, incoming) {
  if (!Array.isArray(incoming) || incoming.length === 0) return state;
  const seen = new Set(state.events.map(eventKey));
  const ordered = [...incoming].sort(
    (a, b) => (a.ts ?? 0) - (b.ts ?? 0) || (a.seq ?? 0) - (b.seq ?? 0),
  );
  const merged = [...state.events];
  let overflowAt = state.overflowAt ?? null;
  let eventGap = state.eventGap;
  let eventGapAt = state.eventGapAt ?? null;
  let seqKnown = state.eventSeqKnown ?? false;
  let top = state.seenEventSeq ?? 0;
  for (const event of state.events) {
    if (event.boot_id != null && event.boot_id === state.bootId && event.seq != null) {
      top = Math.max(top, extendSeq(event.seq, top, seqKnown) ?? top);
      seqKnown = true;
    }
  }

  for (const event of ordered) {
    const key = eventKey(event);
    if (seen.has(key)) continue;
    seen.add(key);
    merged.push(event);
    if (event.name === 'event_overflow') {
      overflowAt = Math.max(overflowAt ?? 0, event.ts ?? 0);
    }
    if (event.boot_id != null && event.boot_id === state.bootId && event.seq != null) {
      const extended = extendSeq(event.seq, top, seqKnown);
      if (seqKnown && extended != null && extended > top + 1) {
        eventGap = true;
        eventGapAt = Math.max(eventGapAt ?? 0, event.ts ?? 0);
      }
      top = Math.max(top, extended ?? top);
      seqKnown = true;
    }
  }

  merged.sort((a, b) => (a.ts ?? 0) - (b.ts ?? 0) || (a.seq ?? 0) - (b.seq ?? 0));
  while (merged.length > EVENT_LIMIT) merged.shift();
  return { ...state, events: merged, overflowAt, eventGap, eventGapAt, eventSeqKnown: seqKnown };
}

/** Applies a daemon message; unknown types pass through unchanged. */
export function apply(state, message) {
  switch (message?.type) {
    case 'hello':
      return {
        ...state,
        protocol: message.protocol ?? null,
        server: message.server ?? null,
        connection: {
          ...state.connection,
          robot: state.connection.robot ?? message.robot?.model ?? null,
        },
      };
    case 'connection':
      return applyConnection(state, message);
    case 'snapshot':
      return applySnapshot(state, message);
    case 'stream':
      return applyStream(state, message);
    case 'event':
      return applyEvent(state, message);
    case 'bridge':
      return { ...state, bridge: message };
    case 'command_result':
      return applyCommandResult(state, message);
    case '@link':
      return {
        ...state,
        link: message.link ?? state.link,
        linkDetail: message.detail ?? '',
      };
    case '@seen':
      return markEventsSeen(state);
    case '@drive':
      return { ...state, drive: message.drive ?? emptyDrive() };
    case '@dismiss':
      return dismissReport(state);
    default:
      return state;
  }
}

function applyConnection(state, message) {
  let next = state;
  if (message.boot_id != null && state.bootId != null && message.boot_id !== state.bootId) {
    next = resetForBoot(state, message.boot_id, state.bridge);
  }
  return {
    ...next,
    bootId: message.boot_id ?? next.bootId,
    connection: {
      state: message.state ?? 'searching',
      robot: message.robot ?? next.connection.robot,
      detail: message.detail ?? '',
      retry_in_ms: message.retry_in_ms ?? null,
      boot_id: message.boot_id ?? next.bootId,
      ts: message.ts ?? 0,
    },
  };
}

/** Builds the Result Report entry shape from a `procedure_completed` Event. */
function terminalFromEvent(event) {
  return {
    name: 'activity',
    seq: 0,
    ts: event.ts,
    v: {
      id: event.v?.id ?? 0,
      stage: event.v?.stage ?? 'complete',
      percent: null,
      detail: event.v?.detail ?? '',
    },
  };
}

/** Returns the newest replayed `procedure_completed` Event, or `null`. */
function newestTerminalEvent(events) {
  let newest = null;
  for (const event of events ?? []) {
    if (event?.name !== 'procedure_completed') continue;
    if (
      !newest ||
      (event.ts ?? 0) > (newest.ts ?? 0) ||
      ((event.ts ?? 0) === (newest.ts ?? 0) && (event.seq ?? 0) > (newest.seq ?? 0))
    ) {
      newest = event;
    }
  }
  return newest;
}

function applySnapshot(state, message) {
  let next = state;
  if (message.boot_id != null && state.bootId != null && message.boot_id !== state.bootId) {
    next = resetForBoot(state, message.boot_id, message.bridge ?? state.bridge);
  }
  const bootId = message.boot_id ?? next.bootId;
  const streams = {};
  for (const entry of message.streams ?? []) {
    if (entry?.name) streams[entry.name] = entry;
  }
  const events = message.events ?? [];
  // The Relay's last terminal is authoritative; when it is absent, a replayed
  // completion Event still reconstructs the Result Report after a reload.
  const replayedTerminal = newestTerminalEvent(events);
  const lastTerminal =
    message.last_terminal ?? (replayedTerminal ? terminalFromEvent(replayedTerminal) : null);
  const activity = streams.activity?.v;
  // A snapshot is a fresh baseline after a reconnect: the latest running sample
  // is the earliest point this tab can vouch for, so an older completion cannot
  // be attributed to a run that restarted while the app was away.
  const activityRun =
    activity && activity.id !== IDLE_ID && activity.stage === 'running'
      ? { id: activity.id, startedAt: streams.activity.ts ?? message.ts ?? null }
      : null;
  const merged = mergeEvents(
    {
      ...next,
      bootId,
      streams,
      lastTerminal,
      activityRun,
      catalogue: message.catalogue?.procedures ?? [],
      bridge: message.bridge ?? next.bridge,
    },
    events,
  );
  // Keep a dismissal only when the snapshot replays the same result; a new
  // terminal that finished while the app was offline becomes visible again.
  const dismissedKey = next.resultDismissedKey;
  const resultDismissedKey =
    dismissedKey != null && lastTerminal && resultIdentity(lastTerminal) === dismissedKey
      ? dismissedKey
      : null;
  const cursor = message.event_cursor;
  const cursorExt =
    cursor == null ? null : extendSeq(cursor, merged.seenEventSeq ?? 0, merged.eventSeqKnown);
  return {
    ...merged,
    bootId,
    // A snapshot is reload-safe state, not proof that the robot Session is live;
    // only a `connection` message may say `connected`.
    connection: { ...next.connection, boot_id: bootId },
    // The scalar cursor is the Relay's replay cursor; the unread badge counts
    // after it, so the replayed history is not re-flagged on every reload. The
    // locally extended cursor keeps counting across the wire's `u16` wrap.
    eventCursor: cursor ?? next.eventCursor,
    seenEventSeq: Math.max(merged.seenEventSeq ?? 0, cursorExt ?? 0),
    eventSeqKnown: merged.eventSeqKnown || cursor != null,
    resultDismissedKey,
    // A same-Boot-Id snapshot preserves the in-flight plan; a Boot Id change was
    // already reset by `resetForBoot`, which marks in-flight work unknown.
    drive: next.drive,
  };
}

/**
 * Returns whether an Event proves that the Activity run started at
 * `startedAt` finished. A Procedure completion belongs to that Procedure only;
 * a drive/rotate completion belongs to a Manual Drive only. An Event from before
 * the run cannot end it.
 */
function completionCovers(event, id, startedAt) {
  if ((event.ts ?? 0) < (startedAt ?? 0)) return false;
  if (event.name === 'procedure_completed') return event.v?.id === id;
  if (id === MANUAL_DRIVE_ID) {
    return event.name === 'drive_distance_completed' || event.name === 'rotate_completed';
  }
  return false;
}

function applyStream(state, message) {
  if (!message?.name) return state;
  const previous = state.streams[message.name];
  if (previous && message.seq <= previous.seq) return state;
  const entry = { name: message.name, seq: message.seq, ts: message.ts, v: message.v };
  let next = { ...state, streams: { ...state.streams, [message.name]: entry } };

  if (message.name === 'activity') {
    const before = previous?.v;
    const after = message.v;
    if (after && after.id !== IDLE_ID && after.stage === 'running') {
      const started = !before || before.id !== after.id || before.stage !== 'running';
      const activityRun = started
        ? { id: after.id, startedAt: message.ts ?? null }
        : (state.activityRun ?? { id: after.id, startedAt: message.ts ?? null });
      next = { ...next, activityRun };
      if (started) {
        next = { ...next, activityStopped: null, lastTerminal: null, resultDismissedKey: null };
      }
    } else if (after && after.id !== IDLE_ID && (after.stage === 'complete' || after.stage === 'failed')) {
      // A refreshed terminal sample is the same result; only a transition into
      // that stage (or a different Activity) ends a dismissal.
      const sameTerminal = before && before.id === after.id && before.stage === after.stage;
      next = {
        ...next,
        lastTerminal: entry,
        activityStopped: null,
        activityRun: null,
        resultDismissedKey: sameTerminal ? state.resultDismissedKey : null,
      };
    } else if (
      after &&
      after.id === IDLE_ID &&
      before &&
      before.id !== IDLE_ID &&
      before.stage === 'running'
    ) {
      const startedAt = state.activityRun?.id === before.id ? state.activityRun.startedAt : null;
      const completed =
        startedAt != null &&
        state.events.some((event) => completionCovers(event, before.id, startedAt));
      next = {
        ...next,
        activityRun: null,
        activityStopped: completed
          ? null
          : { id: before.id, label: labelForActivity(before.id, state.catalogue), ts: message.ts },
      };
    }
  }
  return next;
}

function applyEvent(state, message) {
  if (message?.boot_id != null && state.bootId != null && message.boot_id !== state.bootId) {
    return state;
  }
  const key = eventKey(message);
  const exists = state.events.some((event) => eventKey(event) === key);
  if (exists) return state;

  let topSeq = state.seenEventSeq ?? 0;
  let seqKnown = state.eventSeqKnown ?? false;
  for (const event of state.events) {
    if (event.boot_id === message.boot_id && event.seq != null) {
      topSeq = Math.max(topSeq, extendSeq(event.seq, topSeq, seqKnown) ?? topSeq);
      seqKnown = true;
    }
  }
  const extended = extendSeq(message.seq, topSeq, seqKnown);
  const gap = message.seq != null && seqKnown && extended != null && extended > topSeq + 1;
  let next = {
    ...state,
    events: [...state.events, message].slice(-EVENT_LIMIT),
    eventSeqKnown: seqKnown || message.seq != null,
    eventGap: gap ? true : state.eventGap,
    eventGapAt: gap ? Math.max(state.eventGapAt ?? 0, message.ts ?? 0) : state.eventGapAt,
    overflowAt:
      message.name === 'event_overflow'
        ? Math.max(state.overflowAt ?? 0, message.ts ?? 0)
        : state.overflowAt,
  };
  if (message.name === 'procedure_completed') {
    next = {
      ...next,
      lastTerminal: terminalFromEvent(message),
      activityStopped: null,
      resultDismissedKey: null,
    };
  }
  if (message.name === 'drive_distance_completed' || message.name === 'rotate_completed') {
    next = { ...next, activityStopped: null };
  }
  return next;
}

function applyCommandResult(state, message) {
  if (message.command_seq == null) return state;
  const commands = { ...state.commands, [message.command_seq]: message };
  const keys = Object.keys(commands);
  if (keys.length > 80) {
    for (const key of keys.slice(0, keys.length - 80)) delete commands[key];
  }
  return { ...state, commands };
}

/**
 * Returns the Relay's clock expressed in the page's wall-clock scale. The
 * Relay stamps every timestamp with `clock::now_ms()` (milliseconds since the
 * daemon started), so incoming `ts` values are offset from `Date.now()` by a
 * fixed amount: `clockOffset`.
 */
export function relayNow(state, wallNow = Date.now()) {
  return wallNow - (state?.clockOffset ?? 0);
}

/**
 * Stamps an applied message with the Relay-clock offset. Only a message that
 * changed the state contributes, so an ignored or unknown message leaves the
 * store exactly as it was.
 */
function stampClock(state, message, wallNow) {
  const ts = message?.ts;
  if (typeof ts !== 'number' || !Number.isFinite(ts) || ts <= 0) return state;
  return { ...state, clockOffset: wallNow - ts };
}

/** A small observable wrapper around the pure reducer. */
export function createStore(initial = initialState()) {
  let state = initial;
  const listeners = new Set();

  function emit() {
    for (const listener of listeners) listener(state);
  }

  return {
    get state() {
      return state;
    },
    dispatch(message) {
      const next = apply(state, message);
      if (next === state) return;
      state = stampClock(next, message, Date.now());
      emit();
    },
    patch(updater) {
      const next = updater(state);
      if (next == null || next === state) return;
      state = next;
      emit();
    },
    subscribe(listener) {
      listeners.add(listener);
      return () => listeners.delete(listener);
    },
  };
}
