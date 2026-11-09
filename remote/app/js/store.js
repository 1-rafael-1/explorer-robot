// The one domain store: a pure `apply(state, message)` plus selectors.
//
// The radar, Activity, Result Report, Events and Bridge Status all read this
// state, so there is one dedupe rule and one Boot Id rule (ADR-0038, ADR-0043).
// Ephemeral view state (overlay, tab, form inputs, camera) lives in the views,
// not here. `apply` never mutates its argument.

import { EVENT_LIMIT, labelForActivity } from './protocol.js';

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
    lastTerminal: null,
    catalogue: [],
    bridge: null,
    commands: {},
    activityStopped: null,
    eventGap: false,
    overflowAt: null,
    drive: { plan: null, stage: 'idle', report: null, sequence: null },
  };
}

/** An empty drive state. */
export function emptyDrive() {
  return { plan: null, stage: 'idle', report: null, sequence: null };
}

/** Returns the label for an Activity id using the current Catalogue. */
export function activityLabel(state, id) {
  return labelForActivity(id, state.catalogue);
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

/** Returns the live Activity, the last terminal, the Stopped card, or Idle. */
export function activityView(state) {
  const live = state.streams.activity?.v;
  if (live && live.id !== 0 && live.stage === 'running') {
    return { kind: 'live', data: live };
  }
  if (state.lastTerminal) {
    return { kind: 'result', data: state.lastTerminal.v, ts: state.lastTerminal.ts };
  }
  if (live && live.id !== 0) {
    return { kind: 'result', data: live };
  }
  if (state.activityStopped) {
    return {
      kind: 'stopped',
      data: {
        id: state.activityStopped.id,
        label: state.activityStopped.label,
        stage: 'stopped',
        percent: null,
        detail: 'Stopped by an operator',
      },
    };
  }
  return { kind: 'idle', data: { id: 0, label: 'Idle', stage: 'running', percent: null, detail: '' } };
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

/** Returns whether the app holds a live robot Session. */
export function isConnected(state) {
  return state.link === 'open' && state.connection.state === 'connected';
}

/** Returns whether a drive may be issued. */
export function controlEnabled(state) {
  return isConnected(state) && state.connection.state === 'connected';
}

/** Returns the unread Event count after the replay cursor. */
export function unreadEvents(state) {
  return state.events.filter(
    (event) => event.seq != null && event.boot_id === state.bootId && event.seq > state.seenEventSeq,
  ).length;
}

/** Marks every current Event as read. */
export function markEventsSeen(state) {
  const top = state.events.reduce(
    (max, event) =>
      event.boot_id === state.bootId ? Math.max(max, event.seq ?? 0) : max,
    state.seenEventSeq,
  );
  return { ...state, seenEventSeq: top };
}

/** Clears the Result Report and the Stopped card. */
export function dismissReport(state) {
  return { ...state, lastTerminal: null, activityStopped: null };
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
    bootId,
    connection: { ...state.connection, boot_id: bootId },
    bridge: bridge ?? state.bridge,
    drive: wasActive
      ? {
          plan: null,
          stage: 'idle',
          sequence: null,
          report: {
            kind: 'unknown',
            title: 'Outcome unknown',
            text: 'The robot restarted (new Boot Id). Check it before re-issuing.',
          },
        }
      : emptyDrive(),
  };
}

/** Merges Events by `(boot_id, seq)`, keeping a bounded, ordered replay. */
function mergeEvents(state, incoming) {
  if (!Array.isArray(incoming) || incoming.length === 0) return state;
  const seen = new Set(
    state.events.map((event) => (event.boot_id == null ? `notice:${event.ts}` : `${event.boot_id}:${event.seq}`)),
  );
  const merged = [...state.events];
  for (const event of incoming) {
    const key = event.boot_id == null ? `notice:${event.ts}` : `${event.boot_id}:${event.seq}`;
    if (seen.has(key)) continue;
    seen.add(key);
    merged.push(event);
    if (event.name === 'event_overflow') {
      state = { ...state, overflowAt: Math.max(state.overflowAt ?? 0, event.ts ?? 0) };
    }
  }
  merged.sort((a, b) => (a.ts ?? 0) - (b.ts ?? 0) || (a.seq ?? 0) - (b.seq ?? 0));
  while (merged.length > EVENT_LIMIT) merged.shift();
  return { ...state, events: merged };
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

function applySnapshot(state, message) {
  let next = state;
  if (message.boot_id != null && state.bootId != null && message.boot_id !== state.bootId) {
    next = resetForBoot(state, message.boot_id, message.bridge ?? state.bridge);
  }
  const streams = {};
  for (const entry of message.streams ?? []) {
    if (entry?.name) streams[entry.name] = entry;
  }
  const firstSnapshot = next.eventCursor == null;
  const merged = mergeEvents(
    {
      ...next,
      streams,
      lastTerminal: message.last_terminal ?? null,
      catalogue: message.catalogue?.procedures ?? [],
      bridge: message.bridge ?? next.bridge,
    },
    message.events ?? [],
  );
  return {
    ...merged,
    bootId: message.boot_id ?? next.bootId,
    // A snapshot is reload-safe state, not proof that the robot Session is live;
    // only a `connection` message may say `connected`.
    connection: {
      ...next.connection,
      boot_id: message.boot_id ?? next.bootId,
    },
    eventCursor: message.event_cursor ?? next.eventCursor,
    seenEventSeq: firstSnapshot
      ? Math.max(next.seenEventSeq, message.event_cursor ?? 0)
      : next.seenEventSeq,
    // A same-Boot-Id snapshot preserves the in-flight plan; a Boot Id change already reset it.
    drive:
      next.bootId === message.boot_id || message.boot_id == null
        ? next.drive
        : emptyDrive(),
  };
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
    if (after && after.id !== 0 && after.stage === 'running') {
      const started = !before || before.id !== after.id || before.stage !== 'running';
      if (started) next = { ...next, activityStopped: null, lastTerminal: null };
    } else if (after && after.id !== 0 && (after.stage === 'complete' || after.stage === 'failed')) {
      next = { ...next, lastTerminal: entry, activityStopped: null };
    } else if (after && after.id === 0 && before && before.id !== 0 && before.stage === 'running') {
      const completed = state.events.some(
        (event) =>
          event.name === 'procedure_completed' &&
          event.v?.id === before.id &&
          event.ts >= (message.ts ?? 0) - 5000,
      );
      next = {
        ...next,
        activityStopped: completed
          ? null
          : { id: before.id, label: activityLabel(state, before.id), ts: message.ts },
      };
    }
  }
  return next;
}

function applyEvent(state, message) {
  if (message?.boot_id != null && state.bootId != null && message.boot_id !== state.bootId) {
    return state;
  }
  const key =
    message.boot_id == null ? `notice:${message.ts}` : `${message.boot_id}:${message.seq}`;
  const exists = state.events.some(
    (event) =>
      (event.boot_id == null ? `notice:${event.ts}` : `${event.boot_id}:${event.seq}`) === key,
  );
  if (exists) return state;

  const topSeq = state.events.reduce(
    (max, event) =>
      event.boot_id === message.boot_id && event.seq != null ? Math.max(max, event.seq) : max,
    0,
  );
  const gap = message.seq != null && message.seq > topSeq + 1 ? true : state.eventGap;
  let next = {
    ...state,
    events: [...state.events, message].slice(-EVENT_LIMIT),
    eventGap: gap,
    overflowAt: message.name === 'event_overflow' ? Math.max(state.overflowAt ?? 0, message.ts ?? 0) : state.overflowAt,
  };
  if (message.name === 'procedure_completed') {
    next = {
      ...next,
      lastTerminal: {
        name: 'activity',
        seq: 0,
        ts: message.ts,
        v: {
          id: message.v?.id ?? 0,
          stage: message.v?.stage ?? 'complete',
          percent: null,
          detail: message.v?.detail ?? '',
        },
      },
      activityStopped: null,
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
      state = next;
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
