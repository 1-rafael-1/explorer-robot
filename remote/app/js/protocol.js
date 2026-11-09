// The App Boundary's names, constants and small pure helpers.
//
// This module is the app's copy of the implemented schema (ADR-0038, ADR-0043):
// array streams, a string robot label, a scalar event cursor, a flat catalogue,
// `host` and `port` as separate fields, and the `bridge` message. Unknown types
// and fields are ignored by the store, never guessed at here.

/** The one App Boundary protocol version (`hello.protocol`). */
export const PROTOCOL = 1;

/** The stable service name the Relay advertises (`hello.server.name`). */
export const SERVICE_NAME = 'explorer-remote';

/** The close codes the Relay uses (ADR-0039). */
export const CLOSE = Object.freeze({
  UNAUTHORIZED: 4401,
  PROTOCOL: 4426,
  SHUTDOWN: 1001,
});

/** The Pairing Token alphabet: digits and letters except I, L, O and U. */
export const CODE_ALPHABET = '0123456789ABCDEFGHJKMNPQRSTVWXYZ';

/** The two leg speeds ADR-0043 fixes for v1. */
export const DRIVE_SPEED = 35;
export const ROTATE_SPEED = 60;

/** Activity ids the app labels locally; every other label comes from the Catalogue. */
export const IDLE_ID = 0;
export const BOOTING_ID = 1;
export const MANUAL_DRIVE_ID = 0x000d;

/** The headless test / bench default manual Link port. */
export const DEFAULT_LINK_PORT = 9000;

/** The event sequence is a wire `u16`; a replayed cursor is a scalar. */
export const EVENT_LIMIT = 100;

/** Maps a Relay connection state onto the copy the app shows. */
export const CONNECTION_COPY = Object.freeze({
  searching: {
    chip: 'Searching',
    title: 'Looking for robots',
    detail: 'Listening for a Discovery Beacon on the local network.',
  },
  connecting: {
    chip: 'Connecting',
    title: 'Connecting',
    detail: 'Opening the authenticated link to the robot.',
  },
  connected: {
    chip: 'Connected',
    title: 'Connected',
    detail: '',
  },
  reconnecting: {
    chip: 'Reconnecting',
    title: 'Link lost',
    detail: 'The robot stops Remote-originated work after 2 s of silence.',
  },
  no_robots: {
    chip: 'No robots',
    title: 'No robots found',
    detail: 'No Discovery Beacon on this network. Check that the robot is on and on the same Wi-Fi, or enter its address.',
  },
  pairing_required: {
    chip: 'Pair',
    title: 'Pair with this robot',
    detail: 'Type the twelve-character code from the robot panel’s Link screen.',
  },
  unauthorized: {
    chip: 'Unauthorized',
    title: 'Unauthorized',
    detail: 'The robot rejected the pairing code. Show the code on the robot panel’s Link screen and try again.',
  },
  busy: {
    chip: 'Busy',
    title: 'Another client is connected',
    detail: 'Only one remote can control this robot. It will keep retrying.',
  },
  setup_mode: {
    chip: 'In setup',
    title: 'Robot is in setup mode',
    detail: 'Finish Wi-Fi setup on the robot panel. The app cannot reach the robot while it is setting up.',
  },
  protocol_mismatch: {
    chip: 'Update',
    title: 'Update the app',
    detail: 'The Relay speaks an incompatible protocol version. Reload the page from the Relay.',
  },
  stopped: {
    chip: 'Stopped',
    title: 'Relay stopped',
    detail: 'The Relay closed the connection. Reconnecting.',
  },
});

/** The ordered list of states the connection screen knows. */
export const CONNECTION_STATES = Object.freeze([
  'searching',
  'connecting',
  'connected',
  'reconnecting',
  'no_robots',
  'pairing_required',
  'unauthorized',
  'busy',
  'setup_mode',
  'protocol_mismatch',
  'stopped',
]);

/** Returns the copy for a connection state, falling back to `searching`. */
export function connectionCopy(state) {
  return CONNECTION_COPY[state] ?? CONNECTION_COPY.searching;
}

/** Uppercases and strips every non-alphanumeric character. */
export function normaliseCode(raw) {
  return String(raw ?? '')
    .toUpperCase()
    .replace(/[^A-Z0-9]/g, '');
}

/** Formats a code as three groups of four, e.g. `ABCD-EFGH-JKMN`. */
export function formatCode(raw) {
  return normaliseCode(raw)
    .slice(0, 12)
    .replace(/(.{4})(?=.)/g, '$1-');
}

/** Returns whether `raw` has twelve characters of the panel's alphabet. */
export function isCompleteCode(raw) {
  const code = normaliseCode(raw);
  return code.length === 12 && [...code].every((character) => CODE_ALPHABET.includes(character));
}

/** Returns the characters that are not in the panel's alphabet. */
export function invalidCodeCharacters(raw) {
  const code = normaliseCode(raw);
  return [...new Set([...code].filter((character) => !CODE_ALPHABET.includes(character)))];
}

/** Returns the hint for an invalid character, or `null` when the code is valid. */
export function codeHint(raw) {
  const invalid = invalidCodeCharacters(raw);
  if (invalid.length > 0) {
    const letters = invalid.join(', ');
    return `The Pairing Token does not use ${letters}. Use digits and letters except I, L, O and U.`;
  }
  const length = normaliseCode(raw).length;
  if (length > 0 && length !== 12) {
    return `The Pairing Token is twelve characters; ${length} entered.`;
  }
  return null;
}

/** Returns the label for an Activity id: three local labels, then the Catalogue. */
export function labelForActivity(id, catalogue = []) {
  if (id === IDLE_ID) return 'Idle';
  if (id === BOOTING_ID) return 'Booting';
  if (id === MANUAL_DRIVE_ID) return 'Manual Drive';
  const entry = catalogue.find((procedure) => procedure.id === id);
  return entry?.label ?? `Activity 0x${id.toString(16).padStart(4, '0')}`;
}

/** Normalises an angle into `[-180, 180)`. */
export function normalise180(degrees) {
  let value = (degrees + 180) % 360;
  if (value < 0) value += 360;
  return value - 180;
}

/** Normalises an angle into `[0, 360)`. */
export function normalise360(degrees) {
  let value = degrees % 360;
  if (value < 0) value += 360;
  return value;
}

/** Rounds to one decimal place. */
export function round1(value) {
  return Math.round(value * 10) / 10;
}

/** Rounds an angle to whole centidegrees, the Relay's rotate precision. */
export function centidegrees(degrees) {
  return Math.round(normalise180(degrees) * 100);
}

/** Returns whether a bearing is a non-zero centidegree, so a rotate is needed. */
export function needsRotate(degrees) {
  return centidegrees(degrees) !== 0;
}

/** Returns the connection state the app should show for a Relay message. */
export function connectionStateOf(message) {
  return message?.state ?? 'searching';
}
