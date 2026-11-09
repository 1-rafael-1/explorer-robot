// The app's one socket seam: handshake, auth/pair, heartbeat, reconnect and close
// codes (ADR-0038, ADR-0039, ADR-0043). A socket factory is injected so the
// headless suite drives a fake socket in, and observes client frames out.
//
// The Pairing Token is kept in `sessionStorage` for the tab only, sent as the
// first frame on a gated peer, and never rendered or placed in the URL.

import { CLOSE, PROTOCOL, normaliseCode } from './protocol.js';

/** The tab's Pairing Token key. */
export const TOKEN_KEY = 'explorer-remote.pairing-token';

/** How long the app waits for the server's `hello` before reconnecting. */
export const HELLO_TIMEOUT_MS = 5000;

/** The reconnect backoff (ADR-0039's shape). */
export const RECONNECT_BASE_MS = 500;
export const RECONNECT_MAX_MS = 8000;

/** Returns the WebSocket URL derived from the page origin. */
export function defaultWsUrl() {
  const location = globalThis.location;
  if (!location?.host) return 'ws://127.0.0.1:8765/ws';
  const scheme = location.protocol === 'https:' ? 'wss' : 'ws';
  return `${scheme}://${location.host}/ws`;
}

/** A `sessionStorage`-backed token slot, with an in-memory fallback. */
export function createTokenStorage(storage = globalThis.sessionStorage) {
  let fallback = null;
  return {
    read() {
      try {
        return storage?.getItem(TOKEN_KEY) ?? fallback;
      } catch {
        return fallback;
      }
    },
    write(code) {
      fallback = code;
      try {
        storage?.setItem(TOKEN_KEY, code);
      } catch {
        // Privacy modes may refuse storage; the tab keeps the in-memory copy.
      }
    },
    clear() {
      fallback = null;
      try {
        storage?.removeItem(TOKEN_KEY);
      } catch {
        // Nothing to clear.
      }
    },
  };
}

/**
 * Creates the transport.
 *
 * `socketFactory(url)` returns a WebSocket-like object (`send`, `close`,
 * `readyState`, and `onopen`/`onmessage`/`onclose`/`onerror`). `onLink(link,
 * detail)` receives the app-visible socket state; `onMessage(message)` receives
 * every parsed server message, including `hello`.
 */
export function createTransport({
  socketFactory = (url) => new WebSocket(url),
  url = defaultWsUrl(),
  storage = createTokenStorage(),
  heartbeatMs = 1000,
  reconnectBaseMs = RECONNECT_BASE_MS,
  reconnectMaxMs = RECONNECT_MAX_MS,
  random = Math.random,
  onLink = () => {},
  onMessage = () => {},
  setTimeoutFn = (fn, ms) => globalThis.setTimeout(fn, ms),
  clearTimeoutFn = (id) => globalThis.clearTimeout(id),
  setIntervalFn = (fn, ms) => globalThis.setInterval(fn, ms),
  clearIntervalFn = (id) => globalThis.clearInterval(id),
} = {}) {
  let socket = null;
  let link = 'searching';
  let detail = '';
  let stopped = true;
  let helloReceived = false;
  let helloTimeout = null;
  let heartbeat = null;
  let reconnectTimer = null;
  let backoff = reconnectBaseMs;
  let pendingPair = null;
  let manualEndpoint = null;
  let generation = 0;
  let commandSeq = (Date.now() % 1_000_000_000) + 1;

  function notify(next, nextDetail = '') {
    if (next === link && nextDetail === detail) return;
    link = next;
    detail = nextDetail;
    onLink(link, detail);
  }

  function sendRaw(message) {
    if (!socket || socket.readyState !== 1) return false;
    socket.send(JSON.stringify(message));
    return true;
  }

  function command(name, args) {
    commandSeq += 1;
    sendRaw({ type: 'command', command_seq: commandSeq, command: name, args });
    return commandSeq;
  }

  function startHeartbeat() {
    if (heartbeat != null) clearIntervalFn(heartbeat);
    heartbeat = setIntervalFn(() => sendRaw({ type: 'heartbeat' }), heartbeatMs);
  }

  function stopHeartbeat() {
    if (heartbeat != null) {
      clearIntervalFn(heartbeat);
      heartbeat = null;
    }
  }

  function sendClientHello() {
    if (!sendRaw({ type: 'hello', protocol: PROTOCOL })) return;
    notify('open');
    startHeartbeat();
    if (manualEndpoint) {
      sendRaw({ type: 'connect', host: manualEndpoint.host, port: manualEndpoint.port });
    }
  }

  function handleHello(message) {
    helloReceived = true;
    if (helloTimeout != null) {
      clearTimeoutFn(helloTimeout);
      helloTimeout = null;
    }
    onMessage(message);
    if ((message.protocol ?? PROTOCOL) !== PROTOCOL) {
      notify('protocol_mismatch', 'The Relay speaks an incompatible protocol version.');
      closeSocket(CLOSE.PROTOCOL, 'protocol mismatch');
      return;
    }
    if (message.auth_required === true) {
      const stored = storage.read();
      if (pendingPair) {
        const code = pendingPair;
        pendingPair = null;
        storage.write(code);
        sendRaw({ type: 'pair', code });
        sendClientHello();
        return;
      }
      if (stored) {
        sendRaw({ type: 'auth', code: stored });
        sendClientHello();
        return;
      }
      // Probe-close: keep the code out of the page until the operator submits it.
      notify('pairing_required', 'No pairing code is stored for this tab.');
      closeSocket(1000, 'pairing probe');
      return;
    }
    sendClientHello();
  }

  function handleMessage(raw) {
    let message;
    try {
      message = JSON.parse(String(raw));
    } catch {
      return;
    }
    if (!message || typeof message.type !== 'string') return;
    if (message.type === 'hello') {
      handleHello(message);
      return;
    }
    if (message.type === 'connection' && message.state === 'unauthorized') {
      storage.clear();
    }
    onMessage(message);
  }

  function handleClose(code) {
    stopHeartbeat();
    socket = null;
    helloReceived = false;
    if (helloTimeout != null) {
      clearTimeoutFn(helloTimeout);
      helloTimeout = null;
    }
    if (stopped) {
      notify('stopped');
      return;
    }
    if (code === CLOSE.UNAUTHORIZED) {
      storage.clear();
      notify('unauthorized', 'The pairing code was rejected.');
      return;
    }
    if (code === CLOSE.PROTOCOL) {
      notify('protocol_mismatch', 'The Relay speaks an incompatible protocol version.');
      return;
    }
    if (code === CLOSE.SHUTDOWN) {
      notify('stopped', 'The Relay is shutting down.');
    } else {
      notify('reconnecting', 'The connection was lost.');
    }
    scheduleReconnect();
  }

  function scheduleReconnect() {
    if (stopped) return;
    const jitter = 1 + (random() * 0.4 - 0.2);
    const delay = Math.min(backoff * jitter, reconnectMaxMs);
    backoff = Math.min(backoff * 2, reconnectMaxMs);
    reconnectTimer = setTimeoutFn(() => {
      reconnectTimer = null;
      if (!stopped) connect();
    }, Math.max(0, Math.round(delay)));
  }

  function connect() {
    if (stopped) return;
    if (reconnectTimer != null) {
      clearTimeoutFn(reconnectTimer);
      reconnectTimer = null;
    }
    helloReceived = false;
    notify('searching');
    const mine = ++generation;
    const next = socketFactory(url);
    socket = next;
    next.onopen = () => {
      if (mine === generation) notify('connecting');
    };
    next.onmessage = (event) => {
      if (mine === generation) handleMessage(event?.data);
    };
    next.onclose = (event) => {
      if (mine === generation) handleClose(event?.code ?? 1006);
    };
    next.onerror = () => {};
    helloTimeout = setTimeoutFn(() => {
      helloTimeout = null;
      if (mine !== generation || helloReceived) return;
      notify('reconnecting', 'The Relay did not greet.');
      closeSocket(1000, 'hello timeout');
      scheduleReconnect();
    }, HELLO_TIMEOUT_MS);
  }

  function closeSocket(code = 1000, reason = '') {
    if (!socket) return;
    const current = socket;
    socket = null;
    generation += 1;
    try {
      current.close(code, reason);
    } catch {
      // The socket may already be closing.
    }
  }

  return {
    get link() {
      return link;
    },
    start() {
      if (!stopped) return;
      stopped = false;
      backoff = reconnectBaseMs;
      connect();
    },
    stop() {
      stopped = true;
      if (reconnectTimer != null) {
        clearTimeoutFn(reconnectTimer);
        reconnectTimer = null;
      }
      stopHeartbeat();
      closeSocket(1000, 'app stopped');
      notify('stopped');
    },
    send: sendRaw,
    command,
    pair(code) {
      const normalised = normaliseCode(code);
      if (normalised.length === 0) return;
      pendingPair = normalised;
      storage.write(normalised);
      backoff = reconnectBaseMs;
      closeSocket(1000, 're-pair');
      connect();
    },
    manual(host, port) {
      manualEndpoint = { host, port };
      if (socket && helloReceived) {
        sendRaw({ type: 'connect', host, port });
      } else {
        connect();
      }
    },
    disconnect() {
      sendRaw({ type: 'disconnect' });
    },
    reconnectNow() {
      backoff = reconnectBaseMs;
      closeSocket(1000, 'retry');
      connect();
    },
  };
}
