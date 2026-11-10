// A fake WebSocket and a token slot for the headless transport tests.

/** One scripted WebSocket; the test drives open, receive and close. */
export class FakeSocket {
  constructor(url) {
    this.url = url;
    this.readyState = 0;
    this.sent = [];
    this.closed = null;
    this.onopen = null;
    this.onmessage = null;
    this.onclose = null;
    this.onerror = null;
  }

  open() {
    this.readyState = 1;
    this.onopen?.();
  }

  send(text) {
    this.sent.push(JSON.parse(text));
  }

  receive(message) {
    if (this.readyState !== 1) return;
    this.onmessage?.({ data: JSON.stringify(message) });
  }

  close(code = 1000, reason = '') {
    this.readyState = 3;
    this.closed = { code, reason };
    this.onclose?.({ code, reason });
  }

  /** Simulates the peer closing the socket (not the transport). */
  peerClose(code = 1006, reason = '') {
    this.readyState = 3;
    this.onclose?.({ code, reason });
  }

  frames(type) {
    return this.sent.filter((message) => message.type === type);
  }

  last(type) {
    return this.sent.filter((message) => message.type === type).at(-1);
  }
}

/** A socket factory that records every socket it creates. */
export function createSocketFactory() {
  const sockets = [];
  function factory(url) {
    const socket = new FakeSocket(url);
    sockets.push(socket);
    return socket;
  }
  factory.sockets = sockets;
  factory.last = () => sockets.at(-1);
  factory.count = () => sockets.length;
  return factory;
}

/** An in-memory token slot, matching the transport's storage interface. */
export function createMemoryStorage(initial = null) {
  let value = initial;
  return {
    read: () => value,
    write: (next) => {
      value = next;
    },
    clear: () => {
      value = null;
    },
    get value() {
      return value;
    },
  };
}

/** Builds a server `hello` frame. */
export function hello({ authRequired = false, protocol = 1 } = {}) {
  return {
    type: 'hello',
    protocol,
    server: { name: 'explorer-remote' },
    robot: { model: null },
    auth_required: authRequired,
  };
}

/** The client `hello` frame. */
export function clientHello() {
  return { type: 'hello', protocol: 1 };
}
