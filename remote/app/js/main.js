// Wiring only: the store, the transport, the planner, the radar and the views.
// No module keeps a second copy of robot state (ADR-0038).

import { createPlanner } from './planner.js';
import { createRadar } from './radar.js';
import { createStore, relayNow } from './store.js';
import { createTransport } from './transport.js';
import { byId } from './views/dom.js';
import { announce, mountApp, startRenderLoop } from './views/index.js';

// A truthful boot signal for the headless smoke check: a module that fails to
// load never reaches the final marker, and an uncaught error flips it to error.
document.documentElement.dataset.boot = 'loading';
globalThis.addEventListener?.('error', () => {
  document.documentElement.dataset.boot = 'error';
});
globalThis.addEventListener?.('unhandledrejection', () => {
  document.documentElement.dataset.boot = 'error';
});

const store = createStore();

const radar = createRadar(byId('radar'), {
  onPick: (world, info) => {
    // The planner guards the drawn footprint too, so the same rule holds for
    // any future caller, not only a radar tap.
    planner.pick(world, { inside_robot: info?.inside_robot, scale: radar.scale });
  },
});

const transport = createTransport({
  onLink: (link, detail) => store.dispatch({ type: '@link', link, detail }),
  onMessage: (message) => store.dispatch(message),
});

const planner = createPlanner({
  store,
  send: (name, args) => transport.command(name, args),
  now: () => relayNow(store.state),
  announce,
});

mountApp({ store, transport, planner, radar });
startRenderLoop(store, radar);
transport.start();
document.documentElement.dataset.boot = 'ok';
