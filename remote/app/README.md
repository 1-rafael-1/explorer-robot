# app — the Remote App

The static single-page app the Relay serves at `/` (ADR-0038, ADR-0039, ADR-0043).
Vanilla ES modules and one `<canvas>`: no bundler, build step, wasm or third-party
dependency. The daemon embeds this folder at compile time; `--assets <path>` serves
it from disk during development. It follows the implemented App Boundary, not the
archived prototype's `CONTRACT.md`.

## Run it

```sh
cd remote
cargo run -p remote-daemon -- --no-open --assets app
```

Open the printed URL (`http://127.0.0.1:8765/` by default). Without `--assets`, a
normal build embeds this folder, so the daemon is still a single binary.

On loopback no token is needed. On a LAN, the app asks for the twelve-character
Pairing Token from the robot panel's Link screen; the code is kept for the tab
(`sessionStorage`), sent as the first WebSocket frame, and never shown back. The
app derives `ws://<page-host>/ws` from the page origin, so loopback, a LAN and a
future shell need no configuration.

## Layout

| path | what it owns |
|---|---|
| `index.html`, `styles.css` | the responsive shell: wide radar-first, compact status strip + radar + drive bar + tabbed sheet |
| `js/protocol.js` | message names, close codes, the Pairing Token alphabet, drive constants and labels |
| `js/transport.js` | the one socket seam: hello/auth/pair, 1 Hz heartbeat, reconnect, close codes; the socket factory is injected |
| `js/store.js` | the pure `apply(state, message)` reducer and selectors: snapshot merge, `(name, seq)` and `(boot_id, seq)` dedupe, Boot Id reset, Activity and Events |
| `js/planner.js` | the Target Point and the two-leg state machine; the sender and clock are injected |
| `js/radar.js`, `js/geometry.js` | the World Frame renderer, camera and the pure bearing/distance/footprint math |
| `js/views/` | the DOM panels (connection, power, Activity, Events, Robot/Link, drive) |
| `js/main.js` | wiring only |

One rule binds the modules: no module keeps a second copy of robot state. The
app's only durable state is the tab's Pairing Token; a reload reconstructs the
rest from the App Snapshot.

## Test it

The gate is headless Chrome with a fake socket and a fake clock, with no Node or
npm (see the spec's Testing Decisions). It is also the CI `web` job:

```sh
.github/scripts/run_web_tests.sh
```

The runner serves this folder same-origin with a small Python server and renders
every `tests/*.html` page under headless Chrome. A page reports
`data-result="pass"` on success; a timeout, a load error or a failed assertion
fails the job. `tests/pure.html` covers the pure modules; `tests/fake.html`
covers the transport against its injected socket.

The manual matrix is desktop Chrome and Firefox on loopback and a LAN, the
844×390 phone-landscape viewport, 200% zoom, a long activity label, a missing
battery or RSSI value, and a reload that reconstructs the snapshot, the last
terminal Result Report and the replayed Events. The real Relay and Bridge Stub
chain is covered by the daemon's `host_chain` and `link_stub` suites.

## Limits

- The real-phone pinch pass is an open item; the 844×390 viewport is a layout
  check, not a touch test.
- The fixed drive/rotate speeds, icon footprint and preview styling are
  bench-confirmable.
- The Relay's timers (2 s heartbeat, 3 s stall, 5 s grace, 8 s backoff) are
  provisional until a real LAN and phone exercise them.
- The test pages ship inside the embedded folder, unlinked from the app.
