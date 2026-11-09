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

The gate is headless Chrome, with no Node, npm or bundler (see the spec's
Testing Decisions). There are three kinds of page under `tests/`:

| page | seam | what it covers |
|---|---|---|
| `pure.html` | none | the protocol helpers, geometry and the store's core reducer cases |
| `state.html` | none | snapshot merging, dedupe, Boot Id reset, Activities, Events and outcome-unknown |
| `planner.html` | injected send/clock | the Target Point, the two legs, completion, busy/unknown and Stop |
| `radar.html` | none | the World Frame camera, scan/Pose pairing and the tap guard |
| `fake.html` | injected socket | the transport handshake, heartbeat, close codes and reconnect |
| `pairing.html` | injected socket + DOM | auth/pair, 4401, protocol mismatch and the pairing screens |
| `bridge.html` | none | the Bridge Status message, chip and Robot/Link panel |
| `accessible.html` | real markup | focus order, targets, contrast tokens and the compact/phone layout |
| `real.html` | the real `/ws` | the live Relay + Bridge Stub: discovery, Session, App Snapshot, Streams, Events, point-and-go ACKs, Stop and reload reconstruction |

The static suite (and the app boot smoke) is the CI `web` job:

```sh
.github/scripts/run_web_tests.sh
```

That runner serves this folder same-origin with a small Python server and
renders every page except `real.html` (which needs the live daemon) under
headless Chrome with a virtual-time budget. A page reports `data-result="pass"`
on success; a timeout, a load error or a failed assertion fails the job.

The real chain is its own script:

```sh
.github/scripts/run_real_chain.sh
```

It builds `remote-daemon` and `bridge-stub` from `remote/`, starts the stub with
fast `motion`/`activity` Streams and scripted `obstacle`/`floor_drop` Events,
writes a temporary daemon config with the stub's `bench-token`, starts
`remote-daemon --no-open --assets app --port 0`, waits for `GET /` and the CSP
header, then drives the real page. `run_real_chain.sh` also loads `/` itself and
asserts `data-boot="ok"`, which is the embedded/`--assets` boot check; the
ADR-0039 `frame-ancestors 'none'` CSP deliberately forbids the page from making
the same assertion in an iframe.

A page finishes asynchronously over a real WebSocket, so `chrome --dump-dom`
(which dumps at the load event) is too early. The runner uses
`.github/scripts/chrome_wait.py`, a standard-library DevTools-pipe client that
polls the page's `data-result`/`data-boot` attribute and prints the page's
`#results` text on failure. It needs no Node and no Python package.

The real chain also pins the boundary the fake socket cannot: it found that the
planner's Stop sent `brake: 1` while the Relay's `stop_motion` schema requires a
boolean. The planner now sends `{ brake: true }`; `tests/planner.html` asserts
the same shape.

## The manual matrix

The repeatable headless suite is not a substitute for a person at a browser. The
manual matrix is desktop Chrome and Firefox on loopback and a LAN; the 844×390
phone-landscape viewport; 200% zoom; a long Activity label; a missing battery or
RSSI value; and a reload that reconstructs the snapshot, the last terminal
Result Report and the replayed Events. The daemon's `host_chain` and
`link_stub` suites cover the same chain at the protocol seam.

## Accessibility and phone check

The repeatable part of this pass is `tests/accessible.html`. It fetches the real
`index.html`, parses it with `DOMParser`, and then adopts the same markup to
mount the view layer with a fake transport and planner:

```sh
WEB_TESTS_FILTER=accessible .github/scripts/run_web_tests.sh
```

Checked in this pass:

- **Keyboard and focus.** Every control is a native button, input or summary and
  no element asks for a positive `tabindex`. The canvas, Stop and the numeric
  form are reachable; the compact tabs are a roving `tablist`. The runtime
  checks open Events and Robot/Link and assert that focus moves into the panel,
  that Escape closes it and restores focus to the opener, and that Escape
  cancels the Target Point preview without ever sending Stop.
- **Focus ring and contrast.** `:focus-visible` is a 3 px `#8ad4ff` ring, and the
  audit asserts the rule. Palette pairs measure at least 4.5:1 for text; control
  boundaries use `--control-line` (`#55708a`) at 3:1 or better against the panel,
  with stronger states for Primary and Stop.
- **Targets.** Every button, input and summary keeps a computed and rendered
  target of at least 34×34 CSS px, including `.small` and the phone drive bar.
- **Layout.** The real markup was measured at 844×390, at 500×195 (the narrowest
  headless Chrome window) and with the app constrained to a 422 px content width
  (844 at 200% zoom). None forced horizontal scroll; the battery and Bridge slots
  stayed fixed with a missing value; a long Activity label ellipsised without
  changing the panel height; Stop stayed visible.
- **Meaning without colour.** Connection, station, battery, Activity status,
  perception and the selected tab all carry words; the selected tab is also bold
  and underlined. The radar suite renders the scans, and the layers differ by
  stroke, dots, dashed preview, crosshair and the robot icon as well as hue.

Still manual:

- A real screen-reader pass over announcement order and live-region timing. The
  audit checks the DOM and focus behaviour, not a screen reader's output.
- A real-phone pinch and pan pass. The 844×390 and 422 px checks are layout
  checks, not touch tests.

## Limits

- The real-phone pinch pass is an open item; the 844×390 viewport is a layout
  check, not a touch test. The Firefox pass and the LAN run are manual too.
- The fixed drive/rotate speeds, icon footprint and preview styling are
  bench-confirmable.
- The Relay's timers (2 s heartbeat, 3 s stall, 5 s grace, 8 s backoff) are
  provisional until a real LAN and phone exercise them.
- The real chain performs the two drive legs as separate `go()` calls (a
  rotate-only leg and a zero-bearing drive leg) because the Bridge Stub does not
  script a completion Event; the planner's rotate-then-drive completion path is
  covered by `tests/planner.html`.
- The test pages ship inside the embedded folder, unlinked from the app.
