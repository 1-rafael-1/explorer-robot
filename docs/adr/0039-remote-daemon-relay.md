# The remote daemon: raw TCP to the Bridge, one JSON WebSocket to the app

The Link Protocol (ADR-0030, ADR-0031) settles the wire between the robot and the Bridge.
ADR-0038 settles the app-facing JSON: one versioned WebSocket, centimetres and counter-clockwise
degrees, a reload-safe snapshot, replayed events and the connection states. Ticket 15 shapes the
daemon that sits between the two, and the shape is mostly fixed by two facts: the app is a static
folder with no build step, and the Bridge is a `no_std` C3 whose only protocol is a framed binary
stream.

The daemon therefore has two independent peers and two independent failure modes. A stalled app
must not back-pressure the robot; a stalled robot must not wedge the app. The daemon is the only
place both are visible, so it owns the robot Session, the app connections, the retention that
makes a reload safe, and the liveness rule that stops Remote-originated work when the operator's
browser stops beating.

The daemon is not a byte pump. It decodes the wire, flips units and signs, retains the latest
snapshot, and owns the link Session. It is a **pure relay in the app-facing sense**: the app-facing
path, schema and semantics name neither the daemon nor its process, so the WebSocket server can
later move into the Bridge without touching the app. "Pure" is about where the contract stops, not
about forwarding bytes.

**Status:** accepted. One Rust binary connects to the Bridge over **raw TCP with the ADR-0037
HMAC preamble**, serves the SPA at `/` and one JSON WebSocket at `/ws`, and owns the app-facing
state. It binds loopback by default; serving on the LAN is an opt-in and the same pairing token
gates both the Bridge link and the app WebSocket. Retention is in memory: latest-wins slots per
stream, a 300-event replay ring, one `last_terminal` and the catalogue. Stream sequence numbers
are the daemon's and monotonic; events keep the wire's `(boot_id, seq)`. Commands get a
daemon-owned wire sequence mapped back to the originating app's `command_seq`. A 2 s app heartbeat
loss stops PING and, only for a claim the daemon started, sends `STOP_MOTION(brake=1)`; a 3 s
robot-frame loss is `reconnecting` and a retry. Ticket 14 places the crates and the static folder;
no code is written by this decision.

**Considered Options**

- **WebSocket to the Bridge.** Rejected. The Link Protocol is already framed, and the Bridge is a
  `no_std` C3 with no HTTP server; a WebSocket upgrade would add HTTP, SHA-1/base64 and a second
  framing layer for a segment whose only client is the daemon. Raw TCP plus a fixed preamble is
  the smallest carrier that still proves the token before any Link byte.
- **A daemon-specific REST surface per panel.** Rejected. ADR-0038 already rejects it: the app
  would depend on daemon routes, and the WebSocket server could not move into the Bridge. One
  versioned schema keeps the app static and the server replaceable.
- **A byte-passthrough relay.** Rejected. It would put the wire codec in the browser (a JS port or
  wasm), duplicate the daemon's job, and move unit/sign conversion and retention into the app.
  ADR-0031 already says the daemon is the app's typed boundary.
- **One app connection, a second replaces the first.** Rejected. A reload, a second window and a
  phone on the same desk are normal; the Bridge still enforces exactly one authenticated daemon,
  so the daemon can serve many tabs of one operator.
- **Per-app unbounded outbound queues.** Rejected. A slow or frozen browser would grow memory and
  eventually back-pressure the link. Streams are latest-wins slots, events are a shared ring with
  a per-app cursor, and a client that makes no progress is closed.
- **Daemon-issued `THROTTLE` or credit accounting.** Rejected. The wire is about 10 KB/s, the
  relay coalesces per stream, and the Bridge already owns UART pressure. A second throttle owner
  adds state without changing the app's data rate.
- **Persisting the event ring, `last_terminal` or the catalogue.** Rejected. The daemon is the
  operator's live Session, not a recorder. The panel is the durable Result Report, and a daemon
  restart is honestly a fresh snapshot. Only the token, the cached endpoint and the last device
  id are persisted; there is no recording to replay.
- **An epoch per handshake or reconnect.** Rejected. A new epoch resets the robot's command dedupe,
  so a retry after a reconnect could re-execute a volatile command. The epoch is chosen once per
  daemon process and reused; a robot Boot Id change already resets the robot's dedupe.
- **Always send `STOP_MOTION` on app-frozen.** Rejected. It would abort a panel-originated
  Procedure because a laptop froze. The daemon stops only a claim it started; the robot's deadman
  already stops every Remote-originated claim after 2 s of silence, and never touches panel work.
- **A token in the WebSocket URL or HTTP Basic.** Rejected. Both put the pairing token into
  history, proxy logs and screenshots. A first message keeps it out of the URL and still works
  inside a browser WebSocket, which cannot set arbitrary headers.
- **A fixed HTTP port with no fallback.** Rejected. Port conflicts are common on a developer PC; an
  ephemeral fallback with a printed URL is cheap and keeps the browser open path honest.

**Decision**

*Two peers, one process.*

One binary, `remote-daemon`, built on `tokio` + `axum`. It is runnable with `cargo run -p
remote-daemon` during development and wraps in Tauri later without an app rewrite. It hosts two
adapters around one relay:

| module | responsibility |
|---|---|
| `link` | TCP to the Bridge: preamble, `HELLO`/`PING`, stream/event decode, event ACK, command sequence and retry |
| `relay` / `session` | app-facing state: latest-wins slots, event ring, `last_terminal`, catalogue, command map, app cursors, app liveness and the app-frozen timer |
| `app` | `axum` WebSocket and static files, the JSON schema, `Origin` and auth gates |
| `discovery` | the UDP beacon listener and the endpoint cache |
| `config` | the token store, last endpoint and device id |
| `handshake` | the HMAC preamble, shared with the Bridge |

The `relay` is the deep module and the seam. The `link` adapter never awaits an app; the `app`
adapter never awaits the robot. Every per-stream and per-event path crosses the relay as a
bounded, non-blocking update.

*The Bridge link: raw TCP and the ADR-0037 preamble.*

The daemon opens a TCP connection to the Bridge's Link Protocol port (advertised by the Beacon,
default 9000). Before any Link byte, the two sides exchange a fixed framed preamble:

| message | direction | payload |
|---|---|---|
| `CLIENT_HELLO` | daemon → Bridge | 32-byte client nonce |
| `BRIDGE_CHALLENGE` | Bridge → daemon | 32-byte bridge proof, 32-byte bridge nonce, device id (6 B), flags, name |
| `CLIENT_AUTH` | daemon → Bridge | 32-byte client proof |
| `ACCEPT` | Bridge → daemon | empty; the Link Protocol starts |
| `DENY` | Bridge → daemon | reason (`unauthorized` / `busy` / `setup_mode` / `version`) and optional `retry_ms` |

Every message is `magic "EXPH"`, preamble major, type, length and payload. The proofs are
HMAC-SHA256 with domain-separated labels:

```
bridge_proof = HMAC(token, "explorer-link-v1 bridge\0" || client_nonce || bridge_nonce
                          || device_id || flags || name)
client_proof = HMAC(token, "explorer-link-v1 client\0" || client_nonce || bridge_nonce)
```

The daemon sends only its nonce before verifying the Bridge (ADR-0037's rule), so a spoofer cannot
make the daemon reveal token-derived bytes. The Bridge verifies the client proof, rate-limits
failures and never forwards a frame before `ACCEPT`. The daemon uses `getrandom`; the Bridge uses
its hardware RNG. `hmac` + `sha2` are `no_std` with no allocator; their code-size cost on the C3 is
a Bridge open item. The identity fields in the challenge let a manual `host:port` connection learn
its device id and let the daemon report `setup_mode`; `DENY` reasons map directly onto the app's
`unauthorized`, `busy`, `setup_mode` and version failure states.

*The Session: discovery, bring-up and recovery.*

The daemon listens for the ADR-0037 Discovery Beacon on fixed UDP port 9001, deduplicates by device
id and caches the last endpoint. Discovery runs from process start; the robot Session is opened
only while at least one app connection has authenticated, and kept through the post-app grace. The
cached endpoint is tried first; a beacon is needed only on first connect or after an address
change. A manual `host:port` from the app covers networks that block broadcast.

A Session is brought up in this order:

1. TCP connect and the preamble; a `DENY` ends this attempt with the mapped state.
2. `HELLO` with a **random u32 epoch chosen once at daemon start** and baseline stream rates; no
   optional capability is requested in v1. `HELLO` retries every 1 s, up to five. `HELLO_ACK`
   returns the robot's Boot Id and starts the Session.
3. `GET_STATUS` and `GET_CATALOGUE` (paged) seed the snapshot, with a 1 s budget. A failure does
   not block the Session: the snapshot carries what it has, `catalogue` may be `null`, and the
   daemon retries once in the background.
4. `connection{state:"connected"}` and then a `snapshot`.

While the Session is open the daemon `PING`s at 1 Hz whenever an app is live, honours the robot's
`THROTTLE` hints only by logging them, ACKs events after storing them, and keeps one command in
flight. The epoch stays fixed across reconnects, so a retried command is still recognised as a
duplicate; a robot Boot Id change resets the robot's own dedupe and voids in-flight commands.

If no decoded robot frame arrives for **3 s**, the daemon declares `reconnecting`, closes the TCP
and retries with backoff: 500 ms, 1, 2, 4, 8 s cap, ±20% jitter, reset on `HELLO_ACK`. A `BUSY`
denial retries after its `retry_ms` (default 4 s); `unauthorized` is surfaced and **not**
auto-retried (a rotated token needs the operator to re-pair); `setup_mode` does not connect at all;
no beacon for 3 s at startup is `no_robots`. Three seconds is safe: an idle robot still sends
motion and activity at 1 Hz.

A **same-Boot-Id** reconnect keeps the stream slots (their `ts` exposes the age), `last_terminal`,
the event ring and the command records, then sends a fresh snapshot; the robot retransmits its
unACKed events and the app deduplicates. A **Boot Id change** clears the stream slots,
`last_terminal`, the event ring and the command map, marks every pending command `indeterminate`,
re-fetches the catalogue and sends a fresh snapshot after bring-up. Clearing the ring matters: the
new boot's event `seq` restarts at 1, so mixed-boot history would confuse the app's gap and
ordering rules.

In v1 the daemon talks to one robot: the last-used device id if it is still seen, otherwise the
only beacon. Several beacons with no history pick one and name it in `detail`; a picker is
additive later.

*The app boundary: one WebSocket and the names it may use.*

The daemon serves the SPA at `/` and one WebSocket at **`/ws`**. The app derives the URL from the
page origin, `ws(s)://${location.host}/ws`; the version lives in `hello.protocol`, never in the
path, because the schema is additive-only. The server's `hello` uses a stable service name,
`"explorer-remote"`, not a process name; `robot.model` is `null` unless the Bridge advertises one.
Nothing in the app-facing contract names the daemon, its files or its CLI.

Auth is decided **per peer address**, not per bind: a loopback peer needs no token (the PC user
account is the boundary, per ADR-0037), and a non-loopback peer does. The server advertises
`auth_required` in `hello`. When required, the first app message must be `auth` (a stored token) or
`pair` (a new code, which also stores it); anything else, a wrong code, or 5 s of silence gets
`connection{state:"unauthorized"}` and close code **4401**. Failures are constant-time compared
and rate-limited per IP. A protocol mismatch closes **4426**; shutdown is **1001**. The 2 s
app-liveness timer and the robot Session start only after auth: an auth-pending browser must not
hold the Bridge's single-client slot or look like a live operator. The app sends a 1 Hz
`heartbeat`; commands are accepted only from a connection whose heartbeat is fresh (within 2 s),
and a command does not itself renew liveness.

The WebSocket upgrade checks `Origin` before accepting: it must match the request's `Host`, or be
one of the shell's origins (`tauri://localhost`, `http://tauri.localhost`). A missing `Origin` is
allowed for local non-browser tools on loopback, and after auth on the LAN. Static responses carry
`X-Content-Type-Options: nosniff`, `Referrer-Policy: no-referrer`, a strict CSP
(`default-src 'self'; script-src 'self'; style-src 'self' 'unsafe-inline'; img-src 'self' data:;
connect-src 'self' ws: wss:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'`), and
`Cache-Control: no-store` — there is no build step and no hashed asset names. An existing file is
served; a path without an extension falls back to `index.html`; a missing asset with an extension
is 404; `/ws` is never a fallback.

*The relay: state, retention and sequence.*

The relay owns the app-facing state in memory:

| state | retention |
|---|---|
| stream slots | one latest sample per stream (`power`, `motion`, `activity`, `perception`, `lidar`, `calibration`); an unseen stream is omitted from the snapshot |
| event ring | 300 events, oldest dropped; an overflow emits `event_overflow{dropped}` |
| terminal Activity | one `last_terminal`, retained until the next terminal sample or a Boot Id change |
| catalogue | the robot's `GET_CATALOGUE` response for the current Session |
| app cursors | one per app connection, for the shared event ring |
| command map | wire `command_seq` → app connection and app `command_seq`, bounded (64 entries) |

Stream sequence numbers are **the daemon's and monotonic per stream, never reset**. The wire's
`seq` is u16 per stream and wraps or restarts; the app only needs `(name, seq)` dedupe, and a reset
would make it drop fresh samples. Events keep the wire's `(boot_id, seq)`, because the app needs
the Boot Id to decide "outcome unknown". The daemon ACKs a robot event only after storing it, so a
retransmission is never lost.

A `snapshot` may arrive **at any time**: on app connect, on a robot reconnect, on a Boot Id change,
and after a catalogue retry. It replaces the streams, Boot Id, `last_terminal` and catalogue; its
events are merged by `(boot_id, seq)`. On a same-Boot-Id snapshot an in-flight command and drive
plan are not invalidated; only a Boot Id change voids them.

*Commands and events.*

The app's `command_seq` is app-local, and two tabs can both use `1`; the wire's is a u16 scoped to
the Session epoch and is the robot's dedupe key. The daemon allocates the wire sequence and maps
it back to `(app connection, app command_seq, command name)`. `command_result` goes to the
originating connection with its own `command_seq`. Completion events are broadcast, with the
originating app's `command_seq` and `null` for every other tab; the app never sees a wire sequence.
A completion record outlives the command ACK timeout, because a drive's outcome arrives seconds
after its ACK; it is evicted when the completion event arrives, after 60 s, or on a Boot Id change.
The ACK is never the drive outcome; the completion event is.

Volatile commands (`drive_distance`, `rotate_by`) are sent once and never retried: a 3 s no-ACK or
a Boot Id change is `timeout` / `indeterminate` and the operator re-issues. Idempotent commands
retry with the same wire sequence and the `RETRY` flag every 1 s, up to three times, then
`timeout`. Commands from an app whose heartbeat is stale are rejected `stale`; commands while
disconnected are rejected `stale` with "Robot link lost" rather than queued. One command is in
flight at a time; `stop_motion` is prioritised to the front of the queue but does not preempt an
in-flight frame (the ACK window is milliseconds; the app's Stop is explicitly best-effort, while
the panel Stop and the local brakes are the immediate ones).

A `disconnect` from any tab releases the shared robot Session for all tabs and stops auto-connect,
but **keeps the stored token**; forgetting or rotating it is the panel's `BRIDGE_CONTROL` action.
A second tab does not replace the first. The daemon's own `GET_STATUS`/`GET_CATALOGUE` queries
never appear in the app's command map.

*Units, signs and the conversion.*

The conversion is ADR-0031 → ADR-0038 with these concrete rules:

| wire | JSON |
|---|---|
| `POWER` `0xFF` / `0xFFFF` | `null` |
| `ACTIVITY` stage 0/1/2 | `running` / `complete` / `failed`; `percent 0xFF` → `null` |
| `CALIBRATION` 0/1/2 | `unknown` / `loaded` / `missing` |
| `PERCEPTION` flags bit 0/1 | `obstacle`, `floor_drop` booleans |
| `LIDAR_CLOUD` 360 × u16, `0xFFFF` | `ranges_cm` with `null`; `first_deg:0`, `step_deg:1`, `clockwise:true` |
| `OBSTACLE` source 0 | `"lidar"`; an unknown source is `"unknown"` |
| `ROTATE_BY` `angle_cdeg` (clockwise) | `angle_deg` counter-clockwise; the daemon sends `round(-angle_deg × 100)` |
| `ROTATE_COMPLETED` `target_cdeg` | `target_deg` counter-clockwise |
| `ROTATE_COMPLETED` `error_cdeg` | `error_deg` = **actual − target**, counter-clockwise |
| `DRIVE_DISTANCE_COMPLETED` `distance_cm` | `actual_cm`, forward-positive |
| ACK `VERSION_MISMATCH` | `rejected` with a detail (ADR-0038's status set has no separate value) |

`rotate_by` is validated to `[-18000, 18000]` centidegrees before the clamp the app already
applies. `start_procedure` takes no `params` in v1 — no Remote-startable Procedure takes one and
the catalogue carries no parameter schema — so a `params` field is `bad_argument` until that
changes. `ts` is the daemon's receive time; the app derives age, so the robot and PC clocks need
not agree.

The one ambiguity is `ROTATE_COMPLETED.error_cdeg`: ADR-0031 names the field but not its sign.
The boundary rule is **actual − target**, so the protocol crate must define the wire's
`error_cdeg` the same way in clockwise centidegrees and the daemon flips it. If the drive
subsystem's telemetry already means target − actual, the conversion flips once, in the protocol
crate, not at the boundary. This is the only mapping open item.

*Independent stalls.*

**A slow app** never touches the robot link. Each connection has a writer with three lanes:
latest-wins stream slots (not a queue), a cursor into the shared event ring (a fall behind the
oldest entry emits `event_overflow` and jumps), and prioritised control messages. A stream message
is serialized once and shared. A connection that makes no write progress for 2 s is closed; it
reconnects and receives a snapshot. The relay has no unbounded per-app queue, and the link task
updates it synchronously.

**A frozen app** is a liveness failure, not a transport failure. After 2 s without a heartbeat the
daemon sends `STOP_MOTION(brake=1)` **only if it started the live claim** (a Manual Drive it issued
or a Procedure it started that is still the running Activity), then stops PING. If the live
Activity is panel work the daemon did not start, it only stops PING: the robot's deadman stops
Remote-originated claims and never panel work. The robot TCP is held for a 5 s grace so a reload
reconnects without re-handshaking; if no heartbeat returns, the daemon closes it and frees the
Bridge's single-client slot. On resume it sends a fresh snapshot; a missed `command_result` is not
replayed, because the snapshot is the truth and an in-flight volatile command is already "not
confirmed". Zero app connections uses the same timer and grace. Graceful shutdown takes the same
path: conditional stop, no PING, close.

**A stalled robot** is detached, not fatal. The app keeps its connection and the last snapshot
(dimmed by age), and receives `connection{state:"reconnecting", retry_in_ms}`. The daemon closes
and retries with backoff. A same-Boot-Id recovery sends a fresh snapshot; a Boot Id change resets
the relay as above and moves in-flight commands to `indeterminate`. Neither direction ever waits
on the other.

*Process shape, config and the browser.*

The daemon binds `127.0.0.1:8765` by default; `--host 0.0.0.0` and `--port` opt into serving
elsewhere (the auth gate follows the peer address, not the bind). If 8765 is taken it falls back
to an ephemeral port and prints the URL. It opens the default browser unless `--no-open`. The SPA
is embedded in the binary by default; `--assets <path>` serves from disk during development, which
matters because the app has no build step. `--config`, `--verbose` and `--no-open` complete the
CLI; no secret is accepted on the command line.

The token store is a per-user `config.toml` via `directories`, mode `0600`, written atomically:
per-device tokens, a `default_token` for manual endpoints, the last endpoint and device id. The
Beacon is UDP 9001 (a fixed shared constant); the Link port is the Beacon's advertised value with
9000 as the manual default; HTTP is 8765 by default. Tauri later points a webview at the daemon's
URL, keeping the app same-origin, or injects the endpoint; either way the app is unchanged.

*The pure-relay invariant.*

> The app-facing surface is exactly `GET /` (the static SPA), `GET /ws` (one WebSocket), protocol
> version 1, and the ADR-0038 schema with the ADR-0039 clarifications. No message, field, path or
> close code names the daemon, its process, its files or its CLI. Any server implementing that
> surface is interchangeable from the app's point of view; discovery, HMAC, token storage,
> retention and transport live behind it.

"Pure relay" means no domain policy and no daemon identity in the contract, not byte passthrough.
The daemon keeps no Procedure table (the Catalogue comes from the robot), no obstacle logic and no
map. Translation, retention and the app-frozen rule are the boundary's job. The future
Bridge-hosted app (map "Not yet specified") is a server change, not an app change.

*Tests.*

The relay is host-tested through a fake link and an in-memory app transport: snapshot assembly,
latest-wins coalescing, ring overflow, command-sequence mapping, the app-frozen timer, Boot Id
resets, sign conversion and JSON round-trips. The preamble is tested against a fake Bridge,
including a bad proof, `busy`, `setup_mode` and the 5 s deadline. The protocol crate's simulated
lossy link backs the ACK/retry and event-ACK tests. There is no HIL, and none of this needs a
robot.

**Consequences**

- The app remains a static folder and one WebSocket. Moving the server into the Bridge changes the
  server, not the app: same path, schema and semantics.
- The daemon is a Session, not a recorder. A daemon restart loses the replay and `last_terminal`;
  the panel remains the durable Result Report. Only the token and endpoint cache are persisted.
- Several browser tabs share one robot Session; the Bridge still sees one authenticated client.
  One tab's stop or disconnect affects all, which is the operator model the panel already has.
- The app-frozen rule never stops panel-originated work. A laptop that sleeps stops only the work
  that laptop started; the robot's deadman covers the rest.
- A stalled app cannot slow the robot, and a stalled robot cannot wedge the app. The cost is
  coalescing: a slow browser sees fewer scans, never a queue.
- The daemon grows discovery, the HMAC preamble, a token store and the relay; the Bridge must
  implement the preamble, the Beacon flag and the rate limit. Both are small, and the robot is
  unchanged.
- Loopback is unprotected by the token but protected by the PC account and the `Origin` check.
  LAN serving is plaintext plus the pairing token; TLS remains out of scope (ADR-0037).
- `setup_mode` is only honest in APSTA, where the station Beacon can carry the flag. In AP-only
  setup there is no Beacon, so the app shows `no_robots`; the daemon does not guess.
- The app's prototype must adjust its snapshot handler (do not reset the drive plan on a same-boot
  snapshot), accept `null` `command_seq`/`model`, and handle `auth` and the close codes.
- The 2 s heartbeat, 3 s robot stall, 5 s session grace and 8 s backoff are provisional until a
  real network and a real browser exercise them; there is no HIL rig to do that now.

**Open items**

- Implement the daemon, the preamble in the Bridge, the Beacon `setup_mode` flag and the app's
  `auth`/snapshot handling; ticket 14 places the crates, the static folder and CI. Ticket 14 is
  resolved in ADR-0040: the daemon and `bridge-net` live in the `remote/` host workspace, the app
  at `remote/app/`, and CI runs three jobs.
- Define `ROTATE_COMPLETED.error_cdeg` as actual − target in `remote-protocol`; confirm the drive
  telemetry's sign before writing the mapping.
- Confirm the HMAC/SHA-256 code-size cost and the C3's hardware RNG use on the Bridge.
- Implement the `setup_mode` flag and `tcp_port = 0` in the Beacon (ADR-0037 amendment); AP-only
  setup remains `no_robots` until the app's copy says so.
- Confirm the 2 s heartbeat, 3 s stall, 5 s grace, 1 s bring-up budget and 8 s backoff cap on a
  real LAN and a real phone browser.
- Give Tauri a shell that loads the daemon URL (same origin) or injects the WS endpoint; no app
  rewrite.
- Revisit a remote-startable Procedure with parameters: the catalogue needs a parameter schema and
  `start_procedure` a params encoding before the daemon accepts `params`.
