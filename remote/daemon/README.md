# remote-daemon

The host relay (ADR-0039). One binary connects to the Bridge over raw TCP with the ADR-0037
HMAC preamble, owns the robot Session, and serves the static app at `/` and one JSON WebSocket
at `/ws` (ADR-0038). It is a pure relay in the app-facing sense: it decodes the wire, flips the
sign of angles, retains latest-wins state and never exposes the wire or its own process to the
app. Its shared dependencies are `remote-protocol` (wire) and `bridge-net` (preamble, Beacon,
ports); ADR-0040 records the layout.

## Running

From `remote/` (the current-directory rule, ADR-0040):

```sh
cargo run -p remote-daemon -- --help
cargo run -p remote-daemon -- --no-open
```

| flag | meaning |
|---|---|
| `--host <HOST>` | app bind address; default `127.0.0.1` |
| `--port <PORT>` | app port; the default 8765 falls back to an ephemeral port when taken |
| `--no-open` | do not open the default browser |
| `--assets <PATH>` | serve the SPA from disk instead of the embedded copy |
| `--config <PATH>` | use this `config.toml` instead of the per-user default |
| `--verbose` | trace discovery and Session changes to stderr |

There is deliberately **no token flag**: a pairing secret is never accepted on the command
line, so it cannot land in shell history, process listings or screenshots. The config store is
the only token source.

## The config store

`config.toml` lives in the platform's per-user config directory
(`explorer-remote/config.toml`; `$XDG_CONFIG_HOME/explorer-remote/config.toml` on Linux). It is
created mode `0600` on Unix and written atomically. Fields:

| key | meaning |
|---|---|
| `default_token` | token for a manual `host:port` endpoint |
| `devices` | map of device id (lower-case hex) to `{ token, name }` |
| `last_endpoint` | last `host:port` used |
| `last_device` | last device id used |

A discovered device uses only its own token; an unpaired device is reported as
`pairing_required`, and the app's `pair` message stores the code under the last device. A
manual endpoint uses `default_token`. On Unix an existing file is tightened to `0600`; a
malformed file is an error rather than a silent loss of tokens.

## Network defaults

| surface | default |
|---|---|
| Discovery Beacon listener | UDP **9001** (`bridge-net::ports::BEACON_UDP`), 10 s freshness |
| Link Protocol to the Bridge | the Beacon's advertised port; **9000** for a manual endpoint |
| App HTTP/WebSocket | **8765** on `127.0.0.1`, ephemeral fallback |

The daemon listens for Beacons from process start but opens the robot Session only while at
least one app connection has authenticated (and through the 5 s post-app grace). Three seconds
without a decoded robot frame is `reconnecting`; reconnect backoff is 500 ms doubling to 8 s
with ±20% jitter. A robot Boot Id change clears robot-derived retained state and voids in-flight
commands; the Bridge status survives it.

## The app surface

`GET /` serves the SPA (embedded at compile time, or `--assets`). `GET /ws` is one text-frame
JSON WebSocket; its path never carries a version — `hello.protocol` does. Server messages are
`hello`, `connection`, `snapshot`, `stream`, `event`, `bridge` and `command_result`; client
messages are `hello`, `auth`, `pair`, `connect`, `disconnect`, `heartbeat` and `command`. A
`snapshot` carries the reload-safe state — the latest stream samples, the last terminal Activity, a
bounded event replay, the catalogue and the Bridge's latest `bridge` status (`null` before the
first, independent of the robot Boot Id). Close codes are **4401** (unauthorised), **4426**
(protocol mismatch) and **1001** (shutdown). A loopback peer needs no token; a non-loopback peer
must send `auth` or `pair` first. `Origin` is checked on the upgrade, and static responses carry the
ADR-0039 CSP and `no-store`.

## Running against the Bridge Stub headlessly

The daemon needs a paired token in its store even on a bench. With the stub's default token
(`bench-token`) and device id (`stub01`), a loopback run is:

```sh
# terminal 1 — from remote/
cargo run -p bridge-stub -- --beacon-addr 127.0.0.1:9001
```

```toml
# bench.toml — the hex of the stub's six-byte device id, "stub01"
[devices."737475623031"]
token = "bench-token"
```

```sh
# terminal 2 — from remote/
cargo run -p remote-daemon -- --config bench.toml --no-open
```

The stub advertises its Link port in the Beacon, so the daemon discovers it on UDP 9001. The
Session still opens only on the first authenticated app; point a headless WebSocket client at
`ws://127.0.0.1:8765/ws` (send `hello`, then a 1 Hz `heartbeat`) or open the printed URL. A
manual `connect` message covers a network that blocks broadcast.

## Tests

```sh
cargo test -p remote-daemon
cargo test -p remote-daemon --test host_chain   # discovery → preamble → HELLO → /ws
cargo test -p remote-daemon --test link_stub    # daemon link ↔ Bridge Stub
```

The integration suite covers the relay (`relay_*`), the token store (`config_store`), the
Beacon listener (`discovery_beacon`), the preamble adapter (`handshake`) and the app adapter
(`app_auth`, `app_commands`, `app_hooks`, `app_origin`, `app_slow`, `app_static`, `app_ws`).

## Validation

CI's `remote` job runs from `remote/`:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build --workspace
```

## Known limits

- `remote/app/` currently holds only its README, so `/` answers 404 until the SPA lands; the
  daemon's WebSocket, discovery, auth and relay paths are complete and tested.
- v1 talks to one robot; several discovered Beacons pick one deterministically and name it in
  the app's `detail`.
- Timers (2 s heartbeat, 3 s stall, 5 s grace, 8 s backoff) are provisional until a real LAN
  and browser exercise them; there is no HIL rig.
- TLS and the internet are out of scope (ADR-0037); the LAN is plaintext plus the pairing
  token.
- The Bridge enforces one authenticated client, so several browser tabs share one Session.
