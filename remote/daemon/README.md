# remote-daemon

The host relay (ADR-0039). One binary connects to the Bridge over raw TCP with the ADR-0037
HMAC preamble, owns the robot Session, and serves the static app at `/` and one JSON WebSocket
at `/ws` (ADR-0038). It is a pure relay in the app-facing sense: it decodes the wire, flips the
sign of angles, retains latest-wins state and never exposes the wire or its own process to the
app. Its shared dependencies are `link-protocol` (wire) and `bridge-net` (preamble, Beacon,
ports); ADR-0040 records the layout. It is a member of the `remote/` host workspace; the operator
page is [docs/components/remote.md](../../docs/components/remote.md) and the repo map is
[docs/architecture.md](../../docs/architecture.md).

## Installing for the current user

Install once from the `remote/` workspace (the current-directory rule, ADR-0040):

```sh
cd remote
cargo install --path daemon --locked
```

That builds the `remote-daemon` package and puts `remote-daemon` in
`~/.cargo/bin`, which is on the user's `PATH`, so it starts from any directory.
The install builds only the daemon and its dependencies (`bridge-net`,
`link-protocol` and the crates they use), not the bench stubs: the
`libudev`/`pkg-config` prerequisite comes from `robot-stub`'s `serialport`
dependency and does **not** apply to this install. The SPA is embedded at
compile time, so the installed binary is self-contained; `--assets` remains a
development override.

`cargo install` is per-machine and source-based. It needs this clone and a Rust
toolchain, and it is a convenience for the operator on this machine, not a way
to distribute the daemon to someone else. There is no release artifact or
package.

## Running from a terminal

Once installed, start it from any directory:

```sh
remote-daemon
```

It binds the app on loopback, prints the URL, opens the default browser at it,
and serves until `Ctrl-C`, which shuts it down cleanly and closes the app's
WebSockets with code **1001**. Trace and error output goes to stderr. To suppress
the browser launch, or to pick a port:

```sh
remote-daemon --no-open
remote-daemon --no-open --port 9000
```

The printed URL is the address actually bound: `--port` is honoured as given and
the default **8765** falls back to an ephemeral port when it is taken. Pairing is
unchanged — there is deliberately no token flag, and the token is entered in the
app and kept in the config store described below. The process opens no robot
Session until an app connects, so leaving it running is cheap.

## Running as a user service

The installed binary can also run as a systemd **user** unit, started and
stopped by hand with `systemctl --user`. It is **not enabled at login**: nothing
starts it until you do. This is the path that survives closing a terminal, keeps
a log in the journal and restarts the process after a crash.

The unit is checked in beside the daemon
([`remote-daemon.service`](remote-daemon.service)) so its flags cannot drift
from the CLI. From the repository root, install it once for the current user by
copying it into the user unit directory — `$XDG_CONFIG_HOME/systemd/user`, or
`~/.config/systemd/user` when `XDG_CONFIG_HOME` is unset — and reloading the
manager:

```sh
mkdir -p ~/.config/systemd/user
cp remote/daemon/remote-daemon.service ~/.config/systemd/user/
systemctl --user daemon-reload
```

Start, inspect and stop it:

```sh
systemctl --user start remote-daemon
systemctl --user status remote-daemon
systemctl --user stop remote-daemon
```

While it runs, the app is at **http://127.0.0.1:8765/**, which is stable across
restarts and worth bookmarking. The choices that matter:

- `ExecStart=%h/.cargo/bin/remote-daemon --no-open --host 127.0.0.1 --port 8765`
  names the installed binary by an absolute, home-relative path, because the user
  manager does not inherit the shell's `PATH`.
- `--host 127.0.0.1` keeps the loopback-only default of a terminal run.
- `--port 8765` is fixed so the bookmark never moves. The CLI's ephemeral
  fallback applies only when the port is omitted, so a port already in use fails
  the unit visibly instead of moving the app.
- `--no-open` suppresses the browser, because a unit has no browser session; the
  bookmark replaces it.
- `Type=simple` because the daemon stays in the foreground and holds the
  process.
- `Restart=on-failure` with `RestartSec=5` retries a crash after five seconds;
  systemd's start rate limiting turns a persistent failure (a taken port, say)
  into a failed unit rather than a restart storm.

There is no `After=network-online.target` and no `WorkingDirectory`: the user
manager has no such target, and the daemon binds its sockets and retries
discovery while the network settles. The SPA is embedded, so the working
directory does not matter.

Logs — trace and error output written to stderr reach the user journal:

```sh
journalctl --user -u remote-daemon -f
```

Add `--verbose` to `ExecStart` in a drop-in if you want discovery and Session
changes traced. To enable the unit at login instead of starting it by hand, run
`systemctl --user enable remote-daemon`; that is optional and outside the
documented path.

## Running from a source checkout

For development, run it through Cargo from `remote/` (the current-directory rule,
ADR-0040):

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
| `--serve-tests` | serve the browser bench suite under `tests/` (never a production surface) |
| `--config <PATH>` | use this `config.toml` instead of the per-user default |
| `--verbose` | trace discovery and Session changes to stderr |

There is deliberately **no token flag**: a pairing secret is never accepted on the command
line, so it cannot land in shell history, process listings or screenshots. The config store is
the only token source.

## The config store

`config.toml` lives in the platform's per-user config directory
(`explorer-remote/config.toml`; on Linux `$XDG_CONFIG_HOME/explorer-remote/config.toml`, or
`~/.config/explorer-remote/config.toml` when `XDG_CONFIG_HOME` is unset). It is created mode
`0600` on Unix and written atomically. Fields:

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

The store is the only token source. Pairing still happens in the app: there is
**no token flag**, and no token is passed on the command line. The operator types
the robot's Pairing Token into the app's connection screen; the app's `pair`
message stores it under the last device in this file, and the daemon reads it
from here.

## Upgrading

Re-run the install from `remote/` (the same command as the first install):

```sh
cd remote
cargo install --path daemon --locked
```

Cargo replaces the installed binary. A process that is already running keeps the
old binary until it is restarted, so restart it — `Ctrl-C` and start
`remote-daemon` again from the terminal, or, under the user unit:

```sh
systemctl --user restart remote-daemon
```

The config store above is not touched, so the pairing token and last endpoint
survive.

## Uninstalling

If you installed the user unit, stop it and remove it first:

```sh
systemctl --user stop remote-daemon
rm ~/.config/systemd/user/remote-daemon.service
systemctl --user daemon-reload
```

Then remove the binary:

```sh
cargo uninstall remote-daemon
```

That removes `remote-daemon` from `~/.cargo/bin`. The per-user config store and
the pairing token are deliberately left in place; delete
`explorer-remote/config.toml` from the platform's config directory yourself if
you want to forget them.

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
JSON WebSocket; its path never carries a version — `hello.protocol` does. During app
development, `cargo run -p remote-daemon -- --no-open --assets app` serves the folder from
disk so a module or stylesheet edit needs no rebuild. The browser suite under `tests/`
contains a page that drives on load, so it is served only with `--serve-tests`. Server messages are
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

The app's browser suite is separate and runs from the repository root:

```sh
.github/scripts/run_web_tests.sh     # app boot smoke + every static test page
.github/scripts/run_real_chain.sh    # real Relay + Bridge Stub, plus the `/` boot check
```

`run_real_chain.sh` builds this daemon and `bridge-stub`, writes a temporary config with
the stub's `bench-token`, and serves `app` with `--assets app --serve-tests --port 0`. The page reports
through the DOM; the runner polls its `data-result`/`data-boot` attribute with a
standard-library DevTools-pipe client because `chrome --dump-dom` dumps at load and the
page finishes over a real WebSocket.

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

- The Remote App in `remote/app/` is served at `/` from the embedded copy (or `--assets`);
  the browser suite is a bench asset under `--serve-tests`, and its pure/fake/real pages run in
  CI's `web` job. The manual Firefox, LAN and real-phone steps are listed in the
  [Remote page](../../docs/components/remote.md).
- v1 talks to one robot; several discovered Beacons pick one deterministically and name it in
  the app's `detail`.
- Timers (2 s heartbeat, 3 s stall, 5 s grace, 8 s backoff) are provisional until a real LAN
  and browser exercise them; there is no HIL rig.
- TLS and the internet are out of scope (ADR-0037); the LAN is plaintext plus the pairing
  token.
- The Bridge enforces one authenticated client, so several browser tabs share one Session.
