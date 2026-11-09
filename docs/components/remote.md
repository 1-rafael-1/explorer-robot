# Remote

The PC-side client: the `remote-daemon` binary hosts the **Relay** and serves
the static web app, with the two bench stubs and the shared `bridge-net`
network edge in the same workspace. The daemon owns the **Session** and the
protocol; the app draws pixels. Nothing here is built for the robot's target.
There is no installer or release artifact: the operator path is a clone plus
the commands below.

**Verified:** 2026-11-09 — from `remote/`: `cargo test --workspace` (every
suite `ok`, 0 failed), `cargo build --workspace` (`Finished` the dev profile,
exit 0), and `cargo run -p remote-daemon -- --help` (usage printed, exit 0).
Steps that need a real robot or Bridge, a **Pairing Token**, serial adapters or
provisioning are **bench-required** and were not verified here.

## Prerequisites

- Rust stable **1.99.1** (the repository-root pin, which this workspace inherits) with `cargo`.
- On Linux, the `libudev` and `pkg-config` development packages, because
  `robot-stub` depends on `serialport`, which links `libudev`. CI installs the
  Debian/Ubuntu packages with:

  ```sh
  sudo apt-get update && sudo apt-get install -y libudev-dev pkg-config
  ```

- On macOS or Windows, `libudev` is not needed, but the workspace config pins
  the Linux host triple. Set `CARGO_BUILD_TARGET` to the host triple before
  building; see [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md).

## The current-directory rule

Cargo merges `.cargo/config.toml` from the **current directory upward**. The
repository root sets the robot's `thumbv8m.main-none-eabihf` default, and the
host workspace's [`.cargo/config.toml`](../../remote/.cargo/config.toml)
overrides it with `x86_64-unknown-linux-gnu`. The override applies only when
the current directory is `remote/` or below.

Enter the workspace before running any host command:

```sh
cd remote
```

The wrong way is `cargo test --workspace --manifest-path remote/Cargo.toml`
from the repository root. Cargo discovers configuration from the working
directory, not from the manifest path, so that form still inherits the robot's
`thumbv8m.main-none-eabihf` target and fails to compile the host code (for
example `std` and `tokio`). Enter the workspace instead; see
[ADR-0040](../../docs/adr/0040-repo-and-build-layout.md).

## Test, build and run

Each command below assumes the repository root as the starting directory and
enters the `remote/` host workspace (`remote/Cargo.toml`). The workspace
members are `bridge-net`, `bridge-stub`, `remote-daemon` (the `daemon/`
package) and `robot-stub`; `app/` is a static folder, not a Cargo package. All
three commands build for `x86_64-unknown-linux-gnu`, the target set by
`remote/.cargo/config.toml`.

The shared `link-protocol` and `link-core` crates are root-workspace
members and are tested by the robot workspace's host tests, not by the command
below; the [remote README](../../remote/README.md) has those commands.

### Test the workspace

```sh
cd remote && cargo test --workspace
```

This compiles and runs every test in all four members: `bridge-net`'s Beacon,
preamble and version suites; `bridge-stub`'s Beacon, preamble, deny, traffic and
binary suites; `remote-daemon`'s unit tests and its app, Relay, token-store,
discovery, handshake and link suites; and `robot-stub`'s persona, fault-knob and
loopback suites. The first run compiles the whole workspace and takes several
minutes.

Expected output: one `Running …` block per test binary, each ending in
`test result: ok.` with `0 failed`, then the four `Doc-tests` blocks, and exit
0. The end-to-end suites to look for are `host_chain` (discovery → preamble →
`HELLO` → `/ws`) and `link_stub` (daemon link ↔ Bridge Stub); `config_store`
covers the token store. Timings vary by machine:

```text
     Running tests/host_chain.rs (target/x86_64-unknown-linux-gnu/debug/deps/host_chain-...)
test result: ok. 8 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 3.01s
...
   Doc-tests robot_stub
test result: ok. 0 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

A non-zero exit, a `FAILED` line, or a compile error naming `thumbv8m` means the
command did not run from `remote/`.

### Build the workspace

```sh
cd remote && cargo build --workspace
```

Expected output: the crates compile and the run ends with exit 0:

```text
   Compiling remote-daemon v0.1.0 (remote/daemon)
    Finished `dev` profile [unoptimized + debuginfo] target(s) in ...
```

The host artifacts land under `remote/target/x86_64-unknown-linux-gnu/debug/`.
Nothing is produced for the robot, the Bridge or the Release profile.

### Run the daemon

Print the flags without starting the long-running process:

```sh
cd remote && cargo run -p remote-daemon -- --help
```

Expected output (exit 0):

```text
The remote explorer daemon: one JSON WebSocket for the app, raw TCP to the Bridge.

Usage: remote-daemon [OPTIONS]

Options:
      --host <HOST>    Serve the app on this address instead of loopback [default: 127.0.0.1]
      --port <PORT>    Serve the app on this port; the default 8765 falls back to an ephemeral port when taken
      --no-open        Do not open the default browser
      --assets <PATH>  Serve the SPA from this folder instead of the embedded copy
      --config <PATH>  Use this config file instead of the per-user default
      --verbose        Trace discovery and Session changes to stderr
  -h, --help           Print help
  -V, --version        Print version
```

To serve the app, start the daemon and stop it with `Ctrl-C`:

```sh
cd remote && cargo run -p remote-daemon -- --no-open
```

It prints the app URL and serves at `http://127.0.0.1:8765/` (or an ephemeral
port if 8765 is taken). `--no-open` suppresses the browser launch. This
long-running form was not started during verification; `--help` was used
instead.

## Daemon flags

| flag | meaning |
|---|---|
| `--host <HOST>` | app bind address; default `127.0.0.1` (loopback only) |
| `--port <PORT>` | app port; default 8765, with an ephemeral fallback when taken |
| `--no-open` | do not open the default browser |
| `--assets <PATH>` | serve the SPA from disk instead of the embedded copy |
| `--config <PATH>` | use this `config.toml` instead of the per-user default |
| `--verbose` | trace discovery and Session changes to stderr |

There is deliberately **no token flag**: a pairing secret is never accepted on
the command line, so it cannot land in shell history, process listings or
screenshots. The config store is the only token source. The
[daemon README](../../remote/daemon/README.md) documents the network defaults,
the app messages and the headless Bridge Stub recipe.

## App serving and `--assets`

`GET /` serves the static SPA and `GET /ws` is the Relay's one versioned JSON
WebSocket boundary (ADR-0038, ADR-0039). The daemon embeds `remote/app/` at
compile time. During development, `--assets <path>` serves a folder from disk
instead, so an edit does not need a rebuild:

```sh
cd remote && cargo run -p remote-daemon -- --no-open --assets app
```

`app/` is currently a placeholder containing only its README, so `GET /`
answers 404 until the SPA lands. Discovery, auth, the Relay and `/ws` are
complete and tested. See the [app README](../../remote/app/README.md).

## What the daemon stores

- **Configuration directory.** `explorer-remote/config.toml` lives in the
  platform's per-user configuration directory. On Linux that is
  `$XDG_CONFIG_HOME/explorer-remote/config.toml`, or
  `~/.config/explorer-remote/config.toml` when `XDG_CONFIG_HOME` is unset. It
  is created mode `0600` on Unix and written atomically; `--config <PATH>`
  points at a different file.
- **Token store.** The file is the only token source. `default_token` is used
  for a manual `host:port` endpoint; `devices` maps a lower-case hex device id
  to `{ token, name }`; `last_endpoint` and `last_device` record the last
  choices. A discovered device uses only its own token, and an unpaired one is
  reported as `pairing_required`.
- **Pairing.** A **Pairing Token** is one 60-bit secret per robot, shown as
  twelve base32 characters (no `I`, `L`, `O` or `U`) in three groups of four on
  the robot panel's `Link` screen. The operator types it into the app once; the
  app's `pair` message stores it under the last device. The Bridge and the
  daemon prove knowledge of it with a mutual HMAC-SHA256 challenge-response
  before the first Link Protocol frame, so the token itself is never sent
  (ADR-0036, ADR-0037). Rotation is local from the panel and needs no reflash.

The Relay's stream and event retention is in memory only: a daemon restart is
honestly a fresh `snapshot`. Only the token, the last endpoint and the last
device id are persisted; nothing is recorded for replay.

## Bench-required steps

These need hardware and were not verified on this machine:

- discovering a real Bridge's Discovery Beacon and opening a Session to the
  real robot;
- reading the panel's Pairing Token, typing it into the app, and rotating it;
- running `robot-stub --port /dev/ttyUSB0` (or `/dev/ttyACM*`) against a real
  Bridge;
- running `bridge-stub` as the daemon's peer;
- Wi-Fi provisioning and Setup Mode.

The stubs replace hardware but are still bench tools: see the
[robot-stub README](../../remote/robot-stub/README.md) and
[bridge-stub README](../../remote/bridge-stub/README.md), and the
[daemon README](../../remote/daemon/README.md#running-against-the-bridge-stub-headlessly)
for the headless daemon + Bridge Stub recipe. The [bench page](bench.md) owns
the harness, power and wiring.

## Common failures

| symptom | cause and fix |
|---|---|
| compile error naming `thumbv8m.main-none-eabihf`, or `std`/`tokio` missing | the command ran outside `remote/`; `cd remote` first |
| `libudev` or `pkg-config` not found while building `robot-stub` | install `libudev-dev` and `pkg-config` (Linux) |
| `GET /` answers 404 | expected while `app/` is a placeholder; the `/ws` Relay works |
| `pairing_required` or `unauthorized` | the device has no stored token; pair from the panel (bench-required) |
| port 8765 in use | the daemon falls back to an ephemeral port; use the printed URL or `--port` |
| no robots discovered | broadcast is blocked, or the Bridge is in AP-only Setup Mode; use a manual `host:port` (bench-required) |

## Source material

- [remote README](../../remote/README.md) — the current-directory rule and the
  workspace commands.
- [remote-daemon README](../../remote/daemon/README.md) — the daemon flags, the
  config store and the network defaults.
- [app README](../../remote/app/README.md) — the static SPA and the `--assets`
  development override.
- [bridge-net README](../../remote/bridge-net/README.md) — the shared Beacon,
  preamble and ports.
- [robot-stub README](../../remote/robot-stub/README.md) and
  [bridge-stub README](../../remote/bridge-stub/README.md) — the bench personas.
- [link-protocol README](../../robot/crates/link/link-protocol/README.md) — the wire contract;
  [link-core README](../../robot/crates/link/link-core/README.md) — the core the Robot Stub
  runs.
- [CONTEXT.md](../../CONTEXT.md) — the glossary's Remote, Relay, Session and
  Pairing Token entries.

Owning decisions:

- [ADR-0036](../../docs/adr/0036-wifi-provisioning.md) — Wi-Fi provisioning, the
  credential record and the Pairing Token slot.
- [ADR-0037](../../docs/adr/0037-reachability-and-trust.md) — reachability and
  trust.
- [ADR-0038](../../docs/adr/0038-remote-app-boundary-and-rendering.md) — the app
  boundary.
- [ADR-0039](../../docs/adr/0039-remote-daemon-relay.md) — the daemon relay.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the
  current-directory rule and the three workspaces.
