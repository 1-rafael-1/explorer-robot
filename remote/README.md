# remote — the PC side

The host workspace for the remote-control effort: the `remote-daemon` relay, the shared
`bridge-net` network edge, two bench stubs and the static app. It consumes the robot's
`link-protocol` and `link-core` crates by path; the Bridge firmware
(`../bridge/`) consumes the same `link-protocol` and `bridge-net`.

The operator page is [docs/components/remote.md](../docs/components/remote.md); the repo map and
data flow are in [docs/architecture.md](../docs/architecture.md).

| crate / folder | what it is | README |
|---|---|---|
| `bridge-net/` | `no_std` Beacon, `EXPH` preamble and fixed ports, shared by the Bridge and the daemon (ADR-0037, ADR-0040) | [bridge-net](bridge-net/README.md) |
| `daemon/` | the `remote-daemon` binary: raw TCP to the Bridge, `/ws` JSON and the `/` static app (ADR-0039) | [daemon](daemon/README.md) |
| `robot-stub/` | the robot's bench persona: the real `LinkCore` behind a serial port | [robot-stub](robot-stub/README.md) |
| `bridge-stub/` | the daemon's bench peer: preamble server, Beacon and scripted robot traffic | [bridge-stub](bridge-stub/README.md) |
| `app/` | the Remote App: static ES modules and `<canvas>` served at `/`; no build step | [app](app/README.md) |

## The current-directory rule

Cargo reads `.cargo/config.toml` from the current directory **upward**, and the repository root
sets the robot's `thumbv8m.main-none-eabihf` default. Run host-workspace commands from this
directory (or below); `cargo build --manifest-path remote/Cargo.toml` from the root is not
equivalent and targets the robot. See ADR-0040.

## The Remote App

`app/` is the static **Remote App** the daemon serves at `/`: vanilla ES modules
and one `<canvas>`, no bundler or build step (ADR-0038, ADR-0043). It connects to
the Relay's `/ws`, renders the App Snapshot and live Streams/Events in the World
Frame, drives by point-and-go, and shows Bridge and link status. Serve an edit
without rebuilding the daemon with the `--assets` override:

```sh
cargo run -p remote-daemon -- --no-open --assets app
```

The headless browser suite lives under `app/tests/` and runs from the repository
root with no Node or npm:

```sh
.github/scripts/run_web_tests.sh     # app boot smoke + every static test page
.github/scripts/run_real_chain.sh    # real Relay + Bridge Stub, plus the `/` boot check
```

The real-chain script builds `remote-daemon` and `bridge-stub`, starts a
`bridge-stub` with scripted Streams and `obstacle`/`floor_drop`/
`procedure_completed` Events, writes a temporary config with the stub's
`bench-token`, and runs the daemon with `--assets app --serve-tests --port 0`. The real page
checks reload reconstruction, including the Result Report rebuilt from the
replayed `procedure_completed`. See the [app README](app/README.md) for the
module map, the test pages and the manual matrix.

## Common commands

```sh
cargo run -p remote-daemon            # serve the app; see daemon/README.md for the flags
cargo run -p robot-stub -- --port /dev/ttyUSB0
cargo run -p bridge-stub -- --help
cargo test --workspace
cargo clippy --workspace --all-targets -- -D warnings
cargo build --workspace
```

On Linux, `robot-stub` links `libudev` through `serialport`, so the build needs `libudev-dev`
and `pkg-config` (the `remote` CI job installs them). The two shared contract crates are
root-workspace members; test them from the repository root:

```sh
cargo test -p link-protocol --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p link-core --features std --target x86_64-unknown-linux-gnu --tests
```

## Validation

CI's `remote` job runs, from this directory:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo build --workspace
```

The end-to-end chain worth naming is the daemon's `host_chain` test: discovery selects a
Bridge Stub's Beacon, the HMAC preamble succeeds, and a headless WebSocket client receives
`connecting`, `connected`, a snapshot and streams. The browser level of the same chain is
`run_real_chain.sh`, which renders `app/tests/real.html` against the real Relay and Bridge
Stub. The stubs are bench tools, not the robot or the Bridge firmware.
