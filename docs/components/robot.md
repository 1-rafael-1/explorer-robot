# Robot

The robot firmware: the `explorer-robot` package on the RP2350B, covering the
drive subsystem, LiDAR perception, the panel UI and the robot side of the Link
Protocol (the `LinkCore` in [link-core](../../robot/crates/link/link-core/README.md)).

The firmware package's manifest is `robot/Cargo.toml`; the root `Cargo.toml` is
a virtual workspace manifest that lists `robot` and its crates as members.

Run every command on this page from the **repository root**. Cargo discovers
`.cargo/config.toml` from the current directory upward, and the root file sets
both the `thumbv8m.main-none-eabihf` default target and the `probe-rs` runner.
Do not run a robot build from `remote/`, and do not use `--manifest-path` to
reach the robot workspace from another directory: neither changes the config
Cargo reads, so the host workspace's target would apply instead. The three
workspaces and their build targets are described in
[ADR-0040](../../docs/adr/0040-repo-and-build-layout.md).

**Not packaged.** There is no published robot binary, release artifact, OTA
update or installer. The operator path is a clone, the pinned toolchain and
`probe-rs`; the firmware is built from source and flashed over SWD.

## Prerequisites

- **Rust 1.99.1.** [`rust-toolchain.toml`](../../rust-toolchain.toml) pins one
  channel for the whole repository and requests the `rustfmt`, `clippy` and
  `rust-src` components plus the `thumbv8m.main-none-eabihf` and
  `riscv32imc-unknown-none-elf` targets. `rustup` selects and installs it
  automatically inside the repository; confirm with `rustup show
  active-toolchain`. If a target is ever missing, add it explicitly.
- **Embassy master.** The robot follows embassy's own advice to use git when a
  needed change has no release: the root manifest's `[patch.crates-io]` pins
  `embassy-rp`, `embassy-embedded-hal`, `embassy-sync`, `embassy-executor`,
  `embassy-time` and `embassy-futures` to one commit (`9b0680bd`).
  Update every patched crate together and let the lockfile record the revision.
- **Nightly rustfmt.** The formatting gate deliberately invokes nightly
  (`cargo +nightly fmt`), not the pinned stable toolchain; install a nightly
  toolchain with the `rustfmt` component.
- **`probe-rs`.** Installed separately and **not pinned** by this repository.
  Install a recent release (for example `cargo install probe-rs-tools
  --locked`) and check it knows the chip with `probe-rs --version`. A version
  that cannot select `RP235x` cannot flash the robot.
- **`memory.x`.** Lives at the package root and is part of the firmware, not a
  generated file. [`build.rs`](../../robot/build.rs) reads it with
  `include_bytes!("memory.x")` — a path relative to `build.rs`, not to the
  shell's current directory — copies it into `OUT_DIR`, adds `OUT_DIR` to the
  linker search path, and passes `-Tlink.x -Tdefmt.x`. The copy matters in a
  workspace: the linker does not search the package directory, so the
  package-relative copy is what makes the memory map findable. The
  `cargo:rerun-if-changed=memory.x` directive rebuilds when it changes. Do not
  move or rename it.

```sh
rustup show active-toolchain
rustup target add thumbv8m.main-none-eabihf   # only if the pinned toolchain lacks it
cargo +nightly fmt --version                   # nightly rustfmt for the formatting gate
probe-rs --version                             # separate install, not pinned here
```

## Build and test

These are the robot commands from the `robot` job in
[`.github/workflows/ci.yml`](../../.github/workflows/ci.yml); the workflow also
runs an `--all-features` clippy pass and builds the crate examples.

### Build

```sh
cargo build --workspace --release --target thumbv8m.main-none-eabihf
```

A cold cache compiles the whole workspace; a warm cache compiles only what
changed. Either way the last line is the invariant:

```text
   Compiling link-protocol v1.1.0 (...)
   Compiling link-core v0.1.0 (...)
    Finished `release` profile [optimized + debuginfo] target(s) in 0.61s
```

The firmware binary is
`target/thumbv8m.main-none-eabihf/release/explorer-robot`.

### Formatting

```sh
cargo +nightly fmt --all -- --check
```

Expected: no output and exit status 0. A formatting diff prints the affected
files and exits non-zero.

### Clippy

```sh
cargo clippy --workspace --target thumbv8m.main-none-eabihf --no-deps -- -D warnings
```

Expected: `Checking`/`Compiling` lines for the workspace crates, then:

```text
    Finished `dev` profile [unoptimized + debuginfo] target(s) in 2.15s
```

There are no warnings; `-D warnings` promotes any to an error.

### Host tests

The firmware is `no_std` and cannot run the built-in test harness, so the
testable logic lives in workspace crates behind their `std` features. Each
command forces the host target, because the workspace default is the bare-metal
one. Every suite must end with `test result: ok.` and `0 failed`.

```sh
cargo test -p coin-d6 --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p touch-async --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p lidar-cloud --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p touch-ui --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p link-protocol --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p link-core --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p claim-slot --features std --target x86_64-unknown-linux-gnu --tests
cargo test -p link-state --features std --target x86_64-unknown-linux-gnu --tests
```

Expected, once per test binary:

```text
running N tests
...
test result: ok. N passed; 0 failed; 0 ignored; 0 measured; 0 filtered out
```

## Flash and run

**Bench-required** — these steps need the board and a debug probe. The root
`.cargo/config.toml` selects `probe-rs` as the runner for bare-metal ARM
targets:

```toml
runner = "probe-rs run --chip RP235x --protocol swd --speed 16000"
```

So a plain `cargo run` builds for `thumbv8m.main-none-eabihf`, flashes, resets
and attaches RTT:

```sh
cargo run -p explorer-robot --release
```

Expected: the release build finishes, `probe-rs` reports the selected probe and
`RP235x` chip, and the firmware's `defmt` output appears over RTT. The runner
uses SWD at 16 MHz (`--speed 16000` kHz).

### RTT and `defmt` logging

The firmware logs through `defmt` over RTT (`defmt-rtt` is linked from
`src/main.rs`). The default filter is `DEFMT_LOG=info` from
`.cargo/config.toml`; override it in the environment:

```sh
DEFMT_LOG=debug cargo run -p explorer-robot --release
```

Because the log stream is RTT, `probe-rs` must stay attached; closing it stops
the output. **Bench-required.**

Some bring-up diagnostics are compiled only with the `telemetry_logs` feature:
the boot reset cause, the LiDAR power/UART-up lines, and the room-scan intent.
Use `--features telemetry_logs` when bench-testing those:

```sh
cargo run -p explorer-robot --release --features telemetry_logs
```

### Recovery

If the application is stale but the board still runs, re-running `cargo run
-p explorer-robot --release` is enough. If `probe-rs` reports a locked or invalid image, or the
firmware will not start, erase the chip and reflash:

```sh
probe-rs erase --chip RP235x
cargo run -p explorer-robot --release
```

The erase mass-erases the RP2350 flash, including the `FLASH_STORAGE`
calibration region at the top of flash (see [`memory.x`](../../robot/memory.x)), so
calibration data is lost. Both steps are **bench-required**.

## LiDAR power and harness

The LiDAR is on switched 5 V (`LIDAR_PWR`, GPIO30) through `J7`. Set a bench
supply to at least 500 mA: the measured baseline is about 320 mA with the LiDAR
spinning and the remote link up, and the motors, rangefinders, RGB LED and AI
camera off. A weaker link can raise it slightly.

`J7.1` is switched 5 V, `J7.2` GND, `J7.3` the sensor RX / MCU TX, and `J7.4`
the sensor TX / MCU RX. A swapped RX/TX pair leaves the branch powered but
produces no scans.

The device-info probe is best-effort: a missed frame is logged and tolerated,
and warm-up is the proof that data flows. The power/probe and room-scan intent
logs are behind `telemetry_logs` (see above).

The LiDAR task's release frame is about 29 KiB, so the 64 KiB core1 stack and
the dev package `opt-level = 1` override are load-bearing. Do not reduce either
without re-measuring.

## Common failures

| Symptom | Cause and fix |
|---|---|
| `can't find crate` or a missing-target error for `thumbv8m...` | The target is not installed: `rustup target add thumbv8m.main-none-eabihf`. |
| Host tests try to build for `thumbv8m...` | `--target x86_64-unknown-linux-gnu` was omitted, or the command ran from `remote/`. |
| `probe-rs` finds no probe or cannot attach | Check the SWD wiring and that the probe is connected; this step is bench-required. |
| Linker cannot find `memory.x` or a memory region | `memory.x` was moved or removed from the package root; `build.rs` embeds it from there. |
| `cargo +nightly fmt` cannot find the toolchain | Install a nightly toolchain with `rustfmt`; the pin only supplies stable 1.99.1. |
| Host test crate cannot resolve `std` | The `--features std` flag was omitted; the feature is host-only. |
| LiDAR never spins in Room Scan; caption may show `LiDAR failed` | Check the J7 RX/TX pair (`J7.3` = sensor RX/MCU TX, `J7.4` = sensor TX/MCU RX) and set the bench supply to at least 500 mA; a 300 mA limit prevents spin-up. Use `--features telemetry_logs` for the probe/warm-up lines. |
| No `[lidar]` log and no power gate | Core1 may have faulted on an undersized stack; keep the 64 KiB `CORE1_STACK` and the dev package `opt-level = 1`. |

## Source material

- [Root README](../../README.md) — the front door and the hardware summary.
- [Robot package README](../../robot/README.md) — the package and its crate buckets.
- [link-protocol](../../robot/crates/link/link-protocol/README.md) — the Link
  Protocol wire contract.
- [link-core](../../robot/crates/link/link-core/README.md) — `LinkCore`, the
  robot side of the protocol.
- [coin-d6](../../robot/crates/drivers/coin-d6/README.md) — the LiDAR driver and
  its wiring.
- [st7789-async](../../robot/crates/drivers/st7789-async/README.md) — the display
  driver.
- [hardware-tests](../../robot/crates/hardware-tests/README.md) — standalone sensor
  checks that run beside the firmware.
- [CONTEXT.md](../../CONTEXT.md) — the domain glossary.

Owning decisions:

- [ADR-0007](../../docs/adr/0007-odometry-architecture.md) — odometry
  architecture.
- [ADR-0010](../../docs/adr/0010-lidar-on-demand-power.md) — the LiDAR's on-demand
  power lifecycle.
- [ADR-0030](../../docs/adr/0030-link-protocol-envelope.md) — the Link Protocol
  envelope.
- [ADR-0031](../../docs/adr/0031-link-protocol-payloads.md) — the Link Protocol
  payloads.
- [ADR-0032](../../docs/adr/0032-remote-link-task.md) — the robot's Link task.
- [ADR-0033](../../docs/adr/0033-uart1-baud-and-ring-budget.md) — the UART baud
  and ring budget.
- [ADR-0034](../../docs/adr/0034-one-active-claim.md) — one Active Claim.
- [ADR-0035](../../docs/adr/0035-failsafe-deadman.md) — the Deadman.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the three
  workspaces.
