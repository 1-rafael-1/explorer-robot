# Three workspaces: the robot, the Bridge and the host remote

The Link Protocol (ADR-0030, ADR-0031), the robot task (ADR-0032), the app boundary (ADR-0038) and
the daemon (ADR-0039) name crates, files and ports but never say where those live or how they
build. The three consumers have three incompatible build environments: the robot is
`thumbv8m.main-none-eabihf` `no_std`, the Bridge is `riscv32imc-unknown-none-elf` `no_std` with
`build-std`, and the daemon is a host `std` binary. Cargo allows a package in exactly one workspace
and a default target per workspace, so the layout is a matter of which workspace owns what and how
the host binary escapes the robot's default target.

The Bridge already answered half the question by being a separate workspace (ticket 09). This
decision settles the rest: the two shared `no_std` crates, the host workspace, the network-edge
crate, the static app, and the three CI jobs.

**Status:** accepted. There are **three cargo workspaces**: the robot root workspace, `esp-bridge/`
and a new host workspace at `remote/`. `remote-protocol` and `remote-link` are members of the root
workspace; `remote-daemon` and a new `bridge-net` crate are members of `remote/`; `remote/app/` is a
static folder inside the host workspace. Shared crates are consumed by **path** only, every path
dependency carries a `version`, and the protocol crate's version mirrors the wire version. CI runs
three independent jobs. The crates and the static folder are placed; their contents are skeletons,
and the protocol, daemon, Bridge and app implementations remain separate efforts.

**Considered Options**

- **One workspace for everything.** Rejected. The daemon's `tokio`/`axum` graph cannot build for
  `thumbv8m`, and the Bridge needs `build-std` and a RISC-V target. Root `cargo check --workspace`
  would either fail or need per-crate `--target` on every command, and a single lockfile would mix
  three toolchains' dependency sets.
- **The daemon as a root workspace member with `--target` aliases.** Rejected. A host package in
  the root workspace is still compiled by `cargo check --workspace` and by clippy on the default
  target, so aliases would protect only commands that remember to use them.
- **`remote-protocol` as a standalone workspace.** Rejected. A crate can be a member of one
  workspace, and a third lockfile plus a `cd` in CI buys neutrality nobody needs. Ticket 01 already
  places it in the root workspace, and the robot is its most constrained consumer.
- **`remote-protocol` under `remote/`.** Rejected. The robot and the Bridge would depend into the
  host effort. The contract belongs with the robot; the host side consumes it by path.
- **A root alias that runs the daemon with `--manifest-path remote/Cargo.toml`.** Rejected. Cargo
  config discovery follows the **current directory**, not the manifest path: from the repo root the
  root `.cargo/config.toml` still supplies the `thumbv8m` target and the daemon fails to compile
  for want of `std` (verified in a throwaway spike). The host workspace must be entered.
- **A constants-only module for the Beacon and preamble.** Rejected. The preamble is
  security-sensitive byte handling plus two HMAC computations that the Bridge and the daemon must
  agree on exactly; a second hand-written copy is how the two halves drift.
- **`bridge-net` inside `esp-bridge/`.** Rejected. The daemon would then path-depend into the
  RISC-V workspace, whose config carries `build-std` and the espflash runner.
- **An unrelated `0.1.0` version for `remote-protocol` with wire constants beside it.** Rejected.
  The crate is never published; its only useful version is the wire contract it implements. Two
  independent numbers invite exactly the drift this ticket exists to prevent.

**Decision**

*The shape.*

```
explorer-robot/
├── Cargo.toml              # robot workspace: + remote-protocol, remote-link; excludes the others
├── .cargo/config.toml      # thumbv8m.main-none-eabihf default, probe-rs runner
├── remote-protocol/        # root member; no_std + std feature; the three-way contract
├── remote-link/            # root member; no_std + std feature; robot-only LinkCore
├── esp-bridge/             # own workspace (unchanged); path-depends the shared crates
│   └── .cargo/config.toml  # riscv32imc-unknown-none-elf, build-std, espflash
└── remote/                 # host workspace
    ├── Cargo.toml          # [workspace] members = ["bridge-net", "daemon"]
    ├── .cargo/config.toml  # build target x86_64-unknown-linux-gnu
    ├── Cargo.lock
    ├── bridge-net/         # no_std; Beacon + preamble + ports; Bridge and daemon only
    ├── daemon/             # package remote-daemon (host binary)
    └── app/                # static SPA; not a cargo package, no build step
```

The root workspace gains the two members and one more exclusion:

```toml
[workspace]
members = [
    "st7789-async", "coin-d6", "hardware-tests", "touch-async", "lidar-cloud", "touch-ui",
    "remote-protocol", "remote-link",
]
exclude = ["esp-bridge", "remote"]
```

This is the structural gate the ticket asks for: the root `cargo check`/`clippy`/`build` never
loads the RISC-V or host-only packages, because they are not members and nothing path-depends on
them. If `tokio` ever entered the root graph, the thumb build would fail loudly rather than silently
change the robot's dependency set. Both new root members take `[lints] workspace = true` and the
`std` feature pattern `coin-d6` already uses; host tests are integration tests under `tests/`, run
with the explicit host target.

*`remote-protocol` is the robot's crate first.*

It stays a root member, as ticket 01 decided. The daemon and the Bridge consume it by path:

```toml
# esp-bridge/Cargo.toml
remote-protocol = { path = "../remote-protocol", version = "1" }
bridge-net = { path = "../remote/bridge-net", version = "1" }

# remote/daemon/Cargo.toml
remote-protocol = { path = "../../remote-protocol", version = "1" }
bridge-net = { path = "../bridge-net", version = "1" }
```

A crate can belong to one workspace, and cross-workspace path dependencies are supported: in the
spike, the Bridge workspace and the host workspace each resolved the *same* `remote-protocol 1.0.0`
manifest under the root. The `version` requirement is the build-level tripwire described below.

*`remote-link` is a root member beside it.*

ADR-0032's robot-side `LinkCore` crate is `no_std`, HAL-agnostic and host-tested, and the firmware
(`src/task/remote_link/`) is its only consumer. It lives at `remote-link/`, depends on
`remote-protocol` by path, and is not visible to the Bridge or the host workspace.

*The host workspace, and the current-directory rule.*

The PC side is one workspace at `remote/` with its own lockfile, target directory and config. Cargo
merges `.cargo/config.toml` from the current directory upward, so the root's
`build.target = "thumbv8m.main-none-eabihf"` is inherited and cannot be unset; the host workspace
overrides it explicitly:

```toml
# remote/.cargo/config.toml
[build]
target = "x86_64-unknown-linux-gnu"
```

The workspace's `Cargo.toml` also copies the root's `[workspace.lints]` table so the clippy gate is
the same in both places.

The override applies only when the current directory is `remote/` or below. That is why
`cargo run -p remote-daemon` is the documented form and why the CI job sets
`working-directory: remote`; `cargo build --manifest-path remote/Cargo.toml` from the repo root is
*not* equivalent and builds for the robot's target. The spike confirmed both halves: from
`remote/` (and from `remote/daemon/`) the rustc invocations carry
`--target x86_64-unknown-linux-gnu`, `cargo run -p remote-daemon` works, and from the repo root the
same manifest is built for `thumbv8m.main-none-eabihf`. The pinned triple matches `CONTEXT.md`'s
host-test precedent; a macOS or Windows contributor overrides with `CARGO_BUILD_TARGET`.

*`bridge-net`: the network edge, shared with the Bridge.*

ADR-0037 and ADR-0039 put discovery and authentication outside the Link Protocol, and ticket 12
requires they stay out of the robot's path. `remote/bridge-net/` is a `no_std` crate that only the
Bridge and the daemon depend on. It owns the whole Bridge↔daemon contract, not just its numbers:

| module | contents |
|---|---|
| `beacon` | magic `EXPR`, the 3 s interval, the flag bits (provisioned, robot link ready, `setup_mode`), `tcp_port = 0` as portal-only, and the datagram encode/decode |
| `preamble` | magic `EXPH`, the preamble major, the five message types, the deny reasons and payload codecs, the 5 s auth deadline, and `bridge_proof`/`client_proof` with the `explorer-link-v1 bridge\0` / `explorer-link-v1 client\0` HMAC-SHA256 labels |
| `ports` | `BEACON_UDP_PORT = 9001`, `LINK_TCP_PORT_DEFAULT = 9000` |

Its only dependencies are `hmac` and `sha2` (both `no_std`, no allocator) plus `remote-protocol` for
the Link major the Beacon advertises. Fixed-size arrays serve the ≤16-byte name and the 32-byte
nonces, so `heapless` is not needed. The HTTP port 8765 stays in `remote-daemon`: it is the app's
port, owned by the daemon and overridable on its CLI, not part of the Bridge contract.

The daemon's ADR-0039 `handshake` module is therefore a thin adapter over
`bridge-net::preamble`, not a second implementation. This satisfies ticket 12 without reopening it:
the robot never depends on `bridge-net`, and no Beacon, preamble or fixed port enters its path.
The HMAC/SHA-256 code-size cost on the C3 remains ADR-0039's open item; sharing the crate does not
change it.

*`remote/app/` is a folder, not a package.*

The SPA lives at `remote/app/` beside the daemon. It has no `Cargo.toml`, no `package.json`, no
bundler and no generated output. The daemon embeds it at compile time from
`$CARGO_MANIFEST_DIR/../app` (the `include_dir`-style embed is an implementation choice, the path is
the decision); `--assets <path>` serves from disk for development, as ADR-0039 requires. The
prototype stays untracked; ticket 14 places the destination, it does not populate it.

*One protocol version.*

`remote-protocol` starts at **1.0.0**, mirroring wire `major = 1`, `minor = 0`. A test asserts
`PROTOCOL_MAJOR`/`PROTOCOL_MINOR` equal `CARGO_PKG_VERSION_MAJOR`/`MINOR`, so the two cannot drift.
`bridge-net` follows the same rule against its preamble major. `remote-link` and `remote-daemon`
are not contracts and keep ordinary versions.

The bump policy follows the wire:

| change | crate version | wire |
|---|---|---|
| new id, new enum variant, additive optional field | minor | minor |
| payload layout, id removal/repurposing, header semantics | major | major |
| no wire change | patch | unchanged |

Build-level detection is threefold and mechanical:

1. Shared crates are **path dependencies only**, never a crates.io range. One source, one copy per
   build.
2. Every consumer requires `version = "1"`. A major bump fails resolution in `esp-bridge/` and
   `remote/` until each consumer is reviewed; a minor bump resolves silently, which is correct
   because additive changes are safe by ADR-0030's unknown-id rules. The spike confirmed the failure
   mode: bumping `remote-protocol` to `2.0.0` produced
   `failed to select a version for the requirement remote-protocol = "^1"` in both workspaces.
3. The constant/version test above keeps the manifest honest.

Runtime mismatch stays ADR-0030's and is not re-decided here: `major` in every frame, `minor` and
capabilities in `HELLO`/`HELLO_ACK`, and the Bridge checking only the preamble major. Ticket 14 owns
only the build-level side.

*CI: three jobs, not one.*

| job | working directory | gate |
|---|---|---|
| `robot` (existing, extended) | repo root | `cargo +nightly fmt --all -- --check`; clippy and build `--workspace --target thumbv8m.main-none-eabihf`; the existing per-crate host tests plus `cargo test -p remote-protocol --features std --target x86_64-unknown-linux-gnu --tests` and the same for `remote-link` |
| `bridge` (new) | `esp-bridge/` | `cargo +nightly fmt --all -- --check`; `cargo build`; `cargo clippy -- -D warnings` |
| `remote` (new) | `remote/` | `cargo +nightly fmt --all -- --check`; `cargo clippy --workspace --all-targets -- -D warnings`; `cargo test --workspace`; `cargo build --workspace` |
| app | — | no gate: no build step and no artifacts |

Three jobs, not one, so a Bridge toolchain failure does not mask a daemon failure, and so each gets
its own cache key from its own lockfile. Details that matter:

- The `bridge` job must **never** use `--all-targets`: a `no_std` binary cannot build the built-in
  test harness (`can't find crate for test`, missing `panic_handler`). It is `cargo build` plus
  clippy on the bin target, as ticket 14's progress note already recorded.
- The `bridge` job is also the **cross-target proof** for both shared crates: because `esp-bridge`
  path-depends on `remote-protocol` and `bridge-net`, building the Bridge compiles them for
  `riscv32imc-unknown-none-elf`. That is worth more than a separate test step.
- `bridge-net`'s host tests ride the `remote` job's `cargo test --workspace`.
- The `remote` job needs neither the RISC-V nor the thumb target; it is host-only.
- The `bridge` job installs the toolchain pinned by `esp-bridge/rust-toolchain.toml` and must
  reproduce whatever `build-std` needs locally. The Bridge's toolchain is ticket 09's, not this
  ADR's to change.
  **Superseded by the 2026-11-09 amendment:** `esp-bridge/` is now `bridge/`, its
  `rust-toolchain.toml` is deleted, and every CI job installs the root pin instead. The toolchain
  is no longer a separate tracker's to own.
- Nightly `rustfmt` is required in all three jobs because `rustfmt.toml` uses
  `group_imports`/`imports_granularity`, which are unstable options. Config discovery upward means
  the root `rustfmt.toml` also styles the other two workspaces.
- The app has no build step, so it has no CI gate; its correctness is the headless checks on the
  prototype (ticket 13) and manual browser use.

**Consequences**

- The robot's CI can never accidentally build the daemon or the Bridge; the `exclude` is the
  contract, and the two new root members are the only shared code it sees.
- Three lockfiles and three fmt/clippy invocations are the price of three targets. Each job caches
  independently, and none can mask another.
- `cargo run -p remote-daemon` works only from `remote/`. This is a cargo property, not a choice;
  the ADR, `CONTEXT.md` and the CI `working-directory` all say so, but a developer who uses
  `--manifest-path` from the root will meet the thumb target.
- `remote-protocol` is 1.0.0 from its first commit while the robot package is 0.1.0. The mismatch
  is intentional: the former is a wire contract, the latter a project version.
- The Bridge's lockfile now tracks the two shared crates, so a wire change is visible in the Bridge
  job's diff. That is the desired coupling.
- A source-only public-API refactor of `remote-protocol` bumps the patch, not the major. Nothing
  outside the repo consumes the API, and the wire is the contract.
- The host workspace's lint table duplicates the root's. There is no cross-workspace inheritance;
  the duplication is deliberate so one clippy standard covers both jobs.
- `bridge-net` is one codebase for both sides of the preamble, so a Bridge/daemon divergence is a
  compile-time non-event rather than a test finding.
- The app stays a static folder with no build step, now with a home that the daemon can embed.

**Open items**

- Write the code inside the four crates. The skeletons, the two workspace edits, the host
  workspace config, the static folder and the three CI jobs are in place.
- Pick the embed crate (`include_dir`, `rust-embed` or equivalent) when `remote-daemon` is
  implemented. The path is fixed; the tool is not.
- Reproduce `esp-bridge/`'s toolchain and `build-std` setup exactly in the `bridge` job (ticket 09's
  configuration), and confirm it on a clean runner.
- Run the `robot`, `bridge` and `remote` jobs on GitHub. They were verified locally (format,
  clippy, build, host tests and the cross-target Bridge compile) but not yet on a clean runner.
- Confirm the hardcoded `x86_64-unknown-linux-gnu` host pin is acceptable, or provide a documented
  `CARGO_BUILD_TARGET` override for macOS and Windows contributors.
- Define `remote-protocol`'s `sim` module and its feature name when the codec lands (ticket 18).
- Confirm the HMAC/SHA-256 code size on the C3 once `bridge-net::preamble` exists (ADR-0039's open
  item, now with a concrete home).

## Amendment — 2026-11-07: the Bridge workspace rename

The RISC-V workspace is renamed so its name says what it is (ticket 01 of the
`bridge-and-daemon` tracker; the naming decision is that tracker's spec, "Trackers, names and
homes"). Recorded here as an amendment because the layout above is otherwise unchanged.

- The directory `esp-bridge/` is renamed **`bridge-firmware/`** (history-preserving `git mv`;
  the untracked `target/` moved with it). The package is renamed **`esp-bridge` →
  `bridge-firmware`**, so the directory and the package agree. The `bridge` firmware binary
  joins this workspace in ticket 04.
- The ticket-09 spike binary is renamed **`esp-bridge` → `stack-soak`** and moves to
  `src/bin/stack-soak.rs` (cargo auto-discovery replaces the explicit `[[bin]]` path). It stays
  the regression soak that can reproduce the ticket-09 evidence; `tools/soak.py` is unchanged.
- The CI `bridge` job's `working-directory` and the `Swatinem/rust-cache` `workspaces:` key
  now say `bridge-firmware` (`.github/workflows/ci.yml`).
- The root workspace's `exclude` now says `bridge-firmware` (`Cargo.toml`); the exclude remains
  the contract that keeps the RISC-V workspace out of the robot's `thumbv8m` builds.
- Historical records keep the old name deliberately: this ADR's body above, ADR-0033, and the
  resolved tickets of the expansion-and-remote effort (notably ticket 09's Answer) all predate the
  rename and are not rewritten.

## Amendment — 2026-11-07: the host workspace's two bench-tool crates

The layout above lists `remote/`'s members as `bridge-net` and `daemon`. The host workspace now
also holds **`robot-stub`** and **`bridge-stub`**, each a library plus a binary rather than a
library alone. Recorded here as an amendment because the layout is otherwise unchanged.

- `robot-stub` is the robot's bench persona: the real `remote-link` `LinkCore` behind a serial
  port, with load and fault personas. It path-depends on `remote-link` and `remote-protocol` (both
  with their `std` feature) and is the only host crate that depends on `serialport`.
- `bridge-stub` is the daemon's bench peer: the `bridge-net` preamble server side, the Discovery
  Beacon and scripted robot traffic. It path-depends on `bridge-net` and `remote-protocol`.
- Both are **bench tools, not domain concepts**: they exist so a developer can drive the daemon
  and the firmware from a host machine, and so they stay in the host workspace instead of becoming
  members of the robot or Bridge workspaces.
- The `remote` CI job builds and tests them: its `cargo clippy --workspace --all-targets` and
  `cargo test --workspace` cover the new members, and `robot-stub`'s binary needs `libudev` on the
  runner.
- They follow the existing host rules unchanged: path dependencies carry a `version`, and each
  crate takes `[lints] workspace = true`.

## Amendment — 2026-11-08: the role tree, the crate buckets and the renames

The layout above places the firmware at the repository root and its shared crates beside it. The
tree now groups the workspace by role. Recorded here as an amendment because the decision above
stands; only the homes and three names change.

- The root is a **virtual workspace manifest**: it has no package. Its members are `robot`,
  `robot/crates/link/*`, `robot/crates/drivers/*`, `robot/crates/models/*` and
  `robot/crates/hardware-tests`; it excludes `bridge` and `remote`. The profiles and the shared
  lint table stay in the root manifest, so the robot package and the members keep inheriting them,
  and `.cargo/config.toml` and `rust-toolchain.toml` keep applying upward from the root.
- The robot package (`explorer-robot`) and its `src/`, `build.rs` and `memory.x` move into
  `robot/`. The build script still embeds `memory.x` relative to the package.
- The crates are grouped into three buckets under `robot/crates/`: `link/` (`link-protocol`,
  `link-core`), `drivers/` (`coin-d6`, `st7789-async`, `touch-async`) and `models/`
  (`claim-slot`, `lidar-cloud`, `touch-ui`). `hardware-tests/` stays a member of the robot
  workspace, beside the buckets, because its examples run on the robot hardware rather than the
  host bench.
- The renames: `remote-protocol` → `link-protocol` (package version `1.1.0` unchanged; the
  wire-version test still couples `PROTOCOL_MAJOR`/`MINOR` to `CARGO_PKG_VERSION_MAJOR`/`MINOR`),
  and `remote-link` → `link-core` (version `0.1.0` unchanged). Every consumer, path dependency, CI
  `-p` flag and live doc follows the new names.
- The Bridge directory `bridge-firmware/` → `bridge/`. The package name `bridge-firmware` and the
  `bridge` binary stay, so the path changed and the names did not. Its path dependencies and its
  `build-std` workspace are unchanged.
- `remote/` keeps its name deliberately: the glossary owns **Remote** for the PC-side client, so
  the directory named after it stays `remote/`. `bridge-net` stays in the host workspace, as the
  decision above requires.
- The hardware folders `assembly/`, `cad files/` and `eda files/` → `hardware/assembly/`,
  `hardware/cad/` and `hardware/eda/`, with `hardware/README.md` owning the BOM, assembly, print
  and safety prose.
- CI keeps the `robot`, `bridge` and `remote` jobs with every step, updating only working
  directories, cache keys, package flags and step names, and gains one `docs` job that runs the
  live-Markdown link check. The ADRs stay historical and are excluded from that check.
- Historical ADRs keep their old paths; this amendment is the path note, and the body above is not
  rewritten.

## Amendment — 2026-11-09: one Rust pin, and embassy master in the robot

The three workspaces still have three targets and three lockfiles; they no longer have three Rust
versions. The robot and host workspaces were pinned to 1.96.0 while the Bridge followed `stable`
(1.99.1 at the time of writing). Recorded here because the toolchain is part of the layout above.

- The **root** `rust-toolchain.toml` is the only pin: channel `1.99.1`, components `rustfmt`,
  `clippy` and `rust-src`, targets `thumbv8m.main-none-eabihf` and
  `riscv32imc-unknown-none-elf`. `bridge/rust-toolchain.toml` is deleted, so the Bridge inherits the
  root pin like `remote/` does. Every CI job installs 1.99.1 with both targets
  (`thumbv8m.main-none-eabihf`, `riscv32imc-unknown-none-elf`) and the `rustfmt`, `clippy` and
  `rust-src` components; it is not only the `bridge` job that does.
- The **robot** workspace follows embassy's "use git when there is no release" policy. Root
  `[patch.crates-io]` pins `embassy-rp`, `embassy-embedded-hal`, `embassy-sync`, `embassy-executor`,
  `embassy-time` and `embassy-futures` to embassy commit
  `9b0680bdd75c6a665aabb3e760ef08ea576abaae` (2026-11-09). All six move together; the revision is
  pinned in root `[patch.crates-io]` and recorded in `Cargo.lock`, so builds stay reproducible.
- The **Bridge** cannot follow that pin yet. `esp-hal` 1.2.2 and `esp-radio` 1.0.0-beta.1 implement
  the crates.io `xarxa-driver` 0.1.0 API, while embassy master's `embassy-net` depends on a git
  `xarxa-driver` at revision `8f6f33b` whose buffer-ownership and driver-trait API is different. A
  graph with both would contain two packages named `xarxa-driver` 0.1.0 from different sources, and
  the Wi-Fi `Interface` would implement the registry trait rather than the git trait the stack
  accepts. The Bridge therefore keeps the released embassy crates and `embassy-net` 0.9.1
  (smoltcp-backed) until Espressif ships a compatible `xarxa-driver` generation. The direct, unused
  `smoltcp` dependency is removed; smoltcp remains a transitive dependency of released
  `embassy-net` only.
- Consequences: a single `rustup` pin and one stable toolchain cover the whole repository, so a
  contributor no longer enters `bridge/` to get a different compiler. The robot can take embassy
  fixes and API changes immediately; the Bridge waits on its HAL rather than carrying a second
  xarxa driver or a packet-copying adapter. Revisit the Bridge pin when `esp-hal`/`esp-radio` update
  `xarxa-driver`, and re-check the embassy commit then.
