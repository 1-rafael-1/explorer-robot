# Architecture — what lives where and how the parts talk

This page is the repository map and the data flow across it. It uses the vocabulary of the
[domain glossary](../CONTEXT.md#architecture), and every boundary links to the decision record that
owns it. The physical layout and the three build targets are decided in
[ADR-0040](adr/0040-repo-and-build-layout.md).

## Three workspaces at a glance

The repository is one tree with three Cargo workspaces, split by build target so the robot's
`thumbv8m` build never sees the RISC-V or host dependency graphs.

| Workspace | Directory | Cargo target | What it builds | Why it is separate |
|---|---|---|---|---|
| Robot workspace (root virtual manifest) | `/`; the package is `robot/` | `thumbv8m.main-none-eabihf`, from the root `.cargo/config.toml` | `explorer-robot` firmware, the shared `link-protocol` and `link-core`, the drivers and models | The root `cargo check --workspace` must never load the host or RISC-V packages ([ADR-0040](adr/0040-repo-and-build-layout.md)). |
| Bridge workspace | `bridge/` | `riscv32imc-unknown-none-elf` with `build-std`, espflash runner | `bridge-firmware` (the `bridge` binary) and `bridge-core` | It needs its own runner and lockfile, so it is excluded from the root; the root toolchain pin supplies its target and `rust-src` ([ADR-0040](adr/0040-repo-and-build-layout.md)). |
| Remote workspace | `remote/` | `x86_64-unknown-linux-gnu`, overridden in `remote/.cargo/config.toml` | `remote-daemon`, `bridge-net`, `robot-stub`, `bridge-stub`, and the static `app/` | It is a host `std` graph. Cargo reads its config from the current directory upward, so its commands run from `remote/` ([ADR-0040](adr/0040-repo-and-build-layout.md)). |

`link-protocol` is the one crate consumed by all three workspaces; `link-core` is consumed by the
robot firmware and the Robot Stub. Shared crates are path dependencies only, and every consumer
states a `version` ([ADR-0040](adr/0040-repo-and-build-layout.md)).

## Repo map

| Directory | Role | Crates | Build target | Owning ADR |
|---|---|---|---|---|
| `/` (root) | Virtual workspace manifest; shared profiles and lint table; `.cargo/config.toml` and `rust-toolchain.toml`; front door `README.md`; glossary `CONTEXT.md`; license notices | Workspace members only — there is no root package | `thumbv8m.main-none-eabihf` default | [ADR-0040](adr/0040-repo-and-build-layout.md) |
| `robot/` | The robot firmware package: core0 tasks, the core1 LiDAR task, `src/`, `build.rs`, `memory.x` | `explorer-robot` | `thumbv8m.main-none-eabihf` | [ADR-0040](adr/0040-repo-and-build-layout.md), [ADR-0032](adr/0032-remote-link-task.md) |
| `robot/crates/link/` | The wire and the robot-side core: frame envelope, payload codecs, reliability mechanisms, `LinkCore` | `link-protocol`, `link-core` | `thumbv8m.main-none-eabihf`; host tests on `x86_64-unknown-linux-gnu` | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0031](adr/0031-link-protocol-payloads.md), [ADR-0032](adr/0032-remote-link-task.md), [ADR-0041](adr/0041-reliability-mechanisms-in-protocol.md) |
| `robot/crates/drivers/` | Hardware-facing async drivers: LiDAR, display, touch | `coin-d6`, `st7789-async`, `touch-async` | `thumbv8m.main-none-eabihf`; host tests where the driver allows | [ADR-0001](adr/0001-async-st7789-display-driver.md), [ADR-0002](adr/0002-coin-d6-async-driver.md), [ADR-0004](adr/0004-touch-controller-on-shared-display-bus.md) |
| `robot/crates/models/` | Pure, HAL-agnostic, host-testable logic: claim slot, LiDAR cloud grid, link state, panel model | `claim-slot`, `lidar-cloud`, `link-state`, `touch-ui` | `thumbv8m.main-none-eabihf`; host tests on `x86_64-unknown-linux-gnu` | [ADR-0034](adr/0034-one-active-claim.md), [ADR-0029](adr/0029-cloud-grid-wider-than-native-to-avoid-holes.md), [ADR-0013](adr/0013-menu-tree-and-procedure-identity-in-the-ui-model-crate.md) |
| `robot/crates/hardware-tests/` | Standalone, breadboard-scale HIL example binaries that run on the robot hardware | `hardware-tests` | `thumbv8m.main-none-eabihf` | [ADR-0040](adr/0040-repo-and-build-layout.md) |
| `bridge/` | The Bridge workspace: network ⇄ UART1 firmware, credential record, Discovery Beacon, preamble gate | `bridge-firmware` (binaries `bridge`, `stack-soak`), `bridge-core` | `riscv32imc-unknown-none-elf` with `build-std` | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0040](adr/0040-repo-and-build-layout.md), [ADR-0042](adr/0042-bridge-telemetry.md) |
| `bridge/bridge-core/` | The Bridge's HAL-agnostic pump and provisioning rules: coalescing, routing, flow, auth, Setup Mode, Credential Record | `bridge-core` | `riscv32imc-unknown-none-elf`; host tests on `x86_64-unknown-linux-gnu` | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0036](adr/0036-wifi-provisioning.md), [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0042](adr/0042-bridge-telemetry.md) |
| `remote/` | Host workspace for the daemon, the shared network-edge crate, the bench stubs and the static Remote App | `remote-daemon`, `bridge-net`, `robot-stub`, `bridge-stub` | `x86_64-unknown-linux-gnu` | [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md), [ADR-0039](adr/0039-remote-daemon-relay.md), [ADR-0040](adr/0040-repo-and-build-layout.md), [ADR-0043](adr/0043-remote-app-drive-policy.md) |
| `remote/bridge-net/` | The Bridge↔daemon network edge: Discovery Beacon, HMAC preamble, fixed ports | `bridge-net` | `x86_64-unknown-linux-gnu`; also compiled for `riscv32imc` by the Bridge | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0040](adr/0040-repo-and-build-layout.md) |
| `remote/daemon/` | The Remote daemon: raw TCP Link Protocol to the Bridge, the Relay, one `/ws` JSON WebSocket | `remote-daemon` | `x86_64-unknown-linux-gnu` | [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md), [ADR-0039](adr/0039-remote-daemon-relay.md) |
| `remote/robot-stub/` | Bench robot persona: the real `LinkCore` behind a serial port | `robot-stub` | `x86_64-unknown-linux-gnu` | [ADR-0040 amendment](adr/0040-repo-and-build-layout.md#amendment--2026-11-07-the-host-workspaces-two-bench-tool-crates) |
| `remote/bridge-stub/` | Bench Bridge persona: the preamble server, the Discovery Beacon and scripted robot traffic | `bridge-stub` | `x86_64-unknown-linux-gnu` | [ADR-0040 amendment](adr/0040-repo-and-build-layout.md#amendment--2026-11-07-the-host-workspaces-two-bench-tool-crates) |
| `remote/app/` | Static Remote App: vanilla ES modules and `<canvas>`, one store, no build step; `tests/` holds the static browser pages and the real-chain page | None — not a Cargo package | Browser; served by the daemon | [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md), [ADR-0040](adr/0040-repo-and-build-layout.md), [ADR-0043](adr/0043-remote-app-drive-policy.md) |
| `hardware/` | `assembly/`, `cad/`, `eda/`, plus the BOM, assembly, print and safety notes | None | No Cargo target | [ADR-0040](adr/0040-repo-and-build-layout.md) |
| `docs/` | Docs index, this page, the component pages and the ADRs under `adr/` | None | No Cargo target | [ADR-0040](adr/0040-repo-and-build-layout.md) |
| `.github/` | CI workflows, the Remote App's browser runners and the Markdown link check | None | The three workspace jobs, the `web` browser job (static and real-chain pages) and the `docs` job | [ADR-0040](adr/0040-repo-and-build-layout.md) for the workspace jobs; the `web` and docs jobs are repo policy |

`target/` is generated and is not part of the map.

## Data flow

| Hop | Direction | Protocol / contract | Owner ADR and glossary |
|---|---|---|---|
| Robot firmware | Producers → orchestrator, state modules and drive task | Event bus (`EVENT_CHANNEL`, capacity 64), direct sensor channels, perception atomics, state-module locks | [ADR-0032](adr/0032-remote-link-task.md) / [Message Passing](../CONTEXT.md#message-passing) |
| Robot Link Task | Robot internal, core0 | Samples the state modules in lock order, publishes Streams, forwards curated Events, owns Link Port power and UART1 | [ADR-0032](adr/0032-remote-link-task.md), [ADR-0031](adr/0031-link-protocol-payloads.md) / [Link Port](../CONTEXT.md#remote-link) |
| Link Port | Robot ↔ Bridge | UART1 at 921600 8N1 carrying Link Protocol frames: a 12-byte header and a payload of at most 1024 B robot→PC or 256 B PC→robot | [ADR-0033](adr/0033-uart1-baud-and-ring-budget.md), [ADR-0030](adr/0030-link-protocol-envelope.md) / [Link Port](../CONTEXT.md#remote-link) |
| Bridge forwarding | Robot → LAN and LAN → robot | Latest-wins per Stream in an 8 KiB coalescing ring; `THROTTLE` out, `BRIDGE_CONTROL` in, `BRIDGE_STATUS` out | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0036](adr/0036-wifi-provisioning.md), [ADR-0042](adr/0042-bridge-telemetry.md) / [Bridge](../CONTEXT.md#remote-link) |
| LAN discovery | Bridge → Remote | Station Wi-Fi; Discovery Beacon every 3 s as a UDP broadcast on port 9001, carrying no secret | [ADR-0037](adr/0037-reachability-and-trust.md) / [Discovery Beacon](../CONTEXT.md#remote-link) |
| LAN authentication | Remote ↔ Bridge | Mutual HMAC-SHA256 challenge-response in a TCP preamble on the Link Protocol port (default 9000) before any Link Protocol byte | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0040](adr/0040-repo-and-build-layout.md) / [Pairing Token](../CONTEXT.md#remote-link) |
| Remote daemon link | Bridge ↔ daemon | Raw TCP carrying Link Protocol frames: `HELLO`/`HELLO_ACK`, `PING`, Commands and ACKs, Events and ACKs, the `BRIDGE_STATUS` Stream | [ADR-0039](adr/0039-remote-daemon-relay.md), [ADR-0030](adr/0030-link-protocol-envelope.md) / [Session](../CONTEXT.md#remote-link) |
| Relay | Daemon ↔ app | One `/ws` JSON WebSocket derived from the page origin: `hello`, `connection`, `snapshot`, `stream`, `event`, `bridge`, `command_result`; centimetres and counter-clockwise degrees | [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md), [ADR-0039](adr/0039-remote-daemon-relay.md) / [Relay](../CONTEXT.md#remote-link) |
| App | Browser | The static Remote App keeps one store from the `/ws` messages, draws the World Frame radar (no app-side map), and sends `auth`/`pair` and `command`; it never parses the binary wire | [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md), [ADR-0043](adr/0043-remote-app-drive-policy.md) / [Remote](../CONTEXT.md#remote-link) |

## Contracts and where they live

| Contract | Crate / location | Owning workspace | What it owns | Owning ADR and glossary |
|---|---|---|---|---|
| Link Protocol | `robot/crates/link/link-protocol/` (`link-protocol`) | Robot (root) workspace; path dependency of the Bridge and the Remote | Frame envelope, ids and classes, payload codecs, CRC, reliability mechanisms | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0031](adr/0031-link-protocol-payloads.md), [ADR-0041](adr/0041-reliability-mechanisms-in-protocol.md) / [Link Protocol](../CONTEXT.md#remote-link) |
| `LinkCore` | `robot/crates/link/link-core/` (`link-core`) | Robot workspace; consumed by the firmware and `robot-stub` | Robot-side parser, stream slots, event ring, lease, Deadman, command dedupe, TX priority | [ADR-0032](adr/0032-remote-link-task.md), [ADR-0041](adr/0041-reliability-mechanisms-in-protocol.md) / [LinkCore](../CONTEXT.md#remote-link) |
| Bridge rules | `bridge/bridge-core/` (`bridge-core`) | Bridge workspace | Coalescing, routing, flow and `THROTTLE`, the auth gate, Setup Mode, Credential Record codec | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0036](adr/0036-wifi-provisioning.md), [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0042](adr/0042-bridge-telemetry.md) / [Bridge](../CONTEXT.md#remote-link) |
| Network edge | `remote/bridge-net/` (`bridge-net`) | Remote workspace; path dependency of the Bridge and the daemon | Discovery Beacon, HMAC preamble, fixed UDP and TCP ports | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0040](adr/0040-repo-and-build-layout.md) / [Discovery Beacon](../CONTEXT.md#remote-link) |
| JSON App Boundary | `remote/daemon/src/{schema,relay,app}/`, served at `/ws` | Remote workspace | Versioned additive JSON schema, units and sign conversion, snapshot and event replay | [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md), [ADR-0039](adr/0039-remote-daemon-relay.md) / [Relay](../CONTEXT.md#remote-link) |

## Authentication

The trust gate is transport-level and sits at the Bridge, before any Link Protocol byte is
forwarded. The robot never sees an unauthenticated frame, so its lease and Deadman count only
authenticated peers ([ADR-0037](adr/0037-reachability-and-trust.md)).

| Step | What happens | Party | Owner ADR and glossary |
|---|---|---|---|
| 1. Mint the token | At first boot the Bridge draws a 60-bit Pairing Token from the hardware TRNG and stores it in the Credential Record; the panel shows it as twelve base32 characters. | Bridge | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0036](adr/0036-wifi-provisioning.md) / [Pairing Token](../CONTEXT.md#remote-link) |
| 2. Advertise | Every 3 s the Bridge broadcasts the Discovery Beacon on the station interface: magic `EXPR`, Link major, a MAC-derived device id, a name, the TCP port and flags. No secret is carried. | Bridge → LAN | [ADR-0037](adr/0037-reachability-and-trust.md) / [Discovery Beacon](../CONTEXT.md#remote-link) |
| 3. Discover | The daemon listens on UDP 9001, deduplicates by device id and caches the endpoint; a manual `host:port` covers networks that block broadcast. | Remote | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0039](adr/0039-remote-daemon-relay.md) / [Discovery Beacon](../CONTEXT.md#remote-link) |
| 4. Open and nonce | The daemon opens TCP to the Link Protocol port (default 9000) and sends `CLIENT_HELLO` with a 32-byte client nonce. | Remote → Bridge | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0039](adr/0039-remote-daemon-relay.md) / [Session](../CONTEXT.md#remote-link) |
| 5. Bridge proves | The Bridge returns `BRIDGE_CHALLENGE` with its bridge proof, a 32-byte bridge nonce and its identity fields. | Bridge → Remote | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0039](adr/0039-remote-daemon-relay.md) / [Pairing Token](../CONTEXT.md#remote-link) |
| 6. Remote verifies | The daemon verifies the bridge proof before it sends any token-derived bytes, so a spoofer cannot harvest the token. | Remote | [ADR-0037](adr/0037-reachability-and-trust.md) / [Pairing Token](../CONTEXT.md#remote-link) |
| 7. Remote proves | The daemon returns `CLIENT_AUTH` with its client proof over the two nonces. | Remote → Bridge | [ADR-0039](adr/0039-remote-daemon-relay.md) / [Pairing Token](../CONTEXT.md#remote-link) |
| 8. Accept or deny | A valid proof gets `ACCEPT`, and only then does the byte stream become the Link Protocol. A failure gets `DENY` with `unauthorized`, `busy`, `setup_mode` or `version`; failures are rate-limited. | Bridge | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0039](adr/0039-remote-daemon-relay.md) / [Bridge](../CONTEXT.md#remote-link) |
| 9. Enforce one client | Exactly one authenticated client at a time. An unauthenticated connection has 5 s to authenticate, never blocks a new connection, and a second authenticated client is refused. | Bridge | [ADR-0037](adr/0037-reachability-and-trust.md) / [Bridge](../CONTEXT.md#remote-link) |
| 10. Rotate or recover | `BRIDGE_CONTROL` action 0 forgets the network (rotating the token) and enters Setup Mode, action 1 shows the code, and action 2 rotates it and closes the live session. No reflash and no PC are needed. | Panel → Bridge | [ADR-0036](adr/0036-wifi-provisioning.md), [ADR-0037](adr/0037-reachability-and-trust.md) / [Credential Record](../CONTEXT.md#remote-link) |

The two proofs are HMAC-SHA256 with domain-separated labels, exactly as specified in
[ADR-0039](adr/0039-remote-daemon-relay.md):

```text
bridge_proof = HMAC(token, "explorer-link-v1 bridge\0" || client_nonce || bridge_nonce
                          || device_id || flags || name)
client_proof = HMAC(token, "explorer-link-v1 client\0" || client_nonce || bridge_nonce)
```

The Beacon's `setup_mode` flag and `tcp_port = 0` tell the daemon that the Bridge is portal-only and
must not be connected to ([ADR-0037](adr/0037-reachability-and-trust.md)). The Setup Mode AP itself
never carries the Link Protocol listener ([ADR-0036](adr/0036-wifi-provisioning.md) /
[Setup Mode](../CONTEXT.md#remote-link)).

## Inside the robot firmware

### Cores and entry point

`robot/src/main.rs` binds the two executors. Core0 runs the orchestrator, the drive subsystem, the
UI, the sensors on the I2C buses, flash storage and the main event loop; core1 runs only the LiDAR
task ([CONTEXT.md § Architecture](../CONTEXT.md#architecture)).

### State modules

The state modules live in `robot/src/system/state/`: `power`, `calibration`, `perception`, `motion`
and `activity`. The lock order is **power → calibration → perception → motion → activity**, and the
Link Task samples them one lock at a time. Hot-path readers use the lock-free mirrors, while the
touch UI is the only module that draws. The LiDAR cloud lives in `lidar-cloud` and the pose work is
described by [ADR-0007](adr/0007-odometry-architecture.md) /
[CONTEXT.md § Odometry](../CONTEXT.md#odometry).

### Event bus

`robot/src/system/event.rs` defines `EVENT_CHANNEL`, a bounded multi-producer, single-consumer
channel of capacity 64, and the `Events` enum. `robot/src/task/orchestrate.rs` is its only consumer:
`orchestrate()` waits for an event and routes it to an initialization, behavior or UI handler. The
bus carries transitions, not progress; high-frequency sensor data uses direct channels and the
perception atomics, which is the three-way split in
[CONTEXT.md § Message Passing](../CONTEXT.md#message-passing).

### Claim path

`robot/src/task/procedure/` owns the one **Active Claim** ([ADR-0034](adr/0034-one-active-claim.md)).
A producer takes a `Claim` with `claim` or `start`, reports its phase and terminal outcome through
it, and `stop` latches the stop for whichever claim holds the slot. The kinds are `Boot`,
`Boot`, `Procedure(P)` and `Screen(RoomScan)`, and the claim's owner is the only writer of the
`activity` state. A claim records its origin, `Panel` or `Remote`; the Deadman uses it to stop only
Remote-originated work ([ADR-0035](adr/0035-failsafe-deadman.md)). A second claimant is `BUSY`;
anyone may stop; there is no owner or takeover mode.

### Drive path

`robot/src/task/drive/` owns motion. `send_drive_command` and `send_drive_interrupt` in `api.rs` are
the two public entry points; `queue.rs` runs one drive queue at a time; `dispatch.rs` routes each
command to an intent in `distance.rs`, `rotation.rs`, `brake_coast.rs` or `differential.rs`. An
interrupt cancels the active intent, bumps the epoch and drains the queue, so stale queued commands
cannot execute after it. The obstacle and floor-drop edges send `EmergencyBrake`, and the Deadman
takes the same path for a Remote-originated claim ([ADR-0035](adr/0035-failsafe-deadman.md) /
[CONTEXT.md § Driving](../CONTEXT.md#driving)).

### Link side

The Link Task is the firmware's side of the Link Protocol, specified in
[ADR-0032](adr/0032-remote-link-task.md) and implemented by `robot/src/task/remote_link/`. It owns
the Link Port's power, UART1 and the `EXP_IRQ` presence line; drives `LinkCore` from `link-core`;
samples the state modules for Streams; forwards the curated Events; dispatches commands through the
same seam the panel uses, with a long-lived drive job for the volatile drive commands; and runs the
Remote-originated Deadman. It publishes the Bridge Status snapshot into the link state for the
panel. The port resources are assigned in `robot/src/resources.rs` (`RemoteLinkResources`). The
glossary entry for the [Link Task](../CONTEXT.md#remote-link) is the canonical description.

## Vocabulary

| Term | Where it lives | What it means here | Owning ADR and glossary |
|---|---|---|---|
| Robot | `robot/` | The RP2350B firmware package, `explorer-robot` | [ADR-0040](adr/0040-repo-and-build-layout.md) / [Architecture](../CONTEXT.md#architecture) |
| Link Protocol | `link-protocol` | The framed binary contract on UART1, carried over the network to the Remote | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0031](adr/0031-link-protocol-payloads.md) / [Link Protocol](../CONTEXT.md#remote-link) |
| LinkCore | `link-core` | The robot-side parser, stream slots, event ring, lease and Deadman over the protocol | [ADR-0032](adr/0032-remote-link-task.md), [ADR-0041](adr/0041-reliability-mechanisms-in-protocol.md) / [LinkCore](../CONTEXT.md#remote-link) |
| Link Port | `robot/src/resources.rs` | The connector and switched 5 V branch carrying UART1 to the Bridge | [ADR-0032](adr/0032-remote-link-task.md), [ADR-0023](adr/0023-quiesce-uart-before-branch-power-off.md) / [Link Port](../CONTEXT.md#remote-link) |
| Bridge | `bridge/` | The network ⇄ UART1 translation and the network edge | [ADR-0037](adr/0037-reachability-and-trust.md), [ADR-0042](adr/0042-bridge-telemetry.md) / [Bridge](../CONTEXT.md#remote-link) |
| Bridge Presence | `EXP_IRQ` | The Bridge's level-high assertion that its firmware and UART are ready | [ADR-0032](adr/0032-remote-link-task.md) / [Bridge Presence](../CONTEXT.md#remote-link) |
| Bridge Status | `bridge-core`, `link-core` | The 1 Hz 28-byte link and resilience snapshot on control `0x0005` and Stream `0x0007` | [ADR-0042](adr/0042-bridge-telemetry.md) / [Bridge Status](../CONTEXT.md#remote-link) |
| Discovery Beacon | `bridge-net` `beacon` | The 3 s UDP broadcast on the station interface, carrying no secret | [ADR-0037](adr/0037-reachability-and-trust.md) / [Discovery Beacon](../CONTEXT.md#remote-link) |
| Pairing Token | `bridge-core` `record` | The 60-bit per-robot secret, shown as twelve base32 characters, never sent on the wire | [ADR-0037](adr/0037-reachability-and-trust.md) / [Pairing Token](../CONTEXT.md#remote-link) |
| Credential Record | `bridge-core` `record` | The versioned, CRC-checked store of SSID, passphrase and token | [ADR-0036](adr/0036-wifi-provisioning.md) / [Credential Record](../CONTEXT.md#remote-link) |
| Setup Mode | `bridge-core` `setup` | The bounded WPA2 SoftAP and captive portal | [ADR-0036](adr/0036-wifi-provisioning.md) / [Setup Mode](../CONTEXT.md#remote-link) |
| Remote | `remote/` | The PC-side client: daemon plus static app | [ADR-0039](adr/0039-remote-daemon-relay.md) / [Remote](../CONTEXT.md#remote-link) |
| Relay | `remote/daemon/src/relay.rs` | The app-facing state and the one JSON WebSocket plus static SPA | [ADR-0039](adr/0039-remote-daemon-relay.md), [ADR-0038](adr/0038-remote-app-boundary-and-rendering.md) / [Relay](../CONTEXT.md#remote-link) |
| Session | `remote-daemon` | The Remote's claim on the robot; on the wire, a liveness lease | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0034](adr/0034-one-active-claim.md) / [Session](../CONTEXT.md#remote-link) |
| Command | `link-protocol` | A reliable PC→robot request | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0031](adr/0031-link-protocol-payloads.md) / [Command](../CONTEXT.md#remote-link) |
| Event | `link-protocol` | A reliable, rare robot→PC edge notification | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0031](adr/0031-link-protocol-payloads.md) / [Event](../CONTEXT.md#remote-link) |
| Stream | `link-protocol` | A lossy, latest-wins robot→PC channel | [ADR-0030](adr/0030-link-protocol-envelope.md), [ADR-0031](adr/0031-link-protocol-payloads.md) / [Stream](../CONTEXT.md#remote-link) |
| Deadman | `link-core` `LinkCore::tick` | The 2 s peer-silence failsafe that stops Remote-originated claims | [ADR-0035](adr/0035-failsafe-deadman.md) / [Deadman](../CONTEXT.md#remote-link) |
| Active Claim | `robot/src/task/procedure/` | The single exclusive right to be the Activity and produce motion | [ADR-0034](adr/0034-one-active-claim.md) / [Active Claim](../CONTEXT.md#state-modules) |
