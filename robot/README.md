# explorer-robot (robot package)

The robot firmware package: the RP2350B `thumbv8m.main-none-eabihf` binary, its
`build.rs` and `memory.x`, and the workspace crates under `crates/`. It is a
member of the root virtual workspace, which is the robot's Cargo workspace;
`bridge/` and `remote/` are separate workspaces for the RISC-V and host
build targets ([ADR-0040](../docs/adr/0040-repo-and-build-layout.md)).

- [Front door](../README.md)
- [Robot operator page](../docs/components/robot.md) — build, test, flash and recovery
- [Architecture — repo map and data flow](../docs/architecture.md)
- [Domain glossary](../CONTEXT.md)
- [ADR-0040 — the three workspaces and the layout](../docs/adr/0040-repo-and-build-layout.md)

The crate buckets are `link/` (`link-protocol`, `link-core`), `drivers/`
(`coin-d6`, `st7789-async`, `touch-async`), `models/` (`claim-slot`,
`lidar-cloud`, `touch-ui`) and `hardware-tests/`.

Run the robot commands from the repository root; see the operator page.
