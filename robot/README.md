# explorer-robot

The robot firmware package: the RP2350B `thumbv8m.main-none-eabihf` binary, its
`build.rs` and `memory.x`, and the workspace crates under `crates/`.

- [Front door](../README.md)
- [Robot operator page](../docs/components/robot.md) — build, test, flash and recovery
- [Domain glossary](../CONTEXT.md)
- [ADR-0040 — the three workspaces and the layout](../docs/adr/0040-repo-and-build-layout.md)

The crate buckets are `link/` (`link-protocol`, `link-core`), `drivers/`
(`coin-d6`, `st7789-async`, `touch-async`), `models/` (`claim-slot`,
`lidar-cloud`, `touch-ui`) and `hardware-tests/`.

Run the robot commands from the repository root; see the operator page.
