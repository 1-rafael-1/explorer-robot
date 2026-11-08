# Robot

The robot firmware: the `explorer-robot` package on the RP2350B, covering the
drive subsystem, LiDAR perception, the panel UI and the robot side of the Link
Protocol.

**Verified:** pending — page content lands in ticket 02.

## Source material

- [Root README](../../README.md) — the front door and the hardware summary.
- [remote-protocol](../../remote-protocol/README.md) — the Link Protocol wire
  contract.
- [remote-link](../../remote-link/README.md) — `LinkCore`, the robot side of the
  protocol.
- [coin-d6](../../coin-d6/README.md) — the LiDAR driver and its wiring.
- [st7789-async](../../st7789-async/README.md) — the display driver.
- [hardware-tests](../../hardware-tests/README.md) — standalone sensor checks that
  run beside the firmware.
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
