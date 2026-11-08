# Bridge

The Bridge firmware: the ESP32-C3 `no_std` image that translates between the home
network and the robot's UART1 Link Port, and acts as the network edge.

**Verified:** pending — page content lands in ticket 03.

## Source material

- [bridge-firmware README](../../bridge-firmware/README.md) — bench wiring, the
  credential record, the partition table, flash and monitor, and the acceptance
  runs.
- [bridge-core README](../../bridge-firmware/bridge-core/README.md) — the
  host-testable pump rules.
- [bridge-net README](../../remote/bridge-net/README.md) — the shared Beacon,
  preamble and ports.
- [remote-protocol README](../../remote-protocol/README.md) — the wire contract
  the Bridge parses and forwards.
- [CONTEXT.md](../../CONTEXT.md) — the glossary's Bridge, Setup Mode and Pairing
  Token entries.

Owning decisions:

- [ADR-0036](../../docs/adr/0036-wifi-provisioning.md) — Wi-Fi provisioning and
  Setup Mode.
- [ADR-0037](../../docs/adr/0037-reachability-and-trust.md) — the Discovery
  Beacon and the HMAC preamble.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the three
  workspaces.
- [ADR-0041](../../docs/adr/0041-reliability-mechanisms-in-protocol.md) —
  reliability mechanisms in the protocol.
- [ADR-0042](../../docs/adr/0042-bridge-telemetry.md) — Bridge telemetry and
  Bridge Status.
