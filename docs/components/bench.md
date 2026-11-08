# Bench

The bench operator's path: the harness, the power and level rules, the serial
adapters and the scripts that drive the Bridge and the Remote.

**Verified:** pending — page content lands in ticket 05.

## Source material

- [bridge-firmware README](../../bridge-firmware/README.md) — bench wiring, the
  power rules and the acceptance runs.
- [robot-stub README](../../remote/robot-stub/README.md) — the robot persona on
  the bench.
- [bridge-stub README](../../remote/bridge-stub/README.md) — the daemon's bench
  peer.
- [hardware-tests README](../../hardware-tests/README.md) — standalone sensor
  checks and wiring.
- [remote-daemon README](../../remote/daemon/README.md) — the daemon the bench
  scripts exercise.
- [bridge-firmware/tools/](../../bridge-firmware/tools/) — `link-probe.py`,
  `auth-probe.py`, `beacon-watch.py`, `soak.py` and the shared preamble module.
- [CONTEXT.md](../../CONTEXT.md) — the glossary's Link Port, Bridge and Pairing
  Token entries.

Owning decisions:

- [ADR-0033](../../docs/adr/0033-uart1-baud-and-ring-budget.md) — the 921600 8N1
  link and its budget.
- [ADR-0037](../../docs/adr/0037-reachability-and-trust.md) — the Discovery
  Beacon and the HMAC preamble.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the workspaces and
  the current-directory rule.
- [ADR-0042](../../docs/adr/0042-bridge-telemetry.md) — Bridge Status on the
  bench.
