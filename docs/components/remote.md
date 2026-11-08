# Remote

The PC-side client: the `remote-daemon` host relay plus the static web app, the
two bench stubs and the shared `bridge-net` network edge.

**Verified:** pending — page content lands in ticket 04.

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
- [remote-protocol README](../../remote-protocol/README.md) — the wire contract;
  [remote-link README](../../remote-link/README.md) — the core the Robot Stub
  runs.
- [CONTEXT.md](../../CONTEXT.md) — the glossary's Remote, Relay and Session
  entries.

Owning decisions:

- [ADR-0037](../../docs/adr/0037-reachability-and-trust.md) — reachability and
  trust.
- [ADR-0038](../../docs/adr/0038-remote-app-boundary-and-rendering.md) — the app
  boundary.
- [ADR-0039](../../docs/adr/0039-remote-daemon-relay.md) — the daemon relay.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the
  current-directory rule and the three workspaces.
