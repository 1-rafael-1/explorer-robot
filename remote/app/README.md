# app — the remote web app

The static SPA served by the daemon at `/` (ADR-0038, ADR-0039), part of the
`remote/` host workspace. Vanilla ES modules and `<canvas>`: no bundler, no build
step and no generated output. The daemon embeds this folder at compile time and
`--assets <path>` serves it from disk during development.

This folder is currently a **placeholder**: it holds only this README, so `GET /` answers 404
until the SPA lands. The daemon's WebSocket, auth, discovery and relay paths are complete and
tested. The earlier prototype that pinned the app contract is not part of this repository.

The server contract is in [`../daemon/README.md`](../daemon/README.md),
[the remote page](../../docs/components/remote.md) and
[ADR-0038](../../docs/adr/0038-remote-app-boundary-and-rendering.md).
