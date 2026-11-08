# app — the remote web app

The static SPA served by the daemon at `/` (ADR-0038, ADR-0039). Vanilla ES modules and
`<canvas>`: no bundler, no build step and no generated output. The daemon embeds this folder at
compile time and `--assets <path>` serves it from disk during development.

This folder is currently a **placeholder**: it holds only this README, so `GET /` answers 404
until the SPA lands. The daemon's WebSocket, auth, discovery and relay paths are complete and
tested. The prototype that pinned the app contract is archived in the local, untracked
`.scratch/archive/` tracker; it is not this folder.

The server contract is in [`../daemon/README.md`](../daemon/README.md) and
[ADR-0038](../../docs/adr/0038-remote-app-boundary-and-rendering.md).
