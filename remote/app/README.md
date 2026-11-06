# app — the remote web app

The static SPA served by the daemon at `/` (ADR-0038, ADR-0039). Vanilla ES modules and
`<canvas>`: no bundler, no build step and no generated output. The daemon embeds this folder at
compile time and `--assets <path>` serves it from disk during development.

The prototype that pins the app contract lives in `.scratch/expansion-remote/prototype/`; it is
not this folder.
