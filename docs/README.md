# Documentation

The index for the project's written material. Start at the [front door](../README.md)
if you are new to the repository.

## Component pages

| Component | Page | Role |
|---|---|---|
| Robot | [robot.md](components/robot.md) | The firmware package on the RP2350B: drive, perception and the panel UI. |
| Bridge | [bridge.md](components/bridge.md) | The C3 image that translates between the home network and the robot's UART1 Link Port. |
| Remote | [remote.md](components/remote.md) | The PC-side daemon and the static web app. |
| Bench | [bench.md](components/bench.md) | The harness, wiring, power rules and bench scripts. |

## Reference

- [CONTEXT.md](../CONTEXT.md) — the domain glossary and the canonical component names.
- [docs/adr/](adr/) — the architecture decision records. Historical records keep
  their original paths.
- [Architecture](architecture.md) — the repo map and the data-flow tables against the
  final tree.
