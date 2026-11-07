# bridge-stub

The daemon's bench peer (ADR-0040's host-workspace amendment). It serves the server side of
`bridge-net`'s HMAC preamble — the challenge with identity, client-proof verification,
`ACCEPT` and the `DENY` variants — plus the Discovery Beacon and scripted robot traffic behind
an accepted connection. Both sides compose `bridge-net`'s framing and `remote-protocol`'s
payload codecs, so the stub never invents wire semantics. It replaces the Bridge, not the
robot; see [`../robot-stub/README.md`](../robot-stub/README.md) for the robot end.

## Library

- **`Stub::start(Scenario)`** — bind the Link listener and Beacon socket and start serving.
- **`Stub::link_addr()`**, `beacon_target()`, `stop()` — the bound endpoint, the Beacon
  destination and idempotent shutdown.
- **`Scenario::new(token)`** — the token, `identity`, `bind`, `link_port` (`None` = ephemeral),
  `beacon`, `deny`, `auth_deadline_ms` (5 s) and `traffic`.
- **`Traffic::baseline()`** — 1 Hz motion and activity, Boot Id 1, no events.
- **`Traffic { boot_id, streams, events, event_retry, command_acks }`** — the scripted robot
  behind `ACCEPT`; unlisted commands ACK `OK`.
- **`StreamRate`**, **`ScriptEvent`**, **`BeaconScript`**, **`Deny`** — the script vocabulary.

## Binary

```sh
cargo run -p bridge-stub -- --help
```

| flag | meaning |
|---|---|
| `--token <hex>` | pairing token bytes; default `62656e63682d746f6b656e` (`bench-token`) |
| `--name <text>` | display name; default `Explorer-STUB` |
| `--bind <ip>` | Link listener address; default `127.0.0.1` |
| `--link-port <port>` | Link port; default 9000, `0` picks an ephemeral one |
| `--beacon-addr <ip:port>` | Beacon destination; default `255.255.255.255:9001` |
| `--beacon-interval <ms>` | Beacon interval; default 3000 |
| `--setup-mode` | set the Setup Mode flag and deny with `setup_mode` |
| `--portal-only` | advertise `tcp_port = 0` (portal only) |
| `--deny <reason>` | deny every connection: `unauthorized`, `busy`, `setup_mode` or `version` |
| `--retry-ms <ms>` | the deny's retry hint |
| `--stream <name>:<chz>` | script a stream at a rate; repeatable |
| `--event <name>` | script an event; repeatable |

Stream names are `power`, `motion`, `activity`, `perception`, `lidar_cloud` and `calibration`;
events are `obstacle`, `floor_drop`, `procedure_completed`, `drive_distance_completed`,
`rotate_completed`, `calibration_loaded` and `event_overflow`.

## What it scripts

- The preamble: challenge, both proofs, `ACCEPT`, and every `DENY` reason with its optional
  `retry_ms`; a silent client is dropped at the 5 s deadline, and a second live client is
  denied `busy`.
- The Beacon: identity, flags, advertised Link port, `--portal-only`, and the interval.
- Behind `ACCEPT`: `HELLO_ACK`, `PING_ACK`, streams at their centi-Hz rates, events retried
  until acknowledged, and command ACKs with scripted statuses.

## Tests

```sh
cargo test -p bridge-stub
```

`beacon`, `binary`, `deny`, `preamble` and `traffic` run the stub in-process over real
loopback sockets; `preamble` recomputes HMAC-SHA256 straight from the ADR's formula, and the
`binary` test spawns the built executable. No daemon, Bridge or serial port is needed.

## Validation

From `remote/`:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
```

## Known limits

- A bench tool, not the Bridge firmware: no Wi-Fi, provisioning or real hardware RNG, and the
  token is whatever bytes `--token` supplies.
- Plain blocking `std::net`; a test starts it in-process without an async runtime.
- One authenticated client at a time, matching the Bridge's contract.
