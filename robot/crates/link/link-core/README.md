# link-core

The robot side of the Link Protocol (ADR-0032, ADR-0041). `LinkCore` owns the RX parser, the
latest-wins stream slots, the reliable event ring, the 5 s lease and 2 s Deadman, command
dedupe and TX priority. It composes the mechanisms from `link-protocol` rather than
reimplementing them. It is `#![no_std]` and allocation-free; the firmware owns the UART, port
power, sampling and the mapping onto the wire vocabulary.

It is a member of the robot workspace and is consumed by the firmware's Link Task and by the
Robot Stub. The repo map is in [architecture.md](../../../../docs/architecture.md); the operator
context is in the [robot page](../../../../docs/components/robot.md).

## API map

- **`LinkCore::new(config, boot_id, now_ms)`** — one core for this powered link period.
- **`on_rx(bytes, now_ms)`** — feed inbound bytes; **`next_frame(out)`** — drain one outbound
  frame; **`tick(now_ms)`** — lease, Deadman, retries and throttle expiry.
- **`publish_stream(&StreamSample, requested_rate, now_ms)`** — replace a stream's pending
  sample; **`publish_event(&EventMessage, now_ms)`** — append to the event ring (`Publish`).
- **`poll()`** — yields one `Inbound`: a `Command`, an `EventAck`, a `ControlReply` or a
  `ControlTimeout`.
- **`ack_command(seq, status, result)`** — answer the command awaiting a reply.
- **`request_bridge_control(action, now_ms)`** — queue an acknowledged `BRIDGE_CONTROL`.
- **`bridge_status()`** — the stored `BRIDGE_STATUS` control snapshot from the Bridge (`None`
  before the first); it is kept even with no Remote connected.
- **`peer()`**, `take_peer_lost()`, `dropped_events()`, `pending_events()`,
  `exhausted_events()`, `last_seq()`, `last_status()`, `version_mismatch()` — observability.
- **`reset_parser()`**, `reset_epoch(epoch)` — recovery without losing the Boot Id or ring.

`timing` holds the contract values: `BAUD` (921600), `LEASE_MS` (5 s), `DEADMAN_MS` (2 s),
the event retry schedule and ring size, and `Config`, whose fields default to those constants
so a host test can compress time. The encoded event ring is internal; `Inbound`,
`CommandRequest` and `ControlReply` are the firmware seam.

## Command seam

`poll` yields a structurally valid, known, same-major, in-order command; the firmware answers
with `ack_command` before polling again. Unknown ids, malformed payloads, bad flags, version
mismatches and stale sequences never reach the firmware. `HELLO` and `PING` are answered
inside the core. Events are sequenced per Boot Id, retried until acknowledged and reported
honestly on ring overflow.

## The `std` host-test pattern

The crate is always `no_std`; `std` enables `link-protocol/std` and the simulated link, and
is host-only. Integration tests declare `required-features = ["std"]`.

From the repository root:

```sh
cargo test -p link-core --features std --target x86_64-unknown-linux-gnu --tests
```

## Tests

`streams`, `commands`, `session`, `events`, `throttle`, `bridge_control`, `bridge_status`,
`acceptance` and `fault_matrix` cover the stream schedule, the two-phase command seam, the
Session and lease, the event ring, throttle merging, the `BRIDGE_CONTROL` and `BRIDGE_STATUS`
paths and loss/truncation/stall recovery. A `no_std` binary cannot use the built-in harness, so
the robot firmware links this crate rather than testing it on target.

## Validation

CI's `robot` job runs:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --target thumbv8m.main-none-eabihf --no-deps -- -D warnings
cargo clippy --workspace --target thumbv8m.main-none-eabihf --all-features --no-deps -- -D warnings
cargo test -p link-core --features std --target x86_64-unknown-linux-gnu --tests
```

## Known limits

- Robot-only: the Bridge and the host workspace cannot depend on it (ADR-0040).
- It holds no HAL, UART, clock or logging; the firmware drives it with millisecond
  timestamps.
- The baseline rates and retry schedules are v1 constants; reserved streams have no payloads.
- There is no HIL test; the fault matrix is a host simulation.
