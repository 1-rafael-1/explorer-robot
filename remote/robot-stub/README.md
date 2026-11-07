# robot-stub

The robot's bench persona (ADR-0040's host-workspace amendment). It puts the **real**
`remote-link::LinkCore` behind a serial port, replacing the robot on the Bridge's bench: it
answers `HELLO`/`PING`, deduplicates commands, ACKs with the robot's rules and retries
unacknowledged events. The personas only configure rates and load around that core; the wire
semantics stay the crate's. The library never depends on a port library — that is the binary's
job.

## Library map

- **`Stub::new(config, boot_id, persona, transport)`** — one stub over a `Transport`.
- **`Stub::step(now_ms)`** — drain, service and transmit once; the caller owns time.
- **`Stub::set_faults(FaultKnobs)`**, `faults()`, `persona()` — load and fault state.
- **`Stub::peer()`**, `dropped_events()`, `executions()` — observable link state and log.
- **`Stub::request_bridge_control(action, now_ms)`** and `take_bridge_control_report()` — the
  `BRIDGE_CONTROL` request path and the Bridge's reply or timeout.
- **`Persona`** — `Baseline`, `WorstCase`, `BridgeControl`.
- **`worst_case_config()`** — the timing config `WorstCase` needs (all streams at 10 Hz).
- **`FaultKnobs`** — `drop_every`, `corrupt_every`, `delay_ms`, applied to outbound bytes only.
- **`Transport`** — `read`/`write`; `duplex()` gives two in-memory ends for tests.
- **`Execution`**, **`BridgeControlReport`**, **`OwnedReply`** — the binary's log and report
  types.

`LinkCore` itself is documented in [`../../remote-link/README.md`](../../remote-link/README.md).

## Personas

| persona | what it does |
|---|---|
| `baseline` | moving robot at the negotiated or baseline rates; idle Activity; a completion event per executed drive |
| `worst-case` | every stream changed at the 10 Hz sampling tick plus the catalogue's largest event paced at 40 Hz, sustaining ADR-0033's budget |
| `bridge-control` | the baseline profile, selected so the bench can exercise the `BRIDGE_CONTROL` path |

## Binary

```sh
cargo run -p robot-stub -- --port /dev/ttyUSB0
```

The port opens at **921600 8N1**, the baud all three projects share (ADR-0033), with a 1 ms
read timeout. The stub steps every 10 ms and prints a one-line summary every 5 s.

| flag | meaning |
|---|---|
| `--port <path>` | serial device; required |
| `--persona baseline\|worst-case\|bridge-control` | load profile; default `baseline` |
| `--action <0\|1\|2>` | queue one `BRIDGE_CONTROL`; requires `--persona bridge-control` |
| `--boot-id <N>` | Boot Id for this powered link period; default `0x53545542` |
| `--drop-every <N>` | drop every n-th outbound byte |
| `--corrupt-every <N>` | flip every n-th outbound byte |
| `--delay-ms <N>` | hold outbound bytes for n milliseconds |

There is no `--help`: the parser is deliberately tiny and rejects an unknown flag by name.
`--port` is required, and `--action` needs the `bridge-control` persona.

## Wiring

| robot signal | adapter |
|---|---|
| UART TX | adapter RX |
| UART RX | adapter TX |
| GND | GND |

The stub presents the robot side of the robot↔Bridge UART, so a USB-serial adapter wired to
the Bridge's expansion header replaces the robot. No flow control is used; the protocol
tolerates loss.

## Tests

```sh
cargo test -p robot-stub
```

`baseline`, `bridge_control`, `fault_knobs`, `loopback` and `worst_case` drive the stub over
in-memory `duplex()` transports, so no serial port is needed. On Linux the crate still needs
`libudev` at build time because the binary depends on `serialport`.

## Validation

From `remote/`:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --all-targets -- -D warnings
```

## Known limits

- A bench tool, not the robot firmware: it exercises `remote-link`, not the RP2350 HAL, LiDAR
  or drive subsystem.
- Fault knobs act on outbound bytes only; inbound damage is the peer's to inject.
- Time is the caller's: the library takes `now_ms`, and the binary feeds the wall clock.
