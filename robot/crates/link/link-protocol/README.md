# link-protocol

The shared Link Protocol contract (ADR-0030, ADR-0031, ADR-0041). It is `#![no_std]`,
transport agnostic and allocation-free: the frame envelope, the shared identifiers, the
payload codecs, the CRC and the reliability mechanisms that the robot, the Bridge and the
daemon all speak. There is no UART, clock, allocator or logging; every reliability entry
point takes `now` or a sequence number.

It is a member of the robot workspace so the firmware gets it by path; the Bridge and the
host workspace consume the same manifest. The repo map is in
[architecture.md](../../../../docs/architecture.md); the operator context is in the
[robot page](../../../../docs/components/robot.md).

## Versioning

The crate version **mirrors the wire version**: `PROTOCOL_MAJOR`/`PROTOCOL_MINOR` are `2`/`1`,
the manifest is `2.1.0`, and a test asserts the two agree. Every consumer states
`version = "2"`, so a major bump fails resolution until each is reviewed. Additive ids,
enums and optional fields bump the minor; a payload-layout or header-semantics change bumps
the major (ADR-0040). The 2.1.0 bump gave the reserved `PERCEPTION` bits 2/3 the front and
rear floor-drop meanings (ADR-0051). The 2.0.0 bump added the Station Network SSID to
`BridgeStatus` (ADR-0050); the Bridge's `EXPH` preamble keeps its own major.

## API map

| module | contents |
|---|---|
| `frame` | `MAGIC`, `HEADER_LEN`, `Frame`, `FrameHeader`, `Class`, `Direction`, `Flags`, `Seq`, `Status` |
| `codec` | `encode_*`/`decode_*` for command, control, event and stream; `PayloadCodec` |
| `crc` | `Crc16`, incremental and one-shot, CRC-16/CCITT-FALSE |
| `ids` | `Stream`, `BridgeStream` (`Status = 0x0007`), `Command`, `Event`, `Control`, `ProcedureId`, `Caps` |
| `payload` | the typed stream, event, command and control payloads, including `BridgeControl` and the 61-byte `BridgeStatus` |
| `parser` | `Parser<CAP>`, `RobotToPcParser`, `PcToRobotParser`, `MAX_*_FRAME` |
| `reliability` | `AckTracker`, `CommandWindow`, `ReplayWindow`, `RetryPolicy`, `Throttle` |
| `sim` | `FakeRemote`, `Faults`, `Wire`, behind the `std` feature |
| `error` | `EncodeError`, `CodecError`, `FrameError` |

`Class` is stream/command/event/control: commands are reliable and at-most-once, events
reliable and at-least-once, streams lossy and latest-wins, and control is per frame (ADR-0030).

## Bridge Status

`BridgeStatus` is 61 bytes in both directions. The first 28 bytes are the link and resilience
snapshot: `rssi_dbm`, `channel`, `station`, `flags`, then six little-endian `u32` counters. Offset
28 is `ssid_len` and 29–60 are the zero-padded **Station Network** SSID. `ssid_len = 0` means
unknown; the decoder rejects any other payload length and any `ssid_len` above 32. The SSID is an
octet string and is kept verbatim (ADR-0042, ADR-0050).

## Parser statistics

The incremental parser counts what damage it discarded, for the Bridge's pump diagnostics:

- `crc_failures()` — candidates rejected because the CRC field did not match;
- `resyncs()` — candidates discarded and the scan restarted;
- `discarded_bytes()` — header and payload bytes dropped while resynchronising.

The counters are cumulative for the parser's lifetime; `reset()` discards a partial candidate
but does not clear them. The parser also exposes `timeout_ms()` and `set_timeout_ms()` for
tests; the production timeout is four times the largest frame's wire time with a 50 ms floor
(ADR-0033).

## The `std` host-test pattern

The crate is always `no_std`; the `std` feature exists only for host tests and the simulated
lossy link, and compiles to nothing on a bare-metal target so the workspace's `--all-features`
clippy pass stays valid. Integration tests declare `required-features = ["std"]`.

From the repository root:

```sh
cargo test -p link-protocol --features std --target x86_64-unknown-linux-gnu --tests
```

The explicit target overrides the root workspace's `thumbv8m.main-none-eabihf` default.

## Tests

`crc`, `frame`, `ids`, `parser`, `streams`, `commands`, `control`, `bridge_control`,
`bridge_status`, `retry`, `events`, `throttle`, `sequence` and `version` cover the envelope, every
payload, the reliability schedules and the version mirror.

## Validation

CI's `robot` job runs the host tests above plus:

```sh
cargo +nightly fmt --all -- --check
cargo clippy --workspace --target thumbv8m.main-none-eabihf --no-deps -- -D warnings
cargo clippy --workspace --target thumbv8m.main-none-eabihf --all-features --no-deps -- -D warnings
```

The `bridge` job compiles the same crate for `riscv32imc-unknown-none-elf` by building the
firmware, which is the cross-target gate (ADR-0040).

## Known limits

- The baseline stream rates and event retry interval remain protocol constants; the
  catalogue's reserved streams (`POSE`, `MAP`, …) have no payloads yet.
- Reliability is mechanism only: policy lives in `link-core` (robot) and the daemon.
- The simulated link is `std`-only and is not a conformance suite for a third-party peer.
