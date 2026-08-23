# COIN-D6 360° LiDAR: hand-rolled async driver

The COIN-D6 360° spinning dToF LiDAR streams a continuous point cloud over
UART. We chose to hand-roll a `#![no_std]`, HAL-agnostic async driver — owning
the UART and the power-MOSFET pin, feeding bytes through a pure decoder, and
exposing single-revolution and aggregated scans — modelled on `st7789-async`.
The pure decoder and post-processing stages are host-tested so the framing and
aggregation logic can be validated without hardware.

**Status:** accepted

**Decision**

- Hand-rolled `#![no_std]`, HAL-agnostic async driver in the `coin-d6` workspace
  crate, modelled on `st7789-async`.
- Generic over `embedded_io_async::Read + Write` and a synchronous
  `embedded_hal::digital::OutputPin` for the power MOSFET.
  **Deviation:** the spec named `embedded_hal_async::serial`, but
  `embedded-hal-async` 1.0 has no `serial` module, so the driver uses
  `embedded-io-async` (the standard async serial trait). The example drives the
  sensor with embassy's interrupt-driven `BufferedUart`, which implements those
  traits directly, so no adapter is needed.
- Decoupled `Scan`/`Point` with no knowledge of the robot's point-cloud
  representation; native 0.9° / 400 points, millimetres, with `Scan` sized to
  512 for headroom against slow-spin truncation.
- A pure byte-stream `Decoder` (checksum, `AA 55` header resync, ring-start
  delimiting, angle interpolation incl. wrap, `0xFE`/`0xFF` spin-up skipping)
  plus pure `aggregate` (median/validity gate) and `angle_correction_deg`.
- An optional rotor warm-up phase (`CoinD6::warm_up`) driven by a pure `Warmup`
  state machine: the per-revolution point count proxies rotor speed, and the
  phase settles, plateaus, or exhausts according to a `WarmupConfig`. The pure
  decoder, post-processing, and warm-up stages are host-tested via
  `cargo test -p coin-d6 --features std --target x86_64-unknown-linux-gnu`.
- The passive concurrency model: no spawned tasks; the caller awaits
  `read_scan`/`read_aggregated`/`warm_up`.
- UART read errors (overrun, break, framing, parity) are treated as recoverable
  resync rather than fatal: `embedded_io::ErrorKind` cannot distinguish them
  (embassy maps every UART error to `Other`), so the driver resyncs the decoder
  and keeps reading instead of surfacing `Error::Uart`.
- The distance-dependent angle correction transcribed verbatim from the vendor
  SDK and gated by `Config::angle_correction` (flagged for hardware validation).
- A portable data-starvation watchdog for the `Error::Timeout` variant — byte
  count, end-of-stream, and missed chunks from UART errors all count toward it;
  the driver is executor-agnostic and has no wall clock, and the example can add
  a wall-clock timeout at the caller level.

**Consequences**

- The driver lives in `coin-d6/`, decoupled from `embassy-rp`, so it can be
  host-tested in isolation and reused later.
- Because UART read errors resync rather than fail, a caller that pauses after
  `power_on()` (for example a delay) no longer crashes on the resulting RX
  overrun — the driver re-synchronises and continues.
- Robot firmware integration — rewiring core1 and the 400/0.9° `LidarPointCloud`
  — is a separate follow-up.
