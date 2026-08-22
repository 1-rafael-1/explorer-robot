# COIN-D6 360° LiDAR: hand-rolled async DMA driver

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
- Generic over `embedded_io_async::Read + Write` (DMA via embassy's plain `Uart`)
  and a synchronous `embedded_hal::digital::OutputPin` for the power MOSFET.
  **Deviation:** the spec named `embedded_hal_async::serial`, but
  `embedded-hal-async` 1.0 has no `serial` module, so the driver uses
  `embedded-io-async` (the standard async serial trait) and the example adapts
  embassy's DMA `Uart` to it with a small example-local adapter.
- Decoupled `Scan`/`Point` with no knowledge of the robot's point-cloud
  representation; native 0.9° / 400 points, millimetres, with `Scan` sized to
  512 for headroom against slow-spin truncation.
- A pure byte-stream `Decoder` (checksum, `55 AA` header resync, ring-start
  delimiting, angle interpolation incl. wrap, `0xFE`/`0xFF` spin-up skipping)
  plus pure `aggregate` (median/validity gate) and `angle_correction_deg`; the
  pure layer is host-tested via `cargo test -p coin-d6 --features std`.
- The passive concurrency model: no spawned tasks; the caller awaits
  `read_scan`/`read_aggregated`.
- The distance-dependent angle correction transcribed verbatim from the vendor
  SDK and gated by `Config::angle_correction` (flagged for hardware validation).
- A portable data-starvation watchdog for the `Error::Timeout` variant — the
  driver is executor-agnostic and has no wall clock; the example can add a
  wall-clock timeout at the caller level.

**Consequences**

- The driver lives in `coin-d6/`, decoupled from `embassy-rp`, so it can be
  host-tested in isolation and reused later.
- Robot firmware integration — rewiring core1 and the 400/0.9° `LidarPointCloud`
  — is a separate follow-up.
