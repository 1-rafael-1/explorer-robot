# coin-d6

An async driver for the COIN-D6 360° spinning dToF LiDAR. It's `#![no_std]` and
HAL-agnostic: it owns an `embedded_io_async::Read + Write` UART and a
synchronous power `OutputPin`, feeds bytes through a pure decoder, and exposes
scans either one revolution at a time or aggregated over several revolutions.

## Public API

- **`Point`** — one LiDAR return: `angle_deg` (degrees), `distance_mm`
  (`Option<NonZeroU16>`, millimetres — `None` means no return), `intensity` (0–255).
- **`Scan<const N = 512>`** — one revolution (native 400 points at 0.9°, with
  512-point capacity), plus `len`.
- **`Decoder`** — pure byte-stream decoder (checksum, `AA 55` header resync,
  ring-start delimiting, angle interpolation).
- **`aggregate`** — fuse several scans by angle with a validity gate and a
  median/mean reducer.
- **`angle_correction_deg`** — the vendor's distance-dependent angle correction.
- **`Warmup`** — pure warm-up settle state machine (settle/plateau/exhaust from
  per-revolution point counts), configured by **`WarmupConfig`** and reporting a
  **`WarmupOutcome`**.
- **`NATIVE_POINTS`** — the steady-state point count per revolution (400).
- **`CoinD6`** — owns the UART and power pin; lifecycle is `power_on` →
  `start` → `warm_up` (optional) → `read_scan`/`read_aggregated` → `stop` →
  `power_off` (or `release` to hand the peripherals back).

## Wiring

| Signal | RP2350 pin |
|--------|-----------|
| LiDAR TX → MCU RX | GPIO 13 |
| LiDAR RX ← MCU TX | GPIO 12 |
| Power MOSFET gate (active-high) | GPIO 15 |
| GND | GND |
| +5 V | 5 V rail |

> These are the standalone example's pins. The robot integration uses different
> pins (LiDAR RX on GPIO1, MOSFET on GPIO26); see the repository `CONTEXT.md`.

## Tests

The pure decoder, post-processing, and warm-up stages are host-tested. The
workspace's
`.cargo/config.toml` defaults to the bare-metal `thumbv8m` target, so pass the
host target explicitly:

```sh
cargo test -p coin-d6 --features std --target x86_64-unknown-linux-gnu --tests
```

## Running the example

The bare-metal example uses the `probe-rs` runner from
`coin-d6/.cargo/config.toml`:

```sh
cargo run -p coin-d6 --example example --release
```
