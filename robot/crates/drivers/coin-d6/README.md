# coin-d6

An async driver for the COIN-D6 360° spinning dToF LiDAR. It's `#![no_std]` and
HAL-agnostic: it owns an `embedded_io_async::Read + Write` UART and a
synchronous power `OutputPin`, feeds bytes through a pure decoder, and exposes
scans either one revolution at a time or aggregated over several revolutions.
It is one of the robot workspace's drivers, used by the firmware's core1 LiDAR
task; see the [robot page](../../../../docs/components/robot.md) for that side and
[architecture.md](../../../../docs/architecture.md) for the repo map.

## Public API

- **`Point`** — one LiDAR return: `angle_deg` (degrees), `distance_mm`
  (`Option<NonZeroU16>`, millimetres — `None` means no return), `intensity` (0–255).
- **`Scan<const N = 512>`** — one revolution (native 400 points at 0.9°, with
  512-point capacity), plus `len`.
- **`Reduction<const BUCKETS>`** — a by-angle reduction's output: up to `BUCKETS`
  fused buckets with its own bucket count, independent of the consumed scans'
  capacity, plus `len`.
- **`Decoder`** — pure byte-stream decoder (checksum, `AA 55` header resync,
  ring-start delimiting, angle interpolation).
- **`aggregate`** — fuse several scans by angle with a validity gate and a
  median/mean reducer, returning a `Reduction<BUCKETS>`.
- **`angle_correction_deg`** — the vendor's distance-dependent angle correction.
- **`Warmup`** — pure warm-up settle state machine (settle/plateau/exhaust from
  per-revolution point counts), configured by **`WarmupConfig`** and reporting a
  **`WarmupOutcome`**.
- **`NATIVE_POINTS`** — the steady-state point count per revolution (400).
- **`CoinD6`** — owns the UART and power pin; lifecycle is `power_on` →
  `start` → `warm_up` (optional) → `read_scan`/`read_aggregated` → `stop` →
  `power_off` (or `release` to hand the peripherals back).

The firmware runs a bounded best-effort `probe()` before `start()`: the
device-info frame is emitted on power-up, before the firmware's UART exists, so
it can be missed; a missing frame is logged and tolerated, and warm-up is the
proof that data flows.

## Wiring

The standalone example uses the breadboard pin map:

| Signal | RP2350 pin |
|--------|-----------|
| LiDAR TX → MCU RX | GPIO 13 |
| LiDAR RX ← MCU TX | GPIO 12 |
| Power MOSFET gate (active-high) | GPIO 15 |
| GND | GND |
| +5 V | 5 V rail |

The robot board uses the `J7` connector and the TPS22917 branch:

| Signal | Board |
|--------|-------|
| MCU TX → sensor RX | GPIO 28 → `J7.3` |
| MCU RX ← sensor TX | GPIO 29 ← `J7.4` |
| TPS22917 enable (active high) | GPIO 30 |
| Switched 5 V | `J7.1` |
| GND | `J7.2` |

A swapped RX/TX pair leaves the module powered but silent. The robot's measured
baseline is about 320 mA with the LiDAR spinning and the remote link up; use a
bench supply limit of at least 500 mA, because a 300 mA limit prevents the rotor
from starting.

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
`robot/crates/drivers/coin-d6/.cargo/config.toml`:

```sh
cargo run -p coin-d6 --example example --release
```
