# Touch controller on the shared display SPI bus

The 2.8″ panel's resistive touch layer is being added as a secondary input
alongside the rotary encoder. Its controller has to share one SPI bus with the
ST7789 display, but the two devices want very different clock speeds — 64 MHz
for the display and ~200 kHz for the touch controller — so a single bus must be
arbitrated and reconfigured per device. This is developed and proven on the
bench only; the robot's own display bus is left untouched.

**Status:** accepted

**Considered Options**

- **Hand-rolled arbiter:** a bespoke lock/select/reconfigure routine around the
  bare `Spi`. Dropped in favour of the proven library device, which already
  handles cancellation and flush ordering.
- **`SpiDeviceWithConfig`:** `embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig`
  over an async `Mutex`, one handle per device. Chosen.

**Decision**

- Per-device configuration is mandatory, not optional: the display runs at
  64 MHz and the touch controller at ~200 kHz, so the bus must be reconfigured
  for whichever device is selected.
- Arbitration and reconfiguration come from
  `embassy_embedded_hal::shared_bus::asynch::spi::SpiDeviceWithConfig` rather
  than a hand-rolled arbiter. Each transaction locks the async bus `Mutex`,
  applies the device's config through the HAL's `SetConfig`, asserts that
  device's chip select, and flushes the write. It is cancellation-safe, so an
  aborted await cannot leave the bus locked with a stale configuration.
- Both devices use **Mode 3** (`CaptureOnSecondTransition` + `IdleHigh`) and
  differ only in clock, so reconfiguration changes the frequency alone. Mode 0
  is the documented fallback if the panel misbehaves.
- This sits in tension with ADR-0003, which specified the robot display as a
  dedicated, **write-only** SPI bus with **no chip select**. Sharing a bus in
  the robot would instead require a full-duplex bus with a driven chip select,
  contradicting that decision. The robot migration is deliberately deferred and
  would reopen ADR-0003.
- Scope is bench-only: the `hardware-tests` coexistence example shares SPI0 on
  the rp235xa board. No robot firmware changed.
- The touch IC's markings are sanded off, so the XPT2046/TSC2046-class protocol
  is an assumption, not a verified fact. That includes the Z (pressure)
  channels, which the reference design never exercised; the assumption is
  confirmed empirically on the bench by the `touch_probe` example.
- Pre-integration check: the display is assumed ST7789-compatible because the
  existing async driver works on the test panel. A cheap 2.8″ panel can be an
  ILI9341 clone with a similar-but-not-identical init sequence, so the display
  controller identity must be confirmed before the robot display is migrated.
- Filtering boundary: the driver returns raw counts and does no filtering. The
  example applies the `moving_median` crate caller-side, and `moving_median` is
  **not** a dependency of the driver crate.

**Bring-up outcome (bench)**

- The `touch_probe` example confirmed the XPT2046/TSC2046-class framing on the
  bench: raw X/Y track the finger, and both Z channels respond (`z1 > 0` and
  `z2 > z1` on a touch), including `Z2`, which the reference design never read.
- The controller only asserts `PENIRQ` after a conversion has selected
  auto-power-down, so the driver performs one discarded read to arm it before
  waiting on the edge. Without that, `wait_for_touch` blocks forever on a cold
  start.
- `PENIRQ` is level-sensitive, not an edge per touch, so `wait_for_touch` waits
  on `wait_for_low` rather than a falling edge. After a low level that yields no
  valid sample it waits for the line to return high before awaiting the next low
  edge; a bare level wait would re-read immediately while the line stayed low
  (noise, or a release mid-read) and busy-spin SPI/CPU. A falling-edge wait is
  unusable here because the arming conversion is itself what drives the line low,
  so the edge would already have passed.
- The panel is RGB-ordered: `ColorOrder::Bgr` (as used by the LiDAR example)
  swaps red and blue on this panel, so the coexistence example uses `Rgb`.
- Calibration measured from the four coexistence corner targets is recorded as
  `Calibration::MEASURED`: raw endpoints `x1=3810, x2=160, y1=276, y2=3844` for
  the 320 × 240 framebuffer in the coexistence orientation. The measurement
  procedure is documented in the `hardware-tests` README.

**Consequences**

- The touch driver stays HAL-agnostic and depends only on an async `SpiDevice`,
  so it is equally usable on a dedicated or a shared bus.
- The `touch_coexistence` example proves both devices arbitrate cleanly on one
  bus, which is the evidence the deferred robot migration will build on.
- The `Calibration::default` constants are reference-only values from the
  vendor's board, not our panel's calibration; for this panel they have been
  superseded by `Calibration::MEASURED`, measured on the bench.
- Migrating the robot to this panel would reopen ADR-0003 and is a separate
  follow-up, outside this effort.
