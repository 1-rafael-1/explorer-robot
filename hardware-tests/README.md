# hardware-tests

Standalone, breadboard-scale hardware tests for the explorer-robot sensors.
Each test is a self-contained embassy binary under `examples/` that wires one or
two sensors directly to the RP2350, independently of the main robot firmware.
This is where small "does the hardware work" checks live; the in-firmware
`task/testmode/` tasks remain the place for procedures that run inside the robot.

## Examples

| Example              | What it shows                                                                                          |
|----------------------|--------------------------------------------------------------------------------------------------------|
| `lidar_tft_radar`    | Top-down radar: 1–5 m white range rings centred on the LiDAR, one red cross per valid return at its angle/range. |
| `touch_probe`        | Raw XPT2046/TSC2046-class samples alone on SPI0: logs every sample's `x`/`y`/`z1`/`z2` and the raw extents per corner, to confirm the protocol and seed calibration. |
| `touch_coexistence`  | ST7789 TFT and touch controller sharing one full-duplex SPI0 bus: draws the moving-median-filtered, calibrated touch point, with four coloured corner targets for calibration. |
| `touch_menu`         | Touch-driven mock of the robot's real menu tree on the 2.8″ panel: Main Menu → submenus → placeholder leaves, a mocked System Info screen, and draggable value entry. |

Run one with (from the repository root):

```sh
cargo run -p hardware-tests --example lidar_tft_radar --release
cargo run -p hardware-tests --example touch_menu --release
```

Use `--release`: at `opt-level = 0` the framebuffer clear/draw is unusably slow.

## Wiring

The examples use the same pins as the standalone `coin-d6` and `st7789-async`
examples (which do not collide):

| Signal | RP2350 pin |
|--------|-----------|
| LiDAR RX ← MCU TX | GPIO 12 |
| LiDAR TX → MCU RX | GPIO 13 |
| LiDAR power MOSFET gate (active-high) | GPIO 15 |
| SPI SCK (TFT + touch, shared) | GPIO 18 |
| SPI MOSI (TFT + touch, shared) | GPIO 19 |
| SPI MISO (TFT + touch, shared) | GPIO 16 |
| TFT CS | GPIO 17 |
| TFT DC/RS | GPIO 26 |
| TFT RST | GPIO 27 |
| TFT BLK | GPIO 20 |
| Touch CS | GPIO 21 |
| Touch IRQ (PENIRQ, active-low) | GPIO 22 |

> The LiDAR power gate is **GPIO 15** here (the standalone-example pin), not the
> robot firmware's GPIO 26 — GPIO 26 is the TFT DC pin on this wiring.

> Panel notes (2.8″ 240×320, measured on the bench): the display is
> **RGB-ordered** (`ColorOrder::Bgr`, as the LiDAR example uses, swaps red and
> blue), and the touch calibration measured from the coexistence corner targets
> is recorded as `touch_async::Calibration::MEASURED`.

## Calibrating the touch panel

Touch calibration maps raw counts onto the 320 × 240 framebuffer.
`touch_coexistence` draws four coloured targets at known framebuffer
coordinates, inset 12 px from the edges, so a raw reading can be paired with an
exact pixel:

| Target | Colour | Framebuffer |
|--------|--------|-------------|
| top-left | red | (12, 12) |
| top-right | green | (307, 12) |
| bottom-right | blue | (307, 227) |
| bottom-left | yellow | (12, 227) |

To (re-)measure the panel:

1. Flash `touch_coexistence` and, for each target, press and hold it, noting the
   logged `raw x=… y=…` (ignore the `pixel` values while calibrating).
2. Average the readings per edge: `raw_x_left` from red+yellow, `raw_x_right`
   from green+blue, `raw_y_top` from red+green, `raw_y_bottom` from blue+yellow.
3. Extrapolate the 12 px inset out to the framebuffer edges, with `W = 320`,
   `H = 240`, `INSET = 12`, `FAR_X = W-1-INSET = 307`, `FAR_Y = H-1-INSET = 227`:

   ```text
   x1 = raw_x_left - INSET * (raw_x_right  - raw_x_left)  / (FAR_X - INSET)
   x2 = x1 + (W - 1) * (raw_x_right - raw_x_left) / (FAR_X - INSET)

   y1 = raw_y_top  - INSET * (raw_y_bottom - raw_y_top)   / (FAR_Y - INSET)
   y2 = y1 + (H - 1) * (raw_y_bottom - raw_y_top) / (FAR_Y - INSET)
   ```

4. Record the four endpoints as `touch_async::Calibration::MEASURED` in
   `touch-async/src/calibration.rs`.

`Calibration::REFERENCE` keeps the vendor board's uncalibrated values and is what
`Calibration::default()` returns; `touch_coexistence` uses `Calibration::MEASURED`.
