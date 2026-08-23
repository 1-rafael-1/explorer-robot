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

Run one with (from the repository root):

```sh
cargo run -p hardware-tests --example lidar_tft_radar --release
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
| TFT SCK | GPIO 18 |
| TFT MOSI | GPIO 19 |
| TFT CS | GPIO 17 |
| TFT DC/RS | GPIO 26 |
| TFT RST | GPIO 27 |
| TFT BLK | GPIO 20 |

> The LiDAR power gate is **GPIO 15** here (the standalone-example pin), not the
> robot firmware's GPIO 26 — GPIO 26 is the TFT DC pin on this wiring.
