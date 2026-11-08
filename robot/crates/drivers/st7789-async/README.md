# st7789-async

A custom async driver for the ST7789 (and ST7789-compatible) TFT
controller. It's written against `embedded-hal-async` so it isn't tied to a
specific HAL: it owns the SPI bus and data/command pin, borrows a
caller-allocated framebuffer, and exposes an `embedded_graphics::DrawTarget`
plus an async `flush`.

## Examples

Both examples run on an RP2350 using a compatible display (SPI0, write-only). They store the framebuffer as **big-endian RGB565** so it
can be DMA'd to the panel verbatim.

| Example          | What it shows                                                                      |
|------------------|------------------------------------------------------------------------------------|
| `full_frame`     | Compose the whole 320×240 frame and flush it all every tick. Simplest, but bandwidth-bound. |
| `partial_redraw` | Track the changed rectangle and flush only that. Fast.                            |

Run one with:

```sh
cargo run -p st7789-async --example full_frame --release
cargo run -p st7789-async --example partial_redraw --release
```

Use `--release`: at `opt-level = 0` the framebuffer clear/draw is unusably slow.

## Wiring

| Signal | RP2350 pin |
|--------|-----------|
| SCK    | GPIO 18   |
| MOSI   | GPIO 19   |
| CS     | GPIO 17   |
| DC/RS  | GPIO 26   |
| RST    | GPIO 27   |
| BLK    | GPIO 20   |
