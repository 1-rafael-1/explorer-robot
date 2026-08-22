# Async ST7789 display driver, hand-rolled instead of mipidsi

The SSD1306 OLED is being replaced with an ST7789 TFT over SPI. We chose to
hand-roll an async driver — own the SPI bus and data/command pin, borrow a
caller-allocated framebuffer, and expose `embedded_graphics::DrawTarget` plus an
async `flush` — rather than use `mipidsi`, which is blocking and write-through.
Async redraw needs a framebuffer + `flush()` shape, and
`embedded_graphics::DrawTarget::draw_iter` is synchronous (you cannot `.await`
inside it), so mipidsi's write-through `DrawTarget` is the wrong architecture.
mipidsi's DCS init sequence and orientation mapping remain the reference.

**Status:** accepted

**Considered Options**

- **mipidsi (blocking):** proven init sequence, but no async and write-through. Dropped.
- **`mipidsi-async`:** an empty stub (`// TODO`), so not a real option.
- **Hand-rolled async driver:** owns `embedded-hal-async` SPI and the DC pin,
  borrows a caller-allocated framebuffer, implements `DrawTarget` and an async
  `flush`; DCS init copied from mipidsi as reference. Chosen.

**Consequences**

- The driver lives in the `st7789-async/` workspace member, decoupled from
  `embassy-rp`, so it can be bench-tested in isolation and reused later.
- Pixels are encoded big-endian RGB565 into the framebuffer by the driver's
  `DrawTarget` implementation, so `flush` can DMA the buffer verbatim.
