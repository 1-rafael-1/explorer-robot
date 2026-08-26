# ST7789 TFT on dedicated SPI1, replacing the SSD1306 OLED (RP2350B variant)

The SSD1306 128×64 OLED was replaced with a 240×240 ST7789 TFT driven by the
hand-rolled `st7789-async` driver (ADR-0001). The TFT needs a dedicated
write-only SPI bus plus five GPIOs, which the RP2350A pin budget could not
provide without collisions against the IMU, I2C0, LiDAR, and AI-cam pins — so
the firmware moved to the RP2350B variant for GPIO headroom.

**Status:** accepted

**Decision**

- Move the main crate to the RP2350B variant (`rp235xb`) for GPIO headroom;
  `hardware-tests` and the `st7789-async` examples stay on `rp235xa`.
- Drive the TFT over a dedicated, write-only **SPI1** (`Spi::new_txonly`,
  64 MHz), leaving the IMU untouched on SPI0 and I2C0 free for the VL53L0X only.
- The display module has **no CS pin**, so there is no `ExclusiveDevice` and no
  `embedded-hal-bus` dependency: the bare `Spi` is wrapped in an
  `embassy_sync::Mutex` (the IMU's pattern) to satisfy `SpiDevice`.
- Pin assignment (all RP2350B-only, no collision with the 0–28 map):
  SCK=GPIO30, MOSI=GPIO31, DC=GPIO32, RST=GPIO33, BLK=GPIO34. SPI1 TX on
  DMA_CH6.
- Keep the display seam: the `DisplayAction` channel and `display` task contract
  are unchanged; only the driver internals move from I2C/SSD1306/`BinaryColor`
  to SPI1/ST7789/`Rgb565`.
- Text-only, monochrome white-on-black for now; `TextStyle` Normal/Bold maps to
  `FONT_9X15` / `FONT_9X15_BOLD`.
- Typography: 9×15 font, 17 px/line → 14 lines × 26 chars
  (`DISPLAY_LINES = 14`, `MAX_LINE_LEN = 26`).
- Retain the scrolling logic (scroll-offset state, `max_system_info_scroll`,
  menu windowing) even though every screen now fits; it stays dormant and
  future-proofs longer lists.
- The display task owns BLK (backlight on) and RST (hardware reset sequence)
  before driver `init`.

**Consequences**

- The framebuffer is 115,200 bytes of static `.bss` (`ConstStaticCell`), fine
  against the 520 KB SRAM; no `memory.x` change.
- Full-frame `flush()` is ~15 ms at 64 MHz — acceptable for a menu, so the
  partial-redraw path from the driver example is not needed.
- Orientation defaults to `Deg0` with `Bgr` colour order; the colour order is
  panel-specific and is confirmed on first bring-up.
