# Graphics TFT and touch on the robot's shared SPI1 bus

The robot's display moves from a dedicated, write-only SPI1 bus with no chip select
(ADR-0003) to a shared, full-duplex SPI1 bus carrying both the ST7789 panel and the
resistive touch controller, arbitrated with `SpiDeviceWithConfig` so each device locks the
bus, applies its own clock, and asserts its own chip select. This reopens ADR-0003 and
completes the robot migration ADR-0004 deliberately deferred; the bench-only scope of
both is retired.

**Status:** accepted. Supersedes the display-bus and text-only parts of ADR-0003.

**Considered Options**

- **Keep the dedicated write-only display bus and give touch its own SPI bus.** Rejected:
  the panel module shares one set of SPI pins between its display and touch layers, and a
  second bus costs five pins against a robot that has to keep expansion room.
- **Move the panel to SPI0's high quad and the IMU to SPI1.** Rejected: SPI0's high pins
  are a clean full-duplex group, but the IMU's move would collide with the motor, RGB and
  LiDAR pins and force a bigger reshuffle for no gain.

**Decision**

- One full-duplex SPI1 carries both devices in Mode 3, differing only in clock: 64 MHz for
  the display, ~200 kHz for the touch controller. Per-device configuration is mandatory,
  not an optimisation.
- Arbitration comes from `SpiDeviceWithConfig`, as proven by `touch_coexistence`, rather
  than a hand-rolled arbiter.
- The bus mutex is `CriticalSectionRawMutex`. The bench examples used `NoopRawMutex`, which
  is only sound for a single executor and priority.
- Pins, chosen fresh because nothing was wired yet: SPI1 SCK/MOSI/MISO = GPIO 42/43/44,
  TFT CS/DC/RST/BLK = 41/45/46/47, touch CS/IRQ = 38/39, with the battery ADC staying on
  GPIO 40 and the LiDAR and I2C0 assignments unchanged.
- The display's 240 × 240 text-mode contract is retired with the bus decision; the panel
  becomes a graphics device owned by one task.

**Consequences**

- SPI1 MISO is forced to GPIO 44: the only MISO-capable SPI1 pins are GPIO 40 and 44, and
  40 is the battery ADC.
- The bus needs a receive DMA channel, so `DMA_CH7` joins `DMA_CH6` in the `DMA_IRQ_0`
  binding group.
- The display no longer has a CS pin, so `embedded-hal-bus` enters the firmware's
  dependency set and the `NoCsSpiDevice` adapter is deleted.
- The former display pins (30–34) and the rotary footprint (22–24) return to the spare
  pool.
- The panel's touch pins and its display pins are one bus, so a panel fault and a touch
  fault are no longer independent.
