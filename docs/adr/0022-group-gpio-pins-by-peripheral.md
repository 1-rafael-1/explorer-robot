# Group RP2350B GPIOs by peripheral for a routable four-layer board

The board is a four-layer design: signals on the top and bottom layers, a solid ground
plane on layer 2, and a split 3.3 V / 5 V plane on layer 3. The RP2350B's QFN-80 pads run
around the package in GPIO order — GPIO 4–47 form one continuous ring (with power pads
interspersed) and GPIO 0–3 close the ring — so a peripheral that occupies a contiguous
GPIO run leaves the package as one bundle. The first schematic scattered the motor,
encoder and LiDAR signals across the low GPIOs (0, 1, 2, 3, 6–12, 26), interleaving them
with UART, LED and IMU functions. Routing those bundles to the motor driver, the LiDAR
connector and the IMU would force traces to cross each other on the two signal layers.

**Status:** accepted. Replaces the pin choices in the firmware and defines the intended
board pinout for the incomplete schematic. It revises the pin choices of ADR-0021 and
supersedes the GPIO 23–25 rangefinder reserve in ADR-0009.

**Current assignment**

The schematic and firmware had drifted apart: the board sheet did not yet carry the
camera, I2C, battery or display nets, and the IMU chip-select/INT pins differed by one.
This is the starting point, before the regrouping.

| GPIO | Schematic net | Firmware function |
|---:|---|---|
| 0 | `MOT.A.PWM` | left PWM, slice 0 |
| 1 | `LIDAR_TX` | UART0 RX (D6 TX) |
| 2 | `MOT.A.IN.2` | left backward |
| 3 | `MOT.B.PWM` | right PWM, slice 1 |
| 4 | — | AI cam UART1 TX |
| 5 | — | AI cam UART1 RX |
| 6 | `MOT.STBY` | standby |
| 7 | `MOT.A.L.ENC.A` | left encoder, slice 3 |
| 8 | `MOT.A.IN.1` | left forward |
| 9 | `MOT.B.R.ENC.A` | right encoder, slice 4 |
| 10 | `MOT.B.IN.1` | right forward |
| 11 | `MOT.B.IN.2` | right backward |
| 12 | `LIDAR_RX` | UART0 TX |
| 13–15 | `LED_R/G/B` | RGB LED, PIO1 |
| 16–17 | — | I2C0 SDA/SCL |
| 18 | `ICM-20948_SCLK` | SPI0 SCK |
| 19 | `ICM-20948_SDI` | SPI0 MOSI |
| 20 | `ICM-20948_SDO` | SPI0 MISO |
| 21 | `ICM-20948_CS` | SPI0 CS |
| 22 | `ICM-20948_INT` | not sampled |
| 26 | `LIDAR_PWR` | D6 power MOSFET |
| 27 | — | AI cam power MOSFET |
| 38–39 | — | touch CS / PENIRQ |
| 40 | — | battery ADC |
| 41–47 | — | display CS/SCK/MOSI/MISO/DC/RST/BLK |

**Decision**

Give every peripheral one contiguous GPIO run, in package order. The runs are chosen so
that every alternate-function constraint still holds.

| GPIO | Function | Constraint |
|---:|---|---|
| 0–3 | spare | UART0 pair 0/1, I2C0 pair 0/1 |
| 4 | `MOT.A.PWM` | slice 2, channel A |
| 5 | `MOT.A.IN.1` | direct output |
| 6 | `MOT.A.IN.2` | direct output |
| 7 | `MOT.B.PWM` | slice 3, channel B |
| 8 | `MOT.B.IN.1` | direct output |
| 9 | `MOT.A.L.ENC.A` | slice 4, channel B (PWM input mode needs B) |
| 10 | `MOT.B.IN.2` | direct output |
| 11 | `MOT.B.R.ENC.A` | slice 5, channel B (PWM input mode needs B) |
| 12 | `MOT.STBY` | direct output |
| 13–15 | RGB LED | PIO1, any GPIO |
| 16–17 | I2C0 SDA/SCL | valid I2C0 pair |
| 18 | IMU SPI0 SCK | SPI0 CLK |
| 19 | IMU SPI0 MOSI | SPI0 MOSI |
| 20 | IMU SPI0 MISO | SPI0 MISO |
| 21 | IMU SPI0 CS | SPI0 CS |
| 22 | IMU interrupt | any GPIO |
| 23 | AI cam power | direct output |
| 24 | `AICAM_RX` (MCU → cam) | PIO0 UART TX |
| 25 | `AICAM_TX` (cam → MCU) | PIO0 UART RX |
| 26 | I2C1 SDA | valid I2C1 pair, second rangefinder bus |
| 27 | I2C1 SCL | valid I2C1 pair, second rangefinder bus |
| 28 | LiDAR UART0 TX | UART0 TX |
| 29 | LiDAR UART0 RX | UART0 RX |
| 30 | LiDAR power | direct output |
| 31–32 | spare | |
| 33 | `EXP_IRQ` | any GPIO |
| 34 | `EXP_EN` | any GPIO |
| 35 | `EXP_BOOT` | any GPIO |
| 36 | `EXP_TX` | UART1 TX |
| 37 | `EXP_RX` | UART1 RX |
| 38 | touch CS | direct output |
| 39 | touch PENIRQ | any GPIO |
| 40 | battery ADC | ADC0 |
| 41 | display CS | direct output |
| 42 | `SPI1_SCK` | SPI1 CLK, shared display + touch |
| 43 | `SPI1_MOSI` | SPI1 MOSI, shared display + touch |
| 44 | `SPI1_MISO` | SPI1 MISO, shared display + touch |
| 45 | display DC | direct output |
| 46 | display RST | direct output |
| 47 | display BLK | direct output |

The resulting occupied runs are GPIO 4–27 (motor, RGB, I2C0, IMU, camera, I2C1),
28–30 (LiDAR), and 33–47 (expansion, touch, battery, display). The remaining pins
(0–3, 31–32) are spare.

**Consequences**

- The motor and encoder harness is one 9-pin run (GPIO 4–12) instead of pins scattered
  through the UART and LED groups. The two encoder inputs stay on odd GPIOs because
  `embassy-rp`'s `Pwm::new_input` binds to channel B.
- The LiDAR moves from GPIO 1/12/26 to the adjacent run 28/29/30, and the camera moves
  from 4/5/27 to 23/24/25. This frees GPIO 0–3 completely.
- The camera now runs on a **PIO0 UART** on GPIO 24 (MCU TX) / GPIO 25 (MCU RX), with the
  power gate on GPIO 23. Hardware UART1 is therefore free for the expansion connector on
  GPIO 36/37; the connector itself is still not implemented.
- AI-cam net names follow the LiDAR convention (sensor perspective): `AICAM_TX` is
  camera → MCU and lands on the PIO UART RX pin (GPIO 25); `AICAM_RX` is MCU → camera and
  lands on the PIO UART TX pin (GPIO 24).
- The schematic's `LIDAR_TX`/`LIDAR_RX` names are from the sensor's perspective:
  `LIDAR_TX` is sensor → MCU and lands on UART0 RX; `LIDAR_RX` is MCU → sensor and lands
  on UART0 TX.
- The camera's UART1 assignment is temporary. ADR-0021's architecture (camera on PIO0,
  expansion on UART1) still holds, but this ADR moves the camera wiring to GPIO 23/24/25
  and the expansion UART1 to GPIO 36/37. When the expansion connector is added, the
  camera moves to PIO0 on the same pins, so the board wiring does not change.
- The second rangefinder gets a dedicated I2C1 bus on GPIO 26/27. Two VL53L0X sensors
  can therefore both keep the default `0x29` address: each bus has its own address space
  and its own pull-ups. The firmware binds `I2C1_IRQ` and initialises the second bus
  separately from I2C0.
- GPIO 38–47 and the battery ADC on GPIO 40 are unchanged from ADR-0008. The touch
  header exposes five signals (`T_CLK`, `T_CS`, `T_DIN`, `T_DO`, `T_IRQ`), but `T_CLK`,
  `T_DIN` and `T_DO` are the shared SPI1 `SCK`/`MOSI`/`MISO` nets. Only `T_CS` (GPIO 38)
  and `T_IRQ`/`PENIRQ` (GPIO 39) are dedicated GPIOs. A separate touch-only SPI bus, as
  used by the `touch_probe` bench example, would need three more pins and is not the
  production panel wiring.
- At layout, keep the layer-3 3.3 V/5 V split out from under the SPI1 display bundle and
  the battery-ADC trace. Layer 2 is the only continuous reference plane, so the display
  clock/data and the ADC trace should cross the split as little as possible.
- The spare pins are not contiguous (0–3, 31–32). They are intentionally left
  as routing channels and for future point-to-point additions rather than being packed
  into one block and forcing a second long bundle across the MCU.
