# Panel connector: switched 5 V and protected signal harness

The 2.8″ ST7789 panel and its resistive touch layer leave the carrier board on
two keyed connectors instead of one 14-pin header: a 5-pin bus/power connector
(`PANEL.BUS`) carrying 5 V, GND and the shared `SPI1_SCK`/`SPI1_MOSI`/`SPI1_MISO`
lines, and a 6-pin control connector (`PANEL.CTRL`) carrying `LCD_CS`, `LCD_DC`,
`LCD_RST`, `LCD_BLK`, `TOUCH_CS` and `TOUCH_PENIRQ`. Splitting the harness keeps
the shared bus and its power return together and gives the control lines their
own keyed connector, so a mis-plugged control cable cannot short the 5 V feed
onto a 3.3 V GPIO.

The panel is an external harness like the LiDAR, camera and expansion branches,
so it needs the same branch treatment: a high-side load switch on 5 V and series
resistors on the logic lines. It is not hot-pluggable.

**Status:** accepted. The topology and pin allocation are settled; the 0603 33 Ω
part selection and the load-switch rise time remain provisional pending bring-up.

**Considered Options**

- **Leave the panel 5 V permanently on.** Rejected: it gives no way to recover a
  latched touch controller or display without cutting the whole 5 V rail, and it
  forgoes the TPS22917's reverse-current blocking and QOD. It also breaks the
  pattern established by ADR-0010 and ADR-0021.
- **Control the panel's backlight instead of its 5 V branch.** Rejected: `LCD_BLK`
  is a display signal, not a branch switch. It does not remove the panel's
  quiescent load, does not isolate the harness, and cannot recover a wedged
  controller.
- **Use one 11-pin connector.** Rejected: the panel module's pinout is fixed and
  an 11-pin keyed connector is less common than the two JST-PH parts. Two
  connectors also separate the bus/power bundle from the control bundle.
- **Put the series resistors only on the MCU side.** Accepted as the placement
  guidance: they are drawn on this sheet but should be laid out near the MCU so
  they source-terminate the 64 MHz clock and bound fault current before it
  reaches the harness.
- **Use 33 Ω on the 64 MHz SPI lines.** Accepted for `SPI1_SCK`, `SPI1_MOSI`
  and `SPI1_MISO`. At 64 MHz the period is only 15.6 ns; 100 Ω into a 30–50 pF
  harness and panel load gives a 10–90 % rise time of roughly 9–14 ns, which
  consumes most of the timing margin. 33 Ω roughly halves that while still
  providing series damping and fault-current limiting. The five slow control
  lines keep 100 Ω; `TOUCH_PENIRQ` keeps 1 kΩ.
- **Add an ESD clamp array at the connector.** Deferred for the same reason as
  the expansion connector (ADR-0021): the series resistors bound the fault
  current, and the final ESD part is not selected yet. The connector is internal
  and not hot-pluggable.

**Decision**

- **Harness split.**
  - `PANEL.BUS` (`J11`, 5-pin JST-PH): 5 V, GND, `SPI1_SCK`, `SPI1_MOSI`,
    `SPI1_MISO`.
  - `PANEL.CTRL` (`J12`, 6-pin JST-PH): `LCD_CS`, `LCD_DC`, `LCD_RST`,
    `LCD_BLK`, `TOUCH_CS`, `TOUCH_PENIRQ`.
- **Switched 5 V.** A TPS22917 high-side load switch (`U11`) sits between the
  `+5V` rail and `PANEL.BUS` pin 1. `VIN` = `+5V`, `VOUT` → connector, `CT` tied
  to `VIN`, `QOD` returned to `VOUT` through 100 Ω, and 1 µF `CIN` and 1 µF
  `COUT`. The enable is `PANEL_PWR` on **GPIO 0**, active-high. GPIO 0–3 close
  the QFN-80 ring and are physically adjacent to GPIO 47, so GPIO 0 keeps the
  enable trace short even though it is outside the 41–47 display run.
- **Series resistors.** 33 Ω in series with each of the three shared SPI lines
  (`SPI1_SCK`, `SPI1_MOSI`, `SPI1_MISO`), 100 Ω on the five slow control lines
  (`LCD_CS`, `LCD_DC`, `LCD_RST`, `LCD_BLK`, `TOUCH_CS`), and 1 kΩ on
  `TOUCH_PENIRQ` (an MCU input). All use the 0603 footprint. The 100 Ω parts are
  the existing `0603WAF1000T5E` BOM line and the 1 kΩ is the existing
  `0603WAF1001T5E`; the 0603 33 Ω part is to be selected. The resistors bound the
  current that can flow through the panel's ESD diodes when the branch is off,
  damp the shared bus, and keep the 64 MHz clock edges fast enough for the panel's
  timing.
- **Power sequencing.** The panel task holds `PANEL_PWR` high before releasing
  the display reset. Before switching the branch off, the display and touch
  transactions must be stopped and the SPI pins left idle; the series resistors
  bound the residual back-feed current (ADR-0023).
- **Firmware.** `PanelResources` gains `power_enable: PIN_0`. `Panel` owns the
  enable as an `Output`, drives it high at construction, and re-asserts it before
  the first reset pulse.
- **Power budget.** The panel's 5 V branch joins the LiDAR, camera and expansion
  branches on the TPS5430 rail. Its backlight and module current must be included
  in the peak budget tracked by ADR-0019; the display is not powered from 3.3 V.

**Consequences**

- The panel is no longer permanently powered. A wedged display or touch
  controller can be recovered by cycling `PANEL_PWR`, and the panel cannot
  back-feed the 5 V rail while the branch is off.
- GPIO 0 leaves the spare pool; GPIO 1–3 and GPIO 31 remain spare.
- The 5 V branch's peak current joins the ADR-0019 budget. The ST7789 backlight
  is the dominant panel load and must be measured; the TPS5430's 6.0 V low-line
  margin already needs attention.
- The shared SPI lines use 33 Ω 0603 parts; the five control lines use 100 Ω and
  `TOUCH_PENIRQ` uses 1 kΩ. The 0603 33 Ω part is not yet selected, so its
  MPN/LCSC metadata is intentionally absent from the three resistors until a
  suitable JLCPCB part is chosen.
- No ESD clamp array is populated yet. The connector is internal and keyed; if the
  harness becomes user-accessible or the panel is frequently unplugged, add the
  clamp array at the connector as ADR-0021 proposes for expansion.
- Open items: select the 0603 33 Ω part; measure the panel's 5 V peak current and
  backlight inrush; confirm the display's SPI timing with the 33 Ω series parts at
  64 MHz.
