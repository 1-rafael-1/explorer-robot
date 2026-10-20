# Expansion UART connector, with the AI camera moved to PIO

The RP2350 has two hardware UARTs and both are committed: **UART0** carries the
COIN-D6 LiDAR on GPIO 28/29, and **UART1** is reserved for the expansion
connector on GPIO 36/37. The Grove Vision AI V2 camera moved to a PIO0 UART on
GPIO 24/25 (ADR-0022), which frees UART1 for the expansion link. We want a
connector for a second MCU — initially a Wi-Fi/BLE module — that can be added
later without re-spinning the board.

The camera is the right device to move. It reports only bounding-box results at
30 Hz — four coordinates at 320×240, a confidence and a class, plus framing — on
the order of a few hundred bytes per frame, or roughly 1–2 KB/s at typical
detection counts. That is comfortably below 115200 baud, and the link is idle most
of the time. A PIO UART is therefore sufficient for the camera, while the
expansion MCU gets the hardware UART with DMA and buffering.

**Status:** accepted. ADR-0022 revised the pin choices (camera GPIO 23/24/25,
expansion GPIO 32–37). The expansion 5 V feed is now switched by a TPS22917
high-side load switch enabled by `EXP_PWR` on GPIO32, and the camera branch uses
the same switch on GPIO23. ADR-0025 moved the camera branch from 5 V to 3.3 V;
the expansion connector remains on 5 V. The camera's PIO0 transport is
implemented; the expansion connector is still absent from the schematic and PCB. Branch power-down
now requires UART quiesce, TX/RX tri-state and UART deinit (ADR-0023). The LiDAR
implementation is the reference sequence.

**Considered Options**

- **Keep the camera on UART1 and put the expansion MCU on a PIO UART.** Rejected
  as the default: the expansion link is a request/response AT-style protocol that
  benefits from the hardware UART's DMA ring buffer, whereas the camera's 30 Hz
  stream tolerates an occasional dropped frame and can resynchronise on the next
  header. Accepted as the fallback if the camera's PIO FIFO proves too tight under
  display load.
- **Use a third hardware UART.** Not available: the RP2350 has only UART0 and
  UART1. The "B" package adds GPIOs, not UARTs.
- **Use I2C or SPI for the expansion MCU.** Rejected: both require custom firmware
  on the expansion device. UART plus an AT-style firmware is the least-effort path
  and is what the candidate Wi-Fi/BLE modules already speak.
- **Power the expansion MCU from the existing 3V3 rail.** Rejected: it loads the
  `AMS1117-3.3` LDO with the module's peak current and risks back-feeding the
  expansion board's own regulator through its 3.3 V output. The 5 V rail is the
  intended supply and lets the expansion board regulate its own 3.3 V; its peak
  budget is tracked in ADR-0019.
- **Power the expansion MCU from `VM` (the battery/motor rail).** Rejected: 6–8.4 V,
  unregulated and noisy from the motor drive. The expansion MCU expects a stable
  5 V input.
- **Add a low-side power switch on the expansion connector's ground.** Rejected:
  with the module unpowered, the 3.3 V UART lines would back-power it through its
  ESD diodes. If power switching is added later, it must be a **high-side** load
  switch on 5 V. The later choice is the TPS22917, which adds true reverse current
  blocking and QOD; see the Decision.
- **A 4-pin connector carrying only 5 V, GND, TX and RX.** Rejected: it leaves no
  way to reset or force download mode, and no interrupt line for event-driven
  operation. A 7-pin connector costs almost nothing and avoids a connector re-spin.

**Decision**

*Camera — move to PIO on GPIO 24/25:*

- Drive the AI camera from a **PIO UART on PIO0**, using two state machines (one
  `PioUartTx`, one `PioUartRx`) from `embassy-rp`'s `pio_programs::uart`. PIO1
  remains the RGB LED's block (SM 0–2); PIO2 stays free.
- Keep the camera's physical wiring on **GPIO 24 (MCU TX) / GPIO 25 (MCU RX)**
  per ADR-0022; only the peripheral behind those pins changes. The camera's 5 V
  branch is switched high-side by a TPS22917 enabled by **GPIO 23**
  (`AICAM_PWR`), with `QOD` to `VOUT`, `CT` to `VIN`, and a pulldown on `ON`.
  The module's Grove input is 5 V and its UART is 3.3 V.
- Run the camera UART at **115200 8N1**, matching the current configuration.

*Expansion — UART1 on new pins:*

- Use **UART1** on **GPIO 36 (TX) / GPIO 37 (RX)** per ADR-0022. Both are valid
  UART1 pins on the RP2350B; the camera moved to PIO0.
- Keep UART1's DMA channels (`DMA_CH4`/`DMA_CH5`) and `UART1_IRQ`, now owned by the
  expansion UART rather than the camera. Default baud **115200 8N1**; the AT
  client may raise it after testing.

*Expansion connector — electrical contract:*

- Use a **7-pin keyed connector** with this fixed pinout:

  | Pin | Net | Direction | Notes |
  |---|---|---|---|
  | 1 | `EXP_5V` | Power out | From the TPS22917 output through the ferrite bead |
  | 2 | `EXP_GND` | Ground | |
  | 3 | `EXP_TX` | RP2350 → expansion | UART1 TX, GPIO 36, 3.3 V logic |
  | 4 | `EXP_RX` | Expansion → RP2350 | UART1 RX, GPIO 37, 3.3 V logic |
  | 5 | `EXP_EN` | RP2350 → expansion | Reset/enable; GPIO 34 |
  | 6 | `EXP_BOOT` | RP2350 → expansion | Download-mode select; GPIO 35 |
  | 7 | `EXP_IRQ` | Expansion → RP2350 | Interrupt/status; GPIO 33 |

  `EXP_PWR` (GPIO32) is the carrier-side TPS22917 enable; it is not a connector
  pin.

- The connector's logic is **3.3 V only**. Pin 1 is **5 V power**, not a 5 V logic
  reference. An expansion MCU with 5 V UART logic needs level shifting; do not wire
  it directly.
- The expansion board is expected to have **its own 3.3 V regulator**; the carrier
  does not provide 3.3 V on the connector.
- Take 5 V from the existing TPS5430 buck in
  `eda files/explorer-robot/power_and_batt.kicad_sch`. Add **10 µF + 100 nF** at the
  connector, and a **low-DCR ferrite bead** in series with the connector's 5 V feed
  (provisional part, rated above the expansion board's peak current) so Wi-Fi/BLE
  burst current does not inject noise into the 5 V rail that feeds the
  `AMS1117-3.3`.
- **5 V is switched by a TPS22917 high-side load switch.** `VIN` = `+5V`,
  `VOUT` → ferrite bead → `EXP_5V`; `ON` = `EXP_PWR` GPIO32 with a pulldown;
  `CT` to `VIN`; `QOD` to `VOUT`; 1 µF `CIN` and 1 µF `COUT`. The TPS22917's
  reverse current blocking prevents a USB-powered expansion board from
  back-feeding the 5 V rail. It has no current limit; the expansion board is not
  hot-pluggable, and the TPS5430/pack protection are the forward-fault backstop.
  Keep the 10 µF + 100 nF at the connector and the ferrite bead.
- **Off-state back-power handling.** Before deasserting `EXP_PWR`, stop UART1,
  tri-state `EXP_TX`, hold `EXP_EN` low and leave `EXP_BOOT` low/high-Z; then
  switch the 5 V branch off. On power-up, assert `EXP_PWR`, wait for the CT rise,
  then release `EXP_EN` and bring up UART1. QOD holds `EXP_5V` low, but it does
  not discharge the expansion board's own 3.3 V rail, so the 3.3 V lines must be
  tri-stated (ADR-0023). The 100 Ω/1 kΩ series resistors bound residual current.
- Protect the connector: **100 Ω series** on `EXP_TX`/`EXP_RX`, **1 kΩ series** on
  `EXP_EN`/`EXP_BOOT`/`EXP_IRQ`, and an **ESD clamp array** at the connector. The
  connector is **not hot-pluggable** — power down before connecting or removing an
  expansion board.
- Place the connector at the **board edge** so the expansion board's antenna has
  clearance and is not butted against copper, the battery, the motors, or a metal
  chassis.

*Firmware shape:*

- `AiCamResources` loses `uart`, `dma_tx` and `dma_rx`; it gains the PIO0 block, its two
  state machines, and the TX/RX pins. `init_ai_cam_uart` becomes a PIO UART
  bring-up; the camera driver consumes `PioUartTx`/`PioUartRx` (`embedded_io_async`)
  instead of `Uart`. Its `power_mosfet` field is renamed `power_enable`, because it
  now drives a TPS22917 enable rather than a low-side FET gate.
- `bind_interrupts!` replaces the camera's `UART1_IRQ` handler with
  `PIO0_IRQ_0 => PioInterruptHandler<PIO0>`; `UART1_IRQ` moves to the expansion
  UART.
- `ExpansionResources` gains `power_enable` on GPIO32 (`EXP_PWR`) for the 5 V
  branch switch. The expansion task owns UART1, the switch and the connector
  control lines, and sequences power/UART bring-up and teardown (ADR-0023).
- The LiDAR establishes the reference sequence (ADR-0023): stop, bounded flush and
  shifter drain, drop the buffered UART, release TX/RX, clear `UARTCR.EN`, then
  deassert `*_PWR`. The camera and expansion tasks must follow the same order.
- The expansion UART gets its own task (AT client plus timeouts/retries). The camera
  task must drain its PIO RX FIFO promptly (see Consequences).

**Consequences**

- UART1 is free for expansion, and the camera no longer competes for it. The camera
  protocol is still unimplemented, so this is the cheapest moment to change its
  transport.
- The PIO UART's RX FIFO is shallow: `FifoJoin::RxOnly` gives 8 bytes, about 0.7 ms
  of slack at 115200. The camera task must be dedicated/high-priority, or run on
  core1, or the camera baud must be lowered (the payload is small enough that even
  38400 would carry it). The frame parser must resynchronise on a header/checksum so
  a dropped frame is harmless.
- The expansion connector is a **5 V-powered, 3.3 V-logic** interface. It is not a
  general-purpose power tap: the expansion board must accept 5 V and regulate its
  own 3.3 V. The total 5 V load (expansion board, LiDAR, panel and the
  `AMS1117-3.3`, which carries the camera and the rest of the 3.3 V rail) must
  stay within the TPS5430's rating and the peak budget in ADR-0019.
- The connector's 5 V feed is now switched. With the switch off, QOD holds
  `EXP_5V` low. Before deasserting `EXP_PWR`, UART1 is quiesced and dropped,
  `EXP_TX` is tri-stated, `EXP_EN` is held low and `EXP_BOOT` is left low or
  high-Z, so the expansion board's 3.3 V logic lines cannot back-power it through
  ESD diodes. The switch's reverse current blocking also prevents a USB-powered
  expansion board from back-feeding the 5 V rail. The connector is still not
  hot-pluggable.
- The three switched 5 V branches share the TPS5430 rail; the AI camera's
  switched 3.3 V branch loads that rail through the `AMS1117-3.3` (ADR-0025).
  The summed peak (LiDAR unmeasured, ESP32-C6 ~250–350 mA, the camera's
  100–150 mA inside the LDO input, plus the rest of the 3.3 V rail) can approach
  or exceed 1 A; the 6.0 V low-line may not hold 5 V at that peak. ADR-0019 owns
  the budget; measure before relying on all three at once.
- `CONTEXT.md` and `README.md` describe the current arrangement; ADR-0022 is the
  authoritative pin table. Keep them in sync.
- If the camera is ever changed to stream images rather than bounding boxes, the
  PIO UART is no longer adequate — that is a different transport decision.
- Pins to keep clear at layout: camera GPIO 23/24/25, LiDAR GPIO 28/29/30,
  expansion `EXP_PWR` GPIO32, `EXP_IRQ` GPIO33, `EXP_EN` GPIO34, `EXP_BOOT`
  GPIO35, UART1 GPIO36/37; GPIO31 is the rear rangefinder INT (ADR-0026);
  touch 38/39; battery ADC 40; display 41–47. Confirm all of this against the PCB
  netlist before routing.
- Open items: confirm the chosen expansion board's 5 V input range, peak current and
  UART logic level; confirm the TPS5430 5 V budget with the `AMS1117-3.3`, LiDAR
  and camera loads; confirm the TPS22917's availability and QOD load-capacitance
  limit; confirm whether the camera module's baud is configurable; and measure the
  PIO UART's FIFO margin under display/SPI load during bring-up. Implement the
  camera's PIO UART release/tri-state and boot delay, and the expansion task's
  UART1/`EN`/`BOOT` sequencing, following ADR-0023.
