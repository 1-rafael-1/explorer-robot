# Expansion UART connector, with the AI camera moved to PIO

The RP2350 has two hardware UARTs and both are committed: **UART0** carries the
COIN-D6 LiDAR (GPIO 12/1) and **UART1** is reserved for the Grove Vision AI V2
(GPIO 4/5). We want a connector for a second MCU — initially a Wi-Fi/BLE module —
that can be added later without re-spinning the board. That requires freeing a
UART.

The camera is the right device to move. It reports only bounding-box results at
30 Hz — four coordinates at 320×240, a confidence and a class, plus framing — on
the order of a few hundred bytes per frame, or roughly 1–2 KB/s at typical
detection counts. That is comfortably below 115200 baud, and the link is idle most
of the time. A PIO UART is therefore sufficient for the camera, while the
expansion MCU gets the hardware UART with DMA and buffering.

**Status:** accepted. ADR-0022 revised the pin choices (camera GPIO 23/24/25,
expansion UART1 GPIO 33–37). The camera's PIO0 transport is implemented; the
expansion connector is still absent from the schematic and PCB.

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
  expansion board's own regulator through its 3.3 V output. The 5 V rail has
  headroom and lets the expansion board regulate its own 3.3 V.
- **Power the expansion MCU from `VM` (the battery/motor rail).** Rejected: 6–8.4 V,
  unregulated and noisy from the motor drive. The expansion MCU expects a stable
  5 V input.
- **Add a low-side power switch on the expansion connector's ground.** Rejected:
  with the module unpowered, the 3.3 V UART lines would back-power it through its
  ESD diodes. If power switching is added later, it must be a **high-side** load
  switch on 5 V.
- **A 4-pin connector carrying only 5 V, GND, TX and RX.** Rejected: it leaves no
  way to reset or force download mode, and no interrupt line for event-driven
  operation. A 7-pin connector costs almost nothing and avoids a connector re-spin.

**Decision**

*Camera — move to PIO, keep the same pins:*

- Drive the AI camera from a **PIO UART on PIO0**, using two state machines (one
  `PioUartTx`, one `PioUartRx`) from `embassy-rp`'s `pio_programs::uart`. PIO1
  remains the RGB LED's block (SM 0–2); PIO2 stays free.
- Keep the camera's physical wiring on **GPIO 4 (TX) / GPIO 5 (RX)**; only the
  peripheral behind those pins changes. The camera's power MOSFET stays on
  **GPIO 27**.
- Run the camera UART at **115200 8N1**, matching the current configuration.

*Expansion — UART1 on new pins:*

- Move **UART1** to **GPIO 6 (TX) / GPIO 7 (RX)** for the expansion connector.
  Both pins are unused in the current firmware and are valid UART1 pins on the
  RP2350B.
- Keep UART1's DMA channels (`DMA_CH4`/`DMA_CH5`) and `UART1_IRQ`, now owned by the
  expansion UART rather than the camera. Default baud **115200 8N1**; the AT
  client may raise it after testing.

*Expansion connector — electrical contract:*

- Use a **7-pin keyed connector** with this fixed pinout:

  | Pin | Net | Direction | Notes |
  |---|---|---|---|
  | 1 | `EXP_5V` | Power out | From the TPS5430 5 V rail |
  | 2 | `EXP_GND` | Ground | |
  | 3 | `EXP_TX` | RP2350 → expansion | UART1 TX, 3.3 V logic |
  | 4 | `EXP_RX` | Expansion → RP2350 | UART1 RX, 3.3 V logic |
  | 5 | `EXP_EN` | RP2350 → expansion | Reset/enable; GPIO 28 |
  | 6 | `EXP_BOOT` | RP2350 → expansion | Download-mode select; GPIO 29 |
  | 7 | `EXP_IRQ` | Expansion → RP2350 | Interrupt/status; GPIO 35 |

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
- **5 V is always-on in this revision.** No load switch: the expansion MCU's `EN`
  line is the recovery mechanism. If a switch is added later, it must be high-side.
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
  instead of `Uart`.
- `bind_interrupts!` replaces the camera's `UART1_IRQ` handler with
  `PIO0_IRQ_0 => PioInterruptHandler<PIO0>`; `UART1_IRQ` moves to the expansion
  UART.
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
  own 3.3 V. The total 5 V load (expansion board plus the `AMS1117-3.3`) must stay
  within the TPS5430's rating.
- Because the connector is always-on, an unpowered or absent expansion board leaves
  `EXP_TX` driving 3.3 V into an unpowered input; the 100 Ω series resistor bounds
  the back-feed current. Do not hot-plug.
- `CONTEXT.md` and `README.md` currently describe the AI Cam as UART1 and list no
  expansion connector. Both must be updated when this ADR is implemented; until
  then they describe the present arrangement, not this decision.
- If the camera is ever changed to stream images rather than bounding boxes, the
  PIO UART is no longer adequate — that is a different transport decision.
- Pins to keep clear at layout: GPIO 4/5 remain the camera's PIO pins, GPIO 6/7
  become UART1, GPIO 28/29/35 are the expansion control lines, and GPIO 23/24/25
  remain the rangefinder reserve from ADR-0009. GPIO 30–34 are free after ADR-0008
  moved the TFT and touch to 38–47, but are deliberately left unused here to avoid
  confusion with ADR-0003's superseded pin assignment. Confirm all of this against
  the PCB netlist before routing.
- Open items: confirm the chosen expansion board's 5 V input range, peak current and
  UART logic level; confirm the TPS5430 5 V budget with the `AMS1117-3.3` load;
  confirm whether the camera module's baud is configurable; and measure the PIO
  UART's FIFO margin under display/SPI load during bring-up.
