# Quiesce and tri-state a branch's UART before switching its power off

The LiDAR, AI camera and expansion connector are each on a switched 5 V branch
behind a TPS22917 high-side load switch (ADR-0010, ADR-0021). With the branch
unpowered but its 3.3 V UART and control lines still connected, the MCU can
back-power the load through its ESD diodes, drive a floating input, or leave a
UART transmitter idling high into an unpowered receiver. The TPS22917's reverse
current blocking stops current flowing from `VOUT` to `VIN`; it does not isolate
the 3.3 V logic lines from the MCU. QOD holds the switched 5 V rail low, but it
does not discharge the load's own 3.3 V rail.

This applies to all three branches, not just the LiDAR. The LiDAR task currently
keeps its `BufferedUart` alive for the whole runtime; that is no longer acceptable
once the 5 V branch is switched independently.

**Status:** accepted. Applies to the LiDAR, AI camera and expansion branches.

**Considered Options**

- **Keep the UART initialized and rely on QOD and reverse current blocking.**
  Rejected. QOD only discharges `VOUT`; the UART TX still idles high and can feed
  the load's 3.3 V rail through its ESD diode. Reverse current blocking is a
  `VOUT`-to-`VIN` feature, not a logic-line isolator.
- **Hold TX low with a UART break while the branch is off.** Rejected. It keeps
  the UART clocked, and a sustained low is a break/framing condition if the load
  is powered independently (for example, over USB).
- **Add series resistors only.** Rejected as the primary mechanism. The 100 Ω
  UART and 1 kΩ control-line resistors are still required, but they bound the
  back-feed current rather than preventing it.
- **Add a bus switch or level shifter with an enable.** Deferred. It would work,
  but it adds parts and board area; tri-stating the MCU pins plus the branch power
  sequence is sufficient.
- **Keep the UART static and just reconfigure the pin low.** Rejected. The
  `BufferedUart` owns the pins and has no `free()`; the clean path is to stop
  using it and reconstruct it for the next acquisition.

**Decision**

The common rule is:

- **Power-down:** stop the branch's protocol, quiesce and deinit its UART, then
  drive or tri-state the MCU-side logic lines so they cannot source current into
  the unpowered load. Only then deassert the branch's `*_PWR` enable.
- **Power-up:** assert the branch enable, wait for the TPS22917 `CT` rise, then
  bring up the UART and drive the control lines. Do not drive logic into a branch
  that is still unpowered.

Per branch:

- **LiDAR.** `release()` stops the device, calls `CoinD6::release()`, drops the
  `BufferedUart` (which deinits its ring buffers and IRQ), leaves the MCU TX pin
  high-Z with `Input::new(tx.reborrow(), Pull::None)`, then deasserts
  `LIDAR_PWR`. `acquire()` asserts `LIDAR_PWR`, waits for the switch rise, then
  reconstructs the `BufferedUart` from the retained `Peri` handles and runs
  `power_on()` → `start()` → warm-up. The UART TX/RX buffers are `'static mut`
  and are reborrowed for each construction. Optionally clear `UARTCR.EN` through
  the PAC; `BufferedUart::new` re-enables it.
- **AI camera.** Stop the PIO UART, deinit/drop the PIO state machines and
  transport, reconfigure GPIO 24/25 as inputs, then deassert `AICAM_PWR`.
  Power-up: assert `AICAM_PWR`, wait for the switch rise and the camera boot, then
  initialise the PIO UART.
- **Expansion.** Stop UART1, drop the buffered UART, tri-state `EXP_TX` (GPIO 36),
  hold `EXP_EN` (GPIO 34) low and leave `EXP_BOOT` (GPIO 35) low/high-Z, then
  deassert `EXP_PWR`. Power-up: assert `EXP_PWR`, wait for the `CT` rise, then
  release `EXP_EN` and bring up UART1.

The LiDAR task therefore retains the raw `Peri` handles for UART0 and its pins
and constructs the `BufferedUart` per acquisition. `CoinD6::release()` is the
hook that returns the UART and the power enable. The `BufferedUart` API has no
`free()`, but it does not retain the `Peri` values after construction, so
`Peri::reborrow()` lets the task keep the handles and pass fresh reborrows to
`BufferedUart::new`. Reborrow the `'static mut` TX/RX buffers the same way.

The hardware keeps the second layer: 100 Ω series on the UART lines, 1 kΩ on the
expansion `EN`/`BOOT`/`IRQ` lines (ADR-0021), the TPS22917 QOD, and its reverse
current blocking. Those reduce the fault current; they do not replace tri-stating
the 3.3 V logic.

**Consequences**

- The LiDAR UART is no longer a runtime-static resource. The task constructs it
  per acquisition and drops it on release; `LidarDriver` no longer owns a UART
  that outlives the powered branch.
- The camera and expansion tasks must own their UART/PIO lifecycle and their
  off-state pin configuration. Power-up and power-down sequencing becomes part of
  each branch's API rather than an incidental detail.
- The resource fields that used to be called `power_mosfet` are renamed
  `power_enable`: `D6LidarResources::power_enable` (GPIO 30) and
  `AiCamResources::power_enable` (GPIO 23). `ExpansionResources` gains
  `power_enable` on GPIO 32 (`EXP_PWR`).
- A branch that is powered but whose UART has not yet been brought up is a valid
  state; the task must not assume the UART exists immediately after `power_on`.
- A branch that is powered down must not have its UART TX driven high, and must
  not have a low driven into a load that is independently powered.
- Bench check: with the branch off, measure no current from the MCU TX/control
  pins into the load's 3.3 V rail; with the branch on, verify the UART starts
  cleanly after the switch rise and that a power cycle does not leave a stuck
  framing/overrun condition.

**Open items**

- Implement the LiDAR per-acquisition UART refactor and verify that
  `Peri::reborrow()` plus `BufferedUart::new` behaves correctly on the
  `embassy-rp` version in use.
- Confirm that the camera's PIO UART can release or safely tri-state GPIO 24/25
  when the branch is off.
- Confirm the expansion board's `EN`/`BOOT` pull-ups and whether they float high
  when its 5 V is off; hold or tri-state them accordingly.
- Measure the off-state back-feed on each branch before and after the refactor.
