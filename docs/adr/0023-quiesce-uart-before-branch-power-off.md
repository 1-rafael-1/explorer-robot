# Quiesce and tri-state a branch's UART before switching its power off

The LiDAR, AI camera and expansion connector are each on a switched 5 V branch
behind a TPS22917 high-side load switch (ADR-0010, ADR-0021). With the branch
unpowered but its 3.3 V UART and control lines still connected, the MCU can
back-power the load through its ESD diodes, drive a floating input, or leave a
UART transmitter idling high into an unpowered receiver. The TPS22917's reverse
current blocking stops current flowing from `VOUT` to `VIN`; it does not isolate
the 3.3 V logic lines from the MCU. QOD holds the switched 5 V rail low, but it
does not discharge the load's own 3.3 V rail.

The requirement is the off-state electrical invariant, not the existence of a
Rust object: with the branch unpowered, no MCU-driven line may source current
into it. Quiescing, deinitializing and dropping the UART is the mechanism that
makes the invariant hold. Dropping `BufferedUart` alone does not: it deinits only
its ring buffers and IRQ, leaving `UARTCR.EN` set and the UART pins muxed.

**Status:** accepted. Applies to the LiDAR, AI camera and expansion branches.
The LiDAR is implemented per this ADR; the camera and expansion tasks follow the
same pattern when they gain a runtime lifecycle.

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
- **Keep the UART object alive and re-mux the pins manually.** Rejected. The
  `BufferedUart` does not retain the `Peri` handles after construction, so the
  task can retain them and reborrow, but re-muxing a pin into the UART function
  would duplicate embassy's private `Uart::init` or poke `FUNCSEL` directly.
  Constructing the UART per powered period is the supported path.
- **Keep the UART static and just reconfigure the pin low.** Rejected. A driven
  low is a break condition, and it does not deinit the UART. The clean path is to
  drop the `BufferedUart`, clear `UARTCR.EN`, release the pads, and reconstruct
  for the next acquisition.

**Decision**

The common rule is:

- **Power-down:** stop the branch's protocol, quiesce and deinit its UART, then
  drive or tri-state the MCU-side logic lines so they cannot source current into
  the unpowered load. Only then deassert the branch's `*_PWR` enable.
- **Power-up:** assert the branch enable, wait for the TPS22917 `CT` rise, then
  bring up the UART and drive the control lines. Do not drive logic into a branch
  that is still unpowered.

For the LiDAR, the sequence is:

1. Send the stop command with a bounded timeout (best-effort).
2. `CoinD6::release()` to recover the buffered UART and the power enable.
3. Wait, bounded, for `BufferedUart::busy()` to clear. `flush()` only drains the
   software ring; `BUSY` covers the hardware shifter, so without this wait the
   stop command can be truncated.
4. Drop the `BufferedUart`; this deinits its ring buffers and disables
   `UART0_IRQ`.
5. Release both UART pads with `Input::new(pin, Pull::None)`, dropped
   immediately. This leaves TX and RX high-Z with no pull.
6. Clear `UARTCR.EN`, `TXE` and `RXE` through the PAC.
7. Deassert `LIDAR_PWR`.
8. Record the power-down instant so the next power-up can enforce a minimum off
   interval.

Power-up reverses it: wait out the minimum off interval, assert `LIDAR_PWR`,
wait `BRANCH_RISE` (50 ms provisional, pending the TPS22917 `CT` value), then
construct the `BufferedUart` from the retained `Peri` handles and static buffers,
construct the driver, and run `probe()` → `start()` → warm-up. `probe()` is the
public device-info read; `power_on()` remains `set_high()` + `probe()` for the
simple case.

Per branch:

- **LiDAR.** The task retains the raw `Peri` handles for UART0, GPIO28 and
  GPIO29, plus the `'static mut` TX/RX buffers and the ingest buffer. A
  `LidarSession` owns those resources, one `Output<'static>` for `LIDAR_PWR`
  (held low while off), and the sequencing. It does not store the driver: the
  driver borrows the ingest buffer only for the powered period, which avoids a
  self-referential struct. `power_up(&mut ingest)` returns the driver;
  `power_down(driver)` consumes it and runs the sequence above. Every path that
  deasserts power — acquire failure, retry, streaming restart, terminal failure,
  last lease return — goes through `power_down`.
- **AI camera.** Stop the PIO UART, deinit/drop the PIO state machines and
  transport, reconfigure GPIO 24/25 as inputs, then deassert `AICAM_PWR`.
  Power-up: assert `AICAM_PWR`, wait for the switch rise and the camera boot, then
  initialise the PIO UART.
- **Expansion.** Stop UART1, drop the buffered UART, tri-state `EXP_TX` (GPIO 36),
  hold `EXP_EN` (GPIO 34) low and leave `EXP_BOOT` (GPIO 35) low/high-Z, then
  deassert `EXP_PWR`. Power-up: assert `EXP_PWR`, wait for the `CT` rise, then
  release `EXP_EN` and bring up UART1.

The task-side ownership API is `acquire() -> Lease` and `return_lease(lease)`.
`return_lease` names the ownership transfer; `CoinD6::release()` keeps its
standard driver meaning of handing the UART and power pin back. The task, not
the driver, owns the branch sequencing because only the task knows how to
tri-state the pads and clear `UARTCR.EN`.

`BufferedUart::new` accepts `Peri<'d, ...>` and `&'d mut [u8]` and returns a
`BufferedUart` with no lifetime parameter, so retaining the `Peri` handles and
reborrowing them per construction is supported. The TX/RX buffers are
`'static mut` and are reborrowed the same way. `BufferedUart`'s drop deinits its
ring buffers and disables its IRQ but does not clear `UARTCR.EN` or reset the pad
mux; the explicit `Input` release and PAC clear are therefore required.

The hardware keeps the second layer: 100 Ω series on the UART lines, 1 kΩ on the
expansion `EN`/`BOOT`/`IRQ` lines (ADR-0021), the TPS22917 QOD, and its reverse
current blocking. Those reduce the fault current; they do not replace tri-stating
the 3.3 V logic.

**Consequences**

- The LiDAR UART is no longer a runtime-static resource. The task constructs it
  per powered period and drops it on power-down; `LidarDriver` no longer owns a
  UART that outlives the powered branch.
- The LiDAR task retains the `Peri` handles and the `'static mut` buffers, and
  the driver exists only inside `power_up`/`power_down`. The ingest buffer is
  borrowed per acquisition; the session does not store the driver.
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
- `CoinD6::release()` remains a pure handback. The task-side operation is
  `return_lease(lease)`; the two names no longer collide in prose.
- The power-down path is bounded: `STOP_TIMEOUT` (100 ms) bounds the stop,
  `BUSY_TIMEOUT` (10 ms) bounds the shifter drain, and `POWER_CYCLE_OFF` (200 ms)
  is the provisional minimum off interval between power-ups.
- The attempt timeout wraps `probe` → `start` → warm-up, not power assertion or
  UART construction. There is no await point between `BufferedUart::new` and
  returning the driver, so cancellation cannot leave a muxed UART with the
  branch powering down.
- Bench check: with the branch off, measure no current from the MCU TX/RX and
  control pins into the load's 3.3 V rail; with the branch on, verify the UART
  starts cleanly after the switch rise and that a power cycle does not leave a
  stuck framing/overrun condition. The board is not manufactured yet, so this
  measurement is deferred; acceptance for the firmware change is code review,
  `cargo check`/clippy and the existing host tests.

**Open items**

- Measure the TPS22917 `CT` rise time and confirm or replace the provisional
  50 ms `BRANCH_RISE`.
- Measure the LiDAR's required off time and confirm or replace the provisional
  200 ms minimum off interval.
- Implement the camera's PIO UART release/tri-state and its boot delay.
- Implement the expansion task's UART1/`EN`/`BOOT` sequencing.
- Measure the off-state back-feed on each branch when the board exists.
