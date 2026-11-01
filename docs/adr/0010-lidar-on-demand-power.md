# Power the LiDAR on demand, not continuously

The COIN-D6 is powered down whenever no mode needs it, rather than left spinning from
boot. The sensor is the most expensive part on the robot and a spinning dTOF head is a
wear item; keeping it running through motor tests, calibration and idle trades its life
for nothing. The cost is a warm-up wait at every mode entry and a power lifecycle to get
right.

**Status:** accepted. Revised in place: the LiDAR's 5 V branch is switched by a TPS22917 high-side load switch, and the UART is quiesced and tri-stated before power-off (ADR-0023). The leased ownership model originally recorded here is superseded by ADR-0017: the LiDAR task owns its power lifecycle, and a mode enables and disables the sensor rather than holding a lease. The bullets below state the enable/disable model.

**Considered Options**

- **Spin from boot and always.** Simpler and with no warm-up stall, and the earlier
  firmware took this for granted. Rejected on part life and power draw.
- **Pre-warm when the operator opens the relevant menu, or keep a grace period after a
  mode exits.** Rejected for now: it couples menu navigation to sensor state for a wait
  that is legible on screen. Revisit if the stall proves annoying in use.
- **Low-side N-FET on the LiDAR ground.** Rejected. It leaves the LiDAR's 5 V pin
  live and makes the UART lines the only ground-referenced path into the module when
  the switch is off; it also assumes the module is fully floating. The high-side
  TPS22917 keeps the connector ground solid, blocks reverse current, and lets QOD
  hold the switched 5 V rail low.
- **High-side TPS22917 load switch on 5 V.** Accepted. 1–5.5 V, 2 A, active-high
  enable, adjustable rise, QOD, and true reverse current blocking. It has no current
  limit; the 5 V buck and the pack protection are the forward-fault backstop.

**Decision**

- A persistent core1 task owns the driver for the whole runtime and exposes
  `enable()` / `disable()` / `status()`. A mode enables the sensor when it needs perception and
  disables it when it is done; the task owns power, warm-up, retries and streaming (ADR-0017).
  At most one mode needs perception at a time, so the enabled sensor serves every reader.
- `enable()` asserts the `LIDAR_PWR` enable, waits for the TPS22917 rise time, then
  constructs the `BufferedUart` from the retained `Peri` handles and runs
  `probe()` → `start()` → `warm_up()` → streaming. A `start()` failure is
  tolerated; warm-up is the real proof that data flows.
- Warm-up is bounded by a wall-clock timeout at the call site, because the driver has only
  byte-count watchdogs. On timeout the task power-cycles and retries a bounded number of
  times before reporting `Failed`.
- `disable()` stops the device, quiesces and deinits the UART, releases the MCU
  TX and RX pins to high-Z, then deasserts the `LIDAR_PWR` enable and clears stale
  state: the stored cloud becomes `None`, the obstacle flag is cleared, and a
  cleared `ObstacleDetected` edge is raised if the flag had been set (ADR-0023).
  It calls `CoinD6::release()` to recover the UART and power enable before the final
  teardown steps.
- Coast-and-Avoid enables the LiDAR on entry and **refuses to start** if the sensor cannot be
  brought up, rather than driving with only the downward rangefinder for obstacle sense.
- The LiDAR is off at boot; motor tests, IMU tests, calibration and idle never power it.
- The load switch is a high-side TPS22917 on GPIO30. The connector's pin 2 is ground;
  only the 5 V pin is switched. The UART lines carry 100 Ω series resistors. The
  TPS22917 uses a 1 µF `CIN`, a 1 nF `CT` capacitor to `VIN` (`C29`) for a
  controlled rise, `QOD` tied to `VOUT`, and a pulldown on the enable (it also
  has a smart pulldown).

**Consequences**

- Entering Coast-and-Avoid or Room Scan may wait several seconds while the rotor reaches
  steady state; the screen shows the warming state so the wait does not read as a hang.
- Consumers must treat an absent cloud as unknown, not as an empty room: a `None` snapshot
  means the sensor is off or not yet warmed.
- A stopped sensor can leave an obstacle flag behind, so `disable()` carries the
  obligation to clear it in the same step.
- `disable()` is one-way and not owner-scoped, so a leave delivered after another mode has
  enabled the sensor powers it down under that mode; the window is small and accepted rather
  than guarded (ADR-0017).
- The TPS22917 load switch stays wired even though it is now load-bearing rather than
  optional. It keeps the LiDAR ground solid, blocks reverse current from the module, and
  uses its QOD to hold the switched 5 V rail low when off. It has no current limit; the
  5 V buck and the pack protection are the forward-fault backstop.
- The LiDAR UART is no longer a runtime-static resource. `BufferedUart` is
  constructed per powered period from retained `Peri` handles, `UARTCR.EN` is
  cleared on power-down, and both TX and RX are tri-stated while the branch is off
  (ADR-0023).
- The LiDAR's 5 V peak is unmeasured and shares the rail with the AI camera and
  expansion branches; the 5 V budget is revisited in ADR-0019.
