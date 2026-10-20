# Two rangefinders on unswitched 3.3 V with XSHUT and INT

The robot carries two VL53L0X time-of-flight modules on the enclosure: one
front-down/forward and one rear-down/backward. They are external parts on cables,
one per I²C bus, so the board only needs a connector, decoupling, pull-ups and
series resistors for each. The modules expose six signals: `VCC`, `GND`, `SCL`,
`SDA`, `INT` and `SHUT`.

The two buses are electrically separate: the front unit is on I2C0 (GPIO 16/17)
and the rear unit on I2C1 (GPIO 26/27). Both keep the default `0x29` address, so
`XSHUT` is not needed for address sequencing. It is still useful as a hard reset
and standby control, and the spare GPIOs are available.

**Status:** accepted. The topology, connector pin order and GPIO allocation are
settled. The 0603 33 Ω part selection remains provisional.

**Considered Options**

- **Switch each 3.3 V branch with a TPS22917.** Rejected: the sensor is idle
  until a measurement is started, the laser is off when idle, and continuous
  power does not meaningfully degrade the VCSEL. A load switch would cut the
  whole module, including the carrier's regulator and level shifters, but it adds
  two parts, two capacitors and two enable pulldowns for a capability the robot
  does not need.
- **Leave `XSHUT` unconnected and rely on the carrier's pull-up.** Rejected:
  the spare GPIOs exist, and `XSHUT` gives a reset path and a low-power standby
  state without a load switch. Driving it open-drain is safe with the module's
  2.8 V local rail.
- **Use a 4-pin or 5-pin connector.** Rejected: the module has six pins and a
  6-pin JST PH connector lets the cable be straight-through. `INT` is wired even
  though the current firmware only polls; the two spare GPIOs are not needed for
  anything else.
- **Rely on the module's I²C pull-ups.** Rejected: the cable runs from the
  enclosure to the board and the module's pull-up value is not under our
  control. One 4.7 kΩ pair per bus on the board makes the bus independent of the
  module.
- **Use 100 Ω series resistors as on the UART sheets.** Rejected in favour of
  33 Ω: the I²C bus is open-drain and 400 kHz, and 33 Ω still bounds fault
  current while keeping the edges clean. The 0603 33 Ω part is not yet selected.
- **Add a polyfuse and ESD arrays per connector.** Rejected to match the other
  connectors on the board: no other external connector has a polyfuse, and the
  connectors are keyed and internal. A cable short relies on the AMS1117's
  current limit and thermal shutdown.
- **One sheet per rangefinder.** Rejected: the two blocks share the same
  treatment and are easier to audit side by side on one sheet.

**Decision**

- **Power.** Both modules run from the unswitched `+3.3V` rail. Each connector
  has 100 nF and 10 µF decoupling. No load switch, no polyfuse, no ESD array.
- **Connectors.** `J13` (`RF.FRONT`) and `J14` (`RF.REAR`), 6-pin JST PH
  (`B6B-PH-K`), pin order: 1 `VCC`, 2 `GND`, 3 `SCL`, 4 `SDA`, 5 `INT`,
  6 `SHUT`. The cable is straight-through.
- **Buses.** Front on I2C0 (GPIO 16/17), rear on I2C1 (GPIO 26/27). Both use the
  default `0x29`; no address reassignment.
- **Pull-ups and series resistors.** One 4.7 kΩ pull-up on each of SDA and SCL
  per bus, placed on the connector side of the series resistor. One 33 Ω series
  resistor on each of SDA and SCL per bus, between the MCU-side hierarchical
  label and the connector. No series resistors on `INT` or `SHUT`.
- **Control pins.** Front `XSHUT` = GPIO 1, front `INT` = GPIO 2; rear `XSHUT`
  = GPIO 3, rear `INT` = GPIO 31. `XSHUT` is driven open-drain/high-Z; `INT` is
  read as an input. The modules provide the `INT` pull-up.
- **Sheet.** `rangefinders.kicad_sch` holds two independent blocks, one per
  connector, with hierarchical labels `I2C0_SDA/SCL`, `I2C1_SDA/SCL`,
  `RF_FRONT_XSHUT/INT` and `RF_REAR_XSHUT/INT`. The master sheet wires those to
  the `rp2354b` sheet pins.
- **Firmware.** `Rangefinder0Resources` / `Rangefinder1Resources` gain `xshut`
  and `interrupt` pins. No driver or task behaviour is added in this change.

**Consequences**

- GPIO 1, 2, 3 and 31 leave the spare pool. ADR-0021's note, ADR-0022's spare
  list and ADR-0024's note that they remain spare are amended.
- `XSHUT` must not be driven push-pull high: the module pulls it to its local
  2.8 V rail. Firmware reserves the pins now and must configure open-drain when
  the driver is written.
- The rangefinders are a floor-drop/near-field pair for now. Using them as
  obstacle sources would change the `ObstacleSource` model and is deferred.
- The front and rear floor-drop sources are independent. The current single
  `FLOOR_DROP` flag and `FloorDropDetected` event do not distinguish them; that
  is a firmware follow-up, not a hardware change.
- The 0603 33 Ω part is not yet selected, so its MPN/LCSC metadata is
  intentionally absent until a suitable JLCPCB part is chosen.
- No polyfuse or ESD protection is populated. A shorted enclosure cable relies
  on the AMS1117's current limit and thermal shutdown.
