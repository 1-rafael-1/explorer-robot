# One ground net at the TB6612FNG, with `VM` bulk banked at the driver

The TB6612FNG splits its returns into a power-stage ground (`PGND1` on pins 3/4,
`PGND2` on pins 9/10) and a logic ground (`GND` on pin 11), which invites keeping
the motor return on a separate net. On this board it must not: a single 2S 18650
pack (8.4 V max) feeds both the H-bridge (`VM`) and every other load, so there is
one ground reference and nothing for a second net to reference against. The pins
exist so the high-current return can be *routed* apart from the logic return — a
layout concern, not a schematic one. The same reasoning applies to `VM` decoupling:
the datasheet's minimum bulk is a floor for Toshiba's eval board, not a target for
a battery-fed board with lead and motor-cable inductance.

**Status:** accepted. The single-ground topology and the three-tier `VM`
decoupling stack are settled. The bulk tier is one 220 µF/25 V polymer (shared
with the battery rail, ADR-0019); the required values are placed in
`eda files/explorer-robot/tb6612fng.kicad_sch`, the selected part is recorded here,
and the BOM data lives in the schematic.

**Considered Options**

- **Keep `PGND` and `GND` as two nets, joined at one star point.** Rejected. With
  one battery both nets sit at the same potential, so they can be joined in exactly
  one place; every additional bond is a ground loop, and a split under the motor
  return path is an antenna. Two nets buy nothing here and risk more.
- **Run the logic return to the battery on its own wire.** Rejected. There is no
  second supply domain to isolate; the logic current is milliamps against motor
  amps, and a separate wire only adds a loop.
- **Trust the datasheet's minimum bulk (10 µF).** Rejected. It is a floor for the
  vendor's reference layout; it ignores battery-lead inductance, motor-cable
  inductance, direction-reversal and braking transients — the exact things a
  battery-fed board must tolerate.
- **Two-tier decoupling (`VM` bulk + 100 nF only).** Considered, and common. The
  100 nF at the pin is the mandatory stroke. Rejected in favour of the three-tier
  stack: the 1 µF covers the ~100 kHz–1 MHz band that the bulk (inductive above
  ~100 kHz) and the 100 nF (only reaching down to ~1 MHz) each miss, and the 1 µF is
  a JLCPCB basic part, so the middle tier adds a part number but no handling fee.
- **One 220 µF bulk cap.** Accepted in the final revision: one 220 µF/25 V polymer
  at `VM` (and 2× 220 µF at the battery rail, ADR-0019) consolidates the BOM on a
  single capacitor part number. The 220 µF value is within the 2–3× 100 µF target,
  and the polymer's low ESR and high ripple rating are better than the electrolytic
  it replaces.
- **One bulky input reservoir (4700 µF, as on the predecessor).** Rejected. A stiff
  2S pack over a ~10 cm cable has little inductive droop for a large reservoir to
  cover; the battery is itself the reservoir. The extra capacitance only adds
  inrush energy through the pack switch (see ADR-0019).
- **A lower-voltage (16–25 V) bulk part.** Preferred on size, but a well-stocked
  part may be rated 63 V. 63 V at 8.4 V is electrically harmless — the rating only
  sets size and cost, and the extra margin is welcome against inductive spikes. The
  gating check is the voltage floor and the physical fit, not the exact rating.

**Decision**

- **One ground net.** In `eda files/explorer-robot/tb6612fng.kicad_sch`, land
  `PGND1` (pins 3/4), `PGND2` (pins 9/10) and `GND` (pin 11) of U6 on the single
  `GND` net. No second net, no net-tie; the pin split is expressed only in copper.
- **Realize the split in layout:**
  - Via cluster into the solid inner `GND` plane at the ground pins — the physical
    star point.
  - Keep the `VM` decoupling loop tiny: bulk plus ceramics between `VM` and `PGND`,
    at the pins.
  - Route the motor return to the battery connector without passing under the MCU,
    IMU, ADC battery sense, or the SPI/display routing.
  - No plane splits or via fences under any signal or under the motor return;
    stitch top-layer `GND` pours along the return path.
- **`VM` decoupling is a three-tier stack at the `VM`/`PGND` pins:** one 220 µF/25 V
  polymer aluminum bulk part (KNSCHA 217EC0037, JLCPCB `C42371908`; 35 mΩ ESR,
  1.95 A ripple), one 1 µF ceramic (mid-band), and 100 nF (pin-level). The tiers
  cover successive bands — the 220 µF is bulk, the 100 nF is the mandatory pin
  bypass, and the 1 µF bridges the ~100 kHz–1 MHz span that the other two miss.
- **The bulk polymer is accepted in a bulk-only role.** Its 35 mΩ ESR and 1.95 A
  ripple are better than the electrolytic it replaces; ESR is not a gating spec,
  but the lower ESR is welcome. The 25 V rating is above the 16 V floor and the
  14.5 V TVS clamp, and the 6.3×7.7 mm can fits. The whole rail's bulk — driver and
  battery connector (ADR-0019) — is one part number.
- **`VCC` logic decoupling = 100 nF plus the same 1 µF ceramic** at the `VCC`/`GND`
  pins. `VCC` has no bulk beside it, so the 1 µF is what keeps its mid-band covered.
- **Do not reduce bulk to the datasheet minimum.** 10 µF stays only as the vendor
  floor, never the target.

**Consequences**

- One ground reference; noise control lives in geometry (loop area, via stitching,
  return-path steering), which is where it belongs for a single-supply design.
- The rail's bulk is one 220 µF/25 V polymer at `VM` and 2× 220 µF at the battery
  (ADR-0019), all one part number. The 6.3×7.7 mm can fits; the polymer's lower ESR
  and higher ripple rating are better than the electrolytic, but the battery bank's
  higher connect/inrush peak must be checked against the pack switch and BMS
  (ADR-0019).
- Bulk cost is dominated by the per-part extended-part handling fee, not by the
  value; consolidating on one 220 µF polymer part number removes the separate
  100 µF part. The polymer costs more per unit, so compare at volume. Board area
  drops from five cans (3 battery + 2 `VM`) to three (2 battery + 1 `VM`).
- The 1 µF middle tier is one JLCPCB **basic** part reused at both `VM` and `VCC`,
  so it adds a part number but neither a handling fee nor a second value to source.
- Open items before layout: confirm the 220 µF polymer can fits at the drive pins
  and its polarity. The battery bank's inrush/BMS check is in ADR-0019.
