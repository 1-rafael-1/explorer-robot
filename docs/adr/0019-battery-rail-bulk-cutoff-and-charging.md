# Battery rail: a protected pack, modest input bulk, a low-voltage cutoff at 6 V, and charging kept off-board

The robot runs from a 2S 18650 pack (two cells in series: 8.4 V full, 6.0 V at
3.0 V/cell) that carries an in-pack switch and an integrated protection board
(over-current, over-charge, over-discharge and over-temperature; confirm whether
short-circuit protection is included). There is no on-board BMS. The pack-to-board
cable is about 10 cm. The predecessor robot put a 4700 µF reservoir on this rail;
this revision does not. This ADR records the rail's bulk, the low-voltage cutoff
policy, and the decision to leave charging off the board. It also records the 5 V
rail derived from the protected battery rail: a TPS5430 buck, its low-line
behaviour at the 6.0 V cutoff, and the RP2350 `VREG_VIN` sequencing change.

**Status:** accepted for the pack-protection, charging, and 5 V-rail decisions.
The input bulk is settled: 2× 220 µF/25 V polymer at the battery and 1× 220 µF of
the same part at `VM` (ADR-0018), one capacitor part number shared with the 5 V
`COUT`. The in-pack switch rating and the inrush/BMS check remain open. The 5 V
rail is accepted with the caveat that it may sit around 4.9 V at the 6.0 V cutoff
and 500 mA; the Coin D6 is specified for 4.5–5.5 V, so that is inside its range.
The Coin D6 current draw and the bench measurement remain open. An on-board BMS is
no longer planned.

**Considered Options**

- **Rely on the pack's built-in protection.** Accepted: the ordered 2S packs
  include over-current, over-charge, over-discharge and over-temperature
  protection. This is the hardware safety net; no on-board BMS is added. Confirm
  the exact feature list (short-circuit protection is commonly included) and
  whether the BMS balances the cells.
- **A large reservoir on the battery input (the predecessor's 4700 µF).** Rejected.
  A stiff 2S pack over a ~10 cm cable has minimal inductive droop, and the battery
  is already the reservoir; a large input bank mainly adds inrush energy through the
  switch contacts and costs board area for little benefit. The selected 2× 220 µF
  polymer bank (440 µF) is an order of magnitude below that reservoir, and its
  inrush is checked separately below. Geometry is shared with the driver bank in
  ADR-0018.
- **Firmware cutoff only, no hardware protection.** Partially accepted — it gives
  the exact 6.0 V behaviour — but on its own it is a discharge *policy*, not
  protection against over-current, short circuit, or a firmware hang. Rejected as
  the sole mechanism.
- **A dedicated 2S protection/BMS board.** Rejected: the protected pack already
  provides the safety net. An on-board BMS would duplicate it, add cost and board
  area, and put a second series element in the motor return. Its over-discharge
  threshold is typically ~2.4–2.5 V/cell — lower than the 3.0 V we want — so even
  if it were fitted it would be the backstop, not the policy.
- **Integrate on-board charging now.** Deferred. A safe 2S on-board charger needs
  CC/CV, cell balancing, input negotiation and thermal care (e.g. a 2S
  charger-with-balance IC, BQ25887- or IP2326-class) — more responsibility than this
  revision wants. Revisit as its own ADR.
- **Expose a balance-charge connector and charge from an external 2S balance
  charger.** Accepted: cheap, safe, and no charge current flows through the board.

For the 5 V rail:

- **TPS5430DDAR buck (JLCPCB Basic).** Accepted. The TPS5430 is a 3 A, 500 kHz,
  non-synchronous buck in SOIC-8-EP with a PowerPAD (RθJA ≈ 42–45 °C/W), an
  adjustable output, and a 5.5–36 V input. It is Basic at JLCPCB, runs cooler and
  with smaller passives than the XL1509, and its 3 A rating leaves ample current
  margin for a 300–500 mA load. Its 87% guaranteed maximum duty cycle (89% max)
  is the catch: at 5 V output the minimum input is about 6.0 V at 0.5 A, 6.2 V at
  1 A, 6.5 V at 2 A, and 6.9 V at 3 A, so the 5 V rail can sit around 4.9 V at the
  6.0 V cutoff.
- **XL1509-5.0 (JLCPCB Basic).** Rejected. The fixed 5 V version is simpler and
  has 100% maximum duty, but it is a 150 kHz non-synchronous part with a 1.5 V
  dropout, is only characterized for 5 V output from 7 V input, and its SOP-8 has
  RθJA ≈ 100 °C/W. It is marginal at the 6.0 V cutoff and thermally limited well
  below its 2 A headline.
- **TPS562201 or a similar synchronous buck (JLCPCB Extended).** Rejected. It is
  synchronous and smaller, but its datasheet recommends a maximum duty cycle of
  75%, so 5 V output needs at least ~6.67 V input. Synchronous rectification alone
  does not solve the low-headroom problem.
- **A buck-boost (TPS63060-class).** Deferred. It would hold 5 V below 6.0 V, but
  it adds cost and complexity and is likely an Extended part. Revisit if the
  Coin D6 or the 4.9 V low-line proves unacceptable.
- **Raise the cutoff to 6.5–7.0 V.** Deferred. It would give the TPS5430 margin
  (comfortable to ~2 A at 6.5 V and to 3 A at 7.0 V), but it sacrifices pack
  capacity. Revisit if bench testing shows the 4.9 V low-line is not acceptable.
- **Keep `VREG_VIN` on `+5V`.** Rejected. The RP2350 requires `VREG_VIN` and
  `VREG_AVDD` to power up together; with `VREG_VIN` on `+5V` and `VREG_AVDD` on
  the 5 V-derived 3.3 V rail, the 5 V rail comes up first. Move `VREG_VIN` to
  `+3.3V`, where the datasheet's combined-supply configuration applies.

**Decision**

- **Battery bulk = 2× 220 µF/25 V polymer aluminum** (KNSCHA 217EC0037, JLCPCB
  `C42371908`; 35 mΩ ESR each, ~17.5 mΩ parallel, 440 µF total). The `VM` bulk is
  1× 220 µF of the same part (ADR-0018), so the battery bank, the driver bank and
  the 5 V `COUT` share one capacitor part number. Total rail bulk is 660 µF, well
  under 1000 µF rather than thousands. The earlier 0–1× 100 µF input-bulk target is
  superseded by this consolidation; the tradeoff is a higher connect/inrush peak
  from the polymer's low ESR, which must be checked against the pack switch and BMS.
- **Low-voltage cutoff = 6.0 V pack (3.0 V/cell), in two layers:**
  - **Firmware (policy):** the existing ADC battery monitor detects <6.0 V,
    commands the motors off, and begins a graceful shutdown. The threshold must
    account for load sag — sample with the motors quiescent, or latch with
    hysteresis — because 6.0 V *under load* can be reached while the cells still
    rest well above 3.0 V.
  - **Hardware (safety net):** rely on the protected pack's integrated BMS for
    over-discharge, over-current and short-circuit protection (and over-temperature
    where fitted). Do not add an on-board BMS.
- **Charging stays off-board.** Provide a 2S balance-charge connector (e.g.
  JST-XH, 3-pin); charge with an external 2S balance charger. Do not route charge
  current through the board. The pack's over-charge protection is a backup; the
  external charger must be a proper 2S CC/CV unit and must not rely on the pack BMS
  to terminate charge. A cheap *and* safe on-board charger is a separate, future
  decision.
- **The in-pack switch must be rated for motor stall current**, not cruise current,
  and its contact life is helped by keeping the rail's total bulk small (ADR-0018).
- **The pack BMS's over-current trip must sit above the worst-case motor
  start/stall current** and below the pack's safe continuous rating. If it trips
  during normal driving, the whole robot resets; verify the threshold against the
  worst-case battery current drawn by the motor driver (the TB6612FNG is rated
  1.2 A/channel continuous and 3.2 A/channel peak; battery current is lower at
  partial duty).

**5 V rail**

- **Regulator.** Use the TPS5430DDAR (JLCPCB Basic, `C9864`) in SOIC-8-EP, fed
  from the protected `+8V` net after Q1. Do not feed it from the raw battery side
  before Q1, or the 5 V rail loses reverse-polarity protection. Output is 5.0 V;
  the load budget is 300–500 mA with 1 A-class margin. Loads are the switched
  Coin D6 LiDAR branch (ADR-0010), the AMS1117-3.3 input, and any 5 V
  display/backlight/peripheral. `VREG_VIN` is no longer a 5 V load (see
  sequencing below).
- **Design values.** R1 = 10 kΩ / R2 = 3.24 kΩ (1%) for 5.0 V; L1 = 22 µH/1.4 A
  (SHOU HAN CYN5040-22UH, JLCPCB `C2879700`, 1.8 A saturation), which limits the
  practical output to ~1.4 A; use a 15 µH/3 A part if the TPS5430's full 3 A
  capability is needed. D1 = B340A (40 V, 3 A) catch diode; CIN = 3× 10 µF/25 V
  X5R 0805 (Samsung CL21A106KAYNNNE, JLCPCB `C15850`) close to VIN/GND, in
  addition to the existing `+8V` bulk; CBOOT = 0.01 µF between BOOT and PH;
  COUT = 220 µF/25 V polymer aluminum, 35 mΩ ESR, 1.95 A ripple (KNSCHA
  217EC0037, JLCPCB `C42371908`). With 22 µH this gives a
  ~12.3 kHz crossover and a ~20.7 kHz ESR zero, inside the internal
  compensation's range. If a ceramic output capacitor is used instead, add the
  external type-3 compensation network from the datasheet. `ENA` is active-high:
  float or pull it up to enable; pull it below 0.5 V to disable. Add a `PWR_FLAG`
  on `+5V` for ERC. The 25 V input-cap rating is required because the `+8V` rail
  is exposed to the 14.5 V TVS clamp; the earlier 47 µF/10 V part is not acceptable.
- **Low-line behaviour.** The TPS5430's 87% guaranteed maximum duty cycle (89%
  max) limits the minimum input for 5 V to about 6.0 V at 0.5 A, 6.2 V at 1 A,
  6.5 V at 2 A and 6.9 V at 3 A, using the datasheet's worst-case Rds(on), diode
  drop and inductor DCR (the regulator's capability; the selected inductor limits
  the practical output to ~1.4 A). The device's 5.5 V minimum input is below
  the 6.0 V cutoff, so the limit is duty cycle, not UVLO. At the cutoff and
  500 mA the 5 V rail may therefore sit around 4.9 V. This is outside the datasheet's guaranteed
  5 V regulation but is accepted because the downstream LDOs still regulate and
  the Coin D6 is specified for 4.5–5.5 V, so ~4.9 V is inside its range with
  ~400 mV margin to the low end; a slightly lower spin speed is acceptable.
  Confirm its current draw at 4.9 V. If bench testing shows the rail falls below
  4.5 V, raise the cutoff to 6.5 V (comfortable to ~2 A) or use a buck-boost.
- **LDO behaviour.** The AMS1117-3.3 holds 3.3 V until its input falls below about
  4.4 V at 500 mA (the 1.1 V dropout is specified at 1 A; it is lower at 500 mA),
  so the 4.9 V low-line is above dropout. The ICM-20948 1.8 V LDO (ADR-0017)
  holds 1.8 V until the 3.3 V rail falls below about 2.0 V, and the RP2350's
  internal 1.1 V regulator holds 1.1 V while `VREG_VIN` is at least 2.7 V. A
  0.1 V sag on the 5 V rail therefore does not sag the 3.3 V, 1.8 V or 1.1 V rails
  proportionally; the LiDAR is the only load that sees the 5 V sag.
- **Sequencing.** Move `VREG_VIN` from `+5V` to `+3.3V`; keep `VREG_AVDD` on
  `+3.3V` through R1 33 Ω and C1 4.7 µF. The RP2350 datasheet allows
  `VREG_VIN` from 2.7 V to 5.5 V and, when `VREG_VIN` is limited to 3.135–3.63 V,
  permits a single combined supply for both rails; this also satisfies the
  requirement that `VREG_VIN` and `VREG_AVDD` power up together. The existing
  L1 3.3 µH, C2 4.7 µF, R1 33 Ω, C1 4.7 µF and C3 4.7 µF already match the
  datasheet's Figure 22; only the `VREG_VIN` net changes. This does not
  meaningfully reduce the 5 V load — the 3.3 V LDO must supply the regulator input
  current, so the 5 V current may rise slightly — but it keeps the core regulator
  off the sagging 5 V rail and fixes the sequencing.
- **LiDAR branch.** Keep the Coin D6 on its own switched 5 V branch per ADR-0010;
  do not put the LiDAR switch in series with the AMS1117/`VREG_VIN`, and do not use
  the TPS5430 `ENA` for the LiDAR, because that would shut down the whole 5 V rail
  and the 3.3 V LDO.
- **Layout.** Solder the PowerPAD to a copper pour with vias. Keep CIN close to
  VIN/GND, keep the SW/PH and catch-diode loop tight, keep the FB node away from
  the inductor, and place COUT close to the inductor output. The PCB is still
  unrouted, so these are layout constraints rather than fixes.

**Consequences**

- The protected pack provides the hardware safety net (over-discharge, over-current,
  over-temperature, and usually short-circuit). The firmware cutoff remains the
  normal-operation policy and must fire before the pack's over-discharge threshold.
- A pack-protection hard-cut disconnects the whole board, including the MCU,
  because everything shares one rail. The firmware cutoff (6.0 V) must therefore
  fire well before the hardware threshold (~2.4–2.5 V/cell) — and it does: 6.0 V
  sits far above the pack trip.
- The pack BMS adds a small series resistance and a hard cutoff in the discharge
  path. The pack must be rated for the motor current, not just the cell capacity;
  confirm its continuous and peak current ratings.
- The 440 µF battery bank's ~17.5 mΩ parallel ESR makes the connect/inrush peak
  higher than the 3× 100 µF electrolytics (~200 mΩ). The stored energy is still
  small (~15.5 mJ), but verify the pack switch and BMS over-current/short-circuit
  ratings; add a pre-charge/soft-start if they are marginal.
- The pack BMS does not replace board-level reverse-polarity protection: it
  protects the cells, not the board, and its behaviour with the pack connected
  backwards is not specified. That remains a board-level decision.
- The AMS1117-3.3 holds 3.3 V until its input falls below about 4.4 V at 500 mA;
  the 4.9 V low-line of the 5 V rail is above that, so the 3.3 V rail stays in
  regulation at the 6.0 V cutoff. Below about 4.4 V, the 3.3 V rail follows the
  5 V rail down and the RP2350's `VREG_AVDD` approaches its 3.135 V minimum; that
  is the real brown-out boundary. The 1.8 V LDO (ADR-0017) holds 1.8 V until the
  3.3 V rail falls below about 2.0 V.
- Charging off-board means the charge connector must be externally reachable and
  clearly distinct from the discharge path; keep charge and discharge wiring
  separate. The pack's over-charge protection is a backup, not the charge
  controller.
- The 5 V rail is a nominal 5 V rail, not a guaranteed 5 V rail at the 6.0 V
  cutoff: at 500 mA it may sit around 4.9 V. The 3.3 V, 1.8 V and 1.1 V rails
  hold; the Coin D6 sees the sag but is specified for 4.5–5.5 V, so ~4.9 V is
  within spec. This is the deliberate tradeoff for keeping the 6.0 V cutoff and
  a Basic part.
- The 5 V low-line is outside the TPS5430's guaranteed 5 V regulation; bench-test
  at 6.0 V/500 mA (and ideally 6.0 V/1 A) with the LiDAR and the 3.3 V LDO
  connected. If the LiDAR browns out or the 5 V rail falls below the LiDAR's
  4.5 V minimum, raise the cutoff to 6.5 V or switch to a buck-boost.
- The TPS5430's 3 A rating is credible with the PowerPAD, but the selected
  22 µH/1.4 A inductor is the practical 5 V rail current limit (~1.4 A); the
  B340A catch diode is also a thermal weak point at 3 A. At 300–500 mA both are
  comfortable, and the 1.4 A limit still leaves 1 A-class margin. Use a
  15 µH/3 A inductor if the full 3 A capability is ever needed.
- The TPS5430 itself is Basic, but the selected 22 µH inductor and 220 µF polymer
  output capacitor are Extended at JLCPCB, so the 5 V rail adds two extended-part
  handling fees. Confirm the final assembly BOM and whether a Basic equivalent is
  stocked for either passive.
- The 5 V rail is non-synchronous with a catch diode and a boot capacitor, and its
  internal compensation expects a low-ESR polymer/electrolytic/POSCAP output
  capacitor (selected: 220 µF/25 V polymer, 35 mΩ ESR). A ceramic-only output
  requires the datasheet's external compensation network. This is more complex
  than the XL1509 but smaller and cooler.
- Moving `VREG_VIN` to `+3.3V` fixes the sequencing note: `VREG_VIN` and
  `VREG_AVDD` now come up together. It does not reduce the 5 V load; the 3.3 V LDO
  carries the regulator input current.
- The TPS5430's quiescent current is 2 mA typ (4.4 mA max); pulling `ENA` low drops
  it to ~15 µA. If the pack stays connected in storage, provide a main switch or
  an `ENA` pull-down.
- Open items: confirm the BMS over-current trip against the motor start/stall
  current and the 440 µF battery-bank inrush; confirm whether the BMS balances the
  cells and includes short-circuit protection; confirm the in-pack switch's current
  rating against the inrush; the battery bulk is settled at 2× 220 µF polymer and
  the `VM` bulk at 1× 220 µF, removing the 3× 100 µF and 2× 100 µF parts; decide
  (later) whether to add an on-board 2S charger.
- Open items (5 V rail): confirm the Coin D6 current draw at 4.9 V (its
  4.5–5.5 V input range is verified); bench-test the TPS5430 low-line at
  6.0 V/500 mA and 1 A; confirm the 3×0805 input-cap bank footprint and placement,
  the selected output capacitor's footprint and polarity, and the PowerPAD layout.
