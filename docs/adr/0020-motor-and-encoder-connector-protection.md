# Protecting the motor and encoder connectors

Each track's motor connects through a single 6-pin JST-PH connector that carries
both the motor drive and the encoder, with the vendor's pin order fixed as
`M+, 3V3, ENC_A, ENC_B, GND, M−` — `M+` and `M−` at opposite ends. The two motors
share this wiring with the 20 kHz PWM drive, and there is no option to reorder pins,
twist the cable as a whole, or separate the encoder wires from the motor wires.
This ADR records the protection and noise-mitigation decisions for these
connectors: transient clamps on the drive side, and encoder signal integrity on the
signal side.

**Status:** proposed. The protection strategy is settled. The encoder output type
is now **verified push-pull** by bench measurement (see *Bring-up result*), so the
provisioned pull-ups stay unpopulated. The remaining part values and the RC
tuning are still provisional pending bring-up.

**Considered Options**

- **Rely on the driver's freewheel diodes and bulk caps alone.** Partially accepted
  — they handle the ordinary PWM inductive kick — but they do not cover unplugging a
  motor under load (the stored `½LI²` spikes the output pins) or sustained
  regenerative back-drive lifting `VM`. Rejected as sufficient on its own.
- **A unidirectional TVS across each motor pair.** Rejected: the H-bridge drives
  both polarities, so a unidirectional clamp would forward-conduct at ~0.7 V and
  short the motor whenever it is driven that way. A **bidirectional** part is
  required.
- **A bidirectional TVS on `VM` to `GND`.** Accepted as a BOM-consolidation
  compromise: it still clamps positive regen spikes, and the one-polarity `VM`
  rail makes negative excursions unlikely. The penalty is that a negative
  excursion is clamped at ~−9.5 V instead of ~−0.7 V; the unidirectional part is
  still the better choice if a second part number is acceptable. Reusing the
  `P4SMA10CA` from the motor pairs avoids a second extended part.
- **Pin-shuffle the connector to space motor and encoder pins apart.** Not
  available: the vendor fixes the pinout.
- **Twist or separate the encoder wires from the motor wires.** Not available: they
  share one connector and one cable.
- **Lean on the RC filter alone for encoder integrity.** Rejected: with `M+` and
  `M−` far apart, the motor-current loop encloses the encoder wires, giving large
  magnetic coupling; a filter alone fights the symptom.
- **Harness-level mitigation (pair `M+/M−`, ferrites).** Accepted — it attacks the
  dominant coupling at its source and costs almost nothing.

**Decision**

*Drive side — transient clamps:*

- **A bidirectional TVS across each motor pair** (`AO1–AO2`, `BO1–BO2`), to clamp
  the differential spike when a motor is unplugged under load. Working voltage
  ≥ ~12 V (above the ±`VM` drive swing); clamp as low as practical.
- **A TVS from `VM` to `GND`**, to clamp regenerative/back-drive spikes. A
  unidirectional part is preferred (it also clamps negative excursions at
  ~−0.7 V); the bidirectional `P4SMA10CA` already used on the motor pairs is
  accepted as a BOM-consolidation compromise, with the negative clamp at
  ~−9.5 V. Working voltage just above the maximum pack voltage (~10 V), clamping
  below the TB6612 `VM` absolute maximum (~15 V — verify in the datasheet).
- **Do not add reverse-polarity protection on the motor connectors.** The motor
  lines are bipolar (reversing them just reverses rotation) and the JST-PH connector
  is keyed, so reverse insertion is mechanically prevented. Reverse-polarity
  protection belongs on the battery input.

*Signal side — encoder noise mitigation, layered:*

- **Harness first (it does the most).** Pair — and twist, if the wires allow —
  `M+` and `M−` immediately after the connector, and keep them together for the
  whole run to the motor, so their equal-and-opposite currents cancel the field and
  collapse the coupling loop. The short `M+/M−` splay at the connector itself is an
  accepted residual.
- **A ferrite ring on the `M+/M−` pair**, both wires through it in the same
  direction, forming a common-mode choke: the differential motor current cancels, so
  the core does not saturate and the motor drive is unaffected.
- **Filter the encoder `3V3` supply:** a ferrite bead in series, plus 1 µF and
  100 nF to `GND` at the connector's `3V3` pin, because that pin runs alongside `M+`
  and feeds the encoder.
- **An RC low-pass on each encoder line**, single-pole baseline: series R ≈ 2.2 kΩ
  and C ≈ 10 nF to `GND` at the MCU pin (fc ≈ 7 kHz). The series R doubles as the
  ESD/overvoltage current limit into the MCU pin. Lay out a **second pole** as a
  fallback, to populate only if bring-up shows glitches.
- **A pull-up (10 kΩ to 3V3) on each encoder line**, populated only if the
  encoder turns out to be open-collector (in which case C drops to ~2.2 nF);
  harmless if the encoder is push-pull. **Not populated:** the encoder is
  push-pull (see *Bring-up result*), and the v0.1 released to fabrication carries
  no pull-up on either encoder line.
- **A firmware minimum-pulse-width reject** (~20–30 µs). The encoder is slow
  (≤ 3.6 kHz, pulses ≥ ~278 µs) while the interference is PWM-rate, so a
  debounce/glitch filter removes residual glitches. It is the cheapest, most
  effective net and depends on no board layout.

**Consequences**

- Protection is layered by domain: harness geometry and ferrites reduce coupling,
  the supply filter cleans the encoder's rail, the RC catches the residual and
  protects the pin, and firmware removes what is left. No single layer is expected
  to carry the whole job.
- The `VM` TVS has a narrow window: stand off above 8.4 V (the `P4SMA10CA` is
  8.6 V) and clamp below ~15 V (14.5 V at its rated current). It must be checked
  against the datasheet's absolute maximum, not picked generically.
- Reverse-polarity protection is explicitly **out of scope** for these connectors;
  it belongs on the battery input. ADR-0019 records the battery-rail decisions and
  notes that reverse-polarity protection remains a board-level decision.
- Pull-up and second-pole parts were intended to be laid out as DNP so one board
  supports both encoder output types and both filter orders without a re-spin.
  The v0.1 released to fabrication omits them, so changing the output type or
  strengthening the filter now needs rework rather than a population variant.
- Encoder reliability leans on a harness practice (pairing `M+/M−`) that is easy to
  get wrong in assembly; capture it in the build/assembly notes, not only here.
- Open items: confirm the `P4SMA10CA` clamp against the TB6612 `VM` absolute
  maximum; tune the RC (add the second pole) from measured glitching at 20 kHz
  and maximum RPM. The encoder output type is confirmed push-pull (see
  *Bring-up result*).

## Bring-up result

The encoder output type was measured on the bench. With the encoder powered from
3V3 and a scope probe on one output channel, the channel produced a clean
0–3.3 V square wave as the motor turned. Adding a 1 kΩ from the output to `GND`
left the high level at **3.28 V**: the output sourced ~3.3 mA with only ~20 mV of
droop, an apparent source impedance of a few ohms. An open-collector output with
a 1 kΩ on-board pull-up would have collapsed to ~1.6 V, and the usual 4.7–10 kΩ
pull-ups to below 0.6 V.

**Conclusion: the encoder outputs are push-pull.** The provisioned pull-up is not
needed and stays unpopulated.

What v0.1 actually carries, as recorded against the schematic now in fabrication:

- The encoder network is a **2.2 kΩ series resistor** (`R5`, `R7`) and a **2.2 nF
  shunt capacitor** (`C32`, `C34`) at the MCU pin, with no pull-up. That is the
  open-collector capacitor value, not the ~10 nF push-pull target above, so its
  corner is ~33 kHz rather than ~7 kHz.
- The v0.1 layout also omits the second filter pole and the per-connector 3V3
  ferrite/decoupling described above.

With a push-pull source the network is electrically safe and counts correctly:
2.2 kΩ into 2.2 nF settles in a few microseconds against a ≥278 µs pulse. The
cost is less PWM rejection than the ~10 nF target. Bring-up should watch the
MCU-side waveform at full RPM with the 20 kHz drive active; if it glitches,
rework `C32`/`C34` to 10 nF (0603) or add the second pole. The firmware
minimum-pulse-width reject is not yet implemented, so until it is, the RC and
the harness are the only glitch defences. The full bench procedure was recorded
during the hardware-integration review (§5.2 covers the encoder noise check).
