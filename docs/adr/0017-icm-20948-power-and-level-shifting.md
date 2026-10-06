# ICM-20948 on a dedicated 1.8 V rail, with its SPI bus level-shifted

The ICM-20948's fields are not symmetric with the rest of the board. Its **VDD**
accepts 1.71–3.6 V, but its **VDDIO is 1.71–1.95 V** — 1.8 V only, *not* 3.3 V. The
InvenSense datasheet calls this out in its abstract: "Note: ICM-20948 VDDIO range is
1.71 V to 1.95 V, different than the MPU-9250 9-axis device." The board's only rail
today is 3V3 (from the `AMS1117-3.3` in `eda files/explorer-robot/rp2354b.kicad_sch`),
and the MCU's internal regulator is the digital core rail (DVDD = `+1V1`), which is
neither 1.8 V nor a peripheral supply. So the IMU needs its own 1.8 V rail, and
because its logic then runs at 1.8 V while the RP2350B's I/O runs at 3V3, its SPI bus
and interrupt need level translation.

**Status:** accepted. The IMU is not yet placed in the schematic; this records the
power and interface topology before it is.

**Considered Options**

- **Power both VDD and VDDIO from the existing 3V3 rail, no shifters.** Rejected:
  3.3 V is out of spec for VDDIO (max 1.95 V). This is the tempting shortcut and the
  trap this ADR exists to prevent.
- **VDD from 3V3, VDDIO from 1.8 V.** Allowed (both pins stay in range), but carries
  two rails to the part for no benefit over a single 1.8 V feed.
- **Run the RP2350's IOVDD at 1.8 V and skip the shifters.** Rejected: the RP2350 has
  a single IOVDD domain, so this drags the whole 3.3 V I/O population (the ST7789
  panel and the rest) down to 1.8 V or onto its own translation. Worse than shifting
  only the IMU.
- **Use an IMU with a 3.3 V-tolerant VDDIO (e.g. MPU-9250-class).** Rejected for now:
  `icm20948-rs` (with the DMP) is already integrated into the firmware. Noted as an
  option for a future revision if the 1.8 V rail and shifter ever become a burden.

**Decision**

- Add a small 1.8 V LDO off 3V3 and feed it to **both** ICM-20948 `VDD` and `VDDIO`.
  This matches the datasheet's typical operating circuit (VDD = VDDIO = 1.8 V) and
  satisfies VDDIO ≤ VDD.
- Keep **3V3 at 3.3 V** for the RP2350B IOVDD and the other peripherals; do not lower
  IOVDD to 1.8 V.
- Level-shift the IMU's digital lines — **SCLK, MOSI, MISO, CS and INT** — with a
  **TXB0108** (8-bit auto-direction, push-pull). Wiring: **VCCA = 1.8 V** (A port, IMU
  side), **VCCB = 3.3 V** (B port, MCU side), so VCCA ≤ VCCB holds.
- **OE (output enable) is active-high and must end up HIGH.** Low places all outputs
  in high-impedance and the bus goes dead. The datasheet's "tie OE to GND through a
  pulldown resistor" is only the power-up/power-down default (hold outputs Hi-Z until
  VCCA and VCCB are ramped and stable) — it is **not** the operating state. Enable it
  after both rails are up, by pulling OE **up to VCCA** through a resistor (~100 kΩ),
  or by driving it from an MCU GPIO released after boot. Never tie OE low
  permanently.
- Note that OE is referenced to VCCA, which here is the 1.8 V rail, so "high" is ~1.8 V.
  If an MCU GPIO drives OE, confirm the TXB0108 recommended-operating/absolute-max
  table allows that level against VCCA = 1.8 V; the safe default is to keep OE at VCCA.
- The "external pull resistors > 50 kΩ" rule applies to the **I/O lines**, not to OE
  (the TXB's internal ~4 kΩ series resistor degrades VOH/VOL under any external pull;
  `VOH = VCCx · RPD / (RPD + 4.5 kΩ)`). With push-pull SPI and a push-pull INT, no line
  pull-ups are needed; if one is ever added, keep it ≥ 50 kΩ.
- The TXB0108 translates **push-pull only**. If any line on this bus is ever
  open-drain (I²C, for instance), use a **TXS0108**-class part instead.
- Run the IMU SPI at or below its **~7 MHz** device limit; that ceiling makes any
  standard translator fast enough.

**Consequences**

- The power tree gains a second regulated level: 3V3 (AMS1117) → 1.8 V LDO → IMU. The
  1.8 V rail is IMU-only for now.
- The BOM grows by the 1.8 V LDO, the TXB0108 and its decoupling (bypass caps at both
  VCCA and VCCB pins, close to the pins).
- The SPI bus now crosses a translation seam: keep those traces short (the TXB's
  one-shot is ~10 ns, so long traces reflect) and keep them out of the noisy motor
  routing.
- The IMU is a **1.8 V logic domain**, not a 3.3 V one. Do not hang it on, or power it
  from, the RP2350 core rail (`+1V1`), and do not assume "the ICM-20948 is 3.3 V like
  the MPU-9250" — the 1.95 V VDDIO max is the recurring mistake this board must avoid.
- Enabling OE is a real step, not a tie-off. A design review should treat a
  ground-referenced OE (or an OE left floating) as a bug that silently dead-buses the
  IMU.
- If the IMU is ever re-homed or the board re-spun, revisit the shifter in favour of a
  3.3 V-VDDIO IMU, which would remove both the 1.8 V rail and the translation seam.
