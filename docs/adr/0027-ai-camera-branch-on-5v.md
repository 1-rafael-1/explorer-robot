# AI camera branch on 5 V

The Grove Vision AI V2 module used on this robot takes **5 V** on its Grove
`VCC` input. The module's own schematic (`gvai2.pdf`) shows that pin feeding an
on-module `SY80004` 5 V→3.3 V buck, which then supplies the WiseEye2 and its
peripherals. Feeding the branch 3.3 V therefore under-drives that buck, and the
module's 3.3 V rail and its boot/inference behaviour are not guaranteed.

ADR-0025 had moved the branch from 5 V to `+3.3V` on the reading that Seeed's
bootloader-recovery wiring specified the module input as 3.3 V. The user
confirmed against the v2 module and its schematic that the normal operating
input is 5 V; that reading of the recovery wiring did not describe the normal
Grove `VCC` path. This ADR supersedes ADR-0025.

**Status:** accepted and implemented 2026-10-23. `U9.1` (`VIN`) and `C55` are
back on `+5V`; the change is verified in the netlist and ERC is clean. The
module harness mapping (which module pins `J8` feeds) remains open.

**Considered Options**

- **Keep the branch on 3.3 V (ADR-0025).** Rejected: it under-drives the
  module's own buck. This is a confirmed functional error, not a preference.
- **Add an on-board 3.3 V→5 V boost for the camera.** Rejected: the 5 V rail
  already reaches the load switch, so this adds a converter, its inductor and
  its losses for no benefit.
- **Feed the camera from 5 V through a series LDO.** Rejected: the module needs
  5 V at its input; an LDO would only drop it further, and it adds dissipation.
- **Power the module's 3.3 V rail directly, bypassing its buck.** Rejected: it
  requires modifying the module and its harness and is not a supported input.
- **Keep the camera on switched 5 V.** Accepted. It is the module's specified
  input and removes the camera from the `AMS1117-3.3` load.

**Decision**

- **Rail.** `U9` (`TPS22917DBVR`, LCSC `C2681320`) `VIN` (pin 1) is tied to
  `+5V` by `#PWR078` at `(124.46, 86.36)`. The reference, position and rotation
  are unchanged. `C55` (1 µF, `C15849`) is the branch input capacitor across
  `+5V`/GND. `U9.VOUT` (pin 6) still feeds `J8.1` (`AI.CAM`) with `C54`/`C60`,
  and `AICAM_PWR` on GPIO23 still drives `ON` (pin 3). No firmware change.
- **The switch remains suitable at 5 V.** The TPS22917 is specified over
  `VIN` = 1–5.5 V; 5 V is inside that range, and the on-resistance and timing
  figures quoted for 3.3 V in ADR-0025 no longer apply. `CT` is tied to `VIN`
  through `C30` = 1 nF (about 4 ms rise, much lower inrush than open CT),
  `QOD` remains tied to `VOUT` through `R20` = 100 Ω, and the `ON` threshold
  (`VIH` 1.0 V) is still driven directly by the 3.3 V GPIO.
- **The UART stays 3.3 V.** The camera's UART is on the XIAO header D6/D7 and
  the Grove connector's pins 1/2 are 5 V-level I2C, not UART. `R18`/`R19` and
  the 3.3 V logic are unchanged. The RP2354B is not 5 V tolerant, so no 5 V
  signal may reach GPIO24/GPIO25.
- **The 5 V budget now includes the camera directly.** The module draws roughly
  100–150 mA while inferring (bench-measure it). It is now a fourth switched
  5 V branch rather than an `AMS1117-3.3` load, so the 3.3 V rail drops by that
  current and U2's dissipation falls by about `1.6 V × (0.1–0.15 A)` =
  0.16–0.24 W. ADR-0019 owns the 5 V budget; the low-line caveat (peak can
  approach 1.2–1.5 A) still applies.
- **Harness.** The module has two 5 V inputs: Grove `J3` pin 3 (`GROVE_5V_IN`)
  and XIAO header pin 14 (`VMAIN`). Feed **exactly one**; feeding both makes two
  on-module bucks drive `VCC_3V3`. Never put 5 V on XIAO pin 12 (`3V3`).

**Consequences**

- The camera sees its specified 5 V input, and its UART remains compatible with
  the 3.3 V RP2350 without level shifting.
- `C55` moves with the rail; `U9`, `C54`, `C60`, `R18`, `R19`, `R20` and the
  MCU-side nets do not change.
- The `AMS1117-3.3` loses the camera load. The 400 mA/0.68 W bound and the
  3.3 V pour/stitched-ground constraint from ADR-0025 stay as a conservative
  check; the rail is now closer to 250–300 mA.
- The 5 V peak grows by the camera's 100–150 mA and no longer passes through
  U2. At the 6.0 V cutoff the rail's low-line behaviour (W15) is the binding
  constraint; measure at the camera's peak with all branches on.
- Open items: confirm which module pins `J8.1`/`J8.2` land on (Grove pin 3 or
  XIAO pin 14), confirm the UART pins (XIAO D6/D7), and measure the camera's
  5 V current and inrush.
