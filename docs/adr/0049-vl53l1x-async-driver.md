# VL53L1X rangefinder pair: in-repo async driver and 8-bit status

The robot's rangefinder modules are TOF0400C/VL53L1X parts, not the VL53L0X
assumed by ADR-0048. The pair is still the floor-drop safety sensor on two
dedicated I2C buses (ADR-0026): unswitched 3.3 V, low power, with `XSHUT` and
an active-low `INT` already wired. The available Rust drivers for this part are
blocking and poll-only, and no async driver preserves the raw range status and
the signal/ambient/SPAD diagnostics the test readout needs.

**Status:** accepted.

**Considered Options**

- **Use a blocking driver (`vl53l1x` or `vl53l1x-simple`) with a blocking I2C
  bus.** Rejected: it would make the two dedicated buses blocking and split the
  sensor path away from the async architecture. `vl53l1x-simple` is also
  poll-only: it never configures `GPIO1`, so an `INT`-driven task could not rely
  on the line at all.
- **Depend on `vl53l1x-simple` and wrap it.** Rejected: the reference discards
  the raw status into a small enum, exposes no diagnostics, and its
  initialisation omits the interrupt configuration and boot-NACK tolerance.
  Wrapping it would still leave the async and status gaps.
- **Vendor the ST VL53L1X ULD's default configuration blob.** Rejected: the
  88-byte tuning blob is opaque, large, and unnecessary for the short-range
  profile. The ported register subset is auditable.
- **Keep `edrv-vl53l0x`.** Rejected: it addresses the wrong part, with an 8-bit
  register map and a 4-bit status vocabulary that cannot represent the VL53L1X
  statuses. ADR-0048 recorded this choice only for the VL53L0X hardware.
- **One task per sensor.** Rejected for the same reason as ADR-0048: one task
  owning both keeps one command surface, one pair status, and one `select` over
  both `INT` lines and the command channel.

**Decision**

- **Driver.** Add a `vl53l1x` workspace driver crate that talks to the device
  over async I2C. It owns the I2C device, the open-drain `XSHUT` output and the
  active-low `INT` input, and exposes `new`, `init`, `read`, `wait_ready`,
  `reset` and `standby`. It never depends on the `rangefinder` policy crate.
- **Register model.** Every register is addressed with 16-bit big-endian bytes,
  not the VL53L0X's 8-bit map. The register names follow the ST ULD.
- **Identification and boot.** `init` pulses `XSHUT` low then high, reads the
  model ID at `0x010F`, and requires `0xEACC`; any other value returns
  `InvalidDevice(raw)`. It then soft-resets and polls the firmware status for up
  to 500 ms. A generic async-I2C error type cannot distinguish a NACK from
  another bus error, so every error during this poll is treated as "still
  booting"; a device that never answers ends as `Timeout`.
- **Interrupt wiring.** `GPIO_HV_MUX__CTRL` is written to `0x11` (range/error
  interrupts, active low) and `SYSTEM__INTERRUPT_CONFIG_GPIO` to `0x20` (new
  sample ready). The poll-only reference omits both; without them `GPIO1` may
  never assert or may be active high. Active low is safer for a powered-down
  module, whose line idles high through the pull-up. `read` also checks
  `GPIO__TIO_HV_STATUS` bit 0 before consuming a block, so it does not depend on
  the interrupt edge alone.
- **Measurement profile.** Short distance mode (about 1.36 m, the best
  ambient-light tolerance) with a 33 ms timing budget and a 50 ms
  inter-measurement period. The period is scaled by the oscillator calibration
  value read at init. The floor-drop threshold remains policy, not driver.
- **Calibration timing.** ST's one-time VHV/phasecal setup runs at the top of
  the first `read`, not in `init`, because it consumes that measurement's
  phasecal result. A flag guards it, the DSS update runs on every read, and the
  caller still receives a calibrated first reading.
- **Read path.** A not-ready read clears the interrupt so the task cannot spin,
  then returns `NotReady`. A ready read consumes the 17-byte result block, runs
  the one-time calibration, updates the DSS, decodes the reading, and clears the
  interrupt only after the block is consumed — that clear re-arms the next
  measurement.
- **Status vocabulary.** The raw 8-bit `RESULT__RANGE_STATUS` is decoded into
  `Valid`, `MinRangeClipped`, `NoWrapCheck`, `SignalFail`, `OutOfBounds`,
  `MinRangeFail`, `SigmaFail`, `WrapFail`, `XtalkFail`, `HardwareFail`,
  `Synchronization`, `NoUpdate` or `Unknown`. Raw `9` with a zero stream count
  is `NoWrapCheck` and deliberately classifies as **No Target**, not **Valid**;
  a min-range-clipped result classifies as **Valid**. Only the recovery set
  (hardware, sigma, wrap, crosstalk, no-update and unknown) is a fault.
- **Provenance.** The initialisation and read sequences are ported from
  `vl53l1x-simple` 0.2.0 (MIT OR Apache-2.0), async-ified and adapted to this
  API. The source and licence are recorded in `THIRD_PARTY_LICENSES.md`; the
  driver carries no dependency on the blocking crate and no ST ULD blob.

**Consequences**

- ADR-0048's driver choice is superseded. Its on-demand lifecycle, floor-drop
  detector, pair state machine, gate, policy constants and test-mode decisions
  remain valid and unchanged.
- The driver crate, its host tests and CI job land in ticket 01; tickets 02 and
  03 then switch the panel model, telemetry and firmware task to it. There is no
  second driver path left after that switch.
- Short distance mode is the delivered profile. Long and medium modes remain one
  constant away for the bench if the floor return proves weak.
- The gain correction is applied once, and the diagnostics carry the raw 7.7
  fixed-point signal and ambient rates plus the effective SPAD count, so no
  cause is hidden from telemetry or the test readout.
- A zero oscillator calibration falls back to the reference's 256 instead of
  producing a zero inter-measurement period; a zero measured frequency fails
  bring-up with `Timeout` instead of dividing by zero.
- The driver still needs the manual bench pass (ticket 04) before the floor-drop
  threshold and profile are trusted on the real modules.
