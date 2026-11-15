# Scene statuses do not drive rangefinder recovery

The VL53L1X reports why a measurement was not usable. ADR-0049 put `SigmaFail`,
`WrapFail` and `XtalkFail` in the recovery set, so a sustained streak of any of
them reset the sensor. A bench run showed the consequence: a `WrapFail` (raw 7,
a wrapped far target) kept a healthy sensor in the reset loop, consumed the
three attempts in about a second, and latched `Failed` until Test Mode was
stopped and restarted. The sensor was not wedged; the target was simply beyond
the unambiguous range.

A reset cannot change a wrapped, noisy or over-threshold target. Treating those
statuses as a hardware fault also hides the real cause from the operator.

**Status:** accepted. Supersedes ADR-0049's recovery set.

**Considered Options**

- **Keep resetting on every fault status.** Rejected: the reset cannot clear a
  scene condition, so the only effect is to exhaust the attempt budget and flap
  the pair.
- **Treat scene statuses as valid readings.** Rejected: the range is not
  trusted, so the detector must not use it as a floor decision.
- **Hold the last decision and fail safe, without a reset.** Accepted.

**Decision**

- The recovery set is `HardwareFail`, `NoUpdate` and `Unknown` only. They can
  indicate a wedged device that a reset may clear.
- `SigmaFail`, `WrapFail` and `XtalkFail` stay `Reading::Invalid`. They hold the
  last safety decision, fail safe to `Drop` after the 200 ms invalid hold, and
  never reset a healthy sensor. The sensor keeps ranging so it can produce a
  valid floor again.
- The firmware logs the sensor side and the fault status, raw code, signal,
  ambient and SPAD diagnostics when recovery starts, and says a reset succeeded
  rather than claiming the sensor recovered.

**Consequences**

- A wrapped far target now latches `Drop` (motion stays refused) until two valid
  floor readings clear it, instead of latching the sensor `Failed`.
- A genuinely wedged sensor still resets and still latches `Failed` after three
  attempts. A fresh `Disable`/`Enable` releases it.
- The drive gate is unchanged: it still gates on the aggregate floor-drop flag
  and the pair state.
