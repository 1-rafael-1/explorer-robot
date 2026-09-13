# Calibration is a validated value type

`Calibration` maps raw touch counts onto the screen's pixel grid, and that
mapping is arithmetic with real preconditions: each axis is linearly interpolated
by dividing by the difference between its two raw endpoints, and the result is
clamped to `0..width - 1` (or `0..height - 1`). Equal endpoints make the divisor
zero, and a non-positive dimension makes the clamp range invalid; both panic. The
struct shipped with public fields, so any caller could construct such a value and
nothing documented the panic contract.

**Status:** accepted

**Considered Options**

- **Public fields plus a documented panic contract:** the minimal change, but it
  leaves the invalid states representable and merely warns about them.
- **Private fields plus a panicking `const fn new` only:** keeps shipped
  calibrations const-constructible, but gives a runtime-measured calibration no
  way to reject a bad value without panicking.
- **Private fields plus a panicking `const fn new` and a fallible `try_new`:**
  chosen. The const path keeps the shipped constants const; the fallible path
  lets runtime callers handle a bad measurement.

**Decision**

- The six fields are private. Construction goes through `Calibration::new`
  (`const`, asserts, documents `# Panics`) or `Calibration::try_new` (returns
  `Result<Self, CalibrationError>`).
- The enforced invariants are `width >= 1`, `height >= 1`, and distinct raw
  endpoints on each axis (`x1 != x2`, `y1 != y2`).
- Endpoint magnitudes are deliberately **not** bounded. Raw endpoints are
  legitimately extrapolated from the inset calibration targets, so limiting them
  to the 12-bit raw range would reject valid calibrations. `to_pixels` instead
  widens its intermediate arithmetic to `i64`, so no endpoint values that fit an
  `i32` can overflow it.
- `CalibrationError` has two variants — `NonPositiveDimension` and
  `DegenerateAxis` — and derives the crate's error shape
  (`Debug, Clone, Copy, PartialEq, Eq`). It does not implement `Display` or
  `core::error::Error`, matching `touch-async::Error`.
- `Calibration::REFERENCE` and `Calibration::MEASURED` are built through `new`,
  so an invalid shipped constant fails the build.
- `Default` remains `Calibration::REFERENCE` (the spec's shipped reference
  defaults), documented as vendor values for first bring-up only; this panel uses
  `MEASURED`.

**Consequences**

- Callers can no longer build a `Calibration` with a struct literal, and cannot
  read the endpoints back: the fields are private and there are no accessors
  until a persistence or logging caller needs them.
- `to_pixels` has no panic path for any value constructible through the public
  API, so the "public API can panic on a division by zero or an invalid clamp
  range" review finding is closed.
- The const constructor keeps both shipped calibrations as compile-time
  constants.
