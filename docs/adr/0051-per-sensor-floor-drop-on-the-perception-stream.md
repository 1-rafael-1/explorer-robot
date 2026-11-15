# Per-sensor floor-drop bits on the Perception stream

The Remote App shows the live `PERCEPTION` stream, but it only carried one
aggregate `FLOOR_DROP` bit. The two VL53L1X rangefinders are separate hardware
and either can trip alone, so the app could not say which one saw the ledge; the
operator had to read the panel's Rangefinder Test to tell front from rear.
ADR-0048 deliberately kept per-sensor detail panel-only because the first
Remote only needed the aggregate safety flag.

**Status:** accepted. Supersedes ADR-0048's consequence that per-sensor detail
stays panel-only.

**Considered Options**

- **Keep the aggregate bit.** Rejected: an operator driving from the Remote
  cannot tell a front or rear drop from the live display, and the Events view
  cannot say which sensor tripped either.
- **Add a second `FLOOR_DROP` event with a sensor id.** Rejected: Events are
  edge-triggered, so a live two-sensor display would have to reconstruct state
  from edges and would show the wrong value after reconnects.
- **Add a new `RANGEFINDERS` stream.** Rejected: the `PERCEPTION` payload
  already has six reserved bits for exactly this kind of extension, and a new
  stream would duplicate the obstacle and aggregate flags.
- **Use the reserved bits for front and rear.** Accepted.

**Decision**

- `PERCEPTION` bit 2 is the front rangefinder's floor drop and bit 3 the rear's.
  Bit 1 stays the aggregate `FLOOR_DROP`, set whenever either sensor drops, so a
  peer that only understands the aggregate keeps working. This is an additive
  optional field, so the link protocol moves to wire minor 1 / crate `2.1.0`
  (ADR-0040); the payload length and layout are unchanged.
- The robot sets the per-sensor bits from the rangefinder task's held
  per-sensor decisions. The same flags fill `GET_STATUS` because it shares the
  `Perception` type.
- The daemon exposes `floor_drop_front` and `floor_drop_rear` beside
  `floor_drop`; each is `null` when an older robot sets the aggregate without
  the per-sensor bits.
- The Remote App labels both sensors explicitly (`Front: drop|floor|unknown`,
  `Rear: drop|floor|unknown`) and always names the obstacle flag, so a clear
  flag never renders as a bare `Clear`.

**Consequences**

- A new robot and a current daemon show which sensor tripped. An older robot
  still shows the aggregate; the app falls back to a single `Floor drop` line
  when the per-sensor split is absent.
- The `FLOOR_DROP` event and the drive gate still use the aggregate; the new
  bits are display-only and never weaken the safety decision.
- The panel's Rangefinder Test remains the detailed readout (range, status,
  signal, ambient, SPADs); the Remote App gets only the floor decision.
