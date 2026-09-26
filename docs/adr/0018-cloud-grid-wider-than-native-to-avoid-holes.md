# The cloud grid is wider than the sensor's native resolution, so empty slots usually mean measured no-returns

The cloud is a fixed 360-slot grid of one-degree bins (ADR-0011), while the COIN-D6 emits at a
native 0.9° / 400-point resolution (ADR-0002). That mismatch is deliberate. Binning assigns a
sample to a slot by angular containment, and it reports a slot as `None` both when the slot
received no sample and when its samples were all no-returns. A grid at or under the sensor's
inter-sample spacing therefore manufactures empty slots, and an empty slot is indistinguishable
from a measured no-return — the one ambiguity ADR-0011 exists to prevent. 1.0° is chosen because
it is strictly wider than the sensor's sample spacing, so a slot is normally sampled and a `None`
slot usually means "measured, no return" rather than an unsampled hole. It is not an absolute
guarantee — the per-point angle correction noted below can still spread adjacent returns past a
boundary — only a strong default.

**Status:** accepted.

**Considered Options**

- **Narrow the grid to the native 0.9° (400 slots).** Rejected: at ~400 points per revolution
  the native spacing *is* 0.9°, the same as the slot width, so any phase drift empties slots.
  The rotor settles into 397–403 points (`WarmupConfig::settle_tolerance`), i.e. spacing
  0.893°–0.907°; at the slow end the spacing exceeds the width and slots go empty. An empty slot
  bins to `None`, exactly as a measured no-return does, so gap selection would see phantom blind
  spots (making `is_clear` wrongly conservative) and the radar would draw phantom dots. A
  sub-native grid is worse still: it adds empty slots by construction.
- **Carry the native points with no grid (a variable-length cloud).** Not pursued: it would
  avoid both holes and collisions and preserve the true bearings, but it turns the cloud from a
  fixed slot array into a point list, so the radar widget's neutral slot type and the
  slot-index geometry in the Front Sector and gap rules all change. It is a larger change than
  this decision needs, and the grid's bounded loss is acceptable.

**Decision**

- The cloud stays a 360-slot, one-degree grid (`lidar_cloud::SLOTS`, `SLOT_WIDTH_DEG`). The
  firmware reduces onto it explicitly (`resolution_deg = SLOT_WIDTH_DEG`) rather than taking the
  driver's native 0.9° default.
- The width is chosen to be **wider than the largest inter-sample angular gap** at the settled
  rotor speed, so angular-containment binning normally finds at least one sample per slot and a
  `None` slot is usually a *measured* no-return rather than an unsampled hole.
- The occupancy is not absolute: the driver applies a distance-dependent angle correction to each
  point, so two adjacent samples at very different ranges can be spread past a 1.0° boundary and
  leave the slot between them unsampled. The width is chosen to keep this rare, not to exclude it.
- Binning stays by containment with the nearest-valid-return collapse (ADR-0002): where two
  native samples fall in one slot the farther is dropped — a bounded, safety-conserving loss
  that can never hide a closer obstacle.

**Consequences**

- 1.0° is the *minimum safe* width for the settled band, not an arbitrary round number. Any move
  to the native 0.9° reintroduces holes and must not be made without re-deriving the occupancy
  estimate.
- The occupancy depends on the rotor settling near 400 points before streaming: a revolution
  carrying fewer than 360 points could leave even 1.0° slots empty. The warm-up phase
  (ADR-0010, ADR-0017) is what holds this.
- The cloud is ~11% coarser than the sensor (1.0° vs 0.9°) and collapses ~40 of 400 samples per
  revolution. That resolution cost is accepted in exchange for a `No Return` that is usually
  measured rather than a sampling artefact.
- The max-gap figure (360/397 ≈ 0.907°) is derived from the settle band, not yet measured on
  hardware. Revisit if a bench measurement of consecutive point-angle deltas (taken after the
  driver's angle correction) shows the true largest gap reaching 1.0°, or if full 360° coverage
  per revolution does not hold (a seam gap could empty a slot at any width). Narrow only with
  margin, never to the nominal 0.9°.
