# A no-return bin is neither an obstacle nor a clear angle

The LiDAR point cloud stores each one-degree bin as an explicit `Option<f32>` rather than a
`0.0` sentinel, and the two consumers that read it interpret "no return" in opposite,
deliberate ways: a bin with no return is not an obstacle, and it is not a clear angle
either. The old sentinel made a dropout a *drivable* angle, which is the one reading a
blind spot must never have.

**Status:** accepted

**Considered Options**

- **Keep `0.0` as the sentinel.** Rejected: it makes "no data" indistinguishable from a
  distance, and it forces every consumer to remember which meaning applies.
- **Carry the driver's native `Option<NonZeroU16>` millimetres into the cloud.** Rejected:
  it halves the memory, but it pushes millimetre-to-centimetre conversion into every
  consumer instead of one boundary.
- **Treat a dropout as clear, as the old gap analysis did.** Rejected: the lidar returns
  no reading for a black or specular surface as readily as for open space, so "clear" would
  mean "drive toward the thing we could not see".

**Decision**

- The cloud is 360 bins of `Option<f32>` in centimetres; `None` is no return, and the
  millimetre-to-centimetre conversion happens at the driver-to-firmware boundary.
- The obstacle test ignores `None` bins: a dropout cannot phantom-brake the robot.
- Gap selection treats `None` as not clear: a blind spot can never extend a navigable gap.
- Both rules live in the host-testable `lidar-cloud` crate, with the two behaviours named
  and tested rather than implied.

**Consequences**

- The obstacle test and the gap test deliberately give opposite answers to the same input.
  That is the point, and a reader who "fixes" the inconsistency will reintroduce the
  hazard.
- Gap-following navigation becomes more conservative: an absorbent surface at range reads as
  an obstacle rather than as an opening.
