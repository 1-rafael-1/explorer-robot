# Odometry: dead reckoning corrected by tilt-gated scan-to-map matching

The robot needs a continuous **Pose** in the **World Frame** from onboard sensing
alone, and every available source is individually inadequate. Encoders give good
longitudinal speed but **infer direction from the motor command** (the PWM input
counter cannot decrement) and slip 10–30 % in skid-steer turns. The ICM-20948's DMP
gives trustworthy roll/pitch but `Axis6` yaw drifts without bound. A single 2D
spinning LiDAR is the only source of absolute `(x, y, yaw)` correction, but
scan matching assumes a flat world, and terrain tilt violates that. The decision is
to fuse them by role: **encoders and IMU predict, and a scan-to-map matcher supplies
a low-rate absolute pose update admitted only while the robot is level.** The matcher
aligns each spin against an incrementally built occupancy grid (**Odometry Grid**)
grown from the robot's own scans and never persisted. Tilt is handled by *per-point*
masking rather than rejecting whole spins, because tilt is bearing-dependent —
forward beams dip under pitch while lateral beams stay horizontal — so discarding the
whole scan would throw away usable geometry. Dead reckoning ships first, with the
matcher added behind it.

## Considered Options

- **Encoder-only dead reckoning** — rejected: yaw from the track differential is
  unusable on a skid-steer platform.
- **Encoders + IMU only** — rejected: drift is unbounded. Adequate per leg, not per
  mission, and not a base to build mapping on.
- **Scan-to-scan matching** — rejected: cannot bound drift and cannot re-acquire after
  a tilt-gate closure, which is precisely what the stairs/slope use case needs.
- **Scan-to-map with whole-scan tilt gating** — rejected: on a slope it discards the
  near-field and lateral geometry that is still valid.
- **Full SLAM with loop closure (Cartographer-class)** — rejected: needs a 64-bit CPU
  and 16 GB RAM.
- **Adding a ground-relative sensor** — see ADR-0006; optical flow rejected, Doppler
  radar held as a reserve.

## Consequences

- Pose becomes a first-class state module (**Pose State**) in the documented lock
  order, because the drive intents, gap analysis, UI and mission layer all read it.
- The **Odometry Grid** is session-local: no flash persistence and no pre-load. The
  robot seldom powers up in the same pose it powered down in, so a persisted grid
  would need re-anchoring anyway.
- The magnetometer (`Axis9`) is enabled as a **weak, high-variance absolute yaw
  observation** — the only absolute heading available while the gate is closed — with
  a runtime fallback to `Axis6` when mag data is bogus, extending the existing
  start-up-time `effective_mode` downgrade.
- Encoder covariance becomes **state-dependent**, inflated by commanded turn rate and
  by unpowered states where the sign is unknown.
- The scan matcher runs on core1 and the EKF on core0; tuning parameters are
  centralised and documented in `.scratch/odometry/spec.md` so they can be adjusted
  without re-deriving the design.
