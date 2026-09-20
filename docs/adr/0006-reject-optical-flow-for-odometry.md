# Reject optical flow as an odometry sensor

Bounding track slip requires a ground-relative speed source, and a downward
optical-flow sensor was the obvious candidate. Every available part fails this
chassis's requirements. Mouse-class sensors (ADNS-3080/9800, PMW3360/3389) need a
**2.1–2.6 mm** standoff, forcing a ground-hugging skirt that defeats the ground
clearance the chassis is being designed to gain. The one dedicated near-field
part, PixArt's **PAA5100JE**, is specced for **15–35 mm** and loses tracking as
soon as the surface departs that window — constantly, on grass, rock and stair
edges. The drone family (**PMW3901**, 80 mm–infinity) does survive the clearance,
but optical flow measures *angular* velocity, so converting to ground speed
divides by the standoff: a rover whose ground distance varies continuously gets a
proportionally wrong speed, and the signal collapses on low-texture ground at that
range. We therefore drop optical flow and rely on the 2D LiDAR scan matcher as the
absolute odometry reference, with dead reckoning between corrections.

## Considered Options

- **Mouse-class optical flow** (ADNS-3080 2.40 mm, ADNS-9800 2.4 mm, PMW3360
  2.2–2.6 mm, PMW3389 ≈2.1 mm) — rejected: requires the low skirt we are trying to
  avoid.
- **PixArt PAA5100JE / PAA5100JE-Q** (15–35 mm) — rejected: the 35 mm ceiling is
  exceeded by ordinary terrain, and the standoff must be near-constant.
- **PMW3901 drone family** (80 mm–infinity) — rejected: standoff-proportional
  scale factor makes it wrong on varying ground distance, and it needs texture.
- **24 GHz Doppler radar** (OmniPreSense OPS243-A ~$224, uRAD Doppler ~€190,
  Infineon BGT24LTR11-based) — the correct technology for this clearance budget if
  ground-relative speed is ever needed, since it measures range-rate directly and
  ignores standoff. Held as a documented reserve, not adopted: it yields only
  longitudinal speed, gives no heading, and costs two orders of magnitude more
  than an encoder.

## Notes

- Research: `.scratch/optical-flow-sensor-working-range.md`.
- The reserve is worth revisiting only if bench data shows longitudinal track slip
  dominating the error budget; it does not address yaw.
