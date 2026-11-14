# An occlusion shadow is inferred for display only

The Remote App drew every adjacent pair of valid LiDAR returns as one polyline, so a
sharp range step at the edge of a near object — a person standing in front of the robot —
was stroked as a chord in the same blue as a measured surface. The operator could not tell
a boundary of unseen space from a real return. The app now derives an **Occlusion Shadow**
from a near protrusion and draws it as a translucent red fan with dotted **Occlusion
Edges**. The robot, the Link Protocol and the daemon carry no such concept: it is a display
inference, never a No Return, never free space, and never an input to Go or the Target
Point (ADR-0038, ADR-0043).

**Status:** superseded by ADR-0047. The display-only, app-only principles below remain; the protrusion rule is replaced by the exact unseen complement.

**Considered Options**

- **Shadow every range step.** Rejected: a concave room corner is a range step with
  nothing hidden behind it, so the overlay would mark measured space as unseen.
- **Shadow runs of No Returns.** Rejected: a No Return is unknown, not far (ADR-0011).
  Marking it as occluded would turn "we do not know" into "we know it is hidden", and a
  black or specular surface would paint a shadow that is not there.
- **Fill the whole angular sector from the robot origin.** Rejected: it tints the measured
  space between the robot and the obstacle, so the red stops meaning "cannot see".
- **Extend the fan to the canvas edge.** Rejected: unbounded red dominates the radar.
  Capping at the local background range ties the layer to measured returns.
- **Compute it on the robot.** Rejected: the panel's Room Scan radar draws marks only and
  perception has no consumer for an inferred shadow; keeping the heuristic in the app
  keeps it off the safety path.
- **A one-slot protrusion.** Rejected by geometry: two Occlusion Edges through a single
  return coincide, so the fan has zero width. A minimum of two adjacent returns both
  avoids a degenerate fan and rejects a lone outlier.

**Decision**

- An Occlusion Shadow is a **near segment**: a run of valid returns bounded in slot order
  by a fall and the next rise, where each step is at least **50 cm** and both returns at
  the step are valid. A No Return neighbour disqualifies the segment; a null is unknown,
  not far. The segment must hold at least **two** adjacent returns, and a candidate must
  be shorter than half the scan: its longer complement is the near background around a
  far recess (a wide doorway or a far corner), not a protrusion.
- The shadow's inner edge is the segment's measured contour, its sides are the two
  Occlusion Edge rays from the contour's endpoint returns, and its cap is a straight line
  at the greater of the two flanking far returns. All distances are measured from the
  scan's own pose.
- The polyline no longer draws a chord across either Occlusion Edge; the dotted edge
  replaces it. Every other range step keeps its surface chord.
- The layer is painted under the live cloud, recomputed on every render, and muted to the
  stale palette when the scan is stale. It does not outlive its scan.
- The Remote App only. The Link Protocol, daemon, firmware and the panel's Room Scan radar
  are unchanged. The 50 cm threshold is a display constant to confirm on the bench.

**Consequences**

- A lone convex wall corner (one qualifying step, the other side continuous) is not marked
  at all, and a doorway jamb's occlusion into a farther room is not filled. Both need
  ray-casting rather than a protrusion rule; that is deliberately out of scope.
- A thin object that occupies one slot at this resolution casts no fan. The shadow is
  honest to the sampled data rather than to the object.
- Overlapping shadows are drawn independently and darken slightly where they meet; that is
  acceptable for a display layer.
- The overlay is an inference, so the glossary and the radar's accessible label say so.
