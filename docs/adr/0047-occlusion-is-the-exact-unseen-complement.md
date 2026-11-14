# Occlusion is the exact unseen complement

The protrusion rule in ADR-0046 asked for two significant edges around a near run. Live scans
showed why that is too brittle. A person close to the robot appears as a fragmented run of
30–45 cm returns with 1-slot No Returns and small steps, so the rule rarely fired; the long solid
chords at single range steps (72→281 cm, 99→248 cm, 289→427 cm) were not part of any protrusion,
so they stayed solid. The operator asked for the truth: everything the scan has not seen should be
marked as unseen.

**Status:** accepted.

**Considered Options**

- **The protrusion rule (ADR-0046).** Rejected: it needed two significant valid edges, which a
  close, fragmented object and a No Return-bounded object do not have.
- **A near-cluster threshold (for example 33 % of the far background).** Rejected: the threshold
  is arbitrary, and it merges an object with the near wall beside it.
- **Only the slivers behind significant steps.** Rejected: it marks the chords but not the whole
  unseen area.
- **The exact complement.** Chosen: it is the ray model itself, it needs no object heuristic, and
  it is honest about a mostly red pane.

**Decision**

- The Remote App builds the **Seen Polygon**: the star-shaped region from the scan pose out to
  every measured return, in slot order. The polygon closes through the returns when the seam is
  valid, so the last and first returns do not leave a false wedge; a No Return contributes a
  radial notch to the pose, so the null sector stays unseen. A No Return run that crosses the
  seam closes through the pose.
- The **Occlusion Shadow** is the complement of the Seen Polygon. The app fills the whole viewport
  outside it with translucent red; there is no cap other than the viewport.
- An **Occlusion Edge** is either an adjacent valid pair whose range step is at least both
  **20 cm** and **25 %** of the nearer return (the greater of the two), or the chord between the
  two returns that flank a No Return notch. The cloud does not draw a solid chord across a
  significant step; the app draws the chord dotted soft red. A No Return notch has no measured
  surface, so that chord starts and ends at measured returns and never runs to the viewport.
- The fill and the No Return notch chords are computed from the scan's own pose on every render
  and painted under the cloud. The significant-step edges are painted above the cloud, so a thin
  wall outline cannot hide them. The layer is muted to the stale palette when the scan is stale
  and never outlives its scan.
- The radar carries a compact legend in the top-right naming the measured return, the near return,
  the unseen fill, the Occlusion Edge, the robot and the Target. It is inert, so it never blocks a
  radar tap.
- The Remote App only, display only. The Link Protocol, daemon, firmware and the panel's Room Scan
  radar are unchanged. The 20 cm and 25 % values are display constants to confirm on the bench.

**Consequences**

- The pane is mostly red whenever the measured returns are the room boundary. That is the point:
  the red is the part of the world the scan has not seen.
- A No Return paints a red radial wedge closed at its measured returns by a dotted chord, because
  it is unknown, not free. A short null run may be a thin red spoke at the radar scale.
- The legend carries the words, so the display does not rely on hue alone.
- The red also appears behind far walls. The exact model does not distinguish an object in front
  of visible background from the boundary of the visible area.
- Self-occlusion, such as the robot's own body, is shaded like any other unseen region. The lidar
  mounting and the near-field returns should be checked on the bench.
- The exact complement is unbounded; the viewport is the only cap. The fill is recomputed on every
  render, so it cannot outlive the scan.
