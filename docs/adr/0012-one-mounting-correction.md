# One mounting correction, owned by the LiDAR cloud crate

The COIN-D6 sits in the robot's mount rotated half a turn from the direction the robot
faces, so bearings must be rotated 180° exactly once between the driver and the glass. The
cloud crate applies that rotation when it maps a spin into slots, which makes slot zero dead
ahead and lets the Front Sector and the gap analysis read slots directly. The radar widget
is therefore a neutral consumer: it plots the slot order the bench radar example drew, and
carries no mounting correction of its own. Applying the rotation in both places cancels it,
which draws the room inverted and pushes a dead-ahead return off the bottom edge of the
Panel.

**Status:** accepted.

**Considered Options**

- **Correct in the radar widget and leave the cloud in native bearings.** Rejected: the stop
  test and the gap analysis read slots, so slot zero must already be dead ahead for the
  safety logic; a display-only correction would leave the obstacle rule reasoning in native
  bearings.
- **Keep both corrections as the status quo.** Rejected: they compose to a full turn, so the
  radar disagrees with the bench example about which half of the screen is forward.
- **Split the rotation, half in each place.** Rejected: 180° does not divide into two
  meaningful halves, and a split would make both consumers carry a correction that neither
  can be read alone.

**Decision**

- The mounting rotation lives once, in the LiDAR cloud crate, which maps native bearings to
  slots with slot zero dead ahead. Its direction of application to native bearings is that
  crate's contract and is host-tested there.
- The radar widget plots the neutral slot convention and applies no mounting correction:
  slot zero is drawn at the top of the Panel and slot numbers increase clockwise on the
  glass, matching the bench radar example for the same scene.
- The direction is bench-derived, not asserted from the code: it is confirmed on the robot
  when the LiDAR is mounted, and an inverted room means the cloud's offset sign is wrong,
  not that the widget needs a compensating adjustment.

**Consequences**

- A consumer that plots slots, or reasons in slots, applies no correction. Converting the
  other way is the cloud crate's inverse, and belongs there with it.
- The earlier statements that increasing slots run counter-clockwise — in the cloud crate's
  documentation, in the widget's documentation and in the parent spec — are superseded by
  the bench-matched visual.
- Compensating elsewhere silently re-inverts the radar, so the convention is written in both
  crates' documentation and pinned by a widget test that asserts the drawn position of the
  first slots.
- The parent spec's Room Scan acceptance item (a plausible room, the same way up as the
  bench) becomes the check that keeps this decision honest.
