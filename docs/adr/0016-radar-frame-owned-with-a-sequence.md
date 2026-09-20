# The Room Scan frame crosses the model's seam owned, with a sequence

The Room Scan frame — one optional distance per one-degree slot — was represented three
times: in the perception module's cloud, in the copy the Panel's tick fetched out of the
mutex, and in the model's retained frame. Each tick copied the whole array a second time and
compared it in full to decide whether to redraw, while the cloud's own sequence counter,
which exists to make exactly that decision cheap, was discarded at the seam.

**Status:** accepted.

**Considered Options**

- **The model stores a borrow of the frame.** Rejected: it puts a lifetime parameter on the
  model, on the task's state and on every test, and the model cannot outlive a fetch it does
  not own.
- **The model stores only the sequence; the frame is passed at render time.** Rejected: it
  shrinks the model's retained state by the array's size, but it splits one invariant —
  which frame is on screen — across two calls in two owners. A caller that updates the
  sequence without passing the matching frame freezes the screen silently, and the model has
  no way to detect it. It also makes the radar the only screen whose content does not live in
  the model, while the System Info and status snapshots continue to.
- **Narrow the slot type so the frame is half the size.** Deferred, not rejected: it reaches
  into the cloud crate, the gap analysis and the cloud tests, so it belongs in its own change
  rather than in the ticket that moves the seam.

**Decision**

- The model keeps its frame. It is set with the cloud's sequence as the change token, so the
  redraw decision is a counter comparison and the per-tick copy of the array goes.
- The model continues to own every screen's content, so nothing about which screen is showing
  can disagree with itself.
- The cloud crate keeps learning no firmware types, and the slot convention keeps its single
  owner (ADR-0012).

**Consequences**

- The Panel's tick no longer spends a whole-array copy and comparison per refresh, and the
  sequence counter earns its place.
- A frame reaches the model with its identity attached, so the same frame twice reports no
  change and a new frame always reports one.
- The model's retained state stays the size of one frame. That is the cost of keeping one
  owner for one invariant, and it is small beside the Panel's framebuffer, which is fifty
  times larger and resident regardless.
