# A Procedure is a lifecycle the producers call, not a runner they hand a body to

Nine producers — six test modes and three calibrations — each repeated the same six steps:
arm the stop, publish the activity, phase the percent and detail, record the terminal
outcome, release the single-active slot, raise the completion event. Two of them repeated
the stop latch's four forwarding functions verbatim. A single runner that every producer
hands a body to is the obvious collapse, and it is wrong here.

**Status:** accepted.

**Considered Options**

- **One runner all nine producers pass a body to.** Rejected: it would need roughly as many
  parameters as the variance it absorbs. Distance calibration's lifecycle outlives its task
  and is driven from the value-entry screen across a `begin`/`commit`/`abort` sequence; the
  magnetometer is an eight-state body with its own phase results; three producers publish no
  terminal outcome and raise no event; three publish a terminal outcome only on failure; the
  two families claim different slots and raise different events. The deletion test fails:
  the complexity reappears as runner parameters.
- **Collapse only the verbatim stop-latch duplication and leave the rest.** Rejected: the
  repetition beyond the latch is real and is what makes a new Procedure a many-file edit.
- **Move the lifecycle into a host-testable crate.** Rejected: its whole content is embassy
  channels, statics and a critical-section mutex, so a crate would have no pure behaviour
  behind it — a seam with nothing to test.

**Decision**

- A Procedure's lifecycle lives in one module that the producers call: arm the stop, publish
  the phase and percent to the Activity State, publish the terminal outcome, release the
  single-active slot, raise the completion event. A producer supplies its body and its phase
  words.
- The slot claim and the completion event are parameters of the lifecycle, so each family
  keeps its own.
- Distance calibration and the magnetometer keep their own phases and call the lifecycle
  around them.
- The stop latch keeps its per-family instances, so a stop aimed at a test mode cannot reach
  a calibration.

**Consequences**

- A new Procedure is a body and its phase vocabulary, and the lifecycle around it is written
  once.
- The rule that a finished Procedure is presented as a Result Report has one implementation,
  rather than being re-derived by each producer and again by the Panel.
- A reader looking for "what happens when a procedure stops" finds one module instead of
  nine.
- The lifecycle is firmware behaviour and stays compile- and bench-verified; the display
  policy it feeds is host-testable through the model crate (ADR-0013).
