# The LiDAR task owns its power lifecycle; a mode enables and disables the sensor

The COIN-D6 is powered on demand (ADR-0010), and its driver is owned for the whole runtime
by a single core1 task. Access to *that* task's service was leased and ref-counted, so that
two modes could hold the powered sensor at once. But only one mode ever needs perception at
a time — the panel is modal, and the test-mode and autonomous-mode slots each admit one mode
— so the reference count only ever papered over a teardown race, while forcing every consumer
to thread a token, to reproduce the bring-up's failure and retry semantics in its own docs,
and to reason about an acquisition generation. This replaces the lease with a one-way command.

**Status:** accepted. Supersedes the ownership paragraph of ADR-0010.

**Considered Options**

- **Keep the lease and its ref count.** Rejected: there is never a second holder, so the count
  is machinery for a case that does not exist; its one real effect is to hide an ordering
  accident in the panel rather than fix it.
- **Let an orchestrator (the UI) be the single enable/disable writer.** Rejected: the panel is
  just one channel that may need to command the sensor, and it would have to learn every
  mode's end, including the modes that finish on their own.
- **Let the task derive need from the system's state.** Rejected: to know when to power down it
  must count consumers, which is the lease again, and it couples the task to the procedure
  model.

**Decision**

- A mode enables the sensor when it needs perception and disables it when it is done. The
  commands are one-way on a dedicated `Channel`, as motor and flash commands are; the task
  owns power, warm-up, retries, streaming and power-off, and keeps the driver for the runtime.
- Readiness is observed through the lock-free `status()` — `Off`, `Warming`, `Streaming`,
  `Failed`. The task does not signal; callers poll.
- Bring-up is edge-triggered: a failed attempt holds at `Failed` until a `Disable` clears it,
  and the clear publishes `Off` so the caller can observe the reset land. A duplicate `Enable`
  is ignored, so a caller cannot spin retries by polling; a caller retrying a latched failure
  clears it before enabling, so its own bring-up — not the stale `Failed` — decides the outcome.
- The data path is unchanged: the enabled sensor publishes its cloud, its obstacle flag and
  its `ObstacleDetected` edge, and serves every reader.
- `Lease`, `AcquireError::Busy`, the acquisition generation and both reply channels are
  deleted.

**Consequences**

- A departing mode's `Disable` can land after an arriving mode's `Enable`, powering the sensor
  off under a mode that has just enabled it. The window is the departing task's teardown
  (brake and driver-disable, a few hundred milliseconds) and is reached only by navigating
  three screens inside it. Room Scan widens the same window in miniature: its helper converges
  on the screen's latch when it is next scheduled, so a leave just before a new mode enables
  can be delivered as a `Disable` after that `Enable` — a scheduler delay rather than a
  teardown. Both are accepted rather than guarded, because a guard is either a count or an
  identity token, which is the lease this decision removes. Revisit if a bench or field report
  ever shows the sensor dropping under an active mode.
- There is no compile-time reminder to hand the sensor back, so a forgotten `Disable` keeps
  the sensor powered; a bench run shows that immediately.
- This rests on *at most one mode needs perception at a time* (the glossary's **Enabled**),
  the same invariant the panel and the mode slots already enforce.
