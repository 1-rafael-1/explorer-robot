# Power the LiDAR on demand, not continuously

The COIN-D6 is powered down whenever no mode needs it, rather than left spinning from
boot. The sensor is the most expensive part on the robot and a spinning dTOF head is a
wear item; keeping it running through motor tests, calibration and idle trades its life
for nothing. The cost is a warm-up wait at every mode entry and a power lifecycle to get
right.

**Status:** accepted

**Considered Options**

- **Spin from boot and always.** Simpler and with no warm-up stall, and the earlier
  firmware took this for granted. Rejected on part life and power draw.
- **Pre-warm when the operator opens the relevant menu, or keep a grace period after a
  mode exits.** Rejected for now: it couples menu navigation to sensor state for a wait
  that is legible on screen. Revisit if the stall proves annoying in use.

**Decision**

- A persistent core1 task owns the driver for the whole runtime and exposes
  `acquire()` / `release()` / `status()`, with ref-counted ownership returning `Busy` on a
  double acquire.
- `acquire()` runs `power_on()` → `start()` → `warm_up()` → streaming. A `start()` failure
  is tolerated; warm-up is the real proof that data flows.
- Warm-up is bounded by a wall-clock timeout at the call site, because the driver has only
  byte-count watchdogs. On timeout the task power-cycles and retries a bounded number of
  times before reporting `Failed`.
- `release()` stops the device, deasserts the power pin, and clears stale state: the stored
  cloud becomes `None`, the obstacle flag is cleared, and a cleared `ObstacleDetected`
  edge is raised if the flag had been set.
- Coast-and-Avoid acquires the LiDAR on entry and **refuses to start** if the acquisition
  fails, rather than driving with only the downward rangefinder for obstacle sense.
- The LiDAR is off at boot; motor tests, IMU tests, calibration and idle never power it.

**Consequences**

- Entering Coast-and-Avoid or Room Scan may wait several seconds while the rotor reaches
  steady state; the screen shows the warming state so the wait does not read as a hang.
- Consumers must treat an absent cloud as unknown, not as an empty room: a `None` snapshot
  means the sensor is off or not yet warmed.
- A stopped sensor can leave an obstacle flag behind, so `release()` carries the
  obligation to clear it in the same step.
- The power MOSFET stays wired even though it is now load-bearing rather than optional.
