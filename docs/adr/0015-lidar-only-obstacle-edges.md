# Only the LiDAR raises obstacle edges; a floor drop is its own edge

Obstacle detection was described by two producers of one edge. The LiDAR task raised
`ObstacleDetected` from its Front Sector test, and coast-and-avoid raised the same edge from
its own poll — while that poll read the obstacle flag *and* the floor-drop flag together, so
a floor drop was reported to the behaviour handler as `ObstacleSource::Lidar`. The handler
then wrote that boolean back into the perception module and read it out again, which latched
the LiDAR obstacle flag true on a floor drop and made the flag's value depend on the last
event rather than on the sensor. The glossary already said the rangefinder is a stair/drop
sensor and only the LiDAR reports obstacles, so the code contradicted the record.

**Status:** accepted.

**Considered Options**

- **Reshape the coast loop's re-raised edge into a floor-drop-shaped one.** Rejected once the
  facts were checked: the floor drop already raises its own edge, and its handler already
  sets the flag, drives its own indication and brakes. The coast loop's re-raise duplicated
  work that was already done by the producer that owns the measurement.
- **Keep the coast loop's re-raise for safety.** Rejected: the brake it caused was redundant
  — the loop had already braked directly, and the interrupt only cancelled a drive intent
  the mode does not use. What it added was a mislabel, a spurious latch and an extra blink.
- **Keep the handler's write-back as a defensive re-assert.** Rejected: the value it wrote
  was the value it had been handed, so it could only ever overwrite the sensor's truth with a
  stale copy of itself through the event bus.

**Decision**

- The LiDAR task remains the only writer of the obstacle flag. The permission's handler reads
  the flag rather than writing it, so the mutex no longer mirrors the lock-free flag.
- The obstacle event carries an edge, not state: a handler acts on what the state module
  holds.
- Coast-and-avoid raises no edge. It stays a consumer of the LiDAR's Front Sector edge and
  the floor drop's edge, and keeps its own direct brake, which is the only thing that
  actually stops it.
- The dead perception readers, the mirrored fields with no live reader, and the stale
  dead-code allowance hiding a live caller are deleted, with no new allowance added.

**Consequences**

- A floor drop stops the robot and reads to the operator as a floor drop. An obstacle stops
  it and reads as an obstacle. Neither claims to be the other.
- The obstacle flag has one writer, so it cannot be set by anything but the sensor.
- One detection has one brake path, so the drive subsystem is not interrupted twice with
  different latencies for the same event.
- A future obstacle source adds its own edge and its own handler rather than widening this
  one — which is what the reserved rangefinder pins are for.
