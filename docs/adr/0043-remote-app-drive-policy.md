# The Remote App drives by point-and-go, and the robot owns motion safety

ADR-0038 settled click-to-drive with a preview, clamp-don't-refuse, and a set of prototype policy
values: a 3 m distance cap, an 18 cm clearance standoff, a 30 cm refusal floor, a 1.5 degree rotate
skip, a 2 degree / 5 cm plan invalidation, a 350 ms Go gate and a fixed completion timeout. The
prototype demonstrated them, but the grilling established that none is derived from the wire or the
robot: the Relay's only distance bound is `i16` centimetres, and whether a very short leg completes
is the robot's drive tolerance, not the app's business. The app is explicitly not a safety surface
(ADR-0038); the robot's drive protection is. The operator wants to tap a place and have the robot go
there, and to own the decision about whether it is safe to start.

The Remote App therefore commits to a **Target Point** in the World Frame and reports what the robot
did. It does not plan around obstacles, cap the distance, refuse a short leg or warn about an
unvalidated scan. The live scan is display, not a planning input. Future multi-waypoint paths,
target-point pathfinding and area exploration belong to the robot's drive and spatial memory, not to
the app; the Target Point and the world renderer are the seam they will use.

**Status:** accepted. Supersedes ADR-0038's *Click-to-drive* policy values and its clamp-don't-refuse
rule; ADR-0038's JSON boundary, world-frame renderer, two-tap preview, two-command/two-claim sequence
and no-map rule still stand. The context-sensitive Stop below is an amendment to ADR-0038's Stop
rule.

**Considered Options**

- **Keep the ADR-0038 caps and clamps.** Rejected. The values were prototype constants, not wire or
  robot limits. Clamping to `nearest - 18 cm` also aims the robot's centre at the obstacle surface;
  the standoff was a heuristic. The robot's drive protection already refuses to crash.
- **Keep a 3 m cap and a 30 cm floor only.** Rejected for the same reason: neither is a wire or
  robot bound, and a short leg is a completion question for the robot.
- **Keep the 350 ms hard Go gate.** Rejected. With the robot owning safety, the age is display
  truth; the operator decides. The badge and stale dimming stay.
- **Keep the 1.5 degree rotate skip.** Rejected as an arbitrary tolerance; skip only a bearing that
  rounds to a zero centidegree, which is the Relay's own command precision.
- **Keep the fixed completion timeout.** Rejected. With no distance cap, a legitimate leg can take
  longer; the completion Event, link loss, a Boot Id change or Stop end the wait.
- **App-side path planning, waypoints or target-point pathfinding.** Rejected for v1. It needs
  spatial memory and a map, both robot-side; the app would become a second planner. The Target Point
  is the additive seam.
- **One-tap drive.** Rejected by ADR-0038; the two-tap preview stays as the accidental-touch guard
  and the legible preview.
- **A compound wire command.** Rejected by ADR-0038; two commands and two claims are enough, and a
  compound command would need new claim scope and firmware surface.

**Decision**

*Point-and-go.*

A tap produces a Target Point and a preview with bearing and distance from the current Pose. Nothing
moves. **Go commits to the Target Point**, not to the preview numbers: the app recomputes the legs
from the current Pose at Go. If the robot moved since the tap, it still drives to the point; there
is no second confirmation and no invalidation. The operator is responsible for stopping the robot
first if that is what they wanted.

The app does not clamp to the live scan, cap the distance, refuse a short leg or warn beyond scan
range. The live scan is display only. The robot's drive protection owns obstacle safety, and the
completion Event reports the actual distance and the reason. The only minimum is the drawn robot
footprint: a tap inside the icon produces no command, so the app does not send a zero-length leg.

The app sends `rotate_by` only when the bearing rounds to a non-zero centidegree, then
`drive_distance`. It uses fixed conservative speeds (drive 35, rotate 60; bench-confirmable). This is
two commands and two claims; a `busy` between the legs is reported as "rotation done, drive not
sent" and is not retried. No compound command is added.

*Completion.*

The app waits for `rotate_completed` and `drive_distance_completed`. It imposes no fixed completion
timeout: the wait ends on the completion Event, a link loss, a Boot Id change, Stop, or the Activity
leaving running. A slow leg shows "waiting for the robot" with Stop. The app reports target versus
actual and the reason (obstacle, stopped, link lost, failed). A Volatile Command is never
auto-retried; `indeterminate` and `timeout` are unknown and the operator re-issues. A completion
that arrives with `command_seq: null` (the Relay retains the attribution for 60 s) is matched by
leg, target and the single in-flight plan.

*Stop.*

Stop is context-sensitive. A live Manual Drive is stopped with `stop_motion(brake=1)`; a live
Procedure is stopped with `stop_procedure{id}`. Stop is always visible. "Stopped" is shown only on
the ACK; otherwise "not confirmed — the robot stops on link loss". The app is not a safety surface;
the robot's obstacle brake, the panel Stop and the Deadman are.

*The renderer and the seam.*

The world-frame rules of ADR-0038 stand: each scan is paired with the Pose at its own timestamp, the
robot marker uses the newest Pose, and the camera is a view transform. The renderer is layered
(grid, map layer, live scan, robot/target/preview, overlay), so the future `POSE` and `MAP` streams
and the robot's Odometry Grid layer in without a rewrite. There is no app-side map. The age badge and
stale dimming stay; they never gate Go, and there is no prediction or smoothing.

**Consequences**

- Requirement §3's clearance validation, clamp and distance limits are superseded, and the
  prototype's clamp and raycast planning checks are not ported. The busy and Stop, Deadman, Boot Id
  and overflow scenarios remain.
- The app no longer uses the scan to alter a command. The robot's drive protection becomes the only
  obstacle safety; the panel Stop and the Deadman remain. A drive based on a stale or uninformative
  scan is possible and is the operator's decision.
- A long leg can outlive the Relay's 60 s completion attribution; the app matches by leg, target and
  the in-flight plan instead of capping the distance to protect the timeout.
- Stop now distinguishes Manual Drive and Procedure, an amendment to ADR-0038's single
  `STOP_MOTION` rule.
- The Target Point is the additive seam for future target, waypoint and exploration commands; the
  robot owns the planning and spatial memory. The app does not grow a second planner.
- The drive and rotate speeds, the icon footprint and the target/preview styling are provisional
  and bench-confirmable; the real-phone pinch pass remains an open item.

**Open items**

- Confirm the fixed speeds, the icon footprint and the target/preview styling on the bench.
- Test the pan and pinch gestures on a real phone in landscape; the 844x390 frame is a layout check,
  not a touch test (ADR-0038's open item stands).
- When the robot grows target, waypoint or exploration commands, add them to the App Boundary and
  layer them onto the Target Point; the app's planning stays out of scope.
