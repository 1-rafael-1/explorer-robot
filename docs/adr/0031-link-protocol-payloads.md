# Link Protocol payloads: a curated catalogue and a compile-time drift rule

The Link Protocol envelope (ADR-0030) says how frames travel; it does not say what they carry.
The robot already has an internal event bus, five state modules, a `Procedure` enum, a drive
command vocabulary and a drive completion telemetry. The Remote needs some of that and must not
receive all of it: exposing every internal event would turn incidental firmware churn into a
public contract, while exposing a hand-picked subset risks silent drift — a new event or
Procedure appears and the Remote never learns about it.

The payload catalogue is therefore a **curated observable set** with an explicit rule for
adding to it. The rule matters as much as the tables: it must make omission a decision, not an
oversight.

**Status:** accepted. The catalogue and ids are settled. The `remote-protocol` crate and the
firmware mapping functions are not implemented yet; the first implementation lands with the
robot-side transport work. ADR-0034 narrows the Remote-startable subset, adds the Activity-only
`0x000D` Manual Drive id, and settles command authority.

**Considered Options**

- **Expose every internal event and state module automatically.** Rejected. The event bus
  carries internal coordination (`ObstacleAvoidanceAttempted`, boot calibration loads) that has
  no meaning to the Remote, and state modules hold raw internals (calibration factors, encoder
  counts, raw IMU vectors). Automatic exposure makes every internal rename or variant a wire
  break and bloats the app.
- **Pick a subset and rely on a written checklist.** Rejected as the only mechanism. It is
  exactly the “remember to update the other side” rule that fails quietly. A checklist is
  useful, but it needs a compiler-enforced seam.
- **A curated subset with exhaustive mapping functions and definition-site comments.** Accepted.
  The wire enums live in `remote-protocol`; the firmware maps its own enums onto them with
  `match`es that fail to compile when a variant is added, so the author must choose “wire” or
  “internal” before the code builds.
- **Have the daemon duplicate Procedure labels and ids.** Rejected. Labels are operator-facing
  text that changes with the panel; the robot serves them through `GET_CATALOGUE` and the
  daemon keeps no copy.

**Decision**

*The curated set.*

The link exposes what the panel can observe plus what click-to-drive needs:

| data | source | why |
|---|---|---|
| power | `power` state | panel System Info |
| motion | `motion` state | panel status/running views |
| activity | `activity` state | panel running screen |
| perception flags | `perception` atomics | obstacle/floor-drop state |
| LiDAR cloud | `perception` cloud | the radar view |
| calibration statuses | `calibration` state | panel System Info |
| pose | reserved | not until odometry |

Calibration factors, raw encoder counts, raw IMU vectors, drive-internal intent state and
per-module configuration are not streams. They may become query responses if a debugging need
appears.

*Streams (robot → PC, lossy, latest-wins).* Every field is little-endian. Each stream has one
pending snapshot; a newer sample replaces it.

| id | stream | payload | rate / coalescing |
|---|---|---|---|
| `0x0001` | `POWER` | `level u8` (`0xFF` unknown), `voltage_mv u16` (`0xFFFF` unknown) | on change, at least every 5 s |
| `0x0002` | `MOTION` | `left i8`, `right i8` | 10 Hz moving, 1 Hz idle |
| `0x0003` | `ACTIVITY` | `procedure_id u16`, `stage u8`, `percent u8` (`0xFF` none), `detail_len u8`, `detail` UTF-8 ≤32 B | on change + 1 Hz |
| `0x0004` | `PERCEPTION` | `flags u8` — bit 0 obstacle, bit 1 floor drop | 10 Hz |
| `0x0005` | `LIDAR_CLOUD` | `slots [u16; 360]` cm, `0xFFFF` = no return | 10 Hz |
| `0x0006` | `CALIBRATION` | `motor u8`, `imu u8`, `mag u8`, `distance u8` — 0 unknown, 1 loaded, 2 missing | on change + 5 s |
| `0x0010` | `POSE` | reserved | 20 Hz once odometry lands |
| `0x0011` | `POSE_TRAIL` | reserved | — |
| `0x0012` | `MAP` | reserved | — |
| `0x0013` | `CAMERA_DETECTIONS` | reserved | — |
| `0x0014` | `LOG` | reserved | — |

`ACTIVITY` encodes `procedure_id` 0 = idle, 1 = boot, `2..` = a startable Procedure; `stage`
0 = running, 1 = complete, 2 = failed. `LIDAR_CLOUD` is the crate's 360-slot grid quantised to
whole centimetres, slot 0 dead ahead and clockwise; the cloud's `u64` sequence is not sent
because the stream `seq` already exposes coalescing. `0x0007–0x000F` stay free for future core
streams.

*Events (robot → PC, reliable, rare).* All are ACKed, live in the 8-entry ring from ADR-0030,
and carry the firmware's operator-facing reason where one exists (UTF-8, ≤32 B).

| id | event | payload | when |
|---|---|---|---|
| `0x0001` | `OBSTACLE` | `detected u8`, `source u8` (0 = LiDAR) | obstacle flag edge |
| `0x0002` | `FLOOR_DROP` | `detected u8` | floor-drop flag edge |
| `0x0003` | `PROCEDURE_COMPLETED` | `procedure_id u16`, `stage u8`, `detail_len u8`, `detail` | terminal Procedure outcome |
| `0x0004` | `DRIVE_DISTANCE_COMPLETED` | `command_seq u16`, `status u8`, `target_cm i16`, `distance_cm i32`, `duration_ms u32`, `detail_len+detail` | after `DRIVE_DISTANCE` |
| `0x0005` | `ROTATE_COMPLETED` | `command_seq u16`, `status u8`, `target_cdeg i16`, `error_cdeg i32`, `duration_ms u32`, `detail_len+detail` | after `ROTATE_BY` |
| `0x0006` | `CALIBRATION_LOADED` | `kind u8` (0 motor, 1 imu flags, 2 distance), `status u8` (0 loaded, 1 missing) | boot/calibration load |
| `0x0007` | `EVENT_OVERFLOW` | `dropped u16` | event ring overflow |
| `0x0008–0x000F` | reserved | | low battery, map/landmark updates |

Completion `status` mirrors the drive subsystem's `CompletionStatus`: 0 success, 1 cancelled,
2 failed. `command_seq` is the originating command's seq, so the Remote can report “asked
100 cm, got 94 cm”. `PROCEDURE_COMPLETED` needs no command seq because only one Activity exists
at a time.

*Commands (PC → robot, reliable, at-most-once).*

| id | command | payload | semantics |
|---|---|---|---|
| `0x0001` | `START_PROCEDURE` | `procedure_id u16` + params | second start while the family is active is `BUSY`; an unavailable id is `REJECTED` |
| `0x0002` | `STOP_PROCEDURE` | `procedure_id u16` | idempotent; idle is `OK` |
| `0x0003` | `DRIVE_DISTANCE` | `distance_cm i16`, `speed u8` | volatile, relative, + forward |
| `0x0004` | `ROTATE_BY` | `angle_cdeg i16`, `speed u8` | volatile, relative, + clockwise |
| `0x0005` | `STOP_MOTION` | `brake u8` (0 coast, 1 brake) | idempotent, immediate |
| `0x0006` | `GET_CATALOGUE` | `page u8` | response in the ACK |
| `0x0007` | `GET_STATUS` | none | response in the ACK |
| `0x0010` | `DRIVE_TO_POSE` | reserved | absolute |
| `0x0011` | `DRIVE_TO_BEARING` | reserved | absolute |
| `0x0012` | `SET_DIFFERENTIAL` | reserved | joystick/teleop |
| `0x0013` | `GET_POSE` | reserved | |
| `0x0014` | `GET_MAP` | reserved | |
| `0x0015` | `GET_CAMERA` | reserved | |

Procedure ids are stable and never reused: `0x0000` idle and `0x0001` boot are Activity-only;
`0x0002` MotorCalibration, `0x0003` MagCalibration, `0x0004` DistanceCalibration, `0x0005`
CoastAndAvoid, `0x0006` AttemptStraight (the only one with params, `target_distance_cm u16`),
`0x0007` BasicMotor, `0x0008` Turns, `0x0009` StraightDrive, `0x000A` ArcDrive, `0x000B`
Imu6Axis, `0x000C` Imu9Axis.

Not every listed Procedure is available to start from the Remote yet: AttemptStraight models a
value screen but the firmware does not start it, and DistanceCalibration needs the panel's
value-entry flow after its drive step. The catalogue carries an `available` flag rather than
omitting them, so the panel and Remote show the same menu with the same disabled entries;
`START_PROCEDURE` on an unavailable Procedure is `REJECTED`.

`DRIVE_DISTANCE` and `ROTATE_BY` are **volatile commands**: relative and not idempotent. They
are at-most-once within an epoch via the command dedupe, and the daemon never auto-retries after
a `boot_id` change or an indeterminate ACK; it reports “outcome unknown” and the operator
re-issues. Procedure start/stop and `STOP_MOTION` remain idempotent. `ROTATE_BY` carries the
shortest signed angle in `[-18000, 18000]` centidegrees; the robot does not normalise a longer
one.

`GET_CATALOGUE` response, in the ACK payload:

```
total u8, page u8, count u8,
count × { procedure_id u16, flags u8, label_len u8, label UTF-8 ≤24 B }
```

Flags: bit 0 interactive, bit 1 calibration, bit 2 takes params, bit 3 available (startable
from the Remote now); bits 4–7 reserved. Page size 8, so the 11 Procedures are two pages; the
app pages until it has `total`.

`GET_STATUS` response:

```
level u8, voltage_mv u16, left i8, right i8,
procedure_id u16, stage u8, percent u8, detail_len u8, detail,
flags u8, motor_cal u8, imu_cal u8, mag_cal u8, distance_cal u8
```

*Control ids.* `0x0001 HELLO`, `0x0002 PING` (the reply is `IS_ACK` on the same id), `0x0003
THROTTLE`; `0x0004+` reserved.

*The drift rule.*

The wire vocabulary lives in `remote-protocol`. When the robot's expansion task exists, the
firmware owns **exhaustive mapping functions**:

- `event_to_wire(&Events) -> Option<WireEvent>`: every internal event variant is matched.
  Returning `None` is the explicit “keep internal” decision.
- `procedure_id(Procedure) -> ProcedureId`: every Procedure variant is matched.
- the drive-completion encoder: every `CompletionTelemetry` variant is matched.

Adding a variant to `Events`, `Procedure` or `CompletionTelemetry` therefore **fails to compile**
until the author decides whether it is exposed. A new state module or field is not an enum, so
the same rule is carried by definition-site comments in the state modules and by this ADR; it is
not on the wire until a stream or query is added to the catalogue and the Remote is updated.

The mapping functions and the definition-site comments are **implementation-time requirements**:
they are written when the `remote-protocol` crate and the expansion task exist, not before. The
alignment mechanism is settled: shared enums matched exhaustively at each boundary with no
wildcard arms, enforced by `clippy::wildcard_enum_match_arm` scoped to those modules; a
`PayloadCodec` trait for the codec; and capabilities limited to behavioural modes rather than
one bit per feature. The compile-time guarantee comes from the match, not from a trait.

The default is internal. Labels are served by the robot through `GET_CATALOGUE`; the daemon and
app keep no duplicate Procedure table, and they tolerate unknown ids (ADR-0030), so a new wire
event does not break an older remote.

**Consequences**

- Adding an event or Procedure is a deliberate, reviewed change: the compiler forces the wire
  decision, and the ADR records the result. The internal event bus remains free for internal
  coordination.
- The mapping functions are the seam between firmware vocabulary and wire vocabulary. Host tests
  must exercise every variant that maps to `Some` and every variant deliberately mapped to
  `None`.
- The `ACTIVITY` stream and `PROCEDURE_COMPLETED` event both carry the Procedure id, so the
  panel and Remote share identity without sharing a UI tree. Submenus and screens are not
  commands. The catalogue lists every Procedure with an `available` flag, so a Procedure whose
  panel-only flow is not ready stays visible and disabled instead of silently disappearing from
  the Remote.
- The completion events make the drive subsystem's telemetry operator-visible without exposing
  `DriveCommand` itself. Volatile commands are the one exception to “commands are idempotent”,
  and the daemon's retry policy must encode that exception.
- `Events::ObstacleDetected` currently carries only `source`; the wire event needs the new
  value, so the internal variant must gain `detected` before the mapper is written. Reading the
  live flag at map time would collapse two queued edges into the latest value.
- Open items: implement `remote-protocol` and the mapping functions; change
  `ObstacleDetected`; add the definition-site comments when the components exist; write the
  payload host tests; confirm the stream rates against the chosen baud; decide command authority
  (a separate decision); implement the alignment mechanism (exhaustive matches, `PayloadCodec`,
  scoped wildcard lint).
