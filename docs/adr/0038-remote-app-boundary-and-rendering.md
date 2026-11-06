# The remote app: a JSON boundary and a world-frame radar

The Link Protocol (ADR-0030, ADR-0031) settles the wire between the robot and the Bridge, and the
daemon is meant to be the app's typed boundary. Ticket 13 prototyped the app against fake data and
found that the app-facing contract was not written down anywhere: what a stream message looks like,
how an event survives an app reload, what the Activity snapshot is, and what the three reachability
failures look like. The same prototype raised the rendering questions: which frame the radar draws
in, whether latency needs prediction, what a click to drive means, and where a map lives.

The app is vanilla ES modules and `<canvas>`, served as static files, with no build step, no
bundler and no wasm (map Notes). The daemon owns the protocol; the app draws pixels. That boundary
is what this decision makes explicit.

**Status:** accepted. The app consumes **JSON** from the daemon over one WebSocket, in a versioned,
additive schema described below. The radar draws the **world frame**, placing each scan with the
pose sample at its own timestamp; the pose is identity until odometry lands. Click-to-drive is a
**two-tap preview**: a bearing and distance from the current pose, clamped to the last clear point,
then a `ROTATE_BY` and a `DRIVE_DISTANCE` with a report from the completion events. Latency is shown
raw with a data-age badge and a 350 ms gate, never predicted. The radar is a **pannable, zoomable
world view**. **No app-side map exists in v1**; the robot's future Odometry Grid and `MAP` stream
own that, and click-to-drive always validates against the live scan.

**Considered Options**

- **Give the app the binary protocol.** Rejected. It puts the wire codec in a browser (a second
  implementation, a wasm build or a JS port) and duplicates the daemon's job. ADR-0031 already says
  the daemon is the app's typed boundary.
- **A daemon-specific HTTP/REST surface per panel.** Rejected. The app would depend on daemon
  routes, which is exactly what stops the WebSocket server moving into the Bridge later. One
  versioned message schema keeps that option open (map Notes; ticket 15).
- **A browser-side map from accumulated scans.** Rejected. It needs a trustworthy pose, which v1
  does not have; it duplicates the robot's Odometry Grid at another resolution and lifetime; and
  mapping/SLAM is outside this effort. It also invites driving from remembered free space, which is
  unsafe. The world-frame renderer keeps the seam open instead.
- **Prediction or smoothing for latency.** Rejected for v1. Without odometry, prediction is
  guessing; smoothing hides the age that the operator needs to see. The honest answer is a raw scan
  plus an age indicator and a gate.
- **One-tap click-to-drive.** Rejected. It cannot show a clamp or a refusal before motion and it is
  a poor phone affordance. A preview makes the command legible and doubles as the accidental-touch
  guard.
- **A compound "click-to-drive" wire command.** Rejected for v1. It would need a new claim scope in
  ADR-0034 and new firmware surface for a UI convenience. Two commands are already in the catalogue;
  the app reports `BUSY` if the claim is taken between legs.
- **A robot-frame radar only.** Rejected. It cannot become a map view additively and it throws away
  the pose transform that the first odometry work will need. The world frame costs a handful of
  lines now.
- **A camera locked to the robot.** Rejected. The operator needs to inspect a floor plan away from
  the robot. Zoom and pan are view transforms over the same world coordinates; they do not change
  the frame, the robot's pose or the meaning of a tap.

**Decision**

*The JSON boundary.*

One WebSocket, text frames, one JSON object per frame. The schema is additive and versioned: unknown
message types, stream names, event names and extra fields are ignored; no field is removed or
repurposed. The app dedupes streams by `(name, seq)`, where `seq` is the daemon's monotonic
per-stream sequence (ADR-0039), and events by `(boot_id, seq)`. The app never parses the binary
wire.

Message envelope: every message has a string `type`. The named messages are:

| type | direction | purpose |
|---|---|---|
| `hello` | both | protocol version and names; the server adds `auth_required` |
| `connection` | daemon -> app | reachability and failure state |
| `snapshot` | daemon -> app | reload-safe state after connect |
| `stream` | daemon -> app | one latest-wins sample |
| `event` | daemon -> app | one reliable, replayable edge |
| `command` | app -> daemon | a robot command |
| `command_result` | daemon -> app | the ACK or rejection, not the outcome |
| `pair` / `connect` / `disconnect` | app -> daemon | pairing and manual endpoint |
| `auth` | app -> daemon | LAN app-WebSocket gate; the pairing code, sent before any other message when `hello.auth_required` |
| `heartbeat` | app -> daemon | app liveness (ticket 15's app-frozen rule) |

*Units and conventions.* Distances are integer centimetres; angles are degrees, counter-clockwise
positive, with shortest deltas in `[-180, 180]`; `x` is forward, `y` is left and yaw is CCW (the
World Frame). Track speeds are `-100..100`; voltages are millivolts; an unknown value is `null`,
never a sentinel. `ts` is the daemon's receive time in epoch milliseconds; the app derives age, so
robot and PC clocks need not agree. **The wire's `ROTATE_BY` is clockwise centidegrees; the daemon
flips the sign.** The app never sees the wire convention: a positive `angle_deg` is a left turn.
`ROTATE_COMPLETED.error_deg` is actual minus target, counter-clockwise positive (ADR-0039).

*Streams.* Every `stream` message is
`{type, name, seq, ts, v}`. The values are:

| stream | `v` |
|---|---|
| `power` | `{level, voltage_mv}` |
| `motion` | `{left, right}` |
| `activity` | `{id, stage, percent, detail}`; `stage` is running / complete / failed |
| `perception` | `{obstacle, floor_drop}` |
| `lidar` | `{ranges_cm, first_deg, step_deg, clockwise}`; 360 entries, `null` = no return, slot 0 ahead and clockwise |
| `calibration` | `{motor, imu, mag, distance}`; loaded / missing / unknown |
| `pose` (reserved) | `{x_cm, y_cm, yaw_deg, confidence}`; absent in v1 |

*Events.* Every `event` message is `{type, boot_id, seq, ts, name, v}`. The names are `obstacle`
(`{detected, source}`), `floor_drop` (`{detected}`), `procedure_completed` (`{id, stage, detail}`),
`drive_distance_completed` and `rotate_completed` (both carrying `command_seq`, `status`, target and
actual values, `duration_ms`, `detail`), `calibration_loaded` (`{kind, status}`) and `event_overflow`
(`{dropped}`). Completion `status` mirrors the drive `CompletionStatus`: ok / cancelled / failed.
The **completion event, not the ACK, carries the outcome** of a drive or rotation.

*The snapshot.* On connect, and whenever the robot Session is re-established, the daemon sends
`{type:"snapshot", protocol, boot_id, ts, streams, last_terminal, events, event_cursor, catalogue}`:
the latest sample of every stream, the last **terminal** Activity sample, a bounded replay of recent
events and the robot's `GET_CATALOGUE` response. `last_terminal` is what makes a Result Report
survive an app reload, including the `Failed("Link lost")` case that raises no completion event
(ADR-0035). A gap in event `seq` means the app missed events; an in-flight volatile command is then
"outcome unknown". A reload reconnects and receives a fresh snapshot; nothing else is persisted. A
mid-session snapshot replaces streams, `boot_id`, `last_terminal` and `catalogue`; its events are
merged by `(boot_id, seq)`, and only a Boot Id change voids an in-flight command (ADR-0039).

*Activity and labels.* A live Activity is `id != 0` with `stage = "running"`; it holds the claim and
offers Stop. A finished Procedure is a Result Report from `last_terminal` or a
`procedure_completed` event. Manual Drive (`0x000D`) has no Result Report; its outcome is the drive
completion event. Activity leaving `running` for idle with no terminal stage and no completion event
means an operator stopped it. The app owns labels for `0` (Idle), `1` (Booting) and `0x000D` (Manual
Drive); every other label comes from the catalogue. The app keeps no Procedure table.

*Commands.* A command is `{type:"command", command_seq, command, args}` with
`rotate_by {angle_deg, speed}`, `drive_distance {distance_cm, speed}`,
`stop_motion {brake}`, `start_procedure {id, params?}` and `stop_procedure {id}`. The reply is
`{type:"command_result", command_seq, command, outcome, status, detail, ts}` with `outcome` one of
acked / rejected / indeterminate / timeout and `status` one of ok / busy / rejected / unknown_id /
stale / bad_argument / bad_length / unknown. The app never auto-retries a **volatile** command; an
indeterminate or timed-out volatile command is reported as unknown and the operator re-issues it
(ADR-0031, ADR-0035). Completion events carry the originating app's `command_seq` and `null` for
other connected clients; the app never sees a wire sequence (ADR-0039).

*Connection and the failure states.* A `connection` message is
`{type, state, robot, detail, retry_in_ms, boot_id, ts}`. The states are `searching`, `connecting`,
`connected`, `reconnecting`, `no_robots`, `pairing_required`, `unauthorized`, `busy` and
`setup_mode`. The app renders exactly the three ADR-0037 failures: **no robots** with a manual
`host:port` fallback, **unauthorized** naming the panel's `Link` screen, and **another client
connected** with a retry. A discovered robot without a stored token shows the 12-character pairing
field (three groups of four, base32, case and separators normalised). `setup_mode` is announced by
the reserved Beacon flag (ADR-0037); in AP-only setup there is no station Beacon, so the app can
only say "no robots found" (ADR-0039). While not connected, the connection screen **replaces** the
shell; while reconnecting, the shell stays with the last data dimmed and a link-lost banner.

*The renderer.*

The radar is a **world-frame** view from day one. The pose is identity until odometry lands, so the
robot's start pose is the world origin and the scan is robot-frame geometry placed in world
coordinates. The transform is written once now; the same code becomes a map view when pose arrives.

Two rendering rules follow:

1. **Pair each scan with the pose at its own timestamp.** Never transform an old scan with the
   newest pose: that would turn latency into fresh-looking geometry. The robot marker uses the
   newest pose, so a moving robot and a slightly old scan are visibly distinct.
2. **The camera is a view transform.** It follows the robot by default, but it can pan and zoom
   (drag / wheel / pinch, plus an on-screen zoom / pan / center cluster) without changing the
   frame, the pose or the meaning of a world point. A tap picks the world point under it at any
   zoom; a drag pans and never triggers a drive. The cluster is present in every layout, including
   the phone; center returns the camera to the robot and keeps the zoom. The camera's center and
   scale are in world coordinates; nothing about the robot changes.

Latency is shown raw. The scan carries its age; stale scans are dimmed; click-to-drive is disabled
above **350 ms** of scan age. There is no prediction or smoothing in v1, because the only motion
estimate is the track speeds and extrapolating them would be a guess. If pose lands, extrapolating
the marker is permissible; the scan still uses its own timestamp.

*Click-to-drive.*

A tap produces a **preview**: bearing and distance from the current pose, validated against the
latest scan. The preview shows the rotation and the leg, the reason for any adjustment, and a **Go**
action; nothing moves on the first tap. The policy is to **clamp, not refuse**, while a safe leg
remains:

- a return closer than the target along the bearing clamps the leg to `nearest - 18 cm`;
- a target beyond **3 m** clamps to 300 cm;
- if the clear leg is under **30 cm**, the plan is refused;
- a live Activity that is not our Manual Drive, a stale scan, or a robot that moved more than 2
  degrees / 5 cm since the tap refuses or invalidates the plan.

**Go** sends `ROTATE_BY` (skipped within 1.5 degrees) and waits for `rotate_completed`, then sends
`DRIVE_DISTANCE` and waits for `drive_distance_completed`. The app reports the distance actually
reached and the reason from the completion event (Obstacle, Stopped, Link lost, failed). This is
**two commands and two claims**: the claim is released between the legs, another operator can take
it, and the second leg then comes back `BUSY`; the app reports that and does not retry. A compound
wire command is deliberately not added. Stop is always available and is `STOP_MOTION(brake=1)`;
"stopped" is shown only on the ACK, otherwise "not confirmed - the robot stops on link loss"
(ADR-0035). The app is not a safety surface: the robot's obstacle brake, the panel Stop and the
deadman are.

*Panels and layout.*

Always visible: connection state, battery, motion, scan age and zoom, and the Stop action. Activity
and its progress are always visible while live. Events are on demand with an unread badge;
calibration and procedures are on demand. The wide layout is radar-first (A); the phone layout uses
a top status strip, the radar, a drive bar over the radar and a tabbed sheet for Activity, Events
and Robot (C); the same responsive shell serves both. The activity card is a live card, a Result
Report, or a transient "Stopped" card; the Result Report survives a reload through `last_terminal`.
The radar is the tap target; the confirmation button is at least 34 px and is protected by the
two-tap preview.

*The map.*

There is **no app-side map in v1**. The app draws the live `LIDAR_CLOUD` only. The authoritative map
is the robot's **Odometry Grid**, built by scan matching, and the reserved `MAP` stream will carry
it. The world-frame renderer is the seam: when pose and map land, the same view layers them in.
The prototype's "accumulate" mode (free-space clearing, occupied returns, fading cells) is a
demonstration of that seam and the overwrite rule, not a contract. **Click-to-drive always validates
against the live scan**, never against remembered free space; a map is a display and planning aid,
not a safety source.

**Consequences**

- ADR-0039 implements the JSON schema, the snapshot/replay retention, the sign conversion and the
  app-facing auth; ticket 14 places the static folder. The app remains a static folder with no
  build step.
- The app can move into the Bridge later without changing: same schema, same paths, same semantics.
  Nothing in the contract names the daemon process.
- The world-frame rules and the scan/pose pairing are in the renderer from the first version, so a
  map view is additive; until odometry lands the view is honestly robot-centric in effect.
- The 350 ms gate, 3 m limit, 18 cm clearance and 30 cm refusal floor are prototype values to
  confirm on the board; they are policy constants, not wire values.
- A Result Report survives a reload because the daemon retains one terminal Activity sample; the
  event ring is bounded, so an overflow is still reported as an unknown outcome, never a success.
- The app's failure states, pairing field and "in setup" copy are fixed; the `setup_mode` signal
  is the reserved Beacon flag (ADR-0037), consumed by ADR-0039.
- The map stays out of the app, so there is one map and one resolution. The cost is that a
  browser-only view will feel forgetful until the robot's map stream exists.

**Amendments from ticket 15**

ADR-0039 implements this contract and records the app-facing clarifications:

- `hello.server.name` is a stable service name (`explorer-remote`), not a process name;
  `robot.model` may be `null`.
- Stream `seq` is the daemon's monotonic per-stream sequence, so the app's `(name, seq)` dedupe
  survives wire wraps and robot reboots.
- A `snapshot` may arrive at any time, not only on first connect; a same-Boot-Id snapshot does not
  void an in-flight command, a Boot Id change does.
- `hello.auth_required` and the app-to-daemon `auth` message gate the WebSocket when the peer is
  not loopback; close codes 4401 (auth) and 4426 (protocol) are app-visible.
- Completion events carry `command_seq` only for the originating client and `null` for others.
- `disconnect` releases the shared robot Session but keeps the stored token; forgetting or
  rotating the token is the panel's action.
- `setup_mode` is observable only in APSTA; AP-only setup is honestly `no_robots`.
- `ROTATE_COMPLETED.error_deg` is actual minus target, counter-clockwise positive.

**Open items**

- Ticket 14 is resolved in ADR-0040: the app lives at `remote/app/`, is not a cargo package, and
  the daemon embeds it at compile time with `--assets` as the development override.
- The Bridge must implement the `setup_mode` Beacon flag (ADR-0037 amendment); the daemon side is
  ADR-0039.
- Confirm the 350 ms gate, 3 m limit and 18 cm clearance on the board.
- When odometry lands: add the `POSE` stream and the `MAP` stream to the snapshot/renderer; the app
  must keep validating click-to-drive against the live scan even when a map is available.
- Test the pan/zoom and pinch gestures on a real phone in landscape; the prototype's 844x390 frame
  is a layout check, not a touch test.
