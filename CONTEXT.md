# Explorer Robot — Domain Glossary

> This glossary records the canonical terms for the explorer-robot v3 firmware. It is a living document — update it whenever a term is resolved during design or implementation.

## Hardware

- **Weight Distribution** — Motors (~400g) at the rear, battery (~100g) at the front. Remaining components (PCB, LiDAR, rangefinder) biased forward to achieve roughly centered center of mass for even track pressure and stable climbing.
- **Track** — Left or right side of the robot. Each track has three sprockets: rear drive sprocket (motor-coupled, toothed), middle road wheel (weight-bearing, smooth), and front idler (tensioner, smooth). Rear drive keeps the bottom track run in tension under load, maximizing traction and preventing track bunching.
- **Motor** — A single JGB37-520 6V 165RPM DC motor with hall encoder. Two motors (one per track).
- **Motor Driver** — The TB6612FNG dual H-bridge chip. Controls direction (via direct GPIO) and speed (via PWM) for both motors. One standby pin enables/disables the driver.
- **Encoder** — Two-channel (A/B) hall encoder on each JGB37-520 motor. Counts 1320 pulses per output shaft revolution (165:1 gear ratio) via PWM input mode. Only one channel is read, so direction is inferred from the motor command rather than measured from the A/B phase.
- **IMU** — ICM-20948 9-axis inertial measurement unit (accelerometer, gyroscope, magnetometer). Connected via dedicated SPI bus (CS, SCK, MOSI, MISO). Its **VDDIO is 1.8 V only** (1.71–1.95 V, unlike the MPU-9250), so it runs on a dedicated 1.8 V rail and its SPI and INT lines are level-shifted to the 3V3 MCU (ADR-0017). The CS line is pulled up to 3V3 on the MCU side (`R46`, 2.2 kΩ) so the IMU stays deselected while the RP2350 is in reset; firmware resets the IMU before verifying its identity.
- **LiDAR** — COIN-D6 360° spinning dTOF LiDAR on dedicated UART0 (core1): MCU TX on GPIO28, MCU RX on GPIO29, with an active-high TPS22917 high-side load switch on GPIO30. The 5 V branch is switched; the connector ground is solid. Emits continuous scan data at a native 0.9° resolution (400 points per revolution) with distances in millimetres, over UART at 230400 baud 8N1 with a start/stop command protocol. Driven by the `coin-d6` workspace crate on core1, and powered only while a mode needs it. Its UART is quiesced, deinitialized, and both TX and RX are tri-stated before the branch is switched off (ADR-0023).
- **AI Cam** — Grove Vision AI V2 on-device ML camera module (Himax WiseEye2), on a PIO0 UART (core0): MCU TX on GPIO24, MCU RX on GPIO25, with an active-high TPS22917 high-side load switch on GPIO23. The module requires a 5 V input (its Grove `VCC` feeds an on-module 3.3 V buck); the branch is switched from `+5V`, and its UART is 3.3 V. Not yet integrated — pins reserved only. The PIO transport leaves hardware UART1 free for the expansion connector (ADR-0021). Its UART is quiesced, its PIO state machines are dropped, and both pins are tri-stated before the branch is switched off (ADR-0023, ADR-0027).
- **Expansion** — The port for a second MCU on UART1, built as two JST-PH 2.0 mm connectors. `J9` (`EXP.PWR`) carries switched 5 V on pin 1 and ground on pin 2, from a TPS22917 high-side load switch enabled by `EXP_PWR` (GPIO32). `J10` (`EXP.DATA`) carries, in pin order, `EXP_TX` (GPIO36) and `EXP_RX` (GPIO37) through 100 Ω series resistors, then `EXP_EN` (GPIO34), `EXP_BOOT` (GPIO35) and `EXP_IRQ` (GPIO33) through 1 kΩ. The port is 5 V-powered and 3.3 V-logic; the expansion board regulates its own 3.3 V. Firmware support is not implemented. It is not hot-pluggable, and the signal lines have no ESD protection. Before the branch is switched off, UART1 is quiesced, `EXP_TX` is tri-stated, `EXP_EN` is held low and `EXP_BOOT` is left low or high-Z (ADR-0021, ADR-0023).
- **Load Switch** — TPS22917 high-side load switch used on each switched branch (5 V for the AI camera, LiDAR, expansion connector and panel). 1–5.5 V, 2 A, active-high enable, adjustable rise (`CT` to `VIN` through 1 nF on each branch), QOD, and true reverse current blocking; no current limit. Each branch has a 1 µF input capacitor and a pulldown on its enable (the TPS22917 also has a smart pulldown). See ADR-0010, ADR-0021, ADR-0023 and ADR-0027.
- **Switched Branch** — A load behind its own TPS22917: the LiDAR, expansion connector, panel and AI Cam on 5 V. Before a branch is powered down, its protocol is stopped, its UART is quiesced and deinitialized, and its MCU-side lines are tri-stated; only then is `*_PWR` deasserted. On power-up the branch is powered first, its rise time elapses, and then its UART is brought up and its control lines are driven. See ADR-0023.
- **Quiesce** — To stop a branch's protocol, drain the UART's software buffer, and wait for the hardware transmitter to go idle before removing the UART from its pins. Quiescing is not the same as dropping the UART object.
- **Tri-state** — To release an MCU pin to high impedance with no pull, so it cannot source current into an unpowered branch. A branch's UART lines are tri-stated rather than driven low; a driven low is a break condition.
- **Branch Off-state** — The state of a switched branch after its protocol is stopped, its UART is quiesced and deinitialized, its MCU-side lines are tri-stated, and its `*_PWR` enable is low. No MCU pin may source current into the branch in this state.
- **Front Rangefinder** — VL53L0X time-of-flight module at the front of the enclosure, angled down and forward, on dedicated I2C0 (GPIO16/17). Its `XSHUT` is GPIO1 and its `INT` is GPIO2; it runs from the unswitched `+3.3V` rail through a 6-pin JST-PH connector (ADR-0026). Detects floor drops and near-field obstacles ahead.
- **Rear Rangefinder** — VL53L0X time-of-flight module at the rear of the enclosure, angled down and backward, on dedicated I2C1 (GPIO26/27). Its `XSHUT` is GPIO3 and its `INT` is GPIO31; it has the same power and connector treatment as the front unit (ADR-0026). Detects floor drops and near-field obstacles behind when reversing.
- **Rangefinder Pair** — The front and rear VL53L0X modules together. Their buses are electrically separate, so both keep the default `0x29` address without XSHUT sequencing. The pair is currently a floor-drop/near-field sensor; feeding it into obstacle avoidance is deferred (ADR-0026).
- **Panel** — The 2.8″ 240×320 (320×240 landscape) ST7789 TFT display module and its resistive touch layer, sharing a full-duplex SPI1 bus. Its 5 V branch is switched by a TPS22917 high-side load switch enabled by `PANEL_PWR` (GPIO0, ADR-0024). The robot's only input and output surface: the Display draws, the Touch Panel senses.
- **Display** — The Panel's drawing surface: a 320×240 landscape RGB565 framebuffer rendered by the ST7789 controller.
  _Avoid_: OLED, screen
- **RGB LED** — Common-cathode RGB LED. Indicates battery state (green → yellow → red) and obstacle alerts (flashing red).
- **Battery** — 2S LiPo (twin 18650, 8.4V max). Placed at the front to counterbalance the rear-mounted motors (~400g combined). Voltage read via ADC. Motors compensated to 6V target.
- **Standby Pin** — Direct GPIO that enables/disables the TB6612FNG motor driver. Active high.
- **Pin Group** — The RP2350B GPIO assignment keeps each peripheral on a contiguous run of pins (motor 4–12, RGB 13–15, I2C0 16–17, IMU 18–22, camera 23–25, I2C1 26–27, LiDAR 28–30, expansion 32–37, touch 38–39, battery 40, display 41–47) so its harness leaves the package as one bundle; the panel power switch is the point-to-point exception on GPIO0 (ADR-0024), and the rangefinder `XSHUT`/`INT` lines are point-to-point on GPIO1–3 and GPIO31 (ADR-0026). No spare GPIOs remain. See ADR-0022.

### LiDAR

- **Spin** — One full 360° revolution of LiDAR returns: 400 points at the native 0.9° resolution.
- **Snapshot** — A single-revolution capture, as returned by `read_scan`.
- **Point** — One LiDAR return, `{ angle_deg, distance_mm, intensity }` (bearing in degrees, range in millimetres as an `Option` — `None` means no return — and return strength 0–255).
- **Ring-start** — The packet flag (`T == 1`, the low bit of the `CT` byte) that delimits the start of a new spin.
- **Validity ratio** — The minimum fraction of aggregated spins that must report a valid (`distance_mm.is_some()`) sample at an angular bucket for that bucket to be kept; otherwise the bucket is emitted as a no-return.
- **Bin representative** — The single LiDAR return chosen to stand for all native samples that fall within one angular bucket during aggregation. It is the *nearest valid return*, so coarsening a bucket never hides a closer obstacle (not the first sample, strongest return, or a mean).
- **Warm-up** — The post-`start` phase during which revolutions are discarded until the rotor reaches steady-state speed. Settling is proxied by the per-revolution point count stabilising near the native 400 (as opposed to plateauing below it, or exhausting a spin budget).
- **Enabled** — The LiDAR's lifecycle state: `Off`, `Warming`, `Streaming` or `Failed`. A mode enables the sensor when it needs perception and disables it when it is done; at most one mode needs perception at a time, and the enabled sensor serves data to every reader.
- **No Return** — A bin for which no valid range was measured. It is not an obstacle, and it must not be read as a clear angle: gap selection never treats it as navigable.
- **Front Sector** — The angular window about dead ahead that the Coast-and-Avoid stop test sweeps: ±45° by default, with a 30 cm stop threshold.
- **Room Scan** — The operator-initiated mode that renders live Spins as a radar on the Display, for verifying the LiDAR and surveying a room. It does not drive.

### Touch

- **Touch Panel** — The 2.8″ 240×320 SPI panel's resistive touch layer, and the robot's sole operator input. Its controller IC is unmarked; the XPT2046/TSC2046-class protocol is an assumption confirmed on the bench.
- **Touch Sample** — One raw reading from the touch panel: X, Y, and the two Z (pressure) channels in unfiltered 12-bit counts.
- **Touch Calibration** — The mapping from raw touch counts to screen pixels, measured at the panel's corners.
- **Reference Calibration** — The vendor reference board's raw endpoints, shipped as `Calibration::REFERENCE` for first bring-up only; not measured on this robot's panel.
- **Measured Calibration** — This robot's panel raw endpoints, measured from the corner calibration targets during bring-up and recorded as `Calibration::MEASURED`.
- **Calibration Target** — A marked point on the display at a known screen coordinate, touched during calibration to pair a raw reading with that coordinate.
- **Touch Pressure** — A position-independent resistance proxy derived from a touch sample's two Z channels, distinguishing a firm press from a light one; a proxy, not a calibrated physical pressure.
- **Touch IRQ** — The touch controller's active-low pen-down interrupt line (`PENIRQ`).
- **Shared SPI Bus** — One SPI bus carrying more than one device, arbitrated by chip select and per-device configuration (the Display and Touch Panel share SPI1). The touch header's `T_CLK`/`T_DIN`/`T_DO` are the shared `SPI1_SCK`/`SPI1_MOSI`/`SPI1_MISO` nets; only `T_CS` and `T_IRQ` (`PENIRQ`) are dedicated touch GPIOs.

### Menu

- **Menu Entry** — One entry of the Panel's menu tree, carrying its label, the screen it opens, and the screen the header Back returns to. An entry is a submenu, a Procedure, or a screen. Entries are named, not positioned, so reordering a menu cannot change what an entry starts.
  _Avoid_: option, row, index

## Architecture

- **Core0** — Runs the orchestrator, drive subsystem, encoder reader, UI (Display, Touch Panel, RGB LED), IMU (SPI0), the two I2C buses (front and rear VL53L0X rangefinders), flash storage, and the main event loop.
- **Core1** — Runs the LiDAR task: the `coin-d6` driver, its on-demand high-side power lifecycle (including UART quiesce, TX/RX tri-state and UART deinit), and the spin-to-point-cloud mapping.
- **Orchestrator** — Central event loop. Waits for events from the system event channel and dispatches them: calibration events → initialization module, obstacle/battery events → behavior handlers, lifecycle events → UI subsystem, sensor events → logged or forwarded. Pure routing — no domain logic.
- **Event System** — Typed, multi-producer single-consumer event channel (capacity 64). Sensor tasks and input tasks raise events; the orchestrator consumes them. The seam between producers and consumers.
- **Behavior Handler** — A module under `task/behavior/` that reacts to specific event types. Domain logic lives here — obstacle fusion (perception atomics + EmergencyBrake interrupt), battery state updates, obstacle avoidance completion. Called by the orchestrator.
- **Task Spawn Order** (core0) — Tasks are spawned in dependency order: orchestrator → battery → rgb_led → motor_driver → encoders → drive_queue_executor → drive → panel → vl53l0x_stub → imu → flash_storage → testing → autonomous_mode → startup. Core1 runs only the LiDAR task.

### Message Passing

Three data flow patterns coexist, chosen by latency requirements:

- **Event bus** — Semantic events (obstacle detected, button pressed, calibration loaded) flow through `Events` channel → orchestrator → handlers. Used for state changes that multiple consumers may care about.
- **Direct channels** — High-frequency sensor data (encoder pulses, IMU orientation) flows point-to-point via dedicated `Channel`s directly into the drive subsystem, bypassing the event bus. Avoids event channel congestion.
- **Perception atomics** — `LIDAR_OBSTACLE` and `FLOOR_DROP` atomic booleans provide lock-free reads on the hot path. The `LiDAR` task writes the obstacle flag and the floor-drop stub writes the floor-drop flag; read by autonomous modes and the UI without acquiring a mutex.

### Remote Link

- **Link Protocol** — The framed binary contract on UART1 between the robot and the Bridge, carried over the network to the Remote. Three consumers share it: the robot firmware, the Bridge firmware and the PC daemon. It carries frames, not domain objects.
- **Link Port** — The connector and switched 5 V branch that carries the Link Protocol to the Bridge (`J9`/`J10`; firmware `RemoteLinkResources`). The PCB keeps its `EXP_*` net names.
- **Bridge** — The expansion board's role: network ⇄ UART1 translation. It parses the envelope (magic, class, id, length, CRC), forwards latest-wins per stream, synthesises flow-control hints, and is a peer for three control messages: it emits `THROTTLE`, consumes `BRIDGE_CONTROL`, and originates its own `BRIDGE_STATUS` snapshot (ADR-0042). It interprets no other payload. It is also the **network edge**: it broadcasts the **Discovery Beacon**, authenticates the one Remote against the **Pairing Token** before forwarding any frame, and enforces the single client (ADR-0037).
- **Bridge Presence** — The Bridge's assertion of `EXP_IRQ` high once its firmware and UART are ready. A level sampled after the branch is powered; it says the board is attached, not that a Remote is connected.
- **Bridge Status** — The Bridge's 1 Hz link and resilience snapshot: RSSI, channel, station state, setup/link flags and UART/watchdog counters. The Bridge sends the same 28-byte payload to the robot as an unacknowledged control frame (`BRIDGE_STATUS`, `0x0005`) and to the daemon as a coalesced stream (`0x0007`); it is Bridge-originated in both directions and the robot does not relay it (ADR-0042).
- **Link Task** — The core0 task that owns the Link Port's power, UART1 and the robot side of the Link Protocol (the firmware's Link Task module). It samples the state modules for streams and forwards the curated events. It also publishes the Bridge Status snapshot (`LinkCore::bridge_status()`) into `state::link` for the panel.
- **Remote** — The PC-side client: a Rust daemon plus a static web app. The daemon owns the Session and the protocol; the app draws pixels.
- **Relay** — The Remote's app-facing component: one JSON WebSocket and the static SPA, translating the Link Protocol into the versioned contract and retaining the latest snapshot and a bounded event replay. It owns no robot authority and names no process; a server implementing the same path, schema and semantics is interchangeable (ADR-0038, ADR-0039).
  _Avoid_: server, backend (they name a process)
- **App Snapshot** — The Relay's reload-safe retained state: the latest sample of each Stream, the last terminal Activity sample, a bounded Event replay and the Catalogue, sent on connect and whenever the robot Session is re-established (ADR-0039). The JSON message is `snapshot`.
  _Avoid_: Snapshot (ambiguous with the LiDAR Snapshot), state dump, cache
- **App Boundary** — The versioned JSON contract between the Relay and the app: one `/ws` WebSocket, centimetres and counter-clockwise degrees, additive-only (ADR-0038, ADR-0039).
  _Avoid_: API, REST
- **Frame** — One Link Protocol unit: a fixed 12-byte header followed by up to 1024 payload bytes. The header skeleton is frozen across major versions; only flag meanings and payload semantics may change. The maximum frame is 1036 bytes robot→PC and 268 bytes PC→robot.
- **Stream** — A lossy, latest-wins robot→PC channel identified by a stream id. One pending snapshot per stream, sequence numbers for gap detection, no acknowledgement. A dropped snapshot is never retransmitted.
- **Command** — A reliable PC→robot request identified by a command id. Commands are absolute and idempotent by construction, sequenced per Session epoch, acknowledged, deduplicated and retried. An unacknowledged command is indeterminate, not failed.
- **Event** — A reliable, rare robot→PC edge notification identified by an event id and sequenced per Boot Id. The Remote acknowledges it and deduplicates by `(boot_id, seq)`.
- **Control** — A link-management frame: `HELLO`, `HELLO_ACK`, `PING`/reply and `THROTTLE`. Control frames are not domain data.
- **ACK** — The reply to a Command or Event: the same class, id and seq with the `IS_ACK` flag and a status byte. `IS_ACK` reverses the normal direction for commands and events.
- **Session** — The Remote's claim on the robot. On the wire it is a liveness lease: a `HELLO`/`HELLO_ACK` handshake carrying an epoch and capabilities, renewed by any valid PC→robot frame and expiring after 5 s. The daemon owns the Session; the robot keeps only link state and streams at baseline without one.
  _Avoid_: connection, login
- **Peer** — The Remote as the robot's link state sees it: the source of valid PC→robot frames, known only by the frame timestamp and the link state it carries. The robot keeps no object for it and grants it no authority. The Link Task tracks `NoPeer` (no valid frame since the link was enabled) and `PeerLive` (a frame within the Deadman); `peer_lost` is the one-shot edge out of `PeerLive`.
  _Avoid_: client, operator, user (the operator may be at the panel); attached (that is **Bridge Presence**); connected (that is the **Session**)
- **Epoch** — The 32-bit value a Remote chooses for its lifetime and sends in `HELLO`. A new epoch resets the robot's command dedupe state.
- **Boot Id** — The 32-bit value the robot returns in `HELLO_ACK`, minted for each powered link period. A change tells the Remote that its in-flight commands are void and lets the event sequence restart safely. A Bridge reset keeps it; a Link Port power-down mints a new one.
- **Throttle** — An advisory per-stream rate or pause hint sent by the Bridge or the daemon when a buffer is filling. The robot may ignore it; the link uses throttle hints, not credits.
- **Requested Rate** — The per-stream rate the Remote asks for in `HELLO`: which streams it wants and how often, lasting for the Session. It is a subscription, not a hint. A `Throttle` may lower or pause it temporarily; the robot applies the most restrictive of the two and reverts to the Requested Rate (or the baseline) when the hint expires.
  _Avoid_: subscription, preference (the wire field is a rate)
- **Capabilities (Caps)** — The 32-bit mode bitset exchanged in `HELLO`/`HELLO_ACK`: delta cloud, pose trail, map, fragmentation and throttle support. It negotiates optional behaviour, not individual streams, events or commands; unknown ids are handled by the ignore/reject rules.
- **Procedure Id** — The stable 16-bit wire identity of an Activity: `0` is idle, `1` is boot, `2..` name an Activity — the startable ones are Procedures, and `0x000D` is the Activity-only **Manual Drive**. Never reused; the firmware maps `Activity` onto it with an exhaustive match, so adding an Activity forces a wire decision (ADR-0031, ADR-0034).
- **Catalogue** — The robot's list of startable Procedures (id, label, flags and parameter schema) served by `GetCatalogue`. The Remote keeps no duplicate Procedure table. It lists startable Procedures only; **Manual Drive** is an Activity id, not a Catalogue entry.
- **Volatile Command** — A relative, non-idempotent command (`DriveDistance`, `RotateBy`). It is at-most-once within a Session epoch and is never auto-retried after a boot change or an indeterminate outcome; the operator re-issues it. Procedure start/stop and `StopMotion` are not volatile.
- **Deadman** — The robot's 2 s peer-silence failsafe: while a Remote-originated claim is live, 2 s without a valid PC→robot frame is a **Link Loss**, and the claim is stopped with `EmergencyBrake` through the one stop verb. Owned by the Link Task's `LinkCore::tick`, which moves from `NoPeer` to `PeerLive` on the first frame in a powered link period and back when the 5 s lease expires; `peer_lost` is the one-shot edge after 2 s of silence. It never uses Bridge Presence and never touches panel-originated work (ADR-0035).
- **Link Loss** — The Deadman's verdict: a Remote-originated claim is live and no frame has arrived for the deadman timeout. A `Procedure` records `Failed("Link lost")` so the panel holds a Result Report; a Manual Drive clears to Idle and reports through its completion event. The daemon sends `PING` at 1 Hz so a healthy but idle Session cannot look like a loss (ADR-0035).
- **Setup Mode** — The Bridge's bounded provisioning state: a WPA2 SoftAP `ExplorerRobot-XXXX` with a captive portal, raised when the credential record is missing, after 5 minutes without a station link, or when the panel asks, and expiring after 10 minutes. The setup AP serves the portal only; the Link Protocol listener binds to the station interface, so the robot is never commandable over it. The setup password is public by design; trust is the Remote's (ADR-0036).
- **Credential Record** — The Bridge's versioned, CRC-checked store of the station SSID, passphrase and the **Pairing Token**, written with `sequential-storage` to the dedicated `bridge_data` data partition. A missing record, wrong magic, wrong version, bad CRC, bad flags, an out-of-range credential field or an invalid token means unprovisioned and fails closed into **Setup Mode**. It survives an application firmware update but not a flash erase or a partition-layout change (ADR-0036).
- **Bridge Control** — The acked control frame `BRIDGE_CONTROL` (`0x0004`, robot → Bridge) that asks the Bridge to forget the network and enter **Setup Mode**. The Bridge consumes it instead of forwarding it and ACKs with a status plus the setup SSID and password, which the panel shows. It is local only; the Remote cannot send it (ADR-0036). Its reserved action byte also carries "show pairing code" (1) and "rotate pairing code" (2), which answer with the twelve-character code; action 0 clears the network and rotates the token, and action 2 drops any live authenticated session (ADR-0037).
- **Discovery Beacon** — The Bridge's 3-second UDP broadcast on the station interface: magic, protocol major, a MAC-derived device id, a display name, the Link Protocol port and flags. It carries no secret; the daemon listens, deduplicates by device id and caches the last endpoint. It is not Link Protocol traffic and the robot never sees it (ADR-0037).
- **Pairing Token** — One 60-bit secret per robot, generated from the hardware TRNG at first boot and stored as twelve unambiguous base32 characters (no `I`, `L`, `O` or `U`) in the **Credential Record**'s token slot. Those twelve characters' bytes are the HMAC key. The panel's `Link` screen displays the code in three groups of four; the operator types it into the app once and the daemon stores it. The token is never sent on the wire: the Bridge and the daemon prove knowledge of it in a mutual HMAC-SHA256 challenge-response before the first frame, so an unauthenticated client cannot reach the robot or keep its lease alive. Rotatable from the panel without a reflash (ADR-0037).

## Driving

The `drive` module tree owns all motion control. Commands flow through a thin dispatch that routes to control modules. Two public entry points: `send_drive_command` (queue a command) and `send_drive_interrupt` (preempt).

- **SetTracks** — The primary motor command. Sets left and right track speeds (-100 to +100). Calibration and voltage compensation are applied before PWM output.
- **Manual Drive** — A transient Activity for one volatile Remote drive command (`DriveDistance` or `RotateBy`). It holds the **Active Claim** while it runs, appears in Activity State under the Activity-only id `0x000D` so the panel can follow and stop it, and leaves no Result Report; its outcome is the drive completion event. It is not a Procedure and is not in the Catalogue.
- **Drive Distance** — A `DriveAction` that commands travel of a specified distance (straight or curved arc) using encoder feedback with optional IMU correction. Completes when target is reached or an interrupt preempts it.
- **Rotate Exact** — A `DriveAction` that commands in-place rotation to a target angle using IMU feedback. Completes when the angle is reached within tolerance.
- **Drift Compensation** — Encoder-based correction that equalizes left/right track speeds to prevent veering. Handled by higher-level intents (distance, rotation) via IMU heading correction and encoder feedback.
- **Ramp-Down** — Progressive speed reduction as a distance or rotation command approaches its target. Prevents overshoot.
- **Voltage Compensation** — Scales PWM duty cycle to maintain 6V effective motor voltage as battery drains: `compensation = 6.0 / battery_voltage`.
- **Continuous Drive** — A drive intent that commands a speed and holds a heading until interrupted. It has no distance target, no ramp-down and no successful completion: it ends `Cancelled` on an operator stop or an obstacle/floor-drop brake, or `Failed` on stall, encoder loss or IMU loss. Used by **Coast-and-Avoid**'s forward phase.

### Drive Subsystem Internals

- **Drive Queue** — A builder (`DriveQueueBuilder`) that accumulates `DriveCommand` steps and submits them for sequential execution. A single `drive_queue_executor` task runs one queue at a time, emitting one queue-level completion.
- **Drive Task** — The main control loop (`drive()`). Polls active intents, steps idle when no intent is active, and handles interrupts. Selects over the command queue and the interrupt signal.
- **Intent** — A state machine that owns a specific motion behavior — rotation, distance drive, brake/coast settle, or idle. Each intent is an `ActiveIntent` variant carrying its controller state and completion flag. Commands that complete instantly (e.g., `Differential`) are not intents; they are fire-and-forget.
- **Dispatch** — The thin seam between the drive command queue and the control modules. Routes incoming `DriveCommand` envelopes, handles standby wake-up, and executes `IntentTeardown` descriptors on completion or interrupt. Owns no per-intent knowledge.
- **IntentTeardown** — An enum declaring what sensor streams must be stopped when an intent completes or is interrupted. Each `ActiveIntent` returns its teardown descriptor; the dispatch executes it. Keeps the dispatch thin — it knows to stop sensors but not which specific sensors each intent required.
- **InterruptKind** — An enum (`EmergencyBrake`, `Stop`, `CancelCurrent`) that specifies how the drive subsystem preempts the active intent. Sent via `send_drive_interrupt`.
- **EmergencyBrake** — An `InterruptKind::EmergencyBrake` sent when the `LiDAR` reports an obstacle or a floor drop is detected, and by the **Deadman** on link loss. Causes immediate active motor braking, cancels the active intent, bumps the command epoch, and drains queued commands. Mode-agnostic — dispatched unconditionally on either the obstacle edge or the floor-drop edge, and conditionally for the Deadman (only a Remote-originated claim is in scope, because the drive queue is shared).
- **Epoch** — A monotonic counter incremented on each interrupt. Queued commands stamped with an old epoch are discarded when dequeued, preventing stale commands from executing after an interrupt.

## Odometry

- **Odometry** — The self-contained estimate of the robot's pose and its change, from onboard sensing alone; any reference it aligns against is one it built itself.
  _Avoid_: position tracking, dead reckoning (that is the uncorrected subset)
- **Pose** — The robot's position and heading `(x, y, yaw)` in the World Frame.
  _Avoid_: position, location
- **World Frame** — The reference frame that pose is expressed in; its origin is the robot's pose at the start of a mission. Convention: `+x` forward, `+y` left, `yaw` counter-clockwise positive, degrees at the API boundary and radians internally.
- **Dead Reckoning** — Propagating pose from a motion model (encoders plus IMU) with no absolute correction, so error grows without bound.
  _Avoid_: inertial navigation
- **Scan Matching** — Estimating the rigid transform `(Δx, Δy, Δθ)` that best aligns one LiDAR spin with a reference, either the previous spin or the Odometry Grid.
  _Avoid_: registration, alignment
- **Odometry Grid** — The occupancy grid that scan matching aligns spins against. It is built from the robot's own scans and anchors the estimate to the world, bounding drift rather than merely slowing it.
  _Avoid_: map (reserved for the mission's map of the environment)
- **Scan Plane** — The plane swept by the LiDAR rotor over one spin. It is fixed to the chassis, so it tilts with the robot.
- **Flat-World Assumption** — The premise that every spin slices the world at a constant height and orientation. It is what makes 2D scan matching well-posed, and what terrain tilt violates.
- **Tilt Gate** — The IMU-driven admission test that admits or discards a scan-match pose according to roll and pitch.
- **Drift** — Unbounded growth of pose error under dead reckoning.
  _Avoid_: accumulated drift (that names the gap-analysis term **Leg Drift**)
- **Pose Confidence** — The system's uncertainty in the pose, raised by track slip and by tilt-gate closures.
- **Pose State** — The module holding the fused pose and its uncertainty, with lock-free mirrors for hot-path readers.
- **Estimation Error** — The difference between the robot's believed pose and its true pose. The error that odometry work is measured by.
- **Execution Error** — The difference between the robot's true pose and the pose the drive was commanded to reach, caused by slip, calibration and ramp-down overshoot. Odometry does not fix it.
- **Closure Error** — The pose discrepancy measured after the robot returns to its starting point, expressed as a fraction of the path length. The mission-level accuracy figure.

## State Modules

Lock order (documented in each module): **power → calibration → perception → motion → activity**

- **Power State** — Battery level (0–100%) and voltage. Accessed via `power::try_get_battery_voltage()` for hot-path readers.
- **Calibration State** — Motor calibration factors (`left_factor`/`right_factor`), IMU calibration status, distance calibration factor. Persisted to flash.
- **Procedure** — A long-running, operator-started, stoppable unit of work: a test mode, a calibration, or a drive mode. Identified by the Menu Entry that starts it, and it holds the **Active Claim** while it runs.
  _Avoid_: job, task (that names an embassy task), mode (that names an autonomous mode)
- **Active Claim** — The single exclusive right to be the robot's current Activity and to produce motion: held by the boot flow, one Procedure, one **Manual Drive**, or the Room Scan screen. A second claimant is `BUSY`; the holder is the only writer of a live Activity. Any operator may stop the holder; there is no owner, no takeover mode and no authority derived from the link lease (ADR-0034).
- **Claim Origin** — The transient `Panel` or `Remote` tag on an **Active Claim**, cleared with it. A `Drive` claim is always Remote and a `Procedure` takes the origin of its starter. It exists only so the **Deadman** can stop the Remote's work without touching the panel's; it is not authority and never reaches the wire (ADR-0035).
- **Activity State** — What the robot is currently doing: a Procedure, recorded by its identity, the boot flow, or a transient **Manual Drive**, with a small progress snapshot — phase and percent. Written by the **Active Claim**'s owner, read by the touch UI, which is the only thing that draws, and mirrored by the `ACTIVITY` stream. A finished Procedure is presented as a Result Report.
- **Result Report** — The status screen for a finished Activity State: it names the outcome, gives the reason on failure, and holds until the operator deliberately leaves it. A successful Procedure may return on its own; a failed one never does.
- **Perception State** — Dual-path architecture for obstacle detection. *Lock-free path:* the `LIDAR_OBSTACLE` and `FLOOR_DROP` atomic booleans for hot-path reads. *Detailed path:* a mutex-protected optional cloud (`lidar-cloud`'s `Cloud` — 360 one-degree slots of an optional distance in centimetres, with a sequence counter). The old zero-distance sentinel is retired: a slot is `None` for no return.
- **ObstacleSource** — Enum (`Lidar` only, for now) carried by `ObstacleDetected` events. Identifies which sensor triggered the detection. Only the LiDAR raises obstacle edges: the rangefinder pair is a floor-drop/near-field sensor and raises its own `FloorDropDetected` edge, which is never an obstacle. A rangefinder obstacle source remains deferred (ADR-0026).
- **ChangeDetected** — Enum returned by perception setters: `NoChange`, `ChangedToDetected`, `ChangedToCleared`. Enables edge-triggered reactions to obstacle state transitions without polling.
- **Motion State** — Track speeds (left and right, -100 to +100) with lock-free atomic mirrors for high-frequency readers.

## Autonomous Modes

- **Coast-and-Avoid** — Drive forward until the obstacle flag signals detection. Brake, back up, random-angle turn (±45°–180° via nanorand), resume forward. The flag comes from the LiDAR's Front Sector test; the mode reads only the pre-computed boolean, and refuses to run if the LiDAR cannot be enabled. Simple, reliable.
- **Avoidance Maneuver** — **Coast-and-Avoid**'s response to an obstacle or floor drop: brake, back up a fixed distance, then turn a random angle. It is one closed-loop drive queue, not a sequence of raw motor commands.
- **Attempt Straight Line** — User sets target distance (100–1000 cm). The LiDAR point cloud is analyzed for navigable gaps; the widest gap within the forward cone (±60°) is chosen. The robot rotates toward the gap center, drives a leg, and repeats. IMU heading used for drift correction. Completes when target distance is reached or no forward path exists. *(Deferred — UI integration pending.)*

### Gap Analysis

- **Gap** — A contiguous angular arc in the LiDAR point cloud where no obstacle return is within threshold (default 30 cm). A valid gap has sufficient width at its constriction depth, is flanked by obstacles, and lies within the forward cone.
- **Constriction Depth** — The minimum distance across the angles that make up a gap. Determines how far the robot can safely travel through that gap.
- **Leg** — One rotate-then-drive maneuver through a chosen gap. The leg length is the constriction depth minus a safety margin, or the remaining target distance if shorter.
- **Leg Drift** — The signed sum of deviation angles across all legs of a gap-following run. Later legs apply a correction toward the ideal straight line.
  _Avoid_: Accumulated Drift (that names the estimation concept **Drift**)
- **Correction Angle** — The gap midpoint angle that would cancel the current leg drift (= `-drift`). The gap closest to this angle is preferred.

## Calibration

- **Motor Calibration** — Per-track speed multipliers (`left_factor`/`right_factor`, range `(0.0, 1.0]`). The faster track is always the reference at 1.0; the slower track gets a factor < 1.0. Computed by running each track at fixed PWM and measuring encoder pulses.
- **Distance Calibration** — Single factor (range 0.5–2.0) mapping encoder pulses to real-world centimeters. Corrects for surface friction and tire wear.
- **IMU Calibration** — Magnetometer hard/soft iron calibration and motor interference compensation (`MagCalibration`). Gyroscope and accelerometer bias are handled internally by the ICM-20948 DMP.
- **Boot Sequence** — On `Initialize` event, the initialization module sends individual `GetData` commands to flash storage for motor, IMU, distance, and IMU flag calibration data. Each response raises a `CalibrationDataLoaded` event. When all four are received, the UI shows the main menu.
- **Flash Storage** — `sequential-storage` + `embedded-storage-async`. Saves motor calibration (2× f32 as `MotorCalibration`), distance factor (1× f32), and magnetometer calibration flags (`ImuCalibrationFlags`) to dedicated FLASH_STORAGE region (last 8KB of flash).

## Development Practices

- **No HIL (Hardware-in-the-Loop) rig** — The project has no automated hardware test rig. All tests that require physical hardware (sensors, motors) must be done manually on the target RP2350 board. `cargo check` and `cargo clippy` are the automated compile-time gates; the `coin-d6` crate additionally has host-side decoder/aggregation/warm-up tests run with `cargo test -p coin-d6 --features std --target x86_64-unknown-linux-gnu --tests` (the explicit host target is required because the default build target is `thumbv8m.main-none-eabihf`). Do not write `#[cfg(test)]` unit tests — they cannot exercise the real hardware and offer no value over compile-time checks. The `testmode` embassy tasks under `task/testmode/` serve as manual HIL verification procedures.
- **Hardware-tests crate** — `hardware-tests` is a workspace member for standalone, breadboard-scale HIL checks that exercise one or two sensors in isolation (e.g. the LiDAR→TFT radar), running as their own embassy binaries under `examples/`. Distinct from the in-firmware `testmode` tasks: these run outside the robot firmware, on a breadboard, rather than inside the running robot.
- **Testable logic in workspace crates** — Calculations that are hardware-independent (decoders, state machines, controllers, estimators) belong in `#![no_std]`, HAL-agnostic workspace crates that can be host-tested behind a `std` feature, as `coin-d6` does. This is the desired pattern. Older firmware code still performs such calculations inline, where they can only be verified by driving the robot, and extracting them is a standing refactoring opportunity.
- **Three workspaces** — The robot firmware and the shared `no_std` crates `link-protocol` and `link-core` are the root workspace; `bridge/` is a separate RISC-V workspace; `remote/` is a separate host workspace for `remote-daemon`, `bridge-net` and the static `remote/app/`. Cargo reads `.cargo/config.toml` from the current directory upward, so host-workspace commands run from `remote/` — from the repo root, even with `--manifest-path`, the robot's `thumbv8m.main-none-eabihf` default applies. Shared crates are path dependencies only, and `link-protocol`'s version mirrors the wire version (ADR-0040).
