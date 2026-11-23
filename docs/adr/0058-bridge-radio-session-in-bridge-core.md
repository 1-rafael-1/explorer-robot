# The Bridge Radio Session is a pure reducer in bridge-core

The Bridge's provisioning behaviour is the last policy layer inside its binary. The supervisory
loop in `bridge/src/bin/bridge.rs` (1030–1383) decides station association, reconnect backoff,
DHCP timeout, join failure, Setup Mode transitions, portal command handling, portal status and
scan gating, interleaved with `esp-hal` calls and `defmt` logging. `bridge-core` already owns the
HAL-free rules — the pump, routing, flow, the auth gate, the bounded `SetupWindow`, the credential
record and the DHCP/DNS/portal codecs — and is host-tested. The Radio Session is the one policy
surface still exercised only on the bench.

Three constraints shape the move. `bridge-core` is `no_std`, allocation-free and I/O-free; every
timed module takes a caller-supplied `u32 now_ms` and uses wrapping arithmetic. The Bridge has no
automated hardware-in-the-loop rig: it is observed with `espflash` and `defmt`, so a policy change
that is not host-tested is also not verifiable. ADR-0036, ADR-0037, ADR-0042 and ADR-0050 fix the
Setup Mode bounds, the trust gate, the wire and Bridge Status; none of that changes here.

**Status:** accepted. The Bridge's radio and provisioning policy becomes one
`bridge-core::radio` module exposing a **Radio Session** as a pure snapshot reducer: it takes the
observed radio facts, the queued commands and the results of the previous step's blocking effects,
and returns typed actions, typed events and an edge-triggered publication. The binary keeps every
`esp-hal` call, socket, UART, flash write, the pump and all encoding. It is a behaviour-preserving
extraction except for the small, named corrections in the parity ledger below.

**Considered Options**

- **Leave the policy in the binary.** Rejected. Reconnect backoff, wrong-password handling, DHCP
  timeout and the Setup Mode transitions are the most failure-prone provisioning behaviour and
  the only path to testing them is the manual bench.
- **Extract only small pure helpers, keep the state in the binary.** Rejected. The value is the
  state machine, not the individual comparisons: the interactions between the station link, the
  setup window, the retry block and the portal status are where the bugs live. Helpers would move
  the arithmetic and leave the transitions on the bench.
- **Inject a `Radio` trait into the core.** Rejected. It makes the never-testable half the generic
  parameter of the testable half, grows the core's API for a hardware abstraction the crate
  explicitly does not own, and departs from every existing `bridge-core` module, which is a plain
  struct over caller-supplied facts.
- **An edge-event interface.** Rejected. The loop already samples `is_connected()`, the DHCP
  address and the setup flags every pass; an event interface would require the binary to remember
  what changed and to detect edges, moving a class of bugs into the adapter. Edges are derived
  inside the session from the previous snapshot.
- **In-flight radio phases.** Rejected. The supervisory loop is single-threaded, so nothing else
  can call the session while `connect_async` is awaited. Modelling a connect phase would duplicate
  the `with_timeout` cancellation that only the adapter can perform, for no test benefit.
- **Keep the shared atomics authoritative.** Rejected. Feeding `SETUP_OPEN`, `PROVISIONED`,
  `STATION_UP` and the portal status back into the session as inputs recreates the feedback edge
  that makes the scan gate depend on a published value. The session owns the facts; the binary
  publishes them.
- **Let the binary order the credential write.** Rejected. Write-before-join and fail-closed are
  provisioning policy, exactly what this extraction exists to make host-testable.
- **A fake adapter.** Rejected. The seam is justified by a pure model with host tests and a
  hardware-bound executor; the tests drive the reducer directly and are the second caller. No fake
  radio, no public `sim` module and no test-only fake adapter is planned. If a multi-step test
  becomes unreadable, a private test helper is a local refactor, not part of the interface. This
  supersedes the idea file's "two adapters" heuristic.
- **Migrate `stack-soak` in the same effort.** Deferred. It duplicates only the connect/scan
  helpers and does not exercise provisioning; it gets its own follow-up ticket.
- **A pure snapshot reducer in `bridge-core::radio`.** Accepted. It is the shape the existing
  crate already uses, it keeps the blocking edges in the binary, and it makes every decision that
  can be wrong reachable from the host.

**Decision**

*One deep module.* `bridge-core::radio` exposes a `RadioSession` facade. The binary constructs it
from the boot Credential Record and drives it from the supervisory loop with `u32 now_ms`. The
module owns the station link state, the reconnect and backoff policy, the join-failure block,
`SetupWindow`, the provisioned flag and the Station Network's SSID, the portal status policy and
the scan gate. Private sub-state may be composed inside the facade; the public surface is the one
session type, its observation/input and outcome types, and its snapshot accessors.

*Synchronous effects, results on the next step.* The session returns actions; the binary executes
each to completion before the next step. The results of blocking effects — a join, a scan, a
credential write, a disconnect — arrive as inputs on the following step. The session keeps the
small amount of state needed to avoid re-emitting an effect whose result is pending.

*Single source of truth for shared state.* The session owns `provisioned`, the Station Network's
SSID, `SetupWindow`, the station link state and the portal status. The binary publishes derived
values to the existing statics only when they change; the session never reads them back. The
portal HTTP workers keep their optimistic status writes as UI hints for the redirect, so
publication is edge-triggered rather than overwriting on every step. The scan cache, sockets,
flash, the Beacon and the watchdog stay in the binary.

*Persistence is an action.* A portal provision yields a persist action; the adapter writes the
record and reports the result on the next step. The session holds a pending-provision phase and
only mutates the credential state and emits join actions after a successful write. A failed write
leaves the previous credentials in force and reports the failure through the portal status.

*The binary keeps the blocking edges.* `esp-hal` calls, `with_timeout` cancellation, sockets,
UART, flash, the pump, the Beacon and all encoding and logging remain in the binary. The session is
`no_std`, allocation-free and HAL-free, like the rest of `bridge-core`.

*The interface.* One call per step: `step(&Observation, inputs: &[Input]) -> Outcome`.
`Observation` carries the HAL facts (`now_ms`, `controller_connected`, `station_ip`); `Input`
carries the ordered edges (`Portal`, `Panel`, `Persist`, `Join`). `Outcome` carries a
fixed-capacity, saturating `ActionSet` and `EventSet`, plus an edge-triggered `Publication` whose
fields are `Option`s. A join is one action carrying the target SSID, because the diagnostic scan
is inseparable from the attempt and its result does not gate the connect. Disconnect and
mode-switch results are ignored as they are today; a portal scan is fire-and-forget. The session
owns the existing decision counters (`station_reconnects`, `portal_provisions`, `portal_forgets`)
and exposes them through `Stats`; the adapter owns the scan, AP, UART and ring counters. The
session is poll-driven: the binary keeps the 500 ms loop and no deadline API is added. The Bridge
Status station-state and flag derivation becomes a pure accessor in the module; the binary
supplies RSSI, channel and counters and encodes.

*Parity ledger.* Observable behaviour is preserved except for these named corrections:

| Change | Reason |
|---|---|
| The scan gate no longer reads the published portal status; it reads explicit session state. | Removes a feedback edge from a value written by another task. Same outcome in every current path. |
| Portal Forget and panel Forget pass the same computed station-up fact to `SetupWindow::forget`. | The panel passed a hard-coded `false`; the difference is at most one loop tick in when the outage clock starts. |
| `join_failed` is renamed to a name that says "the operator must act", and its transitions are unchanged. | One bool currently means wrong password, generic failure and DHCP timeout; the name, not the behaviour, was wrong. |
| `radio_mode` is removed. | It was written five times and read only by one warning; the failed action is already logged. |
| The backoff sequence stays 1, 2, 4 … 60 s and is expressed as one transition. | The reset-before-use duplication is confusing but produces the same sequence on every reachable path. |

Nothing else changes deliberately: the `Status::Connected` overwrite while the station is up
remains, the adopt path still does not clear the retry block, and a portal provision still does not
reset the scan clock. Those are recorded as observations, not fixes.

*Landing.* The module is built core-first and wired in one commit: (1) the types and the station
link with host tests, no binary change; (2) Setup Mode, provisioning and the persist-pending phase;
(3) status, scan and telemetry; (4) the loop's policy is replaced by `step` plus an action executor
and the old locals are deleted; (5) the bench. The merge gate is the core bench path: unprovisioned
boot and portal, portal scan, a successful provision through association, DHCP and the 30 s stable
close, wrong password, portal Forget, panel Forget where the rig allows it, link loss and backoff,
Bridge Status and the Beacon flags, and reboot persistence. The 5-minute outage, the 10-minute
expiry, the DHCP timeout, the persist-failure branch and a long soak are the feature exit, not the
merge gate.

**Consequences**

- Reconnect, wrong-password, DHCP-timeout and Setup Mode transitions gain host tests; the binary
  loses the policy and keeps the wiring.
- `bridge-core` gains one public module and a host test suite; its `no_std`, allocation-free and
  HAL-free properties are unchanged.
- The `esp-hal` executor — the match from actions to controller calls — remains bench-only. The
  extraction does not and cannot test it; it only makes the decisions it executes testable.
- No wire, trust, Setup Mode bound, Bridge Status layout or beacon change. ADR-0036, ADR-0037,
  ADR-0042 and ADR-0050 stand.
- `stack-soak` keeps its duplicated connect/scan helpers until its follow-up ticket.
- There is no second adapter to maintain: the test suite is the other caller, and a fake radio
  would test only our own model, not the `esp-hal` executor.

**Open items**

- The spec and implementation tickets, created separately.
- The `stack-soak` follow-up ticket.
