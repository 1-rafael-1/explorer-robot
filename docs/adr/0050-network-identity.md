# Network identity: a readable Device Name and the Station Network on the wire

Two halves of the same missing fact: the home network cannot tell which device the Bridge is, and
the operator cannot tell which network the Bridge is on.

On the home network the Bridge appears in the router's client list by its MAC address. With a
laptop, phones, a TV and a printer on the same LAN, a bare MAC address is not something the
operator can find the robot by. The Bridge already has a readable identity — the Discovery Beacon
(ADR-0037) carries `Explorer-XXXX`, and the Setup Mode AP (ADR-0036) is `ExplorerRobot-XXXX` — but
neither reaches the router's client list, because the station interface never sends a DHCP
hostname.

On the robot, the Link screen shows Bridge presence, the peer state, RSSI, channel, the station
state, the setup state and whether the snapshot is fresh. It does not show the SSID of the network
the Bridge is joined to, so the operator cannot confirm what the Bridge is talking to, which
network won after a re-provisioning, or diagnose a Bridge that associated to the wrong AP.

Three existing decisions constrain the shape. ADR-0042 fixed `BRIDGE_STATUS` at 28 bytes, and
ADR-0031's catalogue names one id and one layout per payload. ADR-0040 maps a payload-layout
change to a **major** bump on both the crate and the wire, and requires every consumer to state
the major. ADR-0036's Credential Record is the only place the Bridge knows the station SSID; this
`esp-radio` beta exposes no current-SSID getter on the controller, and a scan shows nearby APs
rather than which one the station joined.

**Status:** accepted. The Bridge presents one **Device Name**, `ExplorerRobot-XXXX`, derived from
its station MAC: it is the DHCP hostname (option 12) and the Setup Mode AP's SSID. The Discovery
Beacon keeps the short label `Explorer-XXXX`, because its 16-byte name field cannot hold the full
eighteen. `BRIDGE_STATUS` gains a length-prefixed Station Network SSID and becomes 61 bytes. This
supersedes ADR-0042's payload layout, but that record's delivery classes, ids, routing and lease
rules still stand. The wire major becomes 2 and the crate 2.0.0 with no compatibility shim.
Neither the name nor the SSID adds a way to change the network.

**Considered Options**

- **mDNS.** Rejected. It publishes a `.local` name by multicast; it does not put a name in the
  router's DHCP client list, which is the gap. It would also need a responder in the Bridge
  firmware over the beta network stack, for a consumer that is a Rust daemon, not a browser.
- **A fixed device name.** Rejected. Two robots would present the same name on one LAN, and the
  router entry could not be matched to the Setup Mode AP that the operator joined.
- **Widening the Discovery Beacon's name field.** Rejected. The field is 16 bytes and
  `ExplorerRobot-` alone is 14; the full name is 18. Widening it moves the Beacon's port and flag
  offsets, and it grows the identity that rides in the `EXPH` preamble's `BRIDGE_CHALLENGE`, so the
  preamble layout and its maximum payload change too. That is a second wire break for a string
  that is display-only: the daemon deduplicates discovered devices by device id, not by name.
- **An operator-configurable name.** Rejected. It needs a new Credential Record field and a record
  version bump, a portal input, a panel way to read it back, and a rule for what happens when the
  name and the Setup Mode AP's SSID disagree. Nothing in the workflow needs a name that is not
  derived from the hardware.
- **A separate `NETWORK_INFO` message.** Rejected. It allocates an id and a second lifecycle for
  one field that belongs to the same snapshot: it would need its own routing, coalescing and
  staleness, and the panel could then show two Bridge facts with different ages. `BRIDGE_STATUS`
  already carries the station state and flags the SSID is read beside.
- **A tolerant dual-length decoder.** Rejected. Accepting both 28 and 61 bytes hides a
  half-updated build, leaves "empty SSID" ambiguous between "unknown" and "old firmware", and
  contradicts ADR-0040's rule that a layout change is refused, not degraded. Every component in
  this tree is rebuilt and reflashed together during bring-up, so there is nothing to tolerate.
- **One derived name, a DHCP hostname, and a length-prefixed SSID in the existing status.**
  Accepted. It reuses the derivation the Beacon and the AP already need, reaches the router with
  no router-side configuration, and puts the network on the message that already reports the
  station.

**Decision**

*One Device Name, three representations.*

The **Device Name** is `ExplorerRobot-XXXX`, with `XXXX` the last two bytes of the station (base)
MAC in upper-case hex. The derivation lives once in `bridge-net`'s identity module and is
host-testable; both the DHCP hostname and the setup SSID read that one value. The AP interface's
MAC differs only in the first octet's local-admin bit, so the setup AP carries the same suffix
even though it has its own interface address.

| representation | value | length | where |
|---|---|---|---|
| DHCP hostname (option 12) | `ExplorerRobot-XXXX` | 18 B | the station stack's DHCP configuration |
| Setup Mode AP SSID | `ExplorerRobot-XXXX` | 18 B | the SoftAP |
| Discovery Beacon name | `Explorer-XXXX` | 13 B in a 16 B field | the Beacon datagram |

The Bridge enables `embassy-net`'s `dhcpv4-hostname` feature and sets the hostname in its DHCP
configuration when the station stack is created. It is sent unconditionally — the hostname
identifies the device, not the network — and needs no re-request path, because it is derived from
the MAC and cannot change at runtime. The Setup Mode AP's SSID is expressed through the same
derivation rather than a second copy of the hex logic.

The Beacon keeps the short label. `Name::ENCODED_LEN` is 16 bytes, `ExplorerRobot-` is 14 and four
hex digits make 18, so the full name does not fit. **Do not widen the field to "fix" the label:**
the Beacon's port and flags follow the name, and the preamble's `BRIDGE_CHALLENGE` carries the same
identity, so a wider name is a Beacon and preamble wire break for a display-only string. The label
is not an identity: deduplication is by device id.

*The Station Network in the Bridge Status.*

`BRIDGE_STATUS` gains a length-prefixed SSID after the existing 28 bytes. The first 28 bytes do not
move; the payload becomes 61 bytes.

| offset | size | field | encoding |
|---|---|---|---|
| 0 | 1 | `rssi_dbm` | `i8`; `0x80` (`i8::MIN`) = unknown |
| 1 | 1 | `channel` | `u8`; `0` = unknown |
| 2 | 1 | `station` | `0` down, `1` connecting, `2` associated |
| 3 | 1 | `flags` | bit 0 provisioned, bit 1 setup open, bit 2 link ready; bits 3–7 reserved |
| 4 | 4 | `uart_dropped_bytes` | `u32` little-endian |
| 8 | 4 | `dma_stalls` | `u32` little-endian |
| 12 | 4 | `crc_failures` | `u32` little-endian |
| 16 | 4 | `resyncs` | `u32` little-endian |
| 20 | 4 | `wdt_feeds` | `u32` little-endian |
| 24 | 4 | `wdt_withheld` | `u32` little-endian |
| 28 | 1 | `ssid_len` | `0` = unknown; `1..=32` = a Station Network SSID length |
| 29 | 32 | `ssid` | the Station Network's SSID bytes, zero-padded after `ssid_len` |

Decoding is strict. The payload is exactly 61 bytes or a length error, `ssid_len` above 32 is a
value error, and `ssid_len = 0` yields no SSID regardless of the padding bytes. An SSID is an
octet string, not text, so the codec keeps the bytes verbatim; a round trip preserves spaces and
non-UTF-8 bytes exactly. Sanitising is a display concern, not a wire one.

The source is the **Credential Record**, not a scan. The Bridge retains the provisioned SSID at
boot and updates it when the portal writes a record, and clears it when the network is forgotten.
The field therefore names the network the Bridge is joining, which may be before the station has
associated. An unprovisioned Bridge reports `ssid_len = 0`.

Origination and routing are unchanged: `BRIDGE_STATUS` remains Bridge-originated in both
directions, remains unacknowledged, and still does not renew the Session lease or re-arm the
Deadman (ADR-0030, ADR-0035, ADR-0042).

*Display-only, on the panel and in the app.*

The Link screen's info block becomes five rows; the fifth is full width, `Network: <ssid>`, in the
block's small font. A maximum 32-byte SSID plus the nine-character label fits the row, and the
text is sanitised and bounded before it is formatted. An SSID byte outside printable ASCII is
shown as `?`; an unknown SSID renders `Network: --`. The info rows and the result lines are a
pixel or two shorter than in the four-row layout, so the fifth row and the result band both stay
inside the 240-pixel framebuffer; the three actions keep their order, size and hit rules and move
down with the block. The existing
setup result band, which shows the Setup Mode AP's SSID after a Bridge Control reply, is a
different thing and is untouched.

The link-state model gains an optional SSID bounded by its existing capacity, and staleness is
unchanged: a rejected status does not advance the receipt counter, so the block freezes and reads
`Status: Stale` rather than presenting a frozen SSID as current. The firmware's wire-to-model
mapping is exhaustive, so the new field is revisited at compile time.

The daemon's Bridge-status schema gains an optional `ssid` string and passes it through; absent
serialises as `null`. The wire bytes are decoded as UTF-8 with every invalid sequence replaced, so
a non-text SSID stays readable. The Remote App's Bridge panel renders the same value and its own
unknown placeholder; a whitespace-only SSID reads as unknown there.

The row is display-only. It adds no Bridge Control action and no new path that changes the
network. Local Operation, the panel's other screens and the Procedures are unaffected.

*The version rule.*

ADR-0040 maps a payload-layout change to **major** on both axes, so this is not an additive field
and cannot take a minor bump.

- `PROTOCOL_MAJOR` goes 1 → 2 and `PROTOCOL_MINOR` goes 1 → 0, keeping the crate/wire mirror test
  satisfiable.
- `link-protocol` goes 1.1.0 → 2.0.0.
- Every consumer requires `version = "2"`: the robot workspace's shared dependency, `link-core`,
  `bridge-core`, the Bridge firmware, `bridge-net`, the daemon, `bridge-stub` and `robot-stub`.
  Resolution fails until all are updated, which is the intended mechanical detection.
- `bridge-net` keeps version 1.0.0 and preamble major 1, because the `EXPH` preamble is unchanged;
  only its `link-protocol` requirement moves. The Discovery Beacon's `major` byte carries the new
  value, so the daemon's existing Beacon filter rejects a mismatched Bridge before the preamble
  runs. A manual `host:port` connection skips discovery, so it is refused at the `HELLO` exchange
  with a version-mismatch status instead.
- There is no tolerant decode, no optional payload, no new control message and no dual-length
  acceptance.
- All three components are rebuilt and reflashed together during bring-up. An old build is refused
  with a version message, not shown frozen telemetry that looks live.

**Consequences**

- The router can list the Bridge by a readable name with no static lease, reservation or router-side
  configuration, and the suffix matches the Setup Mode AP the operator joined. A router that does
  not honour DHCP option 12 may still show the MAC or a client identifier; the client-side fix is
  the only one the firmware has.
- The name is stable across reboots, re-provisioning and firmware updates because it is derived
  from the MAC, so the router's list does not churn.
- The Beacon label and the Device Name are deliberately different strings with the same suffix.
  That is a 16-byte field constraint, not drift; the glossary records why so a future reader does
  not "fix" it into a second wire break.
- The status grows from 28 to 61 bytes. At 1 Hz that is 33 more bytes per second in each direction;
  with the 12-byte header a frame is 73 bytes, still well under a tenth of a percent of the
  921600-baud link. The ring sizes and baud are unchanged (ADR-0033).
- Strict length decoding means a half-updated build fails loudly instead of misreading bytes, and
  the major bump makes a mismatched daemon or robot refuse to attach. The cost is that this tree is
  not wire-compatible with its previous build; that is safe only while every component comes from
  one tree and is worth revisiting if the daemon is ever distributed.
- The SSID may appear while the station is still connecting, because it comes from the Credential
  Record rather than from association. A stale snapshot still says `Status: Stale`, so a frozen
  SSID is never presented as current.
- The Credential Record layout is unchanged: no new field and no record version bump. There is
  still no operator-configurable name.
- The Setup Mode AP, its portal and the pairing flow are unchanged, so the recovery path the
  operator already knows does not move.

**Open items**

- Confirm on the bench that a real router's client list shows the Device Name, which requires the
  router to honour DHCP option 12. This has not been observed on hardware.
- Confirm the on-glass Link screen row and the app's Bridge panel row against a real Bridge. The
  row text, geometry and app rendering are host-tested with fake statuses, but the panel display
  itself is **bench-required**.
- If a future payload needs more than the Beacon's 16-byte name field, decide it as a Beacon and
  preamble change with their own major; do not widen the field in place.
