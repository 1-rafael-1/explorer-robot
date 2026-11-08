# Reachability and trust: a broadcast beacon and one pairing token at the Bridge

The robot is about to be reachable over the home network, and nothing in the Link Protocol decides
who may talk to it. ADR-0030 gives the daemon a lease but no identity, and the robot deliberately
holds no Session and no network identity; ADR-0036 gave the Bridge a credential record with a token
slot but did not say what the token is for. Two gaps remain: the daemon has no way to find the
Bridge on a network whose address it does not control, and any device on the LAN can currently open
a TCP connection and command the robot.

The network-facing component is the Bridge. It owns the Wi-Fi, the TCP listener and the credential
record; the robot sits behind a UART that carries only frames the Bridge forwards. That placement
decides where the trust gate belongs.

**Status:** accepted. Discovery is a UDP broadcast beacon the daemon listens for. Trust is one
**Pairing Token** per robot, verified by the Bridge with a mutual HMAC-SHA256 challenge-response
before any Link Protocol byte is forwarded, and the Bridge accepts exactly one authenticated client.
The Link Protocol is unchanged; the gate is transport-level. The trust boundary is the home LAN
plus the token; TLS and the internet stay out of scope. No firmware is written by this decision.
ADR-0039 adds a `setup_mode` flag bit to the Beacon and defines `tcp_port = 0` as "no Link
listener", so the daemon can report provisioning honestly where the station Beacon is available.

**Considered Options**

- **mDNS.** Rejected. It needs a responder implementation on an already-beta stack (`smoltcp` here
  has only a DNS client, and no responder crate is in the tree), and name resolution is the
  flakiest part of every OS the daemon might run on — for a consumer that is a Rust daemon, not a
  browser.
- **A DHCP reservation.** Rejected. It needs router configuration, and it still does not announce
  the address; it complements discovery at best.
- **The fixed setup-AP address.** Rejected. Since ADR-0036 the setup AP serves the portal only and
  the Link listener binds to the station interface, so the app never runs in AP mode; the ticket's
  "same mechanism in both modes" premise no longer holds.
- **A UDP broadcast beacon.** Accepted. It is OS-independent, needs no name service, is a few dozen
  lines over `socket-udp`, and its only consumer is the daemon.
- **No auth (LAN trust).** Rejected. The ticket asks for the boundary to be stated, and "any device
  on the home LAN can drive the robot" is a weaker answer than one field, one record slot and one
  gate. It also lets a guest's device keep the robot's lease alive.
- **A token sent in the clear on connect.** Rejected. It can be harvested by a cheap beacon spoof —
  a Wi-Fi guest advertises `ExplorerRobot-FAKE`, the daemon connects and hands over the token —
  whereas a challenge-response never reveals it.
- **A per-session code shown on the panel.** Rejected. It needs a new code for every daemon session,
  or a cached session token that converges back to a shared pairing token, plus a code-entry
  ceremony on every reconnect. The panel's display advantage is kept at pairing time, where it
  matters.
- **Enforcing auth at the robot.** Rejected. The robot holds no network identity and would need the
  token provisioned across the UART; the Bridge already owns the network edge and the record.
- **One pairing token, mutual challenge-response, enforced at the Bridge.** Accepted.

**Decision**

*Discovery.*

The Bridge broadcasts, every 3 s on its station interface, a small UDP datagram:

| field | size | meaning |
|---|---|---|
| magic | 4 B | `EXPR` |
| major | 1 B | Link Protocol major, so an old daemon can filter rather than misparse |
| device id | 6 B | derived from the Bridge MAC; stable across IP changes |
| name | ≤16 B | display name, derived by default as `Explorer-XXXX` |
| tcp port | 2 B | the Link Protocol listener |
| flags | 1 B | bit 0 provisioned, bit 1 robot link ready, bit 2 setup_mode; bits 3–7 reserved (zero) |

It is sent only while the station is associated and the Link Port is enabled. It carries no secret;
its only consumer is the daemon. The daemon listens on a fixed UDP port, deduplicates by device id,
lists what it finds and caches the last endpoint per robot, so discovery is only needed on first
connect or after an address change. A manual `host:port` entry covers networks that block broadcast
or isolate clients. IPv4 only.

`setup_mode` is observable only where the station Beacon exists. ADR-0036's Setup Mode is APSTA
when a station config exists and AP-only otherwise; in APSTA the Beacon still reaches the home LAN
and carries the flag, so the daemon can report `setup_mode` and decline to connect. In AP-only
setup (unprovisioned, or a station that is down) there is no station Beacon, so the daemon shows
`no_robots` and must not infer setup from silence. `tcp_port = 0` means the Beacon is advertising
a portal-only Bridge with no Link listener; the daemon treats it as `setup_mode` and does not
connect. The Beacon flag is a small Bridge change, not a robot one.

*The pairing token.*

One token per robot, 60 bits from the hardware RNG, displayed as twelve unambiguous base32
characters in three groups of four. It is generated at first boot, so no unauthenticated window
ever exists, and it is stored in ADR-0036's credential record. The user reads it on the panel's
`Link` screen (action 1 below) and types it into the app once; the daemon stores it locally,
plaintext, protected by the PC user account. The token is never sent on the wire.

*The handshake.*

Before the Bridge forwards a single Link Protocol byte, the daemon and the Bridge prove knowledge of
the token in a mutual HMAC-SHA256 challenge-response:

1. the daemon opens the TCP connection and sends a nonce;
2. the Bridge returns `HMAC(token, nonce)`; the daemon verifies it, so a spoofer cannot make the
   daemon believe it is the robot, and the daemon never sends token-derived bytes first;
3. the Bridge sends its own nonce; the daemon returns `HMAC(token, nonce)`, which the Bridge
   verifies and rate-limits failures against;
4. only then does the byte stream become the Link Protocol.

A captured transcript is a keyed hash of a chosen nonce; it does not reveal the token, and 60 bits
is out of offline brute force at that point. This is not TLS: an active interceptor can relay a
session, which is outside the boundary below.

*Enforcement.*

The gate is the Bridge, the only network-facing component. The robot never sees an unauthenticated
frame, so its `last_peer_frame` and its deadman only ever count authenticated peers — a LAN device
without the token cannot even keep the lease alive. The robot's Active Claim model (ADR-0034) is
untouched, and the Link Protocol is unchanged: the exchange is a transport preamble, not a
command, event, stream or control id. Ticket 15 chooses the concrete carrier at its boundary (a TCP
preamble, or an equivalent), provided the Bridge enforces it before forwarding.

*One client.*

One authenticated connection at a time. A connection has 5 s to authenticate before the Bridge drops
it, and an unauthenticated connection never blocks a new one, so a stale or hostile half-open cannot
lock the owner out. A second authenticated client is refused while the first is live, and the daemon
reconnects with backoff. Multiple operators and multiple daemons stay out of scope, consistent with
ADR-0034; several browser tabs of one operator are the daemon's business.

*Revocation.*

Three local actions on the panel's `Link` screen, carried by `BRIDGE_CONTROL`'s reserved action byte
(ADR-0036):

| action | effect |
|---|---|
| 0 | forget Wi-Fi and enter Setup Mode — clears the network **and** the token |
| 1 | show the pairing code on the panel |
| 2 | rotate the pairing code — new token, old one invalid, live authenticated connection closed |

The portal also offers "New pairing code" while the setup AP is up, so a code seen over a shoulder
can be revoked without touching the home network. No revocation needs a reflash, and none needs the
PC.

**Amendment (2026-11-07, ticket 07).** Action 0 clears the token by **rotating** it in the same
step rather than leaving the slot empty. The requirement is that the old code stops working and
the network is forgotten; a token must also exist at all times so no unauthenticated window opens,
and a later `BRIDGE_CONTROL` action 1 can show the new code without a reboot. The record therefore
becomes a token-only record with a freshly minted token. This supersedes the table's "clears the
network **and** the token" wording only in how the clear is represented: the old token is invalid
immediately **when the record write succeeds**. If the write fails, the acknowledgement is
`Internal`, the old code stays live, and the change is RAM-only; a TRNG failure clears the network
but cannot rotate the code, which is also reported as `Internal`. Action 2 follows the same rule:
the new token is persisted before it is acknowledged and before the live session is closed, and a
failed write leaves the old code and session in place. The portal's "New pairing code" action
remains unimplemented; ticket 07 ships the panel's action 2, and the portal page's route set is
unchanged from the spec's Setup Mode section.

*The trust boundary.*

The boundary is the home LAN behind WPA2/WPA3, plus one pairing token per robot. Anyone holding both
may command it. There are no user accounts, no per-operator identity and no audit. The link is
plaintext in v1: an active LAN interceptor or relay is outside the boundary, and TLS is the change
that would move it. The internet is out of scope. What a token holder can do is bounded by the
robot's local obstacle/floor-drop brakes, the panel Stop and the 2 s deadman (ADR-0035).

*Failure UX.*

| failure | what the user sees |
|---|---|
| nothing discovered | "no robots found" and the manual `host:port` field |
| pairing rejected | "unauthorized", naming the panel's `Link` screen |
| a second client | "another client is connected", then backoff |
| robot or Bridge absent | the link status the app already shows |

The daemon binds to loopback unless the operator opts into serving the app on the LAN; when it does,
the same token gates the app-facing WebSocket too (ticket 15).

**Consequences**

- The Bridge stops being a pure pump: it is the network edge, it owns discovery and authentication,
  and it enforces the single client. Its role and the robot's stay separated: network trust is the
  Bridge's, authority is the robot's.
- The robot gains no auth state, no identity and no protocol change; its deadman and lease become
  stronger for free, because they only ever see authenticated peers.
- A guest or a stray device on the LAN cannot drive the robot or keep its session alive, even
  without TLS. A LAN attacker who can actively intercept or relay can, and that is accepted.
- Discovery is one broadcast per 3 s from the Bridge; the daemon caches endpoints, so it is not a
  continuous dependency.
- Revocation is local and instant: the panel can rotate the code or forget the network, and the
  daemon fails closed to "unauthorized".
- The beacon advertises the robot's presence to the whole LAN. That is the same boundary, and the
  beacon carries no secret, but it is a deliberate statement rather than an accident.
- The daemon grows a discovery listener, a token store and an auth handshake; the app grows a
  pairing field and three failure states. Ticket 15 and ticket 13 own those.

**Open items**

- Implement the beacon (including the `setup_mode` flag and `tcp_port = 0`), the auth preamble, the
  RNG token generation, `BRIDGE_CONTROL` actions 1 and 2, the daemon's discovery and token store,
  and the app's pairing/failure screens.
- Choose the fixed UDP/TCP ports and keep the beacon interval a constant; decide where the
  Bridge↔daemon constants live (ticket 14's layout) without putting them in the robot's path.
  Ticket 14 is resolved in ADR-0040: `remote/bridge-net` holds the Beacon, the preamble and the
  ports, and the robot never depends on it.
- Pick the `no_std` HMAC/SHA-256 crates and record their code-size cost on the Bridge.
- Confirm the C3's hardware RNG source and generate the token once, at first boot.
- Ticket 15 is resolved in ADR-0039: the daemon's auth carrier is raw TCP with a framed HMAC
  preamble; the token file, loopback default and app-facing auth are recorded there.
- Ticket 13 owns the pairing entry and the "no robots / unauthorized / another client" states.
- TLS is the trigger that would move the plaintext part of this boundary; it is out of scope now.
- If `embassy-net` moves to `xarxa` (map Notes), the beacon and portal are userspace and unchanged.
