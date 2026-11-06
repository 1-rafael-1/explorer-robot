# Wi-Fi provisioning: a bounded setup AP the panel can raise

The Link Port gives the robot a network, but the Bridge is a sealed, headless board: no display,
no touch, and — after this decision — no user-accessible recovery button. Credentials still have
to get in without a reflash, and the robot's touch panel is the only local UI the system owns.
The C3 supports station, access point and both at once, but the Rust stack ships no DHCP server,
so a portal is a real dependency rather than a page. The repo already persists calibration with
`sequential-storage`, and the Bridge has no RTC.

The ticket's constraint is the awkward one: an AP that never expires is a standing security
surface, yet the only local control surface is a robot the Bridge is supposed to serve. The
decision is therefore about bounded exposure and about which side of the UART owns recovery.

**Status:** accepted. The Bridge raises a **Setup Mode** — a 10-minute SoftAP with a captive portal
— when it is unprovisioned, after a sustained outage, or when the robot's panel asks. Credentials
live in a versioned record on a dedicated partition. Recovery is a panel action carried by one new
acked control frame, `BRIDGE_CONTROL` (`0x0004`), which amends ADR-0030's control acknowledgement
rule and ADR-0031's control id table. No firmware is written by this decision. ADR-0037 defines the
contents of the reserved trust-token slot and uses `BRIDGE_CONTROL`'s reserved actions 1 and 2 to
show and rotate the pairing code.

**Considered Options**

- **A plain fixed-address page instead of a captive portal.** Rejected. It needs no DNS or HTTP
  catch-all, but many phones will not stay on an AP that hands out no address, and typing
  `192.168.4.1` is a paper cut the portalless design never explains. The portal is what makes a
  headless box provisionable without a manual.
- **An open AP.** Rejected. Some phones nag on open networks, it is a standing lure, and the
  password it would save is public anyway, so it buys nothing the captive portal does not.
- **A per-unit AP password** (a label, or delivered over the UART handshake). Rejected. It needs a
  label or a Bridge→robot exchange, and the secret protects only the 10-minute window; the real
  trust boundary is ticket 12's token.
- **A permanent AP, or a recurring one.** Rejected. The ticket names a standing AP as the hazard.
  The window is one per boot plus on demand, and it always expires.
- **BLE provisioning.** Rejected. The template has no BLE stack, the stack it would add is larger
  than the portal's, and Web Bluetooth is browser-restricted.
- **USB-serial only.** Rejected as primary. The Bridge sits behind `J9`/`J10`; USB is the flashing
  path and cannot be the day-to-day one. It remains the out-of-band last resort.
- **A Boot-button long press.** Unavailable: the Bridge is sealed and the user has no access to
  the button. A reset while BOOT is held also enters ROM download mode, so even an accessible
  button would need the gesture to be detected while running.
- **`EXP_BOOT` as a long-press gesture from the robot.** Rejected. It repurposes a flashing strap,
  a coincident reset would enter download mode instead of booting, and it has no acknowledgement,
  so the panel could not tell the user whether the Bridge heard it.
- **Serving the control channel over the setup AP.** Rejected. It turns a bounded setup surface
  into an operating one and makes ticket 12's discovery work in two network modes. The Link
  listener binds to STA only.
- **ESP-IDF's `wifi_prov_mgr`.** Rejected. The Bridge stays full Rust (owner decision on ticket
  09), so the portal is built on `esp-radio`/`embassy-net` and the record on `sequential-storage`.
  The `esp-bootloader-esp-idf` crate stays: it is the ESP-IDF-format bootloader, not the
  framework.
- **A bounded setup AP plus a panel-raised recovery action.** Accepted.

**Decision**

*Setup Mode.*

The Bridge raises a SoftAP with a captive portal when any of three things is true:

1. the credential record is missing or unusable at boot;
2. the station link has been down for 5 continuous minutes since boot (once per boot);
3. the robot's panel sends `BRIDGE_CONTROL` requesting it.

It never runs permanently. The window is 10 minutes, closed earlier about 30 s after a stable STA
association, and a new window is raised only by the panel, a boot, or the next outage rule.

*Identity and security.*

| property | value |
|---|---|
| SSID | `ExplorerRobot-XXXX`, uppercase hex of the last two bytes of the AP interface MAC |
| auth | WPA2-PSK, password build-fixed, documented, and shown on the panel result screen |
| visibility | visible |
| clients | at most 2 |
| lifetime | 10 min, closed 30 s after a stable STA association |

The setup password is public by construction. It gates casual joining, not attack: someone in
range during the window can only move the Bridge to another network — a denial — and cannot
command the robot, because the control channel is not on the setup AP and ticket 12 owns trust.
The home passphrase is stored on the Bridge, is never sent to the robot and is never logged.

*Modes and exposure.*

| state | radio | what is served |
|---|---|---|
| steady | STA only | the Link Protocol on the STA interface |
| Setup Mode | APSTA when a station config exists, AP-only otherwise | the portal on the AP; the Link Protocol still on STA if associated |
| unprovisioned | AP only | the portal only |

The Link listener is bound to the STA interface. The setup AP is a provisioning surface, not a
second operating network. In AP mode the daemon cannot reach the robot and shows "in setup"; in
APSTA recovery the robot remains reachable on the home network exactly as before. That also
narrows ticket 12: the app is only expected to run in STA mode.

*Storage.*

A dedicated data partition, written with `esp-storage` and the repo's existing
`sequential-storage` pattern — one storage mechanism rather than a second one for the Bridge. One
record:

```
magic, version, flags, ssid_len, ssid, pass_len, passphrase, trust token slot, crc
```

A missing record, a wrong magic, a wrong version or a bad CRC all mean **unprovisioned**: the
Bridge fails closed into Setup Mode instead of misreading a changed layout. `espflash flash`
writes the bootloader, partition table and app, not the data partition, so credentials survive a
normal firmware update. `espflash erase-flash`, or a partition-table change that moves the data
partition, does not — and the version and CRC is what makes that case safe rather than silently
wrong. `espflash` images are always flashed as an explicit set; the build must not rely on an
implicit whole-flash write.

*Retry and recovery.*

Station reconnection backs off 1, 2, 4 … 60 s, forever, and the backoff resets on success. The
three recovery gestures, most available first:

1. **The panel.** A `Link` screen (a `ScreenEntry`, not a Procedure, so it takes no Active Claim)
   shows Bridge present and Remote connected, and offers **"Forget Wi-Fi & set up"** behind a
   confirmation. It clears the record and raises Setup Mode, and because it deliberately kills
   the STA link it **fires ADR-0035's failsafe first** instead of waiting for peer silence. It is
   local only; the Remote cannot trigger it.
2. **The portal.** A "Forget network" button while the AP is up.
3. **A power cycle.** A record that is unprovisioned raises a window at boot.

*Talking to the Bridge.*

There is no robot→Bridge message today, so the panel action needs one:

| property | value |
|---|---|
| class | control |
| id | `0x0004` `BRIDGE_CONTROL` |
| direction | robot → Bridge |
| payload | `action u8`: 0 = forget and enter Setup Mode; 1–255 reserved |
| routing | consumed by the Bridge, never forwarded to the daemon |
| reply | `IS_ACK` on the same id and seq: `status u8`, setup SSID, setup password |

`BRIDGE_CONTROL` joins `HELLO` and `PING` as an acknowledged control frame, amending ADR-0030's
"control is acked only for `HELLO`/`PING`". The Bridge is now a peer for two control messages —
`THROTTLE` outbound, `BRIDGE_CONTROL` inbound — rather than for envelopes alone. That is the
smallest protocol change that lets a sealed device recover itself, and it stays inside the existing
framing instead of adding a second robot↔Bridge channel.

The ACK carries the setup SSID and password so the panel can show them. Sourcing them from the
Bridge avoids a constant duplicated in two firmware projects, and it keeps the home passphrase out
of the robot entirely. If the Bridge does not answer, the panel says so and leaves the stored
credentials alone.

*What the user sees.*

At the robot: a Link screen with Bridge present / Remote connected, then a confirmed action, then
the setup SSID and password to join. At the phone: the setup SSID; then one page listing 2.4 GHz
APs (the C3's radio cannot see anything else), SSID and passphrase fields, a status line
(`unprovisioned` / `connecting` / `wrong password` / `connected to X at 192.168.…`) and Forget.
The scan list takes the best of several sweeps, because `scan_async` on this beta drops whole
sweeps (ticket 09). The page is plain HTML with inline CSS; it is Bridge firmware, not the PC app.

*Time.*

No clock requirement in v1. Plain HTTP and WebSocket on the LAN, DHCP for addressing, no TLS, no
certificates, no NTP; logs stamp uptime. If a cloud connection or HTTPS appears, it brings SNTP
with it; that is the trigger, not a requirement to design now.

**Consequences**

- A sealed Bridge can be provisioned and re-provisioned without a button, a reflash or opening the
  robot, because the UART is independent of Wi-Fi.
- The setup AP is a bounded surface with a public password and no control channel; the worst case
  in its window is a denial, and ticket 12's trust boundary is unchanged.
- The robot never learns the home passphrase, and the setup credentials have one source — the
  Bridge — so the two build efforts cannot drift.
- The Bridge gains a DHCP/DNS/HTTP dependency, and stops being envelope-only: it now consumes one
  control message. The cost is a small protocol amendment, not a second protocol.
- Credentials survive firmware updates but not a partition erase or a layout change; both fail
  closed into Setup Mode rather than misreading.
- The robot's only local recovery surface is a screen that cannot be reached while a Procedure is
  running (the panel follows Activity, ADR-0034). A stuck Procedure must be stopped first, which is
  correct: the panel is the operator's surface, and provisioning is not an Activity.
- USB flashing remains the last resort and now requires physical access to the port or the board;
  the enclosure must allow one of the two (ticket 10/16 territory).

**Open items**

- Implement the portal (DHCP via `edge-dhcp` or equivalent, a DNS catch-all, an HTTP server), the
  credential record, the three radio states, the `Link` screen and the confirmation flow.
- Add `BRIDGE_CONTROL` to `remote-protocol`'s control enum, the Bridge's routing filter and its
  ACK, and the robot's Link Task action path; amend ADR-0030's control-ack rule and ADR-0031's
  control ids as this ADR does.
- Confirm WPA2 and the client limit on the current `esp-radio` beta, and confirm the best-of-N scan
  behaviour once the stack is stable (ticket 09).
- Ticket 12 owns the trust token: the credential record reserves its slot and the portal is its
  delivery point; the setup AP address is `192.168.4.1`.
- The Bridge stays full Rust (ticket 09). If the beta's portal cost proves too high, the lever is
  a different `esp-radio`/`esp-hal` version or a hand-rolled DHCP/DNS responder, not a framework
  switch.
- Decide whether the enclosure keeps the C3's USB port reachable; the design assumes one of USB
  access or a removable board for the out-of-band recovery path.
