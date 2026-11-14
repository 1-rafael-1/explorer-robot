# bridge-firmware

The C3 (ESP32-C3) side of the Bridge: a `no_std` Embassy firmware that sits between the robot's
UART and the home network. This workspace targets `riscv32imc-unknown-none-elf` and keeps its own
`.cargo/config.toml`, so it stays out of the robot's and daemon's host workspaces (ADR-0040).
The full operator page is [docs/components/bridge.md](../docs/components/bridge.md); the repo map
and data flow are in [docs/architecture.md](../docs/architecture.md).

It currently holds two binaries:

| binary | purpose |
|---|---|
| `bridge` | the current firmware: UHCI RX DMA into the pump, the `bridge-net` preamble and single-client gate, the Discovery Beacon, the persisted credential record and pairing token, the bounded setup AP with its hand-rolled DHCP, DNS catch-all and HTTP portal, the panel's `BRIDGE_CONTROL` actions, Bridge Presence and the task watchdog |
| `stack-soak` | the original language spike, kept as the regression soak: APSTA, a UART loopback and a TCP echo |

`bridge-core` is the host-testable pump crate the `bridge` binary wires to the HAL (see
[`bridge-core/README.md`](bridge-core/README.md)).

## Bench wiring

The C3's USB connector is the flashing and `defmt` console. Identify it by its
`usb-Espressif_USB_JTAG_serial_debug_unit_…` entry under `/dev/serial/by-id`; the number can be
`/dev/ttyACM0`, `/dev/ttyACM1`, or another `ACM` index. The robot-facing link is **UART1** on the
C3 SuperMini harness pins:

| signal | C3 SuperMini | connects to |
|---|---|---|
| UART1 TX | GPIO4 | the USB-TTL adapter's RX, or the robot's `EXP_RX` (GPIO37 on J10) |
| UART1 RX | GPIO3 | the USB-TTL adapter's TX, or the robot's `EXP_TX` (GPIO36 on J10) |
| ground | GND | the adapter's GND / robot J9.2 |
| Bridge Presence | GPIO0 | the robot's `EXP_IRQ` (GPIO33 on J10) |
| Bridge reset | `EN` (RST button's **outer** leg) | the robot's `EXP_EN` (GPIO34 on J10) |
| spare | GPIO1 | the robot's `EXP_BOOT` (GPIO35 on J10); reserved input |
| power | 5V | the robot's `EXP_5V` (J9.1) in the sealed build; USB on the bench |

The presence pin is fixed by the harness at **GPIO0** (no build-time override). The RST button's
**outer** leg is `EN` (high when powered) and its **inner** leg is GND; the robot drives `EXP_EN`
high to run, low to reset, and the 10 ms low pulse is ADR-0032's presence-retry. GPIO1 is a spare
**robot → Bridge** input with a weak pull-down; it is deliberately not the C3's BOOT strap
(GPIO9), which stays unconnected. Do not connect the C3's 3V3 pin to the robot: each side
regulates its own 3.3 V.

Use a **3.3 V** USB-TTL adapter; the C3's pins are not 5 V tolerant. Identify the adapter by its
`usb-1a86_…` (QinHeng/CH343) entry under `/dev/serial/by-id`; depending on the kernel it binds as
`/dev/ttyUSB*` (`usb-serial`) or `/dev/ttyACM*` (`cdc_acm`). That by-id entry is where the Robot
Stub runs. Never point the stub at the Espressif console entry: that is the C3's USB console, not
the robot-facing UART.

For the `stack-soak` regression run, no adapter is needed: jumper **GPIO6 to GPIO7** so its UART1
loopback works (that binary keeps the C3's default pins). For `bridge`, jumper **GPIO3 to GPIO4**
instead: a `link-probe.py` `HELLO` is forwarded out of UART1, parsed as a valid robot-side frame on
the way back, and sets the Discovery Beacon's link-ready flag for five seconds (the router then
ignores it, because a robot-side `HELLO` without `IS_ACK` is not a legal robot frame).

Both directions run **921600 8N1**, the baud ADR-0033 fixes for the robot, the Bridge and the
daemon.

## The credential record and the pairing token

The `bridge` binary has no compile-time station credential. Provisioning is the setup portal's
job; the result is one versioned, CRC-checked record on a dedicated data partition:

```text
magic, version, flags, ssid_len, ssid, pass_len, passphrase, token[12], crc16
```

`bridge-core::record` owns the layout and the CRC, so the codec is host-tested; the firmware owns
the `sequential-storage` map over `esp-storage` through a small partition adapter. The record's
CRC is the Link Protocol's CRC-16/CCITT-FALSE, reused rather than inventing a second checksum.

A blank or erased partition boots **unprovisioned**: the Bridge mints a 60-bit token from the
hardware TRNG (`TrngSource` plus `Trng`; a bare `Rng` is not cryptographic before the radio is up)
and stores a token-only record. The twelve characters in three groups of four are the HMAC key
verbatim; the alphabet omits `I`, `L`, `O` and `U` so the code is unambiguous when read from the
panel. The boot banner prints it as `pairing_code=XXXX-XXXX-XXXX`.

Only a record with a station credential pair sets the Beacon's `provisioned` bit. A missing
record, wrong magic, wrong version, bad CRC, bad flags, bad credential lengths or a token outside
the alphabet all mean unprovisioned, and the Bridge fails closed into Setup Mode instead of
misreading a changed layout.

The setup AP is `ExplorerRobot-XXXX` (uppercase hex of the AP interface MAC's last two bytes),
WPA2-PSK `explorer-setup`, at most two clients, on `192.168.4.1/24` with a two-address DHCP pool.
`scan_async` returns `Other` in AP-only mode on this beta, so the unprovisioned window runs APSTA
with a parked station config that is never connected; the portal is still the only served surface.

`esp-radio` 1.0.0-beta.1 exposes only `Wpa2Personal`, so a WPA3-only or PMF-required access point
cannot be joined yet. The startup scan logs each BSS's advertised auth mode, which settles that
before any credentials are tried.

## The partition table

`partitions.csv` is the ESP-IDF table the firmware expects:

| partition | type | offset | size |
|---|---|---:|---:|
| `nvs` | data/nvs | `0x9000` | 24 KiB |
| `phy_init` | data/phy | `0xF000` | 4 KiB |
| `factory` | app/factory | `0x10000` | 3 MiB |
| `bridge_data` | data/undefined | `0x310000` | 16 KiB |

`bridge_data` is deliberately `undefined` (data subtype `0x06`), not a custom `0x40` subtype:
`espflash` and `esp-bootloader-esp-idf` both reject `0x40` at parse or lookup time. The runtime
finds the partition by its label, so moving it is a one-line change here plus a re-provision.

The table is flashed as an **explicit set**. `espflash flash` writes the bootloader, this table and
the app, and leaves `bridge_data` alone: credentials and the pairing token survive a normal
firmware update. `espflash erase-flash`, or moving `bridge_data`, loses the record — which the
version and CRC make safe (the Bridge fails closed into Setup Mode and mints a new token).

## Flash and monitor

The repository's runner in `.cargo/config.toml` now passes the custom table, so `cargo run --bin
bridge` flashes the same explicit set. The equivalent direct command is:

```sh
cd bridge
touch src/bin/bridge.rs
cargo build --release --bin bridge && \
espflash flash --monitor --chip esp32c3 --port /dev/ttyACM1 --log-format defmt \
    --partition-table partitions.csv --partition-table-offset 0x8000 \
    target/riscv32imc-unknown-none-elf/release/bridge
```

`--partition-table` makes `espflash` write the bootloader, the table and the app; it does not
write `bridge_data`, so the record survives the next flash. The port in these commands is the C3
console from `by-id`; on the current bench it is `/dev/ttyACM1`. If `espflash` reports every section
`Skipped! (checksum matches)`, it does not reset the chip; run `espflash reset` or change the build
before testing, or the 10-minute window will already be running.

The end-to-end sequences, with the exact boot lines and probe output to expect, are
[Bench acceptance run: the network edge](#bench-acceptance-run-the-network-edge),
[Bench acceptance run: Setup Mode](#bench-acceptance-run-setup-mode) and
[Bench acceptance run: the credential record](#bench-acceptance-run-the-credential-record).
`Ctrl-C` ends the monitor and frees `/dev/ttyACM0`.

The monitor resets the board, so the boot banner and the association lines appear after it
attaches. It holds the serial port, so stop it before the next flash.

The console prints the boot banner, the association/scan lines, the station address once DHCP
completes, and a status line every ten seconds.

## Bench acceptance run: the credential record

This is the record and token lifecycle. It needs no station credentials; the portal's successful
provisioning step is the owner's, because the home SSID and passphrase are never written down.
The Robot Stub's `BRIDGE_CONTROL` item additionally needs the USB-TTL adapter, which is not on the
bench (see the note at the end).

### Build, flash and boot

```sh
cd bridge
touch src/bin/bridge.rs
cargo build --release --bin bridge && \
espflash flash --monitor --chip esp32c3 --port /dev/ttyACM0 --log-format defmt \
    --partition-table partitions.csv --partition-table-offset 0x8000 \
    target/riscv32imc-unknown-none-elf/release/bridge
```

On a blank (or deliberately erased) `bridge_data` the first boot mints the token:

```text
[INFO ] record: partition bridge_data at 0x310000 (16384 bytes)
[INFO ] record: none on the data partition
[INFO ] record: stored (provisioned=false bytes=22)
[INFO ] record: minted a pairing token into the data partition
[INFO ] bridge identity: device_id=… name=Explorer-XXXX pairing_code=XXXX-XXXX-XXXX
[INFO ] setup: window opened (unprovisioned)
[INFO ] ap: setup ssid=ExplorerRobot-XXXX password=explorer-setup channel=1 max_clients=2 address=192.168.4.1/24
```

The token is twelve characters written as three groups of four. Those twelve characters' bytes
are the HMAC key the daemon stores; there is no decoding step. The record is a 22-byte token-only
record until credentials are submitted.

### The persistence matrix

```sh
# 1. reset: the same token loads again
espflash reset --chip esp32c3 --port /dev/ttyACM0
# expect: record: loaded (provisioned=false pairing_code=XXXX-XXXX-XXXX)

# 2. reflash the app and table: the record is outside them and survives
espflash flash --chip esp32c3 --port /dev/ttyACM0 \
    --partition-table partitions.csv --partition-table-offset 0x8000 \
    target/riscv32imc-unknown-none-elf/release/bridge
# expect the same pairing_code on the next boot

# 3. lose the record: fail closed, mint a new token
espflash erase-parts --chip esp32c3 --port /dev/ttyACM0 \
    --partition-table partitions.csv --after no-reset bridge_data
# next boot: record: none -> stored -> minted ..., with a new pairing_code
```

`espflash erase-flash` is the same case at whole-chip scope: the app is gone too, so reflash the
table and app, and the blank `bridge_data` mints afresh. Moving `bridge_data` in `partitions.csv`
has the same effect because the runtime finds it by label.

### Provisioning through the portal (owner)

Join `ExplorerRobot-XXXX` with `explorer-setup`, load `http://192.168.4.1/`, submit the home
SSID/passphrase, and expect `record: stored (provisioned=true bytes=…)` before the join. After the
30 s stable close, `espflash reset` boots **without** a setup window:

```text
[INFO ] record: loaded (provisioned=true pairing_code=XXXX-XXXX-XXXX)
[INFO ] wifi: station mode
[INFO ] sta: associated ssid=… channel=…
```

The Beacon then carries `tcp_port=9000 flags=0x01 (provisioned)`. A reset, a power cycle and a
reflash all preserve the pair; `espflash erase-parts … bridge_data` returns to unprovisioned.

### The Robot Stub's `BRIDGE_CONTROL` actions (blocked)

With the USB-TTL adapter on UART1 (`/dev/ttyUSB0`), the panel path is:

```sh
cd remote
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona bridge-control --action 0
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona bridge-control --action 1
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona bridge-control --action 2
```

Expect, on the stub:

```text
robot-stub: BRIDGE_CONTROL action 0 seq 1 replied: Setup { status: Ok, ssid: "ExplorerRobot-XXXX", passphrase: "explorer-setup" }
robot-stub: BRIDGE_CONTROL action 1 seq 1 replied: PairingCode { status: Ok, code: "XXXXXXXXXXXX" }
robot-stub: BRIDGE_CONTROL action 2 seq 1 replied: PairingCode { status: Ok, code: "YYYYYYYYYYYY" }
```

On the Bridge: `pump: robot BRIDGE_CONTROL action=… seq=…`, then
`panel: forgot the network; acknowledging with the setup identity` and `panel: forgot the network
and raised Setup Mode` (action 0), `panel: showed the pairing code` (action 1) or `panel: rotated
the pairing code` (action 2). Action 0 rotates the token, persists a token-only record and
acknowledges with the setup credentials; the loop then raises the window. Action 2 persists the
new token before acknowledging and, only on success, drops the live session (`tcp: panel action
dropped connection …`). A `Status::Internal` reply means the write failed (or, for action 0, the
window raise could not be queued): the change is RAM-only, and a failed action 2 leaves the old
code and session live.

**Blocked on the bench:** no 3.3 V USB-TTL adapter is present, so this last item is not observed
on hardware yet. The wire contract is covered by `remote/robot-stub/tests/bridge_control.rs` and
`link-core`'s tests, and the Bridge's action→reply mapping by `bridge-core`'s routing tests; the
remaining risk is the firmware glue (the ACK frame flags/seq and the UART enqueue).

## Browser provisioning round-trip

This is the browser-driven version of the portal round trip: forget the station over UART1, join the
Bridge's setup AP, provision the target network from the page, and rejoin the bench AP. A human
drives the radio and the browser; no agent is involved.

### Prerequisites

- The C3 console on `/dev/ttyACM1` and the CH343G UART adapter on `/dev/ttyACM0`.
- The `robot-stub` binary: `cd remote && cargo build -p robot-stub`.
- The target AP's SSID and passphrase (the throwaway AP or the home LAN), and the bench AP's saved
  NetworkManager connection (named `PandorasMesh` here). Keep the passphrase out of this
  repository, shell history and saved logs; use a placeholder such as `<target-ap-passphrase>`
  anywhere you write it down.

### 1. Monitor the Bridge and forget the network

Start the monitor in one terminal:

```sh
cd bridge
espflash monitor --chip esp32c3 --port /dev/ttyACM1 --log-format defmt \
    --elf target/riscv32imc-unknown-none-elf/release/bridge
```

Wait for the boot banner (`pairing_code=XXXX-XXXX-XXXX`) and the ten-second `status:` lines. In a
second terminal, send `BRIDGE_CONTROL` action 0 (Forget) from the robot-facing UART:

```sh
cd remote
cargo run -p robot-stub -- --port /dev/ttyACM0 --persona bridge-control --action 0
```

Expect the stub's ACK and the Bridge raising Setup Mode:

```text
robot-stub: BRIDGE_CONTROL action 0 seq 1 replied: Setup { status: Ok, ssid: "ExplorerRobot-D064", passphrase: "explorer-setup" }
panel: forgot the network; acknowledging with the setup identity
panel: forgot the network and raised Setup Mode
ap: setup ssid=ExplorerRobot-D064 channel=1 max_clients=2 address=192.168.4.1/24
```

### 2. Arm the safety timer before joining the setup AP

`ExplorerRobot-D064` has no Internet. Arm a detached timer **before** the radio leaves the bench AP,
so a forgotten step cannot strand the host offline:

```sh
setsid bash -c 'sleep 300; nmcli connection up <internet-connection>' </dev/null >/dev/null 2>&1 &
```

Use whichever saved connection has Internet (`PandorasStarMobile` or `PandorasMesh` here). The guard
restores only Internet; it does not restore the bench AP or recover a lost portal session. Keep the
five minutes in mind.

### 3. Provision through the portal

Join `ExplorerRobot-D064` with the setup passphrase `explorer-setup` in the GUI network menu, then
open `http://192.168.4.1/` and submit the target AP's SSID and passphrase. The page's status line
advances `unprovisioned` → `connecting` → `connected at <ip>`; the first poll after the submit can
time out for a few seconds while the single radio scans off-channel. On the monitor:

```text
portal: joining … while the AP stays up
sta: associated … channel=…
sta: address=<bench-subnet address>   # e.g. 10.48.199.94; 192.168.4.100 is the laptop on the setup AP
```

About 30 s after the association is stable the setup window closes and the setup AP disappears.

### 4. Rejoin the bench AP and repair the daemon

The guard restores an Internet connection; rejoin the bench AP (`PandorasStarMobile`) for the
daemon. Action 0 rotated the pairing token, so the daemon's stored code is stale. Learn the new code
from the boot banner
(`pairing_code=XXXX-XXXX-XXXX`) after a reset, or ask the panel to rotate it again:

```sh
cd remote
cargo run -p robot-stub -- --port /dev/ttyACM0 --persona bridge-control --action 2
```

Action 2 replies `PairingCode { …, code: "YYYYYYYYYYYY" }`; use that reply (or the boot banner) as
the new key. The config stores the twelve characters **without dashes**. Update
`~/.config/explorer-remote/config.toml` — or the file passed with `--config` — under the Bridge's
twelve-hex device id:

```toml
[devices."<device-id-hex>"]
token = "<new-pairing-code>"
name = "Explorer-D064"
```

Then start the daemon and a `/ws` client:

```sh
cd remote
cargo run -p remote-daemon -- --no-open
# Open the printed http://127.0.0.1:8765 URL, or point any WebSocket client at
# ws://127.0.0.1:8765/ws, send {"type":"hello","protocol":1}, then a 1 Hz heartbeat.
```

The app's `connection` message and the bench's headless client both report
`connection state=connected` once the Session comes up.

### 5. Confirm the Beacon and the Bridge status

On the bench LAN:

```sh
cd bridge
python3 tools/beacon-watch.py --count 3
```

A provisioned Bridge advertises `tcp_port=9000` with `flags=0x01 (provisioned)`; while the setup
window is open it advertises `tcp_port=0` with the `setup_mode` bit. The monitor's next `status:`
line carries `ring_used`, `ring_entries` and the drop counters, and `auth_ok` / `tcp_accepts` rise as
the daemon connects. Never commit the passphrase, the pairing code or a filled-in
`config.toml`.

## Bridge Presence and the watchdog

### Bridge Presence

The Bridge drives its presence line high once the scheduler, the radio controller, the UHCI RX
task and the UART TX task are up, and leaves it high; only a reset drops it. The pin is fixed by
the SuperMini harness at **GPIO0** and wired to the robot's `EXP_IRQ` (GPIO33 on J10). Presence says
the board is attached, not that a Remote is connected; it is not the deadman (ADR-0035).

The robot's `EXP_EN` is wired to the C3's `EN` through the RST button's **outer** leg (the inner
leg is GND): the robot drives it high to run and low to reset, so ADR-0032's 10 ms presence-retry
pulse still works. The robot's `EXP_BOOT` is wired to **GPIO1** as a spare **robot → Bridge**
input with a weak pull-down; it is not the C3's BOOT strap, and GPIO9 stays unconnected. The C3's
`EN` is the only reset path in the sealed build (no buttons or USB); the RWDT covers a hang and
the robot's `EXP_EN` pulse covers a silent Bridge.

Bench check without the harness: a multimeter or LED from GPIO0 to GND reads high a few hundred
milliseconds after reset and stays high. `espflash reset` drops it while the chip resets. If either
UART task fails to spawn, the line is not driven at all. With the harness fitted, verify the 10 ms
`EXP_EN` pulse actually resets the board through the EN pull-up/RC; lengthen the pulse if not, and
measure the C3's boot-to-presence time against the robot's `PRESENCE_GRACE` from the `EXP_EN`
release.

### The watchdog

The RTC watchdog (RWDT stage 0) resets the system after **20 s** without a feed. The watchdog task
feeds it only while every covered task has beaten within **12 s**, and it checks once a second.
Covered:

| slot | task |
|---|---|
| `uart-rx` | `uart_rx_dma_task`, the robot→PC pump |
| `uart-tx` | `uart_tx_task`, the PC→robot pump |
| `net-station` | the station `embassy-net` runner |
| `net-ap` | the setup AP `embassy-net` runner |
| `beacon` | the Discovery Beacon |
| `client-0`, `client-1`, `client-2` | the three Link Protocol connection workers |

Deliberately **not** covered: the portal's DHCP, DNS and HTTP workers (they block on captive-portal
traffic and only run in the bounded setup window; a reset there would drop the operator's session),
and the supervisory loop (its station joins and scan sweeps legitimately exceed the budget, and
covering it would force a much longer timeout). A slot is covered only after its task's spawn
succeeds, so a pool exhaustion logs and does not boot-loop.

The longest legitimate silence is the 5 s preamble deadline; the 3 s Beacon period and the 10 s
socket timeout sit below the 12 s maximum age. A record write blocks the executor briefly but is
well inside the 20 s hardware timeout.

### The stall hook (bench only)

Compile in a deliberate stall with:

```sh
BRIDGE_STALL_TASK=uart-rx cargo build --release --bin bridge
# uart-tx, net-station, net-ap, beacon, client-0; aliases: pump, net
```

The named task logs `watchdog: the … stall hook is armed; stopping the beat` and awaits forever.
The watchdog then logs `watchdog: … has not beaten for over 12s; withholding the feed` and stops
feeding, so the RWDT resets the board. In the 10 s status line, `wdt_feeds` stops rising and
`wdt_withheld` rises. The default image has no armed hook.

After the reset expect the normal boot banner with the **same** `pairing_code`, the station rejoin
and the daemon reconnect with no operator action; the robot keeps streaming at baseline and its
Boot Id is unchanged (ADR-0035). The daemon/Robot Stub half of that acceptance needs the USB-TTL
adapter and a provisioned station, as in the credential-record notes.

### UART RX recovery

The UHCI RX stream buffer stops the DMA when every descriptor is full. A station join or the
setup-AP teardown can block the executor long enough for the robot stream to fill the ring, so the
RX task treats done-and-silent for one second as an exhausted ring: it increments `dma_stalls`,
cancels the transfer and starts a new one. Frames resume after the restart; a frame split across
the restart is dropped and the parser resyncs. The task keeps beating throughout, so this is not a
watchdog event.

## Running the Robot Stub against it

The stub lives in the host `remote` workspace and opens the USB-TTL adapter at 921600 8N1:

```sh
cd remote
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona baseline
```

For the worst-case load (every stream at the ADR-0033 10 Hz tick, events and control traffic):

```sh
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona worst-case
```

For the panel's `BRIDGE_CONTROL` actions (one request at start-up, then the reply or timeout):

```sh
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona bridge-control --action 0
```

The stub prints its peer state every five seconds. The Bridge's status line is the other half of
the observation.

## The network edge

The listener runs the `bridge-net` preamble before a single Link Protocol byte crosses (ADR-0037):
`CLIENT_HELLO` → the Bridge proves the token → `CLIENT_AUTH` → `ACCEPT`, all inside 5 s. A wrong
proof is `DENY(unauthorized)`; a second authenticated client is `DENY(busy)` with `retry_ms=4000`;
five failures inside a minute close a one-minute refusal window. Several connection workers run
at once, so an unauthenticated connection is superseded and never blocks a new one.

The Bridge also broadcasts the Discovery Beacon every 3 s on the station interface, on UDP 9001:

| flag | source |
|---|---|
| `provisioned` | a credential record with a usable station credential pair; a token-only record is not provisioned |
| `link-ready` | a valid Link Protocol frame from the robot in the last 5 s (`beacon::LINK_READY_WINDOW_MS`) |
| `setup_mode` | the bounded window is open |
| `tcp_port` | the Link listener's port (9000); zero while Setup Mode advertises the portal only |

The identity served in the challenge is the same one the Beacon advertises: the efuse station
MAC as the device id, and `Explorer-XXXX` (the MAC's last two bytes) as the name.

`tools/beacon-watch.py` listens on UDP 9001 and prints each decoded Beacon with its flags. Run it
while `link-probe.py --loopback` sends a `HELLO` to see `link-ready` flip:

```sh
python3 tools/beacon-watch.py --count 3
```

### The authentication matrix

`tools/auth-probe.py` runs the bench matrix with a dependency-free `bridge-net` client
(`tools/bridge_preamble.py`). Pass the token in the environment, never as a flag:

```sh
# The boot banner prints XXXX-XXXX-XXXX; the key is those twelve characters without separators.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/auth-probe.py 192.168.1.42
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/auth-probe.py 192.168.1.42 --mode busy
```

Modes are `valid`, `wrong`, `silent`, `supersede`, `busy`, `ratelimit` and the default `matrix`.
The matrix runs `wrong` before `ratelimit`, so it spends one of the five failures and the
rate-limit check needs four more. The `ratelimit` check leaves the Bridge refusing new connections
for a minute. `supersede` holds a silent connection open and authenticates a second client while
it is pending; the second must succeed in under two seconds, not wait out the 5 s deadline.

The real daemon can be the client instead: give it the same token in
`~/.config/explorer-remote/config.toml`. `default_token` applies only to a **manual**
`host:port`; a discovered Bridge needs its own token keyed by the twelve-hex device id the
challenge and the Beacon serve. There is no token flag on the daemon CLI (ADR-0039).

## The `link-probe.py` round trip and soak

`tools/link-probe.py` is the dependency-free TCP client for the bench. It completes the
`bridge-net` preamble, connects to the Bridge's listener, sends a valid `HELLO` (control id 1, with
one enabled Requested Rate per stream) and decodes every robot frame it receives.

The pairing token is the twelve characters the Bridge prints at boot as `XXXX-XXXX-XXXX` (or the
panel shows through `BRIDGE_CONTROL` action 1). Those twelve characters' bytes are the HMAC key,
with the separators removed and no decoding step. Pass it in the environment; it is never a
command-line flag, so it does not land in the process arguments.

```sh
# One HELLO/HELLO_ACK round trip, then a five-second idle timeout.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42

# A sixty-second observation of the baseline or worst-case persona.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42 --duration 60 --quiet

# Stop after one hundred decoded frames.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42 --count 100 --quiet

# No robot: GPIO3 and GPIO4 jumpered together instead of wired to a USB-TTL adapter.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42 --loopback
```

With the jumper, the Bridge forwards the probe's `HELLO` out of UART1 and the jumper returns it. The
router then **ignores** it by design — a robot-side `HELLO` without `IS_ACK` is not a legal robot
frame — so nothing comes back over the socket and `--loopback` only sends and exits 0. The proof
is the Bridge's status line: `tcp_rx`, `uart_tx`, `uart_rx` and `good` must rise while `crc`,
`resync` and `bad` stay zero, and `link_ready=true` appears in the next status line and the
Beacon's link-ready flag for five seconds. (`dma_stalls` can tick once on a quiet line; see the
acceptance run.)

Pass the address the Bridge logs (`sta: address=…`). The probe prints lines such as:

```
control HELLO_ACK class=3 id=0x0001 seq=1 flags=IS_ACK len=41 major=1 minor=0 caps=0x00000008 boot=0x53545542 status=OK rates=POWER:10Hz/enabled ...
stream POWER class=0 id=0x0001 seq=17 flags=- len=3 payload=640c4a
event OBSTACLE class=2 id=0x0001 seq=4 flags=ACK_REQUESTED len=2 payload=0100
control THROTTLE class=3 id=0x0003 seq=2 flags=- len=7 stream=ALL action=PAUSE rate=0Hz ttl=1000ms
```

Its exit status is `0` when a `HELLO_ACK` arrived (and `--count`, if given, was reached) and `1`
on an idle timeout, a closed peer or a missing `HELLO_ACK`. The probe deliberately does **not**
acknowledge events; the real daemon does, so the stub retransmits events during a probe-only
soak. That is expected and is not Bridge loss.

The ADR-0033 acceptance observation is a worst-case stub run with the probe counting frames and
the Bridge status line showing `uart_drop=0 dma_stalls=0 crc=0` throughout.

## Status line

The `bridge` binary prints one line every ten seconds:

```
status: t=…s rssi=…dBm uart_rx=…B uart_tx=…B uart_kbps=…
        uart_drop=…B dma_stalls=… crc=… resync=… good=… bad=…
        tcp_rx=…B tcp_tx=…B tcp_kbps=… tcp_accepts=… tcp_tx_errors=…
        tcp_read_errors=… client_crc=… client_resync=…
        ring_used=…B ring_entries=… ring_reserved=…B drop_stream=… drop_no_client=…
        drop_reliable=… drop_bridge_control=… drop_on_close=…
        ring_encode_errors=… tx_q_drop=… reconnects=…
        auth_ok=… auth_bad=… auth_expired=… auth_superseded=… auth_failed=…
        deny_busy=… deny_refused=… deny_version=…
        beacons=… beacon_errors=… link_ready=…
        setup_open=… provisioned=… dhcp_offers=… dhcp_acks=… dns=…
        portal_reqs=… provisions=… forgets=… scan_sweeps=… scan_aps=…
        wdt_feeds=… wdt_withheld=…
```

| field | meaning |
|---|---|
| `uart_rx` / `uart_tx` | bytes drained from the UHCI RX DMA / written by the async UART TX task |
| `uart_drop` | bytes the parser discarded while resynchronising (a bad CRC or an out-of-range length) |
| `dma_stalls` | times the UHCI transfer reported done with no data for 1 s; a quiet bench UART can tick this once, so read it with `uart_drop`/`crc`/`good` |
| `crc` / `resync` | rejected candidates for a bad CRC / every rejected candidate |
| `good` / `bad` | valid robot-to-PC frames / candidate rejections |
| `tcp_rx` / `tcp_tx` | bytes read from / written to the client |
| `ring_used` / `ring_entries` / `ring_reserved` | the pump's coalescing-ring bytes, queued descriptors and event reservation; `THROTTLE` pauses at 6 KiB or 12 entries and resumes only at or below 2 KiB and 4 entries, refreshing the pause every 500 ms while either mark stays above its low watermark |
| `drop_stream` | stream samples the ring discarded (latest-wins pressure) |
| `drop_no_client` | robot frames dropped because no client was authenticated |
| `drop_reliable` | reliable entries the ring had to discard |
| `drop_bridge_control` | client-side `BRIDGE_CONTROL` frames dropped and counted |
| `drop_on_close` | client-bound entries discarded when a client disconnected |
| `ring_encode_errors` | client-bound entries that could not be re-encoded (should stay zero) |
| `tx_q_drop` | robot-bound frames dropped because the UART TX queue was full |
| `reconnects` | station re-associations |
| `auth_ok` / `auth_bad` | preambles that reached `ACCEPT` / proofs that were rejected |
| `auth_expired` | connections dropped at the 5 s authentication deadline |
| `auth_superseded` | preamble attempts a newer connection replaced |
| `auth_failed` | exchanges that ended without a verdict (closed or malformed) |
| `deny_busy` | connections denied because an authenticated client held the slot |
| `deny_refused` | connections denied because the failure limit's 60 s window was open |
| `deny_version` | connections denied for a preamble major mismatch |
| `beacons` / `beacon_errors` | Discovery Beacons sent / send attempts that failed (usually no DHCP yet) |
| `link_ready` | the Beacon's pinned link-ready flag at status time |
| `setup_open` / `provisioned` | the bounded Setup Mode window / a usable credential pair exists |
| `dhcp_offers` / `dhcp_acks` | DHCP replies the setup AP's hand-rolled server sent |
| `dns` | DNS catch-all answers sent |
| `portal_reqs` / `provisions` / `forgets` | HTTP requests read / valid provisioning submits / Forget actions |
| `scan_sweeps` / `scan_aps` | best-of-N sweeps completed / AP observations merged into the scan cache |
| `wdt_feeds` / `wdt_withheld` | RTC watchdog feeds / checks that found a stale covered task and withheld the feed (the second should stay zero in a healthy run) |

`uart_drop=0` on a sustained run is the ADR-0033 "zero loss" observation.

## The `stack-soak` regression soak

`stack-soak` is the original language spike. It brings up **APSTA**, transmits a known frame
pattern on UART1 (jumper GPIO6 to GPIO7), echoes TCP bytes on port 9000 on both interfaces, and
forces station disconnects after ten minutes.

```sh
cd bridge
SPIKE_STA_SSID='your-ssid' SPIKE_STA_PASSWORD='your-passphrase' \
    cargo run --release --bin stack-soak
```

Point `tools/soak.py` at the logged station address (or at `192.168.4.1` on the setup AP
`explorer-spike`):

```sh
python3 tools/soak.py 192.168.1.42 9000 600 1024
```

`stack-soak` is not the Bridge and must keep building alongside it; it has no Link Protocol
framing, no robot lifecycle and no auth.

## Validation

From `bridge/`:

```sh
cargo build --release
cargo clippy -- -D warnings
cargo +nightly fmt --all -- --check
cargo test -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests
espflash partition-table partitions.csv
```

`bridge-core` has 176 host tests: the pump/window/DHCP/DNS/portal suites, 7 `pairing` tests
(length, alphabet, bit mapping, grouping), 12 `record` tests (round trips and every fail-closed
case: magic, version, CRC, length, flags, token, credentials) and 11 `heartbeat` tests (the
watchdog's coverage and liveness rule). `bridge-net` keeps its 18.

`espflash partition-table` parses the CSV and prints the four partitions; the `factory` app has
3 MiB, and the current image uses about 18.7% of it. The record adds roughly 21 KiB of flash over
the Setup Mode image.

`cargo build --release` must link both `bridge` and `stack-soak` for
`riscv32imc-unknown-none-elf`:

```sh
file target/riscv32imc-unknown-none-elf/release/bridge
file target/riscv32imc-unknown-none-elf/release/stack-soak
```

The probes' syntax check needs no dependencies:

```sh
python3 -m py_compile tools/bridge_preamble.py tools/link-probe.py tools/auth-probe.py tools/beacon-watch.py
ruff check tools/bridge_preamble.py tools/link-probe.py tools/auth-probe.py tools/beacon-watch.py   # when ruff is installed
```

## Troubleshooting

- **`espflash` cannot open the port.** Check that the C3 console port from `by-id` exists, that the
  user is in the `dialout` (Debian/Ubuntu) or `uucp` (Arch) group, and that no monitor is holding
  the port.
- **The board does not enter the bootloader.** Hold **BOOT**, tap **RESET**, release **BOOT**, then
  flash.
- **Multiple boards.** `espflash` picks the first matching port; pass `--port` explicitly and keep
  `--chip esp32c3`.
- **The stub opens but the Bridge sees nothing.** Confirm the adapter is the `usb-1a86_…` entry,
  not the C3's `usb-Espressif_…` console; confirm TX↔RX are crossed (GPIO4 to the adapter's RX,
  GPIO3 to its TX); confirm both ends are at 921600 8N1 and share ground.
- **Record lines are absent or say `no data partition`.** The board was flashed without the custom
  table. Flash again with `--partition-table partitions.csv --partition-table-offset 0x8000`; the
  boot log must show `record: partition bridge_data at 0x310000`.
- **A reflash re-provisioned the Bridge.** That is only possible after a partition erase or a table
  change. A normal `espflash flash` writes the bootloader, table and app only; check that
  `bridge_data` still points at `0x310000`.
- **`dma_stalls` rises.** On a quiet bench line the ADR-0033 "done-and-silent for 1 s" rule
ticks once; if `uart_drop=0`, `crc=0` and `good` rises, nothing was lost. Under real robot load,
suspect the Wi-Fi stack holding the DMA task longer than the descriptor ring can absorb. Check the
heap size and the task priorities before suspecting the UART wiring.
- **The listener is unreachable.** The Bridge logs `sta: address=…` only once DHCP completes; the
  listener exists on the station interface only, never on an AP interface.
- **Association fails.** The startup scan logs the BSS's auth mode. The beta radio cannot do WPA3
  or PMF; use a WPA2 access point for now.
