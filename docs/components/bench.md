# Bench

The bench is where the Bridge and the Remote meet hardware: a C3 SuperMini
running the Bridge image, the robot's Link Port or a 3.3 V USB-TTL adapter on
UART1, and the host tools that replace the robot or the Bridge. Every wiring,
serial-port and radio step here is **bench-required**: the bench has no
hardware-in-the-loop rig, so CI cannot run it.

**Verified:** 2026-11-08 — `python3 bridge-firmware/tools/link-probe.py --help`,
`python3 bridge-firmware/tools/auth-probe.py --help`,
`python3 bridge-firmware/tools/beacon-watch.py --help` and
`python3 bridge-firmware/tools/soak.py --help` (all printed usage, exit 0);
`cd remote && cargo run -q -p bridge-stub -- --help` and
`cd remote && cargo run -q -p remote-daemon -- --help` (usage, exit 0);
`cd remote && cargo run -q -p robot-stub -- --help` (no help flag; the parser
rejected it with `robot-stub: --help needs a value`, exit 1); and
`python3 .github/scripts/check_links.py` (26 Markdown files, 165 relative links,
all resolve, exit 0). No serial port was opened, nothing was flashed, and no
board or Wi-Fi link was touched.

## What the bench is

| Bench piece | Replaces | What runs there |
|---|---|---|
| C3 SuperMini on the harness | — | the real Bridge image, on UART1 and Wi-Fi |
| Robot Stub over a 3.3 V USB-TTL adapter | the robot | the real `remote-link` `LinkCore` behind a serial port |
| Bridge Stub on loopback | the Bridge | the `bridge-net` preamble, Discovery Beacon and scripted robot traffic |
| `remote-daemon` plus the probes | the Remote | discovery, the HMAC preamble and the Link Protocol |

The Link Port is the robot's `J9`/`J10` connector pair: switched 5 V and ground
on `J9`, UART1 and the handshake lines on `J10`. The Bridge is the network edge;
the Remote authenticates with the Pairing Token before any frame is forwarded.
Setup Mode is the Bridge's bounded provisioning AP, and it serves the portal
only—the Link Protocol listener binds to the station interface.

## Wiring and power — bench-required

### The harness

The C3's USB connector is the flashing and `defmt` console. The robot-facing
link is **UART1** on the C3 SuperMini harness pins:

| signal | C3 SuperMini | connects to |
|---|---|---|
| UART1 TX | GPIO4 | the USB-TTL adapter's RX, or the robot's `EXP_RX` (GPIO37 on J10) |
| UART1 RX | GPIO3 | the USB-TTL adapter's TX, or the robot's `EXP_TX` (GPIO36 on J10) |
| ground | GND | the adapter's GND / robot `J9.2` |
| Bridge Presence | GPIO0 | the robot's `EXP_IRQ` (GPIO33 on J10) |
| Bridge reset | `EN` (RST button's **outer** leg) | the robot's `EXP_EN` (GPIO34 on J10) |
| spare | GPIO1 | the robot's `EXP_BOOT` (GPIO35 on J10); reserved input |
| power | 5V | the robot's `EXP_5V` (`J9.1`) in the sealed build; USB on the bench |

The presence pin is fixed by the harness at **GPIO0**. The RST button's outer
leg is `EN` (high when powered) and its inner leg is GND; the robot drives
`EXP_EN` high to run and low to reset. GPIO1 is a spare robot-to-Bridge input
with a weak pull-down, not the C3's BOOT strap; GPIO9 stays unconnected. Do not
connect the C3's `3V3` pin to the robot: each side regulates its own 3.3 V.

Both directions run **921600 8N1** (ADR-0033). For the `bridge` image, jumper
**GPIO3 to GPIO4** on the C3 for a loopback; the `stack-soak` image keeps the
C3's default pins, so jumper **GPIO6 to GPIO7** for that one. The ticket-05/06/07
transcripts in the Bridge README predate the remap and show the old GPIO6/7
pins; use the table above for the current image.

### Power and the no-USB+J9 rule

The rule is: **never power from USB and `J9` at once.** Most C3 dev boards tie
VBUS to the 5 V pin, so with `J9.1` also connected the USB supply back-feeds the
robot's switched 5 V rail and, with the robot off, back-powers the RP2350
through the Link Port's 100 Ω series resistors.

- **Sealed build:** power from the robot's `EXP_5V` on `J9.1`, ground on
  `J9.2`. There is no USB and no button in the field; the C3's `EN`, driven by
  the robot through the RST button's outer leg, is the only reset path.
- **Bench with the robot attached:** power from `J9.1` and disconnect USB.
- **Bench without the robot (USB-TTL stubs):** power the C3 from USB and leave
  `J9.1` disconnected.
- **To flash over USB with `J9` attached:** unplug `J9` first, flash, then
  reconnect it only after the USB cable is removed. Never have both connected.

The Link Port is 5 V-powered and 3.3 V-logic: UART1, `EXP_IRQ`, `EXP_EN` and
`EXP_BOOT` are all 3.3 V, and nothing 5 V may reach a C3 or RP2350 GPIO. Use a
**3.3 V** USB-TTL adapter; the C3's pins are not 5 V tolerant, and its 5 V pin is
power only.

Every check in this section is **bench-required**.

## Prerequisites

| Tool | Used for | Notes |
|---|---|---|
| Python 3 | all four probes | standard library only; no packages to install |
| `nmcli` | joining the Setup Mode AP | NetworkManager on the bench host |
| `curl` | the Setup Mode portal | `http://192.168.4.1/` while the setup AP is up |
| 3.3 V USB-TTL adapter | the Robot Stub's serial port | QinHeng/CH343 `usb-1a86_…`; binds as `/dev/ttyUSB*` or `/dev/ttyACM*` |
| C3 console | flashing and `defmt` | `usb-Espressif_USB_JTAG_serial_debug_unit_…`; `/dev/ttyACM0`, `/dev/ttyACM1`, or another `ACM` index |
| Rust stable and `libudev` | `robot-stub`, `bridge-stub`, `remote-daemon` | the `remote/` workspace; `libudev-dev` and `pkg-config` on Linux |
| `espflash` | flashing the Bridge | see the [Bridge page](bridge.md) |

Identify serial devices by their `/dev/serial/by-id` entry, not by the device
number; the numbers can change between boots. This listing is
**bench-required**:

```sh
ls -l /dev/serial/by-id/
```

The Robot Stub examples name `/dev/ttyUSB0` for the robot-side adapter and
`/dev/ttyACM0` or `/dev/ttyACM1` for the C3 console. Never point the Robot Stub
at the `usb-Espressif_…` console entry; that is the C3's USB console, not the
robot-facing UART.

Run the probes from `bridge-firmware/` and the stubs and daemon from `remote/`.
Cargo reads `.cargo/config.toml` upward, so the current directory selects the
workspace's target and build settings (ADR-0040).

## Bench scripts

The `--help` forms in this section are safe and were run for the `Verified:`
line. Every invocation that opens a socket, a serial port or a radio is
**bench-required**.

### `link-probe.py`

The dependency-free Link Protocol client. It completes the HMAC preamble,
connects to the Bridge's listener, sends a `HELLO` and decodes every robot frame
it receives. Pass the Bridge's station address from its `sta: address=…` log
line. The Pairing Token is the twelve characters printed at boot as
`XXXX-XXXX-XXXX`, with the separators removed; it is never a command-line flag.

```sh
cd bridge-firmware

# One HELLO/HELLO_ACK round trip, then the five-second idle timeout.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42

# A sixty-second observation of a persona, printing only the final count.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42 --duration 60 --quiet

# Stop after one hundred decoded frames.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42 --count 100 --quiet

# No robot: jumper GPIO3 to GPIO4 on the bridge image and let the HELLO echo.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/link-probe.py 192.168.1.42 --loopback

# Read the token from a file instead of the environment.
python3 tools/link-probe.py 192.168.1.42 --token-file ~/.config/explorer-bench-token
```

Exit status is `0` only when a `HELLO_ACK` arrived (and `--count`, if given, was
reached). The full option list and the round-trip expectations are in the
[Bridge README](../../bridge-firmware/README.md#the-link-probepy-round-trip-and-soak).

### `auth-probe.py`

The bridge-net authentication matrix: `valid`, `wrong`, `silent`, `supersede`,
`busy` and `ratelimit`, or all of them in `matrix` order. It spends the
Bridge's failure budget, so run it last; the `ratelimit` check leaves a 60 s
refusal window.

```sh
cd bridge-firmware

# The full matrix against the Bridge's station address.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/auth-probe.py 192.168.1.42

# One check at a time.
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/auth-probe.py 192.168.1.42 --mode valid
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/auth-probe.py 192.168.1.42 --mode busy
BRIDGE_BENCH_TOKEN='XXXXXXXXXXXX' python3 tools/auth-probe.py 192.168.1.42 --token-file ~/.config/explorer-bench-token
```

Exit status is `0` only when every requested check passed.

### `beacon-watch.py`

The Discovery Beacon listener on UDP 9001. It decodes the Bridge's 3 s
datagram, including the `provisioned`, `link-ready` and `setup_mode` flags. It
needs no token, but it does need a station address (a Beacon is sent only while
the station is associated).

```sh
cd bridge-firmware
python3 tools/beacon-watch.py --count 3
python3 tools/beacon-watch.py --duration 15
```

Exit status is `0` when at least one Beacon was decoded; the printed line names
the device, the advertised Link port and the flag set.

### `soak.py` and the `stack-soak` image

`soak.py` is the TCP echo soak for the `stack-soak` regression image, not a Link
Protocol client. It sends self-identifying frames to port 9000 and requires each
one back unchanged.

```sh
cd bridge-firmware

# Terminal 1: the APSTA spike image, with its UART1 pins jumpered GPIO6 to GPIO7.
SPIKE_STA_SSID='your-ssid' SPIKE_STA_PASSWORD='your-passphrase' \
    cargo run --release --bin stack-soak

# Terminal 2: host port, duration in seconds, frame size in bytes.
python3 tools/soak.py 192.168.1.42 9000 600 1024
```

On the spike's setup AP (`explorer-spike`), point it at `192.168.4.1` instead.
Exit status is `0` only when at least one frame completed and every echo
matched; a Bridge that was never reachable fails the soak. See
[the `stack-soak` regression soak](../../bridge-firmware/README.md#the-stack-soak-regression-soak).

### `robot-stub`

The robot persona for the Bridge's bench: the real `remote-link` `LinkCore`
behind a 3.3 V USB-TTL adapter, opening the port at 921600 8N1. It has no
`--help`; its parser rejects an unknown flag by name.

```sh
cd remote

# Baseline moving robot.
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona baseline

# Every stream at the 10 Hz tick plus the largest event paced at 40 Hz.
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona worst-case

# One BRIDGE_CONTROL action (0, 1 or 2), then the reply or timeout.
cargo run -p robot-stub -- --port /dev/ttyUSB0 --persona bridge-control --action 0
```

`--action` requires `--persona bridge-control`. The other flags are `--boot-id`,
`--drop-every`, `--corrupt-every` and `--delay-ms`. It prints a one-line link
summary every five seconds; see
[the Robot Stub README](../../remote/robot-stub/README.md).

### `bridge-stub`

The Remote's bench peer: the server side of the HMAC preamble, the Discovery
Beacon and scripted robot traffic. It replaces the Bridge, not the robot.

```sh
cd remote

# The full flag list.
cargo run -p bridge-stub -- --help

# A loopback Bridge for the daemon: the Beacon goes to the local listener.
cargo run -p bridge-stub -- --beacon-addr 127.0.0.1:9001
```

The default token is `bench-token` (hex `62656e63682d746f6b656e`), the default
device id is `stub01` and the default Link port is 9000 on `127.0.0.1`. The
README lists the `--deny`, `--setup-mode`, `--portal-only`, `--stream` and
`--event` scripts.

### `remote-daemon`

The Remote's host relay. Run it from `remote/`; there is deliberately no token
flag, because the config store is the only token source.

```sh
cd remote

# The full flag list.
cargo run -p remote-daemon -- --help

# Serve the app on loopback without opening a browser.
cargo run -p remote-daemon -- --no-open
```

To drive it against the Bridge Stub headlessly, give it a config whose device id
is the stub's six-byte id (`stub01` → `737475623031`) and whose token is
`bench-token`:

```sh
# bench.toml, in remote/
cat > bench.toml <<'EOF'
[devices."737475623031"]
token = "bench-token"
EOF

cargo run -p remote-daemon -- --config bench.toml --no-open
```

The stub advertises its Link port in the Beacon, so the daemon discovers it on
UDP 9001; a manual `connect` message covers a network that blocks broadcast. See
[the daemon README](../../remote/daemon/README.md) for the app flags and the
config store.

### The Setup Mode portal — bench-required

The Bridge raises Setup Mode when its credential record is missing, after five
minutes without a station link, or when the panel asks. Join its AP and use the
portal:

```sh
nmcli device wifi connect ExplorerRobot-XXXX password explorer-setup
curl -s http://192.168.4.1/status     # unprovisioned
curl -s http://192.168.4.1/ | head
```

The AP is `ExplorerRobot-XXXX` (uppercase hex of the AP MAC's last two bytes),
WPA2-PSK `explorer-setup`, on `192.168.4.1/24`. The portal is the only service;
the Link Protocol listener is never on the AP interface. The successful
provisioning submit needs the owner's home SSID and passphrase and is not part
of this page: follow the
[ticket-06 run](../../bridge-firmware/README.md#bench-acceptance-run-ticket-06)
or the current [ticket-07 record run](../../bridge-firmware/README.md#bench-acceptance-run-ticket-07).

## Bench acceptance runs and further detail

These are **bench-required** and are the source material for the sequences above:

- [Bench wiring](../../bridge-firmware/README.md#bench-wiring) — the current pin
  table, levels and loopback jumpers.
- [Bench acceptance run (ticket 05)](../../bridge-firmware/README.md#bench-acceptance-run-ticket-05)
  — the network edge and the gate matrix (historical build inputs; the measured
  output is still the reference).
- [Bench acceptance run (ticket 06)](../../bridge-firmware/README.md#bench-acceptance-run-ticket-06)
  — Setup Mode and the portal.
- [Bench acceptance run (ticket 07)](../../bridge-firmware/README.md#bench-acceptance-run-ticket-07)
  — the credential record, the Pairing Token and the `BRIDGE_CONTROL` actions.
- [Browser provisioning round-trip (ticket 14)](../../bridge-firmware/README.md#browser-provisioning-round-trip-ticket-14)
  — the full portal and rejoin sequence.
- [Bridge Presence and the watchdog (ticket 08)](../../bridge-firmware/README.md#bridge-presence-and-the-watchdog-ticket-08)
  — presence, the RWDT and the bench-only stall hook.
- [The `link-probe.py` round trip and soak](../../bridge-firmware/README.md#the-link-probepy-round-trip-and-soak)
  — the probe's output and exit status.
- [Running the Robot Stub against it](../../bridge-firmware/README.md#running-the-robot-stub-against-it)
  — the serial side of the Bridge.
- [Troubleshooting](../../bridge-firmware/README.md#troubleshooting) — the
  failure symptoms and their fixes.

The bench tools and hardware tests have their own material:

- [Robot Stub README](../../remote/robot-stub/README.md) — personas, wiring and
  the `BRIDGE_CONTROL` request path.
- [Bridge Stub README](../../remote/bridge-stub/README.md) — the preamble
  scenarios, the Beacon scripts and the deny reasons.
- [remote-daemon README](../../remote/daemon/README.md) — the current-directory
  rule, the config store and the headless Bridge Stub run.
- [hardware-tests README](../../robot/crates/hardware-tests/README.md) — standalone
  sensor checks and their wiring.

## Common failures

- **`denied: unauthorized`, or `token=MISSING` on boot.** The probe's token
  differs from the one in the Bridge's record, or the record is not provisioned.
  Rotate or re-read the code on the panel, then retry; see
  [Troubleshooting](../../bridge-firmware/README.md#troubleshooting).
- **The listener is unreachable.** The Bridge logs `sta: address=…` only after
  DHCP; the listener exists on the station interface only, never on the setup
  AP.
- **The stub opens but the Bridge sees nothing.** Confirm the adapter is the
  `usb-1a86_…` entry, not the C3 console; confirm TX is crossed to RX, both ends
  are at 921600 8N1 and they share ground.
- **A connection test fails after the auth matrix.** The `ratelimit` check
  leaves a 60 s refusal window; reset the board or wait it out.
- **`dma_stalls` ticks once.** On a quiet bench UART that is the
  done-and-silent accounting, not lost data; read it with `uart_drop`, `crc` and
  a rising `good`.

## Source material

- [bridge-firmware README](../../bridge-firmware/README.md) — bench wiring, the
  power rules and the acceptance runs.
- [bridge-core README](../../bridge-firmware/bridge-core/README.md) — the
  host-testable pump rules.
- [robot-stub README](../../remote/robot-stub/README.md) — the robot persona on
  the bench.
- [bridge-stub README](../../remote/bridge-stub/README.md) — the daemon's bench
  peer.
- [hardware-tests README](../../robot/crates/hardware-tests/README.md) — standalone
  sensor checks and wiring.
- [remote-daemon README](../../remote/daemon/README.md) — the daemon the bench
  scripts exercise.
- [bridge-firmware/tools/](../../bridge-firmware/tools/) — `link-probe.py`,
  `auth-probe.py`, `beacon-watch.py`, `soak.py` and the shared preamble module.
- [CONTEXT.md](../../CONTEXT.md) — the glossary's Link Port, Bridge, Remote,
  Setup Mode and Pairing Token entries.

Owning decisions:

- [ADR-0033](../../docs/adr/0033-uart1-baud-and-ring-budget.md) — the 921600 8N1
  link and its budget.
- [ADR-0036](../../docs/adr/0036-wifi-provisioning.md) — Wi-Fi provisioning and
  the bounded Setup Mode AP.
- [ADR-0037](../../docs/adr/0037-reachability-and-trust.md) — the Discovery
  Beacon and the HMAC preamble.
- [ADR-0039](../../docs/adr/0039-remote-daemon-relay.md) — the daemon relay and
  its network defaults.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the workspaces and
  the current-directory rule.
- [ADR-0042](../../docs/adr/0042-bridge-telemetry.md) — Bridge Status on the
  bench.
