# bridge-firmware

The C3 (ESP32-C3) side of the Bridge: a `no_std` Embassy firmware that sits between the robot's
UART and the home network. This workspace targets `riscv32imc-unknown-none-elf` and keeps its own
`.cargo/config.toml`, so it stays out of the robot's and daemon's host workspaces (ADR-0040).

It currently holds two binaries:

| binary | purpose |
|---|---|
| `bridge` | the ticket-05 network edge: station-only Wi-Fi, UHCI RX DMA into the pump, the `bridge-net` preamble and single-client gate on a station-only TCP listener, and the Discovery Beacon |
| `stack-soak` | the ticket-09 language spike, kept as the regression soak: APSTA, a UART loopback and a TCP echo |

`bridge-core` is the host-testable pump crate the `bridge` binary wires to the HAL (see
[`bridge-core/README.md`](bridge-core/README.md)).

## Bench wiring

The C3's USB connector is the flashing and `defmt` console (`/dev/ttyACM0` on the bench). The
robot-facing link is **UART1** on two header pins:

| signal | C3 pin | connects to |
|---|---|---|
| UART1 TX | GPIO6 | the USB-TTL adapter's RX |
| UART1 RX | GPIO7 | the USB-TTL adapter's TX |
| ground | GND | the USB-TTL adapter's GND |

Use a **3.3 V** USB-TTL adapter; the C3's pins are not 5 V tolerant. The adapter appears as
`/dev/ttyUSB0` (or similar) and is where the Robot Stub runs. Do not point the stub at
`/dev/ttyACM0`: that is the C3's console, not the robot-facing UART.

For the `stack-soak` regression run, no adapter is needed: jumper **GPIO6 to GPIO7** so UART1
transmits into its own receiver. The same jumper is useful with `bridge`: a `link-probe.py`
`HELLO` is forwarded out of UART1, parsed as a valid robot-side frame on the way back, and sets
the Discovery Beacon's link-ready flag for five seconds (the router then ignores it, because a
robot-side `HELLO` without `IS_ACK` is not a legal robot frame).

Both directions run **921600 8N1**, the baud ADR-0033 fixes for the robot, the Bridge and the
daemon.

## Build-time credentials and the bench token

The `bridge` binary takes its station credentials and its ticket-05 bench pairing token from the
environment at compile time, so the image contains no secret at rest:

```sh
cd bridge-firmware
BRIDGE_STA_SSID='your-ssid' BRIDGE_STA_PASSWORD='your-passphrase' \
    BRIDGE_BENCH_TOKEN='your-bench-token' cargo build --release --bin bridge
```

With neither station variable set, or with either left at `CHANGE_ME`, the firmware still boots
and runs the UART, the pump and the Beacon task, but it logs a clear warning every minute and
never attempts to associate. The listener is useless without a station address.

`BRIDGE_BENCH_TOKEN` is a **temporary build-time stand-in** for ticket 07's credential record. Its
**UTF-8 bytes are the HMAC key**: the daemon stores the same text in its config (`default_token`
for a manual endpoint, or the per-device token) and passes the string's bytes, and the bench
probes read the text from the environment or a file. Ticket 07 will mint a 60-bit token displayed
as twelve base32 characters; those twelve characters' bytes are then the key, with no decoding
step. With no token compiled in, every client is denied and the firmware logs it.

`esp-radio` 1.0.0-beta.1 exposes only `Wpa2Personal`, so a WPA3-only or PMF-required access point
cannot be joined yet. The startup scan logs each BSS's advertised auth mode, which settles that
before any credentials are tried.

The bench flow puts the values on the build command itself, together with the flash:

```sh
cd bridge-firmware
touch src/bin/bridge.rs

BRIDGE_STA_SSID='your-ssid' \
BRIDGE_STA_PASSWORD='your-passphrase' \
BRIDGE_BENCH_TOKEN='bench05' \
cargo build --release --bin bridge && \
espflash flash --monitor --chip esp32c3 --port /dev/ttyACM0 --log-format defmt \
    target/riscv32imc-unknown-none-elf/release/bridge
```

The variables only need to reach `cargo build`; `espflash` uses the built image. The `touch`
forces a rebuild so a changed credential or token is definitely baked in. This leaves the values
in shell history; if that matters, use `read -r -s BRIDGE_STA_PASSWORD; export BRIDGE_STA_PASSWORD`
(or the same for the token) before the build instead. A later `cargo build` from a shell without
the variables silently rebuilds with `CHANGE_ME`/empty values, so check the boot banner's
`token=` field and the station warning.

## Flash and monitor

The repository's runner is `espflash flash --monitor --chip esp32c3 --log-format defmt`. The
end-to-end sequence, with the exact boot lines and probe output to expect, is
[Bench acceptance run](#bench-acceptance-run-ticket-05). The command above flashes and attaches
the monitor in one go; `Ctrl-C` ends the monitor and frees `/dev/ttyACM0`.

The monitor resets the board, so the boot banner and the association lines appear after it
attaches. It holds the serial port, so stop it before the next flash.

The console prints the boot banner, the association/scan lines, the station address once DHCP
completes, and a status line every ten seconds.

## Bench acceptance run (ticket 05)

This is the flow that passed on 2026-11-07: C3 at `192.168.0.138`, token `bench05`, the
GPIO6↔GPIO7 jumper installed, and a WPA2 station. Replace the address with the one the Bridge
logs; a DHCP lease can change it.

### Terminal 1 — build, flash and monitor

Stop any running monitor first (`Ctrl-C`) so `/dev/ttyACM0` is free.

```sh
cd bridge-firmware
touch src/bin/bridge.rs

BRIDGE_STA_SSID='your-ssid' \
BRIDGE_STA_PASSWORD='your-passphrase' \
BRIDGE_BENCH_TOKEN='bench05' \
cargo build --release --bin bridge && \
espflash flash --monitor --chip esp32c3 --port /dev/ttyACM0 --log-format defmt \
    target/riscv32imc-unknown-none-elf/release/bridge
```

Expect (the bench's values shown):

```text
[INFO ] bridge firmware: ticket 05 network edge, station-only, bridge-net preamble, Beacon every 3000ms; uart 921600 8N1 TX=GPIO6 RX=GPIO7
[INFO ] bridge identity: device_id=907069c1d064 name=Explorer-D064 token=configured
[INFO ] sta: station mode configured for ssid=…
[INFO ] tcp: listening on port 9000 (station interface only, three preamble workers)
[INFO ] beacon: broadcasting on UDP 9001 every 3000ms
[INFO ] sta: associated ssid=… channel=…
[INFO ] sta: address=192.168.0.138
[INFO ] status: … beacons=3 beacon_errors=0 link_ready=false
```

There must be **no** `executor: failed to spawn the TCP client …` lines, and `token=configured`,
not `token=MISSING`. The Beacon and the station come up with no client connected.

### Terminal 2 — the gate matrix

Put the token on each command (inline environment); no token file is needed. Replace the address
with the one from Terminal 1.

```sh
cd bridge-firmware

# 1. Baseline Beacon: needs no Link credentials.
python3 tools/beacon-watch.py --count 3

# 2. Link-ready through the jumper. Keep a watcher running to see the flag.
BRIDGE_BENCH_TOKEN='bench05' \
    python3 tools/link-probe.py 192.168.0.138 --loopback

# 3. The matrix. Run it last: it leaves a 60 s refusal window.
BRIDGE_BENCH_TOKEN='bench05' \
    python3 tools/auth-probe.py 192.168.0.138
```

Expected for step 1:

```text
beacon from=192.168.0.138 device=907069c1d064 name='Explorer-D064' major=1 tcp_port=9000 flags=0x00 (-)
```

Expected for step 2:

```text
link-probe: authenticated device=907069c1d064 name='Explorer-D064' flags=0x00
link-probe: HELLO sent to 192.168.0.138:9000 (epoch=0, rate=10Hz)
link-probe: done after 1.0s frames=0 hello_ack=no loopback=sent
link-probe: loopback HELLO sent; watch the Bridge status line …
```

The `HELLO` is ignored by the router on the way back, so `frames=0` and `hello_ack=no` are
expected. The proof is the next status line —
`uart_rx=52B uart_tx=52B … good=1 … link_ready=true` — and the Beacon watcher showing
`flags=0x02 (link-ready)` for one or two datagrams, then back to `0x00` after the 5 s window.

Expected for step 3:

```text
auth-probe: valid
  accepted device=907069c1d064 name='Explorer-D064' flags=0x00
  PASS
auth-probe: busy
  second client refused: denied: busy, retry_ms=4000
  PASS
auth-probe: silent
  dropped after 5.1s
  PASS
auth-probe: supersede
  new client 907069c1d064 authenticated in 0.04s while a silent one was pending
  PASS
auth-probe: wrong
  denied denied: unauthorized
  PASS
auth-probe: ratelimit
  refusal window opened after 5 failures: denied: unauthorized, retry_ms=60000
  PASS
auth-probe: all checks passed
```

The Bridge counters corroborate it: `auth_ok` rises, `auth_bad` and `auth_expired` move with the
matrix, and `deny_busy` / `deny_refused` are non-zero. The `valid` line's `flags=0x02` instead of
`0x00` if the matrix starts inside the loopback's 5 s link-ready window.

### Notes

- `dma_stalls` may tick once in step 2 because the UART is quiet between tests. Read it with
  `uart_drop`, `crc`, `bad` and the rising `good`/`uart_rx`: the smooth run was
  `uart_rx=52B uart_tx=52B good=1 uart_drop=0 crc=0`. A tick after a quiet line is the ticket-04
  "done-and-silent for 1 s" accounting, not lost data.
- The matrix's last check leaves a 60 s refusal window. Reset the board (or wait) before another
  connection test; a post-window attempt answers `DENY(unauthorized, retry_ms=60000)`.
- `token=MISSING` on the boot banner means the token was not exported for that `cargo build`.
  `preamble failed: denied: unauthorized` in a probe means the probe's token differs from the one
  compiled into the image.

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

The stub prints its peer state every five seconds. The Bridge's status line is the other half of
the observation.

## The network edge (ticket 05)

The listener runs the `bridge-net` preamble before a single Link Protocol byte crosses (ADR-0037):
`CLIENT_HELLO` → the Bridge proves the token → `CLIENT_AUTH` → `ACCEPT`, all inside 5 s. A wrong
proof is `DENY(unauthorized)`; a second authenticated client is `DENY(busy)` with `retry_ms=4000`;
five failures inside a minute close a one-minute refusal window. Several connection workers run
at once, so an unauthenticated connection is superseded and never blocks a new one.

The Bridge also broadcasts the Discovery Beacon every 3 s on the station interface, on UDP 9001:

| flag | source in ticket 05 |
|---|---|
| `provisioned` | false; ticket 07 adds the credential record |
| `link-ready` | a valid Link Protocol frame from the robot in the last 5 s (`beacon::LINK_READY_WINDOW_MS`) |
| `setup_mode` | false; station-only until ticket 06 adds APSTA |
| `tcp_port` | the Link listener's port (9000); portal-only zero belongs to ticket 06 |

The identity served in the challenge is the same one the Beacon advertises: the efuse station
MAC as the device id, and `Explorer-XXXX` (the MAC's last two bytes) as the name.

`tools/beacon-watch.py` listens on UDP 9001 and prints each decoded Beacon with its flags. Run it
while `link-probe.py --loopback` sends a `HELLO` to see `link-ready` flip:

```sh
python3 tools/beacon-watch.py --count 3
```

### The authentication matrix

`tools/auth-probe.py` runs the bench matrix with a dependency-free `bridge-net` client
(`tools/bridge_preamble.py`). Put the token on the command line, never in argv as a flag:

```sh
BRIDGE_BENCH_TOKEN='bench05' python3 tools/auth-probe.py 192.168.1.42
BRIDGE_BENCH_TOKEN='bench05' python3 tools/auth-probe.py 192.168.1.42 --mode busy
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

The pairing token comes from `BRIDGE_BENCH_TOKEN` (its UTF-8 bytes are the HMAC key); set it
inline on the command or export it. It is never a command-line flag, so it does not land in the
process arguments.

```sh
# One HELLO/HELLO_ACK round trip, then a five-second idle timeout.
BRIDGE_BENCH_TOKEN='bench05' python3 tools/link-probe.py 192.168.1.42

# A sixty-second observation of the baseline or worst-case persona.
BRIDGE_BENCH_TOKEN='bench05' python3 tools/link-probe.py 192.168.1.42 --duration 60 --quiet

# Stop after one hundred decoded frames.
BRIDGE_BENCH_TOKEN='bench05' python3 tools/link-probe.py 192.168.1.42 --count 100 --quiet

# No robot: GPIO6 and GPIO7 jumpered together instead of wired to a USB-TTL adapter.
BRIDGE_BENCH_TOKEN='bench05' python3 tools/link-probe.py 192.168.1.42 --loopback
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

The ticket-04 acceptance observation is a worst-case stub run with the probe counting frames and
the Bridge status line showing `uart_drop=0 dma_stalls=0 crc=0` throughout.

## Status line

The `bridge` binary prints one line every ten seconds:

```
status: t=…s rssi=…dBm uart_rx=…B uart_tx=…B uart_kbps=…
        uart_drop=…B dma_stalls=… crc=… resync=… good=… bad=…
        tcp_rx=…B tcp_tx=…B tcp_kbps=… tcp_accepts=… tcp_tx_errors=…
        tcp_read_errors=… client_crc=… client_resync=…
        ring_used=…B ring_reserved=…B drop_stream=… drop_no_client=…
        drop_reliable=… drop_bridge_control=… drop_on_close=…
        ring_encode_errors=… tx_q_drop=… reconnects=…
        auth_ok=… auth_bad=… auth_expired=… auth_superseded=… auth_failed=…
        deny_busy=… deny_refused=… deny_version=…
        beacons=… beacon_errors=… link_ready=…
```

| field | meaning |
|---|---|
| `uart_rx` / `uart_tx` | bytes drained from the UHCI RX DMA / written by the async UART TX task |
| `uart_drop` | bytes the parser discarded while resynchronising (a bad CRC or an out-of-range length) |
| `dma_stalls` | times the UHCI transfer reported done with no data for 1 s; a quiet bench UART can tick this once, so read it with `uart_drop`/`crc`/`good` |
| `crc` / `resync` | rejected candidates for a bad CRC / every rejected candidate |
| `good` / `bad` | valid robot-to-PC frames / candidate rejections |
| `tcp_rx` / `tcp_tx` | bytes read from / written to the client |
| `ring_used` / `ring_reserved` | the pump's coalescing-ring occupancy and the event reservation |
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

`uart_drop=0` on a sustained run is the ticket-04 "zero loss" observation.

## The `stack-soak` regression soak

`stack-soak` is the original ticket-09 spike. It brings up **APSTA**, transmits a known frame
pattern on UART1 (jumper GPIO6 to GPIO7), echoes TCP bytes on port 9000 on both interfaces, and
forces station disconnects after ten minutes.

```sh
cd bridge-firmware
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

From `bridge-firmware/`:

```sh
cargo build --release
cargo clippy -- -D warnings
cargo +nightly fmt --all -- --check
cargo test -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests
```

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

- **`espflash` cannot open the port.** Check that `/dev/ttyACM0` exists, that the user is in the
  `dialout` (Debian/Ubuntu) or `uucp` (Arch) group, and that no monitor is holding the port.
- **The board does not enter the bootloader.** Hold **BOOT**, tap **RESET**, release **BOOT**, then
  flash.
- **Multiple boards.** `espflash` picks the first matching port; pass `--port` explicitly and keep
  `--chip esp32c3`.
- **The stub opens but the Bridge sees nothing.** Confirm the adapter is on `/dev/ttyUSB*`, not
  `/dev/ttyACM0`; confirm TX↔RX are crossed (GPIO6 to the adapter's RX, GPIO7 to its TX); confirm
  both ends are at 921600 8N1 and share ground.
- **`dma_stalls` rises.** On a quiet bench line the ticket-04 "done-and-silent for 1 s" rule
ticks once; if `uart_drop=0`, `crc=0` and `good` rises, nothing was lost. Under real robot load,
suspect the Wi-Fi stack holding the DMA task longer than the descriptor ring can absorb. Check the
heap size and the task priorities before suspecting the UART wiring.
- **The listener is unreachable.** The Bridge logs `sta: address=…` only once DHCP completes; the
  listener exists on the station interface only, never on an AP interface.
- **Association fails.** The startup scan logs the BSS's auth mode. The beta radio cannot do WPA3
  or PMF; use a WPA2 access point for now.
