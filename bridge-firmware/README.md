# bridge-firmware

The C3 (ESP32-C3) side of the Bridge: a `no_std` Embassy firmware that sits between the robot's
UART and the home network. This workspace targets `riscv32imc-unknown-none-elf` and keeps its own
`.cargo/config.toml`, so it stays out of the robot's and daemon's host workspaces (ADR-0040).

It currently holds two binaries:

| binary | purpose |
|---|---|
| `bridge` | the ticket-04 skeleton: station-only Wi-Fi, UHCI RX DMA into the pump, one raw TCP listener on the station interface, no auth and no Beacon |
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
transmits into its own receiver.

Both directions run **921600 8N1**, the baud ADR-0033 fixes for the robot, the Bridge and the
daemon.

## Build-time credentials

The `bridge` binary takes its station credentials from the environment at compile time, so the
image contains no secret at rest:

```sh
cd bridge-firmware
BRIDGE_STA_SSID='your-ssid' BRIDGE_STA_PASSWORD='your-passphrase' \
    cargo build --release --bin bridge
```

With neither variable set, or with either left at `CHANGE_ME`, the firmware still boots and runs
the UART and the pump, but it logs a clear warning every minute and never attempts to associate.
The listener is useless without a station address; set both variables and reflash.

`esp-radio` 1.0.0-beta.1 exposes only `Wpa2Personal`, so a WPA3-only or PMF-required access point
cannot be joined yet. The startup scan logs each BSS's advertised auth mode, which settles that
before any credentials are tried.

## Flash and monitor

The repository's runner is `espflash flash --monitor --chip esp32c3 --log-format defmt`, so:

```sh
cd bridge-firmware
BRIDGE_STA_SSID='your-ssid' BRIDGE_STA_PASSWORD='your-passphrase' \
    cargo run --release --bin bridge
```

Equivalently, build and flash the ELF explicitly:

```sh
cargo build --release --bin bridge
espflash flash --monitor --chip esp32c3 \
    target/riscv32imc-unknown-none-elf/release/bridge
```

The console prints the boot banner, the association/scan lines, the station address once DHCP
completes, and a status line every ten seconds.

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

## The `link-probe.py` round trip and soak

`tools/link-probe.py` is the dependency-free TCP client for the bench. It connects to the
Bridge's listener, sends a valid `HELLO` (control id 1, with one enabled Requested Rate per
stream) and decodes every robot frame it receives.

```sh
# One HELLO/HELLO_ACK round trip, then a five-second idle timeout.
python3 tools/link-probe.py 192.168.1.42

# A sixty-second observation of the baseline or worst-case persona.
python3 tools/link-probe.py 192.168.1.42 --duration 60 --quiet

# Stop after one hundred decoded frames.
python3 tools/link-probe.py 192.168.1.42 --count 100 --quiet
```

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
```

| field | meaning |
|---|---|
| `uart_rx` / `uart_tx` | bytes drained from the UHCI RX DMA / written by the async UART TX task |
| `uart_drop` | bytes the parser discarded while resynchronising (a bad CRC or an out-of-range length) |
| `dma_stalls` | times the UHCI descriptor ring was exhausted and DMA stopped; the loss indicator |
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

The probe's syntax check needs no dependencies:

```sh
python3 -m py_compile tools/link-probe.py
ruff check tools/link-probe.py   # when ruff is installed
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
- **`dma_stalls` rises.** The Wi-Fi stack is holding the DMA task longer than the descriptor ring
  can absorb. Check the heap size and the task priorities before suspecting the UART wiring.
- **The listener is unreachable.** The Bridge logs `sta: address=…` only once DHCP completes; the
  listener exists on the station interface only, never on an AP interface.
- **Association fails.** The startup scan logs the BSS's auth mode. The beta radio cannot do WPA3
  or PMF; use a WPA2 access point for now.
