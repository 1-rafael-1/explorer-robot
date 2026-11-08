# Bridge

The Bridge firmware: the ESP32-C3 `no_std` image that translates between the home
network and the robot's UART1 Link Port, and acts as the network edge. It broadcasts
the Discovery Beacon, authenticates the one Remote against the Pairing Token, and
raises Setup Mode when it has no usable credentials. The glossary's
[Remote Link vocabulary](../../CONTEXT.md#remote-link) defines the Bridge, the
Link Port, Setup Mode, the Credential Record and the Pairing Token.

There is no packaged Bridge: no published binary, OTA update, tag or release
artifact. You build the image from source and write it over USB with `espflash`.
The Wi-Fi credentials and Pairing Token are device state on the `bridge_data`
partition, not repository files.

**Verified:** 2026-11-08 on Linux x86_64 with stable Rust 1.99.1 and espflash
4.1.0. From `bridge/`: `cargo build` (dev profile, cached, 0.48 s);
`cargo test -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests`
(176 tests in 12 suites, 0 failed); `cargo clippy -- -D warnings` and
`cargo clippy -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests -- -D warnings`
(clean); `cargo +nightly fmt --all -- --check` (clean); and
`espflash partition-table partitions.csv` (four partitions). From the repository
root: `python3 .github/scripts/check_links.py` (exit 0). Flashing, the monitor,
bench wiring, first boot, Setup Mode and the portal were **not** run — they are
**bench-required**.

## Prerequisites

All commands below start at the repository root and enter `bridge/`; run
the workspace commands from there. Cargo reads `.cargo/config.toml` from the
current directory upward, and this workspace owns its target, `build-std` and
flash runner; the repository root does not.

- **Stable Rust, the RISC-V target and `rust-src`.** The workspace pins them in
  [`rust-toolchain.toml`](../../bridge/rust-toolchain.toml): channel
  `stable`, component `rust-src`, target `riscv32imc-unknown-none-elf`. `rustup`
  installs them when you enter the directory. If a component is missing:

  ```sh
  rustup component add rust-src
  rustup target add riscv32imc-unknown-none-elf
  ```

- **`build-std`.** [`bridge/.cargo/config.toml`](../../bridge/.cargo/config.toml)
  sets `[unstable] build-std = ["alloc", "core"]` and the target, so no `-Z
  build-std` or `--target` flag is needed by hand. `rust-src` supplies those
  sources; the first clean build compiles them and takes several minutes.

- **`espflash` on `PATH`.** The Bridge README's flash and monitor commands assume
  the upstream tool is already installed; the repository does not pin a version.
  Install it with `cargo install espflash`, or use the esp-rs prebuilt release.
  The bench used 4.1.0. Put your user in the `dialout` (Debian/Ubuntu) or `uucp` (Arch)
  group so the C3 console is writable.

- **A USB cable to the C3 console** for flashing and the `defmt` console. Identify
  it by its `usb-Espressif_USB_JTAG_serial_debug_unit_…` entry under
  `/dev/serial/by-id`; the port may be `/dev/ttyACM0`, `/dev/ttyACM1`, or another
  `ACM` index. A **3.3 V** USB-TTL adapter is only needed for the robot-facing
  UART1 link — see [Bench wiring](#bench-wiring).

## Build and host tests

The device workspace build, from `bridge/`:

```sh
cd bridge
cargo build
```

Expected output on a warm tree:

```text
   Compiling bridge-firmware v0.1.0 (/path/to/bridge)
    Finished `dev` profile [optimized + debuginfo] target(s) in 0.48s
```

A fresh clone prints one `Compiling` line per dependency (and for the `core` and
`alloc` sources `build-std` compiles), then the same `Finished` line. The dev
profile is `opt-level = "s"`; the build produces both binaries under
`target/riscv32imc-unknown-none-elf/debug/`: `bridge` and `stack-soak`.

The host tests exercise the HAL-free pump and provisioning rules in
`bridge-core`. The explicit host target keeps host and firmware artifacts apart;
the `std` feature is what makes the crate testable on `x86_64`.

```sh
cd bridge
cargo test -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests
```

Expected output is twelve suite runs, each ending `test result: ok`:

```text
     Running tests/auth.rs (target/x86_64-unknown-linux-gnu/debug/deps/auth-…)
test result: ok. 10 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
…
     Running tests/watermark.rs (target/x86_64-unknown-linux-gnu/debug/deps/watermark-…)
test result: ok. 9 passed; 0 failed; 0 ignored; 0 measured; 0 filtered out; finished in 0.00s
```

The suites and their counts are `auth` (10), `coalescing` (10), `dhcp` (15),
`dns` (11), `heartbeat` (11), `pairing` (7), `portal` (39), `pump` (11), `record`
(12), `routing` (18), `setup` (23) and `watermark` (9): **176 passed, 0 failed**.
The rules themselves are described in the
[bridge-core README](../../bridge/bridge-core/README.md); this page does
not restate them.

The Bridge CI job also gates clippy and formatting. These are the exact gates from
[`.github/workflows/ci.yml`](../../.github/workflows/ci.yml):

```sh
cd bridge
cargo clippy -- -D warnings
cargo clippy -p bridge-core --features std --target x86_64-unknown-linux-gnu --tests -- -D warnings
cargo +nightly fmt --all -- --check
```

Both clippy commands finish with `Finished` and no `warning:` lines. The first is
deliberately the bin target only: a `no_std` binary cannot build the built-in test
harness, so `--all-targets` would fail with a missing `test` crate and
`panic_handler`. The second applies the workspace's pedantic/nursery lint table to
`bridge-core` on the host.

## Flash and the partition table

[`partitions.csv`](../../bridge/partitions.csv) is the ESP-IDF table the
firmware expects:

| partition | type | subtype | offset | size |
|---|---|---|---:|---:|
| `nvs` | `data` | `nvs` | `0x9000` | 24 KiB |
| `phy_init` | `data` | `phy` | `0xF000` | 4 KiB |
| `factory` | `app` | `factory` | `0x10000` | 3 MiB |
| `bridge_data` | `data` | `undefined` (`0x06`) | `0x310000` | 16 KiB |

`bridge_data` is deliberately subtype `undefined`, not a custom `0x40` subtype:
`espflash` and `esp-bootloader-esp-idf` reject `0x40` at parse or lookup time. The
runtime finds the partition by its **label**, so moving it in the CSV is a one-line
change — and loses the record.

`espflash flash` writes an **explicit set of three sections**:

1. the bootloader;
2. this partition table, at `--partition-table-offset 0x8000`;
3. the `factory` app image.

The credential record in `bridge_data` is **outside** that set, as are `nvs` and
`phy_init`. A normal firmware update therefore leaves the Wi-Fi credentials and
the Pairing Token intact.

`espflash partition-table partitions.csv` parses the table without a board
attached. It prints the four partitions with the sizes above; this step was run
for the `Verified:` line.

### Flash and monitor — bench-required

The invocation used by the [Bridge README](../../bridge/README.md#flash-and-monitor):

```sh
cd bridge
touch src/bin/bridge.rs
cargo build --release --bin bridge && \
espflash flash --monitor --chip esp32c3 --port /dev/ttyACM1 --log-format defmt \
    --partition-table partitions.csv --partition-table-offset 0x8000 \
    target/riscv32imc-unknown-none-elf/release/bridge
```

The build half runs anywhere; flashing and monitoring require the board and are
**bench-required**. Replace `/dev/ttyACM1` with the C3 console from
`/dev/serial/by-id`.

- `touch src/bin/bridge.rs` forces a changed build so `espflash` writes and
  resets. If every section reports `Skipped! (checksum matches)`, it does not
  reset the chip: run `espflash reset`, or change the build, before testing.
- `cargo run --bin bridge` is the same explicit set: the runner in
  [`bridge/.cargo/config.toml`](../../bridge/.cargo/config.toml)
  passes the custom partition table. It is also **bench-required**.
- The monitor resets the board when it attaches and holds the serial port. Stop it
  with `Ctrl-C` before the next flash. Expect the boot banner, the
  association/scan lines, `sta: address=…` once DHCP completes, and a status line
  every ten seconds.
- The exact boot lines and probe output for the network edge, Setup Mode and the
  record/token are in the README's
  [ticket 05](../../bridge/README.md#bench-acceptance-run-ticket-05),
  [ticket 06](../../bridge/README.md#bench-acceptance-run-ticket-06) and
  [ticket 07](../../bridge/README.md#bench-acceptance-run-ticket-07)
  acceptance runs. The probes live in
  [`bridge/tools/`](../../bridge/tools/): `link-probe.py`,
  `auth-probe.py`, `beacon-watch.py`, `soak.py` and the shared preamble module.

## The credential record, first boot and Setup Mode

All of this section is **bench-required** to exercise; it describes what the
firmware does.

**What is stored and where.** One versioned, CRC-checked record on the dedicated
`bridge_data` partition:

```text
magic, version, flags, ssid_len, ssid, pass_len, passphrase, token[12], crc16
```

`bridge-core::record` owns the layout and the CRC (the Link Protocol's
CRC-16/CCITT-FALSE), so the codec is host-tested; the firmware owns the
`sequential-storage` map over `esp-storage`. The magic is `BRID`, the version is
1, and the twelve token characters are the HMAC key verbatim. The alphabet omits
`I`, `L`, `O` and `U` so the Pairing Token is unambiguous when read from the
panel. See
[ADR-0036](../../docs/adr/0036-wifi-provisioning.md) and the
[bridge-core README](../../bridge/bridge-core/README.md).

**First boot.** A blank or erased `bridge_data` boots **unprovisioned**. The
Bridge mints a 60-bit Pairing Token from the hardware TRNG (`TrngSource` plus
`Trng`; a bare `Rng` is not cryptographic before the radio is up) and stores a
token-only record. The banner prints it as
`pairing_code=XXXX-XXXX-XXXX`. Only a record with a station credential pair sets
the Discovery Beacon's `provisioned` bit. A missing record, wrong magic, wrong
version, bad CRC, bad flags, bad credential lengths or a token outside the
alphabet all mean unprovisioned — the Bridge fails closed into Setup Mode instead
of misreading a changed layout.

**Setup Mode.** The bounded provisioning state from
[ADR-0036](../../docs/adr/0036-wifi-provisioning.md): a WPA2-PSK SoftAP
`ExplorerRobot-XXXX` (uppercase hex of the AP interface MAC's last two bytes),
password `explorer-setup`, at most two clients, on `192.168.4.1/24` with a
two-address DHCP pool, a DNS catch-all and a one-page HTTP portal. It is raised
by an unprovisioned boot, once per boot after five continuous minutes without a
station link, or by the robot panel's `BRIDGE_CONTROL` request. The window is ten
minutes and closes about 30 seconds after a stable station association. It is
never permanent.

On this `esp-radio` beta, `scan_async` returns `Other` in AP-only mode, so the
unprovisioned window runs APSTA with a parked station config that never connects;
the portal is still the only served surface. The Link Protocol listener binds to
the station interface only, never to the setup AP.

**Operating it.** Join `ExplorerRobot-XXXX` with `explorer-setup`, open
`http://192.168.4.1/`, choose the home 2.4 GHz SSID, enter the passphrase and
submit. The page shows its status (`unprovisioned` / `connecting` / `wrong
password` / `connected to X at 192.168.…`) and offers **Forget**. The setup
password is public by design; trust is the Pairing Token's job
([ADR-0037](../../docs/adr/0037-reachability-and-trust.md)).

**What survives what.**

| action | Wi-Fi credentials | Pairing Token |
|---|---|---|
| reset or power cycle | survive | survive |
| `espflash flash` (bootloader + table + app) | survive | survive |
| `espflash erase-parts … bridge_data` | lost | lost; a new one is minted |
| `espflash erase-flash` | lost | lost; a new one is minted |
| moving `bridge_data` in `partitions.csv` | lost | lost; a new one is minted |
| portal **Forget network** | cleared | survives |
| panel `BRIDGE_CONTROL` action 0 | cleared | rotated to a new one |
| panel `BRIDGE_CONTROL` action 2 | survive | rotated to a new one |

A fresh boot with a valid record loads and prints the existing token; it mints
only when the partition is blank or the record is unusable.

## Recovery

All commands here are **bench-required**.

**Erase only the credential record** (keeps the app and bootloader), then reset:

```sh
cd bridge
espflash erase-parts --chip esp32c3 --port /dev/ttyACM1 \
    --partition-table partitions.csv --after no-reset bridge_data
espflash reset --chip esp32c3 --port /dev/ttyACM1
```

The boot is now unprovisioned: expect
`record: loaded (provisioned=false pairing_code=XXXX-XXXX-XXXX)` with a **new**
token, and a Setup Mode window. The old token is invalid immediately.

**Erase the whole chip and reflash:**

```sh
cd bridge
espflash erase-flash --chip esp32c3 --port /dev/ttyACM1
cargo build --release --bin bridge
espflash flash --chip esp32c3 --port /dev/ttyACM1 --log-format defmt \
    --partition-table partitions.csv --partition-table-offset 0x8000 \
    target/riscv32imc-unknown-none-elf/release/bridge
```

The blank `bridge_data` mints afresh on the next boot. The same happens if you
move `bridge_data` in `partitions.csv`: the runtime finds it by label, so its old
contents are ignored.

**The resulting new Pairing Token** must be entered into the Remote again. The
daemon stores the token locally; the old one fails the mutual HMAC-SHA256
challenge and the Bridge denies the client. To rotate deliberately without a
reflash, use the panel's `BRIDGE_CONTROL` action 2 (rotate) or action 0 (forget
Wi-Fi and rotate); the portal's **Forget network** clears the credentials but
keeps the token. The token is never sent on the wire, so recovery is always
"read the new code from the panel/portal and retype it".

If a normal flash unexpectedly re-provisioned the Bridge, it was flashed without
the custom table, or the table moved `bridge_data`; a plain `espflash flash` writes
the bootloader, table and app only. The boot log must show
`record: partition bridge_data at 0x310000`.

## Bench wiring

The C3's USB connector is the flashing and `defmt` console. The robot-facing link
is **UART1** on the C3 SuperMini harness pins. The full rules — the RST button's
outer leg, the 3.3 V limit, the unconnected BOOT strap — are in the
[Bridge README](../../bridge/README.md#bench-wiring); this is the signal
map:

| signal | C3 SuperMini | connects to |
|---|---|---|
| UART1 TX | GPIO4 | the USB-TTL adapter's RX, or the robot's `EXP_RX` (GPIO37 on J10) |
| UART1 RX | GPIO3 | the USB-TTL adapter's TX, or the robot's `EXP_TX` (GPIO36 on J10) |
| ground | GND | the adapter's GND / robot `J9.2` |
| Bridge Presence | GPIO0 | the robot's `EXP_IRQ` (GPIO33 on J10) |
| Bridge reset | `EN` (RST button's **outer** leg) | the robot's `EXP_EN` (GPIO34 on J10) |
| spare | GPIO1 | the robot's `EXP_BOOT` (GPIO35 on J10); reserved input |
| power | 5V | the robot's `EXP_5V` (`J9.1`) in the sealed build; USB on the bench |

- Use a **3.3 V** USB-TTL adapter; the C3's pins are not 5 V tolerant. Identify
  it by its `usb-1a86_…` entry under `/dev/serial/by-id`, not the C3's
  `usb-Espressif_…` console. Never point the Robot Stub at the console entry.
- Both directions run **921600 8N1**, the baud
  [ADR-0033](../../docs/adr/0033-uart1-baud-and-ring-budget.md) fixes for the
  robot, the Bridge and the daemon.
- Do not connect the C3's 3V3 pin to the robot; each side regulates its own
  3.3 V.
- Loopback for a bench run: for `bridge`, jumper **GPIO3 to GPIO4**; for
  `stack-soak`, jumper **GPIO6 to GPIO7** (that binary keeps the default pins).
  A `link-probe.py` `HELLO` on the GPIO3/4 loopback is forwarded out of UART1,
  parsed back as a valid robot-side frame, and sets the Discovery Beacon's
  link-ready flag for five seconds.

## The `stack-soak` knobs (`SPIKE_*`)

`SPIKE_*` are compile-time `option_env!` overrides for the **`stack-soak`**
regression binary only. The production `bridge` binary does not read them; its
network configuration comes from the credential record and the portal. They are
set on the command line at build time, so touch the source when changing one:

```sh
cd bridge
touch src/bin/stack-soak.rs
SPIKE_STA_SSID='your-ssid' SPIKE_STA_PASSWORD='your-passphrase' \
    cargo run --release --bin stack-soak
```

| knob | default | effect |
|---|---|---|
| `SPIKE_STA_SSID` | `CHANGE_ME` | station SSID; the default leaves the station unassociated |
| `SPIKE_STA_PASSWORD` | `CHANGE_ME` | station passphrase |
| `SPIKE_UART_BAUD` | `921600` | UART1 baud for a control run |
| `SPIKE_RX_THRESHOLD` | `32` | UART RX FIFO threshold |
| `SPIKE_UART_RX_ASYNC` | unset (DMA) | set it to select the async reader instead of UHCI RX DMA |
| `SPIKE_DMA_BUFFER` | `4096` | UHCI RX DMA buffer size |
| `SPIKE_DMA_CHUNK` | `128` | DMA descriptor chunk size |
| `SPIKE_UART_FRAME_INTERVAL_MS` | `0` | delay between transmitted test frames |
| `SPIKE_SOAK_SECS` | `600` | run time before a forced reconnect |
| `SPIKE_RECONNECT_SECS` | `120` | reconnect period |

The source of truth is
[`bridge/src/bin/stack-soak.rs`](../../bridge/src/bin/stack-soak.rs);
the bench procedure is in the README's
[`stack-soak` regression soak](../../bridge/README.md#the-stack-soak-regression-soak).
Running the soak is **bench-required**.

## Common failures

- **`espflash` cannot open the port.** Confirm the C3 console entry exists, that
  you are in `dialout`/`uucp`, and that no monitor still holds the port.
- **The board does not enter the bootloader.** Hold **BOOT**, tap **RESET**,
  release **BOOT**, then flash.
- **Multiple boards.** Pass `--port` explicitly and keep `--chip esp32c3`.
- **Every section is `Skipped! (checksum matches)`.** The chip was not reset.
  Run `espflash reset` or rebuild with a changed input; otherwise a Setup Mode
  window may already be running.
- **Record lines are absent or say `no data partition`.** The board was flashed
  without the custom table. Reflash with `--partition-table partitions.csv
  --partition-table-offset 0x8000`; the boot log must show
  `record: partition bridge_data at 0x310000`.
- **Association fails.** The startup scan logs each BSS's auth mode. This
  `esp-radio` beta cannot do WPA3 or PMF; use a WPA2 access point.
- **The listener is unreachable.** The Bridge logs `sta: address=…` only once
  DHCP completes, and the listener binds to the station interface only.

The [Bridge README's troubleshooting](../../bridge/README.md#troubleshooting)
covers the full list, including DMA stalls and UART RX recovery.

## Source material

- [Bridge README](../../bridge/README.md) — bench wiring, the
  credential record, the partition table, flash and monitor, and the acceptance
  runs.
- [bridge-core README](../../bridge/bridge-core/README.md) — the
  host-testable pump rules, the Setup Mode codecs and the record/token codec.
- [bridge-net README](../../remote/bridge-net/README.md) — the shared Discovery
  Beacon, preamble and ports.
- [link-protocol README](../../robot/crates/link/link-protocol/README.md) — the wire contract
  the Bridge parses and forwards.
- [CONTEXT.md](../../CONTEXT.md) — the glossary's Bridge, Link Port, Setup Mode
  and Pairing Token entries.

Owning decisions:

- [ADR-0036](../../docs/adr/0036-wifi-provisioning.md) — Wi-Fi provisioning,
  the credential record and Setup Mode.
- [ADR-0037](../../docs/adr/0037-reachability-and-trust.md) — the Discovery
  Beacon, the Pairing Token and the HMAC preamble.
- [ADR-0040](../../docs/adr/0040-repo-and-build-layout.md) — the three
  workspaces and the current-directory rule.
- [ADR-0041](../../docs/adr/0041-reliability-mechanisms-in-protocol.md) —
  reliability mechanisms in the shared protocol crate.
- [ADR-0042](../../docs/adr/0042-bridge-telemetry.md) — Bridge telemetry and
  Bridge Status.
