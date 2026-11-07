#!/usr/bin/env sh
# Build and flash the Bridge firmware with the station credentials compiled in.
#
# The credentials are compiled into the image, so they are read from the environment or from
# ~/.config/explorer-bench.env (mode 0600), which may contain:
#
#   BRIDGE_STA_SSID=...
#   BRIDGE_STA_PASSWORD=...
#
# Usage:
#   tools/bench-flash.sh [serial-port]      # default /dev/ttyACM0
#
# The firmware is left running. Attach a monitor to read the logs:
#   espflash monitor --chip esp32c3 --log-format defmt --elf \
#       target/riscv32imc-unknown-none-elf/release/bridge
set -eu

cd "$(dirname "$0")/.."

if [ -f "${HOME}/.config/explorer-bench.env" ]; then
    # shellcheck disable=SC1091
    . "${HOME}/.config/explorer-bench.env"
fi

: "${BRIDGE_STA_SSID:?set BRIDGE_STA_SSID or create ~/.config/explorer-bench.env}"
: "${BRIDGE_STA_PASSWORD:?set BRIDGE_STA_PASSWORD or create ~/.config/explorer-bench.env}"
export BRIDGE_STA_SSID BRIDGE_STA_PASSWORD

port="${1:-/dev/ttyACM0}"

cargo build --release --bin bridge
espflash flash --chip esp32c3 --port "${port}" \
    target/riscv32imc-unknown-none-elf/release/bridge
