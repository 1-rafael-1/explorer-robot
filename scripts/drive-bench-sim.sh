#!/bin/sh
# Build and run the permanent drive-intent simulation on the board.
#
# `bench_drive_sim` runs the compiled `drive-core` intents with scripted
# synthetic observations and a recording action executor. It issues no motor
# commands and touches no sensors, so this script is safe with the motors
# free-standing and the rangefinders covered. The Basic Motor Test remains the
# gate for the real motor and encoder path.
set -eu

BIN=target/thumbv8m.main-none-eabihf/release/explorer-robot
LOG=/tmp/drive-bench-sim.log
FINAL=/tmp/drive-bench-sim-final.log

cargo build -p explorer-robot --release --features bench_drive_sim

# probe-rs runs until the target stops; a timeout exit of 124 is expected.
if timeout -k 5 30 probe-rs run --chip RP235x --protocol swd --speed 16000 \
    "$BIN" > "$LOG" 2>&1; then
    :
else
    status=$?
    if [ "$status" -ne 124 ]; then
        echo "probe-rs run (drive-sim) failed with status $status" >&2
        exit 1
    fi
fi

grep 'drive-sim:' "$LOG" || true

# Fail on the first scenario failure, including the final FAIL line.
if grep -q 'drive-sim: .* FAIL' "$LOG"; then
    echo "drive-sim: a scenario failed; see $LOG" >&2
    exit 1
fi

# probe-rs appends its own shutdown lines after the firmware's last log, so keep
# only the final result line before deciding the outcome.
grep 'drive-sim: PASS' "$LOG" > "$FINAL" || true

if tail -n 1 "$FINAL" | grep -q 'drive-sim: PASS'; then
    exit 0
fi

echo "drive-sim: no final PASS line at the end of $LOG" >&2
exit 1
