#!/bin/sh
# Build and run the permanent two-phase motor bench on the board.
#
# Phase 1 (`bench_motor_calibrate`) dispatches the Motor Calibration and holds on
# its Result Report. Phase 2 (`bench_motor_measure`) boots fresh, loads the saved
# v4 curve, and measures corrected left/right divergence from the menu. The two
# probe-rs sessions never overlap, so no report is on screen while the precision
# loop drives the motors.
set -eu

BIN=target/thumbv8m.main-none-eabihf/release/explorer-robot

# ── Phase 1: calibrate ────────────────────────────────────────────────────────
cargo build -p explorer-robot --release --features bench_motor_calibrate

# probe-rs runs until the target stops; a timeout exit of 124 is expected.
if timeout -k 5 180 probe-rs run --chip RP235x --protocol swd --speed 16000 \
    "$BIN" > /tmp/motor-bench-calibrate.log 2>&1; then
    :
else
    status=$?
    if [ "$status" -ne 124 ]; then
        echo "probe-rs run (calibrate) failed with status $status" >&2
        exit 1
    fi
fi

grep 'bench-calibrate:' /tmp/motor-bench-calibrate.log || true

if ! grep -q 'bench-calibrate: calibration saved' /tmp/motor-bench-calibrate.log; then
    echo "bench-calibrate: no saved line in /tmp/motor-bench-calibrate.log" >&2
    exit 1
fi

# ── Phase 2: measure ──────────────────────────────────────────────────────────
cargo build -p explorer-robot --release --features bench_motor_measure

if timeout -k 5 240 probe-rs run --chip RP235x --protocol swd --speed 16000 \
    "$BIN" > /tmp/motor-bench-measure.log 2>&1; then
    :
else
    status=$?
    if [ "$status" -ne 124 ]; then
        echo "probe-rs run (measure) failed with status $status" >&2
        exit 1
    fi
fi

grep 'bench-precision:' /tmp/motor-bench-measure.log || true

# probe-rs appends its own shutdown lines after the firmware's last log, so keep
# only the final result line before deciding the outcome.
grep 'bench-precision: max=' /tmp/motor-bench-measure.log > /tmp/motor-bench-measure-final.log || true

if tail -n 1 /tmp/motor-bench-measure-final.log | grep -q 'PASS'; then
    exit 0
fi

echo "bench-precision: no PASS line at the end of /tmp/motor-bench-measure.log" >&2
exit 1
