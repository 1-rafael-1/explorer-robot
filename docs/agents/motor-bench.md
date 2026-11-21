# Motor HIL bench testing

How an agent can exercise the motor calibration and the Basic Motor Test on the
real board without touching the panel. This is the recipe used for the
2026-11-21 bench pass; the results are in
`.scratch/motor-calibration/issues/09-bench-verification.md`.

The panel is the only normal input. For an agent, the reliable seam is a
**temporary auto-start task** that dispatches a Procedure after boot, plus
`defmt` logs over RTT. Remove every temporary change before committing.

## Safety and setup

- Supply current limit **≥ 1000 mA**. At 600 mA the board hit intermittent
  exceptions at 100 % Power Steps. A weak supply looks like a firmware fault.
- Secure the board. For a free-standing test, put the motors on their brackets
  with nothing in the tracks, and keep hands and cables clear of the wheels.
- Cover the rangefinders if the tracks are free-standing. A floor-drop event
  with a covered sensor is an environment problem; the real gate should be left
  in place unless you are deliberately isolating it.
- The LiDAR is not needed for a motor sweep. Disable it only if you are
  isolating a core1 fault; otherwise leave the real firmware intact.
- Use a **release** build. Debug builds are large and slow and can change timing.

## 1. Build, flash and capture

Run from the repository root:

```sh
probe-rs list
cargo build -p explorer-robot --release
timeout -k 5 200 probe-rs run --chip RP235x --protocol swd --speed 16000 \
  target/thumbv8m.main-none-eabihf/release/explorer-robot > /tmp/bench.log 2>&1
```

`timeout` is deliberate: `probe-rs run` runs until the target stops. A timeout
exit of `124` is normal. Inspect `/tmp/bench.log`; `Firmware exited
unexpectedly: Exception` is not.

For boot/calibration-load checks, a 15 s run is enough:

```sh
timeout -k 5 15 probe-rs run --chip RP235x --protocol swd --speed 16000 \
  target/thumbv8m.main-none-eabihf/release/explorer-robot > /tmp/boot.log 2>&1
grep -n "Motor calibration curve loaded\|System initialization complete" /tmp/boot.log
```

## 2. Temporary auto-start recipe

Create `robot/src/bench_auto.rs` (a temporary file, **do not commit it**):

```rust
//! TEMPORARY bench auto-start; remove before commit.
use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use touch_ui::Procedure;

use crate::{
    system::state::{activity::{self, Stage}, calibration},
    task::procedure::{self, ClaimOrigin},
};

/// The Procedure to run.
const PROCEDURE: Procedure = Procedure::BasicMotor;

#[embassy_executor::task]
async fn bench_auto_task() {
    while !calibration::is_initialized().await {
        Timer::after(Duration::from_millis(500)).await;
    }
    Timer::after(Duration::from_secs(3)).await;
    info!("bench-auto: dispatching {}", PROCEDURE.label());
    let _ = procedure::dispatch(PROCEDURE, ClaimOrigin::Panel).await;
    // For a Stop test, wait, then:
    // procedure::stop(StopOutcome::Operator, InterruptKind::Stop);
}

pub fn init(spawner: Spawner) {
    spawner.spawn(bench_auto_task().unwrap());
}
```

Add to `robot/src/main.rs`:

```rust
mod bench_auto;                  // with the other `mod` lines

// inside executor0's closure, after `init_startup(spawner);`
bench_auto::init(spawner);
```

The auto-start waits for the boot calibration loads (`is_initialized`) and then
dispatches the Procedure. The panel follows the running screen, but the logs are
the evidence. For a second Procedure, wait for `activity::snapshot().stage` to
reach `Stage::Complete`/`Stage::Failed`, then dispatch again.

## 3. Reading the motor logs

The Basic Motor Test logs one line per Power Step:

```text
basic-motor: Left forward 40% count=591 speed=295/s
basic-motor: Left forward 50% invalid: stale sample
```

The Motor Calibration logs the same count/speed per point; on success it logs
`Motor calibration curve saved successfully` and the boot load logs
`Motor calibration curve loaded: run=N, floor=...%, counts_per_cm=...`.
Add a temporary `bench-cal` log in `finish()` if you need the derived
floor/breakaways/corrections over RTT; the panel table is not visible to an
agent. Remove it afterwards.

Useful greps:

```sh
grep -c "basic-motor:.*invalid" /tmp/bench.log
grep -n "basic-motor\|bench-auto\|Procedure finished\|Firmware exited" /tmp/bench.log
grep -n "Motor calibration curve loaded\|bench-cal\|saved successfully" /tmp/bench.log
```

Expected Basic Motor Test: 40 steps, ~2.1–2.3 min. A clean run has **0**
`invalid` points. At 100 % the 2 s window reads roughly 1 550–1 800 counts/
track (≈ 2 350–2 700 over the old 3 s step). A 100 % step returning ~11 000
counts means the encoder scale is wrong (960 pulses/rev), not 330.

## 4. What the Basic Motor Test measures

The sweep commands **raw** `SetTracksRaw` on purpose. It measures the
drivetrain, not the curve. The counts therefore still show the pre-calibration
divergence between tracks; the `Correction` row and the Result Report's
correction table show the curve in force.

**Do not use the raw counts to judge calibration accuracy.** To judge the curve,
command a corrected speed (normal `SetTracks`, curve applied) and compare the
two tracks' measured counts, or run the same Step through a temporary
corrected-command harness. Target: **< 5 %** left/right divergence after
correction.

## 5. Motor Calibration

The Procedure needs a rail sample ≥ 6.0 V and arms the floor-drop gate. It runs
the same 40-step sweep, derives the curve, and saves it under the new record.
The saved record is loaded on the next boot. Two runs on the same robot should
have the same floor and target legs; per-point corrections may move a few
percent because the motors/encoders are not perfectly repeatable.

A successful run holds the Result Report; Back is a panel action. A failed run
leaves the previous curve in force.

## 6. Precision / corrected-divergence harness

To answer "is the calibration actually getting under 5 % error?", add a
temporary Procedure (or extend the auto-start) that, for each power in
`40, 60, 80, 100`:

1. brakes, waits for quiescence, resets the counters;
2. commands **both** tracks at the same requested power with normal `SetTracks`
   (so the curve is applied);
3. discards 500 ms, measures both encoder deltas over 2 000 ms;
4. repeats the point 3–5 times and averages;
5. computes `|left − right| / min(left, right)` for forward and reverse.

Run it once before calibration (identity) and once after. The before/after
difference is the calibration's real effect; the absolute after value is the
number to beat (< 5 %). Keep the raw per-repeat counts in the log so the
run-to-run spread is visible separately from the left/right bias.

## 7. Stop, floor-drop, brake

- **Stop:** dispatch the Procedure, wait mid-sweep, then call
  `procedure::stop(StopOutcome::Operator, InterruptKind::Stop)`. Expect the
  motor driver disabled, encoder sampling stopped, no further Power Step, and
  no `ProcedureFinished` event (no Result Report).
- **Floor drop:** with the real gate armed, a drop aborts with
  `detail=floor drop detected` and brakes. With covered sensors, a drop is an
  environment issue; do not count it as a motor fault.
- **Brake-after-fault:** the floor-drop path brakes and fails; verify on glass.

## 8. Cleanup checklist

Before committing:

- [ ] Delete `robot/src/bench_auto.rs`.
- [ ] Revert the `mod bench_auto;` / `bench_auto::init(spawner);` lines.
- [ ] Remove any `[DEBUG-...]`/`bench-` logs and floor-drop/trace bypasses.
- [ ] Re-enable the LiDAR task and the real floor-drop gate.
- [ ] `git --no-pager status --short` shows only the intended source changes.
- [ ] `cargo check --workspace`, `cargo clippy --workspace`,
      `cargo test -p motor-cal --features std --target x86_64-unknown-linux-gnu`,
      `cargo test -p touch-ui --features std --target x86_64-unknown-linux-gnu`,
      `cargo fmt --all -- --check`.
- [ ] Re-run the original bench command with the clean release build.

## 9. Human-only observations

- Result Report contents, Back, and a second run with no stale numbers.
- Distance Calibration with a tape measure on two legs.
- The live six-row readout on glass.
- The physical brake after a floor-drop.

## Known pitfalls from the 2026-11-21 pass

- **600 mA supply limit** produced intermittent exceptions at 100 % steps.
  Raising to 1000 mA removed them. Check the supply before suspecting firmware.
- **Same-millisecond reset sample** was rejected as stale at 20 ms sampling;
  fixed in `f2bc2f7`. If `stale sample` reappears, capture the reset/sample
  timestamps before changing the engine.
- **Covered rangefinders** can still produce a floor-drop event; that is the
  sensor/environment, not the motor path.
- The Basic Motor Test's raw counts are not the calibration error. Measure
  corrected commands (section 6).
