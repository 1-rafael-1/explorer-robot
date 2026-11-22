# Motor HIL bench testing

How an agent exercises the motor calibration, the Basic Motor Test and the
corrected-divergence regression on the real board without touching the panel.
The permanent harness is the non-default `bench_motor_calibrate` and
`bench_motor_measure` firmware features plus `scripts/motor-bench-precision.sh`;
the Stop and floor-drop checks below still use the normal Procedure seam and
`defmt` logs over RTT.

The panel is the only normal input. For an agent, the reliable seams are the two
bench features and, for a one-off check, a temporary auto-start task that
dispatches a Procedure after boot. Remove every temporary change before
committing; the permanent features stay.

The bench runs in **two separate boots**. Phase 1 calibrates and holds on the
Result Report with the motors stopped. Phase 2 flashes fresh, loads the saved v4
curve, starts at the menu, and only then drives the precision sweep. **A report
and motor motion never coexist**: do not merge the phases back into one boot, and
do not start a precision pass while a calibration report is on screen.

## Safety and setup

- Supply current limit **≥ 2 A (2000 mA)** for both phases. The
  corrected-divergence regression's measured points stop at 90 %, but every boot
  first runs the discarded warm-up, which includes the pair 100 % step, so the
  measure phase needs ≥ 2 A too. At 600 mA the board hit intermittent exceptions
  at 100 % Power Steps. A weak supply looks like a firmware fault.
- The **20-step pair sweep** (forward and reverse, 10–100 %) commands both
  tracks together at every step, including the **pair 100 % step**. That step
  needs a supply rated for **≥ 2 A** (2000 mA). The 2026-11-21 pass faulted at
  the first both-track 100 % point on a 1000 mA supply. The Basic Motor Test and
  the Motor Calibration both run the full sweep, so phase 1 needs ≥ 2 A to reach
  the end; the corrected-divergence regression alone stops at 90 % for this
  reason. Add `100` to the regression only with a ≥ 2 A supply.
- The **discarded warm-up** each boot runs before measuring (see §2) is the same
  20-step raw pair sweep, so it also hits the pair 100 % step and needs the
  ≥ 2 A supply. It adds roughly 50 s of motion before the calibration or the
  regression.
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
scripts/motor-bench-precision.sh
```

The script runs two separate probe-rs sessions, each with its own build, flash
and reset. A reset does not carry the panel state across, so the phase 1 report
is gone before phase 2 commands a motor.

**Phase 1 — calibrate** (holds on the Result Report, motors stopped):

```sh
cargo build -p explorer-robot --release --features bench_motor_calibrate
timeout -k 5 180 probe-rs run --chip RP235x --protocol swd --speed 16000 \
  target/thumbv8m.main-none-eabihf/release/explorer-robot > /tmp/motor-bench-calibrate.log 2>&1
```

The script prints every `bench-calibrate:` line and fails unless the
`bench-calibrate: calibration saved` line appears.

**Phase 2 — measure** (menu, no Procedure or report, saved v4 curve loaded):

```sh
cargo build -p explorer-robot --release --features bench_motor_measure
timeout -k 5 240 probe-rs run --chip RP235x --protocol swd --speed 16000 \
  target/thumbv8m.main-none-eabihf/release/explorer-robot > /tmp/motor-bench-measure.log 2>&1
```

The script prints every `bench-precision:` line and exits 0 only when the final
`bench-precision: max=` line is a `PASS`.

`timeout` is deliberate: `probe-rs run` runs until the target stops. A timeout
exit of `124` is normal for both sessions. probe-rs appends its own shutdown
lines after the firmware log, so the script keeps only the final result line
before deciding the outcome.

For boot/calibration-load checks, a 15 s run of the normal release build is
enough:

```sh
timeout -k 5 15 probe-rs run --chip RP235x --protocol swd --speed 16000 \
  target/thumbv8m.main-none-eabihf/release/explorer-robot > /tmp/boot.log 2>&1
grep -n "Motor calibration curve loaded\|System initialization complete" /tmp/boot.log
```

A phase-2 boot must log `Motor calibration curve loaded: run=N, ...` before the
regression starts; without that record the corrected divergence is meaningless.

## 2. The permanent bench features

`robot/src/bench_precision/` is compiled when either of the two non-default
features is enabled. If both are enabled (e.g. `--all-features`), the calibrate
phase takes precedence and the measure phase is not compiled; enable exactly one
for a bench run. Both are off by default, so the production firmware is unchanged.
Do not add a Procedure or a panel screen for them: they are bench diagnostics,
not normal Activities.

**`bench_motor_calibrate`** waits for `calibration::is_initialized()`, then:

1. dispatches `Procedure::MotorCalibration` from the panel origin; the Procedure
   runs its own discarded warm-up before measuring;
2. waits for Activity State to reach `Complete`/`Failed`;
3. logs `bench-calibrate: calibration saved` (or
   `bench-calibrate: calibration failed: <reason>`), then holds without starting
   anything else.

The calibration's `release_idle` runs before the Result Report is published, so
the drivers coast and go to standby and the wheels are stopped before the report
appears. **No motor command is issued after the report.**

**`bench_motor_measure`** boots to the menu with no Procedure or report active,
waits for initialization (the saved v4 record loads on this boot), then runs the
corrected-divergence regression. It logs `bench-precision: ...` and one final
`bench-precision: max=<pct> PASS|FAIL`, then holds.

### Motor warm-up (both Procedures)

The warm-up is product behaviour, not a bench-only step: **both** the Motor
Calibration and the Basic Motor Test begin with a discarded 20-step **raw** pair
sweep (forward and reverse, 10–100 %) that dwells `motor_sweep::WARMUP_DWELL_MS`
(2 500 ms) per step — `2 500 ms × 20 ≈ 50 s`. It logs each step as
`motor-warmup: dir=<Forward|Reverse> p=<power>` and then `motor-warmup: complete`;
grep for `motor-warmup:` to confirm it ran. The results are thrown away: the
warm-up feeds neither the calibration nor the test or the regression.

Its purpose is to remove the cold low-duty imbalance from the measurement: the
HIL pass showed cold friction, not rounding, was the dominant low-power effect.
Because both Procedures run it, the calibration and the corrected-divergence
regression are measured with the motors in the same warm state.

The calibrate boot relies on the Procedure's own warm-up — there is no separate
pre-dispatch warm-up. The measure boot runs outside a Procedure, so it calls
`motor_sweep::run_warm_up(None)` before the regression; that is the same shared
implementation, with no claim (so no Stop or floor-drop checks).

It commands raw `SetTracksRaw` so no curve is applied (the point is load, not a
corrected speed), and it includes the **pair 100 % step**, so like the full sweep
it needs the **≥ 2 A** supply. The two-boot split and its safety rules are
unchanged: the warm-up runs as part of the dispatched Procedure in phase 1 and
before the regression in phase 2, never while a report is on screen.

Do not combine a report and motor motion in one boot: that is the failure the
two-phase split exists to prevent.

For a one-off check, the temporary auto-start recipe is still useful. Create
`robot/src/bench_auto.rs` (a temporary file, **do not commit it**):

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

#[embassy_executor::task]
async fn bench_auto_task() {
    while !calibration::is_initialized().await {
        Timer::after(Duration::from_millis(500)).await;
    }
    Timer::after(Duration::from_secs(3)).await;
    info!("bench-auto: dispatching {}", Procedure::BasicMotor.label());
    let _ = procedure::dispatch(Procedure::BasicMotor, ClaimOrigin::Panel).await;
}

pub fn init(spawner: Spawner) {
    spawner.spawn(bench_auto_task().unwrap());
}
```

Add `mod bench_auto;` and `bench_auto::init(spawner);` to `robot/src/main.rs`,
then remove both again.

## 3. The pair sweep: twenty simultaneous steps

One cycle is **forward and reverse, each over ten Power Steps from 10 % to
100 %**: 20 pair steps. Each step commands **both** tracks at the same requested
power and measures both encoder deltas over the same window, so the calibration
is derived in the same load/supply condition it is applied in.

Each step:

1. brakes both tracks and waits for **Quiescence** on each — no encoder pulses
   for 250 ms, with a 2 s timeout invalidating the point;
2. resets both counters and confirms a fresh sample at zero;
3. commands both tracks at the step power with raw `SetTracksRaw`;
4. discards the first 500 ms and counts each track's delta over 2 000 ms.

A dead encoder task aborts after 500 ms without a fresh sample. A floor drop
aborts the sweep; an operator Stop ends it with no Result Report. A measurement
problem finishes the sweep and then fails the calibration, so the pattern is
visible.

The Basic Motor Test and the Motor Calibration run the identical sweep, after the
identical discarded warm-up. Its 100 % pair step commands both tracks at once, so a
full 10–100 % run needs the ≥ 2 A supply noted above. The corrected-divergence
regression measures only 20–90 % (gate 30–90 %), but both Procedures' warm-up still
hits the pair 100 % step, so its measure phase also needs ≥ 2 A.

## 4. Reading the motor logs

The Basic Motor Test logs both tracks of every raw pair:

```text
basic-motor: Left forward 40% count=583 speed=291/s
basic-motor: Right forward 40% count=568 speed=284/s
basic-motor: Left forward 50% invalid: stale sample
```

When a curve is loaded it then runs two curve-applied verification passes and
logs each corrected pair plus the averaged result:

```text
basic-motor: corrected pass 1/2
basic-motor: corrected Forward 40% L=560 R=566 div=1.06
basic-motor: corrected-mean Forward 40% div=1.06
basic-motor: corrected low-power Forward 1.64% (informational, below 30%)
basic-motor: corrected max=1.6
```

`div` is the **Track Divergence**: `|L − R| / min(L, R)` in percent. The
`corrected-mean` values are the per-point means of the two passes; `corrected
max` is the highest gated mean (30–100 %). The 20 % mean is information only.

Correction factors are displayed with the stored per-mille resolution, e.g.
`0.970` or `1.000`, never rounded to hundredths like `0.97`.

The live readout is a **pair readout**: the current direction and power, both
live counts, the last completed pair and both counts, both Breakaways, both
corrections and the step index within the twenty-step cycle. One progress bar
spans the whole test — the discarded warm-up, the raw cycle and both corrected
cycles — so it advances once instead of resetting per stage. Rows read like
`Fwd 40%`, `L 583 R 568`, `L 20% R 20%` and `L 0.970 R 1.000`. During a corrected
pass the `Step` and `Last` rows carry a `Corr` prefix, e.g. `Corr Fwd 40%`.

The Motor Calibration logs the same per-track counts and speeds; on success it
logs `Motor calibration curve saved successfully`, and the boot load logs
`Motor calibration curve loaded: run=N, floor=...%, counts_per_cm=...`.

Useful greps:

```sh
grep -c "basic-motor:.*invalid" /tmp/motor-bench-calibrate.log
grep -n "basic-motor: corrected" /tmp/basic.log
grep -n "motor-cal\|bench-calibrate\|bench-motor" /tmp/motor-bench-calibrate.log
grep -n "Motor calibration curve loaded\|saved successfully" /tmp/motor-bench-calibrate.log
grep -n "bench-precision\|bench-motor" /tmp/motor-bench-measure.log
```

The `bench-motor: left requested=... resolved_permille=... hardware_permille=... inverted=...`
line (emitted only while a bench feature is enabled) shows the left-output
inversion; the resolved and hardware values are per-mille (tenths of a percent,
`0..=1000`). The requested sign and the curve stay logical; only the left
hardware output is negated.

Expected Basic Motor Test: **one raw cycle and, with a curve, two corrected
verification cycles**, so roughly **4 minutes** including the warm-up. A clean
run has **0** `invalid` points. At 100 % the 2 s window reads roughly 1 500–1 800
counts per track during the raw cycle (the pair step loads the supply, so it can
sit below the old single-track figure). A 100 % step returning ~11 000 counts
means the encoder scale is wrong (960 pulses/rev), not 330.

The saved record is format version 4 and is pair-measured. Version 3 records are
rejected, so a board with an old record reports no motor calibration until it is
re-run.

## 5. What the Basic Motor Test measures

Before the measured cycles, the test runs the discarded 20-step raw pair warm-up
described above. The **raw cycle** commands `SetTracksRaw` on purpose: it measures
the drivetrain, not the curve. Both tracks run simultaneously, so the raw counts
already show the same condition normal driving uses. The `Raw cnt` and `Corr`
tables show the raw counts and the curve in force for each track.

**Do not use the raw counts to judge calibration accuracy.** To judge the curve,
the test then runs **two curve-applied verification cycles** through normal
`SetTracks`. Each corrected pair's counts give a Track Divergence, the two passes
are averaged per point, and the worst gated mean (30–100 %) appears in the Result
Report footer as `Corr max 1.6% Fwd 90%`. The `Raw spread` line is the raw
four-leg spread at 100 % and includes forward-versus-reverse; it is not the
left/right calibration error. The automated bench regression
(`bench_motor_measure`) applies the same curve and gates PASS at 30–90 %; it is
the repeatable CI-side check, while this footer is the on-robot verification.

Target: **< 3 % optimal, < 5 % minimal** on `Corr max`. The observed
2026-11-22 runs were 1.6–2.9 % gated in the two-boot bench; the in-boot footer
can show a low-power knee residual near 3 %. A static corrected-command
refinement was tried and HIL-tested on 2026-11-22: it reached 1.3–1.5 % in the
calibration boot but measured 3.8–6.6 % after a fresh boot, so it was reverted.
The base curve is the more robust static estimate; the cross-boot floor needs
closed-loop trim. Evidence:
`.scratch/motor-calibration/issues/10-corrected-command-refinement.md`.

A `Corr max` of `--` means the corrected pass produced no usable gated point, or
one of the two repeats for a gated point did not show sustained motion; the log
names the missing point. `No curve - run Motor` means no Motor Calibration Curve
was loaded, so the verification was skipped.

## 6. Motor Calibration

The Procedure needs a rail sample ≥ 6.0 V and arms the floor-drop gate. It runs the
same discarded 20-step raw pair warm-up, then the same 20-step pair sweep, derives
the per-track/per-direction curve by inverse interpolation over the simultaneous
measurements, and saves it under the version 4 record. The storage task publishes
the saved curve as soon as flash confirms the write, and the record is loaded again
on the next boot. Two runs on the same robot should have the same floor and target
legs; per-point corrections may move a few percent because the motors/encoders are
not perfectly repeatable.

A successful run holds the Result Report; Back is a panel action. A failed run
leaves the previous curve in force. `bench_motor_calibrate` logs
`bench-calibrate: calibration failed: <reason>` and holds; it does **not**
continue into the precision pass, so no motion follows the report.

The left motor's leads are reversed relative to the right, so the driver negates
only the left hardware output (see `LEFT_MOTOR_INVERTED` in
`robot/src/task/motor_driver.rs`). The curve, the logical command and the
reported correction stay logical. A curve saved under the old polarity must be
re-derived; running Motor Calibration again writes the v4 record for the current
polarity.

## 7. Corrected-divergence regression

The `bench_motor_measure` feature measures, for **forward and reverse** and
powers **20, 30, …, 90 %**, two repeats each. It gates the PASS at **30–90 %**:
the 20 % band is stiction-limited, so its result is logged for information only.
Forward 10 % produces zero counts, the breakaway floor is 20 %, and the 20 %
left/right ratio changes sign from run to run, so a 20 % point cannot separate a
calibration error from breakaway noise. The 20 % readings are reported on a
separate `bench-precision: low-power-max=<pct> (informational, below 30 %)` line
before the final result.

1. brake both tracks and wait for Quiescence;
2. reset both counters;
3. command both tracks at the same signed power with normal `SetTracks` (curve
   applied);
4. discard 500 ms, then measure both encoder deltas over 2 000 ms;
5. log
   `bench-precision: dir=Forward p=40 rep=1 L=552 R=558 div=1.1`;
6. track the gated maximum (30–90 %) and the informational 20 % maximum.

The final line is exactly one of:

```text
bench-precision: max=2.41 PASS
bench-precision: max=8.40 FAIL
```

Pass is **max < 5.0 % at 30–90 %; 20 % is informational.** The measure-phase
boot loads the v4 curve saved by phase 1, so `SetTracks` uses it. Run it on a
charged pack; voltage compensation and the loaded rail affect both tracks
together but a sagging pack makes the point noisy.

### Full-power measurement variant

The regression measures 20–90 % (the gate is 30–90 %); the discarded warm-up
already runs the pair 100 % step, so the whole bench needs a supply rated for
**≥ 2 A**. To also *measure* 100 %, add `100` to `BENCH_POWERS` in
`robot/src/bench_precision/measure.rs`. The earlier one-track sweep completed at
1000 mA, but that result does not apply to the pair sweep's 100 % step.

## 8. Stop, floor-drop, brake

- **Stop:** dispatch a Procedure, wait mid-sweep, then call
  `procedure::stop(StopOutcome::Operator, InterruptKind::Stop)`. Expect the
  motor driver disabled, encoder sampling stopped, no further pair step, and no
  `ProcedureFinished` event (no Result Report).
- **Floor drop:** with the real gate armed, a drop aborts with
  `detail=floor drop detected` and brakes. With covered sensors, a drop is an
  environment issue; do not count it as a motor fault.
- **Brake-after-fault:** the floor-drop path brakes and fails; verify on glass.

## 9. Cleanup checklist

The `bench_motor_calibrate` / `bench_motor_measure` features and
`scripts/motor-bench-precision.sh` are permanent and stay. Before committing
other changes:

- [ ] Delete any temporary `robot/src/bench_auto.rs`.
- [ ] Revert its `mod bench_auto;` / `bench_auto::init(spawner);` lines.
- [ ] Remove any temporary `[DEBUG-...]`/`bench-` logs and floor-drop/trace
      bypasses.
- [ ] Re-enable the LiDAR task and the real floor-drop gate.
- [ ] `git --no-pager status --short` shows only the intended source changes.
- [ ] `cargo check --workspace`, `cargo clippy --workspace`,
      `cargo test -p motor-cal --features std --target x86_64-unknown-linux-gnu`,
      `cargo test -p touch-ui --features std --target x86_64-unknown-linux-gnu`,
      `cargo fmt --all -- --check`.
- [ ] Re-run `scripts/motor-bench-precision.sh` with the clean release build.

## 10. Human-only observations

- Result Report contents, Back, and a second run with no stale numbers.
- The Basic Motor Test's `Corr`-prefixed rows, the `Raw spread` line and the
  `Corr max` footer on glass.
- Distance Calibration with a tape measure on two legs.
- The live pair readout on glass.
- The physical brake after a floor-drop.

## Known pitfalls from the 2026-11-21 pass

- **600 mA supply limit** produced intermittent exceptions at 100 % steps.
  Raising to 1000 mA removed them. Check the supply before suspecting firmware.
  A two-track 100 % point needs ≥ 2000 mA.
- **Same-millisecond reset sample** was rejected as stale at 20 ms sampling;
  fixed in `f2bc2f7`. If `stale sample` reappears, capture the reset/sample
  timestamps before changing the engine.
- **Covered rangefinders** can still produce a floor-drop event; that is the
  sensor/environment, not the motor path.
- The Basic Motor Test's raw counts are not the calibration error. Use its
  `Corr max` footer (or the `bench-precision:` regression) for the corrected
  residual; the `Raw spread` line includes forward-versus-reverse.
- A `Corr max` near 3 % at forward 30 % is the known low-duty knee, not a
  measurement fault; see `.scratch/motor-calibration/issues/10-corrected-command-refinement.md`.
- An old version 3 calibration record is rejected. The boot log then reports no
  motor calibration; run Motor Calibration again to write a v4 record.
