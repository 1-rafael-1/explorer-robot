# The Basic Motor Test verifies the curve with a corrected pass

The Basic Motor Test measured only raw drivetrain pairs and ended with a raw
four-leg spread. That is the right instrument for the drivetrain, but it cannot
answer "is the calibration working?" — the raw counts still differ by roughly
6 % left-to-right even when the curve removes that difference. On 2026-11-22
three fresh two-boot regression runs passed at **1.58 %, 2.87 % and 2.19 %**
worst corrected divergence, while the Basic Motor Test's raw 50 % row read
roughly **9–11 %** and its `Spread` line included forward-versus-reverse. An
operator could easily read the test as "the calibration is 10 % wrong" when the
corrected residual was under 2 %.

The product therefore had no on-robot view of the number the calibration is
supposed to produce. The bench regression did, but it needs a debug probe, a
feature build and two boots.

**Status:** accepted. Partly supersedes ADR-0055 (the test is no longer one
cycle) and extends ADR-0056 (the corrected verification uses the pair curve).

**Considered Options**

- **Only relabel the raw spread.** Cheap, but it leaves the operator without the
  corrected number; the test still cannot answer the question it is asked.
- **A separate "Motor Precision" Procedure.** Clean separation, but it adds a
  second menu entry, a second readout model and a second report for a check that
  uses the same sweep and the same curve the test already has.
- **Make the Basic Motor Test measure only corrected pairs.** Rejected: the raw
  measurement is what the Motor Calibration derives the curve from, and the
  operator needs the drivetrain evidence the curve is correcting.
- **Raw cycle, then two curve-applied verification cycles, in one Procedure.**
  Chosen. The raw cycle keeps the instrument and the calibration inputs; the
  verification cycles measure the actual corrected result, and the Result Report
  carries both.

**Decision**

- The Basic Motor Test runs the raw 20-step pair cycle exactly as before. Its
  Result Report tables are headed `Raw cnt` and `Corr`.
- When a Motor Calibration Curve is loaded, the test then runs **two** full
  20-step pair cycles through normal `SetTracks`, so the curve is applied. Each
  point is the mean of its two repeats, but only when both repeats produced a
  sustained-motion window; a gated point missing a repeat makes the verification
  incomplete and the footer reads `Corr max --`. The worst mean over **30–100 %**
  is the **Corrected Max**. The 20 % band is measured for information only, as in
  the bench regression.
- The Result Report shows `Floor <p>%  Raw spread <p>%` and, in the reserved
  footer row, `Corr max <p>% <dir> <power>%`. `Raw spread` is the raw four-leg
  spread at 100 %; `Corr max` is the calibration's residual. A missing curve
  yields `No curve - run Motor`; a verification with no complete gated point
  yields `Corr max --`. The summary model is shared with the Motor Calibration, so
  its tables are headed `Raw cnt` and `Corr` and its spread line reads
  `Raw spread` too; that is the same raw evidence, not a second meaning.
- The live readout prefixes `Corr` on its `Step` and `Last` rows during a
  verification cycle and never latches the raw Breakaway from a corrected pair;
  the raw Breakaways are kept per direction, so a corrected pass shows the
  direction it is running.
- The verified **Track Divergence** and its aggregation live in `motor-cal`:
  `divergence_percent`, `divergence_tenths` and the host-tested `Verification`
  model are shared by the test and the bench regression. A gated point measured
  only once is never the headline.
- A Motor Calibration still measures and saves only the raw curve; the
  verification is read-only and never changes the saved curve.

**Consequences**

- The test's measured time roughly doubles when a curve is loaded (about four
  minutes including the warm-up). Without a curve it skips the verification and
  keeps the old duration.
- The operator finally has a visible accuracy number on the robot, and the raw
  table and the `Raw spread` line can no longer be mistaken for it.
- Two repeats cost more time but prevent a single noisy window from becoming the
  headline number. The bench regression also takes two repeats, but it gates the
  worst repeat; the test reports the mean of the two and refuses to name a maximum
  when a gated point lacks a repeat.
- The worst in-boot residual observed on 2026-11-22 was **3.1 % at forward 30 %**
  (2.76 % and 3.52 % in the two repeats), while every other gated point was under
  2 %. The low-duty knee between the 20 % and 30 % calibration points is the
  remaining inverse-interpolation error. A corrected-command refinement was
  implemented and HIL-tested on 2026-11-22: it reduced the in-boot maximum to
  1.3–1.5 %, but the refined table measured 3.8–6.6 % after a fresh boot, worse
  than the base curve's 1.6–2.9 %. The static refinement overfits one thermal
  state and was reverted; the evidence is in
  `.scratch/motor-calibration/issues/10-corrected-command-refinement.md`.
- The test still uses one measurement engine (`run_sweep` with a `SweepMode`) and
  the `motor-cal` verification model, so the test and the bench cannot drift
  apart.
