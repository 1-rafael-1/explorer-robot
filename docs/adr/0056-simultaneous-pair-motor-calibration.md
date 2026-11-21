# Motor calibration measures simultaneous track pairs and inverts the measured curve

The Motor Calibration Curve was derived from single-track measurements: each Power
Step commanded one track at the step power while the other coasted at zero. The
curve was then applied while both tracks ran together. The encoder read is not the
problem — two repeats of the same simultaneous point differ by 0.0–0.8 % — the
condition is. At 40 % forward the single-track calibration measured a 9.8 %
left/right ratio and stored a 0.908 correction, while the simultaneous ratio was
2.6 % and needed 0.974. Applying the single-track curve to a simultaneous run left
a corrected forward divergence of 17.6 % at 20 %, 13.6 % at 30 %, 8.7 % at 40 %,
5.6 % at 50 %, 5.1 % at 60 % and below 3.2 % at 70–90 %. The worst of that error
sits at the low powers normal driving uses, and it grows as the rail sags under
the second track's load.

A prototype that measured both tracks at the same command and derived the
correction from that pair took the 12.1 % raw forward divergence at 20 % down to
3.1 %, with under 3 % at 40–90 %. One point remained: 8.4 % at 30 % with the
linear `target / measured_speed` ratio. The left track measured 435 counts/s at
30 % and the target was the right track's 397 counts/s; the ratio commanded 27,
which measured 369 counts/s (7 % low). The motor's speed-versus-command response
is nonlinear at low duty, so scaling by a ratio does not land on the target.
Inverting the measured (20, 259) and (30, 435) points picks command 28, predicted
to measure about 1.5 % high.

What remains is not bounded by whole-percent command resolution. The resolved
command is carried as per-mille (tenths of a percent) and rounded exactly once,
after voltage compensation, into a 1/1000 PWM duty cycle. The HIL pass then
showed the 30 % / 40 % residual was a cold-state (thermal/friction) effect, not
the rounding floor.

**Status:** accepted. Partly supersedes ADR-0054 (the measurement condition and the correction model) and ADR-0055 (the cycle length and the live readout).

**Considered Options**

- **Keep single-track measurement and the linear ratio.** Rejected: the
  correction is derived in a different load and supply condition from the one it
  is applied in, and the linear ratio cannot represent the nonlinear low-duty
  response. This is the current 17.6 % at 20 %.
- **Simultaneous pair measurement with the linear ratio.** The prototype fixes
  most of it — 12.1 % to 3.1 % at 20 %, under 3 % at 40–90 % — but 30 % forward
  stays at 8.4 %, so it does not meet the < 5 % target.
- **Simultaneous pair measurement with inverse interpolation.** Chosen: the pair
  supplies (command, speed) points from the driving condition, and inverting them
  removes the low-duty nonlinearity (command 28 instead of 27 at 30 % forward,
  predicted about 1.5 %).
- **Measure a pair reference only.** Keep the forty single-track steps as the raw
  record and the test readout, and add a twenty-step pair sweep just to derive the
  correction ratios. Rejected: it costs another minute, keeps two conditions for
  one number, and weakens user story 27 — the test would no longer show the
  numbers that built the correction.

**Decision**

- A **Power Step** is a direction and a commanded power measured as a simultaneous
  pair: both tracks are commanded at the same requested power and both encoder
  counts are taken over the same **Measurement Window**.
- A cycle is 2 directions × 10 Power Steps = 20 pair steps. The four **Test Legs**
  are still reported, two at a time; the Result Report summary remains per Test
  Leg.
- The **Target Speed** at a Power Step is the slower of the two tracks in that
  direction, taken from the simultaneous counts, so no correction exceeds `1.0`.
- Each leg's command is resolved by inverse piecewise-linear interpolation of that
  leg's measured (command, speed) points, with a `(0, 0)` anchor and the monotone
  rule. The linear `target / measured_speed` ratio is superseded. The resolved
  command is carried as per-mille (tenths of a percent), not a whole percent.
- The **Calibration Floor** and the plausibility rules are unchanged: nothing is
  corrected below the floor, and an implausible correction fails the calibration.
- The stored record is version 4. A version 3 single-track record is rejected as
  "not calibrated" and is not migrated; the robot must be recalibrated after the
  upgrade.
- The **Basic Motor Test Readout** shows the pair: the direction and power, both
  live counts, both Breakaways, both corrections, and the index within the
  twenty-step cycle. The summary tables stay per Test Leg.
- The HIL regression is `scripts/motor-bench-precision.sh` with the
  `bench_motor_calibrate` and `bench_motor_measure` features: it runs calibration
  and measurement in two separate boots, each preceded by a discarded 20-step raw
  warm-up, and asserts corrected divergence below 5 % at 20–90 %. A two-track
  100 % run needs a bench supply of at least 2 A.

**Consequences**

- One cycle drops from roughly 2.1–2.3 minutes to about 1.1 minutes, because both
  tracks are measured in the time one took.
- The curve is measured in the same load and supply condition normal driving uses,
  so the correction transfers. Voltage compensation is still applied after the
  curve, as before.
- A version 3 record reads as "not calibrated". The only robot has to be
  recalibrated with the pair sweep; the old single-track curve was the source of
  the low-power over-correction.
- Per-mille resolution is now implemented: the command is rounded once, after
  voltage compensation, into a 1/1000 PWM duty
  (`set_duty_cycle_fraction(_, 1000)`).
- The HIL warm-up experiment showed that cold low-duty imbalance, not rounding, is
  the dominant low-power effect.
- The `ResetUnconfirmed` race is fixed by waiting for a fresh all-zero sample
  rather than a fixed sleep.
- A two-track 100 % step exceeded the 1000 mA bench supply during the prototype.
  The 100 % pair is a supply limit, not firmware: use at least 2 A for a full
  two-track run, or stop the HIL sweep at 90 %.
- The readout now shows both tracks at once, which is the condition the operator
  is judging; the summary is still per Test Leg so left/right and
  forward/reverse remain comparable.
- The Basic Motor Test and Motor Calibration still share one measurement engine;
  the test stays read-only and the calibration remains the only writer.

**Measured outcome (2026-11-21):** with the discarded warm-up and per-mille
resolution, 3 of 4 HIL runs passed at a maximum corrected divergence of 2.2–3.7 %.
The only remaining sample above 5 % was Reverse 20 % at 5.00 %, in the stiction
band. The documented gate is unchanged.
