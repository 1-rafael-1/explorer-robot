# The Basic Motor Test runs one cycle and ends in a held Result Report

The Basic Motor Test cycled forever and never concluded. It was built that way so
an operator could watch power consumption repeat, which a bench supply now does
better. The loop cost more than it gave: the run never reached a result, the
progress bar reset instead of completing, and there was nowhere to show what the
forty measured Power Steps actually said. ADR-0053 had already given the test a
readout, but only a live one, and it deliberately kept the bar because the bar
was the only whole-cycle cue.

**Status:** accepted. Partly supersedes ADR-0053: the test no longer cycles, and
the progress bar is running-only. Partly superseded by ADR-0056: one cycle is
twenty simultaneous-pair steps rather than forty single-track steps, the live
readout carries the pair, and the run is about 1.1 minutes. Partly superseded by
ADR-0057: with a loaded curve the test also runs two curve-applied verification
cycles before the report. The one-shot run, the held Result Report and the
per-Test-Leg summary stand.

**Considered Options**

- **Keep cycling and show a summary every cycle.** Rejected: the summary would
  have to interrupt the run or share the panel with it, and the operator would
  have to take a gesture to resume. The repeat itself is no longer wanted.
- **A timed summary that reverts after ten seconds.** Rejected: a Result Report
  holds until dismissed (ADR-0034), and a self-dismissing one would be a second
  lifecycle for the same kind of screen.
- **Tap anywhere to resume.** Rejected: it introduces a gesture no other screen
  uses, on a panel where the header action is Stop.
- **Two summary screens toggled by tap.** Rejected in favour of one screen once
  the bar is gone: the full readout area fits both tables.
- **A panel-only completion that suppresses the Protocol event.** Rejected: two
  completion lifecycles for one Procedure, and the stream would be lying about
  what the robot is doing.

**Decision**

- One run is one cycle: four Test Legs of ten Power Steps, ~2.1–2.3 minutes. The
  run then calls `claim.complete`, and the Result Report holds until Back.
- An operator Stop ends the run early with no Result Report. Motors stop, the
  readout leaf clears, and the panel returns to Test Mode.
- The Result Report shows the Basic Motor Test Readout summary on one screen:
  the counts table (Breakaway and counts at 40, 60, 80 and 100 per Test Leg) and
  the correction table (the curve in force at those steps), plus one unjudged
  line with the Calibration Floor and the spread at 100 %.
- The progress bar is drawn while running only. It carries position in the
  forty-step cycle; at completion there is no position to carry.
- The readout leaf survives completion and is cleared on the next start, so the
  Result Report has data to draw and a later run never shows stale numbers.
- The `ACTIVITY` stream and the Remote see the completion, as they do for every
  Procedure. The numbers stay panel-only (ADR-0053).

**Consequences**

- `Procedure::BasicMotor` keeps its existing no-op routing in
  `handle_procedure_finished`, so the completion holds instead of returning to
  the menu; the UI's `finished` view supplies Back.
- The test task stops clearing the readout at teardown on success.
- The Motor Calibration Procedure shows the same two tables plus its saved
  confirmation, so the operator can judge the curve without re-running the test.
- The one-shot run is the measurement source for the calibration floor; a
  calibration run repeats the same sweep and writes the curve (ADR-0054).
