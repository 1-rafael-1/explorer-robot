# The Panel's menu tree and Procedure identity live in the touch UI model crate

The model crate owns the Panel's screens but not the Procedures behind them, so a Menu
Entry's label, its destination and the screen Back returns to were each written down in
the firmware as well — a label three times, a parent relation four, and a Procedure's name
in five parallel tables. The firmware also had no way to ask the model for a screen, so it
moved the Panel by fabricating a pointer tap at a rectangle it had computed itself.

**Status:** accepted.

**Considered Options**

- **Keep the screen tree in the firmware and treat the crate as a widget library.** Rejected:
  the duplicate tables were the defect, not a side effect of it.
- **Let the model own outcomes as well as screens, driving every transition from them.**
  Rejected: the model would have to learn drive outcomes and sensor state, which are not
  screen facts, and the actuators' policy would move away from the actuators.
- **Add only the command-level entry point and leave the titles where they were.** Rejected:
  the five title tables are what makes adding a Procedure a many-file edit.

**Decision**

- One enumeration names every Menu Entry and carries its label, the screen it opens, the
  screen Back returns to, and whether it ends only on an explicit stop. Nothing else states
  a label or a parent.
- An entry is a submenu, a Procedure, or a screen, so the distinction is in the type and the
  Activity State cannot name a submenu where a Procedure belongs.
- The model resolves where a finished Procedure lands, from the entry's identity.
- The firmware asks the model for the screen it wants; it never fabricates input.
- The Activity State records a Procedure by identity rather than by a parallel enumeration.

**Consequences**

- Adding, renaming or reordering a Menu Entry is one edit, and reordering is a compile-time
  question rather than a silent change of which Procedure a tap starts.
- Where a finished Procedure lands is a screen-tree fact, so it is expressible through the
  model's public interface and therefore host-testable without the Panel.
- The model accumulates domain vocabulary — Procedures, their labels, their parents — but no
  hardware knowledge, so the crate stays HAL-free.
- The placeholder screen vocabulary, which existed only to be overwritten before it was
  drawn, is deleted rather than maintained.
