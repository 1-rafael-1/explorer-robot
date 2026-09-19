# Retire the rotary encoder: touch is the sole operator input

The EC11 rotary encoder is removed from the robot's design and its firmware, and the
resistive touch panel becomes the only operator input. The decision is deliberate beyond
convenience: the panel can render a control for every action, and the encoder's three GPIOs
are wanted elsewhere, but it leaves the robot with no input that works when the UI task is
hung or the panel is dead.

**Status:** accepted

**Considered Options**

- **Keep the encoder as a fallback input alongside touch.** Rejected: two input paths means
  two UI models, and the encoder's turn/press/hold vocabulary was already the reason the
  menu logic was shaped around a hardware gesture rather than a screen.
- **Add a hardwired emergency-stop button on the motor driver's standby pin.** Rejected for
  now: a kernel-level stop is a different safety property from a touch Stop, but the
  operator's reachable power switch plus the touch Stop on every running screen is judged
  sufficient. This is an accepted risk, not an oversight — revisit if the robot ever runs
  in a space where the operator cannot reach it.
- **Keep the encoder wired but unused.** Rejected: dead hardware with live pin claims is
  worse than either keeping or removing it.

**Decision**

- `src/task/control/`, `Ec11Pins`, the PIO1 SM3 quadrature initialisation,
  `RotaryDirection`, the `Events::Rotary*` and `UiEvent::Rotary*` variants, the
  `handle_rotary_*` handlers and `next_menu_index`'s rotary parameter are all deleted.
  PIO1 keeps only its RGB LED state machines.
- Every action the encoder offered has a screen affordance: tap replaces turn-and-press,
  header Back replaces hold-to-back, and a touch Stop is added to every running screen.
- Because the hold gesture was the only way out of some running procedures, tests that had
  no stop API (`turns`, `straight_drive`, `arc_drive`) gain stop signals.
- GPIO 22, 23 and 24 return to the spare pool, reserved for the future rangefinder
  round-robin.

**Consequences**

- Input now depends on the panel initialising. The display task's existing
  display-offline behaviour keeps the robot running, but it runs uncommandable.
- The UI's input vocabulary is pointer-shaped (tap, press, drag) rather than
  device-shaped, which is what let the menu logic move into a host-testable crate.
