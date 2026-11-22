//! Permanent hardware-in-the-loop motor bench, split into two boot phases.
//!
//! This module is compiled only with one of the non-default
//! `bench_motor_calibrate` / `bench_motor_measure` features, so the production
//! firmware is unchanged. The two phases run in separate boots and never share a
//! screen with motor motion:
//!
//! - `bench_motor_calibrate` waits for `calibration::is_initialized()`, dispatches
//!   `Procedure::MotorCalibration` from the panel origin, waits for Activity State
//!   to reach `Complete`/`Failed`, logs `bench-calibrate: ...`, and then holds.
//!   The calibration's `release_idle` stops the motors before the Result Report,
//!   so nothing drives once the report is visible.
//! - `bench_motor_measure` starts from the menu with no Procedure or report
//!   active, waits for initialization (the second boot loads the saved v4 record),
//!   and runs the corrected-divergence regression in `measure`. It logs
//!   `bench-precision: ...` lines and one final `bench-precision: max=<pct>
//!   PASS|FAIL`.
//!
//! Both phases share the boot helpers in `shared`: waiting for the saved
//! calibration, settling at the menu, the discarded warm-up sweep, and the final
//! hold.
//!
//! The regression commands both tracks at the same requested power through
//! [`crate::task::motor_driver::MotorCommand::SetTracks`], so the Motor Calibration
//! Curve is applied just as it is in normal driving. For each direction and each
//! power it brakes, waits for **Quiescence**, resets both counters, discards a
//! 500 ms settle interval, and measures both encoder deltas over 2 000 ms. The
//! divergence is `|left − right| / min(left, right)`.
//!
//! It measures 20–90 %, but gates the pass at 30–90 %. The 20 % band is
//! stiction-limited, so its result is logged for information only; see
//! `measure::PASS_MIN_POWER`.
//!
//! The pass criterion is a maximum divergence below 5 % across the gated powers.
//! The harness logs exactly one final line, `bench-precision: max=<pct> PASS` or
//! `bench-precision: max=<pct> FAIL`, which `scripts/motor-bench-precision.sh`
//! greps.
//!
//! The regression stops at 90 %: both tracks at 100 % need a supply rated for
//! **≥ 2 A**. A full-power variant can add `100` to `measure::BENCH_POWERS` once
//! the bench supply meets that.
//!
//! This is a bench diagnostic, not a `#[cfg(test)]` test: it needs the real board,
//! motors, encoders and a current-limited supply.
//!
//! When both features are enabled (for example under `cargo --all-features`), the
//! calibrate phase takes precedence and the measure phase is not compiled; enable
//! exactly one feature explicitly for a bench run.

mod shared;

#[cfg(feature = "bench_motor_calibrate")]
mod calibrate;
#[cfg(all(feature = "bench_motor_measure", not(feature = "bench_motor_calibrate")))]
mod measure;

#[cfg(feature = "bench_motor_calibrate")]
pub use calibrate::init;
#[cfg(all(feature = "bench_motor_measure", not(feature = "bench_motor_calibrate")))]
pub use measure::init;
