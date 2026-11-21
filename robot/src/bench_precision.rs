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
//!   and runs the corrected-divergence regression below. It logs
//!   `bench-precision: ...` lines and one final `bench-precision: max=<pct>
//!   PASS|FAIL`.
//!
//! The regression commands both tracks at the same requested power through
//! [`MotorCommand::SetTracks`], so the Motor Calibration Curve is applied just as
//! it is in normal driving. For each direction and each power it brakes, waits for
//! **Quiescence**, resets both counters, discards a 500 ms settle interval, and
//! measures both encoder deltas over 2 000 ms. The divergence is
//! `|left − right| / min(left, right)`.
//!
//! It measures 20–90 %, but gates the pass at 30–90 %. The 20 % band is
//! stiction-limited, so its result is logged for information only; see
//! [`PASS_MIN_POWER`].
//!
//! The pass criterion is a maximum divergence below 5 % across the gated powers.
//! The harness logs exactly one final line, `bench-precision: max=<pct> PASS` or
//! `bench-precision: max=<pct> FAIL`, which [`scripts/motor-bench-precision.sh`]
//! greps.
//!
//! The regression stops at 90 %: both tracks at 100 % need a supply rated for
//! **≥ 2 A**. A full-power variant can add `100` to [`BENCH_POWERS`] once the
//! bench supply meets that.
//!
//! This is a bench diagnostic, not a `#[cfg(test)]` test: it needs the real board,
//! motors, encoders and a current-limited supply.

#[cfg(all(feature = "bench_motor_calibrate", feature = "bench_motor_measure"))]
compile_error!("enable only one of `bench_motor_calibrate` or `bench_motor_measure`");

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

#[cfg(feature = "bench_motor_calibrate")]
use embassy_time::Instant;

use crate::system::state::calibration;

#[cfg(feature = "bench_motor_calibrate")]
use touch_ui::Procedure;

#[cfg(feature = "bench_motor_calibrate")]
use crate::{
    system::state::activity::{self, Stage},
    task::procedure::{self, ClaimOrigin, StartOutcome},
};

// Both phases share the driver command path and the raw sweep schedule for the
// warm-up, so these are needed whenever either feature is on.
#[cfg(any(feature = "bench_motor_calibrate", feature = "bench_motor_measure"))]
use crate::task::{
    motor_driver::{self, MotorCommand},
    motor_sweep,
};

#[cfg(feature = "bench_motor_measure")]
use motor_cal::{Direction, MEASUREMENT_WINDOW_MS, QUIESCENCE_HOLD_MS, QUIESCENCE_TIMEOUT_MS, SETTLE_MS};

#[cfg(feature = "bench_motor_measure")]
use crate::task::{
    drive::{clear_encoder_measurement, get_latest_encoder_measurement},
    motor_sweep::{
        DIRECTIONS, NO_SAMPLE_TIMEOUT_MS, SAMPLE_INTERVAL_MS, now_ms, signed_speed, start_encoders, stop_encoders,
    },
    sensors::encoders::{self, EncoderCommand, EncoderMeasurement},
};

// ── Spawn ───────────────────────────────────────────────────────────────────────

/// Spawn the calibrate-phase bench task.
#[cfg(feature = "bench_motor_calibrate")]
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(bench_calibrate_task().unwrap());
}

/// Spawn the measure-phase bench task.
#[cfg(all(feature = "bench_motor_measure", not(feature = "bench_motor_calibrate")))]
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(bench_measure_task().unwrap());
}

/// Wait for the boot calibration load to publish its first result.
async fn wait_for_initialization() {
    while !calibration::is_initialized().await {
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Let the boot flow release its claim and the panel reach the menu.
async fn settle_at_menu() {
    Timer::after(Duration::from_secs(3)).await;
}

/// Sleep forever so the finished bench boot keeps its final screen.
async fn hold() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

// ── Warm-up ─────────────────────────────────────────────────────────────────────

/// How long each warm-up step dwells before the next, in milliseconds.
#[cfg(any(feature = "bench_motor_calibrate", feature = "bench_motor_measure"))]
const WARMUP_DWELL_MS: u64 = 2500;

/// Run a discarded 20-step raw pair sweep so both boot phases measure warm.
///
/// This mirrors the load of the Basic Motor Test's sweep (forward and reverse, 10
/// to 100 %) for `WARMUP_DWELL_MS × 20 ≈ 50 s`, but it is discarded: nothing here
/// feeds the calibration or the regression. It commands raw `SetTracksRaw` on
/// purpose, so no curve is applied — the point is to load the drivetrain, not to
/// hit a corrected speed. The pair 100 % step commands both tracks at once, so the
/// warm-up needs the ≥ 2 A supply noted for the full sweep.
#[cfg(any(feature = "bench_motor_calibrate", feature = "bench_motor_measure"))]
async fn warm_up() {
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;

    for direction in motor_sweep::DIRECTIONS {
        for power in motor_sweep::STEP_POWERS {
            let signed = motor_sweep::signed_speed(direction, power);
            motor_driver::send_motor_command(MotorCommand::SetTracksRaw {
                left_speed: signed,
                right_speed: signed,
            })
            .await;
            info!(
                "bench-warmup: dir={=str} p={=u8}",
                motor_sweep::direction_label(direction),
                power
            );
            Timer::after(Duration::from_millis(WARMUP_DWELL_MS)).await;
        }
    }

    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
    info!("bench-warmup: complete");
}

// ── Phase 1: calibrate ──────────────────────────────────────────────────────────

/// How long to wait for the dispatched calibration to finish.
#[cfg(feature = "bench_motor_calibrate")]
const PROCEDURE_TIMEOUT_MS: u64 = 240_000;

/// Dispatch the Motor Calibration, wait for its terminal stage, then hold.
///
/// The calibration's `release_idle` stops the motors before the Result Report is
/// published, so this boot never shows a report while anything drives. The
/// precision pass is the separate `bench_motor_measure` firmware.
#[cfg(feature = "bench_motor_calibrate")]
#[embassy_executor::task]
async fn bench_calibrate_task() {
    wait_for_initialization().await;
    settle_at_menu().await;
    warm_up().await;

    info!(
        "bench-calibrate: dispatching {=str}",
        Procedure::MotorCalibration.label()
    );
    match run_calibration().await {
        Some(Stage::Complete) => info!("bench-calibrate: calibration saved"),
        Some(Stage::Failed) => {
            let detail = activity::snapshot().await.detail;
            info!("bench-calibrate: calibration failed: {=str}", detail);
        }
        Some(Stage::Running) | None => info!("bench-calibrate: calibration failed: did not complete"),
    }

    hold().await;
}

/// Dispatch the Motor Calibration from the panel origin and wait for its verdict.
#[cfg(feature = "bench_motor_calibrate")]
async fn run_calibration() -> Option<Stage> {
    let mut attempts = 0;
    loop {
        match procedure::dispatch(Procedure::MotorCalibration, ClaimOrigin::Panel).await {
            StartOutcome::Started(_) => break,
            StartOutcome::Busy => {
                if attempts >= 20 {
                    info!("bench-calibrate: not started: busy");
                    return None;
                }
                attempts += 1;
                Timer::after(Duration::from_millis(100)).await;
            }
            StartOutcome::Rejected => {
                info!("bench-calibrate: not started: rejected");
                return None;
            }
        }
    }
    wait_for_terminal(Procedure::MotorCalibration).await
}

/// Wait until `procedure` records `Complete` or `Failed`, or the timeout expires.
#[cfg(feature = "bench_motor_calibrate")]
async fn wait_for_terminal(procedure: Procedure) -> Option<Stage> {
    let deadline = Instant::now() + Duration::from_millis(PROCEDURE_TIMEOUT_MS);
    loop {
        let snapshot = activity::snapshot().await;
        if snapshot.procedure() == Some(procedure) {
            match snapshot.stage {
                Stage::Complete | Stage::Failed => return Some(snapshot.stage),
                Stage::Running => {}
            }
        }
        if Instant::now() >= deadline {
            return None;
        }
        Timer::after(Duration::from_millis(200)).await;
    }
}

// ── Phase 2: measure ────────────────────────────────────────────────────────────

/// The corrected-divergence pass threshold, in percent.
#[cfg(feature = "bench_motor_measure")]
const PASS_DIVERGENCE_PERCENT: f32 = 5.0;

/// Repeats per direction and power.
#[cfg(feature = "bench_motor_measure")]
const REPEATS: u8 = 2;

/// The powers the corrected-divergence regression measures.
///
/// 100 % is deliberately absent: both tracks at 100 % need a bench supply rated
/// for at least 2 A. Add `100` only with such a supply.
#[cfg(feature = "bench_motor_measure")]
const BENCH_POWERS: [u8; 8] = [20, 30, 40, 50, 60, 70, 80, 90];

/// The lowest power that gates the corrected-divergence pass.
///
/// 20 % is the stiction band: forward 10 % produces zero counts, the breakaway
/// floor is 20 %, and the 20 % left/right ratio changes sign from run to run. A
/// point there cannot distinguish a calibration error from breakaway noise, so
/// 20 % rows are still measured and logged for diagnosis but do not gate PASS.
#[cfg(feature = "bench_motor_measure")]
const PASS_MIN_POWER: u8 = 30;

/// Run the corrected-divergence regression from the menu, then hold.
///
/// The menu is idle in this boot: no Procedure or Result Report is active, so
/// every visible motor command here has a clear cause.
#[cfg(all(feature = "bench_motor_measure", not(feature = "bench_motor_calibrate")))]
#[embassy_executor::task]
async fn bench_measure_task() {
    wait_for_initialization().await;
    settle_at_menu().await;
    warm_up().await;

    info!("bench-precision: starting corrected-divergence regression");
    run_regression().await;
    hold().await;
}

/// One measured corrected pair at one direction and power.
#[cfg(feature = "bench_motor_measure")]
struct PointReading {
    /// The left track's encoder delta over the Measurement Window.
    left: u16,
    /// The right track's encoder delta over the Measurement Window.
    right: u16,
    /// `|left − right| / min(left, right)`, in percent.
    divergence: f32,
}

/// Measure every direction and power, logging each repeat and the final maximum.
#[cfg(feature = "bench_motor_measure")]
async fn run_regression() {
    start_encoders().await;

    let mut max_divergence: Option<f32> = None;
    let mut low_power_max: Option<f32> = None;
    for direction in DIRECTIONS {
        for power in BENCH_POWERS {
            for repeat in 1..=REPEATS {
                if let Some(reading) = measure_point(direction, power).await {
                    info!(
                        "bench-precision: dir={=str} p={=u8} rep={=u8} L={=u16} R={=u16} div={=f32}",
                        motor_sweep::direction_label(direction),
                        power,
                        repeat,
                        reading.left,
                        reading.right,
                        reading.divergence
                    );
                    let observed = reading.divergence;
                    if power >= PASS_MIN_POWER {
                        max_divergence = Some(max_divergence.map_or(observed, |max| max.max(observed)));
                    } else {
                        low_power_max = Some(low_power_max.map_or(observed, |max| max.max(observed)));
                    }
                } else {
                    info!(
                        "bench-precision: dir={=str} p={=u8} rep={=u8} measurement failed",
                        motor_sweep::direction_label(direction),
                        power,
                        repeat
                    );
                    if power >= PASS_MIN_POWER {
                        // A missing gated point cannot pass; keep the worst
                        // possible value so the final line fails visibly.
                        max_divergence = Some(100.0);
                    }
                }
            }
        }
    }

    stop_encoders().await;

    if let Some(low_power_max) = low_power_max {
        info!(
            "bench-precision: low-power-max={=f32} (informational, below {=u8}%)",
            low_power_max, PASS_MIN_POWER
        );
    }

    // A run that measured nothing cannot pass; report the worst possible value.
    let max = max_divergence.unwrap_or(100.0);
    if max < PASS_DIVERGENCE_PERCENT {
        info!("bench-precision: max={=f32} PASS", max);
    } else {
        info!("bench-precision: max={=f32} FAIL", max);
    }
}

/// Brake, quiesce, reset both counters, command the curve-applied pair, and
/// measure both deltas over the settle-excluded window.
#[cfg(feature = "bench_motor_measure")]
async fn measure_point(direction: Direction, power: u8) -> Option<PointReading> {
    if !quiesce().await {
        return None;
    }

    encoders::send_command(EncoderCommand::Reset).await;
    // This reset only keeps the counters small. The measurement below is a delta
    // taken from a sample after `SETTLE_MS`, so it does not depend on the encoder
    // task confirming the reset. The sweep must confirm a fresh zero because it
    // attributes counts to a command; here quiescence plus the delta is enough.
    Timer::after(Duration::from_millis(2 * SAMPLE_INTERVAL_MS)).await;
    clear_encoder_measurement().await;

    // Both tracks at the same requested power, through the normal command so the
    // Motor Calibration Curve is applied.
    let signed = signed_speed(direction, power);
    motor_driver::send_motor_command(MotorCommand::SetTracks {
        left_speed: signed,
        right_speed: signed,
    })
    .await;
    let command_ms = now_ms();

    let start = wait_sample_at_or_after(command_ms.saturating_add(SETTLE_MS)).await?;
    let end = wait_sample_at_or_after(start.timestamp_ms.saturating_add(MEASUREMENT_WINDOW_MS)).await?;

    // Stop the wheels as soon as the window closes; the next point starts braked.
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let left = end.left.wrapping_sub(start.left);
    let right = end.right.wrapping_sub(start.right);
    Some(PointReading {
        left,
        right,
        divergence: divergence_percent(left, right),
    })
}

/// Brake both tracks and wait until neither counter has changed for the hold
/// window.
#[cfg(feature = "bench_motor_measure")]
async fn quiesce() -> bool {
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let started_ms = now_ms();
    let mut last_timestamp: Option<u64> = None;
    let mut last_fresh_ms = started_ms;
    let mut previous: Option<(u16, u16)> = None;
    let mut last_change_ms = started_ms;

    loop {
        Timer::after(Duration::from_millis(SAMPLE_INTERVAL_MS)).await;

        let Some(measurement) = get_latest_encoder_measurement().await else {
            if now_ms().saturating_sub(last_fresh_ms) >= NO_SAMPLE_TIMEOUT_MS {
                return false;
            }
            continue;
        };
        if last_timestamp == Some(measurement.timestamp_ms) {
            if now_ms().saturating_sub(last_fresh_ms) >= NO_SAMPLE_TIMEOUT_MS {
                return false;
            }
            continue;
        }
        last_timestamp = Some(measurement.timestamp_ms);
        last_fresh_ms = now_ms();

        let counts = (measurement.left, measurement.right);
        if previous != Some(counts) {
            previous = Some(counts);
            last_change_ms = measurement.timestamp_ms;
        }
        if measurement.timestamp_ms.saturating_sub(last_change_ms) >= QUIESCENCE_HOLD_MS {
            return true;
        }
        if measurement.timestamp_ms.saturating_sub(started_ms) >= QUIESCENCE_TIMEOUT_MS {
            return false;
        }
    }
}

/// Wait for a fresh sample at or after `threshold_ms`, or give up on a dead
/// encoder task.
#[cfg(feature = "bench_motor_measure")]
async fn wait_sample_at_or_after(threshold_ms: u64) -> Option<EncoderMeasurement> {
    let started_ms = now_ms();
    let mut last_timestamp: Option<u64> = None;
    let mut last_fresh_ms = started_ms;

    loop {
        Timer::after(Duration::from_millis(SAMPLE_INTERVAL_MS)).await;

        let Some(measurement) = get_latest_encoder_measurement().await else {
            if now_ms().saturating_sub(last_fresh_ms) >= NO_SAMPLE_TIMEOUT_MS {
                return None;
            }
            continue;
        };
        if last_timestamp == Some(measurement.timestamp_ms) {
            if now_ms().saturating_sub(last_fresh_ms) >= NO_SAMPLE_TIMEOUT_MS {
                return None;
            }
            continue;
        }
        last_timestamp = Some(measurement.timestamp_ms);
        last_fresh_ms = now_ms();

        if measurement.timestamp_ms >= threshold_ms {
            return Some(measurement);
        }
    }
}

/// `|left − right| / min(left, right)`, in percent.
///
/// A pair with no counts cannot be compared; it reports 100 % so the run fails
/// visibly rather than passing on missing data.
#[cfg(feature = "bench_motor_measure")]
fn divergence_percent(left: u16, right: u16) -> f32 {
    let min = left.min(right);
    if min == 0 {
        return 100.0;
    }
    (f32::from(left.abs_diff(right)) / f32::from(min)) * 100.0
}
