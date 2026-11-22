//! Phase 2: the corrected-divergence regression against the saved v4 curve.

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use motor_cal::{Direction, MEASUREMENT_WINDOW_MS, QUIESCENCE_HOLD_MS, QUIESCENCE_TIMEOUT_MS, SETTLE_MS};

use super::shared::{hold, settle_at_menu, wait_for_initialization};
use crate::task::{
    drive::clear_encoder_measurement,
    motor_driver::{self, MotorCommand},
    motor_sweep::{
        self, DIRECTIONS, NO_SAMPLE_TIMEOUT_MS, SAMPLE_INTERVAL_MS, SampleStream, now_ms, signed_speed, start_encoders,
        stop_encoders,
    },
    sensors::encoders::{self, EncoderCommand, EncoderMeasurement},
};

// ── Spawn ───────────────────────────────────────────────────────────────────────

/// Spawn the measure-phase bench task.
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(bench_measure_task().unwrap());
}

// ── Regression ──────────────────────────────────────────────────────────────────

/// The corrected-divergence pass threshold, in percent.
const PASS_DIVERGENCE_PERCENT: f32 = 5.0;

/// Repeats per direction and power.
const REPEATS: u8 = 2;

/// The divergence reported for a point that cannot be measured, in percent.
///
/// A missing or unusable point must still be a visible failing number: the bench
/// script greps the final `bench-precision: max=` line, and a zero-count pair has
/// no ratio to report.
const FAILING_DIVERGENCE_PERCENT: f32 = 100.0;

/// The powers the corrected-divergence regression measures.
///
/// 100 % is deliberately absent: both tracks at 100 % need a bench supply rated
/// for at least 2 A. Add `100` only with such a supply.
const BENCH_POWERS: [u8; 8] = [20, 30, 40, 50, 60, 70, 80, 90];

/// The lowest power that gates the corrected-divergence pass.
///
/// 20 % is the stiction band: forward 10 % produces zero counts, the breakaway
/// floor is 20 %, and the 20 % left/right ratio changes sign from run to run. A
/// point there cannot distinguish a calibration error from breakaway noise, so
/// 20 % rows are still measured and logged for diagnosis but do not gate PASS.
const PASS_MIN_POWER: u8 = 30;

/// Run the corrected-divergence regression from the menu, then hold.
///
/// The menu is idle in this boot: no Procedure or Result Report is active, so
/// every visible motor command here has a clear cause.
#[embassy_executor::task]
async fn bench_measure_task() {
    wait_for_initialization().await;
    settle_at_menu().await;
    let _ = motor_sweep::run_warm_up(None, motor_sweep::SweepProgress::whole(motor_sweep::TOTAL_STEPS)).await;

    info!("bench-precision: starting corrected-divergence regression");
    run_regression().await;
    hold().await;
}

/// One measured corrected pair at one direction and power.
struct PointReading {
    /// The left track's encoder delta over the Measurement Window.
    left: u16,
    /// The right track's encoder delta over the Measurement Window.
    right: u16,
    /// `|left − right| / min(left, right)`, in percent.
    divergence: f32,
}

/// Measure every direction and power, logging each repeat and the final maximum.
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
                        max_divergence = Some(FAILING_DIVERGENCE_PERCENT);
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
    let max = max_divergence.unwrap_or(FAILING_DIVERGENCE_PERCENT);
    if max < PASS_DIVERGENCE_PERCENT {
        info!("bench-precision: max={=f32} PASS", max);
    } else {
        info!("bench-precision: max={=f32} FAIL", max);
    }
}

/// Brake, quiesce, reset both counters, command the curve-applied pair, and
/// measure both deltas over the settle-excluded window.
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
        divergence: motor_cal::divergence_percent(left, right).unwrap_or(FAILING_DIVERGENCE_PERCENT),
    })
}

/// Brake both tracks and wait until neither counter has changed for the hold
/// window.
async fn quiesce() -> bool {
    motor_driver::send_motor_command(MotorCommand::BrakeAll).await;

    let started_ms = now_ms();
    let mut stream = SampleStream::new(started_ms);
    let mut previous: Option<(u16, u16)> = None;
    let mut last_change_ms = started_ms;

    loop {
        Timer::after(Duration::from_millis(SAMPLE_INTERVAL_MS)).await;

        let Some(measurement) = stream.next().await else {
            if stream.dead(now_ms(), NO_SAMPLE_TIMEOUT_MS) {
                return false;
            }
            continue;
        };

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
async fn wait_sample_at_or_after(threshold_ms: u64) -> Option<EncoderMeasurement> {
    let mut stream = SampleStream::new(now_ms());

    loop {
        Timer::after(Duration::from_millis(SAMPLE_INTERVAL_MS)).await;

        let Some(measurement) = stream.next().await else {
            if stream.dead(now_ms(), NO_SAMPLE_TIMEOUT_MS) {
                return None;
            }
            continue;
        };

        if measurement.timestamp_ms >= threshold_ms {
            return Some(measurement);
        }
    }
}
