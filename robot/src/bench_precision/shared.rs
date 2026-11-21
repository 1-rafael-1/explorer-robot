//! Boot-phase helpers shared by the calibrate and measure bench phases.

use defmt::info;
use embassy_time::{Duration, Timer};

use crate::{
    system::state::calibration,
    task::{
        motor_driver::{self, MotorCommand},
        motor_sweep,
    },
};

/// Wait for the boot calibration load to publish its first result.
pub(super) async fn wait_for_initialization() {
    while !calibration::is_initialized().await {
        Timer::after(Duration::from_millis(500)).await;
    }
}

/// Let the boot flow release its claim and the panel reach the menu.
pub(super) async fn settle_at_menu() {
    Timer::after(Duration::from_secs(3)).await;
}

/// Sleep forever so the finished bench boot keeps its final screen.
pub(super) async fn hold() -> ! {
    loop {
        Timer::after(Duration::from_secs(3600)).await;
    }
}

// ── Warm-up ─────────────────────────────────────────────────────────────────────

/// How long each warm-up step dwells before the next, in milliseconds.
pub(super) const WARMUP_DWELL_MS: u64 = 2500;

/// Run a discarded 20-step raw pair sweep so both boot phases measure warm.
///
/// This mirrors the load of the Basic Motor Test's sweep (forward and reverse, 10
/// to 100 %) for `WARMUP_DWELL_MS × 20 ≈ 50 s`, but it is discarded: nothing here
/// feeds the calibration or the regression. It commands raw `SetTracksRaw` on
/// purpose, so no curve is applied — the point is to load the drivetrain, not to
/// hit a corrected speed. The pair 100 % step commands both tracks at once, so the
/// warm-up needs the ≥ 2 A supply noted for the full sweep.
pub(super) async fn warm_up() {
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
