//! Phase 1: dispatch the Motor Calibration from the panel origin and hold on its
//! Result Report.

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use touch_ui::Procedure;

use super::shared::{hold, settle_at_menu, wait_for_initialization, warm_up};
use crate::{
    system::state::activity::{self, Stage},
    task::procedure::{self, ClaimOrigin, StartOutcome},
};

// ── Spawn ───────────────────────────────────────────────────────────────────────

/// Spawn the calibrate-phase bench task.
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(bench_calibrate_task().unwrap());
}

/// How long to wait for the dispatched calibration to finish.
const PROCEDURE_TIMEOUT_MS: u64 = 240_000;

/// Dispatch the Motor Calibration, wait for its terminal stage, then hold.
///
/// The calibration's `release_idle` stops the motors before the Result Report is
/// published, so this boot never shows a report while anything drives. The
/// precision pass is the separate `bench_motor_measure` firmware.
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
