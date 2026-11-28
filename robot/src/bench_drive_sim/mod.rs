//! Permanent bench diagnostic for the `drive-core` model.
//!
//! This module is compiled only with the non-default `bench_drive_sim` feature.
//! It runs the production [`drive_core::Intent`] against scripted synthetic
//! observations and a recording action executor that mirrors the adapter's
//! action set. It issues **no** motor commands and touches **no** sensors: it is
//! an integration check of the real compiled model, not a motion test.
//!
//! Every scenario mirrors a `drive-core` host worked example. The bench logs one
//! `drive-sim: <name> PASS` / `drive-sim: <name> FAIL <detail>` line per
//! scenario and ends with `drive-sim: PASS` or `drive-sim: FAIL scenario=<name>`.
//! `scripts/drive-bench-sim.sh` captures the RTT lines and fails on the first
//! failure.

mod scenarios;

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

/// Settle before the first scenario so the boot flow and RTT logger are live.
const SETTLE_MS: u64 = 3_000;

/// Spawn the drive-simulation bench task.
#[allow(clippy::unwrap_used)]
pub fn init(spawner: Spawner) {
    spawner.spawn(bench_drive_sim_task().unwrap());
}

/// Run every scenario, then hold so the final screen and log stay visible.
#[embassy_executor::task]
async fn bench_drive_sim_task() {
    Timer::after(Duration::from_millis(SETTLE_MS)).await;
    info!("drive-sim: starting drive-core scenarios");

    let mut first_failure: Option<&'static str> = None;
    for scenario in scenarios::SCENARIOS {
        match (scenario.run)() {
            Ok(()) => info!("drive-sim: {=str} PASS", scenario.name),
            Err(detail) => {
                info!("drive-sim: {=str} FAIL {=str}", scenario.name, detail);
                if first_failure.is_none() {
                    first_failure = Some(scenario.name);
                }
            }
        }
    }

    match first_failure {
        None => info!("drive-sim: PASS"),
        Some(name) => info!("drive-sim: FAIL scenario={=str}", name),
    }

    loop {
        Timer::after(Duration::from_secs(3_600)).await;
    }
}
