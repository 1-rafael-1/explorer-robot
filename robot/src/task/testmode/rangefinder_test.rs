//! Panel-only Rangefinder Test task.
//!
//! The task arms the pair, streams the typed [`touch_ui::RangefinderTestReadout`]
//! from the rangefinder telemetry at 10 Hz, and stays open across faults and
//! recovery until the operator presses Stop. It never writes Activity State or
//! the Link Protocol; the readout lives in its own state leaf.

use defmt::error;
use embassy_executor::Spawner;
use touch_ui::{FloorState as UiFloorState, ProtectionState, RangefinderTestReadout, SensorReadout};

use crate::{
    system::state::{rangefinder as rangefinder_state, rangefinder_test},
    task::{procedure::Claim, sensors::rangefinder as pair},
};

/// Milliseconds between readout publishes: 10 Hz, from the driver crate's
/// documented publish rate.
#[allow(clippy::cast_lossless)] // `u64::from` is not const-stable in the pinned toolchain.
const PUBLISH_INTERVAL_MS: u64 = 1000 / rangefinder::TEST_PUBLISH_HZ as u64;

/// Spawn the Rangefinder Test task via the controller.
///
/// The caller's claim travels with the task; a spawn that fails releases it so a
/// dead task does not hold the slot.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    rangefinder_test_task(claim).map_or_else(
        |_| {
            error!("rangefinder test: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// Rangefinder Test task: arms the pair and streams both sensors until stopped.
#[embassy_executor::task]
async fn rangefinder_test_task(claim: Claim) {
    rangefinder_test::clear().await;
    pair::enable().await;
    claim.phase("Rangefinder test", None).await;

    while !claim.wait_or_stop(PUBLISH_INTERVAL_MS).await {
        let readout = rangefinder_state::snapshot().await.map_or_else(
            || RangefinderTestReadout::waiting(rangefinder::DROP_THRESHOLD_MM),
            to_readout,
        );
        rangefinder_test::publish(readout).await;
    }

    pair::disable().await;
    rangefinder_test::clear().await;
    claim.finish().await;
}

/// Map the pair telemetry onto the panel's neutral readout.
const fn to_readout(snapshot: rangefinder_state::RangefinderTelemetry) -> RangefinderTestReadout {
    RangefinderTestReadout {
        front: sensor_readout(snapshot.front),
        rear: sensor_readout(snapshot.rear),
        pair: Some(pair_state(snapshot.pair)),
        floor_drop: Some(snapshot.floor_drop),
        threshold_mm: snapshot.threshold_mm,
    }
}

/// Map one sensor's telemetry onto the panel's neutral per-sensor readout.
const fn sensor_readout(sensor: rangefinder_state::SensorTelemetry) -> SensorReadout {
    SensorReadout {
        state: Some(sensor_state(sensor.state)),
        range_mm: sensor.range_mm,
        raw_status: sensor.raw_status,
        floor: Some(floor_state(sensor.floor)),
        age_ms: sensor.age_ms,
    }
}

/// Map the driver's pair state onto the panel's Protection state.
const fn pair_state(state: rangefinder::PairState) -> ProtectionState {
    match state {
        rangefinder::PairState::Off => ProtectionState::Off,
        rangefinder::PairState::Arming => ProtectionState::Arming,
        rangefinder::PairState::Armed => ProtectionState::Armed,
        rangefinder::PairState::Degraded => ProtectionState::Degraded,
        rangefinder::PairState::Failed => ProtectionState::Failed,
    }
}

/// Map one sensor's pair state onto the panel's Protection state.
const fn sensor_state(state: rangefinder::SensorState) -> ProtectionState {
    match state {
        rangefinder::SensorState::Waiting => ProtectionState::Arming,
        rangefinder::SensorState::Armed => ProtectionState::Armed,
        rangefinder::SensorState::Recovering => ProtectionState::Degraded,
        rangefinder::SensorState::Failed => ProtectionState::Failed,
    }
}

/// Map the driver's floor state onto the panel's floor state.
const fn floor_state(state: rangefinder::FloorState) -> UiFloorState {
    match state {
        rangefinder::FloorState::Unknown => UiFloorState::Unknown,
        rangefinder::FloorState::Floor => UiFloorState::Floor,
        rangefinder::FloorState::Drop => UiFloorState::Drop,
    }
}
