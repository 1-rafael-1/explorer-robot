//! Host-side tests for the pair state machine and the drive gate.

use rangefinder::{GateRefusal, PairState, SensorState, gate, pair_state};

#[test]
fn disabled_pair_is_off_regardless_of_sensor_states() {
    assert_eq!(
        pair_state(false, SensorState::Armed, SensorState::Armed),
        PairState::Off
    );
    assert_eq!(
        pair_state(false, SensorState::Failed, SensorState::Waiting),
        PairState::Off
    );
}

#[test]
fn both_sensors_armed_is_armed() {
    assert_eq!(
        pair_state(true, SensorState::Armed, SensorState::Armed),
        PairState::Armed
    );
}

#[test]
fn both_sensors_waiting_is_arming() {
    assert_eq!(
        pair_state(true, SensorState::Waiting, SensorState::Waiting),
        PairState::Arming
    );
}

#[test]
fn one_armed_and_one_waiting_is_still_arming() {
    assert_eq!(
        pair_state(true, SensorState::Armed, SensorState::Waiting),
        PairState::Arming
    );
    assert_eq!(
        pair_state(true, SensorState::Waiting, SensorState::Armed),
        PairState::Arming
    );
}

#[test]
fn both_sensors_failed_is_failed() {
    assert_eq!(
        pair_state(true, SensorState::Failed, SensorState::Failed),
        PairState::Failed
    );
}

#[test]
fn a_recovering_or_failed_sensor_degrades_the_pair() {
    let states = [
        SensorState::Waiting,
        SensorState::Armed,
        SensorState::Recovering,
        SensorState::Failed,
    ];

    for front in states {
        for rear in states {
            let expected = match (front, rear) {
                (SensorState::Armed, SensorState::Armed) => PairState::Armed,
                (SensorState::Failed, SensorState::Failed) => PairState::Failed,
                (SensorState::Waiting | SensorState::Armed, SensorState::Waiting)
                | (SensorState::Waiting, SensorState::Armed) => PairState::Arming,
                _ => PairState::Degraded,
            };
            assert_eq!(pair_state(true, front, rear), expected, "{front:?}/{rear:?}");
        }
    }
}

#[test]
fn armed_with_a_clear_flag_passes_the_gate() {
    assert_eq!(gate(PairState::Armed, false), Ok(()));
}

#[test]
fn armed_with_a_floor_drop_is_refused() {
    assert_eq!(gate(PairState::Armed, true), Err(GateRefusal::FloorDrop));
}

#[test]
fn a_floor_drop_is_the_more_specific_refusal_even_when_not_armed() {
    for state in [
        PairState::Off,
        PairState::Arming,
        PairState::Degraded,
        PairState::Failed,
    ] {
        assert_eq!(gate(state, true), Err(GateRefusal::FloorDrop), "{state:?}");
    }
}

#[test]
fn non_armed_states_are_refused_as_not_ready() {
    assert_eq!(gate(PairState::Arming, false), Err(GateRefusal::NotReady));
    assert_eq!(gate(PairState::Degraded, false), Err(GateRefusal::NotReady));
    assert_eq!(gate(PairState::Failed, false), Err(GateRefusal::NotReady));
    assert_eq!(gate(PairState::Off, false), Err(GateRefusal::NotReady));
}

#[test]
fn refusal_reasons_are_exact() {
    assert_eq!(GateRefusal::NotReady.reason(), "floor-drop sensors not ready");
    assert_eq!(GateRefusal::FloorDrop.reason(), "floor drop detected");
}
