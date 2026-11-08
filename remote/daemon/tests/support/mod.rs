//! Shared fixtures for the relay's host tests.
//!
//! The relay is driven exactly as the two adapters will drive it: an in-memory app connection
//! (`connect_app`/`poll`) and a fake link (the `on_*` calls). Every helper fails a test rather
//! than unwrapping, because the workspace denies `unwrap` and `expect` in tests too.

#![allow(dead_code)]

use link_protocol::{
    ids::ProcedureId,
    payload::{
        event::{EventOverflow, FloorDrop, Obstacle},
        stream::{
            Activity, ActivityStage, Calibration, CalibrationStatus as WireCalibrationStatus, LidarCloud, Motion,
            Perception, Power,
        },
    },
};
use remote_daemon::{
    relay::Relay,
    schema::{CommandFrame, CommandResult, EventEntry, EventName, ServerMessage, Snapshot, StreamEntry, StreamName},
};

/// Extracts a value, failing the test if it is absent.
pub fn must<T>(value: Option<T>, what: &str) -> T {
    value.unwrap_or_else(|| {
        assert_eq!(what.len(), usize::MAX, "missing {what}");
        std::process::abort()
    })
}

/// Opens a Session and registers one app connection.
pub fn session_with_app(relay: &mut Relay, boot_id: u32, now_ms: u64) -> u64 {
    relay.session_up(boot_id, now_ms);
    relay.connect_app(now_ms)
}

/// Builds a `POWER` sample.
pub const fn power(level: u8, voltage_mv: u16) -> Power {
    Power { level, voltage_mv }
}

/// Builds a `MOTION` sample.
pub const fn motion(left: i8, right: i8) -> Motion {
    Motion { left, right }
}

/// Builds an `ACTIVITY` sample.
pub const fn activity(id: u16, stage: ActivityStage, percent: Option<u8>, detail: &'static str) -> Activity<'static> {
    Activity {
        procedure_id: ProcedureId::new(id),
        stage,
        percent,
        detail,
    }
}

/// Builds a `CALIBRATION` sample.
pub const fn calibration(
    motor: WireCalibrationStatus,
    imu: WireCalibrationStatus,
    mag: WireCalibrationStatus,
    distance: WireCalibrationStatus,
) -> Calibration {
    Calibration {
        motor,
        imu,
        mag,
        distance,
    }
}

/// Builds a `PERCEPTION` sample from its wire bits.
pub const fn perception(bits: u8) -> Perception {
    Perception::from_bits(bits)
}

/// Builds a 360-slot cloud with one no-return slot and one ordinary range.
pub const fn lidar() -> LidarCloud {
    let mut cloud = LidarCloud { slots: [100; 360] };
    cloud.slots[0] = LidarCloud::NO_RETURN;
    cloud.slots[1] = 42;
    cloud
}

/// Builds an `OBSTACLE` event payload.
pub const fn obstacle(detected: u8, source: u8) -> Obstacle {
    Obstacle { detected, source }
}

/// Builds a `FLOOR_DROP` event payload.
pub const fn floor_drop(detected: u8) -> FloorDrop {
    FloorDrop { detected }
}

/// Builds an `EVENT_OVERFLOW` payload.
pub const fn overflow(dropped: u16) -> EventOverflow {
    EventOverflow { dropped }
}

/// Builds an app `command` frame.
pub fn frame(command_seq: u64, command: &str, args: serde_json::Value) -> CommandFrame {
    CommandFrame {
        command_seq,
        command: command.to_owned(),
        args,
    }
}

/// Returns the snapshot message from a poll, if any.
pub fn snapshot(messages: &[ServerMessage]) -> Option<Snapshot> {
    messages.iter().find_map(|message| match message {
        ServerMessage::Snapshot(snapshot) => Some(snapshot.clone()),
        _ => None,
    })
}

/// Returns every stream message from a poll.
pub fn streams(messages: &[ServerMessage]) -> Vec<StreamEntry> {
    messages
        .iter()
        .filter_map(|message| match message {
            ServerMessage::Stream(entry) => Some(entry.clone()),
            _ => None,
        })
        .collect()
}

/// Returns every event message from a poll.
pub fn events(messages: &[ServerMessage]) -> Vec<EventEntry> {
    messages
        .iter()
        .filter_map(|message| match message {
            ServerMessage::Event(entry) => Some(entry.clone()),
            _ => None,
        })
        .collect()
}

/// Returns every command result from a poll.
pub fn results(messages: &[ServerMessage]) -> Vec<CommandResult> {
    messages
        .iter()
        .filter_map(|message| match message {
            ServerMessage::CommandResult(result) => Some(result.clone()),
            _ => None,
        })
        .collect()
}

/// Returns the stream message with the given name from a poll.
pub fn stream_named(messages: &[ServerMessage], name: StreamName) -> Option<StreamEntry> {
    streams(messages).into_iter().find(|entry| entry.name == name)
}

/// Returns the event message with the given name from a poll.
pub fn event_named(messages: &[ServerMessage], name: EventName) -> Option<EventEntry> {
    events(messages).into_iter().find(|entry| entry.name == name)
}

/// Returns the stream entry with the given name from a snapshot's streams.
pub fn entry(entries: &[StreamEntry], name: StreamName) -> Option<StreamEntry> {
    entries.iter().find(|entry| entry.name == name).cloned()
}
