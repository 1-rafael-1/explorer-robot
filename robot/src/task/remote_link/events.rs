//! The curated Event bridge from the system event bus to the Link Task.
//!
//! The orchestrator is the bus's only consumer. [`forward_event`] maps every
//! [`Events`] variant to either a wire-relevant [`LinkedEvent`] or an explicit
//! "keep internal" decision, then hands it to the Link Task through a bounded
//! channel with a non-blocking send and a dropped counter. The exhaustive match
//! is ADR-0031's compile-time drift rule: adding a bus variant fails the build
//! until the author decides whether it is exposed.
//!
//! The Link Task calls [`drain`] on every loop to publish accepted Events into
//! [`LinkCore`], which owns the encoded Event ring and its retransmission. This
//! module keeps no second store: an Event is either queued in the core's ring or
//! dropped and counted here.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel};
use link_core::LinkCore;
use link_protocol::{
    CalibrationKind as WireCalibrationKind, CalibrationLoadStatus, CalibrationLoaded, EventMessage, FloorDrop,
    OBSTACLE_SOURCE_LIDAR, Obstacle, ProcedureCompleted,
    payload::{
        event::{CompletionStatus as WireCompletionStatus, DriveDistanceCompleted, RotateCompleted},
        stream::ActivityStage,
    },
};
use touch_ui::Procedure;

use super::streams::procedure_id;
use crate::{
    system::{
        event::{Events, ObstacleSource},
        state::activity::Stage,
    },
    task::io::flash_storage::CalibrationKind as FlashCalibrationKind,
};

// ── Curated vocabulary ────────────────────────────────────────────────────────

/// One curated Event, carrying only the data the wire needs.
///
/// It is `Copy` so forwarding never allocates, and the detail is `'static`
/// because the bus's terminal outcome already owns it for the process.
#[derive(Clone, Copy, Debug)]
pub enum LinkedEvent {
    /// The `LiDAR` obstacle flag changed edge.
    Obstacle {
        /// The flag's new value.
        detected: bool,
    },
    /// The floor-drop flag changed edge.
    FloorDrop {
        /// The flag's new value.
        detected: bool,
    },
    /// A Procedure reached a terminal outcome.
    ProcedureCompleted {
        /// The Procedure that finished.
        procedure: Procedure,
        /// Its terminal stage.
        stage: Stage,
        /// The operator-facing result line.
        detail: &'static str,
    },
    /// Boot-time calibration data finished loading.
    CalibrationLoaded {
        /// Which calibration data the Event concerns.
        kind: FlashCalibrationKind,
        /// Whether the data was present.
        loaded: bool,
    },
    /// A Remote `DRIVE_DISTANCE` finished.
    DriveDistanceCompleted {
        /// The originating command's sequence number.
        command_seq: u16,
        /// The drive outcome.
        status: WireCompletionStatus,
        /// The requested distance in centimetres; positive is forward.
        target_cm: i16,
        /// The distance actually travelled, signed.
        distance_cm: i32,
        /// The move's duration in milliseconds.
        duration_ms: u32,
        /// The operator-facing result line.
        detail: &'static str,
    },
    /// A Remote `ROTATE_BY` finished.
    RotateCompleted {
        /// The originating command's sequence number.
        command_seq: u16,
        /// The rotation outcome.
        status: WireCompletionStatus,
        /// The requested angle in centidegrees; positive is clockwise.
        target_cdeg: i16,
        /// The signed error, actual minus target, in clockwise centidegrees.
        error_cdeg: i32,
        /// The rotation's duration in milliseconds.
        duration_ms: u32,
        /// The operator-facing result line.
        detail: &'static str,
    },
}

// ── Channel and diagnostics ───────────────────────────────────────────────────

/// The number of accepted Events the channel can hold before dropping.
const EVENT_CAPACITY: usize = 16;

/// Accepted Events waiting for the Link Task.
static EVENT_QUEUE: Channel<CriticalSectionRawMutex, LinkedEvent, EVENT_CAPACITY> = Channel::new();

/// Events discarded because the channel was full.
static EVENT_DROPS: AtomicU32 = AtomicU32::new(0);

/// Number of Events dropped for want of channel room, for diagnostics.
///
/// A dropped Event is still a state change the state modules reflect; only the
/// link's reliable copy is lost. The counter is exposed through the link task as
/// a diagnostic; nothing reads it in the firmware yet.
#[allow(dead_code)] // Diagnostic, exposed through the link task.
#[must_use]
pub fn event_drops() -> u32 {
    EVENT_DROPS.load(Ordering::Relaxed)
}

// ── Producer ──────────────────────────────────────────────────────────────────

/// Map one bus Event onto the curated vocabulary and enqueue it.
///
/// Never blocks: a full channel drops the Event and increments [`event_drops`].
/// The orchestrator calls this before its own local handling, so local behaviour
/// is untouched.
pub fn forward_event(event: &Events) {
    let Some(linked) = map_event(event) else {
        return;
    };
    publish(linked);
}

/// Enqueue one curated Event, dropping it when the channel is full.
///
/// The Remote drive job uses this directly: its completion Events bypass the
/// system Event bus and carry their own originating command sequence
/// (ADR-0031). Publishing never blocks and never allocates.
pub fn publish(event: LinkedEvent) {
    if EVENT_QUEUE.try_send(event).is_err() {
        EVENT_DROPS.fetch_add(1, Ordering::Relaxed);
    }
}

/// The exhaustive bus-to-link decision.
///
/// `None` is the explicit "keep internal" choice. Adding an [`Events`] variant
/// fails the build here until it is mapped.
const fn map_event(event: &Events) -> Option<LinkedEvent> {
    match event {
        Events::Initialize | Events::ObstacleAvoidanceAttempted | Events::BatteryMeasured { .. } => None,
        Events::CalibrationDataLoaded(kind, data) => Some(LinkedEvent::CalibrationLoaded {
            kind: *kind,
            loaded: data.is_some(),
        }),
        Events::ObstacleDetected {
            source: ObstacleSource::Lidar,
            detected,
        } => Some(LinkedEvent::Obstacle { detected: *detected }),
        Events::FloorDropDetected { detected } => Some(LinkedEvent::FloorDrop { detected: *detected }),
        Events::ProcedureFinished {
            procedure,
            stage,
            detail,
            generation: _,
        } => match stage {
            Stage::Complete | Stage::Failed => Some(LinkedEvent::ProcedureCompleted {
                procedure: *procedure,
                stage: *stage,
                detail,
            }),
            // The bus raises terminal outcomes only; a running stage is not a
            // completion and stays internal.
            Stage::Running => None,
        },
    }
}

// ── Consumer ──────────────────────────────────────────────────────────────────

/// Drain the queue into the link core, one Event at a time.
///
/// Publishing never blocks and never allocates. The core's encoded ring owns the
/// retry schedule and survives a Bridge reset, so a retained Event is
/// retransmitted after reconnect and nothing is stored here.
pub fn drain(core: &mut LinkCore, now_ms: u64) {
    while let Ok(event) = EVENT_QUEUE.try_receive() {
        let message = event_message(event);
        let _ = core.publish_event(&message, now_ms);
    }
}

// ── Wire mapping ──────────────────────────────────────────────────────────────

/// Build the wire message for one curated Event.
///
/// The detail is `'static`, so the returned message borrows nothing from the
/// argument and can be handed straight to the core's ring.
fn event_message(event: LinkedEvent) -> EventMessage<'static> {
    match event {
        LinkedEvent::Obstacle { detected } => EventMessage::Obstacle(Obstacle {
            detected: u8::from(detected),
            source: OBSTACLE_SOURCE_LIDAR,
        }),
        LinkedEvent::FloorDrop { detected } => EventMessage::FloorDrop(FloorDrop {
            detected: u8::from(detected),
        }),
        LinkedEvent::ProcedureCompleted {
            procedure,
            stage,
            detail,
        } => EventMessage::ProcedureCompleted(ProcedureCompleted {
            procedure_id: procedure_id(procedure),
            stage: completion_stage(stage),
            detail,
        }),
        LinkedEvent::CalibrationLoaded { kind, loaded } => EventMessage::CalibrationLoaded(CalibrationLoaded {
            kind: wire_calibration_kind(kind),
            status: if loaded {
                CalibrationLoadStatus::Loaded
            } else {
                CalibrationLoadStatus::Missing
            },
        }),
        LinkedEvent::DriveDistanceCompleted {
            command_seq,
            status,
            target_cm,
            distance_cm,
            duration_ms,
            detail,
        } => EventMessage::DriveDistanceCompleted(DriveDistanceCompleted {
            command_seq,
            status,
            target_cm,
            distance_cm,
            duration_ms,
            detail,
        }),
        LinkedEvent::RotateCompleted {
            command_seq,
            status,
            target_cdeg,
            error_cdeg,
            duration_ms,
            detail,
        } => EventMessage::RotateCompleted(RotateCompleted {
            command_seq,
            status,
            target_cdeg,
            error_cdeg,
            duration_ms,
            detail,
        }),
    }
}

/// The wire stage for a terminal Procedure outcome.
///
/// [`Stage::Running`] never arrives from [`Events::ProcedureFinished`]; if a
/// future producer does send one, it is reported as `Failed` rather than
/// claiming a success that did not happen.
#[must_use]
const fn completion_stage(stage: Stage) -> ActivityStage {
    match stage {
        Stage::Complete => ActivityStage::Complete,
        Stage::Failed | Stage::Running => ActivityStage::Failed,
    }
}

/// The wire calibration kind for a firmware calibration kind.
///
/// The DMP Bias Snapshot has no wire kind of its own: the Link Protocol layout
/// is unchanged, so it is reported under the `ImuFlags` kind, which carries the
/// DMP snapshot's presence.
#[must_use]
const fn wire_calibration_kind(kind: FlashCalibrationKind) -> WireCalibrationKind {
    match kind {
        FlashCalibrationKind::Motor => WireCalibrationKind::Motor,
        FlashCalibrationKind::DmpBias => WireCalibrationKind::ImuFlags,
        FlashCalibrationKind::Distance => WireCalibrationKind::Distance,
    }
}
