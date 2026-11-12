//! Host tests for the shared wire ids (ADR-0030, ADR-0031).
//!
//! These guard the catalogue invariants: every implemented variant is in `ALL`, ids are unique and
//! in the core range, and `index`/`from_wire` round-trip.

use std::collections::HashSet;

use link_protocol::ids::{Caps, ProcedureId, Stream};

/// `ALL` covers every variant, and the ids are unique, in range and round-trip.
#[test]
fn all_covers_every_variant_with_unique_in_range_ids() {
    assert_eq!(Stream::ALL.len(), 6, "the v1 catalogue has six streams");

    let mut ids = HashSet::new();
    let mut indices = HashSet::new();
    for stream in Stream::ALL {
        let id = stream.wire_id();
        assert!(
            (0x0001..=0x00FF).contains(&id),
            "{stream:?} must use a core id, got {id:#06x}"
        );
        assert!(ids.insert(id), "{stream:?} must have a unique id");
        assert!(indices.insert(stream.index()), "{stream:?} must have a unique index");
        assert_eq!(Stream::from_wire(id), Some(stream), "from_wire must recover {stream:?}");
        assert_eq!(
            Stream::ALL[stream.index()],
            stream,
            "index must address the stream in ALL"
        );
    }
}

/// Reserved and unassigned ids are unknown rather than a panic or a wrong variant.
#[test]
fn reserved_and_unassigned_ids_are_none() {
    for id in [0x0000, 0x0007, 0x000F, 0x0010, 0x0014, 0x00FF, 0xFFFF] {
        assert_eq!(Stream::from_wire(id), None, "{id:#06x} must be unknown");
    }
}

/// The Procedure ids are the ADR-0031 constants, and an unknown value is tolerated.
#[test]
fn procedure_ids_match_the_catalogue_and_tolerate_unknown_values() {
    let catalogue = [
        (ProcedureId::IDLE, 0x0000),
        (ProcedureId::BOOT, 0x0001),
        (ProcedureId::MOTOR_CALIBRATION, 0x0002),
        (ProcedureId::MAG_CALIBRATION, 0x0003),
        (ProcedureId::DISTANCE_CALIBRATION, 0x0004),
        (ProcedureId::COAST_AND_AVOID, 0x0005),
        (ProcedureId::ATTEMPT_STRAIGHT, 0x0006),
        (ProcedureId::BASIC_MOTOR, 0x0007),
        (ProcedureId::TURNS, 0x0008),
        (ProcedureId::STRAIGHT_DRIVE, 0x0009),
        (ProcedureId::ARC_DRIVE, 0x000A),
        (ProcedureId::IMU_6_AXIS, 0x000B),
        (ProcedureId::IMU_9_AXIS, 0x000C),
        (ProcedureId::DMP_COMPASS_CALIBRATION, 0x000E),
    ];

    let mut seen = HashSet::new();
    for (procedure, wire) in catalogue {
        assert_eq!(procedure.get(), wire, "{procedure:?} must match its constant");
        assert!(seen.insert(procedure.get()), "{procedure:?} must be unique");
    }

    let unknown = ProcedureId::new(0xBEEF);
    assert_eq!(unknown.get(), 0xBEEF, "an unknown Procedure id must survive");
    assert_eq!(ProcedureId::default(), ProcedureId::IDLE, "the default is idle");
}

/// `Caps` keeps unknown bits, intersects by mode and exposes the known modes.
#[test]
fn caps_preserves_unknown_bits_and_intersects() {
    let unknown_bit = 1u32 << 31;
    let caps = Caps::from_bits(Caps::MAP.bits() | Caps::THROTTLE_SUPPORT.bits() | unknown_bit);

    assert!(caps.contains(Caps::MAP), "a set mode must be contained");
    assert!(caps.contains(Caps::THROTTLE_SUPPORT), "a set mode must be contained");
    assert!(!caps.contains(Caps::DELTA_CLOUD), "an unset mode is not contained");
    assert_eq!(caps.bits() & unknown_bit, unknown_bit, "unknown bits must be preserved");
    assert!(
        !Caps::KNOWN.contains(Caps::from_bits(unknown_bit)),
        "an unknown bit is not a known mode"
    );

    let other = Caps::MAP | Caps::POSE_TRAIL;
    assert_eq!(
        caps.intersection(other),
        Caps::MAP,
        "intersection keeps the shared modes only"
    );
    assert!(Caps::from_bits(0).is_empty(), "empty caps must report empty");
}
