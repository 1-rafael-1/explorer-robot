//! The Deadman's robot-side entry point.
//!
//! ADR-0035 gives the robot one failsafe rule: when the peer has been silent for
//! the Deadman threshold, or the Link Port is deliberately disabled, stop the
//! live claim **only if a Remote started it**. The check is the whole reason the
//! claim carries a transient origin: the drive queue is shared with the panel, so
//! an unconditional brake would stop a panel-started Procedure too.
//!
//! Stopping goes through the one stop verb, so the claim owner still observes the
//! latch, cleans up and makes its own terminal call. The Link Task never releases
//! the claim itself.

use crate::task::{
    drive::InterruptKind,
    procedure::{self, ClaimOrigin, StopOutcome},
};

/// The Deadman's one entry: stop a Remote-originated claim with an emergency brake.
///
/// A live Panel-originated claim, and an idle robot, are left untouched: no
/// interrupt is sent and the shared drive queue is not disturbed.
pub fn link_lost() {
    if procedure::live_origin() == Some(ClaimOrigin::Remote) {
        procedure::stop(StopOutcome::Failsafe("Link lost"), InterruptKind::EmergencyBrake);
    }
}
