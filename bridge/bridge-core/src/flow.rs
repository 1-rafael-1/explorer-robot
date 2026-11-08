//! The watermark to `THROTTLE` policy (ADR-0030, ADR-0033).
//!
//! The Bridge is authoritative for UART overflow. When its robot-facing ring crosses the high
//! watermark it tells the robot to pause every stream; while the ring stays above the low watermark
//! it refreshes that pause, and when both low watermarks are met it resumes them. The hint is
//! advisory and time-limited, and two hints are kept at least 500 ms apart so a ring hovering at a
//! watermark cannot flood the robot with control frames.
//!
//! Pressure is measured both in bytes and in queued entries. Bytes alone are not enough: the ring
//! has a 16-entry descriptor table, and the robot's worst-case mix (six coalesced streams plus many
//! small reliable events) fills that table at roughly 2–3 KiB, long before the 6 KiB byte mark.
//! Without the entry watermark the ring would drop frames and never ask the robot to slow down.
//!
//! The policy owns no ring and no clock: the caller passes an [`Occupancy`] and a monotonic
//! millisecond timestamp, and encodes the returned hint as a `THROTTLE` control frame.

use link_protocol::{ThrottleAction, ThrottleHint};

use crate::coalescing::CAPACITY_ENTRIES;

/// The high watermark: ADR-0033's 6 KiB.
pub const HIGH_WATERMARK_BYTES: usize = 6 * 1024;

/// The low watermark: ADR-0033's 2 KiB.
pub const LOW_WATERMARK_BYTES: usize = 2 * 1024;

/// The entry-count high watermark: three quarters of the ring's descriptors.
pub const HIGH_WATERMARK_ENTRIES: usize = CAPACITY_ENTRIES * 3 / 4;

/// The entry-count low watermark: one quarter of the ring's descriptors.
pub const LOW_WATERMARK_ENTRIES: usize = CAPACITY_ENTRIES / 4;

/// The pause hint's lifetime: ADR-0033's 1000 ms.
pub const PAUSE_TTL_MS: u16 = 1000;

/// The minimum gap between two hints: ADR-0033's 500 ms.
pub const MIN_HINT_INTERVAL_MS: u32 = 500;

/// The ring occupancy the policy watches: used bytes and queued entries.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Occupancy {
    /// The bytes in use across every queued entry.
    pub bytes: usize,
    /// The number of queued entries.
    pub entries: usize,
}

/// The hysteresis policy that turns ring occupancy into `THROTTLE` hints.
#[derive(Clone, Copy, Debug)]
pub struct WatermarkPolicy {
    /// Whether a pause hint is currently in force.
    paused: bool,
    /// When the last hint was emitted, in the caller's millisecond scale.
    last_hint_ms: Option<u32>,
}

impl WatermarkPolicy {
    /// Creates an unpaused policy with no hint history.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            paused: false,
            last_hint_ms: None,
        }
    }

    /// Returns whether a pause hint is currently in force.
    #[must_use]
    pub const fn is_paused(&self) -> bool {
        self.paused
    }

    /// Observes the ring's occupancy and returns the hint to emit, if any.
    ///
    /// The policy pauses when either high watermark is reached and resumes only when both low
    /// watermarks are met. While it is paused and either occupancy stays above its low watermark
    /// it refreshes the pause, so a stall longer than [`PAUSE_TTL_MS`] cannot let the robot resume
    /// into a full ring. Between the marks it holds its state. A hint is suppressed until at least
    /// [`MIN_HINT_INTERVAL_MS`] have passed since the previous hint; the firmware calls this again
    /// on its next pump tick.
    #[must_use]
    pub const fn observe(&mut self, occupancy: Occupancy, now_ms: u32) -> Option<ThrottleHint> {
        let high = occupancy.bytes >= HIGH_WATERMARK_BYTES || occupancy.entries >= HIGH_WATERMARK_ENTRIES;
        let low = occupancy.bytes <= LOW_WATERMARK_BYTES && occupancy.entries <= LOW_WATERMARK_ENTRIES;
        if self.paused {
            if low && self.hint_allowed(now_ms) {
                self.paused = false;
                self.last_hint_ms = Some(now_ms);
                return Some(ThrottleHint::all(ThrottleAction::Resume, 0, 0));
            }
            if !low && self.hint_allowed(now_ms) {
                self.last_hint_ms = Some(now_ms);
                return Some(ThrottleHint::all(ThrottleAction::Pause, 0, PAUSE_TTL_MS));
            }
        } else if high && self.hint_allowed(now_ms) {
            self.paused = true;
            self.last_hint_ms = Some(now_ms);
            return Some(ThrottleHint::all(ThrottleAction::Pause, 0, PAUSE_TTL_MS));
        }
        None
    }

    /// Returns whether enough time has passed since the previous hint.
    const fn hint_allowed(&self, now_ms: u32) -> bool {
        match self.last_hint_ms {
            None => true,
            Some(last) => now_ms.wrapping_sub(last) >= MIN_HINT_INTERVAL_MS,
        }
    }
}

impl Default for WatermarkPolicy {
    fn default() -> Self {
        Self::new()
    }
}
