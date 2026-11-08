//! The watermark to `THROTTLE` policy (ADR-0030, ADR-0033).
//!
//! The Bridge is authoritative for UART overflow. When its robot-facing ring crosses the high
//! watermark it tells the robot to pause every stream; when the ring drains to the low watermark
//! it resumes them. The hint is advisory and time-limited, and two hints are kept at least 500 ms
//! apart so a ring hovering at a watermark cannot flood the robot with control frames.
//!
//! Pressure is measured both in bytes and in queued entries. Bytes alone are not enough: the ring
//! has a 16-entry descriptor table, and the robot's worst-case mix (six coalesced streams plus many
//! small reliable events) fills that table at roughly 2–3 KiB, long before the 6 KiB byte mark.
//! Without the entry watermark the ring would drop frames and never ask the robot to slow down.
//!
//! The policy owns no ring and no clock: the caller passes the used byte count, the queued entry
//! count and a monotonic millisecond timestamp, and encodes the returned hint as a `THROTTLE`
//! control frame.

use remote_protocol::{ThrottleAction, ThrottleHint};

/// The high watermark: ADR-0033's 6 KiB.
pub const HIGH_WATERMARK_BYTES: usize = 6 * 1024;

/// The low watermark: ADR-0033's 2 KiB.
pub const LOW_WATERMARK_BYTES: usize = 2 * 1024;

/// The entry-count high watermark: three quarters of the ring's 16 descriptors.
pub const HIGH_WATERMARK_ENTRIES: usize = 12;

/// The entry-count low watermark: one quarter of the ring's 16 descriptors.
pub const LOW_WATERMARK_ENTRIES: usize = 4;

/// The pause hint's lifetime: ADR-0033's 1000 ms.
pub const PAUSE_TTL_MS: u16 = 1000;

/// The minimum gap between two hints: ADR-0033's 500 ms.
pub const MIN_HINT_INTERVAL_MS: u32 = 500;

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
    /// watermarks are met. Between the two it holds its state. A hint is suppressed until at
    /// least [`MIN_HINT_INTERVAL_MS`] have passed since the previous hint; the firmware calls
    /// this again on its next pump tick.
    #[must_use]
    pub const fn observe(&mut self, used_bytes: usize, entries: usize, now_ms: u32) -> Option<ThrottleHint> {
        let high = used_bytes >= HIGH_WATERMARK_BYTES || entries >= HIGH_WATERMARK_ENTRIES;
        let low = used_bytes <= LOW_WATERMARK_BYTES && entries <= LOW_WATERMARK_ENTRIES;
        if !self.paused && high {
            if self.hint_allowed(now_ms) {
                self.paused = true;
                self.last_hint_ms = Some(now_ms);
                return Some(ThrottleHint::all(ThrottleAction::Pause, 0, PAUSE_TTL_MS));
            }
        } else if self.paused && low && self.hint_allowed(now_ms) {
            self.paused = false;
            self.last_hint_ms = Some(now_ms);
            return Some(ThrottleHint::all(ThrottleAction::Resume, 0, 0));
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
