//! Floor-drop detection over a stream of [`RangefinderReading`]s.
//!
//! The detector is pure: it takes a caller-supplied `now_ms`, so the firmware
//! can drive it from its own clock and host tests can step time deterministically.
//!
//! A valid reading above the threshold sets [`FloorState::Drop`] immediately.
//! Recovering to [`FloorState::Floor`] needs two consecutive valid floor
//! readings. Invalid and no-target samples report [`FloorState::Unknown`] for
//! [`INVALID_HOLD_MS`] while the last safety decision is held, and then fail
//! safe to `Drop`. They never produce a new floor decision, so a bad measurement
//! can only ever make the detector more cautious.

use crate::{
    constants::{CLEAR_SAMPLES, INVALID_HOLD_MS, STALE_TIMEOUT_MS},
    types::{RangefinderReading, Reading},
};

/// The detector's current view of the ground.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FloorState {
    /// No current decision: before the first valid reading, or while an invalid
    /// streak holds the previous decision.
    Unknown,
    /// The floor is present within the threshold.
    Floor,
    /// A drop was detected or the sensor could not prove the floor is present.
    Drop,
}

/// How the last reading was classified.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ReadingHealth {
    /// The reading completed and carried a range.
    Valid,
    /// The sensor saw no target.
    NoTarget,
    /// The sensor reported a failure status.
    Invalid,
}

/// A per-sensor floor-drop detector.
#[derive(Debug, Clone, Copy)]
pub struct FloorDropDetector {
    /// Distance above which a valid reading means the floor is gone.
    threshold_mm: u16,
    /// The state the readout and the detector report.
    state: FloorState,
    /// The safety decision held across an invalid streak. It is `Floor` or
    /// `Drop` once a valid reading has been seen, and never `Unknown` after
    /// that point.
    decision: FloorState,
    /// Consecutive valid floor readings seen while dropped.
    floor_streak: u8,
    /// Timestamp of the first invalid reading in the current streak.
    invalid_since_ms: Option<u64>,
    /// Timestamp of the most recent reading, used by the stale watchdog.
    last_sample_ms: Option<u64>,
}

impl FloorDropDetector {
    /// Create a detector with the given drop threshold.
    #[must_use]
    pub const fn new(threshold_mm: u16) -> Self {
        Self {
            threshold_mm,
            state: FloorState::Unknown,
            decision: FloorState::Unknown,
            floor_streak: 0,
            invalid_since_ms: None,
            last_sample_ms: None,
        }
    }

    /// Start a fresh enable.
    ///
    /// Resets the reported state and the held decision to `Unknown`, clears both
    /// streaks, and starts the stale watchdog at `now_ms`.
    pub const fn arm(&mut self, now_ms: u64) {
        self.state = FloorState::Unknown;
        self.decision = FloorState::Unknown;
        self.floor_streak = 0;
        self.invalid_since_ms = None;
        self.last_sample_ms = Some(now_ms);
    }

    /// Feed one Rangefinder Reading taken at `now_ms`.
    ///
    /// Returns the reading's health; the resulting floor state is read from
    /// [`state`](Self::state) and [`is_drop`](Self::is_drop).
    pub const fn update(&mut self, reading: RangefinderReading, now_ms: u64) -> ReadingHealth {
        self.last_sample_ms = Some(now_ms);
        match reading.reading {
            Reading::Valid(range_mm) => {
                self.invalid_since_ms = None;
                self.observe_valid(range_mm);
                ReadingHealth::Valid
            }
            Reading::NoTarget => {
                self.observe_invalid(now_ms);
                ReadingHealth::NoTarget
            }
            Reading::Invalid(_) => {
                self.observe_invalid(now_ms);
                ReadingHealth::Invalid
            }
        }
    }

    /// The reported state.
    ///
    /// `Unknown` while an invalid or no-target streak holds the previous
    /// decision; use [`is_drop`](Self::is_drop) for the safety decision.
    #[must_use]
    pub const fn state(&self) -> FloorState {
        self.state
    }

    /// Whether the held safety decision is `Drop`.
    ///
    /// This stays true across an invalid streak, so a bad measurement cannot
    /// clear a drop while the detector reports `Unknown`.
    #[must_use]
    pub const fn is_drop(&self) -> bool {
        matches!(self.decision, FloorState::Drop)
    }

    /// Fail safe: hold `Drop` until a fresh arm or two valid floor readings.
    ///
    /// The pair task calls this when an armed sensor stalls or resets, so a
    /// drive already in progress brakes instead of continuing blind.
    pub const fn fail_safe(&mut self) {
        self.state = FloorState::Drop;
        self.decision = FloorState::Drop;
        self.floor_streak = 0;
    }

    /// Whether at least [`STALE_TIMEOUT_MS`] have elapsed since the last reading.
    ///
    /// Returns `false` before the first reading and after [`arm`](Self::arm).
    #[must_use]
    pub const fn is_stale(&self, now_ms: u64) -> bool {
        match self.last_sample_ms {
            Some(last) => now_ms.saturating_sub(last) >= STALE_TIMEOUT_MS,
            None => false,
        }
    }

    /// Apply a valid measurement.
    const fn observe_valid(&mut self, range_mm: u16) {
        if range_mm > self.threshold_mm {
            self.decision = FloorState::Drop;
            self.state = FloorState::Drop;
            self.floor_streak = 0;
        } else {
            match self.decision {
                FloorState::Drop => {
                    self.floor_streak = self.floor_streak.saturating_add(1);
                    if self.floor_streak >= CLEAR_SAMPLES {
                        self.decision = FloorState::Floor;
                        self.state = FloorState::Floor;
                        self.floor_streak = 0;
                    } else {
                        self.state = FloorState::Drop;
                    }
                }
                FloorState::Unknown | FloorState::Floor => {
                    self.decision = FloorState::Floor;
                    self.state = FloorState::Floor;
                    self.floor_streak = 0;
                }
            }
        }
    }

    /// Apply an invalid or no-target measurement.
    ///
    /// Reports `Unknown` while the previous decision is held. After
    /// [`INVALID_HOLD_MS`], the decision fails safe to `Drop`.
    const fn observe_invalid(&mut self, now_ms: u64) {
        let since = if let Some(since) = self.invalid_since_ms {
            since
        } else {
            self.invalid_since_ms = Some(now_ms);
            now_ms
        };
        if now_ms.saturating_sub(since) >= INVALID_HOLD_MS {
            self.state = FloorState::Drop;
            self.decision = FloorState::Drop;
            self.floor_streak = 0;
        } else {
            self.state = FloorState::Unknown;
        }
    }
}
