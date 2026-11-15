//! Per-sensor arming and recovery lifecycle.
//!
//! The firmware task owns the async I2C transactions; this module owns the
//! state machine around them. It decides when a sensor is waiting, armed,
//! recovering or failed, how many attempts remain, and when a reading is a
//! stale or faulty reading. The task supplies readings, init/reset outcomes and
//! `now_ms`, so every transition is host-testable without a sensor.
//!
//! The detector is owned here as well: a sensor's floor decision and its
//! lifecycle state advance together.
//!
//! Only a **Rangefinder Fault** status drives invalid-streak recovery. No-target,
//! min-range, min-clip, no-wrap-check and synchronization readings still hold the
//! last decision and fail safe after the invalid hold, but they never reset a
//! healthy sensor.

use crate::{
    RangefinderReading,
    constants::RESET_ATTEMPTS,
    detector::{FloorDropDetector, FloorState, ReadingHealth},
    pair::SensorState,
};

/// Why an armed sensor should be reset.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RecoveryCause {
    /// No reading arrived within the stale watchdog.
    Stale,
    /// The sensor kept reporting a fault status long enough to fail safe.
    InvalidStreak,
}

/// The lifecycle of one sensor within the pair.
#[derive(Debug, Clone, Copy)]
pub struct SensorLifecycle {
    /// Distance above which a valid reading means the floor is gone.
    threshold_mm: u16,
    /// The floor detector owned by this sensor.
    detector: FloorDropDetector,
    /// The sensor's state within the pair.
    state: SensorState,
    /// Reset attempts used in this enabled period.
    attempts: u8,
    /// Whether the last reading's status is a **Rangefinder Fault**, the only
    /// invalid class that may drive recovery.
    last_fault: bool,
}

impl SensorLifecycle {
    /// Create a lifecycle for a sensor with the given drop threshold.
    #[must_use]
    pub const fn new(threshold_mm: u16) -> Self {
        Self {
            threshold_mm,
            detector: FloorDropDetector::new(threshold_mm),
            state: SensorState::Waiting,
            attempts: 0,
            last_fault: false,
        }
    }

    /// Start a fresh enable: wait for the first valid reading with a full
    /// attempt budget.
    pub const fn begin_enable(&mut self, now_ms: u64) {
        self.detector.arm(now_ms);
        self.state = SensorState::Waiting;
        self.attempts = 0;
        self.last_fault = false;
    }

    /// Reset after an explicit disable, without starting the stale watchdog.
    pub const fn begin_disable(&mut self) {
        self.detector = FloorDropDetector::new(self.threshold_mm);
        self.state = SensorState::Waiting;
        self.attempts = 0;
        self.last_fault = false;
    }

    /// Take the next init/reset attempt, or `None` when the budget is spent.
    ///
    /// The returned number is the human-facing attempt count (1-based).
    pub const fn begin_attempt(&mut self) -> Option<u8> {
        if self.attempts >= RESET_ATTEMPTS {
            return None;
        }
        self.attempts = self.attempts.saturating_add(1);
        Some(self.attempts)
    }

    /// Record a successful init or reset and start waiting for a valid reading.
    ///
    /// A success replenishes the attempt budget: the three attempts are for a
    /// run of consecutive failures, not a lifetime allowance. A held floor-drop
    /// decision survives the reset; fresh valid floor readings must clear it.
    pub const fn attempt_succeeded(&mut self, now_ms: u64) {
        self.detector.restart(now_ms);
        self.state = SensorState::Waiting;
        self.attempts = 0;
        self.last_fault = false;
    }

    /// Latch the sensor failed for this enabled period.
    pub const fn mark_failed(&mut self) {
        self.state = SensorState::Failed;
        self.last_fault = false;
    }

    /// Begin recovering a sensor.
    ///
    /// An armed sensor fails safe immediately, so a drive already in progress
    /// brakes while the reset attempts run. A sensor that never armed has no
    /// valid floor to hold, so it simply reports `Recovering`.
    pub const fn begin_recovery(&mut self) {
        if matches!(self.state, SensorState::Armed) {
            self.detector.fail_safe();
        }
        self.state = SensorState::Recovering;
    }

    /// Apply one Rangefinder Reading to the detector and the lifecycle.
    pub const fn on_reading(&mut self, reading: RangefinderReading, now_ms: u64) {
        let fault = reading.status.is_fault();
        let health = self.detector.update(reading, now_ms);
        self.last_fault = fault;
        if matches!(self.state, SensorState::Waiting) && matches!(health, ReadingHealth::Valid) {
            self.state = SensorState::Armed;
        }
    }

    /// The sensor's state within the pair.
    #[must_use]
    pub const fn state(&self) -> SensorState {
        self.state
    }

    /// The detector's reported floor state.
    #[must_use]
    pub const fn floor(&self) -> FloorState {
        self.detector.state()
    }

    /// The detector's held safety decision.
    #[must_use]
    pub const fn is_drop(&self) -> bool {
        self.detector.is_drop()
    }

    /// Whether the sensor needs a reset, and why.
    ///
    /// A failed sensor needs no further recovery: its attempts are spent and it
    /// waits for a fresh enable. A stale sensor is recovered at any live state;
    /// a sustained **Rangefinder Fault** matters once the sensor has armed or is
    /// still waiting for its first valid reading, and the detector has failed
    /// safe. Waiting matters so a sensor that boots but only ever reports
    /// hardware faults is reset instead of sitting in `Waiting` forever.
    /// No-target, min-range, min-clip, no-wrap-check and synchronization readings
    /// hold the decision and fail safe without starting recovery.
    #[must_use]
    pub const fn recovery_cause(&self, now_ms: u64) -> Option<RecoveryCause> {
        if matches!(self.state, SensorState::Failed) {
            return None;
        }
        if self.detector.is_stale(now_ms) {
            return Some(RecoveryCause::Stale);
        }
        if matches!(self.state, SensorState::Armed | SensorState::Waiting)
            && self.last_fault
            && matches!(self.detector.state(), FloorState::Drop)
        {
            return Some(RecoveryCause::InvalidStreak);
        }
        None
    }
}
