//! Rotor warm-up settle detection (pure, host-testable).
//!
//! [`Warmup`] turns a sequence of per-revolution point counts into a decision:
//! settle once the count stays within a band around [`NATIVE_POINTS`], plateau
//! once it stops climbing below that band, or exhaust the spin budget.

use crate::types::{NATIVE_POINTS, WarmupConfig, WarmupOutcome};

/// State machine that decides when the rotor has reached steady state.
///
/// The point count per revolution proxies rotor speed: it starts below
/// [`NATIVE_POINTS`] and climbs as the rotor accelerates, and both the
/// ring-start bearing and the point count drift until it settles.
pub struct Warmup {
    /// Warm-up policy (settle band, patience, spin budget).
    config: WarmupConfig,
    /// Revolutions observed so far.
    spin: usize,
    /// Consecutive revolutions within the settle band.
    in_band: usize,
    /// Highest point count observed so far.
    ceiling: usize,
    /// Consecutive revolutions since the last new point-count high.
    flat: usize,
}

impl Warmup {
    /// Create a warm-up tracker from `config`.
    #[must_use]
    pub const fn new(config: &WarmupConfig) -> Self {
        Self {
            config: *config,
            spin: 0,
            in_band: 0,
            ceiling: 0,
            flat: 0,
        }
    }

    /// Observe one revolution's point count.
    ///
    /// Returns the [`WarmupOutcome`] once the rotor has settled, plateaued, or
    /// exhausted its spin budget, and `None` while it is still warming up.
    pub const fn observe(&mut self, points: usize) -> Option<WarmupOutcome> {
        self.spin += 1;

        if points.abs_diff(NATIVE_POINTS) <= self.config.settle_tolerance {
            self.in_band += 1;
        } else {
            self.in_band = 0;
        }

        if points > self.ceiling {
            self.ceiling = points;
            self.flat = 0;
        } else {
            self.flat += 1;
        }

        if self.in_band >= self.config.settle_stable_spins {
            return Some(WarmupOutcome::Settled {
                spins: self.spin,
                points,
            });
        }

        // A plateau is only a give-up when it sits below the settle band: an
        // overshoot above the band is a transient that descends back to the
        // native count.
        if self.flat >= self.config.settle_stable_spins
            && self.ceiling < NATIVE_POINTS.saturating_sub(self.config.settle_tolerance)
        {
            return Some(WarmupOutcome::Plateaued {
                spins: self.spin,
                points: self.ceiling,
            });
        }

        if self.spin >= self.config.max_spins {
            return Some(WarmupOutcome::Exhausted { spins: self.spin });
        }

        None
    }
}
