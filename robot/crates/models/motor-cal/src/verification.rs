//! The corrected-verification aggregation behind the Basic Motor Test's
//! `Corr max` line.
//!
//! The test drives two curve-applied repeats and asks one question: once the
//! curve has corrected the tracks, how far apart are they? Each measured pair
//! contributes a **Track Divergence**; this module averages the repeats per
//! point and picks the worst gated mean as the headline.
//!
//! It is pure and `no_std`, so the rules that decide the headline live here and
//! are host-tested instead of inside the firmware task. The 20 % point is the
//! stiction band: it is recorded for information only and never gates the run or
//! becomes the headline.

use crate::curve_maths::{Direction, POWER_STEP_PERCENTS, POWER_STEPS};

/// The lowest power, in whole percent, that gates the verification. Points
/// below it cannot make the report complete or set [`VerificationMax`].
pub const VERIFICATION_MIN_POWER: u8 = 30;

/// The informational-only power, in whole percent, below the gate.
pub const VERIFICATION_INFO_POWER: u8 = 20;

/// How many repeats every gated Power Step needs for a complete report.
pub const VERIFICATION_REPEATS: u8 = 2;

/// `|left - right| / min(left, right)` in percent, or `None` when either count
/// is zero and the ratio is undefined.
#[must_use]
pub fn divergence_percent(left: u16, right: u16) -> Option<f32> {
    let min = left.min(right);
    if min == 0 {
        return None;
    }
    Some((f32::from(left.abs_diff(right)) / f32::from(min)) * 100.0)
}

/// Round a divergence in percent to tenths of a percent, saturating.
#[must_use]
#[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
pub fn divergence_tenths(divergence: f32) -> u16 {
    let rounded = divergence * 10.0 + 0.5;
    if rounded >= f32::from(u16::MAX) {
        u16::MAX
    } else {
        rounded as u16
    }
}

/// The worst gated mean, in tenths of a percent.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VerificationMax {
    /// The worst gated mean, rounded to tenths of a percent.
    pub tenths: u16,
    /// The direction of the worst point.
    pub direction: Direction,
    /// The Power Step of the worst point, in whole percent.
    pub power: u8,
}

/// The result of the verification repeats.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VerificationReport {
    /// The worst gated mean, only when `complete` is true.
    pub max: Option<VerificationMax>,
    /// Per-direction, per-Power-Step mean divergence; `None` when unmeasured.
    pub means: [[Option<f32>; POWER_STEPS]; 2],
    /// Per-direction 20 % mean, informational only.
    pub low_power_means: [Option<f32>; 2],
    /// True only when every gated Power Step (30..=100) has all
    /// [`VERIFICATION_REPEATS`] samples. A point measured once can never be the
    /// headline.
    pub complete: bool,
}

/// Accumulates measured divergences across the verification repeats.
#[derive(Clone, Copy, Debug)]
pub struct Verification {
    /// Summed gated divergence in percent, indexed by direction and Power Step.
    sums: [[f32; POWER_STEPS]; 2],
    /// Number of measured gated repeats, indexed by direction and Power Step.
    samples: [[u8; POWER_STEPS]; 2],
    /// Summed 20 % divergence, indexed by direction.
    low_sums: [f32; 2],
    /// Number of measured 20 % repeats, indexed by direction.
    low_samples: [u8; 2],
}

impl Verification {
    /// No points recorded yet.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            sums: [[0.0; POWER_STEPS]; 2],
            samples: [[0; POWER_STEPS]; 2],
            low_sums: [0.0; 2],
            low_samples: [0; 2],
        }
    }

    /// Record one measured pair. Callers pass counts only for a pair whose both
    /// sides showed sustained motion. A power outside [`POWER_STEP_PERCENTS`] is
    /// ignored; zero counts are ignored.
    pub fn record(&mut self, direction: Direction, power: u8, left_count: u16, right_count: u16) {
        let Some(step_index) = power_step_index(power) else {
            return;
        };
        let Some(divergence) = divergence_percent(left_count, right_count) else {
            return;
        };
        let direction_index = direction.index();
        if power >= VERIFICATION_MIN_POWER {
            self.sums[direction_index][step_index] += divergence;
            self.samples[direction_index][step_index] = self.samples[direction_index][step_index].saturating_add(1);
        } else if power == VERIFICATION_INFO_POWER {
            self.low_sums[direction_index] += divergence;
            self.low_samples[direction_index] = self.low_samples[direction_index].saturating_add(1);
        }
    }

    /// Average the repeats, mark whether the run is complete, and pick the worst
    /// gated mean.
    #[must_use]
    pub fn finish(self) -> VerificationReport {
        let mut means: [[Option<f32>; POWER_STEPS]; 2] = [[None; POWER_STEPS]; 2];
        let mut low_power_means: [Option<f32>; 2] = [None; 2];
        let mut complete = true;

        for direction in [Direction::Forward, Direction::Reverse] {
            let direction_index = direction.index();
            for (step_index, &power) in POWER_STEP_PERCENTS.iter().enumerate() {
                if power < VERIFICATION_MIN_POWER {
                    continue;
                }
                let samples = self.samples[direction_index][step_index];
                if samples == 0 {
                    complete = false;
                    continue;
                }
                means[direction_index][step_index] = Some(self.sums[direction_index][step_index] / f32::from(samples));
                if samples != VERIFICATION_REPEATS {
                    complete = false;
                }
            }

            let low_samples = self.low_samples[direction_index];
            if low_samples > 0 {
                low_power_means[direction_index] = Some(self.low_sums[direction_index] / f32::from(low_samples));
            }
        }

        let max = if complete { worst_gated(&means) } else { None };

        VerificationReport {
            max,
            means,
            low_power_means,
            complete,
        }
    }
}

impl Default for Verification {
    fn default() -> Self {
        Self::new()
    }
}

/// The index of `power` in [`POWER_STEP_PERCENTS`], or `None` when it is not a
/// Power Step.
fn power_step_index(power: u8) -> Option<usize> {
    POWER_STEP_PERCENTS.iter().position(|&percent| percent == power)
}

/// The worst gated mean, chosen before rounding; ties keep the first in
/// direction/step order.
fn worst_gated(means: &[[Option<f32>; POWER_STEPS]; 2]) -> Option<VerificationMax> {
    let mut worst: Option<VerificationMax> = None;
    let mut worst_mean = f32::NEG_INFINITY;
    for direction in [Direction::Forward, Direction::Reverse] {
        let direction_index = direction.index();
        for (step_index, &power) in POWER_STEP_PERCENTS.iter().enumerate() {
            let Some(mean) = means[direction_index][step_index] else {
                continue;
            };
            if mean > worst_mean {
                worst_mean = mean;
                worst = Some(VerificationMax {
                    tenths: divergence_tenths(mean),
                    direction,
                    power,
                });
            }
        }
    }
    worst
}
