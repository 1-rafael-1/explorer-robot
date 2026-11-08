//! Host-side integration tests for the warm-up settle decision.

use coin_d6::{Warmup, WarmupConfig, WarmupOutcome};

/// Drive `points` through a [`Warmup`] tracker and return its first outcome.
fn run(points: &[usize], config: &WarmupConfig) -> Option<WarmupOutcome> {
    let mut warmup = Warmup::new(config);
    points.iter().find_map(|&points| warmup.observe(points))
}

#[test]
fn warmup_settles_after_stable_in_band_spins() {
    let config = WarmupConfig {
        settle_tolerance: 3,
        settle_stable_spins: 12,
        max_spins: 50,
    };

    // Twelve consecutive spins at the native count settle on the twelfth.
    let outcome = run(&[400; 12], &config);

    assert_eq!(outcome, Some(WarmupOutcome::Settled { spins: 12, points: 400 }));
}

#[test]
fn warmup_plateaus_when_stuck_below_the_band() {
    let config = WarmupConfig {
        settle_tolerance: 3,
        settle_stable_spins: 12,
        max_spins: 50,
    };

    // Stuck at 380 (below the 397 lower edge). The first spin establishes the
    // ceiling; the next twelve are non-climbing, so the plateau fires on the
    // thirteenth spin.
    let outcome = run(&[380; 13], &config);

    assert_eq!(outcome, Some(WarmupOutcome::Plateaued { spins: 13, points: 380 }));
}

#[test]
fn warmup_exhausts_the_spin_budget() {
    let config = WarmupConfig {
        settle_tolerance: 3,
        settle_stable_spins: 12,
        max_spins: 10,
    };

    // Never in-band and never plateaued before the budget runs out.
    let outcome = run(&[390; 10], &config);

    assert_eq!(outcome, Some(WarmupOutcome::Exhausted { spins: 10 }));
}

#[test]
fn warmup_overshoot_above_the_band_is_not_a_plateau() {
    let config = WarmupConfig {
        settle_tolerance: 3,
        settle_stable_spins: 12,
        max_spins: 50,
    };

    // Overshoot past the band (ceiling 408), then descend and settle at 400.
    // The plateau detector must not fire during the descent.
    let outcome = run(
        &[
            404, 407, 408, 405, 404, 402, 401, 400, 400, 401, 400, 401, 400, 401, 400, 401, 400,
        ],
        &config,
    );

    assert_eq!(outcome, Some(WarmupOutcome::Settled { spins: 17, points: 400 }));
}
