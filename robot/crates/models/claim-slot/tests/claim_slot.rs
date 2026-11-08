//! Host-side integration tests for the `claim-slot` crate.
//!
//! These exercise exclusivity, generation-scoped release, the stop latch and the
//! generation counter's wrap point. The `std` feature exists only to enable this
//! harness; the crate itself is always `no_std`.

use claim_slot::{ClaimSlot, Generation};

/// The first claim succeeds and a second claim fails while the slot is held.
#[test]
fn first_claim_succeeds_second_fails_while_held() {
    let slot = ClaimSlot::new();
    assert!(slot.generation().is_none(), "a new slot must be free");

    let first = slot.try_claim();
    assert!(first.is_some(), "the first claim must succeed");
    assert!(slot.try_claim().is_none(), "a held slot must reject a second claim");

    let Some(first) = first else { return };
    assert_eq!(
        slot.generation().map(Generation::get),
        Some(first.get()),
        "the held generation must be observable"
    );
}

/// Releasing the held generation frees the slot for a later claim.
#[test]
fn release_frees_the_slot_and_a_later_claim_succeeds() {
    let slot = ClaimSlot::new();

    let first = slot.try_claim();
    assert!(first.is_some(), "the first claim must succeed");
    let Some(first) = first else { return };

    assert!(slot.release(first), "releasing the current generation must succeed");
    assert!(slot.generation().is_none(), "the slot must be free after release");

    let second = slot.try_claim();
    assert!(second.is_some(), "a later claim must succeed after release");
    let Some(second) = second else { return };
    assert!(second.get() > first.get(), "each claim must mint a newer generation");
}

/// A stale generation cannot free a newer claim, and a double release is harmless.
#[test]
fn a_stale_generation_cannot_free_a_newer_claim() {
    let slot = ClaimSlot::new();

    let first = slot.try_claim();
    assert!(first.is_some(), "the first claim must succeed");
    let Some(first) = first else { return };
    assert!(slot.release(first), "releasing the first generation must succeed");

    let second = slot.try_claim();
    assert!(second.is_some(), "the second claim must succeed");
    let Some(second) = second else { return };

    assert!(!slot.release(first), "a stale release must not free the newer claim");
    assert_eq!(
        slot.generation().map(Generation::get),
        Some(second.get()),
        "the newer generation must still hold the slot"
    );

    assert!(slot.release(second), "releasing the current generation must succeed");
    assert!(!slot.release(second), "a double release must be a no-op");
    assert!(slot.generation().is_none(), "the slot must be free after release");
}

/// Stop latches until `arm` clears it, and repeated requests are idempotent.
#[test]
fn stop_latches_and_arm_clears_it() {
    let slot = ClaimSlot::new();
    assert!(!slot.is_stop_requested(), "a new slot must not be stopped");

    slot.request_stop();
    assert!(slot.is_stop_requested(), "the stop must latch");

    slot.request_stop();
    assert!(slot.is_stop_requested(), "a repeated stop must stay latched");

    slot.arm();
    assert!(!slot.is_stop_requested(), "arming must clear the stop latch");
}

/// Each claim mints the next generation, and the counter skips zero on wrap.
#[test]
fn generation_increments_and_skips_zero_on_wrap() {
    let slot = ClaimSlot::new();

    let mut previous = 0;
    for _ in 0..3 {
        let claim = slot.try_claim();
        assert!(claim.is_some(), "the slot must be free for each claim");
        let Some(claim) = claim else { return };
        assert_eq!(claim.get(), previous + 1, "each claim must mint the next generation");
        previous = claim.get();
        assert!(slot.release(claim), "releasing the current generation must succeed");
    }

    let wrapping = ClaimSlot::new();
    wrapping.seed_generation_counter(u32::MAX);
    let claim = wrapping.try_claim();
    assert!(claim.is_some(), "the wrap-point claim must succeed");
    let Some(claim) = claim else { return };
    assert_eq!(claim.get(), 1, "the counter must skip zero when it wraps");
}
