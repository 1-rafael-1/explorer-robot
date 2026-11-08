//! Pure state for the one Active Claim (ADR-0034).
//!
//! A [`ClaimSlot`] owns two pieces of durable state: which generation, if any, currently holds
//! the claim, and whether a stop has been latched. It has no `embassy`, Activity, drive, timer or
//! UI dependency, so the firmware can keep it in a static and host tests can exercise it directly.
//!
//! The slot is a single atomic word: `0` means free and a non-zero value is the current
//! generation. Claiming is a compare-and-swap from `0` to a freshly minted non-zero
//! [`Generation`]; releasing is a compare-and-swap from that generation back to `0`. Because the
//! release carries the generation it expects, a stale release cannot free a newer claim. A
//! separate atomic counter mints the generations and skips zero on wrap, so [`Generation`] is
//! always non-zero.
//!
//! The stop latch is deliberately separate from the claim: [`ClaimSlot::request_stop`] latches it
//! and [`ClaimSlot::arm`] clears it for the next claim. The crate owns the durable flag; the
//! firmware owns any wake signal beside it, because an `embassy_sync::Signal` would make host
//! tests need a runtime.

#![no_std]
#![warn(missing_docs)]

use core::{
    num::NonZeroU32,
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};

/// The raw value of the slot's "free" state.
const FREE: u32 = 0;

/// A non-zero claim generation.
///
/// A generation is minted by [`ClaimSlot::try_claim`] and scoped to the claim that holds the slot.
/// Passing it to [`ClaimSlot::release`] succeeds only while that generation still holds the slot,
/// so terminal calls from an old claim are harmless no-ops.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct Generation(NonZeroU32);

impl Generation {
    /// Returns the wrapped non-zero value.
    #[must_use]
    pub const fn get(self) -> u32 {
        self.0.get()
    }
}

/// The pure claim state: one generation-holding slot and one stop latch.
///
/// All methods take `&self` and use atomics, so a `ClaimSlot` can live in a `static`.
#[derive(Debug)]
pub struct ClaimSlot {
    /// `0` when the slot is free, otherwise the generation that holds it.
    state: AtomicU32,
    /// The last generation value minted; `0` before the first claim.
    counter: AtomicU32,
    /// The latched stop request, cleared by [`ClaimSlot::arm`].
    stop: AtomicBool,
}

impl ClaimSlot {
    /// Creates a free slot with the stop latch cleared.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            state: AtomicU32::new(FREE),
            counter: AtomicU32::new(0),
            stop: AtomicBool::new(false),
        }
    }

    /// Claims the slot for a freshly minted generation.
    ///
    /// Returns the new [`Generation`], or [`None`] when the slot is already held.
    #[must_use]
    pub fn try_claim(&self) -> Option<Generation> {
        // Bail out before consuming a generation when the slot is held.
        if self.state.load(Ordering::Acquire) != FREE {
            return None;
        }

        let generation = mint(&self.counter);
        self.state
            .compare_exchange(FREE, generation.get(), Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
            .then_some(generation)
    }

    /// Releases the slot if `generation` still holds it.
    ///
    /// Returns `true` when the slot was freed. A stale generation, or a generation that never
    /// held the slot, is a no-op returning `false`; this includes a double release.
    #[must_use]
    pub fn release(&self, generation: Generation) -> bool {
        self.state
            .compare_exchange(generation.get(), FREE, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
    }

    /// Returns the generation currently holding the slot, or [`None`] when it is free.
    #[must_use]
    pub fn generation(&self) -> Option<Generation> {
        NonZeroU32::new(self.state.load(Ordering::Acquire)).map(Generation)
    }

    /// Clears the stop latch so a new claim starts unstopped.
    pub fn arm(&self) {
        self.stop.store(false, Ordering::Release);
    }

    /// Latches a stop request.
    ///
    /// The latch stays set until [`ClaimSlot::arm`] clears it; repeated requests are idempotent.
    pub fn request_stop(&self) {
        self.stop.store(true, Ordering::Release);
    }

    /// Returns whether a stop has been latched since the last [`ClaimSlot::arm`].
    #[must_use]
    pub fn is_stop_requested(&self) -> bool {
        self.stop.load(Ordering::Acquire)
    }

    /// Test support: overwrites the generation counter so a host test can exercise the wrap point.
    ///
    /// This is not part of the firmware contract and is compiled only with the `std` feature,
    /// which exists for the host test harness.
    #[cfg(feature = "std")]
    #[doc(hidden)]
    pub fn seed_generation_counter(&self, value: u32) {
        self.counter.store(value, Ordering::Relaxed);
    }
}

impl Default for ClaimSlot {
    fn default() -> Self {
        Self::new()
    }
}

/// Mints the next non-zero generation after `previous`, skipping zero on wrap.
const fn next_generation(previous: u32) -> NonZeroU32 {
    match NonZeroU32::new(previous.wrapping_add(1)) {
        Some(next) => next,
        None => NonZeroU32::MIN,
    }
}

/// Reserves the next generation in `counter` and returns it.
fn mint(counter: &AtomicU32) -> Generation {
    loop {
        let previous = counter.load(Ordering::Relaxed);
        let next = next_generation(previous);
        if counter
            .compare_exchange_weak(previous, next.get(), Ordering::Relaxed, Ordering::Relaxed)
            .is_ok()
        {
            return Generation(next);
        }
    }
}
