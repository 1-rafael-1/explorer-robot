//! The Bridge↔daemon network edge (ADR-0037, ADR-0039, ADR-0040).
//!
//! Discovery and authentication are not Link Protocol traffic: they are a UDP Beacon, a framed
//! HMAC-SHA256 preamble and a handful of fixed ports. Only the Bridge and the daemon depend on
//! this crate; it never enters the robot's path.
//!
//! The crate is the ADR-0040 home for the shared preamble and Beacon implementation, so a
//! Bridge/daemon divergence is a compile error rather than a bench finding. It is `no_std` and
//! allocation-free; the sockets and I/O loops stay in the consumers.
//!
//! The crate version mirrors the preamble version (ADR-0040).

#![no_std]

pub mod beacon;
pub mod error;
pub mod identity;
pub mod ports;
pub mod preamble;

/// Compares two byte strings without an early exit on the first difference.
///
/// The loop always visits the longer of the two inputs, folding a length difference into the
/// accumulator first, so a caller's timing does not reveal *where* a secret differs. The length
/// itself is treated as public. [`core::hint::black_box`] keeps the optimiser from realising the
/// fold is a comparison and converting it back into a short-circuiting one.
#[must_use]
pub fn constant_time_eq(left: &[u8], right: &[u8]) -> bool {
    let mut difference = u8::from(left.len() != right.len());
    let longest = left.len().max(right.len());
    for index in 0..longest {
        let a = left.get(index).copied().unwrap_or(0);
        let b = right.get(index).copied().unwrap_or(0);
        difference |= a ^ b;
    }
    core::hint::black_box(difference) == 0
}
