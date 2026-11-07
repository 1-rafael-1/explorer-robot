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
