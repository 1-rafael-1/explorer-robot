//! The Bridge↔daemon network edge (ADR-0037, ADR-0039, ADR-0040).
//!
//! Discovery and authentication are not Link Protocol traffic: they are a UDP Beacon, a framed
//! HMAC-SHA256 preamble and a handful of fixed ports. Only the Bridge and the daemon depend on
//! this crate; it never enters the robot's path.
//!
//! The crate version mirrors the preamble version (ADR-0040).

#![no_std]

pub mod beacon;
pub mod ports;
pub mod preamble;
