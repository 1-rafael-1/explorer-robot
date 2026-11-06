//! The shared Link Protocol contract (ADR-0030, ADR-0031, ADR-0040).
//!
//! This crate is `no_std` and transport agnostic: it carries the frame envelope, the shared
//! identifiers and the payload codecs that the robot, the Bridge and the daemon all speak. It has
//! no UART, clock, allocator or logging. Enable the `std` feature only for host tests and the
//! simulated lossy link.
//!
//! The crate version mirrors the wire version: changing [`PROTOCOL_MAJOR`] or [`PROTOCOL_MINOR`]
//! changes the crate's major or minor version, and every consumer states a `version = "1"`
//! requirement so a major bump cannot pass unnoticed (ADR-0040).

#![no_std]

/// The wire major version carried in every frame header.
pub const PROTOCOL_MAJOR: u8 = 1;

/// The wire minor version exchanged in the session handshake.
pub const PROTOCOL_MINOR: u8 = 0;
