//! The robot side of the Link Protocol (ADR-0032).
//!
//! `LinkCore` will own the RX parser, the pending stream slots, the event ring, the lease, command
//! dedupe and TX priority. It is driven with plain millisecond timestamps rather than a clock so
//! it can be host-tested; the firmware keeps the UART, the port power, the sampling and the
//! mapping.
//!
//! This crate is `no_std`. The `std` feature exists only for host tests and for the simulated link
//! in `remote-protocol`.

#![no_std]
