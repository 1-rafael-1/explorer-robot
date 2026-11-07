//! The Bridge's pump rules, proven without the C3 (ADR-0030, ADR-0033, ADR-0037).
//!
//! This crate is the testable core of the Bridge firmware: it parses nothing on its own, owns no
//! UART, no socket and no radio, and is `no_std` with no allocator. It carries the four rules the
//! pump applies to frames that the firmware and `bridge-net` have already parsed and authenticated:
//!
//! - the latest-wins coalescing ring ([`coalescing`]),
//! - the routing filter over the Link Protocol's class rules ([`routing`]),
//! - the watermark to `THROTTLE` decisions ([`flow`]),
//! - the single-client authentication gate ([`auth`]).
//!
//! [`pump`] composes them into the frame-level decisions the binary wires to its HAL. Enable the
//! `std` feature only for host tests; it is a no-op on `no_std` targets so `--all-features` stays
//! valid for the firmware.

#![no_std]
#![forbid(unsafe_code)]
#![warn(missing_docs)]

#[cfg(all(feature = "std", not(target_os = "none")))]
extern crate std;

pub mod auth;
pub mod coalescing;
pub mod flow;
pub mod pump;
pub mod routing;

pub use auth::{ClientAuth, ConnectOutcome, Proof, ProofOutcome, TimerEvent, constant_time_eq};
pub use coalescing::{Entry, EntryKind, PushError, PushOutcome, Ring};
pub use flow::WatermarkPolicy;
pub use pump::{BridgePump, Decision};
pub use remote_protocol::{Class, Direction, Flags, Frame, Seq, Status};
pub use routing::{BridgeControlRequest, BridgeReply, Destination, Route, Router, Side};
