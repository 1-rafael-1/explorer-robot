//! The Bridge's pump rules, proven without the C3 (ADR-0030, ADR-0033, ADR-0037).
//!
//! This crate is the testable core of the Bridge firmware: it parses nothing on its own, owns no
//! UART, no socket and no radio, and is `no_std` with no allocator. It carries the rules the
//! firmware applies to frames that the firmware and `bridge-net` have already parsed and
//! authenticated, plus the Setup Mode protocol that runs without a HAL:
//!
//! - the latest-wins coalescing ring ([`coalescing`]),
//! - the routing filter over the Link Protocol's class rules ([`routing`]),
//! - the watermark to `THROTTLE` decisions ([`flow`]),
//! - the single-client authentication gate ([`auth`]),
//! - the watchdog's task-liveness policy ([`heartbeat`]),
//! - the bounded Setup Mode window ([`setup`]),
//! - the portal's DHCP, DNS and HTTP/HTML codecs ([`dhcp`], [`dns`], [`portal`]),
//! - the credential record and the pairing token ([`record`], [`pairing`]).
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
pub mod dhcp;
pub mod dns;
pub mod flow;
pub mod heartbeat;
pub mod pairing;
pub mod portal;
pub mod pump;
pub mod record;
pub mod routing;
pub mod setup;

pub use auth::{ClientAuth, ConnectOutcome, Proof, ProofOutcome, TimerEvent};
pub use coalescing::{Entry, EntryKind, PushError, PushOutcome, Ring};
pub use flow::WatermarkPolicy;
pub use heartbeat::{CoveredTask, HeartbeatSet};
pub use pump::{BridgePump, Decision, PumpStats};
pub use record::{CredentialRecord, RecordError};
pub use remote_protocol::{Class, Direction, Flags, Frame, Seq, Status, crc16};
pub use routing::{BridgeControlRequest, BridgeReply, Destination, Route, Router, Side};
pub use setup::{CloseReason, RaiseReason, SetupEvent, SetupWindow};
