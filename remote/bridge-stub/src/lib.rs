//! The Bridge Stub: the daemon's bench peer (ADR-0037, ADR-0039).
//!
//! The stub replaces the Bridge for the daemon's development and tests. It serves the
//! `bridge-net` preamble's server side — the challenge with the identity fields, the client-proof
//! verification, `ACCEPT`, and the `DENY` variants with `retry_ms` — plus the Discovery Beacon and
//! scripted robot traffic behind the accepted connection: streams at configurable rates, events
//! retried until acknowledged, and command ACKs with configurable statuses.
//!
//! Both sides of the preamble compose [`bridge_net`]'s framing constants and this crate's message
//! codec, so a Bridge/daemon divergence is a compile error, not a bench finding. The Beacon's
//! magic, major and flag bits are likewise `bridge-net`'s. The stub never invents Link Protocol
//! semantics: frames, payloads and the event retry schedule are `link-protocol`'s.
//!
//! The library is plain blocking `std::net`, so a test spins it up in-process without an async
//! runtime ([`Stub::start`]), and a tokio daemon can import it later without friction. The binary
//! runs the same scenario standalone for manual integration runs and the joint milestone's early
//! rehearsals.

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod cli;
pub mod discovery;
pub mod error;
pub mod identity;
pub mod preamble;
pub mod script;
pub mod serve;

pub use cli::{Invocation, Options, USAGE};
pub use discovery::Beacon;
pub use error::DecodeError;
pub use identity::{DeviceId, Flags, Identity, Name};
pub use preamble::{DenyReason, Kind, Message, Received};
pub use script::{BeaconScript, Deny, Scenario, ScriptEvent, StreamRate, Traffic};
pub use serve::Stub;
