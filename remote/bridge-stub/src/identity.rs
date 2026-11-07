//! The Bridge identity served in the challenge and the Beacon (ADR-0037, ADR-0039).
//!
//! The types are `bridge-net`'s, the ADR-0040 home for the shared implementation; this re-export
//! keeps the stub's public surface stable for its tests and the daemon's imports.

pub use bridge_net::identity::{DeviceId, Flags, Identity, Name};
