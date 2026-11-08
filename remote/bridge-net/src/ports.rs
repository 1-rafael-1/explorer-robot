//! Fixed network ports shared by the Bridge and the daemon (ADR-0037, ADR-0039).
//!
//! The app-facing HTTP port belongs to the daemon and is configured there, not here.

/// The fixed UDP port the daemon listens on for the Discovery Beacon.
pub const BEACON_UDP: u16 = 9001;

/// The Link Protocol TCP port used when an endpoint is entered by hand. A discovered endpoint
/// advertises its own port and wins.
pub const LINK_TCP_DEFAULT: u16 = 9000;
