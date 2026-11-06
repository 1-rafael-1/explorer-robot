//! The Discovery Beacon: a 3 s UDP broadcast on the Bridge's station interface (ADR-0037).

/// The Beacon's magic, `EXPR`.
pub const MAGIC: [u8; 4] = *b"EXPR";

/// The Beacon interval in milliseconds.
pub const INTERVAL_MS: u64 = 3_000;

/// The Link Protocol major advertised in the Beacon, so an old daemon can filter rather than
/// misparse.
pub const LINK_MAJOR: u8 = remote_protocol::PROTOCOL_MAJOR;

/// A `tcp_port` of zero advertises a portal-only Bridge with no Link listener; the daemon treats it
/// as Setup Mode and does not connect.
pub const PORTAL_ONLY: u16 = 0;

/// Flag bit 0: the Bridge has a credential record.
pub const FLAG_PROVISIONED: u8 = 1 << 0;

/// Flag bit 1: the robot link is ready.
pub const FLAG_LINK_READY: u8 = 1 << 1;

/// Flag bit 2: the Bridge is in Setup Mode and still reachable on the station interface.
pub const FLAG_SETUP_MODE: u8 = 1 << 2;
