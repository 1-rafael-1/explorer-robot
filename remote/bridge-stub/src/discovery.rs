//! The Discovery Beacon datagram (ADR-0037, ADR-0039).
//!
//! The codec is `bridge-net`'s, the ADR-0040 home for the shared implementation, so the stub and
//! the daemon share one source. A `tcp_port` of zero advertises a portal-only Bridge with no Link
//! listener, which the daemon treats as Setup Mode and does not connect to.

pub use bridge_net::beacon::{Beacon, ENCODED_LEN};
