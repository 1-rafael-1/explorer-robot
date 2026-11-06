//! The remote daemon (ADR-0039).
//!
//! The relay, the Bridge link, the WebSocket and the app embedding are implemented by the daemon
//! effort; this entry point only anchors the crate in the layout and proves its two path
//! dependencies.

/// Placeholder entry point; the daemon effort replaces it with the relay, the Bridge link, the
/// WebSocket and the app embedding.
fn main() {
    let protocol = remote_protocol::PROTOCOL_MAJOR;
    let preamble = bridge_net::preamble::MAJOR;
    println!("explorer-remote placeholder: protocol {protocol}, preamble {preamble}");
}
