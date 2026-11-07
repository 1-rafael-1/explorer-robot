//! The remote daemon (ADR-0039).
//!
//! The daemon hosts two adapters around one relay: a `link` adapter that owns the raw TCP, the
//! preamble and the wire codec, and an `app` adapter that owns the WebSocket, auth and the static
//! SPA. This library crate is the relay itself — the app-facing Session state that both adapters
//! share — plus the versioned JSON schema those adapters speak. The binary entry point anchors the
//! crate in the layout until the adapters land.
//!
//! The relay is the deep module and the seam: the `link` adapter never awaits an app, the `app`
//! adapter never awaits the robot, and every per-stream and per-event path crosses the relay as a
//! bounded, non-blocking update. The relay itself is [`relay::Relay`]. Like `remote-link`, it is
//! driven with plain millisecond timestamps rather than a clock so it is host-testable through a
//! fake link and an in-memory app transport (ADR-0039's test plan).

#![forbid(unsafe_code)]
#![warn(missing_docs)]

pub mod app;
pub mod clock;
pub mod handshake;
pub mod link;
pub mod relay;
pub mod schema;
