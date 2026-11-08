//! The daemon's command line (ADR-0039's `--host`, `--port`, `--no-open`, `--assets`,
//! `--config` and `--verbose`).
//!
//! There is deliberately no token flag: a pairing secret is never accepted on the command line,
//! so it cannot land in shell history, process listings or screenshots. The store is the only
//! token source.

use std::{net::IpAddr, path::PathBuf};

use clap::Parser;

/// The remote explorer daemon: one JSON WebSocket for the app, raw TCP to the Bridge.
#[derive(Clone, Debug, Parser)]
#[command(
    name = "remote-daemon",
    version,
    about = "The remote explorer daemon: one JSON WebSocket for the app, raw TCP to the Bridge.",
    long_about = None
)]
pub struct Cli {
    /// Serve the app on this address instead of loopback.
    #[arg(long, value_name = "HOST", default_value = "127.0.0.1")]
    pub host: IpAddr,

    /// Serve the app on this port; the default 8765 falls back to an ephemeral port when taken.
    #[arg(long, value_name = "PORT")]
    pub port: Option<u16>,

    /// Do not open the default browser.
    #[arg(long)]
    pub no_open: bool,

    /// Serve the SPA from this folder instead of the embedded copy.
    #[arg(long, value_name = "PATH")]
    pub assets: Option<PathBuf>,

    /// Use this config file instead of the per-user default.
    #[arg(long, value_name = "PATH")]
    pub config: Option<PathBuf>,

    /// Trace discovery and Session changes to stderr.
    #[arg(long)]
    pub verbose: bool,
}
