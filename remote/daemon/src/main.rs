//! The remote daemon binary (ADR-0039).
//!
//! The command line, the per-user token store, the Beacon listener and the host wiring live in the
//! library so the end-to-end test drives the same path. This entry point only parses the CLI,
//! builds the wired pieces and runs the tokio runtime.

use std::{process::ExitCode, sync::Arc};

use clap::Parser;
use remote_daemon::{
    app::Assets,
    cli::Cli,
    config::Store,
    discovery::{Discovery, DiscoveryConfig},
    host::{Host, run},
};

fn main() -> ExitCode {
    match start(Cli::parse()) {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("remote-daemon: {error}");
            ExitCode::FAILURE
        }
    }
}

/// Builds the wired pieces from the command line and serves until Ctrl+C.
fn start(cli: Cli) -> Result<(), Box<dyn std::error::Error>> {
    let config_path = match &cli.config {
        Some(path) => path.clone(),
        None => Store::default_path()?,
    };
    let store = Store::open(config_path)?;
    let discovery = Discovery::bind(DiscoveryConfig {
        last_device: store.last_device(),
        ..DiscoveryConfig::default()
    })?;
    let assets = cli.assets.map_or_else(Assets::embedded, Assets::disk);
    let host = Host {
        discovery,
        store: Arc::new(store),
        assets,
        serve_tests: cli.serve_tests,
        bind: cli.host,
        port: cli.port,
        open_browser: !cli.no_open,
        verbose: cli.verbose,
    };
    let runtime = tokio::runtime::Builder::new_multi_thread().enable_all().build()?;
    runtime.block_on(run(host))?;
    Ok(())
}
