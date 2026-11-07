//! The per-user token store and endpoint cache (ADR-0039's `config` module).
//!
//! The seam is the config file on disk: every test opens a real store under a temporary
//! directory, writes through the public API and reads the file back. Expected values come from
//! ADR-0039: mode `0600`, per-device tokens, a `default_token` for manual endpoints, and the last
//! endpoint plus device id.

mod app_support;

use std::{
    fs,
    net::{Ipv4Addr, SocketAddr, SocketAddrV4},
    path::Path,
};

use app_support::must;
use bridge_net::identity::DeviceId;
use remote_daemon::{
    app::TokenGate,
    config::{Config, Store},
};
use tempfile::TempDir;

/// The first device the tests store a token for.
const fn first_device() -> DeviceId {
    DeviceId::new(*b"aaaaaa")
}

/// The second device the tests store a token for.
const fn second_device() -> DeviceId {
    DeviceId::new(*b"bbbbbb")
}

/// Opens a store under `dir` and fails the test rather than unwrapping.
fn open(dir: &TempDir) -> Store {
    must(Store::open(dir.path().join("config.toml")), "open store")
}

/// Returns the mode bits of `path` on Unix.
#[cfg(unix)]
fn mode(path: &Path) -> u32 {
    use std::os::unix::fs::PermissionsExt;
    must(fs::metadata(path), "metadata").permissions().mode() & 0o777
}

/// The store creates its config file with the ADR's `0600` mode and replaces it atomically, leaving
/// no temporary file behind.
#[test]
fn the_config_file_is_0600_and_written_atomically() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = open(&dir);
    must(store.set_token(Some(first_device()), "device-token"), "store token");
    must(
        store.set_last(
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 10), 9000)),
            Some(first_device()),
        ),
        "store endpoint",
    );

    let path = store.path().to_path_buf();
    assert!(path.is_file(), "the config file exists");
    #[cfg(unix)]
    assert_eq!(mode(&path), 0o600, "the config file is mode 0600");

    let text = must(fs::read_to_string(&path), "read config");
    let _: Config = must(toml::from_str(&text), "the file is valid TOML");

    let leftovers: Vec<String> = must(fs::read_dir(dir.path()), "read dir")
        .filter_map(Result::ok)
        .filter_map(|entry| entry.file_name().into_string().ok())
        .filter(|name| name != "config.toml")
        .collect();
    assert!(
        leftovers.is_empty(),
        "atomic writes leave no temporary file: {leftovers:?}"
    );
}

/// Per-device tokens and the manual default live side by side and survive a reopen.
#[test]
fn per_device_tokens_and_the_manual_default_round_trip() {
    let dir = must(tempfile::tempdir(), "temp dir");
    {
        let store = open(&dir);
        must(store.set_token(Some(first_device()), "first-token"), "first token");
        must(store.set_token(Some(second_device()), "second-token"), "second token");
        must(store.set_token(None, "manual-token"), "manual token");
    }

    let reopened = open(&dir);
    assert_eq!(
        reopened.token_for(Some(first_device())),
        Some(String::from("first-token"))
    );
    assert_eq!(
        reopened.token_for(Some(second_device())),
        Some(String::from("second-token"))
    );
    assert_eq!(reopened.token_for(None), Some(String::from("manual-token")));
}

/// A discovered device falls back to the manual default until it has a token of its own.
#[test]
fn a_discovered_device_falls_back_to_the_manual_default() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = open(&dir);
    must(store.set_token(None, "manual-token"), "manual token");
    assert_eq!(
        store.token_for(Some(first_device())),
        Some(String::from("manual-token")),
        "a LAN-only pair lands in default_token before a device is named"
    );
    must(store.set_token(Some(first_device()), "device-token"), "device token");
    assert_eq!(
        store.token_for(Some(first_device())),
        Some(String::from("device-token")),
        "the device's own token wins"
    );
}

/// The last endpoint and the last device id are remembered across a reopen.
#[test]
fn the_last_endpoint_and_device_are_remembered() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let endpoint = SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 20), 9100));
    {
        let store = open(&dir);
        must(store.set_last(endpoint, Some(second_device())), "store endpoint");
    }

    let reopened = open(&dir);
    assert_eq!(reopened.last_endpoint(), Some(endpoint));
    assert_eq!(reopened.last_device(), Some(second_device()));
}

/// A manual endpoint updates the endpoint but never invents a device id.
#[test]
fn a_manual_endpoint_keeps_the_last_device() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = open(&dir);
    must(
        store.set_last(
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 30), 9000)),
            Some(first_device()),
        ),
        "first endpoint",
    );
    must(
        store.set_last(
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 31), 9001)),
            None,
        ),
        "manual endpoint",
    );

    assert_eq!(
        store.last_endpoint(),
        Some(SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 31), 9001)))
    );
    assert_eq!(store.last_device(), Some(first_device()));
}

/// The `TokenGate` the app adapter uses answers with the last device's token, falling back to the
/// manual default, and `pair` stores the new code for that device.
#[test]
fn the_gate_uses_the_last_device_and_pair_stores_it() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = open(&dir);
    must(store.set_token(Some(first_device()), "first-token"), "first token");
    must(store.set_token(None, "manual-token"), "manual token");
    must(
        store.set_last(
            SocketAddr::V4(SocketAddrV4::new(Ipv4Addr::new(192, 0, 2, 40), 9000)),
            Some(first_device()),
        ),
        "last device",
    );

    assert_eq!(store.token(), Some(String::from("first-token")));
    store.pair("re-paired");
    assert_eq!(
        store.token_for(Some(first_device())),
        Some(String::from("re-paired")),
        "pair stores the new code for the last-used device"
    );

    let reopened = open(&dir);
    assert_eq!(reopened.token(), Some(String::from("re-paired")));
}

/// A missing file is an empty store, not an error, and the gate answers `None`.
#[test]
fn a_missing_config_is_an_empty_store() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let store = open(&dir);
    assert_eq!(store.token(), None);
    assert_eq!(store.token_for(Some(first_device())), None);
    assert_eq!(store.last_endpoint(), None);
    assert_eq!(store.last_device(), None);
}

/// A malformed config is a hard error rather than silently dropping the operator's tokens.
#[test]
fn a_malformed_config_is_an_error() {
    let dir = must(tempfile::tempdir(), "temp dir");
    let path = dir.path().join("config.toml");
    must(fs::write(&path, "not = [valid toml"), "write malformed file");
    assert!(Store::open(path).is_err());
}
