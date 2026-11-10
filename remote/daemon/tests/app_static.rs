//! Static serving: the embed, the disk override, the SPA fallback and ADR-0039's headers.

mod app_support;

use std::sync::Arc;

use app_support::{FakeGate, TempDir, TestServer, assert_static_headers, http_get, must, relay_with_session};
use remote_daemon::app::{Assets, CSP, LoopbackFree};

/// A disk-backed server with an index and one script.
async fn server_with_assets() -> (TempDir, TestServer) {
    let dir = must(TempDir::new("static"), "temp dir");
    must(
        dir.write("index.html", "<!doctype html><title>index page</title>"),
        "write index",
    );
    must(dir.write("app.js", "console.log('app');"), "write script");
    must(dir.write("nested/page.html", "<title>nested</title>"), "write nested");
    let server = must(
        TestServer::start_with(
            relay_with_session(1),
            FakeGate::empty(),
            Arc::new(LoopbackFree),
            Assets::disk(dir.path()),
        )
        .await,
        "server",
    );
    (dir, server)
}

/// `/` serves `index.html` with every ADR-0039 header.
#[tokio::test]
async fn the_root_serves_the_index() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/").await, "GET /");
    assert_eq!(response.status, 200);
    assert!(response.body.contains("index page"));
    assert_eq!(response.header("content-type"), Some("text/html; charset=utf-8"));
    assert_static_headers(&response);
}

/// An extensionless path falls back to the index.
#[tokio::test]
async fn an_extensionless_path_falls_back_to_the_index() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/some/deep/route").await, "GET route");
    assert_eq!(response.status, 200);
    assert!(response.body.contains("index page"));
    assert_static_headers(&response);
}

/// An existing asset is served with its content type.
#[tokio::test]
async fn an_existing_asset_is_served() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/app.js").await, "GET app.js");
    assert_eq!(response.status, 200);
    assert_eq!(response.body, "console.log('app');");
    assert_eq!(response.header("content-type"), Some("text/javascript; charset=utf-8"));
    assert_static_headers(&response);

    let nested = must(http_get(server.addr(), "/nested/page.html").await, "GET nested");
    assert_eq!(nested.status, 200);
    assert!(nested.body.contains("nested"));
}

/// A missing asset with an extension is 404, and the 404 still carries the headers.
#[tokio::test]
async fn a_missing_asset_is_404() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/missing.js").await, "GET missing");
    assert_eq!(response.status, 404);
    assert_static_headers(&response);
}

/// A missing extensionless path is not a 404: it is an SPA route and gets the index.
#[tokio::test]
async fn a_missing_extensionless_path_serves_the_index() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/missing").await, "GET missing");
    assert_eq!(response.status, 200);
    assert!(response.body.contains("index page"));
}

/// `/ws` is never a static fallback.
#[tokio::test]
async fn ws_is_never_a_fallback() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/ws").await, "GET /ws");
    assert!(
        response.status == 400 || response.status == 426,
        "a plain GET /ws should not be served statically, got {}",
        response.status
    );
    assert!(!response.body.contains("index page"));
}

/// A traversal attempt is refused rather than served from outside the asset root.
#[tokio::test]
async fn a_traversal_path_is_404() {
    let (_dir, server) = server_with_assets().await;
    let response = must(http_get(server.addr(), "/../Cargo.toml").await, "GET traversal");
    assert_eq!(response.status, 404);
    assert_static_headers(&response);
}

/// A folder without an index returns 404 rather than an empty 200.
#[tokio::test]
async fn an_empty_folder_returns_404() {
    let dir = must(TempDir::new("empty"), "temp dir");
    let server = must(
        TestServer::start_with(
            relay_with_session(1),
            FakeGate::empty(),
            Arc::new(LoopbackFree),
            Assets::disk(dir.path()),
        )
        .await,
        "server",
    );
    let response = must(http_get(server.addr(), "/").await, "GET /");
    assert_eq!(response.status, 404);
    assert_static_headers(&response);
}

/// The compile-time embed serves `remote/app` without a build step.
#[tokio::test]
async fn the_embedded_app_folder_is_served() {
    let server = must(
        TestServer::start_with(
            relay_with_session(1),
            FakeGate::empty(),
            Arc::new(LoopbackFree),
            Assets::embedded(),
        )
        .await,
        "server",
    );
    let index = must(http_get(server.addr(), "/").await, "GET /");
    assert_eq!(index.status, 200);
    assert!(index.body.contains("Explorer Remote"));
    assert_eq!(index.header("content-type"), Some("text/html; charset=utf-8"));
    assert_static_headers(&index);
    assert_eq!(index.header("content-security-policy"), Some(CSP));

    let response = must(http_get(server.addr(), "/README.md").await, "GET README");
    assert_eq!(response.status, 200);
    assert!(response.body.contains("Remote App"));
    assert_eq!(response.header("content-type"), Some("text/plain; charset=utf-8"));
    assert_static_headers(&response);

    // The bench suite includes a page that drives the robot on load, so it is not a
    // production asset.
    let tests = must(http_get(server.addr(), "/tests/real.html").await, "GET tests page");
    assert_eq!(tests.status, 404, "the browser suite needs --serve-tests");
    assert_static_headers(&tests);
}

/// `--serve-tests` explicitly exposes the bench suite.
#[tokio::test]
async fn the_bench_suite_is_served_only_when_asked() {
    let server = must(
        TestServer::start_with_tests(
            relay_with_session(1),
            FakeGate::empty(),
            Arc::new(LoopbackFree),
            Assets::embedded(),
        )
        .await,
        "server",
    );
    let page = must(http_get(server.addr(), "/tests/real.html").await, "GET tests page");
    assert_eq!(page.status, 200);
    assert_eq!(page.header("content-type"), Some("text/html; charset=utf-8"));
    assert_static_headers(&page);
}
