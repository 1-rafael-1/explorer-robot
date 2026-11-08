//! ADR-0039's slow-app rule: a connection that makes no write progress for 2 s is closed.
//!
//! The seam is a real WebSocket. The server is given a snapshot too large for the socket buffers,
//! and the client deliberately never reads it, so the writer blocks and must time out. The
//! observable outcome is the relay app's disconnect, reported by the hooks when the session task
//! ends.

mod app_support;

use std::{
    sync::{Arc, Mutex},
    time::Duration,
};

use app_support::{FakeGate, RecordingHooks, TestServer, connect, must};
use remote_daemon::{
    app::{AppHooks, Assets, LoopbackFree},
    clock,
    relay::Relay,
    schema::{Catalogue, CatalogueProcedure},
};

/// Builds a catalogue whose snapshot cannot fit in the kernel's socket buffers.
fn huge_catalogue() -> Catalogue {
    let procedures = (0..40_000)
        .map(|index| CatalogueProcedure {
            id: u16::try_from(index).unwrap_or(u16::MAX),
            label: "x".repeat(256),
            available: true,
            interactive: false,
            calibration: false,
            takes_params: false,
        })
        .collect();
    Catalogue { procedures }
}

/// A client that stops reading is disconnected within roughly the two-second write timeout.
#[tokio::test]
async fn a_client_that_stops_reading_is_disconnected() {
    let now = clock::now_ms();
    let mut relay = Relay::new();
    relay.session_up(1, now);
    relay.on_catalogue(huge_catalogue(), now);

    let hooks: Arc<RecordingHooks> = Arc::new(RecordingHooks::default());
    let server = must(
        TestServer::start_full(
            Arc::new(Mutex::new(relay)),
            FakeGate::empty(),
            Arc::new(LoopbackFree),
            Assets::embedded(),
            Arc::clone(&hooks) as Arc<dyn AppHooks>,
        )
        .await,
        "server",
    );

    // Complete the upgrade, then never poll the stream: the server's writer fills the buffers.
    let _client = must(connect(server.addr()).await, "connect");

    let deadline = tokio::time::Instant::now() + Duration::from_secs(8);
    while hooks.last_disconnects() == 0 {
        assert!(
            tokio::time::Instant::now() < deadline,
            "the non-reading connection was not disconnected"
        );
        tokio::time::sleep(Duration::from_millis(50)).await;
    }
    assert!(
        tokio::time::Instant::now() < deadline,
        "the write timeout closed the connection"
    );
    server.shutdown();
}
