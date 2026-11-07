//! The binary: the same scenario, standalone (ticket 09).

mod support;

use std::{
    net::{IpAddr, Ipv4Addr, SocketAddr, TcpListener, UdpSocket},
    process::{Child, Command, Stdio},
    thread,
    time::{Duration, Instant},
};

use support::BenchError;

/// The token the binary pairs with.
const TOKEN: &[u8] = b"pairing-token";

/// The same token as hex for `--token`.
const TOKEN_HEX: &str = "70616972696e672d746f6b656e";

/// Kills the child when the test ends, however it ends.
struct KillOnDrop(Child);

impl Drop for KillOnDrop {
    fn drop(&mut self) {
        let _ = self.0.kill();
        let _ = self.0.wait();
    }
}

/// Runs the binary with `args` and returns the child.
fn spawn(args: &[String]) -> Result<Child, BenchError> {
    Command::new(env!("CARGO_BIN_EXE_bridge-stub"))
        .args(args)
        .stdout(Stdio::piped())
        .spawn()
        .map_err(|error| BenchError::Io(error.to_string()))
}

/// Binds and releases a loopback TCP port, yielding it for the binary's listener.
fn free_tcp_port() -> Result<u16, BenchError> {
    let listener =
        TcpListener::bind((IpAddr::V4(Ipv4Addr::LOCALHOST), 0)).map_err(|error| BenchError::Io(error.to_string()))?;
    listener
        .local_addr()
        .map(|addr| addr.port())
        .map_err(|error| BenchError::Io(error.to_string()))
}

/// Binds and releases a loopback UDP port, yielding it for the binary's beacon.
fn free_udp_port() -> Result<u16, BenchError> {
    let socket =
        UdpSocket::bind((IpAddr::V4(Ipv4Addr::LOCALHOST), 0)).map_err(|error| BenchError::Io(error.to_string()))?;
    socket
        .local_addr()
        .map(|addr| addr.port())
        .map_err(|error| BenchError::Io(error.to_string()))
}

/// The binary prints its usage for `--help` and exits successfully.
#[test]
fn the_binary_prints_its_usage() -> Result<(), BenchError> {
    let child = spawn(&[String::from("--help")])?;
    let output = child
        .wait_with_output()
        .map_err(|error| BenchError::Io(error.to_string()))?;
    assert!(output.status.success(), "--help exits successfully");
    let stdout = String::from_utf8_lossy(&output.stdout);
    assert!(stdout.contains("usage: bridge-stub"), "the usage is printed");
    assert!(stdout.contains("--deny <reason>"), "the deny knob is documented");
    Ok(())
}

/// The binary serves a preamble a real client can complete, standalone.
///
/// The port is chosen by binding and releasing it, so a parallel test process can steal it in the
/// gap; each attempt therefore uses fresh ports and is retried a few times before failing.
#[test]
fn the_binary_serves_a_preamble() -> Result<(), BenchError> {
    let mut last = None;
    for _ in 0..4 {
        match serve_once() {
            Ok(identity) => {
                assert_eq!(identity.name.as_str(), "Explorer-STUB");
                return Ok(());
            }
            Err(error) => last = Some(error),
        }
    }
    Err(last.unwrap_or_else(|| BenchError::Io(String::from("the binary never served a preamble"))))
}

/// One spawn-and-connect attempt on freshly chosen loopback ports.
fn serve_once() -> Result<bridge_stub::Identity, BenchError> {
    let port = free_tcp_port()?;
    let beacon_port = free_udp_port()?;
    let args = vec![
        String::from("--token"),
        String::from(TOKEN_HEX),
        String::from("--link-port"),
        port.to_string(),
        String::from("--beacon-addr"),
        format!("127.0.0.1:{beacon_port}"),
    ];
    let child = spawn(&args)?;
    let guard = KillOnDrop(child);

    let addr = SocketAddr::new(IpAddr::V4(Ipv4Addr::LOCALHOST), port);
    let (_client, identity) = connect_until(addr, Duration::from_secs(10))?;
    drop(guard);
    Ok(identity)
}

/// Tries the handshake until the binary's listener is up, within `timeout`.
fn connect_until(addr: SocketAddr, timeout: Duration) -> Result<(support::Client, bridge_stub::Identity), BenchError> {
    let start = Instant::now();
    loop {
        match support::accepted(addr, TOKEN) {
            Ok(pair) => return Ok(pair),
            Err(BenchError::Connect(_)) if start.elapsed() < timeout => {
                thread::sleep(Duration::from_millis(50));
            }
            Err(error) => return Err(error),
        }
    }
}
