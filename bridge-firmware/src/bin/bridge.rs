#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! Ticket 04 — the Bridge firmware skeleton.
//!
//! This is the first end-to-end tracer bullet on the C3: a station-only build that drains the
//! robot-facing UART with UHCI RX DMA, feeds the bytes through `remote-protocol`'s parser into
//! `bridge-core`'s pump, and forwards frames over a raw TCP listener bound to the station
//! interface. There is no provisioning, no Discovery Beacon and no authentication yet.
//!
//! ## Wiring
//!
//! UART1 runs at ADR-0033's 921600 8N1 with **TX = GPIO6** and **RX = GPIO7**. Connect those two
//! pins, through a 3.3 V USB-TTL adapter, to the Robot Stub's serial port. The C3's USB connector
//! stays the flashing and `defmt` console.
//!
//! ## Credentials
//!
//! The station credentials are compile-time inputs so the binary contains no secret at rest:
//!
//! ```text
//! BRIDGE_STA_SSID=... BRIDGE_STA_PASSWORD=... cargo build --release
//! ```
//!
//! With neither set, or with either left as `CHANGE_ME`, the firmware still runs the UART and the
//! pump, but it logs a clear warning and never attempts to associate.
//!
//! ## The ticket-05 placeholder
//!
//! A TCP connection is admitted with a zero proof: both the presented proof and the expected
//! proof are `[0u8; 32]`, so the gate accepts immediately. That keeps the gate, the single-client
//! slot and the deadline paths wired while the `bridge-net` HMAC preamble waits for ticket 05.
//! **This is not authentication.**

use core::cell::RefCell;

use bridge_core::{
    BridgePump, Class, ConnectOutcome, Decision, EntryKind, Flags, Frame, Proof, ProofOutcome, Seq, TimerEvent,
};
use bridge_net::{ports::LINK_TCP_DEFAULT, preamble::PROOF_LEN};
use critical_section::Mutex;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{Runner, Stack, StackResources, tcp::TcpSocket};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    dma_rx_stream_buffer,
    timer::timg::TimerGroup,
    uart::{
        Config as UartConfig, RxConfig, TxError, Uart, UartTx,
        uhci::{RxConfig as UhciRxConfig, Uhci, UhciRx},
    },
};
use esp_println as _;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig, Interface, Ssid, WifiController,
    scan::ScanConfig, sta::StationConfig,
};
use heapless::spsc::Queue;
use portable_atomic::{AtomicU32, Ordering};
use remote_protocol::{
    Control, ControlMessage, Direction, MAX_PC_TO_ROBOT_FRAME, MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, PcToRobotParser,
    RobotToPcParser, ThrottleHint, encode_control,
};
use static_cell::StaticCell;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ---------------------------------------------------------------------------------------------
// Compile-time inputs and bench constants
// ---------------------------------------------------------------------------------------------

/// The station SSID. Pass `BRIDGE_STA_SSID` at build time.
const STA_SSID: &str = match option_env!("BRIDGE_STA_SSID") {
    Some(value) => value,
    None => "CHANGE_ME",
};

/// The station password. Pass `BRIDGE_STA_PASSWORD` at build time; it is never logged.
const STA_PASSWORD: &str = match option_env!("BRIDGE_STA_PASSWORD") {
    Some(value) => value,
    None => "CHANGE_ME",
};

/// ADR-0033's shared baud: all three projects run 921600 8N1.
const UART_BAUD: u32 = 921_600;

/// The UART RX FIFO-full threshold. The driver default of 120 bytes leaves only ~87 us of
/// headroom at 921600; 32 bytes leaves about 1 ms.
const UART_RX_THRESHOLD: u16 = 32;

/// UART1's pins on the C3. Keep in step with the `with_tx`/`with_rx` calls below.
const UART_TX_PIN: u8 = 6;
const UART_RX_PIN: u8 = 7;

/// The UHCI RX stream buffer and descriptor chunk size (ADR-0033's 4 KiB).
const DMA_RX_BUFFER: usize = 4096;
const DMA_RX_CHUNK: usize = 128;

/// The robot-bound frame queue. ADR-0033 budgets 1 KiB; four worst-case PC-to-robot frames fit.
const UART_TX_QUEUE_CAPACITY: usize = 4;

/// The socket buffers. They are heap-allocated once and reused across client connections.
const TCP_RX_BUFFER: usize = 4096;
const TCP_TX_BUFFER: usize = 4096;

/// How much one `read` may take from the client.
const CLIENT_READ_CHUNK: usize = 1024;

/// How often the client loop drains the pump when no bytes arrive.
const CLIENT_POLL: Duration = Duration::from_millis(10);

/// The status-line cadence, matching the spike.
const STATUS_PERIOD: Duration = Duration::from_secs(10);

/// The main loop period.
const LOOP_PERIOD: Duration = Duration::from_millis(500);

/// How long an association may take before the station task gives up and retries.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

/// How long DHCP may take after association before the link is treated as failed.
const DHCP_TIMEOUT: Duration = Duration::from_secs(30);

/// The smoltcp socket inactivity timeout.
const SOCKET_TIMEOUT: Duration = Duration::from_secs(10);

/// The reconnect backoff bounds (ADR-0036's pattern: 1, 2, 4 … 60 s).
const MIN_BACKOFF: Duration = Duration::from_secs(1);
const MAX_BACKOFF: Duration = Duration::from_secs(60);

// ---------------------------------------------------------------------------------------------
// Statics
// ---------------------------------------------------------------------------------------------

/// One station stack is enough: the Beacon and the portal are tickets 06 and 07.
static STA_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();

/// The pump is long-lived state and larger than any task's future, so it lives in a static.
static PUMP: Mutex<RefCell<BridgePump>> = Mutex::new(RefCell::new(BridgePump::new()));

/// The robot-bound frame queue. Two tasks produce (client frames and `THROTTLE` hints) and one
/// consumes, so a critical-section mutex serialises the producers.
static UART_TX_QUEUE: Mutex<RefCell<Queue<UartFrame, UART_TX_QUEUE_CAPACITY>>> = Mutex::new(RefCell::new(Queue::new()));

/// Runs `action` with the pump borrowed inside a critical section.
///
/// Every caller only reads or mutates the pump's in-memory state, so no lock is held across an
/// `await`.
fn with_pump<R>(action: impl FnOnce(&mut BridgePump) -> R) -> R {
    critical_section::with(|cs| action(&mut PUMP.borrow(cs).borrow_mut()))
}

/// Runs `action` with the robot-bound frame queue borrowed inside a critical section.
fn with_uart_queue<R>(action: impl FnOnce(&mut Queue<UartFrame, UART_TX_QUEUE_CAPACITY>) -> R) -> R {
    critical_section::with(|cs| action(&mut UART_TX_QUEUE.borrow(cs).borrow_mut()))
}

/// Spawns a task, logging the failure instead of panicking on a pool exhaustion.
fn spawn_or_warn<T>(
    spawner: &Spawner,
    token: Result<embassy_executor::SpawnToken<T>, embassy_executor::SpawnError>,
    label: &str,
) {
    match token {
        Ok(token) => spawner.spawn(token),
        Err(_) => warn!("executor: failed to spawn the {} task", label),
    }
}

static UART_RX_BYTES: AtomicU32 = AtomicU32::new(0);
static UART_TX_BYTES: AtomicU32 = AtomicU32::new(0);
static UART_DROPPED_BYTES: AtomicU32 = AtomicU32::new(0);
static UART_DMA_STALLS: AtomicU32 = AtomicU32::new(0);
static UART_RX_FRAMES: AtomicU32 = AtomicU32::new(0);
static UART_CRC_FAILURES: AtomicU32 = AtomicU32::new(0);
static UART_RESYNCS: AtomicU32 = AtomicU32::new(0);
static UART_TX_ERRORS: AtomicU32 = AtomicU32::new(0);
static UART_TX_QUEUE_DROPS: AtomicU32 = AtomicU32::new(0);
static TCP_RX_BYTES: AtomicU32 = AtomicU32::new(0);
static TCP_TX_BYTES: AtomicU32 = AtomicU32::new(0);
static TCP_ACCEPTS: AtomicU32 = AtomicU32::new(0);
static TCP_TX_ERRORS: AtomicU32 = AtomicU32::new(0);
static TCP_READ_ERRORS: AtomicU32 = AtomicU32::new(0);
static CLIENT_CRC_FAILURES: AtomicU32 = AtomicU32::new(0);
static CLIENT_RESYNCS: AtomicU32 = AtomicU32::new(0);
static RING_DISCARDED_ON_CLOSE: AtomicU32 = AtomicU32::new(0);
static RING_ENCODE_ERRORS: AtomicU32 = AtomicU32::new(0);
static THROTTLE_SEQ: AtomicU32 = AtomicU32::new(0);
static STATION_RECONNECTS: AtomicU32 = AtomicU32::new(0);

// ---------------------------------------------------------------------------------------------
// Entry point
// ---------------------------------------------------------------------------------------------

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!(
        "bridge firmware: ticket 04 skeleton, station-only, no auth, no Beacon; uart {} 8N1 TX=GPIO{} RX=GPIO{}",
        UART_BAUD, UART_TX_PIN, UART_RX_PIN
    );

    // -- Radio -------------------------------------------------------------------------------

    let mut controller = match WifiController::new(peripherals.WIFI, ControllerConfig::default()) {
        Ok(controller) => controller,
        Err(error) => {
            warn!("wifi: controller initialisation failed: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    };

    let mut station_configured = false;
    match build_station_config() {
        Some(station_config) => match controller.set_config(&WifiConfig::Station(station_config)) {
            Ok(()) => {
                station_configured = true;
                info!("sta: station mode configured for ssid={}", STA_SSID);
            }
            Err(error) => warn!("sta: set_config failed: {:?}", error),
        },
        None => warn!("sta: BRIDGE_STA_SSID/BRIDGE_STA_PASSWORD are empty or too long; station mode is not configured"),
    }

    // -- Network stack -----------------------------------------------------------------------

    let station_interface = Interface::station();
    let (station_stack, station_runner) = embassy_net::new(
        station_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        STA_RESOURCES.init(StackResources::new()),
        0x1234_5678_9abc_def0,
    );

    spawn_or_warn(&spawner, net_task(station_runner), "network");

    // -- UART --------------------------------------------------------------------------------

    let uart_config = UartConfig::default()
        .with_baudrate(UART_BAUD)
        .with_rx(RxConfig::default().with_fifo_full_threshold(UART_RX_THRESHOLD));

    let uart = match Uart::new(peripherals.UART1, uart_config) {
        Ok(uart) => uart.with_rx(peripherals.GPIO7).with_tx(peripherals.GPIO6),
        Err(error) => {
            warn!("uart: configuration rejected: {:?}", error);
            loop {
                Timer::after(Duration::from_secs(60)).await;
            }
        }
    };

    let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
    if let Err(error) = uhci.apply_rx_config(&UhciRxConfig::default().with_chunk_limit(DMA_RX_CHUNK as u16)) {
        warn!("uart: UHCI RX configuration rejected: {:?}", error);
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    }
    let (uhci_rx, uhci_tx) = uhci.into_async().split();

    info!(
        "uart rx: UHCI DMA, buffer={}B chunk={}B, threshold={}B; tx: async FIFO",
        DMA_RX_BUFFER, DMA_RX_CHUNK, UART_RX_THRESHOLD
    );

    spawn_or_warn(&spawner, uart_rx_dma_task(uhci_rx), "UART RX DMA");
    spawn_or_warn(&spawner, uart_tx_task(uhci_tx.uart_tx), "UART TX");

    // -- Station-only TCP listener -----------------------------------------------------------

    spawn_or_warn(&spawner, tcp_task(station_stack), "TCP");

    // -- Supervisory loop --------------------------------------------------------------------

    let started = Instant::now();
    let mut next_status = started + STATUS_PERIOD;
    let mut last_status_at = started;
    let mut last_uart_rx = 0u32;
    let mut last_tcp_rx = 0u32;

    let mut station_connected = false;
    let mut station_associated_at = started;
    let mut next_station_attempt = started;
    let mut backoff = MIN_BACKOFF;
    let mut logged_ip: Option<[u8; 4]> = None;
    let mut next_missing_warning = started;

    loop {
        Timer::after(LOOP_PERIOD).await;
        let now = Instant::now();
        let now_ms = now_ms();

        // The pump's clock-driven transitions: an authentication deadline and the refusal window.
        if let Some(event) = with_pump(|pump| pump.tick(now_ms)) {
            match event {
                TimerEvent::AuthenticationExpired { connection } => {
                    info!("pump: authentication deadline expired for connection {}", connection);
                }
                TimerEvent::RefusalExpired => info!("pump: refusal window ended"),
            }
        }

        // Turn ring pressure into a robot-facing THROTTLE hint.
        emit_throttle(now_ms);

        // -- Station association and reconnect ----------------------------------------------

        if !credentials_present() {
            if now >= next_missing_warning {
                warn!(
                    "sta: BRIDGE_STA_SSID/BRIDGE_STA_PASSWORD are unset or CHANGE_ME; the UART and pump still run, but there is no station link"
                );
                next_missing_warning = now + Duration::from_secs(60);
            }
        } else if station_configured {
            if station_connected {
                if !controller.is_connected() {
                    info!("sta: link lost; reconnecting");
                    STATION_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                    station_connected = false;
                    backoff = MIN_BACKOFF;
                    next_station_attempt = now + backoff;
                    backoff = next_backoff(backoff);
                } else if station_stack.config_v4().is_none()
                    && now.duration_since(station_associated_at) >= DHCP_TIMEOUT
                {
                    warn!("sta: no DHCP address after {}s; reconnecting", DHCP_TIMEOUT.as_secs());
                    STATION_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                    let _ = with_timeout(Duration::from_secs(10), controller.disconnect_async()).await;
                    station_connected = false;
                    backoff = MIN_BACKOFF;
                    next_station_attempt = Instant::now() + backoff;
                    backoff = next_backoff(backoff);
                }
            }

            if !station_connected && now >= next_station_attempt {
                scan_station_target(&mut controller).await;
                if connect_station(&mut controller).await {
                    info!("sta: associated; DHCP runs in the background");
                    station_connected = true;
                    station_associated_at = Instant::now();
                    backoff = MIN_BACKOFF;
                    logged_ip = None;
                } else {
                    STATION_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                    warn!("sta: reconnect in {}s", backoff.as_secs());
                    next_station_attempt = Instant::now() + backoff;
                    backoff = next_backoff(backoff);
                }
            }
        }

        // -- Status line ---------------------------------------------------------------------

        if now >= next_status {
            let now = Instant::now();
            if let Some(config) = station_stack.config_v4() {
                let [a, b, c, d] = config.address.address().octets();
                if logged_ip != Some([a, b, c, d]) {
                    info!("sta: address={}.{}.{}.{}", a, b, c, d);
                    logged_ip = Some([a, b, c, d]);
                }
            }

            let rssi = controller.rssi().unwrap_or_default();
            let interval = now.duration_since(last_status_at);
            let uart_rx = UART_RX_BYTES.load(Ordering::Relaxed);
            let uart_tx = UART_TX_BYTES.load(Ordering::Relaxed);
            let tcp_rx = TCP_RX_BYTES.load(Ordering::Relaxed);
            let tcp_tx = TCP_TX_BYTES.load(Ordering::Relaxed);
            let (ring_used, ring_reserved, drop_stream, drop_no_client, drop_reliable, drop_bridge_control) =
                with_pump(|pump| {
                    (
                        pump.used_bytes(),
                        pump.reserved_bytes(),
                        pump.dropped_streams(),
                        pump.dropped_without_client(),
                        pump.dropped_reliable(),
                        pump.dropped_bridge_controls(),
                    )
                });

            info!(
                "status: t={}s rssi={}dBm uart_rx={}B uart_tx={}B uart_kbps={} uart_drop={}B dma_stalls={} crc={} resync={} good={} bad={} tcp_rx={}B tcp_tx={}B tcp_kbps={} tcp_accepts={} tcp_tx_errors={} tcp_read_errors={} client_crc={} client_resync={} ring_used={}B ring_reserved={}B drop_stream={} drop_no_client={} drop_reliable={} drop_bridge_control={} drop_on_close={} ring_encode_errors={} tx_q_drop={} reconnects={}",
                now.duration_since(started).as_secs(),
                rssi,
                uart_rx,
                uart_tx,
                kbps(uart_rx - last_uart_rx, interval),
                UART_DROPPED_BYTES.load(Ordering::Relaxed),
                UART_DMA_STALLS.load(Ordering::Relaxed),
                UART_CRC_FAILURES.load(Ordering::Relaxed),
                UART_RESYNCS.load(Ordering::Relaxed),
                UART_RX_FRAMES.load(Ordering::Relaxed),
                UART_RESYNCS.load(Ordering::Relaxed),
                tcp_rx,
                tcp_tx,
                kbps(tcp_rx - last_tcp_rx, interval),
                TCP_ACCEPTS.load(Ordering::Relaxed),
                TCP_TX_ERRORS.load(Ordering::Relaxed),
                TCP_READ_ERRORS.load(Ordering::Relaxed),
                CLIENT_CRC_FAILURES.load(Ordering::Relaxed),
                CLIENT_RESYNCS.load(Ordering::Relaxed),
                u32::try_from(ring_used).unwrap_or(u32::MAX),
                u32::try_from(ring_reserved).unwrap_or(u32::MAX),
                drop_stream,
                drop_no_client,
                drop_reliable,
                drop_bridge_control,
                RING_DISCARDED_ON_CLOSE.load(Ordering::Relaxed),
                RING_ENCODE_ERRORS.load(Ordering::Relaxed),
                UART_TX_QUEUE_DROPS.load(Ordering::Relaxed),
                STATION_RECONNECTS.load(Ordering::Relaxed),
            );

            last_uart_rx = uart_rx;
            last_tcp_rx = tcp_rx;
            last_status_at = now;
            next_status = now + STATUS_PERIOD;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Wi-Fi helpers
// ---------------------------------------------------------------------------------------------

/// Returns whether usable build-time credentials were supplied.
fn credentials_present() -> bool {
    !STA_SSID.is_empty() && STA_SSID != "CHANGE_ME" && !STA_PASSWORD.is_empty() && STA_PASSWORD != "CHANGE_ME"
}

/// Builds the station configuration, or `None` when a build-time value is malformed.
#[allow(
    clippy::large_stack_frames,
    reason = "the station configuration is a boot-time value that lives in the radio driver"
)]
fn build_station_config() -> Option<StationConfig> {
    let ssid = Ssid::try_from(STA_SSID).ok()?;
    let password = STA_PASSWORD.try_into().ok()?;
    Some(
        StationConfig::default()
            .with_ssid(ssid)
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(password)),
    )
}

/// Doubles the reconnect backoff, capped at [`MAX_BACKOFF`].
fn next_backoff(backoff: Duration) -> Duration {
    (backoff * 2).min(MAX_BACKOFF)
}

/// Scans for the configured station SSID and logs every BSS's auth mode before associating.
///
/// `esp-radio` 1.0.0-beta.1 only exposes `Wpa2Personal`; a WPA3-only or PMF-required AP is the
/// usual cause of an association failure, and the advertised auth mode settles that before any
/// credentials are tried. `scan_async` on this beta can miss a sweep, so an empty result is retried.
async fn scan_station_target(controller: &mut WifiController<'_>) {
    let Ok(ssid) = Ssid::try_from(STA_SSID) else {
        warn!("sta: SSID too long to scan for; skipping the scan");
        return;
    };
    let config = ScanConfig::default().with_ssid(ssid);

    for attempt in 1..=3 {
        match controller.scan_async(&config).await {
            Ok(aps) if !aps.is_empty() => {
                for ap in &aps {
                    info!(
                        "sta: scan: bssid={:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x} ch={} rssi={}dBm auth={}",
                        ap.bssid[0],
                        ap.bssid[1],
                        ap.bssid[2],
                        ap.bssid[3],
                        ap.bssid[4],
                        ap.bssid[5],
                        ap.channel,
                        ap.signal_strength,
                        ap.auth_method
                    );
                }
                return;
            }
            Ok(_) => info!("sta: scan: sweep {} found nothing; retrying", attempt),
            Err(error) => warn!("sta: scan failed: {:?}", error),
        }
        if attempt < 3 {
            Timer::after(Duration::from_secs(1)).await;
        }
    }
    warn!("sta: scan: target SSID was not found in any sweep");
}

/// Associates the station, returning whether the controller reported success.
async fn connect_station(controller: &mut WifiController<'_>) -> bool {
    match with_timeout(CONNECT_TIMEOUT, controller.connect_async()).await {
        Ok(Ok(info)) => {
            info!("sta: associated ssid={} channel={}", info.ssid, info.channel);
            true
        }
        Ok(Err(error)) => {
            warn!("sta: association failed: {:?}", error);
            false
        }
        Err(_) => {
            warn!("sta: association timed out");
            false
        }
    }
}

/// Bits per second converted to whole kilobits per second for a status interval.
fn kbps(bytes: u32, interval: Duration) -> u32 {
    let seconds = interval.as_secs().max(1);
    ((u64::from(bytes) * 8) / seconds / 1_000) as u32
}

// ---------------------------------------------------------------------------------------------
// Network tasks
// ---------------------------------------------------------------------------------------------

/// The station stack's runner.
#[embassy_executor::task]
async fn net_task(mut runner: Runner<'static, Interface>) {
    info!("net: station runner started");
    runner.run().await;
}

/// The one-client Link Protocol listener, bound to the station interface only.
///
/// One connection is accepted at a time. The five-second authentication deadline, the refusal
/// window and the supersede rule are already in `bridge-core`; ticket 05 replaces the placeholder
/// proof below with the `bridge-net` preamble, which is also where an unauthenticated connection
/// stops blocking a new one.
#[allow(
    clippy::large_stack_frames,
    reason = "the task owns the socket buffers' references, the parsers and a 1 KiB frame buffer"
)]
#[embassy_executor::task]
async fn tcp_task(stack: Stack<'static>) {
    let mut rx_buffer = alloc::vec![0u8; TCP_RX_BUFFER];
    let mut tx_buffer = alloc::vec![0u8; TCP_TX_BUFFER];
    let mut client_buffer = alloc::vec![0u8; CLIENT_READ_CHUNK];
    let mut parser = PcToRobotParser::new(Direction::PcToRobot, UART_BAUD);

    loop {
        let mut socket = TcpSocket::new(stack, &mut rx_buffer[..], &mut tx_buffer[..]);
        socket.set_timeout(Some(SOCKET_TIMEOUT));

        info!("tcp: listening on port {} (station interface only)", LINK_TCP_DEFAULT);
        if socket.accept(LINK_TCP_DEFAULT).await.is_err() {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }
        TCP_ACCEPTS.fetch_add(1, Ordering::Relaxed);

        let now = now_ms();
        let connection = match with_pump(|pump| pump.connect(now)) {
            ConnectOutcome::Challenge { connection } => connection,
            ConnectOutcome::Busy => {
                warn!("tcp: another authenticated client holds the slot; refusing");
                socket.close();
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            ConnectOutcome::Refused { until_ms } => {
                warn!("tcp: the gate refuses connections until {}", until_ms);
                socket.close();
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        };

        // TODO(ticket 05): replace the zero proof with the bridge-net preamble's proof.
        let placeholder = Proof::from_bytes([0u8; PROOF_LEN]);
        let outcome = with_pump(|pump| pump.authenticate(connection, placeholder, &placeholder, now));
        match outcome {
            ProofOutcome::Accepted => {
                info!(
                    "tcp: client {} accepted with the ticket-05 placeholder proof",
                    connection
                );
            }
            ProofOutcome::Rejected { failures } => {
                warn!("tcp: placeholder proof rejected ({} failures)", failures);
                with_pump(|pump| pump.disconnect(connection));
                socket.close();
                Timer::after(Duration::from_millis(200)).await;
                continue;
            }
            ProofOutcome::Refused { until_ms, failures } => {
                warn!("tcp: proof refused until {} after {} failures", until_ms, failures);
                socket.close();
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
            ProofOutcome::DeadlineExpired => {
                warn!("tcp: authentication deadline expired");
                with_pump(|pump| pump.disconnect(connection));
                socket.close();
                continue;
            }
            ProofOutcome::Superseded => {
                warn!("tcp: connection was superseded before it authenticated");
                socket.close();
                continue;
            }
        }

        // A previous client may have left a partial candidate in the parser; discard it so its
        // bytes cannot be glued to this client's first frame.
        parser.reset();

        loop {
            if !drain_to_socket(&mut socket).await {
                info!("tcp: socket write failed; closing the client");
                break;
            }

            // Poll here as well as in the supervisory loop: ring pressure builds while a client
            // is attached, and this loop runs every `CLIENT_POLL`.
            emit_throttle(now_ms());

            match with_timeout(CLIENT_POLL, socket.read(&mut client_buffer[..])).await {
                Ok(Ok(0)) => {
                    info!("tcp: client closed");
                    break;
                }
                Ok(Ok(read)) => {
                    TCP_RX_BYTES.fetch_add(u32::try_from(read).unwrap_or(u32::MAX), Ordering::Relaxed);
                    let now = now_ms();
                    parser.feed(&client_buffer[..read], now, |frame| {
                        match with_pump(|pump| pump.on_client_frame(&frame)) {
                            Decision::ForwardToRobot => forward_to_robot(&frame),
                            Decision::DroppedBridgeControl => {
                                warn!("tcp: BRIDGE_CONTROL from the client dropped and counted");
                            }
                            _ => {}
                        }
                    });
                    CLIENT_CRC_FAILURES.store(parser.crc_failures(), Ordering::Relaxed);
                    CLIENT_RESYNCS.store(parser.resyncs(), Ordering::Relaxed);
                }
                Ok(Err(_)) => {
                    TCP_READ_ERRORS.fetch_add(1, Ordering::Relaxed);
                    info!("tcp: socket read failed; closing the client");
                    break;
                }
                Err(_) => {}
            }
        }

        // A closing client leaves the ring as state, not history. Drop what it never read, and
        // count it: a reliable entry must never disappear silently.
        let discarded = with_pump(|pump| {
            let mut count = 0u32;
            pump.drain(|_| count = count.wrapping_add(1));
            count
        });
        if discarded > 0 {
            RING_DISCARDED_ON_CLOSE.fetch_add(discarded, Ordering::Relaxed);
            warn!("tcp: {} client-bound entries discarded at disconnect", discarded);
        }

        with_pump(|pump| {
            pump.disconnect(connection);
        });
        socket.close();
        Timer::after(Duration::from_millis(100)).await;
    }
}

/// Writes every ready pump entry to the socket, one frame per entry.
///
/// Returns `false` when the socket can no longer be written; the caller closes it, because a
/// partial frame would corrupt the stream.
#[allow(
    clippy::large_stack_frames,
    reason = "one robot-to-PC frame is the largest single encode buffer in the firmware"
)]
async fn drain_to_socket(socket: &mut TcpSocket<'_>) -> bool {
    loop {
        let mut buffer = [0u8; MAX_ROBOT_TO_PC_FRAME];
        let mut encode_failed = false;
        let encoded = with_pump(|pump| {
            let entry = pump.pop()?;
            let (class, flags, id, seq) = entry_parts(entry.kind);
            let frame = match Frame::new(PROTOCOL_MAJOR, class, flags, id, seq, entry.payload) {
                Ok(frame) => frame,
                Err(_) => {
                    encode_failed = true;
                    return None;
                }
            };
            match frame.encode_into(&mut buffer) {
                Ok(len) => Some(len),
                Err(_) => {
                    encode_failed = true;
                    None
                }
            }
        });
        if encode_failed {
            RING_ENCODE_ERRORS.fetch_add(1, Ordering::Relaxed);
            warn!("tcp: could not encode a client-bound ring entry; dropped");
        }

        let Some(len) = encoded else {
            return true;
        };
        if !write_socket_all(socket, &buffer[..len]).await {
            return false;
        }
    }
}

/// Encodes a parsed client frame for the robot-facing UART.
fn forward_to_robot(frame: &Frame<'_>) {
    let Ok(rebuilt) = Frame::new(
        PROTOCOL_MAJOR,
        frame.header.class,
        frame.header.flags,
        frame.header.id,
        frame.header.seq,
        frame.payload,
    ) else {
        warn!("uart tx: could not rebuild the client frame");
        return;
    };
    enqueue_encoded(&rebuilt);
}

/// Polls the pump's watermark policy and queues any resulting `THROTTLE` hint.
fn emit_throttle(now_ms: u32) {
    let Some(hint) = with_pump(|pump| pump.poll_throttle(now_ms)) else {
        return;
    };
    let sequence = THROTTLE_SEQ.fetch_add(1, Ordering::Relaxed);
    let sequence = u16::try_from(sequence & u32::from(u16::MAX)).unwrap_or(0);
    queue_throttle(hint, sequence);
}

/// Encodes a `THROTTLE` hint as a control frame for the robot-facing UART.
fn queue_throttle(hint: ThrottleHint, sequence: u16) {
    let mut payload = [0u8; ThrottleHint::ENCODED_LEN];
    let Ok(len) = encode_control(&ControlMessage::Throttle(hint), &mut payload) else {
        warn!("uart tx: could not encode the THROTTLE hint");
        return;
    };
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR,
        Class::Control,
        Flags::default(),
        Control::Throttle.wire_id(),
        Seq::new(sequence),
        &payload[..len],
    ) else {
        warn!("uart tx: could not build the THROTTLE frame");
        return;
    };
    enqueue_encoded(&frame);
}

/// Encodes one frame into the bounded robot-bound queue.
fn enqueue_encoded(frame: &Frame<'_>) {
    let mut queued = UartFrame::EMPTY;
    let Ok(len) = frame.encode_into(&mut queued.bytes) else {
        warn!("uart tx: frame encoding failed; dropped");
        return;
    };
    let Ok(len) = u16::try_from(len) else {
        warn!("uart tx: frame length does not fit a u16; dropped");
        return;
    };
    queued.len = len;

    let accepted = with_uart_queue(|queue| queue.enqueue(queued).is_ok());
    if !accepted {
        UART_TX_QUEUE_DROPS.fetch_add(1, Ordering::Relaxed);
        warn!("uart tx: queue full; dropped a robot-bound frame (see tx_q_drop)");
    }
}

/// Writes the whole slice; `write` may accept only part of the socket's transmit buffer.
async fn write_socket_all(socket: &mut TcpSocket<'_>, data: &[u8]) -> bool {
    let mut written = 0;
    while written < data.len() {
        match socket.write(&data[written..]).await {
            Ok(0) => {
                TCP_TX_ERRORS.fetch_add(1, Ordering::Relaxed);
                return false;
            }
            Ok(count) => {
                written += count;
                TCP_TX_BYTES.fetch_add(u32::try_from(count).unwrap_or(u32::MAX), Ordering::Relaxed);
            }
            Err(_) => {
                TCP_TX_ERRORS.fetch_add(1, Ordering::Relaxed);
                return false;
            }
        }
    }
    true
}

/// Returns the class, flags, id and sequence to re-encode a pump entry.
///
/// The ring stores the wire header with every entry, so a forwarded frame is rebuilt with the
/// values it arrived with rather than regaining an implied set.
fn entry_parts(kind: EntryKind) -> (Class, Flags, u16, Seq) {
    match kind {
        EntryKind::Stream { id, seq, flags } => (Class::Stream, flags, id, seq),
        EntryKind::Reliable { class, id, seq, flags } => (class, flags, id, seq),
    }
}

// ---------------------------------------------------------------------------------------------
// UART tasks
// ---------------------------------------------------------------------------------------------

/// Drains the UHCI RX stream buffer straight into the robot-to-PC parser.
///
/// The DMA engine owns the UART FIFO, so missing the 1.4 ms FIFO deadline under Wi-Fi load is not
/// fatal; the stream buffer absorbs the descriptor-copy jitter and the parser owns the frame
/// assembly. A descriptor exhaustion is logged and counted, which is the loss indicator.
#[allow(
    clippy::large_stack_frames,
    reason = "the task future owns the UHCI transfer, its stream buffer and the 1 KiB parser"
)]
#[embassy_executor::task]
async fn uart_rx_dma_task(mut rx: UhciRx<'static, Async>) {
    let mut parser = RobotToPcParser::new(Direction::RobotToPc, UART_BAUD);
    let mut buffer = dma_rx_stream_buffer!(DMA_RX_BUFFER, DMA_RX_CHUNK);

    let mut transfer = loop {
        match rx.read(buffer) {
            Ok(transfer) => break transfer,
            Err((error, returned_rx, returned_buffer)) => {
                warn!("uart dma: start failed: {:?}", error);
                rx = returned_rx;
                buffer = returned_buffer;
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let mut stalled = false;

    loop {
        // `is_done` becomes true only when the descriptor ring is exhausted and DMA has stopped;
        // normal flow keeps recycling descriptors and stays false.
        if transfer.is_done() {
            if !stalled {
                UART_DMA_STALLS.fetch_add(1, Ordering::Relaxed);
                warn!("uart dma: descriptor ring exhausted; DMA stopped");
                stalled = true;
            }
        } else {
            stalled = false;
        }

        let total = {
            let data = transfer.peek();
            let total = data.len();
            if total > 0 {
                UART_RX_BYTES.fetch_add(u32::try_from(total).unwrap_or(u32::MAX), Ordering::Relaxed);

                let now = now_ms();
                let mut frames = 0u32;
                parser.feed(data, now, |frame| {
                    frames = frames.wrapping_add(1);
                    match with_pump(|pump| pump.on_robot_frame(&frame)) {
                        Decision::DroppedFull => warn!("pump: ring full; dropped a robot frame"),
                        Decision::BridgeControl(request) => info!(
                            "pump: robot BRIDGE_CONTROL action={} seq={} consumed (ticket 06/07 handoff)",
                            request.action.bits(),
                            request.seq.get()
                        ),
                        _ => {}
                    }
                });

                UART_RX_FRAMES.fetch_add(frames, Ordering::Relaxed);
                UART_CRC_FAILURES.store(parser.crc_failures(), Ordering::Relaxed);
                UART_RESYNCS.store(parser.resyncs(), Ordering::Relaxed);
                UART_DROPPED_BYTES.store(parser.discarded_bytes(), Ordering::Relaxed);
            }
            total
        };

        if total == 0 {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        }

        // The parser has finished with the slice; consuming it lets the DMA descriptors recycle.
        let _ = transfer.consume(total);
    }
}

/// Writes robot-bound frames from the bounded queue with the async UART TX path (no TX DMA).
#[embassy_executor::task]
async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    loop {
        let frame = with_uart_queue(|queue| queue.dequeue());
        let Some(frame) = frame else {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        };

        let len = usize::from(frame.len);
        match write_all(&mut tx, &frame.bytes[..len]).await {
            Ok(()) => {
                UART_TX_BYTES.fetch_add(u32::try_from(len).unwrap_or(u32::MAX), Ordering::Relaxed);
            }
            Err(error) => {
                UART_TX_ERRORS.fetch_add(1, Ordering::Relaxed);
                warn!("uart tx: write failed: {:?}", error);
                Timer::after(Duration::from_millis(10)).await;
            }
        }
    }
}

/// Writes the whole slice; `write_async` may accept only part of the FIFO.
async fn write_all(tx: &mut UartTx<'static, Async>, mut data: &[u8]) -> Result<(), TxError> {
    while !data.is_empty() {
        let written = tx.write_async(data).await?;
        data = &data[written..];
    }
    Ok(())
}

// ---------------------------------------------------------------------------------------------
// Small types
// ---------------------------------------------------------------------------------------------

/// One pre-encoded robot-bound frame in the UART TX queue.
#[derive(Clone, Copy)]
struct UartFrame {
    /// The encoded frame's length.
    len: u16,
    /// The encoded frame's bytes.
    bytes: [u8; MAX_PC_TO_ROBOT_FRAME],
}

impl UartFrame {
    /// An empty frame, ready to be filled by `encode_into`.
    const EMPTY: Self = Self {
        len: 0,
        bytes: [0; MAX_PC_TO_ROBOT_FRAME],
    };
}

// ---------------------------------------------------------------------------------------------
// Clock
// ---------------------------------------------------------------------------------------------

/// A monotonic millisecond clock, wrapping at `u32::MAX` so the pump's wrapping arithmetic is
/// valid across the boundary.
fn now_ms() -> u32 {
    u32::try_from(Instant::now().as_millis() & u64::from(u32::MAX)).unwrap_or(0)
}
