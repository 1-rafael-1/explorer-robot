#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! Ticket 05 — the network edge.
//!
//! The Bridge is the trust boundary for the robot (ADR-0037): the `bridge-net` HMAC preamble
//! completes before a single Link Protocol byte crosses, exactly one authenticated client holds the
//! slot, failed proofs are rate-limited by `bridge-core`'s gate, and the Discovery Beacon is
//! broadcast on the station interface every 3 s with the pinned flags.
//!
//! ## Wiring
//!
//! UART1 runs at ADR-0033's 921600 8N1 with **TX = GPIO6** and **RX = GPIO7**. Connect those two
//! pins, through a 3.3 V USB-TTL adapter, to the Robot Stub's serial port. The C3's USB connector
//! stays the flashing and `defmt` console. A GPIO6↔GPIO7 jumper loops the UART back for bench
//! runs without an adapter; a valid robot-side frame then sets the Beacon's link-ready flag.
//!
//! ## Credentials
//!
//! The station credentials are compile-time inputs so the binary contains no secret at rest:
//!
//! ```text
//! BRIDGE_STA_SSID=... BRIDGE_STA_PASSWORD=... BRIDGE_BENCH_TOKEN=... cargo build --release
//! ```
//!
//! With neither station variable set, or with either left as `CHANGE_ME`, the firmware still runs
//! the UART, the pump and the Beacon task, but it logs a clear warning and never attempts to
//! associate.
//!
//! ## The bench token (TODO(ticket 07))
//!
//! Ticket 07 owns the credential record and mints the 60-bit pairing token. Until then the HMAC key
//! is the **UTF-8 bytes of `BRIDGE_BENCH_TOKEN`**, compiled in exactly like the station
//! credentials. The daemon stores the same text and passes the string's bytes to `bridge-net`, so
//! both sides agree: the token is its text, never a decoded form of it. With the variable unset,
//! every client is denied rather than admitted, and the firmware logs it.
//!
//! ## The Beacon flags
//!
//! `provisioned` is false until ticket 07 stores a record; `setup_mode` is false while the firmware
//! is station-only; `tcp_port` is the Link listener's port while it is up; and `link-ready` is
//! `bridge_net::beacon::FLAG_LINK_READY`'s pinned meaning — a valid Link Protocol frame from the
//! robot within `beacon::LINK_READY_WINDOW_MS`.

use core::cell::RefCell;

use bridge_core::{
    BridgePump, Class, ConnectOutcome, Decision, EntryKind, Flags, Frame, Proof, ProofOutcome, Seq, TimerEvent,
};
use bridge_net::{
    beacon::{self, Beacon},
    identity::{DeviceId, Flags as IdentityFlags, Identity, Name},
    ports::{BEACON_UDP, LINK_TCP_DEFAULT},
    preamble::{self, DenyReason, HEADER_LEN, MAX_PAYLOAD_LEN, Message, NONCE_LEN, Received},
};
use critical_section::Mutex;
use defmt::{info, warn};
use embassy_executor::Spawner;
use embassy_net::{
    Ipv4Address, Runner, Stack, StackResources,
    tcp::TcpSocket,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::{
    Async,
    clock::CpuClock,
    dma_rx_stream_buffer,
    efuse::{InterfaceMacAddress, interface_mac_address},
    rng::Rng,
    timer::timg::TimerGroup,
    uart::{
        Config as UartConfig, RxConfig, TxError, Uart, UartTx,
        uhci::{RxConfig as UhciRxConfig, TxConfig as UhciTxConfig, Uhci, UhciRx},
    },
};
use esp_println as _;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig, Interface, Ssid, WifiController,
    scan::ScanConfig, sta::StationConfig,
};
use heapless::spsc::Queue;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};
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

/// The bench pairing token: the HMAC key's UTF-8 bytes (see the module docs).
///
/// TODO(ticket 07): replace this build-time input with the credential record's minted token. The
/// daemon stores the same text and passes its bytes to `bridge-net`, so the encoding is the token
/// text itself, never a decoded form. An empty value denies every client.
const BENCH_TOKEN: &str = match option_env!("BRIDGE_BENCH_TOKEN") {
    Some(value) => value,
    None => "",
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

/// How long a done-but-silent DMA has to stay silent before it is counted as a stall. The idle
/// EOF makes `is_done` fire at packet boundaries, so a transfer that is merely idle between
/// packets must not be mistaken for an exhausted descriptor ring.
const DMA_STALL_AFTER: Duration = Duration::from_secs(1);

/// The robot-bound frame queue. ADR-0033 budgets 1 KiB; four worst-case PC-to-robot frames fit.
const UART_TX_QUEUE_CAPACITY: usize = 4;

/// The socket buffers. They are heap-allocated once and reused across client connections.
const TCP_RX_BUFFER: usize = 4096;
const TCP_TX_BUFFER: usize = 4096;

/// How much one `read` may take from the client.
const CLIENT_READ_CHUNK: usize = 1024;

/// How often the client loop drains the pump when no bytes arrive.
const CLIENT_POLL: Duration = Duration::from_millis(10);

/// The whole preamble's budget: `bridge-net`'s pinned 5 s (ADR-0037).
const AUTH_DEADLINE: Duration = Duration::from_millis(preamble::AUTH_DEADLINE_MS);

/// The largest framed preamble message: the header plus the challenge's payload.
const PREAMBLE_BUFFER: usize = HEADER_LEN + MAX_PAYLOAD_LEN;

/// How long a socket flush may take while a verdict is delivered or a socket is recycled.
const FLUSH_TIMEOUT: Duration = Duration::from_millis(500);

/// How long a worker lets its FIN reach the peer before forcing the socket back to `Closed` for
/// the next accept.
const CLOSE_SETTLE: Duration = Duration::from_millis(100);

/// The `DENY(busy)` retry hint (ADR-0039's 4 s default).
const BUSY_RETRY_MS: u32 = 4_000;

/// The refusal window, surfaced as the `unauthorized` retry hint when the gate is rate-limiting.
const REFUSAL_RETRY_MS: u32 = 60_000;

/// `LAST_ROBOT_FRAME_MS`'s "never" sentinel.
const NO_ROBOT_FRAME: u32 = u32::MAX;

/// The Beacon socket's buffers: one metadata slot each, larger than the datagram so a later minor
/// can extend it.
const BEACON_SOCKET_BUFFER: usize = 64;

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

/// The station stack's socket pool: DHCP, the Beacon's UDP socket and the three TCP workers.
static STA_RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();

/// Whether the station is associated and holds an address; the Beacon is sent only then.
static STATION_UP: AtomicBool = AtomicBool::new(false);

/// When the last valid Link Protocol frame arrived from the robot, in `now_ms`'s scale.
static LAST_ROBOT_FRAME_MS: AtomicU32 = AtomicU32::new(NO_ROBOT_FRAME);

/// The efuse station MAC's six bytes, read once at boot and served in the identity.
static DEVICE_ID: Mutex<RefCell<[u8; 6]>> = Mutex::new(RefCell::new([0; 6]));

/// The Beacon's UDP receive metadata and buffer (received datagrams are discarded).
static BEACON_RX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
static BEACON_RX_BUFFER: StaticCell<[u8; BEACON_SOCKET_BUFFER]> = StaticCell::new();
/// The Beacon's UDP transmit metadata and buffer.
static BEACON_TX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
static BEACON_TX_BUFFER: StaticCell<[u8; BEACON_SOCKET_BUFFER]> = StaticCell::new();

/// Each connection worker owns its socket buffers for the firmware's lifetime.
static CLIENT_RX_0: StaticCell<[u8; TCP_RX_BUFFER]> = StaticCell::new();
static CLIENT_RX_1: StaticCell<[u8; TCP_RX_BUFFER]> = StaticCell::new();
static CLIENT_RX_2: StaticCell<[u8; TCP_RX_BUFFER]> = StaticCell::new();
static CLIENT_TX_0: StaticCell<[u8; TCP_TX_BUFFER]> = StaticCell::new();
static CLIENT_TX_1: StaticCell<[u8; TCP_TX_BUFFER]> = StaticCell::new();
static CLIENT_TX_2: StaticCell<[u8; TCP_TX_BUFFER]> = StaticCell::new();

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

/// Runs `action` with the efuse station MAC borrowed inside a critical section.
fn with_device_id<R>(action: impl FnOnce(&[u8; 6]) -> R) -> R {
    critical_section::with(|cs| action(&DEVICE_ID.borrow(cs).borrow()))
}

/// Stores the efuse station MAC once at boot.
fn store_device_id(bytes: [u8; 6]) {
    critical_section::with(|cs| *DEVICE_ID.borrow(cs).borrow_mut() = bytes);
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
static AUTH_ACCEPTED: AtomicU32 = AtomicU32::new(0);
static AUTH_REJECTED: AtomicU32 = AtomicU32::new(0);
static AUTH_TIMEOUTS: AtomicU32 = AtomicU32::new(0);
static AUTH_SUPERSEDED: AtomicU32 = AtomicU32::new(0);
static AUTH_FAILED: AtomicU32 = AtomicU32::new(0);
static DENIED_BUSY: AtomicU32 = AtomicU32::new(0);
static DENIED_REFUSED: AtomicU32 = AtomicU32::new(0);
static DENIED_VERSION: AtomicU32 = AtomicU32::new(0);
static BEACONS_SENT: AtomicU32 = AtomicU32::new(0);
static BEACON_SEND_ERRORS: AtomicU32 = AtomicU32::new(0);

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

    let mac = interface_mac_address(InterfaceMacAddress::Station);
    let mut device_id = [0u8; 6];
    device_id.copy_from_slice(mac.as_bytes());
    store_device_id(device_id);

    info!(
        "bridge firmware: ticket 05 network edge, station-only, bridge-net preamble, Beacon every {}ms; uart {} 8N1 TX=GPIO{} RX=GPIO{}",
        beacon::INTERVAL_MS,
        UART_BAUD,
        UART_TX_PIN,
        UART_RX_PIN
    );
    info!(
        "bridge identity: device_id={:02x}{:02x}{:02x}{:02x}{:02x}{:02x} name={} token={}",
        device_id[0],
        device_id[1],
        device_id[2],
        device_id[3],
        device_id[4],
        device_id[5],
        bridge_name().as_str(),
        if bench_token().is_some() {
            "configured"
        } else {
            "MISSING"
        }
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
    // `Uhci::new` clears CONF0, which disables both EOF modes. Without the idle EOF the DMA only
    // hands a packet back once `pkt_thres` (128 B) is reached, so a short or intermittent Link
    // frame would sit in the DMA until the next burst filled the packet. The idle EOF ends a
    // partial packet when the line goes quiet, which is what the robot's baseline traffic needs.
    if let Err(error) = uhci.apply_tx_config(&UhciTxConfig::default()) {
        warn!("uart: UHCI EOF configuration rejected: {:?}", error);
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

    // -- Station-only network edge -----------------------------------------------------------

    if bench_token().is_none() {
        warn!("tcp: BRIDGE_BENCH_TOKEN is unset; every preamble will be denied (TODO ticket 07)");
    }

    let client_rx_0: &'static mut [u8] = CLIENT_RX_0.init([0u8; TCP_RX_BUFFER]);
    let client_rx_1: &'static mut [u8] = CLIENT_RX_1.init([0u8; TCP_RX_BUFFER]);
    let client_rx_2: &'static mut [u8] = CLIENT_RX_2.init([0u8; TCP_RX_BUFFER]);
    let client_tx_0: &'static mut [u8] = CLIENT_TX_0.init([0u8; TCP_TX_BUFFER]);
    let client_tx_1: &'static mut [u8] = CLIENT_TX_1.init([0u8; TCP_TX_BUFFER]);
    let client_tx_2: &'static mut [u8] = CLIENT_TX_2.init([0u8; TCP_TX_BUFFER]);

    spawn_or_warn(&spawner, beacon_task(station_stack), "Beacon");
    info!(
        "tcp: listening on port {} (station interface only, three preamble workers)",
        LINK_TCP_DEFAULT
    );
    spawn_or_warn(
        &spawner,
        client_task(station_stack, client_rx_0, client_tx_0),
        "TCP client 0",
    );
    spawn_or_warn(
        &spawner,
        client_task(station_stack, client_rx_1, client_tx_1),
        "TCP client 1",
    );
    spawn_or_warn(
        &spawner,
        client_task(station_stack, client_rx_2, client_tx_2),
        "TCP client 2",
    );

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

        // The Beacon is only honest while the station is associated and holds an address.
        STATION_UP.store(
            station_connected && station_stack.config_v4().is_some(),
            Ordering::Relaxed,
        );

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
                "status: t={}s rssi={}dBm uart_rx={}B uart_tx={}B uart_kbps={} uart_drop={}B dma_stalls={} crc={} resync={} good={} bad={} tcp_rx={}B tcp_tx={}B tcp_kbps={} tcp_accepts={} tcp_tx_errors={} tcp_read_errors={} client_crc={} client_resync={} ring_used={}B ring_reserved={}B drop_stream={} drop_no_client={} drop_reliable={} drop_bridge_control={} drop_on_close={} ring_encode_errors={} tx_q_drop={} reconnects={} auth_ok={} auth_bad={} auth_expired={} auth_superseded={} auth_failed={} deny_busy={} deny_refused={} deny_version={} beacons={} beacon_errors={} link_ready={}",
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
                AUTH_ACCEPTED.load(Ordering::Relaxed),
                AUTH_REJECTED.load(Ordering::Relaxed),
                AUTH_TIMEOUTS.load(Ordering::Relaxed),
                AUTH_SUPERSEDED.load(Ordering::Relaxed),
                AUTH_FAILED.load(Ordering::Relaxed),
                DENIED_BUSY.load(Ordering::Relaxed),
                DENIED_REFUSED.load(Ordering::Relaxed),
                DENIED_VERSION.load(Ordering::Relaxed),
                BEACONS_SENT.load(Ordering::Relaxed),
                BEACON_SEND_ERRORS.load(Ordering::Relaxed),
                link_ready(),
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

/// One Link Protocol connection worker: accept, run the `bridge-net` preamble, then serve.
///
/// Several workers run at once. Only one can hold an authenticated client (the gate refuses the
/// rest with `busy`), but an unauthenticated attempt never occupies the slot another client needs:
/// a newer connection supersedes it, and a free worker is always accepting. The five-second
/// deadline, the refusal window and the supersede rule all live in `bridge-core`'s gate; this
/// function only performs the `bridge-net` exchange the gate admits.
#[allow(
    clippy::large_stack_frames,
    reason = "the task owns a socket, a parser and a 1 KiB frame buffer for its lifetime"
)]
#[embassy_executor::task(pool_size = 3)]
async fn client_task(stack: Stack<'static>, rx_buffer: &'static mut [u8], tx_buffer: &'static mut [u8]) {
    let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
    let mut client_buffer = alloc::vec![0u8; CLIENT_READ_CHUNK];
    let mut parser = PcToRobotParser::new(Direction::PcToRobot, UART_BAUD);

    loop {
        socket.set_timeout(Some(SOCKET_TIMEOUT));
        if socket.accept(LINK_TCP_DEFAULT).await.is_err() {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }
        TCP_ACCEPTS.fetch_add(1, Ordering::Relaxed);

        let connection = match with_pump(|pump| pump.connect(now_ms())) {
            ConnectOutcome::Challenge { connection } => connection,
            ConnectOutcome::Busy => {
                DENIED_BUSY.fetch_add(1, Ordering::Relaxed);
                warn!("tcp: another authenticated client holds the slot; denying busy");
                deny_and_finish(&mut socket, DenyReason::Busy, Some(BUSY_RETRY_MS)).await;
                continue;
            }
            ConnectOutcome::Refused { until_ms } => {
                DENIED_REFUSED.fetch_add(1, Ordering::Relaxed);
                warn!("tcp: the gate refuses connections until {}; denying", until_ms);
                deny_and_finish(&mut socket, DenyReason::Unauthorized, Some(REFUSAL_RETRY_MS)).await;
                continue;
            }
        };

        let Some(token) = bench_token() else {
            warn!("tcp: BRIDGE_BENCH_TOKEN is unset; denying the connection (TODO ticket 07)");
            with_pump(|pump| pump.disconnect(connection));
            deny_and_finish(&mut socket, DenyReason::Unauthorized, None).await;
            continue;
        };

        match run_preamble(&mut socket, connection, token).await {
            PreambleOutcome::Accepted => {
                AUTH_ACCEPTED.fetch_add(1, Ordering::Relaxed);
                info!("tcp: client {} authenticated", connection);
                // A previous client may have left a partial candidate in the parser; discard it so
                // its bytes cannot be glued to this client's first frame.
                parser.reset();
                serve_client(&mut socket, &mut parser, &mut client_buffer, connection).await;
            }
            PreambleOutcome::Rejected => {
                AUTH_REJECTED.fetch_add(1, Ordering::Relaxed);
                with_pump(|pump| pump.disconnect(connection));
            }
            PreambleOutcome::Refused => {
                AUTH_REJECTED.fetch_add(1, Ordering::Relaxed);
            }
            PreambleOutcome::Version => {
                DENIED_VERSION.fetch_add(1, Ordering::Relaxed);
                with_pump(|pump| pump.disconnect(connection));
            }
            PreambleOutcome::TimedOut => {
                AUTH_TIMEOUTS.fetch_add(1, Ordering::Relaxed);
                with_pump(|pump| pump.disconnect(connection));
            }
            PreambleOutcome::Superseded => {
                AUTH_SUPERSEDED.fetch_add(1, Ordering::Relaxed);
            }
            PreambleOutcome::Failed => {
                AUTH_FAILED.fetch_add(1, Ordering::Relaxed);
                with_pump(|pump| pump.disconnect(connection));
            }
        }

        finish_socket(&mut socket).await;
    }
}

/// Serves an authenticated client until its socket fails or closes.
async fn serve_client(
    socket: &mut TcpSocket<'_>,
    parser: &mut PcToRobotParser,
    client_buffer: &mut [u8],
    connection: u32,
) {
    loop {
        if !drain_to_socket(socket).await {
            info!("tcp: socket write failed; closing the client");
            break;
        }

        // Poll here as well as in the supervisory loop: ring pressure builds while a client is
        // attached, and this loop runs every `CLIENT_POLL`.
        emit_throttle(now_ms());

        match with_timeout(CLIENT_POLL, socket.read(client_buffer)).await {
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

    // A closing client leaves the ring as state, not history. Drop what it never read, and count
    // it: a reliable entry must never disappear silently.
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
}

/// The `bridge-net` preamble's outcome, as the worker reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum PreambleOutcome {
    /// `ACCEPT` was written; the Link Protocol starts.
    Accepted,
    /// The proof did not match; `DENY(unauthorized)` was written.
    Rejected,
    /// This failure reached the rate limit; `DENY(unauthorized)` was written.
    Refused,
    /// The peer's preamble major did not match; `DENY(version)` was written.
    Version,
    /// The five-second deadline expired first.
    TimedOut,
    /// A newer connection superseded this attempt.
    Superseded,
    /// The exchange ended without a verdict (closed, malformed or unwritable).
    Failed,
}

/// Runs the whole preamble under the pinned five-second deadline.
async fn run_preamble(socket: &mut TcpSocket<'_>, connection: u32, token: &[u8]) -> PreambleOutcome {
    match with_timeout(AUTH_DEADLINE, preamble_exchange(socket, connection, token)).await {
        Ok(outcome) => outcome,
        Err(_) => {
            warn!("tcp: connection {} exceeded the authentication deadline", connection);
            PreambleOutcome::TimedOut
        }
    }
}

/// The unbounded preamble exchange; [`run_preamble`] imposes the deadline.
#[allow(
    clippy::large_stack_frames,
    reason = "the HMAC-SHA256 proof state and the identity value live in this future"
)]
async fn preamble_exchange(socket: &mut TcpSocket<'_>, connection: u32, token: &[u8]) -> PreambleOutcome {
    let Ok(received) = read_message(socket).await else {
        return PreambleOutcome::Failed;
    };
    if received.major != preamble::MAJOR {
        warn!(
            "tcp: client preamble major {} is not {}",
            received.major,
            preamble::MAJOR
        );
        send_deny(socket, DenyReason::Version, None).await;
        return PreambleOutcome::Version;
    }
    let Message::ClientHello { nonce: client_nonce } = received.message else {
        warn!("tcp: the first preamble message was not CLIENT_HELLO");
        return PreambleOutcome::Failed;
    };

    let mut bridge_nonce = [0u8; NONCE_LEN];
    Rng::new().read(&mut bridge_nonce);
    let identity = identity_now();
    let bridge_proof = preamble::bridge_proof(token, &client_nonce, &bridge_nonce, &identity);
    if write_message(
        socket,
        &Message::BridgeChallenge {
            bridge_proof,
            bridge_nonce,
            identity,
        },
    )
    .await
    .is_err()
    {
        return PreambleOutcome::Failed;
    }

    let Ok(received) = read_message(socket).await else {
        return PreambleOutcome::Failed;
    };
    let Message::ClientAuth { proof } = received.message else {
        warn!("tcp: the second preamble message was not CLIENT_AUTH");
        return PreambleOutcome::Failed;
    };

    let expected = preamble::client_proof(token, &client_nonce, &bridge_nonce);
    match with_pump(|pump| {
        pump.authenticate(
            connection,
            Proof::from_bytes(proof),
            &Proof::from_bytes(expected),
            now_ms(),
        )
    }) {
        ProofOutcome::Accepted => {
            if write_message(socket, &Message::Accept).await.is_err() {
                return PreambleOutcome::Failed;
            }
            if with_timeout(FLUSH_TIMEOUT, socket.flush()).await.is_err() {
                return PreambleOutcome::Failed;
            }
            PreambleOutcome::Accepted
        }
        ProofOutcome::Rejected { failures } => {
            warn!("tcp: proof rejected ({} failures in the window)", failures);
            send_deny(socket, DenyReason::Unauthorized, None).await;
            PreambleOutcome::Rejected
        }
        ProofOutcome::Refused { until_ms, failures } => {
            warn!(
                "tcp: failed proofs reached the limit ({}); refusing until {}",
                failures, until_ms
            );
            send_deny(socket, DenyReason::Unauthorized, Some(REFUSAL_RETRY_MS)).await;
            PreambleOutcome::Refused
        }
        ProofOutcome::DeadlineExpired => PreambleOutcome::TimedOut,
        ProofOutcome::Superseded => PreambleOutcome::Superseded,
    }
}

/// Reads one complete framed preamble message without over-reading into Link traffic.
async fn read_message(socket: &mut TcpSocket<'_>) -> Result<Received, ()> {
    let mut frame = [0u8; PREAMBLE_BUFFER];
    read_exact(socket, &mut frame[..HEADER_LEN]).await?;
    // The framing offsets belong to `bridge-net`; a complete header always announces a length.
    let Some(total) = preamble::frame_len(&frame[..HEADER_LEN]).map_err(|_| ())? else {
        return Err(());
    };
    if total > PREAMBLE_BUFFER {
        return Err(());
    }
    read_exact(socket, &mut frame[HEADER_LEN..total]).await?;
    match preamble::try_decode(&frame[..total]) {
        Ok(Some(received)) => Ok(received),
        Ok(None) | Err(_) => Err(()),
    }
}

/// Reads exactly `buffer.len()` socket bytes.
async fn read_exact(socket: &mut TcpSocket<'_>, buffer: &mut [u8]) -> Result<(), ()> {
    let mut read = 0;
    while read < buffer.len() {
        match socket.read(&mut buffer[read..]).await {
            Ok(0) | Err(_) => return Err(()),
            Ok(count) => read += count,
        }
    }
    Ok(())
}

/// Encodes and writes one framed preamble message.
async fn write_message(socket: &mut TcpSocket<'_>, message: &Message) -> Result<(), ()> {
    let mut buffer = [0u8; PREAMBLE_BUFFER];
    let Ok(len) = preamble::encode_into(preamble::MAJOR, message, &mut buffer) else {
        return Err(());
    };
    if write_socket_all(socket, &buffer[..len]).await {
        Ok(())
    } else {
        Err(())
    }
}

/// Writes a `DENY` verdict and flushes it before the caller finishes the socket.
async fn send_deny(socket: &mut TcpSocket<'_>, reason: DenyReason, retry_ms: Option<u32>) {
    let message = Message::Deny { reason, retry_ms };
    if write_message(socket, &message).await.is_err() {
        return;
    }
    let _ = with_timeout(FLUSH_TIMEOUT, socket.flush()).await;
}

/// Sends a `DENY` verdict and recycles the socket for the worker's next accept.
async fn deny_and_finish(socket: &mut TcpSocket<'_>, reason: DenyReason, retry_ms: Option<u32>) {
    send_deny(socket, reason, retry_ms).await;
    finish_socket(socket).await;
}

/// Closes the connection with a FIN, then forces the socket back to `Closed` for reuse.
async fn finish_socket(socket: &mut TcpSocket<'_>) {
    let _ = with_timeout(FLUSH_TIMEOUT, socket.flush()).await;
    // A graceful FIN lets the peer observe the close (the silent-deadline case) instead of
    // waiting for its own timeout. The short settle gives the stack a poll to send it; `abort`
    // then forces `Closed` so this worker can accept again.
    socket.close();
    Timer::after(CLOSE_SETTLE).await;
    let _ = with_timeout(FLUSH_TIMEOUT, socket.flush()).await;
    socket.abort();
}

/// Broadcasts the Discovery Beacon every [`beacon::INTERVAL_MS`] while the station is up.
///
/// The flags come from the same helpers the challenge serves, so the Beacon and the preamble can
/// never describe the Bridge differently. `tcp_port` is the Link listener's port: portal-only zero
/// belongs to ticket 06.
#[allow(
    clippy::large_stack_frames,
    reason = "the task owns the UDP socket and the encoded datagram"
)]
#[embassy_executor::task]
async fn beacon_task(stack: Stack<'static>) {
    let mut socket = UdpSocket::new(
        stack,
        BEACON_RX_META.init([PacketMetadata::EMPTY; 1]),
        BEACON_RX_BUFFER.init([0u8; BEACON_SOCKET_BUFFER]),
        BEACON_TX_META.init([PacketMetadata::EMPTY; 1]),
        BEACON_TX_BUFFER.init([0u8; BEACON_SOCKET_BUFFER]),
    );
    if socket.bind(BEACON_UDP).is_err() {
        warn!("beacon: could not bind UDP port {}", BEACON_UDP);
        return;
    }
    info!(
        "beacon: broadcasting on UDP {} every {}ms",
        BEACON_UDP,
        beacon::INTERVAL_MS
    );

    let mut bytes = [0u8; beacon::ENCODED_LEN];
    loop {
        if STATION_UP.load(Ordering::Relaxed) {
            let beacon = Beacon {
                major: beacon::LINK_MAJOR,
                device_id: device_id(),
                name: bridge_name(),
                tcp_port: LINK_TCP_DEFAULT,
                flags: flags_now(),
            };
            match beacon.encode_into(&mut bytes) {
                Ok(len) => {
                    match socket
                        .send_to(&bytes[..len], (Ipv4Address::BROADCAST, BEACON_UDP))
                        .await
                    {
                        Ok(()) => {
                            BEACONS_SENT.fetch_add(1, Ordering::Relaxed);
                        }
                        Err(_) => {
                            BEACON_SEND_ERRORS.fetch_add(1, Ordering::Relaxed);
                        }
                    }
                }
                Err(_) => warn!("beacon: the datagram did not encode"),
            }
        }
        Timer::after(Duration::from_millis(beacon::INTERVAL_MS)).await;
    }
}

// ---------------------------------------------------------------------------------------------
// Identity and Beacon flags
// ---------------------------------------------------------------------------------------------

/// Returns the bench token's bytes, or `None` when no token was compiled in.
fn bench_token() -> Option<&'static [u8]> {
    if BENCH_TOKEN.is_empty() {
        None
    } else {
        Some(BENCH_TOKEN.as_bytes())
    }
}

/// Returns the efuse station MAC's device id.
fn device_id() -> DeviceId {
    with_device_id(|bytes| DeviceId::new(*bytes))
}

/// Returns the display name, `Explorer-XXXX` from the device id's last two bytes.
fn bridge_name() -> Name {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let bytes = device_id().bytes();
    let mut padded = [0u8; Name::ENCODED_LEN];
    padded[..9].copy_from_slice(b"Explorer-");
    padded[9] = HEX[usize::from(bytes[4] >> 4)];
    padded[10] = HEX[usize::from(bytes[4] & 0x0F)];
    padded[11] = HEX[usize::from(bytes[5] >> 4)];
    padded[12] = HEX[usize::from(bytes[5] & 0x0F)];
    Name::from_padded(padded)
}

/// Returns whether a valid Link Protocol frame arrived from the robot inside the pinned window.
fn link_ready() -> bool {
    let last = LAST_ROBOT_FRAME_MS.load(Ordering::Relaxed);
    last != NO_ROBOT_FRAME && now_ms().wrapping_sub(last) < beacon::LINK_READY_WINDOW_MS as u32
}

/// Returns the flags the Beacon and the challenge serve.
///
/// Provisioned stays false until ticket 07 stores a credential record; setup-mode stays false while
/// the firmware is station-only (ticket 06 adds APSTA).
fn flags_now() -> IdentityFlags {
    let mut flags = IdentityFlags::default();
    if link_ready() {
        flags |= IdentityFlags::LINK_READY;
    }
    flags
}

/// Returns the identity served in the challenge and advertised in the Beacon.
fn identity_now() -> Identity {
    Identity {
        device_id: device_id(),
        name: bridge_name(),
        flags: flags_now(),
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

    // `is_done()` is the DMA's `SuccessfulEof`, and with the idle EOF enabled it also fires at
    // every packet boundary, so it is not by itself a stall. A stall is the transfer reporting
    // done while no data arrives for a whole second: an idle-EOF boundary yields data within
    // milliseconds, whereas a genuinely exhausted descriptor ring stays silent.
    let mut done_since: Option<Instant> = None;
    let mut stall_counted = false;

    loop {
        let done = transfer.is_done();

        let total = {
            let data = transfer.peek();
            let total = data.len();
            if total > 0 {
                done_since = None;
                stall_counted = false;
                UART_RX_BYTES.fetch_add(u32::try_from(total).unwrap_or(u32::MAX), Ordering::Relaxed);

                let now = now_ms();
                let mut frames = 0u32;
                parser.feed(data, now, |frame| {
                    // A frame only reaches this callback after the parser accepted its envelope and
                    // CRC, so it is the Beacon's pinned "valid Link Protocol frame from the robot".
                    LAST_ROBOT_FRAME_MS.store(now, Ordering::Relaxed);
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
            if done {
                let since = match done_since {
                    Some(since) => since,
                    None => {
                        let now = Instant::now();
                        done_since = Some(now);
                        now
                    }
                };
                if !stall_counted && Instant::now().duration_since(since) >= DMA_STALL_AFTER {
                    UART_DMA_STALLS.fetch_add(1, Ordering::Relaxed);
                    warn!(
                        "uart dma: done with no data for {}s; the descriptor ring may be exhausted",
                        DMA_STALL_AFTER.as_secs()
                    );
                    stall_counted = true;
                }
            } else {
                done_since = None;
                stall_counted = false;
            }
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
