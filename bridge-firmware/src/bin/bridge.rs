#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! Ticket 07 (the persisted credential record and the Setup Mode lifecycle) and ticket 08 (Bridge
//! Presence and the task watchdog).
//!
//! The Bridge is the trust boundary for the robot (ADR-0037): the `bridge-net` HMAC preamble
//! completes before a single Link Protocol byte crosses, exactly one authenticated client holds the
//! slot, failed proofs are rate-limited by `bridge-core`'s gate, and the Discovery Beacon is
//! broadcast on the station interface every 3 s with the pinned flags.
//!
//! Setup Mode (ADR-0036) is a bounded SoftAP with a hand-rolled userspace portal: DHCP, a DNS
//! catch-all and one HTTP page, all over `embassy-net` on a second stack. `bridge-core` owns the
//! window rules and the protocol codecs; this binary owns the radio states, the sockets and the
//! join attempt. The window opens when the Bridge is unprovisioned at boot, after five continuous
//! minutes without a station link (once per boot), or on the panel's request; it closes after ten
//! minutes, or about 30 s after a stable station association. In APSTA the station Beacon carries
//! `setup_mode` and `tcp_port = 0`; in AP-only there is no station Beacon.
//!
//! Ticket 07 stores the station credentials and the 60-bit pairing token in one versioned,
//! CRC-checked record on the `bridge_data` partition, written with `sequential-storage` through
//! `esp-storage`. A missing or unusable record fails closed into Setup Mode; a reflash of the app
//! leaves the partition alone; the portal's submit path and the panel's `BRIDGE_CONTROL` actions
//! are the record's write points.
//!
//! ## Presence and the watchdog
//!
//! Bridge Presence is the `EXP_IRQ` line driven high once the scheduler, the radio controller and
//! both UART tasks are running; it is not the deadman (ADR-0035), and a reset drops it. On the C3
//! SuperMini harness it is **GPIO0**, the robot's `EXP_EN` is wired to the C3's `EN` through the
//! RST button's outer leg, and the robot's `EXP_BOOT` is a spare pulled-down input on **GPIO1**.
//!
//! The RTC watchdog (RWDT stage 0, reset the system) is fed only while every covered task has
//! beaten inside `HEARTBEAT_MAX_AGE_MS`: the two UART pump tasks, both `embassy-net` runners, the
//! Beacon and the three connection workers. A stalled task therefore reboots the Bridge, which is
//! benign by design: the credential record survives, the station is rejoined, the daemon
//! reconnects, and the robot's Boot Id is unaffected. `BRIDGE_STALL_TASK=<task>` arms a bench-only
//! hook that stops one covered task's beat; the default image leaves it unarmed.
//!
//! ## Wiring
//!
//! UART1 runs at ADR-0033's 921600 8N1 with **TX = GPIO4** and **RX = GPIO3**. Connect those two
//! pins, through a 3.3 V USB-TTL adapter, to the Robot Stub's serial port. The C3's USB connector
//! stays the flashing and `defmt` console. A GPIO3↔GPIO4 jumper loops the UART back for bench
//! runs without an adapter; a valid robot-side frame then sets the Beacon's link-ready flag.
//!
//! ## Provisioning and the pairing token
//!
//! There is no compile-time credential. A blank or erased data partition boots unprovisioned: the
//! Bridge mints a 60-bit token from the hardware TRNG (`Rng` alone is not cryptographic before the
//! radio is up), stores it in a token-only record, and raises the setup AP. Credentials submitted
//! through the portal are written to the same record and survive a reset, a power cycle and a
//! reflash of the bootloader, partition table and app. `espflash erase-flash` (or a partition-table
//! change that moves `bridge_data`) loses the record, which the magic, version and CRC make safe.
//!
//! The token is displayed as twelve unambiguous base32 characters in three groups of four. Those
//! twelve characters' bytes are the HMAC key, with no decoding step; the panel's `BRIDGE_CONTROL`
//! action 1 shows it and action 2 rotates it. The bridge `README.md` documents the partition table
//! and the explicit `espflash` set.
//!
//! ## The Beacon flags
//!
//! `provisioned` means a usable record with a station credential pair; `setup_mode` is false while
//! the firmware is station-only; `tcp_port` is the Link listener's port while it is up; and
//! `link-ready` is `bridge_net::beacon::FLAG_LINK_READY`'s pinned meaning — a valid Link Protocol
//! frame from the robot within `beacon::LINK_READY_WINDOW_MS`.

use core::cell::RefCell;

use bridge_core::{
    BridgeControlRequest, BridgePump, Class, ConnectOutcome, Decision, EntryKind, Flags, Frame, Proof, ProofOutcome,
    Seq, TimerEvent, dhcp, dns,
    heartbeat::{CoveredTask, HeartbeatSet},
    pairing::{self, GROUPED_LEN, write_grouped},
    portal::{self, Credentials, ScanCache, Status},
    record::{self, CredentialRecord},
    setup::{CloseReason, RaiseReason, SetupEvent, SetupWindow},
};
use bridge_net::{
    beacon::{self, Beacon},
    identity::{DeviceId, Flags as IdentityFlags, Identity, Name},
    ports::{BEACON_UDP, LINK_TCP_DEFAULT},
    preamble::{self, DenyReason, HEADER_LEN, MAX_PAYLOAD_LEN, Message, NONCE_LEN, Received},
};
use critical_section::Mutex;
use defmt::{info, warn};
use embassy_embedded_hal::adapter::BlockingAsync;
use embassy_executor::Spawner;
use embassy_net::{
    Ipv4Address, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4,
    tcp::TcpSocket,
    udp::{PacketMetadata, UdpSocket},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, channel::Channel, mutex::Mutex as AsyncMutex};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use embedded_storage::nor_flash::{
    ErrorType, MultiwriteNorFlash, NorFlash, NorFlashError, NorFlashErrorKind, ReadNorFlash,
};
use esp_backtrace as _;
use esp_bootloader_esp_idf::partitions::read_partition_table;
use esp_hal::{
    Async,
    clock::CpuClock,
    dma_rx_stream_buffer,
    efuse::{InterfaceMacAddress, interface_mac_address},
    gpio::{Input, InputConfig, Level, Output, OutputConfig, Pull},
    rng::{Rng, Trng, TrngSource},
    rtc_cntl::{Rtc, Rwdt, RwdtStage, RwdtStageAction},
    time::Duration as HalDuration,
    timer::timg::TimerGroup,
    uart::{
        Config as UartConfig, RxConfig, TxError, Uart, UartTx,
        uhci::{RxConfig as UhciRxConfig, TxConfig as UhciTxConfig, Uhci, UhciRx},
    },
};
use esp_println as _;
use esp_radio::wifi::{
    AuthenticationMethod, AuthenticationMethodConfig, Config as WifiConfig, ConnectionError, ControllerConfig,
    DisconnectReason, Interface, Ssid, WifiController, WifiError, ap::AccessPointConfig, scan::ScanConfig,
    sta::StationConfig,
};
use heapless::spsc::Queue;
use portable_atomic::{AtomicBool, AtomicU32, Ordering};
use remote_protocol::{
    BridgeControlAction, BridgeControlReply, Control, ControlMessage, Direction, MAX_PC_TO_ROBOT_FRAME,
    MAX_ROBOT_TO_PC_FRAME, PROTOCOL_MAJOR, PairingCode, PcToRobotParser, RobotToPcParser, SetupCredentials,
    ThrottleHint, encode_bridge_control_ack, encode_control,
};
use sequential_storage::{
    cache::NoCache,
    map::{Key, MapConfig, MapStorage, SerializationError, Value},
};
use static_cell::StaticCell;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ---------------------------------------------------------------------------------------------
// Compile-time inputs and bench constants
// ---------------------------------------------------------------------------------------------

/// The dedicated data partition's label in `partitions.csv` (ADR-0036).
const DATA_PARTITION_LABEL: &str = "bridge_data";

/// The partition table's read buffer. The custom table has four partitions plus the MD5 entry,
/// so 256 bytes (eight entries) covers it and leaves room for three more without a boot-time
/// stack frame that the firmware's `large_stack_frames` gate rejects.
const PARTITION_TABLE_LEN: usize = 256;

/// The `sequential-storage` scratch buffer: the one-byte key plus the longest record, rounded up
/// to the flash word size.
const RECORD_SCRATCH: usize = 128;

/// ADR-0033's shared baud: all three projects run 921600 8N1.
const UART_BAUD: u32 = 921_600;

/// The UART RX FIFO-full threshold. The driver default of 120 bytes leaves only ~87 us of
/// headroom at 921600; 32 bytes leaves about 1 ms.
const UART_RX_THRESHOLD: u16 = 32;

/// UART1's pins on the C3 SuperMini harness. Keep in step with the `with_tx`/`with_rx` calls below.
const UART_TX_PIN: u8 = 4;
const UART_RX_PIN: u8 = 3;

/// The Bridge Presence line's C3 pin.
///
/// The SuperMini harness wires this to the robot's `EXP_IRQ` (GPIO33 on J10). It is driven high
/// only once both UART tasks are up and is held for the firmware's lifetime; a reset drops it.
const PRESENCE_PIN: u8 = 0;

/// The spare robot→Bridge line: the robot's `EXP_BOOT` (GPIO35 on J10) into this C3 input.
///
/// GPIO1 is an input with a pull-down, reserved for a future hard signal; the firmware does not
/// read it yet. It is deliberately **not** the C3's BOOT strap (GPIO9), so a low idle level here
/// cannot select ROM download mode.
const SPARE_BOOT_PIN: u8 = 1;

/// The RTC watchdog's hardware timeout. It must exceed the longest legitimate block: a flash
/// record write (a `BlockingAsync` program, possibly with an erase), and the covered tasks' own
/// bounded waits. The station join (30 s) and the scan sweeps are not covered, so they do not
/// enter this budget.
const WATCHDOG_TIMEOUT_MS: u64 = 20_000;

/// How often the watchdog task checks the covered tasks' beats.
const WATCHDOG_POLL: Duration = Duration::from_secs(1);

/// The oldest silence a covered task may have. It sits above the 5 s preamble deadline, the 3 s
/// Beacon period and the 10 s socket timeout with margin for executor scheduling jitter.
const HEARTBEAT_MAX_AGE_MS: u32 = 12_000;

/// The longest a covered task may wait on one I/O operation before it has to check in. Tasks whose
/// loop already wakes faster (`uart-rx`, `uart-tx`, the client poll) use their own cadence.
const HEARTBEAT_PERIOD: Duration = Duration::from_secs(2);

/// The bench-only stall hook: a covered task selected at build time that awaits forever, so the
/// watchdog stops being fed and resets the Bridge. It is `None` in the default image.
const STALL_TASK: Option<CoveredTask> = parse_stall_task(option_env!("BRIDGE_STALL_TASK"));

/// Maps the build-time `BRIDGE_STALL_TASK` value to the covered task it stalls.
///
/// The canonical names come from [`CoveredTask::label`], so a new slot cannot publish a label the
/// hook does not understand; `pump` and `net` are the handoff's short aliases.
const fn parse_stall_task(value: Option<&str>) -> Option<CoveredTask> {
    let text = match value {
        Some(text) => text,
        None => return None,
    };
    if str_eq(text, CoveredTask::UartRx.label()) || str_eq(text, "pump") {
        return Some(CoveredTask::UartRx);
    }
    if str_eq(text, CoveredTask::UartTx.label()) {
        return Some(CoveredTask::UartTx);
    }
    if str_eq(text, CoveredTask::NetStation.label()) || str_eq(text, "net") {
        return Some(CoveredTask::NetStation);
    }
    if str_eq(text, CoveredTask::NetAccessPoint.label()) {
        return Some(CoveredTask::NetAccessPoint);
    }
    if str_eq(text, CoveredTask::Beacon.label()) {
        return Some(CoveredTask::Beacon);
    }
    if str_eq(text, CoveredTask::Client0.label()) {
        return Some(CoveredTask::Client0);
    }
    None
}

/// Compares two strings byte by byte in a constant context.
const fn str_eq(left: &str, right: &str) -> bool {
    let left = left.as_bytes();
    let right = right.as_bytes();
    if left.len() != right.len() {
        return false;
    }
    let mut index = 0;
    while index < left.len() {
        if left[index] != right[index] {
            return false;
        }
        index += 1;
    }
    true
}

/// The UHCI RX stream buffer and descriptor chunk size (ADR-0033's 4 KiB).
const DMA_RX_BUFFER: usize = 4096;
const DMA_RX_CHUNK: usize = 128;

/// How long a done-but-silent DMA has to stay silent before it is counted as a stall. The idle
/// EOF makes `is_done` fire at packet boundaries, so a transfer that is merely idle between
/// packets must not be mistaken for an exhausted descriptor ring.
const DMA_STALL_AFTER: Duration = Duration::from_secs(1);

/// How many `BRIDGE_CONTROL` requests one DMA burst may queue for the UART task. The panel sends
/// one at a time; the slack covers a frame split across chunks and a retry.
const PANEL_REQUEST_QUEUE: usize = 4;

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
// Setup Mode constants (ADR-0036)
// ---------------------------------------------------------------------------------------------

/// The setup SSID's prefix; the four hex characters of the AP MAC's last two bytes follow it.
const AP_SSID_PREFIX: &[u8] = b"ExplorerRobot-";

/// The documented, build-fixed WPA2 setup password (ADR-0036: public by construction).
const AP_PASSWORD: &str = "explorer-setup";

/// The setup AP's channel. In APSTA the radio follows the station's channel instead.
const AP_CHANNEL: u8 = 1;

/// The setup AP's client limit (ADR-0036: at most two).
const AP_MAX_CONNECTIONS: u16 = 2;

/// The setup AP's address (ADR-0036).
const AP_ADDRESS: [u8; 4] = [192, 168, 4, 1];
const AP_PREFIX_LEN: u8 = 24;

/// The portal's HTTP port on the AP stack.
const PORTAL_HTTP_PORT: u16 = 80;

/// How long the portal lets a request arrive before it closes the connection.
const PORTAL_READ_TIMEOUT: Duration = Duration::from_secs(10);

/// How many portal commands the HTTP workers may queue for the supervisory loop.
const PORTAL_COMMAND_QUEUE: usize = 4;

/// Best-of-N: `scan_async` on this beta drops whole sweeps (ticket 09), so merge several.
const SCAN_SWEEPS: usize = 3;

/// How often a fresh best-of-N scan refreshes the portal's list while no join is in flight. A
/// sweep off-channels the AP and can drop the laptop, so the first scan (at window open, before
/// the operator joins) is the useful one; the field also accepts a typed SSID.
const SCAN_REFRESH: Duration = Duration::from_secs(300);

/// The portal's TCP socket buffers (the response pages are written in chunks).
const PORTAL_TCP_RX_BUFFER: usize = 1024;
const PORTAL_TCP_TX_BUFFER: usize = 2048;

/// The DHCP and DNS datagram buffers.
const PORTAL_UDP_BUFFER: usize = 576;

/// A station config parked on a non-existent SSID while an unprovisioned window is closed. There
/// is no radio-off API on this beta; a STA-only config naming an absent network is the closest
/// equivalent to turning the AP off while keeping the controller alive.
const PARKED_STATION_SSID: &str = "bridge-parked";
const PARKED_STATION_PASSWORD: &str = "unused-pass";

// ---------------------------------------------------------------------------------------------
// Statics
// ---------------------------------------------------------------------------------------------

/// The station stack's socket pool: DHCP, the Beacon's UDP socket and the three TCP workers.
static STA_RESOURCES: StaticCell<StackResources<8>> = StaticCell::new();

/// The AP stack's socket pool: the DHCP and DNS responders and the two portal HTTP workers.
static AP_RESOURCES: StaticCell<StackResources<6>> = StaticCell::new();

/// Whether the bounded setup window is open; the Beacon and the portal tasks read it.
static SETUP_OPEN: AtomicBool = AtomicBool::new(false);

/// Whether a usable credential pair exists; the Beacon's `provisioned` bit reads it. Ticket 07
/// replaces this with the credential record.
static PROVISIONED: AtomicBool = AtomicBool::new(false);

/// The portal's current status line, shared by the supervisory loop and the HTTP workers.
static PORTAL_STATUS: Mutex<RefCell<Status>> = Mutex::new(RefCell::new(Status::Unprovisioned));

/// The best-of-N scan list, written by the supervisory loop and read by the HTTP workers.
static SCAN_CACHE: Mutex<RefCell<ScanCache>> = Mutex::new(RefCell::new(ScanCache::new()));

/// The portal actions queued for the supervisory loop, which owns the radio controller.
static PORTAL_COMMANDS: Channel<CriticalSectionRawMutex, PortalCommand, PORTAL_COMMAND_QUEUE> = Channel::new();

/// A panel action queued for the supervisory loop, which owns the radio and the RAM credentials.
/// The record write itself happens in the UART task, before the acknowledgement.
#[derive(Clone, Copy, Debug)]
enum PanelCommand {
    /// Clear the RAM credentials and raise Setup Mode; the record is already token-only.
    Forget,
}

/// The panel actions queued for the supervisory loop.
static PANEL_COMMANDS: Channel<CriticalSectionRawMutex, PanelCommand, PORTAL_COMMAND_QUEUE> = Channel::new();

/// The pairing token's twelve base32 characters, or `None` when the TRNG could not mint one.
/// The characters' bytes are the HMAC key (ADR-0037), so there is no decode step.
static PAIRING_TOKEN: Mutex<RefCell<Option<[u8; pairing::CODE_LEN]>>> = Mutex::new(RefCell::new(None));

/// The record's flash map, shared by the boot/portal/panel writers. The async mutex is held only
/// for one short flash operation, never across a station join or a scan.
static CREDENTIAL_STORE: AsyncMutex<CriticalSectionRawMutex, Option<CredentialStore>> = AsyncMutex::new(None);

/// Whether the station is associated and holds an address; the Beacon is sent only then.
static STATION_UP: AtomicBool = AtomicBool::new(false);

/// The watchdog's coverage and last-beat table. Every covered task writes its own slot from its
/// own loop; the watchdog task reads the whole set.
static HEARTBEATS: Mutex<RefCell<HeartbeatSet>> = Mutex::new(RefCell::new(HeartbeatSet::new()));

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

/// The portal's UDP receive metadata and buffers (DHCP and DNS).
static DHCP_RX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
static DHCP_TX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
static DHCP_RX_BUFFER: StaticCell<[u8; PORTAL_UDP_BUFFER]> = StaticCell::new();
static DHCP_TX_BUFFER: StaticCell<[u8; PORTAL_UDP_BUFFER]> = StaticCell::new();
static DNS_RX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
static DNS_TX_META: StaticCell<[PacketMetadata; 1]> = StaticCell::new();
static DNS_RX_BUFFER: StaticCell<[u8; PORTAL_UDP_BUFFER]> = StaticCell::new();
static DNS_TX_BUFFER: StaticCell<[u8; PORTAL_UDP_BUFFER]> = StaticCell::new();

/// Each portal HTTP worker owns its TCP buffers, request buffer and response buffer.
static PORTAL_TCP_RX_0: StaticCell<[u8; PORTAL_TCP_RX_BUFFER]> = StaticCell::new();
static PORTAL_TCP_RX_1: StaticCell<[u8; PORTAL_TCP_RX_BUFFER]> = StaticCell::new();
static PORTAL_TCP_TX_0: StaticCell<[u8; PORTAL_TCP_TX_BUFFER]> = StaticCell::new();
static PORTAL_TCP_TX_1: StaticCell<[u8; PORTAL_TCP_TX_BUFFER]> = StaticCell::new();
static PORTAL_REQUEST_0: StaticCell<[u8; portal::MAX_REQUEST_BYTES]> = StaticCell::new();
static PORTAL_REQUEST_1: StaticCell<[u8; portal::MAX_REQUEST_BYTES]> = StaticCell::new();
static PORTAL_RESPONSE_0: StaticCell<[u8; portal::MAX_RESPONSE_BYTES]> = StaticCell::new();
static PORTAL_RESPONSE_1: StaticCell<[u8; portal::MAX_RESPONSE_BYTES]> = StaticCell::new();

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
///
/// Returns whether the task was spawned. The watchdog only covers tasks that started, so a pool
/// exhaustion cannot turn into a boot loop that resets before the log line is read.
fn spawn_or_warn<T>(
    spawner: &Spawner,
    token: Result<embassy_executor::SpawnToken<T>, embassy_executor::SpawnError>,
    label: &str,
) -> bool {
    match token {
        Ok(token) => {
            spawner.spawn(token);
            true
        }
        Err(_) => {
            warn!("executor: failed to spawn the {} task", label);
            false
        }
    }
}

/// Adds one task to the watchdog's coverage set.
fn cover_task(task: CoveredTask) {
    critical_section::with(|cs| HEARTBEATS.borrow(cs).borrow_mut().cover(task));
}

/// Spawns a task and, if it started, adds it to the watchdog's coverage set.
fn spawn_covered<T>(
    spawner: &Spawner,
    token: Result<embassy_executor::SpawnToken<T>, embassy_executor::SpawnError>,
    label: &str,
    task: CoveredTask,
) -> bool {
    let started = spawn_or_warn(spawner, token, label);
    if started {
        cover_task(task);
    }
    started
}

/// Records one heartbeat for a covered task at the shared millisecond clock.
fn beat_task(task: CoveredTask) {
    let now = now_ms();
    critical_section::with(|cs| HEARTBEATS.borrow(cs).borrow_mut().beat(task, now));
}

/// Stops at the build-time stall hook when it is armed, then records this task's beat.
async fn check_in(task: CoveredTask) {
    stall_if_armed(task).await;
    beat_task(task);
}

/// Returns whether the build-time stall hook selects `task`.
const fn stall_selected(task: CoveredTask) -> bool {
    match STALL_TASK {
        Some(selected) => selected as u8 == task as u8,
        None => false,
    }
}

/// Awaits forever when the build-time stall hook selects `task`; a no-op in the default image.
///
/// The hook uses an await rather than a busy loop on purpose: the rest of the executor keeps
/// running, so the test proves the heartbeat policy withholds the feed, not merely that a starved
/// executor stops feeding it.
async fn stall_if_armed(task: CoveredTask) {
    if stall_selected(task) {
        warn!("watchdog: the {} stall hook is armed; stopping the beat", task.label());
        core::future::pending::<()>().await;
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
static DHCP_OFFERS: AtomicU32 = AtomicU32::new(0);
static DHCP_ACKS: AtomicU32 = AtomicU32::new(0);
static DNS_ANSWERS: AtomicU32 = AtomicU32::new(0);
static PORTAL_REQUESTS: AtomicU32 = AtomicU32::new(0);
static PORTAL_PROVISIONS: AtomicU32 = AtomicU32::new(0);
static PORTAL_FORGETS: AtomicU32 = AtomicU32::new(0);
static SCAN_SWEEPS_RUN: AtomicU32 = AtomicU32::new(0);
static SCAN_APS_SEEN: AtomicU32 = AtomicU32::new(0);
static WATCHDOG_FEEDS: AtomicU32 = AtomicU32::new(0);
static WATCHDOG_WITHHELD: AtomicU32 = AtomicU32::new(0);

/// An action the portal queues for the supervisory loop, which owns the radio controller.
#[derive(Clone, Copy)]
enum PortalCommand {
    /// Join the station with these credentials while the AP stays up.
    Provision(Credentials),
    /// Clear the in-flight credentials and return the portal to `unprovisioned`.
    Forget,
}

/// Runs `action` with the portal's shared status line.
fn with_portal_status<R>(action: impl FnOnce(&mut Status) -> R) -> R {
    critical_section::with(|cs| action(&mut PORTAL_STATUS.borrow(cs).borrow_mut()))
}

/// Stores the portal's status line.
fn set_portal_status(status: Status) {
    with_portal_status(|current| *current = status);
}

/// Returns the portal's status line.
fn portal_status() -> Status {
    with_portal_status(|status| *status)
}

/// Runs `action` with the best-of-N scan cache.
fn with_scan_cache<R>(action: impl FnOnce(&mut ScanCache) -> R) -> R {
    critical_section::with(|cs| action(&mut SCAN_CACHE.borrow(cs).borrow_mut()))
}

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

    let (ap_ssid_bytes, ap_ssid_len) = setup_ssid();
    let ap_ssid = Ssid::try_from(&ap_ssid_bytes[..ap_ssid_len]).unwrap_or_default();

    // -- Credential record -------------------------------------------------------------------
    // The TRNG source stays alive for the firmware's lifetime: the token is minted before any
    // association (ADR-0037), and the panel can rotate it later. The SAR ADC is unavailable while
    // it lives, which is acceptable: nothing else in the Bridge uses it.
    let _entropy_source = TrngSource::new(peripherals.RNG, peripherals.ADC1);
    let store = open_record_store(peripherals.FLASH);
    let loaded = {
        let mut guard = CREDENTIAL_STORE.lock().await;
        *guard = store;
        match guard.as_mut() {
            Some(store) => store.load().await,
            None => LoadOutcome::Missing,
        }
    };
    let (mut record, may_overwrite) = match loaded {
        LoadOutcome::Record(record) => (Some(record), true),
        LoadOutcome::Missing | LoadOutcome::Unusable => (None, true),
        // A read fault is not evidence that the record is bad: fail closed for this boot but leave
        // the flash alone, so a transient busy/IO error cannot destroy a valid pair.
        LoadOutcome::ReadFailed => (None, false),
    };
    if record.is_none() {
        match mint_token() {
            Some(token) => {
                if may_overwrite {
                    let fresh = CredentialRecord::token_only(token);
                    if persist_token_only(token, "the minted token").await {
                        info!("record: minted a pairing token into the data partition");
                    }
                    record = Some(fresh);
                } else {
                    warn!("record: the read failed; using an unpersisted token for this boot");
                    record = Some(CredentialRecord::token_only(token));
                }
            }
            None => warn!("record: the TRNG did not mint a token; every client will be denied"),
        }
    }
    if let Some(token) = record.as_ref().map(|record| record.token) {
        store_pairing_token(token);
    }

    let mut grouped_token = [0u8; GROUPED_LEN];
    let token_text = match record.as_ref() {
        Some(record) => {
            write_grouped(&record.token, &mut grouped_token);
            core::str::from_utf8(&grouped_token).unwrap_or("?")
        }
        None => "MISSING",
    };

    info!(
        "bridge firmware: ticket 07 credential record, setup portal, bridge-net preamble, Beacon every {}ms; uart {} 8N1 TX=GPIO{} RX=GPIO{}",
        beacon::INTERVAL_MS,
        UART_BAUD,
        UART_TX_PIN,
        UART_RX_PIN
    );
    info!(
        "bridge identity: device_id={:02x}{:02x}{:02x}{:02x}{:02x}{:02x} name={} pairing_code={}",
        device_id[0],
        device_id[1],
        device_id[2],
        device_id[3],
        device_id[4],
        device_id[5],
        bridge_name().as_str(),
        token_text
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

    let Some(ap_config) = build_access_point_config(ap_ssid) else {
        warn!("ap: the setup AP configuration was rejected");
        loop {
            Timer::after(Duration::from_secs(60)).await;
        }
    };

    let mut credentials = record.and_then(|record| record.credentials);
    let mut station_config = credentials.as_ref().and_then(station_config_from);
    let provisioned = station_config.is_some();
    if credentials.is_some() && station_config.is_none() {
        warn!("sta: the stored credentials do not form a usable station config");
        credentials = None;
    }
    PROVISIONED.store(provisioned, Ordering::Relaxed);

    let (mut setup, boot_event) = SetupWindow::boot(provisioned, now_ms());
    SETUP_OPEN.store(setup.is_open(), Ordering::Relaxed);
    if let Some(event) = boot_event {
        log_setup_event(event);
    }

    let mut radio_mode = if provisioned {
        if let Some(station) = &station_config {
            apply_station(&mut controller, station);
        }
        RadioMode::Station
    } else {
        // Bench finding: `scan_async` returns `Other` in AP-only mode on this beta, so an
        // unprovisioned window runs APSTA with a parked station that is never associated. The AP
        // and the portal are the only served surfaces; no station Beacon is sent (ADR-0037).
        apply_setup_parked(&mut controller, &ap_config);
        RadioMode::ApStation
    };
    info!(
        "ap: setup ssid={} password={} channel={} max_clients={} address={}.{}.{}.{}/{}",
        ap_ssid.as_str(),
        AP_PASSWORD,
        AP_CHANNEL,
        AP_MAX_CONNECTIONS,
        AP_ADDRESS[0],
        AP_ADDRESS[1],
        AP_ADDRESS[2],
        AP_ADDRESS[3],
        AP_PREFIX_LEN
    );

    // -- Network stacks ----------------------------------------------------------------------

    let station_interface = Interface::station();
    let (station_stack, station_runner) = embassy_net::new(
        station_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        STA_RESOURCES.init(StackResources::new()),
        0x1234_5678_9abc_def0,
    );

    let ap_interface = Interface::access_point();
    let (ap_stack, ap_runner) = embassy_net::new(
        ap_interface,
        embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(
                Ipv4Address::new(AP_ADDRESS[0], AP_ADDRESS[1], AP_ADDRESS[2], AP_ADDRESS[3]),
                AP_PREFIX_LEN,
            ),
            gateway: None,
            dns_servers: heapless::Vec::new(),
        }),
        AP_RESOURCES.init(StackResources::new()),
        0x0fed_cba9_8765_4321,
    );

    // The liveness window starts here, immediately before the first covered task spawns, so a
    // slow radio init cannot make the tasks look stale before they have had a chance to run.
    critical_section::with(|cs| HEARTBEATS.borrow(cs).borrow_mut().seed(now_ms()));

    spawn_covered(
        &spawner,
        net_task(station_runner, CoveredTask::NetStation),
        "station network",
        CoveredTask::NetStation,
    );
    spawn_covered(
        &spawner,
        net_task(ap_runner, CoveredTask::NetAccessPoint),
        "AP network",
        CoveredTask::NetAccessPoint,
    );

    // -- UART --------------------------------------------------------------------------------

    let uart_config = UartConfig::default()
        .with_baudrate(UART_BAUD)
        .with_rx(RxConfig::default().with_fifo_full_threshold(UART_RX_THRESHOLD));

    let uart = match Uart::new(peripherals.UART1, uart_config) {
        Ok(uart) => uart.with_rx(peripherals.GPIO3).with_tx(peripherals.GPIO4),
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

    let uart_rx_started = spawn_covered(&spawner, uart_rx_dma_task(uhci_rx), "UART RX DMA", CoveredTask::UartRx);
    let uart_tx_started = spawn_covered(&spawner, uart_tx_task(uhci_tx.uart_tx), "UART TX", CoveredTask::UartTx);

    // -- Bridge Presence ---------------------------------------------------------------------
    // Drive `EXP_IRQ` high only now that the scheduler, the radio controller and both UART tasks
    // are up. The driver is kept for the firmware's lifetime; nothing clears the line on purpose,
    // so only a reset (including the watchdog's) drops it. The robot samples the level after the
    // expansion branch is powered, and this line is not the deadman (ADR-0035).
    let uart_ready = uart_rx_started && uart_tx_started;
    // Hold the driver for the firmware's lifetime; nothing clears the line on purpose.
    let _presence = if uart_ready {
        info!("presence: EXP_IRQ high on GPIO{}", PRESENCE_PIN);
        Some(Output::new(peripherals.GPIO0, Level::High, OutputConfig::default()))
    } else {
        warn!("presence: the UART tasks did not start; EXP_IRQ is left untouched");
        None
    };

    // The robot's `EXP_BOOT` is a spare robot→Bridge line, not this C3's BOOT strap (GPIO9). The
    // weak pull-down keeps it defined while the robot's output is high-Z; the firmware does not
    // read it yet.
    let _spare_boot = Input::new(peripherals.GPIO1, InputConfig::default().with_pull(Pull::Down));
    info!(
        "spare: EXP_BOOT input on GPIO{} (pull-down; reserved for a future robot→Bridge signal)",
        SPARE_BOOT_PIN
    );

    // -- Station network edge ----------------------------------------------------------------

    if pairing_token().is_none() {
        warn!("tcp: no pairing token; every preamble will be denied");
    }

    let client_rx_0: &'static mut [u8] = CLIENT_RX_0.init([0u8; TCP_RX_BUFFER]);
    let client_rx_1: &'static mut [u8] = CLIENT_RX_1.init([0u8; TCP_RX_BUFFER]);
    let client_rx_2: &'static mut [u8] = CLIENT_RX_2.init([0u8; TCP_RX_BUFFER]);
    let client_tx_0: &'static mut [u8] = CLIENT_TX_0.init([0u8; TCP_TX_BUFFER]);
    let client_tx_1: &'static mut [u8] = CLIENT_TX_1.init([0u8; TCP_TX_BUFFER]);
    let client_tx_2: &'static mut [u8] = CLIENT_TX_2.init([0u8; TCP_TX_BUFFER]);

    spawn_covered(&spawner, beacon_task(station_stack), "Beacon", CoveredTask::Beacon);
    info!(
        "tcp: listening on port {} (station interface only, three preamble workers)",
        LINK_TCP_DEFAULT
    );
    spawn_covered(
        &spawner,
        client_task(station_stack, client_rx_0, client_tx_0, CoveredTask::Client0),
        "TCP client 0",
        CoveredTask::Client0,
    );
    spawn_covered(
        &spawner,
        client_task(station_stack, client_rx_1, client_tx_1, CoveredTask::Client1),
        "TCP client 1",
        CoveredTask::Client1,
    );
    spawn_covered(
        &spawner,
        client_task(station_stack, client_rx_2, client_tx_2, CoveredTask::Client2),
        "TCP client 2",
        CoveredTask::Client2,
    );

    // -- Setup AP portal ---------------------------------------------------------------------

    let portal_tcp_rx_0: &'static mut [u8] = PORTAL_TCP_RX_0.init([0u8; PORTAL_TCP_RX_BUFFER]);
    let portal_tcp_rx_1: &'static mut [u8] = PORTAL_TCP_RX_1.init([0u8; PORTAL_TCP_RX_BUFFER]);
    let portal_tcp_tx_0: &'static mut [u8] = PORTAL_TCP_TX_0.init([0u8; PORTAL_TCP_TX_BUFFER]);
    let portal_tcp_tx_1: &'static mut [u8] = PORTAL_TCP_TX_1.init([0u8; PORTAL_TCP_TX_BUFFER]);
    let portal_request_0: &'static mut [u8] = PORTAL_REQUEST_0.init([0u8; portal::MAX_REQUEST_BYTES]);
    let portal_request_1: &'static mut [u8] = PORTAL_REQUEST_1.init([0u8; portal::MAX_REQUEST_BYTES]);
    let portal_response_0: &'static mut [u8] = PORTAL_RESPONSE_0.init([0u8; portal::MAX_RESPONSE_BYTES]);
    let portal_response_1: &'static mut [u8] = PORTAL_RESPONSE_1.init([0u8; portal::MAX_RESPONSE_BYTES]);

    spawn_or_warn(&spawner, ap_dhcp_task(ap_stack), "AP DHCP");
    spawn_or_warn(&spawner, ap_dns_task(ap_stack), "AP DNS");
    spawn_or_warn(
        &spawner,
        ap_http_task(
            ap_stack,
            portal_tcp_rx_0,
            portal_tcp_tx_0,
            portal_request_0,
            portal_response_0,
        ),
        "portal HTTP 0",
    );
    spawn_or_warn(
        &spawner,
        ap_http_task(
            ap_stack,
            portal_tcp_rx_1,
            portal_tcp_tx_1,
            portal_request_1,
            portal_response_1,
        ),
        "portal HTTP 1",
    );

    // -- Watchdog ----------------------------------------------------------------------------
    // The RTC watchdog covers the pump and network tasks that were spawned above. The portal
    // workers are deliberately not covered: they block on captive-portal traffic by design, and a
    // reset during the bounded setup window would throw away the operator's session. See the
    // ticket for the full scope decision.
    let rwdt = Rtc::new(peripherals.RTC_TIMER).rwdt;
    if !spawn_or_warn(&spawner, watchdog_task(rwdt), "watchdog") {
        warn!("watchdog: the supervisor did not start; the RTC watchdog is not running");
    }

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
    let mut next_scan_at = started;
    // A failed join inside an open window waits for the operator's next submission or Forget
    // instead of retrying on the steady-state backoff: every attempt off-channels the AP and
    // makes the portal unreachable, and a known-bad pair cannot succeed by repetition.
    let mut join_failed = false;

    loop {
        Timer::after(LOOP_PERIOD).await;
        let now = Instant::now();
        let now_ms = now_ms();

        // -- Portal commands -----------------------------------------------------------------
        // The HTTP workers only queue; the radio controller lives here so a join cannot race a
        // mode change.
        while let Ok(command) = PORTAL_COMMANDS.try_receive() {
            match command {
                PortalCommand::Provision(new_credentials) => {
                    let Some(new_station) = station_config_from(&new_credentials) else {
                        warn!("portal: the submitted credentials are not a usable station config");
                        set_portal_status(Status::Failed);
                        continue;
                    };
                    // The record is the source of truth: persist before joining, and fail closed
                    // when it cannot be written rather than reporting a provisioning that a reset
                    // would lose. The token is read inside the store lock, so a concurrent
                    // rotation cannot be overwritten.
                    if !persist_credentials(new_credentials).await {
                        warn!("portal: the credential record could not be written");
                        set_portal_status(Status::Failed);
                        continue;
                    }
                    info!(
                        "portal: joining ssid={} while the AP stays up",
                        core::str::from_utf8(new_credentials.ssid()).unwrap_or("?")
                    );
                    credentials = Some(new_credentials);
                    station_config = Some(new_station.clone());
                    PROVISIONED.store(true, Ordering::Relaxed);
                    if let Some(event) = setup.set_provisioned(true, now_ms) {
                        log_setup_event(event);
                    }
                    SETUP_OPEN.store(setup.is_open(), Ordering::Relaxed);
                    // A join must start from a clean link: the old association (if any) is not the
                    // config being submitted.
                    if controller.is_connected() {
                        let _ = with_timeout(Duration::from_secs(5), controller.disconnect_async()).await;
                    }
                    // APSTA keeps the portal reachable while the station associates. If the mode
                    // switch fails the AP-only portal remains the operator's surface.
                    if apply_ap_station(&mut controller, &new_station, &ap_config) {
                        radio_mode = RadioMode::ApStation;
                    } else {
                        warn!("portal: could not enter APSTA; leaving the radio in {:?}", radio_mode);
                    }
                    station_connected = false;
                    station_associated_at = Instant::now();
                    next_station_attempt = Instant::now();
                    backoff = MIN_BACKOFF;
                    join_failed = false;
                    logged_ip = None;
                    set_portal_status(Status::Connecting);
                    PORTAL_PROVISIONS.fetch_add(1, Ordering::Relaxed);
                }
                PortalCommand::Forget => {
                    // The portal's Forget keeps the pairing code (ADR-0037 assigns the token clear
                    // to the panel's action 0); it clears only the network.
                    match pairing_token() {
                        Some(token) => {
                            if persist_token_only(token, "the cleared credentials").await {
                                info!("portal: the cleared record was stored");
                            }
                        }
                        None => warn!("portal: no pairing token to keep in the cleared record"),
                    }
                    credentials = None;
                    station_config = None;
                    PROVISIONED.store(false, Ordering::Relaxed);
                    let station_up = station_connected && station_stack.config_v4().is_some();
                    let _ = setup.forget(now_ms, station_up);
                    SETUP_OPEN.store(true, Ordering::Relaxed);
                    // Leave the AP untouched: reapplying the mode would bounce the portal for the
                    // laptop. Disconnecting the station is enough, and the parked/real station
                    // config in the driver is unused while `credentials` is None.
                    if controller.is_connected() {
                        let _ = with_timeout(Duration::from_secs(5), controller.disconnect_async()).await;
                    }
                    station_connected = false;
                    backoff = MIN_BACKOFF;
                    next_station_attempt = Instant::now();
                    join_failed = false;
                    set_portal_status(Status::Unprovisioned);
                    PORTAL_FORGETS.fetch_add(1, Ordering::Relaxed);
                    info!("portal: forgot the network; the setup AP stays up");
                }
            }
        }

        // -- Panel commands ------------------------------------------------------------------
        // The UART task already acknowledged the request and persisted the token-only record; this
        // loop clears the RAM credentials and raises the window, because it owns the radio. The
        // panel's two-second control timeout never waits on a join attempt.
        while let Ok(PanelCommand::Forget) = PANEL_COMMANDS.try_receive() {
            credentials = None;
            station_config = None;
            PROVISIONED.store(false, Ordering::Relaxed);
            if controller.is_connected() {
                let _ = with_timeout(Duration::from_secs(5), controller.disconnect_async()).await;
            }
            station_connected = false;
            backoff = MIN_BACKOFF;
            next_station_attempt = Instant::now();
            join_failed = false;
            if let Some(event) = setup.forget(now_ms, false) {
                apply_setup_parked(&mut controller, &ap_config);
                radio_mode = RadioMode::ApStation;
                next_scan_at = Instant::now();
                log_setup_event(event);
            }
            SETUP_OPEN.store(true, Ordering::Relaxed);
            set_portal_status(Status::Unprovisioned);
            info!("panel: forgot the network and raised Setup Mode");
        }

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

        // -- Setup window ---------------------------------------------------------------------

        let station_up = station_connected && station_stack.config_v4().is_some();
        STATION_UP.store(station_up, Ordering::Relaxed);
        if let Some(event) = setup.tick(now_ms, station_up) {
            match event {
                SetupEvent::Opened(_) => {
                    SETUP_OPEN.store(true, Ordering::Relaxed);
                    if let Some(station) = &station_config {
                        if apply_ap_station(&mut controller, station, &ap_config) {
                            radio_mode = RadioMode::ApStation;
                        }
                    } else {
                        apply_setup_parked(&mut controller, &ap_config);
                        radio_mode = RadioMode::ApStation;
                    }
                    next_scan_at = Instant::now();
                    join_failed = false;
                    log_setup_event(event);
                }
                SetupEvent::Closed(_) => {
                    SETUP_OPEN.store(false, Ordering::Relaxed);
                    if let Some(station) = &station_config {
                        // Do not force a reconnect: dropping the AP mode may leave the station
                        // association intact, and the normal `is_connected` check re-associates
                        // only if it actually drops.
                        apply_station(&mut controller, station);
                    } else {
                        apply_parked_station(&mut controller);
                        station_connected = false;
                    }
                    radio_mode = RadioMode::Station;
                    log_setup_event(event);
                }
            }
        }

        // -- Station association and reconnect ----------------------------------------------

        if station_config.is_some() {
            if station_connected {
                if !controller.is_connected() {
                    info!("sta: link lost; reconnecting");
                    STATION_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                    station_connected = false;
                    backoff = MIN_BACKOFF;
                    next_station_attempt = now + backoff;
                    backoff = next_backoff(backoff);
                    if matches!(portal_status(), Status::Connected(_)) {
                        set_portal_status(Status::Connecting);
                    }
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
                    if setup.is_open() {
                        join_failed = true;
                        set_portal_status(Status::Failed);
                    }
                }
            }

            // Dropping the AP mode can leave an existing association in place; adopt it instead
            // of scanning and re-associating the network the station is already on.
            if !station_connected && controller.is_connected() {
                station_connected = true;
                station_associated_at = Instant::now();
            }

            if !station_connected && now >= next_station_attempt && !(setup.is_open() && join_failed) {
                let target = credentials.as_ref().map_or(&[][..], |current| current.ssid());
                scan_station_target(&mut controller, target).await;
                match connect_station(&mut controller).await {
                    JoinOutcome::Connected => {
                        info!("sta: associated; DHCP runs in the background");
                        station_connected = true;
                        station_associated_at = Instant::now();
                        backoff = MIN_BACKOFF;
                        join_failed = false;
                        logged_ip = None;
                        if setup.is_open() {
                            set_portal_status(Status::Connecting);
                        }
                    }
                    JoinOutcome::WrongPassword => {
                        STATION_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                        join_failed = true;
                        set_portal_status(Status::WrongPassword);
                        warn!(
                            "sta: association rejected; check the passphrase; retry in {}s",
                            backoff.as_secs()
                        );
                        next_station_attempt = Instant::now() + backoff;
                        backoff = next_backoff(backoff);
                    }
                    JoinOutcome::Failed => {
                        STATION_RECONNECTS.fetch_add(1, Ordering::Relaxed);
                        join_failed = true;
                        set_portal_status(Status::Failed);
                        warn!("sta: reconnect in {}s", backoff.as_secs());
                        next_station_attempt = Instant::now() + backoff;
                        backoff = next_backoff(backoff);
                    }
                }
            }
        } else if now >= next_missing_warning {
            warn!("sta: no credentials; the setup portal is the only network surface");
            next_missing_warning = now + Duration::from_secs(60);
        }

        // The portal's status line follows the station, without a rejoin.
        let station_up = station_connected && station_stack.config_v4().is_some();
        STATION_UP.store(station_up, Ordering::Relaxed);
        if station_up && let Some(config) = station_stack.config_v4() {
            set_portal_status(Status::Connected(config.address.address().octets()));
        }

        // -- Portal scan list -----------------------------------------------------------------
        // Skip while a join is in flight: a sweep on the one radio can disturb both the AP and
        // the association attempt.
        if setup.is_open() && !station_up && now >= next_scan_at && portal_status() != Status::Connecting {
            run_scan_sweeps(&mut controller, &ap_ssid_bytes[..ap_ssid_len]).await;
            next_scan_at = Instant::now() + SCAN_REFRESH;
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
                "status: t={}s rssi={}dBm uart_rx={}B uart_tx={}B uart_kbps={} uart_drop={}B dma_stalls={} crc={} resync={} good={} bad={} tcp_rx={}B tcp_tx={}B tcp_kbps={} tcp_accepts={} tcp_tx_errors={} tcp_read_errors={} client_crc={} client_resync={} ring_used={}B ring_reserved={}B drop_stream={} drop_no_client={} drop_reliable={} drop_bridge_control={} drop_on_close={} ring_encode_errors={} tx_q_drop={} reconnects={} auth_ok={} auth_bad={} auth_expired={} auth_superseded={} auth_failed={} deny_busy={} deny_refused={} deny_version={} beacons={} beacon_errors={} link_ready={} setup_open={} provisioned={} dhcp_offers={} dhcp_acks={} dns={} portal_reqs={} provisions={} forgets={} scan_sweeps={} scan_aps={} wdt_feeds={} wdt_withheld={}",
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
                setup.is_open(),
                PROVISIONED.load(Ordering::Relaxed),
                DHCP_OFFERS.load(Ordering::Relaxed),
                DHCP_ACKS.load(Ordering::Relaxed),
                DNS_ANSWERS.load(Ordering::Relaxed),
                PORTAL_REQUESTS.load(Ordering::Relaxed),
                PORTAL_PROVISIONS.load(Ordering::Relaxed),
                PORTAL_FORGETS.load(Ordering::Relaxed),
                SCAN_SWEEPS_RUN.load(Ordering::Relaxed),
                SCAN_APS_SEEN.load(Ordering::Relaxed),
                WATCHDOG_FEEDS.load(Ordering::Relaxed),
                WATCHDOG_WITHHELD.load(Ordering::Relaxed),
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

/// The radio's current mode, so a setup transition only reconfigures when it changes something.
#[derive(Clone, Copy, Debug, PartialEq, Eq, defmt::Format)]
enum RadioMode {
    /// Steady state: the station only, no setup AP.
    Station,
    /// Setup Mode: the portal plus the station (a parked one when unprovisioned).
    ApStation,
}

/// The result of one association attempt, as the portal's status line reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum JoinOutcome {
    /// Association succeeded; DHCP may still be pending.
    Connected,
    /// The network rejected the credential pair (a wrong WPA2 passphrase).
    WrongPassword,
    /// Any other failure (timeout, missing network, radio error).
    Failed,
}

/// Builds a station configuration from a credential pair, or `None` when the radio rejects it.
#[allow(
    clippy::large_stack_frames,
    reason = "the station configuration is a boot-time value that lives in the radio driver"
)]
fn station_config_from(credentials: &Credentials) -> Option<StationConfig> {
    let ssid = Ssid::try_from(credentials.ssid()).ok()?;
    let password = credentials.passphrase().try_into().ok()?;
    Some(
        StationConfig::default()
            .with_ssid(ssid)
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(password)),
    )
}

/// Builds the setup AP's configuration: `ExplorerRobot-XXXX`, WPA2-PSK, at most two clients.
#[allow(
    clippy::large_stack_frames,
    reason = "the AP configuration is a boot-time value that lives in the radio driver"
)]
fn build_access_point_config(ssid: Ssid) -> Option<AccessPointConfig> {
    let password = AP_PASSWORD.try_into().ok()?;
    Some(
        AccessPointConfig::default()
            .with_ssid(ssid)
            .with_channel(AP_CHANNEL)
            .with_authentication(AuthenticationMethodConfig::Wpa2Personal(password))
            .with_max_connections(AP_MAX_CONNECTIONS),
    )
}

/// Builds the setup SSID from the AP interface MAC's last two bytes, uppercase hex (ADR-0036).
fn setup_ssid() -> ([u8; portal::MAX_SSID_BYTES], usize) {
    const HEX: &[u8; 16] = b"0123456789ABCDEF";
    let mac = interface_mac_address(InterfaceMacAddress::AccessPoint);
    let bytes = mac.as_bytes();
    let mut ssid = [0u8; portal::MAX_SSID_BYTES];
    ssid[..AP_SSID_PREFIX.len()].copy_from_slice(AP_SSID_PREFIX);
    let mut index = AP_SSID_PREFIX.len();
    for byte in [bytes[4], bytes[5]] {
        ssid[index] = HEX[usize::from(byte >> 4)];
        ssid[index + 1] = HEX[usize::from(byte & 0x0F)];
        index += 2;
    }
    (ssid, index)
}

/// Applies station-only mode.
fn apply_station(controller: &mut WifiController<'_>, station: &StationConfig) -> bool {
    match controller.set_config(&WifiConfig::Station(station.clone())) {
        Ok(()) => {
            info!("wifi: station mode");
            true
        }
        Err(error) => {
            warn!("wifi: station mode rejected: {:?}", error);
            false
        }
    }
}

/// Builds the parked station config: a real SSID/password pair for an absent network. The radio
/// is never asked to connect to it; it exists so the controller stays in APSTA.
fn parked_station_config() -> Option<StationConfig> {
    Credentials::from_slices(PARKED_STATION_SSID.as_bytes(), PARKED_STATION_PASSWORD.as_bytes())
        .and_then(|credentials| station_config_from(&credentials))
}

/// Applies APSTA with the parked station. This is the unprovisioned setup state on this beta:
/// APSTA is what makes `scan_async` work, and the station is never associated, so the portal is
/// the only served surface.
fn apply_setup_parked(controller: &mut WifiController<'_>, access_point: &AccessPointConfig) -> bool {
    let Some(station) = parked_station_config() else {
        warn!("wifi: the parked station configuration was rejected");
        return false;
    };
    apply_ap_station(controller, &station, access_point)
}

/// Applies APSTA: the setup AP plus the station, so the portal stays up during a join.
fn apply_ap_station(
    controller: &mut WifiController<'_>,
    station: &StationConfig,
    access_point: &AccessPointConfig,
) -> bool {
    match controller.set_config(&WifiConfig::AccessPointStation(station.clone(), access_point.clone())) {
        Ok(()) => {
            info!("wifi: setup AP mode (APSTA)");
            true
        }
        Err(error) => {
            warn!("wifi: APSTA mode rejected: {:?}", error);
            false
        }
    }
}

/// Parks the radio in STA-only mode on a network that does not exist. There is no radio-off API
/// on this beta, so this is how a closed, still-unprovisioned window stops serving the AP while
/// the controller stays alive and the next window can re-open it.
fn apply_parked_station(controller: &mut WifiController<'_>) {
    let Some(station) = parked_station_config() else {
        warn!("wifi: the parked station configuration was rejected");
        return;
    };
    apply_station(controller, &station);
}

/// Logs one setup-window transition.
fn log_setup_event(event: SetupEvent) {
    match event {
        SetupEvent::Opened(reason) => info!("setup: window opened ({})", raise_reason(reason)),
        SetupEvent::Closed(reason) => info!("setup: window closed ({})", close_reason(reason)),
    }
}

/// The short textual label for a window's closing reason.
fn close_reason(reason: CloseReason) -> &'static str {
    match reason {
        CloseReason::Expired => "expired",
        CloseReason::StationStable => "station stable",
    }
}

/// The short textual label for a window's opening reason.
fn raise_reason(reason: RaiseReason) -> &'static str {
    match reason {
        RaiseReason::Unprovisioned => "unprovisioned",
        RaiseReason::StationOutage => "station outage",
        RaiseReason::PanelRequest => "panel request",
        RaiseReason::Forgotten => "forgotten",
    }
}

// ---------------------------------------------------------------------------------------------
// Credential record storage (ADR-0036)
// ---------------------------------------------------------------------------------------------

/// The one record's key on the data partition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
enum RecordKey {
    /// The sole record: station credentials plus the pairing token.
    Credential,
}

impl Key for RecordKey {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.is_empty() {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[0] = 0;
        Ok(1)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        match buffer.first() {
            Some(0) => Ok((Self::Credential, 1)),
            Some(_) => Err(SerializationError::InvalidFormat),
            None => Err(SerializationError::BufferTooSmall),
        }
    }
}

/// The raw bytes `sequential-storage` keeps for the record.
///
/// The record's own codec decides validity; this wrapper only carries the bytes so the boot log
/// can name the exact rejection (`wrong magic`, `bad crc`, …) instead of `InvalidFormat`.
struct StoredRecord {
    /// The encoded record, zero-padded after [`StoredRecord::len`].
    bytes: [u8; record::MAX_ENCODED_LEN],
    /// The number of valid bytes.
    len: usize,
}

impl StoredRecord {
    /// Wraps an encoded record.
    const fn new(bytes: [u8; record::MAX_ENCODED_LEN], len: usize) -> Self {
        Self { bytes, len }
    }
}

impl Value<'_> for StoredRecord {
    fn serialize_into(&self, buffer: &mut [u8]) -> Result<usize, SerializationError> {
        if buffer.len() < self.len {
            return Err(SerializationError::BufferTooSmall);
        }
        buffer[..self.len].copy_from_slice(&self.bytes[..self.len]);
        Ok(self.len)
    }

    fn deserialize_from(buffer: &[u8]) -> Result<(Self, usize), SerializationError> {
        if buffer.len() > record::MAX_ENCODED_LEN {
            return Err(SerializationError::InvalidFormat);
        }
        let mut bytes = [0u8; record::MAX_ENCODED_LEN];
        bytes[..buffer.len()].copy_from_slice(buffer);
        Ok((Self::new(bytes, buffer.len()), buffer.len()))
    }
}

/// The data partition's view of the flash, as the sync `NorFlash` traits.
///
/// `esp-storage` is blocking and `sequential-storage` is async; `BlockingAsync` bridges the two.
/// The adapter owns the flash so `MapStorage` can own the adapter without a self-reference.
struct PartitionFlash {
    /// The chip-wide flash handle.
    flash: esp_storage::FlashStorage<'static>,
    /// The partition's offset from the flash base.
    base: u32,
    /// The partition's length in bytes.
    len: u32,
}

impl PartitionFlash {
    /// Creates a view of `base..base + len`.
    const fn new(flash: esp_storage::FlashStorage<'static>, base: u32, len: u32) -> Self {
        Self { flash, base, len }
    }

    /// Returns whether `offset..offset + len` lies inside the partition.
    fn contains(&self, offset: u32, len: usize) -> bool {
        let Ok(width) = u32::try_from(len) else {
            return false;
        };
        offset.checked_add(width).is_some_and(|end| end <= self.len)
    }
}

/// The partition adapter's error.
#[derive(Debug)]
enum PartitionFlashError {
    /// The underlying flash rejected the access.
    Io,
    /// The access left the partition.
    OutOfBounds,
}

impl NorFlashError for PartitionFlashError {
    fn kind(&self) -> NorFlashErrorKind {
        match self {
            Self::Io => NorFlashErrorKind::Other,
            Self::OutOfBounds => NorFlashErrorKind::OutOfBounds,
        }
    }
}

impl ErrorType for PartitionFlash {
    type Error = PartitionFlashError;
}

impl ReadNorFlash for PartitionFlash {
    const READ_SIZE: usize = 4;

    fn read(&mut self, offset: u32, bytes: &mut [u8]) -> Result<(), Self::Error> {
        if !self.contains(offset, bytes.len()) {
            return Err(PartitionFlashError::OutOfBounds);
        }
        self.flash
            .read_nor(self.base + offset, bytes)
            .map_err(|_| PartitionFlashError::Io)
    }

    fn capacity(&self) -> usize {
        usize::try_from(self.len).unwrap_or(usize::MAX)
    }
}

impl NorFlash for PartitionFlash {
    const WRITE_SIZE: usize = 4;
    const ERASE_SIZE: usize = esp_storage::FlashStorage::SECTOR_SIZE as usize;

    fn write(&mut self, offset: u32, bytes: &[u8]) -> Result<(), Self::Error> {
        if !self.contains(offset, bytes.len()) {
            return Err(PartitionFlashError::OutOfBounds);
        }
        self.flash
            .write_nor(self.base + offset, bytes)
            .map_err(|_| PartitionFlashError::Io)
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), Self::Error> {
        let Ok(width) = usize::try_from(to.saturating_sub(from)) else {
            return Err(PartitionFlashError::OutOfBounds);
        };
        if from > to || !self.contains(from, width) {
            return Err(PartitionFlashError::OutOfBounds);
        }
        self.flash
            .erase(self.base + from, self.base + to)
            .map_err(|_| PartitionFlashError::Io)
    }
}

impl MultiwriteNorFlash for PartitionFlash {}

/// The typed `sequential-storage` map over the data partition.
type RecordStorage = MapStorage<RecordKey, BlockingAsync<PartitionFlash>, NoCache>;

/// What a boot read found on the data partition.
#[derive(Clone, Copy, Debug)]
enum LoadOutcome {
    /// A valid record.
    Record(CredentialRecord),
    /// No record key on the partition.
    Missing,
    /// The item exists but is not a valid record; the log names the reason.
    Unusable,
    /// The flash read failed. The record may still be valid, so the boot must not overwrite it.
    ReadFailed,
}

/// The persisted credential record and its flash map.
struct CredentialStore {
    /// The map over the `bridge_data` partition.
    storage: RecordStorage,
}

impl CredentialStore {
    /// Reads the record for the boot decision.
    ///
    /// Every record failure is deliberate: the caller mints a fresh token and fails closed into
    /// Setup Mode. A *read* failure is different: it is not evidence that the record is bad, so the
    /// caller must not overwrite it.
    #[allow(
        clippy::large_stack_frames,
        reason = "the 128-byte scratch buffer and the decoded record live in this future"
    )]
    async fn load(&mut self) -> LoadOutcome {
        let mut scratch = [0u8; RECORD_SCRATCH];
        match self
            .storage
            .fetch_item::<StoredRecord>(&mut scratch, &RecordKey::Credential)
            .await
        {
            Ok(Some(stored)) => match CredentialRecord::decode(&stored.bytes[..stored.len]) {
                Ok(record) => {
                    info!(
                        "record: loaded (provisioned={} pairing_code={})",
                        record.provisioned(),
                        record.token_str()
                    );
                    LoadOutcome::Record(record)
                }
                Err(error) => {
                    warn!(
                        "record: unusable ({}); treating the Bridge as unprovisioned",
                        error.label()
                    );
                    LoadOutcome::Unusable
                }
            },
            Ok(None) => {
                info!("record: none on the data partition");
                LoadOutcome::Missing
            }
            Err(error) => {
                warn!(
                    "record: read failed; treating the Bridge as unprovisioned: {}",
                    defmt::Debug2Format(&error)
                );
                LoadOutcome::ReadFailed
            }
        }
    }

    /// Reads the record silently, for an update already holding the store lock.
    ///
    /// Returns `Err(())` on a flash read fault so an update can abort instead of overwriting a
    /// record it could not read. A missing or corrupt record is `Ok(None)`.
    #[allow(
        clippy::large_stack_frames,
        reason = "the 128-byte scratch buffer and the decoded record live in this future"
    )]
    async fn current(&mut self) -> Result<Option<CredentialRecord>, ()> {
        let mut scratch = [0u8; RECORD_SCRATCH];
        match self
            .storage
            .fetch_item::<StoredRecord>(&mut scratch, &RecordKey::Credential)
            .await
        {
            Ok(Some(stored)) => Ok(CredentialRecord::decode(&stored.bytes[..stored.len]).ok()),
            Ok(None) => Ok(None),
            Err(_) => Err(()),
        }
    }

    /// Writes the record, returning whether it reached the flash.
    #[allow(
        clippy::large_stack_frames,
        reason = "the encoded record and the 128-byte scratch buffer live in this future"
    )]
    async fn save(&mut self, record: &CredentialRecord) -> bool {
        let mut encoded = [0u8; record::MAX_ENCODED_LEN];
        let len = match record.encode_into(&mut encoded) {
            Ok(len) => len,
            Err(error) => {
                warn!("record: cannot encode ({}); not stored", error.label());
                return false;
            }
        };
        let stored = StoredRecord::new(encoded, len);
        let mut scratch = [0u8; RECORD_SCRATCH];
        match self
            .storage
            .store_item(&mut scratch, &RecordKey::Credential, &stored)
            .await
        {
            Ok(()) => {
                info!("record: stored (provisioned={} bytes={})", record.provisioned(), len);
                true
            }
            Err(error) => {
                warn!("record: write failed: {}", defmt::Debug2Format(&error));
                false
            }
        }
    }
}

/// Writes an already-built record through a store guard.
async fn save_locked(store: &mut CredentialStore, record: &CredentialRecord, context: &'static str) -> bool {
    if store.save(record).await {
        true
    } else {
        warn!("record: {} could not be written; it is RAM-only", context);
        false
    }
}

/// Writes a token-only record, clearing the network while keeping a usable code.
#[allow(
    clippy::large_stack_frames,
    reason = "the record and the borrowed store guard live in this future"
)]
async fn persist_token_only(token: [u8; pairing::CODE_LEN], context: &'static str) -> bool {
    let record = CredentialRecord::token_only(token);
    let mut guard = CREDENTIAL_STORE.lock().await;
    match guard.as_mut() {
        Some(store) => save_locked(store, &record, context).await,
        None => {
            warn!("record: no data partition; {} is RAM-only", context);
            false
        }
    }
}

/// Writes a provisioned record, taking the token from the record itself so a concurrent token
/// rotation cannot be overwritten with a stale copy.
#[allow(
    clippy::large_stack_frames,
    reason = "the record and the borrowed store guard live in this future"
)]
async fn persist_credentials(credentials: Credentials) -> bool {
    let mut guard = CREDENTIAL_STORE.lock().await;
    let Some(store) = guard.as_mut() else {
        warn!("record: no data partition; the credentials are RAM-only");
        return false;
    };
    let token = match store.current().await {
        Ok(Some(record)) => record.token,
        Ok(None) => match pairing_token() {
            Some(token) => token,
            None => {
                warn!("record: no pairing token; the credentials were not stored");
                return false;
            }
        },
        Err(()) => {
            warn!("record: the read failed; the credentials were not overwritten");
            return false;
        }
    };
    let record = CredentialRecord {
        credentials: Some(credentials),
        token,
    };
    save_locked(store, &record, "the credentials").await
}

/// Writes the current credentials with a new token, reading the credentials inside the lock.
#[allow(
    clippy::large_stack_frames,
    reason = "the record and the borrowed store guard live in this future"
)]
async fn persist_rotated(token: [u8; pairing::CODE_LEN]) -> bool {
    let mut guard = CREDENTIAL_STORE.lock().await;
    let Some(store) = guard.as_mut() else {
        warn!("record: no data partition; the rotated token is RAM-only");
        return false;
    };
    let credentials = match store.current().await {
        Ok(Some(record)) => record.credentials,
        Ok(None) => None,
        Err(()) => {
            warn!("record: the read failed; the token was not rotated");
            return false;
        }
    };
    let record = CredentialRecord { credentials, token };
    save_locked(store, &record, "the rotated token").await
}

/// Finds the `bridge_data` partition and opens the record store over it.
///
/// Returns `None` when the table is unreadable or the partition is absent, which is what a board
/// flashed without the custom table looks like. The Bridge still boots; it just cannot persist.
fn open_record_store(flash: esp_hal::peripherals::FLASH<'static>) -> Option<CredentialStore> {
    let mut flash = esp_storage::FlashStorage::new(flash);
    let mut table_buffer = [0u8; PARTITION_TABLE_LEN];
    let table = match read_partition_table(&mut flash, &mut table_buffer) {
        Ok(table) => table,
        Err(error) => {
            warn!(
                "record: the partition table is unreadable: {}",
                defmt::Debug2Format(&error)
            );
            return None;
        }
    };

    let Some(partition) = table.iter().find(|entry| entry.label_as_str() == DATA_PARTITION_LABEL) else {
        warn!(
            "record: no {} partition; flash the custom partition table from bridge-firmware/partitions.csv",
            DATA_PARTITION_LABEL
        );
        return None;
    };
    if partition.len() < 2 * esp_storage::FlashStorage::SECTOR_SIZE {
        warn!(
            "record: {} is {} bytes; sequential-storage needs at least two sectors",
            DATA_PARTITION_LABEL,
            partition.len()
        );
        return None;
    }

    info!(
        "record: partition {} at {:#x} ({} bytes)",
        partition.label_as_str(),
        partition.offset(),
        partition.len()
    );
    let base = partition.offset();
    let len = partition.len();
    let storage = MapStorage::new(
        BlockingAsync::new(PartitionFlash::new(flash, base, len)),
        MapConfig::new(0..len),
        NoCache::new(),
    );
    Some(CredentialStore { storage })
}

/// Doubles the reconnect backoff, capped at [`MAX_BACKOFF`].
fn next_backoff(backoff: Duration) -> Duration {
    (backoff * 2).min(MAX_BACKOFF)
}

/// Best-of-N 2.4 GHz scan for the portal: `scan_async` on this beta drops whole sweeps, so merge
/// several and keep the strongest RSSI per SSID. The AP's own SSID is filtered out.
async fn run_scan_sweeps(controller: &mut WifiController<'_>, own_ssid: &[u8]) {
    with_scan_cache(ScanCache::clear);
    for sweep in 1..=SCAN_SWEEPS {
        match controller.scan_async(&ScanConfig::default()).await {
            Ok(aps) => {
                SCAN_SWEEPS_RUN.fetch_add(1, Ordering::Relaxed);
                for ap in &aps {
                    let ssid = ap.ssid.as_str().as_bytes();
                    if ssid.is_empty() || ssid == own_ssid {
                        continue;
                    }
                    let secure = !matches!(ap.auth_method, None | Some(AuthenticationMethod::None));
                    with_scan_cache(|cache| cache.observe(ssid, ap.signal_strength, secure));
                    SCAN_APS_SEEN.fetch_add(1, Ordering::Relaxed);
                }
                info!("portal: scan sweep {}/{} saw {} APs", sweep, SCAN_SWEEPS, aps.len());
            }
            Err(error) => warn!("portal: scan sweep {}/{} failed: {:?}", sweep, SCAN_SWEEPS, error),
        }
        if sweep < SCAN_SWEEPS {
            Timer::after(Duration::from_millis(300)).await;
        }
    }
}

/// Scans for the configured station SSID and logs every BSS's auth mode before associating.
///
/// `esp-radio` 1.0.0-beta.1 only exposes `Wpa2Personal`; a WPA3-only or PMF-required AP is the
/// usual cause of an association failure, and the advertised auth mode settles that before any
/// credentials are tried. `scan_async` on this beta can miss a sweep, so an empty result is retried.
async fn scan_station_target(controller: &mut WifiController<'_>, target: &[u8]) {
    let Ok(ssid) = Ssid::try_from(target) else {
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

/// Associates the station, classifying a WPA2 credential rejection so the portal can say
/// "wrong password" instead of a generic failure.
async fn connect_station(controller: &mut WifiController<'_>) -> JoinOutcome {
    match with_timeout(CONNECT_TIMEOUT, controller.connect_async()).await {
        Ok(Ok(info)) => {
            info!("sta: associated ssid={} channel={}", info.ssid, info.channel);
            JoinOutcome::Connected
        }
        Ok(Err(ConnectionError::Failed(info))) => {
            if auth_failure(info.reason) {
                warn!("sta: association rejected: {:?}", info.reason);
                JoinOutcome::WrongPassword
            } else {
                warn!("sta: association failed: {:?}", info.reason);
                JoinOutcome::Failed
            }
        }
        Ok(Err(ConnectionError::WifiError(WifiError::InvalidPassword))) => {
            warn!("sta: the radio rejected the passphrase");
            JoinOutcome::WrongPassword
        }
        Ok(Err(error)) => {
            warn!("sta: association failed: {:?}", error);
            JoinOutcome::Failed
        }
        Err(_) => {
            warn!("sta: association timed out");
            JoinOutcome::Failed
        }
    }
}

/// Returns whether a disconnect reason means the WPA2 pre-shared key was wrong.
fn auth_failure(reason: DisconnectReason) -> bool {
    matches!(
        reason,
        DisconnectReason::AuthenticationExpired
            | DisconnectReason::MicFailure
            | DisconnectReason::FourWayHandshakeTimeout
            | DisconnectReason::GroupKeyUpdateTimeout
            | DisconnectReason::IeIn4wayDiffers
            | DisconnectReason::_802_1xAuthenticationFailed
            | DisconnectReason::CipherSuiteRejected
            | DisconnectReason::BadCipherOrAkm
    )
}

/// Bits per second converted to whole kilobits per second for a status interval.
fn kbps(bytes: u32, interval: Duration) -> u32 {
    let seconds = interval.as_secs().max(1);
    ((u64::from(bytes) * 8) / seconds / 1_000) as u32
}

// ---------------------------------------------------------------------------------------------
// Network tasks
// ---------------------------------------------------------------------------------------------

/// One `embassy-net` stack's runner (station or AP).
///
/// `runner.run()` never returns and may not wake while the stack is idle, so it is polled in
/// [`HEARTBEAT_PERIOD`] slices. Every slice ends with a beat, and cancelling the poll between
/// slices is safe: the runner keeps its state in the stack and the driver, not in the future.
#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, Interface>, task: CoveredTask) {
    info!("net: {} stack runner started", task.label());
    loop {
        check_in(task).await;
        let _ = with_timeout(HEARTBEAT_PERIOD, runner.run()).await;
    }
}

/// The RTC watchdog supervisor.
///
/// Stage 0 resets the whole system. The feed is withheld while any covered task is stale, so a
/// stalled task turns into one benign Bridge reboot instead of a permanent hang. A reboot is
/// benign by design: `bridge_data` is outside the flashed set, the supervisory loop rejoins the
/// station, the daemon reconnects, and ADR-0035 keeps the robot's Boot Id and motion unaffected.
#[embassy_executor::task]
async fn watchdog_task(mut rwdt: Rwdt) {
    rwdt.set_timeout(RwdtStage::Stage0, HalDuration::from_millis(WATCHDOG_TIMEOUT_MS));
    rwdt.set_stage_action(RwdtStage::Stage0, RwdtStageAction::ResetSystem);
    rwdt.enable();
    rwdt.feed();
    info!(
        "watchdog: rwdt stage 0 resets after {}s; poll {}s; max task silence {}s",
        WATCHDOG_TIMEOUT_MS / 1_000,
        WATCHDOG_POLL.as_secs(),
        HEARTBEAT_MAX_AGE_MS / 1_000
    );

    let mut reported = None;
    loop {
        Timer::after(WATCHDOG_POLL).await;
        let stale = critical_section::with(|cs| HEARTBEATS.borrow(cs).borrow().stale(now_ms(), HEARTBEAT_MAX_AGE_MS));
        match stale {
            None => {
                if reported.take().is_some() {
                    info!("watchdog: every covered task is beating again");
                }
                WATCHDOG_FEEDS.fetch_add(1, Ordering::Relaxed);
                rwdt.feed();
            }
            Some(task) => {
                WATCHDOG_WITHHELD.fetch_add(1, Ordering::Relaxed);
                if reported != Some(task) {
                    warn!(
                        "watchdog: {} has not beaten for over {}s; withholding the feed",
                        task.label(),
                        HEARTBEAT_MAX_AGE_MS / 1_000
                    );
                    reported = Some(task);
                }
            }
        }
    }
}

/// The setup AP's hand-rolled DHCP server (RFC 2131): DISCOVER/OFFER and REQUEST/ACK, broadcast
/// replies, a two-client pool. No DHCP crate.
#[allow(
    clippy::large_stack_frames,
    reason = "the task owns the UDP socket and the two datagram buffers for its lifetime"
)]
#[embassy_executor::task]
async fn ap_dhcp_task(stack: Stack<'static>) {
    let mut socket = UdpSocket::new(
        stack,
        DHCP_RX_META.init([PacketMetadata::EMPTY; 1]),
        DHCP_RX_BUFFER.init([0u8; PORTAL_UDP_BUFFER]),
        DHCP_TX_META.init([PacketMetadata::EMPTY; 1]),
        DHCP_TX_BUFFER.init([0u8; PORTAL_UDP_BUFFER]),
    );
    if socket.bind(dhcp::SERVER_PORT).is_err() {
        warn!("ap: could not bind UDP {}", dhcp::SERVER_PORT);
        return;
    }
    let mut server = dhcp::Server::new(dhcp::ServerConfig::standard());
    let mut request = [0u8; PORTAL_UDP_BUFFER];
    let mut response = [0u8; PORTAL_UDP_BUFFER];
    info!("ap: DHCP server on UDP {}", dhcp::SERVER_PORT);
    loop {
        let (length, _meta) = match socket.recv_from(&mut request).await {
            Ok(pair) => pair,
            Err(error) => {
                warn!("ap: DHCP receive failed: {:?}", error);
                Timer::after(Duration::from_millis(250)).await;
                continue;
            }
        };
        if !SETUP_OPEN.load(Ordering::Relaxed) {
            continue;
        }
        if let Some(reply) = server.handle(&request[..length], now_ms(), &mut response) {
            match reply.kind {
                dhcp::ReplyKind::Offer => {
                    DHCP_OFFERS.fetch_add(1, Ordering::Relaxed);
                }
                dhcp::ReplyKind::Ack => {
                    DHCP_ACKS.fetch_add(1, Ordering::Relaxed);
                }
            }
            if socket
                .send_to(&response[..reply.length], (Ipv4Address::BROADCAST, dhcp::CLIENT_PORT))
                .await
                .is_err()
            {
                warn!("ap: DHCP reply send failed");
            }
        }
    }
}

/// The setup AP's DNS catch-all: every A query resolves to the AP's address, so a captive probe
/// lands on the portal.
#[allow(
    clippy::large_stack_frames,
    reason = "the task owns the UDP socket and the two datagram buffers for its lifetime"
)]
#[embassy_executor::task]
async fn ap_dns_task(stack: Stack<'static>) {
    let mut socket = UdpSocket::new(
        stack,
        DNS_RX_META.init([PacketMetadata::EMPTY; 1]),
        DNS_RX_BUFFER.init([0u8; PORTAL_UDP_BUFFER]),
        DNS_TX_META.init([PacketMetadata::EMPTY; 1]),
        DNS_TX_BUFFER.init([0u8; PORTAL_UDP_BUFFER]),
    );
    if socket.bind(dns::PORT).is_err() {
        warn!("ap: could not bind UDP {}", dns::PORT);
        return;
    }
    let mut request = [0u8; PORTAL_UDP_BUFFER];
    let mut reply = [0u8; PORTAL_UDP_BUFFER];
    info!("ap: DNS catch-all on UDP {}", dns::PORT);
    loop {
        let (length, meta) = match socket.recv_from(&mut request).await {
            Ok(pair) => pair,
            Err(error) => {
                warn!("ap: DNS receive failed: {:?}", error);
                Timer::after(Duration::from_millis(250)).await;
                continue;
            }
        };
        if !SETUP_OPEN.load(Ordering::Relaxed) {
            continue;
        }
        if let Some(reply_len) = dns::answer(&request[..length], AP_ADDRESS, &mut reply) {
            DNS_ANSWERS.fetch_add(1, Ordering::Relaxed);
            if socket.send_to(&reply[..reply_len], meta.endpoint).await.is_err() {
                warn!("ap: DNS reply send failed");
            }
        }
    }
}

/// One portal HTTP worker: accept, read one request, answer it and close.
///
/// Two workers so the page's 1 s status poll and a form submit never block each other.
#[embassy_executor::task(pool_size = 2)]
async fn ap_http_task(
    stack: Stack<'static>,
    rx_buffer: &'static mut [u8],
    tx_buffer: &'static mut [u8],
    request: &'static mut [u8],
    response: &'static mut [u8],
) {
    let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
    loop {
        if !SETUP_OPEN.load(Ordering::Relaxed) {
            Timer::after(Duration::from_millis(250)).await;
            continue;
        }
        socket.set_timeout(Some(PORTAL_READ_TIMEOUT));
        if socket.accept(PORTAL_HTTP_PORT).await.is_err() {
            Timer::after(Duration::from_secs(1)).await;
            continue;
        }
        PORTAL_REQUESTS.fetch_add(1, Ordering::Relaxed);
        if serve_portal(&mut socket, request, response).await {
            info!("portal: response sent");
        }
        finish_socket(&mut socket).await;
    }
}

/// Reads one HTTP request into `request` and answers it, returning whether a response was sent.
async fn serve_portal(socket: &mut TcpSocket<'_>, request: &mut [u8], response: &mut [u8]) -> bool {
    let mut used = 0;
    loop {
        match portal::parse_request(&request[..used]) {
            Ok(parsed) => {
                let body = request.get(parsed.body_start..used).unwrap_or(&[]);
                return answer_portal(socket, parsed, body, response).await;
            }
            Err(portal::RequestError::Incomplete) => {
                if used >= request.len() {
                    return send_error(socket, 400, response).await;
                }
                match with_timeout(PORTAL_READ_TIMEOUT, socket.read(&mut request[used..])).await {
                    Ok(Ok(0)) => return false,
                    Ok(Ok(count)) => used += count,
                    Ok(Err(_)) | Err(_) => return false,
                }
            }
            Err(portal::RequestError::Malformed) => return send_error(socket, 400, response).await,
        }
    }
}

/// Routes one parsed portal request and writes its response.
#[allow(
    clippy::large_stack_frames,
    reason = "the portal credential command is the largest value this router holds"
)]
async fn answer_portal(
    socket: &mut TcpSocket<'_>,
    request: portal::Request<'_>,
    body: &[u8],
    response: &mut [u8],
) -> bool {
    let length = match (request.method, request.path) {
        (portal::Method::Get, "/") | (portal::Method::Get, "/index.html") => {
            let status = portal_status();
            with_scan_cache(|cache| portal::write_index_page(cache, status, response))
        }
        (portal::Method::Get, "/status") => portal::write_status_response(portal_status(), response),
        (portal::Method::Post, "/provision") => match portal::parse_provision(body) {
            Ok(credentials) => match PORTAL_COMMANDS.try_send(PortalCommand::Provision(credentials)) {
                Ok(()) => {
                    set_portal_status(Status::Connecting);
                    portal::write_redirect(response)
                }
                Err(_) => {
                    warn!("portal: the provision command queue is full");
                    portal::write_error(400, response)
                }
            },
            Err(_) => portal::write_error(400, response),
        },
        (portal::Method::Post, "/forget") => match PORTAL_COMMANDS.try_send(PortalCommand::Forget) {
            Ok(()) => {
                set_portal_status(Status::Unprovisioned);
                portal::write_redirect(response)
            }
            Err(_) => {
                warn!("portal: the forget command queue is full");
                portal::write_error(400, response)
            }
        },
        (_, "/" | "/index.html" | "/status" | "/provision" | "/forget") => portal::write_error(405, response),
        _ => portal::write_error(404, response),
    };

    let Some(length) = length else {
        return send_error(socket, 400, response).await;
    };
    write_portal_response(socket, &response[..length]).await
}

/// Writes a complete HTTP error response, falling back to closing the socket if it cannot fit.
async fn send_error(socket: &mut TcpSocket<'_>, code: u16, response: &mut [u8]) -> bool {
    let Some(length) = portal::write_error(code, response) else {
        return false;
    };
    write_portal_response(socket, &response[..length]).await
}

/// Writes the whole portal response without touching the Link Protocol's counters.
async fn write_portal_response(socket: &mut TcpSocket<'_>, data: &[u8]) -> bool {
    let mut written = 0;
    while written < data.len() {
        match socket.write(&data[written..]).await {
            Ok(0) => return false,
            Ok(count) => written += count,
            Err(_) => return false,
        }
    }
    true
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
async fn client_task(
    stack: Stack<'static>,
    rx_buffer: &'static mut [u8],
    tx_buffer: &'static mut [u8],
    task: CoveredTask,
) {
    let mut socket = TcpSocket::new(stack, rx_buffer, tx_buffer);
    let mut client_buffer = alloc::vec![0u8; CLIENT_READ_CHUNK];
    let mut parser = PcToRobotParser::new(Direction::PcToRobot, UART_BAUD);

    loop {
        check_in(task).await;
        // Setup Mode serves the portal, not the Link Protocol: the Beacon advertises
        // `PORTAL_ONLY` and no new client is admitted. An already-authenticated session is left
        // alone, matching ADR-0036's "the Link Protocol still on STA if associated".
        if SETUP_OPEN.load(Ordering::Relaxed) {
            Timer::after(Duration::from_millis(250)).await;
            continue;
        }
        socket.set_timeout(Some(SOCKET_TIMEOUT));
        // The accept wait is sliced so an idle worker still checks in; a timeout is simply the
        // next heartbeat slice, while a real accept error keeps the old one-second backoff.
        match with_timeout(HEARTBEAT_PERIOD, socket.accept(LINK_TCP_DEFAULT)).await {
            Ok(Ok(())) => {}
            Ok(Err(_)) => {
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
            Err(_) => continue,
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

        let Some(token) = pairing_token() else {
            warn!("tcp: no pairing token; denying the connection");
            with_pump(|pump| pump.disconnect(connection));
            deny_and_finish(&mut socket, DenyReason::Unauthorized, None).await;
            continue;
        };

        // The preamble has its own five-second deadline; beat first so the longest silent stretch
        // in this worker stays inside the watchdog's maximum age.
        beat_task(task);
        match run_preamble(&mut socket, connection, &token).await {
            PreambleOutcome::Accepted => {
                AUTH_ACCEPTED.fetch_add(1, Ordering::Relaxed);
                info!("tcp: client {} authenticated", connection);
                // A previous client may have left a partial candidate in the parser; discard it so
                // its bytes cannot be glued to this client's first frame.
                parser.reset();
                serve_client(&mut socket, &mut parser, &mut client_buffer, connection, task).await;
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
    task: CoveredTask,
) {
    loop {
        check_in(task).await;
        // A rotated pairing code disconnects the authenticated connection from the supervisory
        // loop; observe it here so the socket closes instead of serving the old token's session.
        if with_pump(|pump| pump.authenticated_connection()) != Some(connection) {
            info!("tcp: authentication revoked; closing the client");
            break;
        }

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
/// never describe the Bridge differently. In Setup Mode the Beacon advertises
/// [`beacon::PORTAL_ONLY`]: the portal is the served surface and the Link listener does not admit
/// clients, so the daemon must not connect. In AP-only Setup Mode there is no station Beacon at
/// all (ADR-0037's amendment).
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
        check_in(CoveredTask::Beacon).await;
        if STATION_UP.load(Ordering::Relaxed) {
            let tcp_port = if SETUP_OPEN.load(Ordering::Relaxed) {
                beacon::PORTAL_ONLY
            } else {
                LINK_TCP_DEFAULT
            };
            let beacon = Beacon {
                major: beacon::LINK_MAJOR,
                device_id: device_id(),
                name: bridge_name(),
                tcp_port,
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

/// Returns the pairing token, or `None` when the TRNG could not mint one.
fn pairing_token() -> Option<[u8; pairing::CODE_LEN]> {
    critical_section::with(|cs| *PAIRING_TOKEN.borrow(cs).borrow())
}

/// Stores the pairing token for the preamble and the panel's replies.
fn store_pairing_token(token: [u8; pairing::CODE_LEN]) {
    critical_section::with(|cs| *PAIRING_TOKEN.borrow(cs).borrow_mut() = Some(token));
}

/// Mints a 60-bit pairing token from the hardware TRNG (ADR-0037).
///
/// `Rng` alone is not cryptographic before the radio is up, so this path requires the
/// [`TrngSource`] the caller keeps alive. The eight TRNG bytes are reduced to 60 bits by
/// [`pairing::encode`].
fn mint_token() -> Option<[u8; pairing::CODE_LEN]> {
    let trng = match Trng::try_new() {
        Ok(trng) => trng,
        Err(_) => {
            warn!("rng: the TRNG source is not active; refusing to mint a token");
            return None;
        }
    };
    let mut bytes = [0u8; 8];
    trng.read(&mut bytes);
    Some(pairing::encode(u64::from_be_bytes(bytes)))
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
/// `provisioned` is whether the credential record stores a usable station credential pair;
/// `setup_mode` is the bounded window, which is only observable on the station Beacon in APSTA.
fn flags_now() -> IdentityFlags {
    let mut flags = IdentityFlags::default();
    if PROVISIONED.load(Ordering::Relaxed) {
        flags |= IdentityFlags::PROVISIONED;
    }
    if link_ready() {
        flags |= IdentityFlags::LINK_READY;
    }
    if SETUP_OPEN.load(Ordering::Relaxed) {
        flags |= IdentityFlags::SETUP_MODE;
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

/// Encodes a `BRIDGE_CONTROL` acknowledgement and queues it for the robot.
///
/// The reply's `Seq` echoes the request's, and the flag is `IS_ACK` alone: the acknowledgement
/// must not ask for another acknowledgement (ADR-0036's control rule, ADR-0037's revocation).
fn enqueue_bridge_ack(seq: Seq, reply: &BridgeControlReply<'_>) {
    let mut payload = [0u8; 96];
    let Ok(len) = encode_bridge_control_ack(reply, &mut payload) else {
        warn!("uart tx: could not encode the BRIDGE_CONTROL acknowledgement");
        return;
    };
    let Ok(frame) = Frame::new(
        PROTOCOL_MAJOR,
        Class::Control,
        Flags::IS_ACK,
        Control::BridgeControl.wire_id(),
        seq,
        &payload[..len],
    ) else {
        warn!("uart tx: could not build the BRIDGE_CONTROL acknowledgement");
        return;
    };
    enqueue_encoded(&frame);
}

/// Answers one robot-side `BRIDGE_CONTROL` request from the UART task.
///
/// Answers one robot-side `BRIDGE_CONTROL` request from the UART task.
///
/// The record write, the acknowledgement and the token change happen here, before the reply, so
/// the panel is never told a change succeeded when the flash still holds the old record. The
/// supervisory loop only clears the RAM credentials and raises the window for action 0, which keeps
/// the panel's two-second control timeout independent of a station join or a scan.
#[allow(
    clippy::large_stack_frames,
    reason = "the record and the persistence future live in the UART task's panel path"
)]
async fn handle_panel_request(request: BridgeControlRequest) {
    match request.action {
        BridgeControlAction::ForgetNetwork => {
            // ADR-0037 (amended for ticket 07): action 0 clears the network and rotates the token
            // rather than leaving the slot empty. The record is written before the RAM key changes
            // and before the acknowledgement; a failed write keeps the old code live and reports
            // `Internal`. The window is still raised, because the panel asked for the recovery
            // surface.
            let minted = mint_token();
            let persisted = match minted.or_else(pairing_token) {
                Some(token) => persist_token_only(token, "the cleared record").await,
                None => {
                    warn!("panel: no pairing token; the record cannot be cleared");
                    false
                }
            };
            if persisted && let Some(token) = minted {
                store_pairing_token(token);
            }
            let queued = queue_panel_command(PanelCommand::Forget);
            let status = if persisted && minted.is_some() && queued {
                remote_protocol::Status::Ok
            } else {
                remote_protocol::Status::Internal
            };
            let (ssid_bytes, ssid_len) = setup_ssid();
            let ssid = core::str::from_utf8(&ssid_bytes[..ssid_len]).unwrap_or("");
            enqueue_bridge_ack(
                request.seq,
                &BridgeControlReply::Setup(SetupCredentials {
                    status,
                    ssid,
                    passphrase: AP_PASSWORD,
                }),
            );
            disconnect_current_session();
            info!("panel: forgot the network; acknowledging with the setup identity");
        }
        BridgeControlAction::ShowPairingCode => {
            let current = pairing_token();
            let code = current.unwrap_or([b'0'; pairing::CODE_LEN]);
            let code = core::str::from_utf8(&code).unwrap_or("");
            let status = if current.is_some() {
                remote_protocol::Status::Ok
            } else {
                remote_protocol::Status::Internal
            };
            enqueue_bridge_ack(
                request.seq,
                &BridgeControlReply::PairingCode(PairingCode { status, code }),
            );
            info!("panel: showed the pairing code");
        }
        BridgeControlAction::RotatePairingCode => {
            let Some(token) = mint_token() else {
                warn!("panel: the TRNG did not mint a token; refusing the rotation");
                let current = pairing_token().unwrap_or([b'0'; pairing::CODE_LEN]);
                let code = core::str::from_utf8(&current).unwrap_or("");
                enqueue_bridge_ack(
                    request.seq,
                    &BridgeControlReply::PairingCode(PairingCode {
                        status: remote_protocol::Status::Internal,
                        code,
                    }),
                );
                return;
            };
            if persist_rotated(token).await {
                // The record is durable, so the new key becomes live and the old session ends.
                store_pairing_token(token);
                let code = core::str::from_utf8(&token).unwrap_or("");
                enqueue_bridge_ack(
                    request.seq,
                    &BridgeControlReply::PairingCode(PairingCode {
                        status: remote_protocol::Status::Ok,
                        code,
                    }),
                );
                disconnect_current_session();
                info!("panel: rotated the pairing code");
            } else {
                // The write failed: nothing changed durably, so the old code stays live and the
                // session is left alone. The panel sees `Internal` and can retry.
                let current = pairing_token().unwrap_or([b'0'; pairing::CODE_LEN]);
                let code = core::str::from_utf8(&current).unwrap_or("");
                enqueue_bridge_ack(
                    request.seq,
                    &BridgeControlReply::PairingCode(PairingCode {
                        status: remote_protocol::Status::Internal,
                        code,
                    }),
                );
                info!("panel: the rotation did not reach the flash; the old code stays active");
            }
        }
    }
}

/// Drops the current connection, authenticated or still proving the old token.
fn disconnect_current_session() {
    if let Some(connection) = with_pump(|pump| pump.current_connection()) {
        with_pump(|pump| {
            pump.disconnect(connection);
        });
        info!("tcp: panel action dropped connection {}", connection);
    }
}

/// Queues a panel action for the supervisory loop's record and radio work.
/// Queues a panel action for the supervisory loop's radio work.
///
/// Returns false when the queue is full; the caller then reports `Internal`, because the window
/// may not be raised promptly.
fn queue_panel_command(command: PanelCommand) -> bool {
    if PANEL_COMMANDS.try_send(command).is_err() {
        warn!("panel: the command queue is full; the window raise is delayed");
        false
    } else {
        true
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
        check_in(CoveredTask::UartRx).await;
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
                let mut pending: [Option<BridgeControlRequest>; PANEL_REQUEST_QUEUE] = [None; PANEL_REQUEST_QUEUE];
                parser.feed(data, now, |frame| {
                    // A frame only reaches this callback after the parser accepted its envelope and
                    // CRC, so it is the Beacon's pinned "valid Link Protocol frame from the robot".
                    LAST_ROBOT_FRAME_MS.store(now, Ordering::Relaxed);
                    frames = frames.wrapping_add(1);
                    match with_pump(|pump| pump.on_robot_frame(&frame)) {
                        Decision::DroppedFull => warn!("pump: ring full; dropped a robot frame"),
                        Decision::BridgeControl(request) => {
                            info!(
                                "pump: robot BRIDGE_CONTROL action={} seq={}",
                                request.action.bits(),
                                request.seq.get()
                            );
                            if let Some(slot) = pending.iter_mut().find(|slot| slot.is_none()) {
                                *slot = Some(request);
                            } else {
                                warn!("pump: too many BRIDGE_CONTROL frames in one DMA burst; dropped one");
                            }
                        }
                        _ => {}
                    }
                });

                // The acknowledgement and the record write happen outside the parser callback,
                // once the DMA slice is fully parsed. Consume the slice first so the descriptors
                // can refill while the flash write runs; the stream buffer is then not holding a
                // whole record write's worth of incoming robot traffic.
                let _ = transfer.consume(total);
                for slot in &mut pending {
                    if let Some(request) = slot.take() {
                        handle_panel_request(request).await;
                    }
                }

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
                    // The esp-hal stream buffer stops the DMA when every descriptor is full, and
                    // it does not resume by itself. Restart the transfer; without this the RX task
                    // keeps beating while no frame is ever received again, which the watchdog
                    // cannot see.
                    let (returned_rx, returned_buffer) = transfer.cancel();
                    rx = returned_rx;
                    buffer = returned_buffer;
                    transfer = loop {
                        match rx.read(buffer) {
                            Ok(transfer) => break transfer,
                            Err((error, returned_rx, returned_buffer)) => {
                                warn!("uart dma: restart failed: {:?}", error);
                                rx = returned_rx;
                                buffer = returned_buffer;
                                Timer::after(Duration::from_secs(1)).await;
                            }
                        }
                    };
                    done_since = None;
                    stall_counted = false;
                    continue;
                }
            } else {
                done_since = None;
                stall_counted = false;
            }
            Timer::after(Duration::from_millis(1)).await;
        }
    }
}

/// Writes robot-bound frames from the bounded queue with the async UART TX path (no TX DMA).
#[embassy_executor::task]
async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    loop {
        check_in(CoveredTask::UartTx).await;
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
