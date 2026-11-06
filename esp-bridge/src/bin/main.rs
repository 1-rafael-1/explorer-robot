#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

//! Ticket 09 — the Bridge language spike.
//!
//! This binary answers one question: **does the Rust Wi-Fi stack wedge** under sustained
//! bidirectional UART + TCP traffic and forced disconnect/reconnect, with the controller in
//! APSTA?
//!
//! It is not the Bridge firmware. It deliberately has no Link Protocol framing, no robot
//! lifecycle, no boot loader hand-off and no provisioning. It is a load generator and an
//! observer.
//!
//! ## Bench wiring
//!
//! Power the C3 over USB. Jumper **GPIO6 (UART1 TX) to GPIO7 (UART1 RX)**. Everything else is
//! self-contained: the UART task transmits a known frame pattern and verifies the echo, while a
//! TCP client on the LAN (or on the setup AP) drives the network side.
//!
//! ## Credentials
//!
//! The station credentials are compile-time inputs so the binary contains no secrets:
//!
//! ```text
//! SPIKE_STA_SSID=... SPIKE_STA_PASSWORD=... cargo run --release
//! ```
//!
//! The setup AP has fixed defaults (`explorer-spike` / `spike12345`) because the soak client may
//! connect to it directly. With no station credentials the binary still brings the AP up; it
//! simply cannot exercise the station half of APSTA.

use portable_atomic::{AtomicBool, AtomicU32, Ordering};

use bridge_net as _;
use defmt::info;
use embassy_executor::Spawner;
use embassy_net::{Ipv4Address, Ipv4Cidr, Runner, Stack, StackResources, StaticConfigV4};
use embassy_time::{Duration, Instant, Timer, with_timeout};
use esp_backtrace as _;
use esp_hal::dma_rx_stream_buffer;
use esp_hal::{
    Async,
    clock::CpuClock,
    timer::timg::TimerGroup,
    uart::{
        Config as UartConfig, RxConfig, RxError, TxError, Uart, UartRx, UartTx,
        uhci::{RxConfig as UhciRxConfig, Uhci, UhciRx},
    },
};
use esp_println as _;
use esp_radio::wifi::{
    AuthenticationMethodConfig, Config as WifiConfig, ControllerConfig, Interface, WifiController,
    ap::AccessPointConfig, sta::StationConfig,
};
use heapless::spsc::{Consumer, Producer, Queue};
use remote_protocol as _;
use static_cell::StaticCell;

extern crate alloc;

esp_bootloader_esp_idf::esp_app_desc!();

// ---------------------------------------------------------------------------------------------
// Compile-time inputs
// ---------------------------------------------------------------------------------------------

/// Station SSID. Pass `SPIKE_STA_SSID` at build time.
const STA_SSID: &str = match option_env!("SPIKE_STA_SSID") {
    Some(value) => value,
    None => "CHANGE_ME",
};

/// Station password. Pass `SPIKE_STA_PASSWORD` at build time.
const STA_PASSWORD: &str = match option_env!("SPIKE_STA_PASSWORD") {
    Some(value) => value,
    None => "CHANGE_ME",
};

/// Setup AP SSID. Fixed so the soak client can be pointed at it without a lookup.
const AP_SSID: &str = "explorer-spike";

/// Setup AP password. WPA2, so it must be at least eight bytes.
const AP_PASSWORD: &str = "spike12345";

/// Setup AP channel. Ticket 11 says this must become a setting; here it is a spike constant.
const AP_CHANNEL: u8 = 1;

/// Setup AP address. There is no DHCP server in this binary; set the client to 192.168.4.2/24.
const AP_ADDRESS: [u8; 4] = [192, 168, 4, 1];

/// The TCP echo port, on both the station and the setup AP.
const ECHO_PORT: u16 = 9000;

// ---------------------------------------------------------------------------------------------
// Test shape
// ---------------------------------------------------------------------------------------------

/// Parses a decimal `u32` at compile time, returning `fallback` on any non-digit input.
const fn parse_u32(value: &str, fallback: u32) -> u32 {
    let bytes = value.as_bytes();
    if bytes.is_empty() {
        return fallback;
    }

    let mut result = 0u32;
    let mut index = 0;
    while index < bytes.len() {
        let byte = bytes[index];
        if !byte.is_ascii_digit() {
            return fallback;
        }
        result = match result.checked_mul(10) {
            Some(value) => match value.checked_add((byte - b'0') as u32) {
                Some(value) => value,
                None => return fallback,
            },
            None => return fallback,
        };
        index += 1;
    }
    result
}

/// Ticket 04's chosen target baud. Override with `SPIKE_UART_BAUD` for a control run.
const UART_BAUD: u32 = match option_env!("SPIKE_UART_BAUD") {
    Some(value) => parse_u32(value, 921_600),
    None => 921_600,
};

/// RX FIFO-full threshold. The driver default (120) leaves only ~87 us of headroom at 921600;
/// override with `SPIKE_RX_THRESHOLD` to test lower wake points.
const UART_RX_THRESHOLD: u16 = match option_env!("SPIKE_RX_THRESHOLD") {
    Some(value) => parse_u32(value, 32) as u16,
    None => 32,
};

/// UART1's default pins on the C3. Jumper these two together. Keep in step with the
/// `with_tx`/`with_rx` calls below.
const UART_TX_PIN: u8 = 6;
const UART_RX_PIN: u8 = 7;

/// The software ring between the UART reader and the frame verifier. Ticket 04 sizes the real
/// Bridge ring at 8 KiB; using the same number here keeps the observation representative.
const UART_QUEUE_CAPACITY: usize = 8 * 1024;

/// How much the UART reader asks the driver for in one call.
const UART_READ_CHUNK: usize = 512;

/// Use UHCI RX DMA instead of the async FIFO reader. Enable with `SPIKE_UART_RX_DMA=1`.
const UART_RX_DMA: bool = option_env!("SPIKE_UART_RX_DMA").is_some();

/// UHCI RX DMA buffer and descriptor chunk size. Smaller chunks mean lower latency and more
/// interrupts; the stream buffer keeps several descriptors in flight.
const DMA_RX_BUFFER: usize = match option_env!("SPIKE_DMA_BUFFER") {
    Some(value) => parse_u32(value, 4096) as usize,
    None => 4096,
};
const DMA_RX_CHUNK: usize = match option_env!("SPIKE_DMA_CHUNK") {
    Some(value) => parse_u32(value, 128) as usize,
    None => 128,
};

/// Optional delay between loopback frames. Zero means full speed. Ticket 04's real worst case
/// is ~10.3 KB/s, so a 256-byte frame every 25 ms (10.24 KB/s) is the realistic control.
const UART_FRAME_INTERVAL_MS: u32 = match option_env!("SPIKE_UART_FRAME_INTERVAL_MS") {
    Some(value) => parse_u32(value, 0),
    None => 0,
};

/// The loopback frame. Header, sequence, then a deterministic pattern.
const FRAME_SIZE: usize = 256;
const FRAME_MAGIC: [u8; 2] = [0xAA, 0x55];

/// Ten minutes of quiet APSTA, then repeated forced station reconnects. Override with
/// `SPIKE_SOAK_SECS` / `SPIKE_RECONNECT_SECS` for a shorter bench cycle.
const SOAK_BEFORE_RECONNECT: Duration = Duration::from_secs(match option_env!("SPIKE_SOAK_SECS") {
    Some(value) => parse_u32(value, 600) as u64,
    None => 600,
});
const RECONNECT_PERIOD: Duration = Duration::from_secs(match option_env!("SPIKE_RECONNECT_SECS") {
    Some(value) => parse_u32(value, 120) as u64,
    None => 120,
});

/// Status line cadence.
const STATUS_PERIOD: Duration = Duration::from_secs(10);

/// How long the station may take to associate before the spike gives up and retries.
const CONNECT_TIMEOUT: Duration = Duration::from_secs(30);

// ---------------------------------------------------------------------------------------------
// Statics
// ---------------------------------------------------------------------------------------------

static STA_RESOURCES: StaticCell<StackResources<3>> = StaticCell::new();
static AP_RESOURCES: StaticCell<StackResources<2>> = StaticCell::new();
static UART_QUEUE: StaticCell<Queue<u8, UART_QUEUE_CAPACITY>> = StaticCell::new();

static UART_TX_BYTES: AtomicU32 = AtomicU32::new(0);
static UART_RX_BYTES: AtomicU32 = AtomicU32::new(0);
static UART_RX_ERRORS: AtomicU32 = AtomicU32::new(0);
static UART_RX_FIFO_OVERFLOWED: AtomicU32 = AtomicU32::new(0);
static UART_RX_FRAME_VIOLATED: AtomicU32 = AtomicU32::new(0);
static UART_RX_GLITCHED: AtomicU32 = AtomicU32::new(0);
static UART_RX_PARITY_MISMATCH: AtomicU32 = AtomicU32::new(0);
static UART_RX_OTHER_ERROR: AtomicU32 = AtomicU32::new(0);
static UART_RX_FIRST_ERROR_LOGGED: AtomicBool = AtomicBool::new(false);
static UART_DMA_STALLS: AtomicU32 = AtomicU32::new(0);
static UART_DROPPED_BYTES: AtomicU32 = AtomicU32::new(0);
static UART_QUEUE_HIGH_WATER: AtomicU32 = AtomicU32::new(0);
static UART_GOOD_FRAMES: AtomicU32 = AtomicU32::new(0);
static UART_BAD_FRAMES: AtomicU32 = AtomicU32::new(0);
static TCP_RX_BYTES: AtomicU32 = AtomicU32::new(0);
static TCP_TX_BYTES: AtomicU32 = AtomicU32::new(0);
static RECONNECTS: AtomicU32 = AtomicU32::new(0);

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

    info!("ticket 09 spike: APSTA + UART loopback + TCP echo");

    // -- Wi-Fi -------------------------------------------------------------------------------

    let station_config = StationConfig::default()
        .with_ssid(STA_SSID.try_into().expect("station SSID is too long"))
        .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
            STA_PASSWORD.try_into().expect("station password is too long"),
        ));
    let access_point_config = AccessPointConfig::default()
        .with_ssid(AP_SSID.try_into().expect("AP SSID is too long"))
        .with_channel(AP_CHANNEL)
        .with_authentication(AuthenticationMethodConfig::Wpa2Personal(
            AP_PASSWORD.try_into().expect("AP password is too long"),
        ));

    let mut controller = WifiController::new(peripherals.WIFI, ControllerConfig::default())
        .expect("failed to initialize the Wi-Fi controller");
    controller
        .set_config(&WifiConfig::AccessPointStation(station_config, access_point_config))
        .expect("failed to enter APSTA");

    if STA_SSID == "CHANGE_ME" {
        info!("no SPIKE_STA_SSID was set; the setup AP will run but the station will not associate");
    }

    // -- Network stacks ----------------------------------------------------------------------

    let station_interface = Interface::station();
    let access_point_interface = Interface::access_point();

    let (station_stack, station_runner) = embassy_net::new(
        station_interface,
        embassy_net::Config::dhcpv4(Default::default()),
        STA_RESOURCES.init(StackResources::new()),
        0x1234_5678_9abc_def0,
    );
    let (access_point_stack, access_point_runner) = embassy_net::new(
        access_point_interface,
        embassy_net::Config::ipv4_static(StaticConfigV4 {
            address: Ipv4Cidr::new(
                Ipv4Address::new(AP_ADDRESS[0], AP_ADDRESS[1], AP_ADDRESS[2], AP_ADDRESS[3]),
                24,
            ),
            gateway: None,
            dns_servers: heapless::Vec::new(),
        }),
        AP_RESOURCES.init(StackResources::new()),
        0x0fed_cba9_8765_4321,
    );

    spawner.spawn(net_task(station_runner, "sta").expect("net task pool"));
    spawner.spawn(net_task(access_point_runner, "ap").expect("net task pool"));

    info!(
        "ap: ssid={} channel={} address=192.168.4.1/24 (client: 192.168.4.2/24)",
        AP_SSID, AP_CHANNEL
    );

    // -- UART loopback -----------------------------------------------------------------------

    info!(
        "uart: UART1 {} baud, TX=GPIO{} RX=GPIO{}, rx threshold={}B, frame interval={}ms; jumper the two pins",
        UART_BAUD, UART_TX_PIN, UART_RX_PIN, UART_RX_THRESHOLD, UART_FRAME_INTERVAL_MS
    );

    // The driver default is a 120-byte RX FIFO threshold. At 921600 that leaves only
    // 8 bytes (~87 us) before the 128-byte FIFO overflows, which ordinary interrupt latency
    // exceeds. Waking at 32 bytes leaves ~1 ms of headroom at four times the wake rate.
    let uart_config = UartConfig::default()
        .with_baudrate(UART_BAUD)
        .with_rx(RxConfig::default().with_fifo_full_threshold(UART_RX_THRESHOLD));

    let (uart_producer, uart_consumer) = UART_QUEUE.init(Queue::new()).split();

    let uart_tx = if UART_RX_DMA {
        info!("uart rx: UHCI DMA, buffer={}B chunk={}B", DMA_RX_BUFFER, DMA_RX_CHUNK);
        let uart = Uart::new(peripherals.UART1, uart_config)
            .expect("UART configuration rejected")
            .with_rx(peripherals.GPIO7)
            .with_tx(peripherals.GPIO6);
        let mut uhci = Uhci::new(uart, peripherals.UHCI0, peripherals.DMA_CH0);
        uhci.apply_rx_config(&UhciRxConfig::default().with_chunk_limit(DMA_RX_CHUNK as u16))
            .expect("UHCI RX configuration rejected");
        let (uhci_rx, uhci_tx) = uhci.into_async().split();
        spawner.spawn(uart_rx_dma_task(uhci_rx, uart_producer).expect("uart rx dma task"));
        // Keep the async UART TX path: Bridge→robot is small command traffic. The UHCI TX
        // channel is unused and drops with the rest of `uhci_tx`.
        uhci_tx.uart_tx
    } else {
        info!("uart rx: async FIFO reader");
        let uart = Uart::new(peripherals.UART1, uart_config)
            .expect("UART configuration rejected")
            .with_rx(peripherals.GPIO7)
            .with_tx(peripherals.GPIO6)
            .into_async();
        let (uart_rx, uart_tx) = uart.split();
        spawner.spawn(uart_rx_task(uart_rx, uart_producer).expect("uart rx task"));
        uart_tx
    };

    spawner.spawn(uart_tx_task(uart_tx).expect("uart tx task"));
    spawner.spawn(uart_verify_task(uart_consumer).expect("uart verify task"));

    // -- TCP echo ----------------------------------------------------------------------------

    spawner.spawn(tcp_echo_task(station_stack, "sta").expect("echo task pool"));
    spawner.spawn(tcp_echo_task(access_point_stack, "ap").expect("echo task pool"));

    // -- Station association -----------------------------------------------------------------

    connect_station(&mut controller).await;
    let station_has_address = station_stack.is_config_up()
        || with_timeout(Duration::from_secs(20), station_stack.wait_config_up())
            .await
            .is_ok();
    if station_has_address {
        log_station_address(&station_stack);
    } else {
        info!("sta: DHCP did not complete; the soak can still run over the setup AP");
    }

    // -- Soak --------------------------------------------------------------------------------

    let started = Instant::now();
    let mut next_status = started + STATUS_PERIOD;
    let mut next_reconnect = started + SOAK_BEFORE_RECONNECT;
    let mut last_uart_rx = 0;
    let mut last_tcp_rx = 0;
    let mut last_status_at = started;

    loop {
        Timer::after(Duration::from_millis(500)).await;
        let now = Instant::now();

        if now >= next_status {
            let uart_tx = UART_TX_BYTES.load(Ordering::Relaxed);
            let uart_rx = UART_RX_BYTES.load(Ordering::Relaxed);
            let tcp_rx = TCP_RX_BYTES.load(Ordering::Relaxed);
            let tcp_tx = TCP_TX_BYTES.load(Ordering::Relaxed);

            let rssi = controller.rssi().unwrap_or_default();
            // A forced reconnect can stretch the gap between status lines. Rate must use the
            // actual interval, not the nominal period, or the reconnect sample reads high.
            let interval = now.duration_since(last_status_at);

            info!(
                "status: t={}s rssi={}dBm uart_tx={}B uart_rx={}B uart_kbps={} uart_good={} uart_bad={} uart_rxerr={} rxovf={} rxfrm={} rxglitch={} rxpar={} rxother={} dma_stalls={} uart_drop={}B qhw={}B tcp_kbps={} tcp_rx={}B tcp_tx={}B reconnects={}",
                now.duration_since(started).as_secs(),
                rssi,
                uart_tx,
                uart_rx,
                kbps(uart_rx - last_uart_rx, interval),
                UART_GOOD_FRAMES.load(Ordering::Relaxed),
                UART_BAD_FRAMES.load(Ordering::Relaxed),
                UART_RX_ERRORS.load(Ordering::Relaxed),
                UART_RX_FIFO_OVERFLOWED.load(Ordering::Relaxed),
                UART_RX_FRAME_VIOLATED.load(Ordering::Relaxed),
                UART_RX_GLITCHED.load(Ordering::Relaxed),
                UART_RX_PARITY_MISMATCH.load(Ordering::Relaxed),
                UART_RX_OTHER_ERROR.load(Ordering::Relaxed),
                UART_DMA_STALLS.load(Ordering::Relaxed),
                UART_DROPPED_BYTES.load(Ordering::Relaxed),
                UART_QUEUE_HIGH_WATER.load(Ordering::Relaxed),
                kbps(tcp_rx - last_tcp_rx, interval),
                tcp_rx,
                tcp_tx,
                RECONNECTS.load(Ordering::Relaxed),
            );

            last_uart_rx = uart_rx;
            last_tcp_rx = tcp_rx;
            last_status_at = now;
            next_status = now + STATUS_PERIOD;
        }

        if now >= next_reconnect {
            RECONNECTS.fetch_add(1, Ordering::Relaxed);
            info!("forcing station disconnect/reconnect; UART and AP traffic continue");
            if with_timeout(Duration::from_secs(10), controller.disconnect_async())
                .await
                .is_err()
            {
                info!("station disconnect timed out");
            }
            Timer::after(Duration::from_secs(3)).await;
            connect_station(&mut controller).await;
            if with_timeout(Duration::from_secs(20), station_stack.wait_config_up())
                .await
                .is_ok()
            {
                log_station_address(&station_stack);
            } else {
                info!("station DHCP did not recover after reconnect");
            }
            next_reconnect = Instant::now() + RECONNECT_PERIOD;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Wi-Fi helpers
// ---------------------------------------------------------------------------------------------

/// Associates the station, retrying once if the controller reports failure.
async fn connect_station(controller: &mut WifiController<'_>) {
    match with_timeout(CONNECT_TIMEOUT, controller.connect_async()).await {
        Ok(Ok(info)) => info!("sta: associated ssid={} channel={}", info.ssid, info.channel),
        Ok(Err(error)) => info!("sta: association failed: {:?}", error),
        Err(_) => info!("sta: association timed out"),
    }
}

/// Logs the station's DHCP address, if it has one.
fn log_station_address(stack: &Stack<'_>) {
    if let Some(config) = stack.config_v4() {
        let address = config.address.address();
        let [a, b, c, d] = address.octets();
        info!("sta: address={}.{}.{}.{}", a, b, c, d);
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

#[embassy_executor::task(pool_size = 2)]
async fn net_task(mut runner: Runner<'static, Interface>, label: &'static str) {
    info!("net[{}]: runner started", label);
    runner.run().await
}

/// A TCP echo server. The spike's client sends frames and expects them back byte-for-byte.
#[embassy_executor::task(pool_size = 2)]
async fn tcp_echo_task(stack: Stack<'static>, label: &'static str) {
    let mut rx_buffer = alloc::vec![0u8; 2048];
    let mut tx_buffer = alloc::vec![0u8; 2048];

    loop {
        let mut socket = embassy_net::tcp::TcpSocket::new(stack, &mut rx_buffer[..], &mut tx_buffer[..]);
        // A dead station link must free the socket quickly enough for the client's reconnect.
        socket.set_timeout(Some(Duration::from_secs(10)));

        match socket.accept(ECHO_PORT).await {
            Ok(()) => info!("tcp[{}]: client accepted", label),
            Err(_) => {
                Timer::after(Duration::from_secs(1)).await;
                continue;
            }
        }

        let mut buffer = alloc::vec![0u8; 512];
        loop {
            match socket.read(&mut buffer).await {
                Ok(0) => break,
                Ok(read) => {
                    TCP_RX_BYTES.fetch_add(read as u32, Ordering::Relaxed);

                    let mut written = 0;
                    while written < read {
                        match socket.write(&buffer[written..read]).await {
                            Ok(0) => break,
                            Ok(count) => {
                                written += count;
                                TCP_TX_BYTES.fetch_add(count as u32, Ordering::Relaxed);
                            }
                            Err(_) => break,
                        }
                    }
                }
                Err(_) => break,
            }
        }

        info!("tcp[{}]: client disconnected", label);
        socket.close();
        Timer::after(Duration::from_millis(100)).await;
    }
}

// ---------------------------------------------------------------------------------------------
// UART tasks
// ---------------------------------------------------------------------------------------------

/// Writes a known frame pattern as fast as the UART accepts it, optionally rate-limited.
#[allow(
    clippy::absurd_extreme_comparisons,
    reason = "the interval is a build-time env override and folds to zero in a default build"
)]
#[embassy_executor::task]
async fn uart_tx_task(mut tx: UartTx<'static, Async>) {
    let mut frame = [0u8; FRAME_SIZE];
    let mut sequence = 0u32;

    loop {
        fill_frame(&mut frame, sequence);
        match write_all(&mut tx, &frame).await {
            Ok(()) => {
                UART_TX_BYTES.fetch_add(FRAME_SIZE as u32, Ordering::Relaxed);
                sequence = sequence.wrapping_add(1);
                if UART_FRAME_INTERVAL_MS > 0 {
                    Timer::after(Duration::from_millis(u64::from(UART_FRAME_INTERVAL_MS))).await;
                }
            }
            Err(_) => Timer::after(Duration::from_millis(10)).await,
        }
    }
}

/// Drains the UART into the software ring. A full ring drops bytes, which is what the Bridge
/// will do when the network side is slower than the robot.
#[embassy_executor::task]
async fn uart_rx_task(mut rx: UartRx<'static, Async>, mut producer: Producer<'static, u8>) {
    let mut buffer = alloc::vec![0u8; UART_READ_CHUNK];

    loop {
        match rx.read_async(&mut buffer).await {
            Ok(read) if read > 0 => {
                UART_RX_BYTES.fetch_add(read as u32, Ordering::Relaxed);

                let free = producer.capacity() - producer.len();
                let keep = read.min(free);
                for &byte in &buffer[..keep] {
                    // The length was checked above; a failure here would only mean the consumer
                    // raced us, which it cannot: this is the ring's single producer.
                    let _ = producer.enqueue(byte);
                }
                if keep < read {
                    UART_DROPPED_BYTES.fetch_add((read - keep) as u32, Ordering::Relaxed);
                }

                let queued = producer.len() as u32;
                let high_water = UART_QUEUE_HIGH_WATER.load(Ordering::Relaxed);
                if queued > high_water {
                    UART_QUEUE_HIGH_WATER.store(queued, Ordering::Relaxed);
                }
            }
            Ok(_) => {}
            Err(error) => {
                UART_RX_ERRORS.fetch_add(1, Ordering::Relaxed);
                match error {
                    RxError::FifoOverflowed => {
                        UART_RX_FIFO_OVERFLOWED.fetch_add(1, Ordering::Relaxed);
                    }
                    RxError::FrameFormatViolated => {
                        UART_RX_FRAME_VIOLATED.fetch_add(1, Ordering::Relaxed);
                    }
                    RxError::GlitchOccurred => {
                        UART_RX_GLITCHED.fetch_add(1, Ordering::Relaxed);
                    }
                    RxError::ParityMismatch => {
                        UART_RX_PARITY_MISMATCH.fetch_add(1, Ordering::Relaxed);
                    }
                    _ => {
                        UART_RX_OTHER_ERROR.fetch_add(1, Ordering::Relaxed);
                    }
                }
                if !UART_RX_FIRST_ERROR_LOGGED.swap(true, Ordering::Relaxed) {
                    info!("uart: first RX error: {:?}", error);
                }
                // Do not sleep a whole millisecond here: at 921600 that is ~92 bytes, most
                // of the FIFO. Let the next read wait for the new threshold instead.
                Timer::after(Duration::from_micros(100)).await;
            }
        }
    }
}

/// Drains the UHCI RX stream buffer into the software ring. The DMA engine owns the UART FIFO,
/// so missing the 1.4 ms FIFO deadline under Wi-Fi load is no longer fatal; the ring absorbs the
/// descriptor-copy jitter.
#[allow(
    clippy::large_stack_frames,
    reason = "the task future owns the UHCI transfer and stream buffer; ~1.1 KiB is fine"
)]
#[embassy_executor::task]
async fn uart_rx_dma_task(mut rx: UhciRx<'static, Async>, mut producer: Producer<'static, u8>) {
    let mut buffer = dma_rx_stream_buffer!(DMA_RX_BUFFER, DMA_RX_CHUNK);

    let mut transfer = loop {
        match rx.read(buffer) {
            Ok(transfer) => break transfer,
            Err((error, returned_rx, returned_buffer)) => {
                info!("uart dma: start failed: {:?}", error);
                rx = returned_rx;
                buffer = returned_buffer;
                Timer::after(Duration::from_secs(1)).await;
            }
        }
    };

    let mut stalled = false;

    loop {
        // `is_done` becomes true only when the descriptor ring is exhausted and DMA has
        // stopped; normal flow keeps recycling descriptors and stays false.
        if transfer.is_done() {
            if !stalled {
                UART_DMA_STALLS.fetch_add(1, Ordering::Relaxed);
                info!("uart dma: descriptor ring exhausted; DMA stopped");
                stalled = true;
            }
        } else {
            stalled = false;
        }

        if transfer.available_bytes() == 0 {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        }

        let (total, kept) = {
            let data = transfer.peek();
            let total = data.len();
            if total == 0 {
                (0, 0)
            } else {
                let free = producer.capacity() - producer.len();
                let kept = total.min(free);
                for &byte in &data[..kept] {
                    let _ = producer.enqueue(byte);
                }
                (total, kept)
            }
        };

        if total == 0 {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        }

        let consumed = transfer.consume(total);
        UART_RX_BYTES.fetch_add(consumed as u32, Ordering::Relaxed);
        if kept < total {
            UART_DROPPED_BYTES.fetch_add((total - kept) as u32, Ordering::Relaxed);
        }

        let queued = producer.len() as u32;
        let high_water = UART_QUEUE_HIGH_WATER.load(Ordering::Relaxed);
        if queued > high_water {
            UART_QUEUE_HIGH_WATER.store(queued, Ordering::Relaxed);
        }
    }
}

/// Consumes the ring and validates the frame stream, resynchronising on the magic after any
/// drop so one glitch does not poison the rest of the run.
#[embassy_executor::task]
async fn uart_verify_task(mut consumer: Consumer<'static, u8>) {
    let mut buffer = [0u8; FRAME_SIZE];
    let mut length = 0usize;

    loop {
        let Some(byte) = consumer.dequeue() else {
            Timer::after(Duration::from_millis(1)).await;
            continue;
        };

        buffer[length] = byte;
        length += 1;

        loop {
            if length < FRAME_MAGIC.len() {
                break;
            }
            if buffer[..FRAME_MAGIC.len()] != FRAME_MAGIC {
                buffer.copy_within(1..length, 0);
                length -= 1;
                continue;
            }
            if length < FRAME_SIZE {
                break;
            }

            let sequence = u32::from_le_bytes([buffer[2], buffer[3], buffer[4], buffer[5]]);
            let mut expected = [0u8; FRAME_SIZE];
            fill_frame(&mut expected, sequence);

            if buffer[..FRAME_SIZE] == expected {
                UART_GOOD_FRAMES.fetch_add(1, Ordering::Relaxed);
                length = 0;
            } else {
                UART_BAD_FRAMES.fetch_add(1, Ordering::Relaxed);
                buffer.copy_within(1..length, 0);
                length -= 1;
            }
        }
    }
}

/// Fills one loopback frame: magic, little-endian sequence, then an LCG pattern seeded by the
/// sequence so the verifier can regenerate it.
fn fill_frame(frame: &mut [u8; FRAME_SIZE], sequence: u32) {
    frame[0] = FRAME_MAGIC[0];
    frame[1] = FRAME_MAGIC[1];
    frame[2..6].copy_from_slice(&sequence.to_le_bytes());

    let mut state = sequence ^ 0xDEAD_BEEF;
    for byte in &mut frame[6..] {
        state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        *byte = (state >> 24) as u8;
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
