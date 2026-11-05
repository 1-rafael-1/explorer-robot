#![no_std]
#![no_main]
#![deny(
    clippy::mem_forget,
    reason = "mem::forget is generally not safe to do with esp_hal types, especially those \
    holding buffers for the duration of a data transfer."
)]
#![deny(clippy::large_stack_frames)]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Instant, Timer};
use esp_backtrace as _;
use esp_hal::clock::CpuClock;
use esp_hal::timer::timg::TimerGroup;
use esp_println as _;
use esp_radio::wifi::scan::ScanConfig;

extern crate alloc;

// This creates a default app-descriptor required by the esp-idf bootloader.
// For more information see: <https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/system/app_image_format.html#application-description>
esp_bootloader_esp_idf::esp_app_desc!();

/// How long to wait between scans.
const SCAN_INTERVAL: Duration = Duration::from_secs(5);

/// How many access points to keep per scan.
const MAX_APS: usize = 12;

/// Restrict the scan to one channel, or `None` to scan every channel.
///
/// A single-channel scan is the controlled antenna measurement: set this to the channel of the
/// AP you are comparing (11 for the mesh node seen at -42 dBm) so every scan measures the same
/// thing instead of whatever the band happened to return.
const SCAN_CHANNEL: Option<u8> = None;

#[allow(
    clippy::large_stack_frames,
    reason = "it's not unusual to allocate larger buffers etc. in main"
)]
#[esp_rtos::main]
async fn main(spawner: Spawner) -> ! {
    // generator version: 2.0.0
    // generator parameters: -o esp32c3 -o esp32c3-mini-1 -o unstable-hal -o alloc -o embassy -o wifi -o defmt -o esp-backtrace -o zed -o agents

    let config = esp_hal::Config::default().with_cpu_clock(CpuClock::max());
    let peripherals = esp_hal::init(config);

    // The following pins are used to bootstrap the chip. They are available
    // for use, but check the datasheet of the module for more information on them.
    // - GPIO2
    // - GPIO8
    // - GPIO9
    // These GPIO pins are in use by some feature of the module and should not be used.
    let _gpio11 = peripherals.GPIO11;
    let _gpio12 = peripherals.GPIO12;
    let _gpio13 = peripherals.GPIO13;
    let _gpio14 = peripherals.GPIO14;
    let _gpio15 = peripherals.GPIO15;
    let _gpio16 = peripherals.GPIO16;
    let _gpio17 = peripherals.GPIO17;

    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 66320);

    let timg0 = TimerGroup::new(peripherals.TIMG0);
    esp_rtos::start(timg0.timer0, peripherals.FROM_CPU_INTR0);

    info!("Embassy initialized!");

    let mut wifi_controller = esp_radio::wifi::WifiController::new(peripherals.WIFI, Default::default())
        .expect("Failed to initialize Wi-Fi controller");
    let _wifi_interface = esp_radio::wifi::Interface::station();

    // Antenna check: a credential-free scan, strongest AP first.
    //
    // Run it with the external antenna attached, then detached, and compare the RSSI of
    // the same AP. If it barely moves, the board is still feeding the PCB antenna and the
    // RF selector has to be moved. If it swings 10-20 dB, the external path is live.
    let scan_config = {
        let config = ScanConfig::default().with_max(MAX_APS);
        match SCAN_CHANNEL {
            Some(channel) => config.with_channel(channel),
            None => config,
        }
    };

    let _ = spawner;

    loop {
        let started = Instant::now();
        match wifi_controller.scan_async(&scan_config).await {
            Ok(mut aps) => {
                let elapsed_ms = started.elapsed().as_millis();
                aps.sort_by_key(|ap| core::cmp::Reverse(ap.signal_strength));
                // A fresh full-band sweep takes a few hundred milliseconds (13 channels at the
                // default 10-20 ms dwell). A return in single-digit milliseconds means the
                // result did not come from a new sweep, so treat those rows as suspect.
                info!("scan: {} APs in {} ms", aps.len(), elapsed_ms);
                for ap in aps {
                    info!(
                        "  {} bssid={} ch={} rssi={} dBm",
                        ap.ssid,
                        defmt::Debug2Format(&ap.bssid),
                        ap.channel,
                        ap.signal_strength
                    );
                }
            }
            Err(err) => info!("scan failed: {:?}", err),
        }

        Timer::after(SCAN_INTERVAL).await;
    }

    // for inspiration have a look at the examples at https://github.com/esp-rs/esp-hal/tree/esp-hal-v1.2.2/examples
}
