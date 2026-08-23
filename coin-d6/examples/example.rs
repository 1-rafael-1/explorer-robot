//! COIN-D6 power-cycle demo.
//!
//! Drives the physical COIN-D6 through one full power cycle on an RP2350: power
//! on, start, capture a single revolution, aggregate five revolutions, stop,
//! power off, and release the driver. Summary statistics are logged over RTT via
//! defmt rather than dumping every point.
//!
//! The example only has a real body on the bare-metal target (where the embassy
//! and defmt dev-dependencies are available); on other targets it compiles to a
//! no-op `main` so host `cargo test` — which builds examples to check they
//! compile — stays green without pulling in the ARM-only crates.

#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_std)]
#![cfg_attr(all(target_arch = "arm", target_os = "none"), no_main)]
// Demo code: `.unwrap()` on driver/GPIO results, the `f32` frequency cast
// (`u64` milliseconds → `f32`), and the intentionally stack-resident scan/ingest
// buffers (which make the `run` future large) are all intentional.
#![allow(
    clippy::unwrap_used,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_possible_wrap,
    clippy::cast_precision_loss,
    clippy::used_underscore_binding,
    clippy::large_futures
)]

#[cfg(all(target_arch = "arm", target_os = "none"))]
use defmt_rtt as _;
#[cfg(all(target_arch = "arm", target_os = "none"))]
use embassy_executor::Spawner;
#[cfg(all(target_arch = "arm", target_os = "none"))]
use panic_probe as _;

/// The RP2350 implementation: everything that needs embassy/defmt lives here so
/// it can be gated behind the bare-metal target.
#[cfg(all(target_arch = "arm", target_os = "none"))]
mod imp {
    use core::num::NonZeroU16;

    use coin_d6::{AggregationConfig, CoinD6, Config, Point, Scan, aggregate};
    use defmt::{info, warn};
    use embassy_rp::{
        bind_interrupts,
        gpio::{Level, Output},
        peripherals::UART0,
        uart::{self, BufferedUart},
    };
    use embassy_time::{Duration, Instant};
    use static_cell::StaticCell;

    bind_interrupts!(struct Irqs {
        UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    });

    /// COIN-D6 UART baud rate (230400 8N1).
    const BAUD_RATE: u32 = 230_400;
    /// Ingest buffer size; large enough for the largest possible packet.
    const INGEST_BUF_LEN: usize = 1024;
    /// Number of revolutions to aggregate into a stable scan.
    const SPINS: usize = 5;
    /// Upper bound on how many revolutions to discard while the rotor spins up.
    const WARMUP_MAX_SPINS: usize = 50;
    /// Points per revolution the COIN-D6 emits at steady state (native 0.9°).
    const NATIVE_POINTS: usize = 400;
    /// A warm-up spin counts as "settled" when within this many points of
    /// `NATIVE_POINTS`.
    const SETTLE_TOLERANCE: usize = 3;
    /// Consecutive spins needed to declare the rotor settled (stays within the
    /// settle band) or plateaued (fails to set a new point-count high).
    const SETTLE_STABLE_SPINS: usize = 12;

    /// `BufferedUart` RX ring buffer size. Large enough to absorb the RTT
    /// logging pauses in this demo (~178 ms at 230400 baud) so the sensor's
    /// stream does not overrun while the task logs a full scan.
    const RX_BUF_LEN: usize = 4096;
    /// `BufferedUart` TX ring buffer size (start/stop commands are 4 bytes).
    const TX_BUF_LEN: usize = 16;

    /// Static TX ring buffer for the buffered UART.
    static TX_BUF: StaticCell<[u8; TX_BUF_LEN]> = StaticCell::new();
    /// Static RX ring buffer for the buffered UART.
    static RX_BUF: StaticCell<[u8; RX_BUF_LEN]> = StaticCell::new();

    /// The distance for display: `Some(mm)` for a return, `None` for no return.
    fn display_distance(point: &Point) -> Option<u16> {
        point.distance_mm.map(NonZeroU16::get)
    }

    /// Log summary statistics for a scan without dumping every point.
    ///
    /// `spins` is the number of revolutions `elapsed` covers, so the reported
    /// frequency is revolutions per second rather than per elapsed interval.
    fn summarize(label: &str, scan: &Scan, elapsed: Duration, spins: usize) {
        let mut min = u16::MAX;
        let mut max = 0u16;
        for point in &scan.points[..scan.len] {
            if let Some(distance) = point.distance_mm {
                min = min.min(distance.get());
                max = max.max(distance.get());
            }
        }
        // No valid returns in this scan: report an empty range.
        if min == u16::MAX {
            min = 0;
        }

        let elapsed_ms = elapsed.as_millis() as f32;
        let frequency_hz = if elapsed_ms > 0.0 {
            (spins as f32) * 1000.0 / elapsed_ms
        } else {
            0.0
        };

        info!(
            "[{}] points={} frequency={} Hz min={} mm max={} mm",
            label, scan.len, frequency_hz, min, max
        );

        if scan.len > 0 {
            let first = scan.points[0];
            let middle = scan.points[scan.len / 2];
            let last = scan.points[scan.len - 1];
            info!(
                "[{}] samples first=({},{},{}) middle=({},{},{}) last=({},{},{})",
                label,
                first.angle_deg,
                display_distance(&first),
                first.intensity,
                middle.angle_deg,
                display_distance(&middle),
                middle.intensity,
                last.angle_deg,
                display_distance(&last),
                last.intensity
            );
        }
    }

    /// Dump every point of a scan so the physical layout can be eyeballed
    /// against the room.
    fn dump_scan(label: &str, scan: &Scan) {
        info!("[{}] full point list ({} points):", label, scan.len);
        for point in &scan.points[..scan.len] {
            info!(
                "    angle={} dist_mm={} intensity={}",
                point.angle_deg,
                display_distance(point),
                point.intensity
            );
        }
    }

    /// The ring-start bearing of a scan: the first point's angle.
    const fn ring_start_deg(scan: &Scan) -> f32 {
        if scan.len > 0 { scan.points[0].angle_deg } else { 0.0 }
    }

    /// Log a single revolution's point count and ring-start bearing.
    fn log_spin(label: &str, index: usize, scan: &Scan) {
        info!(
            "[{} {}] points={} ring_start={}",
            label,
            index,
            scan.len,
            ring_start_deg(scan)
        );
    }

    /// Drive the COIN-D6 through one full power cycle.
    pub async fn run() {
        let p = embassy_rp::init(embassy_rp::config::Config::default());
        info!("coin-d6 power-cycle demo");

        // Active-high power-enable pin (low-side IRLZ44N), starting off.
        let power = Output::new(p.PIN_15, Level::Low);

        // UART0 with GPIO12 = TX and GPIO13 = RX.
        let mut uart_config = uart::Config::default();
        uart_config.baudrate = BAUD_RATE;
        let uart = BufferedUart::new(
            p.UART0,
            p.PIN_12,
            p.PIN_13,
            Irqs,
            TX_BUF.init([0; TX_BUF_LEN]),
            RX_BUF.init([0; RX_BUF_LEN]),
            uart_config,
        );

        let mut ingest = [0u8; INGEST_BUF_LEN];
        let mut driver = CoinD6::new(uart, power, &mut ingest, Config::default());

        driver.power_on().await.unwrap();
        info!("powered on");

        driver.start().await.unwrap();
        info!("started");

        // Warm up: discard revolutions until the rotor reaches steady state.
        // The point count proxies rotor speed — it starts below 400 and climbs
        // as the rotor accelerates. A slow ramp can look "flat" (near-equal
        // counts) while still far from the target, so we judge against the
        // native count rather than the previous spin, and also stop early if the
        // count plateaus or the spin budget runs out.
        let mut warmup = Scan::new();
        let mut spin = 0usize;
        // Consecutive spins whose point count has stayed within the settle band.
        let mut in_band = 0usize;
        // Highest point count seen so far.
        let mut ceiling = 0usize;
        // Consecutive spins that have failed to set a new point-count high.
        let mut flat = 0usize;
        // Set when the loop stops by settling or plateauing (not the budget).
        let mut stopped_early = false;

        while spin < WARMUP_MAX_SPINS {
            driver.read_scan(&mut warmup).await.unwrap();
            log_spin("warmup", spin, &warmup);
            spin += 1;

            let points = warmup.len;

            if points.abs_diff(NATIVE_POINTS) <= SETTLE_TOLERANCE {
                in_band += 1;
            } else {
                in_band = 0;
            }

            if points > ceiling {
                ceiling = points;
                flat = 0;
            } else {
                flat += 1;
            }

            if in_band >= SETTLE_STABLE_SPINS {
                info!("speed settled after {} warmup spins ({} points)", spin, points);
                stopped_early = true;
                break;
            }
            // A plateau is only a give-up when it sits below the settle band:
            // an overshoot above the band (e.g. 408) is a transient that
            // descends back to 400, so keep waiting for `in_band` to fill.
            if flat >= SETTLE_STABLE_SPINS && ceiling < NATIVE_POINTS - SETTLE_TOLERANCE {
                warn!(
                    "warm-up plateaued at {} points after {} spins (target {} +/- {})",
                    ceiling, spin, NATIVE_POINTS, SETTLE_TOLERANCE
                );
                stopped_early = true;
                break;
            }
        }

        if !stopped_early {
            warn!(
                "warm-up did not settle after {} spins (target {} +/- {})",
                WARMUP_MAX_SPINS, NATIVE_POINTS, SETTLE_TOLERANCE
            );
        }

        // Capture and reduce five revolutions by angle.
        let mut spins: [Scan; SPINS] = core::array::from_fn(|_| Scan::new());
        let start = Instant::now();
        for (index, scan) in spins.iter_mut().enumerate() {
            driver.read_scan(scan).await.unwrap();
            log_spin("spin", index, scan);
        }
        let out = aggregate(&spins, &AggregationConfig::default());

        // Stop the stream before the (slow) RTT logging below, so the sensor
        // isn't still streaming while the task logs the scan.
        driver.stop().await.unwrap();
        info!("stopped");

        summarize("aggregated", &out, start.elapsed(), SPINS);
        dump_scan("aggregated", &out);

        driver.power_off().await.unwrap();
        info!("powered off");

        let (_uart, _power) = driver.release();
        info!("cycle complete");
    }
}

/// The embassy entry point: run one full power cycle, then exit the executor.
#[cfg(all(target_arch = "arm", target_os = "none"))]
#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    imp::run().await;
}

/// Host fallback: the COIN-D6 demo only runs on the RP2350 bare-metal target.
#[cfg(not(all(target_arch = "arm", target_os = "none")))]
fn main() {}
