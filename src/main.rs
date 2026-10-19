//! explorer-robot v3 firmware entry point
//!
//! Core0 hosts orchestrator, motor driver, encoders, battery monitor,
//! RGB LED, panel (TFT + touch), VL53L0X stub, flash storage, UI, testmode,
//! autonomous mode controller, and startup.
//!
//! Core1 hosts the real COIN-D6 `LiDAR` driver task, powered on demand.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::{Executor, Spawner};
use embassy_rp::{
    adc::{Adc, Channel, Config as AdcConfig, InterruptHandler as AdcInterruptHandler},
    bind_interrupts,
    block::ImageDef,
    config::Config,
    dma::InterruptHandler as DmaInterruptHandler,
    flash::{Async, Flash},
    gpio::{Level, Output, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    multicore::{Stack, spawn_core1},
    peripherals::{
        ADC, DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, DMA_CH5, DMA_CH6, DMA_CH7, FLASH, I2C0, I2C1, PIN_16, PIN_17,
        PIN_18, PIN_19, PIN_20, PIN_21, PIN_26, PIN_27, PIN_40, PIO0, PIO1, SPI0, UART0, UART1,
    },
    pio::{InterruptHandler as PioInterruptHandler, Pio},
    pio_programs::{
        pwm::{PioPwm, PioPwmProgram},
        uart::{PioUartRx, PioUartRxProgram, PioUartTx, PioUartTxProgram},
    },
    pwm::{Config as PwmConfig, InputMode, Pwm},
    spi::{self, Spi},
    uart::{self, InterruptHandler as UartInterruptHandler},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

use crate::resources::{
    AiCamResources, AssignedResources, BatteryResources, CoreResources, D6LidarResources, ExpansionResources,
    ImuResources, MotorDriverResources, PanelResources, Rangefinder0Resources, Rangefinder1Resources, RgbLedResources,
    StorageResources,
};

mod resources;
mod system;
mod task;

// ── Crate-level type alias ────────────────────────────────────────────────────

/// Shared I2C0 bus protected by a critical-section mutex.
///
/// Reserved for the first VL53L0X rangefinder on core0.
pub type I2cBusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;

/// Shared I2C1 bus protected by a critical-section mutex.
///
/// Reserved for the second VL53L0X rangefinder on core0. Keeping the two
/// sensors on separate buses avoids an address clash when both use the default
/// `0x29` address.
pub type I2c1BusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C1, embassy_rp::i2c::Async>>;

// ── Interrupt bindings ─────────────────────────────────────────────────────────

bind_interrupts!(pub struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    I2C1_IRQ => I2cInterruptHandler<I2C1>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>, DmaInterruptHandler<DMA_CH1>, DmaInterruptHandler<DMA_CH2>,
        DmaInterruptHandler<DMA_CH3>, DmaInterruptHandler<DMA_CH4>, DmaInterruptHandler<DMA_CH5>,
        DmaInterruptHandler<DMA_CH6>, DmaInterruptHandler<DMA_CH7>;
    UART0_IRQ => uart::BufferedInterruptHandler<UART0>;
    UART1_IRQ => UartInterruptHandler<UART1>;
});

// ── Boot descriptor ────────────────────────────────────────────────────────────

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// ── Core stacks & executors ────────────────────────────────────────────────────

/// Core1 stack — 16 KiB for the real COIN-D6 `LiDAR` driver task.
///
/// Budget: ~1.5 KiB point cloud + ~8 KiB UART/DMA buffers + ~4 KiB
/// frame-parser state + ~2 KiB embassy async overhead.
static mut CORE1_STACK: Stack<16384> = Stack::new();

/// Executor for core0 (orchestrator, drive, UI, sensors on I2C).
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();

/// Executor for core1 (`LiDAR` driver).
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// ── Shared bus helpers ─────────────────────────────────────────────────────────

/// Initialise the shared I2C0 bus for the first VL53L0X rangefinder.
///
/// Must be called on core0 so that `I2c::new_async` enables `I2C0_IRQ` on
/// core0's NVIC. Returns a `'static` reference for sharing across tasks.
fn init_i2c_bus(
    i2c0: embassy_rp::Peri<'static, I2C0>,
    sda: embassy_rp::Peri<'static, PIN_16>,
    scl: embassy_rp::Peri<'static, PIN_17>,
) -> &'static I2cBusShared {
    static I2C_BUS: StaticCell<I2cBusShared> = StaticCell::new();

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;

    let i2c = I2c::new_async(i2c0, scl, sda, Irqs, i2c_config);
    I2C_BUS.init(Mutex::new(i2c))
}

/// Initialise the shared I2C1 bus for the second VL53L0X rangefinder.
///
/// Must be called on core0 so that `I2c::new_async` enables `I2C1_IRQ` on
/// core0's NVIC. The two sensors may share the same 7-bit address because they
/// are on electrically separate buses.
fn init_i2c1_bus(
    i2c1: embassy_rp::Peri<'static, I2C1>,
    sda: embassy_rp::Peri<'static, PIN_26>,
    scl: embassy_rp::Peri<'static, PIN_27>,
) -> &'static I2c1BusShared {
    static I2C1_BUS: StaticCell<I2c1BusShared> = StaticCell::new();

    let mut i2c_config = I2cConfig::default();
    i2c_config.frequency = 400_000;

    let i2c = I2c::new_async(i2c1, scl, sda, Irqs, i2c_config);
    I2C1_BUS.init(Mutex::new(i2c))
}

/// Motor PWM configuration: 20 kHz, the TB6612FNG's sweet spot.
fn motor_pwm_config() -> PwmConfig {
    let desired_freq_hz = 20_000u32;
    let clock_freq_hz = embassy_rp::clocks::clk_sys_freq();
    #[allow(clippy::cast_possible_truncation)]
    let divider = ((clock_freq_hz / desired_freq_hz) / 65535 + 1) as u8;
    #[allow(clippy::cast_possible_truncation)]
    let period = (clock_freq_hz / (desired_freq_hz * u32::from(divider))) as u16 - 1;

    let mut cfg = PwmConfig::default();
    cfg.divider = divider.into();
    cfg.top = period;
    cfg
}

/// Encoder PWM input configuration (no divider, no phase-correct).
fn encoder_pwm_config() -> PwmConfig {
    let mut cfg = PwmConfig::default();
    cfg.divider = 1.into();
    cfg.phase_correct = false;
    cfg
}

// ── Init functions (one per subsystem) ────────────────────────────────────────

/// Spawn the orchestrator event loop.
#[allow(clippy::unwrap_used)]
fn init_orchestrate(spawner: Spawner) {
    spawner.spawn(task::orchestrate::orchestrate().unwrap());
}

/// Set up ADC-based battery voltage monitoring.
#[allow(clippy::unwrap_used)]
fn init_battery_monitoring(
    spawner: Spawner,
    adc: embassy_rp::Peri<'static, ADC>,
    // RP2350B ADC pin (ADC0). Provisional: confirm against the board's battery
    // voltage-divider wiring before first flash.
    adc_pin: embassy_rp::Peri<'static, PIN_40>,
) {
    let adc = Adc::new(adc, Irqs, AdcConfig::default());
    let battery_channel = Channel::new_pin(adc_pin, Pull::None);
    spawner.spawn(task::battery_charge_read::battery_charge_read(adc, battery_channel).unwrap());
}

/// Set up PIO-driven RGB LED (PWM on SM 0–2).
#[allow(clippy::unwrap_used)]
fn init_rgb_led(spawner: Spawner, rgb: RgbLedResources) {
    let Pio {
        common: mut pio_common,
        sm0,
        sm1,
        sm2,
        ..
    } = Pio::new(rgb.pio, Irqs);

    let rgb_program = PioPwmProgram::new(&mut pio_common);
    let pwm_red = PioPwm::new(&mut pio_common, sm0, rgb.red, &rgb_program);
    let pwm_green = PioPwm::new(&mut pio_common, sm1, rgb.green, &rgb_program);
    let pwm_blue = PioPwm::new(&mut pio_common, sm2, rgb.blue, &rgb_program);
    spawner.spawn(task::indicators::rgb_led_indicate::rgb_led_indicate(pwm_red, pwm_green, pwm_blue).unwrap());
}

/// Set up the TB6612FNG motor driver and encoder reader.
///
/// Spawns the motor driver task and the encoder reader task so that both
/// are running before any drive commands arrive.
#[allow(clippy::unwrap_used)]
fn init_motor_driver(spawner: Spawner, motor_pins: MotorDriverResources) {
    let pwm_cfg = motor_pwm_config();
    let pwm_left = Pwm::new_output_a(motor_pins.left_pwm_slice, motor_pins.left_pwm_pin, pwm_cfg.clone());
    let pwm_right = Pwm::new_output_b(motor_pins.right_pwm_slice, motor_pins.right_pwm_pin, pwm_cfg);

    spawner.spawn(
        task::motor_driver::motor_driver(
            pwm_left,
            pwm_right,
            motor_pins.left_fwd,
            motor_pins.left_bwd,
            motor_pins.right_fwd,
            motor_pins.right_bwd,
            motor_pins.standby,
        )
        .unwrap(),
    );

    let enc_cfg = encoder_pwm_config();
    let enc_left = Pwm::new_input(
        motor_pins.enc_left_slice,
        motor_pins.enc_left_pin,
        Pull::None,
        InputMode::RisingEdge,
        enc_cfg.clone(),
    );
    let enc_right = Pwm::new_input(
        motor_pins.enc_right_slice,
        motor_pins.enc_right_pin,
        Pull::None,
        InputMode::RisingEdge,
        enc_cfg,
    );
    spawner.spawn(task::sensors::encoders::encoder_read(enc_left, enc_right).unwrap());

    // Drive subsystem tasks: queue executor runs per-step completions;
    // the drive task coordinates intents, sensors, and motor commands.
    spawner.spawn(task::drive::drive_queue_executor().unwrap());
    spawner.spawn(task::drive::drive().unwrap());
}

/// Spawn the VL53L0X rangefinder stub task on core0.
#[allow(clippy::unwrap_used)]
fn init_vl53l0x_stub(spawner: Spawner) {
    spawner.spawn(task::sensors::vl53l0x_stub::vl53l0x_stub_task().unwrap());
}

/// Set up flash storage for calibration data persistence.
#[allow(clippy::unwrap_used)]
fn init_flash_storage(
    spawner: Spawner,
    flash: embassy_rp::Peri<'static, FLASH>,
    dma_ch: embassy_rp::Peri<'static, DMA_CH0>,
) {
    let flash = Flash::<_, Async, { 2048 * 1024 }>::new(flash, dma_ch, Irqs);
    spawner.spawn(task::io::flash_storage::flash_storage(flash).unwrap());
}

/// Initialise the on-demand testmode controller.
fn init_testing(spawner: Spawner) {
    task::testmode::init_testing(spawner);
}

/// Initialise the touch UI controller, which owns the panel.
///
/// The bus carries both the ST7789 display and the touch controller, each with
/// its own chip select and per-device configuration (ADR-0008).
fn init_ui(spawner: Spawner, panel_pins: PanelResources) {
    task::ui::init_ui(spawner, panel_pins);
}

/// Initialise the autonomous mode controller (coast-and-avoid etc.).
fn init_autonomous_mode(spawner: Spawner) {
    task::autonomous_mode::init_autonomous_mode(spawner);
}

/// Initialise the dedicated SPI0 bus and chip-select for the IMU.
///
/// Returns the SPI bus and CS pin for passing to [`init_imu`].
fn init_imu_spi(
    spi0: embassy_rp::Peri<'static, SPI0>,
    sck: embassy_rp::Peri<'static, PIN_18>,
    mosi: embassy_rp::Peri<'static, PIN_19>,
    miso: embassy_rp::Peri<'static, PIN_20>,
    cs: embassy_rp::Peri<'static, PIN_21>,
    dma_ch1: embassy_rp::Peri<'static, DMA_CH1>,
    dma_ch2: embassy_rp::Peri<'static, DMA_CH2>,
) -> (Spi<'static, SPI0, spi::Async>, Output<'static>) {
    let mut spi_config = spi::Config::default();
    spi_config.frequency = 7_000_000;
    let imu_spi = Spi::new(spi0, sck, mosi, miso, dma_ch1, dma_ch2, Irqs, spi_config);
    let imu_cs = Output::new(cs, embassy_rp::gpio::Level::High);
    (imu_spi, imu_cs)
}

/// Initialise the IMU (ICM-20948) on the dedicated SPI0 bus.
///
/// The IMU task owns the SPI bus exclusively — the `Mutex` wrapper is a
/// formality (single user, never contended) that satisfies the `SpiDevice` API.
#[allow(clippy::unwrap_used)]
fn init_imu(spawner: Spawner, spi: Spi<'static, SPI0, spi::Async>, cs: Output<'static>) {
    static SPI_BUS: StaticCell<Mutex<CriticalSectionRawMutex, Spi<'static, SPI0, spi::Async>>> = StaticCell::new();
    let spi_bus = SPI_BUS.init(Mutex::new(spi));
    spawner.spawn(task::sensors::imu::inertial_measurement_read(spi_bus, cs).unwrap());
}

/// Build the COIN-D6 `LiDAR` session and spawn its task on core1.
///
/// The session retains UART0 and the contiguous TX/RX pair (GPIO 28 / GPIO 29),
/// with the TPS22917 high-side load-switch enable on GPIO 30 held low. No UART
/// is constructed at boot: the task builds one per powered period, after it has
/// asserted the branch and waited for the switch rise (ADR-0023).
///
/// Must be called on **core1** so `UART0_IRQ` is enabled on core1's NVIC,
/// mirroring how `init_i2c_bus` must run on core0 for `I2C0_IRQ`.
#[allow(clippy::unwrap_used)]
fn init_lidar(spawner: Spawner, d6_pins: D6LidarResources) {
    let session =
        task::sensors::lidar::LidarSession::new(d6_pins.uart, d6_pins.uart_tx, d6_pins.uart_rx, d6_pins.power_enable);

    spawner.spawn(task::sensors::lidar::lidar_task(session).unwrap());
}

/// Bring up the Grove Vision AI V2 camera's PIO0 UART and load-switch enable.
///
/// The camera uses two PIO0 state machines (TX on SM0, RX on SM1). This leaves
/// PIO1 for the RGB LED and PIO2 free, and leaves hardware UART1 free for the
/// buffered expansion link.
///
/// This only brings the transport up — no protocol implementation yet.
/// Returns the PIO UART halves and the TPS22917 enable for the real driver task
/// to consume once it exists.
fn init_ai_cam_pio(
    ai_cam_pins: AiCamResources,
) -> (
    PioUartTx<'static, PIO0, 0>,
    PioUartRx<'static, PIO0, 1>,
    Output<'static>,
) {
    let Pio {
        common: mut pio0_common,
        sm0,
        sm1,
        ..
    } = Pio::new(ai_cam_pins.pio, Irqs);

    let tx_program = PioUartTxProgram::new(&mut pio0_common);
    let rx_program = PioUartRxProgram::new(&mut pio0_common);

    let tx = PioUartTx::<PIO0, 0>::new(115_200, &mut pio0_common, sm0, ai_cam_pins.uart_tx, &tx_program);
    let rx = PioUartRx::<PIO0, 1>::new(115_200, &mut pio0_common, sm1, ai_cam_pins.uart_rx, &rx_program);

    // Powered off by default; the real driver drives this high to power the module.
    let power_enable = Output::new(ai_cam_pins.power_enable, Level::Low);

    (tx, rx, power_enable)
}

/// Spawn the startup task that fires the `Initialize` event.
#[allow(clippy::unwrap_used)]
fn init_startup(spawner: Spawner) {
    spawner.spawn(task::startup::startup().unwrap());
}

// ── Entry point ────────────────────────────────────────────────────────────────

/// Main entry point — initializes hardware, spawns all tasks, and starts the executor.
#[allow(clippy::unwrap_used)]
#[allow(clippy::too_many_lines)]
#[cortex_m_rt::entry]
fn main() -> ! {
    let mut config = Config::default();
    config.clocks = embassy_rp::clocks::ClockConfig::system_freq(150_000_000)
        .unwrap_or_else(|e| defmt::panic!("Failed to configure system clocks {}", e));
    let p = embassy_rp::init(config);

    info!("explorer-robot v3 booting...");

    // ── Board resource bundles (ADR-0022) ───────────────────────────────────
    let r = split_resources!(p);

    let rgb_pins = r.rgb;
    let motor_pins = r.motor;
    let d6_pins = r.lidar;
    let ai_cam_pins = r.ai_cam;
    let panel_pins = r.panel;
    let imu = r.imu;
    let rangefinder0 = r.rangefinder0;
    let rangefinder1 = r.rangefinder1;
    let battery = r.battery;
    let storage = r.storage;
    let core = r.core;
    // `r.expansion` (UART1, DMA_CH4/5 and GPIO 32–37, including `EXP_PWR`)
    // stays untouched until the expansion connector lands.

    // ── Core1: LiDAR driver ─────────────────────────────────────────────────
    #[allow(static_mut_refs)]
    spawn_core1(core.core1, unsafe { &mut CORE1_STACK }, move || {
        // Initialised on core1 so UART0_IRQ is enabled on core1's NVIC.
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            init_lidar(spawner, d6_pins);
        });
    });

    // ── Core0: all other tasks ──────────────────────────────────────────────
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(move |spawner| {
        // I2C0 bus reserved for the first VL53L0X rangefinder stub.
        // Initialised inside the closure so I2c::new_async enables I2C0_IRQ
        // on core0's NVIC.
        let _i2c_bus = init_i2c_bus(rangefinder0.i2c, rangefinder0.sda, rangefinder0.scl);

        // I2C1 bus reserved for a second VL53L0X on a separate bus, so both
        // sensors can keep the default 0x29 address.
        let _i2c1_bus = init_i2c1_bus(rangefinder1.i2c, rangefinder1.sda, rangefinder1.scl);

        // SPI bus for ICM20948 IMU
        let (imu_spi, imu_cs) = init_imu_spi(imu.spi, imu.sck, imu.mosi, imu.miso, imu.cs, imu.tx_dma, imu.rx_dma);

        // UART1 is left free for the expansion connector (GPIO 36/37).
        // The camera is brought up on PIO0 so it does not occupy it.
        let (_ai_cam_tx, _ai_cam_rx, _ai_cam_power) = init_ai_cam_pio(ai_cam_pins);

        init_orchestrate(spawner);
        init_battery_monitoring(spawner, battery.adc, battery.pin);
        init_rgb_led(spawner, rgb_pins);
        init_motor_driver(spawner, motor_pins);
        init_ui(spawner, panel_pins);
        init_vl53l0x_stub(spawner);
        init_imu(spawner, imu_spi, imu_cs);
        init_flash_storage(spawner, storage.flash, storage.dma);
        init_testing(spawner);
        init_autonomous_mode(spawner);
        init_startup(spawner);
    });
}
