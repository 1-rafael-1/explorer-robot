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
        ADC, DMA_CH0, DMA_CH1, DMA_CH2, DMA_CH3, DMA_CH4, DMA_CH5, DMA_CH6, DMA_CH7, FLASH, I2C0, PIN_0, PIN_1, PIN_2,
        PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_8, PIN_9, PIN_10, PIN_11, PIN_12, PIN_13, PIN_14, PIN_15, PIN_16,
        PIN_17, PIN_18, PIN_19, PIN_20, PIN_21, PIN_26, PIN_27, PIN_40, PIO1, PWM_SLICE0, PWM_SLICE1, PWM_SLICE3,
        PWM_SLICE4, SPI0, UART0, UART1,
    },
    pio::{Common, InterruptHandler as PioInterruptHandler, Pio, StateMachine},
    pio_programs::pwm::{PioPwm, PioPwmProgram},
    pwm::{Config as PwmConfig, InputMode, Pwm},
    spi::{self, Spi},
    uart::{self, BufferedUart, InterruptHandler as UartInterruptHandler, Uart},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

use crate::task::io::panel::PanelPins;

mod system;
mod task;

// ── Crate-level type alias ────────────────────────────────────────────────────

/// Shared I2C0 bus protected by a critical-section mutex.
///
/// Reserved for the VL53L0X rangefinder on core0.
pub type I2cBusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;

// ── Interrupt bindings ─────────────────────────────────────────────────────────

bind_interrupts!(pub struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
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

// ── Pin structs ────────────────────────────────────────────────────────────────

/// Pins used by the TB6612FNG motor driver and motor encoders.
///
/// Two-track configuration: left PWM on GPIO 0 (slice 0), right PWM on
/// GPIO 3 (slice 1). Direction pins are direct-GPIO outputs (IN1/IN2 per
/// channel), relocated off the RP2350's fixed UART RX pins (GPIO 1/4/5)
/// to free those for the D6 `LiDAR` and AI cam UARTs. Encoders use PWM
/// input mode on slices 3 and 4.
pub struct MotorDriverPins {
    /// Left motor PWM slice + pin.
    pub left_pwm_slice: embassy_rp::Peri<'static, PWM_SLICE0>,
    pub left_pwm_pin: embassy_rp::Peri<'static, PIN_0>,
    /// Left motor forward direction pin (GPIO 8; moved off GPIO 1 to free UART0 RX).
    pub left_fwd: embassy_rp::Peri<'static, PIN_8>,
    /// Left motor backward direction pin.
    pub left_bwd: embassy_rp::Peri<'static, PIN_2>,
    /// Right motor PWM slice + pin.
    pub right_pwm_slice: embassy_rp::Peri<'static, PWM_SLICE1>,
    pub right_pwm_pin: embassy_rp::Peri<'static, PIN_3>,
    /// Right motor forward direction pin (GPIO 10; moved off GPIO 4 to free UART1 TX).
    pub right_fwd: embassy_rp::Peri<'static, PIN_10>,
    /// Right motor backward direction pin (GPIO 11; moved off GPIO 5 to free UART1 RX).
    pub right_bwd: embassy_rp::Peri<'static, PIN_11>,
    /// TB6612FNG standby pin (active high).
    pub standby: embassy_rp::Peri<'static, PIN_6>,
    /// Left encoder PWM slice + pin (input mode).
    pub enc_left_slice: embassy_rp::Peri<'static, PWM_SLICE3>,
    pub enc_left_pin: embassy_rp::Peri<'static, PIN_7>,
    /// Right encoder PWM slice + pin (input mode).
    pub enc_right_slice: embassy_rp::Peri<'static, PWM_SLICE4>,
    pub enc_right_pin: embassy_rp::Peri<'static, PIN_9>,
}

/// Pins for the common-cathode RGB status LED.
pub struct RgbLedPins {
    /// Red channel (GPIO 13).
    pub red: embassy_rp::Peri<'static, PIN_13>,
    /// Green channel (GPIO 14).
    pub green: embassy_rp::Peri<'static, PIN_14>,
    /// Blue channel (GPIO 15).
    pub blue: embassy_rp::Peri<'static, PIN_15>,
}

/// Pins for the COIN-D6 360° spinning dTOF `LiDAR`.
///
/// Runs on **core1** over a dedicated UART0 peripheral. The sensor is off at
/// boot; the driver task powers it on demand and streams scan data at 10 Hz
/// (~230 400 baud). A power MOSFET (IRLS44N) on the supply rail allows
/// firmware-controlled power cycling.
///
/// Pin choice is constrained by the RP2350's fixed UART alt-function
/// table: UART0 RX only exists on GPIO 1/13/17, all otherwise claimed
/// (motor/RGB/I2C0) except GPIO 1 after relocating the motor's
/// `left_fwd` pin. UART0 TX (GPIO 12) was already free and is now wired: the
/// driver's bound needs a readable and writable stream even though start and
/// stop are best-effort.
pub struct D6LidarPins {
    /// UART0 TX — connect to `LiDAR` RX (GPIO 12).
    pub uart_tx: embassy_rp::Peri<'static, PIN_12>,
    /// UART0 RX — connect to `LiDAR` TX (GPIO 1).
    pub uart_rx: embassy_rp::Peri<'static, PIN_1>,
    /// Power MOSFET gate (IRLS44N low-side switch, active-high, GPIO 26).
    pub power_mosfet: embassy_rp::Peri<'static, PIN_26>,
    /// UART0 peripheral instance.
    pub uart: embassy_rp::Peri<'static, UART0>,
}

/// Resources for the Grove Vision AI V2 camera module.
///
/// Runs on **core0** over a dedicated UART1 peripheral. The module runs
/// on-device ML inference (Himax `WiseEye2`) and reports results over
/// serial. A power MOSFET (IRLS44N) on the supply rail allows
/// firmware-controlled power cycling.
///
/// GPIO 25 is unavailable (hardwired to the onboard LED), so this uses
/// UART1 on GPIO 4/5 after relocating the motor's `right_fwd`/`right_bwd`
/// pins, which are the RP2350's only UART1 TX/RX pin pair besides GPIO 8/9
/// (GPIO 9 is taken by the right encoder). Full duplex (unlike the D6),
/// since the module expects host commands and returns results over the
/// same UART — hence a second DMA channel for TX.
pub struct AiCamPins {
    /// UART1 TX — connect to AI cam RX (GPIO 4).
    pub uart_tx: embassy_rp::Peri<'static, PIN_4>,
    /// UART1 RX — connect to AI cam TX (GPIO 5).
    pub uart_rx: embassy_rp::Peri<'static, PIN_5>,
    /// Power MOSFET gate (IRLS44N low-side switch, active-high, GPIO 27).
    pub power_mosfet: embassy_rp::Peri<'static, PIN_27>,
    /// UART1 peripheral instance.
    pub uart: embassy_rp::Peri<'static, UART1>,
    /// DMA channel for UART1 TX streaming.
    pub dma_tx: embassy_rp::Peri<'static, DMA_CH5>,
    /// DMA channel for UART1 RX streaming.
    pub dma_rx: embassy_rp::Peri<'static, DMA_CH4>,
}

// ── Shared bus helpers ─────────────────────────────────────────────────────────

/// Initialise the shared I2C0 bus for the VL53L0X rangefinder.
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

/// Set up PIO-driven RGB LED (PWM on SM 0–2).
#[allow(clippy::unwrap_used)]
fn init_rgb_led(
    spawner: Spawner,
    pio_common: &mut Common<'static, PIO1>,
    sm0: StateMachine<'static, PIO1, 0>,
    sm1: StateMachine<'static, PIO1, 1>,
    sm2: StateMachine<'static, PIO1, 2>,
    rgb_pins: RgbLedPins,
) {
    let rgb_program = PioPwmProgram::new(pio_common);
    let pwm_red = PioPwm::new(pio_common, sm0, rgb_pins.red, &rgb_program);
    let pwm_green = PioPwm::new(pio_common, sm1, rgb_pins.green, &rgb_program);
    let pwm_blue = PioPwm::new(pio_common, sm2, rgb_pins.blue, &rgb_program);
    spawner.spawn(task::indicators::rgb_led_indicate::rgb_led_indicate(pwm_red, pwm_green, pwm_blue).unwrap());
}

/// Set up the TB6612FNG motor driver and encoder reader.
///
/// Spawns the motor driver task and the encoder reader task so that both
/// are running before any drive commands arrive.
#[allow(clippy::unwrap_used)]
fn init_motor_driver(spawner: Spawner, motor_pins: MotorDriverPins) {
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
fn init_ui(spawner: Spawner, panel_pins: PanelPins) {
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

/// `LiDAR` UART baud rate (230400 8N1).
const D6_LIDAR_BAUD: u32 = 230_400;
/// `BufferedUart` TX ring buffer size (start/stop commands are 4 bytes).
const D6_LIDAR_TX_BUF_LEN: usize = 16;
/// `BufferedUart` RX ring buffer size; absorbs stream data between reads.
const D6_LIDAR_RX_BUF_LEN: usize = 4096;

/// Static TX ring buffer for the `LiDAR`'s buffered UART.
static D6_LIDAR_TX_BUF: StaticCell<[u8; D6_LIDAR_TX_BUF_LEN]> = StaticCell::new();
/// Static RX ring buffer for the `LiDAR`'s buffered UART.
static D6_LIDAR_RX_BUF: StaticCell<[u8; D6_LIDAR_RX_BUF_LEN]> = StaticCell::new();

/// Bring up the COIN-D6 `LiDAR`'s buffered UART0 and spawn its task on core1.
///
/// The UART is full duplex — the driver's bound needs a readable and writable
/// stream even though start and stop are best-effort — on the reserved RX/TX pair
/// (GPIO 1 / GPIO 12). The power gate starts low, so the sensor is off at boot.
///
/// Must be called on **core1** so `UART0_IRQ` is enabled on core1's NVIC,
/// mirroring how `init_i2c_bus` must run on core0 for `I2C0_IRQ`.
#[allow(clippy::unwrap_used)]
fn init_lidar(spawner: Spawner, d6_pins: D6LidarPins) {
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = D6_LIDAR_BAUD;

    let uart = BufferedUart::new(
        d6_pins.uart,
        d6_pins.uart_tx,
        d6_pins.uart_rx,
        Irqs,
        D6_LIDAR_TX_BUF.init([0u8; D6_LIDAR_TX_BUF_LEN]),
        D6_LIDAR_RX_BUF.init([0u8; D6_LIDAR_RX_BUF_LEN]),
        uart_config,
    );

    // Powered off by default; the driver asserts this only while enabled.
    let power = Output::new(d6_pins.power_mosfet, Level::Low);

    spawner.spawn(task::sensors::lidar::lidar_task(uart, power).unwrap());
}

/// Bring up the Grove Vision AI V2's UART1 peripheral (full duplex) and power MOSFET.
///
/// Full duplex, unlike the D6: the module expects host commands (e.g.
/// invoke/query) and returns inference results over the same UART.
///
/// This only brings the peripheral up — no protocol implementation yet.
/// Returns the UART and the MOSFET output for the real driver task to
/// consume once it exists.
fn init_ai_cam_uart(ai_cam_pins: AiCamPins) -> (Uart<'static, uart::Async>, Output<'static>) {
    let mut uart_config = uart::Config::default();
    uart_config.baudrate = 115_200;

    let uart = Uart::new(
        ai_cam_pins.uart,
        ai_cam_pins.uart_tx,
        ai_cam_pins.uart_rx,
        Irqs,
        ai_cam_pins.dma_tx,
        ai_cam_pins.dma_rx,
        uart_config,
    );

    // Powered off by default; the real driver drives this high to power the module.
    let power_mosfet = Output::new(ai_cam_pins.power_mosfet, embassy_rp::gpio::Level::Low);

    (uart, power_mosfet)
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

    // ── PIO1: RGB LED (SM 0–2) ─────────────────────────────────────────────
    let Pio {
        common: mut pio1_common,
        sm0: pio1_sm0,
        sm1: pio1_sm1,
        sm2: pio1_sm2,
        ..
    } = Pio::new(p.PIO1, Irqs);

    // ── Pin structs ────────────────────────────────────────────────────────
    let rgb_pins = RgbLedPins {
        red: p.PIN_13,
        green: p.PIN_14,
        blue: p.PIN_15,
    };

    let motor_pins = MotorDriverPins {
        left_pwm_slice: p.PWM_SLICE0,
        left_pwm_pin: p.PIN_0,
        left_fwd: p.PIN_8,
        left_bwd: p.PIN_2,
        right_pwm_slice: p.PWM_SLICE1,
        right_pwm_pin: p.PIN_3,
        right_fwd: p.PIN_10,
        right_bwd: p.PIN_11,
        standby: p.PIN_6,
        enc_left_slice: p.PWM_SLICE3,
        enc_left_pin: p.PIN_7,
        enc_right_slice: p.PWM_SLICE4,
        enc_right_pin: p.PIN_9,
    };

    // D6 LiDAR — dedicated buffered UART0 on core1, powered on demand.
    let d6_pins = D6LidarPins {
        uart_tx: p.PIN_12,
        uart_rx: p.PIN_1,
        power_mosfet: p.PIN_26,
        uart: p.UART0,
    };

    // Grove Vision AI V2 — dedicated UART1 on core0 (reserved, not yet driven).
    let ai_cam_pins = AiCamPins {
        uart_tx: p.PIN_4,
        uart_rx: p.PIN_5,
        power_mosfet: p.PIN_27,
        uart: p.UART1,
        dma_tx: p.DMA_CH5,
        dma_rx: p.DMA_CH4,
    };

    // ── Core1: LiDAR driver ─────────────────────────────────────────────────
    #[allow(static_mut_refs)]
    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        // Initialised on core1 so UART0_IRQ is enabled on core1's NVIC.
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            init_lidar(spawner, d6_pins);
        });
    });

    // ── Core0: all other tasks ──────────────────────────────────────────────
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(move |spawner| {
        // I2C0 bus reserved for the VL53L0X rangefinder stub.
        // Initialised inside the closure so I2c::new_async enables I2C0_IRQ
        // on core0's NVIC.
        let _i2c_bus = init_i2c_bus(p.I2C0, p.PIN_16, p.PIN_17);

        // SPI bus for ICM20948 IMU
        let (imu_spi, imu_cs) = init_imu_spi(p.SPI0, p.PIN_18, p.PIN_19, p.PIN_20, p.PIN_21, p.DMA_CH1, p.DMA_CH2);

        // Shared full-duplex SPI1 bus and pins for the ST7789 panel and its
        // touch layer (ADR-0008).
        let panel_pins = PanelPins {
            spi: p.SPI1,
            sck: p.PIN_42,
            mosi: p.PIN_43,
            miso: p.PIN_44,
            tx_dma: p.DMA_CH6,
            rx_dma: p.DMA_CH7,
            display_cs: p.PIN_41,
            dc: p.PIN_45,
            rst: p.PIN_46,
            blk: p.PIN_47,
            touch_cs: p.PIN_38,
            penirq: p.PIN_39,
        };

        // UART1 for Grove Vision AI V2 — not yet consumed by any task, held
        // here until the real driver lands.
        let (_ai_cam_uart, _ai_cam_power_mosfet) = init_ai_cam_uart(ai_cam_pins);

        init_orchestrate(spawner);
        init_battery_monitoring(spawner, p.ADC, p.PIN_40);
        init_rgb_led(spawner, &mut pio1_common, pio1_sm0, pio1_sm1, pio1_sm2, rgb_pins);
        init_motor_driver(spawner, motor_pins);
        init_ui(spawner, panel_pins);
        init_vl53l0x_stub(spawner);
        init_imu(spawner, imu_spi, imu_cs);
        init_flash_storage(spawner, p.FLASH, p.DMA_CH0);
        init_testing(spawner);
        init_autonomous_mode(spawner);
        init_startup(spawner);
    });
}
