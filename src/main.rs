//! explorer-robot v3 firmware entry point
//!
//! Core0 hosts orchestrator, motor driver, encoders, battery monitor,
//! RGB LED, rotary encoder, display, VL53L0X stub, flash storage, UI,
//! testmode, autonomous mode controller, and startup.
//!
//! Core1 hosts the `LiDAR` stub task producing synthetic point-cloud data.

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
    gpio::{Input, Pull},
    i2c::{Config as I2cConfig, I2c, InterruptHandler as I2cInterruptHandler},
    multicore::{Stack, spawn_core1},
    peripherals::{
        ADC, DMA_CH0, FLASH, I2C0, PIN_0, PIN_1, PIN_2, PIN_3, PIN_4, PIN_5, PIN_6, PIN_7, PIN_9, PIN_13, PIN_14,
        PIN_15, PIN_16, PIN_17, PIN_22, PIN_23, PIN_24, PIN_28, PIO1, PWM_SLICE0, PWM_SLICE1, PWM_SLICE3, PWM_SLICE4,
    },
    pio::{Common, InterruptHandler as PioInterruptHandler, Pio, StateMachine},
    pio_programs::{
        pwm::{PioPwm, PioPwmProgram},
        rotary_encoder::{PioEncoder, PioEncoderProgram},
    },
    pwm::{Config as PwmConfig, InputMode, Pwm},
};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, mutex::Mutex};
use panic_probe as _;
use static_cell::StaticCell;

mod system;
mod task;

// ── Crate-level type alias ────────────────────────────────────────────────────

/// Shared I2C0 bus protected by a critical-section mutex.
///
/// Used by the SSD1306 OLED display and (eventually) the VL53L0X
/// rangefinder array, both on core0.
pub type I2cBusShared = Mutex<CriticalSectionRawMutex, I2c<'static, I2C0, embassy_rp::i2c::Async>>;

// ── Interrupt bindings ─────────────────────────────────────────────────────────

bind_interrupts!(pub struct Irqs {
    I2C0_IRQ => I2cInterruptHandler<I2C0>;
    ADC_IRQ_FIFO => AdcInterruptHandler;
    PIO1_IRQ_0 => PioInterruptHandler<PIO1>;
    DMA_IRQ_0 => DmaInterruptHandler<DMA_CH0>;
});

// ── Boot descriptor ────────────────────────────────────────────────────────────

/// Firmware image type for bootloader
#[unsafe(link_section = ".start_block")]
#[used]
pub static IMAGE_DEF: ImageDef = ImageDef::secure_exe();

// ── Core stacks & executors ────────────────────────────────────────────────────

/// Core1 stack — 4 KiB for the `LiDAR` stub task.
static mut CORE1_STACK: Stack<4096> = Stack::new();

/// Executor for core0 (orchestrator, drive, UI, sensors on I2C).
static EXECUTOR0: StaticCell<Executor> = StaticCell::new();

/// Executor for core1 (`LiDAR` stub).
static EXECUTOR1: StaticCell<Executor> = StaticCell::new();

// ── Pin structs ────────────────────────────────────────────────────────────────

/// Pins used by the TB6612FNG motor driver and motor encoders.
///
/// Two-track configuration: left PWM on GPIO 0 (slice 0), right PWM on
/// GPIO 3 (slice 1). Direction pins are direct-GPIO outputs (IN1/IN2 per
/// channel). Encoders use PWM input mode on slices 3 and 4.
pub struct MotorDriverPins {
    /// Left motor PWM slice + pin.
    pub left_pwm_slice: embassy_rp::Peri<'static, PWM_SLICE0>,
    pub left_pwm_pin: embassy_rp::Peri<'static, PIN_0>,
    /// Left motor forward direction pin.
    pub left_fwd: embassy_rp::Peri<'static, PIN_1>,
    /// Left motor backward direction pin.
    pub left_bwd: embassy_rp::Peri<'static, PIN_2>,
    /// Right motor PWM slice + pin.
    pub right_pwm_slice: embassy_rp::Peri<'static, PWM_SLICE1>,
    pub right_pwm_pin: embassy_rp::Peri<'static, PIN_3>,
    /// Right motor forward direction pin.
    pub right_fwd: embassy_rp::Peri<'static, PIN_4>,
    /// Right motor backward direction pin.
    pub right_bwd: embassy_rp::Peri<'static, PIN_5>,
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

/// Pins for the EC11 rotary encoder (quadrature A/B + push button).
///
/// Button pin is supplied as a bare `Peri`; `Input<Pull::Up>` is created
/// inside `init_rotary_encoder`.
pub struct Ec11Pins {
    /// Encoder A signal (GPIO 22).
    pub a: embassy_rp::Peri<'static, PIN_22>,
    /// Encoder B signal (GPIO 23).
    pub b: embassy_rp::Peri<'static, PIN_23>,
    /// Push button (GPIO 24, configured as Input with `Pull::Up`).
    pub btn: embassy_rp::Peri<'static, PIN_24>,
}

// ── Shared bus helpers ─────────────────────────────────────────────────────────

/// Initialise the shared I2C0 bus for display and VL53L0X.
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

// ── Init functions (v2 pattern: one per subsystem) ─────────────────────────────

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
    adc_pin: embassy_rp::Peri<'static, PIN_28>,
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

/// Set up PIO-driven EC11 rotary encoder (quadrature on SM 3 + button).
///
/// Spawns both the turns reader task and the button handler task.
#[allow(clippy::unwrap_used)]
fn init_rotary_encoder(
    spawner: Spawner,
    pio_common: &mut Common<'static, PIO1>,
    sm3: StateMachine<'static, PIO1, 3>,
    ec11_pins: Ec11Pins,
) {
    let encoder_program = PioEncoderProgram::new(pio_common);
    let encoder = PioEncoder::new(pio_common, sm3, ec11_pins.a, ec11_pins.b, &encoder_program);
    spawner.spawn(task::control::rotary_encoder::rotary_encoder_turns(encoder).unwrap());

    let button = Input::new(ec11_pins.btn, Pull::Up);
    spawner.spawn(task::control::rotary_encoder::rotary_encoder_button(button).unwrap());
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
}

/// Initialise the SSD1306 OLED display on the shared I2C0 bus.
#[allow(clippy::unwrap_used)]
fn init_display(spawner: Spawner, i2c_bus: &'static I2cBusShared) {
    spawner.spawn(task::io::display::display(i2c_bus).unwrap());
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

/// Initialise the UI subsystem (controller + render task).
fn init_ui(spawner: Spawner) {
    task::ui::init_ui(spawner);
}

/// Initialise the autonomous mode controller (coast-and-avoid etc.).
fn init_autonomous_mode(spawner: Spawner) {
    task::autonomous_mode::init_autonomous_mode(spawner);
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

    // ── PIO1: RGB LED (SM 0–2) + rotary encoder quadrature (SM 3) ──────────
    let Pio {
        common: mut pio1_common,
        sm0: pio1_sm0,
        sm1: pio1_sm1,
        sm2: pio1_sm2,
        sm3: pio1_sm3,
        ..
    } = Pio::new(p.PIO1, Irqs);

    // ── Pin structs ────────────────────────────────────────────────────────
    let rgb_pins = RgbLedPins {
        red: p.PIN_13,
        green: p.PIN_14,
        blue: p.PIN_15,
    };

    let ec11_pins = Ec11Pins {
        a: p.PIN_22,
        b: p.PIN_23,
        btn: p.PIN_24,
    };

    let motor_pins = MotorDriverPins {
        left_pwm_slice: p.PWM_SLICE0,
        left_pwm_pin: p.PIN_0,
        left_fwd: p.PIN_1,
        left_bwd: p.PIN_2,
        right_pwm_slice: p.PWM_SLICE1,
        right_pwm_pin: p.PIN_3,
        right_fwd: p.PIN_4,
        right_bwd: p.PIN_5,
        standby: p.PIN_6,
        enc_left_slice: p.PWM_SLICE3,
        enc_left_pin: p.PIN_7,
        enc_right_slice: p.PWM_SLICE4,
        enc_right_pin: p.PIN_9,
    };

    // ── Core1: LiDAR stub task ──────────────────────────────────────────────
    #[allow(static_mut_refs)]
    spawn_core1(p.CORE1, unsafe { &mut CORE1_STACK }, move || {
        let executor1 = EXECUTOR1.init(Executor::new());
        executor1.run(|spawner| {
            spawner.spawn(task::sensors::lidar_stub::lidar_stub_task().unwrap());
        });
    });

    // ── Core0: all other tasks ──────────────────────────────────────────────
    let executor0 = EXECUTOR0.init(Executor::new());
    executor0.run(move |spawner| {
        // I2C0 bus for SSD1306 OLED display and VL53L0X rangefinder array.
        // Initialised inside the closure so I2c::new_async enables I2C0_IRQ
        // on core0's NVIC (matching v2's core1 I2C pattern).
        let i2c_bus = init_i2c_bus(p.I2C0, p.PIN_16, p.PIN_17);

        init_orchestrate(spawner);
        init_battery_monitoring(spawner, p.ADC, p.PIN_28);
        init_rgb_led(spawner, &mut pio1_common, pio1_sm0, pio1_sm1, pio1_sm2, rgb_pins);
        init_rotary_encoder(spawner, &mut pio1_common, pio1_sm3, ec11_pins);
        init_motor_driver(spawner, motor_pins);
        init_display(spawner, i2c_bus);
        init_vl53l0x_stub(spawner);
        init_flash_storage(spawner, p.FLASH, p.DMA_CH0);
        init_testing(spawner);
        init_ui(spawner);
        init_autonomous_mode(spawner);
        init_startup(spawner);
    });
}
