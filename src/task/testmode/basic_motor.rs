//! Basic motor test mode task.
//!
//! Tests each track individually: left track then right track.
//! Displays encoder pulse counts per track on the OLED.
//!
//! Uses `MotorCommand::SetTracks` with one track active at a time.
//! Encoder data comes from `get_latest_encoder_measurement()` (drive sensor
//! channel).

use core::sync::atomic::{AtomicBool, Ordering};

use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_sync::{blocking_mutex::raw::CriticalSectionRawMutex, signal::Signal};
use embassy_time::{Duration, Timer};
use heapless::String;

use super::{TestCommand, release_testmode, request_start};
use crate::task::{
    drive::{clear_encoder_measurement, get_latest_encoder_measurement},
    io::display::{DisplayAction, display_update},
    motor_driver::{self, MotorCommand, Track},
    sensors::encoders::{self, EncoderCommand},
};

/// Signal used to stop the basic motor test mode.
static BASIC_MOTOR_TEST_STOP_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Tracks whether the basic motor test mode is active.
static BASIC_MOTOR_TEST_ACTIVE: AtomicBool = AtomicBool::new(false);

/// Request the basic motor test mode to start (spawns the task on demand).
pub async fn start_basic_motor_test_mode() {
    if BASIC_MOTOR_TEST_ACTIVE.swap(true, Ordering::Relaxed) {
        return;
    }

    if !request_start(TestCommand::BasicMotor).await {
        BASIC_MOTOR_TEST_ACTIVE.store(false, Ordering::Relaxed);
    }
}

/// Request the basic motor test mode to stop.
pub fn stop_basic_motor_test_mode() {
    BASIC_MOTOR_TEST_ACTIVE.store(false, Ordering::Relaxed);
    BASIC_MOTOR_TEST_STOP_SIGNAL.signal(());
}

/// Spawn the basic motor test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(basic_motor_test_task().unwrap());
}

/// Track configuration for the test loop.
#[derive(Clone, Copy)]
struct TrackSpec {
    /// Display name for the track.
    name: &'static str,
    /// Which track side (left/right).
    track: Track,
}

/// Basic motor test mode runner.
///
/// Tests each track individually:
/// 1. Left track forward → show encoder counts → backward → show counts
/// 2. Right track forward → show encoder counts → backward → show counts
#[embassy_executor::task]
#[allow(clippy::too_many_lines)]
async fn basic_motor_test_task() {
    display_update(DisplayAction::Clear).await;

    // Clear any pending stop signal so the next test doesn't end immediately.
    while BASIC_MOTOR_TEST_STOP_SIGNAL.signaled() {
        BASIC_MOTOR_TEST_STOP_SIGNAL.wait().await;
    }

    // Enable both motor drivers before running the test.
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: true }).await;
    Timer::after(Duration::from_millis(10)).await;

    // Ensure encoders are sampling and clean.
    encoders::send_command(EncoderCommand::Stop).await;
    Timer::after(Duration::from_millis(100)).await;
    encoders::send_command(EncoderCommand::Reset).await;
    Timer::after(Duration::from_millis(100)).await;
    clear_encoder_measurement().await;
    encoders::send_command(EncoderCommand::Start { interval_ms: 50 }).await;
    Timer::after(Duration::from_millis(100)).await;

    let tracks = [
        TrackSpec {
            name: "Left Track",
            track: Track::Left,
        },
        TrackSpec {
            name: "Right Track",
            track: Track::Right,
        },
    ];

    'test: loop {
        for track_spec in tracks {
            if !BASIC_MOTOR_TEST_ACTIVE.load(Ordering::Relaxed) {
                break 'test;
            }

            // ── Show header ──────────────────────────────────────────────
            let mut header: String<20> = String::new();
            let _ = header.push_str("Basic Motor Test");
            display_update(DisplayAction::ShowText(header, 0)).await;

            let mut line1: String<20> = String::new();
            let _ = line1.push_str(track_spec.name);
            display_update(DisplayAction::ShowText(line1, 1)).await;

            let mut line3: String<20> = String::new();
            let _ = line3.push_str("Press to exit");
            display_update(DisplayAction::ShowText(line3, 3)).await;

            // ── Test forward ─────────────────────────────────────────────
            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            encoders::send_command(EncoderCommand::Reset).await;
            Timer::after(Duration::from_millis(100)).await;
            clear_encoder_measurement().await;

            // Activate only the track under test.
            let (left_speed, right_speed) = match track_spec.track {
                Track::Left => (50i8, 0i8),
                Track::Right => (0i8, 50i8),
            };
            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed,
                right_speed,
            })
            .await;

            for _ in 0..20 {
                match select(
                    BASIC_MOTOR_TEST_STOP_SIGNAL.wait(),
                    Timer::after(Duration::from_millis(100)),
                )
                .await
                {
                    Either::First(()) => break 'test,
                    Either::Second(()) => {
                        if !BASIC_MOTOR_TEST_ACTIVE.load(Ordering::Relaxed) {
                            break 'test;
                        }

                        let count =
                            get_latest_encoder_measurement()
                                .await
                                .map_or(0, |measurement| match track_spec.track {
                                    Track::Left => measurement.left,
                                    Track::Right => measurement.right,
                                });

                        let mut line2: String<20> = String::new();
                        let _ = core::fmt::write(&mut line2, format_args!("ENC: {count:>6}"));
                        display_update(DisplayAction::ShowText(line2, 2)).await;
                    }
                }
            }

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(200)).await;

            if !BASIC_MOTOR_TEST_ACTIVE.load(Ordering::Relaxed) {
                break 'test;
            }

            // ── Test backward ────────────────────────────────────────────
            let mut line1b: String<20> = String::new();
            let _ = line1b.push_str(track_spec.name);
            let _ = line1b.push_str(" REV");
            display_update(DisplayAction::ShowText(line1b, 1)).await;

            encoders::send_command(EncoderCommand::Reset).await;
            Timer::after(Duration::from_millis(100)).await;
            clear_encoder_measurement().await;

            let (left_speed, right_speed) = match track_spec.track {
                Track::Left => (-50i8, 0i8),
                Track::Right => (0i8, -50i8),
            };
            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed,
                right_speed,
            })
            .await;

            for _ in 0..20 {
                match select(
                    BASIC_MOTOR_TEST_STOP_SIGNAL.wait(),
                    Timer::after(Duration::from_millis(100)),
                )
                .await
                {
                    Either::First(()) => break 'test,
                    Either::Second(()) => {
                        if !BASIC_MOTOR_TEST_ACTIVE.load(Ordering::Relaxed) {
                            break 'test;
                        }

                        let count =
                            get_latest_encoder_measurement()
                                .await
                                .map_or(0, |measurement| match track_spec.track {
                                    Track::Left => measurement.left,
                                    Track::Right => measurement.right,
                                });

                        let mut line2: String<20> = String::new();
                        let _ = core::fmt::write(&mut line2, format_args!("ENC: {count:>6}"));
                        display_update(DisplayAction::ShowText(line2, 2)).await;
                    }
                }
            }

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(200)).await;
        }
    }

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    encoders::send_command(EncoderCommand::Stop).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    release_testmode();
    BASIC_MOTOR_TEST_ACTIVE.store(false, Ordering::Relaxed);
}
