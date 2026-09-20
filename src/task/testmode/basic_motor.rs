//! Basic motor test mode task.
//!
//! Tests each track individually: left track then right track, each forward then
//! backward, logging the encoder pulse count for every leg. The four legs cycle
//! until the operator stops the test.
//!
//! Uses `MotorCommand::SetTracks` with one track active at a time. Encoder data
//! comes from `get_latest_encoder_measurement()` (drive sensor channel).
//!
//! The running screen reads the current leg and its percent from the activity
//! state; the live pulse counts go to the log. The test is interactive: it runs
//! until the operator taps Stop on the running screen.

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use touch_ui::Procedure;

use super::test_lifecycle;
use crate::{
    system::state::activity,
    task::{
        drive::{clear_encoder_measurement, get_latest_encoder_measurement},
        motor_driver::{self, MotorCommand, Track},
        procedure::Lifecycle,
        sensors::encoders::{self, EncoderCommand},
    },
};

/// How long one leg (one track, one direction) drives, in milliseconds.
const LEG_DURATION_MS: u64 = 2_000;

/// The basic motor test's lifecycle: the test family's stop latch and slot, with
/// no completion event — the test runs until the operator stops it.
const LIFECYCLE: Lifecycle = test_lifecycle(Procedure::BasicMotor);

/// How often the encoder count is sampled during a leg, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 100;

/// Motor speed commanded for a leg (0-100).
const LEG_SPEED: i8 = 50;

/// One leg of the test: the track under test, its phase label, and the commanded
/// speeds.
struct Leg {
    /// The track under test.
    track: Track,
    /// The phase line shown while this leg runs.
    phase: &'static str,
    /// Left track command.
    left: i8,
    /// Right track command.
    right: i8,
}

/// The legs, in order: each track forward, then each track backward.
const LEGS: [Leg; 4] = [
    Leg {
        track: Track::Left,
        phase: "Left forward",
        left: LEG_SPEED,
        right: 0,
    },
    Leg {
        track: Track::Left,
        phase: "Left reverse",
        left: -LEG_SPEED,
        right: 0,
    },
    Leg {
        track: Track::Right,
        phase: "Right forward",
        left: 0,
        right: LEG_SPEED,
    },
    Leg {
        track: Track::Right,
        phase: "Right reverse",
        left: 0,
        right: -LEG_SPEED,
    },
];

/// Spawn the basic motor test task via the controller.
#[allow(clippy::unwrap_used)]
pub(super) fn spawn(spawner: Spawner) {
    spawner.spawn(basic_motor_test_task().unwrap());
}

/// Basic motor test mode runner.
#[embassy_executor::task]
async fn basic_motor_test_task() {
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

    // The test is interactive: the legs cycle until the operator stops them.
    'cycling: loop {
        for (index, leg) in LEGS.iter().enumerate() {
            if LIFECYCLE.is_stop_requested() {
                break 'cycling;
            }

            // Progress done so far, so the bar never claims a leg that has not run.
            let percent = activity::percent_done(index, LEGS.len());
            LIFECYCLE.phase(leg.phase, Some(percent)).await;

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(100)).await;
            encoders::send_command(EncoderCommand::Reset).await;
            Timer::after(Duration::from_millis(100)).await;
            clear_encoder_measurement().await;

            motor_driver::send_motor_command(MotorCommand::SetTracks {
                left_speed: leg.left,
                right_speed: leg.right,
            })
            .await;

            run_leg(leg).await;

            motor_driver::send_motor_command(MotorCommand::CoastAll).await;
            Timer::after(Duration::from_millis(200)).await;
        }
    }

    motor_driver::send_motor_command(MotorCommand::CoastAll).await;
    encoders::send_command(EncoderCommand::Stop).await;
    motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
    LIFECYCLE.release();
}

/// Drive one leg for its duration, logging the encoder count as it goes.
async fn run_leg(leg: &Leg) {
    let mut elapsed_ms = 0;
    while elapsed_ms < LEG_DURATION_MS {
        if LIFECYCLE.wait_or_stop(SAMPLE_INTERVAL_MS).await {
            return;
        }
        elapsed_ms += SAMPLE_INTERVAL_MS;

        let count = get_latest_encoder_measurement()
            .await
            .map_or(0, |measurement| match leg.track {
                Track::Left => measurement.left,
                Track::Right => measurement.right,
            });
        info!("basic-motor: {=str} encoder={=u16}", leg.phase, count);
    }
}
