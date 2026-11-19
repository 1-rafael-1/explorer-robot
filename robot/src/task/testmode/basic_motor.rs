//! Basic motor test mode task.
//!
//! Tests each track individually in four Test Legs — left forward, left reverse,
//! right forward and right reverse — with every leg stepping its track through
//! ten Power Steps from 10 % to 100 %. A Power Step commands only the track
//! under test and holds the other at zero, so every encoder count belongs to
//! exactly one known command. The forty steps cycle until the operator stops
//! the test; a full cycle takes a little over two minutes.
//!
//! A Power Step commands its track for 3 000 ms. A coast, an encoder reset and a
//! measurement clear precede it, and a further coast follows it, so the
//! commanded window contains no reset and the next reset never races a wheel
//! spinning down from full power. Uses `MotorCommand::SetTracks` with one track
//! active at a time; encoder data comes from
//! `get_latest_encoder_measurement()` (drive sensor channel).
//!
//! The running screen reads the current Test Leg label and the cycle-wide
//! percent from the activity state. A finished Power Step logs its count once at
//! info and each sample at debug, so a bench log stays readable. The test is
//! interactive: it runs until the operator taps Stop on the running screen.
//!
//! The panel's live Basic Motor Test Readout travels in its own state leaf: the
//! task publishes the typed readout at the sampling rate and the UI formats and
//! draws it. The encoder count never enters Activity State or the Link Protocol.

use defmt::{debug, error, info};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use touch_ui::{BasicMotorReadout, Direction, StepIndex, TestLeg, Track};

use crate::{
    system::state::{activity, basic_motor_test, perception},
    task::{
        drive::{clear_encoder_measurement, get_latest_encoder_measurement},
        motor_driver::{self, MotorCommand},
        procedure::Claim,
        sensors::encoders::{self, EncoderCommand},
    },
};

/// The commanded power of each Power Step within a Test Leg, in percent.
const STEP_POWERS: [i8; 10] = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100];

/// How long a Power Step commands its track, in milliseconds.
const STEP_DURATION_MS: u64 = 3_000;

/// How often the encoder count is sampled during a Power Step, in milliseconds.
const SAMPLE_INTERVAL_MS: u64 = 100;

/// How long the tracks coast before the encoder reset that starts a step, in
/// milliseconds.
const COAST_BEFORE_RESET_MS: u64 = 100;

/// How long the encoder reset settles before its measurement is cleared, in
/// milliseconds.
const RESET_SETTLE_MS: u64 = 100;

/// How long the tracks coast after a Power Step, in milliseconds.
const COAST_AFTER_STEP_MS: u64 = 200;

/// The Test Legs, in order: each track forward, then each track reverse.
const TEST_LEGS: [TestLeg; 4] = [
    TestLeg {
        track: Track::Left,
        direction: Direction::Forward,
    },
    TestLeg {
        track: Track::Left,
        direction: Direction::Reverse,
    },
    TestLeg {
        track: Track::Right,
        direction: Direction::Forward,
    },
    TestLeg {
        track: Track::Right,
        direction: Direction::Reverse,
    },
];

/// The left and right track commands for `leg` at `power` percent.
const fn speeds(leg: TestLeg, power: i8) -> (i8, i8) {
    let signed = match leg.direction {
        Direction::Forward => power,
        Direction::Reverse => -power,
    };
    match leg.track {
        Track::Left => (signed, 0),
        Track::Right => (0, signed),
    }
}

/// Power Steps in one Test Leg.
const STEPS_PER_LEG: usize = STEP_POWERS.len();

/// Power Steps in a full cycle, across all four Test Legs.
const TOTAL_STEPS: usize = TEST_LEGS.len() * STEPS_PER_LEG;

/// Spawn the basic motor test task via the controller.
///
/// The caller's claim travels with the task. Returns `false` when the spawn failed,
/// so the controller can abandon the claim rather than leave its running screen up
/// with no task behind it.
#[must_use]
pub(super) fn spawn(spawner: Spawner, claim: Claim) -> bool {
    basic_motor_test_task(claim).map_or_else(
        |_| {
            error!("basic-motor: spawn failed");
            false
        },
        |token| {
            spawner.spawn(token);
            true
        },
    )
}

/// Basic motor test mode runner.
#[embassy_executor::task]
async fn basic_motor_test_task(claim: Claim) {
    basic_motor_test::clear().await;

    // This test drives the motor driver directly, so it must run the same
    // Floor-Drop Gate the drive dispatch applies before a motor command.
    if let Err(refusal) = crate::task::sensors::rangefinder::arm_for_motion().await {
        claim.fail(refusal.reason()).await;
        return;
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

    // One readout follows the whole run. It names the step it is commanding and
    // holds the live count across the encoder reset between steps; the task only
    // drives its transitions and publishes copies.
    let mut readout = BasicMotorReadout::waiting();

    // The test is interactive: the forty steps cycle until the operator stops
    // them or floor-drop protection trips. A drop can arrive between steps or
    // during one, so both the step boundary and the step loop check the live
    // flag; a braked fault path is never overridden by the next SetTracks.
    let mut fault: Option<&'static str> = None;
    'cycling: loop {
        for (leg_index, leg) in TEST_LEGS.iter().copied().enumerate() {
            for (step_index, power) in STEP_POWERS.iter().copied().enumerate() {
                if claim.is_stop_requested() {
                    break 'cycling;
                }
                if perception::is_floor_drop_detected() {
                    fault = Some("floor drop detected");
                    break 'cycling;
                }

                // Progress done so far, so the bar never claims a step that has
                // not run. The step's one-based position in the same count names
                // it on the panel, so the bar returning to zero reads as a new
                // cycle.
                let done = leg_index * STEPS_PER_LEG + step_index;
                let percent = activity::percent_done(done, TOTAL_STEPS);
                claim.phase(leg.label(), Some(percent)).await;

                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
                Timer::after(Duration::from_millis(COAST_BEFORE_RESET_MS)).await;
                encoders::send_command(EncoderCommand::Reset).await;
                Timer::after(Duration::from_millis(RESET_SETTLE_MS)).await;
                clear_encoder_measurement().await;

                if perception::is_floor_drop_detected() {
                    fault = Some("floor drop detected");
                    break 'cycling;
                }

                // Name the step on the panel before it commands the track, so a
                // count can never appear under the previous step's label. The
                // start keeps the last displayed count: the encoder reset has
                // cleared the hardware, not the operator's reading.
                readout.start_step(leg, power.unsigned_abs(), step_position(done));
                basic_motor_test::publish(readout).await;

                let (left_speed, right_speed) = speeds(leg, power);
                motor_driver::send_motor_command(MotorCommand::SetTracks {
                    left_speed,
                    right_speed,
                })
                .await;

                match run_step(leg, power, &claim, &mut readout).await {
                    StepEnd::Completed(count) => {
                        if let Some(count) = count {
                            readout.complete(count);
                            basic_motor_test::publish(readout).await;
                            info!(
                                "basic-motor: {=str} {=u8}% encoder={=u16}",
                                leg.label(),
                                power.unsigned_abs(),
                                count
                            );
                        } else {
                            info!(
                                "basic-motor: {=str} {=u8}% no encoder sample",
                                leg.label(),
                                power.unsigned_abs()
                            );
                        }
                    }
                    StepEnd::Stopped => break 'cycling,
                    StepEnd::Fault => {
                        fault = Some("floor drop detected");
                        break 'cycling;
                    }
                }

                motor_driver::send_motor_command(MotorCommand::CoastAll).await;
                Timer::after(Duration::from_millis(COAST_AFTER_STEP_MS)).await;
            }
        }
    }

    encoders::send_command(EncoderCommand::Stop).await;
    basic_motor_test::clear().await;
    if fault.is_some() {
        // The floor-drop EmergencyBrake owns this stop: keep the brake latched
        // and leave the drivers enabled until the next activity manages them.
        motor_driver::send_motor_command(MotorCommand::BrakeAll).await;
        claim.fail(fault.unwrap_or("floor drop detected")).await;
    } else {
        motor_driver::send_motor_command(MotorCommand::CoastAll).await;
        motor_driver::send_motor_command(MotorCommand::SetAllDriversEnable { enabled: false }).await;
        claim.finish().await;
    }
}

/// Why a Power Step ended.
#[derive(Clone, Copy, PartialEq, Eq)]
enum StepEnd {
    /// The step ran for its full duration, leaving the last encoder sample it saw
    /// for the track under test, or `None` when no measurement arrived.
    Completed(Option<u16>),
    /// The operator pressed Stop.
    Stopped,
    /// Floor-drop protection tripped.
    Fault,
}

/// Command one Power Step for its duration, publishing its encoder count to the
/// readout and logging each sample at debug.
async fn run_step(leg: TestLeg, power: i8, claim: &Claim, readout: &mut BasicMotorReadout) -> StepEnd {
    let mut elapsed_ms = 0;
    let mut count = None;
    while elapsed_ms < STEP_DURATION_MS {
        if claim.wait_or_stop(SAMPLE_INTERVAL_MS).await {
            return StepEnd::Stopped;
        }
        if perception::is_floor_drop_detected() {
            return StepEnd::Fault;
        }
        elapsed_ms += SAMPLE_INTERVAL_MS;

        // A missing measurement is not a zero count: leave the live reading and
        // this step's own count alone rather than reporting pulses that did not
        // happen.
        let sample = get_latest_encoder_measurement()
            .await
            .map(|measurement| match leg.track {
                Track::Left => measurement.left,
                Track::Right => measurement.right,
            });
        let Some(sample) = sample else {
            continue;
        };

        count = Some(sample);
        readout.sample(sample);
        basic_motor_test::publish(*readout).await;
        debug!(
            "basic-motor: {=str} {=u8}% encoder={=u16}",
            leg.label(),
            power.unsigned_abs(),
            sample
        );
    }
    StepEnd::Completed(count)
}

/// The one-based position of a Power Step within the cycle, as the `Step` row
/// shows it, from the zero-based number of steps done before it.
fn step_position(done: usize) -> StepIndex {
    StepIndex {
        position: u8::try_from(done + 1).unwrap_or(u8::MAX),
        total: u8::try_from(TOTAL_STEPS).unwrap_or(u8::MAX),
    }
}
