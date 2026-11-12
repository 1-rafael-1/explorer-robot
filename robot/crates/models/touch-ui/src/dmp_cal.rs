//! The neutral DMP Compass Calibration snapshot and its Prompt surface.
//!
//! The firmware publishes this snapshot at the panel's telemetry rate while a
//! DMP Compass Calibration runs; the UI is the only formatter. Every value is
//! typed — the Compass Step, the measured heading, the target, the Compass
//! Accuracy, the motion hint and the check state — so the firmware never sends
//! preformatted text.
//!
//! The snapshot owns its one dynamic status line, formatted into a
//! fixed-capacity [`TextBuf`] and kept inside the panel's 48-byte row. It also
//! builds the [`PromptView`] the running Procedure shows, taking the title and
//! the screen Stop returns to from the Procedure's own identity.
//!
//! This module is panel-only: nothing here reaches Activity State or the Link
//! Protocol.

use core::fmt::Write as _;

use crate::{
    buffer::TextBuf,
    screens::{Procedure, PromptView},
    system_info::CalibrationStatus,
};
// The leading `::` selects the `dmp-cal` crate, not this module.
use ::dmp_cal::{CheckFailure, CheckOutcome, CompassStep, MotionHint};

/// The neutral Compass Accuracy: the DMP's `0..=3` scale, or `None` before the
/// first sample.
pub type DmpAccuracy = u8;

/// Why a Compass Check is failing, as the Prompt draws it.
///
/// The measured heading, target and accuracy stay on the same status line, so the
/// reason only needs to say which rule the check is breaking.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmpCheckFail {
    /// A sample in the stability window does not report Compass Accuracy 3.
    Accuracy,
    /// The measured Compass Heading is outside tolerance of the target.
    Heading,
}

/// The outcome of a Compass Check, as the Prompt draws it.
///
/// A movement step reports [`DmpCheckState::NotChecking`]. The remaining states
/// are the two Compass Checks' evaluations; `Done` and `Redo` are never disabled
/// by the state itself, so an incomplete hint cannot strand the operator.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DmpCheckState {
    /// The current step is not a Compass Check.
    NotChecking,
    /// The check has not gathered enough evidence yet.
    Waiting,
    /// The heading is moving around the target.
    Unstable,
    /// The check passed and accepts `Done`.
    Pass,
    /// The check failed and needs the operator's attention.
    Fail(DmpCheckFail),
}

impl DmpCheckState {
    /// The word the status line uses for this state, or `None` while the step is
    /// not a Compass Check.
    #[must_use]
    pub const fn word(self) -> Option<&'static str> {
        match self {
            Self::NotChecking => None,
            Self::Waiting => Some("Waiting"),
            Self::Unstable => Some("Unstable"),
            Self::Pass => Some("Pass"),
            Self::Fail(DmpCheckFail::Accuracy) => Some("Acc"),
            Self::Fail(DmpCheckFail::Heading) => Some("Hdg"),
        }
    }
}

impl From<CheckOutcome> for DmpCheckState {
    /// Map the model's check outcome onto the panel's check vocabulary.
    ///
    /// The measured values stay in the snapshot, so a failure is reduced to the
    /// rule it breaks.
    fn from(outcome: CheckOutcome) -> Self {
        match outcome {
            CheckOutcome::Waiting => Self::Waiting,
            CheckOutcome::Unstable => Self::Unstable,
            CheckOutcome::Pass => Self::Pass,
            CheckOutcome::Fail(CheckFailure::Accuracy { .. }) => Self::Fail(DmpCheckFail::Accuracy),
            CheckOutcome::Fail(CheckFailure::Heading { .. }) => Self::Fail(DmpCheckFail::Heading),
        }
    }
}

/// The informational hint for the current Compass Step.
///
/// Hints report whether the DMP is seeing the intended motion. They never gate
/// `Done`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum DmpHint {
    /// No hint for this step.
    None,
    /// Wrap-aware yaw span observed during the step, in degrees.
    YawSpanDeg(f32),
    /// Pitch span observed during the step, in degrees.
    PitchSpanDeg(f32),
    /// Roll span observed during the step, in degrees.
    RollSpanDeg(f32),
    /// How long the attitude has been still, in milliseconds.
    StillMillis(u64),
}

impl From<MotionHint> for DmpHint {
    /// Map a [`MotionHint`] from the model onto the panel's hint vocabulary.
    fn from(hint: MotionHint) -> Self {
        match hint {
            MotionHint::None => Self::None,
            MotionHint::YawSpanDeg(span) => Self::YawSpanDeg(span),
            MotionHint::PitchSpanDeg(span) => Self::PitchSpanDeg(span),
            MotionHint::RollSpanDeg(span) => Self::RollSpanDeg(span),
            MotionHint::StillMillis(millis) => Self::StillMillis(millis),
        }
    }
}

/// One panel-only snapshot of a running DMP Compass Calibration.
///
/// The firmware builds this from the latest IMU readings and the model's current
/// step and hint, then publishes it into the panel-only state module. The UI
/// formats the status line and the Prompt view from it; nothing here is
/// preformatted or allocated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DmpCalibrationSnapshot {
    /// The current Compass Step.
    pub step: CompassStep,
    /// The measured Compass Heading, in degrees, or `None` before a sample.
    pub heading_deg: Option<f32>,
    /// The target heading when the step is a Compass Check, in degrees.
    pub target_heading_deg: Option<f32>,
    /// The DMP Compass Accuracy, or `None` before a sample.
    pub accuracy: Option<DmpAccuracy>,
    /// The current step's informational motion hint.
    pub hint: DmpHint,
    /// The current Compass Check's outcome.
    pub check: DmpCheckState,
    /// Whether the Prompt offers `Redo` as well as `Done`.
    pub redo: bool,
    /// The DMP calibration status when the Procedure started.
    ///
    /// A ceremony started without a saved snapshot says so before asking the
    /// operator to move the robot; a loaded one keeps the line uncluttered.
    pub prior_status: CalibrationStatus,
}

impl DmpCalibrationSnapshot {
    /// A neutral snapshot for the procedure's first frame.
    ///
    /// It names the first Compass Step with no measured values, so the panel can
    /// show the Prompt from the moment the Procedure starts, before the runner
    /// publishes its first real snapshot.
    #[must_use]
    pub const fn starting() -> Self {
        Self {
            step: CompassStep::FlatHold,
            heading_deg: None,
            target_heading_deg: None,
            accuracy: None,
            hint: DmpHint::None,
            check: DmpCheckState::NotChecking,
            redo: false,
            prior_status: CalibrationStatus::Unknown,
        }
    }

    /// The step's zero-based position among the procedure's Compass Steps.
    #[must_use]
    pub const fn step_index(&self) -> usize {
        self.step.index()
    }

    /// The number of Compass Steps in the procedure.
    #[must_use]
    pub const fn step_count(&self) -> usize {
        CompassStep::ALL.len()
    }

    /// The instruction the Prompt draws for the current step.
    #[must_use]
    pub const fn instruction(&self) -> &'static str {
        self.step.instruction()
    }

    /// The percent progress the Prompt's bar draws.
    ///
    /// The first step is 0%; the value advances one step at a time.
    #[must_use]
    pub fn progress(&self) -> u8 {
        u8::try_from(self.step_index().saturating_mul(100) / self.step_count()).unwrap_or(100)
    }

    /// The one dynamic status line the Prompt draws below the instruction.
    ///
    /// Examples: `Hdg 123 Tgt 0 Acc 3 Span 45`, `Hdg 123 Acc 3 Still 2s`,
    /// `Hdg -- Acc --`. Every value is whole so the line stays inside the
    /// panel's 48-byte row.
    #[must_use]
    pub fn status_text(&self) -> TextBuf {
        let mut line = TextBuf::new();
        if self.prior_status != CalibrationStatus::Loaded {
            let _ = write!(line, "Cal {} ", prior_word(self.prior_status));
        }
        let _ = match self.heading_deg {
            Some(heading) => write!(line, "Hdg {heading:.0}"),
            None => write!(line, "Hdg --"),
        };
        if let Some(target) = self.target_heading_deg {
            let _ = write!(line, " Tgt {target:.0}");
        }
        let _ = match self.accuracy {
            Some(accuracy) => write!(line, " Acc {accuracy}"),
            None => write!(line, " Acc --"),
        };
        match self.hint {
            DmpHint::None => {}
            DmpHint::YawSpanDeg(span) | DmpHint::PitchSpanDeg(span) | DmpHint::RollSpanDeg(span) => {
                let _ = write!(line, " Span {span:.0}");
            }
            DmpHint::StillMillis(millis) => {
                let _ = write!(line, " Still {}s", millis / 1_000);
            }
        }
        if let Some(word) = self.check.word() {
            let _ = write!(line, " Check {word}");
        }
        line
    }

    /// Build the Prompt this Procedure shows, taking its title and where Stop
    /// returns from the Procedure's own identity.
    #[must_use]
    pub fn prompt_view(&self) -> PromptView {
        let view = PromptView::for_procedure(Procedure::DmpCompassCalibration, self.instruction())
            .with_status(self.status_text())
            .with_progress(self.progress());
        if self.redo { view.with_redo() } else { view }
    }
}

/// The lowercase word the status line uses for a non-loaded starting state.
const fn prior_word(status: CalibrationStatus) -> &'static str {
    match status {
        CalibrationStatus::Loaded => "loaded",
        CalibrationStatus::Unknown => "unknown",
        CalibrationStatus::Missing => "missing",
    }
}

impl Default for DmpCalibrationSnapshot {
    /// The same neutral snapshot as [`DmpCalibrationSnapshot::starting`].
    fn default() -> Self {
        Self::starting()
    }
}
