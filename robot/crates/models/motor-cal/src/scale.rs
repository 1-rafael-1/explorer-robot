//! The Distance Calibration scale: counts per centimetre, and the counts per
//! sprocket revolution that follow from it.
//!
//! **Distance Calibration** stores one measured number: how many encoder counts
//! one real-world centimetre of travel produces. It is measured, not assumed —
//! the accumulated counts of a driven leg divided by the distance the operator
//! tapes out — so surface friction and track wear are folded into the same
//! figure as the sprocket geometry.
//!
//! Counts per sprocket revolution is then derived from that scale and the
//! sprocket circumference, so the firmware no longer needs a
//! pulses-per-revolution constant: the circumference is the only geometry input,
//! and [`SPROCKET_CIRCUMFERENCE_CM`] holds the nominal value the drive types use.
//!
//! Derivation is fallible. A zero or negative measured distance cannot be
//! divided by, and a scale outside [`MIN_PLAUSIBLE_COUNTS_PER_CM`]..=
//! [`MAX_PLAUSIBLE_COUNTS_PER_CM`] means the run measured something other than
//! the robot's travel, so it is rejected with a re-run reason rather than saved.

/// The smallest plausible encoder scale, in counts per centimetre.
///
/// The band is deliberately generous: the uncertainty is in a tape measure, not
/// in the model. A result below it means the run barely moved.
pub const MIN_PLAUSIBLE_COUNTS_PER_CM: f32 = 5.0;

/// The largest plausible encoder scale, in counts per centimetre.
///
/// A result above it means the robot was held back or the counts came from the
/// wrong track.
pub const MAX_PLAUSIBLE_COUNTS_PER_CM: f32 = 100.0;

/// The effective drive-sprocket circumference with the track installed, in
/// centimetres.
///
/// This is the same figure the drive geometry uses; the model takes the
/// circumference as an input so no pulses-per-revolution constant is baked in.
pub const SPROCKET_CIRCUMFERENCE_CM: f32 = 19.01;

/// The inferred encoder counts per sprocket revolution: 11 pulses per motor
/// revolution behind a 30:1 gearbox.
///
/// This is the hardware inference the bench run in ticket 09 confirms. It is an
/// input to [`NOMINAL_COUNTS_PER_CM`], never a pulse constant the drive path
/// applies directly.
pub const INFERRED_COUNTS_PER_SPROCKET_REV: f32 = 330.0;

/// The nominal encoder scale: [`INFERRED_COUNTS_PER_SPROCKET_REV`] spread over
/// [`SPROCKET_CIRCUMFERENCE_CM`], in counts per centimetre.
///
/// **Inferred, not measured.** The drive path falls back to
/// [`DistanceScale::nominal`] when no Distance Calibration record exists, so a
/// fresh robot still drives (roughly) the distance it was asked for instead of
/// the roughly three times too far the retired 960-counts-per-revolution
/// constant produced. Ticket 09's bench run decides whether this figure stays.
pub const NOMINAL_COUNTS_PER_CM: f32 = INFERRED_COUNTS_PER_SPROCKET_REV / SPROCKET_CIRCUMFERENCE_CM;

/// The encoder scale: encoder counts per centimetre of travel.
///
/// Construct it with [`from_measurement`](Self::from_measurement) from a
/// Distance Calibration run, or with
/// [`from_counts_per_cm`](Self::from_counts_per_cm) from a known figure. Both
/// reject an implausible scale; a decoded flash record carries the scale that was
/// validated when it was written.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DistanceScale {
    /// The measured counts per centimetre.
    counts_per_cm: f32,
}

impl DistanceScale {
    /// The scale implied by `counts` accumulated encoder counts over the
    /// operator's `measured_cm`.
    ///
    /// # Errors
    ///
    /// Returns [`ScaleError::NoDistance`] when `measured_cm` is zero, negative or
    /// not finite, and [`ScaleError::Implausible`] when the resulting scale falls
    /// outside the plausibility band.
    // The cast is the `f32` conversion of a count total, which is far below the
    // exact-integer range of `f32`. `f32::from` is unavailable in a `no_std`
    // const context and the caller's value is a measurement, not a literal.
    #[allow(clippy::cast_precision_loss)]
    pub fn from_measurement(counts: u32, measured_cm: f32) -> Result<Self, ScaleError> {
        if !measured_cm.is_finite() || measured_cm <= 0.0 {
            return Err(ScaleError::NoDistance);
        }
        Self::from_counts_per_cm(counts as f32 / measured_cm)
    }

    /// The inferred fallback scale used until Distance Calibration has run.
    ///
    /// It is [`NOMINAL_COUNTS_PER_CM`] by construction, so it is inside the
    /// plausibility band and needs no validation. The drive path uses it when the
    /// calibration state leaf holds no measured scale; a measured scale always
    /// replaces it.
    #[must_use]
    pub const fn nominal() -> Self {
        Self {
            counts_per_cm: NOMINAL_COUNTS_PER_CM,
        }
    }

    /// A scale from a known counts-per-centimetre figure.
    ///
    /// # Errors
    ///
    /// Returns [`ScaleError::Implausible`] when `counts_per_cm` is outside
    /// [`MIN_PLAUSIBLE_COUNTS_PER_CM`]..=[`MAX_PLAUSIBLE_COUNTS_PER_CM`],
    /// including when it is not a finite number.
    pub fn from_counts_per_cm(counts_per_cm: f32) -> Result<Self, ScaleError> {
        if !(MIN_PLAUSIBLE_COUNTS_PER_CM..=MAX_PLAUSIBLE_COUNTS_PER_CM).contains(&counts_per_cm) {
            return Err(ScaleError::Implausible { counts_per_cm });
        }
        Ok(Self { counts_per_cm })
    }

    /// A scale that was validated when it was written to flash.
    ///
    /// Only the payload decoder uses this: a checksummed record was validated by
    /// [`from_counts_per_cm`](Self::from_counts_per_cm) before it was stored, so
    /// re-applying today's band would wrongly reject a record written under an
    /// older one.
    pub(crate) const fn from_stored(counts_per_cm: f32) -> Self {
        Self { counts_per_cm }
    }

    /// The measured scale, in encoder counts per centimetre.
    #[must_use]
    pub const fn counts_per_cm(self) -> f32 {
        self.counts_per_cm
    }

    /// The counts per sprocket revolution implied by `circumference_cm`.
    ///
    /// `None` when the circumference is zero, negative or not finite: such a
    /// sprocket has no meaningful revolution.
    #[must_use]
    pub fn counts_per_rev(self, circumference_cm: f32) -> Option<f32> {
        if !circumference_cm.is_finite() || circumference_cm <= 0.0 {
            return None;
        }
        Some(self.counts_per_cm * circumference_cm)
    }
}

/// Why a Distance Calibration scale was rejected.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum ScaleError {
    /// The operator's measured distance was zero, negative or not finite.
    NoDistance,
    /// The resulting scale is outside the plausibility band, so the run must be
    /// repeated.
    Implausible {
        /// The rejected scale, in counts per centimetre.
        counts_per_cm: f32,
    },
}

impl ScaleError {
    /// A short, stable phrase naming the reason for an operator log.
    #[must_use]
    pub const fn label(self) -> &'static str {
        match self {
            Self::NoDistance => "no measured distance",
            Self::Implausible { .. } => "implausible counts/cm - re-run",
        }
    }
}
