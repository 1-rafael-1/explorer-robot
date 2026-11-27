//! The shared physical geometry of the drive: track width and sprocket
//! circumference.
//!
//! These are the two lengths that tie the encoder world to the floor. The
//! sprocket circumference turns sprocket revolutions into travel centimetres,
//! and the track width turns a left/right revolution difference into a heading
//! change. `motor-cal` derives counts per sprocket revolution from the
//! circumference, and the drive path converts accumulated revolutions to
//! centimetres and curves with both figures.
//!
//! Keeping them in one place means the Distance Calibration scale and the drive
//! geometry cannot disagree. It is `#![no_std]`, always available to every
//! target, and the `std` feature exists only to enable the host tests under
//! `tests/`; the library never links `std`.

#![no_std]
#![warn(missing_docs)]

/// The effective drive-sprocket circumference with the track installed, in
/// centimetres.
///
/// This is the single home for the figure: `motor-cal` derives counts per
/// sprocket revolution from it, and the drive path converts accumulated
/// revolutions to centimetres with it. No pulses-per-revolution constant is
/// baked in.
pub const SPROCKET_CIRCUMFERENCE_CM: f32 = 19.01;

/// Distance between left/right track centerlines (cm).
///
/// The drive path uses it to turn a left/right revolution difference into the
/// heading change of a curve.
pub const TRACK_WIDTH_CM: f32 = 14.5;
