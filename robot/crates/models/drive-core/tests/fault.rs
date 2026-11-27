//! Host-side tests pinning the operator-facing fault labels.
//!
//! The labels are the completion details on the wire. They are preserved
//! byte-for-byte from the firmware drive code, including the lowercase settle
//! timeout, so an accidental rename fails loudly.

use drive_core::DriveFault;

#[test]
fn every_fault_keeps_its_operator_facing_label() {
    assert_eq!(DriveFault::EncoderTimeout.label(), "EncoderTimeout");
    assert_eq!(DriveFault::Stall.label(), "StallTimeout");
    assert_eq!(DriveFault::EncoderAnomaly.label(), "EncoderAnomaly");
    assert_eq!(DriveFault::RotationTimeout.label(), "RotateTimeout");
    assert_eq!(DriveFault::CorrectionLimit.label(), "RotateCorrectionLimit");
    assert_eq!(DriveFault::CorrectionTimeout.label(), "RotateCorrectionTimeout");
    assert_eq!(DriveFault::CorrectionExhausted.label(), "CorrectionExhausted");
    assert_eq!(DriveFault::SettleTimeout.label(), "encoder settle timeout");
}
