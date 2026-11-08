//! Host tests for the CRC-16/CCITT-FALSE calculator (ADR-0030).

use link_protocol::crc::{Crc16, crc16};

/// The published check value for the ASCII string `"123456789"` is `0x29B1`.
#[test]
fn check_vector_is_0x29b1() {
    assert_eq!(crc16(b"123456789"), 0x29B1, "the standard check value must match");
}

/// An empty input leaves the initial value untouched.
#[test]
fn empty_input_is_the_initial_value() {
    assert_eq!(crc16(&[]), 0xFFFF, "the initial value must be all ones");
}

/// Feeding the input in chunks gives the same result as one call.
#[test]
fn incremental_crc_equals_one_shot() {
    let bytes = b"the quick brown fox jumps over the lazy dog";
    let mut incremental = Crc16::new();
    for chunk in bytes.chunks(7) {
        incremental.update(chunk);
    }
    assert_eq!(
        incremental.finish(),
        crc16(bytes),
        "chunked and one-shot CRCs must agree"
    );
}

/// Feeding single bytes gives the same result as feeding the whole slice.
#[test]
fn byte_by_byte_crc_equals_slice_crc() {
    let bytes = [0x00, 0xA5, 0x5A, 0xFF, 0x10, 0x20];
    let mut incremental = Crc16::new();
    for &byte in &bytes {
        incremental.update_byte(byte);
    }
    assert_eq!(
        incremental.finish(),
        crc16(&bytes),
        "byte-by-byte and slice CRCs must agree"
    );
}

/// `Default` and `new` start from the same initial value.
#[test]
fn default_matches_new() {
    assert_eq!(Crc16::default(), Crc16::new(), "both constructors must agree");
}
