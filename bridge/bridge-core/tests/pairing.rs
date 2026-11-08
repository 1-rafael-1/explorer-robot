//! Host tests for the pairing code (ADR-0037).
//!
//! The code is 60 bits in twelve unambiguous base32 characters; its characters are also the HMAC
//! key, so the tests pin the length, the alphabet and the bit-to-character mapping.

use bridge_core::pairing::{ALPHABET, CODE_BITS, CODE_LEN, GROUPED_LEN, encode, is_valid, write_grouped};

/// Value zero is twelve zeroes, the fixed point every decoder must handle.
#[test]
fn zero_encodes_as_twelve_zero_symbols() {
    assert_eq!(&encode(0), b"000000000000");
}

/// The largest 60-bit value uses the last alphabet symbol in every position.
#[test]
fn the_full_60_bits_encode_as_the_last_symbol() {
    let max = (1u64 << CODE_BITS) - 1;
    let expected = [ALPHABET[31]; CODE_LEN];
    assert_eq!(encode(max), expected);
}

/// The contract is exactly 60 bits: the top four bits of the caller's word are ignored.
#[test]
fn bits_above_60_are_ignored() {
    let low = (1u64 << CODE_BITS) - 1;
    assert_eq!(encode(low | (0xF << CODE_BITS)), encode(low));
}

/// Every character is a symbol of the 32-character unambiguous alphabet; `I`, `L`, `O` and `U`
/// are not in it, so an operator cannot confuse them with `1`, `1`, `0` and `V`.
#[test]
fn the_alphabet_is_complete_and_unambiguous() {
    assert_eq!(ALPHABET.len(), 32);
    for index in 0..ALPHABET.len() {
        assert!(
            !ALPHABET[..index].contains(&ALPHABET[index]),
            "alphabet repeats {}",
            ALPHABET[index]
        );
    }
    for look_alike in *b"ILOU" {
        assert!(!ALPHABET.contains(&look_alike), "{look_alike} is ambiguous");
    }

    for value in 0..1024u64 {
        for symbol in encode(value) {
            assert!(ALPHABET.contains(&symbol), "{symbol} is outside the alphabet");
        }
    }
}

/// Each five-bit chunk selects a fixed alphabet symbol, most significant first. The expected
/// strings are literals so a mapping change cannot be derived from the production formula.
#[test]
fn each_five_bit_chunk_selects_its_symbol() {
    assert_eq!(&encode(21 << 55), b"N00000000000", "top chunk");
    assert_eq!(&encode(21 << 50), b"0N0000000000", "second chunk");
    assert_eq!(&encode(21 << 25), b"000000N00000", "middle chunk");
    assert_eq!(&encode(21), b"00000000000N", "last chunk");
    assert_eq!(&encode((31 << 55) | 1), b"Z00000000001", "two chunks");
}

/// Grouping is three groups of four, separated by hyphens, and preserves the characters.
#[test]
fn grouping_is_three_groups_of_four() {
    let code = *b"ABCD2345EFGH";
    let mut grouped = [0u8; GROUPED_LEN];
    write_grouped(&code, &mut grouped);
    assert_eq!(&grouped, b"ABCD-2345-EFGH");
}

/// Validity is exactly twelve alphabet characters; wrong lengths and look-alikes are rejected.
#[test]
fn validity_rejects_wrong_lengths_and_look_alikes() {
    assert!(is_valid(b"ABCD2345EFGH"));
    assert!(is_valid(b"000000000000"));
    assert!(is_valid(b"ABCD2345EFG0"), "digits are alphabet symbols");
    assert!(is_valid(b"ABCD2345EFG1"), "digits are alphabet symbols");
    assert!(!is_valid(b"ABCD2345EFG"));
    assert!(!is_valid(b"ABCD2345EFGHJ"));
    assert!(!is_valid(b"ABCD2345EFGI"));
    assert!(!is_valid(b"ABCD2345EFGL"));
    assert!(!is_valid(b"ABCD2345EFGO"));
    assert!(!is_valid(b"ABCD2345EFGU"));
    assert!(!is_valid(b"abcd2345efgh"));
}
