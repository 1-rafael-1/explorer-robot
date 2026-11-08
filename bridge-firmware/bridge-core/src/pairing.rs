//! The 60-bit pairing token and its twelve-character base32 display (ADR-0037).
//!
//! The token is 60 bits from the hardware TRNG, displayed as twelve characters in three groups
//! of four. The displayed characters *are* the HMAC key: `bridge-net` consumes their bytes
//! verbatim, so there is no decode step and no second representation to drift. Encoding is
//! therefore the only operation this module needs; the record stores the twelve characters.
//!
//! The alphabet omits the look-alikes `I`, `L`, `O` and `U`, so an operator reading the code off
//! the panel's `Link` screen cannot mistype it into the app.

/// The number of characters in a pairing code (ADR-0037), taken from the wire codec so the
/// record and `PairingCode` cannot disagree.
pub const CODE_LEN: usize = link_protocol::PAIRING_CODE_LEN;

/// The number of significant bits a pairing code carries.
pub const CODE_BITS: u32 = 60;

/// The number of bits one character carries.
const BITS_PER_CHAR: u32 = 5;

/// The number of characters in one display group.
pub const GROUP_LEN: usize = 4;

/// The displayed code's length: three groups of four plus two separators.
pub const GROUPED_LEN: usize = CODE_LEN + 2;

/// The unambiguous base32 alphabet: `0-9` and `A-Z` without `I`, `L`, `O` and `U`.
pub const ALPHABET: &[u8; 32] = b"0123456789ABCDEFGHJKMNPQRSTVWXYZ";

/// The mask that keeps exactly [`CODE_BITS`] bits.
const CODE_MASK: u64 = (1 << CODE_BITS) - 1;

/// Encodes the low [`CODE_BITS`] bits of `value` as twelve base32 characters.
///
/// The high four bits of `value` are ignored, so a caller that fills a whole `u64` from the RNG
/// cannot accidentally exceed the 60-bit contract.
#[must_use]
pub fn encode(value: u64) -> [u8; CODE_LEN] {
    let value = value & CODE_MASK;
    let mut code = [0u8; CODE_LEN];
    for (index, symbol) in code.iter_mut().enumerate() {
        let position = u32::try_from(index).unwrap_or(0) + 1;
        let shift = CODE_BITS - BITS_PER_CHAR * position;
        let slot = usize::try_from((value >> shift) & 0x1F).unwrap_or(0);
        *symbol = ALPHABET[slot];
    }
    code
}

/// Returns whether `code` is exactly [`CODE_LEN`] symbols of [`ALPHABET`].
#[must_use]
pub fn is_valid(code: &[u8]) -> bool {
    code.len() == CODE_LEN && code.iter().all(|byte| ALPHABET.contains(byte))
}

/// Writes `code` as `XXXX-XXXX-XXXX` into `out`.
///
/// `code` is not re-validated; callers pass a code produced by [`encode`] or validated with
/// [`is_valid`].
pub const fn write_grouped(code: &[u8; CODE_LEN], out: &mut [u8; GROUPED_LEN]) {
    let mut source = 0;
    let mut target = 0;
    while source < CODE_LEN {
        if source > 0 && source % GROUP_LEN == 0 {
            out[target] = b'-';
            target += 1;
        }
        out[target] = code[source];
        source += 1;
        target += 1;
    }
}
