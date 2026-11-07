//! The Setup Mode portal's catch-all DNS responder (UDP 53).
//!
//! A phone that joins the Bridge's Setup Mode AP expects the captive-portal probe to resolve, so
//! every `A` question is answered with the Bridge's own address and every other question gets a
//! `NOERROR` reply with no answer. The codec is `no_std` and allocation-free: it parses exactly
//! one uncompressed question, echoes its bytes and appends a cached `A` record when asked for one.
//!
//! No recursion is attempted, no additional records are added, and an EDNS `OPT` record in the
//! query is ignored by reporting `ARCOUNT = 0`.

/// The DNS server's UDP port.
pub const PORT: u16 = 53;

/// How long the catch-all `A` record may be cached, in seconds.
pub const TTL_SECONDS: u32 = 60;

/// The fixed DNS header length, before the question section.
const HEADER_LEN: usize = 12;

/// The largest single label, in bytes (RFC 1035 section 2.3.4).
const MAX_LABEL_LEN: usize = 63;

/// The largest encoded name, including the root byte (RFC 1035 section 2.3.4).
const MAX_NAME_LEN: usize = 255;

/// The DNS header's question-count offset.
const QDCOUNT_OFFSET: usize = 4;

/// The response bit in the flags word.
const FLAG_QR: u16 = 0x8000;

/// The opcode bits in the flags word.
const OPCODE_MASK: u16 = 0x7800;

/// The recursion-desired bit in the flags word, copied from the query.
const FLAG_RD: u16 = 0x0100;

/// The recursion-available bit in the flags word.
const FLAG_RA: u16 = 0x0080;

/// The `A` record type.
const TYPE_A: u16 = 1;

/// The `ANY` record type.
const TYPE_ANY: u16 = 255;

/// The Internet class.
const CLASS_IN: u16 = 1;

/// The wildcard class.
const CLASS_ANY: u16 = 255;

/// The length of the `A` answer appended to an `A` or `ANY` query: a name pointer, type, class,
/// TTL, `RDLENGTH` and the four address bytes.
const ANSWER_LEN: usize = 2 + 2 + 2 + 4 + 2 + 4;

/// One parsed question.
#[derive(Clone, Copy, Debug)]
struct Question {
    /// The offset just past the question's `QCLASS`.
    end: usize,
    /// The question type from `QTYPE`.
    qtype: u16,
}

/// Builds a DNS response for `query`, pointing every `A` question at `address`.
///
/// Returns the reply's length, or `None` when the datagram is not a parseable query, asks for a
/// class other than `IN` or `ANY`, or `out` is too small.
// The lint counts the inlined parser's temporaries as part of this frame; the function borrows the
// caller's buffer and holds no large arrays.
#[allow(clippy::large_stack_frames)]
pub fn answer(query: &[u8], address: [u8; 4], out: &mut [u8]) -> Option<usize> {
    if query.len() < HEADER_LEN {
        return None;
    }
    let flags = u16::from_be_bytes([query[2], query[3]]);
    if flags & FLAG_QR != 0 || flags & OPCODE_MASK != 0 {
        return None;
    }
    if read_u16(query, QDCOUNT_OFFSET)? != 1 {
        return None;
    }
    let question = parse_question(query)?;
    let question_bytes = query.get(HEADER_LEN..question.end)?;
    let answered = question.qtype == TYPE_A || question.qtype == TYPE_ANY;
    let answer_len = if answered { ANSWER_LEN } else { 0 };
    let length = HEADER_LEN.checked_add(question_bytes.len())?.checked_add(answer_len)?;
    if out.len() < length {
        return None;
    }

    let response_flags = FLAG_QR | (flags & FLAG_RD) | FLAG_RA;
    out[0..2].copy_from_slice(query.get(0..2)?);
    out[2..4].copy_from_slice(&response_flags.to_be_bytes());
    out[4..6].copy_from_slice(&1u16.to_be_bytes());
    let ancount = u16::from(answered);
    out[6..8].copy_from_slice(&ancount.to_be_bytes());
    out[8..10].copy_from_slice(&0u16.to_be_bytes());
    out[10..12].copy_from_slice(&0u16.to_be_bytes());
    let mut index = HEADER_LEN + question_bytes.len();
    out[HEADER_LEN..index].copy_from_slice(question_bytes);

    if answered {
        out[index..index + 2].copy_from_slice(&[0xC0, 0x0C]);
        index += 2;
        out[index..index + 2].copy_from_slice(&TYPE_A.to_be_bytes());
        index += 2;
        out[index..index + 2].copy_from_slice(&CLASS_IN.to_be_bytes());
        index += 2;
        out[index..index + 4].copy_from_slice(&TTL_SECONDS.to_be_bytes());
        index += 4;
        out[index..index + 2].copy_from_slice(&4u16.to_be_bytes());
        index += 2;
        out[index..index + 4].copy_from_slice(&address);
        index += 4;
    }
    Some(index)
}

/// Parses the single question at offset 12.
///
/// Rejects compressed names, labels longer than 63 bytes, names longer than 255 bytes and a
/// `QCLASS` other than `IN` or `ANY`.
fn parse_question(query: &[u8]) -> Option<Question> {
    let mut index = HEADER_LEN;
    let mut name_len = 0usize;
    loop {
        let length = usize::from(*query.get(index)?);
        index = index.checked_add(1)?;
        if length == 0 {
            break;
        }
        if length > MAX_LABEL_LEN {
            return None;
        }
        name_len = name_len.checked_add(length.checked_add(1)?)?;
        if name_len.checked_add(1)? > MAX_NAME_LEN {
            return None;
        }
        index = index.checked_add(length)?;
        if index > query.len() {
            return None;
        }
    }
    let qtype = read_u16(query, index)?;
    let qclass = read_u16(query, index.checked_add(2)?)?;
    if qclass != CLASS_IN && qclass != CLASS_ANY {
        return None;
    }
    Some(Question {
        end: index.checked_add(4)?,
        qtype,
    })
}

/// Reads a big-endian `u16` at `at`, or `None` when it runs past the datagram.
fn read_u16(bytes: &[u8], at: usize) -> Option<u16> {
    let high = *bytes.get(at)?;
    let low = *bytes.get(at.checked_add(1)?)?;
    Some(u16::from_be_bytes([high, low]))
}
