//! Host tests for the Setup Mode portal's catch-all DNS responder.
//!
//! Each test builds one query datagram, hands it to `dns::answer` with a reply buffer and parses
//! the reply by its fixed offsets. The catch-all answers every `A` question with the AP's address
//! and every other question with `NOERROR` and no answer.

use bridge_core::dns;

/// The address every `A` answer points at.
const ADDRESS: [u8; 4] = [192, 168, 4, 1];

/// The wire-encoded name `example.com`.
const EXAMPLE: &[u8] = b"\x07example\x03com\x00";

/// The wire-encoded name `ExAmPlE.CoM`, to prove the question is echoed unchanged.
const MIXED_CASE: &[u8] = b"\x07ExAmPlE\x03CoM\x00";

/// The recursion-desired flag.
const RD: u16 = 0x0100;

/// The `A` record type.
const TYPE_A: u16 = 1;

/// The `AAAA` record type.
const TYPE_AAAA: u16 = 28;

/// The `ANY` record type.
const TYPE_ANY: u16 = 255;

/// The Internet class.
const CLASS_IN: u16 = 1;

/// The CHAOS class.
const CLASS_CH: u16 = 3;

/// The size of the appended answer: pointer, type, class, TTL, length and address.
const ANSWER_LEN: usize = 16;

/// Builds a one-question query with the given id, flags, wire-encoded name, type and class.
#[must_use]
fn query(id: u16, flags: u16, name: &[u8], qtype: u16, qclass: u16) -> Vec<u8> {
    let mut datagram = Vec::new();
    datagram.extend_from_slice(&id.to_be_bytes());
    datagram.extend_from_slice(&flags.to_be_bytes());
    datagram.extend_from_slice(&1u16.to_be_bytes());
    datagram.extend_from_slice(&[0, 0, 0, 0, 0, 0]);
    datagram.extend_from_slice(name);
    datagram.extend_from_slice(&qtype.to_be_bytes());
    datagram.extend_from_slice(&qclass.to_be_bytes());
    datagram
}

/// Builds an `RD`-flagged `example.com` query of `qtype` and `qclass`.
#[must_use]
fn example_query(id: u16, qtype: u16, qclass: u16) -> Vec<u8> {
    query(id, RD, EXAMPLE, qtype, qclass)
}

/// Reads a big-endian `u16` at `at`.
#[must_use]
const fn u16_at(bytes: &[u8], at: usize) -> u16 {
    u16::from_be_bytes([bytes[at], bytes[at + 1]])
}

/// A normal `A` query is answered with the AP's address and every header field the spec fixes.
#[test]
fn an_a_query_gets_the_catch_all_address() {
    let datagram = example_query(0xABCD, TYPE_A, CLASS_IN);
    let mut out = [0u8; 512];
    let Some(length) = dns::answer(&datagram, ADDRESS, &mut out) else {
        return;
    };

    let question_len = EXAMPLE.len() + 4;
    assert_eq!(length, 12 + question_len + ANSWER_LEN);
    assert_eq!(&out[0..2], &[0xAB, 0xCD]);
    assert_eq!(u16_at(&out, 2), 0x8180, "QR, RD and RA, RCODE 0");
    assert_eq!(u16_at(&out, 4), 1, "QDCOUNT");
    assert_eq!(u16_at(&out, 6), 1, "ANCOUNT");
    assert_eq!(u16_at(&out, 8), 0, "NSCOUNT");
    assert_eq!(u16_at(&out, 10), 0, "ARCOUNT");
    assert_eq!(&out[12..12 + question_len], &datagram[12..]);

    let answer = 12 + question_len;
    assert_eq!(&out[answer..answer + 2], &[0xC0, 0x0C]);
    assert_eq!(u16_at(&out, answer + 2), TYPE_A);
    assert_eq!(u16_at(&out, answer + 4), CLASS_IN);
    assert_eq!(&out[answer + 6..answer + 10], &dns::TTL_SECONDS.to_be_bytes());
    assert_eq!(u16_at(&out, answer + 10), 4, "RDLENGTH");
    assert_eq!(&out[answer + 12..answer + ANSWER_LEN], &ADDRESS);
}

/// An `AAAA` query gets `NOERROR` with no answer at all.
#[test]
fn an_aaaa_query_gets_no_answer() {
    let datagram = example_query(7, TYPE_AAAA, CLASS_IN);
    let mut out = [0u8; 512];
    let Some(length) = dns::answer(&datagram, ADDRESS, &mut out) else {
        return;
    };

    let question_len = EXAMPLE.len() + 4;
    assert_eq!(length, 12 + question_len, "no answer bytes are appended");
    assert_eq!(u16_at(&out, 6), 0, "ANCOUNT");
    assert_eq!(&out[12..12 + question_len], &datagram[12..]);
}

/// An `ANY` query gets the same `A` answer an `A` query does.
#[test]
fn an_any_query_gets_the_a_answer() {
    let datagram = example_query(9, TYPE_ANY, CLASS_IN);
    let mut out = [0u8; 512];
    let Some(length) = dns::answer(&datagram, ADDRESS, &mut out) else {
        return;
    };

    let answer = 12 + EXAMPLE.len() + 4;
    assert_eq!(length, answer + ANSWER_LEN);
    assert_eq!(u16_at(&out, 6), 1, "ANCOUNT");
    assert_eq!(&out[answer + 12..answer + ANSWER_LEN], &ADDRESS);
}

/// The catch-all `A` record carries [`dns::TTL_SECONDS`] big-endian.
#[test]
fn the_answer_ttl_is_the_catch_all_ttl() {
    let datagram = example_query(1, TYPE_A, CLASS_IN);
    let mut out = [0u8; 512];
    let Some(length) = dns::answer(&datagram, ADDRESS, &mut out) else {
        return;
    };

    let answer = 12 + EXAMPLE.len() + 4;
    assert_eq!(length, answer + ANSWER_LEN);
    assert_eq!(&out[answer + 6..answer + 10], &dns::TTL_SECONDS.to_be_bytes());
}

/// The question name is echoed byte for byte, including its mixed case.
#[test]
fn the_question_name_is_echoed_unchanged() {
    let datagram = query(3, RD, MIXED_CASE, TYPE_A, CLASS_IN);
    let mut out = [0u8; 512];
    let Some(length) = dns::answer(&datagram, ADDRESS, &mut out) else {
        return;
    };

    let question_len = MIXED_CASE.len() + 4;
    assert_eq!(length, 12 + question_len + ANSWER_LEN);
    assert_eq!(&out[12..12 + question_len], &datagram[12..]);
}

/// A query without recursion desired is answered with `RD` clear and `RA` set.
#[test]
fn the_response_flags_copy_only_recursion_desired() {
    let datagram = query(5, 0, EXAMPLE, TYPE_A, CLASS_IN);
    let mut out = [0u8; 512];
    let Some(length) = dns::answer(&datagram, ADDRESS, &mut out) else {
        return;
    };

    assert!(length > 12);
    assert_eq!(u16_at(&out, 2), 0x8080);
}

/// A question in the CHAOS class is not ours to answer.
#[test]
fn a_chaos_class_question_is_refused() {
    let datagram = example_query(1, TYPE_A, CLASS_CH);
    let mut out = [0u8; 512];
    assert!(dns::answer(&datagram, ADDRESS, &mut out).is_none());
}

/// Short and truncated datagrams return `None` without reading past the end.
#[test]
fn short_and_truncated_queries_are_refused() {
    let datagram = example_query(1, TYPE_A, CLASS_IN);
    let mut out = [0u8; 512];

    assert!(dns::answer(&[], ADDRESS, &mut out).is_none());
    assert!(dns::answer(&datagram[..11], ADDRESS, &mut out).is_none());
    assert!(
        dns::answer(&datagram[..datagram.len() - 1], ADDRESS, &mut out).is_none(),
        "QCLASS is cut in half"
    );
    assert!(
        dns::answer(&datagram[..14], ADDRESS, &mut out).is_none(),
        "the name is cut in the middle of a label"
    );
}

/// A response bit set in the query, or a question count other than one, is refused.
#[test]
fn non_queries_are_refused() {
    let mut out = [0u8; 512];
    let answered = example_query(1, TYPE_A, CLASS_IN);

    let mut response = answered.clone();
    response[2] |= 0x80;
    assert!(dns::answer(&response, ADDRESS, &mut out).is_none());

    let mut no_questions = answered;
    no_questions[4..6].copy_from_slice(&0u16.to_be_bytes());
    assert!(dns::answer(&no_questions, ADDRESS, &mut out).is_none());
}

/// A label longer than 63 bytes, or a compressed name, is refused.
#[test]
fn non_plain_question_names_are_refused() {
    let mut out = [0u8; 512];
    let long_label = query(1, RD, b"\x40bad\x00", TYPE_A, CLASS_IN);
    let compressed = query(1, RD, b"\xC0\x0C", TYPE_A, CLASS_IN);

    assert!(dns::answer(&long_label, ADDRESS, &mut out).is_none());
    assert!(dns::answer(&compressed, ADDRESS, &mut out).is_none());
}

/// A reply buffer smaller than the answer returns `None` without writing past it.
#[test]
fn a_small_output_buffer_gets_no_reply() {
    let datagram = example_query(1, TYPE_A, CLASS_IN);
    let mut small = [0u8; 44];
    let mut tiny = [0u8; 12];

    assert!(dns::answer(&datagram, ADDRESS, &mut small).is_none());
    assert!(dns::answer(&datagram, ADDRESS, &mut tiny).is_none());
}
