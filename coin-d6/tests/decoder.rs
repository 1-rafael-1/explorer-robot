//! Host-side integration tests for the COIN-D6 byte-stream decoder.
//!
//! Each fixture is a hand-built packet stream whose 16-bit XOR checksum is
//! computed with the same rule the decoder implements. Expected decoded values
//! are written as literals, not recomputed through the decoder.

use coin_d6::{Decode, Decoder, Scan};

/// Compute the COIN-D6 16-bit XOR checksum over a built packet, matching the
/// vendor reference: XOR `0x55AA`, `CT | LSN<<8`, `FSA`, `LSA`, and per sample
/// `Si_L` plus `(Si_H << 8) | Si_2nd`, excluding the two `CS` bytes.
fn checksum(packet: &[u8]) -> u16 {
    let lsn = usize::from(packet[3]);
    let mut cs: u16 = 0x55AA;
    cs ^= u16::from(packet[2]) | (u16::from(packet[3]) << 8);
    cs ^= u16::from_le_bytes([packet[4], packet[5]]);
    cs ^= u16::from_le_bytes([packet[6], packet[7]]);
    for i in 0..lsn {
        let base = 10 + 3 * i;
        let si_l = packet[base];
        let si_2nd = packet[base + 1];
        let si_h = packet[base + 2];
        cs ^= u16::from(si_l);
        cs ^= (u16::from(si_h) << 8) | u16::from(si_2nd);
    }
    cs
}

/// Build a valid packet from its header fields and samples, inserting the
/// computed 16-bit XOR checksum into the `CS` field. Samples are given in wire
/// order `(Si_L, Si_2nd, Si_H)`.
fn packet(ct: u8, fsa: u16, lsa: u16, samples: &[(u8, u8, u8)]) -> Vec<u8> {
    let mut bytes = Vec::with_capacity(10 + 3 * samples.len());
    bytes.extend_from_slice(&[0xAA, 0x55, ct, samples.len() as u8]);
    bytes.extend_from_slice(&fsa.to_le_bytes());
    bytes.extend_from_slice(&lsa.to_le_bytes());
    bytes.extend_from_slice(&[0, 0]); // checksum placeholder
    for &(si_l, si_2nd, si_h) in samples {
        bytes.extend_from_slice(&[si_l, si_2nd, si_h]);
    }
    let cs = checksum(&bytes);
    bytes[8..10].copy_from_slice(&cs.to_le_bytes());
    bytes
}

#[test]
fn decoder_decodes_distance_and_intensity() {
    let mut decoder = Decoder::new();
    let mut scan = Scan::new();

    // One sample: Si_H = 0x10, Si_2nd = 0x83, Si_L = 0xFC.
    //   distance = 0x10 * 64 + (0x83 >> 2) = 1024 + 32 = 1056 mm
    //   intensity = (0x83 & 0b11) * 64 + (0xFC >> 2) = 192 + 63 = 255
    let start = packet(0x01, 129, 129, &[(0xFC, 0x83, 0x10)]);
    let close = packet(0x01, 129, 129, &[]);

    let mut stream = Vec::new();
    stream.extend_from_slice(&start);
    stream.extend_from_slice(&close);

    let result = decoder.push(&stream, &mut scan);
    assert!(matches!(result, Decode::Revolution));
    assert_eq!(scan.len, 1);
    assert_eq!(scan.points[0].distance_mm, 1056);
    assert_eq!(scan.points[0].intensity, 255);
    assert_eq!(scan.points[0].angle_deg, 1.0_f32);
}

#[test]
fn decoder_resyncs_on_checksum_mismatch_and_recovers() {
    let mut decoder = Decoder::new();
    let mut scan = Scan::new();

    let good_start = packet(0x01, 129, 257, &[(0x00, 0x00, 0x10)]);
    let mut bad = packet(0x00, 257, 385, &[(0x00, 0x00, 0x10)]);
    bad[8] ^= 0x01; // corrupt the checksum

    let mut stream = Vec::new();
    stream.extend_from_slice(&good_start);
    stream.extend_from_slice(&bad);

    let result = decoder.push(&stream, &mut scan);
    assert!(matches!(result, Decode::Resync));
    assert_eq!(scan.len, 0);

    // A fresh, valid revolution must be decoded after resynchronisation.
    let start = packet(0x01, 129, 257, &[(0x00, 0x00, 0x20)]);
    let close = packet(0x01, 257, 257, &[]);
    let mut stream = Vec::new();
    stream.extend_from_slice(&start);
    stream.extend_from_slice(&close);

    let result = decoder.push(&stream, &mut scan);
    assert!(matches!(result, Decode::Revolution));
    assert_eq!(scan.len, 1);
    assert_eq!(scan.points[0].distance_mm, 2048);
}

#[test]
fn decoder_reassembles_packet_split_across_chunks() {
    let mut decoder = Decoder::new();
    let mut scan = Scan::new();

    let start = packet(0x01, 129, 129, &[(0x00, 0x00, 0x10)]);
    let close = packet(0x01, 129, 129, &[]);

    // Split the ring-start packet right after its 4-byte fixed prefix.
    let (head, tail) = start.split_at(4);

    let result = decoder.push(head, &mut scan);
    assert!(matches!(result, Decode::InProgress));

    let mut rest = Vec::new();
    rest.extend_from_slice(tail);
    rest.extend_from_slice(&close);

    let result = decoder.push(&rest, &mut scan);
    assert!(matches!(result, Decode::Revolution));
    assert_eq!(scan.len, 1);
    assert_eq!(scan.points[0].distance_mm, 1024);
}

#[test]
fn decoder_interpolates_angles_across_zero_crossing() {
    let mut decoder = Decoder::new();
    let mut scan = Scan::new();

    // 359.0° -> 1.0° (wraps through 0°): three samples at 359°, 0°, 1°.
    let start = packet(
        0x01,
        45_953, // 359.0°
        129,    // 1.0°
        &[(0x00, 0x00, 0x01), (0x00, 0x00, 0x01), (0x00, 0x00, 0x01)],
    );
    let close = packet(0x01, 129, 129, &[]);

    let mut stream = Vec::new();
    stream.extend_from_slice(&start);
    stream.extend_from_slice(&close);

    let result = decoder.push(&stream, &mut scan);
    assert!(matches!(result, Decode::Revolution));
    assert_eq!(scan.len, 3);
    assert_eq!(scan.points[0].angle_deg, 359.0_f32);
    assert_eq!(scan.points[1].angle_deg, 0.0_f32);
    assert_eq!(scan.points[2].angle_deg, 1.0_f32);
}

#[test]
fn decoder_skips_spin_up_speed_adjust_bytes() {
    let mut decoder = Decoder::new();
    let mut scan = Scan::new();

    let start = packet(0x01, 129, 129, &[(0x00, 0x00, 0x10)]);
    let close = packet(0x01, 129, 129, &[]);

    let mut stream = vec![0xFE, 0xFF, 0xFE, 0xFF];
    stream.extend_from_slice(&start);
    stream.extend_from_slice(&[0xFE, 0xFF]);
    stream.extend_from_slice(&close);

    let result = decoder.push(&stream, &mut scan);
    assert!(matches!(result, Decode::Revolution));
    assert_eq!(scan.len, 1);
    assert_eq!(scan.points[0].distance_mm, 1024);
}

#[test]
fn decoder_assembles_revolution_with_measured_point_count() {
    let mut decoder = Decoder::new();
    let mut scan = Scan::new();

    // Ring-start (1.0°..2.0°), normal (3.0°..4.0°), closing ring-start.
    let p1 = packet(0x01, 129, 257, &[(0x00, 0x00, 0x01), (0x00, 0x00, 0x02)]);
    let p2 = packet(0x00, 385, 513, &[(0x00, 0x04, 0x00), (0xFC, 0x03, 0x00)]);
    let p3 = packet(0x01, 129, 129, &[]);

    let mut stream = Vec::new();
    stream.extend_from_slice(&p1);
    stream.extend_from_slice(&p2);
    stream.extend_from_slice(&p3);

    let result = decoder.push(&stream, &mut scan);
    assert!(matches!(result, Decode::Revolution));
    assert_eq!(scan.len, 4); // measured, not the 400-point maximum
    assert_eq!(scan.points[0].angle_deg, 1.0_f32);
    assert_eq!(scan.points[1].angle_deg, 2.0_f32);
    assert_eq!(scan.points[2].angle_deg, 3.0_f32);
    assert_eq!(scan.points[3].angle_deg, 4.0_f32);
    assert_eq!(scan.points[0].distance_mm, 64);
    assert_eq!(scan.points[1].distance_mm, 128);
    assert_eq!(scan.points[2].distance_mm, 1);
    assert_eq!(scan.points[3].distance_mm, 0);
    assert_eq!(scan.points[3].intensity, 255);
}
