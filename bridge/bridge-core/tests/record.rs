//! Host tests for the credential record (ADR-0036).
//!
//! A record round-trips exactly, and every layout or integrity failure is a typed error the
//! firmware turns into "unprovisioned". The tests use the real `portal::Credentials` type so the
//! store and the portal cannot disagree about a field's shape.

use bridge_core::{
    CredentialRecord, RecordError, crc16,
    portal::{Credentials, MAX_PASSPHRASE_BYTES, MAX_SSID_BYTES, MIN_PASSPHRASE_BYTES},
    record::{MAGIC, MAX_ENCODED_LEN, MIN_ENCODED_LEN, VERSION},
};

/// The token used by most tests.
const TOKEN: [u8; 12] = *b"ABCD2345EFGH";

/// Builds a credential pair from raw slices.
fn credentials(ssid: &[u8], passphrase: &[u8]) -> Credentials {
    let mut credentials = Credentials {
        ssid: [0u8; MAX_SSID_BYTES],
        ssid_len: 0,
        passphrase: [0u8; MAX_PASSPHRASE_BYTES],
        passphrase_len: 0,
    };
    credentials.ssid[..ssid.len()].copy_from_slice(ssid);
    credentials.ssid_len = u8::try_from(ssid.len()).unwrap_or(0);
    credentials.passphrase[..passphrase.len()].copy_from_slice(passphrase);
    credentials.passphrase_len = u8::try_from(passphrase.len()).unwrap_or(0);
    credentials
}

/// Encodes into the firmware-sized scratch buffer, returning the record bytes.
fn encode(record: &CredentialRecord) -> Vec<u8> {
    let mut buffer = [0u8; MAX_ENCODED_LEN];
    let Ok(len) = record.encode_into(&mut buffer) else {
        return Vec::new();
    };
    buffer[..len].to_vec()
}

/// Recomputes the trailing CRC after a test mutates a body byte.
fn refresh_crc(bytes: &mut [u8]) {
    let end = bytes.len() - 2;
    let crc = crc16(&bytes[..end]);
    bytes[end..].copy_from_slice(&crc.to_le_bytes());
}

/// A provisioned record round-trips byte-for-byte.
#[test]
fn a_provisioned_record_round_trips() {
    let record = CredentialRecord {
        credentials: Some(credentials(b"PandorasMesh", b"correct-horse-battery")),
        token: TOKEN,
    };
    let bytes = encode(&record);
    assert_ne!(bytes, Vec::new());
    assert_eq!(CredentialRecord::decode(&bytes), Ok(record));
    assert!(record.provisioned());
}

/// A token-only record round-trips and stays unprovisioned.
#[test]
fn a_token_only_record_round_trips() {
    let record = CredentialRecord::token_only(TOKEN);
    let bytes = encode(&record);
    assert_eq!(bytes.len(), MIN_ENCODED_LEN, "no credential fields are written");
    assert_eq!(CredentialRecord::decode(&bytes), Ok(record));
    assert!(!record.provisioned());
}

/// The longest legal field pair fits the declared maximum.
#[test]
fn the_longest_fields_fit_the_declared_maximum() {
    let ssid = [b'S'; MAX_SSID_BYTES];
    let passphrase = [b'p'; MAX_PASSPHRASE_BYTES];
    let record = CredentialRecord {
        credentials: Some(credentials(&ssid, &passphrase)),
        token: TOKEN,
    };
    let bytes = encode(&record);
    assert_eq!(bytes.len(), MAX_ENCODED_LEN);
    assert_eq!(
        CredentialRecord::decode(&bytes).map(|decoded| decoded.credentials.is_some()),
        Ok(true)
    );
}

/// The fixed prologue and the token slot are where ADR-0036 says they are.
#[test]
fn the_layout_starts_with_the_magic_version_and_flags() {
    let record = CredentialRecord {
        credentials: Some(credentials(b"net", b"password")),
        token: TOKEN,
    };
    let bytes = encode(&record);
    assert_eq!(&bytes[..4], &MAGIC);
    assert_eq!(bytes[4], VERSION);
    assert_eq!(bytes[5] & 0x01, 0x01, "the has-credentials flag");
    assert_eq!(bytes[6], 3, "the SSID length");
    let token_start = bytes.len() - 14;
    assert_eq!(&bytes[token_start..token_start + 12], TOKEN.as_slice());
}

/// A wrong magic is rejected.
#[test]
fn a_wrong_magic_is_rejected() {
    let mut bytes = encode(&CredentialRecord::token_only(TOKEN));
    bytes[0] = b'X';
    assert_eq!(CredentialRecord::decode(&bytes), Err(RecordError::Magic));
}

/// A wrong version is rejected, so a changed layout fails closed.
#[test]
fn a_wrong_version_is_rejected() {
    let mut bytes = encode(&CredentialRecord::token_only(TOKEN));
    bytes[4] = VERSION.wrapping_add(1);
    assert_eq!(CredentialRecord::decode(&bytes), Err(RecordError::Version));
}

/// A damaged body fails the CRC.
#[test]
fn a_bad_crc_is_rejected() {
    let mut bytes = encode(&CredentialRecord {
        credentials: Some(credentials(b"net", b"password")),
        token: TOKEN,
    });
    let last = bytes.len() - 1;
    bytes[last] ^= 0xFF;
    assert_eq!(CredentialRecord::decode(&bytes), Err(RecordError::Crc));
}

/// A short record, a long record and a length that disagrees with its fields are all rejected.
#[test]
fn structural_length_mismatches_are_rejected() {
    let bytes = encode(&CredentialRecord::token_only(TOKEN));
    assert_eq!(
        CredentialRecord::decode(&bytes[..MIN_ENCODED_LEN - 1]),
        Err(RecordError::Length)
    );

    let mut too_long = bytes.clone();
    too_long.push(0);
    assert_eq!(CredentialRecord::decode(&too_long), Err(RecordError::Length));

    let mut wrong_ssid_length = bytes;
    wrong_ssid_length[6] = 1;
    assert!(matches!(
        CredentialRecord::decode(&wrong_ssid_length),
        Err(RecordError::Length | RecordError::Crc)
    ));
}

/// A reserved flag bit and a flag that disagrees with the credential fields are rejected.
#[test]
fn inconsistent_flags_are_rejected() {
    let mut reserved = encode(&CredentialRecord::token_only(TOKEN));
    reserved[5] = 0x02;
    refresh_crc(&mut reserved);
    assert_eq!(CredentialRecord::decode(&reserved), Err(RecordError::Flags));

    let mut claims_credentials = encode(&CredentialRecord::token_only(TOKEN));
    claims_credentials[5] = 0x01;
    refresh_crc(&mut claims_credentials);
    assert_eq!(CredentialRecord::decode(&claims_credentials), Err(RecordError::Flags));

    let mut hides_credentials = encode(&CredentialRecord {
        credentials: Some(credentials(b"net", b"password")),
        token: TOKEN,
    });
    hides_credentials[5] = 0x00;
    refresh_crc(&mut hides_credentials);
    assert_eq!(CredentialRecord::decode(&hides_credentials), Err(RecordError::Flags));
}

/// A token that is not twelve alphabet characters is rejected.
#[test]
fn a_bad_token_is_rejected() {
    let mut bytes = encode(&CredentialRecord::token_only(TOKEN));
    let token_start = bytes.len() - 14;
    bytes[token_start] = b'I';
    refresh_crc(&mut bytes);
    assert_eq!(CredentialRecord::decode(&bytes), Err(RecordError::Token));
}

/// The encoder refuses credential fields outside the WPA2-PSK range.
#[test]
fn the_encoder_refuses_bad_credentials() {
    let mut buffer = [0u8; MAX_ENCODED_LEN];
    let empty_ssid = CredentialRecord {
        credentials: Some(credentials(b"", b"password")),
        token: TOKEN,
    };
    assert_eq!(empty_ssid.encode_into(&mut buffer), Err(RecordError::Credentials));

    let short_pass = CredentialRecord {
        credentials: Some(credentials(b"net", &[b'p'; MIN_PASSPHRASE_BYTES - 1])),
        token: TOKEN,
    };
    assert_eq!(short_pass.encode_into(&mut buffer), Err(RecordError::Credentials));

    let long_pass = CredentialRecord {
        credentials: Some({
            let mut credentials = credentials(b"net", b"password");
            credentials.passphrase_len = u8::try_from(MAX_PASSPHRASE_BYTES + 1).unwrap_or(u8::MAX);
            credentials
        }),
        token: TOKEN,
    };
    assert_eq!(long_pass.encode_into(&mut buffer), Err(RecordError::Credentials));

    let long_ssid = CredentialRecord {
        credentials: Some({
            let mut credentials = credentials(b"net", b"password");
            credentials.ssid_len = u8::try_from(MAX_SSID_BYTES + 1).unwrap_or(u8::MAX);
            credentials
        }),
        token: TOKEN,
    };
    assert_eq!(long_ssid.encode_into(&mut buffer), Err(RecordError::Credentials));
}

/// The encoder validates the token and refuses an undersized output.
#[test]
fn the_encoder_validates_the_token_and_the_buffer() {
    let mut bad_token = CredentialRecord::token_only(TOKEN);
    bad_token.token[0] = b'l';
    let mut buffer = [0u8; MAX_ENCODED_LEN];
    assert_eq!(bad_token.encode_into(&mut buffer), Err(RecordError::Token));

    let mut small = [0u8; MAX_ENCODED_LEN - 1];
    assert_eq!(
        CredentialRecord::token_only(TOKEN).encode_into(&mut small),
        Err(RecordError::Length)
    );
}
