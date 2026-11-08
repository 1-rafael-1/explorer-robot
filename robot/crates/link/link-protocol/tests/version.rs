//! Guards ADR-0040's rule that the crate version mirrors the wire version.

/// The manifest's major version and the wire's must agree.
#[test]
fn crate_major_mirrors_the_wire_major() {
    assert_eq!(
        env!("CARGO_PKG_VERSION_MAJOR").parse::<u8>(),
        Ok(link_protocol::PROTOCOL_MAJOR)
    );
}

/// The manifest's minor version and the wire's must agree.
#[test]
fn crate_minor_mirrors_the_wire_minor() {
    assert_eq!(
        env!("CARGO_PKG_VERSION_MINOR").parse::<u8>(),
        Ok(link_protocol::PROTOCOL_MINOR)
    );
}
