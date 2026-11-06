//! Guards ADR-0040's rule that the crate version mirrors the preamble version.

/// The manifest's major version and the preamble's must agree.
#[test]
fn crate_major_mirrors_the_preamble_major() {
    assert_eq!(
        env!("CARGO_PKG_VERSION_MAJOR").parse::<u8>(),
        Ok(bridge_net::preamble::MAJOR)
    );
}

/// The manifest's minor version and the preamble's must agree.
#[test]
fn crate_minor_mirrors_the_preamble_minor() {
    assert_eq!(
        env!("CARGO_PKG_VERSION_MINOR").parse::<u8>(),
        Ok(bridge_net::preamble::MINOR)
    );
}
