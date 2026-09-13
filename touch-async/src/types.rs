//! Public domain types shared by the touch driver.

/// One raw, unfiltered reading from the touch panel.
///
/// Every field is the controller's raw 12-bit conversion (`0..=4095`) in the
/// panel's native orientation. No averaging, median filtering, scaling, or
/// normalisation has been applied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct TouchSample {
    /// Raw X-channel conversion.
    pub x: u16,
    /// Raw Y-channel conversion.
    pub y: u16,
    /// Raw Z1-channel conversion, the first axis of the pressure-sense pair.
    pub z1: u16,
    /// Raw Z2-channel conversion, the second axis of the pressure-sense pair.
    pub z2: u16,
}

/// Driver error, parameterised by the SPI and interrupt-wait error types.
///
/// `WaitE` defaults to [`core::convert::Infallible`] so that
/// [`read`](crate::TouchPanel::read), which never waits on an interrupt, can
/// name the same error type with an uninhabited wait error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error<SpiE, WaitE = core::convert::Infallible> {
    /// The SPI transaction reported by the device failed.
    Spi(SpiE),
    /// Waiting on the pen-down interrupt (`PENIRQ`) failed.
    Wait(WaitE),
}
