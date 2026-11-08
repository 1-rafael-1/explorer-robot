//! Async ST7789 TFT display driver.
//!
//! A minimal, hand-rolled driver for the ST7789 (and ST7789-compatible) TFT
//! controller over a write-only SPI bus. Unlike [`mipidsi`], which is blocking
//! and write-through, this driver owns an async SPI bus and a data/command pin,
//! borrows a caller-allocated framebuffer, and exposes an
//! [`embedded_graphics::DrawTarget`] implementation plus an async [`flush`]
//! ([`St7789::flush`]).
//!
//! The driver is generic over [`embedded_hal_async::spi::SpiDevice`] and an
//! async delay, so it is not coupled to any specific HAL. The data/command pin
//! uses the synchronous [`embedded_hal::digital::OutputPin`] trait, since
//! toggling a GPIO is instantaneous.
//!
//! The framebuffer is a caller-allocated byte slice of `width * height * 2`
//! bytes, stored big-endian RGB565 so [`flush`](St7789::flush) can DMA it to
//! the panel verbatim (the ST7789 expects 16-bit pixels MSB-first on the wire).
//!
//! # Example
//!
//! ```ignore
//! let mut display = St7789::new(spi, dc, framebuffer, 320, 240);
//! display.init(&Config::default(), &mut delay).await?;
//! display.clear(Rgb565::BLACK)?;
//! // ...draw with embedded-graphics...
//! display.flush().await?;
//! ```

#![no_std]
#![warn(missing_docs)]

use core::convert::Infallible;

use embedded_graphics::{
    Pixel,
    draw_target::DrawTarget,
    geometry::{Dimensions, Point, Size},
    pixelcolor::{IntoStorage, Rgb565},
    primitives::Rectangle,
};
use embedded_hal::digital::OutputPin;
use embedded_hal_async::{delay::DelayNs, spi::SpiDevice};

// ── MIPI DCS opcodes ──────────────────────────────────────────────────────────

/// Software reset.
const SWRESET: u8 = 0x01;
/// Sleep out.
const SLPOUT: u8 = 0x11;
/// Normal display mode on.
const NORON: u8 = 0x13;
/// Display inversion off.
const INVOFF: u8 = 0x20;
/// Display inversion on.
const INVON: u8 = 0x21;
/// Display on.
const DISPON: u8 = 0x29;
/// Column address set.
const CASET: u8 = 0x2A;
/// Row address set.
const RASET: u8 = 0x2B;
/// Memory write.
const RAMWR: u8 = 0x2C;
/// Memory data access control.
const MADCTL: u8 = 0x36;
/// Interface pixel format.
const COLMOD: u8 = 0x3A;

/// `COLMOD` value for 16 bits per pixel (RGB565).
const COLMOD_RGB565: u8 = 0x55;

/// Driver error.
///
/// Combines errors from the SPI bus and the data/command GPIO.
#[derive(Debug)]
pub enum Error<SpiE, PinE> {
    /// The SPI bus returned an error.
    Spi(SpiE),
    /// The data/command GPIO returned an error.
    Pin(PinE),
}

/// Subpixel order.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub enum ColorOrder {
    /// Red, green, blue subpixel order.
    #[default]
    Rgb,
    /// Blue, green, red subpixel order.
    Bgr,
}

/// Clockwise display rotation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Rotation {
    /// No rotation.
    Deg0,
    /// 90° clockwise.
    Deg90,
    /// 180° clockwise.
    Deg180,
    /// 270° clockwise.
    Deg270,
}

impl Rotation {
    /// The clockwise rotation angle in degrees.
    const fn degree(self) -> i32 {
        match self {
            Self::Deg0 => 0,
            Self::Deg90 => 90,
            Self::Deg180 => 180,
            Self::Deg270 => 270,
        }
    }

    /// Combine this rotation with `other` (adding the two angles, mod 360°).
    const fn rotate(self, other: Self) -> Self {
        match (self.degree() + other.degree()) % 360 {
            0 => Self::Deg0,
            90 => Self::Deg90,
            180 => Self::Deg180,
            _ => Self::Deg270,
        }
    }

    /// Whether this rotation swaps the horizontal and vertical axes.
    const fn is_vertical(self) -> bool {
        matches!(self, Self::Deg90 | Self::Deg270)
    }
}

/// Display orientation: a clockwise rotation plus an optional mirror.
///
/// Mirrors are applied *after* rotation, matching the orientation helpers this
/// API is derived from.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Orientation {
    /// Clockwise rotation.
    pub rotation: Rotation,
    /// Whether the image is mirrored.
    pub mirrored: bool,
}

impl Orientation {
    /// Identity orientation.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            rotation: Rotation::Deg0,
            mirrored: false,
        }
    }

    /// Rotate clockwise by `rotation`.
    #[must_use]
    pub const fn rotate(self, rotation: Rotation) -> Self {
        Self {
            rotation: self.rotation.rotate(rotation),
            mirrored: self.mirrored,
        }
    }

    /// Flip across the horizontal axis.
    #[must_use]
    pub const fn flip_horizontal(self) -> Self {
        if self.rotation.is_vertical() {
            Self {
                rotation: self.rotation.rotate(Rotation::Deg180),
                mirrored: !self.mirrored,
            }
        } else {
            Self {
                rotation: self.rotation,
                mirrored: !self.mirrored,
            }
        }
    }

    /// Flip across the vertical axis.
    #[must_use]
    pub const fn flip_vertical(self) -> Self {
        if self.rotation.is_vertical() {
            Self {
                rotation: self.rotation,
                mirrored: !self.mirrored,
            }
        } else {
            Self {
                rotation: self.rotation.rotate(Rotation::Deg180),
                mirrored: !self.mirrored,
            }
        }
    }
}

impl Default for Orientation {
    fn default() -> Self {
        Self::new()
    }
}

/// Display configuration used during [`St7789::init`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Config {
    /// Subpixel order (MADCTL `BGR` bit).
    pub color_order: ColorOrder,
    /// Display orientation.
    pub orientation: Orientation,
    /// Whether to invert colours (`INVON` vs `INVOFF`).
    pub invert_colors: bool,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            color_order: ColorOrder::Rgb,
            orientation: Orientation::new(),
            invert_colors: false,
        }
    }
}

/// Compute the MADCTL register value from a [`Config`].
fn madctl(config: Config) -> u8 {
    let (reverse_rows, reverse_columns) = match config.orientation.rotation {
        Rotation::Deg0 => (false, false),
        Rotation::Deg90 => (false, true),
        Rotation::Deg180 => (true, true),
        Rotation::Deg270 => (true, false),
    };
    let reverse_columns = reverse_columns ^ config.orientation.mirrored;

    let mut value = 0u8;
    if reverse_rows {
        value |= 1u8 << 7; // MY: row address order
    }
    if reverse_columns {
        value |= 1u8 << 6; // MX: column address order
    }
    if config.orientation.rotation.is_vertical() {
        value |= 1u8 << 5; // MV: row/column exchange
    }
    if config.color_order == ColorOrder::Bgr {
        value |= 1u8 << 3; // BGR: subpixel order
    }
    value
}

/// An asynchronous ST7789 display driver.
///
/// Owns an async SPI bus and a data/command pin, and borrows a caller-allocated
/// framebuffer. A reset pin is deliberately *not* owned here: the caller
/// performs a hardware reset (or relies on the software reset issued during
/// [`init`](Self::init)) before constructing the driver.
pub struct St7789<'fb, SPI, DC> {
    /// The async SPI bus.
    spi: SPI,
    /// The data/command GPIO.
    dc: DC,
    /// The caller-allocated framebuffer, big-endian RGB565.
    framebuffer: &'fb mut [u8],
    /// Framebuffer width in pixels.
    width: u16,
    /// Framebuffer height in pixels.
    height: u16,
}

impl<'fb, SPI, DC, PinE> St7789<'fb, SPI, DC>
where
    SPI: SpiDevice<u8>,
    DC: OutputPin<Error = PinE>,
{
    /// Create a new driver from an SPI device, a data/command pin, a
    /// framebuffer, and the framebuffer's dimensions.
    ///
    /// The framebuffer must hold exactly `width * height * 2` bytes, stored as
    /// big-endian RGB565.
    ///
    /// # Panics
    ///
    /// Panics if `width` or `height` is zero, or if `framebuffer` is not
    /// exactly `width * height * 2` bytes long.
    pub const fn new(spi: SPI, dc: DC, framebuffer: &'fb mut [u8], width: u16, height: u16) -> Self {
        assert!(width != 0 && height != 0, "width and height must be non-zero");
        assert!(
            framebuffer.len() == width as usize * height as usize * 2,
            "framebuffer must hold exactly width * height * 2 bytes"
        );
        Self {
            spi,
            dc,
            framebuffer,
            width,
            height,
        }
    }

    /// Initialise the display.
    ///
    /// Sends a software reset followed by the standard ST7789 bring-up sequence:
    /// sleep out, address mode, inversion, pixel format, normal mode, and
    /// display on, with the required inter-command delays.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI bus fails, or [`Error::Pin`] if the
    /// data/command GPIO fails.
    pub async fn init(&mut self, config: &Config, delay: &mut impl DelayNs) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_command(SWRESET).await?;
        delay.delay_ms(150).await;

        self.write_command(SLPOUT).await?;
        delay.delay_ms(10).await;

        self.write_command_data(MADCTL, &[madctl(*config)]).await?;

        self.write_command(if config.invert_colors { INVON } else { INVOFF })
            .await?;

        self.write_command_data(COLMOD, &[COLMOD_RGB565]).await?;
        delay.delay_ms(10).await;

        self.write_command(NORON).await?;
        delay.delay_ms(10).await;

        self.write_command(DISPON).await?;
        delay.delay_ms(120).await;

        Ok(())
    }

    /// Set the address window for subsequent pixel writes.
    ///
    /// Coordinates are inclusive, matching the ST7789 `CASET`/`RASET` format:
    /// to cover a `w`-wide, `h`-tall region starting at `(x0, y0)`, pass
    /// `(x0, y0, x0 + w - 1, y0 + h - 1)`.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI bus fails, or [`Error::Pin`] if the
    /// data/command GPIO fails.
    pub async fn set_address_window(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    ) -> Result<(), Error<SPI::Error, PinE>> {
        let mut buf = [0u8; 4];

        buf[0..2].copy_from_slice(&x0.to_be_bytes());
        buf[2..4].copy_from_slice(&x1.to_be_bytes());
        self.write_command_data(CASET, &buf).await?;

        buf[0..2].copy_from_slice(&y0.to_be_bytes());
        buf[2..4].copy_from_slice(&y1.to_be_bytes());
        self.write_command_data(RASET, &buf).await?;

        Ok(())
    }

    /// Write raw pixel data at the current address window.
    ///
    /// Sends the `RAMWR` command and then the data verbatim. The caller is
    /// responsible for providing pixels in the byte order the controller
    /// expects (big-endian RGB565 for the ST7789).
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI bus fails, or [`Error::Pin`] if the
    /// data/command GPIO fails.
    pub async fn write_pixels(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_command(RAMWR).await?;
        self.write_data(data).await
    }

    /// Set an address window and write its pixels in one shot.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI bus fails, or [`Error::Pin`] if the
    /// data/command GPIO fails.
    pub async fn fill_region(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
        data: &[u8],
    ) -> Result<(), Error<SPI::Error, PinE>> {
        self.set_address_window(x0, y0, x1, y1).await?;
        self.write_pixels(data).await
    }

    /// Borrow the raw framebuffer bytes (read access).
    pub const fn data(&self) -> &[u8] {
        &*self.framebuffer
    }

    /// Flush the entire framebuffer to the panel.
    ///
    /// # Errors
    ///
    /// Returns [`Error::Spi`] if the SPI bus fails, or [`Error::Pin`] if the
    /// data/command GPIO fails.
    pub async fn flush(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.set_address_window(0, 0, self.width - 1, self.height - 1).await?;
        self.write_framebuffer().await
    }

    /// Consume the driver and return the SPI bus and data/command pin.
    ///
    /// The borrowed framebuffer is simply dropped; the caller retains ownership
    /// of its storage.
    pub fn release(self) -> (SPI, DC) {
        (self.spi, self.dc)
    }

    /// Send a command byte with the data/command line held low.
    async fn write_command(&mut self, command: u8) -> Result<(), Error<SPI::Error, PinE>> {
        self.dc.set_low().map_err(Error::Pin)?;
        self.spi.write(&[command]).await.map_err(Error::Spi)
    }

    /// Send a command byte followed by its data payload.
    async fn write_command_data(&mut self, command: u8, data: &[u8]) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_command(command).await?;
        self.write_data(data).await
    }

    /// Send raw data with the data/command line held high.
    async fn write_data(&mut self, data: &[u8]) -> Result<(), Error<SPI::Error, PinE>> {
        self.dc.set_high().map_err(Error::Pin)?;
        self.spi.write(data).await.map_err(Error::Spi)
    }

    /// Send the `RAMWR` command followed by the full framebuffer payload.
    ///
    /// Uses disjoint field borrows so the framebuffer can be read while the SPI
    /// bus is written in the same statement.
    async fn write_framebuffer(&mut self) -> Result<(), Error<SPI::Error, PinE>> {
        self.write_command(RAMWR).await?;
        self.dc.set_high().map_err(Error::Pin)?;
        self.spi.write(self.framebuffer).await.map_err(Error::Spi)
    }
}

impl<SPI, DC> St7789<'_, SPI, DC> {
    /// Write a single pixel into the framebuffer, clipping out-of-bounds points.
    fn set_pixel(&mut self, x: i32, y: i32, color: Rgb565) {
        let Ok(x) = usize::try_from(x) else { return };
        let Ok(y) = usize::try_from(y) else { return };
        if x >= self.width as usize || y >= self.height as usize {
            return;
        }

        let index = (y * self.width as usize + x) * 2;
        let bytes = color.into_storage().to_be_bytes();
        self.framebuffer[index] = bytes[0];
        self.framebuffer[index + 1] = bytes[1];
    }
}

impl<SPI, DC> Dimensions for St7789<'_, SPI, DC> {
    fn bounding_box(&self) -> Rectangle {
        Rectangle::new(
            Point::new(0, 0),
            Size::new(u32::from(self.width), u32::from(self.height)),
        )
    }
}

impl<SPI, DC> DrawTarget for St7789<'_, SPI, DC> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            self.set_pixel(point.x, point.y, color);
        }
        Ok(())
    }
}
