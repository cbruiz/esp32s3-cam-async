//! OV2640 sensor configuration types.

use super::clock::ClockConfig;

/// Default I2C/SCCB address for the OV2640 sensor.
pub const OV2640_ADDR: u8 = 0x30;

/// OV2640 output pixel format.
///
/// Each format dictates the LCD-CAM input conversion mode
/// ([`ConverterMode`](crate::ConverterMode)) the driver must use — see the table
/// in the crate's `docs/OV2640.md`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PixelFormat {
    /// RGB565 — 16 bpp, `[HB, LB]` per pixel (BSYM=0, big-endian):
    /// `v = (HB << 8) | LB`, fields `R[15:11] G[10:5] B[4:0]`.
    /// Line width: `resolution.line_bytes()` bytes.
    /// Needs [`ConverterMode::Bypass`](crate::ConverterMode::Bypass).
    Rgb565,
    /// YUV422 in **YUYV** byte order `[Y0, Cb, Y1, Cr]` per 2-pixel macroblock
    /// (BSYM=0): Y in the even bytes, Cb/Cr (≈128) shared by the pair. This is
    /// the OV2640 native order — it is **not** UYVY (`[Cb, Y0, Cr, Y1]`).
    /// Line width: `resolution.line_bytes()` bytes.
    /// Needs [`ConverterMode::Yuv422Passthrough`](crate::ConverterMode::Yuv422Passthrough).
    Yuv422,
    /// JPEG compressed output (opaque JFIF byte stream, SOI…EOI).
    ///
    /// `quality` maps to the OV2640 `QS` quantisation-scale register.
    /// Lower → better quality / larger files. Typical range: `4`–`63`.
    /// Needs [`ConverterMode::Bypass`](crate::ConverterMode::Bypass).
    Jpeg { quality: u8 },
}

impl Default for PixelFormat {
    fn default() -> Self {
        Self::Rgb565
    }
}

/// OV2640 output resolution (sensor window + DSP scaling).
///
/// All variants use the SVGA (800×600) sensor window as the base.
/// Sub-SVGA resolutions are achieved with the hardware DCW downscaler
/// followed by the zoom scaler.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Resolution {
    /// 160 × 120 — QQVGA. 5× downscale from the 800×600 sensor window.
    Qqvga,
    /// 320 × 240 — QVGA. 2.5× downscale from 800×600 via DCW + zoom. **Default.**
    Qvga,
    /// 640 × 480 — VGA. Clean 1.25× downscale from the 800×600 sensor window,
    /// which the zoom scaler handles without the shear/colour artefacts that a
    /// 2.5× (QVGA) downscale produces. Matches the proven esp-hal reference.
    Vga,
    /// 800 × 600 — native SVGA sensor output with no downscaling.
    Svga,
    /// 1600 × 1200 — native UXGA, the sensor's maximum, with no downscaling.
    /// JPEG frames are large (≈100–300 KB); size buffers accordingly.
    Uxga,
}

impl Default for Resolution {
    fn default() -> Self {
        Self::Qvga
    }
}

impl Resolution {
    /// Output dimensions `(width, height)` in pixels.
    pub const fn dims(self) -> (u16, u16) {
        match self {
            Resolution::Qqvga => (160, 120),
            Resolution::Qvga  => (320, 240),
            Resolution::Vga   => (640, 480),
            Resolution::Svga  => (800, 600),
            Resolution::Uxga  => (1600, 1200),
        }
    }

    /// Bytes per scan line for packed pixel formats (width × 2 bytes/pixel).
    ///
    /// Not meaningful for [`PixelFormat::Jpeg`], where the compressed frame
    /// size is variable.
    pub const fn line_bytes(self) -> usize {
        let (w, _) = self.dims();
        w as usize * 2
    }
}

/// OV2640 sensor initialisation configuration.
///
/// Construct with [`Default::default`] and override only what you need:
///
/// ```rust,ignore
/// use esp32s3_cam_async::ov2640::{Ov2640Config, PixelFormat, Resolution};
///
/// // QVGA RGB565 (all defaults)
/// let cfg = Ov2640Config::default();
///
/// // QVGA JPEG at quality 12
/// let cfg = Ov2640Config {
///     format: PixelFormat::Jpeg { quality: 12 },
///     ..Default::default()
/// };
///
/// // SVGA YUV422
/// let cfg = Ov2640Config {
///     format: PixelFormat::Yuv422,
///     resolution: Resolution::Svga,
///     ..Default::default()
/// };
///
/// ov2640::check_id(&mut i2c, &cfg).expect("OV2640 not found");
/// ov2640::init(&mut i2c, &cfg).await.expect("OV2640 init failed");
/// ```
#[derive(Debug, Clone, Copy)]
pub struct Ov2640Config {
    /// I2C/SCCB device address. Default: [`OV2640_ADDR`] (`0x30`).
    pub i2c_addr: u8,
    /// Output pixel format.
    pub format: PixelFormat,
    /// Capture resolution.
    pub resolution: Resolution,
    /// Sensor clocking (XCLK + internal dividers) that sets the capture / frame
    /// rate. Only the divider fields ([`ClockConfig::clkrc`] / [`dvp_pclk`]) are
    /// programmed into the sensor here; feed [`ClockConfig::xclk`] into the
    /// LCD-CAM [`Config`](esp_hal::lcd_cam::cam::Config) so both ends share one
    /// rate. Defaults to [`ClockConfig::BALANCED`].
    ///
    /// [`dvp_pclk`]: ClockConfig::dvp_pclk
    pub clock: ClockConfig,
}

impl Default for Ov2640Config {
    fn default() -> Self {
        Self {
            i2c_addr: OV2640_ADDR,
            format: PixelFormat::default(),
            resolution: Resolution::default(),
            clock: ClockConfig::default(),
        }
    }
}
