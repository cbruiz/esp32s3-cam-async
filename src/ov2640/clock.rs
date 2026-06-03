//! Sensor clocking configuration — the knobs that set the capture / frame rate.

use esp_hal::time::Rate;

/// Clocking parameters that together determine how fast the OV2640 streams
/// pixels (and therefore the frame rate).
///
/// Three knobs are in play:
///
/// 1. **XCLK** ([`xclk`](Self::xclk)) — the master clock the ESP32-S3 drives onto
///    the sensor's MCLK pin. It also seeds the LCD-CAM capture clock
///    (`160 MHz / xclk`, integer division). Raising XCLK is the largest and
///    safest frame-rate lever; the OV2640 runs cleanly up to ~24 MHz.
/// 2. **CLKRC** ([`clkrc`](Self::clkrc)) — the sensor's internal clock
///    pre-scalar (reg `0x11`, bits `[5:0]`). `0` applies no pre-scale (fastest);
///    larger values divide the PLL output down.
/// 3. **R_DVP_SP** ([`dvp_pclk`](Self::dvp_pclk)) — the DVP output clock divider
///    (reg `0xD3`). Smaller divisors raise PCLK (more frames) at the cost of a
///    higher AFIFO byte-loss risk on long ribbon cables. `None` keeps the
///    validated per-format default.
///
/// Build one of the [`BALANCED`](Self::BALANCED) / [`HIGH_FPS`](Self::HIGH_FPS)
/// presets, or set the fields directly. Feed [`xclk`](Self::xclk) into the
/// LCD-CAM [`Config`](esp_hal::lcd_cam::cam::Config) and pass the whole
/// `ClockConfig` to the sensor via [`Ov2640Config`](super::Ov2640Config) so both
/// ends agree on one rate.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct ClockConfig {
    /// XCLK / MCLK frequency fed to the sensor and the LCD-CAM clock divider.
    pub xclk: Rate,
    /// CLKRC (reg `0x11`) internal clock pre-scalar, bits `[5:0]`. `0` = no
    /// pre-scale (fastest).
    pub clkrc: u8,
    /// R_DVP_SP (reg `0xD3`) DVP clock divider override. `None` keeps the proven
    /// per-format default (RGB565/YUV422: auto + ÷2; JPEG: the reference value).
    pub dvp_pclk: Option<u8>,
}

impl ClockConfig {

    /// New from custom options
    pub const fn new(xclk: Rate, clkrc: u8, dvp_pclk: Option<u8>) -> Self {
        Self {
            xclk,
            clkrc,
            dvp_pclk,
        }
    }

    /// Conservative clocking — the long-standing validated default:
    /// **10 MHz** XCLK, no PLL pre-scale, per-format DVP divider.
    pub const BALANCED: Self = Self::new(Rate::from_mhz(10), 0, None);

    /// Higher frame rate — **20 MHz** XCLK (the value proven by the
    /// `http_mjpeg_server` example), no PLL pre-scale, per-format DVP divider.
    /// Roughly doubles the BALANCED pixel clock; verify on your wiring, as the
    /// faster PCLK is more sensitive to ribbon-cable signal integrity.
    pub const HIGH_FPS: Self = Self::new(Rate::from_mhz(20), 0, None);

    /// Higher frame rate — **24 MHz** XCLK, no PLL pre-scale, per-format DVP divider.
    /// Roughly doubles the BALANCED pixel clock; verify on your wiring, as the
    /// faster PCLK is more sensitive to ribbon-cable signal integrity.
    pub const MAX_FPS: Self = Self::new(Rate::from_mhz(20), 0, None);

    /// The XCLK frequency, for building the LCD-CAM
    /// [`Config`](esp_hal::lcd_cam::cam::Config) with
    /// [`with_frequency`](esp_hal::lcd_cam::cam::Config::with_frequency).
    pub const fn xclk(self) -> Rate {
        self.xclk
    }
}

impl Default for ClockConfig {
    /// [`BALANCED`](Self::BALANCED) — preserves the historical behaviour.
    fn default() -> Self {
        Self::BALANCED
    }
}
