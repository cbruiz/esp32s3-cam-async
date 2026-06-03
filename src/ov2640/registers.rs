//! OV2640 register address and bit-field constants.
//!
//! Complete register map for the OV2640 image sensor. Not every constant is
//! referenced by the current init sequences; the full map is retained for
//! reference and to support future modes (UXGA, JPEG, raw Bayer, etc.).
//!
//! ## Bank model
//!
//! The OV2640 exposes two register banks through a single I2C address:
//!
//! - **DSP bank** ([`BANK_DSP`] = `0x00`) — image processing, scaling, output format.
//! - **Sensor bank** ([`BANK_SENSOR`] = `0x01`) — analog/timing controls, PLL, window.
//!
//! Switch between banks by writing [`BANK_DSP`] or [`BANK_SENSOR`] to [`BANK_SEL`].
//!
//! ## Reference
//!
//! OV2640 datasheet, Omnivision Technologies. Register addresses and bit-field
//! values are derived from the datasheet and the
//! [esp32-camera](https://github.com/espressif/esp32-camera) reference driver.
#![allow(dead_code)]

// ── Bank selector ────────────────────────────────────────────────────────────

/// Bank selector register. Write [`BANK_DSP`] or [`BANK_SENSOR`] here to
/// switch the active register bank.
pub const BANK_SEL: u8 = 0xFF;

/// Value for [`BANK_SEL`]: select the **DSP bank** (image processing,
/// scaling, output format).
pub const BANK_DSP: u8 = 0x00;

/// Value for [`BANK_SEL`]: select the **Sensor bank** (analog controls,
/// PLL, timing, window).
pub const BANK_SENSOR: u8 = 0x01;

// ── Common Control 7 — resolution and reset (Sensor bank) ────────────────────

/// **COM7** — Common Control 7 (Sensor bank).
/// Controls the resolution preset and triggers a soft-reset.
pub const COM7: u8 = 0x12;

/// [`COM7`] bit 7: software reset. Self-clearing; returns all registers to
/// their power-on defaults.
pub const COM7_SRST: u8 = 0x80;

/// [`COM7`] resolution preset: CIF (352 × 288 sensor window).
pub const COM7_RES_CIF: u8 = 0x20;

/// [`COM7`] resolution preset: SVGA (800 × 600 sensor window).
pub const COM7_RES_SVGA: u8 = 0x40;

/// [`COM7`] resolution preset: UXGA (1600 × 1200 sensor window).
pub const COM7_RES_UXGA: u8 = 0x00;

// ── Clock and timing (Sensor bank) ───────────────────────────────────────────

/// **CLKRC** — Internal clock pre-scalar (Sensor bank).
/// Bits\[5:0\] divide the PLL output to produce the internal pixel clock.
pub const CLKRC: u8 = 0x11;

/// **R_DVP_SP** — DVP output clock divider (DSP bank).
/// Controls the PCLK frequency on the DVP bus relative to the system clock.
pub const R_DVP_SP: u8 = 0xD3;

/// [`R_DVP_SP`] bit 7: enable automatic DVP clock-speed selection.
pub const R_DVP_SP_AUTO_MODE: u8 = 0x80;

// ── Common controls (Sensor bank) ────────────────────────────────────────────

/// **COM1** — Common Control 1 (Sensor bank).
/// Dummy-row count and AEC exposure low-bits extension.
pub const COM1: u8 = 0x03;

/// **COM2** — Common Control 2 (Sensor bank).
/// Output drive strength and sleep mode.
pub const COM2: u8 = 0x09;

/// **COM4** — Common Control 4 (Sensor bank).
/// Average-based AEC/AGC window. Use the datasheet default.
pub const COM4: u8 = 0x0D;

/// **COM8** — Common Control 8 (Sensor bank).
/// Enables AEC, AGC, and the anti-banding filter.
pub const COM8: u8 = 0x13;

/// **COM9** — Common Control 9 (Sensor bank).
/// AGC gain ceiling and freeze controls.
pub const COM9: u8 = 0x14;

// ── Miscellaneous sensor registers ───────────────────────────────────────────

/// **REG04** — Register 04 (Sensor bank).
/// Horizontal / vertical mirror and HREF polarity.
pub const REG04: u8 = 0x04;

/// **ARCOM2** — Reserved sensor register. Set to the datasheet default.
pub const ARCOM2: u8 = 0x34;

/// **AEW** — AEC upper limit (stable-exposure window high threshold).
pub const AEW: u8 = 0x24;

/// **AEB** — AEC lower limit (stable-exposure window low threshold).
pub const AEB: u8 = 0x25;

/// **VV** — Fast-mode large-step range thresholds for AEC.
pub const VV: u8 = 0x26;

/// **HISTO_LOW** — Histogram low threshold for the auto-exposure window.
pub const HISTO_LOW: u8 = 0x61;

/// **HISTO_HIGH** — Histogram high threshold for the auto-exposure window.
pub const HISTO_HIGH: u8 = 0x62;

// ── Anti-banding filters (Sensor bank) ───────────────────────────────────────

/// **BD50** — Anti-banding filter step size for 50 Hz mains illumination.
pub const BD50: u8 = 0x4F;

/// **BD60** — Anti-banding filter step size for 60 Hz mains illumination.
pub const BD60: u8 = 0x50;

// ── Sensor window (Sensor bank) ──────────────────────────────────────────────

/// **HSTART** — Horizontal window start, most-significant 8 bits.
/// LSBs are in [`REG32`] bits\[5:3\].
pub const HSTART: u8 = 0x17;

/// **HSTOP** — Horizontal window stop, most-significant 8 bits.
/// LSBs are in [`REG32`] bits\[2:0\].
pub const HSTOP: u8 = 0x18;

/// **VSTART** — Vertical window start, most-significant 8 bits.
pub const VSTART: u8 = 0x19;

/// **VSTOP** — Vertical window stop, most-significant 8 bits.
pub const VSTOP: u8 = 0x1A;

/// **REG32** — Horizontal window fine-tuning register.
/// Packs the 3 LSBs of [`HSTART`] and [`HSTOP`].
pub const REG32: u8 = 0x32;

/// [`REG32`] value for the CIF sensor window.
pub const REG32_CIF: u8 = 0x89;

/// [`REG32`] value for the UXGA sensor window.
pub const REG32_UXGA: u8 = 0x36;

// ── DSP registers (DSP bank) ─────────────────────────────────────────────────

/// **MC_BIST** — MCU BIST control (DSP bank).
/// Used during boot ROM initialisation; write [`MC_BIST_RESET`] then
/// [`MC_BIST_BOOT_ROM_SEL`] as part of the startup sequence.
pub const MC_BIST: u8 = 0xF9;

/// **RESET** — DSP module reset register (DSP bank).
/// Each bit holds one DSP sub-module in reset. Write `0x00` to release all.
/// See [`RESET_JPEG`] and [`RESET_DVP`] for individual bits.
pub const RESET: u8 = 0xE0;

/// **CTRL0** — DSP Control 0 (DSP bank).
/// Enables individual output signal paths. See [`CTRL0_RAW_EN`],
/// [`CTRL0_RGB_EN`], [`CTRL0_YUV422_EN`].
pub const CTRL0: u8 = 0xC2;

/// **CTRL1** — DSP Control 1 (DSP bank).
/// Fine-grained enable bits for DSP sub-modules (color-bar test, SDE, etc.).
pub const CTRL1: u8 = 0xC3;

/// **CTRL2** — DSP Control 2 (DSP bank).
/// DCW (hardware down-scaler) enable. See [`CTRL2_DCW_EN`].
pub const CTRL2: u8 = 0x86;

/// **CTRL3** — DSP Control 3 (DSP bank).
/// WPC (white-pixel cancellation) and related controls. See [`CTRL3_WPC_EN`].
pub const CTRL3: u8 = 0x87;

/// **CTRLI** — DSP image-pipeline control (DSP bank).
/// Lens correction and scaler enable. See [`CTRLI_LP_DP`].
pub const CTRLI: u8 = 0x50;

/// **IMAGE_MODE** — Output image mode (DSP bank).
/// Selects JPEG vs. raw DVP, colour format, and byte-swap.
/// See [`IMAGE_MODE_JPEG_EN`], [`IMAGE_MODE_YUV422`], [`IMAGE_MODE_RGB565`],
/// [`IMAGE_MODE_HREF_VSYNC`], [`IMAGE_MODE_BSYM`].
pub const IMAGE_MODE: u8 = 0xDA;

/// **R_BYPASS** — DSP bypass control (DSP bank).
/// Routes sensor data either through the DSP ([`R_BYPASS_DSP_EN`]) or
/// directly to the DVP output ([`R_BYPASS_DSP_BYPAS`]).
pub const R_BYPASS: u8 = 0x05;

/// **BPADDR** — Bad-pixel table address register (DSP bank).
/// Set this to the index before reading or writing [`BPDATA`].
pub const BPADDR: u8 = 0x7C;

/// **BPDATA** — Bad-pixel table data register (DSP bank).
/// Read/write the entry at the index set by [`BPADDR`].
pub const BPDATA: u8 = 0x7D;

/// **QS** — JPEG quantisation scale (DSP bank).
/// Lower value → higher JPEG quality (larger output). Typical range: 4–63.
pub const QS: u8 = 0x44;

// ── DSP scaling / windowing (DSP bank) ───────────────────────────────────────

/// **HSIZE8** — Input horizontal size in 8-pixel units (DSP bank).
/// Must be set to `sensor_width / 8`. In the `real/4` register domain this
/// equals `max_x >> 1` (where `max_x = sensor_width / 4`).
pub const HSIZE8: u8 = 0xC0;

/// **VSIZE8** — Input vertical size in 8-pixel units (DSP bank).
/// Must be set to `sensor_height / 8`. In the `real/4` register domain this
/// equals `max_y >> 1`.
pub const VSIZE8: u8 = 0xC1;

/// **SIZEL** — Size low-bits register (DSP bank).
/// Packs the overflow bits of [`HSIZE`], [`VSIZE`], [`ZMOW`], and [`ZMOH`]
/// that do not fit in their 8-bit registers.
pub const SIZEL: u8 = 0x8C;

/// **HSIZE** — Output horizontal size, bits\[7:0\] (in `real/4` units, DSP bank).
pub const HSIZE: u8 = 0x51;

/// **VSIZE** — Output vertical size, bits\[7:0\] (in `real/4` units, DSP bank).
pub const VSIZE: u8 = 0x52;

/// **XOFFL** — Horizontal crop offset low byte (in `real/4` units, DSP bank).
pub const XOFFL: u8 = 0x53;

/// **YOFFL** — Vertical crop offset low byte (in `real/4` units, DSP bank).
pub const YOFFL: u8 = 0x54;

/// **VHYX** — High-bit packing register (DSP bank).
/// Packs overflow bits of [`VSIZE`], [`HSIZE`], [`XOFFL`], and [`YOFFL`].
pub const VHYX: u8 = 0x55;

/// **TEST** — Test register / high bit of [`HSIZE8`] (DSP bank).
pub const TEST: u8 = 0x57;

/// **ZMOW** — Zoom output width, bits\[7:0\] (in `real/4` units, DSP bank).
pub const ZMOW: u8 = 0x5A;

/// **ZMOH** — Zoom output height, bits\[7:0\] (in `real/4` units, DSP bank).
pub const ZMOH: u8 = 0x5B;

/// **ZMHH** — Zoom high-bit packing register (DSP bank).
/// Packs the overflow bits of [`ZMOW`] and [`ZMOH`].
pub const ZMHH: u8 = 0x5C;

// ── Bit-field values for Sensor bank registers ────────────────────────────────

/// [`COM2`]: output drive strength 3× (for short cables).
pub const COM2_OUT_DRIVE_3X: u8 = 0x02;

/// [`COM2`]: output drive strength 4× (recommended for boards with long flex cables).
pub const COM2_OUT_DRIVE_4X: u8 = 0x03;

/// [`REG04`] power-on default (mirror=0, flip=0, HREF polarity normal).
pub const REG04_DEFAULT: u8 = 0x28;

/// [`COM8`] power-on default (AEC and AGC enabled, banding filter disabled).
pub const COM8_DEFAULT: u8 = 0xC0;

/// [`COM8`] bit 5: enable the anti-banding filter.
pub const COM8_BNDF_EN: u8 = 0x20;

/// [`COM8`] bit 2: enable automatic gain control (AGC).
pub const COM8_AGC_EN: u8 = 0x04;

/// [`COM8`] bit 0: enable automatic exposure control (AEC).
pub const COM8_AEC_EN: u8 = 0x01;

/// [`COM9`] AGC gain ceiling: 8×. Value is placed in bits\[5:3\] of COM9.
pub const COM9_AGC_GAIN_8X: u8 = 0x02;

/// [`MC_BIST`] bit 7: reset the MCU BIST controller.
pub const MC_BIST_RESET: u8 = 0x80;

/// [`MC_BIST`] bit 6: select the boot ROM as the BIST target.
pub const MC_BIST_BOOT_ROM_SEL: u8 = 0x40;

/// [`RESET`] bit 4: hold the JPEG encoder in reset.
pub const RESET_JPEG: u8 = 0x10;

/// [`RESET`] bit 2: hold the DVP output block in reset.
pub const RESET_DVP: u8 = 0x04;

/// [`CTRL2`] bit 5: enable the hardware DCW (down-scaler).
pub const CTRL2_DCW_EN: u8 = 0x20;

/// [`CTRL3`] bit 6: enable white-pixel cancellation (WPC).
pub const CTRL3_WPC_EN: u8 = 0x40;

/// [`CTRLI`] bit 7: enable the lens-correction and image-processing pipeline.
pub const CTRLI_LP_DP: u8 = 0x80;

/// [`R_BYPASS`]: route sensor data through the DSP (normal operation).
pub const R_BYPASS_DSP_EN: u8 = 0x00;

/// [`R_BYPASS`]: bypass the DSP and route sensor data directly to DVP output.
pub const R_BYPASS_DSP_BYPAS: u8 = 0x01;

// ── IMAGE_MODE bit-field values ───────────────────────────────────────────────

/// [`IMAGE_MODE`] bit 4: enable the JPEG encoder.
/// When set, the DVP output carries a JPEG byte stream instead of raw pixels.
pub const IMAGE_MODE_JPEG_EN: u8 = 0x10;

/// [`IMAGE_MODE`] bit 1: use HREF/VSYNC timing on the DVP bus.
/// Required for OV2640, which exposes HREF (data-enable) rather than a
/// separate HSYNC pulse.
pub const IMAGE_MODE_HREF_VSYNC: u8 = 0x02;

/// [`IMAGE_MODE`] bits\[3:2\] = `0b00`: YUV422 DVP output (JPEG disabled).
pub const IMAGE_MODE_YUV422: u8 = 0x00;

/// [`IMAGE_MODE`] bits\[3:2\] = `0b10`: RGB565 DVP output (JPEG disabled).
pub const IMAGE_MODE_RGB565: u8 = 0x08;

/// [`IMAGE_MODE`] bit 0 — **BSYM** (byte swap).
/// `0` → `[HB, LB]` (big-endian, default); `1` → `[LB, HB]`.
pub const IMAGE_MODE_BSYM: u8 = 0x01;

// ── CTRL0 (0xC2) bit-field values ─────────────────────────────────────────────
//
// Bit layout per the esp32-camera reference (`ov2640_regs.h`). The previous
// constants here mislabelled bit 2 as `RGB_EN` (it is actually `YUV_EN`), which
// is why the RGB565 path was writing a YUV configuration.

/// [`CTRL0`] bit 7: enable auto-exposure control (AEC).
pub const CTRL0_AEC_EN: u8 = 0x80;

/// [`CTRL0`] bit 6: AEC algorithm select.
pub const CTRL0_AEC_SEL: u8 = 0x40;

/// [`CTRL0`] bit 5: route the statistics engine output. Feeds the AEC/AWB
/// statistics; clearing it leaves auto-white-balance without data (colour cast).
pub const CTRL0_STAT_SEL: u8 = 0x20;

/// [`CTRL0`] bit 4: output V (Cr) byte first in the YUV stream.
pub const CTRL0_VFIRST: u8 = 0x10;

/// [`CTRL0`] bit 3: select YUV422 output format.
pub const CTRL0_YUV422_EN: u8 = 0x08;

/// [`CTRL0`] bit 2: enable the YUV output data path.
pub const CTRL0_YUV_EN: u8 = 0x04;

/// [`CTRL0`] bit 1: enable the RGB output data path.
pub const CTRL0_RGB_EN: u8 = 0x02;

/// [`CTRL0`] bit 0: enable the RAW (Bayer) output data path.
pub const CTRL0_RAW_EN: u8 = 0x01;
