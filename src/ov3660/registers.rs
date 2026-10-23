//! OV3660 register definitions.
//!
//! Ported from `ov3660_regs.h` in espressif/esp32-camera.

// System control registers

/// System control register 0.
///
/// - Bit[7]: Software reset
/// - Bit[6]: Software power down
/// - Bit[5]: Reserved
/// - Bit[4]: SRB clock SYNC enable
/// - Bit[3]: Isolation suspend select
/// - Bit[2:0]: Not used
pub const SYSTEM_CTRL0: u16 = 0x3008;

/// Sensor product identifier, high byte.
pub const PID_HIGH: u16 = 0x300A;
/// Sensor product identifier, low byte.
pub const PID_LOW: u16 = 0x300B;

/// Output format select.
///
/// Bit[2:0]:
/// - `000`: YUV422
/// - `001`: RGB
/// - `010`: Dither
/// - `011`: RAW after DPC
/// - `101`: RAW after CIP
pub const FORMAT_CTRL: u16 = 0x501F;

/// Format control 00.
pub const FORMAT_CTRL00: u16 = 0x4300;

/// Frame control: number of passed frames.
///
/// When both ON and OFF numbers are 0x00, frame control is bypassed.
///
/// - Bit[7:4]: Not used
/// - Bit[3:0]: Frame ON number
pub const FRAME_CTRL01: u16 = 0x4201;

/// Frame control: number of masked frames.
///
/// When both ON and OFF numbers are 0x00, frame control is bypassed.
///
/// - Bit[7:4]: Not used
/// - Bit[3:0]: Frame OFF number
pub const FRAME_CTRL02: u16 = 0x4202;

/// Pre-ISP test setting 1.
///
/// - Bit[7]: Test enable (`0`: disable, `1`: color bar enable)
/// - Bit[6]: Rolling
/// - Bit[5]: Transparent
/// - Bit[4]: Square black and white
/// - Bit[3:2]: Color bar style
///   - `00`: standard 8 color bar
///   - `01`: gradual change at vertical mode 1
///   - `10`: gradual change at horizontal
///   - `11`: gradual change at vertical mode 2
/// - Bit[1:0]: Test select
///   - `00`: color bar
///   - `01`: random data
///   - `10`: square data
///   - `11`: black image
pub const PRE_ISP_TEST_SETTING_1: u16 = 0x503D;

/// AEC/AGC manual mode control.
///
/// - Bit[7:6]: Reserved
/// - Bit[5]: Gain delay option, valid when 0x3503[4] = 0
///   - `0`: delay one frame latch
///   - `1`: one frame latch
/// - Bit[4:2]: Reserved
/// - Bit[1]: AGC manual (`0`: auto, `1`: manual)
/// - Bit[0]: AEC manual (`0`: auto, `1`: manual)
pub const AEC_PK_MANUAL: u16 = 0x3503;

/// Timing control register 20.
///
/// - Bit[2:1]: Vertical flip enable (`00`: normal, `11`: vertical flip)
/// - Bit[0]: Vertical binning enable
pub const TIMING_TC_REG20: u16 = 0x3820;

/// Timing control register 21.
///
/// - Bit[5]: Compression enable
/// - Bit[2:1]: Horizontal mirror enable (`00`: normal, `11`: horizontal mirror)
/// - Bit[0]: Horizontal binning enable
pub const TIMING_TC_REG21: u16 = 0x3821;

/// Clock polarity control.
///
/// - Bit[5]: PCLK polarity (`0`: active low, `1`: active high)
/// - Bit[3]: Gate PCLK under VSYNC
/// - Bit[2]: Gate PCLK under HREF
/// - Bit[1]: HREF polarity (`0`: active low, `1`: active high)
/// - Bit[0]: VSYNC polarity (`0`: active low, `1`: active high)
pub const CLOCK_POL_CONTROL: u16 = 0x4740;

/// Pad drive capability.
///
/// Bit[7:6]: `00`: 1x, `01`: 2x, `10`: 3x, `11`: 4x
pub const DRIVE_CAPABILITY: u16 = 0x302C;

// Window and output size registers

/// X address start, Bit[3:0]: X address start[11:8].
pub const X_ADDR_ST_H: u16 = 0x3800;
/// X address start, Bit[7:0]: X address start[7:0].
pub const X_ADDR_ST_L: u16 = 0x3801;
/// Y address start, Bit[2:0]: Y address start[10:8].
pub const Y_ADDR_ST_H: u16 = 0x3802;
/// Y address start, Bit[7:0]: Y address start[7:0].
pub const Y_ADDR_ST_L: u16 = 0x3803;
/// X address end, Bit[3:0]: X address end[11:8].
pub const X_ADDR_END_H: u16 = 0x3804;
/// X address end, Bit[7:0]: X address end[7:0].
pub const X_ADDR_END_L: u16 = 0x3805;
/// Y address end, Bit[2:0]: Y address end[10:8].
pub const Y_ADDR_END_H: u16 = 0x3806;
/// Y address end, Bit[7:0]: Y address end[7:0].
pub const Y_ADDR_END_L: u16 = 0x3807;

// Size after scaling

/// DVP output width, Bit[3:0]: width[11:8].
pub const X_OUTPUT_SIZE_H: u16 = 0x3808;
/// DVP output width, Bit[7:0]: width[7:0].
pub const X_OUTPUT_SIZE_L: u16 = 0x3809;
/// DVP output height, Bit[2:0]: height[10:8].
pub const Y_OUTPUT_SIZE_H: u16 = 0x380A;
/// DVP output height, Bit[7:0]: height[7:0].
pub const Y_OUTPUT_SIZE_L: u16 = 0x380B;
/// Total horizontal size, Bit[3:0]: size[11:8].
pub const X_TOTAL_SIZE_H: u16 = 0x380C;
/// Total horizontal size, Bit[7:0]: size[7:0].
pub const X_TOTAL_SIZE_L: u16 = 0x380D;
/// Total vertical size, Bit[7:0]: size[15:8].
pub const Y_TOTAL_SIZE_H: u16 = 0x380E;
/// Total vertical size, Bit[7:0]: size[7:0].
pub const Y_TOTAL_SIZE_L: u16 = 0x380F;
/// ISP horizontal offset, Bit[3:0]: offset[11:8].
pub const X_OFFSET_H: u16 = 0x3810;
/// ISP horizontal offset, Bit[7:0]: offset[7:0].
pub const X_OFFSET_L: u16 = 0x3811;
/// ISP vertical offset, Bit[2:0]: offset[10:8].
pub const Y_OFFSET_H: u16 = 0x3812;
/// ISP vertical offset, Bit[7:0]: offset[7:0].
pub const Y_OFFSET_L: u16 = 0x3813;
/// Horizontal subsample increment.
///
/// - Bit[7:4]: odd increment
/// - Bit[3:0]: even increment
pub const X_INCREMENT: u16 = 0x3814;
/// Vertical subsample increment.
///
/// - Bit[7:4]: odd increment
/// - Bit[3:0]: even increment
pub const Y_INCREMENT: u16 = 0x3815;

// ISP and scaling registers

/// ISP control 01. Bit[5]: Scale enable (`0`: disable, `1`: enable).
pub const ISP_CONTROL_01: u16 = 0x5001;

/// Scale control 1.
///
/// - Bit[6:4]: HDIV, DCW scale times
/// - Bit[2:0]: VDIV, DCW scale times
///
/// Both fields: `000`: 1x, `001`: 2x, `010`: 4x, `100`: 8x, `101`: 16x,
/// others: 16x.
pub const SCALE_CTRL_1: u16 = 0x5601;
/// X scale, high bits.
pub const SCALE_CTRL_2: u16 = 0x5602;
/// X scale, low bits.
pub const SCALE_CTRL_3: u16 = 0x5603;
/// Y scale, high bits.
pub const SCALE_CTRL_4: u16 = 0x5604;
/// Y scale, low bits.
pub const SCALE_CTRL_5: u16 = 0x5605;
/// Bit[3:0]: V offset.
pub const SCALE_CTRL_6: u16 = 0x5606;

/// Bit[4:0]: PCLK ratio manual.
pub const PCLK_RATIO: u16 = 0x3824;
/// Bit[1]: PCLK manual enable (`0`: auto, `1`: manual by [`PCLK_RATIO`]).
pub const VFIFO_CTRL0C: u16 = 0x460C;

pub const VFIFO_X_SIZE_H: u16 = 0x4602;
pub const VFIFO_X_SIZE_L: u16 = 0x4603;
pub const VFIFO_Y_SIZE_H: u16 = 0x4604;
pub const VFIFO_Y_SIZE_L: u16 = 0x4605;

// PLL registers

/// Bit[7]: PLLS bypass.
pub const SC_PLLS_CTRL0: u16 = 0x303A;
/// Bit[4:0]: PLLS multiplier.
pub const SC_PLLS_CTRL1: u16 = 0x303B;
/// - Bit[6:4]: PLLS charge pump control
/// - Bit[3:0]: PLLS system divider
pub const SC_PLLS_CTRL2: u16 = 0x303C;
/// PLLS control 3.
///
/// - Bit[5:4]: PLLS pre-divider (`00`: 1, `01`: 1.5, `10`: 2, `11`: 3)
/// - Bit[2]: PLLS root-divider - 1
/// - Bit[1:0]: PLLS seld5 (`00`: 1, `01`: 1, `10`: 2, `11`: 2.5)
pub const SC_PLLS_CTRL3: u16 = 0x303D;

// Compression registers

pub const COMPRESSION_CTRL00: u16 = 0x4400;
pub const COMPRESSION_CTRL01: u16 = 0x4401;
pub const COMPRESSION_CTRL02: u16 = 0x4402;
pub const COMPRESSION_CTRL03: u16 = 0x4403;
pub const COMPRESSION_CTRL04: u16 = 0x4404;
pub const COMPRESSION_CTRL05: u16 = 0x4405;
pub const COMPRESSION_CTRL06: u16 = 0x4406;
/// Bit[5:0]: QS.
pub const COMPRESSION_CTRL07: u16 = 0x4407;
pub const COMPRESSION_ISI_CTRL: u16 = 0x4408;
pub const COMPRESSION_CTRL09: u16 = 0x4409;
pub const COMPRESSION_CTRL0A: u16 = 0x440A;
pub const COMPRESSION_CTRL0B: u16 = 0x440B;
pub const COMPRESSION_CTRL0C: u16 = 0x440C;
pub const COMPRESSION_CTRL0D: u16 = 0x440D;
pub const COMPRESSION_CTRL0E: u16 = 0x440E;

// Register values

/// Value written to [`SYSTEM_CTRL0`] to trigger a software reset.
pub const SOFTWARE_RESET: u8 = 0x82;

/// Enable the rolling color bar test.
pub const TEST_COLOR_BAR: u8 = 0xC0;

/// [`AEC_PK_MANUAL`]: AGC manual enable.
pub const AEC_PK_MANUAL_AGC_MANUALEN: u8 = 0x02;

/// [`AEC_PK_MANUAL`]: AEC manual enable.
pub const AEC_PK_MANUAL_AEC_MANUALEN: u8 = 0x01;

/// [`TIMING_TC_REG20`]: vertical flip enable.
pub const TIMING_TC_REG20_VFLIP: u8 = 0x06;

/// [`TIMING_TC_REG21`]: horizontal mirror enable.
pub const TIMING_TC_REG21_HMIRROR: u8 = 0x06;
