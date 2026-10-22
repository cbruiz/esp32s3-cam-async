//! OV3660 sensor initialization.

use embedded_hal::delay::DelayNs;
use esp_hal::{
    i2c::master::{Error, I2c},
    Blocking,
};

use super::{
    tables::{DEFAULT_REGS, JPEG_REGS},
    Ov3660,
};

use super::tables::RatioSettings;

use super::registers::*;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct FrameGeometry {
    binning: bool,
    scale: bool,
}

impl FrameGeometry {
    fn new(width: u16, height: u16, settings: &RatioSettings) -> Self {
        let binning = width <= settings.max_width / 2 && height <= settings.max_height / 2;

        let scale = !((width == settings.max_width && height == settings.max_height)
            || (width == settings.max_width / 2 && height == settings.max_height / 2));

        Self { binning, scale }
    }
}

#[derive(Clone, Copy, Debug)]
struct PllConfig {
    bypass: bool,
    multiplier: u8,
    sys_div: u8,
    pre_div: u8,
    root_2x: bool,
    seld5: u8,
    pclk_manual: bool,
    pclk_div: u8,
}

impl PllConfig {
    fn calc_sysclk(&self, xclk_hz: u32) -> ClockFrequencies {
        const PRE_DIV_2X: [u64; 4] = [2, 3, 4, 6];
        const SELD5_2X: [u64; 4] = [2, 2, 4, 5];

        let pre_div = PRE_DIV_2X[self.pre_div as usize];
        let seld5 = SELD5_2X[self.seld5 as usize];

        let root_div = if self.root_2x { 2u64 } else { 1 };
        let sys_div = u64::from(self.sys_div.max(1));

        // Reference calculation uses kHz to avoid floating point.
        let vco_khz =
            (u64::from(xclk_hz) / 1000) * u64::from(self.multiplier) * root_div * 2 / pre_div;

        let pllclk_hz = if self.bypass {
            u64::from(xclk_hz)
        } else {
            vco_khz * 1000 * 2 / sys_div / seld5
        };

        let pclk_div = if self.pclk_manual && self.pclk_div != 0 {
            u64::from(self.pclk_div)
        } else {
            1
        };

        ClockFrequencies {
            vco_hz: vco_khz * 1000,
            pllclk_hz,
            sysclk_hz: pllclk_hz / 4,
            pclk_hz: pllclk_hz / 2 / pclk_div,
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ClockFrequencies {
    vco_hz: u64,
    pllclk_hz: u64,
    sysclk_hz: u64,
    pclk_hz: u64,
}

impl Ov3660 {
    /// Reset the sensor and apply its default register configuration.
    pub fn init(&self, i2c: &mut I2c<'_, Blocking>, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.reset(i2c, delay)?;

        self.apply_sequence(i2c, delay, DEFAULT_REGS)?;

        Ok(())
    }

    pub fn set_jpeg_format(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error> {
        self.apply_sequence(i2c, delay, JPEG_REGS)
    }

    fn set_frame_window(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        width: u16,
        height: u16,
        settings: &RatioSettings,
    ) -> Result<(), Error> {
        self.write_addr_reg(i2c, X_ADDR_ST_H, settings.start_x, settings.start_y)?;

        self.write_addr_reg(i2c, X_ADDR_END_H, settings.end_x, settings.end_y)?;

        self.write_addr_reg(i2c, X_OUTPUT_SIZE_H, width, height)?;

        Ok(())
    }

    fn set_frame_timing(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        settings: &RatioSettings,
        geometry: &FrameGeometry,
    ) -> Result<(), Error> {
        if geometry.binning {
            self.write_addr_reg(
                i2c,
                X_TOTAL_SIZE_H,
                settings.total_x,
                (settings.total_y / 2) + 1,
            )?;

            self.write_addr_reg(i2c, X_OFFSET_H, 8, 2)?;
        } else {
            self.write_addr_reg(i2c, X_TOTAL_SIZE_H, settings.total_x, settings.total_y)?;

            self.write_addr_reg(i2c, X_OFFSET_H, 16, 6)?;
        }

        self.write_reg_bits(i2c, ISP_CONTROL_01, 0x20, geometry.scale)?;

        Ok(())
    }

    fn set_image_options(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        geometry: &FrameGeometry,
        jpeg: bool,
        vflip: bool,
        hmirror: bool,
    ) -> Result<(), Error> {
        let mut reg20 = 0u8;
        let mut reg21 = 0u8;
        let mut reg4514_test = 0u8;

        // Compression
        if jpeg {
            reg21 |= 0x20;
        }

        // Binning
        if geometry.binning {
            reg20 |= 0x01;
            reg21 |= 0x01;
            reg4514_test |= 4;
        } else {
            reg20 |= 0x40;
        }

        // Vertical flip
        if vflip {
            reg20 |= 0x06;
            reg4514_test |= 1;
        }

        // Horizontal mirror
        if hmirror {
            reg21 |= 0x06;
            reg4514_test |= 2;
        }

        let reg4514 = match reg4514_test {
            0 | 1 => 0x88,
            2 | 3 => 0xbb,
            4 | 7 => 0xaa,
            5 | 6 => 0xbb,
            _ => unreachable!(),
        };

        self.write_reg(i2c, TIMING_TC_REG20, reg20)?;
        self.write_reg(i2c, TIMING_TC_REG21, reg21)?;
        self.write_reg(i2c, 0x4514, reg4514)?;

        if geometry.binning {
            self.write_reg(i2c, 0x4520, 0x0b)?;
            self.write_reg(i2c, X_INCREMENT, 0x31)?;
            self.write_reg(i2c, Y_INCREMENT, 0x31)?;
        } else {
            self.write_reg(i2c, 0x4520, 0xb0)?;
            self.write_reg(i2c, X_INCREMENT, 0x11)?;
            self.write_reg(i2c, Y_INCREMENT, 0x11)?;
        }

        Ok(())
    }

    fn set_pll(&self, i2c: &mut I2c<'_, Blocking>, config: PllConfig) -> Result<(), Error> {
        // Register field limits from the OV3660 reference driver.
        debug_assert!(config.multiplier <= 31);
        debug_assert!(config.sys_div <= 15);
        debug_assert!(config.pre_div <= 3);
        debug_assert!(config.seld5 <= 3);
        debug_assert!(config.pclk_div <= 31);

        self.write_reg(i2c, SC_PLLS_CTRL0, if config.bypass { 0x80 } else { 0x00 })?;

        self.write_reg(i2c, SC_PLLS_CTRL1, config.multiplier & 0x1f)?;

        self.write_reg(i2c, SC_PLLS_CTRL2, 0x10 | (config.sys_div & 0x0f))?;

        self.write_reg(
            i2c,
            SC_PLLS_CTRL3,
            ((config.pre_div & 0x03) << 4)
                | config.seld5
                | if config.root_2x { 0x40 } else { 0x00 },
        )?;

        self.write_reg(i2c, PCLK_RATIO, config.pclk_div & 0x1f)?;

        self.write_reg(
            i2c,
            VFIFO_CTRL0C,
            if config.pclk_manual { 0x22 } else { 0x20 },
        )?;

        Ok(())
    }

    pub fn set_framesize_qvga(&self, i2c: &mut I2c<'_, Blocking>) -> Result<(), Error> {
        const WIDTH: u16 = 320;
        const HEIGHT: u16 = 240;

        let settings = RatioSettings::RATIO_4X3;
        let geometry = FrameGeometry::new(WIDTH, HEIGHT, &settings);

        // 1. Sensor window and output dimensions.
        self.set_frame_window(i2c, WIDTH, HEIGHT, &settings)?;

        // 2. Timing, offsets and scaling.
        self.set_frame_timing(i2c, &settings, &geometry)?;

        // 3. JPEG, binning and image orientation.
        self.set_image_options(
            i2c, &geometry, true,  // JPEG
            false, // V-Flip
            false, // H-Mirror
        )?;

        // 4. Sensor PLL configuration.
        self.set_pll(
            i2c,
            PllConfig {
                bypass: false,
                multiplier: 30,
                sys_div: 1,
                pre_div: 3,
                root_2x: false,
                seld5: 0,
                pclk_manual: true,
                pclk_div: 10,
            },
        )?;

        Ok(())
    }

    pub fn set_jpeg_quality(&self, i2c: &mut I2c<'_, Blocking>, quality: u8) -> Result<(), Error> {
        self.write_reg(i2c, COMPRESSION_CTRL07, quality & 0x3f)
    }
}
