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
}
