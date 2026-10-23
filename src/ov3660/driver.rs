//! OV3660 SCCB register access and sensor control.

use embedded_hal::delay::DelayNs;
use esp_hal::{
    i2c::master::{Error, I2c},
    Blocking,
};

use super::{
    registers::{PID_HIGH, PID_LOW, SOFTWARE_RESET, SYSTEM_CTRL0},
    tables::RegisterOp,
    Ov3660,
};

impl Ov3660 {
    pub fn read_reg(&self, i2c: &mut I2c<'_, Blocking>, reg: u16) -> Result<u8, Error> {
        let mut value = [0u8; 1];

        i2c.write_read(self.address, &reg.to_be_bytes(), &mut value)?;

        Ok(value[0])
    }

    pub fn write_reg(&self, i2c: &mut I2c<'_, Blocking>, reg: u16, value: u8) -> Result<(), Error> {
        let [high, low] = reg.to_be_bytes();

        i2c.write(self.address, &[high, low, value])
    }

    pub(super) fn write_reg16(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        reg: u16,
        value: u16,
    ) -> Result<(), Error> {
        let [high, low] = value.to_be_bytes();

        self.write_reg(i2c, reg, high)?;
        self.write_reg(i2c, reg + 1, low)?;

        Ok(())
    }

    pub(super) fn write_addr_reg(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        reg: u16,
        x_value: u16,
        y_value: u16,
    ) -> Result<(), Error> {
        self.write_reg16(i2c, reg, x_value)?;
        self.write_reg16(i2c, reg + 2, y_value)?;

        Ok(())
    }

    pub(super) fn set_reg_bits(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        reg: u16,
        offset: u8,
        mask: u8,
        value: u8,
    ) -> Result<(), Error> {
        let current = self.read_reg(i2c, reg)?;

        let shifted_mask = mask << offset;
        let updated = (current & !shifted_mask) | ((value & mask) << offset);

        self.write_reg(i2c, reg, updated)
    }

    pub(super) fn write_reg_bits(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        reg: u16,
        mask: u8,
        enable: bool,
    ) -> Result<(), Error> {
        self.set_reg_bits(i2c, reg, 0, mask, if enable { mask } else { 0 })
    }

    pub fn read_id(&self, i2c: &mut I2c<'_, Blocking>) -> Result<u16, Error> {
        let high = self.read_reg(i2c, PID_HIGH)?;
        let low = self.read_reg(i2c, PID_LOW)?;

        Ok(u16::from_be_bytes([high, low]))
    }

    pub(super) fn reset(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        delay: &mut impl DelayNs,
    ) -> Result<(), Error> {
        self.write_reg(i2c, SYSTEM_CTRL0, SOFTWARE_RESET)?;

        delay.delay_ms(100);

        Ok(())
    }

    pub(super) fn apply_sequence(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        delay: &mut impl DelayNs,
        sequence: &[RegisterOp],
    ) -> Result<(), Error> {
        for op in sequence {
            match *op {
                RegisterOp::Write(reg, value) => {
                    self.write_reg(i2c, reg, value)?;
                }
                RegisterOp::DelayMs(ms) => {
                    delay.delay_ms(ms);
                }
            }
        }

        Ok(())
    }
}
