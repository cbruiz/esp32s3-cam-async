mod init;
pub mod registers;
mod tables;

use esp_hal::i2c::master::{Error, I2c};
use esp_hal::Blocking;

use registers::{PID_HIGH, PID_LOW, SOFTWARE_RESET, SYSTEM_CTRL0};
use tables::RegisterOp;

pub const OV3660_ADDR: u8 = 0x3c;
pub const OV3660_PID: u16 = 0x3660;

pub struct Ov3660 {
    address: u8,
}

impl Ov3660 {
    pub const fn new() -> Self {
        Self {
            address: OV3660_ADDR,
        }
    }

    pub fn read_reg(&self, i2c: &mut I2c<'_, Blocking>, reg: u16) -> Result<u8, Error> {
        let mut value = [0u8; 1];

        i2c.write_read(self.address, &reg.to_be_bytes(), &mut value)?;

        Ok(value[0])
    }

    pub fn write_reg(&self, i2c: &mut I2c<'_, Blocking>, reg: u16, value: u8) -> Result<(), Error> {
        let [high, low] = reg.to_be_bytes();

        i2c.write(self.address, &[high, low, value])
    }

    pub fn write_regs(&self, i2c: &mut I2c<'_, Blocking>, regs: &[(u16, u8)]) -> Result<(), Error> {
        for &(reg, value) in regs {
            self.write_reg(i2c, reg, value)?;
        }

        Ok(())
    }

    pub fn read_id(&self, i2c: &mut I2c<'_, Blocking>) -> Result<u16, Error> {
        let high = self.read_reg(i2c, PID_HIGH)?;
        let low = self.read_reg(i2c, PID_LOW)?;

        Ok(u16::from_be_bytes([high, low]))
    }

    pub fn reset(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        delay: &mut impl embedded_hal::delay::DelayNs,
    ) -> Result<(), Error> {
        self.write_reg(i2c, SYSTEM_CTRL0, SOFTWARE_RESET)?;

        delay.delay_ms(100);

        Ok(())
    }

    fn apply_sequence(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        delay: &mut impl embedded_hal::delay::DelayNs,
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

impl Default for Ov3660 {
    fn default() -> Self {
        Self::new()
    }
}
