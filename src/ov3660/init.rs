//! OV3660 sensor initialization.

use embedded_hal::delay::DelayNs;
use esp_hal::{
    i2c::master::{Error, I2c},
    Blocking,
};

use super::{tables::DEFAULT_REGS, Ov3660};

impl Ov3660 {
    /// Reset the sensor and apply its default register configuration.
    pub fn init(&self, i2c: &mut I2c<'_, Blocking>, delay: &mut impl DelayNs) -> Result<(), Error> {
        self.reset(i2c, delay)?;

        self.apply_sequence(i2c, delay, DEFAULT_REGS)?;

        Ok(())
    }
}
