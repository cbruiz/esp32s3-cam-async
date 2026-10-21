use esp_hal::i2c::master::{Error, I2c};
use esp_hal::Blocking;

pub const OV3660_ADDR: u8 = 0x3c;
pub const OV3660_PID: u16 = 0x3660;

const REG_PID_HIGH: u16 = 0x300a;
const REG_PID_LOW: u16 = 0x300b;

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

    pub fn read_id(&self, i2c: &mut I2c<'_, Blocking>) -> Result<u16, Error> {
        let high = self.read_reg(i2c, REG_PID_HIGH)?;
        let low = self.read_reg(i2c, REG_PID_LOW)?;

        Ok(u16::from_be_bytes([high, low]))
    }
}

impl Default for Ov3660 {
    fn default() -> Self {
        Self::new()
    }
}
