mod driver;
mod init;
pub mod registers;
mod tables;

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
}

impl Default for Ov3660 {
    fn default() -> Self {
        Self::new()
    }
}