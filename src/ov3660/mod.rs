//! OV3660 image sensor support.
//!
//! SCCB configuration and JPEG QVGA capture support for ESP32-S3.
//!
//! Register definitions and initialization logic are derived from
//! Espressif's esp32-camera and the OpenMV OV3660 driver.

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
