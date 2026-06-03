//! OV2640 camera sensor driver — I2C/SCCB configuration.
//!
//! For the complete reference — every supported resolution and pixel format,
//! the init sequence, the register/table map, the DSP window math, and the exact
//! **byte layout** of each output format (RGB565 `[HB,LB]`, YUV422 **YUYV**
//! `[Y0,Cb,Y1,Cr]`, JPEG) — see [`docs/OV2640.md`].
//!
//! [`docs/OV2640.md`]: https://github.com/cbruiz/esp32s3-cam-async/blob/main/docs/OV2640.md
//!
//! # Quick start
//!
//! ```rust,ignore
//! use esp32s3_cam_async::ov2640::{Ov2640, Ov2640Config, PixelFormat, Resolution};
//!
//! let sensor = Ov2640::new(Ov2640Config {
//!     format: PixelFormat::Rgb565,
//!     resolution: Resolution::Qvga,
//!     ..Default::default()
//! });
//! sensor.check_id(&mut i2c).expect("OV2640 not found");
//! sensor.init(&mut i2c).await.expect("OV2640 init failed");
//!
//! // Direct register access when needed
//! sensor.write_reg(&mut i2c, MY_REG, MY_VAL)?;
//! sensor.write_regs(&mut i2c, &CUSTOM_TABLE)?;
//! ```
//!
//! For access to an arbitrary I2C address, use the free-function escape hatches
//! [`write_reg`] / [`write_regs`] together with the constants in [`registers`].

pub mod clock;
pub mod config;
pub mod registers;
mod tables;
mod init;

pub use clock::ClockConfig;
pub use config::{Ov2640Config, OV2640_ADDR, PixelFormat, Resolution};
pub use init::{Ov2640, write_reg, write_regs};
