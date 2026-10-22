//! OV3660 configuration sequences ported from Espressif esp32-camera.
//! Source: sensors/private_include/ov3660_settings.h
//! SPDX-License-Identifier: Apache-2.0
//! Modified: converted C register pairs and sentinels to typed Rust operations.

#[derive(Clone, Copy, Debug)]
pub struct RatioSettings {
    pub max_width: u16,
    pub max_height: u16,
    pub start_x: u16,
    pub start_y: u16,
    pub end_x: u16,
    pub end_y: u16,
    pub offset_x: u16,
    pub offset_y: u16,
    pub total_x: u16,
    pub total_y: u16,
}

impl RatioSettings {
    pub const RATIO_4X3: Self = Self {
        max_width: 2048,
        max_height: 1536,
        start_x: 0,
        start_y: 0,
        end_x: 2079,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2300,
        total_y: 1564,
    };

    pub const RATIO_3X2: Self = Self {
        max_width: 1920,
        max_height: 1280,
        start_x: 64,
        start_y: 128,
        end_x: 2015,
        end_y: 1419,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1436,
    };

    pub const RATIO_16X10: Self = Self {
        max_width: 2048,
        max_height: 1280,
        start_x: 0,
        start_y: 128,
        end_x: 2079,
        end_y: 1419,
        offset_x: 16,
        offset_y: 6,
        total_x: 2300,
        total_y: 1436,
    };

    pub const RATIO_5X3: Self = Self {
        max_width: 1920,
        max_height: 1152,
        start_x: 64,
        start_y: 192,
        end_x: 2015,
        end_y: 1355,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1372,
    };

    // { 1920, 1080,  64, 242, 2015, 1333, 16, 6, 2172, 1322 }, //16x9
    pub const RATIO_16X9: Self = Self {
        max_width: 1920,
        max_height: 1080,
        start_x: 64,
        start_y: 242,
        end_x: 2015,
        end_y: 1333,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1322,
    };

    // { 2048,  880,   0, 328, 2079, 1219, 16, 6, 2300, 1236 }, //21x9
    pub const RATIO_21X9: Self = Self {
        max_width: 2048,
        max_height: 880,
        start_x: 0,
        start_y: 328,
        end_x: 2079,
        end_y: 1219,
        offset_x: 16,
        offset_y: 6,
        total_x: 2300,
        total_y: 1236,
    };

    // { 1920, 1536,  64,   0, 2015, 1547, 16, 6, 2172, 1564 }, //5x4
    pub const RATIO_5X4: Self = Self {
        max_width: 1920,
        max_height: 1536,
        start_x: 64,
        start_y: 0,
        end_x: 2015,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2172,
        total_y: 1564,
    };

    // { 1536, 1536, 256,   0, 1823, 1547, 16, 6, 2044, 1564 }, //1x1
    pub const RATIO_1X1: Self = Self {
        max_width: 1536,
        max_height: 1536,
        start_x: 256,
        start_y: 0,
        end_x: 1823,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2044,
        total_y: 1564,
    };

    // {  864, 1536, 592,   0, 1487, 1547, 16, 6, 2044, 1564 }  //9x16
    pub const RATIO_9X16: Self = Self {
        max_width: 864,
        max_height: 1536,
        start_x: 592,
        start_y: 0,
        end_x: 1487,
        end_y: 1547,
        offset_x: 16,
        offset_y: 6,
        total_x: 2044,
        total_y: 1564,
    };
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegisterOp {
    Write(u16, u8),
    DelayMs(u32),
}

use super::registers::*;

/// Port of `sensor_default_regs` (198 operations).
pub const DEFAULT_REGS: &[RegisterOp] = &[
    RegisterOp::Write(SYSTEM_CTRL0, 0x82), // software reset
    RegisterOp::DelayMs(10),               // delay 10ms
    RegisterOp::Write(0x3103, 0x13),
    RegisterOp::Write(SYSTEM_CTRL0, 0x42),
    RegisterOp::Write(0x3017, 0xff),
    RegisterOp::Write(0x3018, 0xff),
    RegisterOp::Write(DRIVE_CAPABILITY, 0xc3),
    RegisterOp::Write(CLOCK_POL_CONTROL, 0x21),
    RegisterOp::Write(0x3611, 0x01),
    RegisterOp::Write(0x3612, 0x2d),
    RegisterOp::Write(0x3032, 0x00),
    RegisterOp::Write(0x3614, 0x80),
    RegisterOp::Write(0x3618, 0x00),
    RegisterOp::Write(0x3619, 0x75),
    RegisterOp::Write(0x3622, 0x80),
    RegisterOp::Write(0x3623, 0x00),
    RegisterOp::Write(0x3624, 0x03),
    RegisterOp::Write(0x3630, 0x52),
    RegisterOp::Write(0x3632, 0x07),
    RegisterOp::Write(0x3633, 0xd2),
    RegisterOp::Write(0x3704, 0x80),
    RegisterOp::Write(0x3708, 0x66),
    RegisterOp::Write(0x3709, 0x12),
    RegisterOp::Write(0x370b, 0x12),
    RegisterOp::Write(0x3717, 0x00),
    RegisterOp::Write(0x371b, 0x60),
    RegisterOp::Write(0x371c, 0x00),
    RegisterOp::Write(0x3901, 0x13),
    RegisterOp::Write(0x3600, 0x08),
    RegisterOp::Write(0x3620, 0x43),
    RegisterOp::Write(0x3702, 0x20),
    RegisterOp::Write(0x3739, 0x48),
    RegisterOp::Write(0x3730, 0x20),
    RegisterOp::Write(0x370c, 0x0c),
    RegisterOp::Write(0x3a18, 0x00),
    RegisterOp::Write(0x3a19, 0xf8),
    RegisterOp::Write(0x3000, 0x10),
    RegisterOp::Write(0x3004, 0xef),
    RegisterOp::Write(0x6700, 0x05),
    RegisterOp::Write(0x6701, 0x19),
    RegisterOp::Write(0x6702, 0xfd),
    RegisterOp::Write(0x6703, 0xd1),
    RegisterOp::Write(0x6704, 0xff),
    RegisterOp::Write(0x6705, 0xff),
    RegisterOp::Write(0x3c01, 0x80),
    RegisterOp::Write(0x3c00, 0x04),
    RegisterOp::Write(0x3a08, 0x00),
    RegisterOp::Write(0x3a09, 0x62), //50Hz Band Width Step (10bit)
    RegisterOp::Write(0x3a0e, 0x08), //50Hz Max Bands in One Frame (6 bit)
    RegisterOp::Write(0x3a0a, 0x00),
    RegisterOp::Write(0x3a0b, 0x52), //60Hz Band Width Step (10bit)
    RegisterOp::Write(0x3a0d, 0x09), //60Hz Max Bands in One Frame (6 bit)
    RegisterOp::Write(0x3a00, 0x3a), //night mode off
    RegisterOp::Write(0x3a14, 0x09),
    RegisterOp::Write(0x3a15, 0x30),
    RegisterOp::Write(0x3a02, 0x09),
    RegisterOp::Write(0x3a03, 0x30),
    RegisterOp::Write(COMPRESSION_CTRL0E, 0x08),
    RegisterOp::Write(0x4520, 0x0b),
    RegisterOp::Write(0x460b, 0x37),
    RegisterOp::Write(0x4713, 0x02),
    RegisterOp::Write(0x471c, 0xd0),
    RegisterOp::Write(0x5086, 0x00),
    RegisterOp::Write(0x5002, 0x00),
    RegisterOp::Write(0x501f, 0x00),
    RegisterOp::Write(SYSTEM_CTRL0, 0x02),
    RegisterOp::Write(0x5180, 0xff),
    RegisterOp::Write(0x5181, 0xf2),
    RegisterOp::Write(0x5182, 0x00),
    RegisterOp::Write(0x5183, 0x14),
    RegisterOp::Write(0x5184, 0x25),
    RegisterOp::Write(0x5185, 0x24),
    RegisterOp::Write(0x5186, 0x16),
    RegisterOp::Write(0x5187, 0x16),
    RegisterOp::Write(0x5188, 0x16),
    RegisterOp::Write(0x5189, 0x68),
    RegisterOp::Write(0x518a, 0x60),
    RegisterOp::Write(0x518b, 0xe0),
    RegisterOp::Write(0x518c, 0xb2),
    RegisterOp::Write(0x518d, 0x42),
    RegisterOp::Write(0x518e, 0x35),
    RegisterOp::Write(0x518f, 0x56),
    RegisterOp::Write(0x5190, 0x56),
    RegisterOp::Write(0x5191, 0xf8),
    RegisterOp::Write(0x5192, 0x04),
    RegisterOp::Write(0x5193, 0x70),
    RegisterOp::Write(0x5194, 0xf0),
    RegisterOp::Write(0x5195, 0xf0),
    RegisterOp::Write(0x5196, 0x03),
    RegisterOp::Write(0x5197, 0x01),
    RegisterOp::Write(0x5198, 0x04),
    RegisterOp::Write(0x5199, 0x12),
    RegisterOp::Write(0x519a, 0x04),
    RegisterOp::Write(0x519b, 0x00),
    RegisterOp::Write(0x519c, 0x06),
    RegisterOp::Write(0x519d, 0x82),
    RegisterOp::Write(0x519e, 0x38),
    RegisterOp::Write(0x5381, 0x1d),
    RegisterOp::Write(0x5382, 0x60),
    RegisterOp::Write(0x5383, 0x03),
    RegisterOp::Write(0x5384, 0x0c),
    RegisterOp::Write(0x5385, 0x78),
    RegisterOp::Write(0x5386, 0x84),
    RegisterOp::Write(0x5387, 0x7d),
    RegisterOp::Write(0x5388, 0x6b),
    RegisterOp::Write(0x5389, 0x12),
    RegisterOp::Write(0x538a, 0x01),
    RegisterOp::Write(0x538b, 0x98),
    RegisterOp::Write(0x5480, 0x01),
    RegisterOp::Write(0x5000, 0xa7),
    RegisterOp::Write(0x5800, 0x0c),
    RegisterOp::Write(0x5801, 0x09),
    RegisterOp::Write(0x5802, 0x0c),
    RegisterOp::Write(0x5803, 0x0c),
    RegisterOp::Write(0x5804, 0x0d),
    RegisterOp::Write(0x5805, 0x17),
    RegisterOp::Write(0x5806, 0x06),
    RegisterOp::Write(0x5807, 0x05),
    RegisterOp::Write(0x5808, 0x04),
    RegisterOp::Write(0x5809, 0x06),
    RegisterOp::Write(0x580a, 0x09),
    RegisterOp::Write(0x580b, 0x0e),
    RegisterOp::Write(0x580c, 0x05),
    RegisterOp::Write(0x580d, 0x01),
    RegisterOp::Write(0x580e, 0x01),
    RegisterOp::Write(0x580f, 0x01),
    RegisterOp::Write(0x5810, 0x05),
    RegisterOp::Write(0x5811, 0x0d),
    RegisterOp::Write(0x5812, 0x05),
    RegisterOp::Write(0x5813, 0x01),
    RegisterOp::Write(0x5814, 0x01),
    RegisterOp::Write(0x5815, 0x01),
    RegisterOp::Write(0x5816, 0x05),
    RegisterOp::Write(0x5817, 0x0d),
    RegisterOp::Write(0x5818, 0x08),
    RegisterOp::Write(0x5819, 0x06),
    RegisterOp::Write(0x581a, 0x05),
    RegisterOp::Write(0x581b, 0x07),
    RegisterOp::Write(0x581c, 0x0b),
    RegisterOp::Write(0x581d, 0x0d),
    RegisterOp::Write(0x581e, 0x12),
    RegisterOp::Write(0x581f, 0x0d),
    RegisterOp::Write(0x5820, 0x0e),
    RegisterOp::Write(0x5821, 0x10),
    RegisterOp::Write(0x5822, 0x10),
    RegisterOp::Write(0x5823, 0x1e),
    RegisterOp::Write(0x5824, 0x53),
    RegisterOp::Write(0x5825, 0x15),
    RegisterOp::Write(0x5826, 0x05),
    RegisterOp::Write(0x5827, 0x14),
    RegisterOp::Write(0x5828, 0x54),
    RegisterOp::Write(0x5829, 0x25),
    RegisterOp::Write(0x582a, 0x33),
    RegisterOp::Write(0x582b, 0x33),
    RegisterOp::Write(0x582c, 0x34),
    RegisterOp::Write(0x582d, 0x16),
    RegisterOp::Write(0x582e, 0x24),
    RegisterOp::Write(0x582f, 0x41),
    RegisterOp::Write(0x5830, 0x50),
    RegisterOp::Write(0x5831, 0x42),
    RegisterOp::Write(0x5832, 0x15),
    RegisterOp::Write(0x5833, 0x25),
    RegisterOp::Write(0x5834, 0x34),
    RegisterOp::Write(0x5835, 0x33),
    RegisterOp::Write(0x5836, 0x24),
    RegisterOp::Write(0x5837, 0x26),
    RegisterOp::Write(0x5838, 0x54),
    RegisterOp::Write(0x5839, 0x25),
    RegisterOp::Write(0x583a, 0x15),
    RegisterOp::Write(0x583b, 0x25),
    RegisterOp::Write(0x583c, 0x53),
    RegisterOp::Write(0x583d, 0xcf),
    RegisterOp::Write(0x3a0f, 0x30),
    RegisterOp::Write(0x3a10, 0x28),
    RegisterOp::Write(0x3a1b, 0x30),
    RegisterOp::Write(0x3a1e, 0x28),
    RegisterOp::Write(0x3a11, 0x60),
    RegisterOp::Write(0x3a1f, 0x14),
    RegisterOp::Write(0x5302, 0x28),
    RegisterOp::Write(0x5303, 0x20),
    RegisterOp::Write(0x5306, 0x1c), //de-noise offset 1
    RegisterOp::Write(0x5307, 0x28), //de-noise offset 2
    RegisterOp::Write(0x4002, 0xc5),
    RegisterOp::Write(0x4003, 0x81),
    RegisterOp::Write(0x4005, 0x12),
    RegisterOp::Write(0x5688, 0x11),
    RegisterOp::Write(0x5689, 0x11),
    RegisterOp::Write(0x568a, 0x11),
    RegisterOp::Write(0x568b, 0x11),
    RegisterOp::Write(0x568c, 0x11),
    RegisterOp::Write(0x568d, 0x11),
    RegisterOp::Write(0x568e, 0x11),
    RegisterOp::Write(0x568f, 0x11),
    RegisterOp::Write(0x5580, 0x06),
    RegisterOp::Write(0x5588, 0x00),
    RegisterOp::Write(0x5583, 0x40),
    RegisterOp::Write(0x5584, 0x2c),
    RegisterOp::Write(ISP_CONTROL_01, 0x83), // turn color matrix, awb and SDE
];

/// JPEG output format configuration.
///
/// Ported from Espressif's `sensor_fmt_jpeg`.
pub const JPEG_REGS: &[RegisterOp] = &[
    RegisterOp::Write(FORMAT_CTRL, 0x00),   // YUV422
    RegisterOp::Write(FORMAT_CTRL00, 0x30), // YUYV
    RegisterOp::Write(0x3002, 0x00),        //0x1c to 0x00 !!!
    RegisterOp::Write(0x3006, 0xff),        //0xc3 to 0xff !!!
    RegisterOp::Write(0x471c, 0x50),        //0xd0 to 0x50 !!!
];

#[allow(dead_code)]
pub const RAW_REGS: &[RegisterOp] = &[
    RegisterOp::Write(FORMAT_CTRL00, 0x00), // RAW
];

#[allow(dead_code)]
pub const GRAYSCALE_REGS: &[RegisterOp] = &[
    RegisterOp::Write(FORMAT_CTRL, 0x00),   // YUV422
    RegisterOp::Write(FORMAT_CTRL00, 0x10), // Y8
];

#[allow(dead_code)]
pub const YUV422_REGS: &[RegisterOp] = &[
    RegisterOp::Write(FORMAT_CTRL, 0x00),   // YUV422
    RegisterOp::Write(FORMAT_CTRL00, 0x30), // YUYV
];

#[allow(dead_code)]
pub const RGB565_REGS: &[RegisterOp] = &[
    RegisterOp::Write(FORMAT_CTRL, 0x01),   // RGB
    RegisterOp::Write(FORMAT_CTRL00, 0x61), // RGB565 (BGR)
];
