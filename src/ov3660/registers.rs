//! OV3660 sensor register definitions.

/// System control register, including software reset.
pub const SYSTEM_CTRL0: u16 = 0x3008;

/// Software reset command.
pub const SOFTWARE_RESET: u8 = 0x82;

/// Sensor product identifier registers.
pub const PID_HIGH: u16 = 0x300A;
pub const PID_LOW: u16 = 0x300B;

pub const DRIVE_CAPABILITY: u16 = 0x302C;

pub const CLOCK_POL_CONTROL: u16 = 0x4740;

pub const COMPRESSION_CTRL0E: u16 = 0x440e;

pub const ISP_CONTROL_01: u16 = 0x5001;

pub const FORMAT_CTRL: u16 = 0x501f;

pub const FORMAT_CTRL00: u16 = 0x4300;
