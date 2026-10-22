//! Detect an OV3660 sensor over SCCB on Freenove ESP32-S3.
//!
//! Expected sensor address: 0x3C
//! Expected PID: 0x3660

#![no_std]
#![no_main]

use esp32s3_cam_async::{
    ov3660::{Ov3660, OV3660_PID},
    AsyncCameraDriver, ConverterMode,
};

use esp_hal::{
    dma::DmaChannel,
    lcd_cam::{
        cam::{Config as CamConfig, EofMode, VhdeMode, VsyncFilterThreshold},
        ByteOrder,
    },
    time::Rate,
};

use esp_backtrace as _;
use esp_println as _;

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    defmt::info!("boot: OV3660 sensor detection");

    // Configure LCD_CAM and the sensor master clock (XCLK).
    let cam_config = CamConfig::default()
        .with_frequency(Rate::from_mhz(10))
        .with_eof_mode(EofMode::VsyncSignal)
        .with_vh_de_mode(VhdeMode::De)
        .with_vsync_filter_threshold(VsyncFilterThreshold::Four)
        .with_byte_order(ByteOrder::Native)
        .with_enable_2byte_mode(false);

    let _camera = AsyncCameraDriver::new(
        p.LCD_CAM,
        p.DMA_CH0.split().0,
        cam_config,
        ConverterMode::Bypass,
        p.GPIO15, // XCLK
        p.GPIO13, // PCLK
        p.GPIO6,  // VSYNC
        p.GPIO7,  // HREF
        p.GPIO11, // D0
        p.GPIO9,  // D1
        p.GPIO8,  // D2
        p.GPIO10, // D3
        p.GPIO12, // D4
        p.GPIO18, // D5
        p.GPIO17, // D6
        p.GPIO16, // D7
    )
    .unwrap();

    defmt::info!("Camera clock configured");

    let mut i2c = esp_hal::i2c::master::I2c::new(p.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(p.GPIO4)
        .with_scl(p.GPIO5);

    let sensor = Ov3660::new();

    let mut delay = esp_hal::delay::Delay::new();

    // Allow the sensor to stabilize after XCLK starts.
    delay.delay_millis(100u32);

    // Verify the sensor before initialization.
    match sensor.read_id(&mut i2c) {
        Ok(OV3660_PID) => {
            defmt::info!("OV3660 detected: PID=0x3660");
        }
        Ok(pid) => {
            defmt::error!("Unexpected sensor PID: 0x{:04x}", pid);
            loop {
                core::hint::spin_loop();
            }
        }
        Err(err) => {
            defmt::error!("Sensor detection failed: {:?}", defmt::Debug2Format(&err));
            loop {
                core::hint::spin_loop();
            }
        }
    }

    // Apply OV3660 default register configuration.
    defmt::info!("Initializing OV3660...");

    match sensor.init(&mut i2c, &mut delay) {
        Ok(()) => {
            defmt::info!("OV3660 initialization completed");
        }
        Err(err) => {
            defmt::error!(
                "OV3660 initialization failed: {:?}",
                defmt::Debug2Format(&err)
            );
            loop {
                core::hint::spin_loop();
            }
        }
    }

    defmt::info!("Configuring JPEG format...");
    sensor
        .set_jpeg_format(&mut i2c, &mut delay)
        .expect("JPEG format configuration failed");

    defmt::info!("Configuring QVGA 320x240...");
    sensor
        .set_framesize_qvga(&mut i2c)
        .expect("QVGA configuration failed");

    defmt::info!("Configuring JPEG quality...");
    sensor
        .set_jpeg_quality(&mut i2c, 12)
        .expect("JPEG quality configuration failed");

    defmt::info!("OV3660 JPEG QVGA configuration completed");

    let width_high = sensor.read_reg(&mut i2c, 0x3808).unwrap();
    let width_low = sensor.read_reg(&mut i2c, 0x3809).unwrap();

    let height_high = sensor.read_reg(&mut i2c, 0x380A).unwrap();
    let height_low = sensor.read_reg(&mut i2c, 0x380B).unwrap();

    let width = u16::from_be_bytes([width_high, width_low]);
    let height = u16::from_be_bytes([height_high, height_low]);

    defmt::info!("Output dimensions: {}x{}", width, height);

    if width == 320 && height == 240 {
        defmt::info!("QVGA register verification: PASS");
    } else {
        defmt::error!("QVGA register verification: FAIL");
    }

    defmt::info!("Configuring JPEG format...");

    // Check that the sensor remains responsive after initialization.
    match sensor.read_id(&mut i2c) {
        Ok(OV3660_PID) => {
            defmt::info!("OV3660 post-init PID check: PASS");
        }
        Ok(pid) => {
            defmt::error!("Unexpected post-init PID: 0x{:04x}", pid);
        }
        Err(err) => {
            defmt::error!("Post-init PID read failed: {:?}", defmt::Debug2Format(&err));
        }
    }

    loop {
        core::hint::spin_loop();
    }
}
