//! Capture a JPEG frame from an OV3660 on a Freenove ESP32-S3.
//!
//! Configures the sensor for QVGA (320x240) JPEG output, captures one
//! frame using asynchronous DMA, and prints it as hexadecimal over UART.
//!
//! The hex output can be reconstructed into a JPEG file on the host.
//!
//! Requires the `ov3660`, `jfif`, and `defmt` features.

#![no_std]
#![no_main]

use embedded_hal::delay::DelayNs;

use esp32s3_cam_async::{
    AsyncCameraDriver, CameraCapture, CaptureConfig, ConverterMode,
    jfif::{JpegStreamParser, ParserProgressing},
    ov3660::{OV3660_PID, Ov3660},
};

use esp_hal::{
    dma::{DmaChannel, DmaRxStreamBuf},
    dma_rx_stream_buffer,
    interrupt::software::SoftwareInterruptControl,
    lcd_cam::{
        ByteOrder,
        cam::{Config as CamConfig, EofMode, VhdeMode, VsyncFilterThreshold},
    },
    time::Rate,
    timer::timg::TimerGroup,
};

use static_cell::StaticCell;

use esp_backtrace as _;
use esp_println as _;

const JPEG_WIDTH: u16 = 320;
const JPEG_HEIGHT: u16 = 240;
const JPEG_QUALITY: u8 = 12;

const DMA_SIZE: usize = 4096;
const DMA_BLOCK: usize = 512;
const JPEG_BUF_SIZE: usize = 64 * 1024;
const HEX_ROW: usize = 64;

static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

esp_bootloader_esp_idf::esp_app_desc!();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    defmt::info!("boot: OV3660 JPEG capture");

    // Initialize the Embassy runtime.
    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);

    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // Configure LCD_CAM and the sensor clock (XCLK).
    let cam_config = CamConfig::default()
        .with_frequency(Rate::from_mhz(10))
        .with_eof_mode(EofMode::VsyncSignal)
        .with_vh_de_mode(VhdeMode::De)
        .with_vsync_filter_threshold(VsyncFilterThreshold::Four)
        .with_byte_order(ByteOrder::Native)
        .with_enable_2byte_mode(false);

    let camera = AsyncCameraDriver::new(
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
    .expect("Camera setup failed");

    // SCCB interface.
    let i2c = esp_hal::i2c::master::I2c::new(p.I2C0, esp_hal::i2c::master::Config::default())
        .unwrap()
        .with_sda(p.GPIO4)
        .with_scl(p.GPIO5);

    let executor = EXECUTOR.init(esp_rtos::embassy::Executor::new());

    executor.run(|spawner| {
        spawner.spawn(capture_task(camera, i2c).unwrap());
    });
}

#[embassy_executor::task]
async fn capture_task(
    camera: AsyncCameraDriver<'static>,
    mut i2c: esp_hal::i2c::master::I2c<'static, esp_hal::Blocking>,
) {
    let sensor = Ov3660::new();
    let mut delay = esp_hal::delay::Delay::new();

    // Allow XCLK and the sensor to stabilize.
    delay.delay_ms(100);

    let pid = sensor.read_id(&mut i2c).expect("Sensor ID read failed");
    assert_eq!(pid, OV3660_PID, "Unexpected camera sensor");

    defmt::info!("OV3660 detected: 0x{:04x}", pid);

    // Initialize and configure JPEG output.
    sensor
        .init(&mut i2c, &mut delay)
        .expect("Sensor initialization failed");

    sensor
        .set_jpeg_format(&mut i2c, &mut delay)
        .expect("JPEG format configuration failed");

    sensor
        .set_framesize_qvga(&mut i2c)
        .expect("QVGA configuration failed");

    sensor
        .set_jpeg_quality(&mut i2c, JPEG_QUALITY)
        .expect("JPEG quality configuration failed");

    // Temporary workaround: the automatic white balance configuration
    // currently produces a strong green cast on the tested Freenove board.
    // Use Espressif's Sunny preset until automatic WB is validated.
    configure_manual_white_balance(&sensor, &mut i2c);

    defmt::info!(
        "OV3660 configured: JPEG {}x{}, quality {}",
        JPEG_WIDTH,
        JPEG_HEIGHT,
        JPEG_QUALITY
    );

    // Set up DMA capture and the JPEG stream parser.
    let cam_buf: DmaRxStreamBuf = dma_rx_stream_buffer!(DMA_SIZE, DMA_BLOCK);

    let mut capture = CameraCapture::new(camera, cam_buf, CaptureConfig::default());

    capture.start().expect("DMA capture failed");

    defmt::info!("Waiting for JPEG frame...");

    let mut dma_chunk = [0u8; DMA_SIZE];
    let mut jpeg_out = [0u8; JPEG_BUF_SIZE];
    let mut parser = JpegStreamParser::default();

    loop {
        let result = capture.next_chunk(&mut dma_chunk).await;

        if result.resyncing {
            parser.reset_state();
            continue;
        }

        if result.len == 0 {
            continue;
        }

        let chunk = &dma_chunk[..result.len];
        parser.reset_input_offset();

        loop {
            match parser.parse(chunk, &mut jpeg_out) {
                Ok(ParserProgressing::InputBufferEmpty) => break,

                Ok(ParserProgressing::EndOfImage) => {
                    let frame = &jpeg_out[..parser.bytes_written()];

                    if frame.starts_with(&[0xFF, 0xD8]) && frame.ends_with(&[0xFF, 0xD9]) {
                        defmt::info!("JPEG frame captured: {} bytes", frame.len());

                        capture.stop_capture();
                        dump_frame_hex(frame);

                        defmt::info!("Capture complete");
                        return;
                    }

                    defmt::warn!("Invalid JPEG boundaries");
                    parser.reset_state();
                    break;
                }

                Ok(ParserProgressing::OutputBufferFull) => {
                    capture.stop_capture();
                    defmt::error!("JPEG frame exceeds output buffer");
                    return;
                }

                Err(state) => {
                    // The parser resets itself after structural errors.
                    defmt::warn!("JPEG parser error: {:?}", state);
                    break;
                }
            }
        }
    }
}

/// Apply Espressif's Sunny white balance preset.
///
/// This is a temporary capture-example workaround, not a replacement
/// for automatic white balance support in the sensor driver.
fn configure_manual_white_balance(
    sensor: &Ov3660,
    i2c: &mut esp_hal::i2c::master::I2c<'_, esp_hal::Blocking>,
) {
    // Espressif's Sunny white balance preset.
    const SUNNY_WB: &[(u16, u8)] = &[
        (0x3400, 0x05), // Red high
        (0x3401, 0xE0), // Red low
        (0x3402, 0x04), // Green high
        (0x3403, 0x10), // Green low
        (0x3404, 0x05), // Blue high
        (0x3405, 0x40), // Blue low
    ];

    // Enable manual white balance.
    sensor
        .write_reg(i2c, 0x3406, 0x01)
        .expect("Failed to enable manual white balance");

    for &(reg, value) in SUNNY_WB {
        sensor
            .write_reg(i2c, reg, value)
            .expect("White balance register write failed");
    }
}

/// Print a complete JPEG as hexadecimal for host-side reconstruction.
fn dump_frame_hex(jpeg: &[u8]) {
    defmt::info!("PHOTO_BEGIN {}", jpeg.len());

    for row in jpeg.chunks(HEX_ROW) {
        let mut hex = [0u8; HEX_ROW * 2];

        for (i, &byte) in row.iter().enumerate() {
            hex[i * 2] = hex_digit(byte >> 4);
            hex[i * 2 + 1] = hex_digit(byte & 0x0F);
        }

        let line = core::str::from_utf8(&hex[..row.len() * 2]).expect("Invalid hex encoding");

        defmt::info!("PHOTOHEX {=str}", line);
    }

    defmt::info!("PHOTO_END");
}

fn hex_digit(value: u8) -> u8 {
    match value {
        0..=9 => b'0' + value,
        _ => b'a' + value - 10,
    }
}
