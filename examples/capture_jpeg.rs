//! Capture a complete JPEG frame from an OV2640 in JPEG output mode,
//! extracted from the raw DMA byte stream using [`JpegStreamParser`].
//!
//! # Hardware wiring (ESP32-S3)
//!
//! | Signal | GPIO |
//! |--------|------|
//! | MCLK   | 15   |
//! | VSYNC  | 6    |
//! | HREF   | 7    |
//! | PCLK   | 13   |
//! | D0–D7  | 11, 9, 8, 10, 12, 18, 17, 16 |
//! | SDA    | 4    |
//! | SCL    | 5    |
//!
//! # OV2640 JPEG mode
//!
//! The OV2640 can output compressed JPEG directly. Configure it with
//! [`PixelFormat::Jpeg`] in your [`Ov2640Config`]; the `quality` field sets
//! the `QS` quantization register (lower = better quality, larger files).
//! The raw DMA byte stream carries a continuous JPEG byte sequence;
//! [`JpegStreamParser`] extracts one self-contained SOI…EOI frame at a time.
//!
//! JPEG is an opaque byte stream, so the LCD-CAM conversion unit must stay out of
//! the way: pass [`ConverterMode::Bypass`] to [`AsyncCameraDriver::new`] (same as
//! the RGB565 example). Only [`PixelFormat::Yuv422`] needs
//! [`ConverterMode::Yuv422Passthrough`].
//!
//! # Build & flash (ESP32-S3-CAM)
//!
//! Logs are emitted with `defmt` over **UART0** — the usual ESP32-S3-CAM is
//! reached through a USB-UART bridge (e.g. `/dev/cu.usbserial-*`), not the native
//! USB Serial/JTAG, so esp-println is built with the `uart` transport.
//! `esp-println`'s `defmt-espflash` feature installs the defmt global logger and
//! `esp-backtrace` routes panics through defmt.
//!
//! **The serial monitor must decode defmt** (`espflash ... --log-format defmt`)
//! or you'll see nothing but the ESP-IDF bootloader text. espflash has no config
//! key for this, so the flag must be on the espflash command line:
//!
//! ```sh
//! cargo run -p esp32s3-cam-async --example capture_jpeg \
//!   --features ov2640,jfif,defmt -- --log-format defmt
//! ```
//!
//! Cargo forwards everything after `--` to the runner, so espflash ends up as
//! `espflash flash --monitor <elf> --log-format defmt`. In an IDE run
//! configuration, append `-- --log-format defmt` to the cargo command the same
//! way. `DEFMT_LOG` controls verbosity, e.g. `DEFMT_LOG=debug`.
//!
//! # Requirements
//!
//! - Espressif `esp` Rust toolchain (Xtensa fork).
//! - The Embassy time driver is provided here via `esp-rtos`; the
//!   `#[panic_handler]` comes from the `esp-backtrace` dev-dependency.
#![no_std]
#![no_main]

use esp_hal::{
    dma::DmaChannel,
    dma_rx_stream_buffer,
    dma::DmaRxStreamBuf,
    interrupt::software::SoftwareInterruptControl,
    lcd_cam::{
        ByteOrder,
        cam::{Config as CamConfig, EofMode, VhdeMode, VsyncFilterThreshold},
    },
    time::Rate,
    timer::timg::TimerGroup,
};
use esp32s3_cam_async::{
    jfif::{JpegStreamParser, ParserProgressing},
    ov2640::{Ov2640, Ov2640Config, PixelFormat, Resolution},
    AsyncCameraDriver, CameraCapture, CaptureConfig, ConverterMode,
};
use embassy_time::Instant;
use static_cell::StaticCell;

// Registers the defmt global logger (`defmt-espflash`) so `defmt::*` macros work.
use esp_println as _;
// Pulls in the `#[panic_handler]`, which prints the panic via defmt.
use esp_backtrace as _;

const JPEG_BUF_SIZE: usize = 128 * 1024;
const DMA_SIZE: usize = 4096;
const DMA_BLOCK: usize = 512;

const JPEG_QUALITY: u8 = 20;
/// JPEG output resolution. Supported JPEG resolutions (see `docs/OV2640.md`):
/// - `Vga` — hardware-verified path (clean 1.25× downscale from SVGA).
/// - `Svga` — reuses the VGA path with a 1:1 scaler.
/// - `Uxga` — native 1600×1200 (largest frames; derived from esp32-camera).
///
/// `Qvga`/`Qqvga` JPEG is intentionally unsupported (the CIF-derived preset emits
/// an invalid stream); use a raw format at those resolutions instead.
const CAPTURE_RESOLUTION: Resolution = Resolution::Uxga;

/// Emit a full-frame hex dump (for offline `xxd -r -p` reconstruction) at most
/// once per this interval.
const HEX_DUMP_INTERVAL_MS: u64 = 10_000;
/// Bytes per hex line in the dump; 64 bytes → 128 hex chars per line.
const HEX_DUMP_ROW: usize = 64;

esp_bootloader_esp_idf::esp_app_desc!();

static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    defmt::info!("boot: ESP32-S3-CAM OV2640 JPEG capture");

    // esp-rtos owns the scheduler and the Embassy time driver. It MUST be started
    // before the executor runs, otherwise the first `Timer::after` (used by the
    // OV2640 init delays) panics with a `None` unwrap in the time driver.
    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

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
        ConverterMode::Bypass, // JPEG is an opaque byte stream → converter bypassed
        p.GPIO15, // MCLK
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
    let sensor = Ov2640::new(Ov2640Config {
        format: PixelFormat::Jpeg { quality: JPEG_QUALITY },
        resolution: CAPTURE_RESOLUTION, // see the const doc for supported JPEG sizes
        ..Default::default()
    });
    sensor.check_id(&mut i2c).expect("OV2640 not found");
    defmt::info!("OV2640 detected on SCCB");
    sensor.init(&mut i2c).await.expect("OV2640 JPEG init failed");
    defmt::info!("OV2640 init OK ({:?} JPEG, quality {:?})", CAPTURE_RESOLUTION, JPEG_QUALITY);

    let cam_buf: DmaRxStreamBuf = dma_rx_stream_buffer!(DMA_SIZE, DMA_BLOCK);
    let mut capture = CameraCapture::new(camera, cam_buf, CaptureConfig::default());
    capture.start().unwrap();
    defmt::info!("DMA capture started, streaming frames...");

    let mut dma_chunk = [0u8; DMA_SIZE];
    let mut jpeg_out = [0u8; JPEG_BUF_SIZE];
    let mut parser = JpegStreamParser::default();

    // Frame-rate accounting for periodic stats.
    let mut frames: u32 = 0;
    let mut window_frames: u32 = 0;
    let mut window_start = Instant::now();

    // Throttle the hex dump to once per HEX_DUMP_INTERVAL_MS; `dump_seq` is a
    // monotonic mark number so each dump is easy to isolate in the log.
    let mut last_hex_dump = Instant::now();
    let mut dump_seq: u32 = 0;

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
                    frames += 1;
                    window_frames += 1;
                    on_jpeg_frame(frames, frame);

                    // Periodically dump the whole frame as hex for offline
                    // reconstruction (see `dump_frame_hex` for the recipe).
                    if last_hex_dump.elapsed().as_millis() >= HEX_DUMP_INTERVAL_MS {
                        dump_seq += 1;
                        dump_frame_hex(dump_seq, frames, frame);
                        last_hex_dump = Instant::now();
                    }

                    parser.reset_output_offset();

                    // Print an FPS summary roughly once per second.
                    let elapsed = window_start.elapsed();
                    if elapsed.as_millis() >= 1000 {
                        let fps = (window_frames as u64 * 1000) / elapsed.as_millis();
                        defmt::info!("~{} fps ({} frames total)", fps, frames);
                        window_frames = 0;
                        window_start = Instant::now();
                    }
                }

                Ok(ParserProgressing::OutputBufferFull) => {
                    defmt::warn!("frame > {} B buffer, dropping", JPEG_BUF_SIZE);
                    parser.reset_state();
                    break;
                }

                Err(_) => break,
            }
        }
    }
}

/// Called for each complete JPEG frame extracted from the DMA stream.
fn on_jpeg_frame(index: u32, jpeg: &[u8]) {
    // Sanity-check the JPEG markers so the log reflects a valid frame.
    let soi = jpeg.first_chunk::<2>() == Some(&[0xFF, 0xD8]);
    let eoi = jpeg.last_chunk::<2>() == Some(&[0xFF, 0xD9]);
    defmt::info!(
        "frame #{}: {} bytes (SOI={}, EOI={})",
        index,
        jpeg.len(),
        soi,
        eoi
    );
}

/// Dump a complete JPEG frame as rows of contiguous hex digits, bracketed by
/// `MARK <seq> BEGIN/END` markers, so the serial log can be turned back into a
/// `.jpg` offline. `seq` is a monotonic dump counter (1, 2, 3, …); `frame_no` is
/// the source frame index, logged for reference.
///
/// Each hex line is the `defmt` message, so the espflash monitor renders it as
/// `[INFO ] <hex> (<module> <path:line>)` — the hex sits between `] ` and ` (`.
/// To rebuild mark 3 from a captured log:
///
/// ```sh
/// sed -n '/MARK 3 BEGIN/,/MARK 3 END/p' log.txt \
///     | sed -nE 's/.*\] ([0-9a-f]+) \(.*/\1/p' \
///     | tr -d '\n' | xxd -r -p > frame.jpg && open frame.jpg
/// ```
///
/// The `BEGIN`/`END` lines contain letters and spaces, so they never match the
/// pure-hex extraction and are skipped automatically. `od` works too if you
/// prefer: feed the same hex stream to `xxd -r -p`; `od` is only for inspecting.
///
/// Note: emitting a full frame stalls the capture loop briefly while the UART
/// drains, so the next DMA window typically resyncs — expected for a debug dump.
fn dump_frame_hex(seq: u32, frame_no: u32, jpeg: &[u8]) {
    defmt::info!("MARK {} BEGIN (frame {}, {} bytes)", seq, frame_no, jpeg.len());
    let mut line = [0u8; HEX_DUMP_ROW * 2];
    for row in jpeg.chunks(HEX_DUMP_ROW) {
        for (i, &b) in row.iter().enumerate() {
            line[i * 2] = hex_nibble(b >> 4);
            line[i * 2 + 1] = hex_nibble(b & 0x0F);
        }
        // All bytes are ASCII hex digits, so this is always valid UTF-8.
        let s = core::str::from_utf8(&line[..row.len() * 2]).unwrap_or("");
        defmt::info!("{=str}", s);
    }
    defmt::info!("MARK {} END", seq);
}

/// Map a 4-bit value (0–15) to its lowercase ASCII hex digit.
#[inline]
fn hex_nibble(v: u8) -> u8 {
    if v < 10 {
        b'0' + v
    } else {
        b'a' + (v - 10)
    }
}
