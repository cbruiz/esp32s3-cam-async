//! Capture raw YUV422 lines from an OV2640 via DMA.
//!
//! Identical in structure to `capture_rgb565.rs`, but the sensor is configured
//! for YUV422 output and the BMP dump converts YUYV → RGB888 so the window can
//! be eyeballed offline.
//!
//! # Hardware wiring (ESP32-S3-CAM)
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
//! # Build & flash (ESP32-S3-CAM)
//!
//! YUV422 passthrough is selected at runtime by passing
//! [`ConverterMode::Yuv422Passthrough`] to [`AsyncCameraDriver::new`] — it tracks
//! the [`PixelFormat::Yuv422`] sensor config. This configures the LCD-CAM
//! conversion unit for YUV422 passthrough; the [`ConverterMode::Bypass`] default
//! (RGB565 byte-pairing) would instead mangle YUV data into a fixed pattern.
//!
//! ```sh
//! cargo run -p esp32s3-cam-async --example capture_yuv422 \
//!   --features ov2640,defmt -- --log-format defmt
//! ```
//!
//! `DEFMT_LOG` controls verbosity, e.g. `DEFMT_LOG=debug`.
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
    timer::timg::TimerGroup,
};
use esp32s3_cam_async::{
    ov2640::{Ov2640, Ov2640Config, PixelFormat, Resolution},
    AsyncCameraDriver, CameraCapture, CaptureConfig, CaptureEvent, ConverterMode,
};
use embassy_time::Instant;
use static_cell::StaticCell;

// Registers the defmt global logger (`defmt-espflash`) so `defmt::*` macros work.
use esp_println as _;
// Pulls in the `#[panic_handler]`, which prints the panic via defmt.
use esp_backtrace as _;
use esp_hal::time::Rate;
use esp32s3_cam_async::ov2640::ClockConfig;

const RESOLUTION: Resolution = Resolution::Qqvga;
const CLOCK: ClockConfig = ClockConfig::new(Rate::from_mhz(24), 0, None);
const LINE_BYTES: usize = RESOLUTION.line_bytes();
const DMA_SIZE: usize = 2048;
const DMA_BLOCK: usize = 512;

/// Frame geometry (derived from the configured resolution).
const FRAME_W: usize = RESOLUTION.dims().0 as usize;
const FRAME_H: usize = RESOLUTION.dims().1 as usize;
/// Bytes in one full YUV422 frame (2 bytes/pixel, YUYV).
const FRAME_BYTES: usize = FRAME_W * FRAME_H * 2;

/// Desired BMP dump window size, kept small so the hex output stays manageable
/// over UART. Clamped to the frame below so it never exceeds it (e.g. at QQVGA
/// the frame is only 160×120). Frame widths and 128 are all multiples of 4, so
/// the clamped `WIN_W` stays a multiple of 4 → the BMP needs no row padding.
const WIN_DESIRED: usize = 128;
const WIN_W: usize = if WIN_DESIRED < FRAME_W { WIN_DESIRED } else { FRAME_W };
const WIN_H: usize = if WIN_DESIRED < FRAME_H { WIN_DESIRED } else { FRAME_H };
/// Top-left corner of the dumped window (centred).
const WIN_X0: usize = (FRAME_W - WIN_W) / 2;
const WIN_Y0: usize = (FRAME_H - WIN_H) / 2;

/// Byte layout within each 4-byte / 2-pixel YUV422 group, as it arrives over the
/// LCD-CAM. Confirmed empirically to be **YUYV** = `[Y0, Cb, Y1, Cr]` (Y in the
/// even bytes, chroma in the odd bytes ≈128). Offsets within the group:
///   - Y of pixel `px`: byte `px*2 + YUV_Y_OFF`
///   - Cb / Cr (shared by the pair): bytes `pair + YUV_CB_OFF` / `pair + YUV_CR_OFF`
/// For UYVY `[Cb, Y0, Cr, Y1]` use `Y_OFF=1, CB_OFF=0, CR_OFF=2`. If red/blue
/// look swapped, exchange `YUV_CB_OFF` and `YUV_CR_OFF`.
const YUV_Y_OFF: usize = 0;
const YUV_CB_OFF: usize = 1;
const YUV_CR_OFF: usize = 3;

/// Emit a full-frame BMP hex dump (for offline `xxd -r -p` reconstruction) at
/// most once per this interval.
const BMP_DUMP_INTERVAL_MS: u64 = 10_000;
/// Bytes per hex line in the dump; 64 bytes → 128 hex chars per line.
const HEX_DUMP_ROW: usize = 64;

esp_bootloader_esp_idf::esp_app_desc!();

static EXECUTOR: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();

/// Full YUV422 frame buffer. Lives in .bss so there is no large stack temporary;
/// only the single capture task touches it, so the raw-pointer access is sound.
static mut FRAME_BUF: [u8; FRAME_BYTES] = [0u8; FRAME_BYTES];

#[esp_hal::main]
fn main() -> ! {
    let p = esp_hal::init(esp_hal::Config::default());

    defmt::info!("boot: ESP32-S3-CAM OV2640 YUV422 capture");

    // esp-rtos owns the scheduler and the Embassy time driver. It MUST be started
    // before the executor runs, otherwise the first `Timer::after` (used by the
    // OV2640 init delays) panics with a `None` unwrap in the time driver.
    let timg0 = TimerGroup::new(p.TIMG0);
    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    let cam_config = CamConfig::default()
        .with_frequency(CLOCK.xclk())
        .with_eof_mode(EofMode::VsyncSignal)
        .with_vh_de_mode(VhdeMode::De)
        .with_vsync_filter_threshold(VsyncFilterThreshold::Four)
        .with_byte_order(ByteOrder::Native)
        .with_enable_2byte_mode(false);

    let camera = AsyncCameraDriver::new(
        p.LCD_CAM,
        p.DMA_CH0.split().0,
        cam_config,
        ConverterMode::Yuv422Passthrough, // matches PixelFormat::Yuv422 (YUYV)
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
        clock: CLOCK,
        format: PixelFormat::Yuv422,
        resolution: RESOLUTION,
        ..Default::default()
    });
    sensor.check_id(&mut i2c).expect("OV2640 not found");
    defmt::info!("OV2640 detected on SCCB");
    sensor.init(&mut i2c).await.expect("OV2640 init failed");
    defmt::info!("OV2640 init OK (YUV422, {} bytes/line)", LINE_BYTES);

    let cam_buf: DmaRxStreamBuf = dma_rx_stream_buffer!(DMA_SIZE, DMA_BLOCK);
    let mut capture = CameraCapture::new(camera, cam_buf, CaptureConfig::default());
    capture.start().unwrap();
    defmt::info!("DMA capture started, streaming frames...");

    let mut dma_chunk = [0u8; DMA_SIZE];

    // Full-frame accumulator. `frame_pos` is the running byte offset of the
    // current frame; a VSYNC EOF closes the frame and resets it.
    let frame: &mut [u8; FRAME_BYTES] = unsafe { &mut *core::ptr::addr_of_mut!(FRAME_BUF) };
    let mut frame_pos = 0usize;

    // Frame-rate accounting for periodic stats.
    let mut frames: u32 = 0;
    let mut window_frames: u32 = 0;
    let mut window_start = Instant::now();

    // Throttle the BMP dump to once per BMP_DUMP_INTERVAL_MS; `dump_seq` is a
    // monotonic mark number so each dump is easy to isolate in the log.
    let mut last_bmp_dump = Instant::now();
    let mut dump_seq: u32 = 0;

    loop {
        let result = capture.next_chunk(&mut dma_chunk).await;

        if result.event == CaptureEvent::DmaError {
            defmt::warn!("DMA error, resyncing");
            frame_pos = 0;
        }

        if result.resyncing || result.len == 0 {
            // Drop the partial frame on a frame boundary so the next one is clean.
            if result.eof {
                frame_pos = 0;
            }
            continue;
        }

        // Accumulate raw YUV422 bytes into the frame buffer (clamped to one frame).
        let data = &dma_chunk[..result.len];
        let n = (FRAME_BYTES - frame_pos).min(data.len());
        frame[frame_pos..frame_pos + n].copy_from_slice(&data[..n]);
        frame_pos += n;

        if result.eof {
            frames += 1;
            window_frames += 1;
            on_frame_end(frames, frame_pos / LINE_BYTES);

            // Dump a complete frame as a BMP roughly once per interval.
            if frame_pos >= FRAME_BYTES
                && last_bmp_dump.elapsed().as_millis() >= BMP_DUMP_INTERVAL_MS
            {
                dump_seq += 1;
                dump_frame_bmp(dump_seq, frames, &frame[..FRAME_BYTES]);
                last_bmp_dump = Instant::now();
            }

            // Print an FPS summary roughly once per second.
            let elapsed = window_start.elapsed();
            if elapsed.as_millis() >= 1000 {
                let fps = (window_frames as u64 * 1000) / elapsed.as_millis();
                defmt::info!("~{} fps ({} frames total)", fps, frames);
                window_frames = 0;
                window_start = Instant::now();
            }

            frame_pos = 0;
        }
    }
}

/// Called at each VSYNC EOF — `lines_received` reflects how many full lines came in.
fn on_frame_end(index: u32, lines_received: usize) {
    defmt::debug!("frame #{}: {} lines", index, lines_received);
}

/// Dump a centred `WIN_W`×`WIN_H` window of the YUV422 frame as a 24-bit BMP,
/// hex-encoded, bracketed by `MARK <seq> BEGIN/END` markers so the serial log can
/// be rebuilt into a `.bmp`. `seq` is a monotonic dump counter; `frame_no` is the
/// source frame index, logged for reference.
///
/// Each hex line is the `defmt` message, so espflash renders it as
/// `[INFO ] <hex> (<module> <path:line>)` — the hex sits between `] ` and ` (`.
/// To rebuild mark 3:
///
/// ```sh
/// sed -n '/MARK 3 BEGIN/,/MARK 3 END/p' log.txt \
///   | sed -nE 's/.*\] ([0-9a-f]+) \(.*/\1/p' \
///   | tr -d '\n' | xxd -r -p > win.bmp
/// open win.bmp
/// ```
///
/// YUV422 byte order is assumed `[Y0, Cb, Y1, Cr]` (YUYV, BSYM=0) per the sensor
/// config. If colours look swapped (e.g. red/blue), adjust the Cb/Cr byte
/// offsets in the loop below.
fn dump_frame_bmp(seq: u32, frame_no: u32, frame: &[u8]) {
    const ROW_BYTES: usize = WIN_W * 3; // 24-bit BGR; WIN_W is a multiple of 4 → no padding
    let pixel_data = ROW_BYTES * WIN_H;
    let file_size = 54 + pixel_data;

    defmt::info!(
        "MARK {} BEGIN (frame {}, BMP {}x{} window @ {},{}; {} bytes)",
        seq,
        frame_no,
        WIN_W,
        WIN_H,
        WIN_X0,
        WIN_Y0,
        file_size
    );


    // 54-byte header: BITMAPFILEHEADER (14) + BITMAPINFOHEADER (40), 24-bit BI_RGB.
    let mut hdr = [0u8; 54];
    hdr[0] = b'B';
    hdr[1] = b'M';
    put_u32(&mut hdr, 2, file_size as u32); // file size
    put_u32(&mut hdr, 10, 54); // pixel-data offset
    put_u32(&mut hdr, 14, 40); // info-header size
    put_u32(&mut hdr, 18, WIN_W as u32); // width
    put_u32(&mut hdr, 22, WIN_H as u32); // height (positive → bottom-up rows)
    hdr[26] = 1; // planes
    hdr[28] = 24; // bits per pixel
    put_u32(&mut hdr, 34, pixel_data as u32); // image size
    put_u32(&mut hdr, 38, 2835); // x pixels/metre (~72 DPI)
    put_u32(&mut hdr, 42, 2835); // y pixels/metre
    emit_hex(&hdr);

    // Pixel rows. BMP is bottom-up, so the first row written is the window bottom.
    let mut row = [0u8; ROW_BYTES];
    for bmp_row in 0..WIN_H {
        let img_row = WIN_Y0 + (WIN_H - 1 - bmp_row);
        let line = &frame[img_row * LINE_BYTES..(img_row + 1) * LINE_BYTES];
        for col in 0..WIN_W {
            let px = WIN_X0 + col;
            // 4 bytes = 2 pixels. Y is per-pixel; Cb/Cr are shared by the pixel
            // pair starting at `pair`. Byte order is set by the YUV_*_OFF consts.
            let pair = (px / 2) * 4;
            let y = line[px * 2 + YUV_Y_OFF] as i32;
            let cb = line[pair + YUV_CB_OFF] as i32;
            let cr = line[pair + YUV_CR_OFF] as i32;
            let (r, g, b) = yuv_to_rgb(y, cb, cr);
            // BMP stores BGR.
            row[col * 3] = b;
            row[col * 3 + 1] = g;
            row[col * 3 + 2] = r;
        }
        emit_hex(&row);
    }

    // Diagnostic: 24 RAW captured bytes (unconverted) from three rows of the
    // window. Labelled, so these lines do NOT match the pure-hex BMP extraction.
    // They reveal the true byte layout and whether there is real image data.
    for r in [0usize, WIN_H / 2, WIN_H.saturating_sub(1)] {
        let off = (WIN_Y0 + r) * LINE_BYTES + WIN_X0 * 2;
        defmt::info!("RAW row {} @{}: {=[u8]:02x}", r, off, frame[off..off + 24]);
    }

    defmt::info!("MARK {} END", seq);
}

/// Convert one YCbCr (full-range, BT.601 coefficients) sample to RGB888.
/// Fixed-point (×256) integer math — no FPU dependency.
fn yuv_to_rgb(y: i32, cb: i32, cr: i32) -> (u8, u8, u8) {
    let c = cr - 128;
    let d = cb - 128;
    let r = y + ((359 * c) >> 8); // 1.402
    let g = y - ((88 * d) >> 8) - ((183 * c) >> 8); // 0.344, 0.714
    let b = y + ((454 * d) >> 8); // 1.772
    (clamp8(r), clamp8(g), clamp8(b))
}

#[inline]
fn clamp8(v: i32) -> u8 {
    v.clamp(0, 255) as u8
}

/// Write `v` little-endian into `buf` at `off`.
fn put_u32(buf: &mut [u8], off: usize, v: u32) {
    buf[off..off + 4].copy_from_slice(&v.to_le_bytes());
}

/// Emit `bytes` as contiguous lowercase-hex `defmt` lines (HEX_DUMP_ROW bytes each).
fn emit_hex(bytes: &[u8]) {
    let mut line = [0u8; HEX_DUMP_ROW * 2];
    for chunk in bytes.chunks(HEX_DUMP_ROW) {
        for (i, &b) in chunk.iter().enumerate() {
            line[i * 2] = hex_nibble(b >> 4);
            line[i * 2 + 1] = hex_nibble(b & 0x0F);
        }
        let s = core::str::from_utf8(&line[..chunk.len() * 2]).unwrap_or("");
        defmt::info!("{=str}", s);
    }
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
