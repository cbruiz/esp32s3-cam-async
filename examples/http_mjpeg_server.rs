//! Multicore HTTP MJPEG server for an OV2640 on the ESP32-S3.
//!
//! Streams JPEG frames over HTTP as `multipart/x-mixed-replace`, viewable in any
//! browser. The work is split across the two Xtensa cores:
//!
//! - **Core 0 (PRO):** WiFi Station + embassy-net stack (DHCP) + the HTTP MJPEG
//!   server.
//! - **Core 1 (APP):** OV2640 JPEG capture (LCD-CAM + DMA), feeding frames to
//!   core 0 through a lock-free, copy-free 2-buffer handoff.
//!
//! # Usage
//!
//! The WiFi credentials are baked in at **compile time** from environment
//! variables, so nothing secret lives in the source:
//!
//! ```sh
//! WIFI_SSID="MyNetwork" WIFI_PASSWORD="secret" \
//!   cargo run -p esp32s3-cam-async --example http_mjpeg_server \
//!     --features ov2640,jfif,defmt -- --log-format defmt
//! ```
//!
//! The board joins that AP as a station and obtains its IP over DHCP — watch the
//! defmt log for the assigned address, then open `http://<that-ip>/` in a
//! browser (or VLC: *Open Network Stream*).
//!
//! # Hardware wiring (ESP32-S3-CAM): see `capture_jpeg.rs`.
#![no_std]
#![no_main]

extern crate alloc;

use core::fmt::Write as _;

use embassy_futures::select::{select, Either};
use embassy_net::{tcp::TcpSocket, Config as NetConfig, Ipv4Address, Runner, StackResources};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_time::{Duration, Instant, Timer};
use embedded_io_async::Write;
use heapless::String;
use static_cell::StaticCell;

use esp_hal::{
    clock::CpuClock,
    dma::{DmaChannel, DmaRxStreamBuf},
    dma_rx_stream_buffer,
    i2c::master::{Config as I2cConfig, I2c},
    init,
    interrupt::software::SoftwareInterruptControl,
    lcd_cam::{
        cam::{Config as CamConfig, EofMode, VhdeMode, VsyncFilterThreshold},
        ByteOrder,
    },
    rng::Rng,
    system::Stack as CoreStack,
    timer::timg::TimerGroup,
    Blocking, Config as HalConfig,
};
use esp_radio::wifi::{
    sta::{ScanMethod, StationConfig},
    Config as WifiConfig, ControllerConfig, Interface, PowerSaveMode, WifiController,
};

use esp32s3_cam_async::{
    jfif::{JpegStreamParser, ParserProgressing},
    ov2640::{Ov2640, Ov2640Config, PixelFormat, Resolution},
    AsyncCameraDriver, CameraCapture, CaptureConfig, ConverterMode,
};

// Heap (esp-radio allocates) + defmt logger + panic handler.
use esp_alloc as _;
use esp_backtrace as _;
use esp_println as _;
use esp32s3_cam_async::ov2640::ClockConfig;

const CLOCK: ClockConfig = ClockConfig::MAX_FPS;

esp_bootloader_esp_idf::esp_app_desc!();

// ── Configuration ────────────────────────────────────────────────────────────
// WiFi credentials are injected at compile time (see the module docs). They fall
// back to placeholders so the example still builds (e.g. in CI) without them —
// but those placeholders won't join any real network.
const WIFI_SSID: &str = match option_env!("WIFI_SSID") {
    Some(s) => s,
    None => "changeme-ssid",
};
const WIFI_PASSWORD: &str = match option_env!("WIFI_PASSWORD") {
    Some(s) => s,
    None => "changeme-pass",
};

const CAPTURE_RESOLUTION: Resolution = Resolution::Qqvga;

const JPEG_QUALITY: u8 = 24;
/// Upper bound on one encoded VGA JPEG frame; also the per-buffer size.
const MAX_FRAME: usize = 48 * 1024;
/// DMA ring for the camera. Sized to hold a whole VGA JPEG frame so the capture
/// never stalls mid-frame re-arming descriptors — matches the esp-hal reference
/// (`20*1000`). A 4 KB ring re-armed ~5×/frame and throttled throughput hard.
const DMA_RING: usize = 20 * 1024;
const DMA_BLOCK: usize = 1024;
/// Per-iteration buffer drained from the ring (small — lives on the stack).
const DMA_CHUNK: usize = 4096;
const APP_CORE_STACK_SIZE: usize = 16 * 1024;

// ── Cross-core frame handoff ──────────────────────────────────────────────────
//
// Two heap-free buffers shuttle between the cores as owned handles:
//   camera: EMPTY.recv → fill → READY.send
//   server: READY.recv → write to socket → EMPTY.send
// The camera never blocks: if no empty buffer is free it recycles the oldest
// READY one (latest-frame-only — stale frames are dropped, not queued).
struct FrameBuf {
    data: &'static mut [u8; MAX_FRAME],
    len: usize,
}

static EMPTY: Channel<CriticalSectionRawMutex, FrameBuf, 2> = Channel::new();
static READY: Channel<CriticalSectionRawMutex, FrameBuf, 2> = Channel::new();

// Frame buffers live in internal SRAM (.bss): no large stack temporary, and
// internal RAM is uncached on the ESP32-S3 so the two cores see it coherently.
static mut BUF0: [u8; MAX_FRAME] = [0; MAX_FRAME];
static mut BUF1: [u8; MAX_FRAME] = [0; MAX_FRAME];

// Parser output scratch (core 1 only). Static, not a stack array: at VGA sizes a
// 48 KB stack buffer would overflow the secondary-core stack.
static mut JPEG_OUT: [u8; MAX_FRAME] = [0; MAX_FRAME];

static APP_CORE_STACK: StaticCell<CoreStack<APP_CORE_STACK_SIZE>> = StaticCell::new();
static EXECUTOR_0: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
static EXECUTOR_1: StaticCell<esp_rtos::embassy::Executor> = StaticCell::new();
static STACK_RESOURCES: StaticCell<StackResources<4>> = StaticCell::new();

type NetRunner = Runner<'static, Interface<'static>>;

#[embassy_executor::task]
async fn net_task(mut runner: NetRunner) -> ! {
    runner.run().await
}

/// Maintains the Station connection, reconnecting whenever the link drops.
#[embassy_executor::task]
async fn connection_task(mut controller: WifiController<'static>) -> ! {
    loop {
        defmt::info!("[CPU 0] WiFi: connecting to '{}'", WIFI_SSID);
        match controller.connect_async().await {
            Ok(_) => {
                defmt::info!("[CPU 0] WiFi: connected, waiting for DHCP lease");
                controller.wait_for_disconnect_async().await.ok();
                defmt::warn!("[CPU 0] WiFi: disconnected, reconnecting");
            }
            Err(_) => {
                defmt::error!("[CPU 0] WiFi: connect failed, retry in 2s");
                Timer::after(Duration::from_secs(2)).await;
            }
        }
    }
}

#[esp_hal::main]
fn main() -> ! {
    let p = init(HalConfig::default().with_cpu_clock(CpuClock::_240MHz));

    // esp-radio allocates from these heaps.
    esp_alloc::heap_allocator!(#[esp_hal::ram(reclaimed)] size: 64000);
    esp_alloc::heap_allocator!(size: 64000);

    defmt::info!("boot: ESP32-S3 OV2640 HTTP MJPEG server");

    let sw_int = SoftwareInterruptControl::new(p.SW_INTERRUPT);
    let timg0 = TimerGroup::new(p.TIMG0);
    esp_rtos::start(timg0.timer0, sw_int.software_interrupt0);

    // ── Camera (driven on core 1) ────────────────────────────────────────────
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
        ConverterMode::Bypass, // JPEG is an opaque byte stream → converter bypassed
        p.GPIO15, // MCLK
        p.GPIO13, // PCLK
        p.GPIO6,  // VSYNC
        p.GPIO7,  // HREF
        p.GPIO11, p.GPIO9, p.GPIO8, p.GPIO10, p.GPIO12, p.GPIO18, p.GPIO17, p.GPIO16, // D0–D7
    )
    .unwrap();

    let i2c = I2c::new(p.I2C0, I2cConfig::default())
        .unwrap()
        .with_sda(p.GPIO4)
        .with_scl(p.GPIO5);

    // Seed the frame pool with both buffers.
    EMPTY
        .try_send(FrameBuf { data: unsafe { &mut *core::ptr::addr_of_mut!(BUF0) }, len: 0 })
        .ok();
    EMPTY
        .try_send(FrameBuf { data: unsafe { &mut *core::ptr::addr_of_mut!(BUF1) }, len: 0 })
        .ok();

    // ── WiFi Station ─────────────────────────────────────────────────────────
    let cpuctrl = p.CPU_CTRL;
    let (mut controller, interfaces) = esp_radio::wifi::new(
        p.WIFI,
        ControllerConfig::default().with_initial_config(WifiConfig::Station(
            StationConfig::default()
                .with_ssid(WIFI_SSID)
                .with_password(alloc::string::String::from(WIFI_PASSWORD))
                .with_scan_method(ScanMethod::Fast),
        )),
    )
    .unwrap();
    controller.set_power_saving(PowerSaveMode::None).ok();

    // DHCP: the AP's router assigns our address (watch the log for it).
    let (stack, runner) = embassy_net::new(
        interfaces.station,
        NetConfig::dhcpv4(Default::default()),
        STACK_RESOURCES.init(StackResources::new()),
        Rng::new().random() as u64,
    );

    // ── Core 0: network + HTTP server ────────────────────────────────────────
    let executor0 = EXECUTOR_0.init(esp_rtos::embassy::Executor::new());
    executor0.run(move |spawner| {
        spawner.spawn(net_task(runner).unwrap());
        spawner.spawn(connection_task(controller).unwrap());
        spawner.spawn(http_server_task(stack).unwrap());

        // ── Core 1: camera capture ───────────────────────────────────────────
        esp_rtos::start_second_core(
            cpuctrl,
            sw_int.software_interrupt1,
            APP_CORE_STACK.init(CoreStack::new()),
            move || {
                let executor1 = EXECUTOR_1.init(esp_rtos::embassy::Executor::new());
                executor1.run(|spawner| {
                    spawner.spawn(camera_task(camera, i2c).unwrap());
                });
            },
        );
    });
}

/// Core 1: capture JPEG frames and publish them to the [`READY`] queue.
#[embassy_executor::task]
async fn camera_task(camera: AsyncCameraDriver<'static>, mut i2c: I2c<'static, Blocking>) {
    let sensor = Ov2640::new(Ov2640Config {
        clock: CLOCK,
        format: PixelFormat::Jpeg { quality: JPEG_QUALITY },
        resolution: Resolution::Vga,
        ..Default::default()
    });
    sensor.check_id(&mut i2c).expect("OV2640 not found");
    // VGA + JPEG init uses the crate's proven reference sequence internally.
    sensor.init(&mut i2c).await.expect("OV2640 init failed");
    defmt::info!("[CPU 1] OV2640 init OK ({:?} JPEG)", CAPTURE_RESOLUTION);

    let cam_buf: DmaRxStreamBuf = dma_rx_stream_buffer!(DMA_RING, DMA_BLOCK);
    let mut capture = CameraCapture::new(camera, cam_buf, CaptureConfig::default());
    capture.start().unwrap();
    defmt::info!("[CPU 1] DMA capture started");

    let mut dma_chunk = [0u8; DMA_CHUNK];
    let jpeg_out = unsafe { &mut *core::ptr::addr_of_mut!(JPEG_OUT) };
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
            match parser.parse(chunk, &mut jpeg_out[..]) {
                Ok(ParserProgressing::InputBufferEmpty) => break,
                Ok(ParserProgressing::EndOfImage) => {
                    publish_frame(parser.bytes_written(), &jpeg_out[..]);
                    parser.reset_output_offset();
                }
                Ok(ParserProgressing::OutputBufferFull) => {
                    defmt::warn!("[CPU 1] frame > {} B buffer, dropping", MAX_FRAME);
                    parser.reset_state();
                    break;
                }
                Err(_) => break,
            }
        }
    }
}

/// Copy a finished JPEG into a free buffer and hand it to the server.
/// Latest-frame-only: never blocks the camera — recycles a stale READY buffer
/// when the pool is exhausted, and drops the frame if even that fails.
fn publish_frame(len: usize, jpeg: &[u8]) {
    let mut fb = match EMPTY.try_receive().or_else(|_| READY.try_receive()) {
        Ok(fb) => fb,
        Err(_) => return,
    };
    fb.data[..len].copy_from_slice(&jpeg[..len]);
    fb.len = len;
    let _ = READY.try_send(fb);
}

/// Core 0: accept HTTP clients and stream MJPEG until the link drops.
#[embassy_executor::task]
async fn http_server_task(stack: embassy_net::Stack<'static>) -> ! {
    let mut rx_buffer = [0u8; 1024];
    let mut tx_buffer = [0u8; 4 * 1024];

    loop {
        wait_for_ip(stack).await;
        defmt::info!("[CPU 0] HTTP MJPEG server up on http://{}/", ip(stack));

        let server = run_http_server(stack, &mut rx_buffer, &mut tx_buffer);
        match select(server, wait_for_link_loss(stack)).await {
            Either::First(_) => defmt::error!("[CPU 0] server exited"),
            Either::Second(_) => defmt::warn!("[CPU 0] link lost, restarting server"),
        }
    }
}

async fn run_http_server(stack: embassy_net::Stack<'static>, rx: &mut [u8], tx: &mut [u8]) {
    let mut socket = TcpSocket::new(stack, rx, tx);
    socket.set_timeout(Some(Duration::from_secs(30)));

    loop {
        defmt::info!("[CPU 0] listening on :80");
        if let Err(e) = socket.accept(80).await {
            defmt::warn!("[CPU 0] accept error: {:?}", e);
            continue;
        }
        defmt::info!("[CPU 0] client connected");
        if let Err(e) = handle_connection(&mut socket).await {
            defmt::warn!("[CPU 0] connection closed: {:?}", e);
        }
        socket.close();
        socket.abort();
    }
}

async fn handle_connection(socket: &mut TcpSocket<'_>) -> Result<(), embassy_net::tcp::Error> {
    let mut buf = [0u8; 512];
    let n = match socket.read(&mut buf).await {
        Ok(0) => {
            defmt::info!("[CPU 0] client sent 0 bytes (closed)");
            return Ok(());
        }
        Ok(n) => n,
        Err(e) => {
            defmt::warn!("[CPU 0] read failed: {:?}", e);
            return Err(e);
        }
    };
    defmt::info!("[CPU 0] request: {} bytes", n);
    if !buf[..n].starts_with(b"GET") {
        socket.write_all(b"HTTP/1.1 400 Bad Request\r\n\r\n").await?;
        return Ok(());
    }

    const BOUNDARY: &str = "frameboundary";
    let mut headers: String<256> = String::new();
    let _ = write!(
        headers,
        "HTTP/1.1 200 OK\r\n\
         Content-Type: multipart/x-mixed-replace; boundary={}\r\n\
         Connection: close\r\n\
         Cache-Control: no-cache\r\n\
         Access-Control-Allow-Origin: *\r\n\r\n",
        BOUNDARY
    );
    socket.write_all(headers.as_bytes()).await?;
    defmt::info!("[CPU 0] headers sent, streaming");

    let mut last_metrics = Instant::now();
    let mut frames = 0u32;
    let mut bytes = 0u64;

    loop {
        let frame = READY.receive().await;
        let len = frame.len;
        let result = stream_frame(socket, &frame.data[..frame.len], BOUNDARY).await;
        let _ = EMPTY.try_send(frame); // return the buffer to the pool
        result?;

        frames += 1;
        bytes += len as u64;

        let now = Instant::now();
        if now.duration_since(last_metrics) >= Duration::from_secs(10) {
            let secs = now.duration_since(last_metrics).as_secs().max(1);
            defmt::info!(
                "[CPU 0] MJPEG: {} fps, {} kbps",
                frames / secs as u32,
                (bytes * 8) / (secs * 1024)
            );
            last_metrics = now;
            frames = 0;
            bytes = 0;
        }
    }
}

/// Write one `multipart/x-mixed-replace` JPEG part.
async fn stream_frame(
    socket: &mut TcpSocket<'_>,
    jpeg: &[u8],
    boundary: &str,
) -> Result<(), embassy_net::tcp::Error> {
    // Skip anything that is not a self-contained JPEG (SOI … EOI).
    if jpeg.len() < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8 {
        return Ok(());
    }
    let mut part: String<128> = String::new();
    let _ = write!(
        part,
        "--{}\r\nContent-Type: image/jpeg\r\nContent-Length: {}\r\n\r\n",
        boundary,
        jpeg.len()
    );
    socket.write_all(part.as_bytes()).await?;
    socket.write_all(jpeg).await?;
    socket.write_all(b"\r\n").await
}

async fn wait_for_ip(stack: embassy_net::Stack<'static>) {
    loop {
        if stack.is_link_up() && stack.config_v4().is_some() {
            return;
        }
        Timer::after(Duration::from_millis(500)).await;
    }
}

async fn wait_for_link_loss(stack: embassy_net::Stack<'static>) {
    loop {
        if !stack.is_link_up() {
            return;
        }
        Timer::after(Duration::from_secs(1)).await;
    }
}

fn ip(stack: embassy_net::Stack<'static>) -> Ipv4Address {
    stack
        .config_v4()
        .map(|c| c.address.address())
        .unwrap_or(Ipv4Address::new(0, 0, 0, 0))
}
