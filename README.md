# esp32s3-cam-async
[![](https://img.shields.io/crates/v/esp32s3-cam-async.svg)](https://crates.io/crates/esp32s3-cam-async)
[![](https://img.shields.io/badge/license-MIT-blue.svg)](https://opensource.org/licenses/MIT)

Async LCD-CAM DMA camera driver for the **ESP32-S3**, with a built-in capture
state machine.

## Supported hardware

- Espressif ESP32-S3 (only)
- ov2640 sensor (optional feature)

Tested with:
- Freenove ESP32-S3 WROOM-1
- ESP32-S3-CAM MB

## What it provides

### `AsyncCameraDriver`

A low-level async wrapper around the ESP32-S3 LCD-CAM peripheral. Handles:

- Clock/reset sequencing for LCD_CAM and GDMA
- GPIO signal routing (MCLK, PCLK, VSYNC, HREF, 8 data lines)
- DMA descriptor setup and interrupt binding
- Runtime-selectable input conversion via [`ConverterMode`]:
  `Bypass` for RGB565/JPEG, `Yuv422Passthrough` for YUV422 (YUYV)

```rust
let camera = AsyncCameraDriver::new(
    peripherals.LCD_CAM,
    peripherals.DMA_CH0.split().0,
    cam_config,
    ConverterMode::Bypass, // or Yuv422Passthrough for PixelFormat::Yuv422
    mclk_pin, pclk_pin, vsync_pin, href_pin,
    d0, d1, d2, d3, d4, d5, d6, d7,
)?;
```

`ConverterMode` must match the sensor's output format — see
[`docs/OV2640.md`](docs/OV2640.md) for the full format / byte-layout reference.

### `CameraCapture`

High-level async capture loop that owns the DMA state machine:

| Condition                                               | Action                                                                  |
|---------------------------------------------------------|-------------------------------------------------------------------------|
| `DescriptorEmpty` (ring full mid-frame)                 | `restart_dma_no_reset()` — continues current frame without cam_reset    |
| Timeout (no DMA interrupt within `dma_wait_timeout_ms`) | Full `receive()` restart; sets `resyncing` until next VSYNC EOF         |
| DMA error                                               | Stop + `receive()` restart; returns empty chunk with `resyncing = true` |
| VSYNC EOF                                               | Clears `resyncing` flag                                                 |

```rust
let config = CaptureConfig {
    dma_wait_timeout_ms: 300,
    dma_recovery_delay_ms: 1,
    ..CaptureConfig::default()
};

let mut capture = CameraCapture::new(camera, cam_buf, config);
capture.start().unwrap();

loop {
    let result = capture.next_chunk(&mut buf).await;

    if result.resyncing {
        // Data position unknown — discard until next frame
        continue;
    }

    // Process result.len bytes from buf
    // result.eof == true → frame boundary
    // result.event — CaptureEvent::Timeout / DmaError / None for metrics
}
```

## Feature flags

| Feature     | Description                                                                                                  |
|-------------|--------------------------------------------------------------------------------------------------------------|
| `defmt`     | Enables `defmt` logging throughout the driver                                                                |
| `ov2640`    | Enables the OV2640 sensor driver (SCCB init, register/table map). See see [`docs/OV2640.md`](docs/OV2640.md) |
| `jfif`      | Enables the streaming JPEG frame extractor (`JpegStreamParser`)                                              |
| `psram_dma` | Enables PSRAM-backed DMA buffer support                                                                      |


## Examples

Runnable examples target the **ESP32-S3-CAM** (Freenove-style OV2640 board):

| Example             | Output                                                                   | Required features   |
|---------------------|--------------------------------------------------------------------------|---------------------|
| `capture_rgb565`    | Raw RGB565 scan lines via DMA (BMP dump)                                 | `ov2640,defmt`      |
| `capture_yuv422`    | Raw YUV422 (YUYV) scan lines via DMA (BMP dump)                          | `ov2640,defmt`      |
| `capture_jpeg`      | OV2640 JPEG mode → `JpegStreamParser` extracts SOI…EOI frames (hex dump) | `ov2640,jfif,defmt` |
| `http_mjpeg_server` | OV2640 JPEG → `JpegStreamParser` → MJPEG over HTTP/WiFi                  | `ov2640,jfif,defmt` |

### Wiring (ESP32-S3-CAM / Freenove pinout)

| Signal      | GPIO |    | Signal     | GPIO                         |
|-------------|------|----|------------|------------------------------|
| MCLK (XCLK) | 15   |    | D0–D7      | 11, 9, 8, 10, 12, 18, 17, 16 |
| PCLK        | 13   |    | SDA (SIOD) | 4                            |
| VSYNC       | 6    |    | SCL (SIOC) | 5                            |
| HREF        | 7    |    |            |                              |

### Running with defmt logs

The examples log with `defmt` over **UART0** (the ESP32-S3-CAM is reached through
a USB-UART bridge, e.g. `/dev/cu.usbserial-*`, not the native USB Serial/JTAG).
**The serial monitor must decode defmt**, otherwise you only see the ESP-IDF
bootloader text. espflash has no config key for this, so pass `--log-format defmt`
on the command line:

```sh
cargo run -p esp32s3-cam-async --example capture_jpeg \
  --features ov2640,jfif,defmt -- --log-format defmt
```

Cargo forwards everything after `--` to the runner, yielding
`espflash flash --monitor <elf> --log-format defmt`. In an IDE run configuration,
append `-- --log-format defmt` to the cargo command the same way. `DEFMT_LOG`
controls verbosity (e.g. `DEFMT_LOG=debug`).

## Toolchain requirement

This crate targets `xtensa-esp32s3-none-elf` and requires
[Espressif's Xtensa Rust fork](https://github.com/esp-rs/rust):

```toml
# rust-toolchain.toml
[toolchain]
channel = "esp"
```

It cannot be built with the standard stable/nightly Rust toolchain.

## Hardware

Tested with:

- **MCU**: ESP32-S3-WROOM-1-N8R8
- **Sensor**: OV2640 connected via 8-bit DVP (HREF/VSYNC/PCLK)
- **Interface**: LCD-CAM peripheral + GDMA

## License

Licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](LICENSE-APACHE))
- MIT license ([LICENSE-MIT](LICENSE-MIT))

at your option.
