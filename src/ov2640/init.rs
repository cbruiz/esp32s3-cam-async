use embassy_time::{Duration, Timer};
use esp_hal::i2c::master::I2c;
use esp_hal::Blocking;

use super::clock::ClockConfig;
use super::config::{Ov2640Config, PixelFormat, Resolution};
use super::registers::*;
use super::tables::{
    OV2640_BASE, OV2640_FORMAT_JPEG, OV2640_SETTINGS_CIF, OV2640_SETTINGS_TO_SVGA,
    OV2640_WINDOW_SVGA, OV2640_WINDOW_UXGA, OV2640_WINDOW_VGA,
};

/// OV2640 sensor driver instance.
///
/// Holds an [`Ov2640Config`] and exposes `check_id`, `init`, and direct
/// register-write methods. I2C is passed per-call so the bus can be shared.
///
/// ```rust,ignore
/// let sensor = Ov2640::new(Ov2640Config {
///     format: PixelFormat::Jpeg { quality: 12 },
///     ..Default::default()
/// });
/// sensor.check_id(&mut i2c).expect("OV2640 not found");
/// sensor.init(&mut i2c).await.expect("OV2640 init failed");
///
/// // Direct register access when needed
/// sensor.write_reg(&mut i2c, MY_REG, MY_VAL)?;
/// sensor.write_regs(&mut i2c, &CUSTOM_TABLE)?;
/// ```
pub struct Ov2640 {
    config: Ov2640Config,
}

impl Ov2640 {
    pub fn new(config: Ov2640Config) -> Self {
        Self { config }
    }

    pub fn config(&self) -> &Ov2640Config {
        &self.config
    }

    /// Verify the OV2640 product ID over I2C.
    ///
    /// Reads PID_MSB (0x0A) and PID_LSB (0x0B) from the sensor bank and
    /// checks for expected values 0x26 / 0x42. Call before [`init`](Self::init).
    pub fn check_id(
        &self,
        i2c: &mut I2c<'_, Blocking>,
    ) -> Result<(), esp_hal::i2c::master::Error> {
        let addr = self.config.i2c_addr;

        use embedded_hal::delay::DelayNs;
        let mut d = embassy_time::Delay;

        i2c.write(addr, &[0xFF, 0x01])?; // select sensor bank
        d.delay_ms(2);

        let mut pid_msb = [0u8; 1];
        let mut pid_lsb = [0u8; 1];
        i2c.write(addr, &[0x0A])?;
        i2c.read(addr, &mut pid_msb)?;
        i2c.write(addr, &[0x0B])?;
        i2c.read(addr, &mut pid_lsb)?;

        i2c.write(addr, &[0xFF, 0x00])?; // restore DSP bank

        if pid_msb[0] == 0x26 && pid_lsb[0] == 0x42 {
            Ok(())
        } else {
            Err(esp_hal::i2c::master::Error::AcknowledgeCheckFailed(
                esp_hal::i2c::master::AcknowledgeCheckFailedReason::Unknown,
            ))
        }
    }

    /// Initialise the sensor from the stored configuration.
    ///
    /// Sequence:
    /// 1. Sensor soft-reset (50 ms).
    /// 2. CIF base table — AGC/AEC, gamma curve, colour matrix, DSP pipeline.
    /// 3. Sensor window + DSP scaling for the configured [`Resolution`].
    /// 4. DVP reset cycle — clears `IMAGE_MODE`; write format regs only after.
    /// 5. Output format (`CTRL0` + `IMAGE_MODE` ± `QS`).
    pub async fn init(
        &self,
        i2c: &mut I2c<'_, Blocking>,
    ) -> Result<(), esp_hal::i2c::master::Error> {
        let addr = self.config.i2c_addr;

        write_reg(i2c, addr, BANK_SEL, BANK_SENSOR)?;
        write_reg(i2c, addr, COM7, COM7_SRST)?;
        Timer::after(Duration::from_millis(50)).await;

        // JPEG at VGA / SVGA uses the proven composable preset (BASE + window +
        // FORMAT_JPEG): VGA reproduces the esp-hal reference byte-for-byte, and
        // SVGA reuses it with a 1:1 scaler. Everything else (QVGA/QQVGA downscale,
        // RGB565, YUV422) uses the legacy structured path below — the QVGA CIF
        // preset was dropped because it produced an invalid JPEG stream on this
        // driver and needs on-hardware register iteration.
        let jpeg_window = match self.config.resolution {
            Resolution::Vga => Some(OV2640_WINDOW_VGA),
            Resolution::Svga => Some(OV2640_WINDOW_SVGA),
            Resolution::Uxga => Some(OV2640_WINDOW_UXGA),
            _ => None,
        };
        if let (Some(window), PixelFormat::Jpeg { quality }) =
            (jpeg_window, self.config.format)
        {
            write_regs(i2c, addr, OV2640_BASE)?;
            write_regs(i2c, addr, window)?;
            write_regs(i2c, addr, OV2640_FORMAT_JPEG)?;
            // The blocks end in the DSP bank; set JPEG quality (QS) there.
            write_reg(i2c, addr, QS, quality)?;
            // The preset tables already wrote CLKRC (0x11) and R_DVP_SP (0xD3);
            // re-apply the configured clock so an explicit divider override (or a
            // non-zero CLKRC pre-scale) takes effect. With the default clock this
            // rewrites the same values, leaving the proven timing untouched.
            apply_clock(i2c, addr, self.config.clock, None)?;
            Timer::after(Duration::from_millis(10)).await;
            #[cfg(feature = "defmt")]
            defmt::info!(
                "OV2640 configured (preset): {:?} Jpeg, QS={}",
                self.config.resolution,
                quality
            );
            return Ok(());
        }

        write_regs(i2c, addr, OV2640_SETTINGS_CIF)?;
        set_resolution(
            i2c,
            addr,
            self.config.resolution,
            self.config.format,
            self.config.clock,
        )
        .await?;

        // DVP reset clears IMAGE_MODE; write format registers only after release.
        write_reg(i2c, addr, BANK_SEL, BANK_DSP)?;
        write_reg(i2c, addr, RESET, RESET_DVP)?;
        Timer::after(Duration::from_millis(5)).await;
        write_reg(i2c, addr, RESET, 0x00)?;

        apply_format(i2c, addr, self.config.format)?;

        #[cfg(feature = "defmt")]
        defmt::info!("OV2640 configured: {:?} {:?}", self.config.format, self.config.resolution);

        Ok(())
    }

    /// Write a single register to the sensor via I2C.
    ///
    /// Uses the address from the stored config. For writing to a different
    /// I2C address use the free-function [`write_reg`].
    pub fn write_reg(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        reg: u8,
        val: u8,
    ) -> Result<(), esp_hal::i2c::master::Error> {
        write_reg(i2c, self.config.i2c_addr, reg, val)
    }

    /// Write a batch of `(register, value)` pairs to the sensor via I2C.
    ///
    /// Uses the address from the stored config. For writing to a different
    /// I2C address use the free-function [`write_regs`].
    pub fn write_regs(
        &self,
        i2c: &mut I2c<'_, Blocking>,
        regs: &[(u8, u8)],
    ) -> Result<(), esp_hal::i2c::master::Error> {
        write_regs(i2c, self.config.i2c_addr, regs)
    }
}

impl Default for Ov2640 {
    fn default() -> Self {
        Self::new(Ov2640Config::default())
    }
}

// ── Free-function escape hatches (arbitrary I2C address) ─────────────────────

/// Write a single OV2640 register via I2C (blocking SCCB).
///
/// Use this for writing to an address other than the one in [`Ov2640Config`],
/// or for ad-hoc access without an [`Ov2640`] instance.
pub fn write_reg(
    i2c: &mut I2c<'_, Blocking>,
    addr: u8,
    reg: u8,
    val: u8,
) -> Result<(), esp_hal::i2c::master::Error> {
    i2c.write(addr, &[reg, val])
}

/// Write a batch of `(register, value)` pairs over I2C.
///
/// A `(BANK_SEL, BANK_*)` entry anywhere in `regs` switches the active
/// register bank for all subsequent entries in that batch.
pub fn write_regs(
    i2c: &mut I2c<'_, Blocking>,
    addr: u8,
    regs: &[(u8, u8)],
) -> Result<(), esp_hal::i2c::master::Error> {
    for &(reg, val) in regs {
        write_reg(i2c, addr, reg, val)?;
    }
    Ok(())
}

// ── Private helpers ───────────────────────────────────────────────────────────

/// Program the [`ClockConfig`] sensor dividers: CLKRC (`0x11`, sensor bank) and
/// R_DVP_SP (`0xD3`, DSP bank).
///
/// CLKRC is always written (its default of `0` matches the power-on / preset
/// value, so this is a no-op there). R_DVP_SP is written to
/// [`ClockConfig::dvp_pclk`] when set, otherwise to `default_dvp`; passing
/// `default_dvp = None` leaves whatever value the preceding sequence wrote
/// untouched (used on the JPEG preset path, where the tables set it per
/// resolution).
fn apply_clock(
    i2c: &mut I2c<'_, Blocking>,
    addr: u8,
    clock: ClockConfig,
    default_dvp: Option<u8>,
) -> Result<(), esp_hal::i2c::master::Error> {
    write_reg(i2c, addr, BANK_SEL, BANK_SENSOR)?;
    write_reg(i2c, addr, CLKRC, clock.clkrc)?;

    if let Some(dvp) = clock.dvp_pclk.or(default_dvp) {
        write_reg(i2c, addr, BANK_SEL, BANK_DSP)?;
        write_reg(i2c, addr, R_DVP_SP, dvp)?;
    }
    Ok(())
}

async fn set_resolution(
    i2c: &mut I2c<'_, Blocking>,
    addr: u8,
    resolution: Resolution,
    format: PixelFormat,
    clock: ClockConfig,
) -> Result<(), esp_hal::i2c::master::Error> {
    let (out_w, out_h) = resolution.dims();

    match resolution {
        // QQVGA / QVGA: SVGA (800×600) sensor base + hardware DCW (÷2) + zoom.
        Resolution::Qqvga | Resolution::Qvga => {
            set_window(i2c, addr, 0, 0, 800, 600, out_w, out_h, true, format, clock).await
        }
        // VGA: SVGA (800×600) base → 640×480 = clean 1.25× zoom downscale (the
        // proven path). DCW module stays enabled (CTRL2=0x3D) but at ÷1, exactly
        // like the esp-hal reference; the scaler does the 1.25×.
        Resolution::Vga => {
            set_window(i2c, addr, 0, 0, 800, 600, out_w, out_h, true, format, clock).await
        }
        // SVGA: native 800×600; no DCW or zoom downscale.
        Resolution::Svga => {
            set_window(i2c, addr, 0, 0, 800, 600, out_w, out_h, false, format, clock).await
        }
        // UXGA only reaches the legacy path for raw formats (impractical at
        // 1600×1200); JPEG uses the dedicated preset. Treat it as native SVGA
        // here as a non-crashing fallback.
        Resolution::Uxga => {
            set_window(i2c, addr, 0, 0, 800, 600, 800, 600, false, format, clock).await
        }
    }
}

fn apply_format(
    i2c: &mut I2c<'_, Blocking>,
    addr: u8,
    format: PixelFormat,
) -> Result<(), esp_hal::i2c::master::Error> {
    match format {
        PixelFormat::Rgb565 => {
            // AEC_EN + STAT_SEL restore the auto-exposure / AWB statistics that
            // the power-on default carried (esp32-camera never overwrites CTRL0);
            // dropping STAT_SEL starved the AWB and left a green cast. YUV_EN +
            // YUV422_EN keep the data-producing path that was already working.
            write_reg(
                i2c,
                addr,
                CTRL0,
                CTRL0_AEC_EN | CTRL0_STAT_SEL | CTRL0_YUV_EN | CTRL0_YUV422_EN,
            )?;
            write_reg(i2c, addr, IMAGE_MODE, IMAGE_MODE_RGB565)?;
        }
        PixelFormat::Yuv422 => {
            // YUV_EN (0x04) enables the DSP YUV output path — without it the DSP
            // emits a fixed pattern (0x55/0xAA) instead of pixels. Same enable as
            // the working RGB565 path; 0xD7=0x01 is esp32-camera's yuv422 value
            // (the base table leaves it at 0x03, which is for RGB565/JPEG).
            write_reg(
                i2c,
                addr,
                CTRL0,
                CTRL0_AEC_EN | CTRL0_STAT_SEL | CTRL0_YUV_EN | CTRL0_YUV422_EN,
            )?;
            write_reg(i2c, addr, IMAGE_MODE, IMAGE_MODE_YUV422)?;
            write_reg(i2c, addr, 0xD7, 0x01)?;
            write_reg(i2c, addr, 0xE1, 0x67)?;
        }
        PixelFormat::Jpeg { quality } => {
            // CTRL0 gates the DSP output (required for data). AEC_EN + STAT_SEL
            // restore the auto-exposure / AWB statistics the default carried;
            // without STAT_SEL the AWB had no data and the frame came out green.
            write_reg(i2c, addr, CTRL0, CTRL0_AEC_EN | CTRL0_STAT_SEL | CTRL0_YUV422_EN)?;

            // Reset-bracketed JPEG configuration, matching esp32-camera's
            // `ov2640_settings_jpeg3`. IMAGE_MODE and the JPEG/DSP tuning
            // registers must be written while JPEG+DVP are held in reset, then
            // released. Without the tuning writes (0xE5/0xD9/0xDF/0x33/0x3C/
            // 0xEB/0xDD, plus 0xE1=0x77) the JPEG encoder stays stalled, the DVP
            // never clocks out, and the LCD-CAM DMA times out waiting for VSYNC.
            write_reg(i2c, addr, RESET, RESET_JPEG | RESET_DVP)?;
            write_reg(i2c, addr, IMAGE_MODE, IMAGE_MODE_JPEG_EN | IMAGE_MODE_HREF_VSYNC)?;
            write_reg(i2c, addr, 0xD7, 0x03)?;
            write_reg(i2c, addr, 0xE1, 0x77)?;
            write_reg(i2c, addr, 0xE5, 0x1F)?;
            write_reg(i2c, addr, 0xD9, 0x10)?;
            write_reg(i2c, addr, 0xDF, 0x80)?;
            write_reg(i2c, addr, 0x33, 0x80)?;
            write_reg(i2c, addr, 0x3C, 0x10)?;
            write_reg(i2c, addr, 0xEB, 0x30)?;
            write_reg(i2c, addr, 0xDD, 0x7F)?;
            write_reg(i2c, addr, QS, quality)?;
            write_reg(i2c, addr, RESET, 0x00)?;
        }
    }
    Ok(())
}

/// Configure the DSP sensor window and output scaling registers.
///
/// OV2640 DSP pipeline:
/// `sensor_window → [DCW ÷2 per axis, if use_dcw] → zoom_scaler → output`
///
/// All sizes are in real pixels; the function converts to the `/4` register domain.
///
/// - `sensor_w / sensor_h` — DSP input (800×600 for the SVGA base).
/// - `out_w / out_h` — desired output dimensions.
/// - `use_dcw` — enable hardware DCW. Set to `true` when `out ≤ sensor / 2`.
async fn set_window(
    i2c: &mut I2c<'_, Blocking>,
    addr: u8,
    offset_x: u16,
    offset_y: u16,
    sensor_w: u16,
    sensor_h: u16,
    out_w: u16,
    out_h: u16,
    use_dcw: bool,
    format: PixelFormat,
    clock: ClockConfig,
) -> Result<(), esp_hal::i2c::master::Error> {
    let max_x = sensor_w / 4;
    let max_y = sensor_h / 4;
    let w = out_w / 4;
    let h = out_h / 4;

    let vhyx = ((max_y >> 1) & 0x80) as u8
        | ((offset_y >> 4) & 0x70) as u8
        | ((max_x >> 5) & 0x08) as u8
        | ((offset_x >> 8) & 0x07) as u8;

    let test  = ((max_x >> 2) & 0x80) as u8;

    let sizel = (((max_x >> 6) & 0x40)
        | ((max_x >> 11) & 0x01)
        | ((h >> 6) & 0x04)
        | ((w >> 8) & 0x03)) as u8;

    let zmhh  = (((h >> 6) & 0x04) | ((w >> 8) & 0x03)) as u8;

    write_reg(i2c, addr, BANK_SEL, BANK_DSP)?;
    write_reg(i2c, addr, R_BYPASS, R_BYPASS_DSP_BYPAS)?;
    // OV2640_SETTINGS_TO_SVGA always enables DCW (CTRL2 = 0x3D).
    write_regs(i2c, addr, OV2640_SETTINGS_TO_SVGA)?;

    // Override CTRL2 when DCW is not needed (0x1D = 0x3D with bit 5 cleared).
    if !use_dcw {
        write_reg(i2c, addr, CTRL2, 0x1D)?;
    }

    // Sensor-window and zoom-output registers (/4 domain).
    // HSIZE8/VSIZE8 = sensor_dim/8; in /4 domain: max_{x,y} >> 1.
    write_regs(i2c, addr, &[
        (BANK_SEL, BANK_DSP),
        (HSIZE8, (max_x >> 1) as u8),
        (VSIZE8, (max_y >> 1) as u8),
        (SIZEL,  sizel),
        (HSIZE,  (max_x & 0xFF) as u8),
        (VSIZE,  (max_y & 0xFF) as u8),
        (XOFFL,  (offset_x & 0xFF) as u8),
        (YOFFL,  (offset_y & 0xFF) as u8),
        (VHYX,   vhyx),
        (TEST,   test),
        (ZMOW,   (w & 0xFF) as u8),
        (ZMOH,   (h & 0xFF) as u8),
        (ZMHH,   zmhh),
    ])?;

    // CLKRC (0x11) pre-scalar + R_DVP_SP (0xD3) PCLK divider set the capture
    // rate. The per-format DVP default is the validated baseline: JPEG needs a
    // fixed divider (esp32-camera's value, auto off) for stable encoder timing;
    // the raw formats keep the auto + ÷2 mode they were validated with. A
    // `ClockConfig` override (e.g. ClockConfig::HIGH_FPS) replaces these.
    let default_dvp = match format {
        PixelFormat::Jpeg { .. } => 0x10, // fixed ÷16, auto off — esp-hal reference value
        _ => 0x82,                        // auto + ÷2 (RGB565/YUV422)
    };
    apply_clock(i2c, addr, clock, Some(default_dvp))?;
    write_reg(i2c, addr, RESET, 0x00)?;
    write_reg(i2c, addr, R_BYPASS, R_BYPASS_DSP_EN)?;

    Timer::after(Duration::from_millis(10)).await;
    Ok(())
}
