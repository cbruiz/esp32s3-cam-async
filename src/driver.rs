//! Async LCD-CAM camera driver for ESP32-S3.
//!
//! The LCD module is designed to receive parallel video data signals.
//! Its bus supports 8-bit RGB, I8080, and MOTO6800 interfaces at up to 40 MHz,
//! with optional conversion among RGB565, YUV422, YUV420, and YUV411.
use core::future::poll_fn;
use core::mem::ManuallyDrop;
use core::ops::{Deref, DerefMut};
use core::sync::atomic::compiler_fence;
use core::task::{Context, Poll};

use esp_hal::dma::{
    DmaError, DmaPeripheral, DmaRxBuffer, DmaRxInterrupt, InterruptAccess, RegisterAccess,
    RxRegisterAccess,
};
use esp_hal::gpio::interconnect::{PeripheralInput, PeripheralOutput};
use esp_hal::gpio::{InputConfig, InputSignal, OutputConfig, OutputSignal};
use esp_hal::lcd_cam::cam::{ConfigError, EofMode, VhdeMode};
use esp_hal::lcd_cam::{BitOrder, ByteOrder, ClockError};
use esp_hal::peripherals::LCD_CAM;

/// LCD-CAM input data-conversion mode.
///
/// The ESP32-S3 LCD-CAM peripheral has a colour-conversion unit sitting between
/// the parallel DVP byte stream and the GDMA. How it must be programmed depends
/// entirely on the pixel format the sensor is emitting, so the caller selects the
/// mode that matches the sensor configuration (e.g. the OV2640 `PixelFormat`,
/// when the `ov2640` feature is enabled).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ConverterMode {
    /// Conversion unit **bypassed**: the async FIFO pairs consecutive 8-bit DVP
    /// bytes into 16-bit DMA words directly, with no colour processing.
    ///
    /// Correct for any already-packed byte stream:
    /// - **RGB565** (`[HB, LB]` per pixel), and
    /// - **JPEG** (an opaque byte stream the DMA must not touch).
    ///
    /// This is the default.
    #[default]
    Bypass,
    /// YUV422 **passthrough**: the conversion unit is active in 8-bit input mode
    /// and emits the YUV422 stream unchanged (no subsampling, no colour-space
    /// conversion).
    ///
    /// Required for **YUV422** (YUYV `[Y0, Cb, Y1, Cr]`) sensor output. Without
    /// it the converter stays in the RGB565 byte-pairing path and mangles the
    /// YUV data into a fixed pattern.
    Yuv422Passthrough,
}

#[allow(unused)]
pub struct AsyncCameraDriver<'a> {
    lcd_cam: LCD_CAM<'a>,
    dma: esp_hal::dma::AnyGdmaRxChannel<'a>,
    /// Address of the first DMA descriptor, set once in `receive()`.
    /// Used to re-arm the DMA after a `DescriptorEmpty` stop (ring full):
    /// `restart()` reads `GDMA_IN_DSCR_CH0` which is null after the ring
    /// exhausts, so we must use `set_link_addr + start()` instead.
    initial_dma_addr: u32,
}

#[allow(unused)]
impl<'a> AsyncCameraDriver<'a> {
    pub fn new(
        lcd_cam: LCD_CAM<'a>,
        dma: esp_hal::dma::AnyGdmaRxChannel<'a>,
        config: esp_hal::lcd_cam::cam::Config,
        converter: ConverterMode,
        m_clk: impl PeripheralOutput<'a>,
        p_clk: impl PeripheralInput<'a>,
        v_sync_pin: impl PeripheralInput<'a>,
        h_ref_pin: impl PeripheralInput<'a>,
        data_0_pin: impl PeripheralInput<'a>,
        data_1_pin: impl PeripheralInput<'a>,
        data_2_pin: impl PeripheralInput<'a>,
        data_3_pin: impl PeripheralInput<'a>,
        data_4_pin: impl PeripheralInput<'a>,
        data_5_pin: impl PeripheralInput<'a>,
        data_6_pin: impl PeripheralInput<'a>,
        data_7_pin: impl PeripheralInput<'a>,
    ) -> Result<Self, ConfigError> {
        // Enable LCD_CAM clock — equivalent to PeripheralClockControl::enable(LcdCam).
        esp_hal::peripherals::SYSTEM::regs()
            .perip_clk_en1()
            .modify(|_, w| w.lcd_cam_clk_en().bit(true));

        // Pulse reset: assert then deassert — equivalent to reset_racey(LcdCam).
        // LcdCam::new() clears it via GenericPeripheralGuard, but
        // AsyncCameraDriver bypasses LcdCam::new(), so we must do it manually.
        // Without this, all cam_ctrl writes are silently discarded and CAM_CLK
        // never toggles.
        esp_hal::peripherals::SYSTEM::regs()
            .perip_rst_en1()
            .modify(|_, w| w.lcd_cam_rst().bit(true));
        esp_hal::peripherals::SYSTEM::regs()
            .perip_rst_en1()
            .modify(|_, w| w.lcd_cam_rst().bit(false));

        esp_hal::peripherals::SYSTEM::regs()
            .perip_clk_en1()
            .modify(|_, w| w.dma_clk_en().bit(true));

        esp_hal::peripherals::SYSTEM::regs()
            .perip_rst_en1()
            .modify(|_, w| w.dma_rst().bit(true));
        esp_hal::peripherals::SYSTEM::regs()
            .perip_rst_en1()
            .modify(|_, w| w.dma_rst().bit(false));

        let m_clk = m_clk.into();
        m_clk.apply_output_config(&OutputConfig::default());
        m_clk.set_output_enable(true);
        OutputSignal::CAM_CLK.connect_to(&m_clk);

        let p_clk = p_clk.into();
        p_clk.apply_input_config(&InputConfig::default());
        p_clk.set_input_enable(true);
        InputSignal::CAM_PCLK.connect_to(&p_clk);

        let v_sync_pin = v_sync_pin.into();
        // TODO(glitch-filter): InputConfig in this esp-hal version only exposes pull config.
        // For long cables, enabling the ESP32-S3 glitch filter on VSYNC and PCLK via a direct
        // write to GPIO_GLITCHFILTERn_REG (see ESP32-S3 TRM §5.4.3) would be beneficial.
        // The LCD_CAM hardware VSYNC filter (vsync_filter_threshold) already partially mitigates this.
        v_sync_pin.apply_input_config(&InputConfig::default());
        v_sync_pin.set_input_enable(true);
        InputSignal::CAM_V_SYNC.connect_to(&v_sync_pin);

        let h_ref_pin = h_ref_pin.into();
        h_ref_pin.apply_input_config(&InputConfig::default());
        h_ref_pin.set_input_enable(true);
        // OV2640 HREF (data-enable, active-HIGH) must route to CAM_H_ENABLE.
        // In De mode (cam_vh_de_mode_en=0) the LCD-CAM gates PCLK captures on
        // H_ENABLE HIGH.  CAM_H_SYNC is unused and left unconnected.
        InputSignal::CAM_H_ENABLE.connect_to(&h_ref_pin);

        let data_0_pin = data_0_pin.into();
        data_0_pin.apply_input_config(&InputConfig::default());
        data_0_pin.set_input_enable(true);
        InputSignal::CAM_DATA_0.connect_to(&data_0_pin);

        let data_1_pin = data_1_pin.into();
        data_1_pin.apply_input_config(&InputConfig::default());
        data_1_pin.set_input_enable(true);
        InputSignal::CAM_DATA_1.connect_to(&data_1_pin);

        let data_2_pin = data_2_pin.into();
        data_2_pin.apply_input_config(&InputConfig::default());
        data_2_pin.set_input_enable(true);
        InputSignal::CAM_DATA_2.connect_to(&data_2_pin);

        let data_3_pin = data_3_pin.into();
        data_3_pin.apply_input_config(&InputConfig::default());
        data_3_pin.set_input_enable(true);
        InputSignal::CAM_DATA_3.connect_to(&data_3_pin);

        let data_4_pin = data_4_pin.into();
        data_4_pin.apply_input_config(&InputConfig::default());
        data_4_pin.set_input_enable(true);
        InputSignal::CAM_DATA_4.connect_to(&data_4_pin);

        let data_5_pin = data_5_pin.into();
        data_5_pin.apply_input_config(&InputConfig::default());
        data_5_pin.set_input_enable(true);
        InputSignal::CAM_DATA_5.connect_to(&data_5_pin);

        let data_6_pin = data_6_pin.into();
        data_6_pin.apply_input_config(&InputConfig::default());
        data_6_pin.set_input_enable(true);
        InputSignal::CAM_DATA_6.connect_to(&data_6_pin);

        let data_7_pin = data_7_pin.into();
        data_7_pin.apply_input_config(&InputConfig::default());
        data_7_pin.set_input_enable(true);
        InputSignal::CAM_DATA_7.connect_to(&data_7_pin);

        // Match esp32-camera reference (ll_cam.c): CLK160 source (cam_clk_sel=3,
        // always 160 MHz), integer division only (div_b=div_a=0).
        // Formula: div_num = 160_000_000 / xclk_freq_hz  (e.g. 160/12 = 13).
        let xclk_hz = config.frequency().as_hz() as u32;
        let clk_div_num = 160_000_000u32
            .checked_div(xclk_hz)
            .filter(|&d| (2..=256).contains(&d))
            .ok_or(ConfigError::Clock(ClockError::FrequencyTooLow))? as u8;

        if let Some(value) = config.line_interrupt()
            && value > 0b0111_1111
        {
            return Err(ConfigError::LineInterrupt);
        }

        lcd_cam.register_block().cam_ctrl().write(|w| unsafe {
            w.cam_clk_sel().bits(3_u8);
            w.cam_clkm_div_num().bits(clk_div_num);
            w.cam_clkm_div_b().bits(0_u8);
            w.cam_clkm_div_a().bits(0_u8);
            if let Some(threshold) = config.vsync_filter_threshold() {
                w.cam_vsync_filter_thres().bits(threshold as _);
            }
            w.cam_byte_order()
                .bit(config.byte_order() != ByteOrder::default());
            w.cam_bit_order()
                .bit(config.bit_order() != BitOrder::default());
            w.cam_vs_eof_en()
                .bit(matches!(config.eof_mode(), EofMode::VsyncSignal));
            w.cam_line_int_en().bit(config.line_interrupt().is_some());
            w.cam_stop_en().clear_bit()
        });
        lcd_cam.register_block().cam_ctrl1().write(|w| unsafe {
            w.cam_2byte_en().bit(config.enable_2byte_mode());
            w.cam_vh_de_mode_en()
                .bit(matches!(config.vh_de_mode(), VhdeMode::VsyncHsync));
            if let EofMode::ByteLen(value) = config.eof_mode() {
                w.cam_rec_data_bytelen().bits(value);
            }
            if let Some(value) = config.line_interrupt() {
                w.cam_line_int_num().bits(value);
            }
            w.cam_vsync_filter_en()
                .bit(config.vsync_filter_threshold().is_some());
            w.cam_clk_inv().bit(config.invert_pixel_clock());
            w.cam_de_inv().bit(config.invert_h_enable());
            w.cam_hsync_inv().bit(config.invert_hsync());
            w.cam_vsync_inv().bit(config.invert_vsync())
        });

        match converter {
            ConverterMode::Yuv422Passthrough => unsafe {
                lcd_cam.register_block().cam_rgb_yuv().write(|w| {
                    // PAC semantics (counterintuitive naming):
                    //   cam_conv_bypass=1 → converter ACTIVE
                    //   cam_conv_bypass=0 → converter bypassed (AFIFO straight to DMA)
                    //
                    // Correct YUV422 passthrough:
                    //   bypass=1         → converter active (required for yuv2yuv_mode)
                    //   mode_8bits_on=1  → 8-bit DVP: converter pairs bytes without duplication
                    //   trans_mode=1     → YUV output; prerequisite to activate yuv2yuv_mode
                    //   yuv2yuv_mode=0   → output YUV422 (passthrough, no subsampling)
                    //   yuv_mode=0       → input is YUV422
                    //   data_in/out=0    → limited color range (OV2640 default: Y∈[16,235])
                    //   protocol_mode=0  → BT601
                    //   8bits_data_inv=0 → no byte-pair swap; BSYM=0 already gives
                    //                      YUYV [Y0,Cb,Y1,Cr] which the codec expects
                    w.cam_conv_bypass().set_bit();
                    w.cam_conv_mode_8bits_on().set_bit();
                    w.cam_conv_trans_mode().set_bit();
                    w.cam_conv_yuv2yuv_mode().bits(0);
                    w.cam_conv_yuv_mode().bits(0);
                    w.cam_conv_data_in_mode().clear_bit();
                    w.cam_conv_data_out_mode().clear_bit();
                    w.cam_conv_protocol_mode().clear_bit();
                    w.cam_conv_8bits_data_inv().clear_bit()
                });
            },
            ConverterMode::Bypass => {
                lcd_cam.register_block().cam_rgb_yuv().write(|w| {
                    // bypass=0 → converter bypassed; AFIFO pairs consecutive 8-bit
                    // bytes into 16-bit DMA words directly.  OV2640 RGB565 mode
                    // outputs [HB,LB] per pixel on the 8-bit bus; AFIFO produces
                    // correct RGB565 without conversion-unit involvement. JPEG is an
                    // opaque byte stream that must likewise pass through untouched.
                    w.cam_conv_bypass().clear_bit()
                });
            }
        }

        lcd_cam
            .register_block()
            .cam_ctrl()
            .modify(|_, w| w.cam_update().set_bit());

        #[cfg(feature = "defmt")]
        defmt::info!("Camera setup done");

        Ok(Self {
            lcd_cam,
            dma,
            initial_dma_addr: 0,
        })
    }

    pub fn receive<BUF: DmaRxBuffer>(
        mut self,
        mut buf: BUF,
    ) -> Result<AsyncCameraTransfer<'a, BUF>, (DmaError, Self, BUF)> {
        // cam_reset: forces LCD-CAM to wait for the next VSYNC before accepting
        // data, discarding the in-progress partial frame.
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_reset().set_bit());
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_reset().clear_bit());

        // cam_afifo_reset: flushes the async FIFO between the DVP data pins and
        // the DMA.  Any bytes pushed before cam_reset took effect are discarded,
        // so the DMA sees a clean byte stream starting at the next VSYNC.
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_afifo_reset().set_bit());
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_afifo_reset().clear_bit());

        let preparation = buf.prepare();
        self.initial_dma_addr = preparation.start as u32;

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        self.dma.clear_all();
        self.dma.reset();

        #[cfg(feature = "psram_dma")]
        if preparation.accesses_psram && !self.dma.can_access_psram() {
            return Err(DmaError::UnsupportedMemoryRegion);
        }

        #[cfg(feature = "psram_dma")]
        self.rx_impl
            .set_ext_mem_block_size(preparation.burst_transfer.external_memory.into());
        self.dma.set_burst_mode(preparation.burst_transfer);
        self.dma.set_descr_burst_mode(true);
        self.dma.set_mem2mem_mode(false);
        self.dma.set_check_owner(preparation.check_owner);
        self.dma.set_link_addr(preparation.start as u32);
        self.dma.set_peripheral(DmaPeripheral::LcdCam as u8);
        if let Some(handler) = self.dma.async_handler() {
            if let Some(interrupt) = self.dma.peripheral_interrupt() {
                for core in esp_hal::system::Cpu::other() {
                    esp_hal::interrupt::disable(core, interrupt);
                }
                unsafe { esp_hal::interrupt::bind_handler(interrupt, handler) };
                self.dma.unlisten(
                    DmaRxInterrupt::SuccessfulEof
                        | DmaRxInterrupt::ErrorEof
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::Done,
                );
                self.dma.clear(
                    DmaRxInterrupt::SuccessfulEof
                        | DmaRxInterrupt::ErrorEof
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::Done,
                );
                esp_hal::interrupt::enable(interrupt, handler.priority());
            }
        }
        self.dma.set_async(true);
        self.dma.start();

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        #[cfg(feature = "defmt")]
        defmt::trace!(
            "[CPU {}] Starting DMA transfer for camera",
            esp_hal::system::Cpu::current() as usize
        );

        self.lcd_cam.register_block().cam_ctrl().modify(|_, w| {
            w.cam_stop_en().clear_bit();
            w.cam_update().set_bit()
        });
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());

        Ok(AsyncCameraTransfer {
            camera: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
            descriptor_empty: false,
        })
    }

    /// Restart the DMA ring **without** resetting the camera module.
    ///
    /// Use after `DescriptorEmpty` (ring full, DMA stopped) to re-arm the
    /// descriptor chain and continue capturing the **current frame** mid-stream,
    /// without waiting for the next VSYNC.
    ///
    /// Unlike [`receive`](Self::receive):
    /// - **Skips** `cam_reset` — the camera keeps streaming its current frame.
    /// - **Skips** `cam_afifo_reset` — flushing the AFIFO generates a spurious
    ///   EOF/VSYNC event when `cam_vs_eof_en=1`, causing false frame boundaries.
    ///   AFIFO bytes are the exact stream continuation and must NOT be discarded.
    pub fn restart_dma_no_reset<BUF: DmaRxBuffer>(
        mut self,
        mut buf: BUF,
    ) -> Result<AsyncCameraTransfer<'a, BUF>, (DmaError, Self, BUF)> {
        // Re-arm descriptor ownership bits (buf.prepare() is the only path that
        // resets the hardware "owned-by-DMA" bits; consume() does not do this).
        let preparation = buf.prepare();
        self.initial_dma_addr = preparation.start as u32;

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        self.dma.clear_all();
        self.dma.reset();

        self.dma.set_burst_mode(preparation.burst_transfer);
        self.dma.set_descr_burst_mode(true);
        self.dma.set_mem2mem_mode(false);
        self.dma.set_check_owner(preparation.check_owner);
        self.dma.set_link_addr(preparation.start as u32);
        self.dma.set_peripheral(DmaPeripheral::LcdCam as u8);
        if let Some(handler) = self.dma.async_handler() {
            if let Some(interrupt) = self.dma.peripheral_interrupt() {
                for core in esp_hal::system::Cpu::other() {
                    esp_hal::interrupt::disable(core, interrupt);
                }
                unsafe { esp_hal::interrupt::bind_handler(interrupt, handler) };
                self.dma.unlisten(
                    DmaRxInterrupt::SuccessfulEof
                        | DmaRxInterrupt::ErrorEof
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::Done,
                );
                self.dma.clear(
                    DmaRxInterrupt::SuccessfulEof
                        | DmaRxInterrupt::ErrorEof
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::Done,
                );
                esp_hal::interrupt::enable(interrupt, handler.priority());
            }
        }
        self.dma.set_async(true);
        self.dma.start();

        compiler_fence(core::sync::atomic::Ordering::SeqCst);

        // Resume camera without cam_reset — camera keeps streaming mid-frame.
        self.lcd_cam.register_block().cam_ctrl().modify(|_, w| {
            w.cam_stop_en().clear_bit();
            w.cam_update().set_bit()
        });
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());

        Ok(AsyncCameraTransfer {
            camera: ManuallyDrop::new(self),
            buffer_view: ManuallyDrop::new(buf.into_view()),
            descriptor_empty: false,
        })
    }

    fn resume(&mut self, descriptor_empty: bool) {
        if descriptor_empty {
            // After DescriptorEmpty the DMA stopped; GDMA_IN_DSCR_CH0 holds null
            // (the exhausted chain's tail had next=null_ptr). restart() would read
            // that null and fire DescriptorError. Use set_link_addr+start() instead
            // to re-arm from the first descriptor (chain was rebuilt by consume()).
            self.dma.set_link_addr(self.initial_dma_addr);
            self.dma.start();
        } else {
            // DMA is still running (or stopped at VSYNC SuccessfulEof which auto-
            // clears cam_start). restart() is a no-op when DMA is live, and re-arms
            // it for the next frame after VSYNC.
            self.dma.restart();
        }
        self.lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().set_bit());
    }
}

pub struct AsyncCameraTransfer<'d, BUF: DmaRxBuffer> {
    camera: ManuallyDrop<AsyncCameraDriver<'d>>,
    buffer_view: ManuallyDrop<BUF::View>,
    /// Set by `wait()` when `DescriptorEmpty` fired (ring was full and DMA stopped).
    /// Consumed by `resume_dma()` to decide between `set_link_addr+start` vs `restart`.
    descriptor_empty: bool,
}

impl<'d, BUF: DmaRxBuffer> AsyncCameraTransfer<'d, BUF> {
    /// Stops this transfer and returns the peripheral and buffer.
    pub fn stop(mut self) -> (AsyncCameraDriver<'d>, BUF::Final) {
        self.stop_peripherals();
        let (camera, view) = self.release();
        (camera, BUF::from_view(view))
    }

    /// Async poll for DMA interrupts.
    pub async fn wait(&mut self) -> Result<(), DmaError> {
        poll_fn(|cx: &mut Context<'_>| {
            let pending = self.camera.dma.pending_interrupts();
            #[cfg(feature = "defmt")]
            defmt::trace!("DMA pending interrupts: {:?}", pending);

            // DescriptorEmpty is checked FIRST — even when Done fires simultaneously
            // (last descriptor of the ring), the ring is full and the DMA has stopped.
            // restart() would use GDMA_IN_DSCR_CH0 = null and fire DescriptorError;
            // we must use set_link_addr + start() instead (see resume_dma).
            if pending.contains(DmaRxInterrupt::DescriptorEmpty) {
                self.camera.dma.clear(
                    DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::Done
                        | DmaRxInterrupt::SuccessfulEof,
                );
                self.descriptor_empty = true;
                #[cfg(feature = "defmt")]
                defmt::trace!("DMA ring full (DescriptorEmpty)");
                Poll::Ready(Ok(()))
            } else if !pending.is_disjoint(DmaRxInterrupt::SuccessfulEof | DmaRxInterrupt::Done) {
                self.camera.dma.clear(DmaRxInterrupt::SuccessfulEof | DmaRxInterrupt::Done);
                #[cfg(feature = "defmt")]
                defmt::trace!("DMA READY to consume something");
                Poll::Ready(Ok(()))
            } else if pending.contains(DmaRxInterrupt::DescriptorError)
                || pending.contains(DmaRxInterrupt::ErrorEof)
            {
                self.camera.dma.clear(
                    DmaRxInterrupt::DescriptorError | DmaRxInterrupt::ErrorEof,
                );
                #[cfg(feature = "defmt")]
                defmt::trace!("DMA ERROR during frame");
                Poll::Ready(Err(DmaError::DescriptorError))
            } else {
                self.camera.dma.waker().register(cx.waker());
                #[cfg(feature = "defmt")]
                defmt::trace!(
                    "[CPU {}] Spurious interrupt: {:?}",
                    esp_hal::system::Cpu::current() as usize,
                    pending
                );
                self.camera.dma.listen(
                    DmaRxInterrupt::Done
                        | DmaRxInterrupt::SuccessfulEof
                        | DmaRxInterrupt::DescriptorError
                        | DmaRxInterrupt::DescriptorEmpty
                        | DmaRxInterrupt::ErrorEof,
                );
                Poll::Pending
            }
        })
        .await
    }

    fn release(mut self) -> (AsyncCameraDriver<'d>, BUF::View) {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "[CPU {}] Releasing camera",
            esp_hal::system::Cpu::current() as usize
        );
        let result = unsafe {
            let camera = ManuallyDrop::take(&mut self.camera);
            let view = ManuallyDrop::take(&mut self.buffer_view);
            (camera, view)
        };
        core::mem::forget(self);
        result
    }

    /// True if `wait()` last returned due to `DescriptorEmpty` (ring full, DMA stopped).
    pub fn is_descriptor_empty(&self) -> bool {
        self.descriptor_empty
    }

    /// Resume DMA after data was consumed.
    pub fn resume_dma(&mut self) {
        let was_empty = self.descriptor_empty;
        self.descriptor_empty = false;
        self.camera.resume(was_empty);
    }

    fn stop_peripherals(&mut self) {
        #[cfg(feature = "defmt")]
        defmt::trace!(
            "[CPU {}] Stopping camera peripheral",
            esp_hal::system::Cpu::current() as usize
        );
        self.camera
            .lcd_cam
            .register_block()
            .cam_ctrl1()
            .modify(|_, w| w.cam_start().clear_bit());
        self.camera.dma.stop();
    }
}

impl<BUF: DmaRxBuffer> Deref for AsyncCameraTransfer<'_, BUF> {
    type Target = BUF::View;

    fn deref(&self) -> &Self::Target {
        &self.buffer_view
    }
}

impl<BUF: DmaRxBuffer> DerefMut for AsyncCameraTransfer<'_, BUF> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.buffer_view
    }
}

impl<BUF: DmaRxBuffer> Drop for AsyncCameraTransfer<'_, BUF> {
    fn drop(&mut self) {
        self.stop_peripherals();
        unsafe {
            ManuallyDrop::drop(&mut self.camera);
            ManuallyDrop::drop(&mut self.buffer_view);
        }
    }
}
