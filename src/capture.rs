//! High-level async capture loop wrapping [`AsyncCameraDriver`].
//!
//! [`CameraCapture`] owns the DMA state machine extracted from the camera task:
//! - Timeout detection → full `receive()` restart
//! - `DescriptorEmpty` (ring full) → `restart_dma_no_reset()` to continue mid-frame
//! - DMA error → stop + `receive()` + return empty chunk
//! - VSYNC EOF → clears the resyncing flag
use embassy_time::{Duration, Timer};
use esp_hal::dma::{DmaError, DmaRxStreamBuf};

use crate::driver::{AsyncCameraDriver, AsyncCameraTransfer};

/// Timing configuration for [`CameraCapture`].
pub struct CaptureConfig {
    /// Milliseconds to wait for a DMA interrupt before declaring a timeout.
    pub dma_wait_timeout_ms: u64,
    /// Milliseconds to delay after a DMA error or timeout before restarting.
    pub dma_recovery_delay_ms: u64,
}

impl Default for CaptureConfig {
    fn default() -> Self {
        Self {
            dma_wait_timeout_ms: 300,
            dma_recovery_delay_ms: 1,
        }
    }
}

/// What triggered the DMA restart on this iteration, if anything.
#[derive(Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum CaptureEvent {
    None,
    /// 300 ms elapsed with no DMA interrupt — resyncing to next VSYNC.
    Timeout,
    /// Hardware DMA error — full reset performed.
    DmaError,
}

/// Result of a single [`CameraCapture::next_chunk`] call.
pub struct ChunkResult {
    /// Bytes written into the caller's buffer (0 on DMA error).
    pub len: usize,
    /// VSYNC EOF detected — frame boundary.
    pub eof: bool,
    /// True after a timeout or DMA error, until the next VSYNC EOF clears it.
    /// When set, the copied data may be from an unknown frame position and
    /// should be discarded by the caller.
    pub resyncing: bool,
    /// What triggered a restart this iteration (informational; use for metrics).
    pub event: CaptureEvent,
}

enum CaptureState<'d> {
    Idle {
        driver: AsyncCameraDriver<'d>,
        buf: DmaRxStreamBuf,
    },
    Active(AsyncCameraTransfer<'d, DmaRxStreamBuf>),
    /// Transient placeholder used only inside state-transition code.
    /// Must never be observable from outside [`CameraCapture`].
    Transitioning,
}

/// DMA capture state machine for the ESP32-S3 LCD-CAM camera interface.
///
/// Create with [`CameraCapture::new`], call [`start`](Self::start) once,
/// then drive with [`next_chunk`](Self::next_chunk) in a loop.
pub struct CameraCapture<'d> {
    state: CaptureState<'d>,
    resyncing: bool,
    wait_timeout_ms: u64,
    recovery_delay_ms: u64,
}

impl<'d> CameraCapture<'d> {
    pub fn new(driver: AsyncCameraDriver<'d>, buf: DmaRxStreamBuf, config: CaptureConfig) -> Self {
        Self {
            state: CaptureState::Idle { driver, buf },
            resyncing: false,
            wait_timeout_ms: config.dma_wait_timeout_ms,
            recovery_delay_ms: config.dma_recovery_delay_ms,
        }
    }

    /// Arm the DMA and begin receiving sensor data.
    ///
    /// Idempotent — safe to call again after [`stop_capture`](Self::stop_capture).
    pub fn start(&mut self) -> Result<(), DmaError> {
        let CaptureState::Idle { .. } = self.state else {
            return Ok(());
        };
        let CaptureState::Idle { driver, buf } =
            core::mem::replace(&mut self.state, CaptureState::Transitioning)
        else {
            unreachable!()
        };
        match driver.receive(buf) {
            Ok(t) => {
                self.state = CaptureState::Active(t);
                Ok(())
            }
            Err((e, driver, buf)) => {
                self.state = CaptureState::Idle { driver, buf };
                Err(e)
            }
        }
    }

    /// Stop the DMA and return to idle.
    /// Call before pausing the pipeline; follow with [`start`](Self::start) to resume.
    pub fn stop_capture(&mut self) {
        match core::mem::replace(&mut self.state, CaptureState::Transitioning) {
            CaptureState::Active(t) => {
                let (driver, buf) = t.stop();
                self.state = CaptureState::Idle { driver, buf };
            }
            other => self.state = other,
        }
        self.resyncing = false;
    }

    /// True while waiting for the next VSYNC after a timeout or DMA error.
    pub fn is_resyncing(&self) -> bool {
        self.resyncing
    }

    /// Wait for the next DMA chunk and copy it into `out`.
    ///
    /// Handles all DMA restarts internally. The caller should:
    /// - Discard `out` data when `result.resyncing` is true.
    /// - Reset line-tracking counters when `result.event != CaptureEvent::None`.
    /// - Reset frame state on `result.eof`.
    pub async fn next_chunk(&mut self, out: &mut [u8]) -> ChunkResult {
        if let CaptureState::Idle { .. } = self.state {
            if self.start().is_err() {
                return ChunkResult {
                    len: 0,
                    eof: false,
                    resyncing: true,
                    event: CaptureEvent::DmaError,
                };
            }
        }

        // Copy config to locals so the borrow on self.state can proceed freely.
        let wait_timeout_ms = self.wait_timeout_ms;
        let recovery_delay_ms = self.recovery_delay_ms;

        // --- Wait for a DMA interrupt ---
        let do_full_restart;
        let descriptor_empty_restart;

        {
            let CaptureState::Active(ref mut transfer) = self.state else {
                unreachable!()
            };
            match embassy_time::with_timeout(
                Duration::from_millis(wait_timeout_ms),
                transfer.wait(),
            )
            .await
            {
                Ok(Ok(_)) => {
                    descriptor_empty_restart = transfer.is_descriptor_empty();
                    do_full_restart = descriptor_empty_restart;
                }
                Ok(Err(_dma_err)) => {
                    // DMA error: stop, reset, return empty.
                    #[cfg(feature = "defmt")]
                    defmt::warn!("CameraCapture: DMA error, resetting");
                    // Fall through to the error-restart path below.
                    let _ = transfer; // end borrow so we can move self.state
                    let CaptureState::Active(t) =
                        core::mem::replace(&mut self.state, CaptureState::Transitioning)
                    else {
                        unreachable!()
                    };
                    let (driver, buf) = t.stop();
                    Timer::after(Duration::from_millis(recovery_delay_ms)).await;
                    match driver.receive(buf) {
                        Ok(t) => self.state = CaptureState::Active(t),
                        Err((_, d, b)) => self.state = CaptureState::Idle { driver: d, buf: b },
                    }
                    self.resyncing = true;
                    return ChunkResult {
                        len: 0,
                        eof: false,
                        resyncing: true,
                        event: CaptureEvent::DmaError,
                    };
                }
                Err(_timeout) => {
                    #[cfg(feature = "defmt")]
                    defmt::warn!("CameraCapture: DMA timeout, waiting for VSYNC");
                    self.resyncing = true;
                    do_full_restart = true;
                    descriptor_empty_restart = false;
                }
            }
        }

        // --- Peek data and copy into caller's buffer ---
        let (data_len, eof) = {
            let CaptureState::Active(ref mut transfer) = self.state else {
                unreachable!()
            };
            let (data_len, eof) = {
                let (data, eof) = transfer.peek_until_eof();
                let n = data.len();
                if n > 0 {
                    let copy = n.min(out.len());
                    out[..copy].copy_from_slice(&data[..copy]);
                }
                (n, eof)
                // `data` borrow ends here
            };
            transfer.consume(data_len);
            (data_len, eof)
        };

        // --- Restart DMA ---
        let event = if do_full_restart {
            let CaptureState::Active(t) =
                core::mem::replace(&mut self.state, CaptureState::Transitioning)
            else {
                unreachable!()
            };
            let (driver, buf) = t.stop();
            if descriptor_empty_restart {
                // Ring filled mid-frame: re-arm without cam_reset to continue current frame.
                match driver.restart_dma_no_reset(buf) {
                    Ok(t) => self.state = CaptureState::Active(t),
                    Err((_, d, b)) => self.state = CaptureState::Idle { driver: d, buf: b },
                }
                CaptureEvent::None // DescriptorEmpty is normal operation, not an error
            } else {
                // Timeout: full receive with cam_reset — waits for next VSYNC.
                Timer::after(Duration::from_millis(recovery_delay_ms)).await;
                match driver.receive(buf) {
                    Ok(t) => self.state = CaptureState::Active(t),
                    Err((_, d, b)) => self.state = CaptureState::Idle { driver: d, buf: b },
                }
                CaptureEvent::Timeout
            }
        } else {
            let CaptureState::Active(ref mut t) = self.state else {
                unreachable!()
            };
            t.resume_dma();
            CaptureEvent::None
        };

        // VSYNC EOF clears the resyncing flag.
        if eof {
            self.resyncing = false;
        }

        ChunkResult {
            len: data_len,
            eof,
            resyncing: self.resyncing,
            event,
        }
    }
}
