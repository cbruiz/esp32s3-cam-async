//! Streaming JPEG frame extractor following ITU-T T.81.
//!
//! [`JpegStreamParser`] is a zero-allocation JPEG parser. Feed it raw bytes
//! from any source (DMA ring, network, serial) in arbitrary chunk sizes; it
//! copies one complete JPEG (SOI → EOI inclusive) into the provided output
//! buffer and signals completion via [`ParserProgressing::EndOfImage`].
//!
//! # Typical call pattern (streaming DMA chunks)
//!
//! ```ignore
//! let mut parser = JpegStreamParser::default();
//! let mut out = [0u8; MAX_FRAME_SIZE];
//! loop {
//!     let chunk = /* next DMA chunk */;
//!     parser.reset_input_offset();
//!     loop {
//!         match parser.parse(chunk, &mut out) {
//!             Ok(ParserProgressing::InputBufferEmpty) => break,
//!             Ok(ParserProgressing::EndOfImage) => { /* frame in out[..parser.bytes_written()] */ break; }
//!             Ok(ParserProgressing::OutputBufferFull) => { parser.reset_output_offset(); break; }
//!             Err(_) => break, // corrupt — parser auto-resets; try next chunk
//!         }
//!     }
//! }
//! ```

/// Progress value returned by a successful [`JpegStreamParser::parse`] call.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ParserProgressing {
    /// All input bytes have been consumed; feed the next chunk.
    InputBufferEmpty,
    /// Output buffer is full; consume or discard it, then call
    /// [`reset_output_offset`](JpegStreamParser::reset_output_offset).
    OutputBufferFull,
    /// A complete JPEG (SOI … EOI) was written to the output buffer.
    EndOfImage,
}

/// Internal parser state machine states.
///
/// Returned as the `Err` variant when the parser detects a structural error and
/// resets to [`LookingForSOI`](JIFState::LookingForSOI) for the next frame.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum JIFState {
    LookingForSOI,
    ReadSOIMarker,
    LookingForMarker,
    ReadingMarker,
    ReadingLength1,
    ReadingLength2,
    ReadingBody,
    ReadingEntropyData,
}

/// JFIF/JPEG segment marker codes (the byte following an `0xFF` marker prefix).
///
/// The full ITU-T T.81 Table B.1 marker set is enumerated so the parser can
/// classify every code it may encounter into one of three structural classes:
///
/// - **Standalone** — no length field, no payload: [`SOI`](Self::SOI),
///   [`EOI`](Self::EOI), [`RST`](Self::RST), [`TEM`](Self::TEM).
/// - **Length-prefixed segment** — a 2-byte big-endian length follows, then a
///   body of `length − 2` bytes: everything else listed below.
/// - **Reserved/invalid** — codes with no defined meaning
///   ([`Reserved`](Self::Reserved)); their appearance between segments means the
///   byte stream is corrupt.
///
/// Ref: ITU-T T.81 Table B.1 — <https://www.w3.org/Graphics/JPEG/itu-t81.pdf> page 32.
#[derive(Clone, Copy, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum JFIFMarker {
    /// Start of image (`0xD8`). Standalone.
    SOI,
    /// End of image (`0xD9`). Standalone.
    EOI,
    /// Start of frame. `u8` is the offset from `0xC0`, so `SOF(0)` = baseline
    /// SOF0 … `SOF(15)` = SOF15. Excludes the `0xC4`/`0xC8`/`0xCC` codes, which
    /// map to [`DHT`](Self::DHT)/[`JPG`](Self::JPG)/[`DAC`](Self::DAC).
    /// Length-prefixed.
    SOF(u8),
    /// Define Huffman table (`0xC4`). Length-prefixed.
    DHT,
    /// Define arithmetic-coding conditioning (`0xCC`). Length-prefixed.
    DAC,
    /// JPEG extension (`0xC8`). Length-prefixed.
    JPG,
    /// Start of scan (`0xDA`). Length-prefixed header, then entropy-coded data.
    SOS,
    /// Define quantisation table (`0xDB`). Length-prefixed.
    DQT,
    /// Define number of lines (`0xDC`). Length-prefixed.
    DNL,
    /// Define restart interval (`0xDD`). Length-prefixed.
    DRI,
    /// Define hierarchical progression (`0xDE`). Length-prefixed.
    DHP,
    /// Expand reference component (`0xDF`). Length-prefixed.
    EXP,
    /// Application segment APP0 (`u8 = 0`) … APP15 (`u8 = 15`). Length-prefixed.
    APP(u8),
    /// JPEG extension JPG0 (`u8 = 0`) … JPG13 (`u8 = 13`), codes `0xF0`–`0xFD`.
    /// Length-prefixed.
    JPGn(u8),
    /// Comment (`0xFE`). Length-prefixed.
    COM,
    /// Restart marker RST0 (`u8 = 0`) … RST7 (`u8 = 7`), codes `0xD0`–`0xD7`.
    /// Standalone; appears only inside entropy-coded data.
    RST(u8),
    /// Temporary marker for arithmetic coding (`0x01`). Standalone.
    TEM,
    /// Reserved/invalid marker code (`0x02`–`0xBF`, etc.). Indicates a corrupt
    /// stream — the parser drops the partial frame and resyncs to the next SOI.
    Reserved(u8),
    /// No marker — initial / cleared state.
    None,
}

impl From<u8> for JFIFMarker {
    fn from(value: u8) -> Self {
        match value {
            0x01 => JFIFMarker::TEM,
            0xC4 => JFIFMarker::DHT,
            0xC8 => JFIFMarker::JPG,
            0xCC => JFIFMarker::DAC,
            // Remaining 0xC0..=0xCF codes are SOF0..SOF15.
            0xC0..=0xCF => JFIFMarker::SOF(value - 0xC0),
            0xD0..=0xD7 => JFIFMarker::RST(value - 0xD0),
            0xD8 => JFIFMarker::SOI,
            0xD9 => JFIFMarker::EOI,
            0xDA => JFIFMarker::SOS,
            0xDB => JFIFMarker::DQT,
            0xDC => JFIFMarker::DNL,
            0xDD => JFIFMarker::DRI,
            0xDE => JFIFMarker::DHP,
            0xDF => JFIFMarker::EXP,
            0xE0..=0xEF => JFIFMarker::APP(value - 0xE0),
            0xF0..=0xFD => JFIFMarker::JPGn(value - 0xF0),
            0xFE => JFIFMarker::COM,
            // 0x00 (byte stuffing) and 0xFF (fill) never reach here — they are
            // consumed by the marker-scanning states before classification.
            _ => JFIFMarker::Reserved(value),
        }
    }
}

/// Streaming JPEG frame extractor.
///
/// Feed arbitrary-sized byte slices via [`parse`](Self::parse). The parser
/// copies exactly one JPEG (SOI … EOI) into the destination buffer and returns
/// [`ParserProgressing::EndOfImage`] when done.
///
/// ## State across chunks
///
/// Call [`reset_input_offset`](Self::reset_input_offset) before each new
/// input chunk. All other state (frame position, output cursor) persists
/// between chunks so partial frames span chunk boundaries correctly.
#[derive(Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct JpegStreamParser {
    write_offset: usize,
    parse_offset: usize,
    parse_commit: usize,
    state: JIFState,
    discard_data: bool,
    last_byte: u8,
    current_marker: JFIFMarker,
    body_remaining: usize,
}

impl Default for JpegStreamParser {
    fn default() -> Self {
        Self::new()
    }
}

impl JpegStreamParser {
    pub fn new() -> Self {
        Self {
            state: JIFState::LookingForSOI,
            write_offset: 0,
            parse_offset: 0,
            parse_commit: 0,
            discard_data: true,
            last_byte: 0,
            current_marker: JFIFMarker::None,
            body_remaining: 0,
        }
    }

    /// Parse bytes from `src` into `dest`.
    ///
    /// Returns `Ok(progress)` as long as the stream is structurally valid.
    /// Returns `Err(state)` when a structural error is detected; the parser
    /// resets automatically — no need to call [`reset_state`](Self::reset_state).
    pub fn parse(&mut self, src: &[u8], dest: &mut [u8]) -> Result<ParserProgressing, JIFState> {
        loop {
            #[cfg(feature = "defmt")]
            defmt::trace!(
                "state={:?} src_idx={} dst_idx={} marker={:?}",
                self.state,
                self.parse_offset,
                self.write_offset,
                self.current_marker
            );

            if self.write_offset >= dest.len() {
                break Ok(ParserProgressing::OutputBufferFull);
            } else if self.parse_offset >= src.len() {
                break Ok(ParserProgressing::InputBufferEmpty);
            }

            let current_byte = src[self.parse_offset];

            match self.state {
                JIFState::LookingForSOI => {
                    if current_byte == 0xFF {
                        self.discard_data = false;
                        self.write_offset = 0;
                        self.parse_commit = self.parse_offset;
                        #[cfg(feature = "defmt")]
                        defmt::trace!("0xFF @ {} — potential SOI prefix", self.parse_offset);
                        self.state = JIFState::ReadSOIMarker;
                    } else {
                        self.discard_data = true;
                        self.parse_commit = self.parse_offset;
                        self.write_offset = 0;
                    }
                }
                JIFState::ReadSOIMarker => {
                    if JFIFMarker::from(current_byte) == JFIFMarker::SOI {
                        #[cfg(feature = "defmt")]
                        defmt::trace!("SOI @ {}", self.parse_offset);
                        self.parse_commit = self.parse_offset;
                        self.state = JIFState::LookingForMarker;
                    } else {
                        self.discard_data = true;
                        self.current_marker = JFIFMarker::None;
                        self.state = JIFState::LookingForSOI;
                    }
                }
                JIFState::LookingForMarker => {
                    if current_byte == 0xFF {
                        self.state = JIFState::ReadingMarker;
                    } else {
                        self.dump("expected 0xFF delimiter", current_byte, src);
                        self.reset(current_byte);
                        break Err(JIFState::LookingForMarker);
                    }
                }
                JIFState::ReadingMarker => {
                    if current_byte == 0xFF {
                        // Fill byte — stay in ReadingMarker.
                    } else if current_byte == 0x00 && self.last_byte == 0xFF {
                        // Byte-stuffed 0xFF — stay in ReadingMarker.
                    } else {
                        self.current_marker = JFIFMarker::from(current_byte);
                        match self.current_marker {
                            JFIFMarker::SOI => {
                                // SOI inside a frame is a structural error.
                                self.dump("unexpected SOI inside frame", current_byte, src);
                                self.reset(current_byte);
                                break Err(JIFState::ReadingMarker);
                            }
                            JFIFMarker::EOI => {
                                #[cfg(feature = "defmt")]
                                defmt::trace!("EOI @ {}", self.parse_offset);
                                self.state = JIFState::LookingForSOI;
                                self.parse_commit = self.parse_offset;
                                self.accept(current_byte, dest);
                                self.discard_data = true;
                                self.current_marker = JFIFMarker::None;
                                break Ok(ParserProgressing::EndOfImage);
                            }
                            // ── Standalone markers (no length field, no body) ──
                            JFIFMarker::RST(_) => {
                                // Restart marker — resume entropy-coded data.
                                self.state = JIFState::ReadingEntropyData;
                            }
                            JFIFMarker::TEM => {
                                // No payload; resume scanning for the next marker.
                                self.state = JIFState::LookingForMarker;
                            }
                            // ── Corruption: reserved/invalid code between segments ──
                            JFIFMarker::Reserved(_m) => {
                                #[cfg(feature = "defmt")]
                                defmt::warn!(
                                    "Reserved/invalid JFIF marker 0x{:02X} — resyncing to next SOI",
                                    _m
                                );
                                self.dump("reserved marker", current_byte, src);
                                self.reset(current_byte);
                                break Err(JIFState::ReadingMarker);
                            }
                            JFIFMarker::None => unreachable!(),
                            // ── Length-prefixed segments ──
                            // SOF(_), DHT, DAC, JPG, SOS, DQT, DNL, DRI, DHP,
                            // EXP, APP(_), JPGn(_), COM all carry a 2-byte length.
                            _ => {
                                self.state = JIFState::ReadingLength1;
                            }
                        }
                    }
                }
                JIFState::ReadingLength1 => {
                    self.body_remaining = (current_byte as usize) << 8;
                    self.state = JIFState::ReadingLength2;
                }
                JIFState::ReadingLength2 => {
                    self.body_remaining |= current_byte as usize;
                    if self.body_remaining < 2 {
                        self.dump("segment length < 2", current_byte, src);
                        self.reset(current_byte);
                        break Err(JIFState::ReadingLength2);
                    }
                    self.body_remaining -= 2;
                    self.state = if self.body_remaining == 0 {
                        if self.current_marker == JFIFMarker::SOS {
                            JIFState::ReadingEntropyData
                        } else {
                            JIFState::LookingForMarker
                        }
                    } else {
                        JIFState::ReadingBody
                    };
                }
                JIFState::ReadingBody => {
                    self.body_remaining -= 1;
                    if self.body_remaining == 0 {
                        self.state = if self.current_marker == JFIFMarker::SOS {
                            JIFState::ReadingEntropyData
                        } else {
                            JIFState::LookingForMarker
                        };
                    }
                }
                JIFState::ReadingEntropyData => {
                    if self.last_byte == 0xFF {
                        match current_byte {
                            // Stuffed 0xFF: the 0x00 is not emitted; 0xFF was already written.
                            0x00 => {}
                            // Fill byte: stay in entropy data.
                            0xFF => {}
                            _ => {
                                // Real marker — re-process without consuming this byte.
                                self.state = JIFState::ReadingMarker;
                                continue;
                            }
                        }
                    }
                }
            }

            self.accept(current_byte, dest);
        }
    }

    #[inline]
    fn accept(&mut self, current_byte: u8, dest: &mut [u8]) {
        if !self.discard_data {
            dest[self.write_offset] = current_byte;
            self.write_offset += 1;
        }
        self.last_byte = current_byte;
        self.parse_offset += 1;
    }

    #[inline]
    fn reset(&mut self, current_byte: u8) {
        self.last_byte = current_byte;
        self.parse_offset = self.parse_commit;
        self.state = JIFState::LookingForSOI;
        self.current_marker = JFIFMarker::None;
        self.discard_data = true;
        self.write_offset = 0;
    }

    fn dump(&self, reason: &str, current_byte: u8, src: &[u8]) {
        #[cfg(feature = "defmt")]
        defmt::debug!(
            "JPEG parser error ({}): byte=0x{:02X} state={:?} marker={:?} parse_offset={} write_offset={}",
            reason,
            current_byte,
            self.state,
            self.current_marker,
            self.parse_offset,
            self.write_offset,
        );
        #[cfg(feature = "defmt")]
        if self.parse_offset > 0 {
            defmt::debug!("src[..offset]: {:02X}", src[..self.parse_offset]);
        }
        let _ = (reason, current_byte, src);
    }

    /// Reset the input cursor to 0. Call before feeding each new input chunk.
    pub fn reset_input_offset(&mut self) {
        self.parse_offset = 0;
        self.parse_commit = 0;
    }

    /// Abort the current partial frame and resume scanning for the next SOI.
    ///
    /// Use when abandoning a corrupt or oversized frame.
    pub fn reset_state(&mut self) {
        self.state = JIFState::LookingForSOI;
        self.current_marker = JFIFMarker::None;
        self.discard_data = true;
        self.write_offset = 0;
        self.body_remaining = 0;
        self.parse_offset = self.parse_commit;
        self.last_byte = 0;
    }

    /// Reset the output write cursor to 0.
    ///
    /// Use when the caller has consumed the completed frame data and the output
    /// buffer can be reused for the next frame.
    pub fn reset_output_offset(&mut self) {
        self.write_offset = 0;
    }

    /// Number of bytes written to the output buffer for the current frame.
    pub fn bytes_written(&self) -> usize {
        self.write_offset
    }

    /// Current parser state.
    pub fn state(&self) -> JIFState {
        self.state
    }
}

// ── RFC 2435 utilities ───────────────────────────────────────────────────────

/// Round `a` up to the nearest multiple of `b`.
#[allow(unused)]
pub fn ceil_div_u16(a: u16, b: u16) -> u16 {
    (a + (b - 1)) / b
}

/// Metadata extracted from a complete JPEG for RTP/JPEG encapsulation per RFC 2435.
#[allow(unused)]
#[derive(Clone, Copy, Debug)]
pub struct JpegRfc2435Info {
    /// Image width in 8-pixel blocks.
    pub width_blocks: u8,
    /// Image height in 8-pixel blocks.
    pub height_blocks: u8,
    /// RFC 2435 type field: 0 = 4:2:2, 1 = 4:2:0.
    pub jpeg_type: u8,
    /// DRI restart interval (0 if absent).
    pub restart_interval: u16,
    /// Number of bytes in `qt` (64 or 128).
    pub qt_len: u16,
    /// Quantisation tables (up to two 64-byte tables, luma first).
    pub qt: [u8; 128],
    /// Number of bytes written into `scan_dest` by [`parse_jpeg_for_rfc2435`].
    pub scan_len: usize,
    /// Byte offsets in `scan_dest` where each restart interval begins.
    pub restart_offsets: [u32; 128],
    /// Number of valid entries in `restart_offsets`.
    pub num_restarts: usize,
}

/// Parse a complete JPEG buffer and extract fields required for RFC 2435 RTP
/// encapsulation. The raw entropy scan data (without restart markers) is
/// written into `scan_dest`.
#[allow(unused)]
pub fn parse_jpeg_for_rfc2435(
    jpeg: &[u8],
    scan_dest: &mut [u8],
) -> Result<JpegRfc2435Info, &'static str> {
    if jpeg.len() < 4 || jpeg[0] != 0xFF || jpeg[1] != 0xD8 {
        return Err("Invalid JPEG header");
    }

    let mut width: u16 = 0;
    let mut height: u16 = 0;
    let mut qt = [0u8; 128];
    let mut qt_len = 0usize;
    let mut restart_interval: u16 = 0;
    let mut jpeg_type: u8 = 1; // default 4:2:0

    let mut i = 2; // skip SOI

    while i + 1 < jpeg.len() {
        if jpeg[i] != 0xFF {
            i += 1;
            continue;
        }
        let marker = jpeg[i + 1];
        if marker == 0xFF {
            i += 1;
            continue;
        }
        if marker == 0xD9 || (0xD0..=0xD7).contains(&marker) || marker == 0x01 {
            i += 2;
            continue;
        }
        if i + 3 >= jpeg.len() {
            break;
        }
        let seg_len = u16::from_be_bytes([jpeg[i + 2], jpeg[i + 3]]) as usize;
        if seg_len < 2 || i + 2 + seg_len > jpeg.len() {
            break;
        }
        let payload = &jpeg[i + 4..i + 2 + seg_len];

        match marker {
            0xC0 => {
                // SOF0
                if payload.len() >= 6 && payload[0] == 8 {
                    height = u16::from_be_bytes([payload[1], payload[2]]);
                    width = u16::from_be_bytes([payload[3], payload[4]]);
                    if payload[5] == 3 && payload.len() >= 15 {
                        let h_samp = (payload[7] >> 4) & 0x0F;
                        let v_samp = payload[7] & 0x0F;
                        let h_samp_c = (payload[10] >> 4) & 0x0F;
                        let v_samp_c = payload[10] & 0x0F;
                        jpeg_type = if h_samp == 2 && v_samp == 1 && h_samp_c == 1 && v_samp_c == 1
                        {
                            0 // 4:2:2
                        } else {
                            1 // 4:2:0 (also covers 2×2 / 1×1 / 1×1)
                        };
                    }
                } else if payload.len() >= 1 && payload[0] != 8 {
                    return Err("Unsupported precision (not 8-bit)");
                }
            }
            0xDB => {
                // DQT
                let mut off = 0;
                while off < payload.len() && qt_len + 64 <= 128 {
                    let pq_tq = payload[off];
                    off += 1;
                    if pq_tq >> 4 != 0 {
                        return Err("16-bit DQT not supported");
                    }
                    if off + 64 > payload.len() {
                        break;
                    }
                    qt[qt_len..qt_len + 64].copy_from_slice(&payload[off..off + 64]);
                    qt_len += 64;
                    off += 64;
                }
            }
            0xDD => {
                // DRI
                if payload.len() >= 2 {
                    restart_interval = u16::from_be_bytes([payload[0], payload[1]]);
                }
            }
            0xDA => {
                // SOS — extract scan data and return
                let scan_start = i + 2 + seg_len;
                let mut restart_offsets = [0u32; 128];
                let (scan_len, num_restarts) = extract_scan_data_stuffed(
                    &jpeg[scan_start..],
                    scan_dest,
                    &mut restart_offsets,
                )?;
                return Ok(JpegRfc2435Info {
                    width_blocks: ceil_div_u16(width, 8) as u8,
                    height_blocks: ceil_div_u16(height, 8) as u8,
                    jpeg_type,
                    restart_interval,
                    qt_len: qt_len as u16,
                    qt,
                    scan_len,
                    restart_offsets,
                    num_restarts,
                });
            }
            _ => {}
        }

        i += 2 + seg_len;
    }

    Err("SOS marker not found")
}

fn extract_scan_data_stuffed(
    jpeg_scan: &[u8],
    dest: &mut [u8],
    restart_offsets: &mut [u32; 128],
) -> Result<(usize, usize), &'static str> {
    let mut write_idx = 0usize;
    let mut read_idx = 0usize;
    let mut num_restarts = 1usize;
    restart_offsets[0] = 0;

    while read_idx < jpeg_scan.len() {
        if jpeg_scan[read_idx] != 0xFF {
            if write_idx >= dest.len() {
                return Err("Scan buffer overflow");
            }
            dest[write_idx] = jpeg_scan[read_idx];
            write_idx += 1;
            read_idx += 1;
            continue;
        }

        // 0xFF byte
        if read_idx + 1 >= jpeg_scan.len() {
            if write_idx >= dest.len() {
                return Err("Scan buffer overflow");
            }
            dest[write_idx] = 0xFF;
            write_idx += 1;
            read_idx += 1;
            continue;
        }

        let next = jpeg_scan[read_idx + 1];
        match next {
            0xD9 => break, // EOI — end of scan
            0xD0..=0xD7 => {
                // Restart marker — record offset, skip bytes
                read_idx += 2;
                if num_restarts < 128 {
                    restart_offsets[num_restarts] = write_idx as u32;
                    num_restarts += 1;
                }
            }
            _ => {
                if write_idx + 1 >= dest.len() {
                    return Err("Scan buffer overflow");
                }
                dest[write_idx] = 0xFF;
                dest[write_idx + 1] = next;
                write_idx += 2;
                read_idx += 2;
            }
        }
    }

    Ok((write_idx, num_restarts))
}
