use crate::{Error, MacKey, PsduMeta, Radio, TxError, MAX_PSDU_SIZE};

impl openthread::RadioError for Error {
    fn kind(&self) -> openthread::RadioErrorKind {
        use openthread::RadioErrorKind;

        match self {
            Error::TransmitDataTooLarge | Error::ReceiveBufTooSmall => RadioErrorKind::Other,
            // Couldn't even hand the frame to the driver (radio busy after the
            // schedule retries). Treat like a channel-access failure so OpenThread
            // backs off and retransmits.
            Error::ScheduleTransmit => RadioErrorKind::TxFailed,
            Error::Transmit(e) => match e {
                // CSMA-CA gave up: the channel was still busy after all backoffs.
                // This is the only TxError that is genuinely a channel-access
                // failure (`OT_ERROR_CHANNEL_ACCESS_FAILURE`).
                TxError::BusyChannel => RadioErrorKind::TxFailed,
                // The frame WENT OUT but no valid ACK came back. These must be
                // surfaced as NO_ACK (not channel-access) so OpenThread applies its
                // no-ack retransmission policy. Folding them into `TxFailed` was
                // mislabeling every no-ack as a `ChannelAccessFailure`.
                TxError::NoAck => RadioErrorKind::RxAckTimeout,
                TxError::InvalidAck => RadioErrorKind::RxAckInvalid,
                // MPSL denied/ended our radio timeslot — no airtime to transmit.
                // Closest to a channel-access failure; let OpenThread retry.
                TxError::TimeslotEnded | TxError::TimeslotDenied => RadioErrorKind::TxFailed,
                // Aborted by another op, out of ACK buffers, or an unknown driver
                // code: a generic transmit failure (→ `OT_ERROR_ABORT`).
                TxError::Aborted | TxError::NoMem | TxError::Unknown(_) => RadioErrorKind::Other,
            },
            Error::EnterReceive | Error::Receive | Error::ScheduleReceive => {
                RadioErrorKind::RxFailed
            }
            Error::EnterSleep | Error::Security(_) => RadioErrorKind::Other,
        }
    }
}

impl PsduMeta {
    fn as_openthread(&self, channel: u8) -> openthread::PsduRxInfo {
        openthread::PsduRxInfo {
            len: self.len as usize + 2,
            channel,
            rssi: Some(self.power),
            lqi: self.lqi,
            timestamp_us: self.phr_time(),
            ack_security: self.ack_security.map(|sec| openthread::AckSecurity {
                frame_counter: sec.frame_counter,
                key_id: sec.key_id,
            }),
        }
    }

    fn write_crc(&self, buf: &mut [u8]) {
        let len = self.len as usize;
        buf[len..len + 2].copy_from_slice(self.crc.to_le_bytes().as_slice());
    }
}

/// A wrapper around [`Radio`] that implements the [`openthread::Radio`] trait
/// with config caching.
///
/// OpenThread pushes its standing configuration before radio operations, and
/// channel/power now arrive with each operation. Without caching, every call
/// would go directly to the Nordic 802.15.4 C driver, which can disrupt
/// in-progress radio operations. This wrapper caches the last applied values
/// and only forwards actual changes to the driver.
///
/// This matches the caching pattern used by the `NrfRadio` and `EspRadio`
/// wrappers in the `openthread` crate.
///
/// # Example
///
/// ```no_run
/// let radio = nrf_802154::Radio::new(/* ... */);
/// let ot_radio = nrf_802154::OpenThreadRadio::new(radio);
/// // Pass ot_radio to OpenThread::run() or EnetRunner::run()
/// ```
pub struct OpenThreadRadio<'d> {
    radio: Radio<'d>,
    config: openthread::Config,
    power: i8,
    cca_threshold: Option<i8>,
    /// The CSMA-CA backoff limit last applied to the driver; `None` while it
    /// is still the driver's default.
    csma_max_backoffs: Option<u8>,
    /// Whether the receiver is currently commanded by a timed window
    /// (`receive_at`) rather than by `set_receive` / `set_sleep`.
    timed_rx: bool,
    /// The channel of the last timed window, which frames received in it
    /// arrived on.
    window_channel: u8,
    /// The last CSL schedule applied to the driver.
    csl: openthread::CslConfig,
    /// The clock accuracy reported to a CSL parent, in PPM.
    csl_accuracy_ppm: u8,
    /// The timed-receive uncertainty reported to a CSL parent, in 10 µs units.
    csl_uncertainty: u8,
}

/// The driver clock, as an `openthread::RadioClock`.
fn radio_now_us() -> u64 {
    unsafe { crate::raw::nrf_802154_time_get() }
}

impl<'d> OpenThreadRadio<'d> {
    /// Create a new `OpenThreadRadio` wrapper around the given radio.
    ///
    /// The initial config is applied to the driver immediately.
    pub fn new(mut radio: Radio<'d>) -> Self {
        let config = openthread::Config::new();
        Self::apply_config(&mut radio, &config);
        let power = radio.tx_power();
        Self {
            radio,
            config,
            power,
            // The C driver's PIB default: Energy Detection at -75 dBm - which
            // is also OpenThread's own default threshold.
            cca_threshold: Some(-75),
            csma_max_backoffs: None,
            timed_rx: false,
            window_channel: 0,
            csl: openthread::CslConfig::new(),
            csl_accuracy_ppm: Self::DEFAULT_CSL_ACCURACY_PPM,
            csl_uncertainty: Self::DEFAULT_CSL_UNCERTAINTY,
        }
    }

    /// The default clock accuracy reported to a CSL parent: a 32.768 kHz
    /// crystal (LFXO) driving the SL timer, ±20 PPM. With the internal RC
    /// oscillator (LFRC, ±250 PPM even when calibrated) set the real figure
    /// with [`with_csl_timing`](Self::with_csl_timing), or the parent's windows
    /// will be too narrow and frames will be missed.
    pub const DEFAULT_CSL_ACCURACY_PPM: u8 = 20;

    /// The default timed-receive uncertainty reported to a CSL parent, in units
    /// of 10 µs: the jitter of the driver's receive-window start (SL timer
    /// granularity plus the radio ramp-up), a conservative ±120 µs.
    pub const DEFAULT_CSL_UNCERTAINTY: u8 = 12;

    /// Set the CSL timing figures this radio reports to its parent
    /// (`RadioCaps::csl_accuracy_ppm` / `csl_uncertainty`); see the defaults
    /// for what they mean.
    #[must_use]
    pub const fn with_csl_timing(mut self, accuracy_ppm: u8, uncertainty: u8) -> Self {
        self.csl_accuracy_ppm = accuracy_ppm;
        self.csl_uncertainty = uncertainty;
        self
    }

    fn apply_config(radio: &mut Radio<'_>, config: &openthread::Config) {
        // CCA is per-transmit: OpenThread's threshold arrives with each frame
        // and is applied there (see `transmit`).
        radio.set_promiscuous(config.promiscuous);
        radio.set_pan_id(config.pan_id);
        radio.set_short_addr(config.short_addr);
        // `alt_short_addr` is disregarded: the Nordic driver's hardware filter
        // matches a single short address (see the `Config::alt_short_addr`
        // docs - radios like this one are allowed to ignore it).
        radio.set_ext_addr(config.ext_addr);
        // The trait's polarity is "may the radio power down when idle";
        // the driver's is "keep the receiver on when idle".
        radio.set_rx_when_idle(!config.auto_sleep);
    }
}

impl openthread::Radio for OpenThreadRadio<'_> {
    type Error = Error;

    async fn init(&mut self) -> Result<openthread::RadioCaps, Self::Error> {
        // Fixed, statically-known capabilities of the Nordic SoC radio (no
        // hardware handshake needed, unlike a remote-RCP radio).
        Ok(openthread::RadioCaps {
            phy: openthread::Capabilities::ACK_TIMEOUT
                // Hardware CSMA-CA. Required in practice: OpenThread's software
                // CSMA-CA timing is too disrupted when the radio shares a busy
                // executor (e.g. embassy-net), so the attach exchange fails
                // without it.
                .union(openthread::Capabilities::CSMA_BACKOFF)
                // The driver can keep the receiver on during idle periods (or
                // not - `Config::auto_sleep`, forwarded as `rx_when_idle` in
                // `apply_config`), so OpenThread hands it the standing policy
                // instead of issuing explicit sleep/receive commands around
                // every idle gap - a sleep/re-arm gap would drop frames that
                // arrive asynchronously (routed responses, or Parent Responses
                // when the executor is busy with e.g. embassy-net).
                .union(openthread::Capabilities::AUTO_SLEEP)
                // The transmit power arrives per-transmit and is applied
                // before each frame (see `transmit`).
                .union(openthread::Capabilities::TRANSMIT_FRAME_POWER)
                // Timed receive (`receive_at`), a microsecond radio clock, frame
                // timestamps and enhanced-ACK security with the CSL IE: the
                // driver has it all, so this node can be a CSL (Synchronized)
                // Sleepy End Device.
                .union(openthread::Capabilities::RECEIVE_TIMING)
                // The driver finishes the frames it sends: frame counter and
                // CSL IE written at transmit time (its security and IE
                // writers), AES-CCM* with the keys from `set_mac_keys`.
                .union(openthread::Capabilities::TRANSMIT_SEC)
                // Timed transmit (`PsduTxInfo::tx_at_us`), which a Thread FTD
                // uses to send into the receive windows of its CSL children.
                .union(openthread::Capabilities::TRANSMIT_TIMING),
            // Full MAC offload: auto-ACK, address filtering, ACK handling
            // and the source-match table (the ACKs' pending bit consults the
            // driver's pending-bit lists, see `set_src_match_config`) are all
            // done by the Nordic driver/hardware.
            mac: openthread::MacCapabilities::all(),
            // The Nordic OT platform's receive-sensitivity figure for this
            // radio.
            receive_sensitivity: -100,
            // Whatever the driver came up with (0 dBm unless overridden
            // before wrapping).
            default_tx_power: self.radio.tx_power(),
            // The C driver's PIB default ED threshold (see `apply_config`).
            default_cca_threshold: -75,
            clock: Some(openthread::RadioClock(radio_now_us)),
            csl_accuracy_ppm: self.csl_accuracy_ppm,
            csl_uncertainty: self.csl_uncertainty,
            bus_speed: 0,
        })
    }

    async fn set_config(&mut self, config: &openthread::Config) -> Result<(), Self::Error> {
        if self.config != *config {
            self.config = config.clone();
            Self::apply_config(&mut self.radio, &self.config);
        }

        Ok(())
    }

    async fn set_src_match_config(
        &mut self,
        config: &openthread::SrcMatchConfig,
    ) -> Result<(), Self::Error> {
        // The table is small and changes rarely (children with pending
        // indirect frames), so rebuild it wholesale instead of diffing.
        self.radio.clear_pending();

        for &addr in &config.short_addrs {
            // A full driver list (`NRF_802154_PENDING_SHORT_ADDRESSES`) drops
            // the overflowing child to FP = 0. The `openthread` glue already
            // caps the table at its own capacity by answering `NO_BUFS`, which
            // makes OpenThread fall back to FP-on-every-ACK - so overflow here
            // means the driver list is configured smaller than that cap.
            self.radio.set_pending_short(addr, true);
        }

        for &addr in &config.ext_addrs {
            self.radio.set_pending_ext(addr, true);
        }

        // Disabled matching = pending bit set in every ACK, which is exactly
        // the `SrcMatchConfig::enabled == false` contract.
        self.radio.set_src_match_enabled(config.enabled);

        Ok(())
    }

    async fn set_receive(&mut self, channel: u8) -> Result<(), Self::Error> {
        self.timed_rx = false;

        if self.radio.channel() != channel {
            self.radio.set_channel(channel);
        }

        // Wake the receiver (the counterpart of `set_sleep`); reception
        // itself runs driver-side, into the IRQ-fed RX queue.
        if !self.radio.enter_receive() {
            return Err(Error::EnterReceive);
        }

        Ok(())
    }

    async fn set_sleep(&mut self) -> Result<(), Self::Error> {
        if core::mem::take(&mut self.timed_rx) || self.radio.has_pending_window() {
            // A timed window is pending or just ran: the driver sleeps by
            // itself between and after windows, and a frame may be arriving
            // right now - do not abort it. `sleep_if_idle` refusing (busy) is
            // then fine. OpenThread asks for sleep freely (after every
            // operation), so this is the common case for a CSL child.
            self.radio.sleep_if_idle();

            return Ok(());
        }

        // The receiver goes off, so frames sent to a sleeping node are
        // genuinely missed, as the radio contract requires; frames already
        // in the RX queue were received while awake and remain deliverable.
        if !self.radio.sleep() {
            return Err(Error::EnterSleep);
        }

        Ok(())
    }

    async fn receive_at(
        &mut self,
        channel: u8,
        start_us: u64,
        duration_us: u32,
    ) -> Result<(), Self::Error> {
        // The window carries its own channel, and the driver's channel - what
        // it receives on otherwise, and returns to after transmitting - stays:
        // retuning it here would abort a window still running.
        self.window_channel = channel;

        // The driver refuses a start that is not comfortably ahead of its
        // clock, and OpenThread's request may have aged on its way here. Trim
        // a window whose start has (nearly) passed rather than lose it - the
        // frame is still expected in its remaining part.
        const MIN_LEAD_US: u64 = 250;

        let now = self.radio.now_us();
        let end_us = start_us + duration_us as u64;
        let start_us = start_us.max(now + MIN_LEAD_US);

        if end_us <= start_us + MIN_LEAD_US {
            debug!(
                "Timed receive window missed: it ended {} us before it could be armed",
                (now + MIN_LEAD_US).saturating_sub(end_us)
            );

            return Err(Error::ScheduleReceive);
        }

        if !self
            .radio
            .receive_at(start_us, (end_us - start_us) as u32, channel)
        {
            return Err(Error::ScheduleReceive);
        }

        // Until the next `set_sleep` / `set_receive`, `receive` only drains the
        // RX queue: the receiver is the window's to switch on and off.
        self.timed_rx = true;

        Ok(())
    }

    async fn set_csl(&mut self, csl: &openthread::CslConfig) -> Result<(), Self::Error> {
        let peer_changed =
            self.csl.short_addr != csl.short_addr || self.csl.ext_addr != csl.ext_addr;

        // The parent learns this node's receive schedule from the CSL IE in
        // the enhanced ACKs it gets back, so the IE is injected for exactly
        // the current parent, under both of its addresses.
        if self.csl.enabled() && (!csl.enabled() || peer_changed) {
            self.radio
                .clear_csl_ie_peer(Some(self.csl.short_addr), Some(self.csl.ext_addr));
        }

        if !csl.enabled() {
            // No more windows to keep: whatever is scheduled is stale. A window
            // already running is not ended by the cancel - the radio stays in
            // the receive state - and OpenThread does not ask for sleep after
            // CSL either (a timed-receive radio sleeps by itself between its
            // windows), so put it to sleep here.
            let in_window = self.timed_rx;

            self.radio.receive_at_cancel();

            if in_window {
                self.radio.sleep_if_idle();
            }
        }

        if self.csl.period != csl.period {
            // OpenThread's period is in the same 10-symbol units; it caps it
            // at `u16::MAX` itself.
            self.radio
                .set_csl_period(csl.period.min(u16::MAX as u32) as u16);
        }

        if csl.enabled()
            && (!self.csl.enabled() || peer_changed)
            && !self
                .radio
                .set_csl_ie_peer(Some(csl.short_addr), Some(csl.ext_addr))
        {
            warn!("CSL IE not injected into enhanced ACKs: the driver's ACK data table is full");
        }

        if csl.enabled() && (self.csl.sample_time_us != csl.sample_time_us || !self.csl.enabled()) {
            // OpenThread hands out the low 32 bits of the radio clock; widen it
            // against the current time. Its sample time is the expected time
            // of the first symbol of the frame's MHR, which is exactly the
            // driver's definition of the anchor (the time of CSL phase zero).
            let now = self.radio.now_us();
            let offset = csl.sample_time_us.wrapping_sub(now as u32);
            let sample_time = if offset < 0x80000000 {
                now + offset as u64
            } else {
                now.saturating_sub((u32::MAX - offset + 1) as u64)
            };

            self.radio.set_csl_anchor_time(sample_time);
        }

        self.csl = *csl;

        Ok(())
    }

    async fn set_mac_keys(
        &mut self,
        keys: Option<&openthread::MacKeys>,
    ) -> Result<(), Self::Error> {
        match keys {
            Some(keys) => self.radio.set_mac_keys(
                keys.key_id_mode,
                &[
                    MacKey {
                        key_id: keys.prev_key_id(),
                        key: keys.prev,
                    },
                    MacKey {
                        key_id: keys.key_id,
                        key: keys.curr,
                    },
                    MacKey {
                        key_id: keys.next_key_id(),
                        key: keys.next,
                    },
                ],
            ),
            None => {
                self.radio.clear_mac_keys();

                Ok(())
            }
        }
    }

    async fn set_mac_frame_counter(
        &mut self,
        frame_counter: u32,
        if_larger: bool,
    ) -> Result<(), Self::Error> {
        self.radio.set_frame_counter(frame_counter, if_larger);

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &mut [u8],
        psdu_tx: &mut openthread::PsduTxInfo,
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        mut ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<openthread::PsduRxInfo>, Self::Error> {
        if psdu.len() > MAX_PSDU_SIZE + 2
        /* + FCS */
        {
            return Err(Error::TransmitDataTooLarge);
        }

        if let Some(ack_psdu_buf) = ack_psdu_buf.as_ref() {
            if ack_psdu_buf.len() < MAX_PSDU_SIZE + 2
            /* + FCS */
            {
                return Err(Error::ReceiveBufTooSmall);
            }
        }

        // The frame goes out on its own channel; the driver's channel - which it
        // receives on, and returns to afterwards - stays. Retuning it would
        // abort a timed receive window (on a CSL channel) still running, and
        // leave a CSL parent's receiver on its child's channel.
        if self.power != power {
            self.power = power;
            self.radio.set_tx_power(power);
        }

        let len = psdu.len();
        let data = &mut psdu[..len - 2];
        let ack = ack_psdu_buf.as_mut().map(|ack_psdu_buf| {
            let len = ack_psdu_buf.len();
            &mut ack_psdu_buf[..len - 2]
        });

        // What the driver still has to do to the frame before it goes out
        // (`TRANSMIT_SEC`): assign the frame counter and fill the CSL IE unless
        // the header is final already (a retransmission), and secure it unless
        // it is secured already. The driver does that in place, in its own
        // buffer, and the finished frame is copied back below.
        let props = crate::FrameProps {
            is_secured: psdu_tx.security_processed,
            dynamic_data_is_set: psdu_tx.header_updated,
        };
        let finishes_header = !psdu_tx.header_updated;

        // CCA, when requested (a `Some` threshold), is Energy Detection at the
        // requested dBm threshold.
        if let Some(threshold) = cca_threshold {
            if self.cca_threshold != Some(threshold) {
                self.cca_threshold = Some(threshold);
                self.radio.set_cca(crate::Cca::ed_from_dbm(threshold));
            }
        }

        let cca = cca_threshold.is_some();

        // A timed frame, if it can still make its time: its SHR starts 5
        // octets (10 symbols) before the end of the SFD OpenThread times it
        // by, and the driver wants that comfortably ahead of its clock to fit
        // the CCA and the ramp-up in. A frame that is too late goes out right
        // away, as OpenThread's own timing would send it - the window it is
        // aimed at may still be open.
        const SHR_US: u64 = 10 * 16;
        const MIN_LEAD_US: u64 = 400;

        let tx_start_us = psdu_tx
            .tx_at_us
            .map(|tx_at_us| tx_at_us.saturating_sub(SHR_US))
            .filter(|start_us| *start_us >= self.radio.now_us() + MIN_LEAD_US);

        let meta = if let Some(start_us) = tx_start_us {
            Radio::transmit_at_with(&mut self.radio, data, props, start_us, channel, cca, ack)
                .await?
        } else if cca
            && psdu_tx.tx_at_us.is_none()
            && psdu_tx
                .max_csma_backoffs
                .is_none_or(|backoffs| backoffs > 0)
        {
            // We advertise `Capabilities::CSMA_BACKOFF`, so OpenThread expects
            // the radio to perform CSMA-CA channel access itself, with as many
            // backoffs as it asks for.
            if let Some(backoffs) = psdu_tx.max_csma_backoffs {
                if self.csma_max_backoffs != Some(backoffs) {
                    self.csma_max_backoffs = Some(backoffs);
                    self.radio.set_csma_ca_max_backoffs(backoffs);
                }
            }

            Radio::transmit_csma_ca_with(&mut self.radio, data, props, channel, ack).await?
        } else {
            // No CCA, or a single one and no backoff (a late timed frame, or
            // OpenThread asking for no backoffs).
            Radio::transmit_with(&mut self.radio, data, props, channel, cca, ack).await?
        };

        // The driver hands back the frame as it went on the air - with its
        // header finished, and secured.
        if finishes_header {
            psdu_tx.header_updated = true;
        }
        psdu_tx.security_processed = true;

        Ok(if let Some(meta) = meta {
            if let Some(ack_psdu_buf) = ack_psdu_buf {
                meta.write_crc(ack_psdu_buf);
            }

            Some(meta.as_openthread(channel))
        } else {
            None
        })
    }

    async fn receive(
        &mut self,
        psdu_buf: &mut [u8],
    ) -> Result<openthread::PsduRxInfo, Self::Error> {
        if psdu_buf.len() < MAX_PSDU_SIZE + 2
        /* + FCS */
        {
            return Err(Error::ReceiveBufTooSmall);
        }

        let len = psdu_buf.len();
        let meta = if self.timed_rx {
            // Inside a timed window (`receive_at`): just wait for the window's
            // frames, the receiver is the driver's to run - and to put to sleep
            // when the window ends.
            Radio::wait_window_frame(&mut self.radio, &mut psdu_buf[..len - 2]).await
        } else {
            Radio::receive(&mut self.radio, &mut psdu_buf[..len - 2]).await?
        };

        meta.write_crc(psdu_buf);

        let channel = if self.timed_rx {
            self.window_channel
        } else {
            self.radio.channel()
        };

        Ok(meta.as_openthread(channel))
    }
}
