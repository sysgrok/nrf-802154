use crate::{Error, PsduMeta, Radio, TxError, MAX_PSDU_SIZE};

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
            Error::EnterReceive | Error::Receive => RadioErrorKind::RxFailed,
            Error::EnterSleep => RadioErrorKind::Other,
        }
    }
}

impl PsduMeta {
    fn as_openthread(&self, channel: u8) -> openthread::PsduMeta {
        openthread::PsduMeta {
            len: self.len as usize + 2,
            channel,
            rssi: Some(self.power),
            lqi: self.lqi,
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
        }
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
                .union(openthread::Capabilities::TRANSMIT_FRAME_POWER),
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
        // The receiver goes off, so frames sent to a sleeping node are
        // genuinely missed, as the radio contract requires; frames already
        // in the RX queue were received while awake and remain deliverable.
        if !self.radio.sleep() {
            return Err(Error::EnterSleep);
        }

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        channel: u8,
        power: i8,
        cca_threshold: Option<i8>,
        mut ack_psdu_buf: Option<&mut [u8]>,
    ) -> Result<Option<openthread::PsduMeta>, Self::Error> {
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

        if self.radio.channel() != channel {
            self.radio.set_channel(channel);
        }
        if self.power != power {
            self.power = power;
            self.radio.set_tx_power(power);
        }

        let data = &psdu[..psdu.len() - 2];
        let ack = ack_psdu_buf.as_mut().map(|ack_psdu_buf| {
            let len = ack_psdu_buf.len();
            &mut ack_psdu_buf[..len - 2]
        });

        // We advertise `Capabilities::CSMA_BACKOFF`, so OpenThread expects the
        // radio to perform CSMA-CA channel access itself when it requests it
        // (a `Some` threshold). Route to the driver's CSMA-CA transmit in
        // that case - as Energy Detection at the requested dBm threshold -
        // and transmit immediately without CCA otherwise.
        let meta = if let Some(threshold) = cca_threshold {
            if self.cca_threshold != Some(threshold) {
                self.cca_threshold = Some(threshold);
                self.radio.set_cca(crate::Cca::ed_from_dbm(threshold));
            }

            Radio::transmit_csma_ca(&mut self.radio, data, ack).await?
        } else {
            Radio::transmit(&mut self.radio, data, false, ack).await?
        };

        Ok(if let Some(meta) = meta {
            if let Some(ack_psdu_buf) = ack_psdu_buf {
                meta.write_crc(ack_psdu_buf);
            }

            Some(meta.as_openthread(self.radio.channel()))
        } else {
            None
        })
    }

    async fn receive(&mut self, psdu_buf: &mut [u8]) -> Result<openthread::PsduMeta, Self::Error> {
        if psdu_buf.len() < MAX_PSDU_SIZE + 2
        /* + FCS */
        {
            return Err(Error::ReceiveBufTooSmall);
        }

        let len = psdu_buf.len();
        let meta = Radio::receive(&mut self.radio, &mut psdu_buf[..len - 2]).await?;

        meta.write_crc(psdu_buf);

        Ok(meta.as_openthread(self.radio.channel()))
    }
}
