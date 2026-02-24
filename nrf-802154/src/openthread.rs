use crate::{Error, PsduMeta, Radio, MAX_PSDU_SIZE};

impl openthread::RadioError for Error {
    fn kind(&self) -> openthread::RadioErrorKind {
        match self {
            Error::TransmitDataTooLarge | Error::ReceiveBufTooSmall => {
                openthread::RadioErrorKind::Other
            }
            Error::ScheduleTransmit | Error::Transmit(_) => openthread::RadioErrorKind::TxFailed,
            Error::EnterReceive | Error::Receive => openthread::RadioErrorKind::RxFailed,
        }
    }
}

impl PsduMeta {
    fn as_openthread(&self, channel: u8) -> openthread::PsduMeta {
        openthread::PsduMeta {
            len: self.len as usize + 2,
            channel,
            rssi: Some(self.power),
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
/// OpenThread calls `set_config()` before each TX/RX cycle. Without caching,
/// every call would go directly to the Nordic 802.15.4 C driver, which can
/// disrupt in-progress radio operations. This wrapper caches the last applied
/// config and only forwards changes to the driver when the config actually differs.
///
/// This matches the caching pattern used by the `NrfRadio` and `EspRadio` wrappers
/// in the `openthread` crate.
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
}

impl<'d> OpenThreadRadio<'d> {
    /// Create a new `OpenThreadRadio` wrapper around the given radio.
    ///
    /// The initial config is applied to the driver immediately.
    pub fn new(mut radio: Radio<'d>) -> Self {
        let config = openthread::Config::new();
        Self::apply_config(&mut radio, &config);
        Self { radio, config }
    }

    fn apply_config(radio: &mut Radio<'_>, config: &openthread::Config) {
        radio.set_channel(config.channel);
        radio.set_tx_power(config.power);
        // CCA is intentionally NOT forwarded from the openthread Config.
        // The openthread crate defaults to Cca::Carrier (correlator-based carrier sense),
        // which produces false CCABUSY results on nRF52840. The Nordic 802.15.4 C driver's
        // PIB defaults (Energy Detection at -75 dBm, set during nrf_802154_pib_init) are
        // well-tested and IEEE 802.15.4 compliant. Users can still override CCA via
        // Radio::set_cca() before wrapping with OpenThreadRadio if needed.
        radio.set_promiscuous(config.promiscuous);
        radio.set_pan_id(config.pan_id);
        radio.set_short_addr(config.short_addr);
        radio.set_ext_addr(config.ext_addr);
    }
}

impl openthread::Radio for OpenThreadRadio<'_> {
    type Error = Error;

    const CAPS: openthread::Capabilities = openthread::Capabilities::ACK_TIMEOUT;

    const MAC_CAPS: openthread::MacCapabilities = openthread::MacCapabilities::TX_ACK
        .union(openthread::MacCapabilities::RX_ACK)
        .union(openthread::MacCapabilities::FILTER_PAN_ID)
        .union(openthread::MacCapabilities::FILTER_SHORT_ADDR)
        .union(openthread::MacCapabilities::FILTER_EXT_ADDR);

    async fn set_config(&mut self, config: &openthread::Config) -> Result<(), Self::Error> {
        if self.config != *config {
            self.config = config.clone();
            Self::apply_config(&mut self.radio, &self.config);
        }

        Ok(())
    }

    async fn transmit(
        &mut self,
        psdu: &[u8],
        cca: bool,
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

        let meta = Radio::transmit(
            &mut self.radio,
            &psdu[..psdu.len() - 2],
            cca,
            ack_psdu_buf.as_mut().map(|ack_psdu_buf| {
                let len = ack_psdu_buf.len();
                &mut ack_psdu_buf[..len - 2]
            }),
        )
        .await?;

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
