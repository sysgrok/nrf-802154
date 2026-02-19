use crate::{Cca, Error, PsduMeta, Radio, MAX_PSDU_SIZE};

impl openthread::RadioError for Error {
    fn kind(&self) -> openthread::RadioErrorKind {
        match self {
            Error::TransmitDataTooLarge | Error::ReceiveBufTooSmall => openthread::RadioErrorKind::Other,
            Error::ScheduleTransmit | Error::Transmit => openthread::RadioErrorKind::TxFailed,
            Error::EnterReceive | Error::Receive => openthread::RadioErrorKind::RxFailed,
        }
    }
}

impl Cca {
    fn from_openthread(cca: &openthread::Cca) -> Self {
        match cca {
            openthread::Cca::Carrier => Cca::Carrier,
            openthread::Cca::Ed { ed_threshold } => Cca::Ed {
                ed_threshold: *ed_threshold,
            },
            openthread::Cca::CarrierOrEd { ed_threshold } => Cca::CarrierOrEd {
                ed_threshold: *ed_threshold,
            },
            openthread::Cca::CarrierAndEd { ed_threshold } => Cca::CarrierAndEd {
                ed_threshold: *ed_threshold,
            },
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

impl openthread::Radio for Radio<'_> {
    type Error = Error;

    const CAPS: openthread::Capabilities = openthread::Capabilities::empty(); // TODO

    const MAC_CAPS: openthread::MacCapabilities = openthread::MacCapabilities::TX_ACK
        .union(openthread::MacCapabilities::RX_ACK)
        .union(openthread::MacCapabilities::FILTER_PAN_ID)
        .union(openthread::MacCapabilities::FILTER_SHORT_ADDR)
        .union(openthread::MacCapabilities::FILTER_EXT_ADDR);

    async fn set_config(&mut self, config: &openthread::Config) -> Result<(), Self::Error> {
        self.set_channel(config.channel);
        self.set_tx_power(config.power);
        self.set_cca(Cca::from_openthread(&config.cca));
        self.set_pan_id(config.pan_id);
        self.set_short_addr(config.short_addr);
        self.set_ext_addr(config.ext_addr);

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
            self,
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

            Some(meta.as_openthread(self.channel()))
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
        let meta = Radio::receive(self, &mut psdu_buf[..len - 2]).await?;

        meta.write_crc(psdu_buf);

        Ok(meta.as_openthread(self.channel()))
    }
}
