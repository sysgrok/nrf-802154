use core::cell::RefCell;

use embassy_nrf::radio::{Instance, InterruptHandler};
use embassy_nrf::{interrupt, Peri};
use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::raw;

/// Maximum PSDU size, in bytes, excluding the PHY header (PHR) and the CRC/FCS
pub const MAX_PSDU_SIZE: usize = MAX_PACKET_SIZE - 2/*CRC*/ - 1/*PHR*/;

const MAX_PACKET_SIZE: usize = 128;

/// Radio error
// TODO: Extend the error codes with additional information
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// The data to transmit is too large
    TransmitDataTooLarge,
    /// The buffer provided to receive a frame is too small
    ReceiveBufTooSmall,
    /// Could not schedule the transmission (radio busy, etc)
    ScheduleTransmit,
    /// Could not enter receive mode
    EnterReceive,
    /// Transmission failed (no ACK received, etc)
    Transmit,
    /// Reception failed (CRC error, aborted, etc)
    Receive,
}

/// Clear Channel Assessment method
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Cca {
    /// Carrier sense
    #[default]
    Carrier,
    /// Energy Detection / Energy Above Threshold
    Ed {
        /// Energy measurements above this value mean that the channel is assumed to be busy.
        /// Note the measurement range is 0..0xFF - where 0 means that the received power was
        /// less than 10 dB above the selected receiver sensitivity. This value is not given in dBm,
        /// but can be converted. See the nrf52840 Product Specification Section 6.20.12.4
        /// for details.
        ed_threshold: u8,
    },
    /// Carrier sense or Energy Detection
    CarrierOrEd {
        /// Energy measurements above this value mean that the channel is assumed to be busy.
        /// Note the measurement range is 0..0xFF - where 0 means that the received power was
        /// less than 10 dB above the selected receiver sensitivity. This value is not given in dBm,
        /// but can be converted. See the nrf52840 Product Specification Section 6.20.12.4
        /// for details.
        ed_threshold: u8,
    },
    /// Carrier sense and Energy Detection
    CarrierAndEd {
        /// Energy measurements above this value mean that the channel is assumed to be busy.
        /// Note the measurement range is 0..0xFF - where 0 means that the received power was
        /// less than 10 dB above the selected receiver sensitivity. This value is not given in dBm,
        /// but can be converted. See the nrf52840 Product Specification Section 6.20.12.4
        /// for details.
        ed_threshold: u8,
    },
}

/// Details of a received frame
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct PsduMeta {
    /// Length of the received PSDU (PHY service data unit) in bytes, excluding the PHY header (PHR) and the CRC
    pub len: u8,
    /// CRC of the received frame
    pub crc: u16,
    /// Received signal power in dBm
    pub power: i8,
    /// Link Quality Indicator of the received frame
    pub lqi: Option<u8>,
    /// Timestamp taken when the last symbol of the frame was received
    pub time: Option<u64>,
}

/// IEEE 802.15.4 radio driver.
pub struct Radio<'d, T: Instance> {
    _p: Peri<'d, T>,
}

impl<'d, T: Instance> Radio<'d, T> {
    /// Create a new IEEE 802.15.4 radio driver.
    pub fn new(
        radio: Peri<'d, T>,
        _irq: impl interrupt::typelevel::Binding<T::Interrupt, InterruptHandler<T>> + 'd,
    ) -> Self {
        unsafe {
            raw::nrf_802154_init();
        }

        // // Enable NVIC interrupt
        // T::Interrupt::unpend();
        // unsafe { T::Interrupt::enable() };

        unsafe {
            raw::nrf_802154_channel_set(11);
            raw::nrf_802154_tx_power_set(0);
            raw::nrf_802154_cca_cfg_set(&raw::nrf_802154_cca_cfg_t {
                mode: raw::NRF_RADIO_CCA_MODE_CARRIER,
                ed_threshold: 0,
                corr_threshold: 0,
                corr_limit: 0,
            });
        }

        Self { _p: radio }
    }

    /// Get the current radio channel
    pub fn channel(&self) -> u8 {
        unsafe { raw::nrf_802154_channel_get() }
    }

    /// Change the radio channel
    pub fn set_channel(&mut self, channel: u8) {
        if !(11..=26).contains(&channel) {
            panic!("Bad 802.15.4 channel");
        }
        unsafe {
            raw::nrf_802154_channel_set(channel);
        }
    }

    /// Get the current Clear Channel Assessment method
    pub fn cca(&self) -> Cca {
        let mut cfg = raw::nrf_802154_cca_cfg_t {
            mode: 0,
            ed_threshold: 0,
            corr_threshold: 0,
            corr_limit: 0,
        };

        unsafe {
            raw::nrf_802154_cca_cfg_get(&mut cfg);
        }

        match cfg.mode {
            raw::NRF_RADIO_CCA_MODE_CARRIER => Cca::Carrier,
            raw::NRF_RADIO_CCA_MODE_ED => Cca::Ed {
                ed_threshold: cfg.ed_threshold,
            },
            raw::NRF_RADIO_CCA_MODE_CARRIER_OR_ED => Cca::CarrierOrEd {
                ed_threshold: cfg.ed_threshold,
            },
            raw::NRF_RADIO_CCA_MODE_CARRIER_AND_ED => Cca::CarrierAndEd {
                ed_threshold: cfg.ed_threshold,
            },
            _ => unreachable!(),
        }
    }

    /// Change the Clear Channel Assessment method
    pub fn set_cca(&mut self, cca: Cca) {
        let (mode, ed_threshold) = match cca {
            Cca::Carrier => (raw::NRF_RADIO_CCA_MODE_CARRIER, 0),
            Cca::Ed { ed_threshold } => (raw::NRF_RADIO_CCA_MODE_ED, ed_threshold),
            Cca::CarrierOrEd { ed_threshold } => (raw::NRF_RADIO_CCA_MODE_CARRIER_OR_ED, ed_threshold),
            Cca::CarrierAndEd { ed_threshold } => (raw::NRF_RADIO_CCA_MODE_CARRIER_AND_ED, ed_threshold),
        };

        unsafe {
            raw::nrf_802154_cca_cfg_set(&raw::nrf_802154_cca_cfg_t {
                mode,
                ed_threshold,
                corr_threshold: 0,
                corr_limit: 0,
            });
        }
    }

    /// Get the current radio transmission power
    pub fn tx_power(&self) -> i8 {
        unsafe { raw::nrf_802154_tx_power_get() }
    }

    /// Change the radio transmission power
    pub fn set_tx_power(&mut self, power: i8) {
        unsafe {
            raw::nrf_802154_tx_power_set(power);
        }
    }

    /// Set the PAN ID of the device
    ///
    /// # Arguments
    /// - `pan_id`: The PAN ID to set. If `None`, the PAN ID filtering is disabled.
    pub fn set_pan_id(&mut self, pan_id: Option<u16>) {
        unsafe {
            if let Some(pan_id) = pan_id {
                raw::nrf_802154_pan_id_set(pan_id.to_le_bytes().as_slice().as_ptr());
            } else {
                raw::nrf_802154_pan_id_set(core::ptr::null());
            }
        }
    }

    /// Set the short address of the device
    ///
    /// # Arguments
    /// - `addr_id`: The short address to set. If `None`, the short address filtering is disabled.
    pub fn set_short_addr(&mut self, addr_id: Option<u16>) {
        unsafe {
            if let Some(addr_id) = addr_id {
                raw::nrf_802154_short_address_set(addr_id.to_be_bytes().as_slice().as_ptr());
            } else {
                raw::nrf_802154_short_address_set(core::ptr::null());
            }
        }
    }

    /// Set the extended address of the device
    ///
    /// # Arguments
    /// - `ext_addr_id`: The extended address to set. If `None`, the extended address filtering is disabled.
    pub fn set_ext_addr(&mut self, ext_addr_id: Option<u64>) {
        unsafe {
            if let Some(ext_addr_id) = ext_addr_id {
                raw::nrf_802154_extended_address_set(ext_addr_id.to_be_bytes().as_slice().as_ptr());
            } else {
                raw::nrf_802154_extended_address_set(core::ptr::null());
            }
        }
    }

    /// Move the radio from any state to the DISABLED state
    fn disable(&mut self) {
        // TODO: Is this even supported in the C driver?
    }

    /// Receive one radio packet
    ///
    /// # Arguments
    /// - `buf`: A buffer where the received PSDU data will be copied to (excluding PHY fields like PHR and CRC/FCS).
    ///   The buffer must be at least `MAX_PSDU_SIZE` bytes long.
    ///
    /// # Returns
    /// - `Ok(PsduMeta)` if a packet was successfully received
    /// - `Err(Error::EnterReceive)` if the radio could not enter receive mode
    /// - `Err(Error::Receive)` if the reception failed (CRC error, aborted, etc)
    pub async fn receive(&mut self, buf: &mut [u8]) -> Result<PsduMeta, Error> {
        let receive_entered = unsafe { raw::nrf_802154_receive() };

        if !receive_entered {
            return Err(Error::EnterReceive);
        }

        let status = RadioState::wait(|state| {
            if let RadioStatus::ReceiveDone(psdu_meta) = state.status {
                buf.copy_from_slice(&state.rx[1..][..psdu_meta.len as _]);
            }

            matches!(
                state.status,
                RadioStatus::ReceiveFailed(_) | RadioStatus::ReceiveDone { .. }
            )
            .then_some(state.status)
        })
        .await;

        if let RadioStatus::ReceiveDone(details) = status {
            Ok(details)
        } else {
            Err(Error::Receive)
        }
    }

    /// Transmit one radio packet
    ///
    /// # Arguments
    /// - `data`: The PSDU data to transmit; this data should not contain PHY fields like PHR and CRC/FCS.
    ///   The data must be at most `MAX_PSDU_SIZE` bytes long.
    /// - `cca`: If `true`, perform Clear Channel Assessment (CCA) before transmission.
    /// - `ack_buf`: If the radio is configured to wait for ACK frame in response to its transmission,
    ///   this buffer will be filled with the PSDU data of the received ACK frame.
    ///   In either case, `None` can be passed if the user is not interested in the ACK frame.
    ///
    /// # Returns
    /// - `Ok(Some(PsduMeta))` if the packet was successfully transmitted and the radio is configured
    ///   to wait for an ACK frame, which was received.
    /// - `Ok(None)` if the packet was successfully transmitted and the radio is not configured
    ///   to wait for an ACK frame.
    /// - `Err(Error::ScheduleTransmit)` if the transmission could not be scheduled (radio busy, etc)
    /// - `Err(Error::Transmit)` if the transmission failed (no ACK received, etc)
    pub async fn transmit(
        &mut self,
        data: &[u8],
        cca: bool,
        mut ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        if let Some(ack_buf) = ack_buf.as_ref() {
            if ack_buf.len() < MAX_PSDU_SIZE {
                return Err(Error::ReceiveBufTooSmall);
            }
        }

        // TODO: Potential race condition if the radio is scheduled to transmit but not transmitting yet
        let packet_data = RadioState::wait(|state| {
            if !matches!(state.status, RadioStatus::Transmitting) {
                state.tx[0] = data.len() as u8 + 1 + 2; // + PHR + CRC
                state.tx[1..][..data.len()].copy_from_slice(data);
                state.tx[1 + data.len()] = 0; // CRC placeholder
                state.tx[1 + data.len() + 1] = 0; // CRC placeholder

                let packet_data: &mut [u8] = &mut state.tx;

                Some(packet_data.as_mut_ptr())
            } else {
                None
            }
        })
        .await;

        let scheduled = unsafe {
            raw::nrf_802154_transmit_raw(
                packet_data,
                &raw::nrf_802154_transmit_metadata_t {
                    frame_props: raw::nrf_802154_transmitted_frame_props_t {
                        is_secured: false,
                        dynamic_data_is_set: false,
                    },
                    cca,
                    tx_power: raw::nrf_802154_tx_power_metadata_t {
                        use_metadata_value: false,
                        power: 0,
                    },
                    tx_channel: raw::nrf_802154_tx_channel_metadata_t {
                        use_metadata_value: false,
                        channel: 0,
                    },
                },
            )
        };

        if !scheduled {
            return Err(Error::ScheduleTransmit);
        }

        let status = RadioState::wait(|state| {
            if matches!(state.status, RadioStatus::TransmitDone(_)) {
                if let Some(ack_buf) = ack_buf.as_mut() {
                    if let RadioStatus::TransmitDone(Some(meta)) = state.status {
                        ack_buf.copy_from_slice(&state.rx[1..][..meta.len as _]);
                    }
                }
            }

            matches!(
                state.status,
                RadioStatus::TransmitFailed(_) | RadioStatus::TransmitDone(_)
            )
            .then_some(state.status)
        })
        .await;

        if let RadioStatus::TransmitDone(psdu_meta) = status {
            Ok(psdu_meta)
        } else {
            Err(Error::Transmit)
        }
    }
}

impl<T> Drop for Radio<'_, T>
where
    T: Instance,
{
    fn drop(&mut self) {
        self.disable();

        unsafe {
            raw::nrf_802154_deinit();
        }

        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = RadioStatus::Idle;
        });
    }
}

// TODO: Think if we need `nrf_802154_state_t`
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum RadioStatus {
    Idle,
    ReceiveFailed(raw::nrf_802154_tx_error_t),
    ReceiveDone(PsduMeta),
    CcaFailed(raw::nrf_802154_cca_error_t),
    CcaDone(bool),
    EnergyDetectionDetected(i8),
    EnergyDetectionFailed(raw::nrf_802154_ed_error_t),
    Transmitting,
    TxAckStarted,
    TransmitFailed(raw::nrf_802154_tx_error_t),
    TransmitDone(Option<PsduMeta>),
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RadioState {
    status: RadioStatus,
    tx: [u8; MAX_PACKET_SIZE],
    rx: [u8; MAX_PACKET_SIZE],
}

impl RadioState {
    const fn new() -> Self {
        Self {
            status: RadioStatus::Idle,
            tx: [0; MAX_PACKET_SIZE],
            rx: [0; MAX_PACKET_SIZE],
        }
    }

    async fn wait<F, R>(mut f: F) -> R
    where
        F: FnMut(&mut RadioState) -> Option<R>,
    {
        loop {
            if let Some(result) = STATE.lock(|state| f(&mut state.borrow_mut())) {
                break result;
            }

            STATE_SIGNAL.wait().await;
        }
    }

    fn update<F, R>(f: F)
    where
        F: FnOnce(&mut RadioState) -> R,
    {
        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            f(&mut state);

            STATE_SIGNAL.signal(());
        });
    }
}

static STATE: blocking_mutex::Mutex<CriticalSectionRawMutex, RefCell<RadioState>> =
    blocking_mutex::Mutex::new(RefCell::new(RadioState::new()));

static STATE_SIGNAL: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[no_mangle]
unsafe extern "C" fn nrf_802154_cca_done(channel_free: bool) {
    RadioState::update(|state| state.status = RadioStatus::CcaDone(channel_free));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_cca_failed(error: raw::nrf_802154_cca_error_t) {
    RadioState::update(|state| state.status = RadioStatus::CcaFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_energy_detected(p_result: *const raw::nrf_802154_energy_detected_t) {
    RadioState::update(|state| {
        state.status = RadioStatus::EnergyDetectionDetected(unsafe { p_result.as_ref().unwrap().ed_dbm })
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_energy_detection_failed(error: raw::nrf_802154_ed_error_t) {
    RadioState::update(|state| state.status = RadioStatus::EnergyDetectionFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_tx_ack_started() {
    RadioState::update(|state| state.status = RadioStatus::TxAckStarted);
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_received_raw(p_data: *mut u8, power: i8, len: u8) {
    RadioState::update(|state| {
        state
            .rx
            .copy_from_slice(core::slice::from_raw_parts(p_data, len as usize));

        state.status = RadioStatus::ReceiveDone(PsduMeta {
            len: len - 1 - 2, // - PHR - CRC
            crc: u16::from_le_bytes([state.rx[len as usize - 2], state.rx[len as usize - 1]]),
            power,
            lqi: None,
            time: None,
        });

        unsafe {
            raw::nrf_802154_buffer_free_raw(p_data);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_received_timestamp_raw(p_data: *mut u8, power: i8, len: u8, time: u64) {
    RadioState::update(|state| {
        state
            .rx
            .copy_from_slice(core::slice::from_raw_parts(p_data, len as usize));

        state.status = RadioStatus::ReceiveDone(PsduMeta {
            len: len - 1 - 2, // - PHR - CRC
            crc: u16::from_le_bytes([state.rx[len as usize - 2], state.rx[len as usize - 1]]),
            power,
            lqi: None,
            time: Some(time),
        });

        unsafe {
            raw::nrf_802154_buffer_free_raw(p_data);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_receive_failed(error: raw::nrf_802154_rx_error_t, _id: u32) {
    RadioState::update(|state| state.status = RadioStatus::ReceiveFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmitted_raw(
    _p_frame: *mut u8,
    p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    RadioState::update(|state| {
        let p_metadata = unsafe { p_metadata.as_ref().unwrap() };
        if !p_metadata.data.transmitted.p_ack.is_null() {
            let len = p_metadata.data.transmitted.length;

            let packet = unsafe { core::slice::from_raw_parts(p_metadata.data.transmitted.p_ack, len as usize) };

            state.rx.copy_from_slice(packet);

            state.status = RadioStatus::TransmitDone(Some(PsduMeta {
                len: len - 1 - 2, // - PHR - CRC
                crc: u16::from_le_bytes([state.rx[len as usize - 2], state.rx[len as usize - 1]]),
                power: p_metadata.data.transmitted.power,
                lqi: Some(p_metadata.data.transmitted.lqi),
                time: Some(p_metadata.data.transmitted.time),
            }));

            unsafe {
                raw::nrf_802154_buffer_free_raw(p_metadata.data.transmitted.p_ack);
            }
        } else {
            state.status = RadioStatus::TransmitDone(None);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmit_failed(
    _p_frame: *mut u8,
    error: raw::nrf_802154_tx_error_t,
    _p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    RadioState::update(|state| state.status = RadioStatus::TransmitFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_tx_started(_p_frame: *const u8) {
    RadioState::update(|state| state.status = RadioStatus::Transmitting);
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_custom_part_of_radio_init() {}
