use core::cell::RefCell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_nrf::interrupt::typelevel::Binding;
use embassy_nrf::radio::Instance;
use embassy_nrf::Peri;
use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::platform::{Egu0InterruptHandler, Egu0Irq, LpTimerInterruptHandler, LpTimerIrq};
use crate::raw;

/// Maximum PSDU size, in bytes, excluding the PHY header (PHR) and the CRC/FCS
pub const MAX_PSDU_SIZE: usize = MAX_PACKET_SIZE - 2/*CRC*/ - 1/*PHR*/;

const MAX_PACKET_SIZE: usize = 128;

/// Minimum valid PHR value (1 byte PSDU + 2 bytes FCS)
const MIN_PHR: u8 = 3;

/// Nordic default correlator threshold for CCA carrier-sense modes
const CCA_CORR_THRESHOLD_DEFAULT: u8 = 0x14;

/// Nordic default correlator limit for CCA carrier-sense modes
const CCA_CORR_LIMIT_DEFAULT: u8 = 0x02;

/// Maximum number of retries when `nrf_802154_transmit_raw()` returns false
/// because the C driver is busy. Each retry yields to let ISRs complete
/// the in-progress operation (PSDU reception, TX_ACK, etc.). On Cortex-M,
/// the yield returns to the executor which checks for pending interrupts
/// before re-polling this task.
const TRANSMIT_SCHEDULE_RETRIES: usize = 10;

/// Transmit error reason
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum TxError {
    /// CCA reported busy channel before the transmission
    BusyChannel,
    /// Received ACK frame is other than expected
    InvalidAck,
    /// No receive buffer is available to receive an ACK
    NoMem,
    /// Radio timeslot ended during the transmission procedure
    TimeslotEnded,
    /// ACK frame was not received during the timeout period
    NoAck,
    /// Procedure was aborted by another operation
    Aborted,
    /// Transmission did not start due to a denied timeslot request
    TimeslotDenied,
    /// Unknown error code from the C driver
    Unknown(u8),
}

impl From<raw::nrf_802154_tx_error_t> for TxError {
    fn from(e: raw::nrf_802154_tx_error_t) -> Self {
        match e as u32 {
            raw::NRF_802154_TX_ERROR_BUSY_CHANNEL => TxError::BusyChannel,
            raw::NRF_802154_TX_ERROR_INVALID_ACK => TxError::InvalidAck,
            raw::NRF_802154_TX_ERROR_NO_MEM => TxError::NoMem,
            raw::NRF_802154_TX_ERROR_TIMESLOT_ENDED => TxError::TimeslotEnded,
            raw::NRF_802154_TX_ERROR_NO_ACK => TxError::NoAck,
            raw::NRF_802154_TX_ERROR_ABORTED => TxError::Aborted,
            raw::NRF_802154_TX_ERROR_TIMESLOT_DENIED => TxError::TimeslotDenied,
            _ => TxError::Unknown(e),
        }
    }
}

/// Radio error
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
    /// Transmission failed
    Transmit(TxError),
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
pub struct Radio<'d> {
    _p: PhantomData<&'d mut ()>,
}

impl<'d> Radio<'d> {
    /// Create a new IEEE 802.15.4 radio driver.
    ///
    /// # Peripherals
    ///
    /// In addition to the RADIO peripheral and the EGU0/MPSL reference, this constructor
    /// takes ownership of the timer and RTC peripherals used by the 802.15.4 platform layer:
    /// - `TIMER2` — used as the high-precision (1 µs) timer
    /// - Low-power RTC used internally by the Nordic 802.15.4 driver:
    ///   - On nRF52832/52833/52840, this driver takes ownership of `RTC2`.
    ///   - On other chips (nRF52805–nRF52820 and nRF5340-net), this driver takes
    ///     ownership of `RTC1`.
    ///
    /// # Interrupt bindings
    ///
    /// The `_irq` parameter proves at compile time that the required interrupts have been
    /// bound using [`embassy_nrf::bind_interrupts!`]. The following bindings are required:
    /// - LP timer RTC interrupt → [`LpTimerInterruptHandler`](crate::LpTimerInterruptHandler)
    /// - EGU0/SWI0 interrupt → [`Egu0InterruptHandler`](crate::Egu0InterruptHandler)
    ///
    /// **Note:** `RTC1` is typically used by embassy-nrf's time driver. On chips without
    /// `RTC2`, you must ensure that embassy's time driver is configured to use a different
    /// timer or is disabled when this 802.15.4 driver is in use.
    ///
    /// # Example
    ///
    /// ```no_run
    /// use embassy_nrf::bind_interrupts;
    ///
    /// bind_interrupts!(struct Irqs {
    ///     // MPSL and 802.15.4 share the EGU0/SWI0 interrupt line.
    ///     // Both handlers are dispatched when this interrupt fires.
    ///     EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler;
    ///     EGU0_SWI0 => nrf_802154::Egu0InterruptHandler;
    ///     // On nRF5340-net, use EGU0 instead:
    ///     // EGU0 => nrf_mpsl::LowPrioInterruptHandler;
    ///     // EGU0 => nrf_802154::Egu0InterruptHandler;
    ///     // Other MPSL interrupts
    ///     RADIO => nrf_mpsl::HighPrioInterruptHandler;
    ///     TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    ///     RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    ///     POWER_CLOCK => nrf_mpsl::ClockInterruptHandler;
    ///     // 802.15.4 LP timer
    ///     RTC2 => nrf_802154::LpTimerInterruptHandler;  // or RTC1 on chips without RTC2
    /// });
    /// ```
    pub fn new<T: Instance, I>(
        _radio: Peri<'d, T>,
        _egu: Peri<'d, embassy_nrf::peripherals::EGU0>,
        _irq: I,
        _mpsl: &'d nrf_mpsl::MultiprotocolServiceLayer<'_>,
        _hp_timer: Peri<'d, embassy_nrf::peripherals::TIMER2>,
        #[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
        _lp_timer: Peri<'d, embassy_nrf::peripherals::RTC2>,
        #[cfg(not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840")))]
        _lp_timer: Peri<'d, embassy_nrf::peripherals::RTC1>,
    ) -> Self
    where
        I: Binding<LpTimerIrq, LpTimerInterruptHandler> + Binding<Egu0Irq, Egu0InterruptHandler>,
    {
        unsafe {
            raw::nrf_802154_init();
        }

        // // Enable NVIC interrupt
        // T::Interrupt::unpend();
        // unsafe { T::Interrupt::enable() };

        unsafe {
            raw::nrf_802154_channel_set(11);
            raw::nrf_802154_tx_power_set(0);
            // CCA defaults are set by nrf_802154_pib_init() during nrf_802154_init():
            //   mode = NRF_RADIO_CCA_MODE_ED (Energy Detection)
            //   ed_threshold = -75 dBm
            //   corr_threshold = 0x14, corr_limit = 0x02
            // Users can override via set_cca() if needed.
        }

        Self { _p: PhantomData }
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

        // TODO: Solve the i8 vs u8 mismatch
        match cfg.mode {
            raw::NRF_RADIO_CCA_MODE_CARRIER => Cca::Carrier,
            raw::NRF_RADIO_CCA_MODE_ED => Cca::Ed {
                ed_threshold: cfg.ed_threshold as _,
            },
            raw::NRF_RADIO_CCA_MODE_CARRIER_OR_ED => Cca::CarrierOrEd {
                ed_threshold: cfg.ed_threshold as _,
            },
            raw::NRF_RADIO_CCA_MODE_CARRIER_AND_ED => Cca::CarrierAndEd {
                ed_threshold: cfg.ed_threshold as _,
            },
            _ => unreachable!(),
        }
    }

    /// Change the Clear Channel Assessment method
    pub fn set_cca(&mut self, cca: Cca) {
        let (mode, ed_threshold) = match cca {
            Cca::Carrier => (raw::NRF_RADIO_CCA_MODE_CARRIER, 0),
            Cca::Ed { ed_threshold } => (raw::NRF_RADIO_CCA_MODE_ED, ed_threshold),
            Cca::CarrierOrEd { ed_threshold } => {
                (raw::NRF_RADIO_CCA_MODE_CARRIER_OR_ED, ed_threshold)
            }
            Cca::CarrierAndEd { ed_threshold } => {
                (raw::NRF_RADIO_CCA_MODE_CARRIER_AND_ED, ed_threshold)
            }
        };

        // TODO: Solve the i8 vs u8 mismatch
        unsafe {
            raw::nrf_802154_cca_cfg_set(&raw::nrf_802154_cca_cfg_t {
                mode,
                ed_threshold: ed_threshold as _,
                corr_threshold: CCA_CORR_THRESHOLD_DEFAULT,
                corr_limit: CCA_CORR_LIMIT_DEFAULT,
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
                raw::nrf_802154_short_address_set(addr_id.to_le_bytes().as_slice().as_ptr());
            } else {
                raw::nrf_802154_short_address_set(core::ptr::null());
            }
        }
    }

    /// Enable or disable promiscuous mode
    ///
    /// When enabled, the radio will receive all frames regardless of PAN ID, address, or other
    /// filtering. When disabled (the default), only frames matching the configured PAN ID and
    /// addresses are received.
    pub fn set_promiscuous(&mut self, enable: bool) {
        unsafe {
            raw::nrf_802154_promiscuous_set(enable);
        }
    }

    /// Set the extended address of the device
    ///
    /// # Arguments
    /// - `ext_addr_id`: The extended address to set. If `None`, the extended address filtering is disabled.
    pub fn set_ext_addr(&mut self, ext_addr_id: Option<u64>) {
        unsafe {
            if let Some(ext_addr_id) = ext_addr_id {
                raw::nrf_802154_extended_address_set(ext_addr_id.to_le_bytes().as_slice().as_ptr());
            } else {
                raw::nrf_802154_extended_address_set(core::ptr::null());
            }
        }
    }

    /// Set whether the radio should automatically enter receive mode after a transmission or when idle.
    ///
    /// # Arguments
    /// - `rx_when_idle`: If `true`, the radio will automatically enter receive mode after a transmission or when idle.
    ///   If `false`, the radio will remain in idle mode after a transmission or when idle.
    pub fn set_rx_when_idle(&mut self, rx_when_idle: bool) {
        unsafe {
            raw::nrf_802154_rx_on_when_idle_set(rx_when_idle);
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
    /// - `Ok(PsduMeta)` for the next successfully received frame, awaiting one if
    ///   the RX queue is currently empty
    /// - `Err(Error::EnterReceive)` if the radio could not enter receive mode
    ///
    /// Frame-level reception failures (CRC errors, aborts, ...) are dropped rather
    /// than surfaced as errors: `receive()` simply waits for the next good frame.
    pub async fn receive(&mut self, buf: &mut [u8]) -> Result<PsduMeta, Error> {
        DBG_RX_ENTER.fetch_add(1, Ordering::Relaxed);

        // Fast path: a frame may already be queued — the C driver auto-enters RX
        // after a transmit (rx_on_when_idle=true), so responses can arrive before
        // the next receive() call. Also clear any stale TX/CCA `status` now that
        // we're entering the RX phase: RX no longer uses `status` (it uses the
        // queue), but the next `transmit()` waits for a non-`Transmitting` status,
        // so a lingering `Transmitting` from the last TX must be cleared here — the
        // single-buffer implementation relied on this same clearing.
        let pending = STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = RadioStatus::Idle;
            state.rx_queue.dequeue_into(buf)
        });

        if let Some(meta) = pending {
            return Ok(meta);
        }

        let receive_entered = unsafe { raw::nrf_802154_receive() };

        if !receive_entered {
            return Err(Error::EnterReceive);
        }

        // Wait until a frame is queued, then drain the oldest one into `buf`.
        let meta = RadioState::wait(|state| state.rx_queue.dequeue_into(buf)).await;

        Ok(meta)
    }

    /// Number of received frames dropped because the RX queue ([`RX_QUEUE_LEN`])
    /// was full when they arrived. A non-zero value means the stack is draining
    /// `receive()` too slowly (e.g. the radio task is starved by other work) and
    /// the queue depth should be increased.
    pub fn rx_dropped(&self) -> u32 {
        STATE.lock(|state| state.borrow().rx_queue.dropped)
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
        DBG_TX_ENTER.fetch_add(1, Ordering::Relaxed);

        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        if let Some(ack_buf) = ack_buf.as_ref() {
            if ack_buf.len() < MAX_PSDU_SIZE {
                return Err(Error::ReceiveBufTooSmall);
            }
        }

        let packet_data = RadioState::wait(|state| {
            if !TX_BUSY.load(Ordering::Acquire) {
                state.tx[0] = data.len() as u8 + 2; // + CRC/FCS
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

        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = RadioStatus::Idle;
            state.tx_result = None;
        });

        let metadata = raw::nrf_802154_transmit_metadata_t {
            // The OpenThread integration never advertises `TRANSMIT_SEC`, so the
            // stack performs all 802.15.4 MAC security (AES-CCM* encryption + MIC)
            // and writes the frame counter IN SOFTWARE before handing us the PSDU.
            // We must tell the driver the frame is already fully prepared, otherwise
            // it tries to secure it itself, fails to find a stored key for the
            // frame's key ID, and rejects the TX with `KEY_ID_INVALID`. (For frames
            // with MAC security disabled these flags are simply ignored.)
            frame_props: raw::nrf_802154_transmitted_frame_props_t {
                is_secured: true,
                dynamic_data_is_set: true,
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
            // Requires NRF_802154_TX_TIMESTAMP_PROVIDER_ENABLED (which we don't
            // enable); leaving it false keeps the pre-nrfx-4 behavior.
            tx_timestamp_encode: false,
        };

        // nrf_802154_transmit_raw uses TERM_NONE, which cannot abort in-progress
        // RX (during PSDU reception), TX_ACK, or CCA operations. If the C driver
        // is busy, yield to let ISRs complete the current operation, then retry.
        // On Cortex-M, between returning Pending and the next poll, the executor
        // processes any pending hardware interrupts (RADIO ISR, etc.), allowing
        // the C driver's state machine to advance and complete the blocking operation.
        let mut scheduled = false;
        for _ in 0..TRANSMIT_SCHEDULE_RETRIES {
            // nrfx 4.x: transmit_raw now returns nrf_802154_tx_error_t (u8) instead
            // of a bool; NRF_802154_TX_ERROR_NONE (0) means the TX was scheduled.
            let err = unsafe { raw::nrf_802154_transmit_raw(packet_data, &metadata) };
            scheduled = u32::from(err) == raw::NRF_802154_TX_ERROR_NONE;
            if scheduled {
                break;
            }
            // Yield to let the executor poll other tasks and allow pending ISRs
            // (RADIO, TIMER) to fire and complete the in-progress operation.
            core::future::poll_fn(|cx| {
                cx.waker().wake_by_ref();
                core::task::Poll::<()>::Pending
            })
            .await;
        }

        if !scheduled {
            warn!("nrf_802154 TX could not be scheduled after retries (radio busy)");
            return Err(Error::ScheduleTransmit);
        }

        DBG_TX_SCHED.fetch_add(1, Ordering::Relaxed);

        Self::wait_transmit_done(&mut ack_buf).await
    }

    /// Transmit one radio packet using the CSMA-CA algorithm.
    ///
    /// This performs the full CSMA-CA procedure (random backoff + CCA + retry) before
    /// transmitting the frame. Use this instead of [`transmit`](Self::transmit) when
    /// the IEEE 802.15.4 CSMA-CA channel access method is needed.
    ///
    /// # Arguments
    /// - `data`: The PSDU data to transmit; this data should not contain PHY fields like PHR and CRC/FCS.
    ///   The data must be at most `MAX_PSDU_SIZE` bytes long.
    /// - `ack_buf`: If the radio is configured to wait for ACK frame in response to its transmission,
    ///   this buffer will be filled with the PSDU data of the received ACK frame.
    ///   In either case, `None` can be passed if the user is not interested in the ACK frame.
    ///
    /// # Returns
    /// - `Ok(Some(PsduMeta))` if the packet was successfully transmitted and an ACK was received.
    /// - `Ok(None)` if the packet was successfully transmitted and no ACK was expected.
    /// - `Err(Error::ScheduleTransmit)` if the transmission could not be scheduled (radio busy, etc)
    /// - `Err(Error::Transmit)` if the transmission failed (channel access failure, no ACK, etc)
    pub async fn transmit_csma_ca(
        &mut self,
        data: &[u8],
        mut ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        DBG_TX_ENTER.fetch_add(1, Ordering::Relaxed);

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
            if !TX_BUSY.load(Ordering::Acquire) {
                state.tx[0] = data.len() as u8 + 2; // + CRC/FCS
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

        STATE.lock(|state| {
            let mut state = state.borrow_mut();
            state.status = RadioStatus::Idle;
            state.tx_result = None;
        });

        let metadata = raw::nrf_802154_transmit_csma_ca_metadata_t {
            // See the note in `transmit`: OpenThread secures the frame in software
            // (no `TRANSMIT_SEC` cap), so the PSDU is already encrypted with its
            // frame counter set. Mark it as such, or the driver attempts its own
            // security processing and rejects the TX with `KEY_ID_INVALID`.
            frame_props: raw::nrf_802154_transmitted_frame_props_t {
                is_secured: true,
                dynamic_data_is_set: true,
            },
            tx_power: raw::nrf_802154_tx_power_metadata_t {
                use_metadata_value: false,
                power: 0,
            },
            tx_channel: raw::nrf_802154_tx_channel_metadata_t {
                use_metadata_value: false,
                channel: 0,
            },
            // Requires NRF_802154_TX_TIMESTAMP_PROVIDER_ENABLED (which we don't
            // enable); leaving it false keeps the pre-nrfx-4 behavior.
            tx_timestamp_encode: false,
        };

        // Same scheduling-retry as `transmit()`: `nrf_802154_transmit_csma_ca_raw`
        // uses TERM_NONE and cannot preempt an in-progress RX/CCA/TX_ACK. With
        // rx_on_when_idle the receiver is on almost continuously, so a single
        // attempt frequently fails to schedule — yield to let pending ISRs finish
        // the current operation, then retry. Without this, the schedule failure is
        // returned as a TX error that OpenThread reports as `ChannelAccessFailure`,
        // which is fatal for the back-to-back fragments of large frames.
        let mut scheduled = false;
        for _ in 0..TRANSMIT_SCHEDULE_RETRIES {
            // nrfx 4.x: transmit_csma_ca_raw now returns nrf_802154_tx_error_t (u8)
            // instead of a bool; NRF_802154_TX_ERROR_NONE (0) means scheduled.
            let err = unsafe { raw::nrf_802154_transmit_csma_ca_raw(packet_data, &metadata) };
            scheduled = u32::from(err) == raw::NRF_802154_TX_ERROR_NONE;
            if scheduled {
                break;
            }
            core::future::poll_fn(|cx| {
                cx.waker().wake_by_ref();
                core::task::Poll::<()>::Pending
            })
            .await;
        }

        if !scheduled {
            warn!("nrf_802154 TX could not be scheduled after retries (radio busy)");
            return Err(Error::ScheduleTransmit);
        }

        DBG_TX_SCHED.fetch_add(1, Ordering::Relaxed);

        Self::wait_transmit_done(&mut ack_buf).await
    }

    async fn wait_transmit_done(
        ack_buf: &mut Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        let tx_result = RadioState::wait(|state| {
            if let Some(TxResult::Done(_)) = &state.tx_result {
                if let Some(ack_buf) = ack_buf.as_mut() {
                    if let Some(TxResult::Done(Some(meta))) = state.tx_result {
                        ack_buf[..meta.len as usize]
                            .copy_from_slice(&state.ack_rx[1..][..meta.len as usize]);
                    }
                }
            }

            state.tx_result.take()
        })
        .await;

        DBG_TX_DONE.fetch_add(1, Ordering::Relaxed);

        match tx_result {
            TxResult::Done(psdu_meta) => Ok(psdu_meta),
            TxResult::Failed(code) => {
                let err = TxError::from(code);
                warn!("nrf_802154 TX failed: {:?}", err);
                Err(Error::Transmit(err))
            }
        }
    }
}

/// "A frame transmission is in progress" flag.
///
/// Set by the `tx_started` callback (which runs in the **high-priority radio
/// IRQ** that the `CriticalSectionRawMutex` does NOT mask — MPSL keeps it above
/// the critical-section level) and cleared by the TX-completion callbacks. It is
/// a lock-free atomic precisely because it is written from that high-priority
/// context: touching the `RefCell`-protected `RadioState` there would race with a
/// `STATE.lock()` held by the executor and panic with "already borrowed".
/// `transmit()` waits on this instead of a `RadioState` status before reusing the
/// shared TX buffer.
static TX_BUSY: AtomicBool = AtomicBool::new(false);

// Diagnostic run-loop progress counters (lock-free).
static DBG_RX_ENTER: AtomicU32 = AtomicU32::new(0);
static DBG_TX_ENTER: AtomicU32 = AtomicU32::new(0);
static DBG_TX_SCHED: AtomicU32 = AtomicU32::new(0);
static DBG_TX_DONE: AtomicU32 = AtomicU32::new(0);

/// Diagnostic snapshot of the radio's internal state (for wedge localization).
#[derive(Clone, Copy, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct RadioDebug {
    /// Total valid frames offered to the RX queue.
    pub received: u32,
    /// Frames dropped because the RX queue was full.
    pub dropped: u32,
    /// Frames currently sitting in the RX queue (full == not draining).
    pub queue_len: usize,
    /// `TX_BUSY`: 1 = a transmit is in flight, 0 = idle.
    pub status: u8,
    /// Times `receive()` was entered.
    pub rx_enter: u32,
    /// Times `transmit()`/`transmit_csma_ca()` were entered.
    pub tx_enter: u32,
    /// Times a transmit was scheduled and reached `wait_transmit_done`.
    pub tx_sched: u32,
    /// Times `wait_transmit_done` observed a completion and returned.
    pub tx_done: u32,
}

/// Diagnostic snapshot, readable without a [`Radio`] handle.
///
/// During a wedge the frozen counters localize where the run loop is parked:
/// `tx_enter > tx_sched` ⇒ stuck before scheduling (status guard); `tx_sched >
/// tx_done` ⇒ stuck in `wait_transmit_done` (a TX whose completion never fired);
/// `queue_len` at capacity confirms `receive()` is not draining.
pub fn rx_stats() -> RadioDebug {
    let status = if TX_BUSY.load(Ordering::Relaxed) {
        1
    } else {
        0
    };

    STATE.lock(|state| {
        let state = state.borrow();
        RadioDebug {
            received: state.rx_queue.received,
            dropped: state.rx_queue.dropped,
            queue_len: state.rx_queue.len,
            status,
            rx_enter: DBG_RX_ENTER.load(Ordering::Relaxed),
            tx_enter: DBG_TX_ENTER.load(Ordering::Relaxed),
            tx_sched: DBG_TX_SCHED.load(Ordering::Relaxed),
            tx_done: DBG_TX_DONE.load(Ordering::Relaxed),
        }
    })
}

impl Drop for Radio<'_> {
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
    CcaFailed(raw::nrf_802154_cca_error_t),
    CcaDone(bool),
    EnergyDetectionDetected(i8),
    EnergyDetectionFailed(raw::nrf_802154_ed_error_t),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
enum TxResult {
    Failed(raw::nrf_802154_tx_error_t),
    Done(Option<PsduMeta>),
}

/// Depth of the RX ring buffer, in frames.
///
/// Received frames are queued by the `nrf_802154_received*` callbacks and drained
/// by `receive()`. A queue — rather than a single buffer — is needed because
/// OpenThread can be slow to call `receive()` when the executor is busy (e.g. when
/// the radio shares the embassy executor with an `embassy-net` stack). With a
/// single buffer, a frame arriving before the previous one is read would be
/// overwritten and lost; the queue retains in-flight frames until the stack drains
/// them. Increase this if `Radio::rx_dropped()` is non-zero under load.
const RX_QUEUE_LEN: usize = 16;

/// One queued received frame.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RxFrame {
    meta: PsduMeta,
    /// The raw frame as delivered by the driver: `[PHR, PSDU..]` (the PSDU
    /// includes the 2-byte FCS). `receive()` hands out `data[1..][..meta.len]`.
    data: [u8; MAX_PACKET_SIZE],
}

impl RxFrame {
    const EMPTY: Self = Self {
        meta: PsduMeta {
            len: 0,
            crc: 0,
            power: 0,
            lqi: None,
            time: None,
        },
        data: [0; MAX_PACKET_SIZE],
    };
}

/// Fixed-capacity ring buffer of received frames.
#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RxQueue {
    frames: [RxFrame; RX_QUEUE_LEN],
    /// Index of the oldest queued frame.
    head: usize,
    /// Number of queued frames.
    len: usize,
    /// Total valid frames offered to the queue (diagnostic).
    received: u32,
    /// Total frames dropped because the queue was full (saturating-ish, wrapping).
    dropped: u32,
}

impl RxQueue {
    const fn new() -> Self {
        Self {
            frames: [RxFrame::EMPTY; RX_QUEUE_LEN],
            head: 0,
            len: 0,
            received: 0,
            dropped: 0,
        }
    }

    /// Reserve the next free slot and return a mutable reference to fill it, or
    /// `None` (counting a drop) if the queue is full.
    fn enqueue_slot(&mut self) -> Option<&mut RxFrame> {
        self.received = self.received.wrapping_add(1);

        if self.len == RX_QUEUE_LEN {
            self.dropped = self.dropped.wrapping_add(1);
            return None;
        }

        let idx = (self.head + self.len) % RX_QUEUE_LEN;
        self.len += 1;
        Some(&mut self.frames[idx])
    }

    /// Pop the oldest frame's PSDU (excluding the PHR) into `buf`, returning its
    /// metadata, or `None` if the queue is empty.
    fn dequeue_into(&mut self, buf: &mut [u8]) -> Option<PsduMeta> {
        if self.len == 0 {
            return None;
        }

        let frame = &self.frames[self.head];
        let len = frame.meta.len as usize;
        let meta = frame.meta;
        buf[..len].copy_from_slice(&frame.data[1..][..len]);

        self.head = (self.head + 1) % RX_QUEUE_LEN;
        self.len -= 1;

        Some(meta)
    }
}

#[derive(Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
struct RadioState {
    status: RadioStatus,
    /// Separate TX completion result, not overwritten by RX callbacks.
    /// This prevents a race where a received frame (from the C driver's
    /// auto-RX after TX) overwrites a pending TransmitDone status before
    /// `wait_transmit_done` can process it.
    tx_result: Option<TxResult>,
    tx: [u8; MAX_PACKET_SIZE],
    /// Ring buffer of received frames, filled by the `nrf_802154_received*`
    /// callbacks and drained by `receive()`.
    rx_queue: RxQueue,
    /// Separate buffer for TX ACK data, so `nrf_802154_received_raw` cannot
    /// overwrite ACK data before `wait_transmit_done` reads it.
    ack_rx: [u8; MAX_PACKET_SIZE],
}

impl RadioState {
    const fn new() -> Self {
        Self {
            status: RadioStatus::Idle,
            tx_result: None,
            tx: [0; MAX_PACKET_SIZE],
            rx_queue: RxQueue::new(),
            ack_rx: [0; MAX_PACKET_SIZE],
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
unsafe extern "C" fn nrf_802154_energy_detected(
    p_result: *const raw::nrf_802154_energy_detected_t,
) {
    RadioState::update(|state| {
        state.status =
            RadioStatus::EnergyDetectionDetected(unsafe { p_result.as_ref().unwrap().ed_dbm })
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_energy_detection_failed(error: raw::nrf_802154_ed_error_t) {
    RadioState::update(|state| state.status = RadioStatus::EnergyDetectionFailed(error));
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_tx_ack_started() {
    // No-op: unlike the notification callbacks (`received_raw`, `cca_done`,
    // ... — deferred to the maskable EGU/SWI priority via
    // `NRF_802154_NOTIFICATION_IMPL=1` in the sys build), this is a *direct*
    // core callout from the high-priority radio IRQ, which the
    // `CriticalSectionRawMutex` does not mask. It MUST NOT touch the
    // `RefCell`-protected `RadioState` — doing so races with a `STATE.lock()`
    // held by the executor and panics with "already borrowed". The information
    // (we started auto-ACKing a received frame) isn't needed by the driver.
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_received_raw(p_data: *mut u8, power: i8, lqi: u8) {
    RadioState::update(|state| {
        let phr = unsafe { *p_data };
        let total = phr as usize + 1;

        if phr >= MIN_PHR && total <= MAX_PACKET_SIZE {
            if let Some(frame) = state.rx_queue.enqueue_slot() {
                frame.data[..total]
                    .copy_from_slice(unsafe { core::slice::from_raw_parts(p_data, total) });
                frame.meta = PsduMeta {
                    len: phr - 2, // PHR value - FCS
                    crc: u16::from_le_bytes([frame.data[total - 2], frame.data[total - 1]]),
                    power,
                    lqi: Some(lqi),
                    time: None,
                };
            }
            // else: queue full — drop the frame (counted in `rx_queue.dropped`).
        }
        // else: invalid PHR/length — drop silently.

        unsafe {
            raw::nrf_802154_buffer_free_raw(p_data);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_received_timestamp_raw(
    p_data: *mut u8,
    power: i8,
    lqi: u8,
    time: u64,
) {
    RadioState::update(|state| {
        let phr = unsafe { *p_data };
        let total = phr as usize + 1;

        if phr >= MIN_PHR && total <= MAX_PACKET_SIZE {
            if let Some(frame) = state.rx_queue.enqueue_slot() {
                frame.data[..total]
                    .copy_from_slice(unsafe { core::slice::from_raw_parts(p_data, total) });
                frame.meta = PsduMeta {
                    len: phr - 2, // PHR value - FCS
                    crc: u16::from_le_bytes([frame.data[total - 2], frame.data[total - 1]]),
                    power,
                    lqi: Some(lqi),
                    time: Some(time),
                };
            }
            // else: queue full — drop the frame (counted in `rx_queue.dropped`).
        }
        // else: invalid PHR/length — drop silently.

        unsafe {
            raw::nrf_802154_buffer_free_raw(p_data);
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_receive_failed(_error: raw::nrf_802154_rx_error_t, _id: u32) {
    // A frame-level reception failure (CRC error, invalid frame, abort, ...). With
    // rx_on_when_idle the radio stays in RX, so we just drop the failed reception
    // and let `receive()` keep waiting for the next good frame, rather than
    // surfacing transient RX noise to OpenThread as a receive error.
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmitted_raw(
    _p_frame: *mut u8,
    p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    // The hardware TX has completed: clear the `TX_BUSY` flag set by `tx_started`.
    // Done in the completion callback (which always fires when the hardware
    // finishes), not in `wait_transmit_done` — under load the run loop's
    // `select(new_cmd, transmit)` can drop the `transmit()` future mid-flight, so
    // `wait_transmit_done` may never run. A leaked `TX_BUSY` would otherwise stall
    // the next `transmit()` forever on its guard, wedging the run loop.
    TX_BUSY.store(false, Ordering::Release);

    RadioState::update(|state| {
        let p_metadata = unsafe { p_metadata.as_ref().unwrap() };
        if !p_metadata.data.transmitted.p_ack.is_null() {
            let total = p_metadata.data.transmitted.length as usize;

            if (MIN_PHR as usize..=MAX_PACKET_SIZE).contains(&total) {
                let packet = unsafe {
                    core::slice::from_raw_parts(p_metadata.data.transmitted.p_ack, total)
                };

                state.ack_rx[..total].copy_from_slice(packet);

                state.tx_result = Some(TxResult::Done(Some(PsduMeta {
                    len: (total - 1 - 2) as u8, // total - PHR - FCS
                    crc: u16::from_le_bytes([state.ack_rx[total - 2], state.ack_rx[total - 1]]),
                    power: p_metadata.data.transmitted.power,
                    lqi: Some(p_metadata.data.transmitted.lqi),
                    time: Some(p_metadata.data.transmitted.time),
                })));
            } else {
                state.tx_result = Some(TxResult::Done(None));
            }

            unsafe {
                raw::nrf_802154_buffer_free_raw(p_metadata.data.transmitted.p_ack);
            }
        } else {
            state.tx_result = Some(TxResult::Done(None));
        }
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmit_failed(
    _p_frame: *mut u8,
    error: raw::nrf_802154_tx_error_t,
    _p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    // Clear `TX_BUSY` on hardware completion — see the note in `transmitted_raw`.
    TX_BUSY.store(false, Ordering::Release);

    RadioState::update(|state| {
        state.tx_result = Some(TxResult::Failed(error));
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_tx_started(_p_frame: *const u8) {
    // A *direct* core callout, not a notification (see `tx_ack_started`): runs
    // in the high-priority radio IRQ, so use the lock-free `TX_BUSY` flag, not
    // the `RefCell`-protected `RadioState`.
    TX_BUSY.store(true, Ordering::Release);
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_custom_part_of_radio_init() {}
