use core::cell::RefCell;
use core::marker::PhantomData;
use core::sync::atomic::{AtomicBool, AtomicU32, AtomicU8, Ordering};

use embassy_nrf::Peri;
use embassy_sync::blocking_mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::raw;

#[cfg_attr(not(feature = "_nrf54l"), path = "radio/nrf5x.rs")]
#[cfg_attr(feature = "_nrf54l", path = "radio/nrf54l.rs")]
mod imp;

pub use imp::*;

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
    /// Could not enter sleep mode
    EnterSleep,
    /// Transmission failed
    Transmit(TxError),
    /// Reception failed (CRC error, aborted, etc)
    Receive,
    /// Could not schedule a timed receive window (see [`Radio::receive_at`])
    ScheduleReceive,
    /// The driver rejected a MAC key (see [`Radio::set_mac_keys`])
    Security(raw::nrf_802154_security_error_t),
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

impl Cca {
    /// An Energy Detection CCA with its threshold given in dBm, converted to
    /// the hardware's raw 0..0xFF ED scale by the C driver's own helper.
    pub fn ed_from_dbm(dbm: i8) -> Self {
        Self::Ed {
            ed_threshold: unsafe { raw::nrf_802154_ccaedthres_from_dbm_calculate(dbm) },
        }
    }
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
    /// Timestamp taken when the last symbol of the frame was received, in
    /// microseconds of the driver clock ([`Radio::now_us`])
    pub time: Option<u64>,
    /// The security material of the *secured enhanced ACK* the driver sent for
    /// this frame, if it sent one (see [`AckSecurity`]).
    pub ack_security: Option<AckSecurity>,
}

impl PsduMeta {
    /// The time the start of the frame's PHR was at the antenna (i.e. the end
    /// of its SFD), in microseconds of the driver clock, derived from
    /// [`time`](Self::time): the PHR byte and the PSDU (with its FCS) each take
    /// 32 µs on the air.
    pub fn phr_time(&self) -> Option<u64> {
        const SYMBOLS_PER_BYTE: u64 = 2;
        const US_PER_SYMBOL: u64 = 16;

        self.time.map(|end| {
            let bytes = 1 /* PHR */ + self.len as u64 + 2 /* FCS */;

            end.saturating_sub(bytes * SYMBOLS_PER_BYTE * US_PER_SYMBOL)
        })
    }
}

/// What the driver still has to do to a frame before transmitting it.
///
/// The driver can finish a frame itself: assign the frame counter and the
/// key index, fill the CSL IE (if the frame carries one) with the phase of the
/// next receive window as of the moment the frame goes on the air, and secure
/// it (AES-CCM*) with a key from [`Radio::set_mac_keys`]. A caller that did all
/// of that already passes [`FrameProps::PREPARED`].
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FrameProps {
    /// The frame is already secured (MIC computed, payload encrypted), or
    /// needs no security. When `false` the driver secures it.
    pub is_secured: bool,
    /// The frame counter, key index and CSL IE are already final. When `false`
    /// the driver assigns them.
    pub dynamic_data_is_set: bool,
}

impl FrameProps {
    /// A frame the caller has fully prepared: the driver sends it as is.
    pub const PREPARED: Self = Self {
        is_secured: true,
        dynamic_data_is_set: true,
    };
}

/// The security material the driver used for a secured enhanced ACK it sent
/// in response to a received frame.
///
/// A frame secured by the peer (e.g. a Thread 1.2 parent transmitting to a
/// CSL child) is acknowledged with a secured enhanced ACK, which consumes a
/// MAC frame counter of *this* node. The stack, which secures the data frames
/// with the same key in software, has to know each counter the ACKs used up -
/// see the frame-counter feedback of the OpenThread radio platform.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct AckSecurity {
    /// The frame counter of the ACK.
    pub frame_counter: u32,
    /// The key index (key ID mode 1) the ACK was secured with; `0` for another
    /// key ID mode.
    pub key_id: u8,
}

/// A MAC key handed to the driver for securing its enhanced ACKs.
#[derive(Clone, Copy)]
pub struct MacKey {
    /// The key index (key ID mode 1).
    pub key_id: u8,
    /// The key.
    pub key: [u8; 16],
}

/// The security material of the enhanced ACK the driver is currently sending
/// (`nrf_802154_tx_ack_started`), handed over to the received frame the ACK
/// is for (`nrf_802154_received[_timestamp]_raw`, which the driver reports once
/// the ACK is out). Atomics rather than the `RadioState` lock, because the
/// ACK-started callout runs from the radio IRQ, which the lock does not mask.
static ACK_SEC_PENDING: AtomicBool = AtomicBool::new(false);
/// Whether a timed receive window is scheduled (see [`Radio::receive_at`]).
static RX_WINDOW_SCHEDULED: AtomicBool = AtomicBool::new(false);
/// Whether the timed receive window ended (or never opened) since it was
/// scheduled: the driver leaves the radio in the receive state at the end of
/// a window (`receive_at` is a delayed `receive`), so whoever drains the
/// window puts it to sleep (see [`Radio::wait_window_frame`]).
static RX_WINDOW_ENDED: AtomicBool = AtomicBool::new(false);

/// The id of the one timed receive window ([`Radio::receive_at`]) in flight:
/// any id below the driver's reserved range will do, and one window at a time
/// keeps it unambiguous.
const RX_WINDOW_ID: u32 = 1;
static ACK_SEC_FRAME_COUNTER: AtomicU32 = AtomicU32::new(0);
static ACK_SEC_KEY_ID: AtomicU8 = AtomicU8::new(0);

/// Take the security material of the enhanced ACK just sent, if any.
fn take_ack_security() -> Option<AckSecurity> {
    ACK_SEC_PENDING
        .swap(false, Ordering::AcqRel)
        .then(|| AckSecurity {
            frame_counter: ACK_SEC_FRAME_COUNTER.load(Ordering::Relaxed),
            key_id: ACK_SEC_KEY_ID.load(Ordering::Relaxed),
        })
}

/// Parse the auxiliary security header of an ACK frame (PHR + PSDU), returning
/// its frame counter and key index if the ACK is a secured enhanced ACK whose
/// frame counter is not suppressed.
fn parse_ack_security(frame: &[u8]) -> Option<AckSecurity> {
    const FRAME_TYPE_ACK: u16 = 2;
    const FRAME_VERSION_2015: u16 = 2;

    let psdu = frame.get(1..)?;
    let fcf = u16::from_le_bytes([*psdu.first()?, *psdu.get(1)?]);

    if fcf & 0x7 != FRAME_TYPE_ACK || fcf & (1 << 3) == 0 || (fcf >> 12) & 0x3 != FRAME_VERSION_2015
    {
        return None;
    }

    let pan_id_compression = fcf & (1 << 6) != 0;
    let seq_suppressed = fcf & (1 << 8) != 0;
    let dst_mode = (fcf >> 10) & 0x3;
    let src_mode = (fcf >> 14) & 0x3;

    let addr_len = |mode: u16| match mode {
        2 => 2,
        3 => 8,
        _ => 0,
    };

    // IEEE 802.15.4-2015, table 7-2: the PAN ID fields present for a version-2
    // frame, as a function of the address modes and the PAN ID compression bit.
    let (dst_pan, src_pan) = match (dst_mode != 0, src_mode != 0, pan_id_compression) {
        (false, false, false) => (false, false),
        (false, false, true) => (true, false),
        (true, false, false) => (true, false),
        (true, false, true) => (false, false),
        (false, true, false) => (false, true),
        (false, true, true) => (false, false),
        (true, true, false) if dst_mode == 3 && src_mode == 3 => (true, false),
        (true, true, true) if dst_mode == 3 && src_mode == 3 => (false, false),
        (true, true, false) => (true, true),
        (true, true, true) => (true, false),
    };

    let mut offset = 2 + if seq_suppressed { 0 } else { 1 };
    offset += if dst_pan { 2 } else { 0 } + addr_len(dst_mode);
    offset += if src_pan { 2 } else { 0 } + addr_len(src_mode);

    let sec_ctrl = *psdu.get(offset)?;
    offset += 1;

    let key_id_mode = (sec_ctrl >> 3) & 0x3;
    let frame_counter_suppressed = sec_ctrl & (1 << 5) != 0;

    if frame_counter_suppressed {
        return None;
    }

    let fc = psdu.get(offset..offset + 4)?;
    let frame_counter = u32::from_le_bytes([fc[0], fc[1], fc[2], fc[3]]);
    offset += 4;

    let key_id = if key_id_mode == 1 {
        *psdu.get(offset)?
    } else {
        0
    };

    Some(AckSecurity {
        frame_counter,
        key_id,
    })
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
    /// In addition to the RADIO peripheral and the MPSL reference, this constructor takes
    /// ownership of the peripherals used by the 802.15.4 platform layer, bundled into
    /// [`RadioPeripherals`]. Which ones those are is chip-specific — see that type.
    ///
    /// # Interrupt bindings
    ///
    /// The `_irq` parameter proves at compile time that the required interrupts have been
    /// bound using [`embassy_nrf::bind_interrupts!`]. The following bindings are required:
    /// - LP timer interrupt → [`LpTimerInterruptHandler`](crate::LpTimerInterruptHandler)
    ///   (`RTC2`/`RTC1` on nRF52/nRF53, `GRTC_0` on nRF54L)
    /// - EGU interrupt → [`EguInterruptHandler`](crate::EguInterruptHandler)
    ///   (`EGU0_SWI0` on nRF52, `EGU0` on nRF5340-net, `EGU10` on nRF54L)
    /// - On nRF54L only, the CCM00 encryption accelerator interrupt
    ///   (`AAR00_CCM00`) → [`CcmInterruptHandler`](crate::CcmInterruptHandler)
    ///
    /// **Note:** on nRF52 chips without an `RTC2` (nRF52805–nRF52820) and on nRF5340-net,
    /// the LP timer falls back to `RTC1`, which is also embassy-nrf's default time driver.
    /// There you must point embassy's time driver at a different peripheral or disable it.
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
    ///     EGU0_SWI0 => nrf_802154::EguInterruptHandler;
    ///     // On nRF5340-net, use EGU0 instead:
    ///     // EGU0 => nrf_mpsl::LowPrioInterruptHandler;
    ///     // EGU0 => nrf_802154::EguInterruptHandler;
    ///     // Other MPSL interrupts
    ///     RADIO => nrf_mpsl::HighPrioInterruptHandler;
    ///     TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    ///     RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    ///     POWER_CLOCK => nrf_mpsl::ClockInterruptHandler;
    ///     // 802.15.4 LP timer
    ///     RTC2 => nrf_802154::LpTimerInterruptHandler;  // or RTC1 on chips without RTC2
    /// });
    /// ```
    ///
    /// On nRF54L the 802.15.4 driver has its own EGU instance, so nothing is shared with
    /// MPSL's low-priority line:
    ///
    /// ```ignore
    /// bind_interrupts!(struct Irqs {
    ///     SWI00 => nrf_mpsl::LowPrioInterruptHandler;
    ///     RADIO_0 => nrf_mpsl::HighPrioInterruptHandler;
    ///     TIMER10 => nrf_mpsl::HighPrioInterruptHandler;
    ///     GRTC_3 => nrf_mpsl::HighPrioInterruptHandler;
    ///     CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    ///     EGU10 => nrf_802154::EguInterruptHandler;
    ///     GRTC_0 => nrf_802154::LpTimerInterruptHandler;
    ///     AAR00_CCM00 => nrf_802154::CcmInterruptHandler;
    /// });
    /// ```
    pub fn new<I: InterruptBindings>(
        _radio: Peri<'d, embassy_nrf::peripherals::RADIO>,
        _peripherals: RadioPeripherals<'d>,
        _irq: I,
        _mpsl: &'d nrf_mpsl::MultiprotocolServiceLayer<'_>,
    ) -> Self {
        if INITIALIZED.swap(true, Ordering::SeqCst) {
            // The previous instance's teardown reset the driver; do it again
            // in case that reset found the radio busy
            reset_driver();
        } else {
            unsafe {
                raw::nrf_802154_init();
            }
        }

        unsafe {
            raw::nrf_802154_channel_set(11);
            raw::nrf_802154_tx_power_set(0);
            // The Thread flavor of pending-bit source matching (see
            // `set_src_match_enabled`); explicit, though it is the default.
            raw::nrf_802154_src_addr_matching_method_set(
                raw::NRF_802154_SRC_ADDR_MATCH_THREAD as _,
            );
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

    /// Move the radio to the SLEEP state: the receiver is off and nothing is
    /// received until [`enter_receive`](Self::enter_receive) (or an operation
    /// that implies it) brings it back.
    ///
    /// Frames already in the RX queue stay there - they were received while
    /// awake and are still owed to the caller.
    ///
    /// Returns `false` if the driver refused the transition (an operation is
    /// in progress).
    ///
    /// A receive window scheduled with [`receive_at`](Self::receive_at) is
    /// left alone: it is a request for the future, and sleeping now is what a
    /// node does between its windows.
    pub fn sleep(&mut self) -> bool {
        unsafe { raw::nrf_802154_sleep() }
    }

    /// Whether a receive window scheduled with
    /// [`receive_at`](Self::receive_at) is still pending.
    pub fn has_pending_window(&self) -> bool {
        RX_WINDOW_SCHEDULED.load(Ordering::Relaxed)
    }

    /// Move the radio to the RECEIVE state (the counterpart of
    /// [`sleep`](Self::sleep); [`receive`](Self::receive) also enters it on
    /// its own).
    ///
    /// Returns `false` if the driver refused the transition.
    pub fn enter_receive(&mut self) -> bool {
        unsafe { raw::nrf_802154_receive() }
    }

    /// Enable or disable source-address matching for the pending bit of
    /// automatically transmitted ACK frames.
    ///
    /// Enabled: an ACK's pending bit is set only when the frame's source
    /// address is in the pending-bit list (see
    /// [`set_pending_short`](Self::set_pending_short) /
    /// [`set_pending_ext`](Self::set_pending_ext)). Disabled: the pending bit
    /// is set in every ACK - the protocol-safe over-promise.
    pub fn set_src_match_enabled(&mut self, enabled: bool) {
        unsafe {
            raw::nrf_802154_auto_pending_bit_set(enabled);
        }
    }

    /// Add (or remove) a short address to the ACK pending-bit list.
    ///
    /// Returns `false` when adding failed because the list is full
    /// (`NRF_802154_PENDING_SHORT_ADDRESSES`).
    pub fn set_pending_short(&mut self, addr: u16, pending: bool) -> bool {
        let addr = addr.to_le_bytes();

        unsafe {
            if pending {
                raw::nrf_802154_pending_bit_for_addr_set(addr.as_ptr(), false)
            } else {
                raw::nrf_802154_pending_bit_for_addr_clear(addr.as_ptr(), false)
            }
        }
    }

    /// Add (or remove) an extended address to the ACK pending-bit list.
    ///
    /// Returns `false` when adding failed because the list is full
    /// (`NRF_802154_PENDING_EXTENDED_ADDRESSES`).
    pub fn set_pending_ext(&mut self, addr: u64, pending: bool) -> bool {
        let addr = addr.to_le_bytes();

        unsafe {
            if pending {
                raw::nrf_802154_pending_bit_for_addr_set(addr.as_ptr(), true)
            } else {
                raw::nrf_802154_pending_bit_for_addr_clear(addr.as_ptr(), true)
            }
        }
    }

    /// Empty both (short and extended) ACK pending-bit lists.
    pub fn clear_pending(&mut self) {
        unsafe {
            raw::nrf_802154_pending_bit_for_addr_reset(false);
            raw::nrf_802154_pending_bit_for_addr_reset(true);
        }
    }

    /// Move the radio from any state to the DISABLED state
    fn disable(&mut self) {
        // TODO: Is this even supported in the C driver?
    }

    /// The driver clock, in microseconds: the time base of the frame
    /// timestamps ([`PsduMeta::time`]), of the timed receive windows
    /// ([`receive_at`](Self::receive_at)) and of the CSL anchor time.
    pub fn now_us(&self) -> u64 {
        unsafe { raw::nrf_802154_time_get() }
    }

    /// Schedule a receive window: the receiver goes on for `channel` at
    /// `start_us` ([`now_us`](Self::now_us) time base) and off again after
    /// `duration_us`, unless a frame is being received then. Frames received
    /// in the window land in the RX queue, to be drained with
    /// [`wait_frame`](Self::wait_frame) - not [`receive`](Self::receive), which
    /// would switch the receiver on for good.
    ///
    /// This is how a Thread CSL child samples the channel at its parent's
    /// transmit times without polling. One window at a time: scheduling a new
    /// one cancels a still pending one.
    ///
    /// Returns `false` if the driver could not schedule the window (too late,
    /// or the timeslot could not be reserved).
    pub fn receive_at(&mut self, start_us: u64, duration_us: u32, channel: u8) -> bool {
        self.receive_at_cancel();
        RX_WINDOW_ENDED.store(false, Ordering::Relaxed);

        let scheduled =
            unsafe { raw::nrf_802154_receive_at(start_us, duration_us, channel, RX_WINDOW_ID) };

        trace!(
            "nrf_802154 timed receive: in {} us for {} us -> {}",
            start_us as i64 - self.now_us() as i64,
            duration_us,
            scheduled
        );

        if scheduled {
            RX_WINDOW_SCHEDULED.store(true, Ordering::Relaxed);
        }

        scheduled
    }

    /// Cancel the receive window scheduled by [`receive_at`](Self::receive_at),
    /// if it has not started yet (a started one just runs to its end).
    pub fn receive_at_cancel(&mut self) {
        RX_WINDOW_ENDED.store(false, Ordering::Relaxed);

        if RX_WINDOW_SCHEDULED.swap(false, Ordering::Relaxed) {
            unsafe {
                raw::nrf_802154_receive_at_cancel(RX_WINDOW_ID);
            }
        }
    }

    /// Move the radio to the SLEEP state unless an operation is in progress
    /// (in which case nothing changes and `false` is returned). Unlike
    /// [`sleep`](Self::sleep), a scheduled receive window is left alone.
    pub fn sleep_if_idle(&mut self) -> bool {
        unsafe {
            raw::nrf_802154_sleep_if_idle()
                == raw::NRF_802154_SLEEP_ERROR_NONE as raw::nrf_802154_sleep_error_t
        }
    }

    /// Wait for a received frame and drain the oldest one into `buf`, without
    /// changing the radio state - the counterpart of
    /// [`receive`](Self::receive) for a receiver that is on by a timed window
    /// ([`receive_at`](Self::receive_at)), or on by itself
    /// (`rx_when_idle`).
    pub async fn wait_frame(&mut self, buf: &mut [u8]) -> PsduMeta {
        crate::platform::refresh_temperature();

        RadioState::wait(|state| state.rx_queue.dequeue_into(buf)).await
    }

    /// [`wait_frame`](Self::wait_frame), for the frames of a timed receive
    /// window ([`receive_at`](Self::receive_at)) - which also puts the radio
    /// to sleep when the window ends.
    ///
    /// The driver ends a window in the receive state (`receive_at` is a
    /// delayed `receive`), and leaves the transition to sleep to its user, as
    /// Nordic's own OpenThread platform does on the window's timeout. Without
    /// it the receiver would stay on until the next operation - which for a
    /// CSL child is the next window, so it would never really sleep.
    pub async fn wait_window_frame(&mut self, buf: &mut [u8]) -> PsduMeta {
        crate::platform::refresh_temperature();

        loop {
            let frame = RadioState::wait(|state| {
                if let Some(meta) = state.rx_queue.dequeue_into(buf) {
                    Some(Some(meta))
                } else {
                    RX_WINDOW_ENDED
                        .swap(false, Ordering::Relaxed)
                        .then_some(None)
                }
            })
            .await;

            match frame {
                Some(meta) => break meta,
                // A frame being received or acknowledged right now keeps the
                // radio busy, and it then returns to the receive state on its
                // own; that receiver goes off with the next operation.
                None => {
                    self.sleep_if_idle();
                }
            }
        }
    }

    /// Set the CSL period the driver advertises in the CSL IE of its enhanced
    /// ACKs, in units of 10 symbols (160 µs); `0` stops injecting the IE.
    ///
    /// Together with the anchor time
    /// ([`set_csl_anchor_time`](Self::set_csl_anchor_time)) this is what tells
    /// a Thread 1.2 parent when this CSL child listens next.
    pub fn set_csl_period(&mut self, period: u16) {
        unsafe { raw::nrf_802154_csl_writer_period_set(period) }
    }

    /// Set the CSL anchor time: a time at which the CSL phase is zero, i.e.
    /// when the first bit of the MAC header of a frame from the parent is
    /// expected in some receive window (past or future - the driver extends
    /// it by whole periods), in the [`now_us`](Self::now_us) time base. The
    /// driver computes the CSL phase of each enhanced ACK from it.
    pub fn set_csl_anchor_time(&mut self, anchor_us: u64) {
        unsafe { raw::nrf_802154_csl_writer_anchor_time_set(anchor_us) }
    }

    /// The CSL IE the driver injects into its enhanced ACKs: a header IE of
    /// element ID `IE_CSL_ID` (0x1a) with 4 bytes of content - the phase and
    /// the period - that the driver's IE writer fills in at ACK time.
    const CSL_IE: [u8; 6] = [0x04, 0x0d, 0, 0, 0, 0];

    /// Inject the CSL IE into the enhanced ACKs sent to the given peer (the
    /// CSL parent), addressed by its short and/or extended address. Without
    /// this the ACKs carry no CSL IE and the parent cannot keep its transmit
    /// schedule in step with this node's receive windows.
    ///
    /// Returns `false` if the driver's ACK data table is full.
    pub fn set_csl_ie_peer(&mut self, short_addr: Option<u16>, ext_addr: Option<u64>) -> bool {
        let mut ok = true;

        if let Some(short_addr) = short_addr {
            ok &= unsafe {
                raw::nrf_802154_ack_data_set(
                    short_addr.to_le_bytes().as_ptr(),
                    false,
                    Self::CSL_IE.as_ptr() as *const _,
                    Self::CSL_IE.len() as _,
                    raw::NRF_802154_ACK_DATA_IE as raw::nrf_802154_ack_data_t,
                )
            };
        }

        if let Some(ext_addr) = ext_addr {
            ok &= unsafe {
                raw::nrf_802154_ack_data_set(
                    ext_addr.to_le_bytes().as_ptr(),
                    true,
                    Self::CSL_IE.as_ptr() as *const _,
                    Self::CSL_IE.len() as _,
                    raw::NRF_802154_ACK_DATA_IE as raw::nrf_802154_ack_data_t,
                )
            };
        }

        ok
    }

    /// Stop injecting the CSL IE into the enhanced ACKs sent to the given peer.
    pub fn clear_csl_ie_peer(&mut self, short_addr: Option<u16>, ext_addr: Option<u64>) {
        if let Some(short_addr) = short_addr {
            unsafe {
                raw::nrf_802154_ack_data_clear(
                    short_addr.to_le_bytes().as_ptr(),
                    false,
                    raw::NRF_802154_ACK_DATA_IE as raw::nrf_802154_ack_data_t,
                );
            }
        }

        if let Some(ext_addr) = ext_addr {
            unsafe {
                raw::nrf_802154_ack_data_clear(
                    ext_addr.to_le_bytes().as_ptr(),
                    true,
                    raw::NRF_802154_ACK_DATA_IE as raw::nrf_802154_ack_data_t,
                );
            }
        }
    }

    /// Replace the MAC keys the driver secures its enhanced ACKs with (key ID
    /// mode `key_id_mode`, one entry per key index). All keys use the global
    /// frame counter ([`set_frame_counter`](Self::set_frame_counter)).
    ///
    /// The driver copies the key material.
    pub fn set_mac_keys(&mut self, key_id_mode: u8, keys: &[MacKey]) -> Result<(), Error> {
        self.clear_mac_keys();

        for key in keys {
            let mut key_material = key.key;
            let mut key_id = key.key_id;

            let mut entry = raw::nrf_802154_key_t {
                value: raw::nrf_802154_key_t__bindgen_ty_1 {
                    p_cleartext_key: key_material.as_mut_ptr(),
                },
                id: raw::nrf_802154_key_id_t {
                    mode: key_id_mode,
                    p_key_id: &mut key_id,
                },
                type_: raw::NRF_802154_KEY_CLEARTEXT as raw::nrf_802154_key_type_t,
                frame_counter: 0,
                use_global_frame_counter: true,
            };

            let result = unsafe { raw::nrf_802154_security_key_store(&mut entry) };

            if result != raw::NRF_802154_SECURITY_ERROR_NONE as raw::nrf_802154_security_error_t {
                return Err(Error::Security(result));
            }
        }

        Ok(())
    }

    /// Remove all MAC keys from the driver.
    pub fn clear_mac_keys(&mut self) {
        unsafe { raw::nrf_802154_security_key_remove_all() }
    }

    /// Set the global MAC frame counter the driver secures its next enhanced
    /// ACK with - unconditionally, or only if `frame_counter` is larger than
    /// the current one (`if_larger`).
    pub fn set_frame_counter(&mut self, frame_counter: u32, if_larger: bool) {
        unsafe {
            if if_larger {
                raw::nrf_802154_security_global_frame_counter_set_if_larger(frame_counter)
            } else {
                raw::nrf_802154_security_global_frame_counter_set(frame_counter)
            }
        }
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

        crate::platform::refresh_temperature();

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
        ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        let mut buf = [0; MAX_PSDU_SIZE];
        buf[..data.len()].copy_from_slice(data);

        let channel = self.channel();

        self.transmit_with(
            &mut buf[..data.len()],
            FrameProps::PREPARED,
            channel,
            cca,
            ack_buf,
        )
        .await
    }

    /// [`transmit`](Self::transmit), with the driver finishing the frame as
    /// `props` says (see [`FrameProps`]), on `channel`. The frame as it went on
    /// the air is written back into `data`.
    ///
    /// The channel is the frame's own: the receiver stays on (or returns to)
    /// the channel set with [`set_channel`](Self::set_channel) - which a frame
    /// on another channel, e.g. to a CSL peer listening on its CSL channel,
    /// must not move.
    pub async fn transmit_with(
        &mut self,
        data: &mut [u8],
        props: FrameProps,
        channel: u8,
        cca: bool,
        mut ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        DBG_TX_ENTER.fetch_add(1, Ordering::Relaxed);

        crate::platform::refresh_temperature();

        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        if let Some(ack_buf) = ack_buf.as_ref() {
            if ack_buf.len() < MAX_PSDU_SIZE {
                return Err(Error::ReceiveBufTooSmall);
            }
        }

        let (claim, packet_data) = TxClaim::claim(data).await;

        let metadata = raw::nrf_802154_transmit_metadata_t {
            // What is left for the driver to do to the frame: see `FrameProps`.
            frame_props: raw::nrf_802154_transmitted_frame_props_t {
                is_secured: props.is_secured,
                dynamic_data_is_set: props.dynamic_data_is_set,
            },
            cca,
            tx_power: raw::nrf_802154_tx_power_metadata_t {
                use_metadata_value: false,
                power: 0,
            },
            tx_channel: raw::nrf_802154_tx_channel_metadata_t {
                use_metadata_value: true,
                channel,
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
            // Returning drops `claim`, which hands the buffer back: the driver
            // never took the frame, so no completion callback will ever fire
            // for it.
            warn!("nrf_802154 TX could not be scheduled after retries (radio busy)");
            return Err(Error::ScheduleTransmit);
        }

        // The driver has the frame now, and holds our buffer pointer until it
        // reports the outcome. Ownership of the claim passes to its completion
        // callbacks; nothing on this side may release it any more - not even a
        // drop of this future.
        claim.into_driver();

        DBG_TX_SCHED.fetch_add(1, Ordering::Relaxed);

        Self::wait_transmit_done(&mut ack_buf, data).await
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
        ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        let mut buf = [0; MAX_PSDU_SIZE];
        buf[..data.len()].copy_from_slice(data);

        let channel = self.channel();

        self.transmit_csma_ca_with(
            &mut buf[..data.len()],
            FrameProps::PREPARED,
            channel,
            ack_buf,
        )
        .await
    }

    /// [`transmit_csma_ca`](Self::transmit_csma_ca), with the driver finishing
    /// the frame as `props` says (see [`FrameProps`]), on `channel` (see
    /// [`transmit_with`](Self::transmit_with)). The frame as it went on the
    /// air is written back into `data`.
    pub async fn transmit_csma_ca_with(
        &mut self,
        data: &mut [u8],
        props: FrameProps,
        channel: u8,
        mut ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        DBG_TX_ENTER.fetch_add(1, Ordering::Relaxed);

        crate::platform::refresh_temperature();

        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        if let Some(ack_buf) = ack_buf.as_ref() {
            if ack_buf.len() < MAX_PSDU_SIZE {
                return Err(Error::ReceiveBufTooSmall);
            }
        }

        let (claim, packet_data) = TxClaim::claim(data).await;

        let metadata = raw::nrf_802154_transmit_csma_ca_metadata_t {
            // What is left for the driver to do to the frame: see `FrameProps`.
            frame_props: raw::nrf_802154_transmitted_frame_props_t {
                is_secured: props.is_secured,
                dynamic_data_is_set: props.dynamic_data_is_set,
            },
            tx_power: raw::nrf_802154_tx_power_metadata_t {
                use_metadata_value: false,
                power: 0,
            },
            tx_channel: raw::nrf_802154_tx_channel_metadata_t {
                use_metadata_value: true,
                channel,
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
            // Returning drops `claim`, which hands the buffer back: the driver
            // never took the frame, so no completion callback will ever fire
            // for it.
            warn!("nrf_802154 TX could not be scheduled after retries (radio busy)");
            return Err(Error::ScheduleTransmit);
        }

        // The driver has the frame now, and holds our buffer pointer until it
        // reports the outcome. Ownership of the claim passes to its completion
        // callbacks; nothing on this side may release it any more - not even a
        // drop of this future.
        claim.into_driver();

        DBG_TX_SCHED.fetch_add(1, Ordering::Relaxed);

        Self::wait_transmit_done(&mut ack_buf, data).await
    }

    /// [`transmit_with`](Self::transmit_with), at a given time: the frame's
    /// SHR starts at `start_us` (driver clock, see [`now_us`](Self::now_us)),
    /// on `channel`. With `cca`, a single CCA runs right before it - no
    /// backoff - and a busy channel fails the transmission.
    ///
    /// This is how a frame is timed into a peer's receive window, e.g. by a
    /// Thread parent transmitting to a CSL child. `start_us` has to be ahead of
    /// the clock by at least the CCA and the radio ramp-up, or the driver
    /// refuses the frame with [`Error::ScheduleTransmit`].
    pub async fn transmit_at_with(
        &mut self,
        data: &mut [u8],
        props: FrameProps,
        start_us: u64,
        channel: u8,
        cca: bool,
        mut ack_buf: Option<&mut [u8]>,
    ) -> Result<Option<PsduMeta>, Error> {
        DBG_TX_ENTER.fetch_add(1, Ordering::Relaxed);

        crate::platform::refresh_temperature();

        if data.len() > MAX_PSDU_SIZE {
            return Err(Error::TransmitDataTooLarge);
        }

        if let Some(ack_buf) = ack_buf.as_ref() {
            if ack_buf.len() < MAX_PSDU_SIZE {
                return Err(Error::ReceiveBufTooSmall);
            }
        }

        let (claim, packet_data) = TxClaim::claim(data).await;

        let metadata = raw::nrf_802154_transmit_at_metadata_t {
            // What is left for the driver to do to the frame: see `FrameProps`.
            frame_props: raw::nrf_802154_transmitted_frame_props_t {
                is_secured: props.is_secured,
                dynamic_data_is_set: props.dynamic_data_is_set,
            },
            cca,
            channel,
            tx_power: raw::nrf_802154_tx_power_metadata_t {
                use_metadata_value: false,
                power: 0,
            },
            extra_cca_attempts: 0,
            // Requires NRF_802154_TX_TIMESTAMP_PROVIDER_ENABLED (which we don't
            // enable).
            tx_timestamp_encode: false,
        };

        // Unlike an immediate transmission, a timed one does not compete with
        // the operation in progress (the driver schedules it as a timeslot of
        // its own), so there is nothing to retry: a refusal means the time is
        // too close or past, or another timed transmission is pending.
        let err = unsafe { raw::nrf_802154_transmit_raw_at(packet_data, start_us, &metadata) };

        if u32::from(err) != raw::NRF_802154_TX_ERROR_NONE {
            // Returning drops `claim`, which hands the buffer back: the driver
            // never took the frame.
            debug!(
                "nrf_802154 timed TX at {} ({} us ahead) refused ({})",
                start_us,
                start_us as i64 - self.now_us() as i64,
                err
            );
            return Err(Error::ScheduleTransmit);
        }

        // From here on the frame is the driver's, as in `transmit_with`. A drop
        // of this future does not cancel it: it still goes out at its time,
        // and its completion hands the buffer back.
        claim.into_driver();

        DBG_TX_SCHED.fetch_add(1, Ordering::Relaxed);

        Self::wait_transmit_done(&mut ack_buf, data).await
    }

    /// Set how many backoffs CSMA-CA ([`transmit_csma_ca`](Self::transmit_csma_ca))
    /// takes before declaring a busy channel; the driver's default is 4.
    pub fn set_csma_ca_max_backoffs(&mut self, max_backoffs: u8) {
        unsafe { raw::nrf_802154_csma_ca_max_backoffs_set(max_backoffs) }
    }

    async fn wait_transmit_done(
        ack_buf: &mut Option<&mut [u8]>,
        data: &mut [u8],
    ) -> Result<Option<PsduMeta>, Error> {
        let tx_result = RadioState::wait(|state| {
            if state.tx_result.is_some() {
                // The frame as the driver finished it (frame counter, CSL IE,
                // security), for a caller that left any of that to the driver.
                data.copy_from_slice(&state.tx[1..][..data.len()]);
            }

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
                match err {
                    // Routine air-level outcomes - the peer missed the frame,
                    // CSMA-CA lost the channel, or something that was not our
                    // ACK arrived in the ACK window - which the MAC-layer
                    // retransmission policy exists to absorb. Not warnings,
                    // but a burst of them is the first thing to look at when
                    // debugging link quality.
                    //
                    // `InvalidAck` belongs here rather than with the faults
                    // because the driver reports it for anything that turns up
                    // in the ACK window, which it neither address-filters nor
                    // acknowledges: a neighbour transmitting concurrently
                    // lands a perfectly valid non-ACK frame there, and that is
                    // what the overwhelming majority of these are. A genuinely
                    // mismatched ACK - wrong sequence number, or an Enh-Ack
                    // addressing us differently than we sourced the frame -
                    // is reported identically, so a rate that does not track
                    // link load is worth splitting apart at `on_bad_ack` in the
                    // C driver, which is the only place the two are still
                    // distinguishable.
                    TxError::NoAck | TxError::BusyChannel | TxError::InvalidAck => {
                        trace!("nrf_802154 TX failed: {:?}", err);
                    }
                    _ => warn!("nrf_802154 TX failed: {:?}", err),
                }
                Err(Error::Transmit(err))
            }
        }
    }
}

/// An RAII claim on the shared TX buffer.
///
/// The C driver keeps the pointer we hand it for the whole transmission: it
/// reads the PSDU while the frame is on air, and reads it *again* when matching
/// the incoming ACK against the frame that was sent. Refilling the buffer
/// before the transmission is over therefore either tears the outgoing frame or
/// makes the driver compare the peer's ACK against a different frame than the
/// one it acknowledges - which it reports as `TxError::InvalidAck`.
///
/// Hence a guard rather than a claim/release pair: the OpenThread run loop
/// drops `transmit()` futures mid-flight (`select(new_cmd, tx)`) and
/// immediately re-issues the TX, so a claim that is released only on the paths
/// this side actually runs to the end would leak on every cancellation - and a
/// leaked claim wedges every later transmission on the guard, since the frame
/// the driver never took has no completion callback coming to clear it.
///
/// [`into_driver`](Self::into_driver) is the one exit that does *not* release:
/// past that point the driver owns the buffer and its completion callbacks own
/// the release.
struct TxClaim(());

impl TxClaim {
    /// Wait until no transmission is in flight, then fill the shared buffer
    /// with `data` and claim it - all in one critical section, so the flag and
    /// the buffer contents can never disagree.
    async fn claim(data: &[u8]) -> (Self, *mut u8) {
        let packet_data = RadioState::wait(|state| {
            if TX_BUSY.load(Ordering::Acquire) {
                return None;
            }

            state.tx[0] = data.len() as u8 + 2; // + CRC/FCS
            state.tx[1..][..data.len()].copy_from_slice(data);
            state.tx[1 + data.len()] = 0; // CRC placeholder
            state.tx[1 + data.len() + 1] = 0; // CRC placeholder

            // Discard the completion of a previous transmission whose
            // `transmit()` future was dropped before it could consume the
            // result, so this transmission does not pick up that outcome.
            state.status = RadioStatus::Idle;
            state.tx_result = None;

            TX_BUSY.store(true, Ordering::Release);

            let packet_data: &mut [u8] = &mut state.tx;

            Some(packet_data.as_mut_ptr())
        })
        .await;

        (Self(()), packet_data)
    }

    /// Transfer the claim to the C driver, which has accepted the frame.
    ///
    /// Callable only with no `await` between the successful
    /// `nrf_802154_transmit*` call and here, or a cancellation in that gap
    /// would release a buffer the driver is already transmitting from.
    fn into_driver(self) {
        core::mem::forget(self);
    }
}

impl Drop for TxClaim {
    fn drop(&mut self) {
        TX_BUSY.store(false, Ordering::Release);
        // Wake a `claim` parked on the guard - the completion callbacks get
        // this for free from `RadioState::update`.
        STATE_SIGNAL.signal(());
    }
}

/// "The C driver owns the shared TX buffer" flag.
///
/// Set by [`TxClaim::claim`] just before the frame is offered to the driver, and
/// cleared either by dropping the claim or - once the driver has taken the
/// frame - by the TX-completion callbacks. The driver has no "transmission
/// started" callout to hang this on, and in any case ownership begins at
/// `nrf_802154_transmit_raw()`, not at the first symbol on air.
///
/// It is a lock-free atomic rather than a `RadioState` field because the
/// completion callbacks may run in a context above the level the
/// `CriticalSectionRawMutex` masks: touching the `RefCell`-protected
/// `RadioState` there would race with a `STATE.lock()` held by the executor and
/// panic with "already borrowed". It is nonetheless only ever *set* from inside
/// a `STATE.lock()`, so the claim and the buffer fill are one atomic step.
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

        // Not `nrf_802154_deinit`: upstream deprecated it as unsafe to call
        // (nrfxlib 3.4.0). The driver stays initialized for the whole boot
        // and is reset instead.
        reset_driver();
    }
}

/// Whether `nrf_802154_init` has run. The C driver is initialized once per
/// boot; every later `Radio` starts from a [`reset_driver`] instead.
static INITIALIZED: AtomicBool = AtomicBool::new(false);

/// Resets the driver to its post-init defaults with the radio asleep: ongoing
/// and delayed operations cancelled, pending notifications flushed, security
/// keys, pending-bit tables and RX buffers cleared - and drops this crate's
/// view of them along with it.
fn reset_driver() {
    if !unsafe { raw::nrf_802154_reinit() } {
        warn!("nrf_802154 reinit failed (radio busy); the next `Radio` retries");
    }

    TX_BUSY.store(false, Ordering::SeqCst);
    STATE.lock(|state| {
        let mut state = state.borrow_mut();
        state.status = RadioStatus::Idle;
        state.tx_result = None;
        state.rx_queue.clear();
    });
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
            ack_security: None,
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

    /// Drop every queued frame (the diagnostic counters stay).
    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
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
unsafe extern "C" fn nrf_802154_tx_ack_started(p_data: *const u8) {
    // Unlike the notification callbacks (`received_raw`, `cca_done`, ... -
    // deferred to the maskable EGU/SWI priority via
    // `NRF_802154_NOTIFICATION_IMPL=1` in the sys build), this is a *direct*
    // core callout from the high-priority radio IRQ, which the
    // `CriticalSectionRawMutex` does not mask. It MUST NOT touch the
    // `RefCell`-protected `RadioState` - doing so races with a `STATE.lock()`
    // held by the executor and panics with "already borrowed". Hence the
    // atomics: the ACK's security material is picked up by the received-frame
    // notification for the frame this ACK answers, which the driver issues
    // once the ACK is out.
    let phr = unsafe { *p_data };
    let frame = unsafe { core::slice::from_raw_parts(p_data, phr as usize + 1) };

    match parse_ack_security(frame) {
        Some(sec) => {
            ACK_SEC_FRAME_COUNTER.store(sec.frame_counter, Ordering::Relaxed);
            ACK_SEC_KEY_ID.store(sec.key_id, Ordering::Relaxed);
            ACK_SEC_PENDING.store(true, Ordering::Release);
        }
        None => ACK_SEC_PENDING.store(false, Ordering::Release),
    }
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
                    ack_security: take_ack_security(),
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
                    ack_security: take_ack_security(),
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
unsafe extern "C" fn nrf_802154_receive_failed(error: raw::nrf_802154_rx_error_t, id: u32) {
    // A frame-level reception failure (CRC error, invalid frame, abort, ...). With
    // rx_on_when_idle the radio stays in RX, so we just drop the failed reception
    // and let `receive()` keep waiting for the next good frame, rather than
    // surfacing transient RX noise to OpenThread as a receive error.
    //
    // A timed window (`receive_at`) ends through here too: `DELAYED_TIMEOUT`
    // is its normal, frame-less end; `DELAYED_TIMESLOT_DENIED` means the
    // window was never opened (the scheduler refused the timeslot).
    let delayed_timeout = raw::NRF_802154_RX_ERROR_DELAYED_TIMEOUT as raw::nrf_802154_rx_error_t;
    let delayed_denied =
        raw::NRF_802154_RX_ERROR_DELAYED_TIMESLOT_DENIED as raw::nrf_802154_rx_error_t;
    let delayed_aborted = raw::NRF_802154_RX_ERROR_DELAYED_ABORTED as raw::nrf_802154_rx_error_t;

    if id == RX_WINDOW_ID && error == delayed_timeout {
        RX_WINDOW_SCHEDULED.store(false, Ordering::Relaxed);
        RX_WINDOW_ENDED.store(true, Ordering::Relaxed);
        STATE_SIGNAL.signal(());
        trace!("nrf_802154 timed receive window ended without a frame");
    } else if id == RX_WINDOW_ID && (error == delayed_denied || error == delayed_aborted) {
        RX_WINDOW_SCHEDULED.store(false, Ordering::Relaxed);
        RX_WINDOW_ENDED.store(true, Ordering::Relaxed);
        STATE_SIGNAL.signal(());
        debug!("nrf_802154 timed receive window failed: error {}", error);
    } else {
        // A frame that failed inside a window (or in plain RX): the window
        // itself goes on.
        trace!("nrf_802154 receive failed: error {}", error);
    }
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_transmitted_raw(
    _p_frame: *mut u8,
    p_metadata: *const raw::nrf_802154_transmit_done_metadata_t,
) {
    // The transmission is over: the driver is done with the shared TX buffer,
    // so release the claim handed to it by `TxClaim::into_driver`. Done here in
    // the completion callback (which always fires once the driver has accepted
    // a frame), not in `wait_transmit_done` — under load the run loop's
    // `select(new_cmd, transmit)` can drop the `transmit()` future mid-flight, so
    // `wait_transmit_done` may never run. A leaked `TX_BUSY` would otherwise stall
    // the next `transmit()` forever on its guard, wedging the run loop.
    TX_BUSY.store(false, Ordering::Release);

    RadioState::update(|state| {
        let p_metadata = unsafe { p_metadata.as_ref().unwrap() };
        if !p_metadata.data.transmitted.p_ack.is_null() {
            // `length` is the ACK's PSDU length (the PHR value, FCS included),
            // like a received frame's PHR; the buffer carries the PHR byte in
            // front of it. Same layout as in `nrf_802154_received_timestamp_raw`.
            let phr = p_metadata.data.transmitted.length;
            let total = phr as usize + 1;

            if phr >= MIN_PHR && total <= MAX_PACKET_SIZE {
                let packet = unsafe {
                    core::slice::from_raw_parts(p_metadata.data.transmitted.p_ack, total)
                };

                state.ack_rx[..total].copy_from_slice(packet);

                state.tx_result = Some(TxResult::Done(Some(PsduMeta {
                    len: phr - 2, // PHR value - FCS
                    crc: u16::from_le_bytes([state.ack_rx[total - 2], state.ack_rx[total - 1]]),
                    power: p_metadata.data.transmitted.power,
                    lqi: Some(p_metadata.data.transmitted.lqi),
                    time: Some(p_metadata.data.transmitted.time),
                    // An ACK we received, not one we sent.
                    ack_security: None,
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
    // Release the shared TX buffer — see the note in `transmitted_raw`.
    TX_BUSY.store(false, Ordering::Release);

    RadioState::update(|state| {
        state.tx_result = Some(TxResult::Failed(error));
    });
}

#[no_mangle]
unsafe extern "C" fn nrf_802154_custom_part_of_radio_init() {}
