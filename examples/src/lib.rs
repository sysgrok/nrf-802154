//! Shared initialization helpers for the IEEE 802.15.4 examples.
//!
//! The examples themselves are chip-agnostic; everything that differs between
//! the nRF52840 and the nRF54L parts — the interrupt bindings and which peripherals
//! MPSL and the 802.15.4 driver claim — is collected here.

#![no_std]

use embassy_nrf::bind_interrupts;

#[cfg(feature = "nrf52840")]
bind_interrupts!(pub struct Irqs {
    // MPSL low-priority and 802.15.4 EGU handler share EGU0_SWI0
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler, nrf_802154::EguInterruptHandler;
    // MPSL high-priority handlers
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    // MPSL clock handler
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    // 802.15.4 LP timer (RTC2 on nRF52840)
    RTC2 => nrf_802154::LpTimerInterruptHandler;
});

#[cfg(feature = "_nrf54l")]
bind_interrupts!(pub struct Irqs {
    // MPSL low-priority handler. Unlike nRF52, nothing is shared with the
    // 802.15.4 driver here: it has its own EGU instance (EGU10).
    SWI00 => nrf_mpsl::LowPrioInterruptHandler;
    // MPSL high-priority handlers
    RADIO_0 => nrf_mpsl::HighPrioInterruptHandler;
    TIMER10 => nrf_mpsl::HighPrioInterruptHandler;
    GRTC_3 => nrf_mpsl::HighPrioInterruptHandler;
    // MPSL clock handler
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    // 802.15.4 notifications
    EGU10 => nrf_802154::EguInterruptHandler;
    // 802.15.4 LP timer (GRTC interrupt group 0)
    GRTC_0 => nrf_802154::LpTimerInterruptHandler;
    // 802.15.4 frame encryption offload
    AAR00_CCM00 => nrf_802154::CcmInterruptHandler;
});

/// Initialize embassy-nrf with a configuration MPSL and the 802.15.4 driver can
/// live with.
///
/// On nRF54L that means forcing the 128 MHz PLL: embassy-nrf defaults to 64 MHz,
/// but MPSL asserts on 128 MHz at startup, and 128 MHz is the only frequency the
/// 802.15.4 driver supports on this series.
pub fn init() -> embassy_nrf::Peripherals {
    #[cfg(feature = "_nrf54l")]
    scrub_grtc();

    #[allow(unused_mut)]
    let mut config: embassy_nrf::config::Config = Default::default();
    #[cfg(feature = "_nrf54l")]
    {
        config.clock_speed = embassy_nrf::config::ClockSpeed::CK128;
    }
    embassy_nrf::init(config)
}

/// Drop any GRTC interrupt state inherited from the firmware that ran before us.
///
/// The GRTC is in the always-on domain, so a soft reset — which is what a
/// debugger's `SYSRESETREQ` and therefore `probe-rs run` issues — leaves its
/// compare events latched and its per-domain interrupt enables set. The NVIC
/// *is* reset, so nothing fires until someone re-enables the line; embassy's
/// GRTC time driver does exactly that at the end of `embassy_nrf::init`.
///
/// From there an inherited enable is fatal: embassy's ISR only ever clears its
/// own channel (CC[11]), so a stale event on any other channel of the same
/// domain re-fires the instant the handler returns, and `init` never comes back.
/// Boards that shipped with Zephyr or Arduino hit this on the first flash,
/// because nrfx hands out GRTC channels from CC[0] upwards.
///
/// Cheap insurance, and it has to happen before `embassy_nrf::init`.
#[cfg(feature = "_nrf54l")]
fn scrub_grtc() {
    let r = embassy_nrf::pac::GRTC;

    // Every domain, not just ours: at this point in boot nothing else — not
    // MPSL, not the time driver — has claimed one yet.
    for group in 0..4 {
        r.intenclr(group).write(|w| w.0 = u32::MAX);
    }
    for cc in 0..12 {
        r.events_compare(cc).write_value(0);
    }
}

/// Claim the peripherals MPSL needs out of `embassy_nrf::init`'s `Peripherals`.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! mpsl_peripherals {
    ($p:expr) => {
        nrf_mpsl::Peripherals::new(
            $p.RTC0,
            $p.TIMER0,
            $p.TEMP,
            $p.PPI_CH19,
            $p.PPI_CH30,
            $p.PPI_CH31,
        )
    };
}

/// Claim the peripherals MPSL needs out of `embassy_nrf::init`'s `Peripherals`.
#[cfg(feature = "_nrf54l")]
#[macro_export]
macro_rules! mpsl_peripherals {
    ($p:expr) => {
        nrf_mpsl::Peripherals::new(
            $p.GRTC_CH7,
            $p.GRTC_CH8,
            $p.GRTC_CH9,
            $p.GRTC_CH10,
            $p.GRTC_CH11,
            $p.TIMER10,
            $p.TIMER20,
            $p.TEMP,
            $p.PPI10_CH0,
            $p.PPI20_CH1,
            $p.PPIB11_CH0,
            $p.PPIB21_CH0,
        )
    };
}

/// The blocking hardware RNG. nRF54L has no standalone `RNG` peripheral; its
/// entropy comes out of the CRACEN crypto engine instead.
#[cfg(feature = "nrf52840")]
pub type Rng<'d> = embassy_nrf::rng::Rng<'d, embassy_nrf::mode::Blocking>;

/// The blocking hardware RNG. nRF54L has no standalone `RNG` peripheral; its
/// entropy comes out of the CRACEN crypto engine instead.
#[cfg(feature = "_nrf54l")]
pub type Rng<'d> = embassy_nrf::cracen::Cracen<'d, embassy_nrf::mode::Blocking>;

/// Claim the hardware RNG out of `embassy_nrf::init`'s `Peripherals`.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! rng {
    ($p:expr) => {
        embassy_nrf::rng::Rng::new_blocking($p.RNG)
    };
}

/// Claim the hardware RNG out of `embassy_nrf::init`'s `Peripherals`.
#[cfg(feature = "_nrf54l")]
#[macro_export]
macro_rules! rng {
    ($p:expr) => {
        embassy_nrf::cracen::Cracen::new_blocking($p.CRACEN)
    };
}

/// Claim the peripherals the 802.15.4 driver needs out of `embassy_nrf::init`'s
/// `Peripherals`.
#[cfg(feature = "nrf52840")]
#[macro_export]
macro_rules! radio_peripherals {
    ($p:expr) => {
        nrf_802154::RadioPeripherals::new($p.EGU0, $p.TIMER2, $p.RTC2)
    };
}

/// Claim the peripherals the 802.15.4 driver needs out of `embassy_nrf::init`'s
/// `Peripherals`.
#[cfg(feature = "_nrf54l")]
#[macro_export]
macro_rules! radio_peripherals {
    ($p:expr) => {
        nrf_802154::RadioPeripherals::new(
            $p.GRTC_CH3,
            $p.GRTC_CH4,
            $p.GRTC_CH5,
            $p.PPI20_CH2,
            $p.PPI20_CH3,
            $p.PPIB11_CH1,
            $p.PPIB21_CH1,
            $p.PPIB11_CH2,
            $p.PPIB21_CH2,
        )
    };
}

/// MAC header size in bytes: FCF (2) + seq (1) + dest PAN (2) + dest addr (2) + src addr (2).
const MAC_HEADER_LEN: usize = 9;

/// Build an IEEE 802.15.4 Data frame with short addressing and PAN ID compression.
///
/// # Arguments
/// - `seq`: Sequence number
/// - `dst_pan`: Destination PAN ID
/// - `dst_addr`: Destination short address
/// - `src_addr`: Source short address
/// - `ack`: Whether to request an ACK
/// - `payload`: Frame payload (MSDU)
/// - `buf`: Buffer to write the frame into
///
/// # Returns
/// `Some(len)` with the number of bytes written to `buf`, or `None` if the payload
/// exceeds `MAX_PSDU_SIZE` or the buffer is too small to hold the frame.
pub fn build_data_frame(
    seq: u8,
    dst_pan: u16,
    dst_addr: u16,
    src_addr: u16,
    ack: bool,
    payload: &[u8],
    buf: &mut [u8],
) -> Option<usize> {
    let frame_len = MAC_HEADER_LEN + payload.len();
    if frame_len > nrf_802154::MAX_PSDU_SIZE || frame_len > buf.len() {
        return None;
    }

    // Frame Control Field:
    //   bits  0-2 : Frame Type = 001 (Data)
    //   bit     5 : ACK Request
    //   bit     6 : PAN ID Compression (source PAN omitted, same as dest PAN)
    //   bits 10-11: Destination Addressing Mode = 10 (short)
    //   bits 12-13: Frame Version = 00 (IEEE 802.15.4-2003)
    //   bits 14-15: Source Addressing Mode = 10 (short)
    let fcf: u16 = 0x0001              // Data frame type
        | if ack { 0x0020 } else { 0 } // ACK request
        | 0x0040                       // PAN ID Compression
        | 0x0800                       // Dst Addressing Mode: short (bit 11 = 1)
        | 0x8000; // Src Addressing Mode: short (bit 15 = 1)

    buf[0] = (fcf & 0xFF) as u8;
    buf[1] = (fcf >> 8) as u8;
    buf[2] = seq;
    buf[3] = (dst_pan & 0xFF) as u8;
    buf[4] = (dst_pan >> 8) as u8;
    buf[5] = (dst_addr & 0xFF) as u8;
    buf[6] = (dst_addr >> 8) as u8;
    // Source PAN ID is omitted due to PAN ID Compression
    buf[7] = (src_addr & 0xFF) as u8;
    buf[8] = (src_addr >> 8) as u8;
    buf[MAC_HEADER_LEN..frame_len].copy_from_slice(payload);
    Some(frame_len)
}
