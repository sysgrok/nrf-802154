//! Chip-specific pieces of the radio driver for the nRF54L series: which
//! peripherals and interrupts the 802.15.4 platform layer needs here.

use embassy_nrf::interrupt::typelevel::Binding;
use embassy_nrf::Peri;

use crate::platform::{
    CcmInterruptHandler, CcmIrq, EguInterruptHandler, EguIrq, LpTimerInterruptHandler, LpTimerIrq,
};

/// Compile-time proof that every interrupt the driver needs has been bound with
/// [`embassy_nrf::bind_interrupts!`].
///
/// Blanket-implemented; you never implement this yourself. The required set is
/// chip-specific, which is why it lives in a trait rather than in inline bounds
/// on [`Radio::new`](super::Radio::new).
pub trait InterruptBindings:
    Binding<LpTimerIrq, LpTimerInterruptHandler>
    + Binding<EguIrq, EguInterruptHandler>
    + Binding<CcmIrq, CcmInterruptHandler>
{
}

impl<T> InterruptBindings for T where
    T: Binding<LpTimerIrq, LpTimerInterruptHandler>
        + Binding<EguIrq, EguInterruptHandler>
        + Binding<CcmIrq, CcmInterruptHandler>
{
}

/// Peripherals owned by the 802.15.4 platform layer.
///
/// Constructing this proves at compile time that the application is not using
/// any of the peripherals the driver needs. They are only ever taken, never
/// touched through these handles — the driver reaches them through the PAC.
///
/// The set looks nothing like the nRF52/nRF53 one because the time base is the
/// GRTC rather than an RTC, and because the GRTC lives in a different peripheral
/// domain from the radio: carrying frame timestamps and hardware-timed radio
/// tasks between the two costs a PPIB channel pair and a DPPIC20 channel in each
/// direction. There is no high-precision timer to claim — the SL timestamps off
/// the GRTC directly.
///
/// The EGU instance (`EGU10`) is missing for a duller reason: embassy-nrf 0.11
/// does not expose EGU singletons on this series, so there is nothing to take.
pub struct RadioPeripherals<'d> {
    /// GRTC channel for the LP timer's scheduled fire.
    pub grtc_ch3: Peri<'d, embassy_nrf::peripherals::GRTC_CH3>,
    /// GRTC channel driving hardware-timed radio tasks (delayed TX/RX).
    pub grtc_ch4: Peri<'d, embassy_nrf::peripherals::GRTC_CH4>,
    /// GRTC channel latching frame timestamps.
    pub grtc_ch5: Peri<'d, embassy_nrf::peripherals::GRTC_CH5>,
    /// DPPIC20 channel carrying bridged radio events to the GRTC.
    pub ppi20_ch2: Peri<'d, embassy_nrf::peripherals::PPI20_CH2>,
    /// DPPIC20 channel carrying GRTC compares towards the radio domain.
    pub ppi20_ch3: Peri<'d, embassy_nrf::peripherals::PPI20_CH3>,
    /// PPIB11 -> PPIB21 channel: radio domain to peripheral domain (timestamps).
    pub ppib11_ch1: Peri<'d, embassy_nrf::peripherals::PPIB11_CH1>,
    /// PPIB11 -> PPIB21 channel: radio domain to peripheral domain (timestamps).
    pub ppib21_ch1: Peri<'d, embassy_nrf::peripherals::PPIB21_CH1>,
    /// PPIB21 -> PPIB11 channel: peripheral domain to radio domain (hw tasks).
    pub ppib11_ch2: Peri<'d, embassy_nrf::peripherals::PPIB11_CH2>,
    /// PPIB21 -> PPIB11 channel: peripheral domain to radio domain (hw tasks).
    pub ppib21_ch2: Peri<'d, embassy_nrf::peripherals::PPIB21_CH2>,
}

impl<'d> RadioPeripherals<'d> {
    /// Creates a new `RadioPeripherals` instance.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        grtc_ch3: Peri<'d, embassy_nrf::peripherals::GRTC_CH3>,
        grtc_ch4: Peri<'d, embassy_nrf::peripherals::GRTC_CH4>,
        grtc_ch5: Peri<'d, embassy_nrf::peripherals::GRTC_CH5>,
        ppi20_ch2: Peri<'d, embassy_nrf::peripherals::PPI20_CH2>,
        ppi20_ch3: Peri<'d, embassy_nrf::peripherals::PPI20_CH3>,
        ppib11_ch1: Peri<'d, embassy_nrf::peripherals::PPIB11_CH1>,
        ppib21_ch1: Peri<'d, embassy_nrf::peripherals::PPIB21_CH1>,
        ppib11_ch2: Peri<'d, embassy_nrf::peripherals::PPIB11_CH2>,
        ppib21_ch2: Peri<'d, embassy_nrf::peripherals::PPIB21_CH2>,
    ) -> Self {
        Self {
            grtc_ch3,
            grtc_ch4,
            grtc_ch5,
            ppi20_ch2,
            ppi20_ch3,
            ppib11_ch1,
            ppib21_ch1,
            ppib11_ch2,
            ppib21_ch2,
        }
    }
}
