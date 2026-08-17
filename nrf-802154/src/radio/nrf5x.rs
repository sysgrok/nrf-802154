//! Chip-specific pieces of the radio driver for nRF52 and nRF5340-net: which
//! peripherals and interrupts the 802.15.4 platform layer needs here.

use embassy_nrf::interrupt::typelevel::Binding;
use embassy_nrf::Peri;

use crate::platform::{EguInterruptHandler, EguIrq, LpTimerInterruptHandler, LpTimerIrq};

/// Compile-time proof that every interrupt the driver needs has been bound with
/// [`embassy_nrf::bind_interrupts!`].
///
/// Blanket-implemented; you never implement this yourself. The required set is
/// chip-specific, which is why it lives in a trait rather than in inline bounds
/// on [`Radio::new`](super::Radio::new).
pub trait InterruptBindings:
    Binding<LpTimerIrq, LpTimerInterruptHandler> + Binding<EguIrq, EguInterruptHandler>
{
}

impl<T> InterruptBindings for T where
    T: Binding<LpTimerIrq, LpTimerInterruptHandler> + Binding<EguIrq, EguInterruptHandler>
{
}

/// Peripherals owned by the 802.15.4 platform layer.
///
/// Constructing this proves at compile time that the application is not using
/// any of the peripherals the driver needs. They are only ever taken, never
/// touched through these handles — the driver reaches them through the PAC.
pub struct RadioPeripherals<'d> {
    /// The EGU instance the C driver raises notifications on. The interrupt
    /// line is `EGU0_SWI0` on nRF52 and `EGU0` on nRF5340-net.
    pub egu: Peri<'d, embassy_nrf::peripherals::EGU0>,
    /// High-precision (1 µs) timer. `TIMER0` is reserved by MPSL and `TIMER1`
    /// may be used by MPSL on nRF53, so this is `TIMER2`.
    pub hp_timer: Peri<'d, embassy_nrf::peripherals::TIMER2>,
    /// Low-power timer. `RTC0` is reserved by MPSL in all cases.
    #[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
    pub lp_timer: Peri<'d, embassy_nrf::peripherals::RTC2>,
    /// Low-power timer. `RTC0` is reserved by MPSL in all cases.
    ///
    /// **Note:** chips without an `RTC2` fall back to `RTC1`, which is also
    /// embassy-nrf's default time driver. On those chips, point embassy's time
    /// driver at a different peripheral or disable it.
    #[cfg(not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840")))]
    pub lp_timer: Peri<'d, embassy_nrf::peripherals::RTC1>,
}

impl<'d> RadioPeripherals<'d> {
    /// Creates a new `RadioPeripherals` instance.
    pub fn new(
        egu: Peri<'d, embassy_nrf::peripherals::EGU0>,
        hp_timer: Peri<'d, embassy_nrf::peripherals::TIMER2>,
        #[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
        lp_timer: Peri<'d, embassy_nrf::peripherals::RTC2>,
        #[cfg(not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840")))]
        lp_timer: Peri<'d, embassy_nrf::peripherals::RTC1>,
    ) -> Self {
        Self {
            egu,
            hp_timer,
            lp_timer,
        }
    }
}
