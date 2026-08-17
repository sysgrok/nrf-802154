//! Shared plumbing for the `cli_node` test firmware.

#![no_std]

use embassy_nrf::bind_interrupts;

pub mod console;
pub mod settings;

bind_interrupts!(pub struct Irqs {
    // MPSL low-priority and 802.15.4 EGU handler share EGU0_SWI0
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler, nrf_802154::Egu0InterruptHandler;
    // MPSL high-priority handlers
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    // MPSL clock handler (this is why the USB console uses
    // `SoftwareVbusDetect`: `CLOCK_POWER` belongs to MPSL here)
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    // 802.15.4 LP timer (RTC2 on nRF52840)
    RTC2 => nrf_802154::LpTimerInterruptHandler;
    // The USB console
    USBD => embassy_nrf::usb::InterruptHandler<embassy_nrf::peripherals::USBD>;
});
