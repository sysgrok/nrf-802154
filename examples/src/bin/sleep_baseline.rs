//! Scratch diagnostic: the sleep floor of a XIAO nRF52840 with nothing running.
//!
//! The 32 kHz crystal and embassy's RTC time driver run, the onboard QSPI flash is put into
//! deep power-down, and the core sleeps forever - no radio, no MPSL, no logging.

#![no_std]
#![no_main]

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;

use embedded_alloc::LlffHeap as Heap;

use {defmt_rtt as _, panic_probe as _};

// The critical-section implementation the examples use.
use nrf_mpsl as _;

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let mut config = embassy_nrf::config::Config::default();
    config.lfclk_source = embassy_nrf::config::LfclkSource::ExternalXtal;

    let p = embassy_nrf::init(config);

    // The XIAO's QSPI flash: deep power-down (0xB9), bit-banged in SPI mode 0, then keep its
    // inputs driven.
    const DEEP_POWER_DOWN: u8 = 0xb9;

    let mut csn = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
    let mut sck = Output::new(p.P0_21, Level::Low, OutputDrive::Standard);
    let mut si = Output::new(p.P0_20, Level::Low, OutputDrive::Standard);
    let wp = Output::new(p.P0_22, Level::High, OutputDrive::Standard);
    let hold = Output::new(p.P0_23, Level::High, OutputDrive::Standard);

    csn.set_low();

    for bit in (0..8).rev() {
        si.set_level(Level::from(DEEP_POWER_DOWN & (1 << bit) != 0));
        sck.set_high();
        sck.set_low();
    }

    csn.set_high();

    for pin in [csn, sck, si, wp, hold] {
        pin.persist();
    }

    loop {
        Timer::after_secs(3600).await;
    }
}
