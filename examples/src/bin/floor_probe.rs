//! Scratch diagnostic: the sleep floor with MPSL and the 802.15.4 driver initialized and used.
//!
//! `rx_at_lead` (timed receive windows from a sleeping radio), with the XIAO's QSPI flash put into
//! deep power-down first, then idle.
//!
//! For a range of leads, schedules timed receive windows from a sleeping radio
//! (HFXO off) and counts how many the driver accepts - first with MPSL's default
//! HFCLK latency, then with the typical one.

#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_802154_examples::Irqs;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

const CHANNEL: u8 = 26;
const TRIALS: usize = 10;
const LEADS_US: [u64; 13] = [
    500, 1000, 1500, 1800, 2000, 2100, 2200, 2300, 2400, 2600, 3000, 3500, 4000,
];

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = nrf_802154_examples::init();

    // The XIAO's QSPI flash: deep power-down (0xB9), bit-banged in SPI mode 0, then keep its
    // inputs driven.
    {
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
    }

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 20,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = nrf_802154_examples::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let mut radio_p = nrf_802154_examples::radio_peripherals!(p);
    let mut radio = Radio::new(p.RADIO.reborrow(), radio_p.reborrow(), Irqs, mpsl);
    radio.set_channel(CHANNEL);

    // MPSL's default HFCLK latency only: setting it at runtime asserts inside MPSL
    for latency in [None::<u16>] {
        if let Some(latency) = latency {
            let ret = unsafe { nrf_mpsl::raw::mpsl_clock_hfclk_latency_set(latency) };
            info!("HFCLK latency set to {} us -> {}", latency, ret);
        } else {
            info!("HFCLK latency: MPSL default");
        }

        for lead in LEADS_US {
            let mut accepted = 0;

            for _ in 0..TRIALS {
                radio.sleep();
                // Long enough for MPSL to release the HFXO
                Timer::after_millis(50).await;

                let start = radio.now_us() + lead;
                if radio.receive_at(start, 2000, CHANNEL) {
                    accepted += 1;
                }

                Timer::after_micros(lead + 2000 + 5000).await;
                radio.receive_at_cancel();
            }

            info!("lead {} us: {}/{} accepted", lead, accepted, TRIALS);
        }
    }

    info!("RX_AT_LEAD DONE");

    loop {
        Timer::after_secs(1).await;
    }
}
