//! Scratch diagnostic: how much lead does `receive_at` need from an idle radio?
//!
//! For a range of leads, schedules timed receive windows from a sleeping radio
//! (HFXO off) and counts how many the driver accepts - first with MPSL's default
//! HFCLK latency, then with the typical one.

#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;
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

    for latency in [
        None,
        Some(nrf_mpsl::raw::MPSL_CLOCK_HF_LATENCY_TYPICAL as u16),
    ] {
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
