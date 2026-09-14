//! Drop the radio and bring it up again, repeatedly.
//!
//! Exercises the driver reset that `Radio`'s destructor performs (and that a
//! subsequent `Radio::new` starts from): every cycle creates a radio, sends a
//! frame, leaves the radio *receiving*, and drops it - alternately straight out
//! of RX and after an explicit sleep. The next cycle must then find the driver
//! back at its defaults and fully working.
//!
//! Self-contained: the frame is a broadcast without ACK request, so no peer is
//! needed, and channel 26 keeps it out of the way of anything else on the air.

#![no_std]
#![no_main]

use defmt::{assert, assert_eq, info, panic};

use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_time::Timer;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_802154_examples::{build_data_frame, Irqs};
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

const CHANNEL: u8 = 26;
const PAN_ID: u16 = 0x4242;
const SHORT_ADDR: u16 = 0x2323;
const CYCLES: usize = 20;

/// The channel the driver comes up on after `nrf_802154_init` (and, being a
/// driver default, after every reset).
const DEFAULT_CHANNEL: u8 = 11;

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = nrf_802154_examples::init();

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = nrf_802154_examples::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let mut radio_p = nrf_802154_examples::radio_peripherals!(p);

    let mut frame = [0u8; nrf_802154::MAX_PSDU_SIZE];
    let mut rx = [0u8; nrf_802154::MAX_PSDU_SIZE];

    for cycle in 0..CYCLES {
        let mut radio = Radio::new(p.RADIO.reborrow(), radio_p.reborrow(), Irqs, mpsl);

        // Whatever the previous instance configured is gone
        assert_eq!(
            radio.channel(),
            DEFAULT_CHANNEL,
            "cycle {}: the driver did not come up at its defaults",
            cycle
        );

        radio.set_channel(CHANNEL);
        radio.set_pan_id(Some(PAN_ID));
        radio.set_short_addr(Some(SHORT_ADDR));

        let len = build_data_frame(
            cycle as u8,
            PAN_ID,
            0xffff,
            SHORT_ADDR,
            false,
            b"reinit",
            &mut frame,
        )
        .unwrap();
        if let Err(e) = radio.transmit(&frame[..len], true, None).await {
            panic!("cycle {}: transmit failed: {:?}", cycle, e);
        }

        // Leave the radio receiving; the reset has to pull it out of RX (and
        // out of its MPSL timeslot) on its own
        match select(radio.receive(&mut rx), Timer::after_millis(50)).await {
            Either::First(Ok(meta)) => info!("cycle {}: stray frame, {} bytes", cycle, meta.len),
            Either::First(Err(e)) => panic!("cycle {}: receive failed: {:?}", cycle, e),
            Either::Second(()) => {}
        }

        if cycle % 2 == 1 {
            // Every other cycle: hand over an idle radio instead
            assert!(radio.sleep(), "cycle {}: could not sleep", cycle);
        }

        drop(radio);
        info!("cycle {}: ok", cycle);
    }

    info!("REINIT TEST PASSED: {} cycles", CYCLES);

    loop {
        Timer::after_secs(1).await;
    }
}
