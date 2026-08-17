//! Receive a single IEEE 802.15.4 frame.
//!
//! This example listens on channel 15 for frames addressed to PAN 0x4242 and
//! short address 0x2323, then prints metadata about each received frame.
//!
//! Pair with `send_frame` running on another nRF52840 to see received frames.

#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_802154_examples::Irqs;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

const CHANNEL: u8 = 15;
const PAN_ID: u16 = 0x4242;
const SHORT_ADDR: u16 = 0x2323;

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
//
// If the firmware would not use or need heap allocation for other purposes, this could be replaced
// with stub impls of `calloc` and `free` that panic with `unimplemented!()`,
// and the `#[global_allocator]` attribute could be removed.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    info!("boot");
    let p = nrf_802154_examples::init();
    info!("embassy init done");

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = nrf_802154_examples::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    info!("mpsl up");
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let mut radio = Radio::new(
        p.RADIO,
        nrf_802154_examples::radio_peripherals!(p),
        Irqs,
        mpsl,
    );
    info!("radio up");
    radio.set_channel(CHANNEL);
    radio.set_pan_id(Some(PAN_ID));
    radio.set_short_addr(Some(SHORT_ADDR));

    info!(
        "Start receiving on channel {}, PAN 0x{:04x}, addr 0x{:04x}",
        CHANNEL, PAN_ID, SHORT_ADDR
    );

    let mut buf = [0u8; nrf_802154::MAX_PSDU_SIZE];
    loop {
        match radio.receive(&mut buf).await {
            Ok(meta) => {
                info!(
                    "Received frame: {} bytes, power {}dBm, LQI {:?}",
                    meta.len, meta.power, meta.lqi
                );
            }
            Err(e) => {
                info!("Receive error: {:?}", e);
            }
        }
    }
}
