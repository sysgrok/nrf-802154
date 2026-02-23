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
use nrf52840_examples::Irqs;
use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals as MpslPeripherals};
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

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_WORST_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: false,
    };

    let mpsl_p = MpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.must_spawn(mpsl_task(mpsl));

    let mut radio = Radio::new(p.RADIO, p.EGU0, Irqs, mpsl, p.TIMER2, p.RTC2);
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
