//! Receive all IEEE 802.15.4 frames (promiscuous mode).
//!
//! This example listens on channel 15 for all frames regardless of addressing,
//! and prints metadata about each received frame.

#![no_std]
#![no_main]

use defmt::info;
use defmt_rtt as _;
use embassy_executor::Spawner;
use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals as MpslPeripherals};
use nrf52840_examples::Irqs;
use panic_probe as _;
use static_cell::StaticCell;

const CHANNEL: u8 = 15;

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
        rc_ctiv: 16,
        rc_temp_ctiv: 2,
        accuracy_ppm: nrf_mpsl::raw::MPSL_CLOCK_LF_ACCURACY_500_PPM as u16,
    };

    let mpsl_p = MpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.must_spawn(mpsl_task(mpsl));

    let mut radio = Radio::new(p.RADIO, p.EGU0, Irqs, mpsl, p.TIMER2, p.RTC2);
    radio.set_channel(CHANNEL);
    radio.set_promiscuous(true);

    info!("Start receiving all frames on channel {}", CHANNEL);

    let mut buf = [0u8; nrf_802154::MAX_PSDU_SIZE];
    loop {
        match radio.receive(&mut buf).await {
            Ok(meta) => {
                info!(
                    "Received frame: {} bytes, power {}dBm, LQI {:?}",
                    meta.len,
                    meta.power,
                    meta.lqi
                );
            }
            Err(e) => {
                info!("Receive error: {:?}", e);
            }
        }
    }
}
