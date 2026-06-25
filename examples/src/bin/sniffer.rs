//! IEEE 802.15.4 packet sniffer.
//!
//! This example captures all IEEE 802.15.4 frames on a given channel (default: 15)
//! and prints the raw frame bytes via defmt/RTT. Run with a RTT viewer to see the output.
//!
//! Similar to `receive_all_frames` but outputs raw frame bytes suited for analysis.

#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;

use nrf_802154::Radio;
use nrf_802154_examples::Irqs;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals as MpslPeripherals};

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

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
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = MpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let mut radio = Radio::new(p.RADIO, p.EGU0, Irqs, mpsl, p.TIMER2, p.RTC2);
    radio.set_channel(CHANNEL);
    radio.set_promiscuous(true);

    info!("Sniffer started on channel {}", CHANNEL);

    let mut buf = [0u8; nrf_802154::MAX_PSDU_SIZE];
    loop {
        match radio.receive(&mut buf).await {
            Ok(meta) => {
                info!("@RAW {:?}", &buf[..meta.len as usize]);
            }
            Err(e) => {
                info!("Receive error: {:?}", e);
            }
        }
    }
}
