//! Send IEEE 802.15.4 broadcast data frames.
//!
//! This example sends a "Hello World" data frame every second to the broadcast
//! address on PAN 0x4242, channel 15. No ACK is requested.
//!
//! Pair with `receive_all_frames` or `sniffer` running on another nRF52840
//! to observe the frames.

#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use nrf802154_examples::{build_data_frame, Irqs};
use nrf_802154::Radio;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::{MultiprotocolServiceLayer, Peripherals as MpslPeripherals};
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

const CHANNEL: u8 = 15;
const PAN_ID: u16 = 0x4242;
const SRC_SHORT_ADDR: u16 = 0x2323;
const DST_SHORT_ADDR: u16 = 0xFFFF; // broadcast
const PAYLOAD: &[u8] = b"Hello World";

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
    spawner.must_spawn(mpsl_task(mpsl));

    let mut radio = Radio::new(p.RADIO, p.EGU0, Irqs, mpsl, p.TIMER2, p.RTC2);
    radio.set_channel(CHANNEL);
    radio.set_pan_id(Some(PAN_ID));
    radio.set_short_addr(Some(SRC_SHORT_ADDR));

    let mut seq_number = 0u8;
    let mut frame_buf = [0u8; nrf_802154::MAX_PSDU_SIZE];

    loop {
        if let Some(frame_len) = build_data_frame(
            seq_number,
            PAN_ID,
            DST_SHORT_ADDR,
            SRC_SHORT_ADDR,
            false, // no ACK for broadcast
            PAYLOAD,
            &mut frame_buf,
        ) {
            match radio.transmit(&frame_buf[..frame_len], true, None).await {
                Ok(_) => info!("Send frame with sequence number {}", seq_number),
                Err(e) => info!("Transmit error: {:?}", e),
            }
        }

        seq_number = seq_number.wrapping_add(1);
        Timer::after(Duration::from_millis(1000)).await;
    }
}
