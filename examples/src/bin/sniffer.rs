//! IEEE 802.15.4 packet sniffer.
//!
//! This example captures all IEEE 802.15.4 frames on a given channel (`SNIFFER_CHANNEL` at
//! build time; default: 15)
//! and prints each one via defmt/RTT: capture time (us), length, frame control
//! field, sequence number, RSSI and the raw bytes. Run with a RTT viewer to see
//! the output. ACKs are captured too, so a frame's sequence number followed by
//! an ACK with the same number is an acknowledged frame, and the gap between
//! their capture times a coarse view of the ACK spacing.
//!
//! Similar to `receive_all_frames` but outputs raw frame bytes suited for
//! analysis. Thread's test networks live on channel 11.

#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;
use embassy_time::Instant;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_802154_examples::Irqs;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

/// The channel to listen on: `SNIFFER_CHANNEL` at build time, 15 (the channel the other
/// examples use) when unset. Thread's test networks live on 11.
const CHANNEL: u8 = match option_env!("SNIFFER_CHANNEL") {
    Some(channel) => parse_channel(channel),
    None => 15,
};

const fn parse_channel(s: &str) -> u8 {
    let bytes = s.as_bytes();
    let mut channel = 0u8;
    let mut i = 0;
    while i < bytes.len() {
        assert!(
            bytes[i].is_ascii_digit(),
            "SNIFFER_CHANNEL must be a number"
        );
        channel = channel * 10 + (bytes[i] - b'0');
        i += 1;
    }
    assert!(
        channel >= 11 && channel <= 26,
        "SNIFFER_CHANNEL must be 11..=26"
    );
    channel
}

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
    let p = nrf_802154_examples::init();

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

    let mut radio = Radio::new(
        p.RADIO,
        nrf_802154_examples::radio_peripherals!(p),
        Irqs,
        mpsl,
    );
    radio.set_channel(CHANNEL);
    radio.set_promiscuous(true);

    info!("Sniffer started on channel {}", CHANNEL);

    let mut buf = [0u8; nrf_802154::MAX_PSDU_SIZE];
    loop {
        match radio.receive(&mut buf).await {
            Ok(meta) => {
                let t = Instant::now().as_micros();
                let len = meta.len as usize;
                let fcf = if len >= 2 {
                    u16::from_le_bytes([buf[0], buf[1]])
                } else {
                    0
                };
                let seq = if len >= 3 { buf[2] } else { 0 };
                info!(
                    "@{=u64}us len={} fcf={=u16:04x} seq={} rssi={} {=[u8]:02x}",
                    t,
                    len,
                    fcf,
                    seq,
                    meta.power,
                    &buf[..len]
                );
            }
            Err(e) => {
                info!("Receive error: {:?}", e);
            }
        }
    }
}
