[![CI](https://github.com/sysgrok/nrf-802154/actions/workflows/ci.yml/badge.svg)](https://github.com/sysgrok/nrf-802154/actions/workflows/ci.yml)
[![crates.io](https://img.shields.io/crates/v/nrf-802154.svg)](https://crates.io/crates/nrf-802154)
[![Matrix](https://img.shields.io/matrix/rs-matter:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#rs-matter:matrix.org)

# nRF 802.15.4 Radio Driver

Rust type-safe bindings for the Nordic Semiconductor nRF series 802.15.4 C radio driver.

The 802.154.4 radio driver is an open-source C library from Nordic Semiconductor that provides an 802.15.4 protocol stack.

This crate provides high-level, easy-to-use async Rust bindings for the radio driver, which are similar in spirit to the pure Rust ones in `embassy-nrf`.

Unlike the driver in `embassy-nrf`, the C driver provides the following extra functionalities:
- Integration with the NRF MPSL layer which allows the co-existence of the BLE and 802.15.4 stacks (i.e. the BLE and the 802.15.4 stacks can run simultaneously);
- Implementation of 802.15.4 MAC-level features:
 - Automatic send of ACKs for received frames;
 - Automatic receival of ACKs for sent frames;
 - MAC filtering;
 - Enh-ACKs;
 - ... and others. See [this page](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/protocols/thread/overview/supported_features.html) for more information.

## OpenThread

The radio of the nRF chips for the [`openthread`](https://github.com/esp-rs/openthread) crate is `NrfRadio`, over this driver, in the `openthread-driver-nrf` crate of the `openthread` repository. The OpenThread examples for the nRF chips are there too, in `examples/nrf`.

## Example

The following example, [`sniffer`](../examples/src/bin/sniffer.rs), shows how to initialize and run the 802.15.4 radio driver in an application. The parts that differ between the chips - `init`, `Irqs` and the peripherals that MPSL and the driver take - come from the [examples crate](../examples/src/lib.rs).

```rust
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
```

## License

The Rust code in this crate is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](./LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

This crate links against the pre-compiled SoftDevice Controller library from Nordic Semiconductor, which is subject to its own license.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
