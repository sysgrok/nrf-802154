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

## Example

The following example shows how to initialize and run the 802.15.4 radio driver in an application.

```rust
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

use embedded_alloc::LlffHeap as Heap;

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

// Only needed for tinyrlibc's alloc functions which won't be called at runtime.
//
// If the firmware would not use or need heap allocation for other purposes, this could be replaced
// with stub impls of `calloc` and `free` that panic with `unimplemented!()`,
// and the `#[global_allocator]` attribute could be removed.
#[global_allocator]
static HEAP: Heap = Heap::empty();

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
```

## License

The Rust code in this crate is licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](./LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.

This crate links against the pre-compiled SoftDevice Controller library from Nordic Semiconductor, which is subject to its own license.

## Contribution

Unless you explicitly state otherwise, any contribution intentionally submitted for inclusion in the work by you, as defined in the Apache-2.0 license, shall be dual licensed as above, without any additional terms or conditions.
