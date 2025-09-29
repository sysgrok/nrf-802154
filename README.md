# `nrf-802154`

Rust bindings for the Nordic Semiconductor nRF series 802.15.4 Radio Driver.

The 802.15.4 Radio Driver is an open source library written by Nordic for their microcontrollers that provides a
driver API for the 802.15.4 protocol capabilities of their radio peripheral.

The [SoftDevice Controller](https://github.com/alexmoon/nrf-sdc/tree/main) and the 802.15.4 Radio Driver can co-exist and be operated simulaneously.

## High-level bindings

The `nrf-802154` crate contains high-level easy-to-use Rust async/await bindings for the 802.15.4 Radio Driver.

## License

This repo contains submodules with code and libraries provided by Nordic Semiconductor and ARM. Those are subject to
their own respective licenses.

The high level bindings [nrf-802154](nrf-802154) are licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./nrf-sdc/LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](./nrf-sdc/LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
