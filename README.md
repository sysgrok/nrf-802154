# `nrf-802154`

[![CI](https://github.com/sysgrok/nrf-802154/actions/workflows/ci.yml/badge.svg)](https://github.com/sysgrok/nrf-802154/actions/workflows/ci.yml)
[![Matrix](https://img.shields.io/matrix/rs-matter:matrix.org?label=join%20matrix&color=BEC5C9&logo=matrix)](https://matrix.to/#/#rs-matter:matrix.org)

Rust bindings for the Nordic Semiconductor nRF series 802.15.4 Radio Driver.

The 802.15.4 Radio Driver is an open source library written by Nordic for their microcontrollers that provides a
driver API for the 802.15.4 protocol capabilities of their radio peripheral.

The [SoftDevice Controller](https://github.com/alexmoon/nrf-sdc/tree/main) and the 802.15.4 Radio Driver can co-exist and be operated simulaneously.

## Supported chips

| Chip | `nrf-802154` feature | Target | Status |
| --- | --- | --- | --- |
| nRF52805/10/11/20/32/33 | `nrf52805` … `nrf52833` | `thumbv7em-none-eabi` | builds |
| nRF52840 | `nrf52840` | `thumbv7em-none-eabi` | tested on hardware |
| nRF5340 (network core) | `nrf5340-net` | `thumbv8m.main-none-eabi` | builds |
| nRF54L05/L10/L15 (application core) | `nrf54l05-app-s`, `nrf54l10-app-s`/`-ns`, `nrf54l15-app-s`/`-ns` | `thumbv8m.main-none-eabihf` | tested on hardware |
| nRF54LM20A (application core) | `nrf54lm20-app-s` | `thumbv8m.main-none-eabihf` | builds |
| nRF54H20 | — | — | not supported |

Secure and non-secure variants are offered wherever `nrf-mpsl` and `embassy-nrf` offer them,
which is why the nRF54L05 and nRF54LM20 are secure-only. The nRF54LM20 needs its own feature
rather than sharing the nRF54L15 one because the C driver is compiled against a chip define,
and `CCM00` — which its frame-encryption path writes to — sits at a different address there.
The `B` revision of the nRF54LM20 would need another feature again.

The nRF54L platform layer differs substantially from the nRF52/nRF53 one, because the series
has no classic `RTC` and no spare high-precision `TIMER` for this purpose. Both the low-power
timer and frame timestamping run off the GRTC, and since the GRTC sits in a different peripheral
domain from the radio, timestamps and hardware-timed radio tasks are bridged across with PPIB.
See the module docs in [`nrf-802154/src/platform/nrf54l.rs`](nrf-802154/src/platform/nrf54l.rs)
for the full resource allocation, including which GRTC channels and interrupt group the driver
claims alongside MPSL and embassy's time driver.

nRF54H20 is not supported by the high-level `nrf-802154` crate: neither `nrf-mpsl` nor
`embassy-nrf` supports that part yet. The low-level `nrf-802154-sys` crate does build for it
(`--features nrf54h`), so only the Rust platform layer is missing once those land.

## Building from source

This repository vendors the C sources of the 802.15.4 driver via git submodules (Nordic's `nrfx` and
`nrfxlib`, and ARM's `CMSIS_5`), so they must be checked out before building from a git clone:

```sh
git clone --recursive https://github.com/sysgrok/nrf-802154
# ...or, in an already-cloned checkout:
git submodule update --init --recursive
```

(The crates published to crates.io bundle the required sources, so this step is only needed when building
from a git checkout — consumers depending on the published crates do not need it.)

## High-level bindings

The `nrf-802154` crate contains high-level easy-to-use Rust async/await bindings for the 802.15.4 Radio Driver.

## License

This repo contains submodules with code and libraries provided by Nordic Semiconductor and ARM. Those are subject to
their own respective licenses.

The high level bindings [nrf-802154](nrf-802154) are licensed under either of

- Apache License, Version 2.0 ([LICENSE-APACHE](./nrf-sdc/LICENSE-APACHE) or http://www.apache.org/licenses/LICENSE-2.0)
- MIT license ([LICENSE-MIT](./nrf-sdc/LICENSE-MIT) or http://opensource.org/licenses/MIT)

at your option.
