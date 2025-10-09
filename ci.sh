#!/usr/bin/env bash

set -euxo pipefail

export RUSTFLAGS=-Dwarnings

cargo clippy -p nrf-802154-sys --features nrf52
cargo clippy -p nrf-802154-sys --features nrf53 --target thumbv8m.main-none-eabi
# NRF_CONFIG_CPU_FREQ_MHZ issues
#cargo clippy -p nrf-802154-sys --features nrf54l-ns --target thumbv8m.main-none-eabihf
#cargo clippy -p nrf-802154-sys --features nrf54l-s --target thumbv8m.main-none-eabihf
#cargo clippy -p nrf-802154-sys --features nrf54h --target thumbv8m.main-none-eabihf

cargo clippy -p nrf-802154 --features nrf52805
cargo clippy -p nrf-802154 --features nrf52810
cargo clippy -p nrf-802154 --features nrf52811
cargo clippy -p nrf-802154 --features nrf52820
cargo clippy -p nrf-802154 --features nrf52832
cargo clippy -p nrf-802154 --features nrf52833
cargo clippy -p nrf-802154 --features nrf52840
cargo clippy -p nrf-802154 --features nrf5340-net --target thumbv8m.main-none-eabi
cargo clippy -p nrf-802154 --features nrf52840,defmt,openthread
