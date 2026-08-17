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

# Build the IEEE 802.15.4 examples crate (full compilation including linking).
# Note: the chip target and features are baked into nrf-802154-examples/Cargo.toml, so
# no --features flag is needed or accepted here.
cargo build -p nrf-802154-examples --target thumbv7em-none-eabi

# The OpenThread HIL test node (`tests/`, the `cli_node` firmware driven by the
# `openthread` repo's upstream-suite harness). Chip and features are baked into
# its Cargo.toml; the target is passed explicitly because the crate's own
# `.cargo/config.toml` only applies when building from within its directory.
cargo clippy -p nrf-802154-tests --target thumbv7em-none-eabi
cargo build -p nrf-802154-tests --release --target thumbv7em-none-eabi
