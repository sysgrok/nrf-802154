#!/usr/bin/env bash

set -euxo pipefail

export RUSTFLAGS=-Dwarnings

cargo clippy -p nrf-802154-sys --features nrf52
cargo clippy -p nrf-802154-sys --features nrf53 --target thumbv8m.main-none-eabi
cargo clippy -p nrf-802154-sys --features nrf54l-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154-sys --features nrf54l-ns --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154-sys --features nrf54lm20-s --target thumbv8m.main-none-eabihf
# The nRF54H sys build works, but the high-level `nrf-802154` crate cannot follow
# it yet: neither `nrf-mpsl` nor `embassy-nrf` supports that part.
cargo clippy -p nrf-802154-sys --features nrf54h --target thumbv8m.main-none-eabihf

cargo clippy -p nrf-802154 --features nrf52805
cargo clippy -p nrf-802154 --features nrf52810
cargo clippy -p nrf-802154 --features nrf52811
cargo clippy -p nrf-802154 --features nrf52820
cargo clippy -p nrf-802154 --features nrf52832
cargo clippy -p nrf-802154 --features nrf52833
cargo clippy -p nrf-802154 --features nrf52840
cargo clippy -p nrf-802154 --features nrf5340-net --target thumbv8m.main-none-eabi
cargo clippy -p nrf-802154 --features nrf54l05-app-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154 --features nrf54l10-app-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154 --features nrf54l10-app-ns --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154 --features nrf54l15-app-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154 --features nrf54l15-app-ns --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154 --features nrf54lm20-app-s --target thumbv8m.main-none-eabihf
cargo clippy -p nrf-802154 --features nrf52840,defmt,openthread
cargo clippy -p nrf-802154 --features nrf54l15-app-s,defmt,openthread --target thumbv8m.main-none-eabihf

# Build the IEEE 802.15.4 examples crate (full compilation including linking).
# The chip is selected by a feature; nRF52840 is the default.
cargo build -p nrf-802154-examples --target thumbv7em-none-eabi
cargo build -p nrf-802154-examples --no-default-features --features nrf54l15 --target thumbv8m.main-none-eabihf
cargo build -p nrf-802154-examples --no-default-features --features nrf54lm20 --target thumbv8m.main-none-eabihf

# The OpenThread HIL test node (`tests/`, the `cli_node` firmware driven by the
# `openthread` repo's upstream-suite harness). It stays nRF52840-only: its
# console rides on the USB device peripheral, which the nRF54L05/L10/L15 do not
# have. Chip and features are baked into its Cargo.toml; the target is passed
# explicitly because the crate's own `.cargo/config.toml` only applies when
# building from within its directory.
cargo clippy -p nrf-802154-tests --target thumbv7em-none-eabi
cargo build -p nrf-802154-tests --release --target thumbv7em-none-eabi
