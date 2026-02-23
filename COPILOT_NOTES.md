# Copilot Session Notes — sysgrok/nrf-802154

## CI Commands (run these before every PR)

```bash
# 1. Formatting (uses nightly rustfmt + repo-level rustfmt.toml)
cargo +nightly fmt -- --check
# Auto-fix:
cargo +nightly fmt

# 2. Clippy / build checks (defined in ci.sh, mirrors the GitHub Actions workflow)
./ci.sh
```

The GitHub Actions workflow (`.github/workflows/check.yml`) runs both steps on every PR against `main`.

> **Always run `cargo +nightly fmt -- --check` before committing.**
> `rustfmt.toml` sets `group_imports = "StdExternalCrate"` and `imports_granularity = "Module"`,
> which regroups `use` statements differently from the defaults (e.g., trailing `use { a as _, b as _ }` blocks).

---

## Project Structure

```
nrf-802154/           ← root workspace
├── Cargo.toml        ← workspace members: nrf-802154-sys, nrf-802154, nrf802154-examples
├── ci.sh             ← local CI script (clippy for each feature combo)
├── rust-toolchain.toml  ← pinned toolchain (currently 1.89); nightly needed only for fmt
├── rustfmt.toml      ← nightly fmt config
├── .cargo/config.toml   ← default build target: thumbv7em-none-eabi, probe-rs runner
│
├── nrf-802154-sys/   ← low-level C bindings (cmake + bindgen, needs submodules + arm toolchain)
│   └── third_party/  ← git submodules: CMSIS_5, nrfx, nrfxlib (802.15.4 driver + MPSL libs)
│
├── nrf-802154/       ← high-level async Rust API
│   └── src/
│       ├── lib.rs       ← public API, re-exports
│       ├── radio.rs     ← Radio struct: receive/transmit/set_channel/set_promiscuous/…
│       ├── platform.rs  ← interrupt handlers, MPSL glue, LP timer, HP timer, IRQ table
│       ├── openthread.rs
│       └── fmt.rs
│
└── nrf802154-examples/  ← async Embassy examples (this crate)
    ├── .cargo/config.toml   ← thumbv7em-none-eabi, probe-rs runner for nRF52840_xxAA
    ├── memory.x + build.rs  ← 1 MB Flash / 256 KB RAM linker layout
    └── src/
        ├── lib.rs           ← shared Irqs bind_interrupts! + build_data_frame()
        └── bin/
            ├── receive_frame.rs         ← filtered RX (PAN ID + short addr)
            ├── receive_all_frames.rs    ← promiscuous RX
            ├── send_frame.rs            ← unicast TX, ACK requested
            ├── send_broadcast_frame.rs  ← broadcast TX
            └── sniffer.rs              ← promiscuous RX, raw bytes via RTT
```

---

## Build Notes

- **Submodules required to build nrf-802154-sys**: `CMSIS_5`, `nrfx`, `nrfxlib`.
  In this sandbox the submodules are **not populated**, so building `nrf-802154-sys` will fail.
  In CI (GitHub Actions), `submodules: true` is set and the build uses `clang` as the cross-compiler
  (not `arm-none-eabi-gcc`). The warning about missing `arm-none-eabi-gcc` is harmless.

- **Build command for the examples crate**:
  ```bash
  cargo build -p nrf802154-examples --target thumbv7em-none-eabi
  ```

- **⚠️ Do NOT run `cargo build --features nrf52840` at the workspace root.**
  The root `Cargo.toml` is a *virtual workspace* (no `[package]` section). Running
  `--features X` here applies `X` to **all** workspace members. `nrf802154-examples`
  exposes **no features** (the chip feature is baked into its dependency specs), so
  Cargo errors because it can't find `nrf52840` as a feature of that crate.
  Correct commands:
  ```bash
  # Build just the library:
  cargo build -p nrf-802154 --features nrf52840

  # Build just the examples (features are hardcoded in their Cargo.toml):
  cargo build -p nrf802154-examples --target thumbv7em-none-eabi
  ```

- **⚠️ Dependency version alignment**: `embassy-time` must match the version used by `embassy-nrf`.
  `embassy-nrf v0.9` uses `embassy-time v0.5`; if the examples ask for `v0.4`, both get linked and
  the time-driver symbols are duplicated → linker error. Similarly, `panic-probe` must be `"1"` when
  `defmt = "1"` is used (otherwise `defmt v0.3` and `v1` symbols conflict at link time).

- **⚠️ `__embassy_time_queue_item_from_waker` linker error**: `embassy-nrf v0.9` with
  `time-driver-rtc1` depends on `embassy-time-queue-utils v0.3`. By default this uses the
  "integrated" queue (`queue_integrated.rs`) which calls
  `embassy_executor_timer_queue::TimerQueueItem::from_embassy_waker()` → requires the extern
  symbol `__embassy_time_queue_item_from_waker` to be defined by the executor. This symbol is
  only defined by `embassy-executor >= 0.8` which has not been published to crates.io. The fix is to
  enable `embassy-time-queue-utils/generic-queue-64` in the examples crate, which switches to the
  generic heap-less queue (`queue_generic.rs`) that does NOT call `from_embassy_waker`.
  In `nrf802154-examples/Cargo.toml`:
  ```toml
  embassy-time-queue-utils = { version = "0.3", features = ["generic-queue-64"] }
  ```

- **`mpsl_clock_lfclk_cfg_t` fields** (from `mpsl_clock.h`):
  ```rust
  mpsl_clock_lfclk_cfg_t {
      source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
      rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
      rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
      accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
      skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
  }
  ```

- **Peripheral ownership on nRF52840**:
  - MPSL takes: `RTC0`, `TIMER0`, `TEMP`, `PPI_CH19/30/31`
  - 802.15.4 driver takes: `RADIO`, `EGU0`, `TIMER2` (HP timer), `RTC2` (LP timer)
  - Embassy time driver uses: `RTC1` (via `time-driver-rtc1` feature)

---

## Interrupt Bindings (nRF52840)

```rust
bind_interrupts!(pub struct Irqs {
    EGU0_SWI0  => nrf_mpsl::LowPrioInterruptHandler, nrf_802154::Egu0InterruptHandler;
    RADIO      => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0     => nrf_mpsl::HighPrioInterruptHandler;
    RTC0       => nrf_mpsl::HighPrioInterruptHandler;
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    RTC2       => nrf_802154::LpTimerInterruptHandler;
});
```

---

## Key External Dependencies

| Crate | Source |
|---|---|
| `nrf-mpsl` | `github.com/alexmoon/nrf-sdc` rev `43df6b8b` (patched in root Cargo.toml) |
| `openthread` | `github.com/sysgrok/openthread` branch `for-review` (patched) |
| `embassy-nrf` | crates.io v0.9 |
| `embassy-executor` | crates.io v0.7 |
| `embassy-time` | crates.io v0.4 |
