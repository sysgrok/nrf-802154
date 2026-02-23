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
├── Cargo.toml        ← workspace members: nrf-802154-sys, nrf-802154, nrf52840-examples
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
└── nrf52840-examples/  ← async Embassy examples (this crate)
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
  In this sandbox the submodules are **not populated**, so building `nrf-802154-sys` will fail
  (cmake can't find `nrfxlib/nrf_802154`). This is expected in CI-less sandboxes.
  Use `cargo check` for type/syntax checking.

- **ARM GCC toolchain** (`arm-none-eabi-gcc`) is required by the cmake build of the C driver.
  It is not available in this sandbox; `cargo check` still works via clang for bindgen.

- **`cargo check` for the examples crate** (type-check only, no link):
  ```bash
  cargo check -p nrf52840-examples --target thumbv7em-none-eabi
  ```
  This will fail at the `nrf-802154-sys` build step without submodules/arm-gcc.

- **⚠️ Do NOT run `cargo build --features nrf52840` at the workspace root.**
  The root `Cargo.toml` is a *virtual workspace* (no `[package]` section). Running
  `--features X` here applies `X` to **all** workspace members. `nrf52840-examples`
  exposes **no features** (the chip feature is baked into its dependency specs), so
  Cargo errors because it can't find `nrf52840` as a feature of that crate.
  Correct commands:
  ```bash
  # Build just the library:
  cargo build -p nrf-802154 --features nrf52840

  # Check/build just the examples (features are hardcoded in their Cargo.toml):
  cargo check -p nrf52840-examples --target thumbv7em-none-eabi
  cargo build -p nrf52840-examples --target thumbv7em-none-eabi
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
