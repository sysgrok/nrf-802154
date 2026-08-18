//! Chip-specific platform layer for nRF52 and nRF5340-net.
//!
//! Both have a classic RTC (the SL's low-power time base) and a spare TIMER
//! (the SL's high-precision time base), and both keep the radio and those timers
//! in one peripheral domain, so nothing has to be bridged.

use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use embassy_nrf::pac;

use super::{
    lp_timer_irqn, LPTIMER_SUCCESS, LPTIMER_TOO_LATE, LPTIMER_WRONG_STATE, LP_FIRE_LPTICKS,
    LP_HW_TASK_ACTIVE,
};

// =============================================================================
// Interrupt lines
// =============================================================================

/// The EGU line the C driver raises notifications on (`NRF_802154_EGU_INSTANCE_NO`
/// is 0 for both series; nRF52 muxes EGU0 and SWI0 onto one vector).
#[cfg(feature = "nrf52")]
pub type EguIrq = embassy_nrf::interrupt::typelevel::EGU0_SWI0;
/// The EGU line the C driver raises notifications on.
#[cfg(feature = "nrf53")]
pub type EguIrq = embassy_nrf::interrupt::typelevel::EGU0;

/// LP timer line: RTC2 on chips that have three RTCs.
#[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
pub type LpTimerIrq = embassy_nrf::interrupt::typelevel::RTC2;
/// LP timer line: RTC1 on chips with only RTC0 and RTC1.
#[cfg(all(
    feature = "nrf52",
    not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))
))]
pub type LpTimerIrq = embassy_nrf::interrupt::typelevel::RTC1;
/// LP timer line: RTC1 (RTC0 is reserved by MPSL).
#[cfg(feature = "nrf53")]
pub type LpTimerIrq = embassy_nrf::interrupt::typelevel::RTC1;

/// CPU frequency reported to the C driver through `SystemCoreClock`.
pub const SYSTEM_CLOCK: u32 = 64_000_000;

/// The device's unique ID, used to seed the driver's PRNG.
pub fn device_id() -> (u32, u32) {
    #[cfg(feature = "nrf52")]
    {
        let ficr = pac::FICR;
        (ficr.deviceid(0).read(), ficr.deviceid(1).read())
    }
    #[cfg(feature = "nrf53")]
    {
        let ficr = pac::FICR;
        (
            ficr.info().deviceid(0).read(),
            ficr.info().deviceid(1).read(),
        )
    }
}

// =============================================================================
// HP Timer
//
// High-Precision Timer providing 1µs resolution for frame timestamps and
// synchronous radio operations. Uses TIMER2 (TIMER0 is reserved by MPSL,
// TIMER1 may be used by MPSL on nRF53).
//
// Configured as a 32-bit timer at 1 MHz (prescaler=4: 16 MHz / 2^4 = 1 MHz).
//
// Capture/Compare channel allocation:
//   CC[0] - used for current_time_get (capture)
//   CC[1] - used for sync (capture via PPI/DPPI from LP timer)
//   CC[2] - used for timestamping (capture via PPI/DPPI from radio events)
// =============================================================================

fn hp_timer() -> pac::timer::Timer {
    #[cfg(feature = "nrf52")]
    {
        pac::TIMER2
    }
    #[cfg(feature = "nrf53")]
    {
        pac::TIMER2_NS
    }
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_init() {
    let timer = hp_timer();
    timer
        .mode()
        .write(|w| w.set_mode(pac::timer::vals::Mode::Timer));
    timer
        .bitmode()
        .write(|w| w.set_bitmode(pac::timer::vals::Bitmode::_32bit));
    // Prescaler = 4 -> 16 MHz / 2^4 = 1 MHz -> 1µs resolution
    timer.prescaler().write(|w| w.set_prescaler(4));
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_deinit() {
    let timer = hp_timer();
    timer.tasks_stop().write_value(1);
    timer.intenclr().write(|w| {
        for i in 0..6 {
            w.set_compare(i, true);
        }
    });
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_start() {
    let timer = hp_timer();
    timer.tasks_clear().write_value(1);
    timer.tasks_start().write_value(1);
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_stop() {
    let timer = hp_timer();
    timer.tasks_stop().write_value(1);
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_current_time_get() -> u32 {
    let timer = hp_timer();
    // Trigger a capture to CC[0] and read the value
    timer.tasks_capture(0).write_value(1);
    timer.cc(0).read()
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_sync_task_get() -> u32 {
    let timer = hp_timer();
    timer.tasks_capture(1).as_ptr() as u32
}

/// Whether a sync capture has occurred on CC[1]
static HP_SYNC_CAPTURED: AtomicBool = AtomicBool::new(false);

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_sync_prepare() {
    // Mark that no sync capture has occurred yet
    HP_SYNC_CAPTURED.store(false, Ordering::Release);
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_sync_time_get(p_timestamp: *mut u32) -> bool {
    if p_timestamp.is_null() {
        return false;
    }
    if HP_SYNC_CAPTURED.load(Ordering::Acquire) {
        let timer = hp_timer();
        let val = timer.cc(1).read();
        // Safety: null check above guarantees p_timestamp is non-null.
        // The C driver guarantees it points to a valid uint32_t.
        unsafe { p_timestamp.write(val) };
        true
    } else {
        false
    }
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_timestamp_task_get() -> u32 {
    let timer = hp_timer();
    timer.tasks_capture(2).as_ptr() as u32
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_timestamp_get() -> u32 {
    let timer = hp_timer();
    timer.cc(2).read()
}

// =============================================================================
// Timestamper
//
// Peripheral-to-peripheral hardware connections for timestamping.
// On nRF52 (PPI-based), no cross-domain connections are needed.
// On nRF53 (DPPI-based), cross-domain connections would be needed but are not
// yet implemented as they require specific DPPI channel routing.
//
// NOTE: These are currently minimal stubs. Frame timestamps will not be
// accurate without a full PPI/DPPI timestamper implementation. The C driver's
// SL layer handles the basic PPI routing, but cross-domain (nRF53) and
// advanced timestamping scenarios are not supported yet.
// =============================================================================

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_init() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_cross_domain_connections_setup() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_cross_domain_connections_clear() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_local_domain_connections_setup(_dppi_ch: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_local_domain_connections_clear(_dppi_ch: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_captured_timestamp_read(
    _p_captured: *mut u64,
) -> bool {
    false
}

// =============================================================================
// SL LP Timer
//
// Low-Power Timer based on the RTC peripheral. Uses RTC2 on nRF52832/52833/
// nRF52840 (RTC0 is reserved by MPSL, RTC1 is typically used by the embassy
// time driver) and RTC1 on nRF52805–nRF52820 and nRF5340-net (RTC0 is
// reserved by MPSL).
//
// NOTE: On chips without RTC2 (nRF52805–nRF52820, nRF5340-net), this driver
// uses RTC1, which conflicts with embassy-nrf's default time driver. On those
// chips, ensure that embassy's time driver is configured to use a different
// timer or is disabled.
//
// The RTC runs at 32.768 kHz with a 24-bit counter. One lptick equals one RTC
// tick (~30.5 µs). A 64-bit lptick counter is maintained by tracking overflows.
//
// Compare channel allocation:
//   CC[0] - used for schedule_at (timer fire)
//   CC[1] - used for sync_schedule_at / sync_schedule_now
//   CC[2] - used for hw_task_prepare (PPI/DPPI triggered)
// =============================================================================

/// RTC counter width (24 bits)
const RTC_COUNTER_MAX: u32 = 0x00FF_FFFF;

/// RTC frequency in Hz (no prescaler)
const RTC_FREQ_HZ: u64 = 32768;

/// Overflow count tracking for 64-bit lpticks
static LP_OVERFLOW_COUNT: AtomicU32 = AtomicU32::new(0);

/// Sync fire time in lpticks (access under critical section)
static LP_SYNC_LPTICKS: critical_section::Mutex<core::cell::Cell<u64>> =
    critical_section::Mutex::new(core::cell::Cell::new(u64::MAX));

fn lp_timer() -> pac::rtc::Rtc {
    // Use RTC2 where available (nrf52832, nrf52833, nrf52840).
    // Fall back to RTC1 on chips with only RTC0+RTC1 (nrf52805-nrf52820, nrf5340-net).
    // RTC0 is reserved by MPSL in all cases.
    #[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
    {
        pac::RTC2
    }
    #[cfg(not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840")))]
    {
        // nrf52805, nrf52810, nrf52811, nrf52820 use RTC1.
        // nrf5340-net also uses RTC1_NS.
        #[cfg(feature = "nrf52")]
        {
            pac::RTC1
        }
        #[cfg(feature = "nrf53")]
        {
            pac::RTC1_NS
        }
    }
}

/// Reconstruct the full 64-bit lptick value from the 24-bit RTC counter.
fn lp_current_lpticks() -> u64 {
    // Double-read the overflow counter around the counter read to detect
    // races where an overflow ISR fires between the reads.
    //
    // Also account for a pending hardware overflow event (EVENTS_OVRFLW set)
    // that has not yet been serviced — e.g. because the RTC interrupt is masked
    // inside lptimer_critical_section_enter. In that case LP_OVERFLOW_COUNT is
    // stale while the counter has already wrapped, so we add one extra period.
    // We only treat EVENTS_OVRFLW as pending when the counter is in the low half
    // of its range (where it will be right after a wrap) to avoid double-counting
    // if the ISR services the event between reading the flag and the counter.
    let rtc = lp_timer();
    loop {
        let ovf1 = LP_OVERFLOW_COUNT.load(Ordering::Acquire) as u64;
        let cnt = rtc.counter().read().counter() as u64;
        let ovf2 = LP_OVERFLOW_COUNT.load(Ordering::Acquire) as u64;
        if ovf1 == ovf2 {
            let half_period = (RTC_COUNTER_MAX as u64).div_ceil(2);
            let pending_hw_overflow = rtc.events_ovrflw().read() != 0 && cnt < half_period;
            let effective_ovf = if pending_hw_overflow { ovf1 + 1 } else { ovf1 };
            return effective_ovf * (RTC_COUNTER_MAX as u64 + 1) + cnt;
        }
    }
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_init() {
    let rtc = lp_timer();

    // Reset state to safe sentinels
    LP_OVERFLOW_COUNT.store(0, Ordering::Release);
    critical_section::with(|cs| {
        LP_FIRE_LPTICKS.borrow(cs).set(u64::MAX);
        LP_SYNC_LPTICKS.borrow(cs).set(u64::MAX);
    });

    // No prescaler -> 32.768 kHz
    rtc.prescaler().write(|w| w.set_prescaler(0));

    // Clear any stale events before enabling interrupts to avoid spurious ISR fires
    rtc.events_ovrflw().write_value(0);
    rtc.events_compare(0).write_value(0);
    rtc.events_compare(1).write_value(0);
    rtc.events_compare(2).write_value(0);

    // Enable overflow interrupt for 64-bit tracking.
    // Compare[0] and compare[1] interrupts are enabled on-demand by schedule_at
    // and sync_schedule_now/at respectively, so only overflow is enabled at init.
    rtc.intenset().write(|w| w.set_ovrflw(true));
    // Enable compare[2] event routing for hw tasks
    rtc.evtenset().write(|w| w.set_compare(2, true));
    // Start the RTC
    rtc.tasks_start().write_value(1);

    // Enable the NVIC line for the LP timer RTC interrupt so the ISR can fire.
    // Without this, the RTC event flags would be set but never serviced.
    unsafe { cortex_m::peripheral::NVIC::unmask(lp_timer_irqn()) };
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_deinit() {
    let rtc = lp_timer();
    rtc.tasks_stop().write_value(1);
    rtc.intenclr().write(|w| {
        w.set_ovrflw(true);
        w.set_compare(0, true);
        w.set_compare(1, true);
    });
    // Also disable compare[2] event routing and clear any pending event
    rtc.evtenclr().write(|w| w.set_compare(2, true));
    rtc.events_compare(2).write_value(0);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_current_lpticks_get() -> u64 {
    lp_current_lpticks()
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_us_to_lpticks_convert(us: u64, round_up: bool) -> u64 {
    // lpticks = us * 32768 / 1000000
    // Use u128 intermediate to avoid overflow for large us values.
    let product = (us as u128) * (RTC_FREQ_HZ as u128);
    let ticks = if round_up {
        product.div_ceil(1_000_000u128)
    } else {
        product / 1_000_000u128
    };
    if ticks > u64::MAX as u128 {
        u64::MAX
    } else {
        ticks as u64
    }
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_lpticks_to_us_convert(lpticks: u64) -> u64 {
    // us = lpticks * 1000000 / 32768
    // Use u128 intermediate to avoid overflow for large lptick values.
    let product = (lpticks as u128) * 1_000_000u128;
    let us = product / (RTC_FREQ_HZ as u128);
    if us > u64::MAX as u128 {
        u64::MAX
    } else {
        us as u64
    }
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_schedule_at(fire_lpticks: u64) {
    critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).set(fire_lpticks));
    let rtc = lp_timer();

    // Per the C header contract: "If fire_lpticks are in the past, the lptimer
    // event will be triggered asap, but still from the context of an lptimer's ISR."
    // The ISR dispatches only when EVENTS_COMPARE[0] is set by hardware, so a plain
    // NVIC pend wouldn't work. Instead, program CC[0] to an imminent value so the
    // hardware compare fires within ~2 ticks, setting the event flag and triggering
    // the ISR which then checks `now >= fire`.
    let cc_val = if fire_lpticks <= lp_current_lpticks() {
        let now_cc = rtc.counter().read().counter();
        now_cc.wrapping_add(2) & RTC_COUNTER_MAX
    } else {
        (fire_lpticks & RTC_COUNTER_MAX as u64) as u32
    };
    // Clear the event before writing CC[0] to avoid losing an immediate match:
    // if the counter hits the new CC value right after writing, clearing after
    // would erase the just-set event.
    rtc.events_compare(0).write_value(0);
    rtc.cc(0).write(|w| w.set_compare(cc_val));
    rtc.intenset().write(|w| w.set_compare(0, true));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_disable() {
    critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).set(u64::MAX));
    let rtc = lp_timer();
    rtc.intenclr().write(|w| w.set_compare(0, true));
    rtc.events_compare(0).write_value(0);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_prepare(
    fire_lpticks: u64,
    _ppi_channel: u32,
) -> u32 {
    let rtc = lp_timer();
    let cc_val = (fire_lpticks & RTC_COUNTER_MAX as u64) as u32;
    // Clear event and disable routing before writing CC[2] to avoid losing
    // an immediate match, then re-enable routing after.
    rtc.evtenclr().write(|w| w.set_compare(2, true));
    rtc.events_compare(2).write_value(0);
    rtc.cc(2).write(|w| w.set_compare(cc_val));
    rtc.evtenset().write(|w| w.set_compare(2, true));
    LP_HW_TASK_ACTIVE.store(1, Ordering::Release);

    // Check if already too late
    let now = lp_current_lpticks();
    if now >= fire_lpticks {
        LP_HW_TASK_ACTIVE.store(0, Ordering::Release);
        rtc.evtenclr().write(|w| w.set_compare(2, true));
        return LPTIMER_TOO_LATE;
    }

    LPTIMER_SUCCESS
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_cleanup() -> u32 {
    if LP_HW_TASK_ACTIVE.load(Ordering::Acquire) == 0 {
        return LPTIMER_WRONG_STATE;
    }
    let rtc = lp_timer();
    rtc.evtenclr().write(|w| w.set_compare(2, true));
    rtc.events_compare(2).write_value(0);
    LP_HW_TASK_ACTIVE.store(0, Ordering::Release);
    LPTIMER_SUCCESS
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_update_ppi(_ppi_channel: u32) -> u32 {
    if LP_HW_TASK_ACTIVE.load(Ordering::Acquire) == 0 {
        return LPTIMER_WRONG_STATE;
    }
    // PPI/DPPI channel retargeting is not supported by this platform.
    // Return wrong state so the caller doesn't assume the routing was updated.
    LPTIMER_WRONG_STATE
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_schedule_now() {
    let rtc = lp_timer();
    let now = rtc.counter().read().counter();
    // Schedule compare[1] to fire at the next tick
    let cc_val = now.wrapping_add(2) & RTC_COUNTER_MAX;
    // Clear event before writing CC[1] to avoid losing an immediate match.
    rtc.events_compare(1).write_value(0);
    rtc.cc(1).write(|w| w.set_compare(cc_val));
    rtc.intenset().write(|w| w.set_compare(1, true));
    rtc.evtenset().write(|w| w.set_compare(1, true));
    critical_section::with(|cs| LP_SYNC_LPTICKS.borrow(cs).set(lp_current_lpticks() + 2));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_schedule_at(fire_lpticks: u64) {
    critical_section::with(|cs| LP_SYNC_LPTICKS.borrow(cs).set(fire_lpticks));
    let rtc = lp_timer();
    let cc_val = (fire_lpticks & RTC_COUNTER_MAX as u64) as u32;
    // Clear event before writing CC[1] to avoid losing an immediate match.
    rtc.events_compare(1).write_value(0);
    rtc.cc(1).write(|w| w.set_compare(cc_val));
    rtc.intenset().write(|w| w.set_compare(1, true));
    rtc.evtenset().write(|w| w.set_compare(1, true));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_abort() {
    critical_section::with(|cs| LP_SYNC_LPTICKS.borrow(cs).set(u64::MAX));
    let rtc = lp_timer();
    rtc.intenclr().write(|w| w.set_compare(1, true));
    rtc.evtenclr().write(|w| w.set_compare(1, true));
    rtc.events_compare(1).write_value(0);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_event_get() -> u32 {
    let rtc = lp_timer();
    rtc.events_compare(1).as_ptr() as u32
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_lpticks_get() -> u64 {
    critical_section::with(|cs| LP_SYNC_LPTICKS.borrow(cs).get())
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_granularity_get() -> u32 {
    // One lptick at 32.768 kHz ≈ 30.5 µs.
    (1_000_000u64).div_ceil(RTC_FREQ_HZ) as u32
}

/// LP Timer (RTC) interrupt handler implementation.
///
/// Called by `LpTimerInterruptHandler::on_interrupt()` when the RTC interrupt fires.
/// Handles overflow, compare[0] (timer fire), and compare[1] (sync) events.
pub(crate) fn lp_timer_isr() {
    extern "C" {
        fn nrf_802154_sl_timer_handler(now_lpticks: u64);
        fn nrf_802154_sl_timestamper_synchronized();
    }

    let rtc = lp_timer();

    // Handle overflow: increment the 64-bit lptick counter extension
    if rtc.events_ovrflw().read() != 0 {
        rtc.events_ovrflw().write_value(0);
        LP_OVERFLOW_COUNT.fetch_add(1, Ordering::Release);
    }

    // Handle compare[0]: scheduled timer fire
    if rtc.events_compare(0).read() != 0 {
        rtc.events_compare(0).write_value(0);
        let now = lp_current_lpticks();
        let fire = critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).get());
        if now >= fire {
            unsafe { nrf_802154_sl_timer_handler(now) };
        }
    }

    // Handle compare[1]: sync timer fire
    if rtc.events_compare(1).read() != 0 {
        rtc.events_compare(1).write_value(0);
        // Mark sync capture in HP timer
        HP_SYNC_CAPTURED.store(true, Ordering::Release);
        unsafe { nrf_802154_sl_timestamper_synchronized() };
    }
}
