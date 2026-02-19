use core::cell::Cell;
use core::sync::atomic::{AtomicBool, AtomicU32, Ordering};

use cortex_m::interrupt::InterruptNumber as _;
use critical_section::Mutex;
use embassy_nrf::interrupt::typelevel::{Handler, Interrupt};
use embassy_nrf::pac;

use crate::mpsl;

// =============================================================================
// Interrupt handler type aliases
//
// These map chip-specific interrupt vector names to types used by our handler
// structs and by Radio::new()'s Binding bounds.
// =============================================================================

// EGU0 interrupt: EGU0_SWI0 on nRF52, EGU0 on nRF5340-net
#[cfg(feature = "nrf52")]
pub(crate) type Egu0Irq = embassy_nrf::interrupt::typelevel::EGU0_SWI0;
#[cfg(feature = "nrf53")]
pub(crate) type Egu0Irq = embassy_nrf::interrupt::typelevel::EGU0;

// LP timer RTC interrupt: RTC2 on chips with 3 RTCs, RTC1 on others
#[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
pub(crate) type LpTimerIrq = embassy_nrf::interrupt::typelevel::RTC2;
#[cfg(all(
    feature = "nrf52",
    not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))
))]
pub(crate) type LpTimerIrq = embassy_nrf::interrupt::typelevel::RTC1;
#[cfg(feature = "nrf53")]
pub(crate) type LpTimerIrq = embassy_nrf::interrupt::typelevel::RTC1;

// =============================================================================
// Interrupt handler structs
//
// These implement embassy-nrf's Handler trait so that users can wire up
// interrupts via bind_interrupts! instead of manually writing #[interrupt] fns.
// =============================================================================

/// Interrupt handler for the LP timer (RTC2 or RTC1 depending on chip).
///
/// Bind this to the appropriate RTC interrupt using [`embassy_nrf::bind_interrupts!`]:
///
/// ```no_run
/// # use embassy_nrf::bind_interrupts;
/// bind_interrupts!(struct Irqs {
///     RTC2 => nrf_802154::LpTimerInterruptHandler;  // nRF52832/52833/52840
///     // or: RTC1 => nrf_802154::LpTimerInterruptHandler;  // other chips
/// });
/// ```
pub struct LpTimerInterruptHandler;

impl Handler<LpTimerIrq> for LpTimerInterruptHandler {
    unsafe fn on_interrupt() {
        lp_timer_isr();
    }
}

/// Interrupt handler for the EGU0 peripheral used by the 802.15.4 C driver.
///
/// Bind this to the EGU0/SWI0 interrupt using [`embassy_nrf::bind_interrupts!`]:
///
/// ```no_run
/// # use embassy_nrf::bind_interrupts;
/// bind_interrupts!(struct Irqs {
///     EGU0_SWI0 => nrf_802154::Egu0InterruptHandler;  // nRF52
///     // or: EGU0 => nrf_802154::Egu0InterruptHandler;  // nRF5340-net
/// });
/// ```
pub struct Egu0InterruptHandler;

impl Handler<Egu0Irq> for Egu0InterruptHandler {
    unsafe fn on_interrupt() {
        let irqn = Egu0Irq::IRQ.number() as u32;
        irq_handler(irqn);
    }
}

// =============================================================================
// Temperature
//
// Delegates to MPSL's temperature sensor which reads the on-chip TEMP peripheral.
// MPSL returns temperature in units of 0.25°C; the 802.15.4 driver expects integer °C.
// =============================================================================

#[no_mangle]
extern "C" fn nrf_802154_temperature_init() {}

#[no_mangle]
extern "C" fn nrf_802154_temperature_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_temperature_get() -> i8 {
    let raw = unsafe { mpsl::raw::mpsl_temperature_get() };
    // MPSL returns 0.25°C units; convert to integer °C and clamp to i8 range
    // in case MPSL returns an out-of-range or error value.
    (raw / 4).clamp(i8::MIN as i32, i8::MAX as i32) as i8
}

// =============================================================================
// Clock
//
// HF clock: delegates to MPSL clock APIs.
// LF clock: managed by MPSL during initialization; assumed always running.
// =============================================================================

#[no_mangle]
extern "C" fn nrf_802154_clock_init() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_is_running() -> bool {
    let mut running: u32 = 0;
    unsafe { mpsl::raw::mpsl_clock_hfclk_is_running(&mut running) };
    running != 0
}

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_start() {
    extern "C" fn on_hfclk_started() {
        extern "C" {
            fn nrf_802154_clock_hfclk_ready();
        }
        unsafe { nrf_802154_clock_hfclk_ready() };
    }
    unsafe { mpsl::raw::mpsl_clock_hfclk_request(Some(on_hfclk_started)) };
}

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_stop() {
    unsafe { mpsl::raw::mpsl_clock_hfclk_release() };
}

#[no_mangle]
extern "C" fn nrf_802154_clock_lfclk_is_running() -> bool {
    // LFCLK is started and managed by MPSL during initialization
    true
}

#[no_mangle]
extern "C" fn nrf_802154_clock_lfclk_start() {
    // LFCLK is started and managed by MPSL during initialization
}

#[no_mangle]
extern "C" fn nrf_802154_clock_lfclk_stop() {
    // LFCLK is started and managed by MPSL during initialization
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
extern "C" fn nrf_802154_clock_hfclk_latency_set(_latency: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_init() {
    let timer = hp_timer();
    timer.mode().write(|w| w.set_mode(pac::timer::vals::Mode::TIMER));
    timer
        .bitmode()
        .write(|w| w.set_bitmode(pac::timer::vals::Bitmode::_32BIT));
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
    if HP_SYNC_CAPTURED.load(Ordering::Acquire) {
        let timer = hp_timer();
        let val = timer.cc(1).read();
        // Safety: the C driver guarantees p_timestamp is a valid, non-null pointer.
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
extern "C" fn nrf_802154_platform_timestamper_captured_timestamp_read(_p_captured: *mut u64) -> bool {
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

/// Critical section nesting counter for LP timer
static LP_CRIT_SECT_CNT: AtomicU32 = AtomicU32::new(0);

/// Scheduled fire time in lpticks (access under critical section)
static LP_FIRE_LPTICKS: Mutex<Cell<u64>> = Mutex::new(Cell::new(u64::MAX));

/// Sync fire time in lpticks (access under critical section)
static LP_SYNC_LPTICKS: Mutex<Cell<u64>> = Mutex::new(Cell::new(u64::MAX));

/// Whether hw task is prepared
static LP_HW_TASK_ACTIVE: AtomicU32 = AtomicU32::new(0);

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
            let pending_hw_overflow =
                rtc.events_ovrflw().read() != 0 && cnt < ((RTC_COUNTER_MAX as u64 + 1) / 2);
            let effective_ovf = if pending_hw_overflow { ovf1 + 1 } else { ovf1 };
            return effective_ovf * (RTC_COUNTER_MAX as u64 + 1) + cnt;
        }
    }
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_init() {
    let rtc = lp_timer();
    // No prescaler -> 32.768 kHz
    rtc.prescaler().write(|w| w.set_prescaler(0));
    // Enable overflow interrupt for 64-bit tracking
    rtc.intenset().write(|w| w.set_ovrflw(true));
    // Enable compare[0] interrupt for scheduled events
    rtc.intenset().write(|w| w.set_compare(0, true));
    // Enable compare[1] interrupt for sync events
    rtc.intenset().write(|w| w.set_compare(1, true));
    // Enable compare[2] event routing for hw tasks
    rtc.evtenset().write(|w| w.set_compare(2, true));
    // Start the RTC
    rtc.tasks_start().write_value(1);
    LP_OVERFLOW_COUNT.store(0, Ordering::Release);
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
    // Set compare[0] to the lower 24 bits of the target
    let cc_val = (fire_lpticks & RTC_COUNTER_MAX as u64) as u32;
    rtc.cc(0).write(|w| w.set_compare(cc_val));
    rtc.events_compare(0).write_value(0);
    rtc.intenset().write(|w| w.set_compare(0, true));

    // Per the C header contract: "If fire_lpticks are in the past, the lptimer
    // event will be triggered asap, but still from the context of an lptimer's ISR."
    // Without this check, a past fire time would not trigger the compare event until
    // the 24-bit counter wraps (~512 seconds). Pending the interrupt ensures the ISR
    // runs promptly and dispatches via its `now >= fire` check.
    if fire_lpticks <= lp_current_lpticks() {
        cortex_m::peripheral::NVIC::pend(lp_timer_irqn());
    }
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_disable() {
    critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).set(u64::MAX));
    let rtc = lp_timer();
    rtc.intenclr().write(|w| w.set_compare(0, true));
    rtc.events_compare(0).write_value(0);
}

/// Get the RTC interrupt number for the LP timer instance.
fn lp_timer_irqn() -> IrqNumber {
    #[cfg(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840"))]
    {
        // RTC2 IRQ number = 36 on nRF52832/52833/52840
        IrqNumber(36)
    }
    #[cfg(not(any(feature = "nrf52832", feature = "nrf52833", feature = "nrf52840")))]
    {
        // RTC1 IRQ number = 17 on nRF52805-nRF52820, nRF5340-net
        IrqNumber(17)
    }
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_critical_section_enter() {
    // Mask the RTC interrupt to prevent nrf_802154_sl_timer_handler from being called.
    cortex_m::peripheral::NVIC::mask(lp_timer_irqn());
    LP_CRIT_SECT_CNT.fetch_add(1, Ordering::AcqRel);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_critical_section_exit() {
    if LP_CRIT_SECT_CNT.fetch_sub(1, Ordering::AcqRel) == 1 {
        // Last nesting level — re-enable the RTC interrupt.
        unsafe { cortex_m::peripheral::NVIC::unmask(lp_timer_irqn()) };
    }
}

/// NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS
const LPTIMER_SUCCESS: u32 = 0;
/// NRF_802154_SL_LPTIMER_PLATFORM_TOO_LATE
const LPTIMER_TOO_LATE: u32 = 1;
/// NRF_802154_SL_LPTIMER_PLATFORM_WRONG_STATE
const LPTIMER_WRONG_STATE: u32 = 4;

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_prepare(fire_lpticks: u64, _ppi_channel: u32) -> u32 {
    let rtc = lp_timer();
    let cc_val = (fire_lpticks & RTC_COUNTER_MAX as u64) as u32;
    rtc.cc(2).write(|w| w.set_compare(cc_val));
    rtc.events_compare(2).write_value(0);
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
    LPTIMER_SUCCESS
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_schedule_now() {
    let rtc = lp_timer();
    let now = rtc.counter().read().counter();
    // Schedule compare[1] to fire at the next tick
    let cc_val = now.wrapping_add(2) & RTC_COUNTER_MAX;
    rtc.cc(1).write(|w| w.set_compare(cc_val));
    rtc.events_compare(1).write_value(0);
    rtc.intenset().write(|w| w.set_compare(1, true));
    rtc.evtenset().write(|w| w.set_compare(1, true));
    critical_section::with(|cs| LP_SYNC_LPTICKS.borrow(cs).set(lp_current_lpticks() + 2));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_schedule_at(fire_lpticks: u64) {
    critical_section::with(|cs| LP_SYNC_LPTICKS.borrow(cs).set(fire_lpticks));
    let rtc = lp_timer();
    let cc_val = (fire_lpticks & RTC_COUNTER_MAX as u64) as u32;
    rtc.cc(1).write(|w| w.set_compare(cc_val));
    rtc.events_compare(1).write_value(0);
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

// =============================================================================
// IRQ
//
// Interrupt management using the ARM Cortex-M NVIC. The ISR callbacks are stored
// in a static table and dispatched when the interrupt fires.
// =============================================================================

/// Wrapper type for raw interrupt numbers to implement `InterruptNumber`.
#[derive(Clone, Copy)]
struct IrqNumber(u16);

unsafe impl cortex_m::interrupt::InterruptNumber for IrqNumber {
    fn number(self) -> u16 {
        self.0
    }
}

type IrqHandler = unsafe extern "C" fn();

/// ISR callback table (max 64 IRQ lines, covering all nRF52/nRF53 peripherals)
const MAX_IRQ_COUNT: usize = 64;
static mut ISR_TABLE: [Option<IrqHandler>; MAX_IRQ_COUNT] = [None; MAX_IRQ_COUNT];

/// Dispatches a registered ISR for the given IRQ number.
///
/// Called by `Egu0InterruptHandler::on_interrupt()` (and potentially other
/// handlers) to invoke the ISR registered by the C driver via `nrf_802154_irq_init`.
///
/// # Safety
///
/// This function accesses a global mutable ISR table (`ISR_TABLE`) without
/// locking. It is safe to call from interrupt context because
/// `nrf_802154_irq_init` is only called during driver initialization before
/// interrupts are enabled, so no concurrent writes can occur.
pub(crate) unsafe fn irq_handler(irqn: u32) {
    let idx = irqn as usize;
    if idx < MAX_IRQ_COUNT {
        if let Some(handler) = unsafe { ISR_TABLE[idx] } {
            unsafe { handler() };
        }
    }
}

#[no_mangle]
extern "C" fn nrf_802154_irq_init(irqn: u32, prio: i32, isr: Option<IrqHandler>) {
    let idx = irqn as usize;
    if idx < MAX_IRQ_COUNT {
        // Clamp the requested priority to the valid range [0, 255].
        let clamped_prio = prio.clamp(0, 255) as u8;
        unsafe {
            ISR_TABLE[idx] = isr;
            // The C driver calls nrf_802154_irq_init during initialization, before
            // nrf_802154_irq_enable is called, so the interrupt is guaranteed to be
            // disabled at this point. Setting priority on a disabled interrupt is safe.
            let mut nvic = cortex_m::Peripherals::steal().NVIC;
            cortex_m::peripheral::NVIC::set_priority(&mut nvic, IrqNumber(irqn as u16), clamped_prio);
        }
    }
}

#[no_mangle]
extern "C" fn nrf_802154_irq_enable(irqn: u32) {
    unsafe { cortex_m::peripheral::NVIC::unmask(IrqNumber(irqn as u16)) };
}

#[no_mangle]
extern "C" fn nrf_802154_irq_disable(irqn: u32) {
    cortex_m::peripheral::NVIC::mask(IrqNumber(irqn as u16));
}

#[no_mangle]
extern "C" fn nrf_802154_irq_set_pending(irqn: u32) {
    cortex_m::peripheral::NVIC::pend(IrqNumber(irqn as u16));
}

#[no_mangle]
extern "C" fn nrf_802154_irq_clear_pending(irqn: u32) {
    cortex_m::peripheral::NVIC::unpend(IrqNumber(irqn as u16));
}

#[no_mangle]
extern "C" fn nrf_802154_irq_is_enabled(irqn: u32) -> bool {
    cortex_m::peripheral::NVIC::is_enabled(IrqNumber(irqn as u16))
}

#[no_mangle]
extern "C" fn nrf_802154_irq_priority_get(irqn: u32) -> u32 {
    cortex_m::peripheral::NVIC::get_priority(IrqNumber(irqn as u16)) as u32
}

// =============================================================================
// Random
//
// Simple xorshift32 pseudo-random number generator. Sufficient for the
// CSMA-CA backoff procedure which only needs statistical randomness.
// =============================================================================

static RANDOM_STATE: AtomicU32 = AtomicU32::new(0);

#[no_mangle]
extern "C" fn nrf_802154_random_init() {
    // Seed from the device FICR DEVICEID registers for uniqueness.
    // Use the PAC so the correct address is selected for each target (nRF52 vs nRF53).
    #[cfg(feature = "nrf52")]
    let (id0, id1) = {
        let ficr = pac::FICR;
        (ficr.deviceid(0).read(), ficr.deviceid(1).read())
    };
    #[cfg(feature = "nrf53")]
    let (id0, id1) = {
        let ficr = pac::FICR;
        (ficr.info().deviceid(0).read(), ficr.info().deviceid(1).read())
    };
    let seed = id0 ^ id1;
    // Ensure non-zero seed (xorshift requires non-zero state)
    RANDOM_STATE.store(if seed != 0 { seed } else { 0x12345678 }, Ordering::Release);
}

#[no_mangle]
extern "C" fn nrf_802154_random_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_random_get() -> u32 {
    RANDOM_STATE
        .fetch_update(Ordering::AcqRel, Ordering::Acquire, |mut x| {
            if x == 0 {
                x = 0x12345678;
            }
            x ^= x << 13;
            x ^= x >> 17;
            x ^= x << 5;
            Some(x)
        })
        // Closure always returns Some, so unwrap is safe.
        .unwrap()
}

// =============================================================================
// System Core Clock
//
// nRF52 and nRF53 series run at 64 MHz.
// =============================================================================

const SYSTEM_CLOCK: u32 = 64_000_000;

#[allow(non_upper_case_globals)]
#[no_mangle]
static mut SystemCoreClock: u32 = SYSTEM_CLOCK;
