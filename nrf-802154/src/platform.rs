//! The platform layer the Nordic 802.15.4 C driver links against.
//!
//! Everything here is a `#[no_mangle] extern "C"` definition that the driver (or
//! its SL library) declares and expects the integrator to provide: temperature,
//! clocks, timers, timestamping, NVIC plumbing and randomness.
//!
//! The parts that differ per chip family live in [`imp`], selected below. What
//! differs is mostly the time base: nRF52/nRF53 have classic RTC and TIMER
//! peripherals, while nRF54L has neither an RTC nor a usable high-precision
//! timer for this purpose and instead runs everything off the GRTC — with the
//! extra wrinkle that the GRTC sits in a different peripheral domain from the
//! radio, so events have to be bridged across with PPIB.

use core::sync::atomic::{AtomicU32, Ordering};

use core::cell::Cell;

use cortex_m::interrupt::InterruptNumber as _;
use critical_section::Mutex;
use embassy_nrf::interrupt::typelevel::{Handler, Interrupt};

use crate::mpsl;

#[cfg_attr(not(feature = "_nrf54l"), path = "platform/nrf5x.rs")]
#[cfg_attr(feature = "_nrf54l", path = "platform/nrf54l.rs")]
mod imp;

pub use imp::*;

// =============================================================================
// Interrupt handler structs
//
// These implement embassy-nrf's Handler trait so that users can wire up
// interrupts via bind_interrupts! instead of manually writing #[interrupt] fns.
// The interrupt lines they attach to are chip-specific — see `imp`.
// =============================================================================

/// Interrupt handler for the LP timer (RTC on nRF52/nRF53, GRTC on nRF54L).
///
/// Bind this to the appropriate interrupt using [`embassy_nrf::bind_interrupts!`]:
///
/// ```no_run
/// # use embassy_nrf::bind_interrupts;
/// bind_interrupts!(struct Irqs {
///     RTC2 => nrf_802154::LpTimerInterruptHandler;  // nRF52832/52833/52840
///     // or: RTC1 => nrf_802154::LpTimerInterruptHandler;  // other nRF52 / nRF5340-net
///     // or: GRTC_0 => nrf_802154::LpTimerInterruptHandler;  // nRF54L
/// });
/// ```
pub struct LpTimerInterruptHandler;

impl Handler<LpTimerIrq> for LpTimerInterruptHandler {
    unsafe fn on_interrupt() {
        lp_timer_isr();
    }
}

/// Interrupt handler for the EGU peripheral used by the 802.15.4 C driver.
///
/// Bind this to the EGU interrupt using [`embassy_nrf::bind_interrupts!`]:
///
/// ```no_run
/// # use embassy_nrf::bind_interrupts;
/// bind_interrupts!(struct Irqs {
///     EGU0_SWI0 => nrf_802154::EguInterruptHandler;  // nRF52
///     // or: EGU0 => nrf_802154::EguInterruptHandler;  // nRF5340-net
///     // or: EGU10 => nrf_802154::EguInterruptHandler;  // nRF54L
/// });
/// ```
pub struct EguInterruptHandler;

impl Handler<EguIrq> for EguInterruptHandler {
    unsafe fn on_interrupt() {
        let irqn = EguIrq::IRQ.number() as u32;
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
//
// `nrf_802154_clock_hfclk_latency_set` is intentionally NOT defined here.
//
// Up to nrfxlib v3.1.1 the SL library only defined this (empty) function under
// `#ifdef NRF54L_SERIES`, so on nRF52/nRF53 the platform had to supply it. As of
// nrfxlib v3.3.0 the SL library (`nrf_802154_sl_rsch.c`) defines it
// unconditionally for all chips, so a Rust `#[no_mangle]` stub here would be a
// duplicate symbol at link time. Let the C own it.
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
// SL LP Timer — shared state and critical section
//
// The timer itself is chip-specific (see `imp`); what lives here is the state
// both implementations keep and the critical section, which is just NVIC
// masking of whichever line the LP timer ended up on.
// =============================================================================

/// Critical section nesting counter for LP timer
static LP_CRIT_SECT_CNT: AtomicU32 = AtomicU32::new(0);

/// Scheduled fire time in lpticks (access under critical section)
static LP_FIRE_LPTICKS: Mutex<Cell<u64>> = Mutex::new(Cell::new(u64::MAX));

/// Whether hw task is prepared
static LP_HW_TASK_ACTIVE: AtomicU32 = AtomicU32::new(0);

/// NRF_802154_SL_LPTIMER_PLATFORM_SUCCESS
const LPTIMER_SUCCESS: u32 = 0;
/// NRF_802154_SL_LPTIMER_PLATFORM_TOO_LATE
const LPTIMER_TOO_LATE: u32 = 1;
/// NRF_802154_SL_LPTIMER_PLATFORM_WRONG_STATE
const LPTIMER_WRONG_STATE: u32 = 4;

/// Get the interrupt number of the LP timer instance.
/// Derived from the type-level LP timer interrupt to avoid hard-coding chip-specific values.
fn lp_timer_irqn() -> IrqNumber {
    IrqNumber(LpTimerIrq::IRQ.number())
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_critical_section_enter() {
    // Mask the LP timer interrupt to prevent nrf_802154_sl_timer_handler from being called.
    cortex_m::peripheral::NVIC::mask(lp_timer_irqn());
    LP_CRIT_SECT_CNT.fetch_add(1, Ordering::AcqRel);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_critical_section_exit() {
    // Guard against underflow: if the counter is already 0, do nothing.
    // This prevents a C-side bug from wrapping to u32::MAX and permanently masking the IRQ.
    let prev = LP_CRIT_SECT_CNT.fetch_update(Ordering::AcqRel, Ordering::Acquire, |v| {
        if v == 0 {
            None
        } else {
            Some(v - 1)
        }
    });
    if prev == Ok(1) {
        // Last nesting level — re-enable the LP timer interrupt.
        unsafe { cortex_m::peripheral::NVIC::unmask(lp_timer_irqn()) };
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

/// Number of NVIC priority bits implemented by the hardware.
/// All supported nRF52/nRF53/nRF54 chips use 3 priority bits, giving 8 priority levels (0-7).
const NVIC_PRIO_BITS: u8 = 3;

/// ISR callback slots, keyed by IRQ number.
///
/// The C driver registers at most three ISRs: the EGU line (notifications), the
/// RADIO line, and — on nRF54L, where frame encryption is offloaded to the CCM00
/// accelerator — the CCM line. The table is keyed rather than indexed because
/// nRF54L interrupt numbers run into the 200s (`EGU10` alone is 135), so an
/// index-addressed table would have to be dozens of times larger than this one.
const MAX_IRQ_COUNT: usize = 4;
static mut ISR_TABLE: [(u32, Option<IrqHandler>); MAX_IRQ_COUNT] = [(0, None); MAX_IRQ_COUNT];

/// Dispatches a registered ISR for the given IRQ number.
///
/// Called by `EguInterruptHandler::on_interrupt()` (and, on nRF54L,
/// `CcmInterruptHandler::on_interrupt()`) to invoke the ISR registered by the C
/// driver via `nrf_802154_irq_init`.
///
/// # Safety
///
/// This function accesses a global mutable ISR table (`ISR_TABLE`) without
/// locking. It is safe to call from interrupt context because
/// `nrf_802154_irq_init` is only called during driver initialization before
/// interrupts are enabled, so no concurrent writes can occur.
pub(crate) unsafe fn irq_handler(irqn: u32) {
    for (slot_irqn, handler) in unsafe { ISR_TABLE } {
        if let Some(handler) = handler {
            if slot_irqn == irqn {
                unsafe { handler() };
                return;
            }
        }
    }
}

#[no_mangle]
extern "C" fn nrf_802154_irq_init(irqn: u32, prio: i32, isr: Option<IrqHandler>) {
    // The C driver passes CMSIS-style logical priorities (0-7 on nRF52/nRF53/nRF54).
    // cortex_m::NVIC::set_priority expects the raw NVIC register value, where
    // only the top NVIC_PRIO_BITS bits are significant. Convert by shifting
    // left by (8 - NVIC_PRIO_BITS). All supported nRF chips use 3 priority bits.
    let clamped_prio = prio.clamp(0, 7) as u8;
    let raw_prio = clamped_prio << (8 - NVIC_PRIO_BITS);
    unsafe {
        let table = &mut *core::ptr::addr_of_mut!(ISR_TABLE);
        // Re-registering the same line replaces its handler; otherwise take the
        // first free slot.
        let slot = table
            .iter()
            .position(|(slot_irqn, handler)| handler.is_some() && *slot_irqn == irqn)
            .or_else(|| table.iter().position(|(_, handler)| handler.is_none()));
        if let Some(slot) = slot {
            table[slot] = (irqn, isr);
        }

        // The C driver calls nrf_802154_irq_init during initialization, before
        // nrf_802154_irq_enable is called, so the interrupt is guaranteed to be
        // disabled at this point. Setting priority on a disabled interrupt is safe.
        //
        // SAFETY: `Peripherals::steal()` is used because we don't own the
        // core peripherals singleton. This is safe here because:
        // 1. This runs during driver init before interrupts are enabled
        // 2. We only access NVIC to set a priority on a disabled interrupt
        // 3. No concurrent NVIC access is possible at this point
        let mut nvic = cortex_m::Peripherals::steal().NVIC;
        cortex_m::peripheral::NVIC::set_priority(&mut nvic, IrqNumber(irqn as u16), raw_prio);
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
    // The C driver compares this value with NVIC_GetPriority() which returns
    // the logical priority (raw register value >> (8 - __NVIC_PRIO_BITS)).
    // cortex_m::NVIC::get_priority returns the raw register value, so shift
    // it to match CMSIS convention.
    (cortex_m::peripheral::NVIC::get_priority(IrqNumber(irqn as u16)) >> (8 - NVIC_PRIO_BITS))
        as u32
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
    let (id0, id1) = device_id();
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
// =============================================================================

#[allow(non_upper_case_globals)]
#[no_mangle]
static mut SystemCoreClock: u32 = SYSTEM_CLOCK;
