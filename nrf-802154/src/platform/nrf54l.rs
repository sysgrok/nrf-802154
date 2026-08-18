//! Chip-specific platform layer for the nRF54L series.
//!
//! nRF54L has no classic RTC. Its low-power time base is the GRTC SYSCOUNTER: a
//! 52-bit counter ticking at a fixed 1 MHz, which makes one lptick exactly one
//! microsecond and leaves nothing to convert or to track for overflow (52 bits
//! at 1 MHz is ~142 years).
//!
//! There is also no high-precision timer here. The SL library shipped for this
//! series drops the whole `nrf_802154_hp_timer_*` API — and the LP timer's
//! `sync_*` entry points along with it — and timestamps frames by capturing the
//! GRTC directly. That is what the `nrf_802154_platform_timestamper_*` functions
//! below wire up, and on nRF52/nRF53 they are stubs for exactly the same reason.
//!
//! The catch is that the GRTC sits in the peripheral domain (the same APB block
//! as DPPIC20) while the radio's events and tasks are on DPPIC10. Carrying an
//! event between the two needs the PPIB11 <-> PPIB21 bridge, which is precisely
//! the split the SL's timestamper API models as "local domain" (the radio's
//! DPPIC10 channel, handed to us) versus "cross domain" (everything from the
//! bridge onwards, which we allocate).

use core::sync::atomic::Ordering;

use cortex_m::interrupt::InterruptNumber as _;
use embassy_nrf::interrupt::typelevel::{Handler, Interrupt};
use embassy_nrf::pac;

use super::{
    irq_handler, lp_timer_irqn, LPTIMER_SUCCESS, LPTIMER_TOO_LATE, LPTIMER_WRONG_STATE,
    LP_FIRE_LPTICKS, LP_HW_TASK_ACTIVE,
};

// =============================================================================
// Interrupt lines
// =============================================================================

/// The EGU line the C driver raises notifications on
/// (`NRF_802154_EGU_INSTANCE_NO` is 10 on this series).
pub type EguIrq = embassy_nrf::interrupt::typelevel::EGU10;

/// LP timer line, i.e. the GRTC line belonging to [`GRTC_IRQ_GROUP`].
pub type LpTimerIrq = embassy_nrf::interrupt::typelevel::GRTC_0;

/// The CCM00 encryption accelerator line.
///
/// The driver offloads outgoing frame encryption to CCM00 here
/// (`NRF_802154_ENCRYPTION_ACCELERATOR_CCM`) and registers an ISR for it through
/// `nrf_802154_irq_init`. On nRF52/nRF53 it uses ECB synchronously instead and
/// needs no interrupt, which is why this handler is nRF54L-only.
pub type CcmIrq = embassy_nrf::interrupt::typelevel::AAR00_CCM00;

/// Interrupt handler for the CCM00 encryption accelerator used by the
/// 802.15.4 C driver.
///
/// Bind this to the `AAR00_CCM00` interrupt using [`embassy_nrf::bind_interrupts!`]:
///
/// ```ignore
/// bind_interrupts!(struct Irqs {
///     AAR00_CCM00 => nrf_802154::CcmInterruptHandler;
/// });
/// ```
pub struct CcmInterruptHandler;

impl Handler<CcmIrq> for CcmInterruptHandler {
    unsafe fn on_interrupt() {
        let irqn = CcmIrq::IRQ.number() as u32;
        irq_handler(irqn);
    }
}

/// CPU frequency reported to the C driver through `SystemCoreClock`.
///
/// 128 MHz is the only frequency the 802.15.4 driver supports on this series —
/// see the `NRF_CONFIG_CPU_FREQ_MHZ` static assert in `nrf_802154_delayed_trx.c`,
/// which `nrf-802154-sys`'s build script satisfies with the matching define.
pub const SYSTEM_CLOCK: u32 = 128_000_000;

/// The device's unique ID, used to seed the driver's PRNG.
pub fn device_id() -> (u32, u32) {
    let ficr = pac::FICR;
    (
        ficr.info().deviceid(0).read(),
        ficr.info().deviceid(1).read(),
    )
}

// =============================================================================
// Hardware resources
//
// The resources below are the ones this platform layer owns. Everything else is
// picked by the C driver (`nrf_802154_peripherals_nrf54l.h`: EGU10, DPPIC10
// channels, TIMER10, CCM00 plus DPPIC00/PPIB00/PPIB10 channel 7) or by MPSL
// (RADIO, TIMER10/TIMER20, TEMP, GRTC CC[7..=11] and the `GRTC_3` line, DPPIC10
// channel 0, DPPIC20 channel 1, PPIB11/PPIB21 channel 0).
//
// embassy-nrf's GRTC time driver, when enabled, owns GRTC CC[11] and the
// application domain's line (`GRTC_2` when secure, `GRTC_1` when non-secure).
// =============================================================================

/// GRTC compare channel driving the SL LP timer's scheduled fire.
const GRTC_CC_FIRE: usize = 3;

/// GRTC compare channel driving `hw_task_prepare`, i.e. a radio task started by
/// hardware at a precise time (delayed TX/RX).
const GRTC_CC_HW_TASK: usize = 4;

/// GRTC capture channel latching frame timestamps.
const GRTC_CC_TIMESTAMP: usize = 5;

/// GRTC interrupt group: selects the `INTEN<n>`/`INTENSET<n>`/`INTENCLR<n>`
/// register set and, with it, the `GRTC_<n>` NVIC line.
///
/// Group 0 is nominally the FLPR core's, and this driver claims it precisely
/// because nothing else in this stack does: MPSL takes group 3, and embassy's
/// GRTC time driver takes the application group (2 when secure, 1 when
/// non-secure). Applications that run GRTC-driven code on the FLPR core must
/// therefore move it off group 0.
const GRTC_IRQ_GROUP: usize = 0;

/// SYSCOUNTER read port. Each port carries its own `ACTIVE` keep-awake request,
/// so reading through the group-0 port keeps this driver from racing embassy's
/// time driver over the shared `ACTIVE` bit of the application port.
const GRTC_SYSCOUNTER_IDX: usize = 0;

/// DPPIC20 channel carrying the bridged radio event to the GRTC capture task.
const DPPIC20_CH_TIMESTAMP: u8 = 2;

/// DPPIC20 channel carrying the GRTC compare event towards the radio domain.
const DPPIC20_CH_HW_TASK: u8 = 3;

/// PPIB11 -> PPIB21 channel: radio domain to peripheral domain (timestamps).
const PPIB_CH_TIMESTAMP: usize = 1;

/// PPIB21 -> PPIB11 channel: peripheral domain to radio domain (hw tasks).
const PPIB_CH_HW_TASK: usize = 2;

/// NRF_802154_SL_LPTIMER_PLATFORM_NO_RESOURCES
const LPTIMER_NO_RESOURCES: u32 = 3;

/// NRF_802154_SL_HW_TASK_PPI_INVALID
const HW_TASK_PPI_INVALID: u32 = u32::MAX;

// =============================================================================
// GRTC helpers
// =============================================================================

/// Read the GRTC SYSCOUNTER, following the product spec's recommended sequence:
/// request `ACTIVE`, then re-read until the high word reports neither `BUSY`
/// nor a low-word overflow between the two reads.
fn grtc_syscounter() -> u64 {
    use pac::grtc::vals::Busy;

    let r = pac::GRTC;
    if !r.mode().read().syscounteren() {
        return 0;
    }

    let sc = r.syscounter(GRTC_SYSCOUNTER_IDX);
    sc.active().write(|w| w.set_active(true));
    let value = loop {
        let low = sc.syscounterl().read();
        let high = sc.syscounterh().read();
        if high.busy() == Busy::Ready && !high.overflow() {
            break low as u64 | ((high.value() as u64) << 32);
        }
    };
    sc.active().write(|w| w.set_active(false));
    value
}

/// Arm a GRTC compare channel at an absolute SYSCOUNTER value.
///
/// Writing `CCL` disables the compare channel and writing `CCH` re-enables it,
/// so the low word must go first.
fn grtc_compare_arm(cc: usize, fire_lpticks: u64) {
    let r = pac::GRTC;
    r.events_compare(cc).write_value(0);
    r.cc(cc).ccl().write_value(fire_lpticks as u32);
    r.cc(cc)
        .cch()
        .write(|w| w.set_cch((fire_lpticks >> 32) as u32 & 0x000F_FFFF));
}

/// Disarm a GRTC compare channel and drop any pending event.
fn grtc_compare_disarm(cc: usize) {
    let r = pac::GRTC;
    r.cc(cc).ccen().write(|w| w.set_active(false));
    r.events_compare(cc).write_value(0);
}

// =============================================================================
// Timestamper
// =============================================================================

/// Read the 52-bit value latched in the timestamp capture channel.
fn grtc_timestamp_cc_read() -> u64 {
    let cc = pac::GRTC.cc(GRTC_CC_TIMESTAMP);
    let low = cc.ccl().read() as u64;
    let high = cc.cch().read().cch() as u64;
    low | (high << 32)
}

/// Reset the timestamp capture channel to the "nothing captured yet" sentinel.
///
/// Zero is safe as a sentinel: the SYSCOUNTER is started by MPSL long before the
/// radio driver arms a capture, and it never wraps back. Writing `CCL` also
/// leaves the compare side disabled, which is what we want — this channel only
/// ever captures.
fn grtc_timestamp_cc_reset() {
    let cc = pac::GRTC.cc(GRTC_CC_TIMESTAMP);
    cc.ccl().write_value(0);
    cc.cch().write(|w| w.set_cch(0));
    cc.ccen().write(|w| w.set_active(false));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_init() {
    grtc_timestamp_cc_reset();
}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_cross_domain_connections_setup() {
    // PPIB21 receives what PPIB11 sends on `PPIB_CH_TIMESTAMP` and republishes
    // it onto a DPPIC20 channel, which the GRTC capture task subscribes to.
    pac::PPIB21.publish_receive(PPIB_CH_TIMESTAMP).write(|w| {
        w.set_chidx(DPPIC20_CH_TIMESTAMP);
        w.set_en(true);
    });
    pac::GRTC.subscribe_capture(GRTC_CC_TIMESTAMP).write(|w| {
        w.set_chidx(DPPIC20_CH_TIMESTAMP);
        w.set_en(true);
    });
    pac::DPPIC20
        .chenset()
        .write(|w| w.0 = 1 << DPPIC20_CH_TIMESTAMP);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_cross_domain_connections_clear() {
    pac::DPPIC20
        .chenclr()
        .write(|w| w.0 = 1 << DPPIC20_CH_TIMESTAMP);
    pac::GRTC
        .subscribe_capture(GRTC_CC_TIMESTAMP)
        .write(|w| w.set_en(false));
    pac::PPIB21
        .publish_receive(PPIB_CH_TIMESTAMP)
        .write(|w| w.set_en(false));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_local_domain_connections_setup(dppi_ch: u32) {
    // Arm a fresh capture, then hook the radio domain's DPPIC10 channel — the
    // one the SL publishes the timestamped event on — into the bridge.
    grtc_timestamp_cc_reset();
    pac::PPIB11.subscribe_send(PPIB_CH_TIMESTAMP).write(|w| {
        w.set_chidx(dppi_ch as u8);
        w.set_en(true);
    });
}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_local_domain_connections_clear(_dppi_ch: u32) {
    pac::PPIB11
        .subscribe_send(PPIB_CH_TIMESTAMP)
        .write(|w| w.set_en(false));
}

#[no_mangle]
extern "C" fn nrf_802154_platform_timestamper_captured_timestamp_read(
    p_captured: *mut u64,
) -> bool {
    if p_captured.is_null() {
        return false;
    }
    let captured = grtc_timestamp_cc_read();
    if captured == 0 {
        // Still holding the sentinel: no capture has happened.
        return false;
    }
    // SAFETY: null-checked above; the C driver guarantees a valid uint64_t.
    unsafe { p_captured.write(captured) };
    true
}

// =============================================================================
// SL LP Timer
// =============================================================================

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_init() {
    critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).set(u64::MAX));

    let r = pac::GRTC;
    grtc_compare_disarm(GRTC_CC_FIRE);
    grtc_compare_disarm(GRTC_CC_HW_TASK);

    // Clear the whole group, not just our two channels. This driver owns
    // [`GRTC_IRQ_GROUP`] outright, and the GRTC is in the always-on domain: a
    // soft reset leaves its interrupt enables exactly as the previous firmware
    // — quite possibly a previous run of *this* one — left them, while the NVIC
    // line we are about to unmask comes back cleared. An inherited enable on a
    // channel this ISR does not clear would re-fire forever.
    r.intenclr(GRTC_IRQ_GROUP).write(|w| w.0 = u32::MAX);

    // Enable the NVIC line for the LP timer so the ISR can fire. Without this
    // the compare event flags would be set but never serviced.
    unsafe { cortex_m::peripheral::NVIC::unmask(lp_timer_irqn()) };
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_deinit() {
    let r = pac::GRTC;
    r.intenclr(GRTC_IRQ_GROUP)
        .write(|w| w.0 = (1 << GRTC_CC_FIRE) | (1 << GRTC_CC_HW_TASK));
    grtc_compare_disarm(GRTC_CC_FIRE);
    grtc_compare_disarm(GRTC_CC_HW_TASK);
    // The SYSCOUNTER itself belongs to MPSL / the embassy time driver, so it is
    // deliberately left running.
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_current_lpticks_get() -> u64 {
    grtc_syscounter()
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_us_to_lpticks_convert(
    us: u64,
    _round_up: bool,
) -> u64 {
    // The GRTC SYSCOUNTER ticks at 1 MHz, so a lptick *is* a microsecond and
    // there is nothing to round.
    us
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_lpticks_to_us_convert(lpticks: u64) -> u64 {
    lpticks
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_schedule_at(fire_lpticks: u64) {
    critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).set(fire_lpticks));

    // Unlike the nRF52 RTC, the GRTC needs no "too close to now" fixup: per the
    // product spec `EVENTS_COMPARE[n]` is generated immediately when the value
    // written to `CC[n]` is already below the SYSCOUNTER, which is exactly the
    // "fire asap, but still from the lptimer ISR" contract the SL asks for.
    grtc_compare_arm(GRTC_CC_FIRE, fire_lpticks);
    pac::GRTC
        .intenset(GRTC_IRQ_GROUP)
        .write(|w| w.0 = 1 << GRTC_CC_FIRE);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_disable() {
    critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).set(u64::MAX));
    pac::GRTC
        .intenclr(GRTC_IRQ_GROUP)
        .write(|w| w.0 = 1 << GRTC_CC_FIRE);
    grtc_compare_disarm(GRTC_CC_FIRE);
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_prepare(
    fire_lpticks: u64,
    ppi_channel: u32,
) -> u32 {
    if ppi_channel == HW_TASK_PPI_INVALID {
        return LPTIMER_NO_RESOURCES;
    }

    // GRTC COMPARE -> DPPIC20 -> PPIB21 -> PPIB11 -> the radio domain's DPPIC10
    // channel that the SL has hooked the radio task up to.
    hw_task_route(ppi_channel);
    grtc_compare_arm(GRTC_CC_HW_TASK, fire_lpticks);
    LP_HW_TASK_ACTIVE.store(1, Ordering::Release);

    if grtc_syscounter() >= fire_lpticks {
        hw_task_teardown();
        return LPTIMER_TOO_LATE;
    }

    LPTIMER_SUCCESS
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_cleanup() -> u32 {
    if LP_HW_TASK_ACTIVE.load(Ordering::Acquire) == 0 {
        return LPTIMER_WRONG_STATE;
    }
    hw_task_teardown();
    LPTIMER_SUCCESS
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_update_ppi(ppi_channel: u32) -> u32 {
    if LP_HW_TASK_ACTIVE.load(Ordering::Acquire) == 0 {
        return LPTIMER_WRONG_STATE;
    }
    if ppi_channel == HW_TASK_PPI_INVALID {
        return LPTIMER_NO_RESOURCES;
    }
    // Only the radio-domain end of the bridge names the channel, so retargeting
    // is a single register write and the armed compare stays untouched.
    pac::PPIB11.publish_receive(PPIB_CH_HW_TASK).write(|w| {
        w.set_chidx(ppi_channel as u8);
        w.set_en(true);
    });
    LPTIMER_SUCCESS
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_granularity_get() -> u32 {
    // One lptick is one microsecond on the GRTC.
    1
}

/// Wire the GRTC hw-task compare event through to `ppi_channel` on DPPIC10.
fn hw_task_route(ppi_channel: u32) {
    pac::GRTC.publish_compare(GRTC_CC_HW_TASK).write(|w| {
        w.set_chidx(DPPIC20_CH_HW_TASK);
        w.set_en(true);
    });
    pac::PPIB21.subscribe_send(PPIB_CH_HW_TASK).write(|w| {
        w.set_chidx(DPPIC20_CH_HW_TASK);
        w.set_en(true);
    });
    pac::PPIB11.publish_receive(PPIB_CH_HW_TASK).write(|w| {
        w.set_chidx(ppi_channel as u8);
        w.set_en(true);
    });
    pac::DPPIC20
        .chenset()
        .write(|w| w.0 = 1 << DPPIC20_CH_HW_TASK);
}

/// Undo [`hw_task_route`] and disarm the compare channel.
fn hw_task_teardown() {
    pac::DPPIC20
        .chenclr()
        .write(|w| w.0 = 1 << DPPIC20_CH_HW_TASK);
    pac::PPIB11
        .publish_receive(PPIB_CH_HW_TASK)
        .write(|w| w.set_en(false));
    pac::PPIB21
        .subscribe_send(PPIB_CH_HW_TASK)
        .write(|w| w.set_en(false));
    pac::GRTC
        .publish_compare(GRTC_CC_HW_TASK)
        .write(|w| w.set_en(false));
    grtc_compare_disarm(GRTC_CC_HW_TASK);
    LP_HW_TASK_ACTIVE.store(0, Ordering::Release);
}

/// LP Timer (GRTC) interrupt handler implementation.
///
/// Called by `LpTimerInterruptHandler::on_interrupt()` when the GRTC interrupt
/// for [`GRTC_IRQ_GROUP`] fires.
pub(crate) fn lp_timer_isr() {
    extern "C" {
        fn nrf_802154_sl_timer_handler(now_lpticks: u64);
    }

    let r = pac::GRTC;

    if r.events_compare(GRTC_CC_FIRE).read() != 0 {
        // Disarm before dispatching. A GRTC compare whose CC is in the past
        // re-raises its event as soon as it is cleared, so leaving the channel
        // armed here would spin the ISR; and the handler is free to call
        // `schedule_at` again, which re-arms.
        r.intenclr(GRTC_IRQ_GROUP)
            .write(|w| w.0 = 1 << GRTC_CC_FIRE);
        grtc_compare_disarm(GRTC_CC_FIRE);

        let now = grtc_syscounter();
        let fire = critical_section::with(|cs| LP_FIRE_LPTICKS.borrow(cs).get());
        if now >= fire {
            unsafe { nrf_802154_sl_timer_handler(now) };
        }
    }
}
