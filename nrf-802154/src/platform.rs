// TODO

// Temperature

#[no_mangle]
extern "C" fn nrf_802154_temperature_init() {}

#[no_mangle]
extern "C" fn nrf_802154_temperature_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_temperature_get() -> i8 {
    42
}

// #[no_mangle]
// extern "C" fn nrf_802154_temperature_changed() -> i8 {
//     42
// }

// Clock

#[no_mangle]
extern "C" fn nrf_802154_clock_init() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_is_running() -> bool {
    true
}

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_start() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_stop() {}

// #[no_mangle]
// extern "C" fn nrf_802154_clock_hfclk_ready() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_lfclk_is_running() -> bool {
    true
}

#[no_mangle]
extern "C" fn nrf_802154_clock_lfclk_start() {}

#[no_mangle]
extern "C" fn nrf_802154_clock_lfclk_stop() {}

// #[no_mangle]
// extern "C" fn nrf_802154_clock_lfclk_ready() {}

// HP Timer

#[no_mangle]
extern "C" fn nrf_802154_clock_hfclk_latency_set(_latency: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_init(_latency: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_start() {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_stop() {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_current_time_get() -> u32 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_sync_task_get() -> u32 {
    42
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_sync_prepare() {}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_sync_time_get(_timestamp: &mut i32) -> bool {
    false
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_timestamp_get() -> u32 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_hp_timer_timestamp_task_get() -> u32 {
    42
}

// Timestamper

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
extern "C" fn nrf_802154_platform_timestamper_captured_timestamp_read(_p_captured: &mut u32) -> bool {
    true
}

// SL LP Timer

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_init() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lp_timer_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_current_lpticks_get() -> u64 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_us_to_lpticks_convert(_us: u64, _round_up: bool) -> u64 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_lpticks_to_us_convert(_lpticks: u64) -> u64 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_schedule_at(_fire_lpticks: u64) {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_disable() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_critical_section_enter() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_critical_section_exit() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_prepare(_fire_lpticks: u64, _ppi_channel: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_cleanup() -> u32 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_hw_task_update_ppi(_ppi_channel: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_schedule_now() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_schedule_at(_fire_lpticks: u64) {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_abort() {}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_event_get() -> u32 {
    0
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_sync_lpticks_get() -> u64 {
    42
}

#[no_mangle]
extern "C" fn nrf_802154_platform_sl_lptimer_granularity_get() -> u32 {
    42
}

// extern void nrf_802154_sl_timer_handler(uint64_t now_lpticks);

// extern void nrf_802154_sl_timestamper_synchronized(void);

// IRQ

type IrqCallback = extern "C" fn(*mut ());

#[no_mangle]
extern "C" fn nrf_802154_irq_init(_irqn: u32, _prio: i32, _isr: Option<IrqCallback>) {}

#[no_mangle]
extern "C" fn nrf_802154_irq_enable(_irqn: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_irq_disable(_irqn: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_irq_set_pending(_irqn: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_irq_clear_pending(_irqn: u32) {}

#[no_mangle]
extern "C" fn nrf_802154_irq_is_enabled(_irqn: u32) -> bool {
    false
}

#[no_mangle]
extern "C" fn nrf_802154_irq_priority_get(_irqn: u32) -> i32 {
    0
}

// Random

#[no_mangle]
extern "C" fn nrf_802154_random_init() {}

#[no_mangle]
extern "C" fn nrf_802154_random_deinit() {}

#[no_mangle]
extern "C" fn nrf_802154_random_get() -> u32 {
    42
}

// System Core Clock

const SYSTEM_CLOCK: u32 = 12000000;

#[allow(non_upper_case_globals)]
#[no_mangle]
static mut SystemCoreClock: u32 = SYSTEM_CLOCK;
