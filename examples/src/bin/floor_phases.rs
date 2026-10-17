//! Scratch diagnostic: where the sleep floor of the Thread stack comes from.
//!
//! With the XIAO's QSPI flash in deep power-down, steps through phases of ~15 s each, so that one
//! power-profiler recording shows the floor of every phase:
//! A. MPSL initialized (LF crystal), nothing else;
//! B. the 802.15.4 driver created, the radio never used (sleeping, no RX when idle);
//! C. a 1.5 ms timed receive window every second, like a CSL receiver;
//! D. windows stopped, idle - then, ~10 s apart, one more candidate for a leftover load switched
//!    off at a time: the TIMERs stopped, the TIMERs shut down, `POWER.TASKS_LOWPWR`, the radio
//!    powered off, all PPI channels disabled.
//!
//! For a range of leads, schedules timed receive windows from a sleeping radio
//! (HFXO off) and counts how many the driver accepts - first with MPSL's default
//! HFCLK latency, then with the typical one.

#![no_std]
#![no_main]

use defmt::info;

use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_time::Timer;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::Radio;
use nrf_802154_examples::Irqs;
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

const CHANNEL: u8 = 26;

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut p = nrf_802154_examples::init();

    // The XIAO's QSPI flash: deep power-down (0xB9), bit-banged in SPI mode 0, then keep its
    // inputs driven.
    {
        const DEEP_POWER_DOWN: u8 = 0xb9;

        let mut csn = Output::new(p.P0_25, Level::High, OutputDrive::Standard);
        let mut sck = Output::new(p.P0_21, Level::Low, OutputDrive::Standard);
        let mut si = Output::new(p.P0_20, Level::Low, OutputDrive::Standard);
        let wp = Output::new(p.P0_22, Level::High, OutputDrive::Standard);
        let hold = Output::new(p.P0_23, Level::High, OutputDrive::Standard);

        csn.set_low();

        for bit in (0..8).rev() {
            si.set_level(Level::from(DEEP_POWER_DOWN & (1 << bit) != 0));
            sck.set_high();
            sck.set_low();
        }

        csn.set_high();

        for pin in [csn, sck, si, wp, hold] {
            pin.persist();
        }
    }

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_XTAL as u8,
        rc_ctiv: 0,
        rc_temp_ctiv: 0,
        accuracy_ppm: 20,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = nrf_802154_examples::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    info!("PHASE A: MPSL only");
    Timer::after_secs(15).await;

    let mut radio_p = nrf_802154_examples::radio_peripherals!(p);
    let mut radio = Radio::new(p.RADIO.reborrow(), radio_p.reborrow(), Irqs, mpsl);
    radio.set_channel(CHANNEL);
    // As for a sleepy child: the driver's default is to receive whenever idle
    radio.set_rx_when_idle(false);
    radio.sleep();

    info!("PHASE B: radio driver, never used");
    Timer::after_secs(15).await;

    info!("PHASE C: a timed receive window every second");
    let mut accepted = 0;
    for _ in 0..15 {
        let start = radio.now_us() + 5000;
        if radio.receive_at(start, 1500, CHANNEL) {
            accepted += 1;
        }

        Timer::after_secs(1).await;
    }
    info!("PHASE C: {}/15 windows accepted", accepted);

    info!("PHASE D: idle after radio use");

    const TIMERS: [(&str, usize); 5] = [
        ("TIMER0", 0x4000_8000),
        ("TIMER1", 0x4000_9000),
        ("TIMER2", 0x4000_A000),
        ("TIMER3", 0x4001_A000),
        ("TIMER4", 0x4001_B000),
    ];

    let reg = |addr: usize| addr as *mut u32;

    // A TIMER has no status register: capture its counter twice, some time apart
    for (name, base) in TIMERS {
        let capture = || unsafe {
            reg(base + 0x04C).write_volatile(1); // TASKS_CAPTURE[3]
            reg(base + 0x54C).read_volatile() // CC[3]
        };

        let first = capture();
        embassy_time::block_for(embassy_time::Duration::from_millis(10));
        let second = capture();

        info!(
            "{}: {} -> {} ({})",
            name,
            first,
            second,
            if first != second {
                "RUNNING"
            } else {
                "stopped"
            }
        );
    }

    Timer::after_secs(10).await;

    info!("PHASE D2: TIMERs stopped");
    for (_, base) in TIMERS {
        unsafe { reg(base + 0x004).write_volatile(1) }; // TASKS_STOP
    }
    Timer::after_secs(10).await;

    info!("PHASE D3: TIMERs shut down");
    for (_, base) in TIMERS {
        unsafe { reg(base + 0x010).write_volatile(1) }; // TASKS_SHUTDOWN
    }
    Timer::after_secs(10).await;

    info!("PHASE D4: POWER.TASKS_LOWPWR");
    unsafe { reg(0x4000_007C).write_volatile(1) };
    Timer::after_secs(10).await;

    info!("PHASE D5: RADIO.POWER = 0");
    unsafe { reg(0x4000_1FFC).write_volatile(0) };
    Timer::after_secs(10).await;

    info!("PHASE D6: all PPI channels disabled");
    unsafe { reg(0x4001_F508).write_volatile(u32::MAX) }; // CHENCLR
    loop {
        Timer::after_secs(3600).await;
    }
}
