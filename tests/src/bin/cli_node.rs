//! The nRF firmware node of the `openthread` repo's HIL test tier, on the
//! `nrf-802154` driver.
//!
//! Same purpose - and same console/CLI/reset contract - as that repo's
//! `tests/nrf` node.
//!
//! The upstream harness drives this exactly as it drives every other node -
//! CLI lines in over the board's own USB (CDC-ACM) console, CLI output back.
//! The `openthread` repo's `serial_bridge` makes that substitution invisible
//! to the harness:
//!
//! ```sh
//! # over in the openthread repo, with this firmware flashed:
//! cargo xtask itest --hw-port /dev/serial/by-id/<this-board>=mcu \
//!                   --hw-port /dev/serial/by-id/<ot-rcp-dongle>=rcp
//! ```
//!
//! # Reset
//!
//! The CLI `reset`/`factoryreset` commands are intercepted (the C stack
//! cannot re-create itself in place) and honored with a chip reset - after
//! awaiting the settings flush, since persistence is write-behind here (see
//! `settings`). The settings live in the same flash page, with the same
//! image format, as the `openthread` repo's nRF node, so a board keeps its
//! settings across a switch between the two firmwares.
//!
//! # Node identity
//!
//! The EUI-64 comes from the chip's factory device address
//! (`FICR.DEVICEADDR`), unique per board and stable across resets; the
//! node-id-to-board mapping lives entirely in the bridge's port map.

#![no_std]
#![no_main]

use embassy_executor::Spawner;

use embassy_nrf::mode::Blocking;
use embassy_nrf::rng::Rng;
use embassy_nrf::{peripherals, usb};

use embassy_usb::class::cdc_acm::{self, CdcAcmClass};
use embassy_usb::UsbDevice;

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::{OpenThreadRadio, Radio};
use nrf_802154_tests::{console, settings, Irqs};
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::{Flash, MultiprotocolServiceLayer, Peripherals as MpslPeripherals};

use openthread::{OpenThread, OtResources};

use static_cell::StaticCell;

use tinyrlibc as _;

use {defmt_rtt as _, panic_probe as _};

macro_rules! mk_static {
    ($t:ty,$val:expr) => {{
        static STATIC_CELL: static_cell::StaticCell<$t> = static_cell::StaticCell::new();
        #[deny(unused_attributes)]
        let x = STATIC_CELL.uninit().write($val);
        x
    }};
}

/// The flash page `memory.x` carves off the top of the application region -
/// out of the linker's FLASH, so the firmware can never grow into it.
const SETTINGS_OFFSET: u32 = 0x000f_f000;

/// The console's two ends: the board's own USB peripheral (CDC-ACM).
type UsbDriver = usb::Driver<'static, &'static usb::vbus_detect::SoftwareVbusDetect>;
type ConsoleTx = cdc_acm::Sender<'static, UsbDriver>;
type ConsoleRx = cdc_acm::Receiver<'static, UsbDriver>;

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();
static SETTINGS_STATE: settings::FlashSettingsState = settings::FlashSettingsState::new();

// Only needed for tinyrlibc's alloc functions, which are not called at runtime.
#[global_allocator]
static HEAP: Heap = Heap::empty();

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = MpslPeripherals::new(p.RTC0, p.TIMER0, p.TEMP, p.PPI_CH19, p.PPI_CH30, p.PPI_CH31);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(run_mpsl(mpsl).unwrap());

    // Flash access must be MPSL-coordinated (it runs in radio-free
    // timeslots), which is what makes settings persistence write-behind -
    // see `settings`.
    let mut flash = Flash::take(mpsl, p.NVMC);

    let ot_settings_buf = mk_static!([u8; 1024], [0; 1024]);
    let ot_settings = mk_static!(
        settings::FlashSettings,
        settings::FlashSettings::new(
            &SETTINGS_STATE,
            &mut flash,
            SETTINGS_OFFSET,
            ot_settings_buf
        )
    );

    spawner.spawn(run_settings(flash).unwrap());

    let rng = mk_static!(Rng<'static, Blocking>, Rng::new_blocking(p.RNG));

    let ieee_eui64 = ieee_eui64();

    let ot_resources = mk_static!(OtResources, OtResources::new());
    let ot = OpenThread::new(ieee_eui64, rng, ot_settings, ot_resources).unwrap();

    // The full-MAC radio goes straight to the stack: no software MAC, no
    // executor split.
    let radio = OpenThreadRadio::new(Radio::new(p.RADIO, p.EGU0, Irqs, mpsl, p.TIMER2, p.RTC2));
    spawner.spawn(run_ot(ot.clone(), radio).unwrap());

    let (console_tx, console_rx, usb) = build_usb_console(p.USBD);
    spawner.spawn(run_usb(usb).unwrap());
    spawner.spawn(run_console_out(console_tx).unwrap());

    ot.cli_init(console::out);

    // The prompt the harness waits for on connect.
    console::out(b"\r\n> ");

    run_cli(ot, console_rx).await
}

/// Read CLI lines off the console and hand them to the interpreter.
///
/// `reset`/`factoryreset` are handled here with a chip reset - see the module
/// docs.
async fn run_cli(ot: OpenThread<'static>, mut console_rx: ConsoleRx) -> ! {
    let mut reader = console::LineReader::new();
    let mut buf = [0; 64];

    loop {
        console_rx.wait_connection().await;
        let Ok(len) = console_rx.read_packet(&mut buf).await else {
            continue;
        };

        for byte in &buf[..len] {
            if reader.push(*byte) {
                let line = reader.line().trim();

                match line {
                    // The chip reset both need happens here - see the module
                    // docs. `factoryreset` goes through the stack first: its
                    // factory-reset path is what wipes the settings store.
                    // Persistence is write-behind, so await the flush before
                    // rebooting - that is what makes either reset durable.
                    "reset" => {
                        settings::flush(&SETTINGS_STATE).await;
                        cortex_m::peripheral::SCB::sys_reset()
                    }
                    "factoryreset" => {
                        let _ = ot.cli_input_line(line);
                        settings::flush(&SETTINGS_STATE).await;
                        cortex_m::peripheral::SCB::sys_reset()
                    }
                    "" => (),
                    _ => {
                        let _ = ot.cli_input_line(line);
                    }
                }

                reader.clear();

                // Let the response drain fully before accepting the next
                // command - see `console::drained`.
                console::drained().await;
            }
        }
    }
}

/// Drain pending CLI output to the console.
///
/// A task rather than a direct write from the output callback, because that
/// callback is synchronous while the console is not - see `console`.
#[embassy_executor::task]
async fn run_console_out(mut console_tx: ConsoleTx) -> ! {
    // Big chunks off the pipe: fewer task round-trips keeps the drain ahead
    // of the CLI; USB is packetized below.
    let mut buf = [0; 512];

    loop {
        console_tx.wait_connection().await;

        let len = console::read_out(&mut buf).await;

        // How long a packet write may sit unaccepted before the output is
        // dropped. Generous next to USB speeds, so it only fires when nobody
        // is reading at all.
        const TX_STALL: embassy_time::Duration = embassy_time::Duration::from_millis(500);

        // One packet per `write_packet` - it does not split - and a
        // zero-length packet after a final full-size one: a bulk transfer
        // only ends on a *short* packet. The writes are bounded because the
        // host may not be reading at all; blocking forever would back up
        // `drained` and the command loop with it. (See the `openthread`
        // repo's nRF node for the full story.)
        let mps = console_tx.max_packet_size() as usize;

        let mut sent = 0;
        while sent < len {
            let chunk = &buf[sent..(sent + mps).min(len)];

            match embassy_time::with_timeout(TX_STALL, console_tx.write_packet(chunk)).await {
                Ok(Ok(())) => sent += chunk.len(),
                _ => break,
            }
        }

        if sent == len && sent % mps == 0 {
            let _ = embassy_time::with_timeout(TX_STALL, console_tx.write_packet(&[])).await;
        }
    }
}

/// Build the USB CDC console: the board's own USB peripheral presented to
/// the host as a serial port.
///
/// `SoftwareVbusDetect` rather than the hardware one: the `POWER` events
/// belong to MPSL here (see `Irqs`) - and this board is USB-powered, so
/// VBUS is present by definition.
fn build_usb_console(
    usbd: embassy_nrf::Peri<'static, peripherals::USBD>,
) -> (ConsoleTx, ConsoleRx, UsbDevice<'static, UsbDriver>) {
    let vbus = mk_static!(
        usb::vbus_detect::SoftwareVbusDetect,
        usb::vbus_detect::SoftwareVbusDetect::new(true, true)
    );

    let driver = usb::Driver::new(usbd, Irqs, &*vbus);

    let mut config = embassy_usb::Config::new(0x1209, 0x0001);
    config.manufacturer = Some("openthread");
    config.product = Some("openthread test node");
    config.serial_number = None;
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    let mut builder = embassy_usb::Builder::new(
        driver,
        config,
        mk_static!([u8; 256], [0; 256]),
        mk_static!([u8; 256], [0; 256]),
        mk_static!([u8; 128], [0; 128]),
        mk_static!([u8; 128], [0; 128]),
    );

    let class = CdcAcmClass::new(
        &mut builder,
        mk_static!(cdc_acm::State, cdc_acm::State::new()),
        64,
    );

    let usb = builder.build();
    let (console_tx, console_rx) = class.split();

    (console_tx, console_rx, usb)
}

/// The chip's factory device address, as an EUI-64.
fn ieee_eui64() -> [u8; 8] {
    let ficr = embassy_nrf::pac::FICR;

    let low = ficr.deviceaddr(0).read();
    let high = ficr.deviceaddr(1).read();

    let mut eui64 = [0; 8];
    eui64[..4].copy_from_slice(&low.to_be_bytes());
    eui64[4..].copy_from_slice(&high.to_be_bytes());

    // Locally administered, unicast - this is a device address, not an OUI one.
    eui64[0] = (eui64[0] & 0xfe) | 0x02;

    eui64
}

#[embassy_executor::task]
async fn run_mpsl(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[embassy_executor::task]
async fn run_settings(flash: Flash<'static>) -> ! {
    settings::run(&SETTINGS_STATE, flash, SETTINGS_OFFSET).await
}

#[embassy_executor::task]
async fn run_usb(mut usb: UsbDevice<'static, UsbDriver>) -> ! {
    usb.run().await
}

#[embassy_executor::task]
async fn run_ot(ot: OpenThread<'static>, radio: OpenThreadRadio<'static>) -> ! {
    ot.run(radio).await
}
