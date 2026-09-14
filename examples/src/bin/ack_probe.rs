//! Acknowledgement probe: is the board on the other side acknowledging
//! everything it should, every time?
//!
//! Run `receive_frame` on the board under test and this on a second board.
//! It sends a few hundred unicast frames with the ACK request bit, cycling
//! through frame lengths up to the maximum and alternating plain and
//! link-secured headers (a well-formed auxiliary security header over dummy
//! ciphertext - the receiver does not decrypt, but its driver has to parse the
//! whole thing before it may ACK). Every frame must come back acknowledged.
//!
//! What this guards: the driver starts its ACK from a hardware-timed slot
//! 192 us after the frame ends, and it has roughly 130 us of that to parse,
//! filter and prepare the ACK inside the radio interrupt. Miss the slot and
//! the ACK is not sent at all - silently, and only visible to a sleepy child
//! or a peer that gives up on retries. Anything that makes that path slower
//! (a blocking platform hook, a heavier driver release, a slower clock) shows
//! up here as missing ACKs before it shows up as a failing Thread network.

#![no_std]
#![no_main]

use defmt::{assert, info, panic};

use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use embedded_alloc::LlffHeap as Heap;

use nrf_802154::{Error, Radio, TxError};
use nrf_802154_examples::{build_data_frame, Irqs};
use nrf_mpsl::raw::mpsl_clock_lfclk_cfg_t;
use nrf_mpsl::MultiprotocolServiceLayer;

use static_cell::StaticCell;

use {defmt_rtt as _, panic_probe as _};

/// Same network as `receive_frame` / `send_frame`.
const CHANNEL: u8 = 15;
const PAN_ID: u16 = 0x4242;
const DST_SHORT_ADDR: u16 = 0x2323;
const SRC_SHORT_ADDR: u16 = 0x0001;

const FRAMES: usize = 200;
/// Payload lengths cycled through. The last one fills a plain frame (9-byte
/// header) to the largest PSDU the driver takes (the FCS is added on top);
/// the secured variant (15-byte header plus a 4-byte MIC) is trimmed to fit.
const PAYLOAD_LENS: [usize; 4] = [8, 40, 80, 115];
const MAX_FRAME_LEN: usize = nrf_802154::MAX_PSDU_SIZE;
const PLAIN_OVERHEAD: usize = 9;
const SECURED_OVERHEAD: usize = 15 + 4;
const FRAME_PENDING_BIT: u8 = 0x10;

static MPSL: StaticCell<MultiprotocolServiceLayer<'static>> = StaticCell::new();

#[embassy_executor::task]
async fn mpsl_task(mpsl: &'static MultiprotocolServiceLayer<'static>) -> ! {
    mpsl.run().await
}

#[global_allocator]
static HEAP: Heap = Heap::empty();

/// A data frame with a link-security header (ENC-MIC-32, key id mode 1) over
/// `payload_len` bytes of dummy ciphertext plus a dummy MIC.
fn build_secured_frame(seq: u8, frame_counter: u32, payload_len: usize, buf: &mut [u8]) -> usize {
    let fcf: u16 = 0x0001 | 0x0008 | 0x0020 | 0x0040 | 0x0800 | 0x8000;
    buf[0] = fcf as u8;
    buf[1] = (fcf >> 8) as u8;
    buf[2] = seq;
    buf[3..5].copy_from_slice(&PAN_ID.to_le_bytes());
    buf[5..7].copy_from_slice(&DST_SHORT_ADDR.to_le_bytes());
    buf[7..9].copy_from_slice(&SRC_SHORT_ADDR.to_le_bytes());
    // Auxiliary security header: security level 5, key id mode 1, frame
    // counter, key index
    buf[9] = 0x0d;
    buf[10..14].copy_from_slice(&frame_counter.to_le_bytes());
    buf[14] = 1;
    let mut len = 15;
    for i in 0..payload_len {
        buf[len + i] = (seq as usize + i) as u8;
    }
    len += payload_len;
    // MIC
    buf[len..len + 4].copy_from_slice(&[0xaa; 4]);
    len + 4
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = nrf_802154_examples::init();

    let lfclk_cfg = mpsl_clock_lfclk_cfg_t {
        source: nrf_mpsl::raw::MPSL_CLOCK_LF_SRC_RC as u8,
        rc_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_CTIV as u8,
        rc_temp_ctiv: nrf_mpsl::raw::MPSL_RECOMMENDED_RC_TEMP_CTIV as u8,
        accuracy_ppm: nrf_mpsl::raw::MPSL_DEFAULT_CLOCK_ACCURACY_PPM as u16,
        skip_wait_lfclk_started: nrf_mpsl::raw::MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED != 0,
    };

    let mpsl_p = nrf_802154_examples::mpsl_peripherals!(p);
    let mpsl = MPSL.init(MultiprotocolServiceLayer::new(mpsl_p, Irqs, lfclk_cfg).unwrap());
    spawner.spawn(mpsl_task(mpsl).unwrap());

    let mut radio = Radio::new(
        p.RADIO,
        nrf_802154_examples::radio_peripherals!(p),
        Irqs,
        mpsl,
    );
    radio.set_channel(CHANNEL);
    radio.set_pan_id(Some(PAN_ID));
    radio.set_short_addr(Some(SRC_SHORT_ADDR));

    info!(
        "ACK probe: {} frames to 0x{:04x} on channel {}",
        FRAMES, DST_SHORT_ADDR, CHANNEL
    );

    let mut frame = [0u8; nrf_802154::MAX_PSDU_SIZE];
    let mut ack = [0u8; nrf_802154::MAX_PSDU_SIZE];
    let mut payload = [0u8; nrf_802154::MAX_PSDU_SIZE];
    let mut acked = 0usize;
    let mut no_ack = 0usize;
    let mut pending = 0usize;

    for i in 0..FRAMES {
        let seq = i as u8;
        let payload_len = PAYLOAD_LENS[i % PAYLOAD_LENS.len()];
        let secured = i % 2 == 1;

        let len = if secured {
            let payload_len = payload_len.min(MAX_FRAME_LEN - SECURED_OVERHEAD);
            build_secured_frame(seq, i as u32, payload_len, &mut frame)
        } else {
            let payload_len = payload_len.min(MAX_FRAME_LEN - PLAIN_OVERHEAD);
            payload[..payload_len].fill(seq);
            build_data_frame(
                seq,
                PAN_ID,
                DST_SHORT_ADDR,
                SRC_SHORT_ADDR,
                true,
                &payload[..payload_len],
                &mut frame,
            )
            .unwrap()
        };

        match radio.transmit(&frame[..len], true, Some(&mut ack)).await {
            Ok(Some(meta)) => {
                acked += 1;
                if meta.len >= 1 && ack[0] & FRAME_PENDING_BIT != 0 {
                    pending += 1;
                }
            }
            Ok(None) => panic!("frame {}: acknowledged, but no ACK frame handed back", seq),
            Err(Error::Transmit(TxError::NoAck)) => {
                no_ack += 1;
                info!("frame {} ({} bytes, secured={}): NO ACK", seq, len, secured);
            }
            Err(e) => panic!("frame {}: transmit failed: {:?}", seq, e),
        }

        Timer::after(Duration::from_millis(20)).await;
    }

    info!(
        "ACK probe: {} of {} acknowledged ({} not acknowledged, {} ACKs with frame pending set)",
        acked, FRAMES, no_ack, pending
    );
    assert!(
        no_ack == 0,
        "ACK PROBE FAILED: {} of {} frames not acknowledged",
        no_ack,
        FRAMES
    );
    info!(
        "ACK PROBE PASSED: {} of {} frames acknowledged",
        acked, FRAMES
    );

    loop {
        Timer::after_secs(1).await;
    }
}
