//! Shared initialization helpers for the nRF52840 IEEE 802.15.4 examples.

#![no_std]

use embassy_nrf::bind_interrupts;

bind_interrupts!(pub struct Irqs {
    // MPSL low-priority and 802.15.4 EGU handler share EGU0_SWI0
    EGU0_SWI0 => nrf_mpsl::LowPrioInterruptHandler, nrf_802154::Egu0InterruptHandler;
    // MPSL high-priority handlers
    RADIO => nrf_mpsl::HighPrioInterruptHandler;
    TIMER0 => nrf_mpsl::HighPrioInterruptHandler;
    RTC0 => nrf_mpsl::HighPrioInterruptHandler;
    // MPSL clock handler
    CLOCK_POWER => nrf_mpsl::ClockInterruptHandler;
    // 802.15.4 LP timer (RTC2 on nRF52840)
    RTC2 => nrf_802154::LpTimerInterruptHandler;
});

/// MAC header size in bytes: FCF (2) + seq (1) + dest PAN (2) + dest addr (2) + src addr (2).
const MAC_HEADER_LEN: usize = 9;

/// Build an IEEE 802.15.4 Data frame with short addressing and PAN ID compression.
///
/// # Arguments
/// - `seq`: Sequence number
/// - `dst_pan`: Destination PAN ID
/// - `dst_addr`: Destination short address
/// - `src_addr`: Source short address
/// - `ack`: Whether to request an ACK
/// - `payload`: Frame payload (MSDU)
/// - `buf`: Buffer to write the frame into
///
/// # Returns
/// `Some(len)` with the number of bytes written to `buf`, or `None` if the payload
/// exceeds `MAX_PSDU_SIZE` or the buffer is too small to hold the frame.
pub fn build_data_frame(
    seq: u8,
    dst_pan: u16,
    dst_addr: u16,
    src_addr: u16,
    ack: bool,
    payload: &[u8],
    buf: &mut [u8],
) -> Option<usize> {
    let frame_len = MAC_HEADER_LEN + payload.len();
    if frame_len > nrf_802154::MAX_PSDU_SIZE || frame_len > buf.len() {
        return None;
    }

    // Frame Control Field:
    //   bits  0-2 : Frame Type = 001 (Data)
    //   bit     5 : ACK Request
    //   bit     6 : PAN ID Compression (source PAN omitted, same as dest PAN)
    //   bits 10-11: Destination Addressing Mode = 10 (short)
    //   bits 12-13: Frame Version = 00 (IEEE 802.15.4-2003)
    //   bits 14-15: Source Addressing Mode = 10 (short)
    let fcf: u16 = 0x0001              // Data frame type
        | if ack { 0x0020 } else { 0 } // ACK request
        | 0x0040                       // PAN ID Compression
        | 0x0800                       // Dst Addressing Mode: short (bit 11 = 1)
        | 0x8000; // Src Addressing Mode: short (bit 15 = 1)

    buf[0] = (fcf & 0xFF) as u8;
    buf[1] = (fcf >> 8) as u8;
    buf[2] = seq;
    buf[3] = (dst_pan & 0xFF) as u8;
    buf[4] = (dst_pan >> 8) as u8;
    buf[5] = (dst_addr & 0xFF) as u8;
    buf[6] = (dst_addr >> 8) as u8;
    // Source PAN ID is omitted due to PAN ID Compression
    buf[7] = (src_addr & 0xFF) as u8;
    buf[8] = (src_addr >> 8) as u8;
    buf[MAC_HEADER_LEN..frame_len].copy_from_slice(payload);
    Some(frame_len)
}
