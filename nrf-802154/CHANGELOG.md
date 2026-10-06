# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
* Fix (nRF52/nRF53): an LP timer event the driver scheduled could silently never fire - the RTC compare was armed for the counter's next tick, which the RTC does not reliably match - leaving whatever the driver timed by it (a CSMA-CA backoff, say) waiting for good
* Breaking: CSL (Thread 1.2 Synchronized Sleepy End Device) support in `OpenThreadRadio`
* Breaking: `Radio`: everything necessary for Thread CSL support - scheduled receival and enhanced ACKs
* `Radio::sleep` / `enter_receive` no longer cancel a scheduled receive window; `receive_at_cancel` does
* `OpenThreadRadio`: CSL transmitter support for FTDs - timed transmit (`Capabilities::TRANSMIT_TIMING`) and OpenThread's CSMA-CA backoff limit (a single CCA for frames timed into a CSL child's window); new `Radio::transmit_at_with` and `Radio::set_csma_ca_max_backoffs`
* `PsduMeta` gains `acked_frame_pending`: whether the driver acknowledged the frame with Frame Pending set (reported to OpenThread, whose parent serves a sleepy child's data poll only when it was)
* `Radio::last_tx_frame_props`: what the driver did to the last transmitted frame (frame counter / key index / CSL IE set, secured) - reported for failed transmissions too; `OpenThreadRadio` hands it to OpenThread either way, which repeats the frame counter of an unacknowledged frame to a sleepy child when retransmitting it (without, the retransmission went out with frame counter and key index 0 and was refused with `KEY_ID_INVALID`)
* Fix: received frames carried no timestamp (`PsduMeta::time` was always `None`), which a CSL parent times its transmissions to the child by
* Fix: timed transmission (`Radio::transmit_at_with`, i.e. the CSL transmitter) never went out - refused, or failed with `TimeslotDenied`: the LP timer's hw-task compare event was never connected to the (D)PPI channel that starts the radio's ramp-up (the nRF52/nRF53 platform ignored the channel, the nRF54L one refused the "channel to follow" request the driver opens with)
* Fix: a timed receive window left the receiver on after it ended, and retuning the channel for a transmission or the next window aborted a window still running
* Fix: a captured ACK was one byte short (the driver reports its PSDU length, not the buffer length), which made every secured enhanced ACK fail its MIC check

## [0.1.0] - 2026-09-14

- Initial release
