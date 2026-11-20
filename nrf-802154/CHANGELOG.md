# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
* Breaking: remove the `openthread` feature and `OpenThreadRadio`: the OpenThread radio is `NrfRadio` in the new `openthread-driver-nrf` crate of the `openthread` repository
* Fix (nRF52/nRF53): an LP timer event the driver scheduled could silently never fire - the RTC compare was armed for the counter's next tick, which the RTC does not reliably match - leaving whatever the driver timed by it (a CSMA-CA backoff, say) waiting for good
* Breaking: `Radio`: everything necessary for Thread CSL support - scheduled receival and enhanced ACKs
* `Radio::sleep` / `enter_receive` no longer cancel a scheduled receive window; `receive_at_cancel` does
* `Radio::transmit_at_with` (timed transmit) and `Radio::set_csma_ca_max_backoffs`, for the CSL transmitter of an FTD
* `PsduMeta` gains `acked_frame_pending`: whether the driver acknowledged the frame with Frame Pending set (reported to OpenThread, whose parent serves a sleepy child's data poll only when it was)
* `Radio::last_tx_frame_props`: what the driver did to the last transmitted frame (frame counter / key index / CSL IE set, secured), also for failed transmissions
* Fix: received frames carried no timestamp (`PsduMeta::time` was always `None`), which a CSL parent times its transmissions to the child by
* Fix: timed transmission (`Radio::transmit_at_with`, i.e. the CSL transmitter) never went out - refused, or failed with `TimeslotDenied`: the LP timer's hw-task compare event was never connected to the (D)PPI channel that starts the radio's ramp-up (the nRF52/nRF53 platform ignored the channel, the nRF54L one refused the "channel to follow" request the driver opens with)
* Fix: a timed receive window left the receiver on after it ended, and retuning the channel for a transmission or the next window aborted a window still running
* Fix: a captured ACK was one byte short (the driver reports its PSDU length, not the buffer length), which made every secured enhanced ACK fail its MIC check

## [0.1.0] - 2026-09-14

- Initial release
