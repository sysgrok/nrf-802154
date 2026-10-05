# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
* Breaking: CSL (Thread 1.2 Synchronized Sleepy End Device) support in `OpenThreadRadio`
* Breaking: `Radio`: everything necessary for Thread CSL support - scheduled receival and enhanced ACKs
* `Radio::sleep` / `enter_receive` no longer cancel a scheduled receive window; `receive_at_cancel` does
* `OpenThreadRadio`: CSL transmitter support for FTDs - timed transmit (`Capabilities::TRANSMIT_TIMING`) and OpenThread's CSMA-CA backoff limit (a single CCA for frames timed into a CSL child's window); new `Radio::transmit_at_with` and `Radio::set_csma_ca_max_backoffs`
* Fix: a captured ACK was one byte short (the driver reports its PSDU length, not the buffer length), which made every secured enhanced ACK fail its MIC check

## [0.1.0] - 2026-09-14

- Initial release
