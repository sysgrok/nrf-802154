# Change Log

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]
* Breaking: the bindings no longer declare `memset`, `memcpy`, `memmove` and `memcmp` (newer Rust warns that their signatures do not match the runtime's)
* Fix: build on macOS - archive with `llvm-ar` from the `llvm-tools` rustup component (Apple's `ar` drops ELF objects)

## [0.1.0] - 2026-09-14

- Initial release
