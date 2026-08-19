# Top-level CMake project wrapping the vendored Nordic `nrf_802154` tree.
#
# Upstream's `nrf_802154/CMakeLists.txt` is written to be pulled into a Zephyr /
# nRF Connect SDK build with `add_subdirectory()`, so it declares neither
# `cmake_minimum_required()` nor `project()` and relies on the enclosing build
# for both. Configuring it directly as the top-level tree — which is what this
# crate does — is only tolerated up to CMake 3.x; CMake 4 turns the missing
# `cmake_minimum_required()` into a hard `CMP0000` error. `build.rs` therefore
# points CMake at this file, which supplies the two commands and then adds the
# upstream tree unmodified.
#
# The policy version stays low deliberately, so that every policy the upstream
# tree predates (notably `CMP0079`, which its cross-directory
# `target_link_libraries()` calls rely on) keeps the OLD behaviour it gets from
# CMake 3.x today. 3.10 rather than 3.5 because CMake 4 deprecates anything
# below 3.10, and the policies in between do not affect this build.
cmake_minimum_required(VERSION 3.10)

# C and CXX only — the same two languages CMake enables for the implicit
# top-level project it invents when a tree has no `project()` of its own.
project(nrf-802154 C CXX)

add_subdirectory(
  "${CMAKE_CURRENT_SOURCE_DIR}/../third_party/nordic/nrfxlib/nrf_802154"
  nrf_802154
)
