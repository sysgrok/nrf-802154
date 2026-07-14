// Freestanding stub for `<errno.h>`.
//
// nrfx 4.x added `#include <errno.h>` to `nrfx.h`, but neither nrfx nor the
// 802.15.4 driver actually references any `errno`/`E*` symbols in the code paths
// compiled here. This empty shim just satisfies the include in a `-none-eabi`
// (no libc) build. nrf-sdc's sibling sys crates ship the same empty stub.

#ifndef __ERRNO_H__
#define __ERRNO_H__

#endif // __ERRNO_H__
