// The minimal amount of glue necessary when building the nRF 802.15.4 radio driver.

#ifndef __NRFX_GLUE_H__
#define __NRFX_GLUE_H__

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>

#define NRFX_ASSERT(condition) assert(condition)

// The atomic glue nrfx expects its integrator to supply (see nrfx's
// `templates/nrfx_glue.h`). The driver's peer-record map stores an index
// through it; the compiler builtins map straight onto the exclusive
// load/store instructions every supported core has.
typedef uint32_t nrfx_atomic_t;

#define NRFX_ATOMIC_FETCH_STORE(p_data, value) __atomic_exchange_n((p_data), (value), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_OR(p_data, value) __atomic_fetch_or((p_data), (value), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_AND(p_data, value) __atomic_fetch_and((p_data), (value), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_XOR(p_data, value) __atomic_fetch_xor((p_data), (value), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_ADD(p_data, value) __atomic_fetch_add((p_data), (value), __ATOMIC_SEQ_CST)
#define NRFX_ATOMIC_FETCH_SUB(p_data, value) __atomic_fetch_sub((p_data), (value), __ATOMIC_SEQ_CST)

static inline bool nrfx_glue_atomic_cas(nrfx_atomic_t * p_data, nrfx_atomic_t old_value, nrfx_atomic_t new_value)
{
    return __atomic_compare_exchange_n(p_data, &old_value, new_value, false, __ATOMIC_SEQ_CST, __ATOMIC_SEQ_CST);
}

#define NRFX_ATOMIC_CAS(p_data, old_value, new_value) nrfx_glue_atomic_cas((p_data), (old_value), (new_value))

#endif // __NRFX_GLUE_H__
