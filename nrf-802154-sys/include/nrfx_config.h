// nrfx configuration entry point.
//
// nrfx 4.x expects the consumer to provide `nrfx_config.h`; `nrfx.h` includes
// it unconditionally. It must define the `NRFX_CONFIG_H__` guard (which
// `nrfx_config_common.h` keys off) and then pull in nrfx's own common defaults
// plus the per-chip template.
//
// The per-chip template (`nrfx_config_nrf<chip>.h`, reached via
// `nrfx_templates_config.h`) is what supplies the unsuffixed peripheral aliases
// on TrustZone parts — e.g. on the nRF5340 network core it does
// `#define NRF_RADIO NRF_RADIO_NS`, which the 802.15.4 driver's C relies on
// (`nrfx_get_irq_number(NRF_RADIO)`), since the nRF5340 MDK only declares the
// `_NS`-suffixed instances. In nrfx 3.x this chain was reached implicitly via
// `templates/nrfx_config.h`; nrfx 4.x reorganised it and requires this file.
//
// These are compile-time aliases resolving to the same peripheral addresses, so
// this stays behaviourally identical to nrf-sdc's sibling sys crates on the
// shared MPSL — nrf-sdc simply ships an empty stub because its bindgen/mpsl
// path never compiles C that references the unsuffixed peripheral names.

#ifndef NRFX_CONFIG_H__
#define NRFX_CONFIG_H__

#include <nrfx_config_common.h>
#include <nrfx_templates_config.h>

#endif // NRFX_CONFIG_H__
