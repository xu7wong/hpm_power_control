#ifndef _POWER_CONTROL_LOW_LEVEL_H
#define _POWER_CONTROL_LOW_LEVEL_H

#include "hpm_common.h"
#include "hpm_soc.h"
#include "hpm_soc_feature.h"

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

void power_control_init(void);

void adc_clear_done(uint8_t index);

uint8_t adc_get_done(uint8_t index);
#if defined(__cplusplus)
}
#endif /* __cplusplus */
#endif /* _POWER_CONTROL_LOW_LEVEL_H */
