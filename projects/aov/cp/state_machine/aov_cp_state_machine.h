#pragma once

#include <common/bk_err.h>
#include "aov_state_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Free-running motion-detect RTC period (ms).
 * Override at compile time, e.g. -DAOV_CP_DETECT_INTERVAL_MS=2000
 */
#ifndef AOV_CP_DETECT_INTERVAL_MS
#define AOV_CP_DETECT_INTERVAL_MS       (1000u)
#endif

/*
 * CP debug GPIO toggled on each detect tick when
 * CONFIG_AOV_DEBUG_IO_ENABLE is enabled (default GPIO39).
 * Note: GPIO30-39 may also be remuxed by AP JPEG; if LA shows extra
 * edges, pick a pin outside that range via -DAOV_CP_DETECT_DBG_GPIO=N.
 */
#ifndef AOV_CP_DETECT_DBG_GPIO
#define AOV_CP_DETECT_DBG_GPIO          (39)
#endif

bk_err_t aov_cp_state_machine_init(const aov_cp_device_ops_t *ops);
bk_err_t aov_cp_state_machine_deinit(void);
bk_err_t aov_cp_state_machine_post_event(const aov_cp_event_t *event);
bk_err_t aov_cp_state_machine_on_ap_report(const aov_ap_report_t *report);

aov_cp_state_t aov_cp_state_machine_get_state(void);
aov_shared_env_t *aov_cp_state_machine_get_shared_env(void);
const char *aov_cp_state_name(aov_cp_state_t state);

int aov_cp_state_machine_cli_init(void);

#ifdef __cplusplus
}
#endif
