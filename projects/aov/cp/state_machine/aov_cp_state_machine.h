#pragma once

#include <common/bk_err.h>
#include "aov_state_protocol.h"

#ifdef __cplusplus
extern "C" {
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
