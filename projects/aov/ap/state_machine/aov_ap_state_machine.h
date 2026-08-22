#pragma once

#include <common/bk_err.h>
#include "aov_state_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

bk_err_t aov_ap_state_machine_init(const aov_ap_backend_ops_t *ops);
bk_err_t aov_ap_state_machine_start(void);
bk_err_t aov_ap_state_machine_deinit(void);

aov_ap_job_t aov_ap_state_machine_get_pending_job(void);
aov_ap_state_t aov_ap_state_machine_get_state(void);
const char *aov_ap_state_name(aov_ap_state_t state);
bk_err_t aov_ap_state_machine_get_ae_warm_start(
    aov_ae_warm_start_t *info);
bk_err_t aov_ap_state_machine_save_ae_warm_start(
    const aov_ae_warm_start_t *info);

bk_err_t aov_ap_state_machine_report_qr_credential(const void *data, uint16_t len);
bk_err_t aov_ap_state_machine_report_wifi_result(bool connected, int result);
bk_err_t aov_ap_state_machine_start_motion_check(void);
bk_err_t aov_ap_state_machine_notify_event_done(int result);
bk_err_t aov_ap_state_machine_notify_live_stopped(int result);

bk_err_t aov_ap_state_machine_test_complete_qr_provision(void);

#ifdef __cplusplus
}
#endif
