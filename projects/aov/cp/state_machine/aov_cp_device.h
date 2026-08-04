#pragma once

#include <common/bk_err.h>
#include <stdbool.h>
#include "aov_state_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

const aov_cp_device_ops_t *aov_cp_device_ops_get(void);

bk_err_t aov_cp_device_set_credentials_state(bool present,
                                              bool credential_valid,
                                              bool binding_valid);
bk_err_t aov_cp_device_set_wifi_credentials(const char *ssid,
                                             const char *password);
bk_err_t aov_cp_device_clear_wifi_credentials(void);
bk_err_t aov_cp_device_factory_reset_complete(void);
int aov_cp_device_cli_init(void);

#ifdef __cplusplus
}
#endif
