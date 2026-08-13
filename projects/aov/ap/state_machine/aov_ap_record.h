#pragma once
#ifdef __cplusplus
extern "C" {
#endif
int aov_ap_record_start(void *user_data);
int aov_ap_record_capture_snapshot(void *user_data);
int aov_ap_record_stop(void *user_data);
#ifdef __cplusplus
}
#endif
