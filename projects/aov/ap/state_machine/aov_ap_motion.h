#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

int aov_ap_motion_capture_gray(void *user_data, uint8_t *dst, uint32_t size);
int aov_ap_motion_detect(void *user_data,
                         const uint8_t *previous,
                         const uint8_t *current,
                         uint16_t width,
                         uint16_t height,
                         bool *motion);
int aov_ap_motion_stop(void *user_data);

#ifdef __cplusplus
}
#endif
