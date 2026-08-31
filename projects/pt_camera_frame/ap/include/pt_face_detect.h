#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PT_FACE_DETECT_MAX_BOXES 1

typedef struct {
    float x;
    float y;
    float w;
    float h;
    float score;
} pt_face_detect_box_t;

typedef struct {
    uint32_t frame_id;
    uint32_t timestamp_ms;
    uint16_t width;
    uint16_t height;
    uint8_t detected;
    uint8_t count;
    pt_face_detect_box_t boxes[PT_FACE_DETECT_MAX_BOXES];
} pt_face_detect_result_t;

bk_err_t pt_face_detect_init(void);
bk_err_t pt_face_detect_start(void);
bk_err_t pt_face_detect_stop(void);
bk_err_t pt_face_detect_get_result(pt_face_detect_result_t *result);

#ifdef __cplusplus
}
#endif
