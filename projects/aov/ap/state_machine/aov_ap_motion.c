#include "aov_ap_motion.h"

#include <common/bk_include.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_isp_camera.h>
#include <components/log.h>
#include <os/mem.h>
#include <os/os.h>

#include "aov_state_protocol.h"
#include "app_camera.h"
#include "modules/motion_detect.h"

#define TAG "aov_ap_motion"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define AOV_MOTION_FORMAT                 BK_PIXEL_FORMAT_NV12
#define AOV_MOTION_READ_TIMEOUT_MS        (1000u)
#define AOV_MOTION_WARMUP_MS              (1000u)
#define AOV_MOTION_DIFF_THRESHOLD         (64u)
#define AOV_MOTION_COUNT_THRESHOLD        (128u)
#define AOV_MOTION_COUNT_BUFFER_SIZE      \
    ((AOV_GRAY_WIDTH / MOTION_DETECT_GROUP_PIXELS) * AOV_GRAY_HEIGHT)

static uint8_t s_motion_counts[AOV_MOTION_COUNT_BUFFER_SIZE];
static bool s_motion_camera_warmed_up;

static int aov_ap_motion_camera_open(void)
{
    camera_board_config_t *config = app_camera_board_config_get();

    if (config == NULL) {
        return BK_ERR_STATE;
    }

    config->isp.sp_enable = true;
    config->isp.sp_flexa = false;
    config->isp.sp_width = AOV_GRAY_WIDTH;
    config->isp.sp_height = AOV_GRAY_HEIGHT;
    config->isp.sp_format = AOV_MOTION_FORMAT;

    if (app_isp_handle_get() == NULL) {
        int ret = app_isp_mipi_camera_sp_turn_on(config);
        if (ret != BK_OK) {
            LOGE("open SP-only MIPI camera failed: %d\n", ret);
        }
        return ret;
    }

    bk_isp_camera_ctlr_handle_t handle = app_isp_camera_ctlr_handle_get();
    if (handle != NULL &&
        bk_isp_camera_channel_state_get(handle, APP_ISP_SP_CHN_ID) ==
            ISP_CHANNEL_STATE_TURN_ON) {
        return BK_OK;
    }

    int ret = app_isp_camera_sp_channel_turn_on(config);
    if (ret != BK_OK) {
        LOGE("open ISP SP channel failed: %d\n", ret);
    }
    return ret;
}

int aov_ap_motion_capture_gray(void *user_data, uint8_t *dst, uint32_t size)
{
    (void)user_data;
    if (dst == NULL || size < AOV_GRAY_BUFFER_SIZE) {
        return BK_ERR_PARAM;
    }

    int ret = aov_ap_motion_camera_open();
    if (ret != BK_OK) {
        return ret;
    }

    uint32_t frame_size = bk_image_size_get(AOV_GRAY_WIDTH, AOV_GRAY_HEIGHT,
                                             AOV_MOTION_FORMAT);
    if (frame_size == 0) {
        return BK_ERR_PARAM;
    }
    uint8_t *frame = bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, frame_size);
    if (frame == NULL) {
        return BK_ERR_NO_MEM;
    }

    if (!s_motion_camera_warmed_up) {
        rtos_delay_milliseconds(AOV_MOTION_WARMUP_MS);
        s_motion_camera_warmed_up = true;
    }
    ret = app_isp_camera_channel_read(APP_ISP_SP_CHN_ID, frame, frame_size,
                                      AOV_MOTION_READ_TIMEOUT_MS);
    if (ret == BK_OK) {
        os_memcpy(dst, frame, AOV_GRAY_BUFFER_SIZE);
        LOGI("captured gray frame %ux%u\n", AOV_GRAY_WIDTH, AOV_GRAY_HEIGHT);
    } else {
        LOGE("read ISP SP frame failed: %d\n", ret);
    }

    bk_frame_buffer_free(frame);
    return ret;
}

int aov_ap_motion_detect(void *user_data,
                         const uint8_t *previous,
                         const uint8_t *current,
                         uint16_t width,
                         uint16_t height,
                         bool *motion)
{
    (void)user_data;
    if (current == NULL || motion == NULL ||
        width != AOV_GRAY_WIDTH || height != AOV_GRAY_HEIGHT) {
        return BK_ERR_PARAM;
    }

    if (previous == NULL) {
        *motion = false;
        return BK_OK;
    }

    motion_detect_config_t config = {
        .width = width,
        .height = height,
        .diff_threshold = AOV_MOTION_DIFF_THRESHOLD,
        .count_threshold = AOV_MOTION_COUNT_THRESHOLD,
    };
    motion_detect_result_t result = {0};
    int ret = motion_detect_compare_gray(&config, current, previous,
                                         s_motion_counts, &result);
    if (ret != MOTION_DETECT_OK) {
        LOGE("motion compare failed: %d\n", ret);
        return BK_FAIL;
    }

    *motion = result.moving != 0;
    LOGI("motion=%u total=%u max_block=%u count=%u\n",
         (unsigned)*motion, (unsigned)result.total_count,
         (unsigned)result.max_block_index,
         (unsigned)result.max_block_count);
    return BK_OK;
}

int aov_ap_motion_stop(void *user_data)
{
    (void)user_data;
    s_motion_camera_warmed_up = false;
    if (app_isp_handle_get() == NULL) {
        return BK_OK;
    }

    return app_isp_camera_turn_off();
}
