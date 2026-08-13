#include "aov_ap_motion.h"

#include <common/bk_include.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_isp_camera.h>
#include <components/log.h>
#include <driver/aon_rtc.h>
#include <modules/vg_lite_gpu/vg_lite.h>
#include <os/mem.h>
#include <os/os.h>

#include "aov_state_protocol.h"
#include "app_camera.h"
#include "app_gpu.h"
#include "modules/motion_detect.h"

#define TAG "aov_ap_motion"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define AOV_MOTION_SP_WIDTH               (640u)
#define AOV_MOTION_SP_HEIGHT              (360u)
#define AOV_MOTION_FORMAT                 BK_PIXEL_FORMAT_NV12
#define AOV_MOTION_READ_TIMEOUT_MS        (1000u)
#define AOV_MOTION_WARMUP_READ_COUNT      (5u)
#define AOV_MOTION_DIFF_THRESHOLD         (64u)
#define AOV_MOTION_COUNT_THRESHOLD        (128u)
#define AOV_MOTION_COUNT_BUFFER_SIZE      \
    ((AOV_GRAY_WIDTH / MOTION_DETECT_GROUP_PIXELS) * AOV_GRAY_HEIGHT)

static uint8_t s_motion_counts[AOV_MOTION_COUNT_BUFFER_SIZE];
static bool s_motion_camera_warmed_up;
static uint8_t *s_motion_sp_frame;

static int aov_ap_motion_gpu_init(void)
{
    if (app_gpu_handle_get() != NULL) {
        return BK_OK;
    }

    gpu_board_config_t config = {0};
    config.flexa.enable = true;
    config.flexa.degree = 0;
    config.flexa.src_width = AOV_MOTION_SP_WIDTH;
    config.flexa.src_height = AOV_MOTION_SP_HEIGHT;
    config.flexa.dst_width = AOV_GRAY_WIDTH;
    config.flexa.dst_height = AOV_GRAY_HEIGHT;
    config.flexa.src_format = AOV_MOTION_FORMAT;
    config.flexa.dst_format = BK_PIXEL_FORMAT_ARGB8888;
    config.flexa.scale = false;

    return app_gpu_turn_on(&config);
}

static void aov_ap_motion_gpu_deinit(void)
{
    bk_gpu_ctlr_handle_t handle = app_gpu_handle_get();
    if (handle == NULL) {
        return;
    }

    (void)app_gpu_turn_off(handle);
}

static int aov_ap_motion_blit_gray(const uint8_t *src, uint8_t *dst)
{
    if (src == NULL || dst == NULL) {
        return BK_ERR_PARAM;
    }

    uint64_t total_start_us = bk_aon_rtc_get_us();
    uint64_t init_start_us = bk_aon_rtc_get_us();
    int ret = aov_ap_motion_gpu_init();
    uint64_t init_cost_us = bk_aon_rtc_get_us() - init_start_us;
    if (ret != BK_OK) {
        return ret;
    }

    vg_lite_buffer_t src_buf;
    vg_lite_buffer_t dst_buf;
    vg_lite_matrix_t matrix;
    os_memset(&src_buf, 0, sizeof(src_buf));
    os_memset(&dst_buf, 0, sizeof(dst_buf));
    os_memset(&matrix, 0, sizeof(matrix));

    src_buf.width = AOV_MOTION_SP_WIDTH;
    src_buf.height = AOV_MOTION_SP_HEIGHT;
    src_buf.stride = AOV_MOTION_SP_WIDTH;
    src_buf.format = VG_LITE_NV12;
    src_buf.compress_mode = VG_LITE_DEC_DISABLE;
    src_buf.tiled = VG_LITE_LINEAR;
    src_buf.yuv.uv_stride = AOV_MOTION_SP_WIDTH;
    src_buf.yuv.uv_height = AOV_MOTION_SP_HEIGHT / 2U;

    uint8_t *uv = (uint8_t *)src + (AOV_MOTION_SP_WIDTH * AOV_MOTION_SP_HEIGHT);
    vg_lite_error_t vg_ret =
        vg_lite_allocate_with_data(&src_buf, (void *)src, uv, NULL, NULL);
    if (vg_ret != VG_LITE_SUCCESS) {
        LOGE("wrap SP source failed: %d\n", (int)vg_ret);
        return BK_FAIL;
    }

    dst_buf.width = AOV_GRAY_WIDTH;
    dst_buf.height = AOV_GRAY_HEIGHT;
    dst_buf.stride = AOV_GRAY_STRIDE;
    dst_buf.format = VG_LITE_L8;
    dst_buf.compress_mode = VG_LITE_DEC_DISABLE;
    dst_buf.tiled = VG_LITE_LINEAR;

    vg_ret = vg_lite_allocate_with_data(&dst_buf, dst, NULL, NULL, NULL);
    if (vg_ret != VG_LITE_SUCCESS) {
        LOGE("wrap gray destination failed: %d\n", (int)vg_ret);
        (void)vg_lite_free_without_free_data(&src_buf);
        return BK_FAIL;
    }

    vg_lite_identity(&matrix);
    vg_lite_scale((float)AOV_GRAY_WIDTH / (float)AOV_MOTION_SP_WIDTH,
                  (float)AOV_GRAY_HEIGHT / (float)AOV_MOTION_SP_HEIGHT,
                  &matrix);

    uint64_t blit_start_us = bk_aon_rtc_get_us();
    vg_ret = vg_lite_blit(&dst_buf, &src_buf, &matrix,
                          VG_LITE_BLEND_NONE, 0, VG_LITE_FILTER_POINT);
    if (vg_ret == VG_LITE_SUCCESS) {
        vg_ret = vg_lite_finish();
    }
    uint64_t blit_cost_us = bk_aon_rtc_get_us() - blit_start_us;

    (void)vg_lite_free_without_free_data(&dst_buf);
    (void)vg_lite_free_without_free_data(&src_buf);
    if (vg_ret != VG_LITE_SUCCESS) {
        LOGE("VG blit gray failed: %d\n", (int)vg_ret);
        return BK_FAIL;
    }

    LOGI("gpu gray convert cost init=%llu blit=%llu total=%llu us\n",
         (unsigned long long)init_cost_us,
         (unsigned long long)blit_cost_us,
         (unsigned long long)(bk_aon_rtc_get_us() - total_start_us));
    return BK_OK;
}

static int aov_ap_motion_camera_open(void)
{
    camera_board_config_t *config = app_camera_board_config_get();

    if (config == NULL) {
        return BK_ERR_STATE;
    }

    config->isp.sp_enable = true;
    config->isp.sp_flexa = false;
    config->isp.sp_width = AOV_MOTION_SP_WIDTH;
    config->isp.sp_height = AOV_MOTION_SP_HEIGHT;
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

    uint32_t frame_size = bk_image_size_get(AOV_MOTION_SP_WIDTH, AOV_MOTION_SP_HEIGHT,
                                             AOV_MOTION_FORMAT);
    if (frame_size == 0) {
        return BK_ERR_PARAM;
    }

    if (s_motion_sp_frame == NULL) {
        s_motion_sp_frame = bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, frame_size);
        if (s_motion_sp_frame == NULL) {
            return BK_ERR_NO_MEM;
        }
    }

    if (!s_motion_camera_warmed_up) {
        uint64_t start_us = bk_aon_rtc_get_us();
        for (uint32_t i = 0; i < AOV_MOTION_WARMUP_READ_COUNT; i++) {
            ret = app_isp_camera_channel_read(APP_ISP_SP_CHN_ID, s_motion_sp_frame,
                                              frame_size,
                                              AOV_MOTION_READ_TIMEOUT_MS);
            if (ret != BK_OK) {
                LOGE("warmup read ISP SP frame %u/%u failed: %d\n",
                     (unsigned)(i + 1),
                     (unsigned)AOV_MOTION_WARMUP_READ_COUNT,
                     ret);
                return ret;
            }
        }
        LOGI("warmup read ISP SP frame %u times cost %llu us\n",
             (unsigned)AOV_MOTION_WARMUP_READ_COUNT,
             (unsigned long long)(bk_aon_rtc_get_us() - start_us));
        s_motion_camera_warmed_up = true;
    } else {
        ret = app_isp_camera_channel_read(APP_ISP_SP_CHN_ID, s_motion_sp_frame,
                                          frame_size,
                                          AOV_MOTION_READ_TIMEOUT_MS);
    }
    if (ret == BK_OK) {
        ret = aov_ap_motion_blit_gray(s_motion_sp_frame, dst);
    } else {
        LOGE("read ISP SP frame failed: %d\n", ret);
    }

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
    uint64_t start_us = bk_aon_rtc_get_us();
    int ret = motion_detect_compare_gray(&config, current, previous,
                                         s_motion_counts, &result);
    uint64_t cost_us = bk_aon_rtc_get_us() - start_us;
    if (ret != MOTION_DETECT_OK) {
        LOGE("motion compare failed: %d cost=%llu us\n",
             ret, (unsigned long long)cost_us);
        return BK_FAIL;
    }

    *motion = result.moving != 0;
    LOGI("motion=%u total=%u max_block=%u count=%u cost=%llu us\n",
         (unsigned)*motion, (unsigned)result.total_count,
         (unsigned)result.max_block_index,
         (unsigned)result.max_block_count,
         (unsigned long long)cost_us);
    return BK_OK;
}

uint8_t *aov_ap_motion_get_sp_frame(void)
{
    return s_motion_sp_frame;
}

int aov_ap_motion_stop(void *user_data)
{
    (void)user_data;
    s_motion_camera_warmed_up = false;
    if (s_motion_sp_frame != NULL) {
        bk_frame_buffer_free(s_motion_sp_frame);
        s_motion_sp_frame = NULL;
    }
    aov_ap_motion_gpu_deinit();
    if (app_isp_handle_get() == NULL) {
        return BK_OK;
    }

    return app_isp_camera_turn_off();
}
