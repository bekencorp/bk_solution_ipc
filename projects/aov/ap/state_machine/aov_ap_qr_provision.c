#include "aov_ap_qr_provision.h"
#include <common/bk_include.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_isp_camera.h>
#include <components/log.h>
#include <driver/aon_rtc.h>
#include <modules/zbar/zbar_qr.h>
#include <os/mem.h>
#include <os/os.h>
#include <stdbool.h>
#include <stdint.h>
#include "app_camera.h"
#define TAG "aov_ap_qr"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define AOV_QR_WIDTH             (640u)
#define AOV_QR_HEIGHT            (360u)
#define AOV_QR_FORMAT            BK_PIXEL_FORMAT_NV12
#define AOV_QR_GRAY_SIZE         (AOV_QR_WIDTH * AOV_QR_HEIGHT)
#define AOV_QR_READ_TIMEOUT_MS   (1000u)
#define AOV_QR_SCAN_INTERVAL_MS  (100u)
#define AOV_QR_THREAD_PRIORITY   BEKEN_DEFAULT_WORKER_PRIORITY
#define AOV_QR_THREAD_STACK_SIZE (1024u * 8u)
#define AOV_QR_MAX_RESULTS       (4u)
#define AOV_QR_PAYLOAD_SIZE      (8896u)
typedef struct {
    beken_thread_t thread;
    volatile bool running;
    bool opened;
    bool owns_camera;
    bool decoded_once;
    uint32_t frame_index;
    uint8_t *frame;
    uint32_t frame_size;
    zbar_qr_context_t *decoder;
    zbar_qr_result_t results[AOV_QR_MAX_RESULTS];
    uint8_t *payloads;
} aov_ap_qr_ctx_t;
static aov_ap_qr_ctx_t s_qr_ctx;
static int aov_ap_qr_camera_open(void)
{
    camera_board_config_t *config = app_camera_board_config_get();
    if (config == NULL) {
        return BK_ERR_STATE;
    }
    config->isp.sp_enable = true;
    config->isp.sp_flexa = false;
    config->isp.sp_width = AOV_QR_WIDTH;
    config->isp.sp_height = AOV_QR_HEIGHT;
    config->isp.sp_format = AOV_QR_FORMAT;
    if (app_isp_handle_get() == NULL) {
        int ret = app_isp_mipi_camera_sp_turn_on(config);
        if (ret != BK_OK) {
            LOGE("open SP-only MIPI camera failed: %d\n", ret);
        } else {
            s_qr_ctx.owns_camera = true;
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
static int aov_ap_qr_open_buffers(void)
{
    s_qr_ctx.frame_size =
        bk_image_size_get(AOV_QR_WIDTH, AOV_QR_HEIGHT, AOV_QR_FORMAT);
    if (s_qr_ctx.frame_size < AOV_QR_GRAY_SIZE) {
        LOGE("invalid QR frame size=%u\n", (unsigned)s_qr_ctx.frame_size);
        return BK_ERR_PARAM;
    }
    s_qr_ctx.frame = bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED,
                                            s_qr_ctx.frame_size);
    if (s_qr_ctx.frame == NULL) {
        LOGE("malloc QR frame failed, size=%u\n",
             (unsigned)s_qr_ctx.frame_size);
        return BK_ERR_NO_MEM;
    }
    size_t payloads_size = AOV_QR_MAX_RESULTS * AOV_QR_PAYLOAD_SIZE;
    s_qr_ctx.payloads = os_malloc(payloads_size);
    if (s_qr_ctx.payloads == NULL) {
        LOGE("malloc QR payload buffers failed, size=%u\n",
             (unsigned)payloads_size);
        return BK_ERR_NO_MEM;
    }
    for (uint32_t i = 0; i < AOV_QR_MAX_RESULTS; i++) {
        s_qr_ctx.results[i].payload =
            s_qr_ctx.payloads + i * AOV_QR_PAYLOAD_SIZE;
        s_qr_ctx.results[i].payload_capacity = AOV_QR_PAYLOAD_SIZE;
        s_qr_ctx.results[i].payload_len = 0;
    }
    s_qr_ctx.decoder = zbar_qr_create();
    if (s_qr_ctx.decoder == NULL) {
        LOGE("zbar_qr_create failed\n");
        return BK_ERR_NO_MEM;
    }
    LOGI("QR input ready: SP %ux%u NV12 frame=%u gray=%u\n",
         AOV_QR_WIDTH, AOV_QR_HEIGHT, (unsigned)s_qr_ctx.frame_size,
         (unsigned)AOV_QR_GRAY_SIZE);
    return BK_OK;
}
static void aov_ap_qr_log_result(int count, unsigned long long scan_us)
{
    LOGI("frame=%u found %d QR code(s), scan=%llu us\n",
         (unsigned)s_qr_ctx.frame_index, count, scan_us);
    for (int i = 0; i < count; i++) {
        const zbar_qr_result_t *result = &s_qr_ctx.results[i];
        LOGI("QR[%d] corners=(%d,%d),(%d,%d),(%d,%d),(%d,%d)\n",
             i, result->corners[0].x, result->corners[0].y,
             result->corners[1].x, result->corners[1].y,
             result->corners[2].x, result->corners[2].y,
             result->corners[3].x, result->corners[3].y);
        LOGI("QR[%d] len=%u data=%.*s\n",
             i, (unsigned)result->payload_len,
             (int)result->payload_len, (const char *)result->payload);
    }
}
static void aov_ap_qr_process_frame(void)
{
    unsigned long long start_us = bk_aon_rtc_get_us();
    int count = zbar_qr_scan(s_qr_ctx.decoder, s_qr_ctx.frame,
                             AOV_QR_WIDTH, AOV_QR_HEIGHT, AOV_QR_WIDTH,
                             s_qr_ctx.results, AOV_QR_MAX_RESULTS);
    s_qr_ctx.frame_index++;
    if (count < 0) {
        LOGW("zbar scan failed=%d\n", count);
        return;
    }
    if (count == 0) {
        return;
    }
    aov_ap_qr_log_result(count, bk_aon_rtc_get_us() - start_us);
    s_qr_ctx.decoded_once = true;
}
static void aov_ap_qr_thread_entry(beken_thread_arg_t arg)
{
    (void)arg;
    LOGI("QR provision SP thread start\n");
    while (s_qr_ctx.running) {
        int ret = app_isp_camera_channel_read(APP_ISP_SP_CHN_ID,
                                              s_qr_ctx.frame,
                                              s_qr_ctx.frame_size,
                                              AOV_QR_READ_TIMEOUT_MS);
        if (ret != BK_OK) {
            if (s_qr_ctx.running) {
                LOGE("read ISP SP frame failed: %d\n", ret);
                rtos_delay_milliseconds(10);
            }
            continue;
        }
        aov_ap_qr_process_frame();
        rtos_delay_milliseconds(AOV_QR_SCAN_INTERVAL_MS);
    }
    LOGI("QR provision SP thread exit, decoded=%u frames=%u\n",
         (unsigned)s_qr_ctx.decoded_once, (unsigned)s_qr_ctx.frame_index);
    s_qr_ctx.thread = NULL;
    rtos_delete_thread(NULL);
}
static void aov_ap_qr_free_resources(void)
{
    if (s_qr_ctx.decoder != NULL) {
        zbar_qr_destroy(s_qr_ctx.decoder);
        s_qr_ctx.decoder = NULL;
    }
    if (s_qr_ctx.payloads != NULL) {
        os_free(s_qr_ctx.payloads);
        s_qr_ctx.payloads = NULL;
    }
    if (s_qr_ctx.frame != NULL) {
        bk_frame_buffer_free(s_qr_ctx.frame);
        s_qr_ctx.frame = NULL;
    }
    s_qr_ctx.frame_size = 0;
}
int aov_ap_qr_provision_start(void *user_data)
{
    (void)user_data;
    if (s_qr_ctx.opened) {
        LOGW("QR provision already started\n");
        return BK_ERR_BUSY;
    }
    os_memset(&s_qr_ctx, 0, sizeof(s_qr_ctx));
    int ret = aov_ap_qr_camera_open();
    if (ret != BK_OK) {
        goto error;
    }
    ret = aov_ap_qr_open_buffers();
    if (ret != BK_OK) {
        goto error;
    }
    s_qr_ctx.running = true;
    ret = rtos_create_thread(&s_qr_ctx.thread,
                             AOV_QR_THREAD_PRIORITY,
                             "aov_qr",
                             aov_ap_qr_thread_entry,
                             AOV_QR_THREAD_STACK_SIZE,
                             NULL);
    if (ret != BK_OK) {
        LOGE("create QR provision thread failed: %d\n", ret);
        s_qr_ctx.running = false;
        goto error;
    }
    s_qr_ctx.opened = true;
    LOGI("QR provision started on SP %ux%u\n", AOV_QR_WIDTH, AOV_QR_HEIGHT);
    return BK_OK;
error:
    aov_ap_qr_provision_stop(NULL);
    os_memset(&s_qr_ctx, 0, sizeof(s_qr_ctx));
    LOGE("QR provision start failed: %d\n", ret);
    return ret;
}
int aov_ap_qr_provision_stop(void *user_data)
{
    (void)user_data;
    if (s_qr_ctx.thread != NULL) {
        s_qr_ctx.running = false;
        while (s_qr_ctx.thread != NULL) {
            rtos_delay_milliseconds(10);
        }
    }
    aov_ap_qr_free_resources();
    if (s_qr_ctx.owns_camera && app_isp_handle_get() != NULL) {
        int ret = app_isp_camera_turn_off();
        if (ret != BK_OK) {
            LOGE("turn off QR camera failed: %d\n", ret);
            return ret;
        }
    }
    bool had_opened = s_qr_ctx.opened;
    os_memset(&s_qr_ctx, 0, sizeof(s_qr_ctx));
    if (had_opened) {
        LOGI("QR provision stopped\n");
    }
    return BK_OK;
}
