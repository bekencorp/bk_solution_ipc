#include "aov_ap_record.h"
#include <common/bk_err.h>
#include <components/bk_flexa_bond.h>
#include <components/log.h>
#include <driver/aon_rtc.h>
#include <os/mem.h>
#include <stdbool.h>
#include <stdint.h>
#include "app_camera.h"
#include "app_codec.h"
#include "bk_snapshot.h"
#define TAG "aov_ap_record"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define AOV_AP_RECORD_JPEG_QUALITY     (5u)
#define AOV_AP_RECORD_SNAPSHOT_TIMEOUT (3000u)
typedef struct {
    void *isp_handle;
    bk_h264_encode_ctlr_handle_t h264_handle;
    void *h264e_bond;
    bool h264_started;
} aov_ap_record_ctx_t;
static aov_ap_record_ctx_t s_record_ctx;
static int aov_ap_record_start_h264(void)
{
    int ret;
    if (s_record_ctx.h264_started && s_record_ctx.h264e_bond != NULL) {
        return BK_OK;
    }
    s_record_ctx.isp_handle = app_isp_handle_get();
    if (s_record_ctx.isp_handle == NULL) {
        LOGE("ISP handle is NULL\n");
        return BK_ERR_STATE;
    }
    if (!s_record_ctx.h264_started) {
        ret = app_h264e_turn_on();
        if (ret != BK_OK) {
            LOGE("app_h264e_turn_on failed: %d\n", ret);
            return ret;
        }
        s_record_ctx.h264_started = true;
    }
    s_record_ctx.h264_handle =
        (bk_h264_encode_ctlr_handle_t)app_h264_encode_handle_get();
    if (s_record_ctx.h264_handle == NULL) {
        LOGE("H264 encode handle is NULL\n");
        aov_ap_record_stop(NULL);
        return BK_ERR_STATE;
    }
    if (s_record_ctx.h264e_bond == NULL) {
        ret = bk_flexa_isp_h264e_bond_start(&s_record_ctx.h264e_bond,
                                            s_record_ctx.isp_handle,
                                            s_record_ctx.h264_handle);
        if (ret != BK_OK) {
            LOGE("start ISP-H264 bond failed: %d\n", ret);
            aov_ap_record_stop(NULL);
            return ret;
        }
    }
    return BK_OK;
}
int aov_ap_record_start(void *user_data)
{
    (void)user_data;
    return aov_ap_record_start_h264();
}
int aov_ap_record_capture_snapshot(void *user_data)
{
    int ret = aov_ap_record_start(user_data);
    if (ret != BK_OK) {
        return ret;
    }
    bk_snapshot_config_t config = {
        .source_handle = s_record_ctx.isp_handle,
        .h264_handle = s_record_ctx.h264_handle,
        .h264_bond = &s_record_ctx.h264e_bond,
        .jpeg_quality = AOV_AP_RECORD_JPEG_QUALITY,
        .timeout_ms = AOV_AP_RECORD_SNAPSHOT_TIMEOUT,
    };
    bk_snapshot_image_t image = {0};
    uint64_t start_us = bk_aon_rtc_get_us();
    ret = bk_snapshot_capture(&config, &image);
    uint64_t cost_us = bk_aon_rtc_get_us() - start_us;
    uint32_t image_size = image.size;
    LOGI("snapshot ret=%d size=%u %ux%u cost=%llu us\n",
         ret,
         (unsigned)image.size,
         (unsigned)image.width,
         (unsigned)image.height,
         (unsigned long long)cost_us);
    if (image.data != NULL) {
        bk_snapshot_image_release(&image);
    }
    if (ret != BK_OK || image_size == 0) {
        return (ret != BK_OK) ? ret : BK_FAIL;
    }
    return BK_OK;
}
int aov_ap_record_stop(void *user_data)
{
    (void)user_data;
    if (s_record_ctx.h264e_bond != NULL) {
        bk_flexa_isp_h264e_bond_stop(s_record_ctx.h264e_bond);
        s_record_ctx.h264e_bond = NULL;
    }
    if (s_record_ctx.h264_started) {
        int ret = app_h264e_turn_off();
        if (ret != BK_OK) {
            LOGE("app_h264e_turn_off failed: %d\n", ret);
            return ret;
        }
    }
    os_memset(&s_record_ctx, 0, sizeof(s_record_ctx));
    return BK_OK;
}
