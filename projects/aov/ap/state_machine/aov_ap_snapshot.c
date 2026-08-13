#include "aov_ap_snapshot.h"

#include <common/bk_err.h>
#include <components/bk_encode/bk_jpeg_encode_ctlr.h>
#include <components/bk_frame_buffer.h>
#include <components/log.h>
#include <driver/aon_rtc.h>
#include <os/mem.h>
#include <stdint.h>

#include "aov_ap_motion.h"

#define TAG "aov_ap_snapshot"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define AOV_AP_SNAPSHOT_WIDTH        (640u)
#define AOV_AP_SNAPSHOT_HEIGHT       (360u)
#define AOV_AP_SNAPSHOT_QUALITY      (5u)

typedef struct {
    volatile uint32_t result;
    volatile uint32_t jpeg_size;
} aov_ap_snapshot_ctx_t;

static void *aov_ap_snapshot_outbuf_malloc(uint32_t size, void *args)
{
    (void)args;

    uint32_t header_size = ((sizeof(frame_buffer_t) + 63U) >> 6) << 6;
    uint32_t frame_size = header_size + size;
    frame_buffer_t *frame = (frame_buffer_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_CODED, frame_size);
    if (frame == NULL) {
        LOGE("alloc jpeg outbuf failed, size=%u\n", (unsigned)size);
        return NULL;
    }

    frame->frame = ((uint8_t *)frame) + header_size;
    frame->size = size;
    frame->length = 0;
    frame->width = AOV_AP_SNAPSHOT_WIDTH;
    frame->height = AOV_AP_SNAPSHOT_HEIGHT;
    return frame->frame;
}

static uint32_t aov_ap_snapshot_outbuf_complete(bk_jpeg_encode_outbuf_info_t *info)
{
    if (info == NULL || info->outbuf == NULL) {
        return BK_FAIL;
    }

    aov_ap_snapshot_ctx_t *ctx = (aov_ap_snapshot_ctx_t *)info->args;
    if (ctx != NULL) {
        ctx->result = info->status;
        ctx->jpeg_size = info->length;
    }

    LOGI("jpeg frame callback status=%u size=%u\n",
         (unsigned)info->status, (unsigned)info->length);

    uint32_t header_size = ((sizeof(frame_buffer_t) + 63U) >> 6) << 6;
    frame_buffer_t *frame = (frame_buffer_t *)((uint8_t *)info->outbuf - header_size);

    bk_frame_buffer_free(frame);
    return BK_OK;
}

int aov_ap_snapshot_capture(void *user_data)
{
    (void)user_data;

    uint8_t *input = aov_ap_motion_get_sp_frame();
    if (input == NULL) {
        LOGE("no valid SP frame for jpeg encode\n");
        return BK_ERR_STATE;
    }

    aov_ap_snapshot_ctx_t ctx = {
        .result = (uint32_t)~0U,
        .jpeg_size = 0,
    };
    bk_jpeg_encode_frame_config_t config;
    bk_jpeg_encode_input_t enc_in;
    bk_jpeg_encode_ctlr_handle_t handle = NULL;
    os_memset(&config, 0, sizeof(config));
    os_memset(&enc_in, 0, sizeof(enc_in));

    config.width = AOV_AP_SNAPSHOT_WIDTH;
    config.height = AOV_AP_SNAPSHOT_HEIGHT;
    config.input_format = BK_PIXEL_FORMAT_NV12;
    config.input_buf = (uint32_t)(uintptr_t)input;
    config.input_size = AOV_AP_SNAPSHOT_WIDTH * AOV_AP_SNAPSHOT_HEIGHT * 3U / 2U;
    config.quality = AOV_AP_SNAPSHOT_QUALITY;
    config.outbuf_malloc = aov_ap_snapshot_outbuf_malloc;
    config.outbuf_complete = aov_ap_snapshot_outbuf_complete;
    config.outbuf_complete_args = &ctx;

    uint64_t start_us = bk_aon_rtc_get_us();
    uint64_t encode_cost_us = 0;
    int ret = bk_jpeg_encode_frame_new(&handle, &config);
    if (ret == BK_OK) {
        ret = bk_jpeg_encode_init(handle);
    }
    if (ret == BK_OK) {
        ret = bk_jpeg_encode_open(handle);
    }
    if (ret == BK_OK) {
        uint64_t encode_start_us = bk_aon_rtc_get_us();
        ret = bk_jpeg_encode_frame(handle, &enc_in);
        encode_cost_us = bk_aon_rtc_get_us() - encode_start_us;
    }

    if (handle != NULL) {
        (void)bk_jpeg_encode_close(handle);
        (void)bk_jpeg_encode_deinit(handle);
        (void)bk_jpeg_encode_delete(handle);
    }

    uint64_t cost_us = bk_aon_rtc_get_us() - start_us;
    if (ret != BK_OK || ctx.result != BK_OK || ctx.jpeg_size == 0) {
        LOGE("jpeg frame encode failed ret=%d callback=%u size=%u encode=%llu total=%llu us\n",
             ret, (unsigned)ctx.result, (unsigned)ctx.jpeg_size,
             (unsigned long long)encode_cost_us,
             (unsigned long long)cost_us);
        return (ret != BK_OK) ? ret : BK_FAIL;
    }

    LOGI("jpeg frame encode ok size=%u %ux%u encode=%llu total=%llu us\n",
         (unsigned)ctx.jpeg_size,
         AOV_AP_SNAPSHOT_WIDTH,
         AOV_AP_SNAPSHOT_HEIGHT,
         (unsigned long long)encode_cost_us,
         (unsigned long long)cost_us);
    return BK_OK;
}

int aov_ap_snapshot_stop(void *user_data)
{
    (void)user_data;
    return BK_OK;
}
