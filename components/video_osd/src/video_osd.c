#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include <time.h>
#include <components/log.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_encode/bk_h264_encode_ctlr.h>
#include <components/bk_encode/bk_h264_encode_types.h>
#include <driver/aon_rtc.h>
#include "cache.h"
#include "bk_osd_emwin_font.h"
#include "video_osd.h"
#include "video_osd_font.h"

#define TAG "video-osd"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define VIDEO_OSD_TIME_LEN      32
#define VIDEO_OSD_SLOT          0U
#define VIDEO_OSD_DEFAULT_TIME  "2027-07-31 00:00:00"
#define VIDEO_OSD_FONT_COLOR    0xFFFFFFU
#define VIDEO_OSD_X             16U
#define VIDEO_OSD_Y             16U
#define VIDEO_OSD_REFRESH_MS    1000U

typedef struct {
    bk_h264_encode_ctlr_handle_t enc;
    time_t offset_sec;
    char last_str[VIDEO_OSD_TIME_LEN];
    bool active;
    bool lock_inited;
    bool sem_inited;
    bool worker_inited;
    bool timer_inited;
    beken_mutex_t lock;
    beken_semaphore_t tick_sem;
    beken_thread_t worker;
    beken_timer_t timer;
} video_osd_ctx_t;

static video_osd_ctx_t s_ctx;

static bool epoch_is_leap(int year)
{
    return ((year % 4) == 0 && (year % 100) != 0) || ((year % 400) == 0);
}

static time_t epoch_from_parts(int year, int mon, int day, int hour, int min, int sec)
{
    static const int days_before[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    time_t days = 0;
    int i;

    if (year < 1970 || mon < 1 || mon > 12 || day < 1 || day > 31) {
        return (time_t)-1;
    }

    for (i = 1970; i < year; i++) {
        days += epoch_is_leap(i) ? 366 : 365;
    }

    days += days_before[mon - 1];
    if (mon > 2 && epoch_is_leap(year)) {
        days++;
    }
    days += (day - 1);

    return days * 86400 + hour * 3600 + min * 60 + sec;
}

static int parse_two_digits(const char *p, int *out)
{
    if (p[0] < '0' || p[0] > '9' || p[1] < '0' || p[1] > '9') {
        return -1;
    }
    *out = (p[0] - '0') * 10 + (p[1] - '0');
    return 0;
}

static int parse_four_digits(const char *p, int *out)
{
    int i;
    int val = 0;

    for (i = 0; i < 4; i++) {
        if (p[i] < '0' || p[i] > '9') {
            return -1;
        }
        val = val * 10 + (p[i] - '0');
    }
    *out = val;
    return 0;
}

static bool parse_datetime(const char *datetime, time_t *out_epoch)
{
    int year = 0;
    int mon = 0;
    int day = 0;
    int hour = 0;
    int min = 0;
    int sec = 0;
    time_t epoch;

    if (datetime == NULL || out_epoch == NULL) {
        return false;
    }

    if (parse_four_digits(datetime, &year) != 0 ||
        datetime[4] != '-' ||
        parse_two_digits(datetime + 5, &mon) != 0 ||
        datetime[7] != '-' ||
        parse_two_digits(datetime + 8, &day) != 0 ||
        datetime[10] != ' ' ||
        parse_two_digits(datetime + 11, &hour) != 0 ||
        datetime[13] != ':' ||
        parse_two_digits(datetime + 14, &min) != 0 ||
        datetime[16] != ':' ||
        parse_two_digits(datetime + 17, &sec) != 0) {
        return false;
    }

    epoch = epoch_from_parts(year, mon, day, hour, min, sec);
    if (epoch < 0) {
        return false;
    }

    *out_epoch = epoch;
    return true;
}

static time_t video_osd_now_sec(void)
{
    struct timeval tv = {0};

    if (bk_rtc_gettimeofday(&tv, NULL) != BK_OK) {
        return (time_t)0;
    }
    return (time_t)tv.tv_sec + s_ctx.offset_sec;
}

static void video_osd_format_now(char *buf, size_t len)
{
    time_t sec = video_osd_now_sec();
    struct tm tm_info = {0};

    if (localtime_r(&sec, &tm_info) == NULL) {
        os_strncpy(buf, VIDEO_OSD_DEFAULT_TIME, len - 1);
        buf[len - 1] = '\0';
        return;
    }

    os_snprintf(buf, len, "%04d-%02d-%02d %02d:%02d:%02d",
                tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec);
}

static void h264_buffer_free(void *buffer, void *free_arg)
{
    (void)free_arg;
    bk_frame_buffer_free(buffer);
}

static avdk_err_t h264_clear_slot(bk_h264_encode_ctlr_handle_t enc, uint32_t slot)
{
    bk_h264_encode_osd_t cfg = {0};

    if (enc == NULL) {
        return AVDK_ERR_GENERIC;
    }

    cfg.index = slot;
    cfg.buffer = NULL;
    cfg.format = BK_H264_ENCODE_OVERLAY_FORMAT_ARGB8888;
    return bk_h264_encode_set_osd(enc, &cfg);
}

static avdk_err_t h264_submit_argb(bk_h264_encode_ctlr_handle_t enc, uint32_t slot,
                                   uint8_t *buf, uint32_t x, uint32_t y,
                                   uint32_t w, uint32_t h)
{
    bk_h264_encode_osd_t cfg = {0};
    avdk_err_t ret;

    if (enc == NULL || buf == NULL || w == 0U || h == 0U) {
        return AVDK_ERR_INVAL;
    }

    flush_dcache(buf, (long)(w * h * 4U));

    cfg.index = slot;
    cfg.buffer = buf;
    cfg.format = BK_H264_ENCODE_OVERLAY_FORMAT_ARGB8888;
    cfg.alpha = 255U;
    cfg.x = x & ~1U;
    cfg.y = y & ~1U;
    cfg.width = w;
    cfg.height = h;
    cfg.buffer_free = h264_buffer_free;
    cfg.free_arg = NULL;

    ret = bk_h264_encode_set_osd(enc, &cfg);
    if (ret != AVDK_ERR_OK) {
        h264_buffer_free(buf, NULL);
        LOGE("set osd slot %u failed %d\r\n", slot, ret);
    }
    return ret;
}

static avdk_err_t raster_time_string(const char *text, uint8_t **out_buf,
                                     uint32_t *out_w, uint32_t *out_h)
{
    uint16_t sw = 0;
    uint16_t sh = 0;
    uint32_t *sprite;

    if (text == NULL || out_buf == NULL || out_w == NULL || out_h == NULL ||
        video_osd_font_calibri80 == NULL) {
        return AVDK_ERR_INVAL;
    }

    osd_emwin_font_text_extent(video_osd_font_calibri80, text, &sw, &sh);
    if (sw == 0U || sh == 0U) {
        return AVDK_ERR_INVAL;
    }

    sprite = (uint32_t *)bk_frame_buffer_malloc(MEM_SLAB_HEAP_UNCODED, (uint32_t)sw * sh * 4U);
    if (sprite == NULL) {
        return AVDK_ERR_NOMEM;
    }
    os_memset(sprite, 0, (uint32_t)sw * sh * 4U);
    (void)osd_emwin_font_blit(sprite, sw, sh, video_osd_font_calibri80, text,
                              0, 0, VIDEO_OSD_FONT_COLOR, NULL);

    *out_buf = (uint8_t *)sprite;
    *out_w = sw;
    *out_h = sh;
    return AVDK_ERR_OK;
}

static avdk_err_t video_osd_render_time(const char *text)
{
    uint8_t *buf = NULL;
    uint32_t w = 0;
    uint32_t h = 0;
    avdk_err_t ret;

    if (s_ctx.enc == NULL) {
        return AVDK_ERR_GENERIC;
    }

    ret = raster_time_string(text, &buf, &w, &h);
    if (ret != AVDK_ERR_OK) {
        return ret;
    }

    return h264_submit_argb(s_ctx.enc, VIDEO_OSD_SLOT, buf,
                            VIDEO_OSD_X, VIDEO_OSD_Y, w, h);
}

static void video_osd_worker(beken_thread_arg_t arg)
{
    video_osd_ctx_t *ctx = (video_osd_ctx_t *)arg;

    for (;;) {
        char now[VIDEO_OSD_TIME_LEN];

        if (rtos_get_semaphore(&ctx->tick_sem, BEKEN_WAIT_FOREVER) != BK_OK) {
            continue;
        }

        rtos_lock_mutex(&ctx->lock);
        if (!ctx->active) {
            rtos_unlock_mutex(&ctx->lock);
            continue;
        }

        video_osd_format_now(now, sizeof(now));
        if (os_strcmp(now, ctx->last_str) != 0) {
            if (video_osd_render_time(now) == AVDK_ERR_OK) {
                os_strncpy(ctx->last_str, now, sizeof(ctx->last_str) - 1);
                ctx->last_str[sizeof(ctx->last_str) - 1] = '\0';
            }
        }
        rtos_unlock_mutex(&ctx->lock);
    }
}

static void video_osd_timer_cb(void *arg)
{
    video_osd_ctx_t *ctx = (video_osd_ctx_t *)arg;
    (void)rtos_set_semaphore(&ctx->tick_sem);
}

static avdk_err_t video_osd_runtime_init(video_osd_ctx_t *ctx)
{
    if (!ctx->lock_inited) {
        if (rtos_init_mutex(&ctx->lock) != BK_OK) {
            return AVDK_ERR_GENERIC;
        }
        ctx->lock_inited = true;
    }

    if (!ctx->sem_inited) {
        if (rtos_init_semaphore(&ctx->tick_sem, 1) != BK_OK) {
            return AVDK_ERR_GENERIC;
        }
        ctx->sem_inited = true;
    }

    if (!ctx->worker_inited) {
        if (rtos_create_thread(&ctx->worker, BEKEN_DEFAULT_WORKER_PRIORITY,
                               "video_osd", video_osd_worker, 2048, ctx) != BK_OK) {
            return AVDK_ERR_GENERIC;
        }
        ctx->worker_inited = true;
    }

    if (!ctx->timer_inited) {
        if (rtos_init_timer(&ctx->timer, VIDEO_OSD_REFRESH_MS,
                            video_osd_timer_cb, ctx) != BK_OK) {
            return AVDK_ERR_GENERIC;
        }
        ctx->timer_inited = true;
    }

    return AVDK_ERR_OK;
}

avdk_err_t video_osd_time_on(bk_h264_encode_ctlr_handle_t enc, const char *datetime)
{
    time_t target_epoch;
    struct timeval tv = {0};
    const char *seed = datetime;

    if (enc == NULL) {
        return AVDK_ERR_INVAL;
    }

    if (video_osd_runtime_init(&s_ctx) != AVDK_ERR_OK) {
        LOGE("init video_osd runtime failed\r\n");
        return AVDK_ERR_GENERIC;
    }

    if (seed == NULL || seed[0] == '\0') {
        seed = VIDEO_OSD_DEFAULT_TIME;
    }
    if (!parse_datetime(seed, &target_epoch)) {
        LOGE("invalid datetime: %s\r\n", seed);
        return AVDK_ERR_INVAL;
    }

    if (bk_rtc_gettimeofday(&tv, NULL) != BK_OK) {
        LOGE("bk_rtc_gettimeofday failed\r\n");
        return AVDK_ERR_GENERIC;
    }

    rtos_lock_mutex(&s_ctx.lock);
    if (s_ctx.active) {
        rtos_stop_timer(&s_ctx.timer);
        if (s_ctx.enc != NULL) {
            (void)h264_clear_slot(s_ctx.enc, VIDEO_OSD_SLOT);
        }
    }

    s_ctx.enc = enc;
    s_ctx.offset_sec = target_epoch - (time_t)tv.tv_sec;
    s_ctx.last_str[0] = '\0';
    s_ctx.active = true;

    if (rtos_start_timer(&s_ctx.timer) != BK_OK) {
        s_ctx.active = false;
        s_ctx.enc = NULL;
        rtos_unlock_mutex(&s_ctx.lock);
        LOGE("start video_osd timer failed\r\n");
        return AVDK_ERR_GENERIC;
    }
    rtos_unlock_mutex(&s_ctx.lock);

    (void)rtos_set_semaphore(&s_ctx.tick_sem);
    LOGI("time osd on, seed=%s\r\n", seed);
    return AVDK_ERR_OK;
}

void video_osd_time_off(void)
{
    if (!s_ctx.lock_inited) {
        return;
    }

    rtos_lock_mutex(&s_ctx.lock);
    if (s_ctx.active) {
        if (s_ctx.timer_inited) {
            rtos_stop_timer(&s_ctx.timer);
        }
        if (s_ctx.enc != NULL) {
            (void)h264_clear_slot(s_ctx.enc, VIDEO_OSD_SLOT);
        }
        s_ctx.active = false;
        s_ctx.enc = NULL;
        s_ctx.last_str[0] = '\0';
    }
    rtos_unlock_mutex(&s_ctx.lock);
    LOGI("time osd off\r\n");
}
