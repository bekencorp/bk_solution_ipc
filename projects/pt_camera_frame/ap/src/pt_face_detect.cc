#include "pt_face_detect.h"

extern "C" {
#include <common/bk_include.h>
#include <components/bk_frame_buffer.h>
#include <components/bk_isp_camera.h>
#include <components/log.h>
#include <driver/aon_rtc.h>
#include <os/mem.h>
#include <os/os.h>

#include "app_camera.h"
#if CONFIG_ASR_SERVICE_WITH_MIC
int doorbell_asr_turn_off(void);
#endif
}

#include "box.h"
#include "pt_motor.h"
#include "tflm_face_detect.h"

#include <new>

#define PT_FACE_INPUT_WIDTH             (320u)
#define PT_FACE_INPUT_HEIGHT            (320u)
#define PT_FACE_SP_HEIGHT               (180u)
#define PT_FACE_READ_TIMEOUT_MS         (1000u)
#define PT_FACE_LOOP_DELAY_MS           (100u)
#define PT_FACE_START_DELAY_MS          (1500u)
#define PT_FACE_MOTOR_CHECK_TIMEOUT_MS  (30000u)
#define PT_FACE_MOTOR_CENTER_PAN        (2025)
#define PT_FACE_MOTOR_CENTER_TILT       (572)
#define PT_FACE_FRAME_BYTES             (PT_FACE_INPUT_WIDTH * PT_FACE_INPUT_HEIGHT * 4u)
#define PT_FACE_SP_FRAME_BYTES          (PT_FACE_INPUT_WIDTH * PT_FACE_SP_HEIGHT * 4u)
#define PT_FACE_TASK_STACK              (1024 * 16)

#define PT_FACE_TRACK_CENTER_X          (PT_FACE_INPUT_WIDTH / 2.0f)
#define PT_FACE_TRACK_CENTER_Y          (PT_FACE_SP_HEIGHT / 2.0f)
#define PT_FACE_TRACK_STOP_X            (40.0f)
#define PT_FACE_TRACK_STOP_Y            (25.0f)
#define PT_FACE_TRACK_START_X           (50.0f)
#define PT_FACE_TRACK_START_Y           (40.0f)
#define PT_FACE_TRACK_JUMP_X            (60.0f)
#define PT_FACE_TRACK_JUMP_Y            (45.0f)
#define PT_FACE_TRACK_SCORE_MIN         (0.60f)
#define PT_FACE_TRACK_EMA_OLD           (0.75f)
#define PT_FACE_TRACK_EMA_NEW           (0.25f)
#define PT_FACE_TRACK_CMD_INTERVAL_MS   (200u)
#define PT_FACE_TRACK_DIR_NONE          (-1)

typedef struct {
    FaceDetectModel *model;
    beken_thread_t thread;
    volatile bool stop_request;
    volatile bool running;
    bool model_inited;
    uint8_t *frame;
    uint32_t frame_id;
    pt_face_detect_result_t result;
} pt_face_detect_ctx_t;

static pt_face_detect_ctx_t *s_face_ctx = nullptr;

typedef struct {
    bool has_smooth;
    float smooth_cx;
    float smooth_cy;
    int current_pan_dir;
    int current_tilt_dir;
    int current_dir;
    int pending_dir;
    uint8_t pending_count;
    uint32_t last_cmd_ms;
} pt_face_track_ctx_t;

static pt_face_track_ctx_t s_track_ctx = {
    false, 0.0f, 0.0f, 0, 0, PT_FACE_TRACK_DIR_NONE, PT_FACE_TRACK_DIR_NONE, 0, 0
};

static float pt_face_absf(float value)
{
    return value < 0.0f ? -value : value;
}

static int pt_face_track_make_dir(int pan_dir, int tilt_dir)
{
    if (pan_dir > 0 && tilt_dir < 0) return PT_MOTOR_DIR_RIGHT_UP;
    if (pan_dir > 0 && tilt_dir > 0) return PT_MOTOR_DIR_RIGHT_DOWN;
    if (pan_dir < 0 && tilt_dir < 0) return PT_MOTOR_DIR_LEFT_UP;
    if (pan_dir < 0 && tilt_dir > 0) return PT_MOTOR_DIR_LEFT_DOWN;
    if (pan_dir > 0) return PT_MOTOR_DIR_RIGHT;
    if (pan_dir < 0) return PT_MOTOR_DIR_LEFT;
    if (tilt_dir < 0) return PT_MOTOR_DIR_UP;
    if (tilt_dir > 0) return PT_MOTOR_DIR_DOWN;
    return PT_FACE_TRACK_DIR_NONE;
}

static int pt_face_track_dir_from_error(float err_x, float err_y)
{
    int pan_dir = 0;
    int tilt_dir = 0;

    if (err_x > PT_FACE_TRACK_START_X) {
        pan_dir = 1;
    } else if (err_x < -PT_FACE_TRACK_START_X) {
        pan_dir = -1;
    }

    if (err_y > PT_FACE_TRACK_START_Y) {
        tilt_dir = 1;
    } else if (err_y < -PT_FACE_TRACK_START_Y) {
        tilt_dir = -1;
    }

    return pt_face_track_make_dir(pan_dir, tilt_dir);
}

static void pt_face_track_reset(void)
{
    s_track_ctx.has_smooth = false;
    s_track_ctx.current_pan_dir = 0;
    s_track_ctx.current_tilt_dir = 0;
    s_track_ctx.pending_dir = PT_FACE_TRACK_DIR_NONE;
    s_track_ctx.pending_count = 0;
    s_track_ctx.current_dir = PT_FACE_TRACK_DIR_NONE;
}

static void pt_face_track_stop(void)
{
    if (s_track_ctx.current_dir != PT_FACE_TRACK_DIR_NONE) {
        (void)pt_motor_stop();
    }
    pt_face_track_reset();
}

static void pt_face_track_issue_dir(int dir)
{
    uint32_t now = rtos_get_time();

    if (dir == PT_FACE_TRACK_DIR_NONE) {
        pt_face_track_stop();
        return;
    }

    if (dir == s_track_ctx.current_dir &&
        (now - s_track_ctx.last_cmd_ms) < PT_FACE_TRACK_CMD_INTERVAL_MS) {
        return;
    }

    if (pt_motor_move((pt_motor_dir_t)dir) == BK_OK) {
        s_track_ctx.current_dir = dir;
        s_track_ctx.last_cmd_ms = now;
    }
}

static void pt_face_track_issue_axes(int pan_dir, int tilt_dir)
{
    int dir = pt_face_track_make_dir(pan_dir, tilt_dir);

    if (dir == PT_FACE_TRACK_DIR_NONE) {
        pt_face_track_stop();
        return;
    }

    pt_face_track_issue_dir(dir);
    if (s_track_ctx.current_dir == dir) {
        s_track_ctx.current_pan_dir = pan_dir;
        s_track_ctx.current_tilt_dir = tilt_dir;
    }
}

static void pt_face_track_update(const pt_face_detect_result_t *result)
{
    if (result == NULL ||
        result->detected == 0 ||
        result->count == 0 ||
        result->boxes[0].score < PT_FACE_TRACK_SCORE_MIN) {
        pt_face_track_stop();
        return;
    }

    const pt_face_detect_box_t *box = &result->boxes[0];
    float raw_cx = box->x + box->w * 0.5f;
    float raw_cy = box->y + box->h * 0.5f;

    if (!s_track_ctx.has_smooth) {
        s_track_ctx.smooth_cx = raw_cx;
        s_track_ctx.smooth_cy = raw_cy;
        s_track_ctx.has_smooth = true;
    }

    bool jump = pt_face_absf(raw_cx - s_track_ctx.smooth_cx) > PT_FACE_TRACK_JUMP_X ||
                pt_face_absf(raw_cy - s_track_ctx.smooth_cy) > PT_FACE_TRACK_JUMP_Y;
    if (jump) {
        int pending_dir = pt_face_track_dir_from_error(raw_cx - PT_FACE_TRACK_CENTER_X,
                                                       raw_cy - PT_FACE_TRACK_CENTER_Y);
        if (pending_dir == s_track_ctx.pending_dir) {
            s_track_ctx.pending_count++;
        } else {
            s_track_ctx.pending_dir = pending_dir;
            s_track_ctx.pending_count = 1;
        }

        if (s_track_ctx.pending_count < 2) {
            return;
        }
    } else {
        s_track_ctx.pending_dir = PT_FACE_TRACK_DIR_NONE;
        s_track_ctx.pending_count = 0;
    }

    s_track_ctx.smooth_cx = s_track_ctx.smooth_cx * PT_FACE_TRACK_EMA_OLD +
                            raw_cx * PT_FACE_TRACK_EMA_NEW;
    s_track_ctx.smooth_cy = s_track_ctx.smooth_cy * PT_FACE_TRACK_EMA_OLD +
                            raw_cy * PT_FACE_TRACK_EMA_NEW;

    float raw_err_x = raw_cx - PT_FACE_TRACK_CENTER_X;
    float raw_err_y = raw_cy - PT_FACE_TRACK_CENTER_Y;
    if (pt_face_absf(raw_err_x) <= PT_FACE_TRACK_STOP_X &&
        pt_face_absf(raw_err_y) <= PT_FACE_TRACK_STOP_Y) {
        pt_face_track_stop();
        return;
    }

    float smooth_err_x = s_track_ctx.smooth_cx - PT_FACE_TRACK_CENTER_X;
    float smooth_err_y = s_track_ctx.smooth_cy - PT_FACE_TRACK_CENTER_Y;
    int pan_dir = 0;
    int tilt_dir = 0;

    if (pt_face_absf(raw_err_x) <= PT_FACE_TRACK_STOP_X) {
        pan_dir = 0;
    } else if (smooth_err_x > PT_FACE_TRACK_START_X) {
        pan_dir = 1;
    } else if (smooth_err_x < -PT_FACE_TRACK_START_X) {
        pan_dir = -1;
    } else {
        pan_dir = s_track_ctx.current_pan_dir;
    }

    if (pt_face_absf(raw_err_y) <= PT_FACE_TRACK_STOP_Y) {
        tilt_dir = 0;
    } else if (smooth_err_y > PT_FACE_TRACK_START_Y) {
        tilt_dir = 1;
    } else if (smooth_err_y < -PT_FACE_TRACK_START_Y) {
        tilt_dir = -1;
    } else {
        tilt_dir = s_track_ctx.current_tilt_dir;
    }

    pt_face_track_issue_axes(pan_dir, tilt_dir);
}


static void pt_face_detect_copy_result(pt_face_detect_result_t *result)
{
    if (result == nullptr) {
        return;
    }

    if (s_face_ctx == nullptr) {
        os_memset(result, 0, sizeof(*result));
        return;
    }

    uint32_t int_level = rtos_disable_int();
    os_memcpy(result, &s_face_ctx->result, sizeof(*result));
    rtos_enable_int(int_level);
}

static void pt_face_detect_box_cb(Box *boxes, int count)
{
    if (s_face_ctx == nullptr) {
        return;
    }

    pt_face_detect_result_t next = {};
    int copy_count = 0;

    next.frame_id = s_face_ctx->frame_id;
    next.timestamp_ms = rtos_get_time();
    next.width = PT_FACE_INPUT_WIDTH;
    next.height = PT_FACE_INPUT_HEIGHT;

    if (boxes != nullptr && count > 0) {
        copy_count = count > PT_FACE_DETECT_MAX_BOXES ?
            PT_FACE_DETECT_MAX_BOXES : count;
        for (int i = 0; i < copy_count; i++) {
            next.boxes[i].x = boxes[i].x;
            next.boxes[i].y = boxes[i].y;
            next.boxes[i].w = boxes[i].w;
            next.boxes[i].h = boxes[i].h;
            next.boxes[i].score = boxes[i].score;
        }
    }

    next.count = (uint8_t)copy_count;
    next.detected = copy_count > 0 ? 1 : 0;

    uint32_t int_level = rtos_disable_int();
    os_memcpy(&s_face_ctx->result, &next, sizeof(next));
    rtos_enable_int(int_level);
}

static bk_err_t pt_face_detect_model_init(void)
{
    if (s_face_ctx == nullptr || s_face_ctx->model == nullptr) {
        return BK_ERR_STATE;
    }
    if (s_face_ctx->model_inited) {
        return BK_OK;
    }

    int ret = s_face_ctx->model->init();
    if (ret != BK_OK) {
        return BK_FAIL;
    }

    s_face_ctx->model_inited = true;
    return BK_OK;
}

static bk_err_t pt_face_detect_open_camera(void)
{
    camera_board_config_t *config = app_camera_board_config_get();
    if (config == nullptr) {
        return BK_ERR_STATE;
    }

    config->isp.sp_enable = true;
    config->isp.sp_flexa = false;
    config->isp.sp_width = PT_FACE_INPUT_WIDTH;
    config->isp.sp_height = PT_FACE_SP_HEIGHT;
    config->isp.sp_format = BK_PIXEL_FORMAT_RGB888;

    if (app_isp_handle_get() == nullptr) {
        int ret = app_isp_mipi_camera_turn_on(config);
        if (ret != BK_OK) {
            return ret;
        }
    }

    bk_isp_camera_ctlr_handle_t handle = app_isp_camera_ctlr_handle_get();
    if (handle != nullptr &&
        bk_isp_camera_channel_state_get(handle, APP_ISP_SP_CHN_ID) ==
            ISP_CHANNEL_STATE_TURN_ON) {
        return BK_OK;
    }

    int ret = app_isp_camera_sp_channel_turn_on(config);
    return ret;
}

static void pt_face_detect_task(void *arg)
{
    (void)arg;

    s_face_ctx->running = true;

    rtos_delay_milliseconds(PT_FACE_START_DELAY_MS);

    if (pt_face_detect_open_camera() != BK_OK) {
        goto exit;
    }

    if (pt_motor_init() != BK_OK) {
        goto exit;
    }
    if (pt_motor_self_check() != BK_OK) {
        goto exit;
    }
    if (pt_motor_wait_idle(PT_FACE_MOTOR_CHECK_TIMEOUT_MS) != BK_OK) {
        goto exit;
    }
    if (pt_motor_go_pos(PT_FACE_MOTOR_CENTER_PAN, PT_FACE_MOTOR_CENTER_TILT) != BK_OK) {
        goto exit;
    }
    if (pt_motor_wait_idle(PT_FACE_MOTOR_CHECK_TIMEOUT_MS) != BK_OK) {
        goto exit;
    }

#if CONFIG_ASR_SERVICE_WITH_MIC
    (void)doorbell_asr_turn_off();
#endif

    if (pt_face_detect_model_init() != BK_OK) {
        goto exit;
    }

    if (s_face_ctx->frame == nullptr) {
        s_face_ctx->frame = (uint8_t *)bk_frame_buffer_malloc(
            MEM_SLAB_HEAP_UNCODED, PT_FACE_FRAME_BYTES);
        if (s_face_ctx->frame == nullptr) {
            goto exit;
        }
        os_memset(s_face_ctx->frame, 0, PT_FACE_FRAME_BYTES);
    }

    while (!s_face_ctx->stop_request) {
        int ret = app_isp_camera_channel_read(APP_ISP_SP_CHN_ID,
                                             s_face_ctx->frame,
                                             PT_FACE_SP_FRAME_BYTES,
                                             PT_FACE_READ_TIMEOUT_MS);
        if (s_face_ctx->stop_request) {
            break;
        }
        if (ret != BK_OK) {
            rtos_delay_milliseconds(PT_FACE_LOOP_DELAY_MS);
            continue;
        }

        s_face_ctx->frame_id++;
        unsigned long long start_us = bk_aon_rtc_get_us();
        int count = s_face_ctx->model->run(s_face_ctx->frame,
                                           PT_FACE_SP_FRAME_BYTES,
                                           BK_PIXEL_FORMAT_BGRA8888);
        (void)count;
        (void)start_us;

        pt_face_detect_result_t track_result = {0};
        pt_face_detect_copy_result(&track_result);
        pt_face_track_update(&track_result);

        rtos_delay_milliseconds(PT_FACE_LOOP_DELAY_MS);
    }

exit:
    s_face_ctx->running = false;
    s_face_ctx->thread = nullptr;
    rtos_delete_thread(nullptr);
}

bk_err_t pt_face_detect_init(void)
{
    if (s_face_ctx != nullptr && s_face_ctx->model_inited) {
        return BK_OK;
    }

    s_face_ctx = (pt_face_detect_ctx_t *)os_malloc(sizeof(*s_face_ctx));
    if (s_face_ctx == nullptr) {
        return BK_ERR_NO_MEM;
    }
    os_memset(s_face_ctx, 0, sizeof(*s_face_ctx));

    void *model_mem = os_malloc(sizeof(FaceDetectModel));
    if (model_mem == nullptr) {
        os_free(s_face_ctx);
        s_face_ctx = nullptr;
        return BK_ERR_NO_MEM;
    }

    s_face_ctx->model = new (model_mem) FaceDetectModel();
    s_face_ctx->model->LogEnable(true);
    s_face_ctx->model->setBoxDetectionCallback(pt_face_detect_box_cb);
    return BK_OK;
}

bk_err_t pt_face_detect_start(void)
{
    if (s_face_ctx == nullptr || s_face_ctx->model == nullptr) {
        bk_err_t init_ret = pt_face_detect_init();
        if (init_ret != BK_OK) {
            return init_ret;
        }
    }

    if (s_face_ctx->running || s_face_ctx->thread != nullptr) {
        return BK_OK;
    }

    s_face_ctx->stop_request = false;
    bk_err_t ret = rtos_create_thread(&s_face_ctx->thread,
                                      BEKEN_DEFAULT_WORKER_PRIORITY,
                                      "pt_face",
                                      (beken_thread_function_t)pt_face_detect_task,
                                      PT_FACE_TASK_STACK,
                                      nullptr);
    if (ret != BK_OK) {
        s_face_ctx->thread = nullptr;
        return ret;
    }

    return BK_OK;
}

bk_err_t pt_face_detect_stop(void)
{
    if (s_face_ctx == nullptr) {
        return BK_OK;
    }

    s_face_ctx->stop_request = true;
    for (int i = 0; i < 20 && s_face_ctx->running; i++) {
        rtos_delay_milliseconds(50);
    }

    return BK_OK;
}

bk_err_t pt_face_detect_get_result(pt_face_detect_result_t *result)
{
    if (result == nullptr) {
        return BK_ERR_PARAM;
    }

    pt_face_detect_copy_result(result);
    return BK_OK;
}
