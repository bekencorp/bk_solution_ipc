#include "pt_motor.h"

#include <stdbool.h>
#include <string.h>

#include <common/bk_include.h>
#include <components/log.h>
#include <driver/gpio.h>
#include <driver/gpio_types.h>
#include <os/os.h>

#define TAG "pt_motor"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define PT_MOTOR_TICK_MS       2
#define PT_MOTOR_PAN_DIV       3
#define PT_MOTOR_TILT_DIV      3

/* Same pin map as the Tuya PTZ reference:
 * tilt = IN1..IN4 / low nibble, pan = IN5..IN8 / high nibble.
 * pt_camera usr_gpio_cfg.h leaves GPIO32..39 as plain GPIO.
 */
static const gpio_id_t s_tilt_pins[4] = {GPIO_36, GPIO_37, GPIO_38, GPIO_39};
static const gpio_id_t s_pan_pins[4] = {GPIO_32, GPIO_33, GPIO_34, GPIO_35};
static const uint8_t s_half_steps[8] = {0x01, 0x03, 0x02, 0x06, 0x04, 0x0C, 0x08, 0x09};

typedef struct {
    beken_thread_t thread;
    volatile bool thread_running;
    volatile bool stop_thread;
    volatile bool inited;
    volatile uint8_t pan_running;
    volatile uint8_t tilt_running;
    volatile uint8_t pan_dir;
    volatile uint8_t tilt_dir;
    int pan;
    int tilt;
    int pan_goal;
    int tilt_goal;
    uint8_t pan_step_idx;
    uint8_t tilt_step_idx;
} pt_motor_ctx_t;

static pt_motor_ctx_t s_motor;

static int pt_motor_clamp(int value, int min, int max)
{
    if (value < min) {
        return min;
    }
    if (value > max) {
        return max;
    }
    return value;
}

static void pt_motor_gpio_write_nibble(const gpio_id_t pins[4], uint8_t value)
{
    for (int i = 0; i < 4; i++) {
        if (value & (1u << i)) {
            bk_gpio_set_output_high(pins[i]);
        } else {
            bk_gpio_set_output_low(pins[i]);
        }
    }
}

static void pt_motor_output(uint8_t pan_val, uint8_t tilt_val)
{
    pt_motor_gpio_write_nibble(s_tilt_pins, tilt_val);
    pt_motor_gpio_write_nibble(s_pan_pins, pan_val);
}

static void pt_motor_shutdown_outputs(void)
{
    pt_motor_output(0, 0);
}

static void pt_motor_step_index(uint8_t *idx, uint8_t forward)
{
    if (forward) {
        *idx = (uint8_t)((*idx + 1) & 0x07);
    } else {
        *idx = (uint8_t)((*idx + 7) & 0x07);
    }
}

static void pt_motor_worker(void *arg)
{
    (void)arg;
    uint32_t pan_tick = 0;
    uint32_t tilt_tick = 0;

    s_motor.thread_running = true;
    while (!s_motor.stop_thread) {
        uint8_t pan_val = 0;
        uint8_t tilt_val = 0;

        if (s_motor.pan_running && s_motor.pan != s_motor.pan_goal) {
            if (++pan_tick >= PT_MOTOR_PAN_DIV) {
                pan_tick = 0;
                if (s_motor.pan_dir) {
                    s_motor.pan++;
                } else {
                    s_motor.pan--;
                }
                s_motor.pan = pt_motor_clamp(s_motor.pan, 0, PT_MOTOR_MAX_PAN_STEP);
                pt_motor_step_index(&s_motor.pan_step_idx, s_motor.pan_dir);
            }
            pan_val = s_half_steps[s_motor.pan_step_idx];
        } else {
            s_motor.pan_running = 0;
        }

        if (s_motor.tilt_running && s_motor.tilt != s_motor.tilt_goal) {
            if (++tilt_tick >= PT_MOTOR_TILT_DIV) {
                tilt_tick = 0;
                if (s_motor.tilt_dir) {
                    s_motor.tilt++;
                } else {
                    s_motor.tilt--;
                }
                s_motor.tilt = pt_motor_clamp(s_motor.tilt, 0, PT_MOTOR_MAX_TILT_STEP);
                pt_motor_step_index(&s_motor.tilt_step_idx, s_motor.tilt_dir);
            }
            tilt_val = s_half_steps[s_motor.tilt_step_idx];
        } else {
            s_motor.tilt_running = 0;
        }

        pt_motor_output(pan_val, tilt_val);
        if (!s_motor.pan_running && !s_motor.tilt_running) {
            pt_motor_shutdown_outputs();
        }
        rtos_delay_milliseconds(PT_MOTOR_TICK_MS);
    }

    pt_motor_shutdown_outputs();
    s_motor.thread_running = false;
    s_motor.thread = NULL;
    rtos_delete_thread(NULL);
}

static bk_err_t pt_motor_gpio_init(void)
{
    for (int i = 0; i < 4; i++) {
        bk_gpio_set_capacity(s_tilt_pins[i], 3);
        BK_RETURN_ON_ERR(bk_gpio_enable_output(s_tilt_pins[i]));
        BK_RETURN_ON_ERR(bk_gpio_set_output_low(s_tilt_pins[i]));
        bk_gpio_set_capacity(s_pan_pins[i], 3);
        BK_RETURN_ON_ERR(bk_gpio_enable_output(s_pan_pins[i]));
        BK_RETURN_ON_ERR(bk_gpio_set_output_low(s_pan_pins[i]));
    }
    return BK_OK;
}

bk_err_t pt_motor_init(void)
{
    if (s_motor.inited) {
        return BK_OK;
    }

    memset(&s_motor, 0, sizeof(s_motor));
    BK_RETURN_ON_ERR(pt_motor_gpio_init());
    s_motor.pan = PT_MOTOR_MAX_PAN_STEP / 2;
    s_motor.tilt = 572;
    s_motor.pan_goal = s_motor.pan;
    s_motor.tilt_goal = s_motor.tilt;

    bk_err_t ret = rtos_create_thread(&s_motor.thread,
                                      BEKEN_DEFAULT_WORKER_PRIORITY,
                                      "pt_motor",
                                      (beken_thread_function_t)pt_motor_worker,
                                      2048,
                                      NULL);
    if (ret != BK_OK) {
        pt_motor_shutdown_outputs();
        return ret;
    }

    s_motor.inited = true;
    LOGI("init ok, pan=%d tilt=%d\r\n", s_motor.pan, s_motor.tilt);
    return BK_OK;
}

bk_err_t pt_motor_deinit(void)
{
    if (!s_motor.inited) {
        return BK_OK;
    }

    s_motor.stop_thread = true;
    for (int i = 0; i < 20 && s_motor.thread_running; i++) {
        rtos_delay_milliseconds(50);
    }
    pt_motor_shutdown_outputs();
    s_motor.inited = false;
    return BK_OK;
}

bk_err_t pt_motor_stop(void)
{
    if (!s_motor.inited) {
        return BK_ERR_STATE;
    }

    s_motor.pan_running = 0;
    s_motor.tilt_running = 0;
    s_motor.pan_goal = s_motor.pan;
    s_motor.tilt_goal = s_motor.tilt;
    pt_motor_shutdown_outputs();
    return BK_OK;
}

bk_err_t pt_motor_go_pos(int pan, int tilt)
{
    if (!s_motor.inited) {
        return BK_ERR_STATE;
    }

    pan = pt_motor_clamp(pan, 0, PT_MOTOR_MAX_PAN_STEP);
    tilt = pt_motor_clamp(tilt, 0, PT_MOTOR_MAX_TILT_STEP);

    s_motor.pan_goal = pan;
    s_motor.tilt_goal = tilt;
    if (s_motor.pan != s_motor.pan_goal) {
        s_motor.pan_dir = s_motor.pan_goal > s_motor.pan;
        s_motor.pan_running = 1;
    }
    if (s_motor.tilt != s_motor.tilt_goal) {
        s_motor.tilt_dir = s_motor.tilt_goal > s_motor.tilt;
        s_motor.tilt_running = 1;
    }

    return BK_OK;
}

bk_err_t pt_motor_move(pt_motor_dir_t dir)
{
    int pan = s_motor.pan;
    int tilt = s_motor.tilt;

    if (!s_motor.inited) {
        return BK_ERR_STATE;
    }

    switch (dir) {
    case PT_MOTOR_DIR_UP:
        tilt = PT_MOTOR_MAX_TILT_STEP;
        break;
    case PT_MOTOR_DIR_RIGHT_UP:
        pan = PT_MOTOR_MAX_PAN_STEP;
        tilt = PT_MOTOR_MAX_TILT_STEP;
        break;
    case PT_MOTOR_DIR_RIGHT:
        pan = PT_MOTOR_MAX_PAN_STEP;
        break;
    case PT_MOTOR_DIR_RIGHT_DOWN:
        pan = PT_MOTOR_MAX_PAN_STEP;
        tilt = 0;
        break;
    case PT_MOTOR_DIR_DOWN:
        tilt = 0;
        break;
    case PT_MOTOR_DIR_LEFT_DOWN:
        pan = 0;
        tilt = 0;
        break;
    case PT_MOTOR_DIR_LEFT:
        pan = 0;
        break;
    case PT_MOTOR_DIR_LEFT_UP:
        pan = 0;
        tilt = PT_MOTOR_MAX_TILT_STEP;
        break;
    default:
        return BK_ERR_PARAM;
    }

    return pt_motor_go_pos(pan, tilt);
}

bk_err_t pt_motor_self_check(void)
{
    if (!s_motor.inited) {
        return BK_ERR_STATE;
    }

    s_motor.pan = PT_MOTOR_MAX_PAN_STEP;
    s_motor.tilt = PT_MOTOR_MAX_TILT_STEP;
    return pt_motor_go_pos(0, 0);
}

bk_err_t pt_motor_get_pos(pt_motor_pos_t *pos)
{
    if (pos == NULL) {
        return BK_ERR_PARAM;
    }
    if (!s_motor.inited) {
        return BK_ERR_STATE;
    }

    pos->pan = s_motor.pan;
    pos->tilt = s_motor.tilt;
    pos->pan_goal = s_motor.pan_goal;
    pos->tilt_goal = s_motor.tilt_goal;
    pos->pan_running = s_motor.pan_running;
    pos->tilt_running = s_motor.tilt_running;
    return BK_OK;
}

bk_err_t pt_motor_wait_idle(uint32_t timeout_ms)
{
    uint32_t waited_ms = 0;

    if (!s_motor.inited) {
        return BK_ERR_STATE;
    }

    while (s_motor.pan_running || s_motor.tilt_running) {
        if (waited_ms >= timeout_ms) {
            return BK_ERR_TIMEOUT;
        }
        rtos_delay_milliseconds(20);
        waited_ms += 20;
    }

    return BK_OK;
}
