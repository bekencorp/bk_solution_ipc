#pragma once

#include <stdint.h>
#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define PT_MOTOR_MAX_PAN_STEP   4050
#define PT_MOTOR_MAX_TILT_STEP  1100

typedef enum {
    PT_MOTOR_DIR_UP = 0,
    PT_MOTOR_DIR_RIGHT_UP,
    PT_MOTOR_DIR_RIGHT,
    PT_MOTOR_DIR_RIGHT_DOWN,
    PT_MOTOR_DIR_DOWN,
    PT_MOTOR_DIR_LEFT_DOWN,
    PT_MOTOR_DIR_LEFT,
    PT_MOTOR_DIR_LEFT_UP,
} pt_motor_dir_t;

typedef struct {
    int pan;
    int tilt;
    int pan_goal;
    int tilt_goal;
    uint8_t pan_running;
    uint8_t tilt_running;
} pt_motor_pos_t;

bk_err_t pt_motor_init(void);
bk_err_t pt_motor_deinit(void);
bk_err_t pt_motor_move(pt_motor_dir_t dir);
bk_err_t pt_motor_go_pos(int pan, int tilt);
bk_err_t pt_motor_self_check(void);
bk_err_t pt_motor_stop(void);
bk_err_t pt_motor_get_pos(pt_motor_pos_t *pos);
bk_err_t pt_motor_wait_idle(uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif
