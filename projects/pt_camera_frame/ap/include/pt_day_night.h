#pragma once

#include <stdbool.h>

#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Day-color / night-infrared switching mode. */
typedef enum {
	PT_DAY_NIGHT_MODE_AUTO = 0,     /**< Auto: day full-color, night infrared. */
	PT_DAY_NIGHT_MODE_MANUAL_DAY,   /**< Manual: force full-color (day) mode. */
	PT_DAY_NIGHT_MODE_MANUAL_NIGHT, /**< Manual: force infrared (night) mode. */
} pt_day_night_mode_t;

/** Start automatic day-color / night-infrared switching. */
bk_err_t pt_day_night_start(void);

/** Stop detection and restore the day-color hardware state. */
void pt_day_night_stop(void);

/**
 * @brief Select the switching mode.
 *
 * PT_DAY_NIGHT_MODE_AUTO keeps the luminance-driven strategy (day full-color,
 * night infrared). The MANUAL_DAY / MANUAL_NIGHT modes pin the optics to
 * full-color or infrared respectively, bypassing the auto detection (and the
 * IR-CUT chatter lockout so the user can toggle freely). Takes effect promptly
 * when the module is running.
 */
bk_err_t pt_day_night_mode_set(pt_day_night_mode_t mode);

/** @brief Return the currently selected switching mode. */
pt_day_night_mode_t pt_day_night_mode_get(void);

/** @brief Return true if the optics are currently in night-infrared state. */
bool pt_day_night_is_night(void);

#ifdef __cplusplus
}
#endif
