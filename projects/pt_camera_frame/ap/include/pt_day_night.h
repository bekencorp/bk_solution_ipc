#pragma once

#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Start automatic day-color / night-infrared switching. */
bk_err_t pt_day_night_start(void);

/** Stop detection and restore the day-color hardware state. */
void pt_day_night_stop(void);

#ifdef __cplusplus
}
#endif
