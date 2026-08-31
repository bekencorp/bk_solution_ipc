#pragma once

#include <stdbool.h>
#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize the infrared fill-light GPIO in the OFF state. */
bk_err_t pt_ir_led_init(void);

/** Enable or disable the infrared fill light. */
bk_err_t pt_ir_led_set(bool enable);

#ifdef __cplusplus
}
#endif
