#pragma once

#include <stdbool.h>
#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Initialize both IR-CUT coil GPIOs in the de-energized state. */
bk_err_t pt_ircut_init(void);

/**
 * Pulse the IR-CUT actuator into the requested optical path.
 * night=true removes the IR filter; night=false inserts it.
 */
bk_err_t pt_ircut_set(bool night);

#ifdef __cplusplus
}
#endif
