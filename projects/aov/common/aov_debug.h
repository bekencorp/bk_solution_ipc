#pragma once

#ifndef CONFIG_AOV_DEBUG_IO_ENABLE
#define CONFIG_AOV_DEBUG_IO_ENABLE 0
#endif

#if CONFIG_AOV_DEBUG_IO_ENABLE
#include <soc/soc.h>
#define AOV_DEBUG_IO_UP(id)       GPIO_UP(id)
#define AOV_DEBUG_IO_DOWN(id)     GPIO_DOWN(id)
#define AOV_DEBUG_IO_UP_DOWN(id)  GPIO_UP_DOWN(id)
#else
#define AOV_DEBUG_IO_UP(id)       ((void)0)
#define AOV_DEBUG_IO_DOWN(id)     ((void)0)
#define AOV_DEBUG_IO_UP_DOWN(id)  ((void)0)
#endif
