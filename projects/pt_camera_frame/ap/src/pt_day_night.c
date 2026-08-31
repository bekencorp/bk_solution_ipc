#include <stdbool.h>
#include <stdint.h>

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>
#include <components/media_types.h>
#include <components/bk_isp_camera.h>
#include <components/bk_camera_sensor.h>
#include <driver/isp.h>

#include "app_camera.h"
#include "pt_ir_led.h"
#include "pt_ircut.h"
#include "pt_day_night.h"

#define TAG "pt-day-night"

#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define PT_DN_DAY_TO_NIGHT_LUMA   30U
#define PT_DN_NIGHT_TO_DAY_LUMA   369U
#define PT_DN_CONFIRM_COUNT        3U
#define PT_DN_FAST_SAMPLE_MS       100U
#define PT_DN_STABLE_SAMPLE_MS     1000U
#define PT_DN_IR_LED_SETTLE_MS     100U
#define PT_DN_THREAD_STACK_SIZE    4096U
#define PT_DN_STOP_WAIT_MS         2500U

#define PT_DN_CHATTER_FLIPS        5U
#define PT_DN_CHATTER_WINDOW_MS    60000U
#define PT_DN_LOCKOUT_MS           180000U

typedef struct {
	uint32_t day_count;
	uint32_t night_count;
	bool current_is_night;
	bool state_stable;
	int8_t applied_mode;
	int8_t applied_image;
	bool lockout;
	uint32_t lockout_start_ms;
	uint32_t toggle_ticks[PT_DN_CHATTER_FLIPS];
} pt_day_night_ctx_t;

static pt_day_night_ctx_t s_dn;
static volatile bool s_running;
static bool s_initialized;
static beken_thread_t s_thread;
static beken_semaphore_t s_exit_sem;

static bk_err_t pt_dn_set_image_mode(bool night)
{
	bk_isp_camera_ctlr_handle_t camera;
	bk_isp_cproc_attr_t cproc = {0};
	avdk_err_t ret;

	if (s_dn.applied_image == (int8_t)night) {
		return BK_OK;
	}

	camera = app_isp_camera_ctlr_handle_get();
	if (camera == NULL) {
		return BK_FAIL;
	}

	if (night) {
		ret = bk_isp_camera_ctlr_ioctl(camera, BK_CAM_IOCTL_GET_CPROC,
					      &cproc);
		if (ret != AVDK_ERR_OK) {
			return BK_FAIL;
		}
		cproc.enable = 1;
		cproc.op_type = 1;
		cproc.manual.saturation = 0;
	} else {
		bk_camera_sensor_handle_t sensor =
			app_isp_camera_sensor_handle_get();
		if (sensor == NULL) {
			return BK_FAIL;
		}
		ret = bk_camera_sensor_ioctl(
			sensor, BK_CAMERA_SENSOR_IOCTL_GET_DEFAULT_CPROC, &cproc);
		if (ret != AVDK_ERR_OK) {
			return BK_FAIL;
		}
	}

	ret = bk_isp_camera_ctlr_ioctl(camera, BK_CAM_IOCTL_SET_CPROC, &cproc);
	if (ret != AVDK_ERR_OK) {
		return BK_FAIL;
	}
	s_dn.applied_image = (int8_t)night;
	LOGI("image mode -> %s\r\n", night ? "night-gray" : "day-color");
	return BK_OK;
}

static void pt_dn_reset_lockout(void)
{
	s_dn.lockout = false;
	s_dn.lockout_start_ms = 0;
	os_memset(s_dn.toggle_ticks, 0, sizeof(s_dn.toggle_ticks));
}

static bool pt_dn_record_toggle(void)
{
	if (s_dn.lockout) {
		return false;
	}

	for (uint32_t i = PT_DN_CHATTER_FLIPS - 1U; i > 0U; i--) {
		s_dn.toggle_ticks[i] = s_dn.toggle_ticks[i - 1U];
	}
	s_dn.toggle_ticks[0] = rtos_get_time();

	uint32_t oldest = s_dn.toggle_ticks[PT_DN_CHATTER_FLIPS - 1U];
	if (oldest != 0U &&
	    (s_dn.toggle_ticks[0] - oldest) < PT_DN_CHATTER_WINDOW_MS) {
		s_dn.lockout = true;
		s_dn.lockout_start_ms = s_dn.toggle_ticks[0];
		LOGW("IR-CUT chatter lockout for %u ms\r\n", PT_DN_LOCKOUT_MS);
	}
	return true;
}

static void pt_dn_service_lockout(void)
{
	if (s_dn.lockout &&
	    (rtos_get_time() - s_dn.lockout_start_ms) >= PT_DN_LOCKOUT_MS) {
		pt_dn_reset_lockout();
		LOGI("IR-CUT chatter lockout released\r\n");
	}
}

static bk_err_t pt_dn_apply_mode(bool night)
{
	bk_err_t ret;

	if (s_dn.applied_mode == (int8_t)night) {
		return pt_dn_set_image_mode(night);
	}
	if (s_dn.applied_mode >= 0 && !pt_dn_record_toggle()) {
		return BK_ERR_BUSY;
	}

	if (night) {
		/* Turn the image gray before exposing the sensor to infrared. */
		ret = pt_dn_set_image_mode(true);
		if (ret != BK_OK) {
			return ret;
		}
		rtos_delay_milliseconds(PT_DN_IR_LED_SETTLE_MS);
		ret = pt_ir_led_set(true);
		if (ret != BK_OK) {
			return ret;
		}
		ret = pt_ircut_set(true);
	} else {
		/* Restore the optical filter before returning to a color image. */
		ret = pt_ir_led_set(false);
		if (ret != BK_OK) {
			return ret;
		}
		ret = pt_ircut_set(false);
		if (ret != BK_OK) {
			return ret;
		}
		/* The mechanical state is already day; retry only ISP if it fails. */
		s_dn.applied_mode = 0;
		ret = pt_dn_set_image_mode(false);
	}

	if (ret == BK_OK) {
		s_dn.applied_mode = (int8_t)night;
		LOGI("mode -> %s\r\n", night ? "night-infrared" : "day-color");
	}
	return ret;
}

static void pt_day_night_task(beken_thread_arg_t arg)
{
	(void)arg;

	while (s_running) {
		bk_isp_camera_ctlr_handle_t camera =
			app_isp_camera_ctlr_handle_get();
		uint32_t luminance = 0;
		avdk_err_t ret = BK_FAIL;
		if (camera != NULL) {
			ret = bk_isp_camera_ctlr_ioctl(
					camera,
					BK_CAM_IOCTL_GET_EXPOSURE_LUMINANCE,
					&luminance);
		}

		if (ret == AVDK_ERR_OK) {
			if (!s_dn.current_is_night) {
				if (luminance < PT_DN_DAY_TO_NIGHT_LUMA) {
					s_dn.day_count = 0;
					if (++s_dn.night_count >=
					    PT_DN_CONFIRM_COUNT) {
						s_dn.current_is_night = true;
						s_dn.state_stable = true;
						LOGI("DAY->NIGHT luminance=%u\r\n",
						     luminance);
					}
				} else {
					s_dn.night_count = 0;
					if (++s_dn.day_count >=
					    PT_DN_CONFIRM_COUNT) {
						s_dn.state_stable = true;
					}
				}
			} else if (luminance > PT_DN_NIGHT_TO_DAY_LUMA) {
				s_dn.night_count = 0;
				if (++s_dn.day_count >= PT_DN_CONFIRM_COUNT) {
					s_dn.current_is_night = false;
					s_dn.state_stable = true;
					LOGI("NIGHT->DAY luminance=%u\r\n",
					     luminance);
				}
			} else {
				s_dn.day_count = 0;
			}

			pt_dn_service_lockout();
			ret = pt_dn_apply_mode(s_dn.current_is_night);
			if (ret != BK_OK && ret != BK_ERR_BUSY) {
				LOGW("apply %s mode failed: %d\r\n",
				     s_dn.current_is_night ? "night" : "day",
				     ret);
			}
		}

		rtos_delay_milliseconds(s_dn.state_stable ?
					PT_DN_STABLE_SAMPLE_MS :
					PT_DN_FAST_SAMPLE_MS);
	}

	s_thread = NULL;
	if (s_exit_sem != NULL) {
		rtos_set_semaphore(&s_exit_sem);
	}
	rtos_delete_thread(NULL);
}

bk_err_t pt_day_night_start(void)
{
	bk_err_t ret;

	if (s_thread != NULL) {
		return BK_OK;
	}

	os_memset(&s_dn, 0, sizeof(s_dn));
	s_dn.applied_mode = -1;
	s_dn.applied_image = -1;

	ret = pt_ir_led_init();
	if (ret != BK_OK) {
		LOGE("IR LED init failed: %d\r\n", ret);
		return ret;
	}
	ret = pt_ircut_init();
	if (ret != BK_OK) {
		LOGE("IR-CUT init failed: %d\r\n", ret);
		return ret;
	}
	s_initialized = true;

	/* Establish a known daytime optical state before automatic detection. */
	ret = pt_dn_apply_mode(false);
	if (ret != BK_OK) {
		LOGE("initial day mode failed: %d\r\n", ret);
		s_initialized = false;
		return ret;
	}

	ret = rtos_init_semaphore(&s_exit_sem, 1);
	if (ret != BK_OK) {
		s_initialized = false;
		return ret;
	}
	s_running = true;
	ret = rtos_create_thread(&s_thread, BEKEN_DEFAULT_WORKER_PRIORITY,
				 "pt_day_night",
				 (beken_thread_function_t)pt_day_night_task,
				 PT_DN_THREAD_STACK_SIZE, NULL);
	if (ret != BK_OK) {
		s_running = false;
		rtos_deinit_semaphore(&s_exit_sem);
		s_exit_sem = NULL;
		s_initialized = false;
		return ret;
	}

	LOGI("automatic day/night switching started\r\n");
	return BK_OK;
}

void pt_day_night_stop(void)
{
	if (!s_initialized) {
		return;
	}

	if (s_thread != NULL) {
		s_running = false;
		if (rtos_get_semaphore(&s_exit_sem, PT_DN_STOP_WAIT_MS) != BK_OK) {
			LOGW("thread stop timeout\r\n");
		}
	}
	if (s_exit_sem != NULL) {
		rtos_deinit_semaphore(&s_exit_sem);
		s_exit_sem = NULL;
	}

	pt_dn_reset_lockout();
	s_dn.applied_mode = -1;
	s_dn.applied_image = -1;
	if (pt_dn_apply_mode(false) != BK_OK) {
		/* Camera teardown may already have begun; still force the lamp off. */
		pt_ir_led_set(false);
	}
	s_initialized = false;
	LOGI("automatic day/night switching stopped\r\n");
}
