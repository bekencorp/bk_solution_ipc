#include <os/os.h>
#include <driver/gpio.h>

#include "pt_ircut.h"

#define PT_IRCUT_NIGHT_GPIO GPIO_47
#define PT_IRCUT_DAY_GPIO   GPIO_48
#define PT_IRCUT_PULSE_MS   300U

static bk_err_t pt_ircut_drive_low(gpio_id_t gpio)
{
	bk_err_t ret = bk_gpio_enable_output(gpio);
	if (ret != BK_OK) {
		return ret;
	}
	return bk_gpio_set_output_low(gpio);
}

bk_err_t pt_ircut_init(void)
{
	bk_err_t ret = pt_ircut_drive_low(PT_IRCUT_NIGHT_GPIO);
	if (ret != BK_OK) {
		return ret;
	}
	return pt_ircut_drive_low(PT_IRCUT_DAY_GPIO);
}

bk_err_t pt_ircut_set(bool night)
{
	gpio_id_t target = night ? PT_IRCUT_NIGHT_GPIO : PT_IRCUT_DAY_GPIO;
	bk_err_t ret;

	/* Always de-energize both coils before selecting a direction. */
	ret = bk_gpio_set_output_low(PT_IRCUT_NIGHT_GPIO);
	if (ret != BK_OK) {
		return ret;
	}
	ret = bk_gpio_set_output_low(PT_IRCUT_DAY_GPIO);
	if (ret != BK_OK) {
		return ret;
	}
	ret = bk_gpio_set_output_high(target);
	if (ret != BK_OK) {
		return ret;
	}

	rtos_delay_milliseconds(PT_IRCUT_PULSE_MS);
	return bk_gpio_set_output_low(target);
}
