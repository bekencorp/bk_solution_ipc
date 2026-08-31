#include <driver/gpio.h>

#include "pt_ir_led.h"

#define PT_IR_LED_GPIO GPIO_49

bk_err_t pt_ir_led_init(void)
{
	bk_err_t ret = bk_gpio_enable_output(PT_IR_LED_GPIO);
	if (ret != BK_OK) {
		return ret;
	}
	return bk_gpio_set_output_low(PT_IR_LED_GPIO);
}

bk_err_t pt_ir_led_set(bool enable)
{
	return enable ? bk_gpio_set_output_high(PT_IR_LED_GPIO)
		      : bk_gpio_set_output_low(PT_IR_LED_GPIO);
}
