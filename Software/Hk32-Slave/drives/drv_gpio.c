#include "include.h"







void sys_board_gpio_init()
{
	charge_detection_gpio_init();
	pd_usb_gpio_init();
	sys_power_gpio_init();
	pwm_gpio_init();
	adc_gpio_init();
}

