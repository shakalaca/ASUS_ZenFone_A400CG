/*
 * platform_elan_touchpad.c: ELAN Touchpad platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/elan_touchpad.h>
#include "platform_elan_touchpad.h"

void *elan_touchpad_platform_data(void *info)
{
	static struct elan_touchpad_i2c_platform_data elan_touchpad_pdata;

	elan_touchpad_pdata.int_gpio = get_gpio_by_name("I2C_2_INT#_TP");
	elan_touchpad_pdata.tp_sw_gpio = get_gpio_by_name("TP_I2C_SW");
	

	return &elan_touchpad_pdata;
}
