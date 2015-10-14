/*
 * platform_ektf3k.c: ektf3k platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c/ektf3k.h>
#include "platform_ektf3k.h"

void *ektf3k_platform_data(void *info)
{
	static struct elan_ktf3k_i2c_platform_data elan_pdata;

	elan_pdata.abs_x_max	= ELAN_X_MAX - 1; 
	elan_pdata.abs_y_max	= ELAN_Y_MAX - 1;
	elan_pdata.abs_x_min	= 0;
	elan_pdata.abs_y_min	= 0;
	elan_pdata.rst_gpio		= get_gpio_by_name("Touch_RST_N");
	elan_pdata.intr_gpio	= get_gpio_by_name("TP_INT_N");
	elan_pdata.pwr_en_gpio	= get_gpio_by_name("TP_PWR_EN");

	return &elan_pdata;
}
