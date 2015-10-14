/*
 * platform_gt927.c: gt927 platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_gt927.h"

#include <linux/i2c/gt927.h>

void *gt927_platform_data(void *info)
{
	static struct goodix_gt927_i2c_platform_data goodix_pdata;

	//goodix_pdata.abs_x_max	= ELAN_X_MAX - 1; 
	//goodix_pdata.abs_y_max	= ELAN_Y_MAX - 1;
	//goodix_pdata.abs_x_min	= 0;
	//goodix_pdata.abs_y_min	= 0;
	goodix_pdata.rst_gpio		= get_gpio_by_name("Touch_RST_N");
	goodix_pdata.intr_gpio		= get_gpio_by_name("TP_INT_N");
	goodix_pdata.pwr_en_gpio	= get_gpio_by_name("TP_PWR_EN");
	printk("intr_gpio=%d, rst_gpio=%d\n",goodix_pdata.intr_gpio,goodix_pdata.rst_gpio);

	return &goodix_pdata;
}
