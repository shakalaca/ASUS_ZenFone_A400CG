/*
 * platform_hx8528.c: hx8528 platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/hx8528_p72g.h>
#include "platform_hx8528_p72g.h"

void *hx8528_p72g_platform_data(void *info)
{
	static struct himax_i2c_platform_data himax_pdata;

	himax_pdata.abs_x_max			= 800; 
	himax_pdata.abs_y_max			= 1200;
	//himax_pdata.rst_gpio				= get_gpio_by_name("Touch_RST_N");
	//himax_pdata.intr_gpio			= get_gpio_by_name("TP_INT_N");

	return &himax_pdata;
}
