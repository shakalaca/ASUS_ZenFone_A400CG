/*
 * platform_asus_ec.c: ASUS EC platform data initilization file
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/ite8566.h>
#include "platform_asus_ec.h"

void *asus_ec_platform_data(void *info)
{
	static struct asus_ec_i2c_platform_data asus_ec_pdata;

	asus_ec_pdata.int0_gpio	= get_gpio_by_name("EC_KB_INT#");
	asus_ec_pdata.int1_gpio	= get_gpio_by_name("ALS_INT#");

	return &asus_ec_pdata;
}
