/*
 * platform_compass.c: compass platform data initilization file
 *
 * (C) Copyright 2013
 * Author : cheng_kao
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_compass.h"
#include <linux/akm8963.h>
void *compass_platform_data(void *info)
{

	static struct akm8963_platform_data akm8963_pdata;
	akm8963_pdata.gpio_DRDY	= get_gpio_by_name("COMP_DRDY");
	akm8963_pdata.gpio_RSTN	= get_gpio_by_name("ecompass_reset");
	return &akm8963_pdata;	
	
}
