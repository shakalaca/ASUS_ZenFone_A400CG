/*
 * platform_al3320a.c: light sensor platform data initilization file
 *
 * (C) Copyright 2013
 * Author : cheng_kao
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_al3320a.h"
#include <linux/al3320a.h>

void *al3320a_platform_data(void *info)
{
	static struct al3320a_i2c_platform_data al3320a_pdata;
	al3320a_pdata.int_gpio	= get_gpio_by_name("ALS_INT#");
	return &al3320a_pdata;
}
