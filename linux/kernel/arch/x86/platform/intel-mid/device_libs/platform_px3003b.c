/*
 * platform_ltr502als.c: ltr502als platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/px3003b.h>
#include "platform_px3003b.h"


void *px3003b_platform_data(void *info)
{
	static struct px3003b_i2c_platform_data platform_data; 
	platform_data.int_gpio=get_gpio_by_name("ALS_INT#");
	
	return &platform_data;
}
