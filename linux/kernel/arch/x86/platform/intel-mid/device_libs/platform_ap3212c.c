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
#include "platform_ap3212c.h"

static struct ap3212c_platform_data {
	int gpio;
};

void *ap3212c_platform_data(void *info)
{
	static struct ap3212c_platform_data platform_data; 
	platform_data.gpio=get_gpio_by_name("ALS_INT#");
	
	return &platform_data;
}
