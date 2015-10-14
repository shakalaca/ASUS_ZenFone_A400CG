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
#include "platform_cap1106.h"

#include <linux/HWVersion.h>
extern int Read_PROJ_ID(void);

static struct cap1106_platform_data {
	int gpio;
};

void *cap1106_platform_data(void *info)
{
	static struct cap1106_platform_data platform_data; 
/*
	if(Read_PROJ_ID()==PROJ_ID_ME372CG)
		platform_data.gpio=get_gpio_by_name("sar_det_n_3g");
	else
		platform_data.gpio=get_gpio_by_name("SAR_DET_3G");
*/
	platform_data.gpio=94;
	return &platform_data;
}
