/*
 * platform_gyro.c: mpu6500 platform data initilization file
 *
 * (C) Copyright 2013
 * Author : cheng_kao
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_gyro.h"
#include <linux/mpu.h>

void *gyro_platform_data(void *info)
{
	static struct mpu_platform_data mpu6500_pdata;
//	mpu6500_pdata.int_config  = 0x10;
//	mpu6500_pdata.level_shifter = 0;
//	mpu6500_pdata.orientation = {   0,  -1,  0, -1,  0,  0, 0,  0, -1 };
//	mpu6500_pdata.sec_slave_type = SECONDARY_SLAVE_TYPE_NONE;
//	mpu6500_pdata.sec_slave_id   = ID_INVALID;
//	mpu6500_pdata.secondary_i2c_addr = 0;

	mpu6500_pdata.int_gpio	= get_gpio_by_name("GYRO_INT#");
	
	return &mpu6500_pdata;

}
