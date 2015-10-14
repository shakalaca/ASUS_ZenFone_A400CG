/*
 * platform_mt9m114_raw.c: mt9m114_raw platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_ipcutil.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_mt9m114_raw.h"
#include "platform_mt9e013.h"


#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on; //duel sim
static int camera_sensor_2_8v;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;

/*
 * MFLD PR2 secondary camera sensor - MT9M114_RAW platform data
 */
static int mt9m114_raw_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(176, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = 176;
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(60);
		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			gpio_free(camera_reset);
			camera_reset = -1;
			msleep(10);
		}
	}

	return 0;
}

static int mt9m114_raw_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int mt9m114_raw_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err, ret;
	int PCB_ID1 = 0; //For duel SIM++

	if (PROJECT_ID == 0xFF) {
                PROJECT_ID = Read_PROJ_ID();
        }	

	//For duel SIM++
        if(PROJECT_ID == PROJ_ID_ME175CG){
                ret = gpio_request(174, "PCB_ID1");
                if (ret) {
                        pr_err("%s: failed to request gpio(pin 174)\n", __func__);
                        return -EINVAL;
                }

                ret = gpio_direction_input(174);
                PCB_ID1 = gpio_get_value(174);
                printk("%s: PCB_ID1 %d\n", __func__, PCB_ID1);
                gpio_free(174);
        }
        //For duel SIM--	

	if(PCB_ID1 != 0){
		printk(KERN_ALERT "Duel SIM SKU: Change to vprog2\n");
	}else{
		if (camera_sensor_2_8v < 0) {
			ret = camera_sensor_gpio(GP_CORE_014, GP_SENSOR_2_8V,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_SENSOR_2_8V);
				return ret;
			}
			camera_sensor_2_8v = GP_CORE_014;
		}
	}

	if (flag) {
		if (!camera_vprog1_on) {
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			camera_vprog1_on = 1;
			printk("<<< VPROG1 = 1\n");
			msleep(10);
		}
		
		if(PCB_ID1 != 0){
			if (!camera_vprog2_on) {
				ret = regulator_enable(vprog2_reg);
				if (reg_err) {
					printk(KERN_ALERT "Failed to enable regulator vprog2\n");
					return reg_err;
				}
				camera_vprog2_on = 1;
				printk("<<< VPROG2 = 1\n");
				msleep(10);
			}
		}else{		
			//turn on power sensor AVDD 2.8V
			if (camera_sensor_2_8v >= 0){
				gpio_set_value(camera_sensor_2_8v, 1);
				printk("<<< AVDD_SENSOR 2.8V = 1\n");
				msleep(10);
			}
		}
	} else {
		
		if(PCB_ID1 != 0){
			if (camera_vprog2_on) {
				reg_err = regulator_disable(vprog2_reg);
				if (reg_err) {
					printk(KERN_ALERT "Failed to disable regulator vprog2\n");
					return reg_err;
				}
				camera_vprog2_on = 0;
				printk("<<< VPROG2 = 0\n");
				msleep(10);
			}
		}else{
			if (camera_sensor_2_8v >= 0){
				gpio_set_value(camera_sensor_2_8v, 0);
				printk("<<< AVDD_SENSOR 2.8V = 0\n");
				gpio_free(camera_sensor_2_8v);
				camera_sensor_2_8v = -1;
				msleep(10);
			}
		}	
		if (camera_vprog1_on) {
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
			camera_vprog1_on = 0;
			printk("<<< VPROG1 = 0\n");
		}
	}

	return 0;
}

static int mt9m114_raw_csi_configure(struct v4l2_subdev *sd, int flag)
{

#ifdef CONFIG_ME372CL
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
                ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_gbrg, flag);
#else
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
#endif
//	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
//		-1, 0, flag);

//	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
//			ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);	
}

static int mt9m114_raw_platform_init(struct i2c_client *client)
{
	int ret;

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}

#ifdef CONFIG_ME175CG
	vprog2_reg = regulator_get(&client->dev, "vprog2");
        if (IS_ERR(vprog2_reg)) {
                dev_err(&client->dev, "regulator_get failed\n");
                return PTR_ERR(vprog2_reg);
        }
        ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
        if (ret) {
                dev_err(&client->dev, "regulator voltage set failed\n");
                regulator_put(vprog2_reg);
        }
#endif
	return ret;
}

static int mt9m114_raw_platform_deinit(void)
{
	regulator_put(vprog1_reg);

#ifdef CONFIG_ME175CG
	regulator_put(vprog2_reg);
#endif
	return 0;
}

static struct camera_sensor_platform_data mt9m114_raw_sensor_platform_data = {
	.gpio_ctrl	= mt9m114_raw_gpio_ctrl,
	.flisclk_ctrl	= mt9m114_raw_flisclk_ctrl,
	.power_ctrl	= mt9m114_raw_power_ctrl,
	.csi_cfg	= mt9m114_raw_csi_configure,
	.platform_init = mt9m114_raw_platform_init,
	.platform_deinit = mt9m114_raw_platform_deinit,
};

void *mt9m114_raw_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_sensor_2_8v = -1;

	return &mt9m114_raw_sensor_platform_data;
}

