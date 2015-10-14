/*
 * platform_t4k35.c: t4k35 platform data initilization file
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
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_t4k35.h"

static int camera_reset;
static int camera_power_1p2_en;
static int vcm_power_2_8v;

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000

static int camera_vprog1_on;
static int camera_vprog2_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;

/*
 * MRFLD VV primary camera sensor - T4K35 platform data
 */

static int t4k35_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, "5M_CAM_PD#",
						GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
		//Reset control
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 1);
			printk("<<< reset = 1\n");
			/* min 250us -Initializing time of silicon */
			//usleep_range(250, 300);
			msleep(4);
		}
	} else {
		//pull low reset
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< reset = 0\n");
			gpio_free(camera_reset);
			camera_reset = -1;
		}
	}

	return 0;
}

static int t4k35_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	int ret = 0;
	v4l2_err(sd, "%s: ++\n",__func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
	msleep(5);
	return ret;
}

static int t4k35_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int reg_err;
	int ret = 0;

	if (camera_power_1p2_en < 0) {
		ret = camera_sensor_gpio(-1, "CAM_1P2_EN",
						GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("CAM_1P2_EN is not available.\n");
			return ret;
		}
		camera_power_1p2_en = ret;
	}

	if (vcm_power_2_8v < 0) {
		ret = camera_sensor_gpio(-1, "5M_CAM_VCM_PD",
						GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("5M_CAM_VCM_PD is not available.\n");
			return ret;
		}
		vcm_power_2_8v = ret;
	}

	if (flag) {
		if (camera_reset < 0) {
			ret = camera_sensor_gpio(-1, "5M_CAM_PD#",
							GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("5M_CAM_PD# is not available.\n");
				return ret;
			}
			camera_reset = ret;
		}

		//Set power down to 0 first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< reset = 0\n");
			/* min 250us -Initializing time of silicon */
			usleep_range(250, 300);
		}

		//turn on VCM power 2.8V
		if (vcm_power_2_8v >= 0){
			gpio_set_value(vcm_power_2_8v, 1);
			printk("<<< VCM 2.8V = 1\n");
		}

		//turn on 1.2V power
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 1);
			printk("<<< LVDD VDIG 1.2V = 1\n");
		}

		//turn on 1.8V power
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< IOVDD PLLVDD 1.8V = 1\n");
		}

		//turn on AVDD power 2.8V
		if (!camera_vprog2_on) {
			camera_vprog2_on = 1;
			reg_err = regulator_enable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD 2.8V = 1\n");
		}
		msleep(1);
	} else {
		//turn off AVDD power 2.8V
		if (camera_vprog2_on) {
			camera_vprog2_on = 0;
			reg_err = regulator_disable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD 2.8V = 0\n");
		}

		//turn off 1.8V power
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< IOVDD PLLVDD 1.8V = 0\n");
		}

		//turn off 1.2V power
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 0);
			printk("<<< LVDD VDIG 1.2V = 0\n");
			gpio_free(camera_power_1p2_en);
			camera_power_1p2_en = -1;
		}

		//turn off VCM power 2.85V
		if (vcm_power_2_8v >= 0){
			gpio_set_value(vcm_power_2_8v, 0);
			printk("<<< VCM 2.8V = 0\n");
			gpio_free(vcm_power_2_8v);
			vcm_power_2_8v = -1;
		}
	}
	return 0;
}
static int t4k35_platform_init(struct i2c_client *client)
{
	int ret;

	//VPROG1 for IOVDD and PLLVDD, 1.8V
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "vprog1 failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog1 set failed\n");
		regulator_put(vprog1_reg);
	}

	//VPROG2 for AVDD, 2.8V
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "vprog2 failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog2 set failed\n");
		regulator_put(vprog2_reg);
	}

	return 0;
}

static int t4k35_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);

	return 0;
}

static int t4k35_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
		//ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_gbrg, flag);
}


static struct camera_sensor_platform_data t4k35_sensor_platform_data = {
	.gpio_ctrl      = t4k35_gpio_ctrl,
	.flisclk_ctrl   = t4k35_flisclk_ctrl,
	.power_ctrl     = t4k35_power_ctrl,
	.csi_cfg        = t4k35_csi_configure,
	.platform_init = t4k35_platform_init,
	.platform_deinit = t4k35_platform_deinit,
};

void *t4k35_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_1p2_en = -1;
	vcm_power_2_8v = -1;

	return &t4k35_sensor_platform_data;
}
