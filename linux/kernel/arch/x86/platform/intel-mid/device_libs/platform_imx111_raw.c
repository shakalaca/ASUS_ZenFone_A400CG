/*
 * platform_imx111_raw.c: imx111_raw platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
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
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_imx111_raw.h"

static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc1_on;
static int camera_sensor_1_8v;
static int camera_power_1p2_en;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
/*
 * CLV PR0 primary camera sensor - IMX111_RAW platform data
 */

static int imx111_raw_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(GP_CORE_065, GP_CAMERA_MAIN_CAM_PWDN,
					GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			printk("%s not available.\n", GP_CAMERA_MAIN_CAM_PWDN);
			return ret;
		}
		camera_reset = GP_CORE_065;
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(20);
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

static int imx111_raw_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

/*
 * Checking the SOC type is temporary workaround to enable IMX111_RAW
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int imx111_raw_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	if (camera_sensor_1_8v < 0) {
		ret = camera_sensor_gpio(GP_AON_052, GP_CAMERA_CAM2_1V8_EN,
				 GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("%s not available.\n", GP_CAMERA_CAM2_1V8_EN);
			return ret;
		}
		camera_sensor_1_8v = GP_AON_052;
	}

	if (camera_power_1p2_en < 0) {
	ret = camera_sensor_gpio(GP_CORE_015, GP_CAMERA_CAM_1P2_EN,
					 GPIOF_DIR_OUT, 0);
		if (ret < 0){
			printk("%s not available.\n", GP_CAMERA_CAM_1P2_EN);
			return ret;
		}
		camera_power_1p2_en = GP_CORE_015;
	}

	if (flag) {
		//turn on VCM 2.8V
		if (!camera_vprog2_on) {
			ret = regulator_enable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 1;
				printk("<<< VCM 2.8V = 1\n");
			}
			else{
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return ret;
			}
			msleep(10);
		}

		//turn on 2.7V power
		if (!camera_vemmc1_on) {
			ret = intel_scu_ipc_msic_vemmc1(1);
			if (ret) {
				printk(KERN_ALERT "Failed to enable regulator vemmc1\n");
				return ret;
			}
			camera_vemmc1_on = 1;
			printk("<<< 2.7V = 1\n");
			msleep(10);
		}

		//turn on 1.2V power
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 1);
			printk("<<< 1.2V = 1\n");
			msleep(10);
		}

		//turn on 1.8V power
		if (camera_sensor_1_8v >= 0){
			gpio_set_value(camera_sensor_1_8v, 1);
			printk("<<< 1.8V = 1\n");
			msleep(10);
		}

		//turn on I2C 1.8V
		if (!camera_vprog1_on) {
			ret = regulator_enable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 1;
				printk("<<< I2C 1.8V = 1\n");
			}
			else{
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return ret;
			}
			msleep(10);
		}
	} else {
		//turn off I2C 1.8V
		if (camera_vprog1_on) {
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 0;
				printk("<<< I2C 1.8V = 0\n");
			}
			else {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return ret;
			}
			msleep(10);
		}
	
		//turn off 1.8V power
		if (camera_sensor_1_8v >= 0){
			gpio_set_value(camera_sensor_1_8v, 0);
			printk("<<< 1.8V = 0\n");
			gpio_free(camera_sensor_1_8v);
			camera_sensor_1_8v = -1;
			msleep(10);
		}

		//turn off 1.2V power
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 0);
			printk("<<< 1.2V = 0\n");
			gpio_free(camera_power_1p2_en);
			camera_power_1p2_en = -1;
			msleep(10);
		}

		//turn off 2.7V power
		if (camera_vemmc1_on) {
			camera_vemmc1_on = 0;
			ret = intel_scu_ipc_msic_vemmc1(0);
			if (ret) {
				printk(KERN_ALERT "Failed to disable regulator vemmc1\n");
				return ret;
			}
			printk("<<< 2.7V = 0\n");
			msleep(10);
		}

		//turn off VCM 2.8V
		if (camera_vprog2_on) {
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 0;
				printk("<<< VCM 2.8V = 0\n");
			}
			else {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return ret;
			}
			msleep(10);
		}		
	}
	return ret;
}

static int imx111_raw_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

/*
 * Checking the SOC type is temporary workaround to enable IMX111_RAW
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int imx111_raw_platform_init(struct i2c_client *client)
{
	int ret;

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog2_reg);
	}
	
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	} else {
		ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
		if (ret) {
			dev_err(&client->dev, "regulator voltage set failed\n");
			regulator_put(vprog2_reg);
		}		
	}
	return ret;
}

/*
 * Checking the SOC type is temporary workaround to enable IMX111_RAW on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int imx111_raw_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);

	return 0;
}
static struct camera_sensor_platform_data imx111_raw_sensor_platform_data = {
	.gpio_ctrl      = imx111_raw_gpio_ctrl,
	.flisclk_ctrl   = imx111_raw_flisclk_ctrl,
	.power_ctrl     = imx111_raw_power_ctrl,
	.csi_cfg        = imx111_raw_csi_configure,
	.platform_init = imx111_raw_platform_init,
	.platform_deinit = imx111_raw_platform_deinit,
};

void *imx111_raw_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_sensor_1_8v = -1;
	camera_power_1p2_en = -1;

	return &imx111_raw_sensor_platform_data;
}
