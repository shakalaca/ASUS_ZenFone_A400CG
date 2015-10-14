/*
 * platform_imx219.c: imx219 platform data initilization file
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
#include "platform_imx219.h"

static int camera_reset; //XCLR
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc1_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
/*
 * CLV PR0 primary camera sensor - IMX219 platform data
 */

static int imx219_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int reset_gpio_pin;

	printk("<<< imx219_gpio_ctrl = %d\n", flag);

	reset_gpio_pin = 161; //GP_CAMERA_SB2

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(reset_gpio_pin, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 0);
		if (ret < 0) {
			printk("%s not available.\n", GP_CAMERA_0_RESET);
			return ret;
		}
		camera_reset = reset_gpio_pin;
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(10);
		gpio_set_value(camera_reset, 1);
		usleep_range(6000, 6500);
	} else {
		gpio_set_value(camera_reset, 0);
		msleep(10);
		gpio_free(camera_reset);
		camera_reset = -1;
	}

	return 0;
}

static int imx219_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

/*
 * Checking the SOC type is temporary workaround to enable IMX219
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int imx219_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int reg_err;

	printk("<<< imx219_power_ctrl = %d\n", flag);

	if (flag) {
		//turn on VCM power 2.85V
		if (!camera_vemmc1_on) {
			camera_vemmc1_on = 1;
			reg_err = intel_scu_ipc_msic_vemmc1(1);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vemmc1\n");
				return reg_err;
			}
			printk("<<< VCM 2.85V = 1\n");
		}

		//turn on power 2.8V
		if (!camera_vprog2_on) {
			ret = regulator_enable(vprog2_reg);
			msleep(1);
			if (!ret) {
				camera_vprog2_on = 1;
			}
			else {
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return ret;
			}
			printk("<<< 2.8V = 1\n");
		}

		//turn on power 1.8V, 1.2V
		if (!camera_vprog1_on) {
			ret = regulator_enable(vprog1_reg);
			msleep(1);
			if (!ret) {
				camera_vprog1_on = 1;
			} else {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return ret;
			}
			printk("<<< 1.8V, 1.2V = 1\n");
		}
		msleep(2);
	} else {
		//turn off power 1.8V, 1.2V
		if (camera_vprog1_on) {
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 0;
			}
			else {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return ret;
			}
			printk("<<< 1.8V, 1.2V = 0\n");
		}

		//turn off power 2.8V
		if (camera_vprog2_on) {
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 0;
			} else{
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return ret;
			}
			printk("<<< 2.8V = 0\n");
		}

		//turn off VCM power 2.85V
		if (camera_vemmc1_on) {
			camera_vemmc1_on = 0;
			reg_err = intel_scu_ipc_msic_vemmc1(0);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vemmc1\n");
				return reg_err;
			}
			printk("<<< VCM 2.85V = 0\n");
		}
	}
	return 0;
}

static int imx219_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;

	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

/*
 * Checking the SOC type is temporary workaround to enable IMX219
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int imx219_platform_init(struct i2c_client *client)
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

	return ret;
}

/*
 * Checking the SOC type is temporary workaround to enable IMX219 on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int imx219_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);

	return 0;
}
static struct camera_sensor_platform_data imx219_sensor_platform_data = {
	.gpio_ctrl      = imx219_gpio_ctrl,
	.flisclk_ctrl   = imx219_flisclk_ctrl,
	.power_ctrl     = imx219_power_ctrl,
	.csi_cfg        = imx219_csi_configure,
	.platform_init = imx219_platform_init,
	.platform_deinit = imx219_platform_deinit,
};

void *imx219_platform_data(void *info)
{
	camera_reset = -1;

	return &imx219_sensor_platform_data;
}
