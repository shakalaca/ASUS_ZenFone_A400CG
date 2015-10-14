/*
 * platform_gc2155.c: gc2155 platform data initilization file
 *
 * (C) Copyright 2014 ASUS Corporation
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
#include <linux/mfd/intel_mid_pmic.h>
//#include <linux/vlv2_plat_clock.h>
#include "platform_camera.h"
#include "platform_gc2155.h"
#include <linux/acpi_gpio.h>
#include <linux/lnw_gpio.h>

static int camera_reset;
static int gpio_cam_pwdn;

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000

static int camera_vprog1_on;
static int camera_vprog2_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
/*
 * MFLD PR2 secondary camera sensor - gc2155 platform data
 */
static int gc2155_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	pr_info("%s - E, flag: %d\n", __func__, flag);

	if (gpio_cam_pwdn < 0) {
	    ret = camera_sensor_gpio(-1, "SUB_CAM_PWDN",
						GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		gpio_cam_pwdn = GP_CORE_064;
		/* set camera pwdn pin mode to gpio */
		lnw_gpio_set_alt(gpio_cam_pwdn, LNW_GPIO);
	}
	
	if (camera_reset < 0) {	
			ret = camera_sensor_gpio(-1, "SUB_CAM_RST#_R",
						GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		camera_reset = GP_CORE_080;
		/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(camera_reset, LNW_GPIO);
	}

	if (flag) {
		//gc2155 Power up:
		pr_info("%s(%d): pwdn(0)\n", __func__, __LINE__);
		gpio_set_value(gpio_cam_pwdn, 0);
		
		msleep(2);
		
		pr_info("%s(%d): reset(1)\n", __func__, __LINE__);
		gpio_set_value(camera_reset, 1);
	} else {
		// gc2155 Power down: 
		pr_info("%s(%d): pwdn(1)\n", __func__, __LINE__);
		gpio_set_value(gpio_cam_pwdn, 1);
		gpio_free(gpio_cam_pwdn);
		gpio_cam_pwdn = -1;

		msleep(2);

		pr_info("%s(%d): reset(0)\n", __func__, __LINE__);
		gpio_set_value(camera_reset, 0);
		gpio_free(camera_reset);
		camera_reset = -1;
	}

	return 0;
}

static int gc2155_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
    int ret = 0;

	pr_info("%s(), flag: %d\n", __func__, flag);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);	
	return ret;
}

static int gc2155_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	pr_info("%s() %s++\n", __func__, (flag) ? ("on") : ("off"));

	if (flag) {
		// turn on DVDD, IOVDD 1.8V 
		if (!camera_vprog1_on) {
			ret = regulator_enable(vprog1_reg);
			usleep_range(1000,1500);

			if (!ret) {
				camera_vprog1_on = 1;
				printk("<<<VPROG1 1.8V = 1\n");

			} else {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return ret;
			}
		}

		msleep(2);

		// turn on AVDD 2.8V
		if (!camera_vprog2_on) {
			ret = regulator_enable(vprog2_reg);
			usleep_range(1000, 5000);
			if (!ret) {
				camera_vprog2_on = 1;
				printk("<<VPROG2 = 1\n");
			} else {
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return ret;
			}
		}

		// Enable MCLK: 19.2MHz 
		pr_info("%s(%d): mclk on\n", __func__, __LINE__);
		ret = gc2155_flisclk_ctrl(sd, 1);
		if (ret) {
			pr_err("%s flisclk_ctrl on failed\n", __func__);
			return ret;
		}

		usleep_range(5000, 6000);

		//  Power-down & Reset  
		ret = gc2155_gpio_ctrl(sd, 1);
		if (ret) {
			pr_err("%s gpio_ctrl on failed\n", __func__);
			return ret;
		}
		usleep_range(5, 10); // Delay for I2C cmds: 100 mclk cycles 
	} else {
		// Power-down & Reset 
		ret = gc2155_gpio_ctrl(sd, 0);
		if (ret) {
			pr_err("%s gpio_ctrl off failed\n", __func__);
			return ret;
		}

		usleep_range(5000, 6000);

		// Disable MCLK 
		pr_info("%s(%d): mclk off\n", __func__, __LINE__);
		ret = gc2155_flisclk_ctrl(sd, 0);
		if (ret) {
			pr_err("%s flisclk_ctrl off failed\n", __func__);
			return ret;
		}

		usleep_range(1000, 5000);

		// tuen off AVDD 2.8V 
		if (camera_vprog2_on) {
			ret = regulator_disable(vprog2_reg);
			if (!ret) {
				camera_vprog2_on = 0;
				printk("<<VPROG2 = 0\n");
			} else{
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return ret;
			}
		}

		msleep(2);

		// turn oFF DVDD, IOVDD 1.8V
		if (camera_vprog1_on) {
			ret = regulator_disable(vprog1_reg);
			if (!ret) {
				camera_vprog1_on = 0;
				printk("<<<VPROG1 1.8V = 0\n");
			} else {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return ret;
			}
		}
	}

	pr_info("%s() %s--\n", __func__, (flag) ? ("on") : ("off"));
	return ret;
}

static int gc2155_csi_configure(struct v4l2_subdev *sd, int flag)
{
	pr_info("%s: port: SECONDARY; MIPI lane num: 1\n", __func__);
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static int gc2155_platform_init(struct i2c_client *client)
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

	return 0;
}

static int gc2155_platform_deinit(void)
{
	pr_info("%s()\n", __func__);
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
	return 0;
}

static struct camera_sensor_platform_data gc2155_sensor_platform_data = {
	.gpio_ctrl	= gc2155_gpio_ctrl,
	.flisclk_ctrl	= gc2155_flisclk_ctrl,
	.power_ctrl	= gc2155_power_ctrl,
	.csi_cfg	= gc2155_csi_configure,
	.platform_init = gc2155_platform_init,
	.platform_deinit = gc2155_platform_deinit,
};

void *gc2155_platform_data(void *info)
{

	pr_info("%s()\n", __func__);

	gpio_cam_pwdn = -1;
	camera_reset = -1;

	return &gc2155_sensor_platform_data;
}

