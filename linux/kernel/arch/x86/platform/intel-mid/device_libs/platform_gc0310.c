/*
 * platform_gc0310.c: GC0310 with iCatch 7002A ISP platform data initilization file
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
#include "platform_camera.h"
#include "platform_gc0310.h"
#include <linux/lnw_gpio.h>

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000

static int camera_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;

static int gc0310_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200; //Intel just can support 19.2MHz/9.6MHz/4.8MHz 
	int ret = 0;
	v4l2_err(sd, "%s: ++\n",__func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
	return ret;
}

static int gc0310_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	if (camera_power_down < 0) {
                /*ret = camera_sensor_gpio(GP_CORE_064, GP_CAMERA_SUB_CAM_PWDN,
                                 GPIOF_DIR_OUT, 0);
                if (ret < 0){
                        printk("%s not available.\n", GP_CAMERA_SUB_CAM_PWDN);
                        return ret;
                }*/
                camera_power_down = GP_CORE_064;
		lnw_gpio_set_alt(camera_power_down, LNW_GPIO);
        }
		
	if (flag == 1) {
		pr_info("%s(%d): pwdn(0)\n", __func__, __LINE__);
		gpio_set_value(camera_power_down, 0);
		pr_info("%s(%d): pwdn(1)\n", __func__, __LINE__);
		gpio_set_value(camera_power_down, 1);
		usleep_range(10, 20);
		pr_info("%s(%d): pwdn(0)\n", __func__, __LINE__);
		gpio_set_value(camera_power_down, 0);
	} else if (flag == 2) {
		pr_info("%s(%d): pwdn(1)\n", __func__, __LINE__);
                gpio_set_value(camera_power_down, 1);
	}else if (flag == 0) {
		pr_info("%s(%d): pwdn(0)\n", __func__, __LINE__);
                gpio_set_value(camera_power_down, 0);
		gpio_free(camera_power_down);
		camera_power_down = -1;
	}

	return 0;
}

static int gc0310_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret;
	printk("%s: ++\n",__func__);

	if (HW_ID == 0xFF){
		HW_ID = Read_HW_ID();
	}

	if (flag) {

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
			}

		//turn on MCLK
		gc0310_flisclk_ctrl(sd, 1);
		msleep(10);

		/* Power-down & Reset */
		ret = gc0310_gpio_ctrl(sd, 1);
		if (ret) {
			pr_err("%s gpio_ctrl on failed\n", __func__);
			return ret;
		}
		usleep_range(5, 10); /* Delay for I2C cmds: 100 mclk cycles */

	} else {
		/* Power-down & Reset */
		ret = gc0310_gpio_ctrl(sd, 2);
		if (ret) {
			pr_err("%s gpio_ctrl off- Highstage failed\n", __func__);
			return ret;
		}

		usleep_range(5000, 6000);

		 //turn off MCLK
                gc0310_flisclk_ctrl(sd, 0);
                msleep(10);

			if (camera_vprog2_on) {
					ret = regulator_disable(vprog2_reg);
					if (!ret) {
						camera_vprog2_on = 0;
					}
					else{
						printk(KERN_ALERT "Failed to disable regulator vprog2\n");
						return ret;
					}
			}

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
                msleep(10);

		ret = gc0310_gpio_ctrl(sd, 0);
                if (ret) {
                        pr_err("%s gpio_ctrl off- Lowstage failed\n", __func__);
                        return ret;
                }
 
	}
	return 0;
}

static int gc0310_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_RAW_8, atomisp_bayer_order_rggb,
		flag);
}

static int gc0310_platform_init(struct i2c_client *client)
{
	int ret;

	printk("%s: ++\n", __func__);
	//VPROG1 for VDD_HOST and VDD_SEN, 1.8V
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

		camera_power_down = GP_CORE_064;
			/* set camera reset pin mode to gpio */
		lnw_gpio_set_alt(camera_power_down, LNW_GPIO);

		pr_info("%s(%d): pwdn(0)\n", __func__, __LINE__);
		gpio_set_value(camera_power_down, 0);
		gpio_free(camera_power_down);
		camera_power_down = -1;

	return ret;
}

static int gc0310_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);

}

static struct camera_sensor_platform_data gc0310_sensor_platform_data = {
	.gpio_ctrl	 = gc0310_gpio_ctrl,
	.flisclk_ctrl	 = gc0310_flisclk_ctrl,
	.power_ctrl	 = gc0310_power_ctrl,
	.csi_cfg	 = gc0310_csi_configure,
	.platform_init   = gc0310_platform_init,
	.platform_deinit = gc0310_platform_deinit,
};

void *gc0310_platform_data(void *info)
{
	camera_power_down = -1;
	return &gc0310_sensor_platform_data;
}
