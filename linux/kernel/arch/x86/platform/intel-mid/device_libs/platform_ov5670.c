/*
 * platform_ov5670.c: ov5670 platform data initilization file
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
#include "platform_ov5670.h"

static int camera_reset;

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000

static int camera_vprog1_on;
static int camera_vprog2_on; 
static int camera_vemmc1_on;
static int camera_power_1p2_en;
static int vcm_power_2_8v;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;

/*
 * CLV PR0 primary camera sensor - OV5670 platform data
 */

static int ov5670_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int reset_gpio_pin;

	if (PROJECT_ID == 0xFF) {
		PROJECT_ID = Read_PROJ_ID();
	}
	
	if((PROJECT_ID == PROJ_ID_FE171CG) || (PROJECT_ID == PROJ_ID_ME171C)){
		if (camera_reset < 0) {
		    ret = camera_sensor_gpio(-1, "5M_CAM_PD#",
						GPIOF_DIR_OUT, 0);
		    if (ret < 0)
			    return ret;
		    camera_reset = ret;
		}
	}
	else{
	    reset_gpio_pin = 161; //ME175CG

	    if (camera_reset < 0) {
		    ret = camera_sensor_gpio(reset_gpio_pin, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		    if (ret < 0) {
			    printk("%s not available.\n", GP_CAMERA_0_RESET);
			    return ret;
		    }
		    camera_reset = reset_gpio_pin;
		}
	}
	
	if (flag) {
		gpio_set_value(camera_reset, 1);
		printk("<<< camera_reset = 1\n");
		usleep_range(6000, 6500);
	} else {
		gpio_set_value(camera_reset, 0);
		printk("<<< camera_reset = 0\n");
		gpio_free(camera_reset);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		udelay(1);
	}

	return 0;
}

static int ov5670_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

/*
 * Checking the SOC type is temporary workaround to enable OV5670
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov5670_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int reg_err;
    int reset_gpio_pin;
	
	if (PROJECT_ID == 0xFF) {
		PROJECT_ID = Read_PROJ_ID();
	}

	if (HW_ID == 0xFF){
		HW_ID = Read_HW_ID();
	}
	
	if((PROJECT_ID == PROJ_ID_FE171CG) || (PROJECT_ID == PROJ_ID_ME171C)){
		if (camera_reset < 0) {
		    ret = camera_sensor_gpio(-1, "5M_CAM_PD#",
						GPIOF_DIR_OUT, 0);
		    if (ret < 0)
			    return ret;
		    camera_reset = ret;
		}
		
		if (camera_power_1p2_en < 0) {
		    ret = camera_sensor_gpio(-1, "CAM_1P2_EN",
						GPIOF_DIR_OUT, 0);
		    if (ret < 0){
			    printk("CAM_1P2_EN is not available.\n");
			    return ret;
		    }
		    camera_power_1p2_en = ret;
	    }
		
		if (vcm_power_2_8v <0) {
		    ret = camera_sensor_gpio(-1, "5M_CAM_VCM_PD",
						GPIOF_DIR_OUT, 0);
			if (ret < 0){
			    printk("5M_CAM_VCM_PD is not available.\n");
			    return ret;
		    }
            vcm_power_2_8v = ret; 			
		} 		
	}
	else{
	    reset_gpio_pin = 161; //ME175CG

	    if (camera_reset < 0) {
		    ret = camera_sensor_gpio(reset_gpio_pin, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		    if (ret < 0) {
			    printk("%s not available.\n", GP_CAMERA_0_RESET);
			    return ret;
		    }
		    camera_reset = reset_gpio_pin;
	    }
	}

	if (flag) {
	    
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< camera_reset = 0\n");
			msleep(1);
		}
		
		// turn on VCM power 2.85V
		if((PROJECT_ID == PROJ_ID_FE171CG) || (PROJECT_ID == PROJ_ID_ME171C)){
		    if (vcm_power_2_8v >= 0){
			    gpio_set_value(vcm_power_2_8v, 1);
			    msleep(10);
			    printk("<<< VCM 2.85V = 1\n");
		    }
		}
		else{
		    if (!camera_vemmc1_on) {
			    camera_vemmc1_on = 1;
			    reg_err = intel_scu_ipc_msic_vemmc1(1);
			    //reg_err = regulator_enable(vemmc1_reg);
			    if (reg_err) {
				    printk(KERN_ALERT "Failed to enable regulator vemmc1\n");
				    return reg_err;
			    }
			    printk("<<< VCM 2.85V = 1\n");
			    msleep(10);
		    }
		}
		
        // turn on power AVDD 2.8V
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
		
		// turn on power DOVDD 1.8V 
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< DOVDD 1.8V = 1\n");
		}
		
		if((PROJECT_ID == PROJ_ID_FE171CG) || (PROJECT_ID == PROJ_ID_ME171C)){
		    // turn on power DVDD 1.2V
		    if (camera_power_1p2_en >= 0){
			    gpio_set_value(camera_power_1p2_en, 1);
			    msleep(10);
			    printk("<<< DVDD 1.2V = 1\n");
		    }
		}
		
	} else {
	
		if (camera_reset >= 0){
			gpio_free(camera_reset);
			camera_reset = -1;
		}
		
        // turn off power AVDD 2.8V		
		if (camera_vprog2_on) {
			camera_vprog2_on = 0;
			reg_err = regulator_disable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD 2.8V = 0\n");
		}
		
		// turn off power DOVDD 1.8V 	
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			reg_err = regulator_disable(vprog1_reg); 
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< DOVDD 1.8V = 0\n");
		}
		
		if((PROJECT_ID == PROJ_ID_FE171CG) || (PROJECT_ID == PROJ_ID_ME171C)){
		    // turn off DVDD power 1.2V
		    if (camera_power_1p2_en >= 0){
			    gpio_set_value(camera_power_1p2_en, 0);
			    printk("<<< DVDD 1.2V = 0\n");
			    gpio_free(camera_power_1p2_en);
			    camera_power_1p2_en = -1;
			    msleep(10);
		    }
			// turn off VCM power 2.85V
			if (vcm_power_2_8v >= 0){
			    gpio_set_value(vcm_power_2_8v, 0);
				gpio_free(vcm_power_2_8v);
			    vcm_power_2_8v = -1;
			    msleep(10);
			    printk("<<< VCM 2.85V = 0\n");
		    }
		}
		else{
		    if (camera_vemmc1_on) {
			    camera_vemmc1_on = 0;
			    reg_err = intel_scu_ipc_msic_vemmc1(0);
			    //reg_err = regulator_disable(vemmc1_reg);
			    if (reg_err) {
				    printk(KERN_ALERT "Failed to disable regulator vemmc1\n");
				    return reg_err;
			    }
			    printk("<<< VCM 2.85V = 0\n");
			    msleep(10);
		    }
		}
	}
	return 0;
}

static int ov5670_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;

	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
               ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

/*
 * Checking the SOC type is temporary workaround to enable OV5670
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov5670_platform_init(struct i2c_client *client)
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

/*
 * Checking the SOC type is temporary workaround to enable OV5670 on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int ov5670_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
	return 0;
}
static struct camera_sensor_platform_data ov5670_sensor_platform_data = {
	.gpio_ctrl      = ov5670_gpio_ctrl,
	.flisclk_ctrl   = ov5670_flisclk_ctrl,
	.power_ctrl     = ov5670_power_ctrl,
	.csi_cfg        = ov5670_csi_configure,
	.platform_init = ov5670_platform_init,
	.platform_deinit = ov5670_platform_deinit,
};

void *ov5670_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_1p2_en = -1;
	vcm_power_2_8v = -1;

	return &ov5670_sensor_platform_data;
}
