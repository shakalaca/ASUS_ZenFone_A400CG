/*
 * platform_ar0543.c: AR0543 with iCatch 7002A ISP platform data initilization file
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
#include "platform_ar0543.h"
//FW_BSP++
#include <linux/proc_fs.h>	//ASUS_BSP+++
#include <linux/lnw_gpio.h>

#define SPI_magic_number 49
#define EEPROM_magic_number 50

static int camera_SPI_1_CLK;
static int camera_SPI_1_SS3;
static int camera_SPI_1_SDO;
static int camera_I2C_4_SCL;
static int camera_I2C_4_SDA;
static int camera_sensor_2_8v;
//FW_BSP--

#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
#define VEMMC1_VAL 2850000

static int camera_power_1p2_en;
static int camera_reset;
static int camera_suspend;
static int camera_vprog1_on;
static int camera_vprog2_on;
static int camera_vemmc1_on;

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
static struct regulator *vemmc1_reg;

static char DEFAULT_SPI_FILE_WITH_PATH[] = "/system/etc/camera_spi_init.txt";
static char EXTERNAL_SPI_FILE_WITH_PATH[] = "/data/media/0/camera_spi_init.txt";
static char *SPI_FILE_WITH_PATH = EXTERNAL_SPI_FILE_WITH_PATH;




/*
 * ME302C Camera ISP - AR0543 with iCatch 7002A ISP platform data
 */

static int spi_init_extra_parameter()
{
	struct file *fp = NULL;
	int ret = -1;
	u8 *pbootBuf = NULL;

	struct inode *inode;
	int bootbin_size = 0;

	mm_segment_t old_fs;
	printk("%s ++\n", __func__);

	fp = filp_open(SPI_FILE_WITH_PATH, O_RDONLY, 0);
	
	if ( !IS_ERR_OR_NULL(fp) ){
		pr_info("filp_open success fp:%p\n", fp);
		inode = fp->f_dentry->d_inode;
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		if(fp->f_op != NULL && fp->f_op->read != NULL){
			printk("Start to read %s\n", SPI_FILE_WITH_PATH);
		}
		set_fs(old_fs);
		filp_close(fp, NULL);
	} else if(PTR_ERR(fp) == -ENOENT) {
		pr_err(" \"%s\" not found error\n", SPI_FILE_WITH_PATH);
		SPI_FILE_WITH_PATH = DEFAULT_SPI_FILE_WITH_PATH;

	} else{
		pr_err(" \"%s\" open error\n", SPI_FILE_WITH_PATH);
		SPI_FILE_WITH_PATH = DEFAULT_SPI_FILE_WITH_PATH;
	}
	

	char *pFile = SPI_FILE_WITH_PATH;
	fp = filp_open(pFile, O_RDONLY, S_IRWXU|S_IRWXG|S_IRWXO);

	if ( !IS_ERR_OR_NULL(fp) ){
		inode = fp->f_dentry->d_inode;
		bootbin_size = inode->i_size;
		if (bootbin_size > 0) {
			pbootBuf = kmalloc(bootbin_size, GFP_KERNEL);
			old_fs = get_fs();
			set_fs(KERNEL_DS);
			if(fp->f_op != NULL && fp->f_op->read != NULL){
				int byte_count = 0;
				printk("Read SPI init parameter\n");
				byte_count = fp->f_op->read(fp, pbootBuf, bootbin_size, &fp->f_pos);
			}
			set_fs(old_fs);
		}
		filp_close(fp, NULL);
	} else {
		printk("No extra parameter\n");
		return 0;
	}

	if (bootbin_size > 0) {
		ret = pbootBuf[0];
		kfree(pbootBuf);
	}

	return ret;
}


static int ar0543_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200; //Intel just can support 19.2MHz/9.6MHz/4.8MHz 
	int ret = 0;
	v4l2_err(sd, "%s: ++\n",__func__);
	ret = intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
	return ret;
}


static int ar0543_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
		int reg_err;
		int ret;
		printk("%s: ++\n",__func__);
		//read HW ID
		if (HW_ID == 0xFF){
			HW_ID = Read_HW_ID();
		}
		//printk("HW ID:%d\n", HW_ID);

		if (PROJECT_ID == 0xFF) {
			PROJECT_ID = Read_PROJ_ID();
		}	
	
		switch (HW_ID){
			case HW_ID_SR1:
			case HW_ID_SR2:
				if (camera_power_1p2_en < 0) {
					ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_POWER_1P2_EN,
								 GPIOF_DIR_OUT, 0);
					if (ret < 0){
						printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
						return ret;
					}
					camera_power_1p2_en = ret;
				}
				if (camera_reset < 0) {
					ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_RESET,
								 GPIOF_DIR_OUT, 0);
					if (ret < 0){
						printk("%s not available.\n", GP_CAMERA_ISP_RESET);
						return ret;
					}
					camera_reset = ret;
				}
				if (camera_suspend < 0) {
					ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_SUSPEND,
								GPIOF_DIR_OUT, 0);
					if (ret < 0){
						printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
						return ret;
					}
					camera_suspend = ret;
				}
				break;
			case HW_ID_ER:
			case HW_ID_PR:
			case HW_ID_MP:
			default:
				printk("@@@@@HW_ID is unknow:%d, use SR2 setting\n", HW_ID);
				if (camera_power_1p2_en < 0) {
				ret = camera_sensor_gpio(111, GP_CAMERA_ISP_POWER_1P2_EN,
								 GPIOF_DIR_OUT, 0);
					if (ret < 0){
						printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
						return ret;
					}
					camera_power_1p2_en = 111;
				}
				if (camera_reset < 0) {
					ret = camera_sensor_gpio(161, GP_CAMERA_ISP_RESET,
								 GPIOF_DIR_OUT, 0);
					if (ret < 0){
						printk("%s not available.\n", GP_CAMERA_ISP_RESET);
						return ret;
					}
					camera_reset = 161;
				}
				if (camera_suspend < 0) {
					ret = camera_sensor_gpio(162, GP_CAMERA_ISP_SUSPEND,
								GPIOF_DIR_OUT, 0);
					if (ret < 0){
						printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
						return ret;
					}
					camera_suspend = 162;
				}
				break;
		}
	
		printk("<<1p2_en:%d, reset:%d, suspend:%d, flag:%d\n", camera_power_1p2_en, camera_reset, camera_suspend, flag);
		if (flag){

			lnw_gpio_set_alt(GP_CORE_038, LNW_GPIO);
			lnw_gpio_set_alt(GP_CORE_039, LNW_GPIO);
			
			if (PROJECT_ID==PROJ_ID_ME372CG) {

				lnw_gpio_set_alt(GP_CORE_014, LNW_GPIO);

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

			if (camera_I2C_4_SCL < 0) {
				ret = camera_sensor_gpio(GP_CORE_039, GP_I2C_4_SCL,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_I2C_4_SCL);
					return ret;
				}
				camera_I2C_4_SCL = GP_CORE_039;
			}
			
			if (camera_I2C_4_SDA < 0) {
				ret = camera_sensor_gpio(GP_CORE_038, GP_I2C_4_SDA,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_I2C_4_SDA);
					return ret;
				}
				camera_I2C_4_SDA = GP_CORE_038;
			}		
			
			if (camera_I2C_4_SCL >= 0){
				gpio_set_value(camera_I2C_4_SCL, 0);
				printk("<<< I2C_4 SCL = 0\n");
			}
			
			if (camera_I2C_4_SDA >= 0){
				gpio_set_value(camera_I2C_4_SDA, 0);
				printk("<<< I2C_4 SDA = 0\n");
			}
			
			
			//pull low reset first
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 0);
				printk("<<< camera_reset = 0\n");
				msleep(1);
			}
	
			//turn on VCM power 2.85V
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
	
			//turn on DVDD power 1.2V
			if (camera_power_1p2_en >= 0){
				gpio_set_value(camera_power_1p2_en, 1);
				printk("<<< DVDD 1.2V = 1\n");
				msleep(1);
			}
	
			//turn on power VDD_SEN VDD_HOST 1.8V
			if (!camera_vprog1_on) {
				camera_vprog1_on = 1;
				reg_err = regulator_enable(vprog1_reg);
				if (reg_err) {
					printk(KERN_ALERT "Failed to enable regulator vprog1\n");
					return reg_err;
				}
				printk("<<< VDD_SEN VDD_HOST 1.8V = 1\n");
				msleep(10);
			}

			lnw_gpio_set_alt(GP_CORE_038, LNW_ALT_1);
			lnw_gpio_set_alt(GP_CORE_039, LNW_ALT_1);			


			if (PROJECT_ID==PROJ_ID_ME372CG) {

				//turn on power sensor AVDD 2.8V
				if (camera_sensor_2_8v >= 0){
					gpio_set_value(camera_sensor_2_8v, 1);
					printk("<<< AVDD_SENSOR 2.8V = 1\n");
					msleep(1);
				}
			}


			//turn on power ISP AVDD 2.8V
			if (!camera_vprog2_on) {
				camera_vprog2_on = 1;
				reg_err = regulator_enable(vprog2_reg);
				if (reg_err) {
					printk(KERN_ALERT "Failed to enable regulator vprog2\n");
					return reg_err;
				}
				printk("<<< AVDD_ISP 2.8V = 1\n");
				msleep(10);
			}
		

			//turn on MCLK
			ar0543_flisclk_ctrl(sd, 1);
			msleep(1); //need wait 16 clk cycle
	
	//FW_BSP++		
			//Pull low suspend to update firmware
			if (camera_suspend >= 0){
				gpio_set_value(camera_suspend, 0);
				printk("<<< suspend = 0, load fw\n");
			}

			//Reset control
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 1);
				printk("<<< reset = 1\n");
				msleep(6); //wait 6ms
			}				
	//FW_BSP--
	
			//Pull low suspend
			if (camera_suspend >= 0){
				gpio_set_value(camera_suspend, 0);
				printk("<<< suspend = 0\n");
			}
			msleep(10); //delay time for first i2c command
		}else{
			//pull low reset
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 0);
				printk("<<< reset = 0\n");
				gpio_free(camera_reset);
				camera_reset = -1;
			}
	
			//turn off MCLK
			ar0543_flisclk_ctrl(sd, 0);


			//turn off power ISP AVDD 2.8V
			if (camera_vprog2_on) {
				camera_vprog2_on = 0;
				reg_err = regulator_disable(vprog2_reg);
				if (reg_err) {
					printk(KERN_ALERT "Failed to disable regulator vprog2\n");
					return reg_err;
				}
				printk("<<< AVDD_ISP 2.8V = 0\n");
				msleep(1);
			}
			

			if (PROJECT_ID==PROJ_ID_ME372CG) {

				//turn off power sensor AVDD 2.8V
				if (camera_sensor_2_8v >= 0){
					gpio_set_value(camera_sensor_2_8v, 0);
					printk("<<< AVDD_SENSOR 2.8V = 0\n");
					gpio_free(camera_sensor_2_8v);
					camera_sensor_2_8v = -1;
					msleep(10);
				}
			}

			lnw_gpio_set_alt(GP_CORE_038, LNW_GPIO);
			lnw_gpio_set_alt(GP_CORE_039, LNW_GPIO);
		
			if (camera_I2C_4_SCL < 0) {
				ret = camera_sensor_gpio(GP_CORE_039, GP_I2C_4_SCL,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_I2C_4_SCL);
					return ret;
				}
				camera_I2C_4_SCL = GP_CORE_039;
			}

			if (camera_I2C_4_SDA < 0) {
				ret = camera_sensor_gpio(GP_CORE_038, GP_I2C_4_SDA,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_I2C_4_SDA);
					return ret;
				}
				camera_I2C_4_SDA = GP_CORE_038;
			}	


			if (camera_I2C_4_SCL >= 0){
				gpio_set_value(camera_I2C_4_SCL, 0);
				printk("<<< I2C SCL = 0\n");
				gpio_free(camera_I2C_4_SCL);
				camera_I2C_4_SCL = -1;		
				mdelay(1);
			}

			if (camera_I2C_4_SDA >= 0){
				gpio_set_value(camera_I2C_4_SDA, 0);
				printk("<<< I2C SDA = 0\n");
				gpio_free(camera_I2C_4_SDA);
				camera_I2C_4_SDA = -1;		
				mdelay(1);
			}												
	
			//turn off power VDD_SEN VDD_HOST 1.8V
			if (camera_vprog1_on) {
				camera_vprog1_on = 0;
				reg_err = regulator_disable(vprog1_reg);
				if (reg_err) {
					printk(KERN_ALERT "Failed to disable regulator vprog1\n");
					return reg_err;
				}
				printk("<<< VDD_SEN VDD_HOST 1.8V = 0\n");
				msleep(10);
			}
	
			//turn off DVDD power 1.2V
			if (camera_power_1p2_en >= 0){
				gpio_set_value(camera_power_1p2_en, 0);
				printk("<<< DVDD 1.2V = 0\n");
				gpio_free(camera_power_1p2_en);
				camera_power_1p2_en = -1;
			}
			msleep(1);
	
			//release suspend gpio
			if (camera_suspend >= 0){
				printk("<<< Release camera_suspend pin:%d\n", camera_suspend);
				gpio_free(camera_suspend);
				camera_suspend = -1;
			}
	
			//turn off VCM power 2.85V
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

	return 0;
}



static int ar0543_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int reg_err;
	int ret, SPI_ret=0;
	printk("%s: ++\n",__func__);


	if (HW_ID == 0xFF){
		HW_ID = Read_HW_ID();
	}

	if (PROJECT_ID == 0xFF) {
		PROJECT_ID = Read_PROJ_ID();
	}	
	

	if (PROJECT_ID==PROJ_ID_ME302C) {
		switch (HW_ID) {
			case HW_ID_SR1:
			case HW_ID_SR2:
			case HW_ID_ER:		
				SPI_ENABLE=0;
				break;
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}

	if (PROJECT_ID==PROJ_ID_ME372CG) {
		switch (HW_ID) {
			case HW_ID_SR1:	//for EVB
				SPI_ENABLE=0;
				break;				
			case HW_ID_SR2:	//for SR1
			case HW_ID_ER:		
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}

	if (PROJECT_ID==PROJ_ID_GEMINI) {
		switch (HW_ID) {
			case HW_ID_SR1:
				SPI_ENABLE=0;
				break;				
			case HW_ID_SR2:
			case HW_ID_ER:		
			case HW_ID_PR:
			case HW_ID_MP:
				SPI_ENABLE=1;
				break;
			default:
				SPI_ENABLE=1;
		}
	}	

	SPI_ret=spi_init_extra_parameter();

	if (SPI_ret==SPI_magic_number) {
		SPI_ENABLE=1;		
	} else if (SPI_ret==EEPROM_magic_number) {
		SPI_ENABLE=0;
	}

	//printk("HW ID:%d\n", HW_ID);
	switch (HW_ID){
		case HW_ID_SR1:
		case HW_ID_SR2:
			if (camera_power_1p2_en < 0) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_POWER_1P2_EN,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
					return ret;
				}
				camera_power_1p2_en = ret;
			}
			if (camera_reset < 0) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_RESET,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_RESET);
					return ret;
				}
				camera_reset = ret;
			}
			if (camera_suspend < 0) {
				ret = camera_sensor_gpio(-1, GP_CAMERA_ISP_SUSPEND,
							GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
					return ret;
				}
				camera_suspend = ret;
			}
			break;
		case HW_ID_ER:
		case HW_ID_PR:
		case HW_ID_MP:
		default:
			printk("@@@@@HW_ID is unknow:%d, use SR2 setting\n", HW_ID);
			if (camera_power_1p2_en < 0) {
			ret = camera_sensor_gpio(111, GP_CAMERA_ISP_POWER_1P2_EN,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_POWER_1P2_EN);
					return ret;
				}
				camera_power_1p2_en = 111;
			}
			if (camera_reset < 0) {
				ret = camera_sensor_gpio(161, GP_CAMERA_ISP_RESET,
							 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_RESET);
					return ret;
				}
				camera_reset = 161;
			}
			if (camera_suspend < 0) {
				ret = camera_sensor_gpio(162, GP_CAMERA_ISP_SUSPEND,
							GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_ISP_SUSPEND);
					return ret;
				}
				camera_suspend = 162;
			}
			break;
	}

	printk("<<1p2_en:%d, reset:%d, suspend:%d, flag:%d\n", camera_power_1p2_en, camera_reset, camera_suspend, flag);
	if (flag){
		
		lnw_gpio_set_alt(GP_CORE_038, LNW_GPIO);
		lnw_gpio_set_alt(GP_CORE_039, LNW_GPIO);
/*
		if (PROJECT_ID==PROJ_ID_ME372CG) {
			lnw_gpio_set_alt(GP_CORE_014, LNW_GPIO);

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
*/		
		if (camera_I2C_4_SCL < 0) {
			ret = camera_sensor_gpio(GP_CORE_039, GP_I2C_4_SCL,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_I2C_4_SCL);
				return ret;
			}
			camera_I2C_4_SCL = GP_CORE_039;
		}

		if (camera_I2C_4_SDA < 0) {
			ret = camera_sensor_gpio(GP_CORE_038, GP_I2C_4_SDA,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_I2C_4_SDA);
				return ret;
			}
			camera_I2C_4_SDA = GP_CORE_038;
		}		

		if (camera_I2C_4_SCL >= 0){
			gpio_set_value(camera_I2C_4_SCL, 0);
			printk("<<< I2C_4 SCL = 0\n");
			msleep(1);
		}

		if (camera_I2C_4_SDA >= 0){
			gpio_set_value(camera_I2C_4_SDA, 0);
			printk("<<< I2C_4 SDA = 0\n");
			msleep(1);
		}
		
		
		//pull low reset first
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< camera_reset = 0\n");
			msleep(1);
		}

		//turn on VCM power 2.85V
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

		//turn on DVDD power 1.2V
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 1);
			printk("<<< DVDD 1.2V = 1\n");
			msleep(1);
		}

		//turn on power VDD_SEN VDD_HOST 1.8V
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< VDD_SEN VDD_HOST 1.8V = 1\n");
			msleep(10);
		}

		lnw_gpio_set_alt(GP_CORE_038, LNW_ALT_1);
		lnw_gpio_set_alt(GP_CORE_039, LNW_ALT_1);		

/*
		if (PROJECT_ID==PROJ_ID_ME372CG) {

			//turn on power sensor AVDD 2.8V
			if (camera_sensor_2_8v >= 0){
				gpio_set_value(camera_sensor_2_8v, 1);
				printk("<<< AVDD_SENSOR 2.8V = 1\n");
				msleep(1);
			}
		}
*/

		//turn on power ISP AVDD 2.8V
		if (!camera_vprog2_on) {
			camera_vprog2_on = 1;
			reg_err = regulator_enable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to enable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD_ISP 2.8V = 1\n");
			msleep(10);
		}


		//turn on MCLK
		ar0543_flisclk_ctrl(sd, 1);
		msleep(1); //need wait 16 clk cycle

//FW_BSP++		
		if (SPI_ENABLE==0) {
			//Pull high suspend to load fw from SPI
			if (camera_suspend >= 0){
				gpio_set_value(camera_suspend, 1);
				printk("<<< suspend = 1, load fw\n");
			}

			//Reset control
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 1);
				printk("<<< reset = 1\n");
				msleep(6); //wait 6ms
			}			
		} else {
			//Pull low suspend to load fw from host
			if (camera_suspend >= 0){
				gpio_set_value(camera_suspend, 0);
				printk("<<< suspend = 0, load fw\n");
			}

			lnw_gpio_set_alt(GP_AON_019, LNW_GPIO);
			lnw_gpio_set_alt(GP_AON_021, LNW_GPIO);
			lnw_gpio_set_alt(GP_AON_023, LNW_GPIO);
				
				

			if (camera_SPI_1_SS3 < 0) {
				ret = camera_sensor_gpio(GP_AON_019, GP_CAMERA_SPI_SS3,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_SPI_SS3);
					return ret;
				}
				camera_SPI_1_SS3= GP_AON_019;
			}							



			if (camera_SPI_1_SDO < 0) {
				ret = camera_sensor_gpio(GP_AON_021, GP_CAMERA_SPI_SDO,
						 GPIOF_DIR_OUT, 0);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_SPI_SDO);
					return ret;
				}
				camera_SPI_1_SDO= GP_AON_021;
			}							
				

			if (camera_SPI_1_CLK < 0) {
			ret = camera_sensor_gpio(GP_AON_023, GP_CAMERA_SPI_CLK,
							 GPIOF_DIR_OUT, 1);
				if (ret < 0){
					printk("%s not available.\n", GP_CAMERA_SPI_CLK);
					return ret;
				}
				camera_SPI_1_CLK = GP_AON_023;
			}


			if (camera_SPI_1_SS3 >= 0){
				gpio_set_value(camera_SPI_1_SS3, 0);
				printk("<<< SPI SS3 = 0\n");
			}					


			if (camera_SPI_1_SDO >= 0){
				gpio_set_value(camera_SPI_1_SDO, 0);
				printk("<<< SPI SDO = 0\n");
			}				

			if (camera_SPI_1_CLK >= 0){
				gpio_set_value(camera_SPI_1_CLK, 1);
				printk("<<< SPI CLK = 1\n");
				msleep(6);
			}				

			//Reset control
			if (camera_reset >= 0){
				gpio_set_value(camera_reset, 1);
				printk("<<< reset = 1\n");
				msleep(6); //wait 6ms
			}

			if (camera_SPI_1_SS3 >= 0){
				gpio_free(camera_SPI_1_SS3);
				camera_SPI_1_SS3 = -1;
			}				


			if (camera_SPI_1_SDO >= 0){
				gpio_free(camera_SPI_1_SDO);
				camera_SPI_1_SDO = -1;
			}				

			if (camera_SPI_1_CLK >= 0){
				gpio_set_value(camera_SPI_1_CLK, 0);
				printk("<<< SPI CLK = 0\n");
				gpio_free(camera_SPI_1_CLK);
				camera_SPI_1_CLK = -1;
			}

			lnw_gpio_set_alt(GP_AON_019, LNW_ALT_1);
			lnw_gpio_set_alt(GP_AON_021, LNW_ALT_1);
			lnw_gpio_set_alt(GP_AON_023, LNW_ALT_1);		
		}
//FW_BSP--

		//Pull low suspend
		if (camera_suspend >= 0){
			gpio_set_value(camera_suspend, 0);
			printk("<<< suspend = 0\n");
		}
		msleep(10); //delay time for first i2c command
	}else{
		//pull low reset
		if (camera_reset >= 0){
			gpio_set_value(camera_reset, 0);
			printk("<<< reset = 0\n");
			gpio_free(camera_reset);
			camera_reset = -1;
		}

		//turn off MCLK
		ar0543_flisclk_ctrl(sd, 0);


		//turn off power ISP AVDD 2.8V
		if (camera_vprog2_on) {
			camera_vprog2_on = 0;
			reg_err = regulator_disable(vprog2_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog2\n");
				return reg_err;
			}
			printk("<<< AVDD_ISP 2.8V = 0\n");
			msleep(1);
		}


		if (PROJECT_ID==PROJ_ID_ME372CG) {

			lnw_gpio_set_alt(GP_CORE_014, LNW_GPIO);
			
			if (camera_sensor_2_8v < 0) {
				ret = camera_sensor_gpio(GP_CORE_014, GP_SENSOR_2_8V,
						 GPIOF_DIR_OUT, 1);
				if (ret < 0){
					printk("%s not available.\n", GP_SENSOR_2_8V);
					return ret;
				}
				camera_sensor_2_8v = GP_CORE_014;
			}
		

			//turn off power sensor AVDD 2.8V
			if (camera_sensor_2_8v >= 0){
				gpio_set_value(camera_sensor_2_8v, 0);
				printk("<<< AVDD_SENSOR 2.8V = 0\n");
				gpio_free(camera_sensor_2_8v);
				camera_sensor_2_8v = -1;
				msleep(10);
			}
		}
		

		lnw_gpio_set_alt(GP_AON_019, LNW_GPIO);
		lnw_gpio_set_alt(GP_AON_021, LNW_GPIO);
		lnw_gpio_set_alt(GP_CORE_038, LNW_GPIO);
		lnw_gpio_set_alt(GP_CORE_039, LNW_GPIO);
		
		if (camera_I2C_4_SCL < 0) {
			ret = camera_sensor_gpio(GP_CORE_039, GP_I2C_4_SCL,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_I2C_4_SCL);
				return ret;
			}
			camera_I2C_4_SCL = GP_CORE_039;
		}

		if (camera_I2C_4_SDA < 0) {
			ret = camera_sensor_gpio(GP_CORE_038, GP_I2C_4_SDA,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_I2C_4_SDA);
				return ret;
			}
			camera_I2C_4_SDA = GP_CORE_038;
		}				
		
		if (camera_SPI_1_SS3 < 0) {
			ret = camera_sensor_gpio(GP_AON_019, GP_CAMERA_SPI_SS3,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_CAMERA_SPI_SS3);
				return ret;
			}
			camera_SPI_1_SS3= GP_AON_019;
		}			

		if (camera_SPI_1_SDO < 0) {
			ret = camera_sensor_gpio(GP_AON_021, GP_CAMERA_SPI_SDO,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0){
				printk("%s not available.\n", GP_CAMERA_SPI_SDO);
				return ret;
			}
			camera_SPI_1_SDO= GP_AON_021;
		}			

		if (camera_SPI_1_SS3 >= 0){
			gpio_set_value(camera_SPI_1_SS3, 0);
			printk("<<< SPI SS3 = 0\n");
			gpio_free(camera_SPI_1_SS3);
			camera_SPI_1_SS3 = -1;		
			mdelay(1);
		}			

		if (camera_SPI_1_SDO >= 0){
			gpio_set_value(camera_SPI_1_SDO, 0);
			printk("<<< SPI SDO = 0\n");
			gpio_free(camera_SPI_1_SDO);
			camera_SPI_1_SDO = -1;		
			mdelay(1);
		}			


		if (camera_I2C_4_SCL >= 0){
			gpio_set_value(camera_I2C_4_SCL, 0);
			printk("<<< I2C SCL = 0\n");
			gpio_free(camera_I2C_4_SCL);
			camera_I2C_4_SCL = -1;		
			mdelay(1);
		}

		if (camera_I2C_4_SDA >= 0){
			gpio_set_value(camera_I2C_4_SDA, 0);
			printk("<<< I2C SDA = 0\n");
			gpio_free(camera_I2C_4_SDA);
			camera_I2C_4_SDA = -1;		
			mdelay(1);
		}			


		//turn off power VDD_SEN VDD_HOST 1.8V
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
				return reg_err;
			}
			printk("<<< VDD_SEN VDD_HOST 1.8V = 0\n");
			msleep(10);
		}

		//turn off DVDD power 1.2V
		if (camera_power_1p2_en >= 0){
			gpio_set_value(camera_power_1p2_en, 0);
			printk("<<< DVDD 1.2V = 0\n");
			gpio_free(camera_power_1p2_en);
			camera_power_1p2_en = -1;
		}
		msleep(1);

		//release suspend gpio
		if (camera_suspend >= 0){
			printk("<<< Release camera_suspend pin:%d\n", camera_suspend);
			gpio_free(camera_suspend);
			camera_suspend = -1;
		}

		//turn off VCM power 2.85V
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
	return 0;
}

static int ar0543_csi_configure(struct v4l2_subdev *sd, int flag)
{
	/* soc sensor, there is no raw bayer order (set to -1) */
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 4,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1,
		flag);
}

static int ar0543_platform_init(struct i2c_client *client)
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
#if 0
	//VEMMC1 for VCM, 2.85V
	vemmc1_reg = regulator_get(&client->dev, "vemmc1");
	if (IS_ERR(vemmc1_reg)) {
		dev_err(&client->dev, "vemmc1 failed\n");
		return PTR_ERR(vemmc1_reg);
	}
#endif
	return ret;
}

static int ar0543_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vprog2_reg);
	//regulator_put(vemmc1_reg);
}

static struct camera_sensor_platform_data ar0543_sensor_platform_data = {
	.gpio_ctrl	 = ar0543_gpio_ctrl,
	.flisclk_ctrl	 = ar0543_flisclk_ctrl,
	.power_ctrl	 = ar0543_power_ctrl,
	.csi_cfg	 = ar0543_csi_configure,
	.platform_init   = ar0543_platform_init,
	.platform_deinit = ar0543_platform_deinit,
};

void *ar0543_platform_data(void *info)
{
	camera_power_1p2_en = -1;
	camera_reset = -1;
	camera_suspend = -1;
	camera_SPI_1_CLK = -1;
	camera_SPI_1_SS3 = -1;
	camera_SPI_1_SDO = -1;
	camera_I2C_4_SCL = -1;
	camera_I2C_4_SDA = -1;
	camera_sensor_2_8v = -1;
	return &ar0543_sensor_platform_data;
}

