/*
 * platform_camera.h: CAMERA platform library header file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#ifndef _PLATFORM_CAMERA_H_
#define _PLATFORM_CAMERA_H_

#include <linux/atomisp_platform.h>
#include <asm/intel-mid.h>

//Peter++
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
static unsigned int HW_ID = 0xFF;
static unsigned int PROJECT_ID = 0xFF;
static unsigned int SPI_ENABLE = 0;
extern int intel_scu_ipc_msic_vemmc1(int);
//Peter--

extern const struct intel_v4l2_subdev_id v4l2_ids[] __attribute__((weak));

#define IS_BYT (INTEL_MID_BOARD(1, PHONE, BYT) || \
	INTEL_MID_BOARD(1, TABLET, BYT))

/* MFLD iCDK camera sensor GPIOs */

//Peter++
#define GP_CAMERA_ISP_POWER_1P2_EN	"ISP_1P2_EN" //iCatch 1.2V powwer enable pin, GPIO: 111
#define GP_CAMERA_ISP_RESET		"ISP_RST_N" //iCatch reset pin, GPIO: 161
#define GP_CAMERA_ISP_SUSPEND		"ISP_SUSPEND" //iCatch suspend pin, GPIO: 162
#define GP_CAMERA_ISP_INT		"ISP_INT" //iCatch interrupt pin, GPIO: 163
#define GP_CAMERA_CAM2_1V8_EN		"CAM2_1V8_EN" //A12 8M 1.8V, GPIO: 52
#define GP_CAMERA_CAM_1P2_EN		"CAM_1P2_EN" //A12 8M 1.2V, GPIO: 111
#define GP_CAMERA_MAIN_CAM_PWDN		"MAIN_CAM_PWDN" //A12 8M shutdown, GPIO: 161
#define GP_CAMERA_SUB_CAM_PWDN		"SUB_CAM_PWDN" //A12 2M shutdown, GPIO: 160
#if 1
/* Obsolete pin, maybe used by old MFLD iCDK */
#define GP_CAMERA_0_POWER_DOWN          "cam0_vcm_2p8"
/* Camera VDD 1.8V Switch */
#define GP_CAMERA_1P8			"camera_on_n"
/* Camera0 Standby pin control */
#define GP_CAMERA_0_STANDBY		"camera_0_power"
#define GP_CAMERA_1_POWER_DOWN          "camera_1_power"
#define GP_CAMERA_0_RESET               "camera_0_reset"
#define GP_CAMERA_1_RESET               "camera_1_reset"
#endif
//Peter--
//Patrick++
#define GP_CAMERA_SPI_CLK "SPI_CLK"
#define GP_CAMERA_SPI_SS3 "SPI_SS3"
#define GP_CAMERA_SPI_SDO "SPI_SDO"
#define GP_I2C_4_SCL "I2C_4_SCL"
#define GP_I2C_4_SDA "I2C_4_SDA"
#define GP_SENSOR_2_8V "SENSOR_2_8V"
#define GP_AON_019	19
#define GP_AON_021	21
#define GP_AON_023	23
#define GP_AON_052 52 //A12 8M 1.8V
#define GP_CORE_014 110
#define GP_CORE_015 111 //A12 8M 1.2V
#define GP_CORE_038 134
#define GP_CORE_039 135
#define GP_CORE_064 160 //A12 2M shutdown
#define GP_CORE_065 161 //A12 8M shutdown
#define GP_CORE_080 176 //A12 8M shutdown
//Patrick--

//Yushan++
#define GP_CAMERA_1_RESET		"CAM_1_RST_N"
#define GP_CAMERA_2_8V 			"CAM_2P8_EN"
#define GP_CAMERA_1_PDWN		"SUB_CAM_PWDN"
//Yushan--

extern int camera_sensor_gpio(int gpio, char *name, int dir, int value);
extern int camera_sensor_csi(struct v4l2_subdev *sd, u32 port,
			u32 lanes, u32 format, u32 bayer_order, int flag);
extern void intel_register_i2c_camera_device(
				struct sfi_device_table_entry *pentry,
				struct devs_id *dev)
				__attribute__((weak));

/*
 * FIXME! This PMIC power access workaround for BTY
 * since currently no VRF for BTY
 */
#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#define VPROG_ENABLE 0x3
#define VPROG_DISABLE 0x2

enum camera_pmic_pin {
	CAMERA_1P8V,
	CAMERA_2P8V,
	CAMERA_POWER_NUM,
};

struct vprog_status {
	unsigned int user;
};

int camera_set_pmic_power(enum camera_pmic_pin pin, bool flag);
#endif
#endif
