/*
 * me302c_spc_pwr_set.c: ASUS ME302C hardware power related initialization code
 *
 * (C) Copyright 2012 ASUS Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/HWVersion.h>
#include <asm/intel_scu_pmic.h>

#define PIN_3VSUS_SYNC  45
#define PIN_5VSUS_SYNC  159
#define PIN_5VSUS_EN    160
#define PIN_VSUS_SPK_SYNC 43
#define PS_MSIC_VCC330CNT            0xd3
#define PS_VCC330_OFF                0x24

#define PCB_ID0_GPIO1HV3             0x71
#define PCB_ID4_GPIO1HV0             0x74
#define PCB_ID5_GPIO1HV1             0x73
#define PCB_ID6_GPIO1HV2             0x72
#define PCB_ID7_GPIO0HV0             0x70
#define PCB_ID8_GPIO0HV1             0x6f
#define PCB_ID9_GPIO0HV2             0x6e

#define PULL_UP_2K                   0x0a

extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);
extern int Read_PCB_ID(void);

static int PROJ_ID;
static int HW_ID;
static int PCB_ID;

static int __init me302c_specific_gpio_setting_init()
{
	PROJ_ID = Read_PROJ_ID();
	HW_ID = Read_HW_ID();
	PCB_ID = Read_PCB_ID();

	gpio_request(PIN_3VSUS_SYNC, "P_+3VSO_SYNC_5");
	gpio_direction_output(PIN_3VSUS_SYNC, 0);

	// Pull up for PCB_IDs
	if(PROJ_ID == PROJ_ID_A400CG){
		bool SIM_ID = !!(PCB_ID & (1 << 6));
	  
		//set PMIC_GPIO base on PROJ_ID
		if (PROJ_ID & 1)
                        intel_scu_ipc_iowrite8(PCB_ID7_GPIO0HV0, PULL_UP_2K);
                if (PROJ_ID & 2)
                        intel_scu_ipc_iowrite8(PCB_ID8_GPIO0HV1, PULL_UP_2K);
                if (PROJ_ID & 4)
                        intel_scu_ipc_iowrite8(PCB_ID9_GPIO0HV2, PULL_UP_2K);

		//set PMIC_GPIO base on HW_ID
		if (HW_ID & 1)
			intel_scu_ipc_iowrite8(PCB_ID4_GPIO1HV0, PULL_UP_2K);
		if (HW_ID & 2)
			intel_scu_ipc_iowrite8(PCB_ID5_GPIO1HV1, PULL_UP_2K);
		if (HW_ID & 4)
			intel_scu_ipc_iowrite8(PCB_ID6_GPIO1HV2, PULL_UP_2K);
    		//set PMIC_GPIO base on SIM_SKU_ID
    		if (SIM_ID)
      			intel_scu_ipc_iowrite8(PCB_ID0_GPIO1HV3, PULL_UP_2K);
	}

	return 0;
}

module_init(me302c_specific_gpio_setting_init);
