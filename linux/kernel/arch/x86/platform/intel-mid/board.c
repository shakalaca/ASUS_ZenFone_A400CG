/*
 * board-blackbay.c: Intel Medfield based board (Blackbay)
 *
 * (C) Copyright 2008 Intel Corporation
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
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/intel_pmic_gpio.h>
#include <linux/spi/spi.h>
#include <linux/i2c.h>
#include <linux/i2c/pca953x.h>
#include <linux/gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_msic.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/notifier.h>
#include <linux/i2c-gpio.h>

#include <asm/setup.h>
#include <asm/mpspec_def.h>
#include <asm/hw_irq.h>
#include <asm/apic.h>
#include <asm/io_apic.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_vrtc.h>
#include <linux/io.h>
#include <asm/i8259.h>
#include <asm/intel_scu_ipc.h>
#include <asm/apb_timer.h>
#include <linux/reboot.h>

#include "board.h"

/*
 * IPC devices
 */
#include "device_libs/platform_ipc.h"
#include "device_libs/platform_pmic_gpio.h"
#include "device_libs/platform_msic.h"
#include "device_libs/platform_msic_battery.h"
#include "device_libs/platform_msic_gpio.h"
#include "device_libs/platform_msic_audio.h"
#include "device_libs/platform_msic_power_btn.h"
#include "device_libs/platform_msic_ocd.h"
#include "device_libs/platform_msic_vdd.h"
#include "device_libs/platform_mrfl_pmic.h"
#include "device_libs/platform_mrfl_pmic_i2c.h"
#include "device_libs/platform_mrfl_ocd.h"
#include "device_libs/platform_msic_thermal.h"
#include "device_libs/platform_soc_thermal.h"
#include "device_libs/platform_msic_adc.h"
#include "device_libs/platform_bcove_adc.h"
#include <asm/platform_mrfld_audio.h>
#include <asm/platform_ctp_audio.h>
#include "device_libs/platform_mrfl_thermal.h"
#include "device_libs/platform_scu_log.h"

/*
 * I2C devices
 */
#include "device_libs/platform_max7315.h"
#include "device_libs/platform_tca6416.h"
#include "device_libs/platform_emc1403.h"
#include "device_libs/platform_lis331.h"
#include "device_libs/platform_tc35876x.h"
#include "device_libs/platform_rmi4.h"
#include "device_libs/platform_bq24192.h"
#include "device_libs/platform_bq24261.h"
#include "device_libs/platform_r69001.h"
#include "device_libs/platform_pn544.h"
#include "device_libs/platform_l3g4200d.h"
#include "device_libs/platform_lsm303.h"
#include "device_libs/platform_a1026.h"

/* SW devices */
#include "device_libs/platform_panel.h"

#include "device_libs/platform_wm8994.h"
#include <asm/platform_cs42l73.h>

//Peter+++
#include "device_libs/platform_camera.h"
/*
#include "device_libs/platform_mt9e013.h"
#include "device_libs/platform_mt9d113.h"
*/
#include "device_libs/platform_mt9m114.h"
/*
#include "device_libs/platform_lm3554.h"
#include "device_libs/platform_mt9v113.h"
#include "device_libs/platform_ov5640.h"
#include "device_libs/platform_imx175.h"
#include "device_libs/platform_imx135.h"
#include "device_libs/platform_imx134.h"
#include "device_libs/platform_imx132.h"
#include "device_libs/platform_s5k8aay.h"
#include "device_libs/platform_ov9724.h"
#include "device_libs/platform_ov2722.h"
#include "device_libs/platform_lm3559.h"
#include "device_libs/platform_ov8830.h"
*/
// added by cheng_kao 2013.09.26 ++
#include "device_libs/platform_gyro.h"
#include "device_libs/platform_accel.h"
#include "device_libs/platform_compass.h"
#include "device_libs/platform_al3320a.h"
#include "device_libs/platform_ap3212c.h"
#include "device_libs/platform_cap1106.h"
#include "device_libs/platform_akm09911.h"
// added by cheng_kao 2013.09.26 --

// added by cheng_kao 2013.10.23 ++
#include "device_libs/platform_px3212c.h"
#include "device_libs/platform_px3003b.h"
// added by cheng_kao 2013.10.23 --

#include "device_libs/platform_wm5102.h"
#include "device_libs/platform_ov5693.h"
#include "device_libs/platform_ar0543.h"
#include "device_libs/platform_ar0543_raw.h"
#include "device_libs/platform_hm2056_raw.h"
#include "device_libs/platform_imx111_raw.h"
#include "device_libs/platform_mt9m114_raw.h"
#include "device_libs/platform_gc0339_raw.h"
#include "device_libs/platform_imx219.h"
//Peter---

//Joe add for ASUS EC driver ++
#ifdef CONFIG_HID_ASUS_PAD_EC
#include "device_libs/platform_asus_ec.h"
#endif
//Joe add for ASUS EC driver --
//Joe add for Elan touchpad driver ++
#ifdef CONFIG_MOUSE_ELAN_TOUCHPAD
#include "device_libs/platform_elan_touchpad.h"
#endif
//Joe add for Elan touchpad driver --

/*
 * SPI devices
 */
#include "device_libs/platform_max3111.h"
#include "device_libs/platform_max17042.h"
//Patrick++
#include "device_libs/platform_spca700xa.h"
//Patrick--

/* HSI devices */
#include "device_libs/platform_hsi_modem.h"
#include "device_libs/platform_ffl_modem.h"
#include "device_libs/platform_edlp_modem.h"
#include "device_libs/platform_edlp_fast.h"
#include "device_libs/platform_logical_modem.h"

/* SW devices */
#include "device_libs/platform_modem_ctrl.h"

/* WIFI devices */
#include "device_libs/platform_wl12xx.h"
#include "device_libs/platform_wifi.h"

#ifdef CONFIG_TOUCHSCREEN_ELAN_EKTH3374
#include "device_libs/platform_ektf3k.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_GOODIX_GT927
#include "device_libs/platform_gt927.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_A12
#include "device_libs/platform_hx8528_a12.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_P72G
#include "device_libs/platform_hx8528_p72g.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_ME372CL
#include "device_libs/platform_hx8528_me372cl.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528
#include "device_libs/platform_hx8528.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_FE380CG
#include "device_libs/platform_hx8528_fe380cg.h"
#endif

#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_A400
#include "device_libs/platform_hx8528_a400cg.h"
#endif

//chris ((( Battery driver
#ifdef CONFIG_ME372CG_BATTERY_SMB345
	#include "device_libs/platform_me372cg_smb345.h"
#endif
#if defined(CONFIG_ME302C_BATTERY_BQ27520) || defined(CONFIG_ME372CG_BATTERY_BQ27520)
	#include "device_libs/platform_bq27520.h"
#endif

// Jui-Chuan add for Atmel touchscreen ++
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT1664S 
#include "device_libs/platform_mxt1664s.h"
#endif
// Jui-Chuan add for Atmel touchscreen --

// )))
static void __init *no_platform_data(void *info)
{
	return NULL;
}

#ifdef CONFIG_ME372CL
struct devs_id __initconst device_ids_me372cl[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
	{"akm09911", SFI_DEV_TYPE_I2C, 0, &akm09911_platform_data, NULL},
	{"kxtj9", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
	{"ap3212c", SFI_DEV_TYPE_I2C, 0, &ap3212c_platform_data, NULL},
	{"cap1106", SFI_DEV_TYPE_I2C, 0, &cap1106_platform_data, NULL},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
//Josh add for himax HX8528-D48 touch (ME372CL) ++
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_ME372CL
	{"hx8528", SFI_DEV_TYPE_I2C, 0, &hx8528_platform_data},
#endif
//Josh add for himax HX8528-D48 touch (ME372CL) --
// chris ((( Battery driver
#ifdef CONFIG_ME372CG_BATTERY_SMB345
	{"smb345", SFI_DEV_TYPE_I2C, 0, &smb345_platform_data},
#endif
#ifdef CONFIG_ME372CG_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data},
#endif
// )))
	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"scove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"mt9m114_raw", SFI_DEV_TYPE_I2C, 0, &mt9m114_raw_platform_data,
					&intel_register_i2c_camera_device},
	{"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
					&intel_register_i2c_camera_device},
//Peter--
/*	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},*/
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
	{"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	/*
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	*/
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif


#ifdef CONFIG_A400CG
struct devs_id __initconst device_ids_a400cg[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//added by cheng_kao 2013.09.26 ++
	{"kxtj2", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
//added by cheng_kao 2013.09.26 --
	{"px3003b", SFI_DEV_TYPE_I2C, 0, &px3003b_platform_data, NULL},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_A400
	{"hx8528", SFI_DEV_TYPE_I2C, 0, &hx8528_platform_data},
#endif

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
                                        &intel_register_i2c_camera_device},
	{"hm2056_raw", SFI_DEV_TYPE_I2C, 0, &hm2056_raw_platform_data,
                                        &intel_register_i2c_camera_device},
	{"gc0339_raw", SFI_DEV_TYPE_I2C, 0, &gc0339_raw_platform_data,
                                        &intel_register_i2c_camera_device},
//Peter--
/*	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},*/
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*								
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif


#ifdef CONFIG_TX201LA
struct devs_id __initconst device_ids_tx201la[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
//Patrick++
	{"spca700xa", SFI_DEV_TYPE_SPI, 0, &spca700xa_platform_data, NULL},
//Patrick--
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//modified by cheng_kao 2013.09.30 ++
	{"mpu6500", SFI_DEV_TYPE_I2C, 0, &gyro_platform_data, NULL},
	{"ak8963", SFI_DEV_TYPE_I2C, 0, &compass_platform_data, NULL},
//modified by cheng_kao 2013.09.30 --
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"ov5693", SFI_DEV_TYPE_I2C, 0, &ov5693_platform_data,
					&intel_register_i2c_camera_device},
//Peter--
//	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*								
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif


#ifdef CONFIG_ME302C
struct devs_id __initconst device_ids_me302c[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
//Patrick++
	{"spca700xa", SFI_DEV_TYPE_SPI, 0, &spca700xa_platform_data, NULL},
//Patrick--
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//added by cheng_kao 2013.09.26 ++
	{"mpu6500", SFI_DEV_TYPE_I2C, 0, &gyro_platform_data, NULL},
	{"ak8963", SFI_DEV_TYPE_I2C, 0, &compass_platform_data, NULL},
	{"al3320a", SFI_DEV_TYPE_I2C, 0, &al3320a_platform_data, NULL},
//added by cheng_kao 2013.09.26 --
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
*/
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
/*
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"ov5693", SFI_DEV_TYPE_I2C, 0, &ov5693_platform_data,
					&intel_register_i2c_camera_device},
//Peter--
//	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*								
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
// add by leo for ELAN touch ++
#ifdef CONFIG_TOUCHSCREEN_ELAN_EKTH3374
	{"ekth3374", SFI_DEV_TYPE_I2C, 0, &ektf3k_platform_data, NULL},
#endif
// add by leo for ELAN touch --
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif


#ifdef CONFIG_ME372CG
struct devs_id __initconst device_ids_me372cg[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
//Patrick++
	{"spca700xa", SFI_DEV_TYPE_SPI, 0, &spca700xa_platform_data, NULL},
//Patrick--
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//added by cheng_kao 2013.09.26 ++
	{"akm09911", SFI_DEV_TYPE_I2C, 0, &akm09911_platform_data, NULL},
	{"kxtj9", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
//added by cheng_kao 2013.09.26 --
	{"ap3212c", SFI_DEV_TYPE_I2C, 0, &ap3212c_platform_data, NULL},
	{"cap1106", SFI_DEV_TYPE_I2C, 0, &cap1106_platform_data, NULL},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
//Josh add for himax HX8528-C58 touch (ME372CG) ++
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528
	{"hx8528", SFI_DEV_TYPE_I2C, 0, &hx8528_platform_data},
#endif
//Josh add for himax HX8528-C58 touch (ME372CG) --
// chris ((( Battery driver
#ifdef CONFIG_ME372CG_BATTERY_SMB345
	{"smb345", SFI_DEV_TYPE_I2C, 0, &smb345_platform_data},
#endif
#ifdef CONFIG_ME372CG_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data},
#endif
// )))
	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
*/
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114_raw", SFI_DEV_TYPE_I2C, 0, &mt9m114_raw_platform_data,
					&intel_register_i2c_camera_device},
/*
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"ar0543", SFI_DEV_TYPE_I2C, 0, &ar0543_platform_data,
					&intel_register_i2c_camera_device},
	{"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
					&intel_register_i2c_camera_device},
//Peter--
//	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*								
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif

#ifdef CONFIG_PF400CG
struct devs_id __initconst device_ids_pf400cg[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//added by cheng_kao 2013.09.26 ++
	{"akm09911", SFI_DEV_TYPE_I2C, 0, &akm09911_platform_data, NULL},
	{"kxtj2", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
	{"px3212c", SFI_DEV_TYPE_I2C, 0, &px3212c_platform_data, NULL},
	{"px3003b", SFI_DEV_TYPE_I2C, 0, &px3003b_platform_data, NULL},
//added by cheng_kao 2013.09.26 --
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
// chris ((( Battery driver
#ifdef CONFIG_ME372CG_BATTERY_SMB345
	{"smb346", SFI_DEV_TYPE_I2C, 0, &smb345_platform_data},
#endif
#ifdef CONFIG_ME372CG_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data},
#endif
// )))
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
//Joe add for himax HX8528-D48 touch (A12) ++
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_A12
	{"hx8528", SFI_DEV_TYPE_I2C, 0, &hx8528_platform_data},
#endif
//Joe add for himax HX8528-D48 touch (A12) --
//add by leo for himax HX8528-D48 touch (P72G) ++
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_P72G
	{"hx8528_p72g", SFI_DEV_TYPE_I2C, 0, &hx8528_p72g_platform_data},
#endif
//add by leo for himax HX8528-D48 touch (P72G) --
	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

//	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
//						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"imx111_raw", SFI_DEV_TYPE_I2C, 0, &imx111_raw_platform_data,
                                        &intel_register_i2c_camera_device},
	{"hm2056_raw", SFI_DEV_TYPE_I2C, 0, &hm2056_raw_platform_data,
                                        &intel_register_i2c_camera_device},
//Peter--
//	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif


#ifdef CONFIG_ME175CG
struct devs_id __initconst device_ids_me175cg[] = {
	/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//added by cheng_kao 2013.09.26 ++
	{"akm09911", SFI_DEV_TYPE_I2C, 0, &akm09911_platform_data, NULL},
	{"kxtj2", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
	{"px3003b", SFI_DEV_TYPE_I2C, 0, &px3003b_platform_data, NULL},
//added by cheng_kao 2013.09.26 --
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
	{"gt927", SFI_DEV_TYPE_I2C, 0, &gt927_platform_data, NULL},
// chris ((( Battery driver
#ifdef CONFIG_ME372CG_BATTERY_SMB345
	{"smb345", SFI_DEV_TYPE_I2C, 0, &smb345_platform_data},
#endif
#ifdef CONFIG_ME372CG_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data},
#endif
// )))

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
                                        &intel_register_i2c_camera_device},
	{"mt9m114_raw", SFI_DEV_TYPE_I2C, 0, &mt9m114_raw_platform_data,
                                        &intel_register_i2c_camera_device},
//Peter--
//	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//						NULL},
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},	
/*	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif

#ifdef CONFIG_FE380CG
struct devs_id __initconst device_ids_fe380cg[] = {
        /* SD devices */
        {"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
        {"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
        {"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
        {"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

        /* SPI devices */
        {"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
        {"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
        {"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
                                        &ipc_device_handler},
        {"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
        {"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
        {"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
        {"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
        {"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
        {"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
                                        &ipc_device_handler},
        {"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
        {"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
        {"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
        {"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
        {"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

        /* I2C devices*/
//added by cheng_kao 2013.09.26 ++
        {"akm09911", SFI_DEV_TYPE_I2C, 0, &akm09911_platform_data, NULL},
        {"kxtj2", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
        {"ap3212c", SFI_DEV_TYPE_I2C, 0, &ap3212c_platform_data, NULL},
        {"cap1106", SFI_DEV_TYPE_I2C, 0, &cap1106_platform_data, NULL},
//added by cheng_kao 2013.09.26 --
        {"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
        {"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
        {"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
        {"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
        {"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
        {"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
        {"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
        {"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
        {"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
        {"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
        {"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
        
//Josh add for himax HX8528-E58 touch (FE380CG) ++
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_FE380CG

#endif
//Josh add for himax HX8528-E58 touch (FE380CG) --

        /* MSIC subdevices */
        {"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
                                                &ipc_device_handler},
        {"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
                                        &ipc_device_handler},
        {"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
                                        &msic_power_btn_platform_data,
                                        &ipc_device_handler},
        {"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
                                        &ipc_device_handler},
        {"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
                                        &ipc_device_handler},
        {"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
                                        &ipc_device_handler},
        {"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
                                        &ipc_device_handler},
        {"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
                                        &ipc_device_handler},
        {"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
                                        &ipc_device_handler},
        {"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
                                        &ipc_device_handler},
        {"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
                                        &ipc_device_handler},
        {"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
                                        &ipc_device_handler},
        {"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
                                        &ipc_device_handler},
        {"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
                                        &ipc_device_handler},

        /* Panel */
        {"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        /*  above 3 items will be removed
        * after firmware changing
        */
        {"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},

        {"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},

        /* I2C devices for camera image subsystem */
//Peter++
/*
        {"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
                                        &intel_register_i2c_camera_device},
        {"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
                                        &intel_register_i2c_camera_device},
        {"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
                                        &intel_register_i2c_camera_device},
        {"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
                                        &intel_register_i2c_camera_device},
        {"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
                                        &intel_register_i2c_camera_device},
        {"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
                                        &intel_register_i2c_camera_device},
        {"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
                                        &intel_register_i2c_camera_device},
*/
        {"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
                                        &intel_register_i2c_camera_device},
        {"hm2056_raw", SFI_DEV_TYPE_I2C, 0, &hm2056_raw_platform_data,
                                        &intel_register_i2c_camera_device},
//Peter--
//      {"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//                                                NULL},
        {"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

        /* IPC devices */
        {"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
        {"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
                                                &ipc_device_handler},
        {"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
                                                &ipc_device_handler},

        /* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*      {"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},
        {"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},
        {"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},
*/      {"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
                                                &ipc_device_handler},
        {"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
                                                &ipc_device_handler},
        {"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
                                                &soc_thrm_device_handler},
        {"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
        {"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
        {"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
        {"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
        {"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
        {"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
        {"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
                &sfi_handle_edlp_dev},
        {"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
                &sfi_handle_edlp_fast_dev},
#endif
        {"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        /* scalability V2 configurations */
        /* for 7160 modem*/
        {"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        /* for 7260 modem */
        {"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {},
};
#endif

#ifdef CONFIG_A450CG
struct devs_id __initconst device_ids_a450cg[] = {
/* SD devices */
	{"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
	{"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
	{"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

	/* SPI devices */
	{"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
	{"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
					&ipc_device_handler},
	{"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
	{"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
	{"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
	{"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
	{"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
					&ipc_device_handler},
	{"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
	{"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
	{"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
	{"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

	/* I2C devices*/
//added by cheng_kao 2014.02.27 ++
	{"kxtj2", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
    {"akm09911", SFI_DEV_TYPE_I2C, 0, &akm09911_platform_data, NULL},
//added by cheng_kao 2014.02.27 --
	{"px3003b", SFI_DEV_TYPE_I2C, 0, &px3003b_platform_data, NULL},
	{"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
	{"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
	{"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
	{"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
	{"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
	{"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
	{"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
	{"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
	{"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},
#ifdef CONFIG_TOUCHSCREEN_HIMAX_HX8528_A400
	{"hx8528", SFI_DEV_TYPE_I2C, 0, &hx8528_platform_data},
#endif
#ifdef CONFIG_ME372CG_BATTERY_BQ27520
	{"bq27520", SFI_DEV_TYPE_I2C, 0, &bq27520_platform_data},
#endif

	/* MSIC subdevices */
	{"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
						&ipc_device_handler},
	{"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
					&msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
					&ipc_device_handler},
	{"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
					&ipc_device_handler},
	{"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
					&ipc_device_handler},
	{"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
					&ipc_device_handler},
	{"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
					&ipc_device_handler},
	{"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
					&ipc_device_handler},
	{"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
					&ipc_device_handler},
	{"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
					&ipc_device_handler},
	{"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
					&ipc_device_handler},
	{"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
					&ipc_device_handler},
	{"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
					&ipc_device_handler},

	/* Panel */
	{"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	/*  above 3 items will be removed
	* after firmware changing
	*/
	{"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},
	{"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
		&panel_handler},

	{"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},

	/* I2C devices for camera image subsystem */
//Peter++
/*
	{"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
					&intel_register_i2c_camera_device},
	{"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
					&intel_register_i2c_camera_device},
	{"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
					&intel_register_i2c_camera_device},
	{"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
					&intel_register_i2c_camera_device},
	{"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
					&intel_register_i2c_camera_device},
	{"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
					&intel_register_i2c_camera_device},
	{"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
					&intel_register_i2c_camera_device},
	{"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
					&intel_register_i2c_camera_device},
	{"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
					&intel_register_i2c_camera_device},
	{"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
					&intel_register_i2c_camera_device},
	{"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
					&intel_register_i2c_camera_device},
	{"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
					&intel_register_i2c_camera_device},
	{"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
					&intel_register_i2c_camera_device},
*/
	{"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
                                        &intel_register_i2c_camera_device},
	{"gc0339_raw", SFI_DEV_TYPE_I2C, 0, &gc0339_raw_platform_data,
                                        &intel_register_i2c_camera_device},
	{"imx219", SFI_DEV_TYPE_I2C, 0, &imx219_platform_data,
                                        &intel_register_i2c_camera_device},
//Peter--
/*	{"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
						NULL},*/
	{"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

	/* IPC devices */
	{"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
	{"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
						&ipc_device_handler},
	{"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
						&ipc_device_handler},

	/* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
	                                                &ipc_device_handler},
/*								
	{"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
	{"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
						&ipc_device_handler},
*/
	{"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
						&ipc_device_handler},
	{"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
						&ipc_device_handler},
	{"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
						&soc_thrm_device_handler},
	{"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
	{"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
	{"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
	{"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
#ifdef CONFIG_HSI
	{"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
	{"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
	{"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
		&sfi_handle_edlp_dev},
	{"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
		&sfi_handle_edlp_fast_dev},
#endif
	{"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* scalability V2 configurations */
	/* for 7160 modem*/
	{"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	/* for 7260 modem */
	{"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
		&sfi_handle_mdm},
	{},
};
#endif

#ifdef CONFIG_TX201LAF
struct devs_id __initconst device_ids_tx201laf[] = {

        /* SD devices */
        {"wl12xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &wl12xx_platform_data, NULL},
        {"bcm43xx_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
        {"bcm43xx_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},
        {"iwlwifi_clk_vmmc", SFI_DEV_TYPE_SD, 0, &bcm43xx_platform_data, NULL},

        /* SPI devices */
        {"bma023", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
        {"pmic_gpio", SFI_DEV_TYPE_SPI, 1, &pmic_gpio_platform_data, NULL},
        {"pmic_gpio", SFI_DEV_TYPE_IPC, 1, &pmic_gpio_platform_data,
                                        &ipc_device_handler},
        {"spi_max3111", SFI_DEV_TYPE_SPI, 0, &max3111_platform_data, NULL},
        {"i2c_max7315", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
        {"i2c_max7315_2", SFI_DEV_TYPE_I2C, 1, &max7315_platform_data, NULL},
        {"tca6416", SFI_DEV_TYPE_I2C, 1, &tca6416_platform_data, NULL},
        {"emc1403", SFI_DEV_TYPE_I2C, 1, &emc1403_platform_data, NULL},
        {"pmic_audio", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
                                        &ipc_device_handler},
        {"i2c_disp_brig", SFI_DEV_TYPE_I2C, 0, &tc35876x_platform_data, NULL},
        {"r69001-ts-i2c", SFI_DEV_TYPE_I2C, 0, &r69001_platform_data, NULL},
        {"synaptics_3202", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
        {"syn_3400_cgs", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},
        {"syn_3400_igzo", SFI_DEV_TYPE_I2C, 0, &rmi4_platform_data, NULL},

        /* I2C devices*/
//modified by cheng_kao 2014.03.12 ++
		{"kxtj2", SFI_DEV_TYPE_I2C, 0, &accel_platform_data, NULL},
//modified by cheng_kao 2014.03.12 --
        {"max17042", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
        {"max17047", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
        {"max17050", SFI_DEV_TYPE_I2C, 1, &max17042_platform_data, NULL},
        {"bq24192", SFI_DEV_TYPE_I2C, 1, &bq24192_platform_data},
        {"bq24261_charger", SFI_DEV_TYPE_I2C, 1, &bq24261_platform_data, NULL},
        {"pn544", SFI_DEV_TYPE_I2C, 0, &pn544_platform_data, NULL},
        {"l3gd20", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data, NULL},
        {"l3g4200d", SFI_DEV_TYPE_I2C, 0, &l3g4200d_platform_data},
        {"lps331ap", SFI_DEV_TYPE_I2C, 0, &no_platform_data},
        {"lsm303dl", SFI_DEV_TYPE_I2C, 0, &lsm303dlhc_accel_platform_data},
        {"lsm303cmp", SFI_DEV_TYPE_I2C, 0, &no_platform_data, NULL},

        /* MSIC subdevices */
        {"msic_adc", SFI_DEV_TYPE_IPC, 1, &msic_adc_platform_data,
                                                &ipc_device_handler},
        {"bcove_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
                                        &ipc_device_handler},
        {"shadycove_power_btn", SFI_DEV_TYPE_IPC, 1,
                                        &msic_power_btn_platform_data,
                                        &ipc_device_handler},
        {"msic_battery", SFI_DEV_TYPE_IPC, 1, &msic_battery_platform_data,
                                        &ipc_device_handler},
        {"msic_gpio", SFI_DEV_TYPE_IPC, 1, &msic_gpio_platform_data,
                                        &ipc_device_handler},
        {"msic_audio", SFI_DEV_TYPE_IPC, 1, &msic_audio_platform_data,
                                        &ipc_device_handler},
        {"msic_power_btn", SFI_DEV_TYPE_IPC, 1, &msic_power_btn_platform_data,
                                        &ipc_device_handler},
        {"msic_vdd", SFI_DEV_TYPE_IPC, 1, &msic_vdd_platform_data,
                                        &ipc_device_handler},
        {"msic_ocd", SFI_DEV_TYPE_IPC, 1, &msic_ocd_platform_data,
                                        &ipc_device_handler},
        {"bcove_bcu", SFI_DEV_TYPE_IPC, 1, &mrfl_ocd_platform_data,
                                        &ipc_device_handler},
        {"msic_thermal", SFI_DEV_TYPE_IPC, 1, &msic_thermal_platform_data,
                                        &ipc_device_handler},
        {"bcove_adc", SFI_DEV_TYPE_IPC, 1, &bcove_adc_platform_data,
                                        &ipc_device_handler},
        {"bcove_thrm", SFI_DEV_TYPE_IPC, 1, &mrfl_thermal_platform_data,
                                        &ipc_device_handler},
        {"scuLog", SFI_DEV_TYPE_IPC, 1, &scu_log_platform_data,
                                        &ipc_device_handler},

        /* Panel */
        {"PANEL_CMI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PANEL_JDI_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PANEL_JDI_CMD", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        /*  above 3 items will be removed
        * after firmware changing
        */
        {"PNC_CMI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNV_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNC_JDI_7x12", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNC_SHARP_10x19", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNV_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"PNC_SHARP_25x16", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},
        {"JDI_25x16_VID", SFI_DEV_TYPE_MDM, 0, &no_platform_data,
                &panel_handler},

        {"ctp_lt_wm8994", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},

        /* I2C devices for camera image subsystem */
//Joe add for ASUS EC driver ++
#if CONFIG_HID_ASUS_PAD_EC
        {"ITE8566", SFI_DEV_TYPE_I2C, 0, &asus_ec_platform_data, NULL},
#endif
//Joe add for ASUS EC driver --
//Joe add for Elan touchpad driver ++
#if CONFIG_MOUSE_ELAN_TOUCHPAD
        {"T200", SFI_DEV_TYPE_I2C, 0, &elan_touchpad_platform_data, NULL},
#endif
//Joe add for Elan touchpad driver --
//Peter++
/*
        {"lm3554", SFI_DEV_TYPE_I2C, 0, &lm3554_platform_data_func,
                                        &intel_register_i2c_camera_device},
        {"mt9e013", SFI_DEV_TYPE_I2C, 0, &mt9e013_platform_data,
                                        &intel_register_i2c_camera_device},
        {"mt9d113", SFI_DEV_TYPE_I2C, 0, &mt9d113_platform_data,
                                        &intel_register_i2c_camera_device},
        {"mt9m114", SFI_DEV_TYPE_I2C, 0, &mt9m114_platform_data,
                                        &intel_register_i2c_camera_device},
        {"mt9v113", SFI_DEV_TYPE_I2C, 0, &mt9v113_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov8830", SFI_DEV_TYPE_I2C, 0, &ov8830_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov5640", SFI_DEV_TYPE_I2C, 0, &ov5640_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx175", SFI_DEV_TYPE_I2C, 0, &imx175_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx135", SFI_DEV_TYPE_I2C, 0, &imx135_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx134", SFI_DEV_TYPE_I2C, 0, &imx134_platform_data,
                                        &intel_register_i2c_camera_device},
        {"imx132", SFI_DEV_TYPE_I2C, 0, &imx132_platform_data,
                                        &intel_register_i2c_camera_device},
        {"s5k8aay", SFI_DEV_TYPE_I2C, 0, &s5k8aay_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov9724", SFI_DEV_TYPE_I2C, 0, &ov9724_platform_data,
                                        &intel_register_i2c_camera_device},
        {"ov2722", SFI_DEV_TYPE_I2C, 0, &ov2722_platform_data,
                                        &intel_register_i2c_camera_device},
        {"lm3559", SFI_DEV_TYPE_I2C, 0, &lm3559_platform_data_func,
                                        &intel_register_i2c_camera_device},
*/
        {"ar0543_raw", SFI_DEV_TYPE_I2C, 0, &ar0543_raw_platform_data,
                                        &intel_register_i2c_camera_device},
//Peter--
//      {"audience_es305", SFI_DEV_TYPE_I2C, 0, &audience_platform_data,
//                                              NULL},
        {"wm8994", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},

        /* IPC devices */
        {"pmic_charger", SFI_DEV_TYPE_IPC, 1, &no_platform_data, NULL},
        {"pmic_ccsm", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_ccsm_platform_data,
                                                &ipc_device_handler},
        {"i2c_pmic_adap", SFI_DEV_TYPE_IPC, 1, &mrfl_pmic_i2c_platform_data,
                                                &ipc_device_handler},

        /* IPC devices */
        {"ctp_audio", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                        &ipc_device_handler},
/*
        {"ctp_rhb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},
        {"ctp_vb_cs42l73", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},
        {"ctp_ht_wm5102", SFI_DEV_TYPE_IPC, 1, &ctp_audio_platform_data,
                                                &ipc_device_handler},
*/
        {"mrfld_lm49453", SFI_DEV_TYPE_IPC, 1, &merfld_audio_platform_data,
                                                &ipc_device_handler},
        {"mrfld_wm8958", SFI_DEV_TYPE_IPC, 1, &merfld_wm8958_audio_platform_data,
                                                &ipc_device_handler},
        {"soc_thrm", SFI_DEV_TYPE_IPC, 1, &no_platform_data,
                                                &soc_thrm_device_handler},
        {"wm8958", SFI_DEV_TYPE_I2C, 0, &wm8994_platform_data, NULL},
        {"wm5102", SFI_DEV_TYPE_I2C, 0, &wm5102_platform_data, NULL},
        {"lm49453_codec", SFI_DEV_TYPE_I2C, 1, &no_platform_data, NULL},
        {"cs42l73", SFI_DEV_TYPE_I2C, 1, &cs42l73_platform_data, NULL},
//Jui-Chuan add for Atmel touchscreen ++
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT1664S
        {"mxt1664s", SFI_DEV_TYPE_I2C, 0, &mxt1664s_platform_data, NULL},
#endif
//Jui-Chuan add for Atmel touchscreen --

#ifdef CONFIG_HSI
        {"hsi_ifx_modem", SFI_DEV_TYPE_HSI, 0, &hsi_modem_platform_data, NULL},
        {"hsi_ffl_modem", SFI_DEV_TYPE_HSI, 0, &ffl_modem_platform_data, NULL},
        {"hsi_edlp_modem", SFI_DEV_TYPE_HSI, 0, &edlp_modem_platform_data,
                &sfi_handle_edlp_dev},
        {"hsi_edlp_fast", SFI_DEV_TYPE_HSI, 0, &edlp_fast_platform_data,
                &sfi_handle_edlp_fast_dev},
#endif
        {"XMM_6260", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_6268", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_6360", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV3_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7160_REV4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM_7260_REV1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"RMC_CYGNUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"CYGNUS_FFRD_EU", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"CYGNUS_FFRD_NA", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"RMC_CYGNUS_PCI", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"RMC_PEGASUS", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        /* scalability V2 configurations */
        /* for 7160 modem*/
        {"XMM7160_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7160_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7160_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7160_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        /* for 7260 modem */
        {"XMM7260_CONF_1", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_2", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_3", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_4", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_5", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_6", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {"XMM7260_CONF_7", SFI_DEV_TYPE_MDM, 0, &modem_platform_data,
                &sfi_handle_mdm},
        {},

};
#endif

/*
 * Identifies the type of the board using SPID and returns
 * the respective device_id ptr.
 * @ returns NULL for invalid device.
 *
 * In case if you want to override the default device_id table,
 * Create a new device id table in board file and assign it to
 * device pointer.
 *
 * Example
 *	#ifdef CONFIG_BOARD_XXXX
 *		if (INTEL_MID_BOARD(1,PHONE,XXXX))
 *			dev_ptr = XXXX_device_ids;
 *	#endif
 */

struct devs_id __init *get_device_ptr(void)
{
#ifdef CONFIG_ME302C
        return device_ids_me302c;
#endif

#ifdef CONFIG_TX201LA
        return device_ids_tx201la;
#endif

#ifdef CONFIG_ME372CG
        return device_ids_me372cg;
#endif

#ifdef CONFIG_ME372CL
        return device_ids_me372cl;
#endif

#ifdef CONFIG_ME175CG
        return device_ids_me175cg;
#endif

#ifdef CONFIG_PF400CG
        return device_ids_pf400cg;
#endif

#ifdef CONFIG_A400CG
        return device_ids_a400cg;
#endif

#ifdef CONFIG_FE380CG
        return device_ids_fe380cg;
#endif

#ifdef CONFIG_A450CG
        return device_ids_a450cg;
#endif

#ifdef CONFIG_TX201LAF
        return device_ids_tx201laf;
#endif
}


