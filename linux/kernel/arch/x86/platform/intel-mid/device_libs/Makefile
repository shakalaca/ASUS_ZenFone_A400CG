# IPC Devices
obj-y += platform_sst_audio.o
obj-y += platform_clv_regulator.o
obj-y += platform_mrfl_regulator.o
obj-y += platform_byt_regulator.o
obj-y += platform_soc_thermal.o
obj-$(subst m,y,$(CONFIG_SND_BYT_MACHINE)) += platform_byt_audio.o
obj-$(subst m,y,$(CONFIG_SND_CHT_MACHINE)) += platform_byt_audio.o
obj-$(subst m,y,$(CONFIG_SND_MRFLD_MACHINE)) += platform_mrfld_audio.o
obj-$(subst m,y,$(CONFIG_SND_MOOR_MACHINE)) += platform_mrfld_audio.o
obj-$(subst m,y,$(CONFIG_SND_CTP_MACHINE_RT5640)) += platform_ctp_audio.o
obj-$(subst m,y,$(CONFIG_SND_CTP_MACHINE_RT5647)) += platform_ctp_audio.o
obj-$(subst m,y,$(CONFIG_SND_CTP_MACHINE_RT5648)) += platform_ctp_audio.o
obj-y += platform_ipc.o
obj-y += platform_i2c_gpio.o
obj-y += platform_msic.o
obj-y += platform_msic_audio.o
obj-y += platform_msic_gpio.o
obj-y += platform_msic_ocd.o
obj-y += platform_tc35876x.o
obj-y += pci/
obj-$(subst m,y,$(CONFIG_BATTERY_INTEL_MDF)) += platform_msic_battery.o
obj-$(subst m,y,$(CONFIG_INTEL_MID_POWER_BUTTON)) += platform_msic_power_btn.o
obj-$(subst m,y,$(CONFIG_GPIO_INTEL_PMIC)) += platform_pmic_gpio.o
obj-$(subst m,y,$(CONFIG_MID_PWM)) += platform_mid_pwm.o
obj-$(subst m,y,$(CONFIG_INTEL_MFLD_THERMAL)) += platform_msic_thermal.o
obj-$(subst m,y,$(CONFIG_SENSORS_MID_VDD)) += platform_msic_vdd.o
obj-$(subst m,y,$(CONFIG_SENSORS_MRFL_OCD)) += platform_mrfl_ocd.o
obj-$(subst m,y,$(CONFIG_PMIC_CCSM)) += platform_mrfl_pmic.o
obj-$(subst m,y,$(CONFIG_I2C_PMIC)) += platform_mrfl_pmic_i2c.o
obj-$(subst m,y,$(CONFIG_VLV2_PLAT_CLK)) += platform_vlv2_plat_clk.o
ifdef CONFIG_INTEL_BYT_THERMAL
obj-$(subst m,y,$(CONFIG_INTEL_BYT_THERMAL)) += platform_byt_thermal.o
else
obj-$(subst m,y,$(CONFIG_INTEL_BYT_EC_THERMAL)) += platform_byt_thermal.o
endif
obj-$(subst m,y,$(CONFIG_SENSORS_THERMAL_MRFLD)) += platform_mrfl_thermal.o
obj-$(subst m,y,$(CONFIG_INTEL_MOOR_THERMAL)) += platform_moor_thermal.o
obj-$(subst m,y,$(CONFIG_INTEL_SCU_FLIS)) += platform_scu_flis.o

# I2C Devices
obj-$(subst m,y,$(CONFIG_I2C_DESIGNWARE_CORE_FORK)) += platform_dw_i2c.o
obj-$(subst m,y,$(CONFIG_SENSORS_EMC1403)) += platform_emc1403.o
obj-$(subst m,y,$(CONFIG_SENSORS_LIS3LV02D)) += platform_lis331.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X)) += platform_max7315.o
obj-$(subst m,y,$(CONFIG_SENSORS_MPU3050)) += platform_mpu3050.o
obj-$(subst m,y,$(CONFIG_GPIO_PCA953X)) += platform_tca6416.o
obj-$(subst m,y,$(CONFIG_CHARGER_BQ24192)) += platform_bq24192.o
obj-$(subst m,y,$(CONFIG_BQ24261_CHARGER)) += platform_bq24261.o
obj-$(subst m,y,$(CONFIG_SND_SOC_CS42L73)) += platform_cs42l73.o
obj-$(subst m,y,$(CONFIG_SND_SOC_WM8994)) += platform_wm8994.o
obj-$(subst m,y,$(CONFIG_INTEL_CRYSTALCOVE_PAGE0)) += platform_crystalcove_page0.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_SYNAPTICS_I2C_RMI4_FORK)) += platform_rmi4.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_R69001_I2C)) += platform_r69001.o
obj-$(subst m,y,$(CONFIG_NFC_PN544_PLATFORM_DATA)) += platform_pn544.o
obj-$(subst m,y,$(CONFIG_SENSORS_L3G4200D_POLL)) += platform_l3g4200d.o
obj-$(subst m,y,$(CONFIG_SENSORS_LIS3DH_ACC)) += platform_lis3dh.o
obj-$(subst m,y,$(CONFIG_SENSORS_LSM303_MAG)) += platform_lsm303.o
obj-$(subst m,y,$(CONFIG_SENSORS_APDS990X_FORK)) += platform_apds990x.o
obj-$(subst m,y,$(CONFIG_A1026)) += platform_a1026.o
obj-$(subst m,y,$(CONFIG_EXTCON_FSA9285)) += platform_fsa9285.o
obj-$(subst m,y,$(CONFIG_EXTCON_SMSC375X))	+= platform_smsc375x.o
obj-$(subst m,y,$(CONFIG_SND_SOC_WM5102)) += platform_wm5102.o

# SPI Devices
obj-$(subst m,y,$(CONFIG_SERIAL_MRST_MAX3110)) += platform_max3111.o
# HSI Devices
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_hsi_modem.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_ffl_modem.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_edlp_modem.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_edlp_fast.o
obj-$(subst m,y,$(CONFIG_HSI_ARASAN))					+= platform_logical_modem.o
# Modem control Device
obj-y									+= platform_modem_ctrl.o
# MISC Devices
obj-$(subst m,y,$(CONFIG_KEYBOARD_GPIO)) += platform_gpio_keys.o
obj-$(subst m,y,$(CONFIG_SWITCH_MID))                                   += platform_switch_mid.o
obj-$(subst m,y,$(CONFIG_SWITCH_MID_USER_NOTIFY))                       += platform_switch_mid_user_notify.o
# ADC
obj-$(subst m,y,$(CONFIG_MSIC_GPADC))	+= platform_msic_adc.o
obj-$(subst m,y,$(CONFIG_IIO_BASINCOVE_GPADC))	+= platform_bcove_adc.o
# UART Devices
obj-$(subst m,y,$(CONFIG_SERIAL_MFD_HSU_EXT)) += platform_hsu.o
# SD Devices
obj-$(subst m,y,$(CONFIG_WILINK_PLATFORM_DATA)) += platform_wl12xx.o
ifndef CONFIG_ACPI
obj-$(subst m,y,$(CONFIG_BCM_BT_LPM)) += platform_btlpm.o
endif
#I2C Devices
# Camera
obj-$(subst m,y,$(CONFIG_VIDEO_ATOMISP_TMP))				+= platform_camera.o
ifdef CONFIG_VIDEO_ATOMISP_TMP
obj-$(subst m,y,$(CONFIG_VIDEO_MT9E013))				+= platform_mt9e013.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9D113))				+= platform_mt9d113.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9M114))				+= platform_mt9m114.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9V113))				+= platform_mt9v113.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV8830))					+= platform_ov8830.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX))					+= platform_imx175.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX))					+= platform_imx135.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX))					+= platform_imx132.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX))					+= platform_imx134.o
obj-$(subst m,y,$(CONFIG_VIDEO_S5K8AAY))				+= platform_s5k8aay.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV9724))					+= platform_ov9724.o
obj-$(subst m,y,$(CONFIG_VIDEO_OV2722))					+= platform_ov2722.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3554))					+= platform_lm3554.o
obj-$(subst m,y,$(CONFIG_VIDEO_LM3559))					+= platform_lm3559.o
# Peter++
obj-$(subst m,y,$(CONFIG_VIDEO_OV5693))					+= platform_ov5693.o
obj-$(subst m,y,$(CONFIG_VIDEO_AR0543))					+= platform_ar0543.o
obj-$(subst m,y,$(CONFIG_VIDEO_HM2056_RAW))				+= platform_hm2056_raw.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX111_RAW))				+= platform_imx111_raw.o
obj-$(subst m,y,$(CONFIG_VIDEO_AR0543_RAW))				+= platform_ar0543_raw.o
obj-$(subst m,y,$(CONFIG_VIDEO_MT9M114_RAW))				+= platform_mt9m114_raw.o
obj-$(subst m,y,$(CONFIG_VIDEO_GC0339_RAW))              += platform_gc0339_raw.o
obj-$(subst m,y,$(CONFIG_VIDEO_IMX219))					+= platform_imx219.o
endif
# SPI Devices
# Patrick
obj-$(subst m,y,$(CONFIG_VIDEO_SPCA700XA))				+= platform_spca700xa.o
obj-$(subst m,y,$(CONFIG_EXTCON_USB))					+= platform_extcon_usb.o
ifdef CONFIG_BATTERY_MAX17042
obj-$(subst m,y,$(CONFIG_BATTERY_MAX17042)) += platform_max17042.o
else
obj-$(subst m,y,$(CONFIG_BATTERY_MAX17050)) += platform_max17042.o
endif
obj-$(subst m,y,$(CONFIG_CHARGER_SMB347))				+= platform_smb347.o
# Panel Control Device
obj-$(subst m,y,$(CONFIG_DRM_MRFLD)) += platform_panel.o
# GPS
obj-$(subst m,y,$(CONFIG_INTEL_MID_GPS)) += platform_gps.o
# WIFI devices
obj-$(subst m,y,$(CONFIG_WIFI_PLATFORM_DATA)) += platform_wifi.o
obj-$(subst m,y,$(CONFIG_MMC_SDHCI_ACPI)) += platform_sdio_regulator.o

# Battery chris: (((
obj-$(subst m,y,$(CONFIG_ME372CG_BATTERY_SMB345))		+= platform_me372cg_smb345.o
obj-$(subst m,y,$(CONFIG_ME372CG_BATTERY_BQ27520))		+= platform_bq27520.o
# )))

# SCU log
obj-$(subst m,y,$(CONFIG_SCU_LOGGING)) += platform_scu_log.o
# Display Control Device
obj-y += platform_display.o

# alp.D add for Sensors ++
obj-$(subst m,y,$(CONFIG_SENSORS_MPU6500)) 				+= platform_gyro.o
obj-$(subst m,y,$(CONFIG_SENSORS_AKM8963)) 				+= platform_compass.o
obj-$(subst m,y,$(CONFIG_SENSORS_AL3320A)) 				+= platform_al3320a.o
# alp.D add for Sensors --
# Tom add for Sensor ++
obj-$(subst m,y,$(CONFIG_SENSORS_AK09911)) 				+= platform_akm09911.o
obj-$(subst m,y,$(CONFIG_SENSORS_KXTJ9)) 				+= platform_accel.o
obj-$(subst m,y,$(CONFIG_SENSORS_CAP1106)) 				+= platform_cap1106.o
obj-$(subst m,y,$(CONFIG_SENSORS_AP3212C)) 				+= platform_ap3212c.o
obj-$(subst m,y,$(CONFIG_SENSORS_PX3003B))   				+= platform_px3003b.o
obj-$(subst m,y,$(CONFIG_SENSORS_PX3212C))   				+= platform_px3212c.o
# Tom add for Sensor --

# add by leo for Goodix touch ++
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_GOODIX_GT927))	+= platform_gt927.o
# add by leo for Goodix touch --
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528_A12))                   += platform_hx8528_a12.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528_P72G))                   += platform_hx8528_p72g.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528_ME372CL))     	+= platform_hx8528_me372cl.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528))			+= platform_hx8528.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528_FE380CG))		+= platform_hx8528_fe380cg.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528_A400))    += platform_hx8528_a400cg.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_HIMAX_HX8528_A450CG))    += platform_hx8528_a450cg.o
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ELAN_EKTH3374))                   += platform_ektf3k.o
# Jui-Chuan add for Atmel ++
obj-$(subst m,y,$(CONFIG_TOUCHSCREEN_ATMEL_MXT1664S))   +=  platform_mxt1664s.o
# Jui-Chuan add for Atmel --

# Joe add for EC driver ++
obj-$(subst m,y,$(CONFIG_HID_ASUS_PAD_EC))	+= platform_asus_ec.o
# Joe add for EC driver --

# Joe add for Elan touch pad driver ++
obj-$(subst m,y,$(CONFIG_MOUSE_ELAN_TOUCHPAD))	+= platform_elan_touchpad.o
# Joe add for Elan touch pad driver --

