/*
 * drivers/i2c/chips/asusec.c
 *
 * Copyright (C) 2011 ITE Inc.
 *
 * Written by Donald Huang <donald.huang@ite.com.tw>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation (version 2 of the License only).
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
 *
 * v1.1 2011.06.29 Donald use i2c_smbus_read_block_data instead of
 *                 i2c_smbus_read_i2c_block_data
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/earlysuspend.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <asm/gpio.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>

#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/ite8566.h>
#include <asm/unaligned.h>
#include <linux/workqueue.h>
#include "asus_ec_ite.h"
#include <linux/usb/penwell_otg.h>
#include <linux/HWVersion.h>

#define DRIVER_VERSION	"3.0.9"		//Joe modify 

//define flag parameter --------------------------------------------------------------------------------
#define ENABLE_SELF_FIRMWARE_UPGRADE
#define ENABLE_FIRMWARE_UPGRADE

//Global variable --------------------------------------------------------------------------------
static uint8_t i2c_command = 0;
static struct i2c_client ecram_client;
u8 recv_buf[256]={0}; // add by leo for ec register read & write ++
u16 recv_num=0; // add by leo for ec register read & write ++
static int probe_status = 0 ;
static int usb_switch_mode_flag = 0 ;

static BLOCKING_NOTIFIER_HEAD(dock_atow_early_notifier_list);
static BLOCKING_NOTIFIER_HEAD(dock_atow_late_notifier_list);
static BLOCKING_NOTIFIER_HEAD(dock_wtoa_late_notifier_list);
static BLOCKING_NOTIFIER_HEAD(dock_detach_notifier_list);
static BLOCKING_NOTIFIER_HEAD(dock_attach_notifier_list);
static BLOCKING_NOTIFIER_HEAD(base_system_notifier_list);

static int usb_cable_init_value = 2;

#ifdef ENABLE_FIRMWARE_UPGRADE
static struct i2c_client predefine_client;
static unsigned char upgrade_fw[64*1024];
static unsigned char flash_id[3];
//static checksume_bin = 0;
//static checksume_verify = 0;
#endif
// Add by Leo for touch notify EC ++
static int Did_FW_update = 0;
// Add by Leo for touch notify EC --

enum hid_i2c_command {
	ITE_HID_DESCR_LENGTH_CMD,
	ITE_HID_DESCR_CMD,
	ITE_HID_REPORT_DESCR_CMD,
	ITE_HID_INPUT_REPORT_CMD,
	ITE_HID_OUTPUT_REPORT_CMD,
	ITE_HID_POWER_ON_CMD,
	ITE_HID_RESET_CMD,
	ITE_HID_DATA_CMD,
	ITE_HID_PAD_BATTERY_CMD,
	ITE_HID_DOCK_BATTERY_CMD,
	ITE_HID_CABLE_STATUS_NOTIFY_CMD,
	ITE_HID_VERIFY_CHARGER_CMD,
	ITE_HID_ATTACH_DETACH_CMD,
	ITE_HID_ANDROID_POWER_STATUS_CMD,
	ITE_HID_SYSTEM_OWNER_CMD,
	ITE_HID_SCREEN_CHANGE_NOTIFY_CMD,
	ITE_HID_BASE_SYSTEM_STATUS_CMD,
	ITE_HID_READ_EC_REGISTER_CMD,
//	ITE_HID_LED_NOTIFY_CMD,
	ITE_HID_READ_CHARGER_IC_STATUS_CMD,
	ITE_HID_ALS_CTL_CMD,
	ITE_HID_VERIFY_ALS_STATUS_CMD,
	ITE_HID_PWM_CTL_CMD,
	ITE_HID_RW_CHARGERIC_CMD,
	ITE_HID_WIFI_LED_CMD,
	ITE_HID_TURNON_BASE_CMD,
	//add by leo for gauge FW update ++
	ITE_HID_READ_GAUGE_FW_INFO_CMD,
	ITE_HID_START_GAUGE_FW_UPDATE_CMD,
	//add by leo for gauge FW update --
	ITE_HID_ALS_SET_TABLE_CMD,
	ITE_HID_READ_BASE_CHARGER_STATUS_CMD,
	ITE_HID_READ_WIN8_TEST_MODE_CMD,
	ITE_HID_GET_LIGHT_SENSOR_REPORT_CMD, 
	ITE_HID_KB_LED_CMD,
	ITE_HID_BASE_EC_FW_CHECK_CMD,
	ITE_HID_CHECK_CHARGER_STATUS_CMD
};

enum ec_ram_i2c_command {
	ITE_RAM_VERSION_CMD,
	ITE_RAM_ENTER_FLASH_MODE_CMD,
	ITE_RAM_STOP_POLLING_BATTERY_CMD,
	ITE_RAM_START_POLLING_BATTERY_CMD,
	ITE_RAM_WRITE_GAUGE_CMD,
	ITE_RAM_READ_GAUGE_CMD,
	ITE_RAM_ACCESS_GAUGE_STATUE_CMD,
	ITE_RAM_READ_GAUGE_BUFFER_CMD,
	ITE_RAM_CTL_USB0_ID_EC_CMD, //USB Host/Client
	ITE_RAM_TP_I2C_SW_CMD,
	ITE_RAM_NOTIFY_SYSTEM_STATUS_CMD,
	ITE_RAM_NOTIFY_OS_CLASS_CMD, //Normal = 0x01, Charger = 0x02, Recovery Mode = 0x04, Download Mode = 0x08
	ITE_RAM_GAUGE_COMPARE_RESULT_CMD,
	ITE_RAM_SCREEN_CHANGE_NOTIFY_CMD,
	ITE_RAM_CABLE_STATUS_NOTIFY_CMD,
	ITE_RAM_TURN_ON_BASE_CMD,
	ITE_RAM_SET_KEYBOARD_WAKEUP_CMD,
	ITE_RAM_LED_CONTROL_CMD,
	ITE_RAM_GAUGE_TEMP_CONTROL_CMD
};

typedef enum {
	SELF,
	AP,
	ETC,
    LOCAL
}EC_FW_PATH;

//define parameter --------------------------------------------------------------------------------
#define ITE_I2C_COMMAND_TRIES	3
#define ITE_I2C_COMMAND_DELAY	50
#define ITE_REPORT_MAX_LENGTH	256
#define HID_DES_LENGTH_OFFSET	30
#define HID_DES_LENGTH_OFFSET	30
#define ITE_MAX_INPUT_LENGTH	14
#define ITE_IRQ1_MAX_INPUT_LENGTH	2
#define ITE_MAX_RECEIVED_LENGTH 150 // add by leo for ec register read & write ++
#define ITE_INPUT_MAX 100  // add by Josh for ec register read & write ++ 

//I2C command --------------------------------------------------------------------------------
static const u8 ite_hid_descriptor_cmd[] = {0xF1, 0x00};
static const u8 ite_hid_report_descr_cmd[] = {0xF2, 0x00};
static const u8 ite_hid_input_report_cmd[] = {0xF3, 0x00};
static const u8 ite_hid_output_report_cmd[] = {0xF4, 0x00};
static const u8 ite_hid_power_on_cmd[] = {0xF5, 0x00, 0x00, 0x08};
static const u8 ite_hid_reset_cmd[] = {0xF5, 0x00, 0x00, 0x01};
static const u8 ite_hid_data_cmd[] = {0xF6, 0x00};
static const u8 ite_hid_pad_battery_cmd[] = {0xF5, 0x00, 0x1C, 0x02, 0xF6, 0x00};
static const u8 ite_hid_dock_battery_cmd[] = {0xF5, 0x00, 0x1D, 0x02, 0xF6, 0x00};
static const u8 ite_hid_verify_charger_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0x30, 0x00, 0x01};
static const u8 ite_hid_attach_detach_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00 , 0x33, 0x00, 0x01};
static const u8 ite_hid_system_owner_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0x34, 0x00, 0x01};
static const u8 ite_hid_base_system_status_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0x32, 0x00, 0x01};
static const u8 ite_hid_read_ec_register_cmd[] = {0xF5, 0x00, 0x3A, 0x02, 0xF6, 0x00};
static const u8 ite_hid_read_charger_ic_status_cmd[] = {0xF5, 0x00, 0x1B, 0x02, 0xF6, 0x00};
static const u8 ite_hid_verify_als_status_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0x0C, 0x09, 0x01};
static const u8 ite_hid_read_base_charger_status_cmd[] = {0xF5, 0x00, 0x3B, 0x02, 0xF6, 0x00};
static const u8 ite_hid_read_win8_test_mode_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0x03, 0x0E, 0x01};
static const u8 ite_hid_get_light_sensor_report_cmd[] = {0xF5, 0x00, 0x36, 0x02, 0xF6, 0x00};
static const u8 ite_hid_base_ec_fw_check_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0x52, 0x00, 0x01};
static const u8 ite_hid_check_charger_status_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x00, 0xD0, 0x09, 0x01};

u8 ite_hid_cable_status_notify_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x01, 0x30, 0x00, 0x00};
u8 ite_hid_android_power_status_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x01, 0x31, 0x00, 0x00};
u8 ite_hid_screen_change_notify_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x01, 0x34, 0x00, 0x00};
//u8 ite_hid_led_notify_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x01, 0x50, 0x00, 0x00};
u8 ite_hid_als_ctl_cmd[] = {0xF4, 0x00, 0x04, 0x00, 0x06, 0x00}; // light sensor enable/disable
u8 ite_hid_pwm_ctl_cmd[] = {0xF5, 0x00, 0x3E, 0x03, 0xF6, 0x00, 0x03, 0x00, 0x00};
u8 ite_hid_rw_chargeric_cmd[] = {0xF5, 0x00, 0x3B, 0x03, 0xF6, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 ite_hid_wifi_led_cmd[] = {0xF4, 0x00, 0x04, 0x00, 0x04, 0x00};
u8 ite_hid_kb_led_cmd[] = {0xF4, 0x00, 0x04, 0x00, 0x01, 0x00};
u8 ite_hid_turnon_base_cmd[] = {0xF5, 0x00, 0x3A, 0x03, 0xF6, 0x00, 0x06, 0x00, 0x01, 0x36, 0x00, 0x01};
// add by leo for gauge FW update ++
u8 ite_hid_read_gauge_fw_info_cmd[] = {0xF5, 0x00, 0x38, 0x02, 0xf6, 0x00};
u8 ite_hid_start_gauge_fw_update_cmd[] = {0xF5, 0x00, 0x38, 0x03, 0xf6, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// add by leo for gauge FW update --
u8 ite_hid_als_set_table_cmd[256] = {0};

static const u8 ite_ram_version_cmd[] = {0xEC};
static const u8 ite_ram_enter_flash_mode_cmd[] = {0xEF};
static const u8 ite_ram_stop_polling_battery_cmd[] = {0xC1};
static const u8 ite_ram_start_polling_battery_cmd[] = {0xC2};
static const u8 ite_ram_access_gauge_statue_cmd[] = {0xC0, 0x01};
static const u8 ite_ram_read_gauge_buffer_cmd[] = {0xC0, 0x02};

u8 ite_ram_write_gauge_cmd[256] = {0};
u8 ite_ram_read_gauge_cmd[] = {0xC0, 0x02, 0x00, 0x00, 0x00};
u8 ite_ram_ctl_usb0_id_ec_cmd[] = {0xCE, 0x0A, 0x00};
u8 ite_ram_tp_i2c_sw_cmd[] = {0xCE, 0xD0, 0x00};
u8 ite_ram_notify_system_status_cmd[] = {0xCE, 0x0B, 0x00};
u8 ite_ram_notify_os_class_cmd[] = {0xCE, 0x0C, 0x00};
u8 ite_ram_gauge_compare_result_cmd[256] = {0};
u8 ite_ram_screen_change_notify_cmd[] = {0xCE, 0x0D, 0x00};
u8 ite_ram_cable_status_notify_cmd[] = {0xCE, 0x0E, 0x00};
u8 ite_ram_turn_on_base_cmd[] = {0xCE, 0x10, 0x01};
u8 ite_ram_set_keyboard_wakeup_cmd[] = {0xCE, 0x12, 0x00};
u8 ite_ram_led_control_cmd[] = {0xCE, 0x14, 0x00};
u8 ite_ram_gauge_temp_control_cmd[] = {0xCE, 0x15, 0x00};

static const unsigned char i2c_kbd_keycode[256] = {
          0,  0,  0,  0, KEY_A, KEY_B, KEY_C, KEY_D, KEY_E, KEY_F, KEY_G, KEY_H, KEY_I, KEY_J, KEY_K, KEY_L, // 0 ~ 15
         KEY_M, KEY_N, KEY_O, KEY_P, KEY_Q, KEY_R, KEY_S, KEY_T, KEY_U, KEY_V, KEY_W, KEY_X, KEY_Y, KEY_Z, KEY_1, KEY_2, // 16 ~ 31
         KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9, KEY_0, KEY_ENTER, KEY_ESC, KEY_BACKSPACE, KEY_TAB, KEY_SPACE, KEY_MINUS, KEY_EQUAL, KEY_LEFTBRACE, // 32 ~ 47
         KEY_RIGHTBRACE, KEY_BACKSLASH, KEY_BACKSLASH, KEY_SEMICOLON, KEY_APOSTROPHE, KEY_GRAVE, KEY_COMMA, KEY_DOT, KEY_SLASH, KEY_CAPSLOCK, KEY_F1, KEY_F2, KEY_F3, KEY_F4, KEY_F5, KEY_F6, // 48 ~ 63
         KEY_F7, KEY_F8, KEY_F9, KEY_F10, KEY_F11, KEY_F12, ASUSEC_KEYCODE_SCREEN_CAPTURE/*KEY_SYSRQ*/, KEY_SCROLLLOCK, KEY_PAUSE, KEY_INSERT, KEY_HOME, KEY_PAGEUP, KEY_DELETE, KEY_END, KEY_PAGEDOWN, KEY_RIGHT, // 64 ~ 79
         KEY_LEFT, KEY_DOWN, KEY_UP, KEY_NUMLOCK, KEY_KPSLASH, KEY_KPASTERISK, KEY_KPMINUS, KEY_KPPLUS, KEY_KPENTER, KEY_KP1, KEY_KP2, KEY_KP3, KEY_KP4, KEY_KP5, KEY_KP6, KEY_KP7, // 80 ~ 95
         KEY_KP8, KEY_KP9, KEY_KP0, KEY_KPDOT, KEY_102ND, /*ASUSEC_KEYCODE_LAUNCH_SETTING*/KEY_COMPOSE, KEY_POWER, KEY_KPEQUAL, KEY_F13, KEY_F14, KEY_F15, KEY_F16, KEY_F17, KEY_F18, KEY_F19, KEY_F20, // 96 ~ 111
         KEY_F21, KEY_F22, KEY_F23, KEY_F24, KEY_OPEN, KEY_HELP, KEY_PROPS, KEY_FRONT, KEY_STOP, KEY_AGAIN, KEY_UNDO, KEY_CUT, KEY_YEN, KEY_PASTE, KEY_FIND, KEY_MUTE, // 112 ~ 127
         KEY_VOLUMEUP, KEY_VOLUMEDOWN,  0,  0,  0, KEY_KPCOMMA,  0, KEY_RO, KEY_KATAKANAHIRAGANA, KEY_YEN, KEY_HENKAN, KEY_MUHENKAN, KEY_KPJPCOMMA,  0,  0,  0, // 128 ~ 143
         KEY_HANGEUL, KEY_HANJA, KEY_KATAKANA, KEY_HIRAGANA, KEY_ZENKAKUHANKAKU,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 144 ~ 159
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 160 ~ 175
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 176 ~ 191
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 192 ~ 207
          0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0, // 208 ~ 223
         KEY_LEFTCTRL, KEY_LEFTSHIFT, KEY_LEFTALT, KEY_LEFTMETA, KEY_RIGHTCTRL, KEY_RIGHTSHIFT, KEY_RIGHTALT, KEY_RIGHTMETA, KEY_PLAYPAUSE, KEY_STOPCD, KEY_PREVIOUSSONG, KEY_NEXTSONG, KEY_EJECTCD, KEY_VOLUMEUP, KEY_VOLUMEDOWN, KEY_MUTE, // 224 ~ 239
         KEY_WWW, KEY_BACK, KEY_FORWARD, KEY_STOP, KEY_FIND, KEY_SCROLLUP, KEY_SCROLLDOWN, KEY_EDIT, KEY_SLEEP, KEY_COFFEE, KEY_REFRESH, KEY_CALC, 0, 0, 0, 0 // 240 ~ 255
};

//extern function -------------------------------------------------------------------------------------------------
extern unsigned int entry_mode;
extern unsigned int factory_mode;
extern int Read_HW_ID(void);
#ifdef CONFIG_USB_PENWELL_OTG
	extern int check_cable_status(void);
#endif

#ifdef CONFIG_SENSORS_ALS3010
	extern int als3010_work_function(int lux, int ilevel, int raw_data);
#endif

#ifdef CONFIG_MOUSE_ELAN_TOUCHPAD
	extern int elan_i2c_touchpad_enable(int enable);
	extern int elan_i2c_bus_enable(int enable);
#endif

#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT1664S
	// add by leo for touch notify EC ++
	#define IPCMSG_COLD_RESET        0xF1
	extern int rpmsg_send_generic_simple_command(u32 cmd, u32 sub);
	// add by leo for touch notify EC --
	extern int update_touch_fw_called_by_EC(void);
	extern int enable_touch_called_by_EC(void);
#endif

#ifdef CONFIG_TX201LA_BATTERY_EC
extern void asus_pad_batt_set_charger_data(u8 lsb, u8 msb);
extern void pad_bat_set_base_ac_status(int ac_status);
extern void batt_gauge_fw_result_cb(u8 data);
#endif

#ifdef CONFIG_HALL_SENSOR
extern void lid_status_report(int open);
#endif

//static function -------------------------------------------------------------------------------------------------
static bool ite_i2c_hw_init(struct i2c_client *client);
static ssize_t ite_show_disable(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_set_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ite_register_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ite_ec_ver_show(struct device *class, struct device_attribute *attr,char *buf);
static ssize_t ite_system_owner(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_get_ec_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_pad_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t ite_charger_ic_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_light_sensor_switch(struct device *class,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t ite_light_sensor_level(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_base_plug_in(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_light_sensor_status(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_wifi_led(struct device *class,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t ite_base_power_on(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_touchpad_enable(struct device *class,struct device_attribute *attr,const char *buf, size_t count);
static ssize_t ite_switch_to_win8(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_switch_to_android(struct device *class,struct device_attribute *attr,char *buf);
// add by leo for ec register read & write ++
static ssize_t ite_66readwrite_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_66readwrite_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ite_68readwrite_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_68readwrite_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
// add by leo for ec register read & write --
// add by leo for gauge FW update ++
static ssize_t ite_gauge_fw_update_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_gauge_fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t ite_gauge_fw_check_show(struct device *dev, struct device_attribute *attr, char *buf);
// add by leo for gauge FW update --
static ssize_t ite_gauge_read_version(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t ite_win8_test_mode(struct device *class,struct device_attribute *attr,char *buf);
static ssize_t ite_charger_status(struct device *class,struct device_attribute *attr,char *buf);

static int asus_ecram_i2c_command(struct i2c_client *client, int command, unsigned char *buf_recv, int data_len);
// add by Josh for create keyboard device ++
static int asusec_input_device_create(void);
static int asusec_input_device_destroy(void);
//  add by Josh for create keyboard device --
static int ite_detach_attach_function(void);
static int ite_base_power_status(void);
static int ite_system_owner_function(void);
static int ite_turn_on_base_power(void);
static int ite_wifi_led_function(int on_off);
static int ite_get_ec_version(void);
static bool ite_screen_change_function(int screen_status);
static bool ite_android_power_status(APOWER_STATUS power_status);
static int ite_win8_power_switch_status_report(void);
static void ite_base_switch_status_report(void);
static int ite_pad_led_function(int status);
static int ite_read_win8_test_mode(void);
static int ite_keyboard_led_function(KB_LED type);
static int ite_ram_enable_keyboard_wakeup(int enable);
static int ite_base_ec_fw_check(void);
static void ite_work_0x68(struct work_struct *work);

int ite_light_sensor_set_function(int cmd);
int ite_usb_switch_mode(int usb_mode);
int ite_touchpad_switch(SYSTEM_OWNER mode);

static DEVICE_ATTR(disable_kp, (S_IWUSR|S_IRUGO), ite_show_disable, ite_set_disable);
static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO), ite_register_show, ite_register_store);
static DEVICE_ATTR(ec_ver, (S_IWUSR | S_IRUGO), ite_ec_ver_show, NULL);
static DEVICE_ATTR(owner, (S_IWUSR | S_IRUGO), ite_system_owner, NULL);
static DEVICE_ATTR(status, (S_IWUSR|S_IRUGO), ite_get_ec_status, NULL);
static DEVICE_ATTR(pad_led, (S_IWUSR | S_IRUGO), NULL, ite_pad_led_store);
static DEVICE_ATTR(chargerIC_status, (S_IWUSR | S_IRUGO), ite_charger_ic_status, NULL);
static DEVICE_ATTR(als, (S_IWUSR | S_IRUGO), NULL, ite_light_sensor_switch);
static DEVICE_ATTR(als_level, (S_IWUSR | S_IRUGO), ite_light_sensor_level, NULL);
static DEVICE_ATTR(base_in, (S_IWUSR | S_IRUGO), ite_base_plug_in, NULL);
static DEVICE_ATTR(als_status, (S_IWUSR | S_IRUGO), ite_light_sensor_status, NULL);
static DEVICE_ATTR(wifi_led, (S_IWUSR | S_IRUGO), NULL, ite_wifi_led);
static DEVICE_ATTR(turnon_win8, (S_IWUSR | S_IRUGO), ite_base_power_on, NULL);
static DEVICE_ATTR(tp_enable, (S_IWUSR | S_IRUGO), NULL, ite_touchpad_enable);
static DEVICE_ATTR(switch_win8, (S_IWUSR | S_IRUGO), ite_switch_to_win8, NULL);
static DEVICE_ATTR(switch_android, (S_IWUSR | S_IRUGO), ite_switch_to_android, NULL);
// add by leo for ec register read & write ++
static DEVICE_ATTR(ec_66rw, (S_IWUSR|S_IRUGO), ite_66readwrite_show, ite_66readwrite_store);
static DEVICE_ATTR(ec_68rw, (S_IWUSR|S_IRUGO), ite_68readwrite_show, ite_68readwrite_store);
// add by leo for ec register read & write --
// add by leo for gauge FW update ++
static DEVICE_ATTR(gauge_fw_update, (S_IWUSR|S_IRUGO), ite_gauge_fw_update_show, ite_gauge_fw_update_store);
static DEVICE_ATTR(gauge_fw_check, (S_IWUSR|S_IRUGO), ite_gauge_fw_check_show, NULL);
// add by leo for gauge FW update --
static DEVICE_ATTR(gaugeIC_FW, (S_IWUSR|S_IRUGO), ite_gauge_read_version, NULL);
static DEVICE_ATTR(win8_test_mode, (S_IWUSR | S_IRUGO), ite_win8_test_mode, NULL);
static DEVICE_ATTR(check_charger_status, (S_IWUSR | S_IRUGO), ite_charger_status, NULL);

#define	ITE_PROC_FILE	"ite8566"
static struct proc_dir_entry *ite_proc_file;

#define	ITE_PROC_FACTORY_FILE	"asusec_factory"
static struct proc_dir_entry *ite_proc_factory_file;

static struct attribute *ite_attr[] = {
	&dev_attr_register.attr,
	&dev_attr_ec_ver.attr,
	&dev_attr_ec_66rw.attr,// add by leo for ec register read & write ++
	&dev_attr_ec_68rw.attr,// add by leo for ec register read & write ++
	&dev_attr_disable_kp.attr,
	&dev_attr_owner.attr,
	&dev_attr_status.attr,
	&dev_attr_pad_led.attr,
	&dev_attr_chargerIC_status.attr,
	&dev_attr_als.attr,
	&dev_attr_als_level.attr,
	&dev_attr_als_status.attr,
	&dev_attr_base_in.attr,
	&dev_attr_wifi_led.attr,
	&dev_attr_turnon_win8.attr,
	&dev_attr_tp_enable.attr,
	&dev_attr_switch_win8.attr,
	&dev_attr_switch_android.attr,
	&dev_attr_gauge_fw_update.attr,// add by leo for gauge FW update ++
	&dev_attr_gauge_fw_check.attr, // add by leo for gauge FW update ++
	&dev_attr_gaugeIC_FW.attr,
	&dev_attr_win8_test_mode.attr,	
	&dev_attr_check_charger_status.attr,
	NULL
};
//Android switch to windows early notify ++
static int ite_atow_early_notify_driver(int owner)
{
	int rc = 0;
	
	printk("[ITE]: %s: owner = %d \n",__func__, owner);
	rc = blocking_notifier_call_chain(&dock_atow_early_notifier_list, owner, NULL);
	return rc;
}

int register_dock_atow_early_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&dock_atow_early_notifier_list, nb);
}
EXPORT_SYMBOL(register_dock_atow_early_notifier);

int unregister_dock_atow_early_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&dock_atow_early_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_dock_atow_early_notifier);
//Android switch to windows early notify --
//Android switch to windows late notify ++
static int ite_atow_late_notify_driver(int owner)
{
	int rc = 0;
	
	printk("[ITE]: %s: owner = %d \n",__func__, owner);
	rc = blocking_notifier_call_chain(&dock_atow_late_notifier_list, owner, NULL);
	return rc;
}

int register_dock_atow_late_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&dock_atow_late_notifier_list, nb);
}
EXPORT_SYMBOL(register_dock_atow_late_notifier);

int unregister_dock_atow_late_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&dock_atow_late_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_dock_atow_late_notifier);
//Android switch to windows late notify --
//windows switch to android late notify ++
static int ite_wtoa_late_notify_driver(int owner)
{
	int rc = 0;
	
	printk("[ITE]: %s: owner = %d \n",__func__, owner);
	rc = blocking_notifier_call_chain(&dock_wtoa_late_notifier_list, owner, NULL);
	return rc;
}

int register_dock_wtoa_late_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&dock_wtoa_late_notifier_list, nb);
}
EXPORT_SYMBOL(register_dock_wtoa_late_notifier);

int unregister_dock_wtoa_late_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&dock_wtoa_late_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_dock_wtoa_late_notifier);
//windows switch to android late notify --
//android detach notify ++
static int ite_detach_notify_driver(int owner)
{
	int rc = 0;
	
	printk("[ITE]: %s: owner = %d \n",__func__, owner);
	rc =blocking_notifier_call_chain(&dock_detach_notifier_list, owner, NULL);
	return rc;
}

int register_dock_detach_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&dock_detach_notifier_list, nb);
}
EXPORT_SYMBOL(register_dock_detach_notifier);

int unregister_dock_detach_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&dock_detach_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_dock_detach_notifier);
//android detach notify --
//Attach notify ++
static int ite_attach_notify_driver(int owner)
{
	int rc = 0;
	
	printk("[ITE]: %s: owner = %d \n",__func__, owner);
	rc = blocking_notifier_call_chain(&dock_attach_notifier_list, owner, NULL);
	return rc;
}

int register_dock_attach_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&dock_attach_notifier_list, nb);
}
EXPORT_SYMBOL(register_dock_attach_notifier);

int unregister_dock_attach_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&dock_attach_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_dock_attach_notifier);
//Attach notify --
//Base system change status notify --
static int ite_base_system_change_notify_driver(int owner)
{
	int rc = 0;
	
	printk("[ITE]: %s: owner = %d \n",__func__, owner);
	rc = blocking_notifier_call_chain(&base_system_notifier_list, owner, NULL);
	return rc;
}

int register_base_system_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_register(&base_system_notifier_list, nb);
}
EXPORT_SYMBOL(register_base_system_notifier);

int unregister_base_system_notifier(struct notifier_block *nb)
{
	return blocking_notifier_chain_unregister(&base_system_notifier_list, nb);
}
EXPORT_SYMBOL_GPL(unregister_base_system_notifier);
//Base system change status notify --


static void ite_ecram_init(struct i2c_client *client)
{
	ecram_client.adapter = client->adapter;
	ecram_client.addr = ITE8566_EC_ADDR;
	ecram_client.detected = client->detected;
	ecram_client.dev = client->dev;
	ecram_client.driver = client->driver;
	ecram_client.flags = client->flags;
	strcpy(ecram_client.name, "ite8566_ecram");
	printk("[ITE_ECRAM]: ecram_client.name = %s \n", ecram_client.name);
}

#ifdef ENABLE_FIRMWARE_UPGRADE
static void ite_predefine_init(struct i2c_client *client)
{
	predefine_client.adapter = client->adapter;
	predefine_client.addr = ITE8566_HW_ADDR;
	predefine_client.detected = client->detected;
	predefine_client.dev = client->dev;
	predefine_client.driver = client->driver;
	predefine_client.flags = client->flags;
	strcpy(predefine_client.name, "ite8566_predefine");
	printk("[ITE_update]: predefine_client.name = %s \n", predefine_client.name);
}

static int ite_i2c_pre_define_cmd_read(struct i2c_client *client, unsigned char cmd1,unsigned int payload_len, unsigned char *buf_recv)
{
	int result = 0, ret = 0;
	struct i2c_msg msg[2];
	unsigned char buf0[1]={0x17};
	unsigned char buf1[2]={0x18,0x00};
	unsigned char *rec_buf = buf_recv;
		
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *) &buf0;
	
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	buf1[1] = cmd1;
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = (unsigned char *) &buf1;
	
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].len = payload_len;
	msg[1].buf = rec_buf;
	
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}

	return result;
}

static int ite_i2c_pre_define_cmd_write(struct i2c_client *client, unsigned char cmd1,unsigned int payload_len,unsigned char payload[])
{
	int result = 0, i, ret = 0;
	struct i2c_msg msg[1];
	unsigned char buf0[1]={0x17};	//CS High
	unsigned char buf1[256]={0x18};
					
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *) &buf0;
		
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	buf1[1] = cmd1;

	if(payload_len < 256)
	{
		for(i=0; i < payload_len; i++)
		{
			buf1[i+2] =payload[i];
		}
	}
	else{
		printk("[ITE_update]: ite_i2c_pre_define_cmd_write: payload_len over 256! \n");
		result = -1;
		return result;
	}
			
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = (payload_len+2);
	msg[0].buf = (unsigned char *) &buf1;
		
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	return result;
}

static int ite_i2c_pre_define_cmd_fastread(struct i2c_client *client, unsigned char addr[], unsigned int payload_len, unsigned char payload[])
{
	int result = 0, ret = 0;
	struct i2c_msg msg[3];
	unsigned char buf0[1]={0x17};           //CS High
	unsigned char buf1[6]={0x18,0x0B}; 	// 0x0b => Fast Read
		
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *) &buf0;
		
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	buf1[2] = addr[3]; // Address H
	buf1[3] = addr[2]; // Address M
	buf1[4] = addr[1]; // Address L
	buf1[5] = addr[0]; // Dummy
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 6;
	msg[0].buf = (unsigned char *) &buf1;
	
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].flags |= I2C_M_RD;
	msg[0].len = payload_len;
	msg[0].buf = (unsigned char *) payload;
	
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	return result;
}

static int ite_i2c_pre_define_cmd_write_with_status(struct i2c_client *client, unsigned char cmd1,unsigned int payload_len,unsigned char payload[])
{
	int result=0,i, ret = 0;
	int read_status[3];
	struct i2c_msg msg[5];
	unsigned char buf0[1]={0x17};           //CS High
	unsigned char buf1[256]={0x18};
	unsigned char cmd_status[2]={0x18, EFLASH_CMD_READ_STATUS};
		
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *) &buf0;
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	buf1[1] = cmd1;
	if(payload_len < 256)
	{
		for(i=0; i<payload_len; i++)
		{
			buf1[i+2] = payload[i];
		}
	}
	else
	{
		printk("[ITE_update]: ite_i2c_pre_define_cmd_write: payload_len over 256! \n");
		result = -1;
		return result;
	}
	
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = (payload_len+2);
	msg[0].buf = (unsigned char *) &buf1;
		
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *) &buf0;
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
			
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = (unsigned char *) &cmd_status;
	
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = (unsigned char *) &read_status;
	msg[1].len = 3;
	
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		return result;
	}
	
	result = read_status[2];
	
	return result;
}

static int ite_i2c_pre_define_cmd_read_byte(struct i2c_client *client, unsigned int address)
{
	int result=0, ret = 0;
	struct i2c_msg msg[2];
	unsigned char buf0[3]={0x10};
	unsigned char buf2[1]={0x11};
	unsigned char buf3[1];
	
	buf0[1] = (address >> 8) & 0xFF; //addr[15:8]
	buf0[2] = (address) & 0xFF; //addr[7:0]
		
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 3;
	msg[0].buf = (unsigned char *) &buf0;
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		goto err;
	}
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 1;
	msg[0].buf = (unsigned char *) &buf2;
	
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = (unsigned char *) &buf3;
	msg[1].len = 1;
	
	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		goto err;
	}
	
	result = buf3[0];
	return result;
	
err:
	return result;
}

static int ite_i2c_pre_define_cmd_writebyte(struct i2c_client *client, unsigned int address,unsigned char data)
{
	int result = 0, ret = 0;
	struct i2c_msg msg[2];
	unsigned char buf0[3]={0x10};
	unsigned char buf2[2]={0x11};
	
	buf0[1] = (address >> 8) & 0xFF; //addr[15:8]
	buf0[2] = (address) & 0xFF; //addr[7:0]
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 3;
	msg[0].buf = (unsigned char *) &buf0;
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		goto err;
	}
		
	//I2EC WRITE BYTE DATA
	buf2[1] = data;
	
	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = 2;
	msg[0].buf = (unsigned char *) &buf2;
	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0)
	{
		printk("[ITE_update]: %s: %d\n", __func__, __LINE__);
		result = -1;
		goto err;
	}
		
err:
	return result;
}

static int cmd_write_enable(void)
{
	int result;
	
	result = ite_i2c_pre_define_cmd_write(&predefine_client, EFLASH_CMD_WRITE_ENABLE, 0, NULL);
	return result;
}

static int cmd_write_disable(void)
{
	int result;
	
	result = ite_i2c_pre_define_cmd_write(&predefine_client, EFLASH_CMD_WRITE_DISABLE, 0, NULL);
	return result;
}

static unsigned char cmd_check_status(void)
{
	unsigned char status[2];
	
	ite_i2c_pre_define_cmd_read(&predefine_client, EFLASH_CMD_READ_STATUS, 2, status);
	return status[1];
}

static int cmd_erase_all(void)
{
	int result;

	result = ite_i2c_pre_define_cmd_write(&predefine_client, EFLASH_CMD_CHIP_ERASE, 0, NULL);
	return result;
}

static int do_check(void)
{
	int i,j,result=0;
	unsigned char buffer[256];
	unsigned char address[4] = {0,0,0,0};
	
	printk("[ITE_update]: Check................ start. \n");

	for(i=0; i<0x100; i++)
	{
		address[2] = i;

		ite_i2c_pre_define_cmd_fastread(&predefine_client, address, 0x100, &buffer);
		for(j=0; j<256; j++) {
			if(buffer[j] != 0xFF)	
			{
				printk("[ITE_update]: Check Error on offset[%x]; EFLASH=%02x \n",j,buffer[j]);
				result=-1;
				goto out;
				//break;
			}
		}
		
		//ite_i2c_pre_define_cmd_fastread(&predefine_client, address, 0x100, buffer);
	}
		
/*	
	for(i=0; i<65536; i++)
	{
		if(buffer[i] != 0xFF)	
		{
			printk("[ITE_update]: Check Error on offset[%x]; EFLASH=%02x \n",i,buffer[i]);
			result=-1;
			break;
		}
	}
*/
out:		
	if(result==0)
	{
		printk("[ITE_update]: Check................OK! \n");
	}

	return result;
}

static int do_erase_all(void)
{
	int err = 0;
	
	printk("[ITE_update]: ERASE................. start. \n");
	cmd_write_enable();
	while((cmd_check_status() & 0x02)!=0x02);
	cmd_erase_all();
	while(cmd_check_status() & 0x01);
	cmd_write_disable();	
	printk("[ITE_update]: ERASE................. OK! \n");
	
	err = do_check();
	if(err == -1)
	{
		return -1;
	}
	return 0;
}

static int do_program(EC_FW_PATH mode)
{
//	unsigned char* ImageBuffer = fw;
	int result=0,i;
	unsigned char payload[5]={0,0,0,0,0};//A2,A1,A0,Data0,Data1
	//add for progress bar ++
	mm_segment_t oldfs;
    char Progress_file_path[] = "/data/pad_update_progress";
    struct file *filePtr = NULL;
    int len = 0, update_progress = 10;
    loff_t pos = 0;
    char temp_progress[3];
    //add for progress bar --
   	
	cmd_write_enable();
	while((cmd_check_status() & 0x02)!=0x02);
	
//	memcpy(&gBuffer, &ImageBuffer, 65536);
	payload[3] = upgrade_fw[0]; //Data 0
	payload[4] = upgrade_fw[1]; //Data 1

	result = ite_i2c_pre_define_cmd_write_with_status(&predefine_client, EFLASH_CMD_AAI_WORD_PROGRAM, 5, payload);

	//Combine command & check status to enhance speed
	printk("[ITE_update]: Program............... \n");
	for(i=2; i<65536;)
	{	
		result = ite_i2c_pre_define_cmd_write_with_status(&predefine_client, EFLASH_CMD_AAI_WORD_PROGRAM, 2, &upgrade_fw[i]);
		while(result & 0x01)
		{
			result=cmd_check_status();
		}
		if((i%4096)==0)
		{
			printk("[ITE_update]: Program i=0x%04x \n",i);
			if(mode == AP)
			{
				//add for progress bar ++ 
				update_progress = update_progress + 5;
				sprintf(temp_progress, "%d", update_progress);
				//printk("[ITE_update] %s: temp_progress = %s \n", __func__, temp_progress);
				filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
				if(!IS_ERR_OR_NULL(filePtr)) {
					oldfs = get_fs();
					set_fs(get_ds());
					pos = 0;
					len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
					set_fs(oldfs);
					filp_close(filePtr, NULL);
				}
				//add for progress bar --
			}
		}
		i+=2;
	}

	cmd_write_disable();
	printk("[ITE_update]: Program...............OK! \n");
	
	return 0;	
}

static int do_verify(void)
{
	int i,j,result=0;
	unsigned char buffer[256];
	unsigned char address[4] = {0,0,0,0}; // Dummy , L , M , H
	int retry_count=0;
    
    printk("[ITE_update]: %s \n",__func__);

    
	for(i=0;i<0x100;i++)
	{
		address[2]=i;
		result = ite_i2c_pre_define_cmd_fastread(&predefine_client, address, 0x100, &buffer);
		while(result==-1)
		{
			result = ite_i2c_pre_define_cmd_fastread(&predefine_client, address, 0x100, &buffer);
			retry_count++;
			if(retry_count > EFLASH_MAX_RETRY_COUNT) {
				printk("[ITE_update]: Do verify over EFLASH_MAX_RETRY_COUNT on address %02x%02x%02x \n",address[3],address[2],address[1]);
				result = -1;
				goto out;
			}	 
		}	
		
		for(j=0; j<256; j++) {
			if(buffer[j] != upgrade_fw[i*0x100 + j])	
			{
				printk("[ITE_update]: Check Error on offset[%x]; EFLASH=%02x \n",j,buffer[j]);
				result=-1;
				goto out;
				//break;
			}
		}		
		 
		
	}
/*	
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer);
	address[2] = 0x20;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0x2000]);
	address[2] = 0x40;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0x4000]);
	address[2] = 0x60;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0x6000]);
	address[2] = 0x80;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0x8000]);
	address[2] = 0xA0;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0xA000]);
	address[2] = 0xC0;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0xC000]);
	address[2] = 0xE0;
	ite_i2c_pre_define_cmd_fastread(address,8192,&buffer[0xE000]);
*/

/*
	for(i=0;i<65536;i++)
	{
		if(buffer[i] != gBuffer[i])
		{
			printk("[ITE_update]: Verify Error on offset[%x] ; file=%02x EFLASH=%02x \n",i,gBuffer[i],buffer[i]);
			result=-1;
			break;
		}
		checksume_verify += buffer[i];
	}
*/	
//	printk("[ITE_update]: checksume_bin =%d checksume_verify = %d\n", checksume_bin, checksume_verify);
	
//	if(checksume_bin == checksume_verify)
//	{
//		printk("[ITE_update]: checksum OK!! \n");
//	}
	
	if(result==0)
	{
		printk("[ITE_update]: Verify................OK! \n");
	}

out:

	return result;
}

static void do_reset(void)
{
	unsigned char tmp1;
	
	printk("[ITE_update]: reset ite chip \n");
	tmp1 = ite_i2c_pre_define_cmd_read_byte(&predefine_client, 0x1F01);
	ite_i2c_pre_define_cmd_writebyte(&predefine_client, 0x1F01, 0x20);
	ite_i2c_pre_define_cmd_writebyte(&predefine_client, 0x1F07, 0x01);
	ite_i2c_pre_define_cmd_writebyte(&predefine_client, 0x1F01, tmp1);
}
#endif

// add by leo for touch notify EC ++
int notify_EC(void)
{
	if(Did_FW_update == 1)
	{
		printk("[ITE] notify_EC : Immediate COLD RESET\n");
		msleep(300);
		rpmsg_send_generic_simple_command(IPCMSG_COLD_RESET, 0);
	}
	printk("[ITE] notify_EC : NOT RESET\n");
	return 0;
}
EXPORT_SYMBOL(notify_EC);
// add by leo for touch notify EC ++

#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
int ite_firmware_upgrade(int path)
{
	struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    int err = 0;
    int i = 0, open_fail = 0;
    char upgrade_tp_version[8];
    char version[8];
    int upgrade_fw_ver_checksum = 0x00;
    u8 buf_recv[15];
    int erase_count = 0, retry_count = 0, get_count = 0;
    int upgrade_status = 0;
    //add for progress bar ++ 
    mm_segment_t oldfs_progress;
    char Progress_file_path[] = "/data/pad_update_progress";
    struct file *filePtr = NULL;
    int update_progress_file = 0, len = 0, update_progress = 1;
    loff_t pos = 0;
    char temp_progress[3];
    //add for progress bar --
    //add for update result ++
    mm_segment_t oldfs_result;
    char result_state_path[] = "/data/ec_upfw_result";
    struct file *resultfilePtr = NULL;
    //add for update result --
    int ret = 0;
     
	wake_lock(&ite_chip_data->wake_lock);
			
	if(path == AP)
	{
		//add for progress bar ++ 
		filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
		if(!IS_ERR_OR_NULL(filePtr)) {
			update_progress_file=1;
			printk("[ITE_update] %s: %s ok to write progress\n", __func__, Progress_file_path);
			filp_close(filePtr, NULL);
		} else if(PTR_ERR(filePtr) == -ENOENT) {
			update_progress_file=0;
			printk("[ITE_update] %s: %s not found\n", __func__, Progress_file_path);
		} else {
			update_progress_file=-1;
			printk("[ITE_update] %s: %s open error\n", __func__, Progress_file_path);
		}
		if(update_progress_file > 0)
		{		
			update_progress = 5;	
			sprintf(temp_progress, "%d", update_progress);
			//printk("[ITE_update] %s: temp_progress = %s \n", __func__, temp_progress);
			filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
			if(!IS_ERR_OR_NULL(filePtr)) {
				oldfs_progress = get_fs();
				set_fs(get_ds());
				pos = 0;
				len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
				set_fs(oldfs_progress);
				filp_close(filePtr, NULL);
			}
		}
		//add for progress bar --
		//add for update result ++
		resultfilePtr = filp_open(result_state_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
		if(!IS_ERR_OR_NULL(resultfilePtr)) {
			oldfs_result = get_fs();
			set_fs(get_ds());
			pos = 0;
			len = resultfilePtr->f_op->write(resultfilePtr, ASUSEC_FW_UPGRADE_INIT, sizeof(char), &pos);
			set_fs(oldfs_result);
			filp_close(resultfilePtr, NULL);
			//printk("[ITE_update] %s: write %s done. \n", __func__, result_state_path);
		}
		//add for update result --
	}
	
	if((path == SELF) || (path == AP) || (path == ETC))
	{
		printk("[ITE_update] %s: file open:/system/etc/firmware/asus_ec_fw.bin \n",__func__);
		for(i = 1; i <= 3; i++)
		{		
			filp = filp_open("/system/etc/firmware/asus_ec_fw.bin", O_RDONLY, 0);
			//filp = filp_open("/data/local/asus_ec_fw.bin", O_RDONLY, 0);
    		if(!IS_ERR_OR_NULL(filp))
    		{
    			oldfs = get_fs();
				set_fs(get_ds());
				
				result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
				if(result < 0) {
			    	printk("[ITE_update] %s: read firmware file failed\n", __func__);
				}
				else
				{
					open_fail = 0;
					i = 4;
					printk("[ITE_update] %s: read firmware file done\n", __func__);
				}
		
				set_fs(oldfs);
				filp_close(filp, NULL);			
			}
			else
			{
				 printk("[ITE_update] %s: open firmware file failed\n", __func__);
				 open_fail++;
				 msleep(5000);
			}
			
			if(open_fail == 3)
			{
				printk("[ITE_update] %s: open firmware file retry failed.\n", __func__);
				wake_unlock(&ite_chip_data->wake_lock);
				return -1;
			}
		}
	}
	else
	{
		printk("[ITE_update] %s: file open:/data/local/ec_fw.bin \n",__func__);
		for(i = 1; i <= 3; i++)
		{		
			filp = filp_open("/data/local/ec_fw.bin", O_RDONLY, 0);
    		if(!IS_ERR_OR_NULL(filp))
    		{
    			oldfs = get_fs();
				set_fs(get_ds());
				
				result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
				if(result < 0) {
			    	printk("[ITE_update] %s: read firmware file failed\n", __func__);
				}
				else
				{
					open_fail = 0;
					i = 4;
					printk("[ITE_update] %s: read firmware file done\n", __func__);
				}
		
				set_fs(oldfs);
				filp_close(filp, NULL);			
			}
			else
			{
				 printk("[ITE_update] %s: open firmware file failed\n", __func__);
				 open_fail++;
				 msleep(3000);
			}
			
			if(open_fail == 3)
			{
				printk("[ITE_update] %s: open firmware file retry failed.\n", __func__);
				wake_unlock(&ite_chip_data->wake_lock);
				return -1;
			}	
		}
	}
	
	if(path == AP)
	{
		//add for progress bar ++ 
		if(update_progress_file > 0)
		{
			update_progress = 10;
			sprintf(temp_progress, "%d", update_progress);
			//printk("[ITE_update] %s: temp_progress = %s \n", __func__, temp_progress);
			filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
			if(!IS_ERR_OR_NULL(filePtr)) {
				oldfs_progress = get_fs();
				set_fs(get_ds());
				pos = 0;
				len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
				set_fs(oldfs_progress);
				filp_close(filePtr, NULL);
			}
		}
		//add for progress bar --
		//add for update result ++
		resultfilePtr = filp_open(result_state_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
		if(!IS_ERR_OR_NULL(resultfilePtr)) {
			oldfs_result = get_fs();
			set_fs(get_ds());
			pos = 0;
			len = resultfilePtr->f_op->write(resultfilePtr, ASUSEC_FW_UPGRADE_PROCESS, sizeof(char), &pos);
			set_fs(oldfs_result);
			filp_close(resultfilePtr, NULL);
			//printk("[ITE_update] %s: write %s done. \n", __func__, result_state_path);
		}
		//add for update result --		
	}
	
	printk("[ITE_update] %s: upgrade firmware verison, %02X, %02X, %02X, %02X, %02X, %2X, %02X, %02X, \n", __func__, upgrade_fw[0xe800], upgrade_fw[0xe801], upgrade_fw[0xe802], upgrade_fw[0xe803], upgrade_fw[0xe804], upgrade_fw[0xe805], upgrade_fw[0xe806], upgrade_fw[0xe807]);
	
	upgrade_tp_version[0] = upgrade_fw[0xe800];
	upgrade_tp_version[1] = upgrade_fw[0xe801];
	upgrade_tp_version[2] = upgrade_fw[0xe802];
	upgrade_tp_version[3] = upgrade_fw[0xe803];
	upgrade_tp_version[4] = upgrade_fw[0xe804];
	upgrade_tp_version[5] = upgrade_fw[0xe805];
	upgrade_tp_version[6] = upgrade_fw[0xe806];
	upgrade_tp_version[7] = upgrade_fw[0xe807];
	
	strncpy(version, &upgrade_tp_version[0], 8);
	printk("[ITE_update]:new upgrade firmware verison=%s.\n", version);
	
	upgrade_fw_ver_checksum = ((upgrade_tp_version[4] << 24) | (upgrade_tp_version[5] << 16 ) | (upgrade_tp_version[6] << 8) | upgrade_tp_version[7]);
//	printk("[ITE_update]: ite_chip_data->version_checksum = %d \n",ite_chip_data->version_checksum);
	printk("[ITE_update]: upgrade_fw_ver_checksum = %d \n",upgrade_fw_ver_checksum);
	
	for(get_count = 0; get_count < 3; get_count++)
	{
		err = ite_get_ec_version();
		if(err == 0)
			break;
	}
		
	mutex_lock(&ite_chip_data->mutex_lock);
	
	if(result > 0)
	{	
		if(path == SELF)
		{	
			
			if(upgrade_fw_ver_checksum > ite_chip_data->version_checksum)
			{
				upgrade_status = 1;
			}
			else
			{
				upgrade_status = 0;
			}
		}
		else
		{
			upgrade_status = 1;
		}
		
		if(upgrade_status)
		{
		
			ite_chip_data->fw_up_process = 1;
			if(ite_chip_data->version_checksum != 0)
			{
				mutex_unlock(&ite_chip_data->mutex_lock);
				printk("[ITE_update] %s: Enter flash mode. \n", __func__);
				err = asus_ecram_i2c_command(&ecram_client, ITE_RAM_ENTER_FLASH_MODE_CMD, buf_recv, 0);
				if(err != 1)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_ENTER_FLASH_MODE_CMD failed \n", __func__, __LINE__);
				}
				if(ite_chip_data->detach_attach == BASE_DETACH)
				{
					msleep(200);
				}
				else
				{
					msleep(100);
					ret = ite_detach_notify_driver(SYSTEM_UPDATE);
					msleep(100);
				}
				mutex_lock(&ite_chip_data->mutex_lock);
			}
						
			do
			{
				ite_i2c_pre_define_cmd_read(&predefine_client, EFLASH_CMD_READ_ID, 3, flash_id);
				printk(KERN_ERR "[ITE_update] %s: flash_id = 0x%x, 0x%x, 0x%x \n", __func__, flash_id[0], flash_id[1], flash_id[2]);
				msleep(100);
				
				if((flash_id[0] == 0xff)&&(flash_id[1] == 0xff)&&(flash_id[2] == 0xfe))
				{
					break;
				}
				else if(retry_count == EFLASH_MAX_RETRY_COUNT)
				{
					goto upgrade_fail;
				}
					
				retry_count++;
				
			}while(1);
			
			for(retry_count = 0; retry_count < EFLASH_MAX_RETRY_COUNT; retry_count++)
			{
				printk("[ITE_update]: do_erase_all start.\n");
				for(erase_count = 0; erase_count < EFLASH_MAX_RETRY_COUNT; erase_count++)
				{
					err = do_erase_all();
					if(err == 0)
					{
						printk("[ITE_update]: do_erase_all pass .\n");
						break;
					}
				}
				printk("[ITE_update]: do_program start.\n");
				if(path == AP)
				{
					do_program(AP);
				}
				else
				{
					do_program(ETC);
				}
					
				err = do_verify();
				if(err == 0)
				{
					printk("[ITE_update]: do_verify pass .\n");
					
					if(path == AP)
					{
						//add for progress bar ++ 
						if(update_progress_file > 0)
						{
							update_progress = 95;
							sprintf(temp_progress, "%d", update_progress);
							//printk("[ITE_update] %s: temp_progress = %s \n", __func__, temp_progress);
							filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
							if(!IS_ERR_OR_NULL(filePtr)) {
								oldfs_progress = get_fs();
								set_fs(get_ds());
								pos = 0;
								len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
								set_fs(oldfs_progress);
								filp_close(filePtr, NULL);
							}
						}
						//add for progress bar --
					}					
					break;
				}
			}
						
			do_reset();
			msleep(2000);
			
			if(upgrade_status)
			{			
				if((factory_mode == 2) && (entry_mode == 1))
					Did_FW_update = 1; // add by leo for touch notify EC ++
			}

			queue_delayed_work(ite_chip_data->asusec_wq, &ite_chip_data->ite_firmware_upgrade_init_work, 1*HZ);
			
			if(path == AP)
			{
				//add for progress bar ++ 
				if(update_progress_file > 0)
				{	
					update_progress = 100;
					sprintf(temp_progress, "%d", update_progress);
					filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
					if(!IS_ERR_OR_NULL(filePtr)) {
						oldfs_progress = get_fs();
						set_fs(get_ds());
						pos = 0;
						len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
						set_fs(oldfs_progress);
						filp_close(filePtr, NULL);
					}
				}
				//add for progress bar --
			}
		
		}
		else
		{
			printk(KERN_INFO "[ITE_update] %s: Current EC firmware is the latest version.\n", __func__);
		}
	}
	else
	{
		printk("[ITE_update] %s: ite chip not upgrage. \n", __func__);
	}
	
	ite_chip_data->fw_up_process = 0;
	mutex_unlock(&ite_chip_data->mutex_lock);
	wake_unlock(&ite_chip_data->wake_lock);
	
	if(path == AP)
	{	
		resultfilePtr = filp_open(result_state_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
		if(!IS_ERR_OR_NULL(resultfilePtr)) {
			oldfs_result = get_fs();
			set_fs(get_ds());
			pos = 0;
			len = resultfilePtr->f_op->write(resultfilePtr, ASUSEC_FW_UPGRADE_SUCCESS, sizeof(char), &pos);
			set_fs(oldfs_result);
			filp_close(resultfilePtr, NULL);
			//printk("[ITE_update] %s: write %s done. \n", __func__, result_state_path);
		}
	}
	
	if((factory_mode == 2) && (entry_mode == 1))
	{
		#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT1664S
			update_touch_fw_called_by_EC();
		#endif
	}
	
	return 0;
	
upgrade_fail:
	if(path == AP)
	{	
		resultfilePtr = filp_open(result_state_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
		if(!IS_ERR_OR_NULL(resultfilePtr)) {
			oldfs_result = get_fs();
			set_fs(get_ds());
			pos = 0;
			len = resultfilePtr->f_op->write(resultfilePtr, ASUSEC_FW_UPGRADE_FAIL, sizeof(char), &pos);
			set_fs(oldfs_result);
			filp_close(resultfilePtr, NULL);
			//printk("[ITE_update] %s: write %s done. \n", __func__, result_state_path);
		}
	}
	ite_chip_data->fw_up_process = 0;
	mutex_unlock(&ite_chip_data->mutex_lock);
	queue_delayed_work(ite_chip_data->asusec_wq, &ite_chip_data->ite_firmware_upgrade_init_work, 1*HZ);
	wake_unlock(&ite_chip_data->wake_lock);
	
	if((factory_mode == 2) && (entry_mode == 1))
	{
		update_touch_fw_called_by_EC();
	}
	
return -1;
}

int ite_self_firmware_upgrade(void)
{
	int ret = 0;
	
	ret = ite_firmware_upgrade(SELF);
	return 0;
}
#endif

static ssize_t ite_proc_fw_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
//	printk("[ITE]%s: now touch firmware:%s. \n", __func__, ite_chip_data->pad_ec_version);
	return sprintf(page, "%s\n", ite_chip_data->pad_ec_version);
}
#ifdef ENABLE_FIRMWARE_UPGRADE
static ssize_t ite_proc_fw_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char messages[80] = {0};
	int ret = 0;
			
	if (len >= 80) {
		printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	   
	printk("[ITE]%s:input command:%s\n", __func__, messages);
	       
	if ('a' == messages[0])
	{
		printk("[ITE]%s: EC firmware update by AP. \n", __func__);
		ret = ite_firmware_upgrade(AP);
	}
	else if ('s' == messages[0])
	{
		printk("[ITE]%s: EC firmware update by self. \n", __func__);
		ret = ite_firmware_upgrade(SELF);
	}
	else if ('e' == messages[0])
	{
		printk("[ITE]%s: EC firmware update by etc. \n", __func__);
		ret = ite_firmware_upgrade(ETC);
	}
	else if ('l' == messages[0])
	{
		printk("[ITE]%s: EC firmware update by local. \n", __func__);
		ret = ite_firmware_upgrade(LOCAL);
	}
	else {
		printk("[ITE]%s:command not support.\n", __func__);
	}
	
	return len;
}
#endif

void ite_create_proc_fw_file(void)
{
	ite_proc_file = create_proc_entry(ITE_PROC_FILE, 0666, NULL);
	if(ite_proc_file){
		ite_proc_file->read_proc = ite_proc_fw_read;
	#ifdef ENABLE_FIRMWARE_UPGRADE	
		ite_proc_file->write_proc = ite_proc_fw_write;
	#endif
	}
	else{
		printk(KERN_ERR "[ITE] %s: proc file create failed!\n", __func__);
	}
}

static ssize_t ite_proc_factory_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
	return sprintf(page, "%s\n", ite_chip_data->pad_ec_version);
}

static ssize_t ite_proc_factory_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char messages[80] = {0};
	int ret = 0;
			
	if (len >= 80) {
		printk(KERN_INFO "[ITE] %s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	   
	printk("[ITE] %s:input command:%s\n", __func__, messages);
	       
	if(!strncmp(messages, "led_0", 5))
	{
		ret = ite_pad_led_function(0);
	}
	else if(!strncmp(messages, "led_1", 5))
	{
		ret = ite_pad_led_function(1);
	}
	else if(!strncmp(messages, "led_2", 5))
	{
		ret = ite_pad_led_function(2);
	}
	else if(!strncmp(messages, "led_3", 5))
	{
		ret = ite_pad_led_function(3);
	}
	else {
		printk("[ITE]%s:command not support.\n", __func__);
	}
	
	return len;
}

void ite_create_proc_factory_file(void)
{
	ite_proc_factory_file = create_proc_entry(ITE_PROC_FACTORY_FILE, 0666, NULL);
	if(ite_proc_factory_file){
		ite_proc_factory_file->read_proc = ite_proc_factory_read;
		ite_proc_factory_file->write_proc = ite_proc_factory_write;
	}
	else{
		printk(KERN_ERR "[ITE] %s: proc file create failed!\n", __func__);
	}
}

/*
 * The signals are AT-style: the low 7 bits are the keycode, and the top
 * bit indicates the state (1 for down, 0 for up).
 */
static inline u8 ite_whichkey(u8 event)
{
	return event & 0x7f;
}

static inline int ite_ispress(u8 event)
{
	return (event & 0x80) ? 0 : 1;
}

void hexdump(u8 *a,int n)
{
	int i,j;
	printk("0x68: %d bytes. \n",n);
	
	for ( j=0; j<n; j+=16 ) {
		printk("%03x: ",j);
		for ( i=j; i<j+16; i++ ) {
			if((i%16)==0x8) {
				printk(" - ");
			}
			if ( i<n ) {
				printk("%02x ", a[i]&255 );
			} else {
				printk("   ");
			}
		}
		//printk("   ");
		printk("\n");
	} /* next j */
}

void hexdump_0x66(u8 *a,int n)
{
	int i,j;
	printk("0x66: %d bytes. \n",n);
	
	for ( j=0; j<n; j+=16 ) {
		printk("%03x: ",j);
		for ( i=j; i<j+16; i++ ) {
			if((i%16)==0x8) {
				printk(" - ");
			}
			if ( i<n ) {
				printk("%02x ", a[i]&255 );
			} else {
				printk("   ");
			}
		}
		//printk("   ");
		printk("\n");
	} /* next j */
}

/*************IO control setting***************/
static int ite_open(struct inode *inode, struct file *file)
{
//	printk( "[ITE]:%s ++ \n",__func__);
	return nonseekable_open(inode, file);		
}

static int ite_release(struct inode *inode, struct file *file)
{
//	printk( "[ITE]:%s ++ \n",__func__);
	return 0;
}

static long ite_i2c_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0, val = 0;
	int ret = 0;
	
//	printk( "[ITE] %s: cmd = 0x%x ++ \n",__func__, cmd);
	
	if (_IOC_TYPE(cmd) != ASUSEC_IOC_MAGIC) 
	{
		printk( "[ITE] %s: ERROR: CMD type not ASUSEC_IOC_MAGIC. \n",__func__);
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) 
	{
		printk( "[ITE] %s: ERROR: CMD type not READ/WRITE. \n",__func__);
		return -EFAULT;
	}
	
	switch (cmd) {
	case ASUSEC_INIT:
		break;
	case ASUSEC_FW_UPDATE_FLAG:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val){
			printk( "[ITE] %s: ASUSEC_FW_UPDATE - ASUSEC_FW_UPGRADE_ON \n",__func__);
			
			ite_chip_data->fwupdate_owner = ite_chip_data->system_owner;
    		printk("[ITE_update] %s: fwupdate_owner = 0x%x. \n", __func__, ite_chip_data->fwupdate_owner);
			
			elan_i2c_bus_enable(0);
			if((ite_chip_data->detach_attach == BASE_ATTACH) && (ite_chip_data->system_owner == SYSTEM_ANDROID))
			{
				if((ite_chip_data->hw_id == HW_ID_ER)||(ite_chip_data->hw_id == HW_ID_ER2)||(ite_chip_data->hw_id == HW_ID_SR1)||(ite_chip_data->hw_id == HW_ID_SR2))
				{
					ret = ite_touchpad_switch(SYSTEM_WINDOWS);
				}
			}
			ite_chip_data->fw_up_process = 1;
			
		}else{
			printk( "[ITE] %s: ASUSEC_FW_UPDATE - ASUSEC_FW_UPGRADE_OFF \n",__func__);
			ite_chip_data->fw_up_process = 0;
		}
		break;
		
	case ASUSEC_FW_UPDATE_PROCESS:
		printk( "[ITE] %s: ASUSEC_FW_UPDATE_PROCESS \n",__func__);
		ret = ite_firmware_upgrade(AP);
		break;
		
	case ASUSEC_TP_CONTROL:
	#ifdef CONFIG_MOUSE_ELAN_TOUCHPAD	
		if (arg == 1){
			printk( "[ITE] %s: ASUSEC_TP_CONTROL - ASUSEC_TP_ON \n",__func__);
			elan_i2c_touchpad_enable(1);
		}else{
			printk( "[ITE] %s: ASUSEC_TP_CONTROL - ASUSEC_TP_OFF \n",__func__);
			elan_i2c_touchpad_enable(0);
		}
	#endif
		break;
		
	case ASUSEC_CHANGE_OWNER:
		mutex_lock(&ite_chip_data->mutex_lock_irq);
		if(ite_chip_data->switch_process == 0)
		{
			printk( "[ITE] %s: change to Win8 event from AP layer. \n",__func__);
			ite_chip_data->switch_process = 1;
			
			ret = ite_system_owner_function();		
			if(ite_chip_data->system_owner == SYSTEM_ANDROID)
			{
				ret = ite_win8_power_switch_status_report();
				ret = ite_atow_early_notify_driver(SYSTEM_WINDOWS);
				if(ite_chip_data->base_power == WIN8_S5 || ite_chip_data->base_power == WIN8_S3)
				{
					ret = ite_turn_on_base_power();
				}
				ret = ite_screen_change_function(SYSTEM_WINDOWS);
				ret = ite_atow_late_notify_driver(SYSTEM_WINDOWS);
			}
			else
			{
				printk("[ITE]: system owner is windows so switch nothing!  \n");
			}
		}
		ite_chip_data->switch_process = 0;
		mutex_unlock(&ite_chip_data->mutex_lock_irq);
		break;
	case ASUSEC_WIFI:
		if (arg == 1){
			printk( "[ITE] %s: ASUSEC_WIFI - ASUSEC_FLIGHT_MODE_ON \n",__func__);
			ret = ite_wifi_led_function(1);
		}else{
			printk( "[ITE] %s: ASUSEC_WIFI - ASUSEC_FLIGHT_MODE_OFF \n",__func__);
			ret = ite_wifi_led_function(0);
		}
		break;
		
	case ASUSEC_DATA_ACCESS:
		
		ret = ite_system_owner_function();		
		if(ite_chip_data->system_owner == SYSTEM_ANDROID)
		{
			printk( "[ITE] %s: ASUSEC_DATA_ACCESS - turn on win8 \n",__func__);
			ret = ite_win8_power_switch_status_report();
			if(ite_chip_data->base_power == WIN8_S5 || ite_chip_data->base_power == WIN8_S3)
			{
				ret = ite_turn_on_base_power();
			}
		}
		else
		{
			printk("[ITE]: ASUSEC_DATA_ACCESS - system owner is windows so nothing!  \n");
		}
				
		break;
	
	case ASUSEC_CPASLOCK_LED:
		if (arg == ASUSEC_CPAS_LED_ON){
			printk( "[ITE] %s: ASUSEC_CPASLOCK_LED - ASUSEC_CPAS_LED_ON \n",__func__);
			ret = ite_keyboard_led_function(CAPS_LOCK);
		}else{
			printk( "[ITE] %s: ASUSEC_CPASLOCK_LED - ASUSEC_CPAS_LED_OFF \n",__func__);
			ret = ite_keyboard_led_function(LEDOFF);
		}
		break;
	
	case ASUSEC_BASE_EC_CHECK:
		printk( "[ITE] %s: ASUSEC_BASE_EC_CHECK. \n",__func__);
		ret = ite_base_ec_fw_check();
		break;
		
	default:
		printk( "[ITE]:%s: incorrect cmd (%d) \n",__FUNCTION__, _IOC_NR(cmd));
		return -EINVAL;
	}
	
	return 0;
	
}

static struct file_operations ite_fops = {
	.owner = THIS_MODULE,
	.open = ite_open,
	.release = ite_release,
	.unlocked_ioctl = ite_i2c_ioctl
};

static struct miscdevice ite_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ite8566",
	.fops = &ite_fops,
};

static int asusdec_open(struct inode *inode, struct file *file)
{
//	printk( "[ITE]:%s ++ \n",__func__);
	return nonseekable_open(inode, file);		
}

static int asusdec_release(struct inode *inode, struct file *file)
{
//	printk( "[ITE]:%s ++ \n",__func__);
	return 0;
}

static long asusdec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0;
	int ret = 0;
	
//	printk( "[ITE]:%s: cmd = 0x%x ++ \n",__func__, cmd);
	
	if (_IOC_TYPE(cmd) != ASUSEC_IOC_MAGIC) 
	{
		printk( "[ITE] %s: ERROR: CMD type not ASUSEC_IOC_MAGIC. \n",__func__);
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) 
	{
		printk( "[ITE] %s: ERROR: CMD type not READ/WRITE. \n",__func__);
		return -EFAULT;
	}
	
	switch (cmd) {
		case ASUSDEC_EC_WAKEUP:
			if (arg == ASUSDEC_EC_OFF){
				printk( "[ITE] %s: ASUSDEC_EC_WAKEUP - ASUSDEC_EC_OFF \n",__func__);
				ret = ite_ram_enable_keyboard_wakeup(0);
			}
			else{
				printk( "[ITE] %s: ASUSDEC_EC_WAKEUP - ASUSDEC_EC_ON \n",__func__);
				ret = ite_ram_enable_keyboard_wakeup(1);
			}
		break;
		
		default:
			printk( "[ITE] %s: incorrect cmd (%d) \n",__FUNCTION__, _IOC_NR(cmd));
			return -EINVAL;
	}
	
	return 0;
}

static struct file_operations asusdec_fops = {
	.owner = THIS_MODULE,
	.open = asusdec_open,
	.release = asusdec_release,
	.unlocked_ioctl = asusdec_ioctl
};

static struct miscdevice asusdec_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asusdec",
	.fops = &asusdec_fops,
};
/*************IO control setting***************/

static int handle_keyboard(struct ite_chip *lm ,u8 *key_fifo)
{
	int i;
	int l_ctrl = 0, l_shite = 0, l_alt = 0, r_ctrl = 0, r_shite = 0, r_alt = 0, home = 0;
	int old_l_ctrl = 0, old_l_shite = 0, old_l_alt = 0, old_r_ctrl = 0, old_r_shite = 0, old_r_alt = 0, old_home = 0;	

	mutex_lock(&ite_chip_data->mutex_lock_irq);
//	printk("[ITE]: key_fifo[3]=0x%x, key_fifo[4]=0x%x, key_fifo[5]=0x%x. \n", key_fifo[3], key_fifo[4], key_fifo[5]);
	for(i = 5; i < 13; i++)
		ite_chip_data->new[i-5] = key_fifo[i];
		
	if((key_fifo[4] != ASUSEC_KEY_RELASE) && (key_fifo[5] != ASUSEC_KEY_RELASE))
	{
		printk("[ITE]: Fu + key press. \n");
		
		if(key_fifo[5] == ASUSEC_FNKEY_SLEEP)
			ite_chip_data->keypad_data.input_keycode = KEY_SLEEP;
		else if(key_fifo[5] == ASUSEC_FNKEY_FLIGHT_MODE)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_FLIGHT_MODE;
		else if(key_fifo[5] == ASUSEC_FNKEY_KEYLED_DOWN)
		{
			ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
			printk("[ITE]: keyboard led down. \n");
		}
		else if(key_fifo[5] == ASUSEC_FNKEY_KEYLED_UP)
		{
			ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
			printk("[ITE]: keyboard led up. \n");
		}
		else if(key_fifo[5] == ASUSEC_FNKEY_BACKLIGHT_DOWN)
			ite_chip_data->keypad_data.input_keycode = KEY_BRIGHTNESSDOWN;
		else if(key_fifo[5] == ASUSEC_FNKEY_BACKLIGHT_UP)
			ite_chip_data->keypad_data.input_keycode = KEY_BRIGHTNESSUP;
		else if(key_fifo[5] == ASUSEC_FNKEY_LCD_OFF)
			ite_chip_data->keypad_data.input_keycode = KEY_SLEEP;
		else if(key_fifo[5] == ASUSEC_FNKEY_LCD_CHG)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_NO_FN;
		else if(key_fifo[5] == ASUSEC_FNKEY_TOUCHPAD)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_TOUCHPAD;
		else if(key_fifo[5] == ASUSEC_FNKEY_MUTE)
			ite_chip_data->keypad_data.input_keycode = KEY_MUTE;
		else if(key_fifo[5] == ASUSEC_FNKEY_VOLUME_DOWN)
			ite_chip_data->keypad_data.input_keycode = KEY_VOLUMEDOWN;
		else if(key_fifo[5] == ASUSEC_FNKEY_VOLUME_UP)
			ite_chip_data->keypad_data.input_keycode = KEY_VOLUMEUP;
		else if(key_fifo[5] == ASUSEC_FNKEY_POWER_4GEAR)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_NO_FN;
		else if(key_fifo[5] == ASUSEC_FNKEY_SPLENDID)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_SPLENDID;
		else if(key_fifo[5] == ASUSEC_FNKEY_CAMERA)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_LAUNCH_CAMERA;
		else if(key_fifo[5] == ASUSEC_FNKEY_AUTO_LIGHT)
			ite_chip_data->keypad_data.input_keycode = ASUSEC_KEYCODE_BRIGHTNESS_AUTO;
		else if(key_fifo[5] == ASUSEC_FNKEY_PAGEUP)
			ite_chip_data->keypad_data.input_keycode = KEY_PAGEUP;
		else if(key_fifo[5] == ASUSEC_FNKEY_PAGEDOWN)
			ite_chip_data->keypad_data.input_keycode = KEY_PAGEDOWN;
		else if(key_fifo[5] == ASUSEC_FNKEY_HOME)
			ite_chip_data->keypad_data.input_keycode = KEY_HOME;
		else if(key_fifo[5] == ASUSEC_FNKEY_END)
			ite_chip_data->keypad_data.input_keycode = KEY_END;
		else
			ite_chip_data->keypad_data.input_keycode = KEY_RESERVED;
		
		if(ite_chip_data->input_dev_status == 1)
		{
			input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 1);
			input_sync(ite_chip_data->input_dev); 
			input_report_key(ite_chip_data->input_dev, ite_chip_data->keypad_data.input_keycode, 0);
			input_sync(ite_chip_data->input_dev); 
		}
		
		mutex_unlock(&ite_chip_data->mutex_lock_irq);
		return 0;	
	}
		
		
	if((ite_chip_data->modifier_key_press != key_fifo[3]) && (ite_chip_data->modifier_repeat_key == 1))
	{		
		//Modifier Keys release
		
		ite_chip_data->keypad_data.value = 0;
		ite_chip_data->modifier_repeat_key = 0;
						
		if((ite_chip_data->modifier_key_press & 0x01)&& ((key_fifo[3] & 0x01) == 0)) old_l_ctrl = 1;
		if((ite_chip_data->modifier_key_press & 0x02)&& ((key_fifo[3] & 0x02) == 0)) old_l_shite = 1;
		if((ite_chip_data->modifier_key_press & 0x04)&& ((key_fifo[3] & 0x04) == 0)) old_l_alt = 1;
		if((ite_chip_data->modifier_key_press & 0x08)&& ((key_fifo[3] & 0x08) == 0)) old_home = 1;
		if((ite_chip_data->modifier_key_press & 0x10)&& ((key_fifo[3] & 0x10) == 0)) old_r_ctrl = 1;
		if((ite_chip_data->modifier_key_press & 0x20)&& ((key_fifo[3] & 0x20) == 0)) old_r_shite = 1;
		if((ite_chip_data->modifier_key_press & 0x40)&& ((key_fifo[3] & 0x40) == 0)) old_r_alt = 1;
		
		printk("[ITE]: Release: modifier_key_press=0x%x, key_fifo[3]=0x%x, %d,%d,%d,%d,%d,%d,%d \n", ite_chip_data->modifier_key_press, key_fifo[3], old_l_ctrl, old_l_shite, old_l_alt, old_r_ctrl, old_r_shite, old_r_alt, old_home);
		
		if(ite_chip_data->input_dev_status == 1)
		{
			if(old_l_ctrl)
			{
				printk("[ITE]: KEY_LEFTCTRL release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_LEFTCTRL, 0);
			}
			if(old_l_shite)
			{
				printk("[ITE]: KEY_LEFTSHIFT release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_LEFTSHIFT, 0);
			}
			if(old_l_alt)
			{
				printk("[ITE]: KEY_LEFTALT release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_LEFTALT, 0);
			}
			if(old_r_ctrl)
			{
				printk("[ITE]: KEY_RIGHTCTRL release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_RIGHTCTRL, 0);
			}
			if(old_r_shite)
			{
				printk("[ITE]: KEY_RIGHTSHIFT release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_RIGHTSHIFT, 0);
			}
			if(old_r_alt)
			{
				printk("[ITE]: KEY_RIGHTALT release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_RIGHTALT, 0);
			}
			if(old_home)
			{
				printk("[ITE]: KEY_HOMEPAGE release.\n");
				input_report_key(ite_chip_data->input_dev, KEY_HOMEPAGE, 0);
			}
		}
		
		ite_chip_data->modifier_key_release = key_fifo[3];
		if((old_l_ctrl == 1)||(old_l_shite == 1)||(old_l_alt == 1)||(old_r_ctrl == 1)||(old_r_shite == 1)||(old_r_alt == 1)||(old_home == 1))
		{
			if(ite_chip_data->input_dev_status == 1)
			{
				printk("[ITE]: release input sync.\n");
				input_sync(ite_chip_data->input_dev);
			}
		}
	}
	
	if(key_fifo[3] != ASUSEC_KEY_RELASE)
	{		
		//Modifier Keys press
		
		ite_chip_data->keypad_data.value = 1;
		ite_chip_data->modifier_repeat_key = 1;
							
		if(key_fifo[3] & 0x01) l_ctrl = 1;
		if(key_fifo[3] & 0x02) l_shite = 1;
		if(key_fifo[3] & 0x04) l_alt = 1;
		if(key_fifo[3] & 0x08) home = 1;
		if(key_fifo[3] & 0x10) r_ctrl = 1;
		if(key_fifo[3] & 0x20) r_shite = 1;
		if(key_fifo[3] & 0x40) r_alt = 1;
				
		if((ite_chip_data->modifier_key_release & 0x01) && (ite_chip_data->modifier_key_press & 0x01)) l_ctrl = 0;
		if((ite_chip_data->modifier_key_release & 0x02) && (ite_chip_data->modifier_key_press & 0x02)) l_shite = 0;
		if((ite_chip_data->modifier_key_release & 0x04) && (ite_chip_data->modifier_key_press & 0x04)) l_alt = 0;
		if((ite_chip_data->modifier_key_release & 0x08) && (ite_chip_data->modifier_key_press & 0x08)) home = 0;
		if((ite_chip_data->modifier_key_release & 0x10) && (ite_chip_data->modifier_key_press & 0x10)) r_ctrl = 0;
		if((ite_chip_data->modifier_key_release & 0x20) && (ite_chip_data->modifier_key_press & 0x20)) r_shite = 0;
		if((ite_chip_data->modifier_key_release & 0x40) && (ite_chip_data->modifier_key_press & 0x40)) r_alt = 0;
			
		printk("[ITE]: Press: modifier_key_release=0x%x, key_fifo[3]=0x%x, %d,%d,%d,%d,%d,%d,%d \n", ite_chip_data->modifier_key_release, key_fifo[3], l_ctrl, l_shite, l_alt, r_ctrl, r_shite, r_alt, home);
		
		if(ite_chip_data->input_dev_status == 1)
		{
			if(l_ctrl)
			{
				printk("[ITE]: KEY_LEFTCTRL press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_LEFTCTRL, 1);
			}
			if(l_shite)
			{
				printk("[ITE]: KEY_LEFTSHIFT press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_LEFTSHIFT, 1);
			}
			if(l_alt)
			{
				printk("[ITE]: KEY_LEFTALT press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_LEFTALT, 1);
			}
			if(r_ctrl)
			{
				printk("[ITE]: KEY_RIGHTCTRL press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_RIGHTCTRL, 1);
			}
			if(r_shite)
			{
				printk("[ITE]: KEY_RIGHTSHIFT press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_RIGHTSHIFT, 1);
			}
			if(r_alt)
			{
				printk("[ITE]: KEY_RIGHTALT press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_RIGHTALT, 1);
			}
			if(home)
			{
				printk("[ITE]: KEY_HOMEPAGE press.\n");
				input_report_key(ite_chip_data->input_dev, KEY_HOMEPAGE, 1);
			}
			
			ite_chip_data->modifier_key_press = key_fifo[3];
			if((l_ctrl == 1)||(l_shite == 1)||(l_alt == 1)||(r_ctrl == 1)||(r_shite == 1)||(r_alt == 1)||(home == 1))
			{
				printk("[ITE]: press input sync.\n");
				input_sync(ite_chip_data->input_dev);
			}
		}
	}
		
	for (i = 0; i < 8; i++) {
	
		if (ite_chip_data->old[i] > 3 && memscan(ite_chip_data->new, ite_chip_data->old[i], 8) == ite_chip_data->new + 8) {
			if (i2c_kbd_keycode[ite_chip_data->old[i]])
			{
				if(ite_chip_data->input_dev_status == 1)
				{
					input_report_key(ite_chip_data->input_dev, i2c_kbd_keycode[ite_chip_data->old[i]], 0);
				}
			}
			else
				printk("[ITE]: Unknown key (scancode %#x) released.\n", ite_chip_data->old[i]);
		}

		if (ite_chip_data->new[i] > 3 && memscan(ite_chip_data->old, ite_chip_data->new[i], 8) == ite_chip_data->old + 8 ) {
			if (i2c_kbd_keycode[ite_chip_data->new[i]])
			{	
				if(ite_chip_data->input_dev_status == 1)
				{
					input_report_key(ite_chip_data->input_dev, i2c_kbd_keycode[ite_chip_data->new[i]], 1);
				}
			}
			else
				printk("[ITE]: Unknown key (scancode %#x) released.\n", ite_chip_data->new[i]);
		}
	}
	if(ite_chip_data->input_dev_status == 1)
	{
		input_sync(ite_chip_data->input_dev);
	}

	memcpy(ite_chip_data->old, ite_chip_data->new, 8);	
	mutex_unlock(&ite_chip_data->mutex_lock_irq);

	return 0;	
}

static int handle_special_event(struct ite_chip *lm ,u8 *key_fifo)
{		
	int ret = 0;
//	printk("[ITE]: %s: key_fifo[3] = 0x%x. ++ \n", __func__, key_fifo[3]);
	mutex_lock(&ite_chip_data->mutex_lock_irq);
	
	if((key_fifo[3] & ASUSEC_KEY_BASE_DETACH) == ASUSEC_KEY_BASE_DETACH)
	{
		printk("[ITE]: ****** pad and docking detach event. ******\n");
		
		memset(ite_chip_data->dock_ec_version, 0, 32);
		ite_chip_data->detach_attach = BASE_DETACH; //base detach
		ite_base_switch_status_report();
		ret = ite_win8_power_switch_status_report();
		ret = ite_system_owner_function();

		if(ite_chip_data->input_dev_status == 1)
		{
			input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_BASE_DETACH, 1);
			input_sync(ite_chip_data->input_dev); 
			input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_BASE_DETACH, 0);
			input_sync(ite_chip_data->input_dev);
		}
		
		ret = ite_detach_notify_driver(SYSTEM_NONE);
		ret = asusec_input_device_destroy();
		
		if(usb_switch_mode_flag)
		{
			ret = ite_usb_switch_mode(HOST);
		}
		mutex_unlock(&ite_chip_data->mutex_lock_irq);
		return 0;
	}	
	else if((key_fifo[3] & ASUSEC_KEY_BASE_ATTACH) == ASUSEC_KEY_BASE_ATTACH)
	{
		printk("[ITE]: ****** pad and docking attach event ******\n");
		ite_chip_data->detach_attach = BASE_ATTACH; //Base Attach
						
		ite_base_switch_status_report();
		ret = ite_system_owner_function();
		ret = asusec_input_device_create();
		ret = ite_win8_power_switch_status_report();
		
		if(ite_chip_data->input_dev_status == 1)
		{
			input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_BASE_ATTACH, 1);
			input_sync(ite_chip_data->input_dev); 
			input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_BASE_ATTACH, 0);
			input_sync(ite_chip_data->input_dev);
		}
		
		if(ite_chip_data->system_owner == SYSTEM_ANDROID)
			ret = ite_attach_notify_driver(SYSTEM_ANDROID);
		else if(ite_chip_data->system_owner == SYSTEM_WINDOWS)
			ret = ite_attach_notify_driver(SYSTEM_WINDOWS);
		else
			ret = ite_attach_notify_driver(SYSTEM_ANDROID);
			
		if(ite_chip_data->base_power == WIN8_S0)
		{
			ret = ite_read_win8_test_mode();
		}
		else
		{
			printk("[ITE] %s: Win8 without turn on!! \n",__func__);
		}
		
		ret = ite_get_ec_version();
	}
	
	if((key_fifo[3] & ASUSEC_KEY_OWNER_ANDROID) == ASUSEC_KEY_OWNER_ANDROID)
	{
		printk("[ITE]: ****** switch to android. ******\n");
		ret = ite_win8_power_switch_status_report();
		ret = ite_system_owner_function();

		if(ite_chip_data->detach_attach == BASE_ATTACH)
		{	
			if(ite_chip_data->input_dev_status == 1)
			{
				input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_SWITCH_ANDROID, 1);
				input_sync(ite_chip_data->input_dev); 
				input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_SWITCH_ANDROID, 0);
				input_sync(ite_chip_data->input_dev);
			}			
			ret = ite_wtoa_late_notify_driver(SYSTEM_ANDROID);		
			ret = asusec_input_device_create();
		}
	}
		
	if(ite_chip_data->detach_attach == BASE_ATTACH)
	{
		if((key_fifo[3] & ASUSEC_KEY_GEMINI_KEY) == ASUSEC_KEY_GEMINI_KEY)
		{
			printk("[ITE]: ****** android and windows switch (Gemini Key event) ******\n");
			
			ret = ite_win8_power_switch_status_report();
			
			if(ite_chip_data->input_dev_status == 1)
			{	
				input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_MODE_SWITCH, 1);
				input_sync(ite_chip_data->input_dev); 
				input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_MODE_SWITCH, 0);
				input_sync(ite_chip_data->input_dev);
			}
				
			if((factory_mode == 2) && (entry_mode == 1))
			{
				if(ite_chip_data->switch_process == 0)
				{
					printk("[ITE]: Factory image Gemini Key event. \n");
					ite_chip_data->switch_process = 1;
					
					ret = ite_system_owner_function();		
					if(ite_chip_data->system_owner == SYSTEM_ANDROID)
					{
						ret = ite_win8_power_switch_status_report();
						ret = ite_atow_early_notify_driver(SYSTEM_WINDOWS);
						if(ite_chip_data->base_power == WIN8_S5 || ite_chip_data->base_power == WIN8_S3)
						{
							ret = ite_turn_on_base_power();
						}
						ret = ite_screen_change_function(SYSTEM_WINDOWS);
						ret = ite_atow_late_notify_driver(SYSTEM_WINDOWS);
					}
					else
					{
						printk("[ITE]: android and windows switch nothing ++ \n");
					}
				}
				ite_chip_data->switch_process = 0;
			}
		}
		//if((key_fifo[3] & ASUSEC_KEY_EC_SWITCH) == ASUSEC_KEY_EC_SWITCH)
		//{
		//	printk("[ITE]: ****** attach switch to windows (EC auto) ******\n");
		//	
		//	ret = ite_win8_power_switch_status_report();
		//	if(ite_chip_data->switch_process == 0)
		//	{
		//		printk("[ITE]: EC auto Gemini Key event. \n");
		//		ite_chip_data->switch_process = 1;
		//		
		//		ret = ite_system_owner_function();		
		//		ret = ite_atow_early_notify_driver(SYSTEM_WINDOWS);
		//		ret = ite_screen_change_function(SYSTEM_WINDOWS); //notify EC switch
		//		ret = ite_atow_late_notify_driver(SYSTEM_WINDOWS);
		//	}
		//	ite_chip_data->switch_process = 0;
		//}		
		if((key_fifo[3] & ASUSEC_KEY_OWNER_WINDOWS) == ASUSEC_KEY_OWNER_WINDOWS)
		{
			printk("[ITE]: ****** switch to windows 8.****** \n");
			ret = ite_system_owner_function();
			ret = ite_win8_power_switch_status_report();
			ret = ite_atow_late_notify_driver(SYSTEM_WINDOWS); 
		}
		if((key_fifo[3] & ASUSEC_KEY_BASE_STATUS) == ASUSEC_KEY_BASE_STATUS)
		{
			printk("[ITE]: ****** Base system status change.****** \n");
			ret = ite_win8_power_switch_status_report();
			if((ite_chip_data->base_power == WIN8_S3)||(ite_chip_data->base_power == WIN8_S5))
				ite_base_system_change_notify_driver(ite_chip_data->base_power);
		}
	}
	
	if((key_fifo[3] & ASUSEC_KEY_WAKEUP_SOC) == ASUSEC_KEY_WAKEUP_SOC)
	{
		printk("[ITE]: ****** Wake UP SOC ****** \n");
		ret = ite_win8_power_switch_status_report();
		ret = ite_system_owner_function();

		if(ite_chip_data->detach_attach == BASE_ATTACH)
		{	
			if(ite_chip_data->input_dev_status == 1)
			{
				input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_SWITCH_ANDROID, 1);
				input_sync(ite_chip_data->input_dev); 
				input_report_key(ite_chip_data->input_dev, ASUSEC_KEYCODE_SWITCH_ANDROID, 0);
				input_sync(ite_chip_data->input_dev);
			}			
			ret = ite_wtoa_late_notify_driver(SYSTEM_ANDROID);		
			ret = asusec_input_device_create();
		}
	}
	
	mutex_unlock(&ite_chip_data->mutex_lock_irq);
	return 0;
}

static int handle_light_sensor_event(struct ite_chip *lm ,u8 *msg_fifo)
{
	mutex_lock(&ite_chip_data->mutex_lock_irq);
	ite_chip_data->als_lux_data = (msg_fifo[4] << 8) | msg_fifo[3];	
	ite_chip_data->als_raw_data = (msg_fifo[6] << 8) | msg_fifo[5];
	ite_chip_data->als_level = msg_fifo[7];
	printk("[ITE]: light sensor level = %d, lux = %d, raw_data=0x%x. \n", ite_chip_data->als_level, ite_chip_data->als_lux_data, ite_chip_data->als_raw_data);
	ite_chip_data->als_status = 0x01;
	
#ifdef CONFIG_SENSORS_ALS3010
	als3010_work_function(ite_chip_data->als_lux_data, ite_chip_data->als_level, ite_chip_data->als_raw_data);
#endif
	mutex_unlock(&ite_chip_data->mutex_lock_irq);
	return 0;
}

static int handle_charger_ic_event(struct ite_chip *lm ,u8 *msg_fifo)
{
	mutex_lock(&ite_chip_data->mutex_lock_irq);
#ifdef CONFIG_TX201LA_BATTERY_EC
	asus_pad_batt_set_charger_data(msg_fifo[3], msg_fifo[4]);
#endif
	mutex_unlock(&ite_chip_data->mutex_lock_irq);
	return 0;
}

static int handle_charger_lid_event(struct ite_chip *lm ,u8 *msg_fifo)
{
	int base_charger_status;
	int lid_status;
	
	mutex_lock(&ite_chip_data->mutex_lock_irq);	
	base_charger_status = msg_fifo[4] & 0x0F;
	lid_status = msg_fifo[4] & 0x30;
	
	printk("[ITE]: %s: msg_fifo[4] = 0x%x, base_charger_status =0x%x, lid_status =0x%x. \n",__func__, msg_fifo[4], base_charger_status, lid_status);
	
	if(base_charger_status != 0x00)
	{
		pad_bat_set_base_ac_status(base_charger_status);
	}
	if(lid_status != 0x00)
	{
		lid_status_report(lid_status);
	}
	
	mutex_unlock(&ite_chip_data->mutex_lock_irq);
	return 0;
}

static int handle_irq1_gauge_event(struct ite_chip *lm ,u8 *msg_fifo)
{
	mutex_lock(&ite_chip_data->mutex_lock_irq);
#ifdef CONFIG_TX201LA_BATTERY_EC
	batt_gauge_fw_result_cb(msg_fifo[0]);	
#endif
	mutex_unlock(&ite_chip_data->mutex_lock_irq);
	return 0;
}

static int handle_irq1_base_system_status(struct ite_chip *lm ,u8 *key_fifo)
{
	int ret = 0;
	
	mutex_lock(&ite_chip_data->mutex_lock_irq);
	printk("[ITE]: ====== Base system status change ====== \n");
	ret = ite_win8_power_switch_status_report();
	if((ite_chip_data->base_power == WIN8_S3)||(ite_chip_data->base_power == WIN8_S5))
		ite_base_system_change_notify_driver(ite_chip_data->base_power);
	
	mutex_unlock(&ite_chip_data->mutex_lock_irq);
	return 0;
}

/*
 * We cannot use I2C in interrupt context, so we just schedule work.
 */
static irqreturn_t ite_irq_0(int irq, void *data)
{
//	printk("[ITE]: %s ++ \n",__func__);
	
	event = kzalloc(sizeof(struct work_struct), GFP_KERNEL);
	INIT_WORK(event, ite_work_0x68);
	
	wake_lock_timeout(&ite_chip_data->wake_lock_timeout, 1*HZ);
	
	disable_irq_nosync(ite_chip_data->irq0);
		
	schedule_work(event);
//	printk("[ITE]: %s -- \n",__func__);
	return IRQ_HANDLED;
}

static int ite_read_0x68_buf(struct i2c_client *client,u8 *buf)
{
	int ret;
	struct i2c_msg msg[1];

	mutex_lock(&ite_chip_data->mutex_lock);

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD | I2C_M_RECV_LEN;
	msg[0].len = ITE_MAX_INPUT_LENGTH;
	msg[0].buf = buf;
	
	ret = i2c_transfer(client->adapter, msg, 1);
	
//	hexdump(buf, ITE_MAX_INPUT_LENGTH);
	mutex_unlock(&ite_chip_data->mutex_lock);
	
	return 0;
}

static void ite_work_0x68(struct work_struct *work)
{
	int ret = 0;
	u8 i2c_data[ITE_MAX_INPUT_LENGTH];
	
//	printk("[ITE]: %s ++ \n",__func__);


	ret = ite_read_0x68_buf(ite_chip_data->client,i2c_data);
	
	if(i2c_data[2] == ASUSEC_IRQ0_ID_KEYBOARD)
	{
		ret = handle_keyboard(ite_chip_data, i2c_data);
	} 
	else if(i2c_data[2] == ASUSEC_IRQ0_EVENT_1)
	{
		printk("[ITE]: Report ID  = 0x03 \n");
	//	handle_keyboard_function_1(ite_chip_data, i2c_data);
	}
	else if(i2c_data[2] == ASUSEC_IRQ0_EVENT_2)
	{
		printk("[ITE]: Report ID  = 0x04 \n");
	//	handle_keyboard_function_2(ite_chip_data, i2c_data);
	}
	else if(i2c_data[2] == ASUSEC_IRQ0_GEMINI_EVENT)
	{
		if(i2c_data[3] != 0x00)
		{
 			ret = handle_special_event(ite_chip_data, i2c_data);
 		}
 		if(i2c_data[4] != 0x00)
 		{
	 		ret = handle_charger_lid_event(ite_chip_data, i2c_data);
	 	}
	}
	else if(i2c_data[2] == ASUSEC_IRQ0_LIGHT_SENSOR)
	{
		ret = handle_light_sensor_event(ite_chip_data, i2c_data);
	}
	else if(i2c_data[2] == ASUSEC_IRQ0_CHARGER_IC)
	{
	//	printk("[ITE]: Report ID = 0x0B, charger IC \n");
		ret = handle_charger_ic_event(ite_chip_data, i2c_data);
	}
	else
	{
		printk("[ITE]: NO SUCH EVENT!! buffer[2]=0x%x \n", i2c_data[2]);
	}
	
	kfree(event);
	enable_irq(ite_chip_data->irq0);
//	printk("[ITE]: %s -- \n",__func__);
}

static irqreturn_t ite_irq_1(int irq, void *data)
{
//	printk("[ITE]: %s ++ \n",__func__);
	
	disable_irq_nosync(ite_chip_data->irq1);
	schedule_work(&ite_chip_data->work_0x66);
	return IRQ_HANDLED;
}

static int ite_read_0x66_buf(struct i2c_client *client,u8 *buf)
{
	int ret;
	struct i2c_msg msg[1];

	mutex_lock(&ite_chip_data->mutex_lock);

	msg[0].addr = client->addr;
	msg[0].flags = I2C_M_RD | I2C_M_RECV_LEN;
	msg[0].len = ITE_IRQ1_MAX_INPUT_LENGTH;
	msg[0].buf = buf;
	
	ret = i2c_transfer(client->adapter,msg,1);
	
	hexdump_0x66(buf, ITE_IRQ1_MAX_INPUT_LENGTH);
	mutex_unlock(&ite_chip_data->mutex_lock);
	
	return 0;
}

static void ite_work_0x66(struct work_struct *work)
{
	int ret = 0;
	u8 i2c_data[ITE_MAX_INPUT_LENGTH];
	
//	printk("[ITE]: %s ++ \n",__func__);
	
	ret = ite_read_0x66_buf(&ecram_client, i2c_data);
		
	if((i2c_data[0] & ASUSEC_IRQ1_BASE_STATUS) == ASUSEC_IRQ1_BASE_STATUS)
	{
		ret = handle_irq1_base_system_status(ite_chip_data, i2c_data);
	}
	
	if((i2c_data[0] & ASUSEC_IRQ1_COMPARE_FAIL) == ASUSEC_IRQ1_COMPARE_FAIL)
	{
		ret = handle_irq1_gauge_event(ite_chip_data, i2c_data);
	}
	else if((i2c_data[0] & ASUSEC_IRQ1_COMPARE_PASS) == ASUSEC_IRQ1_COMPARE_PASS)
	{
		ret = handle_irq1_gauge_event(ite_chip_data, i2c_data);
	}
	else if((i2c_data[0] & ASUSEC_IRQ1_I2C_FAIL) == ASUSEC_IRQ1_I2C_FAIL)
	{
		ret = handle_irq1_gauge_event(ite_chip_data, i2c_data);
	}
					
	enable_irq(ite_chip_data->irq1);
}

static ssize_t ite_show_disable(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", !ite_chip_data->kp_enabled);
}

static ssize_t ite_set_disable(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret;
	unsigned long i;
	
	ret = strict_strtoul(buf, 10, &i);
	
	mutex_lock(&ite_chip_data->mutex_lock);
	ite_chip_data->kp_enabled = !i;
	mutex_unlock(&ite_chip_data->mutex_lock);
	
	return count;
}

void getHIDDescriptor(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msg[2];
	u8 buf1[2]={0xf1,0x00};
	u8 buf2[30];
	int i;
	
	printk("[ITE]: %s addr:0x%x ++ \n",__func__, ite_chip_data->client->addr);
	
	msg[0].addr = ite_chip_data->client->addr;
	msg[0].flags = 0; //I2C_M_WR;
	msg[0].len = 2;
	msg[0].buf = &buf1[0];
	
	msg[1].addr = ite_chip_data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &buf2[0];
	msg[1].len = 30;
	
	ret=i2c_transfer(client->adapter,msg,2);
	
	for (i = 0; i < 30; i++)
	{
		printk("[ITE]: buf[%d]=0x%x \n", i, buf2[i]);
	}
	hexdump(buf2,30);	
}

void getReportDescriptor(struct i2c_client *client)
{
	int ret;
	struct i2c_msg msg[2];
	u8 buf1[2]={0xf2,0x00};
	u8 buf2[117];
	
	printk("[ITE]: %s addr:0x%x ++ \n",__func__, ite_chip_data->client->addr);
	
	msg[0].addr = ite_chip_data->client->addr;
	msg[0].flags = 0; //I2C_M_WR;
	msg[0].len = 2;
	msg[0].buf = &buf1[0];
	
	msg[1].addr = ite_chip_data->client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = &buf2[0];
	msg[1].len = 117;
	
	ret=i2c_transfer(client->adapter,msg,2);
	hexdump(buf2,117);
}

void sendCmd(struct i2c_client *client,u8 *cmd,u8 rw)
{
	int ret = 0;
	int len = cmd[1]+1;
	
	
	printk("[ITE]: %s: sendCmd=====> len=%x \n", __func__, len);
	//hexdump(cmd[1],len);	
	//test_flag =1;
	
	if(rw==0) {
		ret = i2c_smbus_write_i2c_block_data(client,cmd[0],len,&cmd[1]);
	} else {
	}
	
	if (ret < 0) {
		printk("[ITE]: failed writing the cmd=%x subcmd=%x \n",cmd[0],cmd[2]);
		//return ret;
	} else {
		printk("[ITE]: success writing the cmd=%x subcmd=%x \n",cmd[0],cmd[2]);
	}
}

int ite_i2c_event(struct input_dev *dev, unsigned int type, unsigned int code, int value)
{
	printk("[ITE]: %s ++ \n",__func__);
	
	input_sync(ite_chip_data->input_dev);
	return 0;
}

static int asus_ec_i2c_command(struct i2c_client *client, int command,
				unsigned char *buf_recv, int data_len)
{
	const u8 *cmd = NULL;
	unsigned char *rec_buf = buf_recv;
	int ret;
	int tries = ITE_I2C_COMMAND_TRIES;
	int length = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;
	int count =0;
	
	mutex_lock(&ite_chip_data->mutex_lock);
	
	switch (command) {
	case ITE_HID_DESCR_LENGTH_CMD:
	case ITE_HID_DESCR_CMD:
		cmd = ite_hid_descriptor_cmd;
		length = sizeof(ite_hid_descriptor_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;

	case ITE_HID_REPORT_DESCR_CMD:
		cmd = ite_hid_report_descr_cmd;
		length = sizeof(ite_hid_report_descr_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
		
	case ITE_HID_POWER_ON_CMD:
		cmd = ite_hid_power_on_cmd;
		length = sizeof(ite_hid_power_on_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_RESET_CMD:
		cmd = ite_hid_reset_cmd;
		length = sizeof(ite_hid_reset_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_PAD_BATTERY_CMD:
		cmd = ite_hid_pad_battery_cmd;
		length = sizeof(ite_hid_pad_battery_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
		
	case ITE_HID_DOCK_BATTERY_CMD:
		cmd = ite_hid_dock_battery_cmd;
		length = sizeof(ite_hid_dock_battery_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
		
	case ITE_HID_CABLE_STATUS_NOTIFY_CMD:		
		cmd = ite_hid_cable_status_notify_cmd;
		length = sizeof(ite_hid_cable_status_notify_cmd);
		msg_num = 1;
		
	//	for(i==0; i < 12; i++)
	//	{
	//		printk("[ITE] ite_hid_cable_status_notify_cmd[%d] = 0x%x \n", i, ite_hid_cable_status_notify_cmd[i]);
	//	}
		
		break;
		
	case ITE_HID_VERIFY_CHARGER_CMD:
		cmd = ite_hid_verify_charger_cmd;
		length = sizeof(ite_hid_verify_charger_cmd);
		msg_num = 1;		
		break;
		
	case ITE_HID_ATTACH_DETACH_CMD:
		cmd = ite_hid_attach_detach_cmd;
		length = sizeof(ite_hid_attach_detach_cmd);
		msg_num = 1;
		break;
		
	case ITE_HID_ANDROID_POWER_STATUS_CMD:
		cmd = ite_hid_android_power_status_cmd;
		length = sizeof(ite_hid_android_power_status_cmd);
		msg_num = 1;
		
	//	for(i==0; i < 12; i++)
	//	{
	//		printk("[ITE] ite_hid_android_power_status_cmd[%d] = 0x%x \n", i, ite_hid_android_power_status_cmd[i]);
	//	}
		break;
		
	case ITE_HID_SYSTEM_OWNER_CMD:
		cmd = ite_hid_system_owner_cmd;
		length = sizeof(ite_hid_system_owner_cmd);
		msg_num = 1;		
		break;
			
	case ITE_HID_SCREEN_CHANGE_NOTIFY_CMD:
		cmd = ite_hid_screen_change_notify_cmd;
		length = sizeof(ite_hid_screen_change_notify_cmd);
		msg_num = 1;
		
	//	for(i==0; i < 12; i++)
	//	{
	//		printk("[ITE] ite_hid_screen_change_notify_cmd[%d] = 0x%x \n", i, ite_hid_screen_change_notify_cmd[i]);
	//	}
		
		break;
	
	case ITE_HID_BASE_SYSTEM_STATUS_CMD:
		cmd = ite_hid_base_system_status_cmd;
		length = sizeof(ite_hid_base_system_status_cmd);
		msg_num = 1;		
		break;
	
	case ITE_HID_READ_EC_REGISTER_CMD:
		cmd = ite_hid_read_ec_register_cmd;
		length = sizeof(ite_hid_read_ec_register_cmd);
		msg[1].len = data_len;
		msg_num = 2;		
		break;
	
//	case ITE_HID_LED_NOTIFY_CMD:
//		cmd = ite_hid_led_notify_cmd;
//		length = sizeof(ite_hid_led_notify_cmd);
//		msg_num = 1;
//		break;
		
	case ITE_HID_READ_CHARGER_IC_STATUS_CMD:
		cmd = ite_hid_read_charger_ic_status_cmd;
		length = sizeof(ite_hid_read_charger_ic_status_cmd);
		msg[1].len = data_len;
		msg_num = 2;		
		break;
		
	case ITE_HID_ALS_CTL_CMD:
		cmd = ite_hid_als_ctl_cmd;
		length = sizeof(ite_hid_als_ctl_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_VERIFY_ALS_STATUS_CMD:
		cmd = ite_hid_verify_als_status_cmd;
		length = sizeof(ite_hid_verify_als_status_cmd);
		msg_num = 1;		
		break;
		
	case ITE_HID_PWM_CTL_CMD:
		cmd = ite_hid_pwm_ctl_cmd;
		length = sizeof(ite_hid_pwm_ctl_cmd);
		msg_num = 1;
		break;
		
	case ITE_HID_RW_CHARGERIC_CMD:
		cmd = ite_hid_rw_chargeric_cmd;
		length = sizeof(ite_hid_rw_chargeric_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_WIFI_LED_CMD:
		cmd = ite_hid_wifi_led_cmd;
		length = sizeof(ite_hid_wifi_led_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_TURNON_BASE_CMD:
		cmd = ite_hid_turnon_base_cmd;
		length = sizeof(ite_hid_turnon_base_cmd);
		msg_num = 1;
		break;

	// add by leo for gauge FW update ++
	case ITE_HID_READ_GAUGE_FW_INFO_CMD:
		cmd = ite_hid_read_gauge_fw_info_cmd;
		length = sizeof(ite_hid_read_gauge_fw_info_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;

	case ITE_HID_START_GAUGE_FW_UPDATE_CMD:
		cmd = ite_hid_start_gauge_fw_update_cmd;
		length = sizeof(ite_hid_start_gauge_fw_update_cmd);
		msg_num = 1;
		break;
	// add by leo for gauge FW update --
	
	case ITE_HID_ALS_SET_TABLE_CMD:
		cmd = ite_hid_als_set_table_cmd;
		length = data_len;//sizeof(ite_hid_als_set_table_cmd);
		msg_num = 1;
				
		printk("[ITE]: length = %d, als_table = ",length);
		for(count = 0; count < length; count++){
			printk("0x%02x ", ite_hid_als_set_table_cmd[count]);
		}
		printk("\n");
		break;
		
	case ITE_HID_READ_BASE_CHARGER_STATUS_CMD:
		cmd = ite_hid_read_base_charger_status_cmd;
		length = sizeof(ite_hid_read_base_charger_status_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
		
	case ITE_HID_READ_WIN8_TEST_MODE_CMD:
		cmd = ite_hid_read_win8_test_mode_cmd;
		length = sizeof(ite_hid_read_win8_test_mode_cmd);
		msg_num = 1;
		break;
	
	case ITE_HID_GET_LIGHT_SENSOR_REPORT_CMD:
		cmd = ite_hid_get_light_sensor_report_cmd;
		length = sizeof(ite_hid_get_light_sensor_report_cmd);
		msg[1].len = data_len;
		msg_num = 2;
		break;
	
	case ITE_HID_KB_LED_CMD:
		cmd = ite_hid_kb_led_cmd;
		length = sizeof(ite_hid_kb_led_cmd);
		msg_num = 1;
		break;
		
	case ITE_HID_BASE_EC_FW_CHECK_CMD:
		cmd = ite_hid_base_ec_fw_check_cmd;
		length = sizeof(ite_hid_base_ec_fw_check_cmd);
		msg_num = 1;
		break;
		
	case ITE_HID_CHECK_CHARGER_STATUS_CMD:
		cmd = ite_hid_check_charger_status_cmd;
		length = sizeof(ite_hid_check_charger_status_cmd);
		msg_num = 1;
		break;
		
			
	default:
		printk("[ITE]: %s: command=%d unknow. \n", __func__, command);
		return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;
	
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = rec_buf;
	
	do {
		ret = i2c_transfer(client->adapter, msg, msg_num);
		
		if (ret > 0)
			break;
		tries--;
		printk("[ITE]: retrying ec_i2c_command: 0x%x (%d) \n", command, tries);
		
	} while (tries > 0);
	
	mutex_unlock(&ite_chip_data->mutex_lock);
	
	switch (command) {
	//	case ITE_HID_DESCR_LENGTH_CMD:
	//	case ITE_HID_DESCR_CMD:
	//	case ITE_HID_REPORT_DESCR_CMD:
		case ITE_HID_PAD_BATTERY_CMD:
		case ITE_HID_DOCK_BATTERY_CMD:
		case ITE_HID_READ_EC_REGISTER_CMD:
		case ITE_HID_READ_CHARGER_IC_STATUS_CMD:
		case ITE_HID_READ_GAUGE_FW_INFO_CMD:
		case ITE_HID_READ_BASE_CHARGER_STATUS_CMD:
		case ITE_HID_GET_LIGHT_SENSOR_REPORT_CMD:
			hexdump(rec_buf, msg[1].len);
			break;
		default:
			break;
	}

	return ret;
}

static int asus_ecram_i2c_command(struct i2c_client *client, int command,
				unsigned char *buf_recv, int data_len)
{
	const u8 *cmd = NULL;
	unsigned char *rec_buf = buf_recv;
	int ret;
	int tries = ITE_I2C_COMMAND_TRIES;
	int length = 0;
	struct i2c_msg msg[2];
	int msg_num = 0;
		
	mutex_lock(&ite_chip_data->mutex_lock);
	
	switch (command) {
		case ITE_RAM_VERSION_CMD:
			cmd = ite_ram_version_cmd;
			length = sizeof(ite_ram_version_cmd);
			msg[1].len = data_len;
			msg_num = 2;
			break;
		
		case ITE_RAM_ENTER_FLASH_MODE_CMD:
			cmd = ite_ram_enter_flash_mode_cmd;
			length = sizeof(ite_ram_enter_flash_mode_cmd);
			msg_num = 1;
			break;
				
		case ITE_RAM_STOP_POLLING_BATTERY_CMD:
			cmd = ite_ram_stop_polling_battery_cmd;
			length = sizeof(ite_ram_stop_polling_battery_cmd);
			msg_num = 1;
			break;
		
		case ITE_RAM_START_POLLING_BATTERY_CMD:
			cmd = ite_ram_start_polling_battery_cmd;
			length = sizeof(ite_ram_start_polling_battery_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_WRITE_GAUGE_CMD:
			cmd = ite_ram_write_gauge_cmd;
			length = data_len;//sizeof(ite_ram_write_gauge_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_READ_GAUGE_CMD:
			cmd = ite_ram_read_gauge_cmd;
			length = sizeof(ite_ram_read_gauge_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_ACCESS_GAUGE_STATUE_CMD:
			cmd = ite_ram_access_gauge_statue_cmd;
			length = sizeof(ite_ram_access_gauge_statue_cmd);
			msg[1].len = data_len;
			msg_num = 2;
			break;
		
		case ITE_RAM_READ_GAUGE_BUFFER_CMD:
			cmd = ite_ram_read_gauge_buffer_cmd;
			length = sizeof(ite_ram_read_gauge_buffer_cmd);
			msg[1].len = data_len;
			msg_num = 2;
			break;
		
		case ITE_RAM_CTL_USB0_ID_EC_CMD:
			cmd = ite_ram_ctl_usb0_id_ec_cmd;
			length = sizeof(ite_ram_ctl_usb0_id_ec_cmd);
			msg_num = 1;
			break;
		
		case ITE_RAM_TP_I2C_SW_CMD:
			cmd = ite_ram_tp_i2c_sw_cmd;
			length = sizeof(ite_ram_tp_i2c_sw_cmd);
			msg_num = 1;
			break;
		
		case ITE_RAM_NOTIFY_SYSTEM_STATUS_CMD:
			cmd = ite_ram_notify_system_status_cmd;
			length = sizeof(ite_ram_notify_system_status_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_NOTIFY_OS_CLASS_CMD:
			cmd = ite_ram_notify_os_class_cmd;
			length = sizeof(ite_ram_notify_os_class_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_GAUGE_COMPARE_RESULT_CMD:
			cmd = ite_ram_gauge_compare_result_cmd;
			length = data_len;//sizeof(ite_ram_gauge_compare_result_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_SCREEN_CHANGE_NOTIFY_CMD:
			cmd = ite_ram_screen_change_notify_cmd;
			length = sizeof(ite_ram_screen_change_notify_cmd);
			msg_num = 1;
			break;
		
		case ITE_RAM_CABLE_STATUS_NOTIFY_CMD:
			cmd = ite_ram_cable_status_notify_cmd;
			length = sizeof(ite_ram_cable_status_notify_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_TURN_ON_BASE_CMD:
			cmd = ite_ram_turn_on_base_cmd;
			length = sizeof(ite_ram_turn_on_base_cmd);
			msg_num = 1;
			break;
		
		case ITE_RAM_SET_KEYBOARD_WAKEUP_CMD:
			cmd = ite_ram_set_keyboard_wakeup_cmd;
			length = sizeof(ite_ram_set_keyboard_wakeup_cmd);
			msg_num = 1;
			break;
			
		case ITE_RAM_LED_CONTROL_CMD:
			cmd = ite_ram_led_control_cmd;
			length = sizeof(ite_ram_led_control_cmd);
			msg_num = 1;
			break;
		
		case ITE_RAM_GAUGE_TEMP_CONTROL_CMD:
			cmd = ite_ram_gauge_temp_control_cmd;
			length = sizeof(ite_ram_gauge_temp_control_cmd);
			msg_num = 1;
			break;
														
		default:
			printk("[ITE_ECRAM]: %s: command = %d unknow. \n", __func__, command);
			return -1;
	}

	msg[0].addr = client->addr;
	msg[0].flags = client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;
	
	msg[1].addr = client->addr;
	msg[1].flags = client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = rec_buf;
	
	do {
		ret = i2c_transfer(client->adapter, msg, msg_num);
		
		if (ret > 0)
			break;
		tries--;
		printk("[ITE_ECRAM]: retrying ecram_i2c_command: 0x%x (%d) \n", command, tries);
		
	} while (tries > 0);
	
	mutex_unlock(&ite_chip_data->mutex_lock);
	
	switch (command) {
		case ITE_RAM_VERSION_CMD:
			hexdump_0x66(rec_buf, msg[1].len);
			break;
		default:
			break;
	}

	return ret;
}

// add by leo for ec register read & write ++
static ssize_t ite_66readwrite_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i=0;
	ssize_t sprintf_count=0;

	sprintf_count += sprintf(buf + sprintf_count, "buf_recv = \n");
	//printk("[ITE]: buf_recv = ");
	for (i = 0; i < ITE_MAX_RECEIVED_LENGTH; i++)
	{
		if((i/8!=0) && ( ((i-8)%15)==0))sprintf_count += sprintf(buf + sprintf_count, "- ");
		if((i/15!=0) && (i%15==0))sprintf_count += sprintf(buf + sprintf_count, "\n");
		sprintf_count += sprintf(buf + sprintf_count, "0x%02x ", recv_buf[i]);
		//printk("0x%02x ", recv_buf[i]);
	}
	sprintf_count += sprintf(buf + sprintf_count, "\n");
	//printk("\n");
	
	return sprintf_count;
}

static ssize_t ite_66readwrite_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const u8 *cmd = NULL;
	//u8 rec_buf[15]={0};
	int i=0 ,iloop=0, msg_num=0, cmd_num=0, number=2;
	int tries = ITE_I2C_COMMAND_TRIES;
	int length=0;
	struct i2c_msg msg[2];
	static u8 temp[ITE_MAX_INPUT_LENGTH]={0};

	memset(recv_buf, 0x0, sizeof(recv_buf));

	if(buf[0]=='w'){
		printk("[ITE]: write command: ");
		number=1;
		sscanf (buf, "w %x %x %x %x %x %x %x %x %x %x %x %x",
		&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5],
		&temp[6],&temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);
		
	}else if(buf[0]=='r'){
		printk("[ITE]: read command: ");
		number=2;
		sscanf (buf, "r %x %x %x %x %x %x %x %x %x %x %x %x",
		&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5],
		&temp[6],&temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);

	}else{
		printk("[ITE]: unknown command.\n");
		return count;
	}

	for(iloop=0;iloop<500;iloop++){
		if(buf[iloop]=='x'){
			cmd_num++;
		}
		
		if(buf[iloop]=='\n'){
			//printk("[ITE]: iloop =%d\n",iloop);
			printk("number =%d, cmd_num =%d\n",number, cmd_num);
			break;
		}
	}

	u8* input_cmd = (char*)kcalloc(cmd_num,sizeof(char),GFP_KERNEL);

	printk("[ITE]: input_cmd = ");
	for(i=0;i<cmd_num;i++){
		input_cmd[i]=temp[i]&0xff;
		printk("0x%02x ", input_cmd[i]);
	}
	printk("\n");

	cmd = input_cmd;
	length = cmd_num;//sizeof(input_cmd);
	msg[1].len = ITE_MAX_RECEIVED_LENGTH;
	msg_num = number;

	msg[0].addr = ITE8566_EC_ADDR;
	msg[0].flags = ite_chip_data->client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;

	msg[1].addr = ITE8566_EC_ADDR;
	msg[1].flags = ite_chip_data->client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = recv_buf;
	
	do {
		if (i2c_transfer(ite_chip_data->client->adapter, msg, msg_num) > 0)
			break;
		tries--;
		printk("[ITE]: retrying read_test_cmd (%d) \n", tries);
		
	} while (tries > 0);

	printk("[ITE]: buf_recv = ");
	for (i = 0; i < ITE_MAX_RECEIVED_LENGTH; i++)
	{
		printk("0x%02x ", recv_buf[i]);
	}
	printk("\n");

	//recv_num=((recv_buf[0]>>8)|recv_buf[1]);
	//printk("[ITE]: recv_num=%d \n", recv_num);
	
	if(buf[0]=='r')hexdump(recv_buf, msg[1].len);
	
	kfree(input_cmd);
	
    return count;
} 

static ssize_t ite_68readwrite_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int i=0;
	ssize_t sprintf_count=0;

	sprintf_count += sprintf(buf + sprintf_count, "buf_recv = \n");
	//printk("[ITE]: buf_recv = ");
	for (i = 0; i < ITE_MAX_RECEIVED_LENGTH; i++)
	{
		if((i/8!=0) && ( ((i-8)%16)==0))sprintf_count += sprintf(buf + sprintf_count, "- ");
		if((i/16!=0) && (i%16==0))sprintf_count += sprintf(buf + sprintf_count, "\n");
		sprintf_count += sprintf(buf + sprintf_count, "0x%02x ", recv_buf[i]);
		//printk("0x%02x ", recv_buf[i]);
	}
	sprintf_count += sprintf(buf + sprintf_count, "\n");
	//printk("\n");
	
	return sprintf_count;
}

static ssize_t ite_68readwrite_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	const u8 *cmd = NULL;
	//u8 rec_buf[15]={0};
	int i=0;
	int tries = ITE_I2C_COMMAND_TRIES;
	int length=0, iloop=0;
	struct i2c_msg msg[2];
	int msg_num=0, cmd_num=0, number=2;	
	static u8 temp[ITE_INPUT_MAX]={0};
	u8 buffer_temp[4]={0};
	int cmd_loop = 0;
	
	memset(recv_buf, 0x0, sizeof(recv_buf));
	
	if(buf[0]=='w'){
		printk("[ITE]: write command: ");
		number=1;
		
		/*sscanf (buf, "w %x %x %x %x %x %x %x %x %x %x %x %x",
		&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5],
		&temp[6],&temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);*/
		
	}else if(buf[0]=='r'){
		printk("[ITE]: read command: ");
		number=2;

		/*sscanf (buf, "r %x %x %x %x %x %x %x %x %x %x %x %x",
		&temp[0], &temp[1], &temp[2], &temp[3], &temp[4], &temp[5],
		&temp[6],&temp[7], &temp[8], &temp[9], &temp[10], &temp[11]);*/
		
	}else{
		printk("[ITE]: unknown command.\n");
		return count;
	}

	for(iloop=0;iloop<500;iloop++){
		if(buf[iloop]=='x'){
			cmd_num++;
		}
		
		if(buf[iloop]=='\n'){
			//printk("[ITE]: iloop =%d\n",iloop);
			printk("number =%d, cmd_num =%d\n",number, cmd_num);
			break;
		}
	}
	
	for(cmd_loop = 0; cmd_loop < cmd_num; cmd_loop++){
		buffer_temp[0] = buf[cmd_loop*5 + 2];
		buffer_temp[1] = buf[cmd_loop*5 + 3];
		buffer_temp[2] = buf[cmd_loop*5 + 4];
		buffer_temp[3] = buf[cmd_loop*5 + 5];
		sscanf(buffer_temp, "%x",&temp[cmd_loop]);
	}

	u8* input_cmd = (char*)kcalloc(cmd_num,sizeof(char),GFP_KERNEL);

	printk("[ITE]: input_cmd = ");
	for(i=0;i<cmd_num;i++){
		input_cmd[i]=temp[i]&0xff;
		printk("0x%02x ", input_cmd[i]);
	}
	printk("\n");
/*
	printk("[ITE]: sizeof(input_cmd)=%d\n",sizeof(input_cmd));
	printk("[ITE]: sizeof(ite_ram_version_cmd)=%d\n",sizeof(ite_ram_version_cmd));
	printk("[ITE]: cmd_num=%d\n",cmd_num);
*/	
	cmd = input_cmd;
	length = cmd_num;//sizeof(input_cmd);
	msg[1].len = ITE_MAX_RECEIVED_LENGTH;
	msg_num = number;

	msg[0].addr = ite_chip_data->client->addr;
	msg[0].flags = ite_chip_data->client->flags & I2C_M_TEN;
	msg[0].len = length;
	msg[0].buf = (char *) cmd;

	msg[1].addr = ite_chip_data->client->addr;
	msg[1].flags = ite_chip_data->client->flags & I2C_M_TEN;
	msg[1].flags |= I2C_M_RD;
	msg[1].buf = recv_buf;
	
	do {
		if (i2c_transfer(ite_chip_data->client->adapter, msg, msg_num) > 0)
			break;
		tries--;
		printk("[ITE]: retrying read_test_cmd (%d) \n", tries);
		
	} while (tries > 0);

	printk("[ITE]: buf_recv = ");
	for (i = 0; i < ITE_MAX_RECEIVED_LENGTH; i++)
	{
		printk("0x%02x ", recv_buf[i]);
	}
	printk("\n");

	//recv_num=((recv_buf[1]>>8)|recv_buf[0]);
	//printk("[ITE]: recv_num=%d \n", recv_num);

	if(buf[0]=='r')hexdump(recv_buf, msg[1].len);
	
	kfree(input_cmd);
    return count;
} 
// add by leo for ec register read & write --
//Battery + Charger IC ++ 
int ite8566_read_pad_battery_info(int *bat_present, int *bat_status, int *bat_temp, int *bat_vol, int *bat_current, int *bat_capacity, int *bat_energy)
{
	int rc;
	u8 buf_recv[15];
		
//	printk("[ITE]: %s \n", __func__);
	
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_PAD_BATTERY_CMD, buf_recv, 15);
			if (rc != 2)
			{
				printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_PAD_BATTERY_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			else {
				
				*bat_present = buf_recv[2];
				*bat_status = (buf_recv[4] << 8 ) | buf_recv[3];
				*bat_temp = (buf_recv[6] << 8 ) | buf_recv[5];
				*bat_vol = (buf_recv[8] << 8 ) | buf_recv[7];
				*bat_current = (((char)buf_recv[10]) << 8 )| buf_recv[9];
				*bat_capacity = (buf_recv[12] << 8 ) | buf_recv[11];
				*bat_energy = (buf_recv[14] << 8 ) | buf_recv[13];
			//	*bat_remaining_capacity = (buf_recv[16] << 8 )| buf_recv[15];
			//	*bat_avg_time_to_empty = (recbuf[13] << 8 )| recbuf[12];
			//	*bat_avg_time_to_full = (buf_recv[20] << 8 )| buf_recv[19];
				return 0;
			}
		}
		else
		{		
			printk(KERN_ERR "[ITE] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{	
		printk(KERN_ERR "[ITE] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);	
		return -1;
	}
}
EXPORT_SYMBOL(ite8566_read_pad_battery_info);

int ite8566_read_dock_battery_info(int *bat_present, int *bat_status, int *bat_temp, int *bat_vol, int *bat_current, int *bat_capacity, int *bat_energy)
{
	int rc;
	u8 buf_recv[15];
		
//	printk("[ITE]: %s \n", __func__);
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DOCK_BATTERY_CMD, buf_recv, 15);
			if (rc != 2)
			{
				printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DOCK_BATTERY_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			else
			{
				
				*bat_present = buf_recv[2];
				*bat_status = (buf_recv[4] << 8 ) | buf_recv[3];
				*bat_temp = (buf_recv[6] << 8 ) | buf_recv[5];
				*bat_vol = (buf_recv[8] << 8 ) | buf_recv[7];
				*bat_current = (((char)buf_recv[10]) << 8 )| buf_recv[9];
				*bat_capacity = (buf_recv[12] << 8 ) | buf_recv[11];
				*bat_energy = (buf_recv[14] << 8 ) | buf_recv[13];
			//	*bat_remaining_capacity = (buf_recv[16] << 8 )| buf_recv[15];
			//	*bat_avg_time_to_empty = (recbuf[13] << 8 )| recbuf[12];
			//	*bat_avg_time_to_full = (buf_recv[20] << 8 )| buf_recv[19];
				return 0;
			}
		}
		else
		{
			printk(KERN_ERR "[ITE] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
		
}	
EXPORT_SYMBOL(ite8566_read_dock_battery_info);

int ite_read_chargeric_reg(u8 reg_lsb, u8 reg_msb, u8 data_lsb, u8 data_msb)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE]: %s \n", __func__);
	
	ite_hid_rw_chargeric_cmd[8] = 0x01; //Command (LSB)
	ite_hid_rw_chargeric_cmd[9] = 0x00; //Command (MSB)
	
	ite_hid_rw_chargeric_cmd[10] = reg_lsb; //Register (LSB)
	ite_hid_rw_chargeric_cmd[11] = reg_msb; //Register (LSB)
	
	ite_hid_rw_chargeric_cmd[12] = data_lsb; //Data (LSB)
	ite_hid_rw_chargeric_cmd[13] = data_msb; //Data (LSB)
	
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_RW_CHARGERIC_CMD, buf_recv, 0);
			if (rc != 1)
			{
				printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_RW_CHARGERIC_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			else
				return 0;
		}
		else
		{
			printk(KERN_ERR "[ITE] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
}
EXPORT_SYMBOL(ite_read_chargeric_reg);

int ite_write_chargeric_reg(u8 reg_lsb, u8 reg_msb, u8 data_lsb, u8 data_msb)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE]: %s \n", __func__);
	
	ite_hid_rw_chargeric_cmd[8] = 0x02; //Command (LSB)
	ite_hid_rw_chargeric_cmd[9] = 0x00; //Command (MSB)
	
	ite_hid_rw_chargeric_cmd[10] = reg_lsb; //Register (LSB)
	ite_hid_rw_chargeric_cmd[11] = reg_msb; //Register (LSB)
	
	ite_hid_rw_chargeric_cmd[12] = data_lsb; //Data (LSB)
	ite_hid_rw_chargeric_cmd[13] = data_msb; //Data (LSB)
	
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_RW_CHARGERIC_CMD, buf_recv, 0);
			if (rc != 1)
			{
				printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_RW_CHARGERIC_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			else
				return 0;
		}
		else
		{
			printk(KERN_ERR "[ITE] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
}
EXPORT_SYMBOL(ite_write_chargeric_reg);

int ite_gauge_fw_update_flag(int enable)
{
	if(enable)
		ite_chip_data->gauge_fw_update = 0x01;
	else
		ite_chip_data->gauge_fw_update = 0x00;
	
	return 0;
}
EXPORT_SYMBOL(ite_gauge_fw_update_flag);

// add by leo for gauge FW update ++
int ite_gauge_firmware_update(u16 unseal_key_l, u16 unseal_key_h, u16 full_access_key_l, u16 full_access_key_h, unsigned char *buf)
{
	int rc = 0, count = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE]: %s ", __func__);

	printk("[ITE]: %s: unseal_key_l=%04x,  unseal_key_h=%04x, full_access_key_l=%04x, full_access_key_h=%04x", __func__,unseal_key_l,unseal_key_h,full_access_key_l,full_access_key_h);
	
	ite_hid_start_gauge_fw_update_cmd[8]=unseal_key_l&0xff;
	ite_hid_start_gauge_fw_update_cmd[9]=unseal_key_l>>8;
	ite_hid_start_gauge_fw_update_cmd[10]=unseal_key_h&0xff;
	ite_hid_start_gauge_fw_update_cmd[11]=unseal_key_h>>8;

	ite_hid_start_gauge_fw_update_cmd[12]=full_access_key_l&0xff;
	ite_hid_start_gauge_fw_update_cmd[13]=full_access_key_l>>8;
	ite_hid_start_gauge_fw_update_cmd[14]=full_access_key_h&0xff;
	ite_hid_start_gauge_fw_update_cmd[15]=full_access_key_h>>8;

	printk("[ITE]: ite_hid_start_gauge_fw_update_cmd : ");
	for(count = 0; count < 16; count++){
		printk("0x%02x ", ite_hid_start_gauge_fw_update_cmd[count]);
	}
	printk("\n");

	if(probe_status){	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process)){

			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_START_GAUGE_FW_UPDATE_CMD, buf_recv, 0);
			if(rc != 1){
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_HID_START_GAUGE_FW_UPDATE_CMD failed \n", __func__, __LINE__);
				return -1;
			}			
			
		}else{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}else{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_gauge_firmware_update);

int ite_gauge_read_info(unsigned char *return_buf)
{
	int rc = 0;
	u8 buf_recv[64];
	
	printk("[ITE]: %s ", __func__);	

	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_GAUGE_FW_INFO_CMD, buf_recv, 10);
			if(rc != 2)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_HID_READ_GAUGE_FW_INFO_CMD failed \n", __func__, __LINE__);
			}
			memcpy(return_buf, buf_recv, 10);
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_gauge_read_info);
// add by leo for gauge FW update --

int ite_gauge_stop_polling(void)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE]: %s \n", __func__);
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_STOP_POLLING_BATTERY_CMD, buf_recv, 0);
			if(rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_STOP_POLLING_BATTERY_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			else
				return 0;
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
}
EXPORT_SYMBOL(ite_gauge_stop_polling);

int ite_gauge_start_polling(void)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE]: %s \n", __func__);
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_START_POLLING_BATTERY_CMD, buf_recv, 0);
			if(rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_START_POLLING_BATTERY_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			else
				return 0;
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
}
EXPORT_SYMBOL(ite_gauge_start_polling);

int ite_gauge_write_reg(unsigned char *buf)
{
	int rc = 0, count = 0, cmd_num = 0, err_count = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE]: %s \n", __func__);
	
	cmd_num = buf[2] + 3;
	//cmd_num = cmd_num + 5;
	printk("[ITE]: cmd_num: %d, cmd = ", cmd_num);
	
	ite_ram_write_gauge_cmd[0] = 0xC0;
	ite_ram_write_gauge_cmd[1] = 0x03;
	for(count = 0; count < cmd_num; count++){
		ite_ram_write_gauge_cmd[count + 2] = buf[count]&0xff;
	//	printk("0x%02x ", ite_ram_write_gauge_cmd[count]);
	}
	//printk("\n");
	for(count = 0; count < cmd_num + 2; count++){
		printk("0x%02x ", ite_ram_write_gauge_cmd[count]);
	}
	printk("\n");

	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_WRITE_GAUGE_CMD, buf_recv, cmd_num + 2); // {0xC0, 0X03, 0x.., 0x.., 0x.., ....}
			if(rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_WRITE_GAUGE_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			
			do
			{
				msleep(50);
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_ACCESS_GAUGE_STATUE_CMD, buf_recv, 1); //{0xC0, 0x01}
				if(rc != 2)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_ACCESS_GAUGE_STATUE_CMD failed \n", __func__, __LINE__);
					
				}
				err_count ++;
				
			}while((buf_recv[0] != 0x01) && (err_count < 5));
			
			if(buf_recv[0] == 0x01)
			{
				return 0;
			}
			else
			{
				return -3;
			}
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_gauge_write_reg);

int ite_gauge_read_reg(int addr, int reg, int length, unsigned char *return_buf)
{
	int rc = 0, cmd_num = 0, err_count = 0;
	u8 buf_recv[64];
	
	printk("[ITE]: %s \n", __func__);
	
	cmd_num = length;
	printk("[ITE]: cmd_num: %d \n", cmd_num);
	
	ite_ram_read_gauge_cmd[2] = addr;
	ite_ram_read_gauge_cmd[3] = reg;
	ite_ram_read_gauge_cmd[4] = length;
	
	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_READ_GAUGE_CMD, buf_recv, 0); //{0xC0, 0x02, addr, reg, length}
			if(rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_READ_GAUGE_CMD failed \n", __func__, __LINE__);
				return -1;
			}
	
			do
			{
				msleep(50);
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_ACCESS_GAUGE_STATUE_CMD, buf_recv, 1); //{0xC0, 0x01}
				if(rc != 2)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_ACCESS_GAUGE_STATUE_CMD failed \n", __func__, __LINE__);
					
				}
				err_count ++;
				
			}while((buf_recv[0] != 0x02) && (err_count < 5));
			
			if(buf_recv[0] == 0x02)
			{
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_READ_GAUGE_BUFFER_CMD, return_buf, length); //{0xC0, 0x02}
				if(rc != 2) // modified by leo from 1 to 2
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_READ_GAUGE_BUFFER_CMD failed \n", __func__, __LINE__);
					return -1;
				}
				else
					return 0;
			}
			else
			{
				return -3;
			}
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_gauge_read_reg);

int ite_read_base_charger_status_info(unsigned char *return_buf)
{
	int rc = 0;
	u8 buf_recv[64];
	
	printk("[ITE]: %s ", __func__);

	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_BASE_CHARGER_STATUS_CMD, buf_recv, 4);
			if(rc != 2)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_HID_READ_BASE_CHARGER_STATUS_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			*return_buf = buf_recv[2];
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_read_base_charger_status_info);

int ite_ram_gauge_compare_result(unsigned char *buf)
{
	int rc = 0, count = 0, cmd_num = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE_ECRAM]: %s ", __func__);
	
	cmd_num = buf[0] + 2;
	//cmd_num = cmd_num + 5;;
	printk("[ITE_ECRAM]: cmd_num: %d, cmd = ", cmd_num);
	
	ite_ram_gauge_compare_result_cmd[0] = 0xCB;
	for(count = 0; count < cmd_num; count++){
		ite_ram_gauge_compare_result_cmd[count + 1] = buf[count]&0xff;
	//	printk("0x%02x ", ite_ram_write_gauge_cmd[count]);
	}
	//printk("\n");
	for(count = 0; count < cmd_num; count++){
		printk("0x%02x ", ite_ram_gauge_compare_result_cmd[count]);
	}
	printk("\n");

	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process))
		{
			rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_GAUGE_COMPARE_RESULT_CMD, buf_recv, cmd_num); // {0xC0, 0X03, 0x.., 0x.., 0x.., ....}
			if(rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_GAUGE_COMPARE_RESULT_CMD failed \n", __func__, __LINE__);
				return -1;
			}			
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_ram_gauge_compare_result);

// add by leo for gauge FW update ++
static ssize_t ite_gauge_fw_update_show(struct device *dev, struct device_attribute *attr, char *buf)
{
   	int ret = 0;
		
	printk("[ITE] %s: ++ \n", __func__);
		
   	return ret;
}

static ssize_t ite_gauge_fw_update_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
   	int i=0, ret=0;
	u8 ukey_L_LSB=0, unkey_L_MSB=0, ukey_H_LSB=0, unkey_H_MSB=0, full_akey_L_LSB=0, full_akey_L_MSB=0, full_akey_H_LSB=0, full_akey_H_MSB=0;
	u8 temp[2]={0};
	u8 key_buff[8]={0};

	printk("[ITE] %s: ++ \n", __func__);

	for (i=0;i<8;i++){
		temp[0]=buf[i*2+(i/4)];
		temp[1]=buf[i*2+(i/4)+1];
		sscanf(temp, "%x", &key_buff[i]);
		printk("[ITE] %s: key_buff[%d]=%02x \n", __func__,i,key_buff[i]);
	}

	unkey_H_MSB=key_buff[0];
	ukey_H_LSB=key_buff[1];
	unkey_L_MSB=key_buff[2];
	ukey_L_LSB=key_buff[3];
	full_akey_H_MSB=key_buff[4];
	full_akey_H_LSB=key_buff[5];
	full_akey_L_MSB=key_buff[6];
	full_akey_L_LSB=key_buff[7];

	//sscanf(buf, "%x%x%x%x %x%x%x%x",&unkey_H_MSB,&ukey_H_LSB,&unkey_L_MSB,&ukey_L_LSB,&full_akey_H_MSB,&full_akey_H_LSB,&full_akey_L_MSB,&full_akey_L_LSB);

	printk("[ITE] %s:unkey_H_MSB=%x \n", __func__,unkey_H_MSB);
	printk("[ITE] %s:ukey_H_LSB=%x \n", __func__,ukey_H_LSB);
	printk("[ITE] %s:unkey_L_MSB=%x \n", __func__,unkey_L_MSB);
	printk("[ITE] %s:ukey_L_LSB=%x \n", __func__,ukey_L_LSB);
	
	printk("[ITE] %s:full_akey_H_MSB=%x \n", __func__,full_akey_H_MSB);
	printk("[ITE] %s:full_akey_H_LSB=%x \n", __func__,full_akey_H_LSB);
	printk("[ITE] %s:full_akey_L_MSB=%x \n", __func__,full_akey_L_MSB);
	printk("[ITE] %s:full_akey_L_LSB=%x \n", __func__,full_akey_L_LSB);
	
	ret=ite_gauge_firmware_update((unkey_L_MSB<<8|ukey_L_LSB),(unkey_H_MSB<<8|ukey_H_LSB),(full_akey_L_MSB<<8|full_akey_L_LSB),(full_akey_H_MSB<<8|full_akey_H_LSB), NULL);
	if(ret<0){
		printk("[ITE] %s: ite_gauge_firmware_update fail \n", __func__);
	}
    
   	printk("[ITE] %s: -- \n", __func__);
	
   	return count;
}

static ssize_t ite_gauge_fw_check_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	unsigned char info[10];
	int gauge_fw_ver, chen_id, df_ver, ec_update_status;
		
	printk("[ITE] %s: ++ \n", __func__);
	ret = ite_gauge_read_info(info);
	//printk("[ITE] %s: 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x \n", __func__,info[0],info[1],info[2],info[3],info[4],info[5],info[6],info[7],info[8],info[9]);
	
	gauge_fw_ver = (info[3] << 8) | info[2];
	chen_id = (info[5] << 8) | info[4];
	df_ver = (info[7] << 8) | info[6];
	ec_update_status = (info[9] << 8) | info[8];
	
    return sprintf(buf, "gauge_fw_ver=0x%X, CHEN_ID=0x%X, DF_ver=0x%X, ec_update_status=0x%X\n", gauge_fw_ver, chen_id, df_ver, ec_update_status);
}

static ssize_t ite_gauge_read_version(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
	unsigned char info[10];
	int gauge_fw_ver, chen_id, df_ver, ec_update_status;
		
	printk("[ITE] %s: ++ \n", __func__);
	ret = ite_gauge_read_info(info);
	//printk("[ITE] %s: 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x \n", __func__,info[0],info[1],info[2],info[3],info[4],info[5],info[6],info[7],info[8],info[9]);
	
	gauge_fw_ver = (info[3] << 8) | info[2];
	chen_id = (info[5] << 8) | info[4];
	df_ver = (info[7] << 8) | info[6];
	ec_update_status = (info[9] << 8) | info[8];
	
	if(ec_update_status == 0x0002)
		return sprintf(buf, "%x\n", df_ver);
	else
		return sprintf(buf, "0 \n");
}
// add by leo for gauge FW update --
//Battery + Charger IC --
static ssize_t ite_register_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    int bufsize = 55;
    uint8_t data[55] = { 0 }, loop_i;

    printk(KERN_INFO "[ITE] %s: i2c_command = %x\n", __func__, i2c_command);

    if (i2c_smbus_read_i2c_block_data(ite_chip_data->client, i2c_command, bufsize, &data[0]) < 0) {
        printk(KERN_WARNING "[ITE] %s: read fail\n", __func__);
        return ret;
    }

    ret += sprintf(buf, "command: %x\n", i2c_command);
    for (loop_i = 0; loop_i < bufsize; loop_i++) {
        ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
        if ((loop_i % 16) == 15)
            ret += sprintf(buf + ret, "\n");
    }
    ret += sprintf(buf + ret, "\n");
    return ret;
}

static ssize_t ite_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    char buf_tmp[6], length = 0;
    uint8_t veriLen = 0;
    uint8_t write_da[100];
    unsigned long result = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
	
	printk("[ITE] %s: ++ \n", __func__);
	
    if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
        if (buf[2] == 'x') {
            uint8_t loop_i;
            uint16_t base = 5;
            memcpy(buf_tmp, buf + 3, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
                i2c_command = result;
            for (loop_i = 0; loop_i < 100; loop_i++) {
                if (buf[base] == '\n') {
                    if (buf[0] == 'w')
                        i2c_smbus_write_i2c_block_data(ite_chip_data->client, i2c_command, length, &write_da[0]);
                    printk(KERN_INFO "CMD: %x, %x, %d\n", i2c_command,
                        write_da[0], length);
                    for (veriLen = 0; veriLen < length; veriLen++)
                    {
                        printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));
                    }

                    printk(KERN_INFO "\n");
                    return count;
                }
                if (buf[base + 1] == 'x') {
                    buf_tmp[4] = '\n';
                    buf_tmp[5] = '\0';
                    memcpy(buf_tmp, buf + base + 2, 2);
                    if (!strict_strtoul(buf_tmp, 16, &result))
                        write_da[loop_i] = result;
                    length++;
                }
                base += 4;
            }
        }
    }
    
    printk("[ITE] %s: -- \n", __func__);
    return count;
}

static ssize_t ite_ec_ver_show(struct device *class,struct device_attribute *attr,char *buf)
{			
	return sprintf(buf, "%s \n", ite_chip_data->pad_ec_version);
}

static ssize_t ite_system_owner(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	
	ret = ite_system_owner_function();
	return sprintf(buf, "%d \n", ite_chip_data->system_owner);
}

static ssize_t ite_get_ec_status(struct device *dev, struct device_attribute *attr, char *buf)
{
//	printk(KERN_INFO "[ITE]:ec status = %d. \n",ite_chip_data->init_success);
	return sprintf(buf, "%d \n", ite_chip_data->init_success);
}

static ssize_t ite_pad_led_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
		
	if(!strncmp(buf, "led_0", 5))
	{
		ret = ite_pad_led_function(0);
	}
	else if(!strncmp(buf, "led_1", 5))
	{
		ret = ite_pad_led_function(1);
	}
	else if(!strncmp(buf, "led_2", 5))
	{
		ret = ite_pad_led_function(2);
	}
	else if(!strncmp(buf, "led_3", 5))
	{
		ret = ite_pad_led_function(3);
	}
	else {
		printk("[ITE]%s:command not support.\n", __func__);
	}
	
	return count;
}

static ssize_t ite_charger_ic_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rc;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk(KERN_ERR "[ITE] %s:\n", __func__);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_CHARGER_IC_STATUS_CMD, buf_recv, 4);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_READ_CHARGER_IC_STATUS_CMD failed \n", __func__, __LINE__);
		ite_chip_data->charger_status = 0x0000;
	}
	else
	{
		printk("[ITE] %s: buf_recv[2]=0x%x, buf_recv[3]=0x%x  \n", __func__, buf_recv[2], buf_recv[3]);
		ite_chip_data->charger_status = (buf_recv[3] << 8) | buf_recv[2];
		
	}
	return sprintf(buf, "%d \n", ite_chip_data->charger_status);
}

static ssize_t ite_light_sensor_switch(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	if (buf[0] == '0')
	{
		ret = ite_light_sensor_set_function(ASUSEC_LIGHT_SENSOR_DISABLE);
	}
	else if(buf[0] == '1')
	{
		ret = ite_light_sensor_set_function(ASUSEC_LIGHT_SENSOR_ENABLE);
	}
	else if(buf[0] == 'w')
	{
		ret = ite_light_sensor_set_function(ASUSEC_LIGHT_SENSOR_RAWDATA);
	}
	else if(buf[0] == 'r')
	{
		ret = ite_light_sensor_set_function(ASUSEC_LIGHT_SENSOR_RESET);
	}
	else
	{
		printk("[ITE] %s: Unknown Command. \n",__func__);
	}
	
	return count;
}

static ssize_t ite_light_sensor_level(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d \n", ite_chip_data->als_level);
}

static ssize_t ite_base_plug_in(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d \n", ite_chip_data->detach_attach);
}

static ssize_t ite_light_sensor_status(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d \n", ite_chip_data->als_status);
}

static ssize_t ite_wifi_led(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int ret = 0;
	
	if (buf[0] == '0')
	{
		ret = ite_wifi_led_function(0);
	}
	else if(buf[0] == '1')
	{
		ret = ite_wifi_led_function(1);
	}
	else
	{
		printk("[ITE] %s: Unknown Command. \n",__func__);
	}
	
	return count;
}

static ssize_t ite_base_power_on(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	
	ret = ite_turn_on_base_power();
	return sprintf(buf, "[ITE] wake-up win8. \n");
}

static ssize_t ite_touchpad_enable(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if (buf[0] == '0')
	{
		elan_i2c_touchpad_enable(0);
	}
	else if(buf[0] == '1')
	{
		elan_i2c_touchpad_enable(1);
	}
	else
	{
		printk("[ITE] %s: Unknown Command. \n",__func__);
	}
	
	return count;
}

static ssize_t ite_switch_to_win8(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	
	ret = ite_system_owner_function();
	ret = ite_win8_power_switch_status_report();
	ret = ite_atow_early_notify_driver(SYSTEM_WINDOWS);
	ret = ite_screen_change_function(SYSTEM_WINDOWS);
	ret = ite_atow_late_notify_driver(SYSTEM_WINDOWS);
		
	return sprintf(buf, "[ITE]: receive cmd switch to Win8. \n");
}

static ssize_t ite_switch_to_android(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret = 0;
		
	ret = ite_screen_change_function(SYSTEM_ANDROID);
	ret = ite_system_owner_function();	
	ret = ite_win8_power_switch_status_report();
	ret = ite_wtoa_late_notify_driver(SYSTEM_ANDROID);
	ret = asusec_input_device_create();
		
	return sprintf(buf, "[ITE]: receive cmd switch to Android. \n");
}

static ssize_t ite_win8_test_mode(struct device *class,struct device_attribute *attr,char *buf)
{
	if(ite_chip_data->win8_test_mode == 0x00)
		return sprintf(buf, "[ITE]: Win8 normal mode.\n");
	else if (ite_chip_data->win8_test_mode == 0x01)
		return sprintf(buf, "[ITE]: Win8 Test mode.\n");
	else
		return sprintf(buf, "[ITE]: get win8 test mode fail!!\n");
}

static ssize_t ite_charger_status(struct device *class,struct device_attribute *attr,char *buf)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	int charger_state;
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_CHECK_CHARGER_STATUS_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s: write ITE_HID_CHECK_CHARGER_STATUS_CMD failed \n", __func__);
		return -1;
	}
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_EC_REGISTER_CMD, buf_recv, 3);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_EC_REGISTER_CMD failed \n", __func__, __LINE__);
		return -1;
	}
	charger_state = buf_recv[2];
		
	return sprintf(buf, "%d \n", charger_state);
}

int ite_light_sensor_set_function(int cmd) //0x01 = Reset ALS, 0x02 = Enable ALS, 0x04 = Disable ALS, 0x08 = Report ALS rawdata
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: cmd = 0x%x \n",__func__, cmd);

	ite_hid_als_ctl_cmd[5] = cmd; //LSB
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_ALS_CTL_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_ALS_CTL_CMD failed \n", __func__, __LINE__);
		return -1;
	}

	return 0;
}
EXPORT_SYMBOL(ite_light_sensor_set_function);

int ite_light_sensor_set_table(unsigned char *buf)
{
	int rc = 0, count = 0, cmd_num = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s ++ \n",__func__);
	
	ite_hid_als_set_table_cmd[0] = 0xF5;
	ite_hid_als_set_table_cmd[1] = 0x00;
	ite_hid_als_set_table_cmd[2] = 0x36;
	ite_hid_als_set_table_cmd[3] = 0x03;
	ite_hid_als_set_table_cmd[4] = 0xF6;
	ite_hid_als_set_table_cmd[5] = 0x00;

	cmd_num = buf[0];
//	printk("[ITE]: cmd_num: %d ,cmd = ", cmd_num);
		
	for(count = 0; count < cmd_num; count++){
		ite_hid_als_set_table_cmd[count + 6] = buf[count]&0xff;
	}
//	for(count = 0; count < cmd_num + 6; count++){
//		printk("0x%02x ", ite_hid_als_set_table_cmd[count]);
//	}
//	printk("\n");

	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_ALS_SET_TABLE_CMD, buf_recv, cmd_num + 6); //{0x1C, 0x00, 0x.., 0x.., 0x.., ...}
			if(rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_HID_ALS_SET_TABLE_CMD failed \n", __func__, __LINE__);
				return -1;
			}			
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	
	return 0;
}
EXPORT_SYMBOL(ite_light_sensor_set_table);

int ite_light_sensor_get_table(unsigned char *report_buf)
{
	int rc = 0;
	u8 buf_recv[128];
	
	printk("[ITE] %s ++ \n",__func__);
	memset(buf_recv, 0, 128);

	if(probe_status)
	{	
		if(ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_GET_LIGHT_SENSOR_REPORT_CMD, buf_recv, 128);
			if(rc != 2)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: read ITE_HID_GET_LIGHT_SENSOR_REPORT_CMD failed \n", __func__, __LINE__);
				return -1;
			}
			memcpy(report_buf, buf_recv, sizeof(buf_recv));
		}
		else
		{
			printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);
		return -1;
	}
	
	return 0;
}
EXPORT_SYMBOL(ite_light_sensor_get_table);

int ite_pwm_setting(u8 pwm)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: PWM = 0x%x\n",__func__, pwm);
	ite_hid_pwm_ctl_cmd[8] = pwm;
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_PWM_CTL_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_PWM_CTL_CMD failed \n", __func__, __LINE__);
		return -1;
	}
	
	return 0;
}
EXPORT_SYMBOL(ite_pwm_setting);

static int ite_detach_attach_function(void)
{
	int rc = 0;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	
//	printk(KERN_ERR "[ITE] %s ++ \n", __func__);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_ATTACH_DETACH_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_ATTACH_DETACH_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_EC_REGISTER_CMD, buf_recv, 3);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_EC_REGISTER_CMD failed \n", __func__, __LINE__);
		return false;
	}
	ite_chip_data->detach_attach = buf_recv[2];
	printk("[ITE] %s: detach(0) or attach(1) = 0x%x. \n", __func__, ite_chip_data->detach_attach);
	
	return ite_chip_data->detach_attach;
	
}
static int ite_system_owner_function(void)
{
	int rc = 0;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	int old_owner = 0;
	
//	printk(KERN_ERR "[ITE] %s ++ \n", __func__);
	
	old_owner = ite_chip_data->system_owner;
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_SYSTEM_OWNER_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_SYSTEM_OWNER_CMD failed \n", __func__, __LINE__);
		return -1;
	}
		
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_EC_REGISTER_CMD, buf_recv, 3);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_EC_REGISTER_CMD failed \n", __func__, __LINE__);
		return -1;
	}
	ite_chip_data->system_owner = buf_recv[2];
	printk("[ITE] %s: system owner = 0x%x. \n", __func__, ite_chip_data->system_owner);
	
	if(ite_chip_data->system_owner != old_owner)
	{
		switch_set_state(&ite_chip_data->owner_sdev, ite_chip_data->system_owner);
	}
	
	return ite_chip_data->system_owner;
}

static int ite_base_power_status(void)
{
	int rc = 0;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	
//	printk(KERN_ERR "[ITE] %s ++ \n", __func__);
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_BASE_SYSTEM_STATUS_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_BASE_SYSTEM_STATUS_CMD failed \n", __func__, __LINE__);
		return 0;
	}

	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_EC_REGISTER_CMD, buf_recv, 3);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_EC_REGISTER_CMD failed \n", __func__, __LINE__);
		return 0;
	}
	ite_chip_data->base_power = buf_recv[2];
	printk("[ITE] %s: Base Power Status = 0x%x. \n", __func__, ite_chip_data->base_power);
	
	return ite_chip_data->base_power;
}

static bool ite_screen_change_function(int screen_status)
{
	int rc = 0;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: (0x%x).\n", __func__, screen_status);
	
//	ite_hid_screen_change_notify_cmd[11] = screen_status; //android screen
//	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_SCREEN_CHANGE_NOTIFY_CMD, buf_recv, 0);
//	if (rc != 1)
//	{
//		printk(KERN_ERR "[ITE] %s:write ITE_HID_SCREEN_CHANGE_NOTIFY_CMD failed \n", __func__);
//		return false;
//	}
		
	ite_ram_screen_change_notify_cmd[2] = screen_status;
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_SCREEN_CHANGE_NOTIFY_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:write ITE_RAM_SCREEN_CHANGE_NOTIFY_CMD failed \n", __func__);
		return false;
	}
	return true;
}

static bool ite_android_power_status(APOWER_STATUS power_status)
{
	int rc = 0;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE_ECRAM] %s: Notify EC FW android system will entry 0x%x \n", __func__, power_status);
	
//	ite_hid_android_power_status_cmd[11] = power_status;
//	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_ANDROID_POWER_STATUS_CMD, buf_recv, 0);
//	if (rc != 1)
//	{
//		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_ANDROID_POWER_STATUS_CMD failed \n", __func__, __LINE__);
//		return false;
//	}

	ite_ram_notify_system_status_cmd[2] = power_status; 
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_NOTIFY_SYSTEM_STATUS_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: Get ITE_RAM_NOTIFY_SYSTEM_STATUS_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
	return true;
}

static int ite_wifi_led_function(int on_off)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	if(on_off == 1)
	{
		printk("[ITE] %s: wifi led on. \n",__func__);
		ite_hid_wifi_led_cmd[5] = 0x01; //LSB
		rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_WIFI_LED_CMD, buf_recv, 0);
		if (rc != 1)
		{
			printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_WIFI_LED_CMD failed \n", __func__, __LINE__);
			return -1;
		}
	}
	else if(on_off == 0)
	{
		printk("[ITE] %s: wifi led off. \n",__func__);
		ite_hid_wifi_led_cmd[5] = 0x00; //LSB
		rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_WIFI_LED_CMD, buf_recv, 0);
		if (rc != 1)
		{
			printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_WIFI_LED_CMD failed \n", __func__, __LINE__);
			return -1;
		}
	}
	else
	{
		printk("[ITE] %s: Unknown Command. \n",__func__);
		return -2;
	}
	return 0;
}

static int ite_pad_led_function(int status)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	if (status == 0)
	{
		printk("[ITE] %s: PAD LED off for factory test.\n",__func__);
		ite_ram_led_control_cmd[2] = 0x01; //LED off
		rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_LED_CONTROL_CMD, buf_recv, 0);
		if (rc != 1)
		{
			printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_LED_CONTROL_CMD failed \n", __func__);
			return -1;
		}
	}
	else if (status == 1)
	{
		printk("[ITE] %s:PAD Orange LED on for factory test.\n",__func__);
		ite_ram_led_control_cmd[2] = 0x02; //Orange LED on
		rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_LED_CONTROL_CMD, buf_recv, 0);
		if (rc != 1)
		{
			printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_LED_CONTROL_CMD failed \n", __func__);
			return -1;
		}
	}
	else if (status == 2)
	{
		printk("[ITE] %s:PAD Green LED on for factory test.\n",__func__);
		ite_ram_led_control_cmd[2] = 0x04; //Green LED on
		rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_LED_CONTROL_CMD, buf_recv, 0);
		if (rc != 1)
		{
			printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_LED_CONTROL_CMD failed \n", __func__);
			return -1;
		}
	}
	else if (status == 3)
	{
		printk("[ITE] %s:PAD LED normal mode for factory test.\n",__func__);
		ite_ram_led_control_cmd[2] = 0x10; //go back to normal mode.
		rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_LED_CONTROL_CMD, buf_recv, 0);
		if (rc != 1)
		{
			printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_LED_CONTROL_CMD failed \n", __func__);
			return -1;
		}
	}
	else
	{
		printk("[ITE] %s: Unknown Command. \n",__func__);
	}
	
	return 0;
}

static int ite_keyboard_led_function(KB_LED type)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	//printk("[ITE] %s: turn on keyboard led = 0x%x. \n",__func__, type);
	ite_hid_kb_led_cmd[5] = type;
	printk("[ITE] %s: ite_hid_kb_led_cmd = %x \n", __func__, ite_hid_kb_led_cmd[5]);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_KB_LED_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_KB_LED_CMD failed \n", __func__, __LINE__);
		return -1;
	}
	return 0;
}

static int ite_turn_on_base_power(void)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: \n",__func__);

//	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_TURNON_BASE_CMD, buf_recv, 0);
//	if (rc != 1)
//	{
//		printk(KERN_ERR "[ITE] %s: write ITE_HID_TURNON_BASE_CMD failed \n", __func__);
//		return -1;
//	}
	
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_TURN_ON_BASE_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_TURN_ON_BASE_CMD failed \n", __func__);
		return -1;
	}

	return 0;
}

static int ite_base_ec_fw_check(void)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: \n",__func__);

	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_BASE_EC_FW_CHECK_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s: write ITE_HID_BASE_EC_FW_CHECK_CMD failed \n", __func__);
		return -1;
	}
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_EC_REGISTER_CMD, buf_recv, 3);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_EC_REGISTER_CMD failed \n", __func__, __LINE__);
		return -1;
	}
	
	ite_chip_data->base_ec_check = buf_recv[2];
	printk("[ITE] %s: determine Base EC firmware update = 0x%x \n", __func__, ite_chip_data->base_ec_check);	
	switch_set_state(&ite_chip_data->baseec_sdev, ite_chip_data->base_ec_check);
	
	return 0;
}

static int ite_ram_notify_os_class(void)
{
	int rc = 0;
	int os_class = 0x00;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	
	if(entry_mode == 1)
		os_class = MOS;
	else if(entry_mode == 2)
		os_class = RECOVERY;
	else if(entry_mode == 3)
		os_class = POS;
	else if(entry_mode == 4)
		os_class = COS;
	else
		os_class = 0x00;
	
	printk("[ITE_ECRAM] %s: OS Class = 0x%x. \n", __func__, os_class);
	
	ite_ram_notify_os_class_cmd[2] = os_class; 
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_NOTIFY_OS_CLASS_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: Get ITE_RAM_NOTIFY_OS_CLASS_CMD failed \n", __func__, __LINE__);
		return -1;
	}
		
	return 0;
}

static int ite_ram_enable_keyboard_wakeup(int enable)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: \n",__func__);
	
	if(enable == 0)
		ite_ram_set_keyboard_wakeup_cmd[2] = 0x02;
	else
		ite_ram_set_keyboard_wakeup_cmd[2] = 0x01;
		
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_SET_KEYBOARD_WAKEUP_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_SET_KEYBOARD_WAKEUP_CMD failed \n", __func__);
		return -1;
	}
	return 0;
}

int ite_ram_gauge_temp_control(int enable)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: \n",__func__);
	
	if(enable == 1)
		ite_ram_gauge_temp_control_cmd[2] = 0x01;
	else
		ite_ram_gauge_temp_control_cmd[2] = 0x02;
		
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_GAUGE_TEMP_CONTROL_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_GAUGE_TEMP_CONTROL_CMD failed \n", __func__);
		return -1;
	}
	return 0;
}
EXPORT_SYMBOL(ite_ram_gauge_temp_control);

static int ite_get_ec_version(void)
{
	int rc = 0;
	char ec_version[16];
	char checksum_version[4];
	
	//Get EC version ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	msleep(500);
	memset(ite_chip_data->pad_ec_version, 0, 32);
	memset(ite_chip_data->dock_ec_version, 0, 32);
	ite_chip_data->version_checksum = 0;
	
	rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_VERSION_CMD, ec_version, 16);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE_ECRAM] %s: Get ITE_RAM_VERSION_CMD failed \n", __func__);
		printk("[ITE]: Version Checksum = %d. \n", ite_chip_data->version_checksum);
		return -1;
	}
	else
	{
		memcpy(ite_chip_data->pad_ec_version, &ec_version[0], 8);
		memcpy(ite_chip_data->dock_ec_version, &ec_version[8], 8);
		printk("[ITE_ECRAM]: PAD EC Version: %s. \n", ite_chip_data->pad_ec_version);
		printk("[ITE_ECRAM]: Base EC Version: %s. \n", ite_chip_data->dock_ec_version);
		checksum_version[0] = ite_chip_data->pad_ec_version[4];
		checksum_version[1] = ite_chip_data->pad_ec_version[5];
		checksum_version[2] = ite_chip_data->pad_ec_version[6];
		checksum_version[3] = ite_chip_data->pad_ec_version[7];
		ite_chip_data->version_checksum = ((checksum_version[0] << 24) | (checksum_version[1] << 16) |(checksum_version[2] << 8) | checksum_version[3]);
		printk("[ITE]: Version Checksum = %d. \n", ite_chip_data->version_checksum);
	}
	
	//Get EC version ------------------------------------------------------------------------------------------------------------
	return 0;
}

static bool ite_i2c_hw_init(struct i2c_client *client)
{
	int rc = 0, ret = 0;
	uint8_t buf_recv[ITE_REPORT_MAX_LENGTH];
	int hid_descr_len;
	int hid_report_len;
	
	/* Fetch the length of HID description */
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DESCR_LENGTH_CMD, buf_recv, HID_DES_LENGTH_OFFSET);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DESCR_LENGTH_CMD failed \n", __func__, __LINE__);
		return false;
	}
	hid_descr_len = buf_recv[0];
	printk("[ITE] %s:[%d]: Get hid_descr_len = %d \n", __func__, __LINE__, hid_descr_len);
	mdelay(3);
		
	/* Fetch the lenght of HID report description */
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DESCR_CMD, buf_recv, hid_descr_len);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_DESCR_CMD failed \n", __func__, __LINE__);
		return false;
	}
	hid_report_len = (buf_recv[5]<< 2 | buf_recv[4]);
	printk("[ITE] %s:[%d]: Get hid_report_len = %d \n", __func__, __LINE__, hid_report_len );
	mdelay(3);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_POWER_ON_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_POWER_ON_CMD failed \n", __func__, __LINE__);
		return false;
	}
	mdelay(3);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_RESET_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_RESET_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
	msleep(ITE_I2C_COMMAND_DELAY);

	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_REPORT_DESCR_CMD, buf_recv, hid_report_len);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_REPORT_DESCR_CMD failed \n", __func__, __LINE__);
		return false;
	}
	msleep(5);
	
	probe_status = 1;
	ite_chip_data->init_success = 1;
		
	ret = ite_base_power_status();
	switch_set_state(&ite_chip_data->win_power_sdev, ite_chip_data->base_power);
	ret = ite_system_owner_function();
	ret = ite_detach_attach_function();
	ite_base_switch_status_report();
	ret = ite_get_ec_version();
	ret = ite_base_ec_fw_check();
				
	if(ite_chip_data->detach_attach == BASE_DETACH)
	{
		printk(KERN_ERR "[ITE] %s: Base Detach. \n",__func__);
		ret = ite_android_power_status(S1);
	}	
	else
	{
		printk("[ITE] %s: Base Attach. \n",__func__);
		printk("[ITE] %s: System Owner = 0x%x. \n",__func__,ite_chip_data->system_owner);
		printk("[ITE] %s: Base Power Status = 0x%x. \n",__func__,ite_chip_data->base_power);
	}
	queue_delayed_work(ite_chip_data->asusec_wq, &ite_chip_data->ite_init_other_drv_work, 10*HZ);

	return true;
}

int ite8566_usb_cable_status(int usb_state)
{
	int rc;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s: USB State = %d \n", __func__, usb_state );
	usb_cable_init_value = usb_state;
	
	if(probe_status)
	{	
		if(ite_chip_data->usb_status_finish && ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			
			if(usb_state == USB_IN)
				ite_ram_cable_status_notify_cmd[2] = 0x02; //USB in
			else if(usb_state == AC_IN)
				ite_ram_cable_status_notify_cmd[2] = 0x01; //AC in
			else if(usb_state == CABLE_OUT)
				ite_ram_cable_status_notify_cmd[2] = 0x00; //remove cable
			else if(usb_state == ENABLE_5V)
				ite_ram_cable_status_notify_cmd[2] = 0x04; //OTG in
			else if(usb_state == DISABLE_5V)
				ite_ram_cable_status_notify_cmd[2] = 0x00; //OTG out
			else
				ite_ram_cable_status_notify_cmd[2] = 0x00;	//remove cable
				
			printk("[ITE_ECRAM] %s: cable cmd = 0x%x. \n", __func__, ite_ram_cable_status_notify_cmd[2]);
						
			rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_CABLE_STATUS_NOTIFY_CMD, buf_recv, 0);
			if (rc != 1)
			{
				printk(KERN_ERR "[ITE_ECRAM] %s: write ITE_RAM_CABLE_STATUS_NOTIFY_CMD failed \n", __func__);
				return 1;
			}
						
			//if(usb_state == USB_IN)
			//	ite_hid_cable_status_notify_cmd[11] = 0x02; //USB in
			//else if(usb_state == AC_IN)
			//	ite_hid_cable_status_notify_cmd[11] = 0x01; //AC in
			//else if(usb_state == CABLE_OUT)
			//	ite_hid_cable_status_notify_cmd[11] = 0x00; //remove cable
			//else if(usb_state == ENABLE_5V)
			//	ite_hid_cable_status_notify_cmd[11] = 0x04; //OTG in
			//else if(usb_state == DISABLE_5V)
			//	ite_hid_cable_status_notify_cmd[11] = 0x00; //OTG out
			//else
			//	ite_hid_cable_status_notify_cmd[11] = 0x00;	//remove cable
			//
			//printk("[ITE] %s: cable_cmd = 0x%x \n", __func__, ite_ram_cable_status_notify_cmd[2]);
			//
			//rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_CABLE_STATUS_NOTIFY_CMD, buf_recv, 0);
			//if (rc != 1)
			//{
			//	printk(KERN_ERR "[ITE] %s: write ITE_HID_CABLE_STATUS_NOTIFY_CMD failed \n", __func__);
			//	return 1;
			//}
		}
	}

	return 0;
}
EXPORT_SYMBOL(ite8566_usb_cable_status);

static int ite_usb_host_client_switch(USB_SWITCH_MODE mode)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	if(probe_status)
	{	
		if(ite_chip_data->usb_status_finish && ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			if(mode == HOST)
			{
				printk("[ITE] %s: USB switch to Host. \n",__func__);
				ite_ram_ctl_usb0_id_ec_cmd[2] = 0x01; //Host
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_CTL_USB0_ID_EC_CMD, buf_recv, 0);
				if(rc != 1)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_CTL_USB0_ID_EC_CMD failed \n", __func__, __LINE__);
					return -1;
				}
			}
			else if(mode == CLIENT)
			{
				printk("[ITE] %s: USB switch to Client. \n",__func__);
				ite_ram_ctl_usb0_id_ec_cmd[2] = 0x02; //Client
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_CTL_USB0_ID_EC_CMD, buf_recv, 0);
				if(rc != 1)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_CTL_USB0_ID_EC_CMD failed \n", __func__, __LINE__);
					return -1;
				}
			}
			else
			{
				printk("[ITE] %s: Unknown Command. \n",__func__);
				return -2;
			}
		}
		else
		{		
			printk(KERN_ERR "[ITE] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{	
		printk(KERN_ERR "[ITE] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);	
		return -1;
	}
	
	return 0;
}

int ite_usb_switch_mode(int usb_mode)
{
	printk("[ITE] %s: USB mode = %d. \n", __func__, usb_mode );
	
	if(usb_mode == CLIENT)
	{
		usb_switch_mode_flag = 1;
		ite_usb_host_client_switch(CLIENT);
	}
	else
	{
		usb_switch_mode_flag = 0;
		ite_usb_host_client_switch(HOST);
	}
		
	return 0;
}
EXPORT_SYMBOL(ite_usb_switch_mode);

int ite8566_switch_notify(void)
{
	int status;
	
//	printk("[ITE] %s ++ \n", __func__);
	status = ite_chip_data->base_power;
	
	return status;
}
EXPORT_SYMBOL(ite8566_switch_notify);

int ite_panel_owner_notify(void)
{
	int status;
	int ret = 0;
	
	printk("[ITE] %s ++ \n", __func__);
	
	ret = ite_system_owner_function();
	
	status = ite_chip_data->system_owner;
	
	return status;
}
EXPORT_SYMBOL(ite_panel_owner_notify);

int ite_charger_ic_status_function(void)
{
	int rc;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s ++ \n", __func__);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_CHARGER_IC_STATUS_CMD, buf_recv, 4);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: write ITE_HID_READ_CHARGER_IC_STATUS_CMD failed \n", __func__, __LINE__);
		return 0x0000;
	}
	else
	{
		ite_chip_data->charger_status = (buf_recv[3] << 8) | buf_recv[2];
		printk(KERN_ERR "[ITE] %s: Charger IC status = 0x%x. \n", __func__, ite_chip_data->charger_status);
	}
		
	return ite_chip_data->charger_status;
}
EXPORT_SYMBOL(ite_charger_ic_status_function);

int ite_touchpad_switch(SYSTEM_OWNER mode)
{
	int rc = 0;
	u8 buf_recv[ITE_MAX_INPUT_LENGTH];
	
	printk("[ITE] %s \n",__func__);
	
	if(probe_status)
	{	
		if(ite_chip_data->usb_status_finish && ite_chip_data->init_success && (!ite_chip_data->fw_up_process) && (!ite_chip_data->gauge_fw_update))
		{
			if(mode == SYSTEM_ANDROID)
			{
				printk("[ITE] %s: Touchpad switch to android. \n",__func__);
				ite_ram_tp_i2c_sw_cmd[2] = 0x01; //GPE4 output low
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_TP_I2C_SW_CMD, buf_recv, 0);
				if(rc != 1)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_TP_I2C_SW_CMD failed \n", __func__, __LINE__);
					return -1;
				}
			}
			else if(mode == SYSTEM_WINDOWS)
			{
				printk("[ITE] %s: Touchpad switch to Win8. \n",__func__);
				ite_ram_tp_i2c_sw_cmd[2] = 0x02; //Client
				rc = asus_ecram_i2c_command(&ecram_client, ITE_RAM_TP_I2C_SW_CMD, buf_recv, 0);
				if(rc != 1)
				{
					printk(KERN_ERR "[ITE_ECRAM] %s:[%d]: write ITE_RAM_TP_I2C_SW_CMD failed \n", __func__, __LINE__);
					return -1;
				}
			}
			else
			{
				printk("[ITE] %s: Unknown Command. \n",__func__);
				return -2;
			}
		}
		else
		{		
			printk(KERN_ERR "[ITE] %s:[%d]: EC driver initial failed. \n", __func__, __LINE__);
			return -2;
		}
	}
	else
	{	
		printk(KERN_ERR "[ITE] %s:[%d]: EC driver probe failed. \n", __func__, __LINE__);	
		return -1;
	}
	
	return 0;
}
EXPORT_SYMBOL(ite_touchpad_switch);

int ite_powerbtn_notify(void)
{
//	printk(KERN_ERR "[ITE] %s ++  \n", __func__);
	
	ite_chip_data->powerbtn_press = 1;
		
//	printk(KERN_ERR "[ITE] %s --  \n", __func__);
	return 0;
}
EXPORT_SYMBOL(ite_powerbtn_notify);

static int ite_read_win8_test_mode(void)
{
	int rc = 0;
	uint8_t buf_recv[ITE_MAX_INPUT_LENGTH];
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_WIN8_TEST_MODE_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_WIN8_TEST_MODE_CMD failed \n", __func__, __LINE__);
		return false;
	}
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_READ_EC_REGISTER_CMD, buf_recv, 3);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE] %s:[%d]: Get ITE_HID_READ_EC_REGISTER_CMD failed \n", __func__, __LINE__);
		return false;
	}
	ite_chip_data->win8_test_mode = buf_recv[2];
	printk(KERN_ERR "[ITE] %s: Win8_Test Mode = 0x%x. \n", __func__, ite_chip_data->win8_test_mode);
	
	return ite_chip_data->win8_test_mode;
}

// add by Josh for create keyboard device ++
static int asusec_input_device_create(void){
	
	int err = 0,i;
	
	printk(KERN_ERR "[ITE] %s: (0x%x)\n", __func__, ite_chip_data->input_dev);
	
	if(ite_chip_data->input_dev)
	{
		printk(KERN_ERR "[ITE]: input device is exist. \n");
		return 0;
	}
	
	ite_chip_data->input_dev = input_allocate_device();
	if (ite_chip_data->input_dev == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR "[ITE] %s:[%d]: Failed to allocate input device.\n", __func__, __LINE__);
		goto exit;
	}

	snprintf(ite_chip_data->input_phys_name, sizeof(ite_chip_data->input_phys_name), "%s/input-kp", dev_name(&ite_chip_data->client->dev));
	ite_chip_data->input_dev->name = "asus-ec";
	ite_chip_data->input_dev->id.bustype = BUS_I2C; //20110901add
	ite_chip_data->input_dev->phys = ite_chip_data->input_phys_name;	
	ite_chip_data->input_dev->event = ite_i2c_event;
		
	//Keypad
	for(i=0;i<ITE_KEYMAP_SIZE;i++) {
		set_bit(i,ite_chip_data->input_dev->keybit);
		ite_chip_data->keymap[i] = i;
	}

	ite_chip_data->input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_MSC);
	__set_bit(EV_KEY, ite_chip_data->input_dev->evbit);
	err = input_register_device(ite_chip_data->input_dev);
		
	if(err < 0) {
		printk(KERN_ERR "[ITE]: %s: Error to register input device. \n", __func__);
		goto exit_input_free;
	}
	
	ite_chip_data->input_dev_status = 1;
	return 0;
	
exit_input_free:
	input_free_device(ite_chip_data->input_dev);
	ite_chip_data->input_dev = NULL;
	ite_chip_data->input_dev_status = 0;
exit:
	return err;
}
static int asusec_input_device_destroy(void){
	
	printk("[ITE] %s: (0x%x) \n", __func__, ite_chip_data->input_dev);
	
	if(ite_chip_data->input_dev)
	{
		input_unregister_device(ite_chip_data->input_dev);
		ite_chip_data->input_dev = NULL;
		ite_chip_data->input_dev_status = 0;
	}
	return 0;
}
// add by Josh for create keyboard device --

// add by Josh for ec switch read & write ++
static ssize_t ite_win8_power_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ite_chip_data->pad_ec_version);
}

static ssize_t ite_win8_power_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ite_chip_data->base_power);
}

static int ite_win8_power_switch_status_report(void)
{
	int ret = 0;
	int old_status = 0;
	
//	printk("[ITE]: %s \n",__func__);
	old_status = ite_chip_data->base_power;
	
	ret = ite_base_power_status();
	printk("[ITE] %s: old = 0x%x, new=0x%x. \n", __func__, old_status , ite_chip_data->base_power );
	if(ite_chip_data->base_power != old_status)
	{
		switch_set_state(&ite_chip_data->win_power_sdev, ite_chip_data->base_power);
	}
	
	return 0;
}
// add by Josh for ec switch read & write --

static ssize_t ite_pad_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ite_chip_data->pad_ec_version);
}

static ssize_t ite_pad_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", "0");
}

static ssize_t ite_base_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ite_chip_data->dock_ec_version);
}

static ssize_t ite_base_switch_state(struct switch_dev *sdev, char *buf)
{
	if(ite_chip_data->detach_attach == BASE_ATTACH)	
		return sprintf(buf, "%s\n", "10");
	else
		return sprintf(buf, "%s\n", "0");
}

static ssize_t ite_panel_owner_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ite_chip_data->pad_ec_version);
}

static ssize_t ite_panel_owner_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ite_chip_data->system_owner);
}

static void ite_base_switch_status_report(void)
{
	printk("[ITE]: %s \n",__func__);
	if(ite_chip_data->detach_attach == BASE_ATTACH)	
		switch_set_state(&ite_chip_data->dock_sdev, 10);
	else
		switch_set_state(&ite_chip_data->dock_sdev, 0);
}

static ssize_t ite_baseec_check_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%s\n", ite_chip_data->dock_ec_version);
}

static ssize_t ite_baseec_check_switch_state(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "%d\n", ite_chip_data->base_ec_check);
}

//static int ite_hw_init_function(void)
//{
//	int err = 0;
//	int usb_cable;
//	
//	if (!ite_i2c_hw_init(ite_chip_data->client)) {
//		err = -EINVAL;
//		printk(KERN_ERR "[ITE]: %s: ite_i2c_hw_init failed!! \n", __func__);
//		return -1;
//	}
//			
//	ite_chip_data->usb_status_finish = 1;
//#ifdef CONFIG_USB_PENWELL_OTG
//	usb_cable = check_cable_status();
//	ite8566_usb_cable_status(usb_cable);
//#endif
//		
//	return 0;
//}

static int ite_init_other_drv_func(void)
{
	int ret = 0;
	int attach_status = 0, panel_owner = 0;
	
	printk("[ITE]: %s \n",__func__);
	attach_status = ite_detach_attach_function();
	panel_owner = ite_system_owner_function();
			
	if(attach_status == BASE_DETACH)
	{
		ret = ite_detach_notify_driver(SYSTEM_NONE);
	}
	else
	{
		if(panel_owner == SYSTEM_ANDROID)
		{
			ret = ite_attach_notify_driver(SYSTEM_ANDROID);
		}
		else
		{
			ret = ite_attach_notify_driver(SYSTEM_WINDOWS);
		}
		asusec_input_device_create();
		ret = ite_win8_power_switch_status_report();
	}
				
	ret = ite_android_power_status(S1);

	return 0;
}

static int ite_firmware_upgrade_init_func(void)
{
	int rc = 0, ret = 0;
	uint8_t buf_recv[ITE_REPORT_MAX_LENGTH];
	int hid_descr_len;
	int hid_report_len;
	int attach_status = 0;
	
	/* Fetch the length of HID description */
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DESCR_LENGTH_CMD, buf_recv, HID_DES_LENGTH_OFFSET);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE_update] %s: Get ITE_HID_DESCR_LENGTH_CMD failed \n", __func__);
		return -1;
	}
	hid_descr_len = buf_recv[0];
	printk("[ITE_update] %s: Get hid_descr_len = %d \n", __func__, hid_descr_len);
	mdelay(3);
		
	/* Fetch the lenght of HID report description */
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_DESCR_CMD, buf_recv, hid_descr_len);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE_update] %s: Get ITE_HID_DESCR_CMD failed \n", __func__);
		return -1;
	}
	hid_report_len = (buf_recv[5]<< 2 | buf_recv[4]);
	printk("[ITE_update] %s: Get hid_report_len = %d \n", __func__, hid_report_len );
	mdelay(3);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_POWER_ON_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_update] %s: Get ITE_HID_POWER_ON_CMD failed \n", __func__);
		return -1;
	}
	mdelay(3);
	
	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_RESET_CMD, buf_recv, 0);
	if (rc != 1)
	{
		printk(KERN_ERR "[ITE_update] %s: Get ITE_HID_RESET_CMD failed \n", __func__);
		return -1;
	}
	
	msleep(ITE_I2C_COMMAND_DELAY);

	rc = asus_ec_i2c_command(ite_chip_data->client, ITE_HID_REPORT_DESCR_CMD, buf_recv, hid_report_len);
	if (rc != 2)
	{
		printk(KERN_ERR "[ITE_update] %s: Get ITE_HID_REPORT_DESCR_CMD failed \n", __func__);
		return -1;
	}
	msleep(5);
		
	ret = ite_base_power_status();
	switch_set_state(&ite_chip_data->win_power_sdev, ite_chip_data->base_power);
	ret = ite_system_owner_function();
	attach_status = ite_detach_attach_function();
	ite_base_switch_status_report();
	ret = ite_get_ec_version();
	ret = ite_base_ec_fw_check();
	
	ret = ite_android_power_status(S1);
	
#ifdef CONFIG_TOUCHSCREEN_ATMEL_MXT1664S
	ret = enable_touch_called_by_EC();
#endif
					
	if(attach_status == BASE_DETACH)
	{
		printk("[ITE_update] %s: Base Detach. \n",__func__);
		msleep(500);
		ret = ite_detach_notify_driver(SYSTEM_NONE);
	}	
	else
	{
		printk("[ITE_update] %s: Base Attach. \n",__func__);
		printk("[ITE_update] %s: System Owner = 0x%x. \n",__func__, ite_chip_data->system_owner);
		printk("[ITE_update] %s: Base Power Status = 0x%x. \n",__func__, ite_chip_data->base_power);
		printk("[ITE_update] %s: firmware update owner = 0x%x. \n",__func__, ite_chip_data->fwupdate_owner);
		msleep(1000);
		
		if(ite_chip_data->fwupdate_owner == SYSTEM_ANDROID)
		{
			ret = ite_screen_change_function(SYSTEM_ANDROID);
			msleep(200);
			ret = ite_attach_notify_driver(SYSTEM_ANDROID);		
		}
		else
		{
			ret = ite_attach_notify_driver(SYSTEM_WINDOWS);
			msleep(200);
			ret = ite_atow_early_notify_driver(SYSTEM_WINDOWS);
			ret = ite_screen_change_function(SYSTEM_WINDOWS);
			ret = ite_atow_late_notify_driver(SYSTEM_WINDOWS);
		}
		asusec_input_device_create();
		ret = ite_win8_power_switch_status_report();
	}
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void ite_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	
	printk("[ITE] %s: powerbtn_press = %d ++ \n", __func__, ite_chip_data->powerbtn_press);
	
	if(ite_chip_data->powerbtn_press == 1)
	{
		ret = ite_android_power_status(EARLY_SUSPEND);
		ite_chip_data->early_powerbtn_press = 1;
	}
	else
	{ 
		ret = ite_android_power_status(AUTO_EARLY_SUSPEND);
		ite_chip_data->early_powerbtn_press = 0;
	}
	
	ite_chip_data->powerbtn_press = 0;
}

static void ite_late_resume(struct early_suspend *h)
{
	int ret = 0;
	
	printk("[ITE] %s \n", __func__);
	ite_chip_data->powerbtn_press = 0;
	ret = ite_android_power_status(S1);
}
#endif

static int __devinit ite_probe(struct i2c_client *client,
								const struct i2c_device_id *id)
{
	struct asus_ec_i2c_platform_data *pdata;
	int err = 0;
	
//	int usb_cable;
			
	printk("[ITE]: %s ++ \n",__func__);
		
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[ITE] %s:[%d]: i2c check functionality error.\n", __func__, __LINE__);
		err = -ENODEV;
		goto probe_err_check_functionality_failed;
	}

	ite_chip_data = kzalloc(sizeof(struct ite_chip), GFP_KERNEL);
	if (ite_chip_data == NULL) {
		printk(KERN_ERR "[ITE] %s:[%d]: allocate ite_chip failed.\n", __func__, __LINE__);
		err = -ENOMEM;
		goto probe_err_alloc_data_failed;
	}

	ite_chip_data->asusec_wq = create_singlethread_workqueue("asusec_wq");
	if (!ite_chip_data->asusec_wq) {
		printk(KERN_ERR "[ITE] %s:[%d]: create workqueue failed.\n", __func__, __LINE__);
		err = -ENOMEM;
		goto probe_err_create_wq_failed;
	}
	
	ite_chip_data->hw_id = Read_HW_ID();
	
	ite_chip_data->client = client;
	i2c_set_clientdata(client, ite_chip_data);
	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		ite_chip_data->int0_gpio = pdata->int0_gpio;
		ite_chip_data->int1_gpio = pdata->int1_gpio;
	}
	printk("[ITE] %s:[%d]: int0_gpio =%d, int1_gpio=%d \n", __func__, __LINE__, ite_chip_data->int0_gpio, ite_chip_data->int1_gpio);
		
	mutex_init(&ite_chip_data->mutex_lock);
	mutex_init(&ite_chip_data->mutex_lock_irq);
	
	wake_lock_init(&ite_chip_data->wake_lock, WAKE_LOCK_SUSPEND, "ite_wake_lock");
	wake_lock_init(&ite_chip_data->wake_lock_timeout, WAKE_LOCK_SUSPEND, "ite_wake_lock_timeout");
	
//	INIT_WORK(&ite_chip_data->work_0x68, ite_work_0x68);
	INIT_WORK(&ite_chip_data->work_0x66, ite_work_0x66);
		
	ite_ecram_init(client);
	err = ite_get_ec_version();
		
#ifdef ENABLE_FIRMWARE_UPGRADE	
	ite_predefine_init(client);
#endif
	
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	printk("[ITE]:os entry_mode = %d, factory_mode =%d \n", entry_mode, factory_mode);
	if((factory_mode == 2) && (entry_mode == 1))
	{
		INIT_DELAYED_WORK(&ite_chip_data->ite_firmware_upgrade_work, ite_self_firmware_upgrade);
	}
#endif
//	INIT_DELAYED_WORK(&ite_chip_data->ite_hw_init_work, ite_hw_init_function);
	INIT_DELAYED_WORK(&ite_chip_data->ite_init_other_drv_work, ite_init_other_drv_func);
	INIT_DELAYED_WORK(&ite_chip_data->ite_firmware_upgrade_init_work, ite_firmware_upgrade_init_func);
			
	ite_chip_data->init_success = 0;
	ite_chip_data->usb_status_finish = 0;
	ite_chip_data->fw_up_process = 0;
	ite_chip_data->base_power = WIN8_DETACH;
	ite_chip_data->als_status = 0x00;
	ite_chip_data->gauge_fw_update = 0x00;
	ite_chip_data->detach_attach = BASE_DETACH;
	ite_chip_data->modifier_key_press = 0;
	ite_chip_data->modifier_key_release = 0;
	ite_chip_data->switch_process = 0;
	ite_chip_data->powerbtn_press = 0;
	ite_chip_data->early_powerbtn_press = 0;
	ite_chip_data->fwupdate_owner = 0;
	ite_chip_data->base_ec_check = 0;
		
	ite_chip_data->kp_enabled = true;
	ite_chip_data->attrs.attrs = ite_attr;
	err = sysfs_create_group(&client->dev.kobj, &ite_chip_data->attrs);
    if(err){
        printk(KERN_ERR "[ITE]: %s: Not able to create the sysfs\n", __func__);
		goto probe_err_device_create_file_failed;
    }
 	 	
	//*init EC_KB_INT# pin*/
	err = gpio_request(ite_chip_data->int0_gpio, "asus-ec-irq0");
	if(err < 0)
		printk(KERN_ERR "Failed to request GPIO%d (asus-ec-interrupt_0) error=%d\n", ite_chip_data->int0_gpio, err);

	err = gpio_direction_input(ite_chip_data->int0_gpio);
	if (err){
		printk(KERN_ERR "Failed to set interrupt direction, error=%d\n", err);
		goto probe_err_gpio_direction_input_failed;
	}
	
	ite_chip_data->irq0 = gpio_to_irq(ite_chip_data->int0_gpio);
	printk("[ITE]: intr0_gpio=%d, irq0=%d \n", ite_chip_data->int0_gpio, ite_chip_data->irq0);
	
	if(request_irq(ite_chip_data->irq0, ite_irq_0, IRQF_TRIGGER_FALLING, ite_chip_data->client->name, ite_chip_data->client))
	{
		printk(KERN_ERR "[ITE]: %s: Can't allocate irq \n", __func__);
		goto probe_err_request_irq_failed;
	}
		
	enable_irq_wake(ite_chip_data->irq0);
	
	//*init ALS_INT# pin*/
	err = gpio_request(ite_chip_data->int1_gpio, "asus-ec-irq1");
	if(err < 0)
		printk(KERN_ERR "Failed to request GPIO%d (asus-ec-interrupt_1) error=%d\n", ite_chip_data->int1_gpio, err);

	err = gpio_direction_input(ite_chip_data->int1_gpio);
	if (err){
		printk(KERN_ERR "Failed to set interrupt direction, error=%d\n", err);
		goto probe_err_gpio_direction_input_failed;
	}
	
	ite_chip_data->irq1 = gpio_to_irq(ite_chip_data->int1_gpio);
	printk("[ITE]: intr1_gpio=%d, irq1=%d \n", ite_chip_data->int1_gpio, ite_chip_data->irq1);
	
	if(request_irq(ite_chip_data->irq1, ite_irq_1, IRQF_TRIGGER_FALLING, "ITE8566_ECRAM", &ecram_client))
	{
		printk(KERN_ERR "[ITE]: %s: Can't allocate irq \n", __func__);
		goto probe_err_request_irq_failed;
	}
	
//	enable_irq(ite_chip_data->irq1);
	
/*************IO control setting***************/
	err = misc_register(&ite_misc_dev);
	if (err < 0) {
		printk( "[ITE]:%s: could not register ITE misc device\n",__func__);
		goto probe_misc_device_failed;
	}
/*************IO control setting***************/

/*************IO control setting***************/
	err = misc_register(&asusdec_dev);
	if (err < 0) {
		printk( "[ITE]:%s: could not register ITE misc device\n",__func__);
		goto probe_misc_device_failed;
	}
/*************IO control setting***************/
		
	msleep(10);
	
	//ec switch add by josh for win8 power
	ite_chip_data->win_power_sdev.name = WIN_POWER_SDEV_NAME;
	ite_chip_data->win_power_sdev.print_name = ite_win8_power_switch_name;
	ite_chip_data->win_power_sdev.print_state = ite_win8_power_switch_state;
	if(switch_dev_register(&ite_chip_data->win_power_sdev) < 0){
		printk("[ITE]: switch_dev_register for win8_power failed!\n");
	}
	switch_set_state(&ite_chip_data->win_power_sdev, WIN8_DETACH);
	
	//add switeh device for report pad ec version
	ite_chip_data->pad_sdev.name = PAD_SDEV_NAME;
	ite_chip_data->pad_sdev.print_name = ite_pad_switch_name;
	ite_chip_data->pad_sdev.print_state = ite_pad_switch_state;
	if(switch_dev_register(&ite_chip_data->pad_sdev) < 0){
		printk("[ITE]: switch_dev_register for pad failed!\n");
	}
	switch_set_state(&ite_chip_data->pad_sdev, 0);
	
	//add switeh device for report base ec version
	ite_chip_data->dock_sdev.name = DOCK_SDEV_NAME;
	ite_chip_data->dock_sdev.print_name = ite_base_switch_name;
	ite_chip_data->dock_sdev.print_state = ite_base_switch_state;
	if(switch_dev_register(&ite_chip_data->dock_sdev) < 0){
		printk("[ITE]: switch_dev_register for dock failed!\n");
	}
	switch_set_state(&ite_chip_data->dock_sdev, BASE_DETACH);
	
	//add broadcast system owner to AP
	ite_chip_data->owner_sdev.name = OWNER_SDEV_NAME;
	ite_chip_data->owner_sdev.print_name = ite_panel_owner_switch_name;
	ite_chip_data->owner_sdev.print_state = ite_panel_owner_switch_state;
	if(switch_dev_register(&ite_chip_data->owner_sdev) < 0){
		printk("[ITE]: switch_dev_register for owner_sdev failed!\n");
	}	
	switch_set_state(&ite_chip_data->owner_sdev, SYSTEM_ANDROID);
	
	//add check base ec version
	ite_chip_data->baseec_sdev.name = BASEEC_SDEV_NAME;
	ite_chip_data->baseec_sdev.print_name = ite_baseec_check_switch_name;
	ite_chip_data->baseec_sdev.print_state = ite_baseec_check_switch_state;
	if(switch_dev_register(&ite_chip_data->baseec_sdev) < 0){
		printk("[ITE]: switch_dev_register for baseec_sdev failed!\n");
	}	
	switch_set_state(&ite_chip_data->baseec_sdev, 0);	
				
	//proc create
	ite_create_proc_fw_file();
	
	if((!strncmp(ite_chip_data->pad_ec_version, "gTcp", 4))||((!strncmp(ite_chip_data->pad_ec_version, "g1cp", 4))))
	{
		if (!ite_i2c_hw_init(ite_chip_data->client)) {
			err = -EINVAL;
			printk(KERN_ERR "[ITE]: %s: ite_i2c_hw_init failed!! \n", __func__);
			goto probe_err_i2c_hw_init;
		}
				
		ite_chip_data->usb_status_finish = 1;
	#ifdef CONFIG_USB_PENWELL_OTG	
	//	usb_cable = check_cable_status();
		ite8566_usb_cable_status(usb_cable_init_value);
	#endif		
	
		ite_ram_notify_os_class();
	}
	
#ifdef CONFIG_HAS_EARLYSUSPEND
    ite_chip_data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
    ite_chip_data->early_suspend.suspend = ite_early_suspend;
    ite_chip_data->early_suspend.resume = ite_late_resume;
    register_early_suspend(&ite_chip_data->early_suspend);
#endif
	

	if((factory_mode == 2) && (entry_mode == 1))
	{
		ite_create_proc_factory_file();
	#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
		printk("[ITE]: factory image trigger firmwaer upgrade. \n");
		queue_delayed_work(ite_chip_data->asusec_wq, &ite_chip_data->ite_firmware_upgrade_work, 10*HZ);
	#endif
	}

	
	printk("[ITE]: ASUS EC Driver ver:%s . \n", DRIVER_VERSION);
	
	return 0;

probe_err_i2c_hw_init:
probe_misc_device_failed:
probe_err_request_irq_failed:
//	free_irq(ite_chip_data->irq, ite_chip_data);
probe_err_gpio_direction_input_failed:
//	input_unregister_device(ite_chip_data->input_dev);
//	gpio_free(ite_chip_data->int0_gpio);
//	gpio_free(ite_chip_data->int1_gpio);
//probe_err_input_register_device_failed:
//	sysfs_remove_group(&client->dev.kobj, &ite_chip_data->attrs);
probe_err_device_create_file_failed:
//	if (ite_chip_data->input_dev)
//       input_free_device(ite_chip_data->input_dev);
//probe_err_input_dev_alloc_failed:
probe_err_create_wq_failed:
//	if (ite_chip_data->asusec_wq)
 //       destroy_workqueue(ite_chip_data->asusec_wq);

	//kfree(ite_chip_data);
probe_err_alloc_data_failed:
probe_err_check_functionality_failed:
	
	return err;
}

static int ite_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;
	
	printk("[ITE] %s: early_powerbtn_press = %d ++ \n", __func__, ite_chip_data->early_powerbtn_press);
	
	if(ite_chip_data->early_powerbtn_press == 1)
	{
		ret = ite_android_power_status(S3);
	}
	else
	{
		ret = ite_android_power_status(AUTO_SUSPEND);
	}
	
	ite_chip_data->early_powerbtn_press = 0;
	
	return 0;
}

static int ite_resume(struct i2c_client *client)
{
	int ret = 0;
	
	printk("[ITE] %s: powerbtn_press = %d, early_powerbtn_press = %d ++ \n", __func__, ite_chip_data->powerbtn_press, ite_chip_data->early_powerbtn_press);
	ite_chip_data->powerbtn_press = 0;
	ret = ite_android_power_status(LATE_RESUME);
	return 0;
}

static int __devexit ite_remove(struct i2c_client *client)
{
	
	disable_irq_wake(ite_chip_data->irq0);
	free_irq(ite_chip_data->irq0, ite_chip_data);
	disable_irq_wake(ite_chip_data->irq1);
	free_irq(ite_chip_data->irq1, ite_chip_data);
	cancel_work_sync(&ite_chip_data->work_0x68);
	cancel_work_sync(&ite_chip_data->work_0x66);
	
	input_unregister_device(ite_chip_data->input_dev);
	ite_chip_data->input_dev_status = 0;
	
	sysfs_remove_group(&client->dev.kobj, &ite_chip_data->attrs);
	
	kfree(ite_chip_data);
	
	return 0;
}

static struct i2c_driver ite_i2c_driver;

static const struct i2c_device_id asusec_id[] = {
	{ ASUS_EC_NAME, 0},
	{ }
};

static struct i2c_driver ite_i2c_driver = {
	.driver = {
		.name	= ASUS_EC_NAME,
		.owner  = THIS_MODULE,
	},
	.probe		= ite_probe,
	.remove		= __devexit_p(ite_remove),
	.suspend    = ite_suspend,
	.resume     = ite_resume,
	.id_table	= asusec_id,
};
MODULE_DEVICE_TABLE(i2c, asusec_id);

static int __init ite_init(void)
{
	printk("[ITE]: %s ++ \n", __func__);
	probe_status = 0;
	return i2c_add_driver(&ite_i2c_driver);
}

static void __exit ite_exit(void)
{
	i2c_del_driver(&ite_i2c_driver);
}
module_init(ite_init);
module_exit(ite_exit);

MODULE_AUTHOR("Joe_CH Chen <joe_ch_chen@asus.com>");
MODULE_DESCRIPTION("ITE EC driver");
MODULE_LICENSE("GPL");
