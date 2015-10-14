/*
 * ite.h - Configuration for ITE keypad driver.
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
 */

#ifndef __LINUX_ITE_H
#define __LINUX_ITE_H

#include <linux/types.h>

/*
 * Largest keycode that the chip can send, plus one,
 * so keys can be mapped directly at the index of the
 * ITE keycode instead of subtracting one.
 */
//#define ITE_KEYMAP_SIZE	(0x7f + 1)
#define ITE_KEYMAP_SIZE	256
//#define ITE_KEYMAP_SIZE	512

#define ITE_NUM_PWMS		3

u8 ITE_CMD_FLASH[6]     ={0x6 , 0x6 ,0x2 ,0x0,0x0,0x0};//cmd len subcmd addr2 addr1 addr0
//u8 ITE_CMD_FLASH_INITFW[6] = {0x6 , 0x6 ,0x1 ,0x49,0x54,0x45};//cmd len subcmd payload[0-2] = { "I", "T", "E"} = { 0x49,0x54,0x45}
u8 ITE_CMD_FLASH_INITFW[6] = {0x6 , 0x4 ,0x1 ,0x49,0x54,0x45};//cmd len subcmd payload[0-2] = { "I", "T", "E"} = { 0x49,0x54,0x45}
u8 ITE_CMD_FLASH_FINALIZEFW[3] = {0x6 , 0x1 ,0x4};//cmd len subcmd 
//u8 ITE_CMD_FLASH_FINALIZEFW[6] = {0x6 , 0x4 ,0x4 ,0x49,0x54,0x45};//cmd lensubcmd payload[0-2] = { "I", "T", "E"} = { 0x49,0x54,0x45}
//u8 ITE_CMD_INIT_EC[4]   ={0x3 , 0x2 ,0x0 ,0x1};//cmd len2 (subcmd payload)

u8 ITE_CMD_INIT_EC[4]   ={0x3 , 0x2 ,0x0 ,0x1};//cmd len2 (subcmd payload)
u8 ITE_CMD_INIT_KB[3]   ={0x4 , 0x1 ,0xf4};//cmd len1 subcmd
u8 ITE_CMD_INIT_TP[4]   ={0x5 , 0x2 ,0x3 ,0x3};//cmd len2 subcmd payload

u8 ITE_CMD_BATTERY[3]   ={0x2 , 0x2 ,0x1 }; //battery => len(2) cmd(2) subcmd(1)
//u8 ITE_CMD_BATTERY_RSOC[3]              ={0x2 , 0x2 ,0x0 }; //battery => len(2) cmd(2) subcmd(1) for Relative State of Charge
//u8 ITE_CMD_BATTERY_VOL[3]               ={0x2 , 0x2 ,0x1 }; //battery => len(2) cmd(2) subcmd(1) for voltage
//u8 ITE_CMD_BATTERY_AVG_CURRENT[3]       ={0x2 , 0x2 ,0x4}; //battery => len(2) cmd(2) subcmd(4) for average current
//u8 ITE_CMD_BATTERY_TEMP[3]              ={0x2 , 0x2 ,0xA }; //battery => len(2) cmd(2) subcmd(9) for temp K

u8 ITE_CMD_TEST[3]   ={0x2 , 0x2 ,0 }; //battery => len(2) cmd(2) subcmd(0)

/* Addresses to scan */
static const unsigned short normal_i2c[] = { 0x45, I2C_CLIENT_END };

#define EVENT_ITE_KEYBOARD	0x80
#define EVENT_ITE_TOUCHPAD	0x81

#define DEV_IOCTLID 0xD0
#define IOCTL_CHKREADY _IOW(DEV_IOCTLID, 24, int)
#define IOCTL_PREREAD _IOW(DEV_IOCTLID, 22, int)
#define IOCTL_PREWRITE _IOW(DEV_IOCTLID, 12, int)
#define IOCTL_READ _IOW(DEV_IOCTLID, 20, int)
#define IOCTL_WRITE _IOW(DEV_IOCTLID, 10, int)
#define IOCTL_FINALIZEFW _IOW(DEV_IOCTLID, 6, int)
#define IOCTL_INITFW _IOW(DEV_IOCTLID, 5, int)
#define IOCTL_RESET _IOW(DEV_IOCTLID, 0, int)
#define IOCTL_ITE_TEST _IOW(DEV_IOCTLID, 4, int)

#define CMD_W 0
#define CMD_R 1

/*************IO control setting***************/
#define ASUSEC_TP_ON			1
#define ASUSEC_TP_OFF			0
#define ASUSEC_EC_ON			1
#define ASUSEC_EC_OFF			0
#define ASUSEC_FW_UPGRADE_ON	1
#define ASUSEC_FW_UPGRADE_OFF	0
#define ASUSEC_WIFI_ON			1
#define ASUSEC_WIFI_OFF			0
#define ASUSEC_ACCESS_ON		1
#define ASUSEC_ACCESS_OFF		0
#define ASUSEC_CPAS_LED_ON		1
#define ASUSEC_CPAS_LED_OFF		0
#define ASUSEC_IOC_MAGIC		0xF4
#define ASUSEC_IOC_MAXNR		10
#define ASUSEC_INIT					_IOR(ASUSEC_IOC_MAGIC,	1,	int)
#define ASUSEC_FW_UPDATE_FLAG		_IOR(ASUSEC_IOC_MAGIC,	2,	int)
#define ASUSEC_FW_UPDATE_PROCESS	_IOR(ASUSEC_IOC_MAGIC,	3,	int)
#define ASUSEC_TP_CONTROL			_IOR(ASUSEC_IOC_MAGIC,	4,	int)
#define ASUSEC_EC_WAKEUP			_IOR(ASUSEC_IOC_MAGIC,	5,	int)
#define ASUSEC_CHANGE_OWNER			_IOR(ASUSEC_IOC_MAGIC,	6,	int)
#define ASUSEC_WIFI					_IOR(ASUSEC_IOC_MAGIC,	7,	int)
#define ASUSEC_DATA_ACCESS			_IOR(ASUSEC_IOC_MAGIC,	8,	int)
#define ASUSEC_CPASLOCK_LED			_IOR(ASUSEC_IOC_MAGIC,	9,	int)
#define ASUSEC_BASE_EC_CHECK		_IOR(ASUSEC_IOC_MAGIC,	10,	int)

#define ASUSEC_FW_UPGRADE_SUCCESS	"0"
#define ASUSEC_FW_UPGRADE_FAIL		"1"
#define ASUSEC_FW_UPGRADE_PROCESS	"2"
#define ASUSEC_FW_UPGRADE_INIT		"3"

/*************IO control setting***************/
#define ASUSDEC_EC_ON			1
#define ASUSDEC_EC_OFF			0
#define ASUSDEC_IOC_MAGIC		0xf4
#define ASUSDEC_IOC_MAXNR		7
#define ASUSDEC_EC_WAKEUP		_IOR(ASUSDEC_IOC_MAGIC,	6,	int)
/*************IO control setting***************/

/*************IO control setting***************/
#define WIN_POWER_SDEV_NAME	"win_power"
#define PAD_SDEV_NAME		"pad"
#define DOCK_SDEV_NAME		"dock"
#define OWNER_SDEV_NAME		"panel_owner"
#define BASEEC_SDEV_NAME	"base_ec_check"

//#define dprintk
//#define dprintk printk

#define ITE8566_HW_ADDR		0x5B
#define ITE8566_EC_ADDR		0x66

//#define EFLASH_CMD_VERSION		0xEC
#define EFLASH_CMD_READ_ID			0x9F
#define EFLASH_CMD_BYTE_PROGRAM	 	0x02
#define EFLASH_CMD_WRITE_DISABLE 	0x04
#define EFLASH_CMD_READ_STATUS	 	0x05
#define EFLASH_CMD_WRITE_ENABLE		0x06
#define EFLASH_CMD_FAST_READ		0x0B
#define EFLASH_CMD_CHIP_ERASE	 	0x60
#define EFLASH_CMD_AAI_WORD_PROGRAM 0xAD
#define EFLASH_MAX_RETRY_COUNT		5

#define ASUSEC_IRQ0_ID_KEYBOARD     0x01
#define ASUSEC_IRQ0_EVENT_1         0x03
#define ASUSEC_IRQ0_EVENT_2         0x04
#define ASUSEC_IRQ0_GEMINI_EVENT    0x05
#define ASUSEC_IRQ0_LIGHT_SENSOR    0x06
#define ASUSEC_IRQ0_CHARGER_IC      0x0B

#define	ASUSEC_IRQ1_COMPARE_FAIL	0x01
#define	ASUSEC_IRQ1_COMPARE_PASS	0x02
#define	ASUSEC_IRQ1_I2C_FAIL		0x04
#define	ASUSEC_IRQ1_BASE_STATUS		0x08

#define ASUSEC_KEY_RELASE			0x00

#define ASUSEC_KEY_LEFT_CTRL		0x01
#define ASUSEC_KEY_LEFT_SHITE		0x02
#define ASUSEC_KEY_LEFT_CTRL_SHITE	0x03
#define ASUSEC_KEY_LEFT_ALT			0x04
#define ASUSEC_KEY_LEFT_CTRL_ALT	0x05
#define ASUSEC_KEY_LEFT_WINDOWS		0x08

#define ASUSEC_KEY_RIGHT_CTRL		0x10
#define ASUSEC_KEY_RIGHT_SHITE		0x20
#define ASUSEC_KEY_RIGHT_CTRL_SHITE	0x30
#define ASUSEC_KEY_RIGHT_ALT		0x40
#define ASUSEC_KEY_RIGHT_CTRL_ALT	0x50

#define ASUSEC_FNKEY_SLEEP			0x3A
#define ASUSEC_FNKEY_FLIGHT_MODE	0x3B
#define ASUSEC_FNKEY_KEYLED_DOWN	0x3C
#define ASUSEC_FNKEY_KEYLED_UP		0x3D
#define ASUSEC_FNKEY_BACKLIGHT_DOWN	0x3E
#define ASUSEC_FNKEY_BACKLIGHT_UP	0x3F
#define ASUSEC_FNKEY_LCD_OFF		0x40
#define ASUSEC_FNKEY_LCD_CHG		0x41
#define	ASUSEC_FNKEY_TOUCHPAD		0x42
#define	ASUSEC_FNKEY_MUTE			0x43
#define	ASUSEC_FNKEY_VOLUME_DOWN	0x44
#define	ASUSEC_FNKEY_VOLUME_UP		0x45
#define	ASUSEC_FNKEY_POWER_4GEAR	0x2C
#define	ASUSEC_FNKEY_SPLENDID		0x06
#define	ASUSEC_FNKEY_CAMERA			0x19
#define	ASUSEC_FNKEY_AUTO_LIGHT		0x04
#define ASUSEC_FNKEY_PAGEUP			0x52
#define ASUSEC_FNKEY_PAGEDOWN		0x51
#define ASUSEC_FNKEY_HOME			0x50
#define ASUSEC_FNKEY_END			0x4f

#define	ASUSEC_KEY_GEMINI_KEY		0x01
#define	ASUSEC_KEY_BASE_ATTACH		0x02
#define	ASUSEC_KEY_BASE_DETACH		0x04
#define	ASUSEC_KEY_EC_SWITCH		0x08
#define	ASUSEC_KEY_OWNER_ANDROID	0x10
#define	ASUSEC_KEY_OWNER_WINDOWS	0x20
#define	ASUSEC_KEY_BASE_STATUS		0x40
#define	ASUSEC_KEY_WAKEUP_SOC		0x80

#define	ASUSEC_LIGHT_SENSOR_RESET		0x01
#define	ASUSEC_LIGHT_SENSOR_ENABLE		0x02
#define	ASUSEC_LIGHT_SENSOR_DISABLE		0x04
#define	ASUSEC_LIGHT_SENSOR_RAWDATA		0x08

#define ASUSEC_KEYCODE_LAUNCH_SETTING	KEY_F13
#define ASUSEC_KEYCODE_SCREEN_CAPTURE	KEY_F14
#define ASUSEC_KEYCODE_LAUNCH_CAMERA	KEY_F15
#define ASUSEC_KEYCODE_SWITCH_ANDROID	KEY_F16
#define ASUSEC_KEYCODE_BASE_DETACH		KEY_F17
#define ASUSEC_KEYCODE_BASE_ATTACH		KEY_F18
#define ASUSEC_KEYCODE_NO_FN			KEY_F19
#define ASUSEC_KEYCODE_SPLENDID			KEY_F20
#define ASUSEC_KEYCODE_MODE_SWITCH		KEY_F21
//#define ASUSEC_KEYCODE_POWER_4GEAR	KEY_F22
#define ASUSEC_KEYCODE_FLIGHT_MODE 		KEY_F22
#define ASUSEC_KEYCODE_LCDOFF			KEY_DISPLAY_OFF
#define ASUSEC_KEYCODE_BRIGHTNESS_AUTO	KEY_F23
#define ASUSEC_KEYCODE_TOUCHPAD			KEY_F24

struct ite_chip;
struct asusec_access_methods {
        int (*read)(u8 reg, int *rt_value, int b_single,
                struct ite_chip *ite_chip_data);
};

struct asusdec_keypad{
	int value;
	int input_keycode;
	int extend;	
};

struct ite_chip {
	/* device lock */
	struct mutex mutex_lock;
	struct mutex mutex_lock_irq;
	struct i2c_client *client;
	struct work_struct work_0x68;
	struct work_struct work_0x66;
	struct input_dev *input_dev;
	struct workqueue_struct *asusec_wq;
	struct attribute_group attrs;
	struct asusdec_keypad keypad_data;
	struct wake_lock wake_lock;
	struct wake_lock wake_lock_timeout;
	struct delayed_work ite_firmware_upgrade_work;
//	struct delayed_work ite_hw_init_work;
	struct delayed_work ite_init_other_drv_work;
	struct delayed_work ite_firmware_upgrade_init_work;
	struct switch_dev win_power_sdev;
	struct switch_dev pad_sdev;
	struct switch_dev dock_sdev;
	struct switch_dev owner_sdev;
	struct switch_dev baseec_sdev;
	struct miscdevice ite_misc_dev;
	struct early_suspend early_suspend;
		
	bool kp_enabled;
	char input_phys_name[32];
	char pad_ec_version[32];
	char dock_ec_version[32];
	unsigned char old[8];
	unsigned char new[14];
	unsigned short keymap[ITE_KEYMAP_SIZE];
	unsigned int modifier_repeat_key;

	int id;
	int voltage_uV;
	int current_uA;
	int temp_C;
	int charge_rsoc;
	int irq0;
	int irq1;
	int int0_gpio;	//EC_KB_INT#:GPIO_84
	int int1_gpio;	//ALS_INT#:GPIO_77
	int version_checksum;
	int ec_firmware_upgrade_proceed;
	int detach_attach; //0x00:detach / 0x01:attach
	int system_owner; //0x00:no use / 0x01:Android / 0x02:Windows
	int base_power; //S0(0x00)/S3(0x03)/S5(0x05)/WindowsSOC Detach(0xFF)
	int init_success;// 0:fail, 1:success
	int fw_up_process;
	int gauge_fw_update;// 0:disable, 1:enable
	int usb_status_finish;
	int input_dev_status;
	int charger_status; // 0x0001: Initial OK ,  0x0000: Initial Fail
	int als_level;
	int als_lux_data;
	int als_raw_data;
	int als_status; //0x00: Initial OK, 0x01: Initial Fail
	int usb_switch_mode; //1 = client , 0 = host
	int win8_test_mode; // 0x00: normal mode, 0x01: Test mode
	int modifier_key_press;
	int modifier_key_release;
	int hw_id;
	int switch_process;
	int powerbtn_press;
	int early_powerbtn_press;
	int fwupdate_owner;
	int base_ec_check;
		
	int pad_bat_present;
	int pad_bat_status;
	int pad_bat_temperature;
	int pad_bat_voltage;
	int pad_bat_current;
	int pad_bat_capacity;	//percentage
	int pad_bat_energy;
	int pad_bat_avg_time_to_empty;
	int pad_bat_remaining_capacity;
	int pad_bat_avg_time_to_full;
	
	int dock_bat_present;
	int dock_bat_status;
	int dock_bat_temperature;
	int dock_bat_voltage;
	int dock_bat_current;
	int dock_bat_capacity;	//percentage
	int dock_bat_energy;
	int dock_bat_avg_time_to_empty;
	int dock_bat_remaining_capacity;
	int dock_bat_avg_time_to_full;

};

struct ite_chip *ite_chip_data;

#define client_to_ite(c)	container_of(c, struct ite_chip, client)
#define dev_to_ite(d)	container_of(d, struct ite_chip, client->dev)
#define work_to_ite(w)	container_of(w, struct ite_chip, work)
#define cdev_to_pwm(c)		container_of(c, struct ite_pwm, cdev)
#define work_to_pwm(w)		container_of(w, struct ite_pwm, work)

//#define ITE_MAX_DATA 8

//struct ite_i2c_data {
	//struct i2c_client       *fake_client;
//};
//struct ite_i2c_data *ite_i2c_data_global;

struct work_struct *event;

void sendCmd(struct i2c_client *client,u8 *cmd,u8 rw);

#endif /* __LINUX_ITE_H */
