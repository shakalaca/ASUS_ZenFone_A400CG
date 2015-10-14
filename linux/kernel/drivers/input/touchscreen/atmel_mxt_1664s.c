/*
 * Atmel maXTouch Touchscreen driver
 *
 * Copyright (C) 2010 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */


#include <linux/file.h>
#include <linux/fs.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/i2c/atmel_mxt_1664s.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ite8566.h>
// Jui add ++
#include <linux/gpio.h>
#include <linux/mutex.h>
#include <linux/debugfs.h>
#include <linux/switch.h>

// Jui add --

// Jui-Chuan add for early suspend and late resume ++
#include <linux/earlysuspend.h>
// Jui-Chuan add for early suspend and late resume --

#include <linux/wakelock.h>

/* Version */
#define MXT_VER_20		20
#define MXT_VER_21		21
#define MXT_VER_22		22

/* Slave addresses */
#define MXT_APP_LOW		0x4a
#define MXT_APP_HIGH		0x4b
#define MXT_BOOT_LOW		0x26
#define MXT_BOOT_HIGH		0x27

/* Firmware */
#define MXT_FW_NAME		"maxtouch.fw"
#define MXT_FW_NAME_1664T	"maxtouch_1664t.fw"
/* Registers */
#define MXT_FAMILY_ID		0x00
#define MXT_VARIANT_ID		0x01
#define MXT_VERSION		0x02
#define MXT_BUILD		0x03
#define MXT_MATRIX_X_SIZE	0x04
#define MXT_MATRIX_Y_SIZE	0x05
#define MXT_OBJECT_NUM		0x06
#define MXT_OBJECT_START	0x07

#define MXT_OBJECT_SIZE		6

/* Object types */
#define MXT_DEBUG_DIAGNOSTIC_T37	37
#define MXT_GEN_MESSAGE_T5		5
#define MXT_GEN_COMMAND_T6		6
#define MXT_GEN_POWER_T7		7
#define MXT_GEN_ACQUIRE_T8		8
#define MXT_GEN_DATASOURCE_T53		53
#define MXT_TOUCH_MULTI_T9_DEFINE		9
#define MXT_TOUCH_MULTI_T100_DEFINE		100
u8 MXT_TOUCH_MULTI_T9;
#define MXT_TOUCH_KEYARRAY_T15		15
#define MXT_TOUCH_PROXIMITY_T23		23
#define MXT_TOUCH_PROXKEY_T52		52
#define MXT_PROCI_GRIPFACE_T20		20
#define MXT_PROCG_NOISE_T22		22
#define MXT_PROCI_ONETOUCH_T24		24
#define MXT_PROCI_TWOTOUCH_T27		27
#define MXT_PROCI_GRIP_T40		40
#define MXT_PROCI_PALM_T41		41
#define MXT_PROCI_TOUCHSUPPRESSION_T42	42
#define MXT_PROCI_STYLUS_T47		47
#define MXT_PROCG_NOISESUPPRESSION_T48	48
#define MXT_SPT_COMMSCONFIG_T18		18
#define MXT_SPT_GPIOPWM_T19		19
#define MXT_SPT_SELFTEST_T25		25
#define MXT_SPT_CTECONFIG_T28		28
#define MXT_SPT_USERDATA_T38		38
#define MXT_SPT_DIGITIZER_T43		43
#define MXT_SPT_MESSAGECOUNT_T44	44
#define MXT_SPT_CTECONFIG_T46		46

#define T56_DEFINE				56
#define T61_DEFINE                              61
#define T65_DEFINE				65
#define T66_DEFINE				66
#define T68_DEFINE                              68
#define T70_DEFINE				70
#define T71_DEFINE                              71
#define T72_DEFINE				72
#define T77_DEFINE				77
#define T79_DEFINE				79
/* MXT_GEN_COMMAND_T6 field */
#define MXT_COMMAND_RESET	0
#define MXT_COMMAND_BACKUPNV	1
#define MXT_COMMAND_CALIBRATE	2
#define MXT_COMMAND_REPORTALL	3
#define MXT_COMMAND_DIAGNOSTIC	5

/* MXT_GEN_POWER_T7 field */
#define MXT_POWER_IDLEACQINT	0
#define MXT_POWER_ACTVACQINT	1
#define MXT_POWER_ACTV2IDLETO	2

/* MXT_GEN_ACQUIRE_T8 field */
#define MXT_ACQUIRE_CHRGTIME	0
#define MXT_ACQUIRE_TCHDRIFT	2
#define MXT_ACQUIRE_DRIFTST	3
#define MXT_ACQUIRE_TCHAUTOCAL	4
#define MXT_ACQUIRE_SYNC	5
#define MXT_ACQUIRE_ATCHCALST	6
#define MXT_ACQUIRE_ATCHCALSTHR	7

/* MXT_TOUCH_MULTI_T9 field */
#define MXT_TOUCH_CTRL		0
#define MXT_TOUCH_XORIGIN	1
#define MXT_TOUCH_YORIGIN	2
#define MXT_TOUCH_XSIZE		3
#define MXT_TOUCH_YSIZE		4
#define MXT_TOUCH_BLEN		6
#define MXT_TOUCH_TCHTHR	7
#define MXT_TOUCH_TCHDI		8
#define MXT_TOUCH_ORIENT	9
#define MXT_TOUCH_MOVHYSTI	11
#define MXT_TOUCH_MOVHYSTN	12
#define MXT_TOUCH_NUMTOUCH	14
#define MXT_TOUCH_MRGHYST	15
#define MXT_TOUCH_MRGTHR	16
#define MXT_TOUCH_AMPHYST	17
#define MXT_TOUCH_XRANGE_LSB	18
#define MXT_TOUCH_XRANGE_MSB	19
#define MXT_TOUCH_YRANGE_LSB	20
#define MXT_TOUCH_YRANGE_MSB	21
#define MXT_TOUCH_XLOCLIP	22
#define MXT_TOUCH_XHICLIP	23
#define MXT_TOUCH_YLOCLIP	24
#define MXT_TOUCH_YHICLIP	25
#define MXT_TOUCH_XEDGECTRL	26
#define MXT_TOUCH_XEDGEDIST	27
#define MXT_TOUCH_YEDGECTRL	28
#define MXT_TOUCH_YEDGEDIST	29
#define MXT_TOUCH_JUMPLIMIT	30

/* MXT_PROCI_GRIPFACE_T20 field */
#define MXT_GRIPFACE_CTRL	0
#define MXT_GRIPFACE_XLOGRIP	1
#define MXT_GRIPFACE_XHIGRIP	2
#define MXT_GRIPFACE_YLOGRIP	3
#define MXT_GRIPFACE_YHIGRIP	4
#define MXT_GRIPFACE_MAXTCHS	5
#define MXT_GRIPFACE_SZTHR1	7
#define MXT_GRIPFACE_SZTHR2	8
#define MXT_GRIPFACE_SHPTHR1	9
#define MXT_GRIPFACE_SHPTHR2	10
#define MXT_GRIPFACE_SUPEXTTO	11

/* MXT_PROCI_NOISE field */
#define MXT_NOISE_CTRL		0
#define MXT_NOISE_OUTFLEN	1
#define MXT_NOISE_GCAFUL_LSB	3
#define MXT_NOISE_GCAFUL_MSB	4
#define MXT_NOISE_GCAFLL_LSB	5
#define MXT_NOISE_GCAFLL_MSB	6
#define MXT_NOISE_ACTVGCAFVALID	7
#define MXT_NOISE_NOISETHR	8
#define MXT_NOISE_FREQHOPSCALE	10
#define MXT_NOISE_FREQ0		11
#define MXT_NOISE_FREQ1		12
#define MXT_NOISE_FREQ2		13
#define MXT_NOISE_FREQ3		14
#define MXT_NOISE_FREQ4		15
#define MXT_NOISE_IDLEGCAFVALID	16

/* MXT_SPT_COMMSCONFIG_T18 */
#define MXT_COMMS_CTRL		0
#define MXT_COMMS_CMD		1

/* MXT_SPT_CTECONFIG_T28 field */
#define MXT_CTE_CTRL		0
#define MXT_CTE_CMD		1
#define MXT_CTE_MODE		2
#define MXT_CTE_IDLEGCAFDEPTH	3
#define MXT_CTE_ACTVGCAFDEPTH	4
#define MXT_CTE_VOLTAGE		5

#define MXT_VOLTAGE_DEFAULT	2700000
#define MXT_VOLTAGE_STEP	10000

/* Define for MXT_GEN_COMMAND_T6 */
#define MXT_BOOT_VALUE		0xa5
#define MXT_BACKUP_VALUE	0x55
#define MXT_BACKUP_TIME		25	/* msec */
#define MXT_RESET_TIME		105	/* msec */

#define MXT_FWRESET_TIME	175	/* msec */

/* Command to unlock bootloader */
#define MXT_UNLOCK_CMD_MSB	0xaa
#define MXT_UNLOCK_CMD_LSB	0xdc

/* Bootloader mode status */
#define MXT_WAITING_BOOTLOAD_CMD	0xc0	/* valid 7 6 bit only */
#define MXT_WAITING_FRAME_DATA	0x80	/* valid 7 6 bit only */
#define MXT_FRAME_CRC_CHECK	0x02
#define MXT_FRAME_CRC_FAIL	0x03
#define MXT_FRAME_CRC_PASS	0x04
#define MXT_APP_CRC_FAIL	0x40	/* valid 7 8 bit only */
#define MXT_BOOT_STATUS_MASK	0x3f

/* Touch status */
#define MXT_SUPPRESS		(1 << 1)
#define MXT_AMP			(1 << 2)
#define MXT_VECTOR		(1 << 3)
#define MXT_MOVE		(1 << 4)
#define MXT_RELEASE		(1 << 5)
#define MXT_PRESS		(1 << 6)
#define MXT_DETECT		(1 << 7)

#define MXT_MOVE2               (1 << 0)
#define MXT_PRESS2              (1 << 2)

/* Touch orient bits */
#define MXT_XY_SWITCH		(1 << 0)
#define MXT_X_INVERT		(1 << 1)
#define MXT_Y_INVERT		(1 << 2)

/* Touchscreen absolute values */
#define MXT_MAX_AREA		0xff

#define MXT_MAX_FINGER		10

// Jui-Chuan add for Self test ++
#define MXT_MSGR_T25_RUN_ALL_TESTS 0xFE
#define MXT_MSG_T25_STATUS 0x01
#define MXT_ADR_T25_CTRL 0x00
#define MXT_ADR_T25_CMD 0x01
char self_test_result[100];
int after_update_fw;
int is_fw_update_finish;
int touch_status_value;
int chip_in_bootloader;
int mxt1664t;
int system_owner;
int need_to_update_fw;
#define IPCMSG_COLD_RESET        0xF1
extern int rpmsg_send_generic_simple_command(u32 cmd, u32 sub);

extern int Read_HW_ID(void);

/*
 *build_vsersion mean TARGET_BUILD_VARIANT
 *eng:1
 *userdebug:2
 *user:3
*/
extern int build_version;


/*
 * entry_mode = 1; MOS
 * entry_mode = 2; recovery
 * entry_mode = 3; POS
 * entry_mode = 4; COS
*/
extern int entry_mode;

/* EC function */
extern int notify_EC(void);

extern int ite8566_switch_notify(void);
#if 1
// Add for new code base
/* Use for Android to WIN8, early notify */
extern int register_dock_atow_early_notifier(struct notifier_block *nb);
extern int unregister_dock_atow_early_notifier(struct notifier_block *nb);

/* Use for Android to WIN8, late notify */
extern int register_dock_atow_late_notifier(struct notifier_block *nb);
extern int unregister_dock_atow_late_notifier(struct notifier_block *nb);

/* Use for WIN8 to Android, late notify */
extern int register_dock_wtoa_late_notifier(struct notifier_block *nb);
extern int unregister_dock_wtoa_late_notifier(struct notifier_block *nb);

/* Use for Detaching */
extern int register_dock_detach_notifier(struct notifier_block *nb);
extern int unregister_dock_detach_notifier(struct notifier_block *nb);

/* Use for Attaching */
extern int register_dock_attach_notifier(struct notifier_block *nb);
extern int unregister_dock_attach_notifier(struct notifier_block *nb);

#else
// Add for old code base
extern int register_dock_notifier(struct notifier_block *nb);
extern int unregister_dock_notifier(struct notifier_block *nb);

#endif

static int touch_panel_irq_controller(struct notifier_block *this,unsigned long code, void *data);


#if 1
// Add for new code base
static struct notifier_block atow_early_dock_notifier = {
        .notifier_call =    touch_panel_irq_controller,
};

static struct notifier_block atow_late_dock_notifier = {
        .notifier_call =    touch_panel_irq_controller,
};

static struct notifier_block wtoa_late_dock_notifier = {
        .notifier_call =    touch_panel_irq_controller,
};

static struct notifier_block detach_dock_notifier = {
        .notifier_call =    touch_panel_irq_controller,
};

static struct notifier_block attach_dock_notifier = {
        .notifier_call =    touch_panel_irq_controller,
};

#else
// Add for old code base
static struct notifier_block dock_notifier = {
        .notifier_call =    touch_panel_irq_controller,
};
#endif

#if 1
// Add for new code base
#else
// Add for old code base
typedef enum {
WIN8_S0 = 0x00,
WIN8_S1 = 0x01,
WIN8_S3 = 0x03,
WIN8_S5 = 0x05
};
extern int ite_system_owner_function(void);
typedef enum {
SYSTEM_NONE = 0,
SYSTEM_ANDROID,
SYSTEM_WINDOWS
};
#endif
//0x00:android / 0x01:Android / 0x02:Windows

/***************/

// Jui-Chuan add for Self test --

// Newest FW version setting
#define TOUCH_SDEV_NAME "touch"
#define NEWEST_FW_VERSION 0x20
#define NEWEST_FW_BUILD 0xab
#define NEWEST_FW_VERSION_1664T 0x10
#define NEWEST_FW_BUILD_1664T 0xac


struct mxt_info {
	u8 family_id;
	u8 variant_id;
	u8 version;
	u8 build;
	u8 matrix_xsize;
	u8 matrix_ysize;
	u8 object_num;
};

struct mxt_object {
	u8 type;
	u16 start_address;
	u8 size;
	u8 instances;
	u8 num_report_ids;

	/* to map object and message */
	u8 max_reportid;
};

struct mxt_message {
	u8 reportid;
	u8 message[7];
	u8 checksum;
};

struct mxt_finger {
	int status;
	int x;
	int y;
	int area;
	int pressure;
};

/* Each client has this additional data */
struct mxt_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	const struct mxt_platform_data *pdata;
	struct mxt_object *object_table;
	struct mxt_info info;
	struct mxt_finger finger[MXT_MAX_FINGER];
	unsigned int irq;
	unsigned int max_x;
	unsigned int max_y;


// 	platform data ++ 
	unsigned int x_line;
	unsigned int y_line;
	unsigned int x_size;
	unsigned int y_size;
	unsigned int blen;
	unsigned int threshold;
	unsigned int voltage;
	unsigned char orient;
//	unsigned long irqflags;
	int reset_gpio;
	int int_gpio;
// 	platform data --


	/* debugfs variables */
	struct dentry *debug_dir;
	int current_debug_datap;

	struct mutex debug_mutex;
	u16 *debug_data;
	u16 num_nodes;		/* Number of sensor nodes */

	/* for read write block */
	u16 msg_proc_addr; 
	u16 last_read_addr;

	// Jui-Chuan add for early suspend and late resume ++
	struct early_suspend early_suspend;
	// Jui-Chuan add for early suspend and late resume --

	int suspend_state;

	int numtouch;

	struct wake_lock wakelock;

	int tp_firmware_upgrade_proceed;

	struct workqueue_struct *mxt_wq;
	struct delayed_work touch_chip_firmware_upgrade_work;
	struct delayed_work touch_chip_firmware_upgrade_work_if_fw_broken;
	struct delayed_work touch_chip_register_dock_notifier;

	int touch_fw_version;
	int touch_fw_build;	

	struct switch_dev touch_sdev;

	struct mutex touch_mutex;
};

static struct mxt_data *touch_chip; // Jui-Chuan add for global variant
static struct input_dev *input_dev;
static struct i2c_client *global_client;
/*
 * Function List
*/

static void mxt_input_report(struct mxt_data *data, int single_id);
static int mxt_make_highchg(struct mxt_data *data);
static int mxt_write_object(struct mxt_data *data,u8 type, u8 offset, u8 val);
static const struct file_operations delta_fops;
static const struct file_operations refs_fops;
static ssize_t switch_device_get_fw_ver(struct switch_dev *sdev, char *buf);
static ssize_t switch_device_state(struct switch_dev *sdev, char *buf);
void mxt_register_dock_notifier();
void judge_1664t_addr();
static int enable_touch();
static int disable_touch();
static int mxt_late_probe(struct i2c_client *client);
/* Functions called by EC */
int irq_is_enable;
int touch_is_enable;

/**
 * try_enable_touch
 * Use for preventing issues
 */
static int try_enable_touch(){
	int error = 0;
	printk("[Atmel] try_enable_touch ++\n");
	if(irq_is_enable==0){
		printk("Enable irq\n");
        	enable_irq(touch_chip->irq);
	        irq_is_enable = 1;
	}
        if(mxt1664t==1){	
		error = enable_touch();	
		if(error != 0){
			if(irq_is_enable==1){
				printk("Disable irq\n");
        		        disable_irq(touch_chip->irq);
		                irq_is_enable = 0;
			}
			printk("[Atmel] enable touch fail\n");
		}else{
			printk("[Atmel] enable touch success\n");
		}
	}
	printk("[Atmel] try_enable_touch --\n");
	return error;
}
/**
 * fine_tune_android
 * Collection of the registers needed to be changed in Android
 * 1. Edge reporting of 5mm copper metal finger
 * 2. Thumb jittering
 */
static int fine_tune_android(){
	int error = 0;
        if(mxt1664t==1){ 
//                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 30, 0x28);
//                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 32, 0x28);
//                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 47, 0x1E);
//                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 49, 0x0F);

		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 32, 0x20);
		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 53, 0x00);
		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 28, 0x11);

		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 47, 0x1E);
		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 49, 0x19);
		
		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 44, 0x46);
		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 45, 0xFF);

		error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 30, 0x2D);

		error |= mxt_write_object(touch_chip, T56_DEFINE, 0, 0x00);
		error |= mxt_write_object(touch_chip, MXT_GEN_ACQUIRE_T8, 0, 0x7E);
        }
        return error;
}

/** 
 * change_resolution
 * For Android, the resolution is 4095 * 4095.
 * For Windows, the resolution is 3095 * 3095.
 */
static int change_resolution(){
	int error = 0;
        if(mxt1664t==1){
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 13, 0xFF);
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 14, 0x0F);
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 24, 0xFF);
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, 25, 0x0F);
        }
        return error;
}

/**
 * enable_punch_rejection
 * Punch and palm suppression
 *
 */
static int enable_punch_rejection(){
	int error = 0;
	if(mxt1664t==1){
		error |= mxt_write_object(touch_chip, MXT_PROCI_TOUCHSUPPRESSION_T42, 0, 0x75);
		error |= mxt_write_object(touch_chip, MXT_PROCI_TOUCHSUPPRESSION_T42, 1, 0x06);
		error |= mxt_write_object(touch_chip, MXT_PROCI_TOUCHSUPPRESSION_T42, 2, 0x1E);
		error |= mxt_write_object(touch_chip, MXT_PROCI_TOUCHSUPPRESSION_T42, 3, 0x1E);
		error |= mxt_write_object(touch_chip, MXT_PROCI_TOUCHSUPPRESSION_T42, 4, 0x50);
                error |= mxt_write_object(touch_chip, MXT_PROCI_TOUCHSUPPRESSION_T42, 11, 0x03);
	}
	return error;
}

static int enable_touch(){
	int error;
	int retry = 3;
RETRY:
	error = 0;
	if(touch_is_enable==1) return 0;
	if(mxt1664t==1){
                printk("Mxt1664T Enable touch\n");
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x83);
		if(error==0){
			error |= change_resolution();
			if(build_version!=1) error |= enable_punch_rejection();
			if(build_version!=1) error |= fine_tune_android();
		}
        }else{
                printk("Mxt1664S Enable touch\n");
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x8F);
        }
	// No need backup and reset !!!

	if(error && retry){
		// Because enable touch occured after the reset, it's better to delay while failure happened.
		msleep(100);

		retry--;
		if(mxt1664t==1) judge_1664t_addr();
		goto RETRY;
	}
	if(error == 0) touch_is_enable = 1;	

	return error;
}

static int disable_touch(){
        int error;
        int retry = 3;
RETRY:
        error = 0;
	if(touch_is_enable==0) return 0;
        if(mxt1664t==1){
		// Because there is a HW reset when switching (include force switching)
		// So this step is not necessary for success
                printk("Mxt1664T Disable touch\n");
                mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x81);		
        }else{
                printk("Mxt1664S Disable touch\n");
                error |= mxt_write_object(touch_chip, MXT_TOUCH_MULTI_T9, MXT_TOUCH_CTRL, 0x8D);
        }

        if(error && retry){
                // Because enable touch occured after the reset, it's better to delay while failure happened.
                msleep(100);

                retry--;
                if(mxt1664t==1) judge_1664t_addr();
                goto RETRY;
        }
	if(error == 0) touch_is_enable = 0;
        return error;
}


void judge_1664t_addr(){
	if(mxt1664t==0) return;	
	int last_addr = touch_chip->client->addr;
	int error;

	touch_chip->client->addr = 0x4b;
	error = mxt_make_highchg(touch_chip);
	if(error == -EBUSY){
		printk("Mxt1664T Addr = 0x%x\n",touch_chip->client->addr);
	}
        else if(error){
	        touch_chip->client->addr = 0x4a;
	        error = mxt_make_highchg(touch_chip);
		if(error == -EBUSY){
			printk("Mxt1664T Addr = 0x%x\n",touch_chip->client->addr);
		}
		else if(error){
			printk("Mxt1664T Addr Unknown, Back to last Addr = 0x%x\n",last_addr);
			touch_chip->client->addr = last_addr;
		}else{
			printk("Mxt1664T Addr = 0x%x\n",touch_chip->client->addr);
		}
        }else{
        	printk("Mxt1664T Addr = 0x%x\n",touch_chip->client->addr);
        }

}

void judge_1664t_bootloader_addr(){
	if(mxt1664t==0) return;
	if(touch_chip->client->addr == 0x4b) touch_chip->client->addr = 0x27;
	else if (touch_chip->client->addr == 0x4a) touch_chip->client->addr = 0x26;
}

int enable_touch_1664t(){

	// This function will be called after switching HW reset.
	msleep(105);

	if(mxt1664t==0) return;
		
	judge_1664t_addr();		

	/* Enable irq */
	if(irq_is_enable==0){
		printk("Enable irq\n");
	        enable_irq(touch_chip->irq);
        	irq_is_enable = 1;
	}else printk("Touch irq is already enable\n");
	//mutex_lock(&input_dev->mutex);	

	/* Enable touch */        
	enable_touch();
}
EXPORT_SYMBOL(enable_touch_1664t);

static int touch_panel_irq_controller(struct notifier_block *this,unsigned long owner, void *data)
{
	printk("[Atmel][EC] touch_panel_irq_controller : ");

//	if(touch_chip==NULL) mxt_late_probe(global_client);	

	struct mxt_finger *finger = touch_chip->finger;
        int id;
	int error = 0;

	if(owner == SYSTEM_UPDATE){
		printk("EC fw is updating, skip\n");
		return NOTIFY_OK;
	}

	if(touch_chip==NULL){
		if(owner == SYSTEM_WINDOWS) return NOTIFY_OK;
		error = mxt_late_probe(global_client);
	}
        if(error) {
		printk("error\n");
		return NOTIFY_OK;
	}

	if(touch_chip->tp_firmware_upgrade_proceed){
		printk("touch fw is updating, skip\n");
		return NOTIFY_OK;
	}

	mutex_lock(&touch_chip->touch_mutex);
#ifdef CONFIG_HID_ASUS_PAD_EC
        if(owner == SYSTEM_ANDROID || owner == SYSTEM_NONE){
		system_owner = SYSTEM_ANDROID;	

		if(touch_chip->suspend_state == 1){
			printk("[Atmel] Android driver is suspended, skip for resuming funciton\n");
			goto OUT;
		}
		if(mxt1664t==0){
			if(irq_is_enable == 0){
				msleep(105);
				/* Enable irq */
				printk("Enable irq\n");
                                enable_irq(touch_chip->irq);
                                irq_is_enable = 1;

				mxt_make_highchg(touch_chip);

			}else printk("Touch irq is already enable\n");
			/* Enable touch */
                        enable_touch();
		}else{
			enable_touch_1664t();
		}
	
	}
        if(owner == SYSTEM_WINDOWS){
		system_owner = SYSTEM_WINDOWS;
		if(mxt1664t==0){
			if(irq_is_enable == 1){
				/* Disable touch */ 
				disable_touch();

				/* Disable irq */
				printk("Disable irq\n");
                                disable_irq(touch_chip->irq);
                                irq_is_enable = 0;
			
				for(id=0;id<10;id++){
                	                if (finger[id].status & (MXT_PRESS | MXT_MOVE)) {
        	                                printk("mxt_start [%d] released\n", id);
	                                        finger[id].status = MXT_RELEASE;
                                        	mxt_input_report(touch_chip, id);
                                	}
                        	}
			}else printk("Touch irq is already disable\n");
		}else{
                        //mutex_lock(&input_dev->mutex);
                        /* Disable touch */
			disable_touch();
			/* Disable irq */			
			if(irq_is_enable == 1){
                                printk("Disable irq\n");
                                disable_irq(touch_chip->irq);
                                irq_is_enable = 0;
                        }else printk("Touch irq is already disable\n");

			for(id=0;id<10;id++){
                                if (finger[id].status & (MXT_PRESS2 | MXT_MOVE2)) {
                                        printk("mxt_start [%d] released\n", id);
                                        finger[id].status = MXT_RELEASE;
                                        mxt_input_report(touch_chip, id);
                                }
                        }
		}
	}
	
#endif

OUT:
	mutex_unlock(&touch_chip->touch_mutex);
	return NOTIFY_OK;
}

/**************************/
#define MXT_ADR_T9_NUMTOUCH 0x0e
#define MXT_ADR_T100_NUMTOUCH 0x06
static int mxt_write_block(struct i2c_client *client,
                    u16 addr,
                    u16 length,
                    u8 *value);
void mxt_config_init(struct mxt_data *mxt){

	u8 T5[] = {1, 128, 77, 213, 212, 0, 0, 0,0};
        u8 T6[] = {0, 0, 0, 0, 0, 0};
        u8 T38[] = {1,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,
                    0,0,0,0,0,0,0,0,0,0};
        u8 T7[] = {50, 8, 220, 87};
        u8 T8[] = {126, 0, 10, 10, 0, 0, 255, 1, 0, 0};
        u8 T9[] = {141, 0, 0, 30, 52, 0, 94, 50, 2, 1, 10, 15,8, 48, 5, 10, 15, 20, 255, 15, 255, 15, 3, 8, 12,
                   11, 207, 35, 138, 20, 18, 23, 41, 44, 0, 0, 68 ,240 ,40 , 0, 0, 0, 0, 0, 0, 0, 0};
        u8 T15[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
        u8 T18[] = {0, 0};
        u8 T24[] = {0 ,0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0};
        u8 T25[] = {3, 0, 84, 111, 204,
                    91, 84, 111, 204, 91,
                    0, 0, 0, 0, 200,
                    184, 11, 184, 11, 0, 0};
        u8 T27[] = {0, 0, 0, 0, 0, 0, 0};
        u8 T40[] = {0, 0, 0, 0, 0};
        u8 T42[] = {49, 42, 50, 50, 127,
                    0, 0, 0, 5, 5, 0, 0, 0};
        u8 T43[] = {173, 10, 146, 0, 1,
                    1, 10, 0, 0, 0, 0, 40};
        u8 T46[] = {0, 0, 16, 16, 0,
                    0, 1, 0, 0, 15, 16};
        u8 T47[] = {0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0, 0, 0, 0,
                    0, 0};

	u8 i= 0, max_objs= 0;
	u16 addr;
	struct mxt_object *obj_index;

	printk("[Atmel] mxt_config_init start\n");
	dev_dbg(&mxt->client->dev, "In function %s", __func__);
//	max_objs = mxt->device_info.num_objs;
	max_objs = mxt->info.object_num;
	obj_index = mxt->object_table;
	
	int instance = 0;

	for (i=0; i<max_objs; i++) {
//		addr = obj_index->chip_addr;
		addr = obj_index->start_address + (obj_index->size + 1) * instance;
		switch(obj_index->type) {
		case MXT_GEN_MESSAGE_T5:
			printk("[Atmel] mxt_config_init T5\n");
			mxt_write_block(mxt->client, addr,
                                        sizeof(T5), T5);
                        break;		
		case MXT_GEN_COMMAND_T6:
			printk("[Atmel] mxt_config_init T6\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T6), T6);
                        break;
		case MXT_GEN_POWER_T7:
			printk("[Atmel] mxt_config_init T7\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T7), T7);
			break;
		case MXT_GEN_ACQUIRE_T8:
			printk("[Atmel] mxt_config_init T8\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T8), T8);
			break;
		case MXT_TOUCH_MULTI_T9_DEFINE:
			printk("[Atmel] mxt_config_init T9\n");
			if (T9[MXT_ADR_T9_NUMTOUCH] != mxt->numtouch)
				T9[MXT_ADR_T9_NUMTOUCH] = mxt->numtouch;
			mxt_write_block(mxt->client, addr,
					sizeof(T9), T9);
			dev_dbg(&mxt->client->dev, "init multitouch object");
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			printk("[Atmel] mxt_config_init T15\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T15), T15);
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			printk("[Atmel] mxt_config_init T18\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T18), T18);
			break;
	//	case MXT_SPT_GPIOPWM_T19:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T19), T19);
	//		break;
	//	case MXT_PROCI_GRIPFACE_T20:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T20), T20);
	//		break;
	//	case MXT_PROCG_NOISE_T22:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T22), T22);
	//		break;
	//	case MXT_TOUCH_PROXIMITY_T23:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T23), T23);
	//		break;
		case MXT_PROCI_ONETOUCH_T24:
			printk("[Atmel] mxt_config_init T24\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T24), T24);
			break;
		case MXT_SPT_SELFTEST_T25:
			printk("[Atmel] mxt_config_init T25\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T25), T25);
			break;
		case MXT_PROCI_TWOTOUCH_T27:
			printk("[Atmel] mxt_config_init T27\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T27), T27);
                        break;
	//	case MXT_SPT_CTECONFIG_T28:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T28), T28);
	//		break;
	//	case MXT_DEBUG_DIAGNOSTIC_T37:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T37), T37);
	//		break;
		case MXT_SPT_USERDATA_T38:
			printk("[Atmel] mxt_config_init T38\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T38), T38);
			break;
		case MXT_PROCI_GRIP_T40:
			printk("[Atmel] mxt_config_init T40\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T40), T40);
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			printk("[Atmel] mxt_config_init T42\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T42), T42);
			break;
		case MXT_SPT_DIGITIZER_T43:
			printk("[Atmel] mxt_config_init T43\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T43), T43);
                        break;
		case MXT_SPT_CTECONFIG_T46:
			printk("[Atmel] mxt_config_init T46\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T46), T46);
			break;
		case MXT_PROCI_STYLUS_T47:
			printk("[Atmel] mxt_config_init T47\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T47), T47);
			break;

#if 0
		case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
			mxt_write_block(mxt->client, addr,
					sizeof(T55), T55);
			break;
		case MXT_PROCI_SHIELDLESS_T56:
			mxt_write_block(mxt->client, addr,
					sizeof(T56), T56);
			break;
		case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
			mxt_write_block(mxt->client, addr,
					sizeof(T57), T57);
			break;
		case MXT_SPT_TIMER_T61:
			mxt_write_block(mxt->client, addr,
					sizeof(T61), T61);
			break;
		case MXT_PROCG_NOISESUPPRESSION_T62:
			mxt_write_block(mxt->client, addr,
					sizeof(T62), T62);
			break;			
#endif
		default:
			break;
		}
		obj_index++;
	}
	dev_dbg(&mxt->client->dev, "config init Done.");

}

void mxt_config_init_1664t(struct mxt_data *mxt){

	
u8 T5[] = {255, 8, 171, 15, 209, 0, 0, 0, 0, 0};
u8 T6[] = {0, 0, 0, 0, 0, 0};
//130911d_Asus_TX201_V10AC_SetOK.raw
u8 T68[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T38[] = { 0x00, 0x00, 0x00, 0x84, 0x0B, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T71[] = { 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x05, 0x01, 0x00, 0x1E, 0x07, 0x0C, 0x01, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T7[] = { 0x06, 0x05, 0xDC, 0x57 };
u8 T8[] = { 0x5A, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0x00, 0x00, 0x00 };
u8 T15[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T18[] = { 0x00, 0x00 };
u8 T19[] = { 0x03, 0x00, 0x00, 0x03, 0x00, 0x00 };
u8 T25[] = { 0x03, 0x00, 0x60, 0x6D, 0xF0, 0x55, 0x60, 0x6D, 0xF0, 0x55, 0x60, 0x6D, 0xF0, 0x55, 0xC8, 0xA0, 0x0F, 0xA0, 0x0F, 0xA0, 0x0F };
u8 T40[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T42[] = { 0x00, 0x01, 0x28, 0x28, 0x46, 0x00, 0x00, 0x1E, 0x05, 0x05, 0x00, 0x00, 0x00 };
u8 T43[] = { 0x8D, 0x00, 0x90, 0x00, 0x00, 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x00, 0x00 };
u8 T46[] = { 0x04, 0x00, 0x10, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x10 };
u8 T47[] = { 0x00, 0x00, 0xFF, 0x14, 0x02, 0x1B, 0x00, 0x82, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T56[] = { 0x03, 0x00, 0x01, 0x27, 0x08, 0x09, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T61[] = { 0x01, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T65[] = { 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00 };
u8 T66[] = { 0x00, 0x00, 0x00 };
u8 T70[] = { 0x01, 0x02, 0x00, 0x08, 0x00, 0x00, 0x06, 0x00, 0x00, 0x04, 0x01, 0x02, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x01, 0x01, 0x0D, 0x00, 0x08, 0x00, 0x00, 0x06, 0x05, 0x00, 0x04, 0x01, 0x15, 0x00, 0x64, 0x00, 0x00, 0x27, 0x28, 0x00, 0x04, 0x01, 0x17, 0x00, 0x64, 0x00, 0x00, 0x27, 0x2D, 0x00, 0x04, 0x01, 0x15, 0x00, 0x3D, 0x00, 0x00, 0x00, 0x0A, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T72[] = { 0xFD, 0x00, 0x00, 0x01, 0x00, 0x05, 0x05, 0x02, 0x02, 0x00, 0x0F, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x07, 0x00, 0x41, 0x50, 0x16, 0x2A, 0x10, 0x10, 0x10, 0x10, 0x10, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x12, 0x55, 0x07, 0x0F, 0xB4, 0x00, 0x41, 0x2A, 0x24, 0x24, 0x24, 0x24, 0x24, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x02, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T77[] = { 0x09, 0x01, 0x08, 0x1E };
u8 T79[] = { 0x00, 0x00, 0x00 };
u8 T100[] = { 0x81, 0x21, 0x00, 0x07, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x1E, 0x32, 0xFC, 0xFC, 0x17, 0x0C, 0x06, 0x08, 0x00, 0x00, 0x00, 0x34, 0x32, 0xFA, 0xFA, 0x17, 0x0C, 0x06, 0x08, 0x10, 0xFF, 0x32, 0x0B, 0x32, 0x00, 0x00, 0x14, 0x00, 0x14, 0x00, 0x02, 0x05, 0x01, 0x00, 0x1E, 0x44, 0xDC, 0x28, 0x0F, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x08 };

/*// 130905b_Asus_TX201_V10AC_FinalConfirmSet.raw 
u8 T68[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T38[] = { 0x00, 0x00, 0x00, 0x84, 0x0B, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T71[] = { 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x05, 0x01, 0x00, 0x1E, 0x07, 0x09, 0x01, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T7[] = { 0x28, 0x06, 0xDC, 0x57 };
u8 T8[] = { 0x5A, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0x00, 0x00, 0x00 };
u8 T15[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T18[] = { 0x00, 0x00 };
u8 T19[] = { 0x03, 0x00, 0x00, 0x03, 0x00, 0x00 };
u8 T25[] = { 0x03, 0x00, 0x60, 0x6D, 0xD8, 0x59, 0x60, 0x6D, 0xD8, 0x59, 0x60, 0x6D, 0xD8, 0x59, 0xC8, 0xB8, 0x0B, 0xB8, 0x0B, 0x00, 0x00 };
u8 T40[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T42[] = { 0x00, 0x01, 0x28, 0x28, 0x46, 0x00, 0x00, 0x1E, 0x05, 0x05, 0x00, 0x00, 0x00 };
u8 T43[] = { 0x8D, 0x00, 0x90, 0x00, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x00, 0x00 };
u8 T46[] = { 0x04, 0x00, 0x10, 0x10, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x10 };
u8 T47[] = { 0x00, 0x00, 0xFF, 0x14, 0x02, 0x1B, 0x00, 0x82, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T56[] = { 0x03, 0x00, 0x01, 0x27, 0x08, 0x09, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T61[] = { 0x03, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T65[] = { 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00 };
u8 T66[] = { 0x00, 0x00, 0x00 };
u8 T70[] = { 0x03, 0x02, 0x00, 0x08, 0x00, 0x00, 0x06, 0x00, 0x00, 0x04, 0x03, 0x02, 0x00, 0x3D, 0x00, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x03, 0x0D, 0x00, 0x08, 0x00, 0x00, 0x06, 0x05, 0x00, 0x04, 0x03, 0x15, 0x00, 0x64, 0x00, 0x00, 0x27, 0x28, 0x00, 0x04, 0x03, 0x17, 0x00, 0x64, 0x00, 0x00, 0x27, 0x2D, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T72[] = { 0xFD, 0x00, 0x00, 0x01, 0x00, 0x05, 0x05, 0x02, 0x02, 0x00, 0x0F, 0x01, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x07, 0x0F, 0x41, 0x50, 0x16, 0x2A, 0x10, 0x10, 0x10, 0x10, 0x10, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x12, 0x55, 0x07, 0x0F, 0xA0, 0x00, 0x41, 0x2A, 0x24, 0x24, 0x24, 0x24, 0x24, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x02, 0x00, 0xAA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T77[] = { 0x04, 0x01, 0x0F, 0x00 };
u8 T79[] = { 0x00, 0x00, 0x00 };
u8 T100[] = { 0x81, 0x21, 0x00, 0x07, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x1E, 0x32, 0xFC, 0xFC, 0xFF, 0x0F, 0x06, 0x08, 0x00, 0x00, 0x00, 0x34, 0x32, 0xFA, 0xFA, 0xFF, 0x0F, 0x06, 0x08, 0x10, 0xFF, 0x32, 0x0B, 0x32, 0x00, 0x00, 0x14, 0x00, 0x14, 0x00, 0x02, 0x05, 0x01, 0x00, 0x1E, 0x44, 0xDC, 0x28, 0x0F, 0x00, 0x0A, 0x00, 0x00, 0x00, 0x08 };
*/

/* // 130816c
u8 T68[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T38[] = { 0x00, 0x00, 0x00, 0x84, 0x0B, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T71[] = { 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T7[] = { 0x1E, 0x06, 0x32, 0x43 };
u8 T8[] = { 0x5A, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0x00, 0x00, 0x00 };
u8 T15[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T18[] = { 0x00, 0x00 };
u8 T19[] = { 0x03, 0x00, 0x00, 0x03, 0x00, 0x00 };
u8 T25[] = { 0x03, 0x00, 0x60, 0x6D, 0xD8, 0x59, 0x60, 0x6D, 0xD8, 0x59, 0x60, 0x6D, 0xD8, 0x59, 0xC8, 0xB8, 0x0B, 0xB8, 0x0B, 0x00, 0x00 };
u8 T40[] = { 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T42[] = { 0x00, 0x01, 0x28, 0x28, 0x46, 0x00, 0x00, 0x1E, 0x05, 0x05, 0x00, 0x00, 0x00 };
u8 T43[] = { 0x8D, 0x0A, 0x90, 0x00, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x6C, 0x00, 0x00 };
u8 T46[] = { 0x04, 0x00, 0x14, 0x14, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x10 };
u8 T47[] = { 0x00, 0x00, 0xFF, 0x14, 0x02, 0x1B, 0x00, 0x82, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T56[] = { 0x01, 0x00, 0x01, 0x28, 0x08, 0x09, 0x09, 0x09, 0x09, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x09, 0x09, 0x09, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T61[] = { 0x01, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T65[] = { 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x04, 0x00, 0x00 };
u8 T66[] = { 0x00, 0x00, 0x00 };
u8 T70[] = { 0x03, 0x02, 0x00, 0x08, 0x00, 0x00, 0x06, 0x00, 0x00, 0x04, 0x03, 0x02, 0x00, 0x3D, 0x00, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x03, 0x0D, 0x00, 0x08, 0x00, 0x00, 0x06, 0x05, 0x00, 0x04, 0x03, 0x15, 0x00, 0x48, 0x00, 0x00, 0x0A, 0x1E, 0x00, 0x00, 0x03, 0x17, 0x00, 0x48, 0x00, 0x00, 0x0A, 0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T72[] = { 0xFD, 0x00, 0x00, 0x01, 0x00, 0x02, 0x05, 0x02, 0x02, 0x00, 0x0F, 0x05, 0x08, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x07, 0x0F, 0x41, 0x50, 0x16, 0x2A, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x12, 0x55, 0x47, 0x0F, 0xA0, 0x00, 0x41, 0x2A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
u8 T77[] = { 0x04, 0x01, 0x0F, 0x00 };
u8 T79[] = { 0x00, 0x00, 0x00 };
u8 T100[] = { 0x8D, 0x23, 0x00, 0x07, 0x00, 0x00, 0x0A, 0x88, 0x00, 0x1E, 0x32, 0x0E, 0x0C, 0xFF, 0x0F, 0x1C, 0x14, 0x00, 0x00, 0x00, 0x34, 0x32, 0x08, 0x08, 0xFF, 0x0F, 0x20, 0x14, 0x10, 0xFF, 0x28, 0x0F, 0x28, 0x00, 0x00, 0x19, 0x00, 0x0F, 0x00, 0x02, 0x02, 0x01, 0x00, 0x18, 0x44, 0xDC, 0x28, 0x0F, 0x00, 0x08, 0x00, 0x00, 0x00, 0x0D };
*/


/*
u8 T68[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T38[] = { 0x00, 0x00, 0x00, 0x84, 0x0B, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T71[] = { 0x0A, 0x0F, 0x1E, 0xC4, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T7[] = { 0x0F, 0xFF, 0x32, 0x43};
u8 T8[] = { 0x5A, 0x00, 0x1E, 0x1E, 0x00, 0x00, 0xFF, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T15[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T18[] = { 0x00, 0x00};
u8 T19[] = { 0x03, 0x00, 0x00, 0x03, 0x00, 0x00};
u8 T25[] = { 0x03, 0x00, 0x6C, 0x6B, 0xE4, 0x57, 0x6C, 0x6B, 0xE4, 0x57, 0x6C, 0x6B, 0xE4, 0x57, 0xC8, 0xB8, 0x0B, 0xB8, 0x0B, 0x00, 0x00};
u8 T40[] = { 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T42[] = { 0x00, 0x01, 0x28, 0x28, 0x46, 0x00, 0x00, 0x1E, 0x05, 0x05, 0x00, 0x00, 0x00};
u8 T43[] = { 0x8D, 0x00, 0x90, 0x00, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x2C, 0x00, 0x00};
u8 T46[] = { 0x04, 0x00, 0x14, 0x14, 0x00, 0x00, 0x03, 0x00, 0x00, 0x03, 0x10};
u8 T47[] = { 0x00, 0x00, 0xFF, 0x14, 0x02, 0x1B, 0x00, 0x82, 0x00, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T56[] = { 0x03, 0x00, 0x01, 0x28, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x08, 0x07, 0x08, 0x08, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x09, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T61[] = { 0x03, 0x00, 0x00, 0x60, 0xEA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T65[] = { 0x01, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x02, 0x00, 0x00};
u8 T66[] = { 0x00, 0x00, 0x00};
u8 T70[] = { 0x03, 0x02, 0x00, 0x08, 0x00, 0x00, 0x06, 0x00, 0x00, 0x04, 0x03, 0x02, 0x00, 0x3D, 0x00, 0x00, 0x01, 0x0A, 0x00, 0x00, 0x03, 0x0D, 0x00, 0x08, 0x00, 0x00, 0x06, 0x05, 0x00, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T72[] = { 0xFD, 0x00, 0x00, 0x01, 0x00, 0x05, 0x05, 0x02, 0x02, 0x00, 0x0F, 0x05, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x07, 0x0F, 0x41, 0x50, 0x16, 0x2A, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x14, 0x00, 0x00, 0x12, 0x55, 0x47, 0x0F, 0xA0, 0x00, 0x41, 0x2A, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
u8 T77[] = { 0x04, 0x01, 0x0F, 0x00};
u8 T79[] = { 0x00, 0x00, 0x00};
u8 T100[] = { 0x81, 0x21, 0x00, 0x07, 0x00, 0x00, 0x0A, 0x00, 0x00, 0x1E, 0x32, 0x00, 0x00, 0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x34, 0x32, 0x00, 0x00, 0xFF, 0x0F, 0x00, 0x00, 0x10, 0xFF, 0x32, 0x0B, 0x32, 0x00, 0x00, 0x0A, 0x00, 0x0A, 0x00, 0x02, 0x02, 0x00, 0x00, 0x0A, 0x44, 0xDC, 0x28, 0x0F, 0x00, 0x08, 0x00, 0x00, 0x00, 0x08};
*/
	u8 i= 0, max_objs= 0;
	u16 addr;
	struct mxt_object *obj_index;

	printk("[Atmel] mxt_config_init start\n");
	dev_dbg(&mxt->client->dev, "In function %s", __func__);
//	max_objs = mxt->device_info.num_objs;
	max_objs = mxt->info.object_num;
	obj_index = mxt->object_table;
	
	int instance = 0;

	for (i=0; i<max_objs; i++) {
//		addr = obj_index->chip_addr;
		addr = obj_index->start_address + (obj_index->size + 1) * instance;
		switch(obj_index->type) {
		case MXT_GEN_MESSAGE_T5:
			printk("[Atmel] mxt_config_init T5\n");
			mxt_write_block(mxt->client, addr,
                                        sizeof(T5), T5);
                        break;		
		case MXT_GEN_COMMAND_T6:
			printk("[Atmel] mxt_config_init T6\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T6), T6);
                        break;
		case MXT_GEN_POWER_T7:
			printk("[Atmel] mxt_config_init T7\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T7), T7);
			break;
		case MXT_GEN_ACQUIRE_T8:
			printk("[Atmel] mxt_config_init T8\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T8), T8);
			break;
		case MXT_TOUCH_MULTI_T100_DEFINE:
			printk("[Atmel] mxt_config_init T100\n");
			if (T100[MXT_ADR_T100_NUMTOUCH] != mxt->numtouch)
				T100[MXT_ADR_T100_NUMTOUCH] = mxt->numtouch;
			mxt_write_block(mxt->client, addr,
					sizeof(T100), T100);
			dev_dbg(&mxt->client->dev, "init multitouch object");
			break;
		case MXT_TOUCH_KEYARRAY_T15:
			printk("[Atmel] mxt_config_init T15\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T15), T15);
			break;
		case MXT_SPT_COMMSCONFIG_T18:
			printk("[Atmel] mxt_config_init T18\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T18), T18);
			break;
		case MXT_SPT_GPIOPWM_T19:
			printk("[Atmel] mxt_config_init T19\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T19), T19);
			break;
	//	case MXT_PROCI_GRIPFACE_T20:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T20), T20);
	//		break;
	//	case MXT_PROCG_NOISE_T22:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T22), T22);
	//		break;
	//	case MXT_TOUCH_PROXIMITY_T23:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T23), T23);
	//		break;
	//	case MXT_PROCI_ONETOUCH_T24:
	//		printk("[Atmel] mxt_config_init T24\n");
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T24), T24);
	//		break;
		case MXT_SPT_SELFTEST_T25:
			printk("[Atmel] mxt_config_init T25\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T25), T25);
			break;
	//	case MXT_PROCI_TWOTOUCH_T27:
	//		printk("[Atmel] mxt_config_init T27\n");
        //                mxt_write_block(mxt->client, addr,
        //                                sizeof(T27), T27);
        //                break;
	//	case MXT_SPT_CTECONFIG_T28:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T28), T28);
	//		break;
	//	case MXT_DEBUG_DIAGNOSTIC_T37:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T37), T37);
	//		break;
		case MXT_SPT_USERDATA_T38:
			printk("[Atmel] mxt_config_init T38\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T38), T38);
			break;
		case MXT_PROCI_GRIP_T40:
			printk("[Atmel] mxt_config_init T40\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T40), T40);
			break;
		case MXT_PROCI_TOUCHSUPPRESSION_T42:
			printk("[Atmel] mxt_config_init T42\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T42), T42);
			break;
		case MXT_SPT_DIGITIZER_T43:
			printk("[Atmel] mxt_config_init T43\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T43), T43);
                        break;
		case MXT_SPT_CTECONFIG_T46:
			printk("[Atmel] mxt_config_init T46\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T46), T46);
			break;
		case MXT_PROCI_STYLUS_T47:
			printk("[Atmel] mxt_config_init T47\n");
			mxt_write_block(mxt->client, addr,
					sizeof(T47), T47);
			break;
	//	case MXT_PROCI_ADAPTIVETHRESHOLD_T55:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T55), T55);
	//		break;
	//	case MXT_PROCI_SHIELDLESS_T56:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T56), T56);
	//		break;
	//	case MXT_PROCI_EXTRATOUCHSCREENDATA_T57:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T57), T57);
	//		break;
	//	case T61_DEFINE:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T61), T61);
	//		break;
	//	case MXT_PROCG_NOISESUPPRESSION_T62:
	//		mxt_write_block(mxt->client, addr,
	//				sizeof(T62), T62);
	//		break;			

                case T56_DEFINE:
			printk("[Atmel] mxt_config_init T56\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T56), T56);
                        break;
		case T61_DEFINE:
                        printk("[Atmel] mxt_config_init T61\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T61), T61);
                        break;
		case T65_DEFINE:
			printk("[Atmel] mxt_config_init T65\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T65), T65);
                        break;
		case T66_DEFINE:
                        printk("[Atmel] mxt_config_init T66\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T66), T66);
                        break;
		case T70_DEFINE:
                        printk("[Atmel] mxt_config_init T70\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T70), T70);
                        break;
		case T68_DEFINE:
                        printk("[Atmel] mxt_config_init T68\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T68), T68);
                        break;
		case T71_DEFINE:
                        printk("[Atmel] mxt_config_init T71\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T71), T71);
                        break;
		case T72_DEFINE:
			printk("[Atmel] mxt_config_init T72\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T72), T72);
                        break;
		case T77_DEFINE:
			printk("[Atmel] mxt_config_init T77\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T77), T77);
                        break;
		case T79_DEFINE:
			printk("[Atmel] mxt_config_init T79\n");
                        mxt_write_block(mxt->client, addr,
                                        sizeof(T79), T79);
                        break;

		default:
			break;
		}
		obj_index++;
	}
	dev_dbg(&mxt->client->dev, "config init Done.");

}


// Jui-Chuan add for early suspend and late resume ++
#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *h);
static void mxt_late_resume(struct early_suspend *h);
#endif
// Jui-Chuan add for early suspend and late resume --

// Jui-Chuan add for MXT_BASE_ADDR ++
static u16 get_object_address(uint8_t object_type,
                              uint8_t instance,
                              struct mxt_object *object_table,
                              int max_objs)
{
        uint8_t object_table_index = 0;
        uint8_t address_found = 0;
        uint16_t address = 0;
        struct mxt_object *obj;

        while ((object_table_index < max_objs) && !address_found) {
                obj = &object_table[object_table_index];
                if (obj->type == object_type) {
                        address_found = 1;
                        /* Are there enough instances defined in the FW? */
                        if (obj->instances >= instance) {
				// change chip_addr to start_address
                                address = obj->start_address +
                                          (obj->size + 1) * instance;
                        } else {
                                return 0;
                        }
                }
                object_table_index++;
        }
        return address;
}

// Change "device_info.num_objs" to "info.object_num" 
#define MXT_BASE_ADDR(object_type, mxt)                                 \
        get_object_address(object_type, 0, mxt->object_table,           \
                           mxt->info.object_num) 


// Jui-Chuan add for MXT_BASE_ADDR --

static bool mxt_object_readable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_MESSAGE_T5:
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_GEN_DATASOURCE_T53:
	case MXT_TOUCH_MULTI_T9_DEFINE:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_USERDATA_T38:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_TOUCH_MULTI_T100_DEFINE:
	case T56_DEFINE:
	case T61_DEFINE:
	case T65_DEFINE:
	case T66_DEFINE:
	case T68_DEFINE:
	case T70_DEFINE:
	case T71_DEFINE:
	case T72_DEFINE:
	case T77_DEFINE:
	case T79_DEFINE:
	
		return true;
	default:
		return false;
	}
}

static bool mxt_object_writable(unsigned int type)
{
	switch (type) {
	case MXT_GEN_COMMAND_T6:
	case MXT_GEN_POWER_T7:
	case MXT_GEN_ACQUIRE_T8:
	case MXT_TOUCH_MULTI_T9_DEFINE:
	case MXT_TOUCH_KEYARRAY_T15:
	case MXT_TOUCH_PROXIMITY_T23:
	case MXT_TOUCH_PROXKEY_T52:
	case MXT_PROCI_GRIPFACE_T20:
	case MXT_PROCG_NOISE_T22:
	case MXT_PROCI_ONETOUCH_T24:
	case MXT_PROCI_TWOTOUCH_T27:
	case MXT_PROCI_GRIP_T40:
	case MXT_PROCI_PALM_T41:
	case MXT_PROCI_TOUCHSUPPRESSION_T42:
	case MXT_PROCI_STYLUS_T47:
	case MXT_PROCG_NOISESUPPRESSION_T48:
	case MXT_SPT_COMMSCONFIG_T18:
	case MXT_SPT_GPIOPWM_T19:
	case MXT_SPT_SELFTEST_T25:
	case MXT_SPT_CTECONFIG_T28:
	case MXT_SPT_DIGITIZER_T43:
	case MXT_SPT_CTECONFIG_T46:
	case MXT_TOUCH_MULTI_T100_DEFINE:
        case T56_DEFINE:
	case T61_DEFINE:
        case T65_DEFINE:
	case T66_DEFINE:
	case T68_DEFINE:
        case T70_DEFINE:
	case T71_DEFINE:
        case T72_DEFINE:
        case T77_DEFINE:
        case T79_DEFINE:

		return true;
	default:
		return false;
	}
}

static void mxt_dump_message(struct device *dev,
				  struct mxt_message *message)
{

/*	dev_dbg(dev, "reportid:\t0x%x\n", message->reportid);
	dev_dbg(dev, "message1:\t0x%x\n", message->message[0]);
	dev_dbg(dev, "message2:\t0x%x\n", message->message[1]);
	dev_dbg(dev, "message3:\t0x%x\n", message->message[2]);
	dev_dbg(dev, "message4:\t0x%x\n", message->message[3]);
	dev_dbg(dev, "message5:\t0x%x\n", message->message[4]);
	dev_dbg(dev, "message6:\t0x%x\n", message->message[5]);
	dev_dbg(dev, "message7:\t0x%x\n", message->message[6]);
	dev_dbg(dev, "checksum:\t0x%x\n", message->checksum);
*/
/*	printk("reportid:\t0x%x\n", message->reportid);
	printk("message1:\t0x%x\n", message->message[0]);
	printk("message2:\t0x%x\n", message->message[1]);
	printk("message3:\t0x%x\n", message->message[2]);
	printk("message4:\t0x%x\n", message->message[3]);
	printk("message5:\t0x%x\n", message->message[4]);
	printk("message6:\t0x%x\n", message->message[5]);
	printk("message7:\t0x%x\n", message->message[6]);
	printk("checksum:\t0x%x\n", message->checksum);
*/
}

static int mxt_check_bootloader(struct i2c_client *client,
				     unsigned int state)
{
	u8 val;

recheck:
	if (i2c_master_recv(client, &val, 1) != 1) {
		dev_err(&client->dev, "%s: i2c recv failed\n", __func__);
		return -EIO;
	}

	switch (state) {
	case MXT_WAITING_BOOTLOAD_CMD:
	case MXT_WAITING_FRAME_DATA:
		val &= ~MXT_BOOT_STATUS_MASK;
		break;
	case MXT_FRAME_CRC_PASS:
		if (val == MXT_FRAME_CRC_CHECK)
			goto recheck;
		break;
	default:
		return -EINVAL;
	}

	if (val != state) {
		dev_err(&client->dev, "Unvalid bootloader mode state\n");
		return -EINVAL;
	}

	return 0;
}

static int mxt_unlock_bootloader(struct i2c_client *client)
{
	u8 buf[2];

	buf[0] = MXT_UNLOCK_CMD_LSB;
	buf[1] = MXT_UNLOCK_CMD_MSB;

	if (i2c_master_send(client, buf, 2) != 2) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_fw_write(struct i2c_client *client,
			     const u8 *data, unsigned int frame_size)
{
	if (i2c_master_send(client, data, frame_size) != frame_size) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int __mxt_read_reg(struct i2c_client *client,
			       u16 reg, u16 len, void *val)
{
	struct i2c_msg xfer[2];
	u8 buf[2];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	/* Write register */
	xfer[0].addr = client->addr;
	xfer[0].flags = 0;
	xfer[0].len = 2;
	xfer[0].buf = buf;

	/* Read data */
	xfer[1].addr = client->addr;
	xfer[1].flags = I2C_M_RD;
	xfer[1].len = len;
	xfer[1].buf = val;

	if (i2c_transfer(client->adapter, xfer, 2) != 2) {
		dev_err(&client->dev, "%s: i2c transfer failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	return __mxt_read_reg(client, reg, 1, val);
}

static int mxt_write_reg(struct i2c_client *client, u16 reg, u8 val)
{
	u8 buf[3];

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = val;

	if (i2c_master_send(client, buf, 3) != 3) {
		dev_err(&client->dev, "%s: i2c send failed\n", __func__);
		return -EIO;
	}

	return 0;
}

static int mxt_read_object_table(struct i2c_client *client,
				      u16 reg, u8 *object_buf)
{
	return __mxt_read_reg(client, reg, MXT_OBJECT_SIZE,
				   object_buf);
}

static struct mxt_object *
mxt_get_object(struct mxt_data *data, u8 type)
{
	struct mxt_object *object;
	int i;
//	printk("[Atmel] data->info.object_num = %d\n",data->info.object_num);
//	printk("[Atmel] type = %d\n",type);
	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;
//		printk("[Atmel] compare object->type %d with input %d\n", object->type, type);
		if (object->type == type)
			return object;
	}

	dev_err(&data->client->dev, "Invalid object type\n");
	return NULL;
}

static int mxt_read_message(struct mxt_data *data,
				 struct mxt_message *message)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, MXT_GEN_MESSAGE_T5);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg,
			sizeof(struct mxt_message), message);
}

static int mxt_read_object(struct mxt_data *data,
				u8 type, u8 offset, u8 *val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return __mxt_read_reg(data->client, reg + offset, 1, val);
}

static int mxt_write_object(struct mxt_data *data,
				 u8 type, u8 offset, u8 val)
{
	struct mxt_object *object;
	u16 reg;

	object = mxt_get_object(data, type);
	if (!object)
		return -EINVAL;

	reg = object->start_address;
	return mxt_write_reg(data->client, reg + offset, val);
}

static void mxt_input_report(struct mxt_data *data, int single_id)
{
	struct mxt_finger *finger = data->finger;
	struct input_dev *input_dev = data->input_dev;
	int status = finger[single_id].status;
	int finger_num = 0;
	int id;

	for (id = 0; id < MXT_MAX_FINGER; id++) {
		if (!finger[id].status)
			continue;

		input_mt_slot(input_dev, id);
		input_mt_report_slot_state(input_dev, MT_TOOL_FINGER,
				finger[id].status != MXT_RELEASE);

		if (finger[id].status != MXT_RELEASE) {
			finger_num++;
			input_report_abs(input_dev, ABS_MT_TRACKING_ID, id); 
			input_report_abs(input_dev, ABS_MT_TOUCH_MAJOR,
					finger[id].area);
			input_report_abs(input_dev, ABS_MT_POSITION_X,
					finger[id].x);
			input_report_abs(input_dev, ABS_MT_POSITION_Y,
					finger[id].y);
			input_report_abs(input_dev, ABS_MT_PRESSURE,
					finger[id].pressure);
		} else {
			finger[id].status = 0;
		}
	}

	input_report_key(input_dev, BTN_TOUCH, finger_num > 0);

	if (status != MXT_RELEASE) {
		input_report_abs(input_dev, ABS_X, finger[single_id].x);
		input_report_abs(input_dev, ABS_Y, finger[single_id].y);
		input_report_abs(input_dev,
				 ABS_PRESSURE, finger[single_id].pressure);
	}

	input_mt_report_pointer_emulation(input_dev, false);
	input_sync(input_dev);

}

static void mxt_input_touchevent(struct mxt_data *data,
				      struct mxt_message *message, int id)
{

	struct mxt_finger *finger = data->finger;
	struct device *dev = &data->client->dev;
	u8 status = message->message[0];
	int x;
	int y; 
	int area;
	int pressure;
	//printk("[Atmel] mxt_input_touchevent status = %d\n",status);
	/* Check the touch is present on the screen */

	if(id < 0) return ;

	if(mxt1664t==1){
		
//		printk("[Atmel] interrupt 1664t\n");
		int event = status & 0x0F;
		int type = status & 0x70;
//		printk("[Ateml] id = %d, type = %d, event = %d, status = %d\n",id,type,event,status);
		if ( (!(status & MXT_DETECT)) || event >= 5  ) {
//			if (status & MXT_RELEASE) {
		
			//if(temp >= 5 || temp ==0){
	
				dev_dbg(dev, "[%d] released\n", id);
				//printk("[%d] released\n", id);
				finger[id].status = MXT_RELEASE;
				mxt_input_report(data, id);
			//}
			return;
		}

		/* Check only AMP detection */
		//printk("[Atmel] Status = %d\n",status);
		if(!(event==1 || event==4))		
		if(!(status & (MXT_MOVE2 |MXT_PRESS2)))
			return;

//		x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
//		y = (message->message[2] << 4) | ((message->message[3] & 0xf));
	
		x = (message->message[1]) | ((message->message[2] << 8));
        	y = (message->message[3]) | ((message->message[4] << 8));

		if (data->max_x < 1024)
			x = x >> 2;
		if (data->max_y < 1024)
			y = y >> 2;

	

		area = message->message[7];
		pressure = message->message[6];


//		printk("[%d] %s x: %d, y: %d, area: %d, pressure: %d,\n", id,
//                        status & MXT_MOVE2 ? "moved" : "pressed",
//                        x, y, area, pressure);

                finger[id].status = status & MXT_MOVE2 ?
                                        MXT_MOVE2 : MXT_PRESS2;
                finger[id].x = x;
                finger[id].y = y;
                finger[id].area = area;
                finger[id].pressure = pressure;
	}else{
//		printk("[Atmel] interrupt 1664s\n");
		if (!(status & MXT_DETECT) || (status & MXT_RELEASE)) {
			//iif (status & MXT_RELEASE) {
				dev_dbg(dev, "[%d] released\n", id);
//				printk("[%d] released\n", id);
				finger[id].status = MXT_RELEASE;
				mxt_input_report(data, id);
			//}
			return;
		}

		/* Check only AMP detection */
		if (!(status & (MXT_PRESS | MXT_MOVE)))
			return;

		x = (message->message[1] << 4) | ((message->message[3] >> 4) & 0xf);
		y = (message->message[2] << 4) | ((message->message[3] & 0xf));
		if (data->max_x < 1024)
			x = x >> 2;
		if (data->max_y < 1024)
			y = y >> 2;

		area = message->message[4];
		pressure = message->message[5];

//		printk("[%d] %s x: %d, y: %d, area: %d, pressure : %d\n", id,
//                     status & MXT_MOVE ? "moved" : "pressed",
//                     x, y, area, pressure);

               finger[id].status = status & MXT_MOVE ?
                                       MXT_MOVE : MXT_PRESS;
               finger[id].x = x;
               finger[id].y = y;
               finger[id].area = area;
               finger[id].pressure = pressure;

	}
	mxt_input_report(data, id);

	
}

static irqreturn_t mxt_interrupt(int irq, void *dev_id)
{
	struct mxt_data *data = dev_id;
	struct mxt_message message;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int id;
	u8 reportid;
	u8 max_reportid;
	u8 min_reportid;
	
//	printk("[Atmel] mxt_interrupt \n");

	do {
//		printk("[Atmel] mxt_interrupt in do-while loop\n");
		if (mxt_read_message(data, &message)) {
			dev_err(dev, "Failed to read message\n");
			goto end;
		}

		reportid = message.reportid;

		/* whether reportid is thing of MXT_TOUCH_MULTI_T9 */
		object = mxt_get_object(data, MXT_TOUCH_MULTI_T9);
		if (!object)
			goto end;
	
		max_reportid = object->max_reportid;
		min_reportid = max_reportid - object->num_report_ids + 1;
		if(mxt1664t==1) id = reportid - min_reportid - 2;
		else id = reportid - min_reportid;
//		printk("[Atmel] max_reportid = %d, min_reportid = %d, reportid = %d\n",max_reportid,min_reportid,reportid);
		if (reportid >= min_reportid && reportid <= max_reportid)
			mxt_input_touchevent(data, &message, id);
		else
                        mxt_dump_message(dev, &message);
		
	} while (reportid != 0xff);

end:
	return IRQ_HANDLED;
}

static int mxt_check_reg_init(struct mxt_data *data)
{
	printk(KERN_ERR "[Atmel] probe : mxt_check_reg_init\n");
	/*const struct mxt_platform_data *pdata = data->pdata;
	struct mxt_object *object;
	struct device *dev = &data->client->dev;
	int index = 0;
	int i, j, config_offset;


	if (!pdata->config) {
		printk(KERN_ERR "[Atmel] probe : mxt_check_reg_init : No cfg data defined, skipping reg init\n");
		dev_dbg(dev, "No cfg data defined, skipping reg init\n");
		return 0;
	}

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		if (!mxt_object_writable(object->type))
			continue;

		for (j = 0;
		     j < (object->size + 1) * (object->instances + 1);
		     j++) {
			config_offset = index + j;
			if (config_offset > pdata->config_length) {
				dev_err(dev, "Not enough config data!\n");
				return -EINVAL;
			}
			mxt_write_object(data, object->type, j,
					 pdata->config[config_offset]);
		}
		index += (object->size + 1) * (object->instances + 1);
	}
	*/
	return 0;
}

static int mxt_make_highchg(struct mxt_data *data)
{
	struct device *dev = &data->client->dev;
	struct mxt_message message;
	int count = 10;
	int error;

	/* Read dummy message to make high CHG pin */
	do {
		error = mxt_read_message(data, &message);
		if (error)
			return error;
	} while (message.reportid != 0xff && --count);

	if (!count) {
		dev_err(dev, "CHG pin isn't cleared\n");
		return -EBUSY;
	}

	return 0;
}

#if 0
static void mxt_handle_pdata(struct mxt_data *data)
{
// Jui-Chuan Modified all "pdata" to "data" ++
//	const struct mxt_platform_data *pdata = data->pdata;
	u8 voltage;

	/* Set touchscreen lines */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_XSIZE,
			data->x_line);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_YSIZE,
			data->y_line);

	/* Set touchscreen orient */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9, MXT_TOUCH_ORIENT,
			data->orient);

	/* Set touchscreen burst length */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_BLEN, data->blen);

	/* Set touchscreen threshold */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_TCHTHR, data->threshold);

	/* Set touchscreen resolution */
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_XRANGE_LSB, (data->x_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_XRANGE_MSB, (data->x_size - 1) >> 8);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YRANGE_LSB, (data->y_size - 1) & 0xff);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YRANGE_MSB, (data->y_size - 1) >> 8);

	/* Set touchscreen voltage */
	if (data->voltage) {
		if (data->voltage < MXT_VOLTAGE_DEFAULT) {
			voltage = (MXT_VOLTAGE_DEFAULT - data->voltage) /
				MXT_VOLTAGE_STEP;
			voltage = 0xff - voltage + 1;
		} else
			voltage = (data->voltage - MXT_VOLTAGE_DEFAULT) /
				MXT_VOLTAGE_STEP;

		mxt_write_object(data, MXT_SPT_CTECONFIG_T28,
				MXT_CTE_VOLTAGE, voltage);
	}
	
	//Jui added for 
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_YEDGECTRL, 204);
	mxt_write_object(data, MXT_TOUCH_MULTI_T9,
			MXT_TOUCH_MOVHYSTN, 10);
// Jui-Chuan Modified all "pdata" to "data" --
}
#endif

static int mxt_get_info(struct mxt_data *data)
{
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;


        error = mxt_read_reg(client, MXT_FAMILY_ID, &val);
        if(error) 
		return error;
	printk("[Atmel] mxt_get_info MXT_FAMILY_ID 0x%02x\n", val);
	info->family_id = val;

	error = mxt_read_reg(client, MXT_VARIANT_ID, &val);
	printk("[Atmel] mxt_get_info MXT_VARIANT_ID 0x%02x\n", val);
	if (error)
		return error;
	info->variant_id = val;

	error = mxt_read_reg(client, MXT_VERSION, &val);
	printk("[Atmel] mxt_get_info MXT_VERSION 0x%02x\n", val);
	if (error)
		return error;
	info->version = val;
	data->touch_fw_version = val;

	error = mxt_read_reg(client, MXT_BUILD, &val);
	printk("[Atmel] mxt_get_info MXT_BUILD 0x%02x\n", val);
	if (error)
		return error;
	info->build = val;
	data->touch_fw_build = val;

	error = mxt_read_reg(client, MXT_OBJECT_NUM, &val);
	printk("[Atmel] mxt_get_info MXT_OBJECT_NUM 0x%02x\n", val);
	if (error)
		return error;

	info->object_num = val;


	return 0;
}

static int mxt_get_object_table(struct mxt_data *data)
{
	int error;
	int i;
	u16 reg;
	u8 reportid = 0;
	u8 buf[MXT_OBJECT_SIZE];

	for (i = 0; i < data->info.object_num; i++) {
		struct mxt_object *object = data->object_table + i;

		reg = MXT_OBJECT_START + MXT_OBJECT_SIZE * i;
		error = mxt_read_object_table(data->client, reg, buf);
		if (error)
			return error;

		object->type = buf[0];
		object->start_address = (buf[2] << 8) | buf[1];
		
#if 0		// Modified by Miracle. 2013/03/27. Refer to protocol guide.
		object->size = buf[3];
		object->instances = buf[4];
		object->num_report_ids = buf[5];
		if (object->num_report_ids) {
			reportid += object->num_report_ids *
					(object->instances + 1);
#else
		object->size = buf[3] + 1;
		object->instances = buf[4] + 1;
		object->num_report_ids = (u8)(object->instances * buf[5]);
		reportid += object->num_report_ids;

//		printk("[Atmel] type = %d, size = %d, instances = %d, num_report = %d, report_id = %d\n", object->type, object->size,object->instances,object->num_report_ids,reportid);
#endif

			object->max_reportid = reportid;
		//}  // Modified by Jui-Chuan
	}

	return 0;
}

static int mxt_initialize(struct mxt_data *data)
{
	
	struct i2c_client *client = data->client;
	struct mxt_info *info = &data->info;
	int error;
	u8 val;
	printk("[Ateml] mxt1664 mxt_initialize\n");
	error = mxt_get_info(data);
	if (error)
		return error;
//	printk("[Ateml] mxt1664 mxt_initialize mxt_get_info\n");
	data->object_table = kcalloc(info->object_num,
				     sizeof(struct mxt_object),
				     GFP_KERNEL);
	if (!data->object_table) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	/* Get object table information */
	error = mxt_get_object_table(data);
	if (error)
		return error;

	/* Check register init values */
	error = mxt_check_reg_init(data);
	if (error)
		return error;

	//mxt_handle_pdata(data);
	printk("[Ateml] mxt1664 mxt_initialize mxt_handle_pdata skip!\n");
	
//	if(after_update_fw == 1){
		if(mxt1664t==1) mxt_config_init_1664t(data);
		else mxt_config_init(data);
		after_update_fw = 0;
//	}
	/* Backup to memory */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_BACKUPNV,
			MXT_BACKUP_VALUE);
	msleep(MXT_BACKUP_TIME);
	printk("[Ateml] mxt1664 mxt_initialize Backup to memory\n");

	if(mxt1664t==1) msleep(100); // For special fw of mxt1664t

	/* Soft reset */
	mxt_write_object(data, MXT_GEN_COMMAND_T6,
			MXT_COMMAND_RESET, 1);
	msleep(MXT_RESET_TIME);
	touch_is_enable = 0;

	judge_1664t_addr();
	
	printk("[Ateml] mxt1664 mxt_initialize Soft reset\n");
	enable_touch();

	/* Update matrix size at info struct */
	error = mxt_read_reg(client, MXT_MATRIX_X_SIZE, &val);
	if (error)
		return error;
	info->matrix_xsize = val;
//	printk("[Ateml] mxt1664 mxt_initialize Update matrix X size at info struct \n");	
	error = mxt_read_reg(client, MXT_MATRIX_Y_SIZE, &val);
	if (error)
		return error;
	info->matrix_ysize = val;
//	printk("[Ateml] mxt1664 mxt_initialize Update matrix Y size at info struct\n");
	dev_info(&client->dev,
			"Family ID: %d Variant ID: %d Version: %d Build: %d\n",
			info->family_id, info->variant_id, info->version,
			info->build);

	dev_info(&client->dev,
			"Matrix X Size: %d Matrix Y Size: %d Object Num: %d\n",
			info->matrix_xsize, info->matrix_ysize,
			info->object_num);

	touch_status_value = 1;

	return 0;
}

static void mxt_calc_resolution(struct mxt_data *data)
{
	//Jui Modified all pdata to data ++ 
	//unsigned int max_x = data->pdata->x_size - 1; //original
	//unsigned int max_y = data->pdata->y_size - 1; //original
	unsigned int max_x = data->max_x - 1;
	unsigned int max_y = data->max_y - 1;
	
	if (data->orient & MXT_XY_SWITCH) {
		data->max_x = max_y;
		data->max_y = max_x;
	} else {
		data->max_x = max_x;
		data->max_y = max_y;
	}
	//Jui Modified all pdata to data --
}


static ssize_t mxt_Android_config(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = touch_chip;
        if(mxt1664t==1) mxt_config_init_1664t(data);
	else mxt_config_init(data);
	printk("[Ateml] mxt1664 mxt_Android_config Change to Android Configuration\n");
        /* Backup to memory */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_BACKUPNV,
                        MXT_BACKUP_VALUE);
        msleep(MXT_BACKUP_TIME);
        printk("[Ateml] mxt1664 mxt_Android_config Backup to memory\n");
        /* Soft reset */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_RESET, 1);
        msleep(MXT_RESET_TIME);
	touch_is_enable = 0;	
 
	judge_1664t_addr();
	
	enable_touch();
 
        printk("[Ateml] mxt1664 mxt_Android_config Soft reset\n");

}

static ssize_t mxt_int_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	int count = 0;
	int i;



		int val = gpio_get_value(touch_chip->int_gpio);
		count += snprintf(buf + count, PAGE_SIZE - count,
                "touch int value = %d\n",val);


	return count;

}


static ssize_t mxt_rst_show(struct device *dev,
                    struct device_attribute *attr, char *buf)
{
	int count = 0;
    int val = gpio_get_value(touch_chip->reset_gpio);
    count += snprintf(buf + count, PAGE_SIZE - count,
                "touch reset value = %d\n",val);
    return count;
}

static ssize_t mxt_object_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct mxt_object *object;
	int count = 0;
	int i, j;
	int error;
	u8 val;

	for (i = 0; i < data->info.object_num; i++) {
		object = data->object_table + i;

		count += snprintf(buf + count, PAGE_SIZE - count,
				"Object[%d] (Type %d)\n",
				i + 1, object->type);
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;

		if (!mxt_object_readable(object->type)) {
			count += snprintf(buf + count, PAGE_SIZE - count,
					"\n");
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
			continue;
		}

		// Modified by Miracle. 2013/03/27.
		// for (j = 0; j < object->size + 1; j++) {
		for (j = 0; j < object->size; j++) {
			error = mxt_read_object(data,
						object->type, j, &val);
			if (error)
				return error;

			count += snprintf(buf + count, PAGE_SIZE - count,
					"\t[%2d]: %02x (%d)\n", j, val, val);
			if (count >= PAGE_SIZE)
				return PAGE_SIZE - 1;
		}

		count += snprintf(buf + count, PAGE_SIZE - count, "\n");
		if (count >= PAGE_SIZE)
			return PAGE_SIZE - 1;
	}

	return count;
}

static int mxt_load_fw(struct device *dev, const char *fn)
{
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	const struct firmware *fw = NULL;
	unsigned int frame_size;
	unsigned int pos = 0;
	int ret;
	int i;



	//add for progress bar ++

//	mm_segment_t oldfs;
	mm_segment_t oldfs_progress;
	char Progress_file_path[] = "/data/touch_update_progress";
	struct file *filePtr = NULL;
	int update_progress_file = 0, len = 0, update_progress = 1;
	loff_t POS = 0;
	char temp_progress[3];

	int update_index = 1;

        filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
       	if(!IS_ERR_OR_NULL(filePtr)) {
               	update_progress_file=1;
                printk("[TouchPanel] %s: %s ok to write progress\n", __func__, Progress_file_path);
       	        filp_close(filePtr, NULL);
        } else if(PTR_ERR(filePtr) == -ENOENT) {
       	        update_progress_file=0;
               	printk("[TouchPanel] %s: %s not found\n", __func__, Progress_file_path);
        } else {
       	        update_progress_file=-1;
               	printk("[TouchPanel] %s: %s open error\n", __func__, Progress_file_path);
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
       	                POS = 0;
               	        len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &POS);
                       	set_fs(oldfs_progress);
                        filp_close(filePtr, NULL);
       	        }
        }
        //add for progress bar --

	ret = request_firmware(&fw, fn, dev);
	if (ret) {
		dev_err(dev, "Unable to open firmware %s\n", fn);
		return ret;
	}

	/* Change to the bootloader mode */
	if(chip_in_bootloader == 0){
		printk("[Atmel] Send command to change to bootloader mode\n");
		mxt_write_object(data, MXT_GEN_COMMAND_T6,
				MXT_COMMAND_RESET, MXT_BOOT_VALUE);
		msleep(MXT_RESET_TIME);	
	}
	

	/* Change to slave address of bootloader */
	if (client->addr == MXT_APP_LOW)
		client->addr = MXT_BOOT_LOW;
	else
		client->addr = MXT_BOOT_HIGH;

	printk("[Atmel] slave address of bootloader = %x\n",client->addr);


	ret = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);
	if (ret)
		goto out;

	/* Unlock bootloader */
	mxt_unlock_bootloader(client);

	while (pos < fw->size) {
#if 0	// Miracle. 2013/05/03. Double check wait frame data event.
		ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
		if (ret)
			goto out;
#else
		for(i=0; i < 3; i++) {
			ret = mxt_check_bootloader(client,
						MXT_WAITING_FRAME_DATA);
			if (!ret)
				break;
			msleep(500);
		}
		if(i==3)
			goto out;
#endif

		frame_size = ((*(fw->data + pos) << 8) | *(fw->data + pos + 1));

		/* We should add 2 at frame size as the the firmware data is not
		 * included the CRC bytes.
		 */
		frame_size += 2;

		//printk("[Atmel] frame_size= %d bytes, fw->size= %zd bytes\n",frame_size,fw->size);


#if 0
		/* Write one frame to device */
		mxt_fw_write(client, fw->data + pos, frame_size);
//		msleep(150);	
		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);

		if (ret)
			goto out;

		pos += frame_size;
#else
#define FRAME_SIZE		(60)
		for(;frame_size>0;) {
			/* Write one frame to device */
			if(frame_size > FRAME_SIZE) {
				mxt_fw_write(client, fw->data + pos, FRAME_SIZE);
				pos += FRAME_SIZE;
				frame_size -= FRAME_SIZE;
			}
			else {
				mxt_fw_write(client, fw->data + pos, frame_size);
				pos += frame_size;
				frame_size  = 0;
			}
//			msleep(50);	
		}

//		msleep(50);	
		ret = mxt_check_bootloader(client,
						MXT_FRAME_CRC_PASS);
		msleep(10);	

		if (ret)
			goto out;

#endif
		//printk("[Atmel] Updated %d bytes / %zd bytes\n", pos, fw->size);

		//dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);

		if(pos >= update_index*10000 && update_index <= 9){
			
			//printk("[Atmel] Updated %d bytes / %zd bytes\n", pos, fw->size);
			//dev_dbg(dev, "Updated %d bytes / %zd bytes\n", pos, fw->size);
			printk("[Atmel] fw update %d %\n", update_index*10);
	
			if(build_version!=1){
				//add for progress bar ++
				//update_progress = update_progress + 5;
				update_progress = update_index*10;
				sprintf(temp_progress, "%d", update_progress);
				//printk("[ITE_update] %s: temp_progress = %s \n", __func__, temp_progress);
				filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
				if(!IS_ERR_OR_NULL(filePtr)) {
					oldfs_progress = get_fs();
					set_fs(get_ds());
					POS = 0;
					len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &POS);
					set_fs(oldfs_progress);
					filp_close(filePtr, NULL);
				}
				//add for progress bar --
			}
			update_index++;
		}
	}// while

	printk("[Atmel] fw update 100 %\n");
	//add for progress bar ++
	/*
	if(build_version!=1){
        	//update_progress = update_progress + 5;
        	update_progress = 100;
        	sprintf(temp_progress, "%d", update_progress);
        	//printk("[ITE_update] %s: temp_progress = %s \n", __func__, temp_progress);
        	filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,(S_IWUSR|S_IRUGO));
        	if(!IS_ERR_OR_NULL(filePtr)) {
	        	oldfs_progress = get_fs();
	        	set_fs(get_ds());
		        POS = 0;
        		len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &POS);
		        set_fs(oldfs_progress);
        		filp_close(filePtr, NULL);
        	}
	}
	*/
        //add for progress bar --

out:


	release_firmware(fw);

	/* Change to slave address of application */
	if (client->addr == MXT_BOOT_LOW)
		client->addr = MXT_APP_LOW;
	else
		client->addr = MXT_APP_HIGH;

	printk("[Atmel] Do soft reset\n");
	msleep(500); // Use for i2c transfer fail
        /* Soft reset */
 	mxt_write_object(data, MXT_GEN_COMMAND_T6,
        MXT_COMMAND_RESET, 1);
	msleep(MXT_RESET_TIME);	
//	touch_is_enable = 0;
	
	judge_1664t_addr();

	if(data->suspend_state==0) enable_touch();

	printk("[Atmel] slave address of application = %x\n",client->addr);

	return ret;
}

static ssize_t mxt_update_fw(struct device *dev,
					struct device_attribute *attr,
					const char *buf)
{
	printk("[Atmel] mxt_update_fw\n");
	struct mxt_data *data = dev_get_drvdata(dev);
	struct i2c_client *client = data->client;
	int error;
	int count;
	int retry = 5;
	wake_lock(&data->wakelock);
	data->tp_firmware_upgrade_proceed = 1;

	if(irq_is_enable==1){
		disable_irq(data->irq);
		irq_is_enable = 0;
	}
	is_fw_update_finish = 1;

	if(mxt1664t==1) error = mxt_load_fw(dev, MXT_FW_NAME_1664T);
	else error = mxt_load_fw(dev, MXT_FW_NAME);
	while (error && retry) {
		dev_err(dev, "The firmware update failed(%d)\n", error);
		printk("[Atmel] retry = %d\n",retry--);
		count = error;
		is_fw_update_finish = -1;
		msleep(500);
		if(mxt1664t==1) {
                        if(client->addr == 0x4b) client->addr = 0x4a;
                	else if(client->addr == 0x4a) client->addr = 0x4b;
                }	
		if(mxt1664t==1) error = mxt_load_fw(dev, MXT_FW_NAME_1664T);
	        else error = mxt_load_fw(dev, MXT_FW_NAME);
	} 
	if(!error) {
		dev_dbg(dev, "The firmware update succeeded\n");
		is_fw_update_finish = 0;
		need_to_update_fw = 0;
		/* Wait for reset */
		msleep(MXT_FWRESET_TIME);

		judge_1664t_addr();

		kfree(data->object_table);
		data->object_table = NULL;
		after_update_fw = 1;
	
		mxt_initialize(data);
	}
	if(irq_is_enable==0){
		enable_irq(data->irq);	
		irq_is_enable = 1;
	}
	data->tp_firmware_upgrade_proceed = 0;
        wake_unlock(&data->wakelock);

	error = mxt_make_highchg(data);
	if (error)
		return error;

	return count;
}

// Jui-Chuan add for Self test ++
static ssize_t mxt_self_test(struct device *dev,struct device_attribute *attr, char *buf){
	struct mxt_data *data = touch_chip;
        struct mxt_message message;
        struct mxt_object *object;
        
        int id;
        u8 reportid;
        u8 max_reportid;
        u8 min_reportid;
	
	int retval = 0;
	int error = 0;
	
	//printf("Start self test\n");
	/* Self test */
	if(irq_is_enable==1){
		disable_irq(touch_chip->irq); 
		irq_is_enable = 0;
	}	

	retval = mxt_write_object(data, MXT_SPT_SELFTEST_T25, MXT_ADR_T25_CTRL, 0x03); // Enable and report message
	if (retval < 0)
			return -1;
	retval = mxt_write_object(data, MXT_SPT_SELFTEST_T25, MXT_ADR_T25_CMD, MXT_MSGR_T25_RUN_ALL_TESTS); // All test
	if (retval < 0)
                        return -1;

	int wait_count = 100;
	while(true) {
                
		printk("[Atmel] touch_chip->int_gpio = %d\n",gpio_get_value(touch_chip->int_gpio));
		if(wait_count && gpio_get_value(touch_chip->int_gpio)!=0){
			dev_err(dev, "Wait for low int_gpio\n");
			msleep(100);
			wait_count--;
                        continue;
		}

		if (mxt_read_message(data, &message)) {
                        dev_err(dev, "Failed to read message\n");
                        break;
                }

                reportid = message.reportid;
                /* whether reportid is thing of MXT_SPT_SELFTEST_T25 */
                object = mxt_get_object(data, MXT_SPT_SELFTEST_T25);
                if (!object)
                     break;
                max_reportid = object->max_reportid;
                min_reportid = max_reportid - object->num_report_ids + 1;
                id = reportid - min_reportid;
                if (reportid >= min_reportid && reportid <= max_reportid){
                        if(message.message[0]==0xFE){   // Run all test OK !
                                dev_info(dev,
                                        "maXTouch: Self-Test OK\n");
                                strcpy(self_test_result,"1");
                        }else{
                                dev_err(dev,
                                        "maXTouch: Self-Test Failed [%02x]:"
                                        "{%02x,%02x,%02x,%02x,%02x}\n",
                                        message.message[MXT_MSG_T25_STATUS],
                                        message.message[MXT_MSG_T25_STATUS + 0],
                                        message.message[MXT_MSG_T25_STATUS + 1],
                                        message.message[MXT_MSG_T25_STATUS + 2],
                                        message.message[MXT_MSG_T25_STATUS + 3],
                                        message.message[MXT_MSG_T25_STATUS + 4]
                                );
                                strcpy(self_test_result,"0");

                                if(message.message[0]==0xfd)
                                        dev_err(dev,"maXTouch: The test code supplied in the CMD field is "
                                                        "not associated with a valid test\n");
                                if(message.message[0]==0xfc)
                                        dev_err(dev,"maXTouch: The test could not be completed due to an unrelated fault"
                                                        "(for example, an internal communications problem)\n");
                                if(message.message[0]==0x01)
                                        dev_err(dev,"maXTouch: AVdd is not present on at least one of the slave devices\n");
                                if(message.message[0]==0x12)
                                        dev_err(dev,"maXTouch: The initial pin fault test failed following power-on or reset\n");
                                if(message.message[0]==0x17)
                                        dev_err(dev,"maXTouch: The test failed because of a signal limit fault\n");
                        }
                }else
                        mxt_dump_message(dev, &message);

		break;
        }

	
	int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "%s\n",self_test_result);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;
	if(irq_is_enable==0){
		enable_irq(touch_chip->irq);
		irq_is_enable = 1;
	}
	error = mxt_make_highchg(data);
        if (error)
                return error;

        return count;	

}

// Jui-Chuan add for Self test --

// Jui-Chuan add for fine tune ++
static ssize_t mxt_object_write(struct device *dev,
                                        struct device_attribute *attr,
                                        const char *buf, size_t count){
	int object = 0;
	int offset = 0;
	int mesg = 0;
	struct mxt_data *data = touch_chip;
	int i;
	int c = 0;
	int index = 0;
	char** temp = (char**)kcalloc(3,sizeof(char*),GFP_KERNEL);
	for(i=0;i<3;i++) temp[i] = (char*)kcalloc(10,sizeof(char),GFP_KERNEL);
	
	i=0;
	while(index<=count){
		if(buf[index] != '+' && buf[index] != '\n') c++;
		else{
			strncpy(temp[i],buf+index-c,c);
			printk("temp[%d] = %s\n",i,temp[i]);
			i++;
			c = 0;
		}
		index++;
	}

	printk("i=%d  index=%d\n",i,index);
	sscanf(temp[0],"%d",&object);
	sscanf(temp[1],"%d",&offset);
	sscanf(temp[2],"%x",&mesg);

	//mxt_write_reg(data->client, MXT_BASE_ADDR(object, data) + offset, mesg);	

	mxt_write_object(data, object, offset, mesg);

	/* Backup to memory */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_BACKUPNV,
                        MXT_BACKUP_VALUE);
        msleep(MXT_BACKUP_TIME);

	/* Soft reset */
        mxt_write_object(data, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_RESET, 1);
        msleep(MXT_RESET_TIME);	
	touch_is_enable = 0;        

	judge_1664t_addr();

	if(data->suspend_state==0) enable_touch();

	for(i=0;i<3;i++) kfree(temp[i]);
	kfree(temp);

	return count;
}

// Jui-Chuan add for fine tune --

static ssize_t mxt_get_fw_ver(struct device *dev,struct device_attribute *attr, char *buf){
        struct mxt_data *data = touch_chip;
	u8 val_1 = 0;
	u8 val_2 = 0;
	int error = 0;
	printk("[Atmel] mxt_get_fw_ver\n");
	if(!chip_in_bootloader){
		error = mxt_read_reg(data->client, MXT_VERSION, &val_1);
		error = mxt_read_reg(data->client, MXT_BUILD, &val_2);
	}

	int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "v%02x%02x\n",val_1,val_2);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

	return count;

}

static ssize_t mxt_check_update_fw(struct device *dev,struct device_attribute *attr, char *buf){
	struct mxt_data *data = touch_chip;
        u8 val_version = 0;
	u8 val_build = 0;
        int error = 0;
	int ret = 1;
	printk("[Atmel] mxt_check_update_fw\n");
	if(chip_in_bootloader == 1){
		 ret = 0;
	}else{
	        error = mxt_read_reg(data->client, MXT_VERSION, &val_version);
		error = mxt_read_reg(data->client, MXT_BUILD, &val_build);

		if(mxt1664t==1){
			if(val_version != NEWEST_FW_VERSION_1664T || val_build != NEWEST_FW_BUILD_1664T) ret = 0;
			else ret = 1;
		}else{
			if(val_version != NEWEST_FW_VERSION || val_build != NEWEST_FW_BUILD) ret = 0;
        	        else ret = 1;
		}
	}

        int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "%d\n",ret);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

        return count;
}

static ssize_t mxt_check_update_fw_finish(struct device *dev,struct device_attribute *attr, char *buf){

        int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "%d\n",is_fw_update_finish);

	if(chip_in_bootloader==1 && build_version!=1) {
		int error;
		//rpmsg_send_generic_simple_command(IPCMSG_COLD_RESET, 0);
		printk("[Ateml] mxt1664 probe mxt_initialize END\n");
	        error = request_threaded_irq(touch_chip->irq, NULL, mxt_interrupt,
                	        IRQF_TRIGGER_FALLING, touch_chip->client->dev.driver->name, touch_chip); // Modified
        	printk("[Ateml] mxt1664 probe request_threaded_irq\n");
	        //if (error) {
                //	dev_err(&client->dev, "Failed to register interrupt\n");
        	//        goto err_free_object;
	        //}

        	error = mxt_make_highchg(touch_chip);

	        //if (error)
        	//        goto err_free_irq;
	        printk("[Ateml] mxt1664 probe mxt_make_highchg\n");
        	error = input_register_device(input_dev);
	        //if (error)
                //	goto err_free_irq;

	        // Jui-Chuan add for debug data ++
        	touch_chip->debug_dir = debugfs_create_dir("maXTouch", NULL);
	        if (touch_chip->debug_dir == ERR_PTR(-ENODEV)) {
                	/* debugfs is not enabled. */
        	        printk(KERN_WARNING "debugfs not enabled in kernel\n");
	        } else if (touch_chip->debug_dir == NULL) {
        	        printk(KERN_WARNING "error creating debugfs dir\n");
	        } else {
        	        //mxt_debug(DEBUG_TRACE, "created \"maXTouch\" debugfs dir\n");

	                debugfs_create_file("deltas", S_IRUSR, touch_chip->debug_dir, touch_chip,
                                	    &delta_fops);
                	debugfs_create_file("refs", S_IRUSR, touch_chip->debug_dir, touch_chip,
                        	            &refs_fops);
        	}
	        // Jui-Chuan add for debug data --
		
	        // Jui-Chuan add for early suspend and late resume ++
	        #ifdef CONFIG_HAS_EARLYSUSPEND
        	touch_chip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	        touch_chip->early_suspend.suspend = mxt_early_suspend;
        	touch_chip->early_suspend.resume = mxt_late_resume;
	        register_early_suspend(&touch_chip->early_suspend);
        	#endif

	        touch_chip->suspend_state = 0;
        	// Jui-Chuan add for early suspend and late resume --

		mxt_register_dock_notifier();
	        //add switeh device for report touch fw version
        	touch_chip->touch_sdev.name = TOUCH_SDEV_NAME;
	        touch_chip->touch_sdev.print_name = switch_device_get_fw_ver;
        	touch_chip->touch_sdev.print_state = switch_device_state;
	        if(switch_dev_register(&touch_chip->touch_sdev) < 0){
                	printk("[Atmel]: switch_dev_register for touch failed!\n");
        	}
	        switch_set_state(&touch_chip->touch_sdev, 0);
		
		chip_in_bootloader = 0;
	}

        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

        return count;
}


static ssize_t mxt_touch_status(struct device *dev,struct device_attribute *attr, char *buf){

        int count = 0;

        count += snprintf(buf + count, PAGE_SIZE - count, "%d\n",touch_status_value);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

        return count;
}

int mxt_load_config_sub_function(char* token){
	int len = strlen(token);
	int c = 0;
	int i = 0;
	int index = 0;	
	int object = 0;
	int instance = 0;
	int num = 0;
	int break_flag = 0;
	int offset = 0;


	while(i<len+1){
		if(token[i] == '\0') break_flag = 1;
		if(token[i] == ' ' || token[i] == '\0'){
			char temp_value[10] = {0};
			u8 value = 0;
			strncpy(temp_value,token+i-c,c);
			sscanf(temp_value,"%x",&value);
                        c = 0;

			if(index == 0) {
				object = value;
				index++;
			}else if(index == 1){
				instance = value;
				index++;
				if(instance!=0) break;
			}else if(index == 2){
				num = value;
				index++;
			}else if(index == 3){
				printk("[Atmel] %d %d %d %d %d \n",object,offset,instance,num,value);
				mxt_write_object(touch_chip, object, offset, value);
				offset++;
			}

			if(break_flag==1) break;
		}else c++;

		i++;
	}
	return 0;
}

mm_segment_t oldfs; 
static ssize_t mxt_load_config(struct device *dev,struct device_attribute *attr, char *buf){
	char file[100] = "/data/mxt_config.raw" ;
	struct file *fp = NULL;
	char one[1024] = {0};
	int count = 0;
	int ret = 0;
	char* temp = kmalloc(10000*sizeof(char),GFP_KERNEL);
	int i = 0;
	int j = 0;
	//char token[10] = {0};
	int c = 0;

	oldfs = get_fs(); 
	set_fs(KERNEL_DS);	

	fp = filp_open(file,O_RDONLY,0);
	if(!fp) {
		printk("[Atmel] Cannot open the config raw\n");
		return 0;
	}
	
	while(fp->f_op->read(fp,one,1024, &fp->f_pos)>0){
//		printk("[Atmel] %s\n",one);
		for(i=0;i<1024;i++){
			temp[j*1024 + i] = one[i];
		}
		j++;
	}
	
	i = 0;
        while(i<=10000){
                if(temp[i] != '\n') c++;
                else{
			char token[1024] = {0};
                        strncpy(token,temp+i-c,c);
//                      printk("token = %s\n",token);
                        c = 0;
			if(strncmp(token,"END",3)==0) break;
			else{
				mxt_load_config_sub_function(token);
			}
                }
                i++;
        }

	filp_close(fp,NULL);
	set_fs(oldfs);

	/* Backup to memory */
        mxt_write_object(touch_chip, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_BACKUPNV,
                        MXT_BACKUP_VALUE);
        msleep(MXT_BACKUP_TIME);


        /* Soft reset */
        mxt_write_object(touch_chip, MXT_GEN_COMMAND_T6,
                        MXT_COMMAND_RESET, 1);
        msleep(MXT_RESET_TIME);
	touch_is_enable = 0;
 
	judge_1664t_addr();

	enable_touch();

	return count;

}

static ssize_t mxt_trigger_reset(struct device *dev,struct device_attribute *attr, char *buf){
	/* Do HW reset */
        printk("[Atmel] Do HW reset\n");
        gpio_direction_output(touch_chip->reset_gpio, 0);
        msleep(1);
        gpio_set_value(touch_chip->reset_gpio, 1);
        msleep(100);
	touch_is_enable = 0;

	judge_1664t_addr();

	if(touch_chip->suspend_state==0) enable_touch();	
}

static ssize_t mxt_highchg(struct device *dev,struct device_attribute *attr, char *buf){

	mxt_make_highchg(touch_chip);

}

static ssize_t mxt_check_suspend_state(struct device *dev,struct device_attribute *attr, char *buf){

	struct mxt_data *data = touch_chip;
        u8 val_1 = 0;
        u8 val_2 = 0;

	mxt_read_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data), &val_1);
        mxt_read_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data) + 1, &val_2);
        
	return sprintf(buf, "%02x %02x\n",val_1,val_2);
}

static ssize_t mxt_need_to_update_fw(struct device *dev,struct device_attribute *attr, char *buf){
	struct mxt_data *data = touch_chip;
        u8 val_version = 0;
        u8 val_build = 0;
        int error = 0;
        printk("[Atmel] mxt_check_update_fw\n");

        error = mxt_read_reg(data->client, MXT_VERSION, &val_version);
        error = mxt_read_reg(data->client, MXT_BUILD, &val_build);

        if(mxt1664t==1){
                if(val_version != NEWEST_FW_VERSION_1664T || val_build != NEWEST_FW_BUILD_1664T) need_to_update_fw = 1;
                else need_to_update_fw = 0;
        }else{
                if(val_version != NEWEST_FW_VERSION || val_build != NEWEST_FW_BUILD) need_to_update_fw = 1;
                else need_to_update_fw = 1;
        }


        int count = 0;
        count += snprintf(buf + count, PAGE_SIZE - count, "%d\n",need_to_update_fw);
        if (count >= PAGE_SIZE)
                        return PAGE_SIZE - 1;

        return count;
}

static DEVICE_ATTR(check_need_to_update_fw, 0444, mxt_need_to_update_fw, NULL);
static DEVICE_ATTR(suspend_state, 0444, mxt_check_suspend_state, NULL);
static DEVICE_ATTR(highchg, 0444, mxt_highchg, NULL);
static DEVICE_ATTR(trigger_reset, 0444, mxt_trigger_reset, NULL);
static DEVICE_ATTR(load_config, 0444, mxt_load_config, NULL);
static DEVICE_ATTR(touch_status, 0444, mxt_touch_status, NULL);
static DEVICE_ATTR(check_update_fw, 0444, mxt_check_update_fw, NULL);
static DEVICE_ATTR(check_update_fw_finish, 0444, mxt_check_update_fw_finish, NULL);
static DEVICE_ATTR(get_fw_ver, 0444, mxt_get_fw_ver, NULL);
static DEVICE_ATTR(Change_to_Android, 0444, mxt_Android_config, NULL);
static DEVICE_ATTR(INT_value, 0444, mxt_int_show, NULL);
static DEVICE_ATTR(RST_value, 0444, mxt_rst_show, NULL);
static DEVICE_ATTR(object, 0444, mxt_object_show, NULL);
static DEVICE_ATTR(update_fw, 0444, mxt_update_fw, NULL);
// Jui-Chuan add for self test ++
static DEVICE_ATTR(self_test, 0444, mxt_self_test, NULL);
// Jui-Chuan add for self test --
// Jui-Chuan add for fine tune ++
static DEVICE_ATTR(write_object, 0664, NULL, mxt_object_write);
// Jui-Chuan add for fine tune --


static struct attribute *mxt_attrs[] = {
	
	&dev_attr_check_need_to_update_fw.attr,
	&dev_attr_suspend_state.attr,
	&dev_attr_highchg.attr,
	&dev_attr_trigger_reset.attr,
	&dev_attr_load_config.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_check_update_fw.attr,
	&dev_attr_check_update_fw_finish.attr,

	&dev_attr_get_fw_ver.attr,

	&dev_attr_Change_to_Android.attr,
	&dev_attr_object.attr,
	&dev_attr_INT_value.attr,
	&dev_attr_RST_value.attr,
	&dev_attr_update_fw.attr,
	// Jui-Chuan add for self test ++
	&dev_attr_self_test.attr,
	// Jui-Chuan add for self test --
	// Jui-Chuan add for fine tune ++
	&dev_attr_write_object.attr,
	// Jui-Chuan add for fine tune --
	NULL
};

static const struct attribute_group mxt_attr_group = {
	.attrs = mxt_attrs,
};

// Jui-Chuan add for read write block ++
static int mxt_read_block(struct i2c_client *client,
                   u16 addr,
                   u16 length,
                   u8 *value)
{
        struct i2c_adapter *adapter = client->adapter;
        struct i2c_msg msg[2];
        __le16  le_addr;
        struct mxt_data *mxt;

        mxt = i2c_get_clientdata(client);

        if (mxt != NULL) {
                if ((mxt->last_read_addr == addr) &&
                        (addr == mxt->msg_proc_addr)) {
                        if  (i2c_master_recv(client, value, length) == length)
                                return length;
                        else
                                return -EIO;
                } else {
                        mxt->last_read_addr = addr;
                }
        }

        //mxt_debug(DEBUG_TRACE, "Writing address pointer & reading %d bytes "
        //        "in on i2c transaction...\n", length);

        le_addr = cpu_to_le16(addr);
        msg[0].addr  = client->addr;
        msg[0].flags = 0x00;
        msg[0].len   = 2;
        msg[0].buf   = (u8 *) &le_addr;

        msg[1].addr  = client->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len   = length;
        msg[1].buf   = (u8 *) value;
        if  (i2c_transfer(adapter, msg, 2) == 2)
                return length;
        else
                return -EIO;

}

static int mxt_write_block(struct i2c_client *client,
                    u16 addr,
                    u16 length,
                    u8 *value)
{
        int i;
        struct {
                __le16  le_addr;
                u8      data[256];

        } i2c_block_transfer;

        struct mxt_data *mxt;

        //mxt_debug(DEBUG_TRACE, "Writing %d bytes to %d...", length, addr);
        if (length > 256)
                return -EINVAL;
        mxt = i2c_get_clientdata(client);
        if (mxt != NULL)
                mxt->last_read_addr = -1;
        for (i = 0; i < length; i++)
                i2c_block_transfer.data[i] = *value++;
        i2c_block_transfer.le_addr = cpu_to_le16(addr);
        i = i2c_master_send(client, (u8 *) &i2c_block_transfer, length + 2);
        if (i == (length + 2))
                return length;
        else
                return -EIO;
}
// Jui-Chuan add for read write block --
    
static void mxt_start(struct mxt_data *data)
{
	// Jui-Chuan add for calibration ++ 
	char buf[2];
     	int error;
	int retry = 3; // Set retry for i2c transfer fail
	
	//dummy read to wake up from deep sleep mode
//	mxt_read_block(data->client, MXT_BASE_ADDR(MXT_SPT_COMMSCONFIG_T18, data), 1, buf);
//	mdelay(25);
//	error = mxt_read_block(data->client, MXT_BASE_ADDR(MXT_USER_INFO_T38, data), 1, buf);
	error = mxt_make_highchg(data);

	// Resume from Deep sleep mode 
	error |= mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data), 0x0F);
	error |= mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data)+1, 0xFF);
	error |= mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_PROCI_TOUCHSUPPRESSION_T42, data), 0);

	/* Touch enable */
	enable_touch();
}

static void mxt_stop(struct mxt_data *data)
{
	/* Touch disable */
	disable_touch();

	// Jui-Chuan add for Deep sleep mode ++

	// Go into Deep sleep Mode
	mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data), 0);
        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_POWER_T7, data) + 1, 0);

	// Jui-Chuan add for Deep sleep mode --

}

static int mxt_input_open(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_start(data);

	return 0;
}

static void mxt_input_close(struct input_dev *dev)
{
	struct mxt_data *data = input_get_drvdata(dev);

	mxt_stop(data);
}


// Jui-Chuan add for raw data ++
#define MXT_ADR_T6_DIAGNOSTIC                           0x05
/* SPT_DEBUG_DIAGNOSTIC_T37 Register offsets from T37 base address */
#define MXT_ADR_T37_PAGE                                0x01
#define	MXT_ADR_T37_DATA				0x02
/* T6 Debug Diagnostics Commands */
#define	MXT_CMD_T6_PAGE_UP          0x01
#define	MXT_CMD_T6_PAGE_DOWN        0x02
#define	MXT_CMD_T6_DELTAS_MODE      0x10
#define	MXT_CMD_T6_REFERENCES_MODE  0x11
#define	MXT_CMD_T6_CTE_MODE         0x31
ssize_t debug_data_read(struct mxt_data *mxt, char *buf, size_t count,
                        loff_t *ppos, u8 debug_command)
{
        int i;
        u16 *data;
        u16 diagnostics_reg;
        int offset = 0;
        int size;
        int read_size;
        int error;
        char *buf_start;
        u16 debug_data_addr;
        u16 page_address;
        u8 page;
        u8 debug_command_reg;

	char *temp_buf = kmalloc(PAGE_SIZE, GFP_KERNEL);        
	ssize_t temp_len, temp_ret = 0;


        data = mxt->debug_data;
        if (data == NULL)
                return -EIO;

	printk("[Atmel] debug hello !\n");
	printk("[Atmel] debug counts = %d!\n", count);


        /* If first read after open, read all data to buffer. */
        if (mxt->current_debug_datap == 0) {
                // Jui-Chuan Modified for raw data ++
		diagnostics_reg =
                        MXT_BASE_ADDR(MXT_GEN_COMMAND_T6, mxt) +
                        MXT_ADR_T6_DIAGNOSTIC;
                if (count > (mxt->num_nodes * 2))
                        count = mxt->num_nodes;
                debug_data_addr =
                        MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
                        MXT_ADR_T37_DATA;
                page_address = MXT_BASE_ADDR(MXT_DEBUG_DIAGNOSTIC_T37, mxt) +
                               MXT_ADR_T37_PAGE; 
		
		
/*		struct mxt_object *temp_object;	
		u16 temp_reg;
		temp_object = mxt_get_object(mxt,MXT_GEN_COMMAND_T6);
		temp_reg = temp_object->start_address;	
		diagnostics_reg = temp_reg + MXT_ADR_T6_DIAGNOSTIC;

		if (count > (mxt->num_nodes * 2))
                        count = mxt->num_nodes;

		temp_object = mxt_get_object(mxt,MXT_DEBUG_DIAGNOSTIC_T37);
                temp_reg = temp_object->start_address;
		debug_data_addr = temp_reg + MXT_ADR_T37_DATA;
		page_address = temp_reg + MXT_ADR_T37_PAGE;
*/
		// Jui-Chuan Modified for raw data --
		printk("[Atmel] debug setting \n");
                // error = mxt_read_block(mxt->client, page_address, 1, &page);
		error = mxt_read_reg(mxt->client,page_address,&page);
                if (error < 0)
                        return error;
                //mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
		

		printk("[Atmel] page = %d, debug_command = %d\n",page,debug_command);
		
		
                while (page != 0) {
                        //error = mxt_write_byte(mxt->client,
                        //                diagnostics_reg,
                        //                MXT_CMD_T6_PAGE_DOWN);
			//printk("[Atmel] in while loop start\n");	
			
			error = mxt_write_reg(mxt->client,
                                        diagnostics_reg,
                                        MXT_CMD_T6_PAGE_DOWN);

                        if (error < 0)
                                return error;
                        /* Wait for command to be handled; when it has, the
                           register will be cleared. */
                        debug_command_reg = 1;
			
                        while (debug_command_reg != 0) {
                                error = mxt_read_block(mxt->client,
                                                diagnostics_reg, 1,
                                                &debug_command_reg);

				//error = mxt_read_reg(mxt->client,
                                //                diagnostics_reg,
                                //                &debug_command_reg);

                                if (error < 0)
                                        return error;
                                //mxt_debug(DEBUG_TRACE,
                                //        "Waiting for debug diag command "
                                //        "to propagate...\n");
				
                        }
                        error = mxt_read_block(mxt->client, page_address, 1,
                                              &page);
			//error = mxt_read_reg(mxt->client, page_address,&page);
			printk("[Atmel] page = %d\n",page);

                        if (error < 0)
                                return error;
                        //mxt_debug(DEBUG_TRACE, "debug data page = %d\n", page);
			printk("[Atmel] in while loop\n");
                }

                /*
                 * Lock mutex to prevent writing some unwanted data to debug
                 * command register. User can still write through the char
                 * device interface though. TODO: fix?
                 */

                //mutex_lock(&mxt->debug_mutex);
                /* Configure Debug Diagnostics object to show deltas/refs */
                //error = mxt_write_byte(mxt->client, diagnostics_reg,
                //                debug_command);
	
		printk("[Atmel] debug_command = %d\n");
		
	
		error = mxt_write_reg(mxt->client, diagnostics_reg,
                                debug_command);
                /* Wait for command to be handled; when it has, the
                 * register will be cleared. */
                debug_command_reg = 1;
                while (debug_command_reg != 0) {
                        error = mxt_read_block(mxt->client,
                                        diagnostics_reg, 1,
                                        &debug_command_reg);
			//error = mxt_read_reg(mxt->client,
                        //                diagnostics_reg,
                        //                &debug_command_reg);

                        if (error < 0)
                                return error;
                        //mxt_debug(DEBUG_TRACE, "Waiting for debug diag command "
                        //        "to propagate...\n");
                }

                if (error < 0) {
                        printk(KERN_WARNING
                                "Error writing to maXTouch device!\n");
                        return error;
                }


                size = mxt->num_nodes * sizeof(u16);

		

                while (size > 0) {
                        read_size = size > 128 ? 128 : size;
			//read_size = 2;
                        //mxt_debug(DEBUG_TRACE,
                        //        "Debug data read loop, reading %d bytes...\n",
                        //        read_size);
                        error = mxt_read_block(mxt->client,
                                               debug_data_addr,
                                               read_size,
                                               (u8 *) &data[offset]);
			//error = mxt_read_reg(mxt->client,
                        //                       debug_data_addr,
                        //                       (u8 *) &data[offset]);

                        if (error < 0) {
                                printk(KERN_WARNING
                                        "Error reading debug data\n");
                                goto error;
                        }
                        offset += read_size/2;
                        size -= read_size;

                        /* Select next page */
                        //error = mxt_write_byte(mxt->client, diagnostics_reg,
                        //                MXT_CMD_T6_PAGE_UP);
			error = mxt_write_reg(mxt->client, diagnostics_reg,
                                        MXT_CMD_T6_PAGE_UP);
                        if (error < 0) {
                                printk(KERN_WARNING
                                        "Error writing to maXTouch device!\n");
                                goto error;
                        }
			msleep(100);
                }
                //mutex_unlock(&mxt->debug_mutex);

        } //end if

        buf_start = buf;
        i = mxt->current_debug_datap;

        while (((buf - buf_start) < (count - 6)) &&
                (i < mxt->num_nodes)) {

                mxt->current_debug_datap++;
                if (debug_command == MXT_CMD_T6_REFERENCES_MODE)
                        buf += sprintf(buf, "%5d ",
                                       (u16) le16_to_cpu(data[i]));
                else if (debug_command == MXT_CMD_T6_DELTAS_MODE)
                        buf += sprintf(buf, "%5d ",
                                       (s16) le16_to_cpu(data[i]));

                i++;

		if(i%mxt->y_line==0) buf += sprintf(buf, "\n");
        }

        return buf - buf_start;
error:
        mutex_unlock(&mxt->debug_mutex);
        return error;
}

ssize_t deltas_read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
        return debug_data_read(file->private_data, buf, count, ppos,
                               MXT_CMD_T6_DELTAS_MODE);
}

ssize_t refs_read(struct file *file, char *buf, size_t count,
                        loff_t *ppos)
{
        return debug_data_read(file->private_data, buf, count, ppos,
                               MXT_CMD_T6_REFERENCES_MODE);
}

int debug_data_open(struct inode *inode, struct file *file)
{

        struct mxt_data *mxt;
        int i;
        mxt = inode->i_private;
        if (mxt == NULL)
                return -EIO;
        mxt->current_debug_datap = 0;
	
        //mxt->debug_data = kmalloc(mxt->device_info.num_nodes * sizeof(u16),
        //                          GFP_KERNEL);
	mxt->debug_data = kmalloc(mxt->num_nodes * sizeof(u16),
                                  GFP_KERNEL);
        if (mxt->debug_data == NULL)
                return -ENOMEM;
        //for (i = 0; i < mxt->device_info.num_nodes; i++)
	for (i = 0; i < mxt->num_nodes; i++)
                mxt->debug_data[i] = 7777;
        file->private_data = mxt;
	printk("[Atmel] debug_data_open\n");
        return 0;
}

static int debug_data_release(struct inode *inode, struct file *file)
{
        struct mxt_data *mxt;
        mxt = file->private_data;
        kfree(mxt->debug_data);
	printk("[Atmel] debug_data_release\n");
        return 0;
}

static const struct file_operations delta_fops = {
        .owner = THIS_MODULE,
        .open = debug_data_open,
        .release = debug_data_release,
        .read = deltas_read,
};

static const struct file_operations refs_fops = {
        .owner = THIS_MODULE,
        .open = debug_data_open,
        .release = debug_data_release,
        .read = refs_read,
};
// Jui-Chuan add for raw data --

// Jui-Chuan add for read write memory ++

/*
 * enable_touch_called_by_EC
 * When EC FW updating, it will trigger touch HW reset.
 * This function used for enable touch by EC driver
 */
int enable_touch_called_by_EC(void){

	if(touch_is_enable==1 && system_owner!=SYSTEM_WINDOWS){
		printk("[Atmel] enable_touch_called_by_EC\n");
//		msleep(300);
		touch_is_enable=0;
//		enable_touch();
	}
	return 0;
}
EXPORT_SYMBOL(enable_touch_called_by_EC);
/* 
 * update_touch_fw_called_by_EC
 * Use for updating or recovery touch fw in factory image
 */
int update_touch_fw_called_by_EC(void){
	if(touch_chip==NULL) {
		notify_EC();
		return 0;	
	}
	if(mxt1664t==0 && (touch_chip->touch_fw_version!=NEWEST_FW_VERSION || touch_chip->touch_fw_build!=NEWEST_FW_BUILD)){
                if(build_version==1){
                        printk("[Atmel] ENG version (ER) !!   We update touch FW after 3 seconds\n");
                        queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 3*HZ);
                }
        }else if(mxt1664t==1 && (touch_chip->touch_fw_version!=NEWEST_FW_VERSION_1664T || touch_chip->touch_fw_build!=NEWEST_FW_BUILD_1664T)){
                if(build_version==1){
                        printk("[Atmel] ENG version (MXT1664T) !!   We update touch FW after 3 seconds\n");
                        queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 3*HZ);
                }
        }else{
                notify_EC();
        }
	
}
EXPORT_SYMBOL(update_touch_fw_called_by_EC);

/*
 * touch_chip_self_firmware_upgrade
 * Use for updating or recovery touch fw in user image
 */
static int touch_chip_self_firmware_upgrade(struct work_struct *dat)
{
        struct i2c_client *client;
        client = touch_chip->client;

        int try_again = 2;
        int error = 0;


//        client->addr = 0x27;
//        error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);

            printk("[Atmel] We try FW update\n");
            struct device *dev = &client->dev;

            wake_lock(&touch_chip->wakelock);
            touch_chip->tp_firmware_upgrade_proceed = 1;

	    if(irq_is_enable==1){
            	disable_irq(touch_chip->irq);
	    	irq_is_enable = 0;
	    }
            if(mxt1664t==1) error = mxt_load_fw(dev, MXT_FW_NAME_1664T);
            else error = mxt_load_fw(dev, MXT_FW_NAME);
            while (error!=0 && try_again!=0) {
                try_again--;
                printk("[Atmel] try again %d\n",try_again);
                msleep(500);
		if(mxt1664t==1) error = mxt_load_fw(dev, MXT_FW_NAME_1664T);
            	else error = mxt_load_fw(dev, MXT_FW_NAME);
            }

            if (error) {
                    printk("[Atmel] The firmware update failed(%d)\n", error);
                    

            } else {
                printk("[Atmel] The firmware update succeeded\n");
                msleep(MXT_FWRESET_TIME);
                kfree(touch_chip->object_table);
                touch_chip->object_table = NULL;
                //after_update_fw = 1;
                mxt_initialize(touch_chip);
                //mxt_probe(client,mxt_id);
		chip_in_bootloader = 0;
            }


            if(irq_is_enable==0){ 
		enable_irq(touch_chip->irq);
	    	irq_is_enable = 1;
	    }
            touch_chip->tp_firmware_upgrade_proceed = 0;
            wake_unlock(&touch_chip->wakelock);

                msleep(200);

	   
		notify_EC();
//        client->addr = 0x4b;
        return 0;
}


static int touch_chip_self_firmware_upgrade_if_fw_broken(struct work_struct *dat)
{
	struct i2c_client *client;	
	client = touch_chip->client; 

	int try_again = 2;
	int error = 0;

        printk("[Atmel] Check is in bootloader mode\n");
        client->addr = 0x27;
//        error = mxt_check_bootloader(client, MXT_WAITING_BOOTLOAD_CMD);

        u8 buf[2];
        buf[0] = 0x00;
        buf[1] = 0x00;
	if (i2c_master_send(client, buf, 2) == 2) {

            printk("[Atmel] We try FW update\n");
            struct device *dev = &client->dev;

            wake_lock(&touch_chip->wakelock);
            touch_chip->tp_firmware_upgrade_proceed = 1;
	    if(irq_is_enable==1){
            	disable_irq(touch_chip->irq);
	    	irq_is_enable = 0;
	    }
	    if(mxt1664t==1) error = mxt_load_fw(dev, MXT_FW_NAME_1664T);
	    else error = mxt_load_fw(dev, MXT_FW_NAME);
            while (error!=0 && try_again!=0) {
		try_again--;
		printk("[Atmel] try again %d\n",try_again);
		msleep(500);
		if(mxt1664t==1) error = mxt_load_fw(dev, MXT_FW_NAME_1664T);
		else error = mxt_load_fw(dev, MXT_FW_NAME);
	    }
	    
            if (error) {
                    printk("[Atmel] The firmware update failed(%d)\n", error);
		    return -1;

            } else {
                printk("[Atmel] The firmware update succeeded\n");
                msleep(MXT_FWRESET_TIME);
                kfree(touch_chip->object_table);
                touch_chip->object_table = NULL;
                //after_update_fw = 1;
                mxt_initialize(touch_chip);
		//mxt_probe(client,mxt_id);

            }
            if(irq_is_enable==0){
            	enable_irq(touch_chip->irq);
	    	irq_is_enable = 1;
	    }
	    touch_chip->tp_firmware_upgrade_proceed = 0;
       	    wake_unlock(&touch_chip->wakelock);

		msleep(200);
        }
        client->addr = 0x4b;
	return 0;        
}


void mxt_register_dock_notifier(){
	printk("[Atmel] Register_dock_notifier\n");
#if 1
	// Add for new code base
	register_dock_atow_early_notifier(&atow_early_dock_notifier);
	register_dock_atow_late_notifier(&atow_late_dock_notifier);
	register_dock_wtoa_late_notifier(&wtoa_late_dock_notifier);
	register_dock_detach_notifier(&detach_dock_notifier);
	register_dock_attach_notifier(&attach_dock_notifier);
#else
	// Add for old code base
	register_dock_notifier(&dock_notifier);
#endif
}

void mxt_unregister_dock_notifier(){
        printk("[Atmel] Unregister_dock_notifier\n");
#if 1
	// Add for new code base
        unregister_dock_atow_early_notifier(&atow_early_dock_notifier);
	unregister_dock_atow_late_notifier(&atow_late_dock_notifier);
        unregister_dock_wtoa_late_notifier(&wtoa_late_dock_notifier);
        unregister_dock_detach_notifier(&detach_dock_notifier);
        unregister_dock_attach_notifier(&attach_dock_notifier);
#else
	// Add for old code base
	unregister_dock_notifier(&dock_notifier);
#endif
}

static ssize_t switch_device_get_fw_ver(struct switch_dev *sdev, char *buf){
        struct mxt_data *data = touch_chip;
        u8 val_1 = 0;
        u8 val_2 = 0;
        
        mxt_read_reg(data->client, MXT_VERSION, &val_1);
        mxt_read_reg(data->client, MXT_BUILD, &val_2);
        return sprintf(buf, "v%02x%02x\n",val_1,val_2);
}


static ssize_t switch_device_state(struct switch_dev *sdev, char *buf)
{
        return sprintf(buf, "%s\n", "0");
}

int check_bootloader(struct i2c_client *client){
	int retval = 0;
	int retval_bootloader = 0;
	int temp = client->addr;
	printk("[Atmel] Check is in bootloader mode\n");
	client->addr = 0x27;
        u8 buf[2];
        buf[0] = 0x00;
        buf[1] = 0x00;
        retval = i2c_master_send(client, buf, 2);
        if (retval != 2) {
                printk("[Atmel] Chip is not in bootloader mode (0x27)\n");
        }else{
                printk("[Atmel] Check again\n");
                msleep(100);
                retval = i2c_master_send(client, buf, 2);
                if(retval != 2){
                        printk("[Atmel] Chip leave bootloader mode (0x27)\n");
                }else{
                        printk("[Atmel] Chip is in bootloader mode (0x27)\n");
                        printk("[Atmel] Need to check touch chip FW\n");
			retval_bootloader = 1;
			return retval_bootloader;
			//For reference, don't remove
			//client->addr = 0x4b;
			//chip_in_bootloader = 1;	
			//need_to_update_fw = 1;
                }
        }
	// Check for mxt1664t at attach open condition ++
	if(mxt1664t==1 && chip_in_bootloader==0){

		client->addr = 0x26;
	        u8 buf[2];
        	buf[0] = 0x00;
	        buf[1] = 0x00;
        	retval = i2c_master_send(client, buf, 2);
	        if (retval != 2) {
        	        printk("[Atmel] Chip is not in bootloader mode (0x26)\n");
	        }else{
                	printk("[Atmel] Check again\n");
        	        msleep(100);
	                retval = i2c_master_send(client, buf, 2);
                	if(retval != 2){
        	                printk("[Atmel] Chip leave bootloader mode (0x26)\n");
	                }else{
                	        printk("[Atmel] Chip is in bootloader mode (0x26)\n");
        	                printk("[Atmel] Need to check touch chip FW\n");
				retval_bootloader = 2;
				return retval_bootloader;
				//For reference, don't remove
				//client->addr = 0x4a;
	                        //chip_in_bootloader = 1;
                        	//need_to_update_fw = 1;
                	}
        	}
	}
	client->addr = temp;
	return retval_bootloader;
	// Check for mxt1664t at attach open condition --
}

#define PASS_PROBE_ERROR 0
static int __devinit mxt_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{

	printk("Atmel mxt1664s probe start ! \n");
	printk("[Atmel][1] touch int value = %d\n",gpio_get_value(62));

	if(entry_mode==4) {
		printk("[Atmel] In COS, skip\n");
		return;
	}else if(entry_mode==3) {
                printk("[Atmel] In POS, skip\n");
                return;
        }

	int HW_ID = Read_HW_ID();
	if(HW_ID==7 || HW_ID==6 || HW_ID==1) {
		printk("[Atmel] mxt1664t in mxt1664s driver\n");
		MXT_TOUCH_MULTI_T9 = 0x64;
		mxt1664t = 1;
	}else{
		MXT_TOUCH_MULTI_T9 = 9;
		mxt1664t = 0;
	}
	printk("[Atmel] mxt1664t = %d\n",mxt1664t);
	// Initialize the global variance ++
	after_update_fw = 0;	
	is_fw_update_finish = 1;	
	irq_is_enable = 0;
	touch_status_value = 0;
	chip_in_bootloader = 0;
	touch_is_enable = 0;	
	system_owner = SYSTEM_ANDROID;
	need_to_update_fw = 0;
	// Initialize the global variance --

	//const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
//	struct input_dev *input_dev;
	int error;
	int retval = 0;
	int retry = 3;
	// Jui-Chuan for pdata ++
	//if (!pdata)
	//	return -EINVAL;
	// Jui-Chuan for pdata --	

	touch_chip = kzalloc(sizeof(struct mxt_data), GFP_KERNEL); // allocate memory for touch_chip
	data = touch_chip;


	touch_chip->mxt_wq = create_singlethread_workqueue("mxt_wq");
	INIT_DELAYED_WORK(&touch_chip->touch_chip_firmware_upgrade_work, touch_chip_self_firmware_upgrade);
	INIT_DELAYED_WORK(&touch_chip->touch_chip_firmware_upgrade_work_if_fw_broken, touch_chip_self_firmware_upgrade_if_fw_broken);
	INIT_DELAYED_WORK(&touch_chip->touch_chip_register_dock_notifier, mxt_register_dock_notifier);
//	queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "Atmel-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
//	input_dev->open = mxt_input_open;
//	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	// Jui-Chuan for pdata ++
	//data->pdata = pdata;
	// Jui-Chuan for pdata --

	// Configuration from Appendix B Reference Configuration ++
	data->max_x = 4095;
	data->max_y = 4095;
	if(mxt1664t==1){
		data->x_line = 41; //
                data->y_line = 72; //
	}else{
		data->x_line = 30; //
		data->y_line = 52; //
	}
	data->x_size = 50;
	data->y_size = 50;
	data->blen = 110;   //
	data->threshold = 50; //
	data->voltage = MXT_VOLTAGE_DEFAULT;
	data->orient = 1;

	data->numtouch = 10;

	mutex_init(&data->touch_mutex);
	// Configuration from Appendix B Reference Configuration --

	// Jui-Chuan for pdata ++
	data->reset_gpio = 88;
	data->int_gpio = 62;
	// Jui-Chuan for pdata --

	data->num_nodes = data->x_line * data->y_line;
	
        //--------------------------------------------------------------------------------------------
	/*init INTERRUPT pin*/
	error = gpio_request(data->int_gpio, "AtmelTouch-irq");
	if(error < 0)
		printk("[Atmel] %s:Failed to request GPIO%d (ElanTouch-interrupt) error=%d\n", __func__, data->int_gpio, error);

	error = gpio_direction_input(data->int_gpio);
	if (error){
		printk("[Atmel] %s:Failed to set interrupt direction, error=%d\n", __func__, error);
		gpio_free(data->int_gpio);
	}	
	irq_is_enable = 1;


	data->irq = gpio_to_irq(data->int_gpio);
	printk("[Atmel] %s: irq=%d \n", __func__, data->irq);
//	irq_is_enable = 1;
	/*init RESET pin*/
	error = gpio_request(data->reset_gpio, "AtmelTouch-reset");
	if (error < 0)
		printk("[Atmel] %s:Failed to request GPIO%d (ElanTouch-reset) error=%d\n", __func__, data->reset_gpio, error);

	// Hardware reset ++
	error = gpio_direction_output(data->reset_gpio, 0);
	if (error){
		printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
		gpio_free(data->reset_gpio);
	}

	msleep(1);
	gpio_set_value(data->reset_gpio, 1);
	msleep(MXT_RESET_TIME);
	// Hardware reset --
        //--------------------------------------------------------------------------------------------
	printk("[Ateml] mxt1664 probe init INTERRUPT pin\n");

	client->irq = data->irq;

	
	mxt_calc_resolution(data);
	printk("[Ateml] mxt1664 probe mxt_calc_resolution\n");

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	input_mt_init_slots(input_dev, MXT_MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MXT_MAX_FINGER, 0, 0);
	printk("[Ateml] mxt1664 probe set abs params\n");
	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

#if 0	// Jui-Chuan add ++
        printk("[Atmel] Check is in bootloader mode\n");
        
	client->addr = 0x27;
        u8 buf[2];
        buf[0] = 0x00;
        buf[1] = 0x00;
        retval = i2c_master_send(client, buf, 2);
        if (retval != 2) {
                printk("[Atmel] Chip is not in bootloader mode (0x27)\n");
        }else{
                printk("[Atmel] Check again\n");
                msleep(100);
                retval = i2c_master_send(client, buf, 2);
                if(retval != 2){
                        printk("[Atmel] Chip leave bootloader mode (0x27)\n");
                }else{
                        printk("[Atmel] Chip is in bootloader mode (0x27)\n");
                        printk("[Atmel] Need to check touch chip FW\n");
			client->addr = 0x4b;
			chip_in_bootloader = 1;	
			need_to_update_fw = 1;
                }
        }
	// Check for mxt1664t at attach open condition ++
	if(mxt1664t==1 && chip_in_bootloader==0){

		client->addr = 0x26;
	        u8 buf[2];
        	buf[0] = 0x00;
	        buf[1] = 0x00;
        	retval = i2c_master_send(client, buf, 2);
	        if (retval != 2) {
        	        printk("[Atmel] Chip is not in bootloader mode (0x26)\n");
	        }else{
                	printk("[Atmel] Check again\n");
        	        msleep(100);
	                retval = i2c_master_send(client, buf, 2);
                	if(retval != 2){
        	                printk("[Atmel] Chip leave bootloader mode (0x26)\n");
	                }else{
                	        printk("[Atmel] Chip is in bootloader mode (0x26)\n");
        	                printk("[Atmel] Need to check touch chip FW\n");
				client->addr = 0x4a;
	                        chip_in_bootloader = 1;
                        	need_to_update_fw = 1;
                	}
        	}
	}
	// Check for mxt1664t at attach open condition --
#else 
        int retval_bootloader = check_bootloader(client);
        if(retval_bootloader==1){
                client->addr = 0x4b;
                chip_in_bootloader = 1;
                need_to_update_fw = 1;
        }else if(retval_bootloader==2){
                client->addr = 0x4a;
                chip_in_bootloader = 1;
                need_to_update_fw = 1;
        }
#endif	// Jui-Chuan add --
	if(chip_in_bootloader==0){
		// Set default application address is 0x4b
		client->addr = 0x4b;
		printk("[Ateml] mxt1664 probe mxt_initialize START\n");
		error = mxt_initialize(data);
		while(error && retry!=0){
			printk("[Atmel] INT_value = %d, ", gpio_get_value(touch_chip->int_gpio));
			printk("[Atmel] RST_value = %d\n", gpio_get_value(touch_chip->reset_gpio));
			// Hardware reset ++
		        error = gpio_direction_output(data->reset_gpio, 0);
	        	if (error){
                		printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
		                gpio_free(data->reset_gpio);
	        	}

	        	msleep(1);
        		gpio_set_value(data->reset_gpio, 1);
	        	msleep(600); // More delay for 1664t first boot
			// Hardware reset --
			retry--;
			check_bootloader(client);
			if(mxt1664t==1) {
				if(client->addr == 0x4b) client->addr = 0x4a;
				else if(client->addr == 0x4a) client->addr = 0x4b;
			}
			printk("[Atmel] mxt_initialize fail, retry = %d\n",retry);
			error = mxt_initialize(data);
		}
	}else{
		if(build_version!=1){	
			sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
			return 0;
		}	
	}
	
	if (error)
		goto err_free_object;



	printk("[Ateml] mxt1664 probe mxt_initialize END\n");
	error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, data); // Modified
	printk("[Ateml] mxt1664 probe request_threaded_irq\n");
	if (error && build_version!=1) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
	
	error = mxt_make_highchg(data);


	if (error && build_version!=1)
		goto err_free_irq;
	printk("[Ateml] mxt1664 probe mxt_make_highchg\n");
	error = input_register_device(input_dev);
	if (error && build_version!=1)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error && build_version!=1)
		goto err_unregister_device;


	// Jui-Chuan add for debug data ++
	data->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (data->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (data->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		//mxt_debug(DEBUG_TRACE, "created \"maXTouch\" debugfs dir\n");

		debugfs_create_file("deltas", S_IRUSR, data->debug_dir, data,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, data->debug_dir, data,
				    &refs_fops);
	}
	// Jui-Chuan add for debug data --

	data->suspend_state = 0;
	
	// Jui-Chuan add for early suspend and late resume ++
	#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
	#endif
	// Jui-Chuan add for early suspend and late resume --



	// Update FW called by EC in factory image
	if(mxt1664t==1){
		if(data->touch_fw_version != NEWEST_FW_VERSION_1664T || data->touch_fw_build != NEWEST_FW_BUILD_1664T){
			need_to_update_fw = 1;
                        //if(build_version==1){
                        //        printk("[Atmel] ENG version (MXT1664T) !!   We update touch FW after 20 seconds\n");
                        //        queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);
                        //}
                }
	}else{
		if(data->touch_fw_version != NEWEST_FW_VERSION || data->touch_fw_build != NEWEST_FW_BUILD){
			need_to_update_fw = 1;
			//if(build_version==1){ 
			//	printk("[Atmel] ENG version (Not ER) !!   We update touch FW after 20 seconds\n");
			//	queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);
			//}
		}
	}

	//register_dock_notifier(&dock_notifier);
	//printk("[Atmel] Register_dock_notifier\n");
	//queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_register_dock_notifier, 5*HZ);
	mxt_register_dock_notifier();

	//add switeh device for report touch fw version
        touch_chip->touch_sdev.name = TOUCH_SDEV_NAME;
        touch_chip->touch_sdev.print_name = switch_device_get_fw_ver;
        touch_chip->touch_sdev.print_state = switch_device_state;
        if(switch_dev_register(&touch_chip->touch_sdev) < 0){
                printk("[Atmel]: switch_dev_register for touch failed!\n");
        }
        switch_set_state(&touch_chip->touch_sdev, 0);


	return 0;

err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	global_client = client;
	kfree(data->object_table);
	touch_chip = NULL;
	mxt_register_dock_notifier();
//	register_dock_wtoa_late_notifier(&late_dock_notifier);
//      register_dock_detach_notifier(&detach_dock_notifier);
	gpio_free(data->int_gpio);
	gpio_free(data->reset_gpio);
	input_free_device(input_dev);
        kfree(data);
        return 0;
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
}

static int mxt_late_probe(struct i2c_client *client){
	
	printk("Atmel mxt1664s probe start ! \n");
	printk("[Atmel][1] touch int value = %d\n",gpio_get_value(62));
	
	if(entry_mode==4) {
		printk("[Atmel] In COS, skip\n");
		return;
	}else if(entry_mode==3) {
                printk("[Atmel] In POS, skip\n");
                return;
        }

	int HW_ID = Read_HW_ID();
	if(HW_ID==7 || HW_ID==6 || HW_ID==1) {
		printk("[Atmel] mxt1664t in mxt1664s driver\n");
		MXT_TOUCH_MULTI_T9 = 0x64;
		mxt1664t = 1;
	}else{
		MXT_TOUCH_MULTI_T9 = 9;
		mxt1664t = 0;
	}
	printk("[Atmel] mxt1664t = %d\n",mxt1664t);
	after_update_fw = 0;	
	is_fw_update_finish = 1;	
	irq_is_enable = 0;
	touch_status_value = 0;
	chip_in_bootloader = 0;
	
	
	system_owner = SYSTEM_ANDROID;
	need_to_update_fw = 0;
	//const struct mxt_platform_data *pdata = client->dev.platform_data;
	struct mxt_data *data;
//	struct input_dev *input_dev;
	int error;
	int retval = 0;
	int retry = 3;
	// Jui-Chuan for pdata ++
	//if (!pdata)
	//	return -EINVAL;
	// Jui-Chuan for pdata --	

	touch_chip = kzalloc(sizeof(struct mxt_data), GFP_KERNEL); // allocate memory for touch_chip
	data = touch_chip;


	touch_chip->mxt_wq = create_singlethread_workqueue("mxt_wq");
	INIT_DELAYED_WORK(&touch_chip->touch_chip_firmware_upgrade_work, touch_chip_self_firmware_upgrade);
	INIT_DELAYED_WORK(&touch_chip->touch_chip_firmware_upgrade_work_if_fw_broken, touch_chip_self_firmware_upgrade_if_fw_broken);
	INIT_DELAYED_WORK(&touch_chip->touch_chip_register_dock_notifier, mxt_register_dock_notifier);
//	queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);

	input_dev = input_allocate_device();
	if (!data || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		error = -ENOMEM;
		goto err_free_mem;
	}

	input_dev->name = "Atmel-touchscreen";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &client->dev;
//	input_dev->open = mxt_input_open;
//	input_dev->close = mxt_input_close;

	data->client = client;
	data->input_dev = input_dev;
	// Jui-Chuan for pdata ++
	//data->pdata = pdata;
	// Jui-Chuan for pdata --

	// Configuration from Appendix B Reference Configuration ++
	data->max_x = 4095;
	data->max_y = 4095;
	if(mxt1664t==1){
		data->x_line = 41; //
                data->y_line = 72; //
	}else{
		data->x_line = 30; //
		data->y_line = 52; //
	}
	data->x_size = 50;
	data->y_size = 50;
	data->blen = 110;   //
	data->threshold = 50; //
	data->voltage = MXT_VOLTAGE_DEFAULT;
	data->orient = 1;

	data->numtouch = 10;

	mutex_init(&data->touch_mutex);
	// Configuration from Appendix B Reference Configuration --

	// Jui-Chuan for pdata ++
	data->reset_gpio = 88;
	data->int_gpio = 62;
	// Jui-Chuan for pdata --

	data->num_nodes = data->x_line * data->y_line;
	
        //--------------------------------------------------------------------------------------------
	/*init INTERRUPT pin*/
	error = gpio_request(data->int_gpio, "AtmelTouch-irq");
	if(error < 0)
		printk("[Atmel] %s:Failed to request GPIO%d (ElanTouch-interrupt) error=%d\n", __func__, data->int_gpio, error);

	error = gpio_direction_input(data->int_gpio);
	if (error){
		printk("[Atmel] %s:Failed to set interrupt direction, error=%d\n", __func__, error);
		gpio_free(data->int_gpio);
	}	
	irq_is_enable = 1;


	data->irq = gpio_to_irq(data->int_gpio);
	printk("[Atmel] %s: irq=%d \n", __func__, data->irq);
//	irq_is_enable = 1;
	/*init RESET pin*/
	error = gpio_request(data->reset_gpio, "AtmelTouch-reset");
	if (error < 0)
		printk("[Atmel] %s:Failed to request GPIO%d (ElanTouch-reset) error=%d\n", __func__, data->reset_gpio, error);

	// Hardware reset ++
	error = gpio_direction_output(data->reset_gpio, 0);
	if (error){
		printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
		gpio_free(data->reset_gpio);
	}

	msleep(1);
	gpio_set_value(data->reset_gpio, 1);
	msleep(MXT_RESET_TIME);
	// Hardware reset --
        //--------------------------------------------------------------------------------------------
	printk("[Ateml] mxt1664 probe init INTERRUPT pin\n");

	client->irq = data->irq;

	
	mxt_calc_resolution(data);
	printk("[Ateml] mxt1664 probe mxt_calc_resolution\n");

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	/* For single touch */
	input_set_abs_params(input_dev, ABS_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE,
			     0, 255, 0, 0);

	/* For multi touch */
	input_mt_init_slots(input_dev, MXT_MAX_FINGER);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR,
			     0, MXT_MAX_AREA, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X,
			     0, data->max_x, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_Y,
			     0, data->max_y, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_PRESSURE,
			     0, 255, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_TRACKING_ID, 0, MXT_MAX_FINGER, 0, 0);
	printk("[Ateml] mxt1664 probe set abs params\n");
	input_set_drvdata(input_dev, data);
	i2c_set_clientdata(client, data);

#if 0	// Jui-Chuan add ++
        printk("[Atmel] Check is in bootloader mode\n");
        
	client->addr = 0x27;
        u8 buf[2];
        buf[0] = 0x00;
        buf[1] = 0x00;
        retval = i2c_master_send(client, buf, 2);
        if (retval != 2) {
                printk("[Atmel] Chip is not in bootloader mode (0x27)\n");
        }else{
                printk("[Atmel] Check again\n");
                msleep(100);
                retval = i2c_master_send(client, buf, 2);
                if(retval != 2){
                        printk("[Atmel] Chip leave bootloader mode (0x27)\n");
                }else{
                        printk("[Atmel] Chip is in bootloader mode (0x27)\n");
                        printk("[Atmel] Need to check touch chip FW\n");
			client->addr = 0x4b;
			chip_in_bootloader = 1;	
			need_to_update_fw = 1;
                }
        }
	// Check for mxt1664t at attach open condition ++
	if(mxt1664t==1 && chip_in_bootloader==0){

		client->addr = 0x26;
	        u8 buf[2];
        	buf[0] = 0x00;
	        buf[1] = 0x00;
        	retval = i2c_master_send(client, buf, 2);
	        if (retval != 2) {
        	        printk("[Atmel] Chip is not in bootloader mode (0x26)\n");
	        }else{
                	printk("[Atmel] Check again\n");
        	        msleep(100);
	                retval = i2c_master_send(client, buf, 2);
                	if(retval != 2){
        	                printk("[Atmel] Chip leave bootloader mode (0x26)\n");
	                }else{
                	        printk("[Atmel] Chip is in bootloader mode (0x26)\n");
        	                printk("[Atmel] Need to check touch chip FW\n");
				client->addr = 0x4a;
	                        chip_in_bootloader = 1;
                        	need_to_update_fw = 1;
                	}
        	}
	}
	// Check for mxt1664t at attach open condition --
#else
	int retval_bootloader = check_bootloader(client);
	if(retval_bootloader==1){
		client->addr = 0x4b;
                chip_in_bootloader = 1;
                need_to_update_fw = 1;
	}else if(retval_bootloader==2){
		client->addr = 0x4a;
                chip_in_bootloader = 1;
                need_to_update_fw = 1;
	}

#endif	// Jui-Chuan add --
	if(chip_in_bootloader==0){
		// Set default application address is 0x4b
		client->addr = 0x4a;
		printk("[Ateml] mxt1664 probe mxt_initialize START\n");
		error = mxt_initialize(data);
		while(error && retry!=0){
			printk("[Atmel] INT_value = %d, ", gpio_get_value(touch_chip->int_gpio));
			printk("[Atmel] RST_value = %d\n", gpio_get_value(touch_chip->reset_gpio));
			// Hardware reset ++
		        error = gpio_direction_output(data->reset_gpio, 0);
	        	if (error){
                		printk("[Atmel] %s:Failed to set reset direction, error=%d\n", __func__, error);
		                gpio_free(data->reset_gpio);
	        	}

	        	msleep(1);
        		gpio_set_value(data->reset_gpio, 1);
	        	msleep(600); // More delay for 1664t first boot
			// Hardware reset --
			retry--;
			check_bootloader(client);
			if(mxt1664t==1) {
				if(client->addr == 0x4b) client->addr = 0x4a;
				else if(client->addr == 0x4a) client->addr = 0x4b;
			}
			printk("[Atmel] mxt_initialize fail, retry = %d, addr = 0x%x.\n",retry, data->client->addr);
			error = mxt_initialize(data);
		}
	}else{
		if(build_version!=1){	
			sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
			return 0;
		}	
	}
	
	if (error)
		goto err_free_object;



	printk("[Ateml] mxt1664 probe mxt_initialize END\n");
	error = request_threaded_irq(data->irq, NULL, mxt_interrupt,
			IRQF_TRIGGER_FALLING, client->dev.driver->name, data); // Modified
	printk("[Ateml] mxt1664 probe request_threaded_irq\n");
	if (error && build_version!=1) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		goto err_free_object;
	}
	
	error = mxt_make_highchg(data);


	if (error && build_version!=1)
		goto err_free_irq;
	printk("[Ateml] mxt1664 probe mxt_make_highchg\n");
	error = input_register_device(input_dev);
	if (error && build_version!=1)
		goto err_free_irq;

	error = sysfs_create_group(&client->dev.kobj, &mxt_attr_group);
	if (error && build_version!=1)
		goto err_unregister_device;


	// Jui-Chuan add for debug data ++
	data->debug_dir = debugfs_create_dir("maXTouch", NULL);
	if (data->debug_dir == ERR_PTR(-ENODEV)) {
		/* debugfs is not enabled. */
		printk(KERN_WARNING "debugfs not enabled in kernel\n");
	} else if (data->debug_dir == NULL) {
		printk(KERN_WARNING "error creating debugfs dir\n");
	} else {
		//mxt_debug(DEBUG_TRACE, "created \"maXTouch\" debugfs dir\n");

		debugfs_create_file("deltas", S_IRUSR, data->debug_dir, data,
				    &delta_fops);
		debugfs_create_file("refs", S_IRUSR, data->debug_dir, data,
				    &refs_fops);
	}
	// Jui-Chuan add for debug data --

	data->suspend_state = 0;
	
	// Jui-Chuan add for early suspend and late resume ++
	#ifdef CONFIG_HAS_EARLYSUSPEND
	data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	data->early_suspend.suspend = mxt_early_suspend;
	data->early_suspend.resume = mxt_late_resume;
	register_early_suspend(&data->early_suspend);
	#endif
	// Jui-Chuan add for early suspend and late resume --



	// Update FW called by EC in factory image
	if(mxt1664t==1){
		if(data->touch_fw_version != NEWEST_FW_VERSION_1664T || data->touch_fw_build != NEWEST_FW_BUILD_1664T){
			need_to_update_fw = 1;
                        //if(build_version==1){
                        //        printk("[Atmel] ENG version (MXT1664T) !!   We update touch FW after 20 seconds\n");
                        //        queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);
                        //}
                }
	}else{
		if(data->touch_fw_version != NEWEST_FW_VERSION || data->touch_fw_build != NEWEST_FW_BUILD){
			need_to_update_fw = 1;
			//if(build_version==1){ 
			//	printk("[Atmel] ENG version (Not ER) !!   We update touch FW after 20 seconds\n");
			//	queue_delayed_work(touch_chip->mxt_wq, &touch_chip->touch_chip_firmware_upgrade_work, 20*HZ);
			//}
		}
	}

	//add switeh device for report touch fw version
        touch_chip->touch_sdev.name = TOUCH_SDEV_NAME;
        touch_chip->touch_sdev.print_name = switch_device_get_fw_ver;
        touch_chip->touch_sdev.print_state = switch_device_state;
        if(switch_dev_register(&touch_chip->touch_sdev) < 0){
                printk("[Atmel]: switch_dev_register for touch failed!\n");
        }
        switch_set_state(&touch_chip->touch_sdev, 0);


	return 0;
/*
err_unregister_device:
	input_unregister_device(input_dev);
	input_dev = NULL;
err_free_irq:
	free_irq(client->irq, data);
err_free_object:
	kfree(data->object_table);
err_free_mem:
	input_free_device(input_dev);
	kfree(data);
	return error;
*/

err_unregister_device:
        input_unregister_device(input_dev);
        input_dev = NULL;
err_free_irq:
        free_irq(client->irq, data);
err_free_object:
        global_client = client;
        kfree(data->object_table);
        touch_chip = NULL;
//        mxt_register_dock_notifier();
//      register_dock_wtoa_late_notifier(&late_dock_notifier);
//      register_dock_detach_notifier(&detach_dock_notifier);
        gpio_free(data->int_gpio);
        gpio_free(data->reset_gpio);
        input_free_device(input_dev);
        kfree(data);
        return -1;
err_free_mem:
        input_free_device(input_dev);
        kfree(data);
        return error;

}

static int __devexit mxt_remove(struct i2c_client *client)
{
	struct mxt_data *data = i2c_get_clientdata(client);

	mxt_unregister_dock_notifier();
	sysfs_remove_group(&client->dev.kobj, &mxt_attr_group);
	free_irq(data->irq, data);
	input_unregister_device(data->input_dev);
	kfree(data->object_table);
	kfree(data);

	return 0;
}

#ifdef CONFIG_PM
static int mxt_suspend(struct device *dev, pm_message_t mesg)
{
	printk("[Atmel] mxt_suspend \n");
	if(touch_chip==NULL) return 0;

	struct i2c_client *client = to_i2c_client(dev);
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = touch_chip;
	struct input_dev *input_dev = data->input_dev;

	struct mxt_finger *finger = data->finger;
	int id;

	if(data->suspend_state == 0){
		if(!data->tp_firmware_upgrade_proceed){
			mutex_lock(&touch_chip->touch_mutex);	
			if(system_owner != SYSTEM_WINDOWS){
				#ifdef CONFIG_HID_ASUS_PAD_EC
	                        u8 WIN8_status = ite8566_switch_notify();
                        	//if(WIN8_status == WIN8_S0 || WIN8_status == WIN8_S1) return;
                	        #endif

        	                mutex_lock(&input_dev->mutex);

	                        if (input_dev->users){
                        	        printk("[Atmel] Disable Touch Chip\n");
                	                mxt_stop(data);
        	                }
	                        mutex_unlock(&input_dev->mutex);
			
	
				if(irq_is_enable == 1){
        	                	printk("[Atmel] Disable Touch Irq\n");
                		        disable_irq(touch_chip->irq);
                	        	irq_is_enable = 0;
		                }
//				data->suspend_state = 1;
			}
			data->suspend_state = 1;
			// Solution of non-release point bug ++
			if(mxt1664t==1){
				for(id=0;id<10;id++){
                                        if (finger[id].status & (MXT_PRESS2 | MXT_MOVE2)) {
                                                printk("mxt_start [%d] released\n", id);
                                                finger[id].status = MXT_RELEASE;
                                                mxt_input_report(data, id);
                                        }
                                }
			}else{
                        	for(id=0;id<10;id++){
                	                if (finger[id].status & (MXT_PRESS | MXT_MOVE)) {
        	                                printk("mxt_start [%d] released\n", id);
	                                        finger[id].status = MXT_RELEASE;
                                        	mxt_input_report(data, id);
                                	}
                        	}
			}
                        // Solution of non-release point bug --	
			mutex_unlock(&touch_chip->touch_mutex);
		}
	}

	return 0;
}

static int mxt_resume(struct device *dev)
{

	printk("[Atmel] mxt_resume \n");
	if(touch_chip==NULL) return 0;
#if 0

	struct i2c_client *client = to_i2c_client(dev);
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = touch_chip;
	struct input_dev *input_dev = data->input_dev;

	struct mxt_finger *finger = data->finger;
        int id;
	char buf[2];
        int error;
	int retry = 2;
	if(data->suspend_state == 1){
		if(!data->tp_firmware_upgrade_proceed){
			/* Soft reset */
			mxt_write_object(data, MXT_GEN_COMMAND_T6,
					MXT_COMMAND_RESET, 1);

			msleep(MXT_RESET_TIME);
			

			// Solution of non-release point bug ++
                        for(id=0;id<10;id++){
                                if (finger[id].status & (MXT_PRESS | MXT_MOVE)) {
                        	        printk("mxt_start [%d] released\n", id);
                                        finger[id].status = MXT_RELEASE;
                                        mxt_input_report(data, id);
                                }
                        }
                        // Solution of non-release point bug --

			// Calibration
                        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 6, 0x05);
                        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 7, 0x50);
                        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 8, 0x06);
                        mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 9, 0xC0);

			if(irq_is_enable==0){
                                printk("[Atmel] Enable Touch Irq\n");
                                enable_irq(touch_chip->irq);
                                irq_is_enable = 1;
                        }
			
			data->suspend_state = 0;
			
			#ifdef CONFIG_HID_ASUS_PAD_EC
                        u8 WIN8_status = ite8566_switch_notify();
                        if(WIN8_status == WIN8_S0 || WIN8_status == WIN8_S1) return;
                        #endif

			mutex_lock(&input_dev->mutex);

			if (input_dev->users){
				printk("[Atmel] Enable Touch Chip\n");
				mxt_start(data); 
			}
			mutex_unlock(&input_dev->mutex);


		}
	}

#endif
	return 0;
}

static const struct dev_pm_ops mxt_pm_ops = {
	.suspend	= mxt_suspend,
	.resume		= mxt_resume,
};

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mxt_early_suspend(struct early_suspend *es)
{
	
	struct mxt_data *mxt;
	if(touch_chip==NULL) return;
	//mxt = container_of(es, struct mxt_data, early_suspend);
	mxt = touch_chip;

	printk("MXT Early Suspend\n");
	if(!mxt) printk("mxt is NULL\n");
	if (mxt_suspend(mxt->client, PMSG_SUSPEND) != 0)
		dev_err(&mxt->client->dev, "%s: failed\n", __func__);

}

static void mxt_late_resume(struct early_suspend *es)
{
	printk("MXT Late Resume\n");
	if(touch_chip==NULL) return;
//	struct i2c_client *client = to_i2c_client(dev);
	struct i2c_client *client = touch_chip->client ;
	//struct mxt_data *data = i2c_get_clientdata(client);
	struct mxt_data *data = touch_chip;
	struct input_dev *input_dev = data->input_dev;

	struct mxt_finger *finger = data->finger;
        int id;
	char buf[2];
        int error;
	int retry = 2;
	if(data->suspend_state == 1){
		if(!data->tp_firmware_upgrade_proceed){
			if(system_owner != SYSTEM_WINDOWS){
				/* 
				 * When soft reseting, the interrupt will be triggered by touch chip.
				 * If owner is windows, we don't do soft reset and enable irq.
				 * This method can avoid the touch crash if we resume Android when onwer is windows.
				 */
				mutex_lock(&touch_chip->touch_mutex);

				/* Soft reset */
				printk("[Atmel] Resume Soft reset\n");
				mxt_write_object(data, MXT_GEN_COMMAND_T6,
						MXT_COMMAND_RESET, 1);

				msleep(MXT_RESET_TIME);

				judge_1664t_addr(); 

				// Calibration
	                        //mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 6, 0x05);
                	        //mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 7, 0x50);
                        	//mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 8, 0x06);
	                        //mxt_write_reg(data->client, MXT_BASE_ADDR(MXT_GEN_ACQUIRE_T8, data) + 9, 0xC0);

				if(irq_is_enable==0){
                	                printk("[Atmel] Enable Touch Irq\n");
                        	        enable_irq(touch_chip->irq);
                                	irq_is_enable = 1;
	                        }
				
//				data->suspend_state = 0;
			
		

				#ifdef CONFIG_HID_ASUS_PAD_EC
	                        u8 WIN8_status = ite8566_switch_notify();
                        	//if(WIN8_status == WIN8_S0 || WIN8_status == WIN8_S1) return;
                	        #endif

				mutex_lock(&input_dev->mutex);

				if (input_dev->users){
					printk("[Atmel] Enable Touch Chip\n");
					mxt_start(data); 
				}
				mutex_unlock(&input_dev->mutex);
			
				mutex_unlock(&touch_chip->touch_mutex);
			}else{
				mutex_lock(&touch_chip->touch_mutex);
				try_enable_touch();
				mutex_unlock(&touch_chip->touch_mutex);
			}
			data->suspend_state = 0;
			printk("[Atmel] set suspend = 0\n");
		}
	}

//	if (mxt_resume(mxt->client) != 0)
//		dev_err(&mxt->client->dev, "%s: failed\n", __func__);

}

#endif
// Jui-Chuan add for early suspend and late resume --


#else
#define mxt_suspend NULL
#define mxt_resume NULL

#endif //#ifdef CONFIG_PM


static const struct i2c_device_id mxt_id[] = {
	{ "mxt1664s", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, mxt_id);

static struct i2c_driver mxt_driver = {
	.driver = {
		.name	= "atmel_mxt_ts",
		.owner	= THIS_MODULE,
// Jui-Chuan mark for early suspend and late resume ++
#ifdef CONFIG_PM
		.pm	= &mxt_pm_ops,
#endif
// Jui-Chuan mark for early suspend and late resume --
	},
	.probe		= mxt_probe,
	.remove		= __devexit_p(mxt_remove),
	.id_table	= mxt_id,
// Jui-Chuan add for early suspend and late resume ++
	#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend = mxt_suspend,
	.resume = mxt_resume,
	#endif
// Jui-Chuan add for early suspend and late resume --
};

module_i2c_driver(mxt_driver);

/* Module information */
MODULE_AUTHOR("Jui-Chuan Chen <jui-chuan_chen@asus.com>");
MODULE_DESCRIPTION("Atmel maXTouch Touchscreen driver for TX201LA");
MODULE_LICENSE("GPL");

