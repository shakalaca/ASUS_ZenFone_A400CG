/* drivers/input/touchscreen/hx8531.c - Himax verions of driver
 *
 * Copyright (C) 2012 Himax Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/module.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/miscdevice.h>
#include <linux/debugfs.h>
#include <linux/irq.h>
#include <linux/syscalls.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/regulator/consumer.h>

#include <linux/fs.h>
#include <asm/segment.h>
#include <asm/uaccess.h>
#include <linux/buffer_head.h>

#include <linux/hx8528.h>		//Joe add
#include <linux/HWVersion.h>
#include <asm/intel_scu_ipc.h>
#include <linux/mfd/intel_msic.h>

#define ASUS_DRIVER_VERSION "3.1.0"		//josh modify (firmware ver.config ver.driver ver)
//#define HIMAX_DRIVER_VERSION "C007"		//Bizzy added
#define HX_TP_SYS_FLASH_DUMP								// Support Sys : Flash dump function	,default is open
#define FLASH_DUMP_FILE "/sdcard/Flash_Dump.bin"

//////////////////////////////////////////////////////
/* Configurable Parameters
 */
static char HIMAX_DRIVER_VERSION[] = "C007";
static u8 r58_ER[] = { 0x58, 0x04, 0x28, 0x88, 0x50 };
static u8 rC5_wintek[] = { 0xC5, 0x0A, 0x1A, 0x08, 0x10, 0x19, 0x1F, 0x0B };
static u8 rC6_wintek[] = { 0xC6, 0x12, 0x10, 0x1C };

//Himax: IC status check
uint8_t IC_STATUS_CHECK = 0xAA;
uint8_t r8D_setting[2] = { 0x03, 0x20};
u16 chip_hw_checksum;
u16 def_checksum = 50474;
u16 touch_count = 0;
u16 reset_count = 0;
//Himax: Change Iref
//#define ChangeIref1u 0

//Himax: Coor. mapping 20121220
//#define Coor_Mapping 0
#ifdef Coor_Mapping
#define Real_X_Start 	5 
#define Real_X_End		795
#define Real_Y_Start	5
#define Real_Y_End		1275
#endif
 
//Himax: ESD 
//#define ESD_DEBUG 1
#define ESD_WORKAROUND 1
#ifdef ESD_WORKAROUND
static u8 reset_activate = 1;
u8 ESD_COUNTER = 0;
int ESD_COUNTER_SETTING = 3;
#endif
 
  //Himax: Set FW and CFG Flash Address 
#define FW_VER_MAJ_FLASH_ADDR        33        //0x0085 
#define FW_VER_MAJ_FLASH_LENG        1 
#define FW_VER_MIN_FLASH_ADDR        182        //0x02D8 
#define FW_VER_MIN_FLASH_LENG        1 
#define CFG_VER_MAJ_FLASH_ADDR       173        //0x02B4 
//#define CFG_VER_MAJ_FLASH_LENG       3 
//#define CFG_VER_MIN_FLASH_ADDR       176        //0x02C0 
#define CFG_VER_MIN_FLASH_LENG       3 

#define DEFAULT_RETRY_CNT            3

  // Himax Feature Support

#define HX_TP_FW_UPDATE
#define HX_TP_MAX_FINGER             10
  // Touch informtion data index
#define HX_TOUCH_INFO_SIZE           56
#define HX_TOUCH_INFO_VKEY           22
#define HX_TOUCH_INFO_ID_1_INFO      53
#define HX_TOUCH_INFO_ID_2_INFO      54
#define HX_TOUCH_INFO_POINT_CNT      52
  // Screen Resolution
#define DEFAUULT_X_RES               800   /* face the TS, x-axis */
#define DEFAUULT_Y_RES               1280  /* face the TS, y-axis */
  // SysFS node
#define HX_TP_SYS_FS
#define RAW_DATA_LENGTH_PER_PKT      (128 - HX_TOUCH_INFO_SIZE)
#define DEFAULT_X_CHANNEL            21 /* ME372CG face the TS, x-axis */
#define DEFAULT_Y_CHANNEL            34 /* ME372CG face the TS, y-axis */
#define DEFAULT_SELF_CHANNEL         55
  //interrupt gpio
#define HIMAX_TPID_GPIO				174  //add by Josh 


/* Himax TP COMMANDS -> Do not modify the below definition
*/
#define HX_CMD_NOP                   0x00   /* no operation */
#define HX_CMD_SETMICROOFF           0x35   /* set micro on */
#define HX_CMD_SETROMRDY             0x36   /* set flash ready */
#define HX_CMD_TSSLPIN               0x80   /* set sleep in */
#define HX_CMD_TSSLPOUT              0x81   /* set sleep out */
#define HX_CMD_TSSOFF                0x82   /* sense off */
#define HX_CMD_TSSON                 0x83   /* sense on */
#define HX_CMD_ROE                   0x85   /* read one event */
#define HX_CMD_RAE                   0x86   /* read all events */
#define HX_CMD_RLE                   0x87   /* read latest event */
#define HX_CMD_CLRES                 0x88   /* clear event stack */
#define HX_CMD_TSSWRESET             0x9E   /* TS software reset */
#define HX_CMD_SETDEEPSTB            0xD7   /* set deep sleep mode */
#define HX_CMD_SET_CACHE_FUN         0xDD   /* set cache function */
#define HX_CMD_SETIDLE               0xF2   /* set idle mode */
#define HX_CMD_SETIDLEDELAY          0xF3   /* set idle delay */

#define HX_CMD_FW_VERSION_ID         0x32   /* firmware version ID */
#define HX_CMD_SELFTEST_BUFFER       0x8D	/* self-test return buffer */

#define HX_CMD_MANUALMODE            0x42
#define HX_CMD_FLASH_ENABLE          0x43
#define HX_CMD_FLASH_SET_ADDRESS     0x44
#define HX_CMD_FLASH_WRITE_REGISTER  0x45
#define HX_CMD_FLASH_R               0x46
#define HX_CMD_FLASH_SET_COMMAND     0x47
#define HX_CMD_FLASH_WRITE_BUFFER    0x48
#define HX_CMD_FLASH_PAGE_ERASE      0x4D
#define HX_CMD_FLASH_SECTOR_ERASE    0x4E
#define HX_CMD_FLASH_BUFFER          0x59

#define HX_CMD_CB                    0xCB
#define HX_CMD_EA                    0xEA
#define HX_CMD_4A                    0x4A
#define HX_CMD_4F                    0x4F

////////////////////////////////////////////
#define FINGER_NUM                   10
#define IDX_FINGER                   3

//#define ENABLE_SELF_FIRMWARE_UPGRADE
#define ENABLE_CHIP_RESET_MACHINE
#define ENABLE_CHIP_STATUS_MONITOR
#define ENABLE_TEST

static int cable_status = -1; //josh add for USB time issue (init : -1) 2013/06/07

struct himax_ts_data {
    struct i2c_client *client;
    struct input_dev *input_dev;
    struct workqueue_struct *himax_wq;
    struct work_struct work;
//    struct mutex init_lock;
    struct mutex mutex_lock;
//    struct wake_lock wake_lock_init;
    struct wake_lock wake_lock;
//    struct wake_lock wake_lock_fw;
    struct early_suspend early_suspend;
    struct switch_dev touch_sdev;
    //----[HX_TP_SYS_FLASH_DUMP]--------------------------------------------------------------------------start
	#ifdef HX_TP_SYS_FLASH_DUMP
	struct workqueue_struct 			*flash_wq;
	struct work_struct 				flash_work;
	#endif
	//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------end
		
#ifdef ENABLE_CHIP_RESET_MACHINE
    struct delayed_work himax_chip_reset_work;
#endif
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	struct delayed_work himax_chip_firmware_upgrade_work;
#endif
#ifdef ENABLE_CHIP_STATUS_MONITOR
	struct delayed_work himax_chip_monitor;
#endif
    int (*power)(int on);
    struct attribute_group attrs;
    int status;
    int abs_x_max;
    int abs_y_max;
    int rst_gpio;
    int intr_gpio;
	int tpid_gpio; // add by josh
    int irq;
    int suspend_state;
    int tp_status;
    int tp_firmware_upgrade_proceed;
    char touch_phys_name[32];
    char himax_version[32];
    int version_checksum;
    int formal_lens;		//0xAA is JTouch lens, 0x55 is wintek lens, 0x5A is Nexus 7 lens.
    int old_tp_version;
    int init_success;
    int retry_time;
	// +++++++ add by Josh for AP update touch fw ++++++
	int AP_update; 
	int AP_progress; 
	// ------ add by Josh for AP update touch fw ------
	int proximity_status; //josh add for USB time issue (init : 1 , don't report = 1 , report = 0) 2013/07/18 
#ifdef ENABLE_CHIP_STATUS_MONITOR
    int running_status;
#endif
};
static struct himax_ts_data *himax_chip;

//static struct himax_ts_data *private_ts = NULL;
static struct semaphore pSem;

struct i2c_client *touch_i2c = NULL;
static int himax_debug_flag = 0;

#ifdef	CONFIG_PROC_FS
#define	HIMAX_PROC_FILE	"himax"
static struct proc_dir_entry *himax_proc_file;
#define	HIMAX_PROC_DIAG_FILE	"himax_diag"
static struct proc_dir_entry *himax_proc_diag_file;
#endif

// add by leo for loading config files from .txt by proc ++
#ifdef ENABLE_TEST
#define himax_proc_config_len 129
#define	HIMAX_PROC_CONFIG_FILE	"himax_config"
static struct proc_dir_entry *himax_proc_config_file;

// add by Josh for Read/Write register by proc 2013/06/04 ++
#define	HIMAX_PROC_REGISTER	"himax_register"
static struct proc_dir_entry *himax_proc_register;
// add by Josh for Read/Write register by proc 2013/06/04 ++
//add by Josh for self_test  poweron and fw_upgrade by proc 2013/07/15 ++
#define	HIMAX_PROC_DEBUG_FLAG	"touch_debug_log"
static struct proc_dir_entry *himax_proc_debug_flag;

#define	HIMAX_PROC_SELF_TEST "himax_self_test"
static struct proc_dir_entry *himax_proc_self_test;

// cat : poweron  echo : fw_upgrade
#define HIMAX_PROC_POWERON_AND_FW_UPGRADE "himax_poweron_and_fw_upgrade"
static struct proc_dir_entry *himax_proc_poweron_and_fw_upgrade;
//add by Josh for self_test  poweron and fw_upgradeby proc 2013/07/15 --
#endif
// add by leo for loading config files from .txt by proc --

//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------start	
		#ifdef HX_TP_SYS_FLASH_DUMP
		static uint8_t *flash_buffer 				= NULL;
		static uint8_t flash_command 				= 0;
		static uint8_t flash_read_step 			= 0;
		static uint8_t flash_progress 			= 0;
		static uint8_t flash_dump_complete	= 0;
		static uint8_t flash_dump_fail 			= 0;
		static uint8_t sys_operation				= 0;
		static uint8_t flash_dump_sector	 	= 0;
		static uint8_t flash_dump_page 			= 0;
		static bool    flash_dump_going			= false;			
	
		static uint8_t getFlashCommand(void);
		static uint8_t getFlashDumpComplete(void);
		static uint8_t getFlashDumpFail(void);
		static uint8_t getFlashDumpProgress(void);
		static uint8_t getFlashReadStep(void);
		static uint8_t getSysOperation(void);
		static uint8_t getFlashDumpSector(void);
		static uint8_t getFlashDumpPage(void);
		static bool	   getFlashDumpGoing(void);
		
		static void setFlashBuffer(void);
		static void setFlashCommand(uint8_t command);
		static void setFlashReadStep(uint8_t step);
		static void setFlashDumpComplete(uint8_t complete);
		static void setFlashDumpFail(uint8_t fail);
		static void setFlashDumpProgress(uint8_t progress);
		static void setSysOperation(uint8_t operation);
		static void setFlashDumpSector(uint8_t sector);
		static void setFlashDumpPage(uint8_t page);
		static void setFlashDumpGoing(bool going);
		#endif
	//----[HX_TP_SYS_FLASH_DUMP]------------------------------------------------------------------------------end

#ifdef HX_TP_SYS_FS
static uint8_t *getMutualBuffer(void);
static void setMutualBuffer(void);
static uint8_t *getSelfBuffer(void);
static uint8_t getDebugLevel(void);
static uint8_t getDiagCommand(void);
static uint8_t getXChannel(void);
static uint8_t getYChannel(void);
static void setXChannel(uint8_t x);
static void setYChannel(uint8_t y);
static int himax_touch_sysfs_init(void);
static void himax_touch_sysfs_deinit(void);
#endif
static int himax_chip_self_test(uint8_t *data);
static int himax_ts_suspend_command(struct i2c_client *client);
static int himax_ts_poweron(struct himax_ts_data *ts_modify);
static void himax_ts_loadconfig(struct himax_ts_data *ts_modify);
static void himax_ts_tpk_config(struct himax_ts_data *ts_modify); //add by josh
static int himax_hang_shaking(void);
static int himax_AP_firmware_upgrade();
static int himax_AP_firmware_check();
#ifdef ESD_WORKAROUND
void ESD_HW_REST(void);
#endif

static ssize_t himax_debug_level_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_chip_firmware_upgrade(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t himax_diag_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_diag_dump(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t himax_register_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_register_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t touch_vendor_show(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_get_fw_version(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_get_touch_status(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_chip_raw_data_store(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_chip_enable_irq(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_chip_poweron(struct device *dev, struct device_attribute *attr, char *buf);
static ssize_t himax_chip_check_running(struct device *dev, struct device_attribute *attr, char *buf);

static DEVICE_ATTR(debug_level, (S_IWUSR|S_IRUGO), himax_debug_level_show, himax_chip_firmware_upgrade);
static DEVICE_ATTR(diag, (S_IWUSR|S_IRUGO), himax_diag_show, himax_diag_dump);
static DEVICE_ATTR(register, (S_IWUSR|S_IRUGO), himax_register_show, himax_register_store);
static DEVICE_ATTR(vendor, 0444, touch_vendor_show, NULL);
static DEVICE_ATTR(tp_fw_upgrade, (S_IWUSR|S_IRUGO), himax_debug_level_show, himax_chip_firmware_upgrade);
static DEVICE_ATTR(tp_fw_version,(S_IWUSR|S_IRUGO), himax_get_fw_version, NULL);
static DEVICE_ATTR(tp_self_test, (S_IWUSR|S_IRUGO), himax_chip_self_test_function, NULL);
static DEVICE_ATTR(touch_status, (S_IWUSR|S_IRUGO), himax_get_touch_status, NULL);
static DEVICE_ATTR(tp_output_raw_data, (S_IWUSR|S_IRUGO), himax_chip_raw_data_store, himax_diag_dump);
static DEVICE_ATTR(touch_irq, (S_IWUSR|S_IRUGO), himax_chip_enable_irq, NULL);
static DEVICE_ATTR(touch_poweron, (S_IWUSR|S_IRUGO), himax_chip_poweron, NULL);
static DEVICE_ATTR(tp_check_running, (S_IWUSR|S_IRUGO), himax_chip_check_running, NULL);

extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
extern unsigned int entry_mode;
extern unsigned int factory_mode;
//extern int check_cable_status(void);


static struct attribute *himax_attr[] = {
	&dev_attr_register.attr,
	&dev_attr_vendor.attr,
	&dev_attr_debug_level.attr,
	&dev_attr_tp_fw_upgrade.attr,
	&dev_attr_diag.attr,
	&dev_attr_tp_fw_version.attr,
	&dev_attr_tp_self_test.attr,
	&dev_attr_touch_status.attr,
	&dev_attr_tp_output_raw_data.attr,
	&dev_attr_touch_irq.attr,
	&dev_attr_touch_poweron.attr,
	&dev_attr_tp_check_running.attr,
	NULL
};

#define PLAIN_I2C
#ifdef PLAIN_I2C
int i2c_himax_read(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry;
    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = 1,
            .buf = &command,
        },
        {
            .addr = client->addr,
            .flags = I2C_M_RD,
            .len = length,
            .buf = data,
        }
    };

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, msg, 2) == 2)
            break;
        msleep(10);
    }
    if (retry == toRetry) {
        printk(KERN_INFO "[Himax] %s: i2c_read_block retry over %d\n", __func__, 
            toRetry);
        return -EIO;
    }
    return 0;

}


int i2c_himax_write(struct i2c_client *client, uint8_t command, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry, loop_i;
    uint8_t *buf = kzalloc(sizeof(uint8_t)*(length+1), GFP_KERNEL);

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length + 1,
            .buf = buf,
        }
    };

    buf[0] = command;
    for (loop_i = 0; loop_i < length; loop_i++)
        buf[loop_i + 1] = data[loop_i];

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }

    if (retry == toRetry) {
        printk(KERN_ERR "[Himax] %s: i2c_write_block retry over %d\n", __func__, 
            toRetry);
        kfree(buf);
        return -EIO;
    }
    kfree(buf);
    return 0;

}

int i2c_himax_write_command(struct i2c_client *client, uint8_t command, uint8_t toRetry)
{
    return i2c_himax_write(client, command, NULL, 0, toRetry);
}

int i2c_himax_master_write(struct i2c_client *client, uint8_t *data, uint8_t length, uint8_t toRetry)
{
    int retry, loop_i;
    uint8_t *buf = kzalloc(sizeof(uint8_t)*length, GFP_KERNEL);

    struct i2c_msg msg[] = {
        {
            .addr = client->addr,
            .flags = 0,
            .len = length,
            .buf = buf,
        }
    };

    for (loop_i = 0; loop_i < length; loop_i++)
        buf[loop_i] = data[loop_i];

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }

    if (retry == toRetry) {
        printk(KERN_ERR "[Himax] %s: i2c_write_block retry over %d\n", __func__, 
               toRetry);
        kfree(buf);
        return -EIO;
    }
    kfree(buf);
    return 0;
}

int i2c_himax_read_command(struct i2c_client *client, uint8_t length, uint8_t *data, uint8_t *readlength, uint8_t toRetry)
{
    int retry;
    struct i2c_msg msg[] = {
        {
        .addr = client->addr,
        .flags = I2C_M_RD,
        .len = length,
        .buf = data,
        }
    };

    for (retry = 0; retry < toRetry; retry++) {
        if (i2c_transfer(client->adapter, msg, 1) == 1)
            break;
        msleep(10);
    }
    if (retry == toRetry) {
        printk(KERN_INFO "[Himax] %s: i2c_read_block retry over %d\n", __func__, 
               toRetry);
        return -EIO;
    }
    return 0;
}
#endif


/*************IO control setting***************/
static int touch_open(struct inode *inode, struct file *file)
{
	printk( "[touch_update]:%s ++ \n",__func__);
	return nonseekable_open(inode, file);		
}

static int touch_release(struct inode *inode, struct file *file)
{
	printk( "[touch_update]:%s ++ \n",__func__);
	return 0;
}

static long touch_i2c_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err = 0, val = 0,result;
	
	printk( "[touch_update]:%s ++ \n",__func__);
	
	if (_IOC_TYPE(cmd) != TOUCH_IOC_MAGIC) 
	{
		return -ENOTTY;
	}
	if (_IOC_DIR(cmd) & _IOC_READ)
		err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
		err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) 
	{
		return -EFAULT;
	}
	
	switch (cmd) {
	case TOUCH_INIT:
	case TOUCH_FW_UPDATE_FLAG:
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val){
			printk( "[touch_update]: TOUCH_FW_UPDATE - TOUCH_FW_UPGRADE_ON \n");
			himax_chip->AP_update = 1;
			himax_chip->AP_progress = 0;
		}else{
			//printk( "[touch_update]:TOUCH_FW_UPDATE - TOUCH_FW_UPGRADE_OFF \n");
			himax_chip->AP_update = 0;
		}
		break;
		
	case TOUCH_FW_UPDATE_PROCESS:
		printk( "[touch_update]:%s: TOUCH_FW_UPDATE_PROCESS \n",__func__);
		himax_AP_firmware_upgrade();
		break;
	case TOUCH_TP_FW_check:
		printk( "[touch_update]:%s: TOUCH_FW_UPDATE_CHECK \n",__func__);
		result = himax_AP_firmware_check();
		printk( "[touch_update]:%s: TOUCH_FW_UPDATE_CHECK = % d\n",__func__,result);
		return result;
	default:
		printk( "[Himax]:%s: incorrect cmd (%d) \n",__FUNCTION__, _IOC_NR(cmd));
		return -EINVAL;
	}
	
	return 0;
	
}

static struct file_operations ite_fops = {
	.owner = THIS_MODULE,
	.open = touch_open,
	.release = touch_release,
	.unlocked_ioctl = touch_i2c_ioctl
};

static struct miscdevice ite_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "hx8528",
	.fops = &ite_fops,
};
/*************IO control setting***************/





#ifdef HX_TP_FW_UPDATE
static unsigned char isTP_Updated = 0;
static unsigned char CTPM_FW[]=
{
//    #include "MCUon_HX8523-A-Checked_20121018.i" //Paul Check
};

unsigned char SFR_1u_1[16][2] = {{0x18,0x07},{0x18,0x17},{0x18,0x27},{0x18,0x37},{0x18,0x47},
			{0x18,0x57},{0x18,0x67},{0x18,0x77},{0x18,0x87},{0x18,0x97},
			{0x18,0xA7},{0x18,0xB7},{0x18,0xC7},{0x18,0xD7},{0x18,0xE7},
			{0x18,0xF7}};

unsigned char SFR_2u_1[16][2] = {{0x98,0x06},{0x98,0x16},{0x98,0x26},{0x98,0x36},{0x98,0x46},
			{0x98,0x56},{0x98,0x66},{0x98,0x76},{0x98,0x86},{0x98,0x96},
			{0x98,0xA6},{0x98,0xB6},{0x98,0xC6},{0x98,0xD6},{0x98,0xE6},
			{0x98,0xF6}};

unsigned char SFR_3u_1[16][2] = {{0x18,0x06},{0x18,0x16},{0x18,0x26},{0x18,0x36},{0x18,0x46},
			{0x18,0x56},{0x18,0x66},{0x18,0x76},{0x18,0x86},{0x18,0x96},
			{0x18,0xA6},{0x18,0xB6},{0x18,0xC6},{0x18,0xD6},{0x18,0xE6},
			{0x18,0xF6}};

unsigned char SFR_4u_1[16][2] = {{0x98,0x05},{0x98,0x15},{0x98,0x25},{0x98,0x35},{0x98,0x45},
			{0x98,0x55},{0x98,0x65},{0x98,0x75},{0x98,0x85},{0x98,0x95},
			{0x98,0xA5},{0x98,0xB5},{0x98,0xC5},{0x98,0xD5},{0x98,0xE5},
			{0x98,0xF5}};

unsigned char SFR_5u_1[16][2] = {{0x18,0x05},{0x18,0x15},{0x18,0x25},{0x18,0x35},{0x18,0x45},
			{0x18,0x55},{0x18,0x65},{0x18,0x75},{0x18,0x85},{0x18,0x95},
			{0x18,0xA5},{0x18,0xB5},{0x18,0xC5},{0x18,0xD5},{0x18,0xE5},
			{0x18,0xF5}};

unsigned char SFR_6u_1[16][2] = {{0x98,0x04},{0x98,0x14},{0x98,0x24},{0x98,0x34},{0x98,0x44},
			{0x98,0x54},{0x98,0x64},{0x98,0x74},{0x98,0x84},{0x98,0x94},
			{0x98,0xA4},{0x98,0xB4},{0x98,0xC4},{0x98,0xD4},{0x98,0xE4},
			{0x98,0xF4}};

unsigned char SFR_7u_1[16][2] = {{0x18,0x04},{0x18,0x14},{0x18,0x24},{0x18,0x34},{0x18,0x44},
			{0x18,0x54},{0x18,0x64},{0x18,0x74},{0x18,0x84},{0x18,0x94},
			{0x18,0xA4},{0x18,0xB4},{0x18,0xC4},{0x18,0xD4},{0x18,0xE4},
			{0x18,0xF4}};

int himax_ManualMode(int enter)
{
    unsigned char cmd[2];
    cmd[0] = enter;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x42, 1, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
}

int himax_FlashMode(int enter)
{
    unsigned char cmd[2];
    cmd[0] = enter;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 1, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    return 0;
}

int himax_lock_flash(void)
{
    unsigned char cmd[5];
    
    /* lock sequence start */
    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x7D;cmd[3] = 0x03;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(50);
    return 0;
    /* lock sequence stop */
}

int himax_unlock_flash(void)
{
    unsigned char cmd[5];
    
    /* unlock sequence start */
    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x03;cmd[1] = 0x00;cmd[2] = 0x00;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }

    cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x3D;cmd[3] = 0x03;
    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }

    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0])< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(50);

    return 0;
    /* unlock sequence stop */
}
// ++++++add by Josh for progress+++++++
static int update_touch_progress(int update_progress){

	if(himax_chip->AP_progress < update_progress || update_progress == 0){
	himax_chip->AP_progress = update_progress;
		mm_segment_t oldfs;
		char Progress_file_path[] = "/data/touch_update_progress";
		struct file *filePtr = NULL;
		int len = 0;
		loff_t pos = 0;
		char temp_progress[3];
		
		sprintf(temp_progress, "%d", update_progress);
		//printk("[touch_fw_update] %s: write %d done. \n", __func__, update_progress);
		filePtr = filp_open(Progress_file_path, O_RDWR|O_CREAT,S_IRUSR|S_IWUSR );
		if(!IS_ERR_OR_NULL(filePtr)) {
			oldfs = get_fs();
			set_fs(get_ds());
			pos = 0;
			len = filePtr->f_op->write(filePtr, &temp_progress, sizeof(temp_progress), &pos);
			set_fs(oldfs);
			filp_close(filePtr, NULL);
			//printk("[touch_fw_update] %s: write %s done. \n", __func__, Progress_file_path);
			return 0;
		}else if(PTR_ERR(filePtr) == -ENOENT) {
			printk("[touch_fw_update] %s: %s not found\n", __func__, Progress_file_path);
			return 1;
		} else {
			printk("[touch_fw_update] %s: %s open error\n", __func__, Progress_file_path);
			return 1;
		}
	}
}
// ------add by Josh for progress------

// ++++++add by Josh for result+++++++
static int update_touch_result(int result){

	mm_segment_t oldfs_result;
    char result_state_path[] = "/data/touch_upfw_result";
    struct file *resultfilePtr = NULL;
	loff_t pos = 0;
	int len = 0;
	
	resultfilePtr = filp_open(result_state_path, O_RDWR|O_CREAT,S_IRUSR|S_IWUSR );
	if(!IS_ERR_OR_NULL(resultfilePtr)) {
		oldfs_result = get_fs();
		set_fs(get_ds());
		pos = 0;
		len = resultfilePtr->f_op->write(resultfilePtr, result, sizeof(char), &pos);
		set_fs(oldfs_result);
		filp_close(resultfilePtr, NULL);
		//printk("[touch_fw_update] %s: write %s done. \n", __func__, result_state_path);
		return 0;
	}else if(PTR_ERR(resultfilePtr) == -ENOENT) {
		printk("[touch_fw_update] %s: %s not found\n", __func__, result_state_path);
		return 1;
	} else {
		printk("[touch_fw_update] %s: %s open error\n", __func__, result_state_path);
		return 1;
	}
}
// ------add by Josh for result------		
#if ChangeIref1u
int ChangeIrefSPP(void)
{
    unsigned char i;
    unsigned char cmd[5];

    unsigned char spp_source[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,		//SPP
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};		//SPP
    
    unsigned char spp_target[16] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,		//SPP
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};		//SPP
    unsigned char retry;
    unsigned char spp_ok;
    
    //--------------------------------------------------------------------------
    //Inital
    //--------------------------------------------------------------------------
    cmd[0] = 0x02;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x42, 1, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    udelay(10);

    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(160);
    
    //--------------------------------------------------------------------------
    //read 16-byte SPP to spp_source
    //--------------------------------------------------------------------------
    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x1A;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    udelay(10);
    
    //4 words
    for (i = 0; i < 4; i++)
    {
    	cmd[0] = i;cmd[1] = 0x00;cmd[2] = 0x00;
	    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    }
	    udelay(10);
	    
	    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0, &cmd[0]))< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    }
	    udelay(10);
	    
	    if((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    	}
	    mdelay(5);				//check this
	    
	    printk(KERN_ERR "[Himax]:read 4 words SPP:%x,%x,%x,%x .\n",cmd[0],cmd[1],cmd[2],cmd[3]);
	    
	    //save data
	    spp_source[4*i + 0] = cmd[0];
	    spp_source[4*i + 1] = cmd[1];
	    spp_source[4*i + 2] = cmd[2];
	    spp_source[4*i + 3] = cmd[3]; 
    }
    
    //--------------------------------------------------------------------------
    //Search 3u Iref
    //--------------------------------------------------------------------------
    for (i = 0; i < 16; i++)
    {
	    if(spp_source[0]==SFR_1u_1[i][0] && spp_source[1]==SFR_1u_1[i][1])
	    {
	    	//found in 1uA
	    	printk(KERN_ERR "[Himax] %s: Find the 1u setting\n", __func__);
	    	return (1);				//OK
	    }
    }
    
    spp_ok = 0;
    for (i = 0; i < 16; i++)
    {
	    if(spp_source[0]==SFR_3u_1[i][0] && spp_source[1]==SFR_3u_1[i][1])
	    {
		    //found in 3uA
		    spp_ok = 1;
		    
		    spp_source[0]= SFR_1u_1[i][0];
		    spp_source[1]= SFR_1u_1[i][1];
		    break;
	    }
    }
    
    if (spp_ok == 0)
    {
    	//no matched pair in SFR_1u_1 or SFR_3u_1
    	printk(KERN_ERR "[Himax] %s: Warning!! No Iref 3u Setting\n", __func__);
    	return 0;
    }
    
    //--------------------------------------------------------------------------
    //write SPP (retry for 3 times if errors occur)
    //--------------------------------------------------------------------------
    for (retry = 0; retry < 3; retry++)
    {
	    himax_unlock_flash();
	    
	    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x1A;
	    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    }
	    udelay(10);
	    
	    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0]))< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    }
	    udelay(10);
	    
	    //write 16-byte SPP
	    for (i = 0; i < 4; i++)
	    {
	    	cmd[0] = i;cmd[1] = 0x00;cmd[2] = 0x00;
		    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
		    
		    cmd[0] = spp_source[4 * i + 0];
		    cmd[1] = spp_source[4 * i + 1];
		    cmd[2] = spp_source[4 * i + 2];
		    cmd[3] = spp_source[4 * i + 3];
		    
		    printk(KERN_ERR "[Himax]:write 4 words SPP:%x,%x,%x,%x .\n",cmd[0],cmd[1],cmd[2],cmd[3]);
		    
		    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0]))< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
		    
		    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4B, 0, &cmd[0]))< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
	    }
	    
		  if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4C, 0, &cmd[0]))< 0){
		      printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		      return 0;
		  }
	    mdelay(10);
	    
	    //read 16-byte SPP to spp_target
	    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x1A;
	    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    }
	    udelay(10);
	    
	    //read 16-byte SPP
	    mdelay(20);
	    for (i = 0; i < 4; i++)
	    {
	    	cmd[0] = i;cmd[1] = 0x00;cmd[2] = 0x00;
		    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
		    
		    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0, &cmd[0]))< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
		    
		    if((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4, &cmd[0]))< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    	}
		    mdelay(5);		//check this
		    
			printk(KERN_ERR "[Himax]:read 16-byte SPP:%x,%x,%x,%x .\n",cmd[0],cmd[1],cmd[2],cmd[3]);
		    
		    spp_target[4*i + 0] = cmd[0];
		    spp_target[4*i + 1] = cmd[1];
		    spp_target[4*i + 2] = cmd[2];
		    spp_target[4*i + 3] = cmd[3];
	    }
	    
	    //compare source and target
	    spp_ok = 1;
	    for (i = 0; i < 16; i++)
	    {
		    if (spp_target[i] != spp_source[i])
		    	spp_ok = 0;
	    }
	    
	    if (spp_ok == 1)
	    	return 1;	//Modify Success
	    
	    //reset SFR
	    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
	    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0])< 0){
	        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
	        return 0;
	    }
	    udelay(10);
	    
	    //write 16-byte SFR
	    for (i = 0; i < 4; i++)
	    {
	    	cmd[0] = i;cmd[1] = 0x00;cmd[2] = 0x00;
		    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0])< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
		    
		    cmd[0] = spp_source[4 * i + 0];
		    cmd[1] = spp_source[4 * i + 1];
		    cmd[2] = spp_source[4 * i + 2];
		    cmd[3] = spp_source[4 * i + 3];
		    if(i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0])< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }
		    udelay(10);
		    
		    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0]))< 0){
		        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
		        return 0;
		    }

		    udelay(10);
	    }
    }	//retry
    
    return 0;			//No 3u Iref setting
}
#endif

int himax_modifyIref(void)
{
    //int readLen;
    unsigned char i, result;
    unsigned char cmd[5];
    unsigned char Iref[2] = {0x00,0x00};
    
    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x08;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x00;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    if((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
             
    mdelay(5);
    
    //Find 1u Iref
    result = 0;
    printk(KERN_ERR "[Himax] %s: Iref0 = %x, Iref1 = %x!\n", __func__, cmd[0], cmd[1]);
    for(i=0;i<16;i++)
    {
        if((cmd[0]==SFR_1u_1[i][0])&&(cmd[1]==SFR_1u_1[i][1]))
        {
            Iref[0]= SFR_4u_1[i][0];
            Iref[1]= SFR_4u_1[i][1];
            
            result = 1;
            i = 16;
        }
        else
        	result = 0; 
    }
    
    if(result == 0)
    {
    	printk(KERN_ERR "[Himax] %s: No find the 1u Iref!\n", __func__);
    	return 0;
    }
    
    cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x06;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    cmd[0] = 0x00;cmd[1] = 0x00;cmd[2] = 0x00;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    cmd[0] = Iref[0];cmd[1] = Iref[1];cmd[2] = 0x27;cmd[3] = 0x27;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4A, 0, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
        
    return 1;    
}

u8 himax_read_FW_ver(void)
{
    const u16 FLASH_VER_START_ADDR = 1030;
    const u16 FLASH_VER_END_ADDR = 1033;
    const u16 FW_VER_END_ADDR = 4120;
    u16 i;
    u16 j = FW_VER_END_ADDR;
    unsigned char cmd[3];
    //int firmware_len = sizeof(CTPM_FW);

    //if(firmware_len < )
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(120);
    
    himax_FlashMode(1);    

    for (i = FLASH_VER_START_ADDR; i < FLASH_VER_END_ADDR; i++)
    {
        cmd[0] = i & 0x1F;
        cmd[1] = (i >> 5) & 0x1F;
        cmd[2] = (i >> 10) & 0x1F;
        
        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }
        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }
        if((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }

        /* TODO: check fw length*/
        if (CTPM_FW[j] != cmd[0]) return 0; j++;
        printk("Himax_marked TP version check, CTPW[%d]:%d, cmd[0]:%d\n", j, CTPM_FW[j], cmd[0]);
        if (CTPM_FW[j] != cmd[1]) return 0; j++;
        printk("Himax_marked TP version check, CTPW[%d]:%d, cmd[0]:%d\n", j, CTPM_FW[j], cmd[1]);
        if (CTPM_FW[j] != cmd[2]) return 0; j++;
        printk("Himax_marked TP version check, CTPW[%d]:%d, cmd[0]:%d\n", j, CTPM_FW[j], cmd[2]);
        if (CTPM_FW[j] != cmd[3]) return 0; j++;
        printk("Himax_marked TP version check, CTPW[%d]:%d, cmd[0]:%d\n", j, CTPM_FW[j], cmd[3]);
    }
    
    himax_FlashMode(0);    

    return 1;
}

static uint8_t himax_hw_checksum(u16 sw_checksum)
{
		unsigned char cmd[5];
		u16 hw_checkusm;
	
		cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x02;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    
    cmd[0] = 0x01;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0xE5, 1, &cmd[0]))< 0){
        printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
        return 0;
    }
    mdelay(10);
    
    if((i2c_smbus_read_i2c_block_data(touch_i2c, 0xAD, 4, &cmd[0]))< 0){
    	printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
      return -1;
    }
    
    printk("[Himax] %s: Himax HW Checksum : %x, %x, %x, %x\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
    
	
    hw_checkusm = cmd[0] + cmd[1]*0x100 + cmd[2]*0x10000 + cmd[3]*1000000;
    printk("[Himax] %s: hw_checkusm = %d, sw_checksum = %d\n", __func__, hw_checkusm, sw_checksum);
    chip_hw_checksum = hw_checkusm;
	if(hw_checkusm == sw_checksum)
    {
    	return 1;
    }
    else
    {
    	return 0;
    }
}

static uint8_t himax_calculateChecksum(char *ImageBuffer, int fullLength)//, int address, int RST)
{
    u16 checksum = 0;
    unsigned char cmd[5], last_byte;
    int FileLength, i, readLen, k, lastLength;

    FileLength = fullLength;
    memset(cmd, 0x00, sizeof(cmd));
	if(himax_chip->AP_update == 1){
		update_touch_progress(70);
	}
    //himax_HW_reset(RST);

    //if((i2c_smbus_write_i2c_block_data(i2c_client, 0x81, 0, &cmd[0]))< 0)
        //return 0;
         
    //msleep(120);
    //printk("himax_marked, Sleep out: %d\n", __LINE__);
    //himax_unlock_flash();
    
    //Bizzy added for Iref
    //if(himax_modifyIref() == 0)
    //    return 0;
    
    himax_FlashMode(1);

    FileLength = (FileLength + 3) / 4;
    for (i = 0; i < FileLength; i++) 
    {
        last_byte = 0;
        readLen = 0;

        cmd[0] = i & 0x1F;
        if (cmd[0] == 0x1F || i == FileLength - 1)
            last_byte = 1;
        cmd[1] = (i >> 5) & 0x1F;cmd[2] = (i >> 10) & 0x1F;
        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x46, 0, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_read_i2c_block_data(touch_i2c, 0x59, 4, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return -1;
        }
		

		
        if (i < (FileLength - 1))
        {
            checksum += cmd[0] + cmd[1] + cmd[2] + cmd[3];
            if (i == 0)
                printk("[Himax] %s: himax_marked cmd 0 to 3 (first 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
        }
        else 
        {
            printk("[Himax] %s: himax_marked cmd 0 to 3 (last 4 bytes): %d, %d, %d, %d\n", __func__, cmd[0], cmd[1], cmd[2], cmd[3]);
            printk("[Himax] %s: himax_marked, checksum (not last): %d\n", __func__, checksum);

            //lastLength = (((fullLength - 2) % 4) > 0)?((fullLength - 2) % 4):4;
            lastLength = 4;
            
            for (k = 0; k < lastLength; k++) 
                checksum += cmd[k];
            
            printk("[Himax] %s: himax_marked, checksum (final): 0x%x\n", __func__, checksum);
            
            //Check Success
            /*if ((u8)ImageBuffer[fullLength - 1] == (u8)(0xFF & (checksum >> 8)) && (u8)ImageBuffer[fullLength - 2] == (u8)(0xFF & checksum)) 
            {
                printk("[Himax] %s: himax_marked, checksum LSB(Success): %d\n", __func__, (u8)ImageBuffer[fullLength - 2]);
                printk("[Himax] %s: himax_marked, checksum MSB(Success): %d\n", __func__, (u8)ImageBuffer[fullLength - 1]);
                himax_FlashMode(0);
                return 1;
            } 
            else //Check Fail
            {
                printk("[Himax] %s: himax_marked, checksum LSB(Fail): %d\n", __func__, (u8)ImageBuffer[fullLength - 2]);
                printk("[Himax] %s: himax_marked, checksum MSB(Fail): %d\n", __func__, (u8)ImageBuffer[fullLength - 1]);
                himax_FlashMode(0);
                return 0;
            }*/
            
            return(himax_hw_checksum(checksum));
            
        }
    }
    return 0;
}
 
//return 1:Success, 0:Fail
int fts_ctpm_fw_upgrade_with_i_file(void)
{
    unsigned char* ImageBuffer = CTPM_FW;
    int fullFileLength = sizeof(CTPM_FW); //Paul Check
  
    int i, j;
    unsigned char cmd[5], last_byte, prePage;
    int FileLength;
    uint8_t checksumResult = 0;

    //Try 3 Times
    for (j = 0; j < 3; j++) 
    {
        FileLength = fullFileLength - 2;

        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0, &cmd[0]))< 0){
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }
       
        mdelay(120);

        himax_unlock_flash();  //ok

        cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0)     //ok
        {
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4F, 0, &cmd[0]))< 0)     //ok
        {
            printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }
        mdelay(50);

        himax_ManualMode(1);                                                 //ok
        himax_FlashMode(1);                                                     //ok

        FileLength = (FileLength + 3) / 4;
        for (i = 0, prePage = 0; i < FileLength; i++) 
        {
            last_byte = 0;

            cmd[0] = i & 0x1F;
            if (cmd[0] == 0x1F || i == FileLength - 1)
                last_byte = 1;
            cmd[1] = (i >> 5) & 0x1F;
            cmd[2] = (i >> 10) & 0x1F;
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
                printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (prePage != cmd[1] || i == 0) 
            {
                prePage = cmd[1];

                cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
            }

            memcpy(&cmd[0], &ImageBuffer[4*i], 4);//Paul
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0]))< 0){
                printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }
                   
            cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (last_byte == 1) 
            {
                cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x05;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk(KERN_ERR "[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                mdelay(10);
                if (i == (FileLength - 1)) 
                {
                    himax_FlashMode(0);
                    himax_ManualMode(0);
                    checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);
                    //himax_ManualMode(0);
                    himax_lock_flash();
                    
                    if (checksumResult) //Success
                    {
                        return 1;
                    } 
                    else if (/*j == 4 && */!checksumResult) //Fail
                    {
                        return 0;
                    } 
                    else //Retry
                    {
                        himax_FlashMode(0);
                        himax_ManualMode(0);
                    }
                }
            }
        }
    }
    return 0;
}

int fts_ctpm_fw_upgrade_with_sys_fs(unsigned char *fw, int len)
{
    unsigned char* ImageBuffer = fw;//CTPM_FW;
    int fullFileLength = len;//sizeof(CTPM_FW); //Paul Check
    int i, j;
    unsigned char cmd[5], last_byte, prePage;
    int FileLength;
    uint8_t checksumResult = 0;
	if(himax_chip->AP_update == 1){
		update_touch_progress(35);
	}
#if 0   
    //Ging for HX8531 test
    unsigned char byteArr[7] = {0};
    
    byteArr[0] = 0x03;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0xE3, 1, &byteArr[0]))< 0){
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
    }
    
    byteArr[0] = 0x10;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0xB7, 1, &byteArr[0]))< 0){
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
    }
    
    byteArr[0] = 0x14;
    byteArr[1] = 0x3C;
    byteArr[2] = 0x0A;
    byteArr[3] = 0x00;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0xCE, 4, &byteArr[0]))< 0){
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
    } 
    
    byteArr[0] = 0x00;
    byteArr[1] = 0x00;
    byteArr[2] = 0x14;
    byteArr[3] = 0x2A;
    byteArr[4] = 0x0D;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0xF6, 5, &byteArr[0]))< 0){
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
    }  
    
    byteArr[0] = 0x0F;
    byteArr[1] = 0x53;
    if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x36, 2, &byteArr[0]))< 0){
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
    } 
    
        
    //////////////////////////////////////////////////////////////
#endif    

    //Try 3 Times
    for (j = 0; j < 3; j++) 
    {
        //FileLength = fullFileLength - 2;
        FileLength = fullFileLength;
        //himax_HW_reset(RST);

        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x81, 0, &cmd[0]))< 0){
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }
       
        mdelay(120);

        himax_unlock_flash();  //ok

        cmd[0] = 0x05;cmd[1] = 0x00;cmd[2] = 0x02;
        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0)     //ok
        {
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }

        if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x4F, 0, &cmd[0]))< 0)     //ok
        {
            printk("[Himax] %s: i2c access fail!\n", __func__);
            return 0;
        }     
        mdelay(50);

        himax_ManualMode(1); //ok
        himax_FlashMode(1);  //ok

        FileLength = (FileLength + 3) / 4;
        for (i = 0, prePage = 0; i < FileLength; i++) 
        {
            last_byte = 0;
            cmd[0] = i & 0x1F;
            if (cmd[0] == 0x1F || i == FileLength - 1)
                last_byte = 1;
            cmd[1] = (i >> 5) & 0x1F;
            cmd[2] = (i >> 10) & 0x1F;
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x44, 3, &cmd[0]))< 0){
                printk("[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (prePage != cmd[1] || i == 0) 
            {
                prePage = cmd[1];
                cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
            }

            memcpy(&cmd[0], &ImageBuffer[4*i], 4);//Paul
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x45, 4, &cmd[0]))< 0){
                printk("[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }

            cmd[0] = 0x01;cmd[1] = 0x0D;cmd[2] = 0x02;
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                printk("[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }
                   
            cmd[0] = 0x01;cmd[1] = 0x09;cmd[2] = 0x02;
            if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                printk("[Himax] %s: i2c access fail!\n", __func__);
                return 0;
            }

            if (last_byte == 1) 
            {
                cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x05;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                cmd[0] = 0x01;cmd[1] = 0x01;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }
                   
                cmd[0] = 0x01;cmd[1] = 0x00;cmd[2] = 0x02;
                if((i2c_smbus_write_i2c_block_data(touch_i2c, 0x43, 3, &cmd[0]))< 0){
                    printk("[Himax] %s: i2c access fail!\n", __func__);
                    return 0;
                }

                mdelay(10);
                if (i == (FileLength - 1)) 
                {
                    himax_FlashMode(0);
                    himax_ManualMode(0);
                    checksumResult = himax_calculateChecksum(ImageBuffer, fullFileLength);//, address, RST);
                    //himax_ManualMode(0);
                    himax_lock_flash();
					
                    if (checksumResult) //Success
                    {
                        //himax_HW_reset(RST);
                        printk("[Himax] %s: checksumResult Success. \n",__func__);
                        return 1;
                    } 
                    else if ((j == 4) && (!checksumResult)) //Fail
                    {
                        //himax_HW_reset(RST);
                        printk("[Himax] %s: checksumResult Fail. \n",__func__);
                        return 0;
                    } 
                    else //Retry
                    {
                    	printk("[Himax] %s: checksumResult Retry \n",__func__);
                    #ifdef ESD_WORKAROUND
                    	reset_activate = 1;
                    #endif
                    	gpio_set_value(himax_chip->rst_gpio, 0);
						msleep(30);
						gpio_set_value(himax_chip->rst_gpio, 1);
						msleep(30);
                        //himax_FlashMode(0);
                        //himax_ManualMode(0);
                    }
                }
            }
        }
    }
    printk("[Himax] %s: End. but upgrade fail. \n",__func__);
    return 0;
}
#endif

//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------start
		#ifdef HX_TP_SYS_FLASH_DUMP
		
		static uint8_t getFlashCommand(void)
		{
			return flash_command;
		}
		
		static uint8_t getFlashDumpProgress(void)
		{
			return flash_progress;
		}
		
		static uint8_t getFlashDumpComplete(void)
		{
			return flash_dump_complete;
		}
		
		static uint8_t getFlashDumpFail(void)
		{
			return flash_dump_fail;
		}
		
		static uint8_t getSysOperation(void)
		{
			return sys_operation;
		}
		
		static uint8_t getFlashReadStep(void)
		{
			return flash_read_step;
		}
		
		static uint8_t getFlashDumpSector(void)
		{
			return flash_dump_sector;
		}
		
		static uint8_t getFlashDumpPage(void)
		{
			return flash_dump_page;
		}

		static bool getFlashDumpGoing(void)
		{
			return flash_dump_going;
		}
		
		static void setFlashBuffer(void)
		{
			int i=0;
			flash_buffer = kzalloc(32768*sizeof(uint8_t), GFP_KERNEL);
			for(i=0; i<32768; i++)
			{
				flash_buffer[i] = 0x00;
			}
		}
		
		static void setSysOperation(uint8_t operation)
		{
			sys_operation = operation;
		}
		
		static void setFlashDumpProgress(uint8_t progress)
		{
			flash_progress = progress;
			printk("TPPPP setFlashDumpProgress : progress = %d ,flash_progress = %d \n",progress,flash_progress);
		}
		
		static void setFlashDumpComplete(uint8_t status)
		{
			flash_dump_complete = status;
		}
		
		static void setFlashDumpFail(uint8_t fail)
		{
			flash_dump_fail = fail;
		}
		
		static void setFlashCommand(uint8_t command)
		{
			flash_command = command;
		}
		
		static void setFlashReadStep(uint8_t step)
		{
			flash_read_step = step;
		}
		
		static void setFlashDumpSector(uint8_t sector)
		{
			flash_dump_sector = sector;
		}
		
		static void setFlashDumpPage(uint8_t page)
		{
			flash_dump_page = page;
		}		

		static void setFlashDumpGoing(bool going)
		{
			flash_dump_going = going;
		}
		
		static ssize_t himax_flash_show(struct device *dev,struct device_attribute *attr, char *buf)
		{
			int ret = 0;
			int loop_i;
			uint8_t local_flash_read_step=0;
			uint8_t local_flash_complete = 0;
			uint8_t local_flash_progress = 0;
			uint8_t local_flash_command = 0;
			uint8_t local_flash_fail = 0;
			
			local_flash_complete = getFlashDumpComplete();
			local_flash_progress = getFlashDumpProgress();
			local_flash_command = getFlashCommand();
			local_flash_fail = getFlashDumpFail();
			
			printk("TPPPP flash_progress = %d \n",local_flash_progress);
			
			if(local_flash_fail)
			{
				ret += sprintf(buf+ret, "FlashStart:Fail \n");
				ret += sprintf(buf + ret, "FlashEnd");
				ret += sprintf(buf + ret, "\n");
				return ret;
			}
			
			if(!local_flash_complete)
			{
				ret += sprintf(buf+ret, "FlashStart:Ongoing:0x%2.2x \n",flash_progress);
				ret += sprintf(buf + ret, "FlashEnd");
				ret += sprintf(buf + ret, "\n");
				return ret;
			}
			
			if(local_flash_command == 1 && local_flash_complete)
			{
				ret += sprintf(buf+ret, "FlashStart:Complete \n");
				ret += sprintf(buf + ret, "FlashEnd");
				ret += sprintf(buf + ret, "\n");
				return ret;
			}
			
			if(local_flash_command == 3 && local_flash_complete)
			{
				ret += sprintf(buf+ret, "FlashStart: \n");
				for(loop_i = 0; loop_i < 128; loop_i++)
				{
					ret += sprintf(buf + ret, "x%2.2x", flash_buffer[loop_i]);
					if((loop_i % 16) == 15)
					{
						ret += sprintf(buf + ret, "\n");
					}
				}
				ret += sprintf(buf + ret, "FlashEnd");
				ret += sprintf(buf + ret, "\n");
				return ret;
			}
			
			//flash command == 0 , report the data
			local_flash_read_step = getFlashReadStep();
			
			ret += sprintf(buf+ret, "FlashStart:%2.2x \n",local_flash_read_step);
			
			for (loop_i = 0; loop_i < 1024; loop_i++) 
			{
				ret += sprintf(buf + ret, "x%2.2X", flash_buffer[local_flash_read_step*1024 + loop_i]);
				
				if ((loop_i % 16) == 15)
				{
					ret += sprintf(buf + ret, "\n");
				}
			}
			
			ret += sprintf(buf + ret, "FlashEnd");
			ret += sprintf(buf + ret, "\n");
			return ret;
		}
		
		//-----------------------------------------------------------------------------------
		//himax_flash_store
		//
		//command 0 : Read the page by step number
		//command 1 : driver start to dump flash data, save it to mem
		//command 2 : driver start to dump flash data, save it to sdcard/Flash_Dump.bin
		//
		//-----------------------------------------------------------------------------------
		static ssize_t himax_flash_store(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
		{
			char buf_tmp[6];
			unsigned long result = 0;
			uint8_t loop_i = 0;
			int base = 0;
			
			memset(buf_tmp, 0x0, sizeof(buf_tmp));
			
			printk(KERN_INFO "[TP] %s: buf[0] = %s\n", __func__, buf);
			
			if(getSysOperation() == 1)
			{
				printk("[TP] %s: SYS is busy , return!\n", __func__);
				return count;
			}
			
			if(buf[0] == '0')
			{
				setFlashCommand(0);
				if(buf[1] == ':' && buf[2] == 'x')
				{
					memcpy(buf_tmp, buf + 3, 2);
					printk(KERN_INFO "[TP] %s: read_Step = %s\n", __func__, buf_tmp);
					if (!strict_strtoul(buf_tmp, 16, &result))
					{
						printk("[TP] %s: read_Step = %lu \n", __func__, result);
						setFlashReadStep(result);
					}
				}
			} 
			else if(buf[0] == '1')
			{
				setSysOperation(1);
				setFlashCommand(1);
				setFlashDumpProgress(0);
				setFlashDumpComplete(0);
				setFlashDumpFail(0);
				queue_work(himax_chip->flash_wq, &himax_chip->flash_work);
			}
			else if(buf[0] == '2')
			{
				setSysOperation(1);
				setFlashCommand(2);
				setFlashDumpProgress(0);
				setFlashDumpComplete(0);
				setFlashDumpFail(0);
				
				queue_work(himax_chip->flash_wq, &himax_chip->flash_work);
			}
			else if(buf[0] == '3')
			{
				setSysOperation(1);
				setFlashCommand(3);
				setFlashDumpProgress(0);
				setFlashDumpComplete(0);
				setFlashDumpFail(0);
				
				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					setFlashDumpSector(result);
				}
				
				memcpy(buf_tmp, buf + 7, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					setFlashDumpPage(result);
				}
				
				queue_work(himax_chip->flash_wq, &himax_chip->flash_work);
			}
			else if(buf[0] == '4')
			{
				printk(KERN_INFO "[TP] %s: command 4 enter.\n", __func__);
				setSysOperation(1);
				setFlashCommand(4);
				setFlashDumpProgress(0);
				setFlashDumpComplete(0);
				setFlashDumpFail(0);
				
				memcpy(buf_tmp, buf + 3, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					setFlashDumpSector(result);
				}
				else
				{
					printk(KERN_INFO "[TP] %s: command 4 , sector error.\n", __func__);
					return count;
				}
				
				memcpy(buf_tmp, buf + 7, 2);
				if (!strict_strtoul(buf_tmp, 16, &result))
				{
					setFlashDumpPage(result);
				}
				else
				{
					printk(KERN_INFO "[TP] %s: command 4 , page error.\n", __func__);
					return count;
				}
				
				base = 11;
				
				printk(KERN_INFO "=========Himax flash page buffer start=========\n");
				for(loop_i=0;loop_i<128;loop_i++)
				{
					memcpy(buf_tmp, buf + base, 2);
					if (!strict_strtoul(buf_tmp, 16, &result))
					{
						flash_buffer[loop_i] = result;
						printk(" %d ",flash_buffer[loop_i]);
						if(loop_i % 16 == 15)
						{
							printk("\n");
						}
					}
					base += 3;
				}
				printk(KERN_INFO "=========Himax flash page buffer end=========\n");
				
				queue_work(himax_chip->flash_wq, &himax_chip->flash_work);		
			}
			return count;
		}
		static DEVICE_ATTR(flash_dump, (S_IWUSR|S_IRUGO),himax_flash_show, himax_flash_store);
			
		static void himax_ts_flash_work_func(struct work_struct *work)
		{
			struct himax_ts_data *ts = container_of(work, struct himax_ts_data, flash_work);
			
			uint8_t page_tmp[128];
			uint8_t x59_tmp[4] = {0,0,0,0};
			int i=0, j=0, k=0, l=0,/* j_limit = 0,*/ buffer_ptr = 0, flash_end_count = 0;
			uint8_t local_flash_command = 0;
			uint8_t sector = 0;
			uint8_t page = 0;
			
			uint8_t x81_command[2] = {0x81,0x00};
			uint8_t x82_command[2] = {0x82,0x00};
			uint8_t x42_command[2] = {0x42,0x00};
			uint8_t x43_command[4] = {0x43,0x00,0x00,0x00};
			uint8_t x44_command[4] = {0x44,0x00,0x00,0x00};
			uint8_t x45_command[5] = {0x45,0x00,0x00,0x00,0x00};
			uint8_t x46_command[2] = {0x46,0x00};
			uint8_t x4A_command[2] = {0x4A,0x00};
			uint8_t x4D_command[2] = {0x4D,0x00};
			/*uint8_t x59_command[2] = {0x59,0x00};*/
		
			disable_irq(ts->client->irq);
			setFlashDumpGoing(true);	
       
			#ifdef HX_RST_PIN_FUNC
				himax_HW_reset();
			#endif

			sector = getFlashDumpSector();
			page = getFlashDumpPage();
			
			local_flash_command = getFlashCommand();
			
			if( i2c_himax_master_write(ts->client, x81_command, 1, 3) < 0 )//sleep out
			{
				printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 81 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(120);

			if( i2c_himax_master_write(ts->client, x82_command, 1, 3) < 0 )
			{
				printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 82 fail.\n",__func__);
				goto Flash_Dump_i2c_transfer_error;
			}
			msleep(100);

			printk(KERN_INFO "[TP] %s: local_flash_command = %d enter.\n", __func__,local_flash_command);
			printk(KERN_INFO "[TP] %s: flash buffer start.\n", __func__);
			for(i=0;i<128;i++)
			{
				printk(KERN_INFO " %2.2x ",flash_buffer[i]);
				if((i%16) == 15)
				{
					printk("\n");
				}
			}
			printk(KERN_INFO "[TP] %s: flash buffer end.\n", __func__);

			if(local_flash_command == 1 || local_flash_command == 2)
			{
				x43_command[1] = 0x01;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0)
				{
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(100);
				
				for( i=0 ; i<8 ;i++)
				{
					for(j=0 ; j<32 ; j++)
					{
						printk("TPPPP Step 2 i=%d , j=%d %s\n",i,j,__func__);
						//read page start
						for(k=0; k<128; k++)
						{
							page_tmp[k] = 0x00;
						}
						for(k=0; k<32; k++)
						{
							x44_command[1] = k;
							x44_command[2] = j;
							x44_command[3] = i;
							if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
							{
								printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
								goto Flash_Dump_i2c_transfer_error;
							}
				
							if( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0)
							{
								printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 46 fail.\n",__func__);
								goto Flash_Dump_i2c_transfer_error;
							}
							//msleep(2);
							if( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0)
							{
								printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 59 fail.\n",__func__);
								goto Flash_Dump_i2c_transfer_error;
							}
							//msleep(2);
							for(l=0; l<4; l++)
							{
								page_tmp[k*4+l] = x59_tmp[l];
							}
							//msleep(10);
						}
						//read page end 
				
						for(k=0; k<128; k++)
						{
							flash_buffer[buffer_ptr++] = page_tmp[k];
							
							if(page_tmp[k] == 0xFF)
							{
								flash_end_count ++;
								if(flash_end_count == 32)
								{
									flash_end_count = 0;
									buffer_ptr = buffer_ptr -32;
									goto FLASH_END;
								}
							}
							else
							{
								flash_end_count = 0;
							}
						}
						setFlashDumpProgress(i*32 + j);
					}
				}
			}
			else if(local_flash_command == 3)
			{
				x43_command[1] = 0x01;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(100);
				
				for(i=0; i<128; i++)
				{
					page_tmp[i] = 0x00;
				}
				
				for(i=0; i<32; i++)
				{
					x44_command[1] = i;
					x44_command[2] = page;
					x44_command[3] = sector;
					
					if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					
					if( i2c_himax_write_command(ts->client, x46_command[0], DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 46 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					if( i2c_himax_read(ts->client, 0x59, x59_tmp, 4, DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 59 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					//msleep(2);
					for(j=0; j<4; j++)
					{
						page_tmp[i*4+j] = x59_tmp[j];
					}
					//msleep(10);
				}
				//read page end
				for(i=0; i<128; i++)
				{
					flash_buffer[buffer_ptr++] = page_tmp[i];
				}
			}
			else if(local_flash_command == 4)
			{
				//page write flow.
				printk(KERN_INFO "[TP] %s: local_flash_command = 4, enter.\n", __func__);

				//-----------------------------------------------------------------------------------------------
				// unlock flash
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x01; 
				x43_command[2] = 0x00; 
				x43_command[3] = 0x06;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x44_command[1] = 0x03; 
				x44_command[2] = 0x00; 
				x44_command[3] = 0x00;
				if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x45_command[1] = 0x00; 
				x45_command[2] = 0x00; 
				x45_command[3] = 0x3D; 
				x45_command[4] = 0x03;
				if( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 45 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				if( i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 4A fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(50);

				//-----------------------------------------------------------------------------------------------
				// page erase
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x01; 
				x43_command[2] = 0x00; 
				x43_command[3] = 0x02;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				x44_command[1] = 0x00; 
				x44_command[2] = page; 
				x44_command[3] = sector;
				if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				if( i2c_himax_write_command(ts->client, x4D_command[0], DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 4D fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(100);
				
				//-----------------------------------------------------------------------------------------------
				// enter manual mode
				//-----------------------------------------------------------------------------------------------
				x42_command[1] = 0x01;
				if( i2c_himax_write(ts->client, x42_command[0],&x42_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 42 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(100);
				
				//-----------------------------------------------------------------------------------------------
				// flash enable
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x01; 
				x43_command[2] = 0x00;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				//-----------------------------------------------------------------------------------------------
				// set flash address
				//-----------------------------------------------------------------------------------------------
				x44_command[1] = 0x00; 
				x44_command[2] = page; 
				x44_command[3] = sector;
				if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				//-----------------------------------------------------------------------------------------------
				// manual mode command : 47 to latch the flash address when page address change.
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x01; 
				x43_command[2] = 0x09; 
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				x43_command[1] = 0x01; 
				x43_command[2] = 0x0D; 
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x43_command[1] = 0x01; 
				x43_command[2] = 0x09; 
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				for(i=0; i<32; i++)
				{
					printk(KERN_INFO "himax :i=%d \n",i);
					x44_command[1] = i; 
					x44_command[2] = page; 
					x44_command[3] = sector;
					if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					msleep(10);
					
					x45_command[1] = flash_buffer[i*4 + 0];
					x45_command[2] = flash_buffer[i*4 + 1];
					x45_command[3] = flash_buffer[i*4 + 2];
					x45_command[4] = flash_buffer[i*4 + 3];
					if( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 45 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					msleep(10);
					
					//-----------------------------------------------------------------------------------------------
					// manual mode command : 48 ,data will be written into flash buffer
					//-----------------------------------------------------------------------------------------------
					x43_command[1] = 0x01; 
					x43_command[2] = 0x0D; 
					if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					msleep(10);
					
					x43_command[1] = 0x01; 
					x43_command[2] = 0x09; 
					if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
					{
						printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
						goto Flash_Dump_i2c_transfer_error;
					}
					msleep(10);
				}

				//-----------------------------------------------------------------------------------------------
				// manual mode command : 49 ,program data from flash buffer to this page
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x01; 
				x43_command[2] = 0x01;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x43_command[1] = 0x01; 
				x43_command[2] = 0x05; 
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x43_command[1] = 0x01; 
				x43_command[2] = 0x01; 
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x43_command[1] = 0x01; 
				x43_command[2] = 0x00; 
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 2, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				//-----------------------------------------------------------------------------------------------
				// flash disable
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x00;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				//-----------------------------------------------------------------------------------------------
				// leave manual mode
				//-----------------------------------------------------------------------------------------------
				x42_command[1] = 0x00;
				if( i2c_himax_write(ts->client, x42_command[0],&x42_command[1], 1, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);
				
				//-----------------------------------------------------------------------------------------------
				// lock flash
				//-----------------------------------------------------------------------------------------------
				x43_command[1] = 0x01; 
				x43_command[2] = 0x00; 
				x43_command[3] = 0x06;
				if( i2c_himax_write(ts->client, x43_command[0],&x43_command[1], 3, DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 43 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				msleep(10);

				x44_command[1] = 0x03; 
				x44_command[2] = 0x00; 
				x44_command[3] = 0x00;
				if( i2c_himax_write(ts->client, x44_command[0],&x44_command[1], 3, DEFAULT_RETRY_CNT) < 0 ) 
				{   
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 44 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}   
				msleep(10);				
				
				x45_command[1] = 0x00; 
				x45_command[2] = 0x00; 
				x45_command[3] = 0x7D; 
				x45_command[4] = 0x03;
				if( i2c_himax_write(ts->client, x45_command[0],&x45_command[1], 4, DEFAULT_RETRY_CNT) < 0 ) 
				{   
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 45 fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}   
				msleep(10);

				if( i2c_himax_write_command(ts->client, x4A_command[0], DEFAULT_RETRY_CNT) < 0 )
				{
					printk(KERN_ERR "[TP]TOUCH_ERR: %s i2c write 4D fail.\n",__func__);
					goto Flash_Dump_i2c_transfer_error;
				}
				
				msleep(50);

				buffer_ptr = 128;
				printk(KERN_INFO "Himax: Flash page write Complete~~~~~~~~~~~~~~~~~~~~~~~\n");
			}

			FLASH_END:

			printk("Complete~~~~~~~~~~~~~~~~~~~~~~~\n");

			printk(" buffer_ptr = %d \n",buffer_ptr);
			
			for (i = 0; i < buffer_ptr; i++) 
			{
				printk("%2.2X ", flash_buffer[i]);
			
				if ((i % 16) == 15)
				{
					printk("\n");
				}
			}
			printk("End~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

			i2c_himax_master_write(ts->client, x43_command, 1, 3);
			msleep(50);

			if(local_flash_command == 2)
			{
				struct file *fn;
				
				fn = filp_open(FLASH_DUMP_FILE,O_CREAT | O_WRONLY ,0);
				if(!IS_ERR(fn))
				{
					fn->f_op->write(fn,flash_buffer,buffer_ptr*sizeof(uint8_t),&fn->f_pos);
					filp_close(fn,NULL);
				}
			}

			#ifdef ENABLE_CHIP_RESET_MACHINE
				if(himax_chip->init_success)
				{
					queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
				}
			#endif

			enable_irq(ts->client->irq);
			setFlashDumpGoing(false);
		
			setFlashDumpComplete(1);
			setSysOperation(0);
			return;

			Flash_Dump_i2c_transfer_error:
				
			#ifdef ENABLE_CHIP_RESET_MACHINE
				if(himax_chip->init_success)
				{
					queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
				}
			#endif

			enable_irq(ts->client->irq);
			setFlashDumpGoing(false);
			setFlashDumpComplete(0);
			setFlashDumpFail(1);
			setSysOperation(0);
			return;
		}
		#endif
	//----[HX_TP_SYS_FLASH_DUMP]------------------------------------------------------------------------------end

#ifdef HX_TP_SYS_FS
static struct kobject *android_touch_kobj = NULL;
static uint8_t himax_command = 0;
static uint8_t debug_log_level= 0;
static uint8_t x_channel = DEFAULT_X_CHANNEL; /* x asix, when you face the top of TS */
static uint8_t y_channel = DEFAULT_Y_CHANNEL; /* y asix, when you face the top of TS  */
static uint8_t *diag_mutual = NULL;
static uint8_t diag_command = 0;
static uint8_t diag_self[DEFAULT_X_CHANNEL+DEFAULT_Y_CHANNEL] = {0};
static unsigned char FW_VER_MAJ_FLASH_buff[FW_VER_MAJ_FLASH_LENG * 4];
static unsigned char FW_VER_MIN_FLASH_buff[FW_VER_MIN_FLASH_LENG * 4];
//static unsigned char CFG_VER_MAJ_FLASH_buff[CFG_VER_MAJ_FLASH_LENG * 4];
//static unsigned char CFG_VER_MIN_FLASH_buff[CFG_VER_MIN_FLASH_ADDR * 4];

static uint8_t *getMutualBuffer(void)
{
    return diag_mutual;
}

static void setMutualBuffer(void)
{
    diag_mutual = kzalloc(x_channel * y_channel * sizeof(uint8_t), GFP_KERNEL);
}

static uint8_t *getSelfBuffer(void)
{
  	return &diag_self[0];
}

static uint8_t getDebugLevel(void)
{
    return debug_log_level;
}

static uint8_t getDiagCommand(void)
{
    return diag_command;
}

static uint8_t getXChannel(void)
{
    return x_channel;
}

static uint8_t getYChannel(void)
{
    return y_channel;
}

static void setXChannel(uint8_t x)
{
    x_channel = x;
}

static void setYChannel(uint8_t y)
{
    y_channel = y;
}

static ssize_t himax_register_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    int ret = 0;
    uint8_t data[96] = { 0 }, loop_i;

    printk(KERN_INFO "[Himax] %s:himax_command=%x\n", __func__, himax_command);

    if (i2c_smbus_read_i2c_block_data(touch_i2c, himax_command, 96, &data[0]) < 0) {
        printk(KERN_WARNING "[Himax] %s: read fail\n", __func__);
        return ret;
    }

    ret += sprintf(buf, "command: %x\n", himax_command);
    for (loop_i = 0; loop_i < 96; loop_i++) {
        ret += sprintf(buf + ret, "0x%2.2X ", data[loop_i]);
        if ((loop_i % 16) == 15)
            ret += sprintf(buf + ret, "\n");
    }
    ret += sprintf(buf + ret, "\n");
    return ret;
}

static ssize_t himax_register_store(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    char buf_tmp[6], length = 0;
    uint8_t veriLen = 0;
    uint8_t write_da[100];
    unsigned long result = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));

    if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
        if (buf[2] == 'x') {
            uint8_t loop_i;
            uint16_t base = 5;
            memcpy(buf_tmp, buf + 3, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
                himax_command = result;
            for (loop_i = 0; loop_i < 100; loop_i++) {
                if (buf[base] == '\n') {
                    if (buf[0] == 'w')
                        i2c_smbus_write_i2c_block_data(touch_i2c, himax_command, length, &write_da[0]);
                    printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
                        write_da[0], length);
                    for (veriLen = 0; veriLen < length; veriLen++)
                        printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));

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
    return count;
}

static ssize_t touch_vendor_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    ssize_t ret = 0;

    sprintf(buf, "%s_%s\n", FW_VER_MAJ_FLASH_buff, FW_VER_MIN_FLASH_buff);
    ret = strlen(buf) + 1;

    return ret;
}

static ssize_t himax_get_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[Himax]:touch firmware version=%s.\n",himax_chip->himax_version);
	return sprintf(buf, "%s-%s-%s\n", himax_chip->himax_version, HIMAX_DRIVER_VERSION, ASUS_DRIVER_VERSION);
}

static ssize_t himax_chip_self_test_function(struct device *dev, struct device_attribute *attr, char *buf)
{
	int val=0x00;
	uint8_t data[16];
	
	val = himax_chip_self_test(data);
	if(val == 0)
	{
		return sprintf(buf, "%d\n",val);
	}
	else
	{
		return sprintf(buf, "%d, Error code= 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n"
		,val, data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],data[9],data[10],data[11],data[12],data[13],data[14],data[15]);
	}
		
}

static ssize_t himax_get_touch_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Himax]:touch status=%d. \n",himax_chip->tp_status);
	return sprintf(buf, "%d \n", himax_chip->tp_status);
}

static ssize_t himax_chip_enable_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Himax]:himax_chip_enable_irq \n");
	enable_irq(himax_chip->irq);
	return sprintf(buf, "%d \n", 0);
}

static ssize_t himax_chip_check_running(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	
	printk(KERN_INFO "[Himax]:himax_chip_check_running \n");
	ret = himax_hang_shaking();	//0:Running, 1:Stop, 2:I2C Fail
	if(ret == 2)
	{
		return sprintf(buf, "I2C Fail.\n");
	}
	if(ret == 1)
	{
		return sprintf(buf, "MCU Stop.\n");
	}
	else
		return sprintf(buf, "MCU Running\n");

}
static ssize_t himax_chip_poweron(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Himax]: adb cat himax_chip_poweron. \n");

	mutex_lock(&himax_chip->mutex_lock);
#ifdef ESD_WORKAROUND
	reset_activate = 1;
#endif
	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);
	mutex_unlock(&himax_chip->mutex_lock);
	himax_ts_poweron(himax_chip);

	printk(KERN_INFO "[Himax]: adb cat himax_chip_poweron done.\n");
	return sprintf(buf, "%d \n", 0);
}

static ssize_t himax_debug_level_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    size_t count = 0;

    count += sprintf(buf, "%d\n", debug_log_level);

    return count;
}

static unsigned char upgrade_fw[32*1024];

static ssize_t himax_chip_firmware_upgrade(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
#ifdef HX_TP_FW_UPDATE
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
#endif
	int err = 0;
	
#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
#endif
	wake_lock(&himax_chip->wake_lock);
	//wake_lock(&himax_chip->wake_lock_fw);
	
	himax_chip->tp_firmware_upgrade_proceed = 1;
	printk(KERN_INFO "[Himax]: himax_chip_firmware_upgrade suspend_state=%x\n",himax_chip->suspend_state);
	if(!himax_chip->suspend_state)
	{
		printk("[Himax] himax chip entry suspend for firmware upgrade \n");
		himax_ts_suspend_command(himax_chip->client);
		disable_irq(himax_chip->irq);
		err = cancel_work_sync(&himax_chip->work);
		if(err)
			enable_irq(himax_chip->irq);
		
		himax_chip->suspend_state = 1;
		msleep(300);
	}	
	
    if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
        debug_log_level = buf[0] - '0';
	
	mutex_lock(&himax_chip->mutex_lock);
#ifdef HX_TP_FW_UPDATE
    if(buf[0] == 'b')
    {
        printk(KERN_INFO "[Himax] %s: upgrade firmware from file start!\n", __func__);
        printk("[Himax] %s: file open:/system/etc/firmware/touch_fw.bin \n",__func__);
        filp = filp_open("/system/etc/firmware/touch_fw.bin", O_RDONLY, 0);
        if(IS_ERR(filp)) {
            printk(KERN_ERR "[Himax] %s: open firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }
        oldfs = get_fs();
        set_fs(get_ds());

        result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
        if(result < 0) {
            printk(KERN_ERR "[Himax] %s: read firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }

        set_fs(oldfs);
        filp_close(filp, NULL);

        printk("[Himax] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
        printk("[Himax] %s: upgrade firmware verison, %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);
        if(result > 0)
        {
            if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
                printk(KERN_INFO "[Himax] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
            else
                printk(KERN_INFO "[Himax] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
            
            goto firmware_upgrade_done;
            //return count;                    
        }
    }
	else if(buf[0] == 't')
    {
        printk(KERN_INFO "[Himax] %s: upgrade firmware from file start!\n", __func__);
        printk("[Himax] %s: file open:/data/local/fw.bin \n",__func__);
        filp = filp_open("/data/local/fw.bin", O_RDONLY, 0);
        if(IS_ERR(filp)) {
            printk(KERN_ERR "[Himax] %s: open firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }
        oldfs = get_fs();
        set_fs(get_ds());

        result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
        if(result < 0) {
            printk(KERN_ERR "[Himax] %s: read firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }

        set_fs(oldfs);
        filp_close(filp, NULL);

        printk("[Himax] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
        printk("[Himax] %s: upgrade firmware verison, %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);

        if(result > 0)
        {
            if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
                printk(KERN_INFO "[Himax] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
            else
                printk(KERN_INFO "[Himax] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
            
            goto firmware_upgrade_done;
            //return count;                    
        }
    }
    else if(buf[0] == 'f') 
    {
        printk(KERN_INFO "[Himax] %s: upgrade firmware from kernel image start!\n", __func__);
        if (isTP_Updated == 0)
        {
            printk("Himax touch isTP_Updated: %d\n", isTP_Updated);
            if(1)// (himax_read_FW_ver() == 0)
            {
                printk("Himax touch firmware upgrade: %d\n", isTP_Updated);
                if(fts_ctpm_fw_upgrade_with_i_file() == 0)
                    printk("himax_marked TP upgrade error, line: %d\n", __LINE__);
                else
                    printk("himax_marked TP upgrade OK, line: %d\n", __LINE__);
                isTP_Updated = 1;
            }
        }
    }
#endif
    
firmware_upgrade_done:
	mutex_unlock(&himax_chip->mutex_lock);
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	himax_chip->tp_firmware_upgrade_proceed = 0;
	himax_chip->suspend_state = 0;
	enable_irq(himax_chip->irq);
	wake_unlock(&himax_chip->wake_lock);
	//wake_unlock(&himax_chip->wake_lock_fw);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	return count;
}

#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
static int himax_chip_self_firmware_upgrade(struct work_struct *dat)
{
	struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    int upgrade_fw_ver_checksum;
    int err = 0;
    char upgrade_tp_version[4];
    char version[32];
    int i = 0, open_fail = 0;
    int touch_fw = 0, onboard_fw = 0;

#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
#endif
	wake_lock(&himax_chip->wake_lock);
	//wake_lock(&himax_chip->wake_lock_fw);
	memset(version, 0, 32);
	
	printk("[Himax] %s: file open:/system/etc/firmware/touch_fw.bin \n",__func__);
	for(i = 1; i <= 3; i++)
	{		
		filp = filp_open("/system/etc/firmware/touch_fw.bin", O_RDONLY, 0);
    	if(!IS_ERR_OR_NULL(filp))
    	{
    		oldfs = get_fs();
			set_fs(get_ds());
			
			result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
			if(result < 0) {
		    	printk("[Himax] %s: read firmware file failed\n", __func__);
			}
			else
			{
				open_fail = 0;
				i = 4;
				printk("[Himax] %s: read firmware file done\n", __func__);
			}
	
			set_fs(oldfs);
			filp_close(filp, NULL);			
		}
		else
		{
			 printk("[Himax] %s: open firmware file failed\n", __func__);
			 open_fail++;
			 msleep(3000);
		}
		
		if(open_fail == 3)
		{
			printk("[Himax] %s: open firmware file retry failed.\n", __func__);
			
			//mutex_unlock(&himax_chip->mutex_lock);
			wake_unlock(&himax_chip->wake_lock);
			//wake_unlock(&himax_chip->wake_lock_fw);
		#ifdef ENABLE_CHIP_STATUS_MONITOR
			queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
		#endif
			return -1;
		}	
	}
	
	printk("[Himax] %s: upgrade len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
	printk("[Himax] %s: %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);
	       
	upgrade_tp_version[0] = upgrade_fw[0x1000];
	upgrade_tp_version[1] = upgrade_fw[0x1001];
	upgrade_tp_version[2] = upgrade_fw[0x1002];
	upgrade_tp_version[3] = upgrade_fw[0x1003];
	
	strncpy(version, &upgrade_tp_version[0], 4);
	printk("[Himax]:upgrade firmware verison=%s.\n",version);

	upgrade_fw_ver_checksum = ((upgrade_tp_version[0] << 24) | (upgrade_tp_version[1] << 16 ) | (upgrade_tp_version[2] << 8) | upgrade_tp_version[3]);
	printk("[Himax]himax_chip->version_checksum = %d \n",himax_chip->version_checksum);
	printk("[Himax]upgrade_fw_ver_checksum = %d \n",upgrade_fw_ver_checksum);



	if(result > 0)
	{
		if((upgrade_fw_ver_checksum > himax_chip->version_checksum) || (onboard_fw == 1)&&(touch_fw == 0))
		{
			himax_chip->tp_firmware_upgrade_proceed = 1;
			if(!himax_chip->suspend_state)
			{
				printk("[Himax] himax chip entry suspend for firmware upgrade \n");
				himax_ts_suspend_command(himax_chip->client);
				disable_irq(himax_chip->irq);
				err = cancel_work_sync(&himax_chip->work);
				if (err)
					enable_irq(himax_chip->irq);
				
				himax_chip->suspend_state = 1;
				msleep(300);
			}
   	
			printk("[Himax] himax chip firmware upgrade start. \n");
			mutex_lock(&himax_chip->mutex_lock);
			
			if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
				printk(KERN_INFO "[Himax] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
			else
				printk(KERN_INFO "[Himax] %s: himax chip firmware upgrade done.\n", __func__);
				
			mutex_unlock(&himax_chip->mutex_lock);
			queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
			himax_chip->tp_firmware_upgrade_proceed = 0;
			himax_chip->suspend_state = 0;
			enable_irq(himax_chip->irq);
			//wake_unlock(&himax_chip->wake_lock_fw);
			wake_unlock(&himax_chip->wake_lock);
		#ifdef ENABLE_CHIP_STATUS_MONITOR
			queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
		#endif
			printk(KERN_INFO "[Himax] %s: himax chip re-start.\n", __func__);
			return 0;
		}
		else
		{
			printk(KERN_INFO "[Himax] %s: Current touch firmware is the latest version.\n", __func__);
		}
	}

	//mutex_unlock(&himax_chip->mutex_lock);
	//wake_unlock(&himax_chip->wake_lock_fw);
	wake_unlock(&himax_chip->wake_lock);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	return 0;
}
#endif

// ++++++ add by Josh for AP update touch fw ++++++

static void himax_AP_chip_reset_function(void)
{
	printk("[touch_fw_update]:himax_chip_reset_function ++ \n");
	
	if(himax_chip->retry_time <= 10 )
	{ 
		wake_lock(&himax_chip->wake_lock);
		mutex_lock(&himax_chip->mutex_lock);
	#ifdef ESD_WORKAROUND
		reset_activate = 1;
	#endif
		gpio_set_value(himax_chip->rst_gpio, 0);
		msleep(30);
		gpio_set_value(himax_chip->rst_gpio, 1);
		msleep(30);
		mutex_unlock(&himax_chip->mutex_lock);
		himax_ts_poweron(himax_chip);
		wake_unlock(&himax_chip->wake_lock);
		//Joe Test ++
		if(gpio_get_value(himax_chip->intr_gpio) == 0)
		{
			printk("[touch_fw_update]%s: IRQ = 0, Enable IRQ\n", __func__); 
			enable_irq(himax_chip->irq);
		}
		//Joe Test --

	}	
	himax_chip->retry_time ++;
	printk("[touch_fw_update]:himax_chip_reset_function retry_time =%d --\n",himax_chip->retry_time);
	
	update_touch_result(TOUCH_FW_UPGRADE_SUCCESS);
	update_touch_progress(100); //add by josh for update touch progress
	
}
static int himax_AP_firmware_check()
{
	struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    int upgrade_fw_ver_checksum;
    int err = 0;
    char upgrade_tp_version[4];
    char version[32];
    int i = 0, open_fail = 1;
    int touch_fw = 0, onboard_fw = 0;

//#ifdef ENABLE_CHIP_STATUS_MONITOR
	//cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
//#endif
	//wake_lock(&himax_chip->wake_lock);
	//wake_lock(&himax_chip->wake_lock_fw);
	memset(version, 0, 32);
	
	printk("[Touch_update] %s: file open:/system/etc/firmware/touch_fw.bin \n",__func__);
	
	filp = filp_open("/system/etc/firmware/touch_fw.bin", O_RDONLY, 0);
    if(!IS_ERR_OR_NULL(filp))
    {
    	oldfs = get_fs();
		set_fs(get_ds());
		
		result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
		if(result < 0) {
	    	printk("[Touch_update] %s: read firmware file failed\n", __func__);
			set_fs(oldfs);
			filp_close(filp, NULL);			
			return 3;
		}
		else
		{
			open_fail = 0;
			printk("[Touch_update] %s: read firmware file done\n", __func__);
		}
		set_fs(oldfs);
		filp_close(filp, NULL);			
	}
	else
	{
		printk("[Touch_update] %s: open firmware file failed\n", __func__);
		return 2;
	}
	
	
	printk("[Touch_update] %s: upgrade len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
	printk("[Touch_update] %s: %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);
	       
	upgrade_tp_version[0] = upgrade_fw[0x1000];
	upgrade_tp_version[1] = upgrade_fw[0x1001];
	upgrade_tp_version[2] = upgrade_fw[0x1002];
	upgrade_tp_version[3] = upgrade_fw[0x1003];
	
	strncpy(version, &upgrade_tp_version[0], 4);
	printk("[Touch_update]:upgrade firmware verison=%s.\n",version);

	upgrade_fw_ver_checksum = ((upgrade_tp_version[0] << 24) | (upgrade_tp_version[1] << 16 ) | (upgrade_tp_version[2] << 8) | upgrade_tp_version[3]);
	printk("[Touch_update]himax_chip->version_checksum = %d \n",himax_chip->version_checksum);
	printk("[Touch_update]upgrade_fw_ver_checksum = %d \n",upgrade_fw_ver_checksum);
	if(upgrade_fw_ver_checksum == 1110454576)
	def_checksum = 50474; //B010
	else
	def_checksum = 22412; //B00F
	
	printk("[Touch_update]def_checksum = %d \n",def_checksum);
	
	if((upgrade_fw_ver_checksum > himax_chip->version_checksum))
	{
		return 0;
	}
	else
	{
		return 1;
	}

}

// +++++++ add by Josh +++++++
static int himax_AP_firmware_upgrade()
{
	struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    int upgrade_fw_ver_checksum;
    int err = 0;
    char upgrade_tp_version[4];
    char version[32];
    int i = 0, j = 0, open_fail = 0;
    int touch_fw = 0, onboard_fw = 0, upgrade_count = 0, suspend_err = 0;
	
#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
#endif
	wake_lock(&himax_chip->wake_lock);
	//wake_lock(&himax_chip->wake_lock_fw);
	memset(version, 0, 32);
	
	// ++++++ add by josh for touch progress & result init ++++++
	update_touch_progress(0);
	update_touch_result(TOUCH_FW_UPGRADE_INIT);
	// ------ add by josh for touch progress & result init ------
	
	printk("[touch_fw_update] %s: file open:/system/etc/firmware/touch_fw.bin \n",__func__);
	for(i = 1; i <= 3; i++)
	{		
		filp = filp_open("/system/etc/firmware/touch_fw.bin", O_RDONLY, 0);

    	if(!IS_ERR_OR_NULL(filp))
    	{
    		oldfs = get_fs();
			set_fs(get_ds());
			
			result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
			if(result < 0) {
		    	printk("[touch_fw_update] %s: read firmware file failed\n", __func__);
			}
			else
			{
				open_fail = 0;
				i = 4;
				printk("[touch_fw_update] %s: read firmware file done\n", __func__);
			}
	
			set_fs(oldfs);
			filp_close(filp, NULL);			
		}
		else
		{
			 printk("[touch_fw_update] %s: open firmware file failed\n", __func__);
			 open_fail++;
			 msleep(30);
		}
		
		if(open_fail == 3)
		{
			printk("[touch_fw_update] %s: open firmware file retry failed.\n", __func__);
			
			//mutex_unlock(&himax_chip->mutex_lock);
			wake_unlock(&himax_chip->wake_lock);
			//wake_unlock(&himax_chip->wake_lock_fw);
		#ifdef ENABLE_CHIP_STATUS_MONITOR
			queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
		#endif
			//return -1;
			goto upgrade_fail;
		}	
	}
	
	// ++++++ add by josh for update touch progress & result ++++++
	update_touch_progress(5);  
	update_touch_result(TOUCH_FW_UPGRADE_PROCESS);
	// ------ add by josh for update touch progress & result ------
	
	/*printk("[touch_fw_update] %s: upgrade len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
	printk("[touch_fw_update] %s: %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);
	  
	upgrade_tp_version[0] = upgrade_fw[0x1000];
	upgrade_tp_version[1] = upgrade_fw[0x1001];
	upgrade_tp_version[2] = upgrade_fw[0x1002];
	upgrade_tp_version[3] = upgrade_fw[0x1003];
	
	strncpy(version, &upgrade_tp_version[0], 4);
	printk("[touch_fw_update]:upgrade firmware verison=%s.\n",version);

	upgrade_fw_ver_checksum = ((upgrade_tp_version[0] << 24) | (upgrade_tp_version[1] << 16 ) | (upgrade_tp_version[2] << 8) | upgrade_tp_version[3]);
	printk("[touch_fw_update]himax_chip->version_checksum = %d \n",himax_chip->version_checksum);
	printk("[touch_fw_update]upgrade_fw_ver_checksum = %d \n",upgrade_fw_ver_checksum);
	*/


	if(result > 0)
	{
		himax_chip->tp_firmware_upgrade_proceed = 1;
		if(!himax_chip->suspend_state)
		{
			printk("[touch_fw_update] himax chip entry suspend for firmware upgrade \n");
			suspend_err = himax_ts_suspend_command(himax_chip->client);
			if(suspend_err != 0){
			    goto upgrade_fail;
			}
			disable_irq(himax_chip->irq);
			err = cancel_work_sync(&himax_chip->work);
			if (err)
				enable_irq(himax_chip->irq);
			
			himax_chip->suspend_state = 1;
			msleep(300);
		}
	
		printk("[touch_fw_update] himax chip firmware upgrade start. \n");
		mutex_lock(&himax_chip->mutex_lock);
		update_touch_progress(10);  //add by josh for update touch progress
		for(j = 0; j <= 2; j++){
			if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0){
				printk(KERN_INFO "[touch_fw_update] %s: TP upgrade error, line: %d, retry times: %d\n", __func__, __LINE__,upgrade_count);
				upgrade_count ++;
			}else{
				printk(KERN_INFO "[touch_fw_update] %s: himax chip firmware upgrade done.\n", __func__);
				j = 5;
			}
		}
		if(upgrade_count == 3 || chip_hw_checksum != def_checksum){
			printk("[touch_fw_update] %s: upgrade firmware file retry failed.\n", __func__);
			goto upgrade_fail;
		}
		
		update_touch_progress(90); //add by josh for update touch progress
		
		mutex_unlock(&himax_chip->mutex_lock);
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
		himax_chip->tp_firmware_upgrade_proceed = 0;
		himax_chip->suspend_state = 0;
		enable_irq(himax_chip->irq);
		//wake_unlock(&himax_chip->wake_lock_fw);
		wake_unlock(&himax_chip->wake_lock);
	#ifdef ENABLE_CHIP_STATUS_MONITOR
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
	#endif
		printk(KERN_INFO "[touch_fw_update] %s: himax chip re-start.\n", __func__);
		
		update_touch_result(TOUCH_FW_UPGRADE_SUCCESS); //add by josh for update touch result sucess
		
		return 0;
	
	}

	//mutex_unlock(&himax_chip->mutex_lock);
	//wake_unlock(&himax_chip->wake_lock_fw);
	wake_unlock(&himax_chip->wake_lock);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	update_touch_result(TOUCH_FW_UPGRADE_SUCCESS); //add by josh for update touch result sucess 
	
	return 0;
upgrade_fail:
	//queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	himax_chip->tp_firmware_upgrade_proceed = 0;
	himax_chip->suspend_state = 0;
	enable_irq(himax_chip->irq);
	mutex_unlock(&himax_chip->mutex_lock);
	wake_unlock(&himax_chip->wake_lock);	
	printk(KERN_INFO "[touch_fw_update] %s: upgrade_fail.\n", __func__);
	himax_AP_chip_reset_function();
	update_touch_result(TOUCH_FW_UPGRADE_FAIL);	
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	return -1;	
	

}

// ------ add by Josh for AP update touch fw ------

#ifdef ENABLE_CHIP_STATUS_MONITOR
static int himax_chip_monitor_function(struct work_struct *dat)	//for ESD solution
{
	int ret;
	
//	printk(KERN_INFO "[Himax] running_status = %d, suspend_state =%d\n", himax_chip->running_status, himax_chip->suspend_state);
	
//	printk("[Himax]%s: IRQ =%x\n", __func__,gpio_get_value(himax_chip->intr_gpio)); //Joe Test
	
	if(himax_chip->running_status == 0 && himax_chip->suspend_state == 0)
	{	
	//	printk(KERN_INFO "[Himax] %s \n", __func__);
		
		//Joe Test ++
		if(gpio_get_value(himax_chip->intr_gpio) == 0)
		{
			printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__); 
			enable_irq(himax_chip->irq);
		}
		//Joe Test --
		
		ret = himax_hang_shaking();	//0:Running, 1:Stop, 2:I2C Fail
		if(ret == 2)
		{
			queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
			printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
		}
		if(ret == 1)
		{
			printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
			himax_chip->retry_time = 0;
			//queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
			//Do HW_RESET??
			ESD_HW_REST();
		}
		//else
			//printk(KERN_INFO "[Himax] %s: MCU Running \n", __func__);
		
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
	}
	
	return 0;	
}	
#endif
#ifdef	CONFIG_PROC_FS
int himax_chip_ap_control_firmware_upgrade(void)
{
	struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
    int err = 0;

#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
#endif
	wake_lock(&himax_chip->wake_lock);
	//wake_lock(&himax_chip->wake_lock_fw);
	himax_chip->tp_firmware_upgrade_proceed = 1;
		
	printk("[Himax] %s: file open:/system/etc/firmware/touch_fw.bin \n",__func__);
	filp = filp_open("/system/etc/firmware/touch_fw.bin", O_RDONLY, 0);
	if(IS_ERR(filp)) {
	    printk(KERN_ERR "[Himax] %s: open firmware file failed\n", __func__);
	     goto upgrade_fail;
	}
	oldfs = get_fs();
	set_fs(get_ds());
	
	result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
	if(result < 0) {
	    printk(KERN_ERR "[Himax] %s: read firmware file failed\n", __func__);
	    goto upgrade_fail;
	}
	
	set_fs(oldfs);
	filp_close(filp, NULL);
	
	printk("[Himax] %s: upgrade len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
	printk("[Himax] %s: upgrade firmware verison, %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);
	
	if(result > 0)
	{
		if(!himax_chip->suspend_state)
		{
			printk("[Himax] himax chip entry suspend for firmware upgrade \n");
			himax_ts_suspend_command(himax_chip->client);
			disable_irq(himax_chip->irq);
			err = cancel_work_sync(&himax_chip->work);
			if (err)
				enable_irq(himax_chip->irq);
			
			himax_chip->suspend_state = 1;
			msleep(300);
		}
		
		mutex_lock(&himax_chip->mutex_lock);
		printk("[Himax] himax chip firmware upgrade start. \n");
		if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0){
			printk(KERN_INFO "[Himax] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
			mutex_unlock(&himax_chip->mutex_lock);
			goto upgrade_fail;
		}
		else
		{
			mutex_unlock(&himax_chip->mutex_lock);
			printk(KERN_INFO "[Himax] %s: himax chip firmware upgrade done.\n", __func__);
		}
	}
	
	//mutex_unlock(&himax_chip->mutex_lock);
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	himax_chip->tp_firmware_upgrade_proceed = 0;
	himax_chip->suspend_state = 0;
	enable_irq(himax_chip->irq);
	//wake_unlock(&himax_chip->wake_lock_fw);
	wake_unlock(&himax_chip->wake_lock);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif

	return 0;
	
upgrade_fail:

	//mutex_unlock(&himax_chip->mutex_lock);
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	himax_chip->tp_firmware_upgrade_proceed = 0;
	himax_chip->suspend_state = 0;
	enable_irq(himax_chip->irq);
	//wake_unlock(&himax_chip->wake_lock_fw);
	wake_unlock(&himax_chip->wake_lock);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	printk(KERN_INFO "[Himax] %s: himax chip re-start.\n", __func__);
	
return -1;
}
#endif

#ifdef ENABLE_CHIP_RESET_MACHINE
static void himax_chip_reset_function(struct work_struct *dat)
{
	printk("[Himax]:himax_chip_reset_function ++ \n");
	
	if(himax_chip->retry_time <= 10 )
	{ 
		wake_lock(&himax_chip->wake_lock);
		mutex_lock(&himax_chip->mutex_lock);
	#ifdef ESD_WORKAROUND
		reset_activate = 1;
	#endif
		gpio_set_value(himax_chip->rst_gpio, 0);
		msleep(30);
		gpio_set_value(himax_chip->rst_gpio, 1);
		msleep(30);
		mutex_unlock(&himax_chip->mutex_lock);
		himax_ts_poweron(himax_chip);
		wake_unlock(&himax_chip->wake_lock);
		//Joe Test ++
		if(gpio_get_value(himax_chip->intr_gpio) == 0)
		{
			printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__); 
			enable_irq(himax_chip->irq);
		}
		//Joe Test --
	}
	himax_chip->retry_time ++;
	printk("[Himax]:himax_chip_reset_function retry_time =%d --\n",himax_chip->retry_time);
}
#endif

int hx8528_cable_status(int status)
{
	uint8_t buf0[2] = {0};
	int ret = 0;
	
	if(Read_PROJ_ID() != PROJ_ID_ME372CG){
		return 0;
	}
	
	if(cable_status == -1){
		cable_status = status;
		printk("[Himax] %s: cable_status = %d " , __func__, status);
		return 0;
	}else{
		cable_status = status;
		printk("[Himax] %s: cable_status=%d, tp_status=%d,init_success=%d.\n", __func__, status,himax_chip->tp_status,himax_chip->init_success);
	
	if((himax_chip->tp_status == 1) && (himax_chip->init_success == 1))
	{
		if(status == 0x02)
		{
			buf0[0] = 0x90;
			buf0[1] = 0x00;
			ret = i2c_himax_master_write(himax_chip->client, buf0, 2, DEFAULT_RETRY_CNT);
		}
		else if((status == 0x00) || (status == 0x01))
		{
			buf0[0] = 0x90;
			buf0[1] = 0x01;
			ret = i2c_himax_master_write(himax_chip->client, buf0, 2, DEFAULT_RETRY_CNT);
		}
	}
	return 0;
	}
}
EXPORT_SYMBOL(hx8528_cable_status);

int proximity_sensor_status(int status)
{
	//himax_chip->proximity_status = status;
	//printk(KERN_ERR "[Himax] %s: proximity_status: %d \n", __func__, himax_chip->proximity_status);
	return 0;
}
EXPORT_SYMBOL(proximity_sensor_status);

int power_status(int status)
{
// status 5: Charg
	int ret = 0;
	uint8_t cmdbuf[2];
	printk(KERN_ERR "[Himax] %s: power_status: %d \n", __func__, status);
	if(Read_PROJ_ID() != PROJ_ID_ME372CG){
		return 0;
	}
	if((himax_chip->tp_status == 1) && (himax_chip->init_success == 1) && (himax_chip->client != NULL)){
	
		mutex_lock(&himax_chip->mutex_lock);
		
		if(status < 5){
			cmdbuf[0] = HX_CMD_TSSOFF; //0x82
			ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
			if(ret < 0) {
				printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSOFF line: %d \n", __func__, __LINE__);
				goto send_i2c_msg_fail;
			} 
			mdelay(120); //120ms
			
			printk(KERN_ERR "[Himax] %s: 0xC5: 0x0A \n", __func__);
			cmdbuf[0] = 0xC5;
			cmdbuf[1] = 0x0A;
			ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 2, DEFAULT_RETRY_CNT);
			if(ret < 0) {
			printk(KERN_ERR "[Himax]:write 0xC5 failed line: %d \n",__LINE__);
			goto send_i2c_msg_fail;
			}
			udelay(10);
			
			cmdbuf[0] = HX_CMD_TSSON; //0x83
			ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);//sense on
			if(ret < 0) {
				printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSON line: %d \n", __func__, __LINE__);
				goto send_i2c_msg_fail;
			} 
			mdelay(120); //120ms
		}else{
			cmdbuf[0] = HX_CMD_TSSOFF; //0x82
			ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
			if(ret < 0) {
				printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSOFF line: %d \n", __func__, __LINE__);
				goto send_i2c_msg_fail;
			} 
			mdelay(120); //120ms
			
			printk(KERN_ERR "[Himax] %s: 0xC5: 0x0C \n", __func__);
			cmdbuf[0] = 0xC5;
			cmdbuf[1] = 0x0C;
			ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 2, DEFAULT_RETRY_CNT);
			if(ret < 0) {
			printk(KERN_ERR "[Himax]:write 0xC5 failed line: %d \n",__LINE__);
			goto send_i2c_msg_fail;
			}
			udelay(10);
			
			cmdbuf[0] = HX_CMD_TSSON; //0x83
			ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);//sense on
			if(ret < 0) {
				printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSON line: %d \n", __func__, __LINE__);
				goto send_i2c_msg_fail;
			} 
			mdelay(120); //120ms
		}
		
		mutex_unlock(&himax_chip->mutex_lock);
		
	}
	else{
		printk(KERN_ERR "[Himax] %s: power_status: return 0 \n", __func__);
		return 0;
	}
	
return 0;
	
send_i2c_msg_fail:	
	mutex_unlock(&himax_chip->mutex_lock);
	printk(KERN_ERR "[Himax] %s: send_i2c_msg_fail: %d \n", __func__, __LINE__);
	return 0;
}
EXPORT_SYMBOL(power_status);

#ifdef ESD_WORKAROUND
//HW Reset
void ESD_HW_REST(void)
{
	reset_activate = 1;
	ESD_COUNTER = 0;
						
	printk("Himax TP: ESD - Reset\n");
	wake_lock(&himax_chip->wake_lock);
	mutex_lock(&himax_chip->mutex_lock);
	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);
	mutex_unlock(&himax_chip->mutex_lock);
	himax_ts_poweron(himax_chip);
	wake_unlock(&himax_chip->wake_lock);
	hx8528_cable_status(cable_status);
	//Joe Test ++
	if(gpio_get_value(himax_chip->intr_gpio) == 0)
	{
		printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__); 
		enable_irq(himax_chip->irq);
	}
	//Joe Test --
}
#endif

static int himax_chip_self_test(uint8_t *data)
{
	uint8_t cmdbuf[11];
	int ret = 0;
	uint8_t valuebuf[16];
	int i = 0, pf_value = 0x00;
	
	wake_lock(&himax_chip->wake_lock);
	mutex_lock(&himax_chip->mutex_lock);
	
	if(himax_chip->suspend_state == 0)
	{
		disable_irq(himax_chip->irq);
	}
#ifdef ESD_WORKAROUND	
	reset_activate = 1;
#endif
	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);
	
	cmdbuf[0] = HX_CMD_MANUALMODE;//0x42
	cmdbuf[1] = 0x02;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 2, DEFAULT_RETRY_CNT);//Reload Disable
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_MANUALMODE line: %d \n", __func__, __LINE__);
	}
	udelay(100);
	
    cmdbuf[0] = HX_CMD_SETMICROOFF;//0x35
	cmdbuf[1] = 0x02;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 2, DEFAULT_RETRY_CNT);//Reload Disable
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SETMICROOFF line: %d \n", __func__, __LINE__);
	}
	udelay(100);	
	
	cmdbuf[0] = HX_CMD_SETROMRDY;//0x36
	cmdbuf[1] = 0x0F;
	cmdbuf[2] = 0x53;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 3, DEFAULT_RETRY_CNT);//enable flash
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SETROMRDY line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
	
	cmdbuf[0] = HX_CMD_SET_CACHE_FUN;//0xDD
	cmdbuf[1] = 0x05;
	cmdbuf[2] = 0x03;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 3, DEFAULT_RETRY_CNT);//prefetch
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SET_CACHE_FUN line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
	
	himax_ts_loadconfig(himax_chip);
	
	cmdbuf[0] = 0xF1;
	cmdbuf[1] = 0x06;
	cmdbuf[2] = 0x05;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 3, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send 0xF1 failed line: %d \n",__LINE__);
	} 
	udelay(10);
		
//	if ((Read_HW_ID() == HW_ID_ER)||(Read_HW_ID() == HW_ID_SR1)||(Read_HW_ID() == HW_ID_SR1_with_final_LCD))
//	{
//		ret = i2c_himax_master_write(himax_chip->client, r58_ER, sizeof(r58_ER), DEFAULT_RETRY_CNT);
//		udelay(5);
//		
//		if(himax_chip->formal_lens == 0x55) //Wintek
//		{
//			ret = i2c_himax_master_write(himax_chip->client, rC5_wintek, sizeof(rC5_wintek), DEFAULT_RETRY_CNT);
//			udelay(5);
//			ret = i2c_himax_master_write(himax_chip->client, rC6_wintek, sizeof(rC6_wintek), DEFAULT_RETRY_CNT);
//			udelay(5);
//		}
//	}
//	else
//	{
//		if(himax_chip->formal_lens == 0x55)//Wintek
//		{
//			ret = i2c_himax_master_write(himax_chip->client, rC5_wintek, sizeof(rC5_wintek), DEFAULT_RETRY_CNT);
//			udelay(5);
//			ret = i2c_himax_master_write(himax_chip->client, rC6_wintek, sizeof(rC6_wintek), DEFAULT_RETRY_CNT);
//			udelay(5);
//		}
//	}
	
	ret = i2c_himax_read(himax_chip->client, 0xF3, valuebuf, 1, DEFAULT_RETRY_CNT);//0xF3
		
	cmdbuf[0] = 0xF3;
	cmdbuf[1] = valuebuf[0] & 0xFD;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 2, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write 0xF3 line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
				 
	cmdbuf[0] = HX_CMD_SELFTEST_BUFFER;
	cmdbuf[1] = r8D_setting[0]; //180
	cmdbuf[2] = r8D_setting[1]; //100
	cmdbuf[3] = 0x19; 
	cmdbuf[4] = 0x0F;
	cmdbuf[5] = 0x2D; 
	cmdbuf[6] = 0x09; 
	cmdbuf[7] = 0x32; 
	cmdbuf[8] = 0x08;//0x19
	
	printk("[Himax] %s: r8D_setting[0]:%x ,  r8D_setting[1]:%x \n", __func__, r8D_setting[0],r8D_setting[1]);
	
    ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 9, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SELFTEST_BUFFER line: %d \n", __func__, __LINE__);
    }
	udelay(100);
	ret = i2c_himax_read(himax_chip->client, HX_CMD_SELFTEST_BUFFER, valuebuf, 9, DEFAULT_RETRY_CNT);//0x8D
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Read HX_CMD_SELFTEST_BUFFER line: %d \n", __func__, __LINE__);
	}
	for(i=0; i<8; i++)
	{
		printk("[Himax]:0x8D[%d] = 0x%x\n",i,valuebuf[i]);
	}
		
	cmdbuf[0] = 0xE9;
	cmdbuf[1] = 0x00;
	cmdbuf[2] = 0x06;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 3, DEFAULT_RETRY_CNT);//write sernsor to self-test mode
	if(ret < 0) {
	    printk(KERN_ERR "[Himax] %s: Write 0xE9 line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
	
	ret = i2c_himax_read(himax_chip->client, 0xE9, valuebuf, 3, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Read 0xE9 line: %d \n", __func__, __LINE__);
	}
	for(i=0; i<8; i++)
	{
		printk("[Himax]:0xE9[%d] = 0x%x\n",i,valuebuf[i]);
	}
		
	cmdbuf[0] = HX_CMD_TSSON; //0x83
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);//sense on
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSON line: %d \n", __func__, __LINE__);
	} 
	mdelay(120); //120ms

	cmdbuf[0] = HX_CMD_TSSLPOUT;//0x81
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSLPOUT line: %d \n", __func__, __LINE__);
	}			
	mdelay(3000); //1500ms
	
	cmdbuf[0] = HX_CMD_TSSOFF; //0x82
    ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);
    if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSOFF line: %d \n", __func__, __LINE__);
    } 
	mdelay(120); //120ms
	
	memset(valuebuf, 0x00 , 16);
	ret = i2c_himax_read(himax_chip->client, HX_CMD_SELFTEST_BUFFER, valuebuf, 16, DEFAULT_RETRY_CNT);//0x8D
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Read HX_CMD_SELFTEST_BUFFER line: %d \n", __func__, __LINE__);
	}
	else
	{
		if(valuebuf[0]==0xAA)
		{
			printk("[Himax]: self-test pass\n");
			pf_value = 0x0;
		}
		else
		{
			printk("[Himax]: self-test fail\n");
			pf_value = 0x1;
			for(i=0;i<16;i++)
			{
				printk("[Himax]:0x8D return buff[%d] = 0x%x\n",i,valuebuf[i]);
				data[i] = valuebuf[i];
			}
		}
	}
	udelay(100);
		
	cmdbuf[0] = 0xE9;
	cmdbuf[1] = 0x00;
	cmdbuf[2] = 0x00;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 3, DEFAULT_RETRY_CNT);//write sensor to normal mode
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write 0xE9 line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
	
	ret = i2c_himax_read(himax_chip->client, 0xF3, valuebuf, 1, DEFAULT_RETRY_CNT);//0xF3
		
	cmdbuf[0] = 0xF3;
	cmdbuf[1] = valuebuf[0] | 0x02;
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 2, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write 0xF3 line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
			
	cmdbuf[0] = HX_CMD_TSSON;//0x83
	ret = i2c_himax_master_write(himax_chip->client, cmdbuf, 1, DEFAULT_RETRY_CNT);//sense on
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSON line: %d \n", __func__, __LINE__);
	}
	msleep(120); //120ms
	
	mutex_unlock(&himax_chip->mutex_lock);
		
	printk("[Himax]:himax_chip->suspend_state =%d\n",himax_chip->suspend_state);
	if(himax_chip->suspend_state == 1)
	{
		himax_ts_suspend_command(himax_chip->client);	
	}
	else
	{
		enable_irq(himax_chip->irq);	
	}	
	
	wake_unlock(&himax_chip->wake_lock);
	return pf_value;
}

static ssize_t himax_chip_raw_data_store(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    size_t count = 0;
    uint32_t loop_i;
    uint16_t mutual_num, self_num, width;
    struct file* filp = NULL;
    mm_segment_t oldfs;

    mutual_num = x_channel * y_channel;
    self_num = x_channel + y_channel;
    width = x_channel;
	
	if(diag_command == 1)
	{
		filp = filp_open("/data/local/touch_dc.txt", O_RDWR|O_CREAT,S_IRUSR);
		if(IS_ERR(filp)) {
	   		printk(KERN_ERR "[Himax] %s: open /data/local/touch_dc.txt failed\n", __func__);
	    	return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}
	else if(diag_command == 2)
	{
		filp = filp_open("/data/local/touch_iir.txt", O_RDWR|O_CREAT,S_IRUSR);
		if(IS_ERR(filp)) {
	   		printk(KERN_ERR "[Himax] %s: open /data/local/touch_iir.txt failed\n", __func__);
	    	return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}
	else if(diag_command == 3)
	{
		filp = filp_open("/data/local/touch_bank.txt", O_RDWR|O_CREAT,S_IRUSR);
		if(IS_ERR(filp)) {
	   		printk(KERN_ERR "[Himax] %s: open /data/local/touch_bank.txt failed\n", __func__);
	    	return 0;
		}
		oldfs = get_fs();
		set_fs(get_ds());
	}

    count += sprintf(buf + count, "Channel: %4d, %4d\n\n", x_channel, y_channel);
    if (diag_command >= 1 && diag_command <= 6) {
        if (diag_command < 4) {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                
                if ((loop_i % width) == (width - 1)) {
                    count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i/width]);
                }
            }
            count += sprintf(buf + count, "\n");
            for (loop_i = 0; loop_i < width; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                {
                    count += sprintf(buf + count, "\n");
                }
            }
        } else if (diag_command > 4) {
            for (loop_i = 0; loop_i < self_num; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i - mutual_num) % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        } else {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        }
    }
	if(diag_command >= 1 && diag_command <= 3)
	{
		filp->f_op->write(filp, buf, count, &filp->f_pos);
		set_fs(oldfs);
		filp_close(filp, NULL);
	}

    return count;
}

static ssize_t himax_diag_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
    size_t count = 0;
    uint32_t loop_i;
    uint16_t mutual_num, self_num, width;

    mutual_num = x_channel * y_channel;
    self_num = x_channel + y_channel;
    width = x_channel;
    count += sprintf(buf + count, "Channel: %4d, %4d\n\n", x_channel, y_channel);

    if (diag_command >= 1 && diag_command <= 6) {
        if (diag_command < 4) {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1)) {
                    count += sprintf(buf + count, " %3d\n", diag_self[width + loop_i/width]);
                }
            }
            count += sprintf(buf + count, "\n");
            for (loop_i = 0; loop_i < width; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                {
                    count += sprintf(buf + count, "\n");
                }
            }
        } else if (diag_command > 4) {
            for (loop_i = 0; loop_i < self_num; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_self[loop_i]);
                if (((loop_i - mutual_num) % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        } else {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                count += sprintf(buf + count, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    count += sprintf(buf + count, "\n");
            }
        }
    }

    return count;
}

static ssize_t himax_diag_dump(struct device *dev,
        struct device_attribute *attr, const char *buf, size_t count)
{
    const uint8_t command_ec_128_raw_flag = 0x01;
    const uint8_t command_ec_24_normal_flag = 0xFC;
    const uint8_t command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
    uint8_t command_91h[2] = {0x91, 0x00};
    uint8_t command_82h[1] = {0x82};
    uint8_t command_F3h[2] = {0xF3, 0x00};
    uint8_t command_83h[1] = {0x83};
	uint8_t receive[1];

    if (buf[0] == '1')
    {
        command_91h[1] = command_ec_128_raw_baseline_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        diag_command = buf[0] - '0';
    }
    else if (buf[0] == '2')
    {
        command_91h[1] = command_ec_128_raw_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        diag_command = buf[0] - '0';
    }
    else if (buf[0] == '3')
	{
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_82h[0], 1, &command_82h[0]);
    	msleep(50);
    	i2c_smbus_read_i2c_block_data(touch_i2c, command_F3h[0], 1, &receive[0]);
    	command_F3h[1] = (receive[0] | 0x80);
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_F3h[0], 2, &command_F3h[1]);    	
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_83h[0], 2, &command_83h[0]);
    	msleep(50);

    	command_91h[1] = command_ec_128_raw_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        diag_command = buf[0] - '0';
    }
    else
    {
        command_91h[1] = command_ec_24_normal_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        
        i2c_smbus_write_i2c_block_data(touch_i2c, command_82h[0], 1, &command_82h[0]);
    	msleep(50);
    	i2c_smbus_read_i2c_block_data(touch_i2c, command_F3h[0], 1, &receive[0]);
    	command_F3h[1] = (receive[0] & 0x7F);
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_F3h[0], 2, &command_F3h[1]);
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_83h[0], 2, &command_83h[0]);
        diag_command = 0;
		touch_count = 0;
    }
	printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
	
    return count;
}

static int himax_touch_sysfs_init(void)
{
    int ret;
    android_touch_kobj = kobject_create_and_add("android_touch", NULL);
    if (android_touch_kobj == NULL) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: subsystem_register failed\n");
        ret = -ENOMEM;
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_fw_upgrade.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: create_file debug_level dev_attr_tp_fw_upgrade failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_debug_level.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: create_file debug_level dev_attr_debug_level failed\n");
        return ret;
    }
    himax_command = 0;
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_register.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: create_file register dev_attr_register failed\n");
        return ret;
    }

    ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_vendor failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_diag.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_diag failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_fw_version.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_fw_version failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_self_test failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_status.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_touch_status failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_output_raw_data.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_output_raw_data failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_irq.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_touch_irq failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_touch_poweron.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_touch_poweron failed\n");
        return ret;
    }
    ret = sysfs_create_file(android_touch_kobj, &dev_attr_tp_check_running.attr);
    if (ret) {
        printk(KERN_ERR "[Himax]TOUCH_ERR: sysfs_create_file dev_attr_tp_check_running failed\n");
        return ret;
    }
    
    #ifdef HX_TP_SYS_FLASH_DUMP
		ret = sysfs_create_file(android_touch_kobj, &dev_attr_flash_dump.attr);
		if (ret) 
		{
			printk(KERN_ERR "[TP]TOUCH_ERR: sysfs_create_file failed\n");
			return ret;
		}
		#endif
		
    return 0 ;
}

static void himax_touch_sysfs_deinit(void)
{
    sysfs_remove_file(android_touch_kobj, &dev_attr_diag.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_debug_level.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_register.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_vendor.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_fw_upgrade.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_fw_version.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_self_test.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_touch_status.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_output_raw_data.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_touch_irq.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_touch_poweron.attr);
    sysfs_remove_file(android_touch_kobj, &dev_attr_tp_check_running.attr);
    kobject_del(android_touch_kobj);
}
#endif

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h);
static void himax_ts_late_resume(struct early_suspend *h);
#endif

int himax_hang_shaking(void)	//0:Running, 1:Stop, 2:I2C Fail
{
	int ret, result;
	uint8_t hw_reset_check[1];
	uint8_t hw_reset_check_2[1];
	uint8_t buf0[2];
	
	mutex_lock(&himax_chip->mutex_lock); 
	
	//Write 0x92
	buf0[0] = 0x92;
	if(IC_STATUS_CHECK == 0xAA)
	{
		buf0[1] = 0xAA;
		IC_STATUS_CHECK = 0x55;
	}
	else
	{
		buf0[1] = 0x55;
		IC_STATUS_CHECK = 0xAA;
	}
	ret = i2c_himax_master_write(himax_chip->client, buf0, 2, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write 0x92 failed line: %d \n",__LINE__);
			goto work_func_send_i2c_msg_fail;
	}
	msleep(15);	//Must more than 1 frame
	
	buf0[0] = 0x92;
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(himax_chip->client, buf0, 2, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write 0x92 failed line: %d \n",__LINE__);
			goto work_func_send_i2c_msg_fail;
	}
	msleep(2);
	
	ret = i2c_himax_read(himax_chip->client, 0xDA, hw_reset_check, 1, DEFAULT_RETRY_CNT);
	if(ret < 0)
	{
		printk(KERN_ERR "[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
		goto work_func_send_i2c_msg_fail;
	}
	//printk("[Himax]: ESD 0xDA - 0x%x.\n", hw_reset_check[0]);	
	
	if((IC_STATUS_CHECK != hw_reset_check[0]))
	{	
		msleep(2);
		ret = i2c_himax_read(himax_chip->client, 0xDA, hw_reset_check_2, 1, DEFAULT_RETRY_CNT);
		if(ret < 0)
		{
			printk(KERN_ERR "[Himax]:i2c_himax_read 0xDA failed line: %d \n",__LINE__);
			goto work_func_send_i2c_msg_fail;
		}
		//printk("[Himax]: ESD check 2 0xDA - 0x%x.\n", hw_reset_check_2[0]);
		
		if(hw_reset_check[0] == hw_reset_check_2[0])
			result = 1; //MCU Stop
		else
			result = 0; //MCU Running
	}
	else
		result = 0; //MCU Running
	
	mutex_unlock(&himax_chip->mutex_lock);
	return result;
	
work_func_send_i2c_msg_fail:
	mutex_unlock(&himax_chip->mutex_lock);
	return 2;
}

static void himax_ts_work_func(struct work_struct *work)
{
#ifdef HX_TP_SYS_FS
	uint8_t *mutual_data;
	uint8_t *self_data;
	int mul_num, self_num;
	int index = 0;
#endif
	int ret, i;
	unsigned int x=0, y=0, area=0, press=0;
	const unsigned int x_res = DEFAUULT_X_RES;
	const unsigned int y_res = DEFAUULT_Y_RES;
	uint8_t diag_cmd;
	unsigned char check_sum_cal = 0;
	struct i2c_msg msg[2];
	uint8_t start_reg;
	uint8_t buf[128] = {0};

#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_chip->running_status = 1;
	cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
#endif
	
	start_reg = HX_CMD_RAE;
	msg[0].addr = himax_chip->client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &start_reg;
	
	msg[1].addr = himax_chip->client->addr;
	msg[1].flags = I2C_M_RD;
#ifdef HX_TP_SYS_FS
	diag_cmd = getDiagCommand();
	if(diag_cmd == 0) 
		msg[1].len =  HX_TOUCH_INFO_SIZE; 
	else         
		msg[1].len = HX_TOUCH_INFO_SIZE + RAW_DATA_LENGTH_PER_PKT;
#else
	msg[1].len =  HX_TOUCH_INFO_SIZE;
#endif
	msg[1].buf = buf;

	// Read out all events from touch IC
	mutex_lock(&himax_chip->mutex_lock);
	
	ret = i2c_transfer(himax_chip->client->adapter, msg, 2);
	if (ret < 0) {
		printk(KERN_INFO "[Himax]:%s:i2c_transfer fail.\n", __func__);
		memset(buf, 0xff , 128);
	#ifdef ENABLE_CHIP_RESET_MACHINE
		mutex_unlock(&himax_chip->mutex_lock);
		enable_irq(himax_chip->irq);
		goto work_func_send_i2c_msg_fail;
	#endif
	}

#ifdef ESD_DEBUG
	for(i = 0; i < HX_TOUCH_INFO_SIZE; i++)
		printk("0x%X ",buf[i]);
		
	printk("\n");
#endif

#ifdef ESD_WORKAROUND
	if(touch_count > 3 && diag_cmd == 0 && reset_count < 5){
		printk("[Himax]: touch up count = 0.\n");	
		touch_count = 0;
		reset_count++;
		mutex_unlock(&himax_chip->mutex_lock);
		enable_irq(himax_chip->irq);
		ESD_HW_REST();
	#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_chip->running_status = 0;
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
	#endif	
		return;
	}
	
	for(i = 0; i < HX_TOUCH_INFO_SIZE; i++)
	{
		if(buf[i] == 0x00)
			check_sum_cal = 1;
		else if(buf[i] == 0xED)
			check_sum_cal = 2;
		else
		{
			check_sum_cal = 0;
			i = HX_TOUCH_INFO_SIZE;
		}
	}
	
	diag_cmd = getDiagCommand();
	if(check_sum_cal != 0 && reset_activate == 0 && diag_cmd == 0)	//ESD Check
	{
		mutex_unlock(&himax_chip->mutex_lock);
		ret = himax_hang_shaking();	//0:Running, 1:Stop, 2:I2C Fail
		enable_irq(himax_chip->irq);
		
		if(ret == 2)
		{	
			goto work_func_send_i2c_msg_fail;
		}
			
		if((ret == 1) && (check_sum_cal == 1))
		{	
			printk("[Himax]: ESD event checked - ALL Zero.\n");	
			ESD_HW_REST();
		}
		else if(check_sum_cal == 2)
		{
			printk("[Himax]: ESD event checked - ALL 0xED.\n");
			ESD_HW_REST();
		}
	#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_chip->running_status = 0;
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
	#endif
		return;
	}
	else if(reset_activate) 
	{
		reset_activate = 0;
		printk(KERN_INFO "[Himax]:%s: Back from ESD reset, ready to serve.\n", __func__);
		mutex_unlock(&himax_chip->mutex_lock);
		enable_irq(himax_chip->irq);
	#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_chip->running_status = 0;
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
	#endif
		return;
	}
#endif 
	
	check_sum_cal = 0;
	for(i = 0; i < HX_TOUCH_INFO_SIZE; i++)
		check_sum_cal += buf[i];

	if ((check_sum_cal != 0x00) || (buf[HX_TOUCH_INFO_POINT_CNT] & 0xF0 )!= 0xF0)
	{
	//	printk("[himax]:check_sum_cal: 0x%0x, buf[HX_TOUCH_INFO_POINT_CNT] = 0x%x\n", check_sum_cal, buf[HX_TOUCH_INFO_POINT_CNT]);
		mutex_unlock(&himax_chip->mutex_lock);
		enable_irq(himax_chip->irq);
		
		if (unlikely(himax_debug_flag))
		{
			for(i = 0; i < HX_TOUCH_INFO_SIZE; i++)
			printk("0x%X ",buf[i]);
			printk("\n");
		}	
	#ifdef ESD_WORKAROUND
		ESD_COUNTER++;
		printk("[Himax]: ESD event checked - check_sum_cal, ESD_COUNTER = %d.\n", ESD_COUNTER);
		if(ESD_COUNTER > ESD_COUNTER_SETTING)
		{
			input_report_key(himax_chip->input_dev, BTN_TOUCH, 0);
			input_mt_sync(himax_chip->input_dev);
			input_sync(himax_chip->input_dev);
			ESD_HW_REST();
		}
	#endif

	#ifdef ENABLE_CHIP_STATUS_MONITOR
		himax_chip->running_status = 0;
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
	#endif
		return;
	}

#ifdef HX_TP_SYS_FS
	// Print out the raw data by printk
	if (getDebugLevel() & 0x1) {
		printk(KERN_INFO "[Himax]%s: raw data:\n", __func__);
		for (i = 0; i < 128; i=i+8) {
			printk(KERN_INFO "%d: 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X, 0x%2.2X \n", i, buf[i], buf[i+1], buf[i+2], buf[i+3], buf[i+4], buf[i+5], buf[i+6], buf[i+7]);
		}
	}

	// Print out the raw data by sysfs node
	diag_cmd = getDiagCommand();
	if (diag_cmd >= 1 && diag_cmd <= 6) 
	{
	//	printk(KERN_INFO "[himax]%s: Diag Raw Data:\n", __func__);
		mutual_data = getMutualBuffer();
		self_data = getSelfBuffer();
		
		/* Header: %x, %x, %x, %x\n", buf[24], buf[25], buf[26], buf[27] */
		//printk("Header: %x, %x, %x, %x\n", buf[48], buf[49], buf[50], buf[51]);
		mul_num = getXChannel() * getYChannel();
		self_num = getXChannel() + getYChannel();
		
		if(buf[56] == buf[57] && buf[57] == buf[58] && buf[58] == buf[59] && buf[56] > 0) 
		{
			index = (buf[56] - 1) * 68;
			//printk("Header[%d]: %x, %x, %x, %x, mutual: %d, self: %d\n", index, buf[56], buf[57], buf[58], buf[59], mul_num, self_num);
			for (i = 0; i < 68; i++) {
				if (index < mul_num) 
				{ //mutual
					mutual_data[index + i] = buf[i + 60];
				} 
				else 
				{//self
					if (i >= self_num)
						break;
					self_data[i] = buf[i + 60];
				}
			}
		}
		else
		{
			printk(KERN_INFO "[Himax]%s: header format is wrong!\n", __func__);
		}
	}
#endif

	// Touch Point information
	if (buf[HX_TOUCH_INFO_POINT_CNT] == 0xff)
	{
		touch_count++;
		input_report_key(himax_chip->input_dev, BTN_TOUCH, 0);
		input_mt_sync(himax_chip->input_dev);
		input_sync(himax_chip->input_dev);

#ifdef ESD_WORKAROUND
		ESD_COUNTER = 0;
#endif
		
		if (unlikely(himax_debug_flag))
		{
			printk("[Himax] %s: touch up!\n", __func__);
		}
	}
	else
	{
		for(i=0; i<HX_TP_MAX_FINGER; i++)
		{
			if(buf[4*i] != 0xFF)
			{
				y = buf[4 * i + 1] | (buf[4 * i] << 8) ;
				x = buf[4 * i + 3] | (buf[4 * i + 2] << 8);
				
				if((x <= x_res) && (y <= y_res))
				{
					press = buf[4*HX_TP_MAX_FINGER+i];
					area = press;
					if(area > 31)
					{
						area = (area >> 3);
					}

/*
#ifdef Coor_Mapping
			//		printk("[Himax] %s: x1 = %d, y1 = %d!\n", __func__, x, y);
					x = ((x - Real_X_Start) * x_res) / (Real_X_End - Real_X_Start);
					y = ((y - Real_Y_Start) * y_res) / (Real_Y_End - Real_Y_Start);
			//		printk("[Himax] %s: x2 = %d, y2 = %d!\n", __func__, x, y);
#endif				
*/	
					input_report_key(himax_chip->input_dev, BTN_TOUCH, 1);
					input_report_abs(himax_chip->input_dev, ABS_MT_TRACKING_ID, i);
					input_report_abs(himax_chip->input_dev, ABS_MT_TOUCH_MAJOR, area); //Finger Size
					input_report_abs(himax_chip->input_dev, ABS_MT_PRESSURE, press);
					input_report_abs(himax_chip->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(himax_chip->input_dev, ABS_MT_POSITION_Y, y);
					input_report_abs(himax_chip->input_dev, ABS_MT_WIDTH_MAJOR, area); //Touch Size
					
					input_report_key(himax_chip->input_dev, BTN_TOUCH, 1);
					input_mt_sync(himax_chip->input_dev);

#ifdef ESD_WORKAROUND
					ESD_COUNTER = 0;
#endif					
				}
				if(reset_count >= 0) reset_count--;
				touch_count = 0;
				if (unlikely(himax_debug_flag))
				{
					printk("[Himax] %s: x = %d, y = %d!\n", __func__, x, y);
				}
			}
			else
			{
				input_mt_sync(himax_chip->input_dev);
			}
		} 
		
		input_sync(himax_chip->input_dev);
		
		/*if (unlikely(himax_debug_flag))
		{
			printk("[Himax] %s: touch down!\n", __func__);
		}*/
	}
	
	himax_chip->retry_time = 0;
	mutex_unlock(&himax_chip->mutex_lock);
	enable_irq(himax_chip->irq);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_chip->running_status = 0;
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	return;

work_func_send_i2c_msg_fail:
	printk(KERN_ERR "[Himax]:work_func_send_i2c_msg_fail: %d \n",__LINE__);
#ifdef ENABLE_CHIP_RESET_MACHINE
	if(himax_chip->init_success)
	{
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	}
#endif
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_chip->running_status = 0;
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	return;
}

static irqreturn_t himax_ts_irq_handler(int irq, void *dev_id)
{

//	printk("[Himax] himax_ts_irq_handler ++ \n");
    disable_irq_nosync(himax_chip->irq);
    queue_work(himax_chip->himax_wq, &himax_chip->work);

    return IRQ_HANDLED;
}

static int himax_ts_register_interrupt(struct i2c_client *client)
{
    int err = 0;
	printk("[Himax]: irq number =%d, client name =%s \n",himax_chip->irq, himax_chip->client->name);
    err = request_irq(himax_chip->irq, himax_ts_irq_handler,
            IRQF_TRIGGER_FALLING, himax_chip->client->name, himax_chip->client);
    if (err)
        dev_err(&himax_chip->client->dev, "[himax] %s: request_irq %d failed\n",
                __func__, himax_chip->irq);

    return err;
}
static void himax_ts_tpk_config(struct himax_ts_data *ts_modify)
{
	int ret = 0;
	
	u8 r62[] =  { 0x62, 0x02, 0x00, 0x02, 0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r63[] =  { 0x63, 0x02, 0x00, 0x12, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r64[] =  { 0x64, 0x02, 0x00, 0x02, 0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r65[] =  { 0x65, 0x00, 0x00, 0x10, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r66[] =  { 0x66, 0x00, 0x00, 0x20, 0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r67[] =  { 0x67, 0x00, 0x00, 0x10, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r68[] =  { 0x68, 0x03, 0x00, 0x43, 0x31, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r69[] =  { 0x69, 0x03, 0x00, 0x13, 0x34, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r6A[] =  { 0x6A, 0x03, 0x00, 0x43, 0x31, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r6B[] =  { 0x6B, 0x04, 0x00, 0x14, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r6C[] =  { 0x6C, 0x04, 0x00, 0x24, 0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r6D[] =  { 0x6D, 0x04, 0x00, 0x14, 0x42, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r6E[] =  { 0x6E, 0x04, 0x00, 0x24, 0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r6F[] =  { 0x6F, 0x04, 0x00, 0x14, 0x42, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r70[] =  { 0x70, 0x04, 0x00, 0x24, 0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r71[] =  { 0x71, 0x04, 0x00, 0x04, 0x42, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r72[] =  { 0x72, 0x04, 0x00, 0x24, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r73[] =  { 0x73, 0x04, 0x00, 0x04, 0x42, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r74[] =  { 0x74, 0x04, 0x00, 0x24, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r75[] =  { 0x75, 0x04, 0x00, 0x04, 0x42, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r76[] =  { 0x76, 0x04, 0x00, 0x24, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r77[] =  { 0x77, 0x03, 0x00, 0x13, 0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r78[] =  { 0x78, 0x03, 0x00, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 r79[] =  { 0x79, 0x03, 0x00, 0x03, 0x31, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
	u8 rC9[] =  { 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x59, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x2B, 0x2C, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x59, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x2B, 0x2C, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	u8 r8A[] =  { 0x8A, 0x18, 0x02, 0xFF, 0xFF, 0x19, 0x01, 0xFF, 0xFF, 0x1A, 0x00, 0xFF, 0xFF, 0x1B, 0x36, 0xFF, 0xFF, 0x1C, 0x35, 0xFF, 0xFF, 0x1D, 0x34, 0xFF, 0xFF, 0x1E, 0xFF, 0x03, 0x17, 0x1F, 0xFF, 0x04, 0x16, 0x20, 0xFF, 0x05, 0x15, 0x21, 0xFF, 0xFF, 0x14, 0x22, 0x33, 0xFF, 0x13, 0x23, 0x32, 0xFF, 0x12, 0x24, 0x31, 0xFF, 0x11, 0x25, 0x30, 0xFF, 0x10, 0x26, 0x2F, 0xFF, 0x0F, 0xFF, 0x2E, 0xFF, 0x0E, 0xFF, 0x2D, 0xFF, 0x0D, 0xFF, 0x2C, 0xFF, 0x0C, 0xFF, 0x2B, 0xFF, 0x0B, 0xFF, 0x2A, 0xFF, 0x0A, 0xFF, 0x29, 0xFF, 0x09, 0x27, 0xFF, 0x06, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0x28, 0xFF, 0x08, 0xFF};
	u8 rF1[] =  { 0xF1, 0x06, 0x06, 0x06, 0x10 };
	u8 rF3[] =  { 0xF3, 0x57 };
	u8 rE3[] =  { 0xE3, 0x00 };
	u8 rB6[] =  { 0xB6, 0x14 };
	u8 rB9[] =  { 0xB9, 0x01, 0x36 };
	u8 r7B[] =  { 0x7B, 0x03 };
	u8 r7C[] =  { 0x7C, 0x40, 0xD8, 0x8C };
	u8 r7F[] =  { 0x7F, 0x00, 0x04, 0x0A, 0x0A, 0x04, 0x00, 0x00, 0x00 };
	u8 rD3[] =  { 0xD3, 0x06, 0x01 };
	u8 rF7[] =  { 0xF7, 0x55, 0x55, 0x00, 0x55, 0x55, 0x55, 0x00, 0x86, 0x06, 0x00 };
	u8 rB4[] =  { 0xB4, 0x0C, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0E, 0x03, 0x0F, 0x03, 0x0F, 0x03, 0x0F, 0x00 };
	u8 rD5[] =  { 0xD5, 0x65, 0x09 };
	u8 rC2[] =  { 0xC2, 0x12 };
	u8 rC5[] =  { 0xC5, 0x0C, 0x1F, 0x08, 0x10, 0x1A, 0x1F, 0x0B };
	u8 rC6[] =  { 0xC6, 0x12, 0x10, 0x19 };
	u8 rCB[] =  { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x01, 0xFF, 0x02, 0xF0, 0x00, 0x00 };
	u8 r91[] =  { 0x91, 0x00 };
	u8 rB7[] =  { 0xB7, 0x10 };
	u8 rCE[] =  { 0xCE, 0x14, 0x30, 0xAB, 0x00 };
	u8 rC7[] =  { 0xC7, 0x03, 0x20, 0x05, 0x00 };
	u8 rCF[] =  { 0xCF, 0x40, 0x1F, 0x1F, 0x01, 0xE0 };
	u8 r8C[] =  { 0x8C, 0x63, 0x22, 0x04, 0x03 };
	u8 rDF[] =  { 0xDF, 0x00, 0x0F, 0x04, 0x18, 0x00 };
	u8 rE1[] =  { 0xE1, 0x20 };
	u8 rEA[] =  { 0xEA, 0x15, 0x22, 0x00, 0x16 };
	u8 rEB[] =  { 0xEB, 0x60, 0x3C, 0xFF, 0x02 };
	u8 rEC[] =  { 0xEC, 0x08, 0x20, 0x20, 0x20, 0x12, 0x00, 0x00 };
	u8 rEE[] =  { 0xEE, 0x00 };
	u8 rEF[] =  { 0xEF, 0x11, 0x00 };
	u8 rF0[] =  { 0xF0, 0xA0 };
	u8 rF2[] =  { 0xF2, 0x0A, 0x0A, 0x14, 0x3C };
	u8 rF4[] =  { 0xF4, 0x96, 0xB4, 0x2D, 0x31 };
	u8 rF6[] =  { 0xF6, 0x00, 0x00, 0x14, 0x2A, 0x0D };
	u8 r53[] =  { 0x53, 0x10, 0x0C, 0x20, 0x04 };
	u8 r54[] =  { 0x54, 0x1E, 0x20, 0x10, 0x10 };
	u8 r56[] =  { 0x56, 0x30, 0x15, 0x3C };
	u8 r57[] =  { 0x57, 0x20, 0x40, 0x01, 0x01 };
	u8 r58[] =  { 0x58, 0x04, 0x28, 0x88, 0x14 };
	
	printk("[Himax]: load TPK config \n");
	printk("[Himax]: %s ++ \n",__func__);
	
	ret = i2c_himax_master_write(ts_modify->client, r62, sizeof(r62), DEFAULT_RETRY_CNT);
	udelay(5);
	
	ret = i2c_himax_master_write(ts_modify->client, r63, sizeof(r63), DEFAULT_RETRY_CNT);
	udelay(5);
	
	ret = i2c_himax_master_write(ts_modify->client, r64, sizeof(r64), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r65, sizeof(r65), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r66, sizeof(r66), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r67, sizeof(r67), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r68, sizeof(r68), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r69, sizeof(r69), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r6A, sizeof(r6A), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r6B, sizeof(r6B), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r6C, sizeof(r6C), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r6D, sizeof(r6D), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r6E, sizeof(r6E), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r6F, sizeof(r6F), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r70, sizeof(r70), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r71, sizeof(r71), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r72, sizeof(r72), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r73, sizeof(r73), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r74, sizeof(r74), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r75, sizeof(r75), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r76, sizeof(r76), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r77, sizeof(r77), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r78, sizeof(r78), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r79, sizeof(r79), DEFAULT_RETRY_CNT);
	udelay(5);
		
	ret = i2c_himax_master_write(ts_modify->client, rC9, sizeof(rC9), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r8A, sizeof(r8A), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rF1, sizeof(rF1), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rF3, sizeof(rF3), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rE3, sizeof(rE3), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rB6, sizeof(rB6), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rB9, sizeof(rB9), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r7B, sizeof(r7B), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r7C, sizeof(r7C), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r7F, sizeof(r7F), DEFAULT_RETRY_CNT);
	udelay(5);
	
	ret = i2c_himax_master_write(ts_modify->client, rD3, sizeof(rD3), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rF7, sizeof(rF7), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rB4, sizeof(rB4), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rD5, sizeof(rD5), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rC2, sizeof(rC2), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rC5, sizeof(rC5), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rC6, sizeof(rC6), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rCB, sizeof(rCB), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r91, sizeof(r91), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rB7, sizeof(rB7), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rCE, sizeof(rCE), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rC7, sizeof(rC7), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rCF, sizeof(rCF), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r8C, sizeof(r8C), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rDF, sizeof(rDF), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rE1, sizeof(rE1), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rEA, sizeof(rEA), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rEB, sizeof(rEB), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rEC, sizeof(rEC), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rEE, sizeof(rEE), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rEF, sizeof(rEF), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rF0, sizeof(rF0), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rF2, sizeof(rF2), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, rF4, sizeof(rF4), DEFAULT_RETRY_CNT);
	udelay(5);
	
	ret = i2c_himax_master_write(ts_modify->client, rF6, sizeof(rF6), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r53, sizeof(r53), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r54, sizeof(r54), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r56, sizeof(r56), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r57, sizeof(r57), DEFAULT_RETRY_CNT);
	udelay(5);

	ret = i2c_himax_master_write(ts_modify->client, r58, sizeof(r58), DEFAULT_RETRY_CNT);
	udelay(5);
	
	printk("[Himax]: %s -- \n",__func__);
}

static void himax_ts_loadconfig(struct himax_ts_data *ts_modify)
{
	int ret = 0;
	
	printk("[Himax]: tpid_gpio = 0x%x. \n",gpio_get_value(himax_chip->tpid_gpio));
	
	if((gpio_get_value(himax_chip->tpid_gpio) == 0))
	{
		HIMAX_DRIVER_VERSION[1] = '1';
		
		himax_ts_tpk_config(ts_modify);
	}
	else
	{
		printk("[Himax]: load wintek config. \n");
		u8 r62[] =  { 0x62, 0x02, 0x00, 0x02, 0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r63[] =  { 0x63, 0x02, 0x00, 0x12, 0x20, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r64[] =  { 0x64, 0x02, 0x00, 0x02, 0x21, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r65[] =  { 0x65, 0x00, 0x00, 0x10, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r66[] =  { 0x66, 0x00, 0x00, 0x20, 0x01, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r67[] =  { 0x67, 0x00, 0x00, 0x10, 0x02, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r68[] =  { 0x68, 0x03, 0x00, 0x43, 0x31, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r69[] =  { 0x69, 0x03, 0x00, 0x13, 0x34, 0x41, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r6A[] =  { 0x6A, 0x03, 0x00, 0x43, 0x31, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r6B[] =  { 0x6B, 0x04, 0x00, 0x14, 0x40, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r6C[] =  { 0x6C, 0x04, 0x00, 0x24, 0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r6D[] =  { 0x6D, 0x04, 0x00, 0x14, 0x42, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r6E[] =  { 0x6E, 0x04, 0x00, 0x24, 0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r6F[] =  { 0x6F, 0x04, 0x00, 0x14, 0x42, 0x21, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r70[] =  { 0x70, 0x04, 0x00, 0x24, 0x41, 0x12, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r71[] =  { 0x71, 0x04, 0x00, 0x04, 0x42, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r72[] =  { 0x72, 0x04, 0x00, 0x24, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r73[] =  { 0x73, 0x04, 0x00, 0x04, 0x42, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r74[] =  { 0x74, 0x04, 0x00, 0x24, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r75[] =  { 0x75, 0x04, 0x00, 0x04, 0x42, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r76[] =  { 0x76, 0x04, 0x00, 0x24, 0x40, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r77[] =  { 0x77, 0x03, 0x00, 0x13, 0x30, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r78[] =  { 0x78, 0x03, 0x00, 0x03, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 r79[] =  { 0x79, 0x03, 0x00, 0x03, 0x31, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00 };
		u8 rC9[] =  { 0xC9, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x59, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x2B, 0x2C, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x26, 0xCA, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x58, 0x59, 0x5A, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x16, 0x18, 0x1C, 0x1D, 0x1E, 0x1F, 0x20, 0x21, 0x22, 0x23, 0x24, 0x25, 0x26, 0x2B, 0x2C, 0x2D, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
		u8 r8A[] =  { 0x8A, 0x18, 0x02, 0xFF, 0xFF, 0x19, 0x01, 0xFF, 0xFF, 0x1A, 0x00, 0xFF, 0xFF, 0x1B, 0x36, 0xFF, 0xFF, 0x1C, 0x35, 0xFF, 0xFF, 0x1D, 0x34, 0xFF, 0xFF, 0x1E, 0xFF, 0x03, 0x17, 0x1F, 0xFF, 0x04, 0x16, 0x20, 0xFF, 0x05, 0x15, 0x21, 0xFF, 0xFF, 0x14, 0x22, 0x33, 0xFF, 0x13, 0x23, 0x32, 0xFF, 0x12, 0x24, 0x31, 0xFF, 0x11, 0x25, 0x30, 0xFF, 0x10, 0x26, 0x2F, 0xFF, 0x0F, 0xFF, 0x2E, 0xFF, 0x0E, 0xFF, 0x2D, 0xFF, 0x0D, 0xFF, 0x2C, 0xFF, 0x0C, 0xFF, 0x2B, 0xFF, 0x0B, 0xFF, 0x2A, 0xFF, 0x0A, 0xFF, 0x29, 0xFF, 0x09, 0x27, 0xFF, 0x06, 0xFF, 0xFF, 0xFF, 0x07, 0xFF, 0x28, 0xFF, 0x08, 0xFF};
		u8 rF1[] =  { 0xF1, 0x06, 0x06, 0x06, 0x10 };
		u8 rF3[] =  { 0xF3, 0x57 };
		u8 rE3[] =  { 0xE3, 0x00 };
		u8 rB6[] =  { 0xB6, 0x14 };
		u8 rB9[] =  { 0xB9, 0x01, 0x36 };
		u8 r7B[] =  { 0x7B, 0x03 };
		u8 r7C[] =  { 0x7C, 0x40, 0xD8, 0x8C };
		u8 r7F[] =  { 0x7F, 0x00, 0x04, 0x0A, 0x0A, 0x04, 0x00, 0x00, 0x00 };
		u8 rD3[] =  { 0xD3, 0x06, 0x01 };
		u8 rF7[] =  { 0xF7, 0x55, 0x55, 0x00, 0x55, 0x55, 0x55, 0x00, 0x86, 0x06, 0x00 };
		u8 rB4[] =  { 0xB4, 0x0C, 0x01, 0x01, 0x01, 0x01, 0x01, 0x0E, 0x03, 0x0F, 0x03, 0x0F, 0x03, 0x0F, 0x00 };
		u8 rD5[] =  { 0xD5, 0x65, 0x09 };
		u8 rC2[] =  { 0xC2, 0x12 };
		u8 rC5[] =  { 0xC5, 0x0C, 0x1F, 0x07, 0x10, 0x1A, 0x1F, 0x0B };
		u8 rC6[] =  { 0xC6, 0x12, 0x10, 0x19 };
		u8 rCB[] =  { 0xCB, 0x01, 0xF5, 0xFF, 0xFF, 0x01, 0x01, 0xFF, 0x02, 0xF0, 0x00, 0x00 };
		u8 r91[] =  { 0x91, 0x00 };
		u8 rB7[] =  { 0xB7, 0x10 };
		u8 rCE[] =  { 0xCE, 0x14, 0x30, 0xAB, 0x00 };
		u8 rC7[] =  { 0xC7, 0x03, 0x20, 0x05, 0x00 };
		u8 rCF[] =  { 0xCF, 0x60, 0x1F, 0x1F, 0x01, 0xE0 };
		u8 r8C[] =  { 0x8C, 0x63, 0x22, 0x04, 0x03 };
		u8 rDF[] =  { 0xDF, 0x00, 0x0F, 0x04, 0x18, 0x00 };
		u8 rE1[] =  { 0xE1, 0x20 };
		u8 rEA[] =  { 0xEA, 0x15, 0x22, 0x00, 0x16 };
		u8 rEB[] =  { 0xEB, 0x60, 0x3C, 0xFF, 0x02 };
		u8 rEC[] =  { 0xEC, 0x08, 0x20, 0x20, 0x20, 0x12, 0x00, 0x00 };
		u8 rEE[] =  { 0xEE, 0x00 };
		u8 rEF[] =  { 0xEF, 0x11, 0x00 };
		u8 rF0[] =  { 0xF0, 0xA0 };
		u8 rF2[] =  { 0xF2, 0x0A, 0x0A, 0x14, 0x3C };
		u8 rF4[] =  { 0xF4, 0x96, 0xB4, 0x2D, 0x31 };
		u8 rF6[] =  { 0xF6, 0x00, 0x00, 0x14, 0x2A, 0x0D };
		u8 r53[] =  { 0x53, 0x10, 0x0C, 0x20, 0x04 };
		u8 r54[] =  { 0x54, 0x1E, 0x20, 0x10, 0x10 };
		u8 r56[] =  { 0x56, 0x30, 0x15, 0x3C };
		u8 r57[] =  { 0x57, 0x20, 0x40, 0x01, 0x01 };
		u8 r58[] =  { 0x58, 0x04, 0x28, 0x88, 0x14 };
		
		
		printk("[Himax]: %s ++ \n",__func__);
		
		ret = i2c_himax_master_write(ts_modify->client, r62, sizeof(r62), DEFAULT_RETRY_CNT);
		udelay(5);
		
		ret = i2c_himax_master_write(ts_modify->client, r63, sizeof(r63), DEFAULT_RETRY_CNT);
		udelay(5);
		
		ret = i2c_himax_master_write(ts_modify->client, r64, sizeof(r64), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r65, sizeof(r65), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r66, sizeof(r66), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r67, sizeof(r67), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r68, sizeof(r68), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r69, sizeof(r69), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r6A, sizeof(r6A), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r6B, sizeof(r6B), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r6C, sizeof(r6C), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r6D, sizeof(r6D), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r6E, sizeof(r6E), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r6F, sizeof(r6F), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r70, sizeof(r70), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r71, sizeof(r71), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r72, sizeof(r72), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r73, sizeof(r73), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r74, sizeof(r74), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r75, sizeof(r75), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r76, sizeof(r76), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r77, sizeof(r77), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r78, sizeof(r78), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r79, sizeof(r79), DEFAULT_RETRY_CNT);
		udelay(5);
			
		ret = i2c_himax_master_write(ts_modify->client, rC9, sizeof(rC9), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r8A, sizeof(r8A), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rF1, sizeof(rF1), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rF3, sizeof(rF3), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rE3, sizeof(rE3), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rB6, sizeof(rB6), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rB9, sizeof(rB9), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r7B, sizeof(r7B), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r7C, sizeof(r7C), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r7F, sizeof(r7F), DEFAULT_RETRY_CNT);
		udelay(5);
		
		ret = i2c_himax_master_write(ts_modify->client, rD3, sizeof(rD3), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rF7, sizeof(rF7), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rB4, sizeof(rB4), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rD5, sizeof(rD5), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rC2, sizeof(rC2), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rC5, sizeof(rC5), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rC6, sizeof(rC6), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rCB, sizeof(rCB), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r91, sizeof(r91), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rB7, sizeof(rB7), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rCE, sizeof(rCE), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rC7, sizeof(rC7), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rCF, sizeof(rCF), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r8C, sizeof(r8C), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rDF, sizeof(rDF), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rE1, sizeof(rE1), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rEA, sizeof(rEA), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rEB, sizeof(rEB), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rEC, sizeof(rEC), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rEE, sizeof(rEE), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rEF, sizeof(rEF), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rF0, sizeof(rF0), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rF2, sizeof(rF2), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, rF4, sizeof(rF4), DEFAULT_RETRY_CNT);
		udelay(5);
		
		ret = i2c_himax_master_write(ts_modify->client, rF6, sizeof(rF6), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r53, sizeof(r53), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r54, sizeof(r54), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r56, sizeof(r56), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r57, sizeof(r57), DEFAULT_RETRY_CNT);
		udelay(5);
    	
		ret = i2c_himax_master_write(ts_modify->client, r58, sizeof(r58), DEFAULT_RETRY_CNT);
		udelay(5);
		
		printk("[Himax]: %s -- \n",__func__);
	}
}

static int himax_ts_poweron(struct himax_ts_data *ts_modify)
{
	uint8_t buf0[11];
	int ret = 0;
	char tp_version[4];
	char checksum_tp_version[4];

	//mutex_lock(&himax_chip->init_lock);
	wake_lock(&himax_chip->wake_lock);
	mutex_lock(&himax_chip->mutex_lock);
	//wake_lock(&himax_chip->wake_lock_init);
	memset(tp_version, 0, 4);
	memset(checksum_tp_version, 0, 4);

	//Bizzy move++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	buf0[0] = HX_CMD_TSSLPOUT;//0x81
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sleep out
	if(ret < 0) 
	{
		printk(KERN_ERR "[Himax]:i2c_master_send sleep out failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	}
	msleep(200);
	
	buf0[0] = HX_CMD_FLASH_ENABLE;//0x43
	buf0[1] = 0x01;
	buf0[2] = 0x00;
	buf0[3] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 4, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write HX_CMD_FLASH_ENABLE failed line: %d \n",__LINE__);
	    goto send_i2c_msg_fail;
	}
	udelay(5);
	
	buf0[0] = HX_CMD_FLASH_SET_ADDRESS;//0x44
	buf0[1] = 0x00;
	buf0[2] = 0x00;
	buf0[3] = 0x01;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 4, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write HX_CMD_FLASH_SET_ADDRESS failed line: %d \n",__LINE__);
	    goto send_i2c_msg_fail;
	}
	udelay(5);
	
	buf0[0] = HX_CMD_FLASH_R;//0x46
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write HX_CMD_FLASH_R failed line: %d \n",__LINE__);
	    goto send_i2c_msg_fail;
	}
	msleep(1);
		
	//ret = i2c_smbus_read_i2c_block_data(ts_modify->client, HX_CMD_FLASH_BUFFER, 4, &tp_version[0]);//0x59
	ret = i2c_himax_read(ts_modify->client, 0x59, tp_version, 4, DEFAULT_RETRY_CNT);//0x59
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_himax_read HX_CMD_FLASH_BUFFER failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	}
	else
	{
		himax_chip->version_checksum = 0;
		printk("[Himax]:tp_version=0x%x, 0x%x, 0x%x, 0x%x. \n",tp_version[0],tp_version[1],tp_version[2],tp_version[3]);
		strncpy(himax_chip->himax_version, &tp_version[0], 4);
		printk("[Himax]:touch firmware version=%s.\n",himax_chip->himax_version);
						
		checksum_tp_version[0] = tp_version[0];
		checksum_tp_version[1] = tp_version[1];
		checksum_tp_version[2] = tp_version[2];
		checksum_tp_version[3] = tp_version[3];
				
		himax_chip->version_checksum = ((checksum_tp_version[0] << 24) | (checksum_tp_version[1] << 16) |(checksum_tp_version[2] << 8) | checksum_tp_version[3]);
		printk("[Himax]:himax_chip->version_checksum=%d \n",himax_chip->version_checksum);
	}
	//Bizzy move-----------------------------------------------------------
	
	//Bizzy added++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
#ifdef ESD_WORKAROUND	
	reset_activate = 1;
#endif

	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);
	//Bizzy added---------------------------------------------------------
	
	buf0[0] = HX_CMD_MANUALMODE; //0x42
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send Reload Disable failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	}
	udelay(100);
	
	buf0[0] = HX_CMD_SETMICROOFF;//0x35
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 2, DEFAULT_RETRY_CNT);//micron off
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send Micron Off failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	}
	udelay(100);	
	
	buf0[0] = HX_CMD_SETROMRDY; //0x36
	buf0[1] = 0x0F;
	buf0[2] = 0x53;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//enable flash
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send enable flash failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	} 
	udelay(100);
	
	buf0[0] = HX_CMD_SET_CACHE_FUN;//0xDD
	buf0[1] = 0x05;
	buf0[2] = 0x03;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send prefetch failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	} 
	udelay(100);
	
	himax_ts_loadconfig(ts_modify);
	
	buf0[0] = 0xF1;
	buf0[1] = 0x06;
	buf0[2] = 0x05;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send prefetch failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	} 
	udelay(10);
	
	buf0[0] = HX_CMD_TSSON;//0x83
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
	if(ret < 0) {
		printk(KERN_ERR "[Himax]:i2c_master_send sense on failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	} 
	mdelay(200);
		
	buf0[0] = HX_CMD_TSSLPOUT;//0x81
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sleep out
	if(ret < 0) 
	{
		printk(KERN_ERR "[Himax]:i2c_master_send sleep out failed line: %d \n",__LINE__);
		goto send_i2c_msg_fail;
	}
	else
	{
		printk("[Himax]: OK slave addr=0x%x\n",ts_modify->client->addr);
	}
	mdelay(300);
	
	//Bizzy remove++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	/*buf0[0] = HX_CMD_TSSOFF; //0x82
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense off
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write HX_CMD_TSSOFF failed line: %d \n",__LINE__);
	    goto send_i2c_msg_fail;
	}
	mdelay(120); //120ms
	
	buf0[0] = HX_CMD_FLASH_ENABLE;//0x43
	buf0[1] = 0x00;
	buf0[2] = 0x00;
	buf0[3] = 0x02;
	ret = i2c_himax_master_write(ts_modify->client, buf0, 4, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write HX_CMD_FLASH_ENABLE failed line: %d \n",__LINE__);
	    goto send_i2c_msg_fail;
	}
	udelay(100);
	
	buf0[0] = HX_CMD_TSSON;//0x83
	ret = i2c_himax_master_write(ts_modify->client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
	if(ret < 0) {
	    printk(KERN_ERR "[Himax]:write HX_CMD_TSSON failed line: %d \n",__LINE__);
	    goto send_i2c_msg_fail;
	}
	msleep(300); //120ms*/
	//Bizzy remove--------------------------------------------------------------
	
	himax_chip->suspend_state = 0;
	
	//mutex_unlock(&himax_chip->init_lock);
	mutex_unlock(&himax_chip->mutex_lock);
	wake_unlock(&himax_chip->wake_lock);
	//wake_unlock(&himax_chip->wake_lock_init);
	
	return ret;

send_i2c_msg_fail:	
	printk(KERN_ERR "[Himax]:send_i2c_msg_failline: %d \n",__LINE__);
	//mutex_unlock(&himax_chip->init_lock);
	mutex_unlock(&himax_chip->mutex_lock);
	wake_unlock(&himax_chip->wake_lock);
#ifdef ENABLE_CHIP_RESET_MACHINE
	if(himax_chip->init_success){
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	}
#endif
	//wake_unlock(&himax_chip->wake_lock_init);
	
return -1;
}

#ifdef	CONFIG_PROC_FS
static ssize_t himax_chip_proc_read(struct seq_file *buf, void *v)
{
//	printk("[Himax]%s: now touch firmware:%s. \n", __func__, himax_chip->himax_version);
	seq_printf(buf,"%s\n", himax_chip->himax_version);
	return 0;
}

static ssize_t himax_chip_proc_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char messages[80] = {0};
			
	if (len >= 80) {
		printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	   
	printk("[Himax]%s:input command:%s\n", __func__, messages);
	       
	if ('b' == messages[0]) {
		printk("[Himax]%s:touch firmware update start.\n", __func__);
		himax_chip_ap_control_firmware_upgrade();
	}
	else {
		printk("[Himax]%s:command not support.\n", __func__);
	}
	
	return len;
}

static ssize_t himax_chip_proc_diag_read(struct seq_file *buf, void *v)
{
	size_t string_count = 0;
    uint32_t loop_i;
    uint16_t mutual_num, self_num, width;

    mutual_num = x_channel * y_channel;
    self_num = x_channel + y_channel;
    width = x_channel;
    seq_printf(buf, "Channel: %4d, %4d\n\n", x_channel, y_channel);

    if (diag_command >= 1 && diag_command <= 6) {
        if (diag_command < 4) {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
               seq_printf(buf, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1)) {
                    seq_printf(buf, " %3d\n", diag_self[width + loop_i/width]);
                }
            }
			seq_printf(buf, "\n");
            for (loop_i = 0; loop_i < width; loop_i++) {
                seq_printf(buf, "%4d", diag_self[loop_i]);
                if (((loop_i) % width) == (width - 1))
                {
                    seq_printf(buf, "\n");
                }
            }
        } else if (diag_command > 4) {
            for (loop_i = 0; loop_i < self_num; loop_i++) {
                seq_printf(buf, "%4d", diag_self[loop_i]);
                if (((loop_i - mutual_num) % width) == (width - 1))
                    seq_printf(buf, "\n");
            }
        } else {
            for (loop_i = 0; loop_i < mutual_num; loop_i++) {
                seq_printf(buf, "%4d", diag_mutual[loop_i]);
                if ((loop_i % width) == (width - 1))
                    seq_printf(buf, "\n");
            }
        }
    }
	
	return 0;
}

static ssize_t himax_chip_proc_diag_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	const uint8_t command_ec_128_raw_flag = 0x01;
    const uint8_t command_ec_24_normal_flag = 0xFC;
    const uint8_t command_ec_128_raw_baseline_flag = 0x02 | command_ec_128_raw_flag;
    uint8_t command_91h[2] = {0x91, 0x00};
    uint8_t command_82h[1] = {0x82};
    uint8_t command_F3h[2] = {0xF3, 0x00};
    uint8_t command_83h[1] = {0x83};
	uint8_t receive[1];
	char messages[80] = {0};
	
	if (len >= 80) {
		printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	   
	printk("[Himax]%s:input command:%s\n", __func__, messages);
		

    if ('1' == messages[0])
    {
        command_91h[1] = command_ec_128_raw_baseline_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        diag_command = messages[0] - '0';
    }
    else if ('2' == messages[0])
    {
        command_91h[1] = command_ec_128_raw_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        diag_command = messages[0] - '0';
    }
    else if ('3' == messages[0])
	{
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_82h[0], 1, &command_82h[0]);
    	msleep(120);
    	i2c_smbus_read_i2c_block_data(touch_i2c, command_F3h[0], 1, &receive[0]);
    	command_F3h[1] = (receive[0] | 0x80);
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_F3h[0], 2, &command_F3h[1]);    	
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_83h[0], 2, &command_83h[0]);
    	msleep(120);

    	command_91h[1] = command_ec_128_raw_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        diag_command = messages[0] - '0';
    }
    else
    {
        command_91h[1] = command_ec_24_normal_flag;
        i2c_smbus_write_i2c_block_data(touch_i2c, command_91h[0], 1, &command_91h[1]);
        
        i2c_smbus_write_i2c_block_data(touch_i2c, command_82h[0], 1, &command_82h[0]);
    	msleep(120);
    	i2c_smbus_read_i2c_block_data(touch_i2c, command_F3h[0], 1, &receive[0]);
    	command_F3h[1] = (receive[0] & 0x7F);
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_F3h[0], 2, &command_F3h[1]);
    	i2c_smbus_write_i2c_block_data(touch_i2c, command_83h[0], 2, &command_83h[0]);
    	msleep(120);
        diag_command = 0;
		touch_count = 0;
    }
	printk(KERN_ERR "[Himax]diag_command=0x%x\n",diag_command);
		
	return len;
}

static int proc_chip_open(struct inode *inode, struct  file *file) {
  return single_open(file, himax_chip_proc_read, NULL);
}

static const struct file_operations proc_fops = {
        .owner = THIS_MODULE,
		.open = proc_chip_open,
		.read = seq_read,
        .write = himax_chip_proc_write,
};

void himax_chip_create_proc_file(void)
{
	himax_proc_file = proc_create(HIMAX_PROC_FILE, 0666, NULL, &proc_fops);
	if(himax_proc_file){
	}
	else{
		printk(KERN_ERR "[Himax] %s: proc file create failed!\n", __func__);
	}
}

void himax_chip_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_file\n");
    remove_proc_entry(HIMAX_PROC_FILE, &proc_root);
}

static int proc_chip_diag_open(struct inode *inode, struct  file *file) {
  return single_open(file, himax_chip_proc_diag_read, NULL);
}

static const struct file_operations diag_fops = {
        .owner = THIS_MODULE,
		.open = proc_chip_diag_open,
		.read = seq_read,
        .write = himax_chip_proc_diag_write,
};

void himax_chip_create_proc_diag_file(void)
{
	himax_proc_diag_file = proc_create(HIMAX_PROC_DIAG_FILE, 0666, NULL, &diag_fops);
	if(himax_proc_diag_file){	}
	else{
		printk(KERN_ERR "[Himax] %s: proc diag file create failed!\n", __func__);
	}
}

void himax_chip_remove_proc_diag_file(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_diag_file\n");
    remove_proc_entry(HIMAX_PROC_DIAG_FILE, &proc_root);
}

// add by leo for loading config files from .txt by proc ++
#ifdef ENABLE_TEST
static ssize_t himax_chip_proc_config_read(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
    struct file* filp = NULL;
    mm_segment_t oldfs;
    size_t sprintf_count = 0; //add for sprintf
    char readfile[5]={0};
    char newline[1]={0};
    int i=0, ret=0, IsThisRegAddr=0, addr_Count=0, value_Count=0;
    char zero={0}, useless={0}, valueH={0}, valueL={0};
    char addr_str[2]={0}, temp_str[2]={0};
    u8	value=0;
    u8	r62[11]={0}, r63[11]={0}, r64[11]={0}, r65[11]={0},r66[11]={0}, r67[11]={0}, r68[11] = {0}, r69[11]={0},r6A[11]={0}, r6B[11]={0},
		r6C[11]={0}, r6D[11]={0},r6E[11]={0}, r6F[11]={0}, r70[11] = {0}, r71[11]={0}, r72[11]={0}, r73[11]={0}, r74[11]={0}, r75[11]={0},
		r76[11]={0}, r77[11]={0}, r78[11] = {0}, r79[11]={0}, rC9[129]={0}, r8A[97] = {0}, rF1[5]={0}, rF3[2]={0}, rE3[2]={0}, rB6[2]={0},
		rB9[3]={0}, r7B[2]={0}, r7C[4]={0}, r7F[9]={0},rD3[3]={0}, rF7[11]={0}, rB4[15]={0}, rD5[3]={0}, rC2[2]={0}, rC5[8]={0}, 
		rC6[4]={0}, rCB[12]={0}, r91[2]={0},rB7[2]={0}, rCE[5]={0}, rC7[5]={0}, rCF[6]={0}, r8C[5]={0}, rDF[6]={0}, rE5[2]={0},
		rEA[5]={0}, rEB[5]={0}, rEC[8]={0}, rEE[2]={0}, rEF[3]={0}, rF0[2]={0}, rF2[5]={0}, rF4[5]={0}, rF6[6]={0}, r53[4]={0},
		r54[4]={0}, r56[4]={0}, r57[5]={0}, r58[5]={0};

    u8* configs[64]={	r62,r63,r64,r65,r66,r67,r68,r69,r6A,r6B,
					r6C,r6D,r6E,r6F,r70,r71,r72,r73,r74,r75,
					r76,r77,r78,r79,rC9,r8A,rF1,rF3,rE3,rB6,
					rB9,r7B,r7C,r7F,rD3,rF7,rB4,rD5,rC2,rC5,
					rC6,rCB,r91,rB7,rCE,rC7,rCF,r8C,rDF,rE5,
					rEA,rEB,rEC,rEE,rEF,rF0,rF2,rF4,rF6,r53,
					r54,r56,r57,r58};

	//read from *.txt
	filp = filp_open("/data/local/touch_config.txt", O_RDONLY, 0);
	if(!IS_ERR_OR_NULL(filp))
	{
		oldfs = get_fs();
		set_fs(get_ds());

		addr_Count=0;
		do{
			value_Count=0;

			while(filp->f_op->read(filp,readfile,5, &filp->f_pos)){

				if(IsThisRegAddr==0){
					sprintf(addr_str,"%c%c",readfile[2],readfile[3]);
					printk(KERN_ERR "[%s]: r%s:\n", __func__,addr_str);
					IsThisRegAddr=1;
				}

				sscanf(readfile, "%c%c%c%c,", &zero, &useless, &valueH, &valueL);
				sprintf(temp_str,"%c%c",valueH,valueL);
				sscanf(temp_str, "%X,", &value);
				//printk(KERN_ERR "[%s]: value = 0x%02X\n", __func__,value);

				*(*(configs+addr_Count)+value_Count)=value;  //write config into register array
				value_Count++;

				if (readfile[4]!=','){
					//printk(KERN_ERR "[%s]: addr_Count=%d, value_Count=%d\n", __func__,addr_Count,value_Count);
					IsThisRegAddr=0;
					break;
				}
			}

			for(i=0;i<value_Count;i++){
				printk(KERN_ERR "[%s]: configs[%d][%d]=0x%02X\n", __func__,addr_Count,i,*(*(configs+addr_Count)+i));
			}

			addr_Count++;
		}while((filp->f_op->read(filp,newline,1, &filp->f_pos))&&(addr_Count<64));

		//printk(KERN_ERR "[%s]: addr_Count=%d, value_Count=%d\n", __func__,addr_Count,value_Count);
		sprintf_count += sprintf(buf + sprintf_count, "[Himax] addr_Count=%d, value_Count=%d\n",addr_Count,value_Count);

		set_fs(oldfs);
		filp_close(filp, NULL);

	}
	else
	{
		//printk(KERN_ERR "[Himax] %s: open /data/local/touch_config.txt failed\n", __func__);
		sprintf_count += sprintf(buf + sprintf_count, "[Himax] %s: open /data/local/touch_config.txt failed\n",__func__);
		return sprintf_count;
	}

	//write configurations into registers
	ret = i2c_himax_master_write(himax_chip->client, r62, sizeof(r62), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r63, sizeof(r63), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r64, sizeof(r64), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r65, sizeof(r65), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r66, sizeof(r66), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r67, sizeof(r67), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r68, sizeof(r68), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r69, sizeof(r69), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r6A, sizeof(r6A), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r6B, sizeof(r6B), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r6C, sizeof(r6C), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r6D, sizeof(r6D), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r6E, sizeof(r6E), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r6F, sizeof(r6F), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r70, sizeof(r70), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r71, sizeof(r71), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r72, sizeof(r72), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r73, sizeof(r73), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client,r74, sizeof(r74), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r75, sizeof(r75), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r76, sizeof(r76), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r77, sizeof(r77), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r78, sizeof(r78), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r79, sizeof(r79), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rC9, sizeof(rC9), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r8A, sizeof(r8A), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF1, sizeof(rF1), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF3, sizeof(rF3), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rE3, sizeof(rE3), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rB6, sizeof(rB6), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rB9, sizeof(rB9), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r7B, sizeof(r7B), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r7C, sizeof(r7C), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r7F, sizeof(r7F), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rD3, sizeof(rD3), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF7, sizeof(rF7), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rB4, sizeof(rB4), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rD5, sizeof(rD5), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rC2, sizeof(rC2), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rC5, sizeof(rC5), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rC6, sizeof(rC6), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rCB, sizeof(rCB), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r91, sizeof(r91), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rB7, sizeof(rB7), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rCE, sizeof(rCE), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rC7, sizeof(rC7), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rCF, sizeof(rCF), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r8C, sizeof(r8C), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rDF, sizeof(rDF), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rE5, sizeof(rE5), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rEA, sizeof(rEA), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rEB, sizeof(rEB), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rEC, sizeof(rEC), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rEE, sizeof(rEE), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rEF, sizeof(rEF), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF0, sizeof(rF0), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF2, sizeof(rF2), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF4, sizeof(rF4), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, rF6, sizeof(rF6), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r53, sizeof(r53), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r54, sizeof(r54), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r56, sizeof(r56), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r57, sizeof(r57), DEFAULT_RETRY_CNT);
	udelay(5);
	ret = i2c_himax_master_write(himax_chip->client, r58, sizeof(r58), DEFAULT_RETRY_CNT);
	udelay(5);

	printk(KERN_ERR "[Himax] %s - Finished\n", __func__);
	sprintf_count += sprintf(buf + sprintf_count, "[Himax] write cofigurations finished\n",__func__);
	//return sprintf_count;
	return 0;
}

static int proc_chip_config_open(struct inode *inode, struct  file *file) {
  return single_open(file, himax_chip_proc_config_read, NULL);
}

static const struct file_operations proc_config_fops = {
        .owner = THIS_MODULE,
		.open = proc_chip_config_open,
		.read = seq_read,
};

void himax_chip_create_proc_config_file(void)
{
	himax_proc_config_file = proc_create(HIMAX_PROC_CONFIG_FILE, 0666, NULL, &proc_config_fops);
	if(himax_proc_config_file){
	}
	else{
		printk(KERN_ERR "[Himax] %s: proc config file create failed!\n", __func__);
	}
}
void himax_chip_remove_proc_config_file(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_config_file\n");
    remove_proc_entry(HIMAX_PROC_CONFIG_FILE, &proc_root);
}

// add by leo for loading config files from .txt by proc --

// add by Josh for Read/Write register by proc 2013/06/04 ++
static ssize_t himax_chip_proc_register_read(struct seq_file *buf, void *v)
{
    int ret = 0;
    uint8_t udata[96] = { 0 }, loop_i;

    printk(KERN_INFO "[Himax] %s:himax_command=%x\n", __func__, himax_command);

    if (i2c_smbus_read_i2c_block_data(touch_i2c, himax_command, 96, &udata[0]) < 0) {
        printk(KERN_WARNING "[Himax] %s: read fail\n", __func__);
        return 0;
    }

    seq_printf(buf, "command: %x\n", himax_command);
    for (loop_i = 0; loop_i < 96; loop_i++) {
       seq_printf(buf, "0x%2.2X ", udata[loop_i]);
        if ((loop_i % 16) == 15)
            seq_printf(buf, "\n");
    }
    seq_printf(buf, "\n");
    return 0;
}
static ssize_t himax_chip_proc_register_write(struct file *filp, const char *buf, unsigned long len, void *data)
{
    char buf_tmp[6], length = 0;
    uint8_t veriLen = 0;
    uint8_t write_da[100];
    unsigned long result = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
	
    if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
        if (buf[2] == 'x') {
            uint8_t loop_i;
            uint16_t base = 5;
            memcpy(buf_tmp, buf + 3, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
                himax_command = result;
            for (loop_i = 0; loop_i < 100; loop_i++) {
                if (buf[base] == '\n') {
                    if (buf[0] == 'w')
                        i2c_smbus_write_i2c_block_data(touch_i2c, himax_command, length, &write_da[0]);
                    printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
                        write_da[0], length);
                    for (veriLen = 0; veriLen < length; veriLen++)
                        printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));

                    printk(KERN_INFO "\n");
                    return len;
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
    return len;
}

static int proc_chip_register_open(struct inode *inode, struct  file *file) {
  return single_open(file, himax_chip_proc_register_read, NULL);
}

static const struct file_operations register_fops = {
        .owner = THIS_MODULE,
		.open = proc_chip_register_open,
		.read = seq_read,
        .write = himax_chip_proc_register_write,
};

void himax_chip_create_proc_register(void)
{
	himax_proc_register = proc_create(HIMAX_PROC_REGISTER, 0666, NULL, &register_fops);
	if(himax_proc_register){
	}
	else{
		printk(KERN_ERR "[Himax] %s: proc config file create failed!\n", __func__);
	}
}

void himax_chip_remove_proc_register(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_register\n");
    remove_proc_entry(HIMAX_PROC_REGISTER, &proc_root);
}

// add by Josh for Read/Write register by proc 2013/06/04 --
#endif
// add by leo for loading config files from .txt by proc --

//add by Josh for self_test poweron & fw_upgrade by proc 2013/07/15 ++

static ssize_t himax_chip_proc_self_test(struct seq_file *buf, void *v)
{
	int val=0x00;
	uint8_t returndata[16];
	
	val = himax_chip_self_test(returndata);
	if(val == 0)
	{
		seq_printf(buf, "%d\n",val);
		return 0;
	}
	else
	{
		seq_printf(buf, "%d, Error code= 0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x,0x%x\n"
		,val, returndata[0],returndata[1],returndata[2],returndata[3],returndata[4],returndata[5],returndata[6],returndata[7],returndata[8],returndata[9],returndata[10],returndata[11],returndata[12],returndata[13],returndata[14],returndata[15]);
		return 0;
	}
}

static ssize_t himax_chip_proc_self_test_setting(struct file *filp, const char *buf, unsigned long len, void *data)
{
    char buf_tmp[6], length = 0;
    uint8_t veriLen = 0;
    uint8_t write_da[100];
    unsigned long result = 0;

    memset(buf_tmp, 0x0, sizeof(buf_tmp));
    memset(write_da, 0x0, sizeof(write_da));
	
    if ((buf[0] == 'r' || buf[0] == 'w') && buf[1] == ':') {
        if (buf[2] == 'x') {
            uint8_t loop_i;
            uint16_t base = 5;
            memcpy(buf_tmp, buf + 3, 2);
            if (!strict_strtoul(buf_tmp, 16, &result))
                himax_command = result;
            for (loop_i = 0; loop_i < 100; loop_i++) {
                if (buf[base] == '\n') {
                    if (buf[0] == 'w')

                    printk(KERN_INFO "CMD: %x, %x, %d\n", himax_command,
                        write_da[0], length);
                    for (veriLen = 0; veriLen < length; veriLen++){
                        printk(KERN_INFO "%x ", *((&write_da[0])+veriLen));
						r8D_setting[veriLen] = *((&write_da[0])+veriLen);
						printk(KERN_INFO "r8D_setting[%d] : %x \n",veriLen ,r8D_setting[veriLen]);
						}

                    printk(KERN_INFO "\n");
                    return len;
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
	
	return len;
}

static int proc_chip_self_test_open(struct inode *inode, struct  file *file) {
  return single_open(file, himax_chip_proc_self_test, NULL);
}

static const struct file_operations self_test_fops = {
        .owner = THIS_MODULE,
		.open = proc_chip_self_test_open,
		.read = seq_read,
        .write = himax_chip_proc_self_test_setting,
};

void himax_chip_create_proc_self_test(void)
{
	himax_proc_self_test = proc_create(HIMAX_PROC_SELF_TEST, 0666, NULL, &self_test_fops);
	if(himax_proc_self_test){
	}
	else{
		printk(KERN_ERR "[Himax] %s: proc config file create failed!\n", __func__);
	}
}

void himax_chip_remove_proc_self_test(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_self_test\n");
    remove_proc_entry(HIMAX_PROC_SELF_TEST, &proc_root);
}

static ssize_t himax_chip_proc_poweron(struct seq_file *buf, void *v)
{
	int len = 0;
	printk(KERN_INFO "[Himax]: adb cat himax_chip_poweron by proc\n");

	mutex_lock(&himax_chip->mutex_lock);
#ifdef ESD_WORKAROUND
	reset_activate = 1;
#endif
	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);
	mutex_unlock(&himax_chip->mutex_lock);
	himax_ts_poweron(himax_chip);

	printk(KERN_INFO "[Himax]: adb cat himax_chip_poweron done by proc.\n");
	seq_printf(buf, "%d \n", len);
	return 0;

}

static ssize_t himax_chip_proc_fw_upgrade(struct file *filep, const char *buf, unsigned long len, void *data)
{
#ifdef HX_TP_FW_UPDATE
    struct file* filp = NULL;
    mm_segment_t oldfs;
    int result = 0;
#endif
	int err = 0;
	char fileName[128];
	
#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);
#endif
	wake_lock(&himax_chip->wake_lock);
	//wake_lock(&himax_chip->wake_lock_fw);
	
	himax_chip->tp_firmware_upgrade_proceed = 1;
	printk(KERN_INFO "[Himax]: himax_chip_firmware_upgrade suspend_state=%x\n",himax_chip->suspend_state);
	if(!himax_chip->suspend_state)
	{
		printk("[Himax] himax chip entry suspend for firmware upgrade \n");
		himax_ts_suspend_command(himax_chip->client);
		disable_irq(himax_chip->irq);
		err = cancel_work_sync(&himax_chip->work);
		if(err)
			enable_irq(himax_chip->irq);
		
		himax_chip->suspend_state = 1;
		msleep(300);
	}	
	
    if (buf[0] >= '0' && buf[0] <= '9' && buf[1] == '\n')
        debug_log_level = buf[0] - '0';
	
	mutex_lock(&himax_chip->mutex_lock);
#ifdef HX_TP_FW_UPDATE
    if(buf[0] == 'b')
    {
        printk(KERN_INFO "[Himax] %s: upgrade firmware from file start!\n", __func__);
        printk("[Himax] %s: file open:/system/etc/firmware/touch_fw.bin \n",__func__);
        filp = filp_open("/system/etc/firmware/touch_fw.bin", O_RDONLY, 0);
        if(IS_ERR(filp)) {
            printk(KERN_ERR "[Himax] %s: open firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }
        oldfs = get_fs();
        set_fs(get_ds());

        result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
        if(result < 0) {
            printk(KERN_ERR "[Himax] %s: read firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }

        set_fs(oldfs);
        filp_close(filp, NULL);

        printk("[Himax] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
        printk("[Himax] %s: upgrade firmware verison, %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);
        if(result > 0)
        {
            if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
                printk(KERN_INFO "[Himax] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
            else
                printk(KERN_INFO "[Himax] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
            
            goto firmware_upgrade_done;
            //return count;                    
        }
    }
	else if(buf[0] == 't')
    {
        printk(KERN_INFO "[Himax] %s: upgrade firmware from file start!\n", __func__);

		memset(fileName, 0, 128);
		// parse the file name
		snprintf(fileName, len-2, "%s", &buf[2]);
		printk(KERN_INFO "[Himax] %s: upgrade from file(%s) start!\n", __func__, fileName);
		// open file
		filp = filp_open(fileName, O_RDONLY, 0);
		
        if(IS_ERR(filp)) {
            printk(KERN_ERR "[Himax] %s: open firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }
        oldfs = get_fs();
        set_fs(get_ds());

        result=filp->f_op->read(filp,upgrade_fw,sizeof(upgrade_fw), &filp->f_pos);
        if(result < 0) {
            printk(KERN_ERR "[Himax] %s: read firmware file failed\n", __func__);
            goto firmware_upgrade_done;
            //return count;
        }

        set_fs(oldfs);
        filp_close(filp, NULL);

        printk("[Himax] %s: upgrade start,len %d: %02X, %02X, %02X, %02X\n", __func__, result, upgrade_fw[0], upgrade_fw[1], upgrade_fw[2], upgrade_fw[3]);
        printk("[Himax] %s: upgrade firmware verison, %02X, %02X, %02X, %02X\n", __func__, upgrade_fw[0x1000], upgrade_fw[0x1001], upgrade_fw[0x1002], upgrade_fw[0x1003]);

        if(result > 0)
        {
            if(fts_ctpm_fw_upgrade_with_sys_fs(upgrade_fw, result) == 0)
                printk(KERN_INFO "[Himax] %s: TP upgrade error, line: %d\n", __func__, __LINE__);
            else
                printk(KERN_INFO "[Himax] %s: TP upgrade OK, line: %d\n", __func__, __LINE__);
            
            goto firmware_upgrade_done;
            //return count;                    
        }
    }
    else if(buf[0] == 'f') 
    {
        printk(KERN_INFO "[Himax] %s: upgrade firmware from kernel image start!\n", __func__);
        if (isTP_Updated == 0)
        {
            printk("Himax touch isTP_Updated: %d\n", isTP_Updated);
            if(1)// (himax_read_FW_ver() == 0)
            {
                printk("Himax touch firmware upgrade: %d\n", isTP_Updated);
                if(fts_ctpm_fw_upgrade_with_i_file() == 0)
                    printk("himax_marked TP upgrade error, line: %d\n", __LINE__);
                else
                    printk("himax_marked TP upgrade OK, line: %d\n", __LINE__);
                isTP_Updated = 1;
            }
        }
    }
#endif
    
firmware_upgrade_done:
	mutex_unlock(&himax_chip->mutex_lock);
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
	himax_chip->tp_firmware_upgrade_proceed = 0;
	himax_chip->suspend_state = 0;
	enable_irq(himax_chip->irq);
	wake_unlock(&himax_chip->wake_lock);
	//wake_unlock(&himax_chip->wake_lock_fw);
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ);
#endif
	return len;
}

static int proc_chip_fw_upgrade_open(struct inode *inode, struct  file *file) {
  return single_open(file, himax_chip_proc_poweron, NULL);
}

static const struct file_operations fw_upgrade_fops = {
        .owner = THIS_MODULE,
		.open = proc_chip_fw_upgrade_open,
		.read = seq_read,
        .write = himax_chip_proc_fw_upgrade,
};

void himax_chip_create_proc_poweron_and_fw_upgrade(void)
{
	himax_proc_poweron_and_fw_upgrade = proc_create(HIMAX_PROC_POWERON_AND_FW_UPGRADE, 0666, NULL, &fw_upgrade_fops);
	if(himax_proc_poweron_and_fw_upgrade){
	}
	else{
		printk(KERN_ERR "[Himax] %s: proc config file create failed!\n", __func__);
	}
}

void himax_chip_remove_proc_poweron_and_fw_upgrade(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_poweron_and_fw_upgrade\n");
    remove_proc_entry(HIMAX_PROC_POWERON_AND_FW_UPGRADE, &proc_root);
}

//add by Josh for self_testpoweron & fw_upgrade by proc 2013/07/15 --

static ssize_t himax_chip_proc_debug_flag(struct file *filp, const char *buf, unsigned long len, void *data)
{
	himax_debug_flag = buf[0];
	himax_debug_flag= himax_debug_flag -48;
	printk(KERN_ERR "[Himax] %s: proc himax_debug_flag: %d\n", __func__,himax_debug_flag);
	return len;
}

static const struct file_operations debug_flag_fops = {
        .owner = THIS_MODULE,
        .write = himax_chip_proc_debug_flag,
};

void himax_chip_create_proc_debug_flag(void)
{
	himax_proc_debug_flag = proc_create(HIMAX_PROC_DEBUG_FLAG, 0666, NULL,&debug_flag_fops);
	if(himax_proc_debug_flag){
	}
	else{
		printk(KERN_ERR "[Himax] %s: proc config file create failed!\n", __func__);
	}
}

void himax_chip_remove_proc_debug_flag(void)
{
    extern struct proc_dir_entry proc_root;
    printk(KERN_ERR "[Himax]:himax_chip_remove_proc_debug_flag\n");
    remove_proc_entry(HIMAX_PROC_DEBUG_FLAG, &proc_root);
}
#endif

// add by Josh for touch switch read & write ++
static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	printk("[Himax]:touch firmware version=%s.\n",himax_chip->himax_version);
	return sprintf(buf, "%s-%s-%s\n", himax_chip->himax_version, HIMAX_DRIVER_VERSION, ASUS_DRIVER_VERSION);
}

static ssize_t touch_switch_state(struct switch_dev *sdev, char *buf)
{
	himax_chip->proximity_status = (himax_chip->proximity_status +1) % 2;
	//printk(KERN_ERR "[Himax] %s: proximity_status: %d \n", __func__, himax_chip->proximity_status);
	return sprintf(buf, "%d\n", himax_chip->proximity_status);
}

static void touch_switch_status_report(void){
	printk("[ITE]: %s \n",__func__);
	
}
// add by Josh for touch switch read & write --

static int himax_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int err = 0;
    struct himax_i2c_platform_data *pdata;
    uint8_t tp_id0, tp_id1, tp_id2;

	// add by josh for skip COS/POS ++
    if(entry_mode==4) {
        printk("[Himax] In COS, skip\n");
        return;
    }else if(entry_mode==3) {
        printk("[Himax] In POS, skip\n");
        return;
    }
    //add by josh for skip COS/POS --
	
#ifdef ESD_WORKAROUND	
		reset_activate = 0;
#endif
  	
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        printk(KERN_ERR "[Himax] %s: i2c check functionality error\n", __func__);
        err = -ENODEV;
        goto err_check_functionality_failed;
    }

	himax_chip = kzalloc(sizeof (struct himax_ts_data), GFP_KERNEL);
	if (!himax_chip) {
		printk(KERN_ERR "[Himax] %s: himax_chip allocate himax_ts_data failed\n", __func__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}
	
	//----[ HX_TP_SYS_FLASH_DUMP ]------------------------------------------------------------------------start
				#ifdef  HX_TP_SYS_FLASH_DUMP
				himax_chip->flash_wq = create_singlethread_workqueue("himax_flash_wq");
				if (!himax_chip->flash_wq) 
				{
					printk(KERN_ERR "[HIMAX TP ERROR] %s: create flash workqueue failed\n", __func__);
					err = -ENOMEM;
					goto err_create_wq_failed;
				}
				#endif
			//----[ HX_TP_SYS_FLASH_DUMP ]--------------------------------------------------------------------------end
	
    himax_chip->himax_wq = create_singlethread_workqueue("himax_wq");
    if (!himax_chip->himax_wq) {
        printk(KERN_ERR "[Himax] %s: create workqueue failed\n", __func__);
        err = -ENOMEM;
        goto err_create_wq_failed;
    }
	
//	intel_scu_ipc_iowrite8(INTEL_MSIC_GPIO1HV1CTLO, 0x1d);
//	intel_scu_ipc_iowrite8(INTEL_MSIC_GPIO1HV2CTLO, 0x1d);
//	intel_scu_ipc_iowrite8(INTEL_MSIC_GPIO1HV3CTLO, 0x1d);
	
	//----[HX_TP_SYS_FLASH_DUMP]--------------------------------------------------------------------------start	
				#ifdef HX_TP_SYS_FLASH_DUMP
				INIT_WORK(&himax_chip->flash_work, himax_ts_flash_work_func);
				#endif
			//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------end	
	
    INIT_WORK(&himax_chip->work, himax_ts_work_func);
#ifdef ENABLE_CHIP_RESET_MACHINE
    INIT_DELAYED_WORK(&himax_chip->himax_chip_reset_work, himax_chip_reset_function);
#endif
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	printk("[Himax]:os entry_mode = %d, factory_mode =%d \n", entry_mode, factory_mode);
	if((factory_mode == 2) && (entry_mode == 1) /*&& (Read_HW_ID()!= HW_ID_EVB) && (Read_HW_ID()!= HW_ID_SR1)*/)
	{
		INIT_DELAYED_WORK(&himax_chip->himax_chip_firmware_upgrade_work, himax_chip_self_firmware_upgrade);
	}
#endif
#ifdef ENABLE_CHIP_STATUS_MONITOR
	INIT_DELAYED_WORK(&himax_chip->himax_chip_monitor, himax_chip_monitor_function); //for ESD solution
#endif
	
    himax_chip->client = client;
    i2c_set_clientdata(client, himax_chip);
    pdata = client->dev.platform_data;

	//mutex_init(&himax_chip->init_lock);
	mutex_init(&himax_chip->mutex_lock);
	
    sema_init(&pSem, 1);
			
    himax_chip->status = 1; // set I2C status is OK;
    himax_chip->abs_x_max	= pdata->abs_x_max;
	himax_chip->abs_y_max	= pdata->abs_y_max;
	himax_chip->intr_gpio	= pdata->intr_gpio;
	himax_chip->rst_gpio	= pdata->rst_gpio;
	himax_chip->formal_lens = 0xAA;
	memset(himax_chip->himax_version, 0, 32);
	himax_chip->old_tp_version = 0x00;
	himax_chip->retry_time = 0;
	himax_chip->init_success = 0;
	// +++++++ add by Josh for AP update touch fw ++++++
	himax_chip->AP_update = 0; 
	himax_chip->AP_progress = 0; 
	himax_chip->proximity_status = 0;
	// ------ add by Josh for AP update touch fw ------
#ifdef ENABLE_CHIP_STATUS_MONITOR
	himax_chip->running_status = 0;
#endif
	    
    wake_lock_init(&himax_chip->wake_lock, WAKE_LOCK_SUSPEND, "himax_touch_wake_lock");

    himax_chip->input_dev = input_allocate_device();
    if (himax_chip->input_dev == NULL) {
        err = -ENOMEM;
        dev_err(&client->dev, "[Himax] Failed to allocate input device\n");
        goto err_input_dev_alloc_failed;
    }
    
    snprintf(himax_chip->touch_phys_name, sizeof(himax_chip->touch_phys_name), "%s/input0", dev_name(&client->dev));
    
    himax_chip->input_dev->name = "himax-touchscreen"; 
    himax_chip->input_dev->phys = himax_chip->touch_phys_name;
    himax_chip->input_dev->id.bustype = BUS_I2C;
	himax_chip->input_dev->dev.parent = &client->dev;
	/*TODO: check data structure */
	printk("[Himax]:himax_chip->input_dev->name=%s\n", himax_chip->input_dev->name);
	printk("[Himax]:himax_chip->input_dev->phys=%s\n", himax_chip->input_dev->phys);
    printk("[Himax]:Max X=%d, Max Y=%d\n", himax_chip->abs_x_max, himax_chip->abs_y_max);

	/*************IO control setting***************/
	err = misc_register(&ite_misc_dev);
	if (err < 0) {
		printk( "[ITE]:%s: could not register ITE misc device\n",__func__);
		goto probe_misc_device_failed;
	}
	/*************IO control setting***************/

    //Joe ++
	/*init INTERRUPT pin*/
	err = gpio_request(himax_chip->intr_gpio, "HimaxTouch-irq");
	if(err < 0)
		printk(KERN_ERR "Failed to request GPIO%d (HimaxTouch-interrupt) error=%d\n",
			himax_chip->intr_gpio, err);

	err = gpio_direction_input(himax_chip->intr_gpio);
	if (err){
		printk(KERN_ERR "Failed to set interrupt direction, error=%d\n", err);
		gpio_free(himax_chip->intr_gpio);
	}
	
	himax_chip->irq = gpio_to_irq(himax_chip->intr_gpio);
	printk("[Himax] intr_gpio=%d, rst_gpio=%d, irq=%d \n", himax_chip->intr_gpio, himax_chip->rst_gpio, himax_chip->irq);
	
	/*init RESET pin*/
	err = gpio_request(himax_chip->rst_gpio, "HimaxTouch-reset");
	if (err < 0)
		printk(KERN_ERR "Failed to request GPIO%d (HimaxTouch-reset) error=%d\n", himax_chip->rst_gpio, err);

	err = gpio_direction_output(himax_chip->rst_gpio, 1);
	if (err){
		printk(KERN_ERR "Failed to set reset direction, error=%d\n", err);
		gpio_free(himax_chip->rst_gpio);
	}
	//Joe -- 
	
	//add by Josh ++
	himax_chip->tpid_gpio = HIMAX_TPID_GPIO;
	err = gpio_request(himax_chip->tpid_gpio, "TP_ID");
	if (err < 0)
		printk(KERN_ERR "Failed to request GPIO%d (HimaxTouch-TP_ID) error=%d\n", himax_chip->tpid_gpio, err);

	err = gpio_direction_input(himax_chip->tpid_gpio);
	if (err){
		printk(KERN_ERR "Failed to set TP_ID direction, error=%d\n", err);
		gpio_free(himax_chip->tpid_gpio);
	}
	printk("[Himax]tpid_gpio %d \n",gpio_get_value(himax_chip->tpid_gpio));
	//add by Josh --
	
	//switch add by josh for Version info ++
	himax_chip->touch_sdev.name = TOUCH_SDEV_NAME;
	himax_chip->touch_sdev.print_name = touch_switch_name;
	himax_chip->touch_sdev.print_state = touch_switch_state;
	if(switch_dev_register(&himax_chip->touch_sdev) < 0){
		printk("switch_dev_register for win8_power failed!\n");
	}
	//switch add by josh for Version info --
	
    __set_bit(EV_KEY, himax_chip->input_dev->evbit);
    __set_bit(EV_ABS, himax_chip->input_dev->evbit);
    __set_bit(EV_SYN, himax_chip->input_dev->evbit);
    __set_bit(BTN_TOUCH, himax_chip->input_dev->keybit);

    input_set_abs_params(himax_chip->input_dev, ABS_MT_TRACKING_ID, 0, HX_TP_MAX_FINGER, 0, 0);
    input_set_abs_params(himax_chip->input_dev, ABS_MT_POSITION_X, 0, DEFAUULT_X_RES, 0, 0);
    input_set_abs_params(himax_chip->input_dev, ABS_MT_POSITION_Y, 0, DEFAUULT_Y_RES, 0, 0);
    input_set_abs_params(himax_chip->input_dev, ABS_MT_TOUCH_MAJOR, 0, 31, 0, 0); //Finger Size
    input_set_abs_params(himax_chip->input_dev, ABS_MT_WIDTH_MAJOR, 0, 31, 0, 0); //Touch Size
    input_set_abs_params(himax_chip->input_dev, ABS_MT_PRESSURE, 0, 255, 0, 0);
	
	input_set_drvdata(himax_chip->input_dev, himax_chip);
	
    err = input_register_device(himax_chip->input_dev);
    if (err) {
        dev_err(&client->dev,
            "[Himax]%s: unable to register %s input device\n",
            __func__, himax_chip->input_dev->name);
        goto err_input_register_device_failed;
    }

    touch_i2c = client;

#if ChangeIref1u
	ChangeIrefSPP();
	msleep(20);
#ifdef ESD_WORKAROUND
	reset_activate = 1;
#endif
	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);
#endif 

	#ifdef ESD_WORKAROUND
	reset_activate = 1;
	#endif

	//reset chip
	gpio_set_value(himax_chip->rst_gpio, 0);
	msleep(30);
	gpio_set_value(himax_chip->rst_gpio, 1);
	msleep(30);

    // Himax: touch screen power on sequence
    himax_chip->tp_status = 1;
	err = himax_ts_poweron(himax_chip);
	if(err==-1)
	{
		printk(KERN_ERR "[Himax]:power on error=%d.\n",err);
		himax_chip->tp_status = 0;
		goto err_input_register_device_failed;
	}
    
#ifdef HX_TP_SYS_FS
    // Himax: register sysfs node for reading raw data
    
    //setI2CDev(client);
	setXChannel(21); // X channel
   	setYChannel(34); // Y channel
	printk("[Himax]:x_channel=%d, y_channel=%d.\n", x_channel, y_channel);
	
	//----[HX_TP_SYS_FLASH_DUMP]--------------------------------------------------------------------------start
				#ifdef HX_TP_SYS_FLASH_DUMP
				setSysOperation(0);
				setFlashBuffer();
				#endif
			//----[HX_TP_SYS_FLASH_DUMP]----------------------------------------------------------------------------end
	
    setMutualBuffer();
    if (getMutualBuffer() == NULL) {
       printk(KERN_ERR "[Himax] %s: mutual buffer allocate fail failed\n", __func__);
       return -1; 
    }

    himax_touch_sysfs_init();

    himax_chip->attrs.attrs = himax_attr;
    err = sysfs_create_group(&client->dev.kobj, &himax_chip->attrs);
    if (err) {
        dev_err(&client->dev, "[Himax] %s: Not able to create the sysfs\n", __func__);
    }
#endif
		
    himax_ts_register_interrupt(himax_chip->client);

    if (gpio_get_value(himax_chip->intr_gpio) == 0) {
        printk(KERN_INFO "[Himax]%s: handle missed interrupt\n", __func__);
        himax_ts_irq_handler(himax_chip->irq, himax_chip);
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    himax_chip->early_suspend.level = EARLY_SUSPEND_LEVEL_STOP_DRAWING - 1;
    himax_chip->early_suspend.suspend = himax_ts_early_suspend;
    himax_chip->early_suspend.resume = himax_ts_late_resume;
    register_early_suspend(&himax_chip->early_suspend);
#endif

    dev_info(&client->dev, "[Himax] Start touchscreen %s in interrupt mode\n", himax_chip->input_dev->name);
        
#ifdef	CONFIG_PROC_FS
    himax_chip_create_proc_file();
    himax_chip_create_proc_diag_file();
    himax_chip_create_proc_config_file(); // add by leo for loading config files from .txt by proc
	himax_chip_create_proc_register(); // add by Josh for Read/Write register by proc 2013/06/04
	himax_chip_create_proc_self_test(); // add by Josh for self_test by proc 2013/07/15
	himax_chip_create_proc_poweron_and_fw_upgrade(); // add by Josh for poweron and fw_upgrade by proc 2013/07/15
	himax_chip_create_proc_debug_flag();
#endif
	himax_chip->init_success = 1;
	printk("[Himax]: Himax H8528-C58 Touch driver ver.%s \n",ASUS_DRIVER_VERSION);
	printk("[Himax]: Himax H8528-C58 Himax driver ver.%s \n",HIMAX_DRIVER_VERSION);

#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	if((factory_mode == 2) && (entry_mode == 1) /*&& (Read_HW_ID()!= HW_ID_EVB) && (Read_HW_ID()!= HW_ID_SR1)*/)
	{
		queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_firmware_upgrade_work, 10*HZ);
	}
#endif

	/*cable_status = check_cable_status();
	hx8528_cable_status(cable_status);*/
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 45*HZ);	//for ESD solution
#endif
	
	if(gpio_get_value(himax_chip->intr_gpio) == 0)
	{
		printk("[Himax]%s: IRQ = 0, Enable IRQ\n", __func__); 
		enable_irq(himax_chip->irq);
	}
    return 0;
	
probe_misc_device_failed:
err_input_register_device_failed:
	gpio_free(himax_chip->intr_gpio);
	gpio_free(himax_chip->rst_gpio);
	
    if (himax_chip->input_dev)
        input_free_device(himax_chip->input_dev);

err_input_dev_alloc_failed:
//err_detect_failed:
	//mutex_destroy(&himax_chip->init_lock);
	mutex_destroy(&himax_chip->mutex_lock);
	wake_lock_destroy(&himax_chip->wake_lock);
	//wake_lock_destroy(&himax_chip->wake_lock_init);
	//wake_lock_destroy(&himax_chip->wake_lock_fw);

#ifdef ENABLE_CHIP_RESET_MACHINE
	cancel_delayed_work(&himax_chip->himax_chip_reset_work);
#endif
#ifdef ENABLE_SELF_FIRMWARE_UPGRADE
	if((factory_mode == 2) && (entry_mode == 1) /*&& (Read_HW_ID()!= HW_ID_EVB) && (Read_HW_ID()!= HW_ID_SR1)*/)
	{
			cancel_delayed_work(&himax_chip->himax_chip_firmware_upgrade_work);
	}
#endif
#ifdef ENABLE_CHIP_STATUS_MONITOR
	cancel_delayed_work(&himax_chip->himax_chip_monitor);
#endif
		
    if (himax_chip->himax_wq)
        destroy_workqueue(himax_chip->himax_wq);

err_create_wq_failed:
    kfree(himax_chip);

err_alloc_data_failed:
err_check_functionality_failed:
	himax_chip->init_success = 0;
    return err;
}

static int himax_ts_remove(struct i2c_client *client)
{
    struct himax_ts_data *ts = i2c_get_clientdata(client);

#ifdef HX_TP_SYS_FS
    himax_touch_sysfs_deinit();
#endif

    unregister_early_suspend(&ts->early_suspend);
    free_irq(ts->irq, ts);
	
	gpio_free(himax_chip->intr_gpio);
	gpio_free(himax_chip->rst_gpio);
	
	//mutex_destroy(&himax_chip->init_lock);
	mutex_destroy(&himax_chip->mutex_lock);
	
    if (ts->himax_wq)
        destroy_workqueue(ts->himax_wq);
    
    input_unregister_device(ts->input_dev);
    wake_lock_destroy(&himax_chip->wake_lock);
//	wake_lock_destroy(&himax_chip->wake_lock_init);
//	wake_lock_destroy(&himax_chip->wake_lock_fw);

#ifdef	CONFIG_PROC_FS
    himax_chip_remove_proc_file();
    himax_chip_remove_proc_diag_file();
    himax_chip_remove_proc_config_file(); // add by leo for loading config files from .txt by proc
	himax_chip_remove_proc_register(); // add by Josh for Read/Write register by proc 2013/06/04 
	himax_chip_remove_proc_self_test(); // add by Josh for self_test by proc 2013/07/15
	himax_chip_remove_proc_poweron_and_fw_upgrade(); //add by Josh for poweron and fw_upgrade by proc 2013/07/15
	himax_chip_remove_proc_debug_flag();
#endif

    kfree(ts);

    return 0;
}

static int himax_ts_suspend_command(struct i2c_client *client)
{
	uint8_t buf[2] = {0};
	int ret = 0;
	
	printk(KERN_INFO "[Himax] himax_ts_suspend_command \n");
	
	wake_lock(&himax_chip->wake_lock);
	mutex_lock(&himax_chip->mutex_lock);
	
	buf[0] = HX_CMD_TSSOFF;
	ret = i2c_himax_master_write(client, buf, 1, DEFAULT_RETRY_CNT);
	if(ret < 0) {
	   printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSOFF line: %d\n", __func__, __LINE__);
	   goto send_i2c_msg_fail;
	} 
	msleep(120);
	
	buf[0] = HX_CMD_TSSLPIN;
	ret = i2c_himax_master_write(client, buf, 1, DEFAULT_RETRY_CNT);
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSLPIN line: %d\n", __func__, __LINE__);
		goto send_i2c_msg_fail;
	} 
	msleep(120);
	
	mutex_unlock(&himax_chip->mutex_lock);
	wake_unlock(&himax_chip->wake_lock);
	return 0;

	send_i2c_msg_fail:
	
	mutex_unlock(&himax_chip->mutex_lock);
	wake_unlock(&himax_chip->wake_lock);
	return -1;
}

static int himax_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int ret = 0;

    printk(KERN_INFO "[Himax] %s: TS suspend, himax_chip->suspend_state = %d\n", __func__, himax_chip->suspend_state);
    
    
    #ifdef HX_TP_SYS_FLASH_DUMP
			if(getFlashDumpGoing())
			{
				printk(KERN_INFO "[himax] %s: Flash dump is going, reject suspend\n",__func__);
				return 0;
			}
			#endif
    
    if(himax_chip->suspend_state == 0)
    {
    	if(!himax_chip->tp_firmware_upgrade_proceed)
		{
			printk(KERN_INFO "[Himax] %s: TS suspend - Start \n", __func__);
		#ifdef ENABLE_CHIP_STATUS_MONITOR
			himax_chip->running_status = 1;
		#endif
						
			disable_irq(himax_chip->irq);
		#ifdef ENABLE_CHIP_STATUS_MONITOR
			cancel_delayed_work_sync(&himax_chip->himax_chip_monitor);	//for ESD solution
		#endif
		
			ret = cancel_work_sync(&himax_chip->work);
			if (ret)
			{	
				printk(KERN_INFO "[Himax] %s: Enable IRQ \n", __func__);
				enable_irq(himax_chip->irq);
			}
						
			himax_ts_suspend_command(himax_chip->client);
			
			himax_chip->suspend_state = 1;
			touch_count = 0;
			reset_count = 0;
		}
	}
	
    return 0;
}

static void himax_ts_resume_command_all(struct i2c_client *client)
{
	uint8_t buf0[4] = {0};
	int ret = 0;
	
	wake_lock(&himax_chip->wake_lock);
	
	buf0[0] = HX_CMD_MANUALMODE;//0x42
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(client, buf0, 2, DEFAULT_RETRY_CNT);//Reload Disable
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_MANUALMODE line: %d \n", __func__, __LINE__);
	}
	udelay(100);
	
	buf0[0] = HX_CMD_SETMICROOFF;//0x35
	buf0[1] = 0x02;
	ret = i2c_himax_master_write(client, buf0, 2, DEFAULT_RETRY_CNT);//micron off
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SETMICROOFF line: %d \n", __func__, __LINE__);
	}
	udelay(100);

	buf0[0] = HX_CMD_SETROMRDY;//0x36
	buf0[1] = 0x0F;
	buf0[2] = 0x53;
	ret = i2c_himax_master_write(client, buf0, 3, DEFAULT_RETRY_CNT);//enable flash
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SETROMRDY line: %d \n", __func__, __LINE__);
	} 
	udelay(100);
	
	buf0[0] = HX_CMD_SET_CACHE_FUN;//0xDD
	buf0[1] = 0x05;
	buf0[2] = 0x03;
	ret = i2c_himax_master_write(client, buf0, 3, DEFAULT_RETRY_CNT);//prefetch
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_SET_CACHE_FUN line: %d \n", __func__, __LINE__);
	} 
	udelay(100);	
	
	buf0[0] = 0xE3;//0xE3
	buf0[1] = 0x00;
	ret = i2c_himax_master_write(client, buf0, 2, DEFAULT_RETRY_CNT);//0xE3
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write 0xE3 line: %d \n", __func__, __LINE__);
	} 
	udelay(100);

	buf0[0] = HX_CMD_TSSON;//0x83
	ret = i2c_himax_master_write(client, buf0, 1, DEFAULT_RETRY_CNT);//sense on
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSON line: %d \n", __func__, __LINE__);
	} 
	mdelay(30);
	
	buf0[0] = HX_CMD_TSSLPOUT;//0x81
	ret = i2c_himax_master_write(client, buf0, 1, DEFAULT_RETRY_CNT);//sleep out
	if(ret < 0) {
		printk(KERN_ERR "[Himax] %s: Write HX_CMD_TSSLPOUT line: %d \n", __func__, __LINE__);
	}
	mdelay(80);

	wake_unlock(&himax_chip->wake_lock);
}

static int himax_ts_resume(struct i2c_client *client)
{
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void himax_ts_early_suspend(struct early_suspend *h)
{
	printk(KERN_INFO "[Himax] %s \n", __func__);
	himax_ts_suspend(himax_chip->client, PMSG_SUSPEND);
}

static void himax_ts_late_resume(struct early_suspend *h)
{
	int ret;
	printk(KERN_INFO "[Himax] %s: himax_chip->suspend_state = %d\n", __func__, himax_chip->suspend_state);

	himax_chip->proximity_status = 0;
	//printk(KERN_ERR "[Himax] %s: proximity_status: %d \n", __func__, himax_chip->proximity_status);
	
	if(himax_chip->suspend_state == 1)
	{
		if(!himax_chip->tp_firmware_upgrade_proceed)
		{	
			printk(KERN_INFO "[Himax] %s: TS resume - Start \n", __func__);
								
			himax_ts_resume_command_all(himax_chip->client);
						
			ret = himax_hang_shaking();	//0:Running, 1:Stop, 2:I2C Fail
			if(ret == 2)
			{
				queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_reset_work, 0);
				printk(KERN_INFO "[Himax] %s: I2C Fail \n", __func__);
			}
			if(ret == 1)
			{
				printk(KERN_INFO "[Himax] %s: MCU Stop \n", __func__);
			#ifdef ESD_WORKAROUND
				//Do HW_RESET??
				ESD_HW_REST();
			#endif
			}
			else
				printk(KERN_INFO "[Himax] %s: MCU Running \n", __func__);
			
			himax_chip->suspend_state = 0;
			enable_irq(himax_chip->irq);
		}
	}
#ifdef ENABLE_CHIP_STATUS_MONITOR
	queue_delayed_work(himax_chip->himax_wq, &himax_chip->himax_chip_monitor, 10*HZ); //for ESD solution
#endif
#ifdef ESD_WORKAROUND
	ESD_COUNTER = 0;
#endif
}
#endif

static const struct i2c_device_id himax_ts_id[] = {
    { HIMAX_TS_NAME, 0 },
    { }
};

static struct i2c_driver himax_ts_driver = {
    .probe        = himax_ts_probe,
    .remove        = himax_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
    .suspend    = himax_ts_suspend,
    .resume        = himax_ts_resume,
#endif
    .id_table    = himax_ts_id,
    .driver        = {
        .name = HIMAX_TS_NAME,
    },
};

static int himax_ts_init(void)
{
	if(Read_PROJ_ID() == PROJ_ID_ME372CG){
		printk(KERN_INFO "[Himax] %s\n", __func__);
		i2c_add_driver(&himax_ts_driver);
	}else{
		printk(KERN_INFO "[Himax] ME372CG init end!!!!\n");
	}
    return 0;
}

static void __exit himax_ts_exit(void)
{
    i2c_del_driver(&himax_ts_driver);
    return;
}

module_init(himax_ts_init);
module_exit(himax_ts_exit);

MODULE_DESCRIPTION("Himax Touchscreen Driver");
MODULE_LICENSE("GPL");



