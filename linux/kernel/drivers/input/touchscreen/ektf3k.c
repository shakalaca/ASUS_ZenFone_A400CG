/* drivers/input/touchscreen/ektf3k.c - ELAN EKTF3K FIFO verions of driver
 *
 * Copyright (C) 2011 Elan Microelectronics Corporation.
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

#define ELAN_BUFFER_MODE

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
#include <linux/time.h>

// for linux 2.6.36.3
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <asm/ioctl.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>

extern int build_version; //add by leo for read build version eng:1, userdebug:2, user:3
extern int Read_TP_ID(void); // add by leo for different touch panel

#define ASUS_DRIVER_VERSION "9019.9021.1.19"	//leo modify (firmware ver(Wintek, AUO).config ver.driver ver)

#define PACKET_SIZE		40
#define NEW_PACKET_SIZE 55
#define FINGER_NUM		10
		
#define PWR_STATE_DEEP_SLEEP	0
#define PWR_STATE_NORMAL		1
#define PWR_NORMAL_STATE 8
#define PWR_IDLE_STATE 1
#define PWR_STATE_MASK			BIT(3)

#define CMD_S_PKT			0x52
#define CMD_R_PKT			0x53
#define CMD_W_PKT			0x54

#define HELLO_PKT			0x55
#define NORMAL_PKT			0x63
#define NEW_NOMARL_PKT      0x66
#define TEN_FINGERS_PKT			0x62

#define RPT_LOCK_PKT		0x56
#define RPT_UNLOCK_PKT		0xA6

#define RESET_PKT			0x77
#define CALIB_PKT			0xA8

#define IDX_FINGER			3
#define MAX_FINGER_SIZE		31
#define MAX_FINGER_PRESSURE	255

#define ABS_MT_POSITION         0x2a    /* Group a set of X and Y */
#define ABS_MT_AMPLITUDE        0x2b    /* Group a set of Z and W */

#include <linux/i2c/ektf3k.h>

// For Firmware Update 
#define ELAN_IOCTLID	0xD0
#define IOCTL_I2C_SLAVE	_IOW(ELAN_IOCTLID,  1, int)
#define IOCTL_MAJOR_FW_VER  _IOR(ELAN_IOCTLID, 2, int)
#define IOCTL_MINOR_FW_VER  _IOR(ELAN_IOCTLID, 3, int)
#define IOCTL_RESET  _IOR(ELAN_IOCTLID, 4, int)
#define IOCTL_IAP_MODE_LOCK  _IOR(ELAN_IOCTLID, 5, int)
#define IOCTL_CHECK_RECOVERY_MODE  _IOR(ELAN_IOCTLID, 6, int)
#define IOCTL_FW_VER  _IOR(ELAN_IOCTLID, 7, int)
#define IOCTL_X_RESOLUTION  _IOR(ELAN_IOCTLID, 8, int)
#define IOCTL_Y_RESOLUTION  _IOR(ELAN_IOCTLID, 9, int)
#define IOCTL_FW_ID  _IOR(ELAN_IOCTLID, 10, int)
#define IOCTL_ROUGH_CALIBRATE  _IOR(ELAN_IOCTLID, 11, int)
#define IOCTL_IAP_MODE_UNLOCK  _IOR(ELAN_IOCTLID, 12, int)
#define IOCTL_I2C_INT  _IOR(ELAN_IOCTLID, 13, int)
#define IOCTL_RESUME  _IOR(ELAN_IOCTLID, 14, int)
#define IOCTL_FW_UPDATE _IOR(ELAN_IOCTLID, 22, int) 

//don't use firmware update
#define FIRMWARE_UPDATE_WITH_HEADER 1

uint16_t checksum_err=0;
static uint8_t RECOVERY=0x00;
int FW_VERSION=0x00;
int X_RESOLUTION=0x00;
int Y_RESOLUTION=0x00;
int FW_ID=0x00;
int FW_TEST_VERSION=0x00;
static int work_lock=0x00;

#define USB_NO_Cable 0
#define USB_DETECT_CABLE 1 
#define USB_SHIFT 0
#define AC_SHIFT 1 
#define USB_Cable ((1 << (USB_SHIFT)) | (USB_DETECT_CABLE))
#define USB_AC_Adapter ((1 << (AC_SHIFT)) | (USB_DETECT_CABLE))
#define USB_CALBE_DETECT_MASK (USB_Cable  | USB_DETECT_CABLE)
static unsigned now_usb_cable_status=0;
static unsigned int gPrint_point = 0; 

/*
//#define TOUCH_STRESS_TEST 1
#ifdef TOUCH_STRESS_TEST
#define STRESS_IOC_MAGIC 0xF3
#define STRESS_IOC_MAXNR 4
#define STRESS_POLL_DATA _IOR(STRESS_IOC_MAGIC,2,int )
#define STRESS_IOCTL_START_HEAVY 2
#define STRESS_IOCTL_START_NORMAL 1
#define STRESS_IOCTL_END 0
#define START_NORMAL	(HZ/5)
#define START_HEAVY	(HZ/200)
static int poll_mode=0;
static struct delayed_work elan_poll_data_work;
static struct workqueue_struct *stress_work_queue;
static atomic_t touch_char_available = ATOMIC_INIT(1);
#endif
*/

#define _ENABLE_DBG_LEVEL    
#ifdef _ENABLE_DBG_LEVEL
	#define PROC_FS_NAME	"ektf_dbg"
	#define PROC_FS_MAX_LEN	8
	static struct proc_dir_entry *dbgProcFile;
#endif

struct elan_ktf3k_ts_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct workqueue_struct *elan_wq;
	struct work_struct work;
	int (*power)(int on);
	struct early_suspend early_suspend;
// Firmware Information
	int fw_ver;
	int fw_test_ver;
	int fw_id;
	int x_resolution;
	int y_resolution;
	uint8_t boot_ver[4]; // add by leo for return touch FW version to AP
// For Firmare Update 
	struct miscdevice firmware;
	struct attribute_group attrs;
	int status;
	struct switch_dev touch_sdev;
	int abs_x_max;
	int abs_y_max;
	int abs_x_min;
	int abs_y_min;
	int intr_gpio;
	int rst_gpio;
	int pwr_en_gpio;
	int irq;
	struct wake_lock wakelock;
	int tp; // add by leo for different touch panel
/*	
#ifdef TOUCH_STRESS_TEST
      struct miscdevice  misc_dev;
#endif
*/
};
static struct elan_ktf3k_ts_data *touch_chip;

static struct elan_ktf3k_ts_data *private_ts = NULL;
static int __fw_packet_handler(struct i2c_client *client, int imediate);
static int elan_ktf3k_ts_rough_calibrate(struct i2c_client *client);
static int elan_ktf3k_ts_hw_reset(struct i2c_client *client);
static int elan_ktf3k_ts_resume(struct i2c_client *client);

#ifdef FIRMWARE_UPDATE_WITH_HEADER
#define FIRMWARE_PAGE_SIZE 132
/*
static unsigned char touch_firmware[] = {
 #include "touchFW/fw_data.b"
 };
 */
 static unsigned char touch_firmware_forWintek[] = {
 #include "touchFW/fw_data_wintek_9020_enduser.b"
 }; 
static unsigned char touch_firmware_forAUO[] = {
 #include "touchFW/fw_data_auo_9022_enduser.b"
 }; 
#define SIZE_PER_PACKET 4
static int firmware_update_header(struct i2c_client *client, unsigned char *firmware, unsigned int page_number);
#endif

static struct semaphore pSem;
static int mTouchStatus[FINGER_NUM] = {0};

#define FIRMWARE_PAGE_SIZE 132
#define MAX_FIRMWARE_SIZE 32868
#define FIRMWARE_ACK_SIZE 2

//others
#define EKTH3374AY_POWER_OFF_WHEN_SUSPEND // add by leo for touch power off when devices suspend
static int CheckHelloPacket = 0;  // add by leo for double trigger problem
static int calibration_flag = 0; // add by leo for keeping touch chip awake when calibration

// add by leo for AP firmware update bar ++ 
#define EKTH3374AY_PROC_FW_CHECK_NAME	"ektf_fw_check"
static struct proc_dir_entry *fw_check_ProcFile;

#define EKTH3374AY_PROC_FW_UPDATE_NAME	"ektf_fw_update"
static struct proc_dir_entry *fw_update_ProcFile;

#define EKTH3374AY_PROC_CALIBRATION_NAME	"ektf_calibration"
static struct proc_dir_entry *calibration_ProcFile;

static int proc_fw_check_result =0;
static int proc_fw_update_result =0;
static int proc_calibration_result =0;
// add by leo for AP firmware update bar --

static int check_fw_version(const unsigned char*firmware, unsigned int size,int fw_id , int fw_version, int fw_test_version);
static int elan_ktf3k_ts_get_power_state(struct i2c_client *client);
static int EKTH3374AY_get_fw_bootcode_version(void); //add by leo for return touch FW version to AP
static ssize_t fw_check_ProcFile_read(struct seq_file *buf, void *v);
static ssize_t fw_check_ProcFile_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
static ssize_t fw_update_ProcFile_read(struct seq_file *buf, void *v);
static ssize_t fw_update_ProcFile_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
static ssize_t calibration_ProcFile_read(struct seq_file *buf, void *v);
static ssize_t calibration_ProcFile_write(struct file *filp, const char __user *buff, unsigned long len, void *data);
static int ektf_proc_read(struct seq_file *buf, void *v);
static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data);

/* Debug levels */
#define NO_DEBUG       0
#define DEBUG_ERROR  1
#define DEBUG_INFO     2
#define DEBUG_MESSAGES 5
#define DEBUG_TRACE   10

static int debug = DEBUG_INFO;

#define touch_debug(level, ...) \
	do { \
		if (debug >= (level)) \
			printk("[ektf3k]:" __VA_ARGS__); \
	} while (0)

int elan_iap_open(struct inode *inode, struct file *filp){ 
	touch_debug(DEBUG_INFO, "[ELAN]into elan_iap_open\n");
		if (private_ts == NULL)  touch_debug(DEBUG_ERROR, "private_ts is NULL~~~");
		
	return 0;
}

int elan_iap_release(struct inode *inode, struct file *filp){    
	return 0;
}

static ssize_t elan_iap_write(struct file *filp, const char *buff, size_t count, loff_t *offp){  
    int ret;
    char *tmp;

    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);
    
    if (tmp == NULL)
        return -ENOMEM;

    if (copy_from_user(tmp, buff, count)) {
        return -EFAULT;
    }
	
    ret = i2c_master_send(private_ts->client, tmp, count);
    if (ret != count) touch_debug(DEBUG_ERROR, "ELAN i2c_master_send fail, ret=%d \n", ret);
    kfree(tmp);
    return ret;

}

ssize_t elan_iap_read(struct file *filp, char *buff, size_t count, loff_t *offp){    
    char *tmp;
    int ret;  
    long rc;
   
    if (count > 8192)
        count = 8192;

    tmp = kmalloc(count, GFP_KERNEL);

    if (tmp == NULL)
        return -ENOMEM;

    ret = i2c_master_recv(private_ts->client, tmp, count);

    if (ret >= 0)
        rc = copy_to_user(buff, tmp, count);
    
    kfree(tmp);

    return ret;
}

static long elan_iap_ioctl(/*struct inode *inode,*/ struct file *filp,    unsigned int cmd, unsigned long arg){

	int __user *ip = (int __user *)arg;
	touch_debug(DEBUG_INFO, "[ELAN]into elan_iap_ioctl cmd=%u\n", cmd);

	switch (cmd) {        
		case IOCTL_I2C_SLAVE: 
			private_ts->client->addr = (int __user)arg;
			break;   
		case IOCTL_MAJOR_FW_VER:            
			break;        
		case IOCTL_MINOR_FW_VER:            
			break;        
		case IOCTL_RESET:
			return elan_ktf3k_ts_hw_reset(private_ts->client);
		case IOCTL_IAP_MODE_LOCK:
			work_lock=1;
			disable_irq(private_ts->client->irq);
			wake_lock(&private_ts->wakelock);
			break;
		case IOCTL_IAP_MODE_UNLOCK:
			work_lock=0;
			enable_irq(private_ts->client->irq);
			wake_unlock(&private_ts->wakelock);
			break;
		case IOCTL_CHECK_RECOVERY_MODE:
			return RECOVERY;
			break;
		case IOCTL_FW_VER:
			__fw_packet_handler(private_ts->client, work_lock);
			msleep(100);
			return FW_VERSION;
			break;
		case IOCTL_X_RESOLUTION:
			__fw_packet_handler(private_ts->client, work_lock);
			msleep(100);
			return X_RESOLUTION;
			break;
		case IOCTL_Y_RESOLUTION:
			__fw_packet_handler(private_ts->client, work_lock);
			msleep(100);
			return Y_RESOLUTION;
			break;
		case IOCTL_FW_ID:
			__fw_packet_handler(private_ts->client, work_lock);
			msleep(100);
			return FW_ID;
			break;
		case IOCTL_ROUGH_CALIBRATE:
			return elan_ktf3k_ts_rough_calibrate(private_ts->client);
		case IOCTL_I2C_INT:
			put_user(gpio_get_value(private_ts->intr_gpio), ip);
			break;
		case IOCTL_RESUME:
			elan_ktf3k_ts_resume(private_ts->client);
			break;
		default:            
			break;   
	}       
	return 0;
}

struct file_operations elan_touch_fops = {    
        .open =         elan_iap_open,    
        .write =        elan_iap_write,    
        .read = 	elan_iap_read,    
        .release =	elan_iap_release,    
	.unlocked_ioctl=elan_iap_ioctl, 
 };

///* Detect old / new FW */
//static int isOldFW(struct i2c_client *client)
//{
//    struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
//        
//    touch_debug(DEBUG_MESSAGES, "[elan] GPIO_TP_INT_N=%d\n", ts->intr_gpio);
//    if (gpio_get_value(ts->intr_gpio) == 0) {
//        // Old FW 
//	 touch_debug(DEBUG_INFO,  "[elan]detect intr=>Old FW\n");
//	 return 1;
//    }
//	
//    return 0;
//}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf3k_ts_early_suspend(struct early_suspend *h);
static void elan_ktf3k_ts_late_resume(struct early_suspend *h);
#endif

static ssize_t elan_ktf3k_gpio_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret = 0;
//	struct elan_ktf3k_ts_data *ts = private_ts;

	ret = gpio_get_value(touch_chip->intr_gpio);
	touch_debug(DEBUG_MESSAGES, "GPIO_TP_INT_N=%d\n", touch_chip->intr_gpio);
	sprintf(buf, "GPIO_TP_INT_N=%d\n", ret);
	ret = strlen(buf) + 1;
	return ret;
}

static DEVICE_ATTR(gpio, S_IRUGO, elan_ktf3k_gpio_show, NULL);

static ssize_t elan_ktf3k_vendor_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	ssize_t ret = 0,sprintf_count=0;
//	struct elan_ktf3k_ts_data *ts = private_ts;

//	sprintf(buf, "%s_x%4.4x\n", "ELAN_KTF3K", touch_chip->fw_ver);
//	ret = strlen(buf) + 1;

	sprintf_count += sprintf(buf + sprintf_count, "Vendor = ELAN_KTF3K\n");
	sprintf_count += sprintf(buf + sprintf_count, "FW ID = 0x%4.4x\n",touch_chip->fw_id);
	sprintf_count += sprintf(buf + sprintf_count, "FW Version = 0x%4.4x\n",touch_chip->fw_ver);
	sprintf_count += sprintf(buf + sprintf_count, "FW Test Version = 0x%4.4x\n",touch_chip->fw_test_ver);
	sprintf_count += sprintf(buf + sprintf_count, "Touch Panel = %d\n",touch_chip->tp);

#ifdef FIRMWARE_UPDATE_WITH_HEADER	
	if((touch_chip->tp==0)||(touch_chip->tp==2)){	//WINTEK Panel
		ret = check_fw_version(touch_firmware_forWintek, sizeof(touch_firmware_forWintek), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
	}else if((touch_chip->tp==3)){	// AUO Panel
		ret = check_fw_version(touch_firmware_forAUO, sizeof(touch_firmware_forAUO), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
	}else{
		sprintf_count += sprintf(buf + sprintf_count, "Touch Panel Unknow \n");
		return sprintf_count;
	}
	
	if(ret>0){
		sprintf_count += sprintf(buf + sprintf_count, "Need to do FW update (%d)\n",ret);
	}else if(ret<0){
		sprintf_count += sprintf(buf + sprintf_count, "chip FW ID is different from touchFW ID  (%d)\n",ret);
	}else{
		sprintf_count += sprintf(buf + sprintf_count, "FW version OK (%d)\n",ret);
	}
#endif

	return sprintf_count;
}

static DEVICE_ATTR(vendor_info, S_IRUGO, elan_ktf3k_vendor_show, NULL);

static ssize_t elan_show_status(struct device *dev, struct device_attribute *devattr, char *buf)
{
//	struct i2c_client *client = to_i2c_client(dev);
//	struct elan_ktf3k_ts_data *data = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", touch_chip->status);
}

DEVICE_ATTR(elan_touchpanel_status, S_IRUGO, elan_show_status, NULL);

static int check_fw_version(const unsigned char*firmware, unsigned int size,int fw_id , int fw_version, int fw_test_version){
       //int id, version;
	int c1=0;
	int	wFWVer=0,wTSVer=0,wFWID=0;
       uint16_t pageAddr;
       uint8_t Hibyte=0, Lowbyte=0;

	// FW Version
	c1=0;
	do
	{
		Lowbyte=firmware[c1];
		Hibyte=firmware[c1+1];
		pageAddr=(Hibyte << 8) | Lowbyte;

		Lowbyte=firmware[c1+2];
		Hibyte=firmware[c1+3];
		wFWVer=(Hibyte << 8) | Lowbyte;
		c1 += 132;
            } while (pageAddr != 0x4AC0);	//0x4AC0
	//printk("FW Version: %4.4X\r\n", wFWVer);
	//printk("Hibyte: %4.4X\r\n", Hibyte);
	//printk("Lowbyte: %4.4X\r\n", Lowbyte);
	//printk("pageAddr: %4.4X\r\n", pageAddr);
	printk("[Elan] %s: touchFW - FW Version: 0x%4.4X\r\n", __func__, wFWVer);

	// Test Version
	c1=0;
	do
	{
		Lowbyte=firmware[c1];
		Hibyte=firmware[c1+1];
		pageAddr=(Hibyte << 8) | Lowbyte;

		Lowbyte=firmware[c1+4];
		Hibyte=firmware[c1+4+1];
		wTSVer=(Hibyte << 8) | Lowbyte;		

		c1 += 132;
         } while (pageAddr != 0x4AC0);	
	//printk("Test Version: %2.2X\r\n", wTSVer);
	printk("[Elan] %s: touchFW - FW Test Version: 0x%4.4X\r\n", __func__, wTSVer);
	
	// FW ID
	c1=0;
	do
	{
		Lowbyte=firmware[c1];
		Hibyte=firmware[c1+1];
		pageAddr=(Hibyte << 8) | Lowbyte;
		Lowbyte=firmware[c1+2];
		Hibyte=firmware[c1+3];
		wFWID=(Hibyte << 8) | Lowbyte;	
		c1 += 132;
            } while (pageAddr != 0x4BC0);	
	//printk("FW ID: %4.4X\r\n", wFWID);
	printk("[Elan] %s: touchFW - FW ID: 0x%4.4x\r\n", __func__, wFWID);
	
/*	   
       if(size < 2*FIRMWARE_PAGE_SIZE)
           return -1;
	   
	 version = firmware[size - 2*FIRMWARE_PAGE_SIZE + 120] | 
	 	      (firmware[size - 2*FIRMWARE_PAGE_SIZE + 121] << 8); 
	 id = firmware[size - 2*FIRMWARE_PAGE_SIZE + 122] | 
	 	      (firmware[size - 2*FIRMWARE_PAGE_SIZE + 123] << 8);
	 
	 touch_debug(DEBUG_INFO, "The firmware was version 0x%X and id:0x%X\n", version, id);
*/

	if(fw_id==0)return -111; // add by leo for old FW on SR board

	if(fw_id != wFWID){
 // modify by  leo for different touch panel ++
		if(((touch_chip->tp==0)||(touch_chip->tp==2))&&(wFWID == 12333)){	//Wintek panel, 12333 =302d
			printk("[Elan] %s: chip FW is AUO but panel is different, need to reload Wintek FW\r\n", __func__);	
			return 1;
		}else if((touch_chip->tp==3)&&(wFWID == 12334)){	//AUO panel, 12334 =302e
			printk("[Elan] %s: chip FW is Wintek but panel is different, need to reload AUO FW\r\n", __func__);	
			return 2;
		}
// modify by  leo for different touch panel --

		printk("[Elan] %s: touchFW ID (0x%4.4x) Unknow\r\n", __func__,wFWID);		
		return -123;
	 }

	if (fw_version == 0xFFFF){
		printk("[Elan] %s: chip ID = touchFW ID (fw_version =0x%4.4x)\r\n", __func__, fw_version);
		return 3;
	}else if((wFWVer - fw_version)>0){
		printk("[Elan] %s: chip ID = touchFW ID (touchFW version > chip FW version)\r\n", __func__);
		return 4;
	}else if(((wFWVer - fw_version)==0)&&((wTSVer - fw_test_version)>0)){
		printk("[Elan] %s: chip ID = touchFW ID (touchFW test version > chip FW test version)\r\n", __func__);
		return 5;
	}

	return 0; // this buffer doesn't contain the touch firmware
}
static ssize_t update_firmware(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int ret=0;
/*
//	 struct i2c_client *client = to_i2c_client(dev);
//	 struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	struct file *firmware_fp;
	char file_path[100];
	unsigned int pos = 0;
	mm_segment_t oldfs;
	unsigned int page_number;
	static unsigned char firmware[MAX_FIRMWARE_SIZE];
	int ret = 0, retry = 0;
	
	oldfs=get_fs();
	set_fs(KERNEL_DS);
	memset(file_path, 0, 100);
	sscanf(buf, "%s\n", file_path);
	touch_debug(DEBUG_INFO, "Update touch firmware with the file path:%s\n", file_path);
	firmware_fp = filp_open(file_path, O_RDONLY, S_IRUSR |S_IRGRP);
	if(PTR_ERR(firmware_fp) == -ENOENT){
		dev_err(&touch_chip->client->dev, "Error to open file %s\n", file_path);
		return -1;
	}    
	//start move the firmware file into memory
	firmware_fp->f_pos = 0;
	for(pos = 0, page_number = 0; pos < MAX_FIRMWARE_SIZE; pos += FIRMWARE_PAGE_SIZE, page_number++){
		if(firmware_fp->f_op->read(firmware_fp, firmware + pos,
			FIRMWARE_PAGE_SIZE, &firmware_fp->f_pos) != FIRMWARE_PAGE_SIZE){
			break;
		}
	}
	filp_close(firmware_fp, NULL);
	// check the firmware ID and version
	if(RECOVERY || check_fw_version(firmware, pos, touch_chip->fw_ver) > 0){
		touch_debug(DEBUG_INFO, "Firmware update start!\n");	
		do{
		//	ret = firmware_update_header(client, firmware, page_number);//add by mars
			touch_debug(DEBUG_INFO, "Firmware update finish ret=%d retry=%d !\n", ret, retry++);
		}while(ret != 0 && retry < 3);
		
		if(ret == 0 && RECOVERY)
			RECOVERY = 0;
	}else 
		touch_debug(DEBUG_INFO, "No need to update firmware\n");
 */

#ifdef FIRMWARE_UPDATE_WITH_HEADER	

//add by leo for different touch panel ++
	if((touch_chip->tp==0)||(touch_chip->tp==2)){	//WINTEK Panel
		ret = firmware_update_header(touch_chip->client, touch_firmware_forWintek, sizeof(touch_firmware_forWintek)/FIRMWARE_PAGE_SIZE);
		
	}else if((touch_chip->tp==3)){ //AUO Panel
		ret = firmware_update_header(touch_chip->client, touch_firmware_forAUO, sizeof(touch_firmware_forAUO)/FIRMWARE_PAGE_SIZE);
	}else{
		printk("[Elan] %s: touch firmware update failed - unknow touch panel\n", __func__);
		return count;
	}
	//ret=firmware_update_header(touch_chip->client, touch_firmware, sizeof(touch_firmware)/FIRMWARE_PAGE_SIZE);
//add by leo for different touch panel --
	   if(ret<0){
		printk("[Elan] %s: touch firmware update failed\n", __func__);
	   }else{
		printk("[Elan] %s: touch firmware update succeed !\n", __func__);
	   }
#endif

	return count;
}

DEVICE_ATTR(update_fw,  S_IWUSR, NULL, update_firmware);

static ssize_t elan_calibration_store(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int cflag=0;
	
	sscanf(buf, "%d\n", &cflag);
	calibration_flag = (cflag>0? 1:0);
	printk("[Elan] %s: calibration_flag = %d\n", __func__,calibration_flag);

	return count;
}

static ssize_t elan_calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret =elan_ktf3k_ts_rough_calibrate(private_ts->client);
	if(ret<0){
		return sprintf(buf, "Calibration Failed (%d)\n",ret);
	}
	return sprintf(buf, "Calibration Succeed (%d)\n",ret);
}

DEVICE_ATTR(calibration, S_IWUSR | S_IRUGO, elan_calibration_show, elan_calibration_store);

static ssize_t elan_get_tp_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	touch_chip->tp = Read_TP_ID();
	return sprintf(buf, "%d \n", touch_chip->tp);
}

DEVICE_ATTR(tp_id, S_IRUGO, elan_get_tp_id, NULL);

static ssize_t elan_get_gpio(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[Elan] %s: intr_gpio = %d\n", __func__, gpio_get_value(touch_chip->intr_gpio));
	printk("[Elan] %s: rst_gpio = %d\n", __func__, gpio_get_value(touch_chip->rst_gpio));
	printk("[Elan] %s: pwr_en_gpio = %d\n", __func__, gpio_get_value(touch_chip->pwr_en_gpio));
	
	return sprintf(buf, "INT =%x \n", gpio_get_value(touch_chip->intr_gpio));
}

DEVICE_ATTR(get_gpio, S_IRUGO, elan_get_gpio, NULL);

// add by leo for return touch FW version to AP ++
static ssize_t elan_get_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk("[Elan] %s ++ \n", __func__);
	return sprintf(buf, "%4.4X-%4.4X-%4.4X %X%X%X %d\n", touch_chip->fw_id, touch_chip->fw_ver, touch_chip->fw_test_ver, (0x0f & touch_chip->boot_ver[1]), touch_chip->boot_ver[2], (touch_chip->boot_ver[3]>>4), touch_chip->tp);
}

DEVICE_ATTR(tp_fw_version, S_IRUGO, elan_get_fw_version, NULL);
// add by leo for return touch FW version to AP --

static ssize_t elan_check_fw_version(struct device *dev, struct device_attribute *attr, char *buf)
{
// add by leo for reading boot code version ++
	int ret=0,retry=5,length=0;
	uint8_t cmd[] = {0x53, 0x10, 0x00, 0x01};
	uint8_t data[4]={0};

	work_lock=1;
	disable_irq(touch_chip->irq);
	wake_lock(&touch_chip->wakelock);

	length = i2c_master_send(touch_chip->client, cmd, 4);
	if (length != sizeof(cmd)) {
		dev_err(&touch_chip->client->dev,"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	for(retry=5;retry>0;retry--){
		
		if(gpio_get_value(touch_chip->intr_gpio)==0){
			ret = i2c_master_recv(touch_chip->client, data, 4);
			if(ret<0){
				printk("[Elan] %s:[%d]: i2c_master_recv failed\n", __func__, __LINE__);
				return -EAGAIN;
			}
			printk("[Elan] %s: received 0x%X 0x%X 0x%X 0x%X\n", __func__, data[0], data[1], data[2], data[3]);
			break;
		}else{
			printk("[Elan] INT is hight\n");
			//printk("[Elan] %s: gpio_get_value(touch_chip->intr_gpio) = %d\n", __func__, gpio_get_value(touch_chip->intr_gpio);
		}
		msleep(500);
	}

	work_lock=0;
	enable_irq(touch_chip->irq);
	wake_unlock(&touch_chip->wakelock);

	touch_chip->boot_ver[0]=data[0];
	touch_chip->boot_ver[1]=data[1];
	touch_chip->boot_ver[2]=data[2];
	touch_chip->boot_ver[3]=data[3];
	printk("[Elan] %s: FW (Boot Code Version) is %X%X%X\n", __func__, (0x0f & touch_chip->boot_ver[1]), touch_chip->boot_ver[2], (touch_chip->boot_ver[3]>>4));
// add by leo for reading boot code version --
	
	//return sprintf(buf, "%4.4X-%4.4X-%4.4X %X%X%X %d\n", touch_chip->fw_id, touch_chip->fw_ver, touch_chip->fw_test_ver, (0x0f & data[1]), data[2], (data[3]>>4), touch_chip->tp);
	return sprintf(buf, "%4.4X-%4.4X-%4.4X %X%X%X %d\n", touch_chip->fw_id, touch_chip->fw_ver, touch_chip->fw_test_ver, (0x0f & touch_chip->boot_ver[1]), touch_chip->boot_ver[2], (touch_chip->boot_ver[3]>>4), touch_chip->tp);
}

DEVICE_ATTR(fw_ver, S_IRUGO, elan_check_fw_version, NULL);

// add by leo for gpio control ++
static ssize_t elan_rst_gpio_store(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int rst_pin=0;
	
	sscanf(buf, "%d\n", &rst_pin);
	
	gpio_set_value(touch_chip->rst_gpio, (rst_pin>0? 1:0));	
	printk("[Elan] %s: set rst_pin = %s\n", __func__, (rst_pin>0? "High":"Low"));

	return count;
}

static ssize_t elan_rst_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int rst_pin=0;

	rst_pin = gpio_get_value(touch_chip->rst_gpio);
	printk("[Elan] %s: get rst_pin = %d\n", __func__, rst_pin);
	
	return sprintf(buf, "rst_gpio = %s\n", (rst_pin>0? "High":"Low"));
}

DEVICE_ATTR(rst_gpio, S_IWUSR | S_IRUGO, elan_rst_gpio_show, elan_rst_gpio_store);

static ssize_t elan_pwr_en_gpio_store(struct device *dev, struct device_attribute *devattr,const char *buf, size_t count)
{
	int pwr_en_pin=0;
	
	sscanf(buf, "%d\n", &pwr_en_pin);
	
	gpio_set_value(touch_chip->pwr_en_gpio, (pwr_en_pin>0? 1:0));	
	printk("[Elan] %s: set pwr_en_pin = %s\n", __func__, (pwr_en_pin>0? "High":"Low"));

	return count;
}

static ssize_t elan_pwr_en_gpio_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int pwr_en_pin=0;

	pwr_en_pin = gpio_get_value(touch_chip->pwr_en_gpio);
	printk("[Elan] %s: get pwr_en_pin = %d\n", __func__, pwr_en_pin);
	
	return sprintf(buf, "pwr_en_gpio = %s\n", (pwr_en_pin>0? "High":"Low"));
}

DEVICE_ATTR(pwr_en_gpio, S_IWUSR | S_IRUGO, elan_pwr_en_gpio_show, elan_pwr_en_gpio_store);

static ssize_t elan_recv_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	ssize_t sprintf_count=0;
	uint8_t data[4]={0};

	enable_irq(touch_chip->irq);

	sprintf_count += sprintf(buf + sprintf_count, "intr_gpio (before recevie data) = %s\n", (gpio_get_value(touch_chip->intr_gpio) > 0? "High":"Low"));

	ret = i2c_master_recv(touch_chip->client, data, 4);
	if(ret<0){
		sprintf_count += sprintf(buf + sprintf_count, "i2c_master_recv failed (%d)\n", ret);
	}

	sprintf_count += sprintf(buf + sprintf_count, "Get data: 0x%x 0x%x 0x%x 0x%x\n", data[0], data[1], data[2], data[3]);
	sprintf_count += sprintf(buf + sprintf_count, "intr_gpio (after recevie data) = %s\n", (gpio_get_value(touch_chip->intr_gpio) > 0? "High":"Low"));

	return sprintf_count;
}

DEVICE_ATTR(recv_data, S_IRUGO, elan_recv_data, NULL);

// add by leo for gpio control --
static struct attribute *elan_attr[] = {
	&dev_attr_elan_touchpanel_status.attr,
	&dev_attr_tp_fw_version.attr,
	&dev_attr_fw_ver.attr,
	&dev_attr_vendor_info.attr,
	&dev_attr_gpio.attr,
	&dev_attr_update_fw.attr,
	&dev_attr_get_gpio.attr,
	&dev_attr_calibration.attr,
	&dev_attr_tp_id.attr,
	&dev_attr_pwr_en_gpio.attr,
	&dev_attr_rst_gpio.attr,
	&dev_attr_recv_data.attr,
	NULL
};

static struct kobject *android_touch_kobj;

//add by leo for return touch FW version to AP ++
static int EKTH3374AY_get_fw_bootcode_version(void){
	
	int ret=0,retry=5,length=0;
	uint8_t cmd[] = {0x53, 0x10, 0x00, 0x01};
	uint8_t data[4]={0};

	work_lock=1;
	//disable_irq(touch_chip->irq);
	//wake_lock(&touch_chip->wakelock);

	length = i2c_master_send(touch_chip->client, cmd, 4);
	if (length != sizeof(cmd)) {
		dev_err(&touch_chip->client->dev,"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	for(retry=5;retry>0;retry--){
		
		if(gpio_get_value(touch_chip->intr_gpio)==0){
			ret = i2c_master_recv(touch_chip->client, data, 4);
			if(ret<0){
				printk("[Elan] %s:[%d]: i2c_master_recv failed\n", __func__, __LINE__);
				return -EAGAIN;
			}
			printk("[Elan] %s: received 0x%X 0x%X 0x%X 0x%X\n", __func__, data[0], data[1], data[2], data[3]);
			break;
		}else{
			printk("[Elan] INT is hight\n");
			//printk("[Elan] %s: gpio_get_value(touch_chip->intr_gpio) = %d\n", __func__, gpio_get_value(touch_chip->intr_gpio);
		}
		msleep(500);
	}

	work_lock=0;
	//enable_irq(touch_chip->irq);
	//wake_unlock(&touch_chip->wakelock);

	touch_chip->boot_ver[0]=data[0];
	touch_chip->boot_ver[1]=data[1];
	touch_chip->boot_ver[2]=data[2];
	touch_chip->boot_ver[3]=data[3];
	printk("[Elan] %s: FW (Boot Code Version) is %X%X%X\n", __func__, (0x0f & touch_chip->boot_ver[1]), touch_chip->boot_ver[2], (touch_chip->boot_ver[3]>>4));

	return 0;
}
//add by leo for return touch FW version to AP --

/*
static int elan_ktf3k_touch_sysfs_init(void)
{
	int ret ;

	android_touch_kobj = kobject_create_and_add("android_touch", NULL) ;
	if (android_touch_kobj == NULL) {
		touch_debug(DEBUG_ERROR, "[elan]%s: subsystem_register failed\n", __func__);
		ret = -ENOMEM;
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_gpio.attr);
	if (ret) {
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_file failed\n", __func__);
		return ret;
	}
	ret = sysfs_create_file(android_touch_kobj, &dev_attr_vendor.attr);
	if (ret) {
		touch_debug(DEBUG_ERROR, "[elan]%s: sysfs_create_group failed\n", __func__);
		return ret;
	}
	return 0 ;
}
*/
static void elan_touch_sysfs_deinit(void)
{
	sysfs_remove_file(android_touch_kobj, &dev_attr_vendor_info.attr);
	sysfs_remove_file(android_touch_kobj, &dev_attr_gpio.attr);
	kobject_del(android_touch_kobj);
}

static int __elan_ktf3k_ts_poll(struct i2c_client *client)
{
//	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int status = 0, retry = 150;

	do {
		status = gpio_get_value(touch_chip->intr_gpio);
		dev_dbg(&client->dev, "%s: status = %d\n", __func__, status);
	//	touch_debug(DEBUG_ERROR, "[Elan] %s: status = %d\n", __func__, status);
		retry--;
		msleep(20);
	} while (status != 0 && retry > 0); // modify by leo

	dev_dbg(&client->dev, "[elan]%s: poll interrupt status %s\n",
			__func__, status == 1 ? "high" : "low");
	return (status == 0 ? 0 : -ETIMEDOUT);
}

static int elan_ktf3k_ts_poll(struct i2c_client *client)
{
	return __elan_ktf3k_ts_poll(client);
}

static int elan_ktf3k_ts_get_data(struct i2c_client *client, uint8_t *cmd,
			uint8_t *buf, size_t size)
{
	int rc;
	
	dev_dbg(&client->dev, "[elan]%s: enter\n", __func__);
	
	if (buf == NULL)
		return -EINVAL;
	
	down(&pSem);
	rc = i2c_master_send(touch_chip->client, cmd, 4);
	up(&pSem);
	if(rc != 4){
		dev_err(&client->dev,"[elan]%s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}
	down(&pSem);
	rc  = i2c_master_recv(touch_chip->client, buf, size);
	up(&pSem);       
	if(rc != size){
		dev_err(&client->dev, "[elan]%s: i2c_master_read failed\n", __func__);
		return -EINVAL;
	}
	
	return 0;
}

static int elan_ktf3k_ts_read_command(struct i2c_client *client,
			   u8* cmd, u16 cmd_length, u8 *value, u16 value_length){
	
	struct i2c_adapter *adapter = touch_chip->client->adapter;
	struct i2c_msg msg[2];
	//__le16 le_addr;
//	struct elan_ktf3k_ts_data *ts;
	int length = 0;

//	ts = i2c_get_clientdata(client);

	msg[0].addr = touch_chip->client->addr;
	msg[0].flags = 0x00;
	msg[0].len = cmd_length;
	msg[0].buf = cmd;

	down(&pSem);
	length = i2c_transfer(adapter, msg, 1);
	up(&pSem);
	
	if (length == 1) // only send on packet
		return value_length;
	else
		return -EIO;
}

static int elan_ktf3k_i2c_read_packet(struct i2c_client *client, u8 *value, u16 value_length)
{
	struct i2c_adapter *adapter = touch_chip->client->adapter;
	struct i2c_msg msg[1];
	//__le16 le_addr;
//	struct elan_ktf3k_ts_data *ts;
	int length = 0;

//	ts = i2c_get_clientdata(client);

	msg[0].addr = touch_chip->client->addr;
	msg[0].flags = I2C_M_RD;
	msg[0].len = value_length;
	msg[0].buf = (u8 *) value;
	down(&pSem);
	length = i2c_transfer(adapter, msg, 1);
	up(&pSem);
	
	if (length == 1) // only send on packet
		return value_length;
	else
		return -EIO;
}

static int __hello_packet_handler(struct i2c_client *client)
{
	int rc;
	uint8_t buf_recv[4] = { 0 };
	//uint8_t buf_recv1[4] = { 0 };

	rc = elan_ktf3k_ts_poll(touch_chip->client);
	if (rc < 0) {
		printk("[Elan] %s:[%d]: IRQ is not low! \n", __func__, __LINE__);
		RECOVERY = 1; 
	}
//	printk("[Elan] %s:[%d] Joe\n", __func__, __LINE__);
	rc = i2c_master_recv(touch_chip->client, buf_recv, 4);
	if(rc < 0){
		printk("[Elan] %s: I2C message no ACK! \n", __func__);
		return -EINVAL;
	}
	printk("[Elan] %s:[%d]: hello packet %2x:%2x:%2x:%2x \n", __func__, __LINE__, buf_recv[0], buf_recv[1], buf_recv[2], buf_recv[3]);
	if(!(buf_recv[0]==0x55 && buf_recv[1]==0x55 && buf_recv[2]==0x55 && buf_recv[3]==0x55)){
		RECOVERY=0x80;
		return RECOVERY;
	}
	return 0;
}

static int wait_for_IRQ_Low(struct i2c_client *client, int utime){
//    struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int retry_times = 10;
	do{
		usleep_range(utime,utime + 500);
	//	printk("[Elan] %s:[%d]: Joe \n", __func__, __LINE__);
		if(gpio_get_value(touch_chip->intr_gpio) == 0)
			return 0; 
	}while(retry_times-- > 0);
	
	printk("[Elan] %s:[%d]: Wait IRQ time out \n", __func__, __LINE__);
	return -1;
}

static int __fw_packet_handler(struct i2c_client *client, int immediate)
{
//	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	int rc;
	int major, minor;
	uint8_t cmd[] = {CMD_R_PKT, 0x00, 0x00, 0x01};
	uint8_t cmd_x[] = {0x53, 0x60, 0x00, 0x00}; /*Get x resolution*/
	uint8_t cmd_y[] = {0x53, 0x63, 0x00, 0x00}; /*Get y resolution*/
	uint8_t cmd_id[] = {0x53, 0xf0, 0x00, 0x01}; /*Get firmware ID*/
	uint8_t cmd_test_ver[] = {0x53, 0xe0, 0x00, 0x01}; /*Get test firmware ver*/
	uint8_t buf_recv[4] = {0};
// Firmware version
	rc = elan_ktf3k_ts_read_command(touch_chip->client, cmd, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	
	if(immediate){
	    wait_for_IRQ_Low(touch_chip->client, 1000);
	    elan_ktf3k_i2c_read_packet(touch_chip->client, buf_recv, 4);
	    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	    touch_chip->fw_ver = major << 8 | minor;
	    FW_VERSION = touch_chip->fw_ver;
	    printk("[Elan] %s:[%d]: firmware version: 0x%4.4x\n", __func__, __LINE__, touch_chip->fw_ver);
	}
// X Resolution
	rc = elan_ktf3k_ts_read_command(touch_chip->client, cmd_x, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	
	if(immediate){
	    wait_for_IRQ_Low(touch_chip->client, 1000);
	    elan_ktf3k_i2c_read_packet(touch_chip->client, buf_recv, 4);
	    minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	    if(unlikely(gPrint_point))
			printk("[Elan] %s:[%d]:read cmd_x: 0x%x 0x%x 0x%x 0x%x \n", __func__, __LINE__,buf_recv[0],buf_recv[1],buf_recv[2],buf_recv[3]);
		
	    touch_chip->x_resolution =minor;
	    X_RESOLUTION = touch_chip->x_resolution;
	    printk("[Elan] %s:[%d]: X resolution: 0x%4.4x\n", __func__, __LINE__, touch_chip->x_resolution);
	}
// Y Resolution	
	rc = elan_ktf3k_ts_read_command(touch_chip->client, cmd_y, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	
	if(immediate){
	    wait_for_IRQ_Low(touch_chip->client, 1000);
	    elan_ktf3k_i2c_read_packet(touch_chip->client, buf_recv, 4);
	    minor = ((buf_recv[2])) | ((buf_recv[3] & 0xf0) << 4);
	    if(unlikely(gPrint_point))
			printk("[Elan] %s:[%d]:read cmd_y: 0x%x 0x%x 0x%x 0x%x \n", __func__, __LINE__,buf_recv[0],buf_recv[1],buf_recv[2],buf_recv[3]);
		
	    touch_chip->y_resolution =minor;
	    Y_RESOLUTION = touch_chip->y_resolution;
	    printk("[Elan] %s:[%d]: Y resolution: 0x%4.4x\n", __func__, __LINE__, touch_chip->y_resolution);
	}
// Firmware ID
	rc = elan_ktf3k_ts_read_command(touch_chip->client, cmd_id, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	
	if(immediate){
	    wait_for_IRQ_Low(touch_chip->client, 1000);
	    elan_ktf3k_i2c_read_packet(touch_chip->client, buf_recv, 4);
	    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	    minor = ((buf_recv[2] & 0x0f) << 4) | ((buf_recv[3] & 0xf0) >> 4);
	    if(unlikely(gPrint_point))
			printk("[Elan] %s:[%d]:read cmd_id: 0x%x 0x%x 0x%x 0x%x \n", __func__, __LINE__,buf_recv[0],buf_recv[1],buf_recv[2],buf_recv[3]);
		
	    touch_chip->fw_id = major << 8 | minor;
	    FW_ID = touch_chip->fw_id;
	    printk("[Elan] %s:[%d]: firmware id: 0x%4.4x\n", __func__, __LINE__, touch_chip->fw_id);
	}
// Firmware Test Version
	rc = elan_ktf3k_ts_read_command(touch_chip->client, cmd_test_ver, 4, buf_recv, 4);
	if (rc < 0)
		return rc;
	
	if(immediate){
	    wait_for_IRQ_Low(touch_chip->client, 1000);
	    elan_ktf3k_i2c_read_packet(touch_chip->client, buf_recv, 4);
	    major = ((buf_recv[1] & 0x0f) << 4) | ((buf_recv[2] & 0xf0) >> 4);
	    if(unlikely(gPrint_point))
			printk("[Elan] %s:[%d]:read cmd_id: 0x%x 0x%x 0x%x 0x%x \n", __func__, __LINE__,buf_recv[0],buf_recv[1],buf_recv[2],buf_recv[3]);
		
	    touch_chip->fw_test_ver = major;
	    FW_TEST_VERSION = touch_chip->fw_test_ver;;
	    printk("[Elan] %s:[%d]: firmware test version: 0x%4.4x\n", __func__, __LINE__, touch_chip->fw_test_ver);
	}
	return 0;
}

static inline int elan_ktf3k_ts_parse_xy(uint8_t *data,
			uint16_t *x, uint16_t *y)
{
	*x = *y = 0;
	
	*x = (data[0] & 0xf0);
	*x <<= 4;
	*x |= data[1];

	*y = (data[0] & 0x0f);
	*y <<= 8;
	*y |= data[2];
		
	return 0;
}

static int elan_ktf3k_ts_setup(struct i2c_client *client)
{
	int rc, count = 10;
	
retry:	
	// Reset
	elan_ktf3k_ts_hw_reset(client);
//	// Check if old firmware. If not, send the notmal_command to enter normal mode
//    if( isOldFW(client) == 0 ){ //if check is new bootcode
//		touch_debug(DEBUG_INFO, "The boot code is new!\n");
//	}else
//		touch_debug(DEBUG_INFO, "The boot code is old!\n");
	
	rc = __hello_packet_handler(client);
	printk("[Elan] %s: hello packet's rc = %d \n", __func__, rc);
	
	if (rc < 0){ 
		if (rc == -ETIME && count > 0) {
			count--;
			printk(KERN_ERR "[Elan] %s: wait main hello timeout, reset \n", __func__);
			goto retry;
		}
		else
			goto hand_shake_failed;
	}

	CheckHelloPacket = 1; // add by leo for double trigger problem
	printk("[Elan] %s: hello packet got. \n", __func__);
	msleep(200);
	rc = __fw_packet_handler(client, 1);
	if (rc < 0)
		goto hand_shake_failed;

	EKTH3374AY_get_fw_bootcode_version(); // add by leo for return touch FW version to AP
	
	printk("[Elan] %s: firmware checking done. \n", __func__);
	
hand_shake_failed:
	return rc;
}

static int elan_ktf3k_ts_set_power_state(struct i2c_client *client, int state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x50, 0x00, 0x01};
	int length;

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);

	cmd[1] |= (state << 3);

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);

      down(&pSem);
      length = i2c_master_send(client, cmd, sizeof(cmd));
      up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

static int elan_ktf3k_ts_rough_calibrate(struct i2c_client *client){
      uint8_t cmd[] = {CMD_W_PKT, 0x29, 0x00, 0x01};
      uint8_t cal_cmd_1[] = {0x54, 0xc0, 0xe1, 0x5a}; // add by leo for calibration
      int length;

	int ret=0,retry = 5;	// add by leo
	uint8_t buf[4]={0};	// add by leo
	
	touch_debug(DEBUG_INFO, "[elan] %s: enter\n", __func__);

	disable_irq(touch_chip->irq);// add by leo

//copy from resume by leo for calibration ++
	if(work_lock == 0){
	    do {
			ret = elan_ktf3k_ts_set_power_state(touch_chip->client, PWR_STATE_NORMAL);
			ret = elan_ktf3k_ts_get_power_state(touch_chip->client);
			if (ret != PWR_NORMAL_STATE && ret != PWR_IDLE_STATE){
					printk("[Elan] %s:[%d]: wake up tp failed! err = %d \n", __func__, __LINE__, ret);
					return -EAGAIN; // add by leo
			}else{
				break;
			}
			
	    } while (--retry);
	}
	printk("[Elan] %s:[%d]: power state = %s \n", __func__, __LINE__, (ret == PWR_STATE_DEEP_SLEEP ? "Deep Sleep" : "Normal/Idle"));
//copy from resume by leo for calibration --

	msleep(300); // add by leo to wait for chip resume
	
//add by leo for calibration ++
        touch_debug(DEBUG_INFO,
                "[elan] dump cmd: %02x, %02x, %02x, %02x\n",
                cal_cmd_1[0], cal_cmd_1[1], cal_cmd_1[2], cal_cmd_1[3]);

        down(&pSem);
        length = i2c_master_send(client, cal_cmd_1, sizeof(cal_cmd_1));
        up(&pSem);
        if (length != sizeof(cal_cmd_1)) {
                dev_err(&client->dev,
                        "[elan] %s: i2c_master_send failed\n", __func__);
                return -EINVAL;
        }
//add by leo for calibration --
	
	touch_debug(DEBUG_INFO,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);
	
	down(&pSem);
	length = i2c_master_send(client, cmd, sizeof(cmd));
	up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

//add by leo for calibration ++
	msleep(5000);
	
	ret = i2c_master_recv(touch_chip->client, buf, 4);
	if(ret<0){
		printk("[Elan] %s:[%d]: i2c_master_recv failed\n", __func__, __LINE__);
		return -EAGAIN;
	}
	printk("[Elan] %s: buf[0]=0x%x, buf[1]=0x%x, buf[2]=0x%x, buf[3]=0x%x\n", __func__, buf[0],buf[1],buf[2],buf[3]);
	
	enable_irq(touch_chip->irq);
	
	if((buf[0]!=0x66)&&(buf[0]!=0x66)&&(buf[0]!=0x66)&&(buf[0]!=0x66)){
		return -EAGAIN;
	}
//add by leo for calibration --
	return 0;
}

static int elan_ktf3k_ts_get_power_state(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x50, 0x00, 0x01};
	uint8_t buf[4], power_state;

	rc = elan_ktf3k_ts_get_data(client, cmd, buf, 4);
	if (rc)
		return rc;

	power_state = buf[1]; //received 52 5"8" 00 01 is normal mode; 52 5"1" 00 01 is idle mode; 52 5"0" 00 01 is sleep mode
	touch_debug(DEBUG_INFO, "[elan] dump repsponse: %0x\n", power_state);
	power_state = power_state & 0x0F;
	dev_dbg(&client->dev, "[elan] power state = %s\n",
		power_state == PWR_STATE_DEEP_SLEEP ?
		"Deep Sleep" : "Normal/Idle");

	return power_state;
}

static int elan_ktf3k_ts_hw_reset(struct i2c_client *client)
{
	printk("[Elan] %s:[%d]: Start HW reset!\n", __func__, __LINE__);
	gpio_direction_output(touch_chip->rst_gpio, 0);
	usleep_range(1000,1500);
	gpio_direction_output(touch_chip->rst_gpio, 1);
	//msleep(250);
//	msleep(1);
//	gpio_set_value(touch_chip->rst_gpio, 1);
//	printk("[Elan] %s:[%d] Joe\n", __func__, __LINE__);
//	msleep(1000); //mark by leo
//	printk("[Elan] %s:[%d] Joe\n", __func__, __LINE__);
	return 0;
}

static int elan_ktf3k_ts_set_power_source(struct i2c_client *client, u8 state)
{
	uint8_t cmd[] = {CMD_W_PKT, 0x40, 0x00, 0x01};
	int length = 0;

	dev_dbg(&client->dev, "[elan] %s: enter\n", __func__);
    /*0x52 0x40 0x00 0x01  =>    Battery Mode
       0x52 0x41 0x00 0x01  =>    USB and AC Adapter Mode
      */
	cmd[1] |= state & 0x0F;

	dev_dbg(&client->dev,
		"[elan] dump cmd: %02x, %02x, %02x, %02x\n",
		cmd[0], cmd[1], cmd[2], cmd[3]);
      dev_info(&private_ts->client->dev, "Update power source to %d\n", state);	
      down(&pSem);
      length = i2c_master_send(client, cmd, sizeof(cmd));
      up(&pSem);
	if (length != sizeof(cmd)) {
		dev_err(&client->dev,
			"[elan] %s: i2c_master_send failed\n", __func__);
		return -EINVAL;
	}

	return 0;
}

/*
static int elan_ktf3k_ts_get_power_source(struct i2c_client *client)
{
	int rc = 0;
	uint8_t cmd[] = {CMD_R_PKT, 0x40, 0x00, 0x01};
	uint8_t buf[4] = {0};

	//rc = elan_ktf2k_ts_get_data(client, cmd, buf, 4);
	rc = elan_ktf3k_ts_read_command(client, cmd, 4, buf, 4);
	if (rc < 0)
		return rc;

	return 0;
}
*/

static void update_power_source(void){
      unsigned power_source = now_usb_cable_status;
      if(private_ts == NULL || work_lock) return;
	// Send power state 1 if USB cable and AC charger was plugged on. 
      elan_ktf3k_ts_set_power_source(private_ts->client, power_source != USB_NO_Cable);
}

int cable_status(unsigned cable_status){ 
      now_usb_cable_status = cable_status;
      update_power_source();
      return 0;
}

static int elan_ktf3k_ts_recv_data(struct i2c_client *client, uint8_t *buf, int size)
{

	int rc, bytes_to_recv = size;

	if (buf == NULL)
		return -EINVAL;

	memset(buf, 0, bytes_to_recv);
	rc = i2c_master_recv(client, buf, bytes_to_recv);

	if (rc != bytes_to_recv) {
	 // add by leo for double trigger problem ++	
		if(CheckHelloPacket==0)
			printk("[Elan] %s: waiting for hello packet\n", __func__);
		else
	 // add by leo for double trigger problem --
			dev_err(&client->dev, "[elan] %s: i2c_master_recv error?! \n", __func__);
		rc = i2c_master_recv(client, buf, bytes_to_recv);
		return -EINVAL;
	}

	return rc;
}

static void elan_ktf3k_ts_report_data(struct i2c_client *client, uint8_t *buf)
{
//	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
//	struct input_dev *idev = touch_chip->input_dev;
	uint16_t x, y, touch_size;
	uint16_t fbits=0, checksum=0;
	uint8_t i, num;
	static uint8_t size_index[10] = {35, 35, 36, 36, 37, 37, 38, 38, 39, 39};
	uint16_t active = 0;
	uint8_t idx=IDX_FINGER;
	
	num = buf[2] & 0xf; 
	for (i=0; i<34;i++)
		checksum +=buf[i];

	if ((num < 3) || ((checksum & 0x00ff) == buf[34])) { 
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
		for(i = 0; i < FINGER_NUM; i++){
			active = fbits & 0x1;
			if(active || mTouchStatus[i]){
				input_mt_slot(touch_chip->input_dev, i);
				input_mt_report_slot_state(touch_chip->input_dev, MT_TOOL_FINGER, active);
				if(active){
					elan_ktf3k_ts_parse_xy(&buf[idx], &x, &y);
					y = touch_chip->abs_y_max - y;
					touch_size = ((i & 0x01) ? buf[size_index[i]] : (buf[size_index[i]] >> 4)) & 0x0F;
					if(touch_size == 0)
						touch_size = 1;
					if (touch_size <= 7)
						touch_size = touch_size << 5;
					else 
						touch_size = 255;
										
					x = touch_chip->abs_x_max - x; //add by leo
					
					input_report_abs(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, touch_size);
					input_report_abs(touch_chip->input_dev, ABS_MT_PRESSURE, touch_size);
					input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_Y, y);
					if(unlikely(gPrint_point))
						printk("[elan] finger id=%d X=%d y=%d size=%d\n", i, x, y, touch_size);
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		input_sync(touch_chip->input_dev);
	} // checksum
	else {
		checksum_err +=1;
		touch_debug(DEBUG_ERROR, "[elan] Checksum Error %d byte[2]=%X\n", checksum_err, buf[2]);
	}   
     	
	return;
}

static void elan_ktf3k_ts_report_data2(struct i2c_client *client, uint8_t *buf)
{
//	struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
//	struct input_dev *idev = ts->input_dev;
	uint16_t x, y, touch_size, pressure_size;
	uint16_t fbits=0, checksum=0;
	uint8_t i, num;
	uint16_t active = 0; 
	uint8_t idx=IDX_FINGER;

	num = buf[2] & 0xf;
	for (i=0; i<34;i++)
		checksum +=buf[i];

	if ( (num < 3) || ((checksum & 0x00ff) == buf[34])) {   
		fbits = buf[2] & 0x30;	
		fbits = (fbits << 4) | buf[1]; 
    	//input_report_key(idev, BTN_TOUCH, 1);
		for(i = 0; i < FINGER_NUM; i++){
			active = fbits & 0x1;
			if(active || mTouchStatus[i]){
				input_mt_slot(touch_chip->input_dev, i);
				input_mt_report_slot_state(touch_chip->input_dev, MT_TOOL_FINGER, active);
				if(active){
					elan_ktf3k_ts_parse_xy(&buf[idx], &x, &y);
					//x = x > ts->abs_x_max ? 0 : ts->abs_x_max - x;
					
// add by leo for display resolution mapping ++					
#ifdef MAPPING_TOUCH_RESOLUTION_TO_DISPLAY
					if(unlikely(gPrint_point))
						touch_debug(DEBUG_INFO, "[elan] original finger id=%d x=%d y=%d\n", i, x, y); 
					x = (x*ELAN_X_MAX)/ELAN_X_RESOLUTION;
					y = (y*ELAN_Y_MAX)/ELAN_Y_RESOLUTION;
#endif
// add by leo for display resolution mapping --				
					x = x > touch_chip->abs_x_max ? touch_chip->abs_x_max : x;
					y = y > touch_chip->abs_y_max ? touch_chip->abs_y_max : y;
					//x = touch_chip->abs_x_max - x; // mark by leo
					
					touch_size = buf[35 + i];
					pressure_size = buf[45 + i];	 
					input_report_abs(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, touch_size);
					input_report_abs(touch_chip->input_dev, ABS_MT_PRESSURE, pressure_size);
					input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_X, x);
					input_report_abs(touch_chip->input_dev, ABS_MT_POSITION_Y, y);
					if(unlikely(gPrint_point))
						touch_debug(DEBUG_INFO, "[elan] finger id=%d x=%d y=%d size=%d pressure=%d\n", i, x, y, touch_size, pressure_size);

					//printk("[Elan] %s END \n",__func__);	// add by leo for touch performance test
				}
			}
			mTouchStatus[i] = active;
			fbits = fbits >> 1;
			idx += 3;
		}
		input_sync(touch_chip->input_dev);
	} // checksum
	else {
		checksum_err +=1;
		touch_debug(DEBUG_ERROR, "[elan] Checksum Error %d byte[2]=%X\n", checksum_err, buf[2]);
	}
	return;
}

static void process_resp_message(struct elan_ktf3k_ts_data *ts, const unsigned char *buf,
	                                                      unsigned int size){
      int major, minor;

      if(buf == NULL) return;
      switch(buf[1] & 0xF0){
      case 0x00:// firmware version
          major = ((buf[1] & 0x0f) << 4) | ((buf[2] & 0xf0) >> 4);
          minor = ((buf[2] & 0x0f) << 4) | ((buf[3] & 0xf0) >> 4);
          ts->fw_ver = major << 8 | minor;
          FW_VERSION = ts->fw_ver;
          break;
      case 0x20: // Rough calibrate response
          major = buf[2] & 0x0F;	  	
          touch_debug(DEBUG_INFO, "Get the Rough Calibrate result:%d\n", major);
          break;
      case 0x40: // the power source
          major = buf[1] & 0x0f;
          touch_debug(DEBUG_INFO, "Get the power source:%d\n", major);
      case 0x50: // the power state
          major = buf[1] & 0x0f;
	   if(major==0)printk("[Elan] %s: Sleep Mode\n", __func__);
	   if(major==1)printk("[Elan] %s: Idle Scan Mode\n", __func__);
	   if(major==8)printk("[Elan] %s: Normal Mode\n", __func__);
          touch_debug(DEBUG_INFO, " Power Status: %d\n", major);
          break;
      case 0xF0: // firmware id
          major = ((buf[1] & 0x0f) << 4) | ((buf[2] & 0xf0) >> 4);
          minor = ((buf[2] & 0x0f) << 4) | ((buf[3] & 0xf0) >> 4);
          ts->fw_id = major << 8 | minor;
          FW_ID = ts->fw_id;
          break;	   
      default: 
          touch_debug(DEBUG_INFO, "[elan] Get unknow packet {0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", buf[0], buf[1], buf[2], buf[3]);	
      }
}

static void elan_ktf3k_ts_work_func(struct work_struct *work)
{
	int rc;
	//struct timeval now;					// add by leo for touch performance test
	//suseconds_t diff,diff_1,diff_2;			// add by leo for touch performance test

	//struct elan_ktf3k_ts_data *ts = container_of(work, struct elan_ktf3k_ts_data, work);
	uint8_t buf[NEW_PACKET_SIZE + 4] = { 0 };
	uint8_t buf1[NEW_PACKET_SIZE] = { 0 };
	uint8_t buf2[NEW_PACKET_SIZE] = { 0 };

	//do_gettimeofday(&now);				//add by leo for touch performance test
	//diff = now.tv_usec; /* microseconds */	//add by leo for touch performance test
	//printk("[Elan] %s Start \n",__func__);	//add by leo for touch performance test

	if(work_lock!=0) {
		touch_debug(DEBUG_INFO, "Firmware update during touch event handling\n");
		//enable_irq(touch_chip->irq); // add by leo
		return;
	}

#ifndef ELAN_BUFFER_MODE
	rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf, 40);
#else
	down(&pSem);
	rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf, 4); // read the first four bytes	//BUFFER MODE FIRST step: received 4 byte
	//do_gettimeofday(&now);				//add by leo for touch performance test
	//diff_1 = now.tv_usec - diff;			//add by leo for touch performance test
#endif 
	if (rc < 0)
	{
		up(&pSem);
		//enable_irq(touch_chip->irq); //add by leo
		return;
	}
#ifndef ELAN_BUFFER_MODE
	elan_ktf3k_ts_report_data2(touch_chip->client, buf);
#else
	switch(buf[0]){
		case NORMAL_PKT:
		rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf+4, 40); // read the finger report packet.
		up(&pSem);
		elan_ktf3k_ts_report_data(touch_chip->client, buf+4);
		// Second package
		if ((buf[1] == 2) || (buf[1] == 3)) {
			rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf1,PACKET_SIZE);
			if (rc < 0){
				//enable_irq(touch_chip->irq); //add by leo
				return;
			}
			elan_ktf3k_ts_report_data(touch_chip->client, buf1);
		}
		
		// Final package
		if (buf[1] == 3) {
			rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf2, PACKET_SIZE);
			if (rc < 0){
				//enable_irq(touch_chip->irq); //add by leo
				return;
			}
			elan_ktf3k_ts_report_data(touch_chip->client, buf2);
		}
		break;
		case NEW_NOMARL_PKT:
			rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf+4, NEW_PACKET_SIZE); // read the finger report packet.	//SECOND step: received 55 byte
			up(&pSem);
			//do_gettimeofday(&now);		//add by leo for touch performance test
			//diff_2 = now.tv_usec - diff;	//add by leo for touch performance test
			elan_ktf3k_ts_report_data2(touch_chip->client, buf+4);
			// Second package
			if ((buf[1] == 2) || (buf[1] == 3)) {	//THIRD step: received 55 byte again (if we are too busy to received and FW detected time out)
				rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf1,NEW_PACKET_SIZE);		
				if (rc < 0){
					//enable_irq(touch_chip->irq); //add by leo
					return;
				}
				elan_ktf3k_ts_report_data2(touch_chip->client, buf1);
			}
			
			// Final package
			if (buf[1] == 3) {	//FOURTH step: received 55 byte again (if we are too busy to received and FW detected time out)
				rc = elan_ktf3k_ts_recv_data(touch_chip->client, buf2, NEW_PACKET_SIZE);	
				if (rc < 0){
					//enable_irq(touch_chip->irq); //add by leo
					return;
				}
				elan_ktf3k_ts_report_data2(touch_chip->client, buf2);
			}
			//do_gettimeofday(&now);		//add by leo for touch performance test
			//diff = now.tv_usec - diff;		//add by leo for touch performance test
			//printk("Elapsed time 1: %lu, Elapsed time 2: %lu, Elapsed time 3: %lu\n", diff_1,diff_2,diff); //add by leo for touch performance test
		break;
		case CMD_S_PKT:
			up(&pSem);
			process_resp_message(touch_chip, buf, 4);
		break;
		case HELLO_PKT:
			up(&pSem);
			if(buf[0]==0x55 && buf[1]==0x55 && buf[2]==0x55 && buf[3]==0x55){
				CheckHelloPacket = 1; // add by leo for double trigger problem
				printk(KERN_ERR "[Elan] %s: Get hello packet {0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", __func__, buf[0], buf[1], buf[2], buf[3]);
			}
		break;
		default:
			up(&pSem);
			touch_debug(DEBUG_INFO, "[elan] Get unknow packet {0x%02X, 0x%02X, 0x%02X, 0x%02X}\n", buf[0], buf[1], buf[2], buf[3]);
	}		 
#endif
	//enable_irq(touch_chip->irq);
}

static irqreturn_t elan_ktf3k_ts_irq_handler(int irq, void *dev_id)
{
	//printk("[Elan] %s ++ \n",__func__);
	//disable_irq_nosync(touch_chip->irq); // add by leo
	queue_work(touch_chip->elan_wq, &touch_chip->work);

	return IRQ_HANDLED;
}

static int elan_ktf3k_ts_register_interrupt(struct i2c_client *client)
{
	int err = 0;
	
//	printk("[Elan] elan_ktf3k_ts_register_interrupt %d++ \n",touch_chip->irq);
	
	err = request_irq(touch_chip->irq, elan_ktf3k_ts_irq_handler,
			IRQF_TRIGGER_FALLING, touch_chip->client->name, touch_chip->client);
	if (err)
		printk(KERN_ERR "[Elan] %s: request_irq %d failed\n", __func__, touch_chip->irq);

	return err;
}

static ssize_t elan_touch_switch_name(struct switch_dev *sdev, char *buf)
{
	return sprintf(buf, "ELAN-%4.4x-%4.4x\n", 
		   private_ts->fw_id, private_ts->fw_ver);
}

static ssize_t elan_touch_switch_state(struct switch_dev *sdev, char *buf)
{ 
      	return sprintf(buf, "%s\n", "0");
}

#ifdef _ENABLE_DBG_LEVEL
static int ektf_proc_read(struct seq_file *buf, void *v)
{	
	touch_debug(DEBUG_MESSAGES, "call proc_read\n");
	
	seq_printf(buf, "Debug Level: Release Date: %s\n","2011/10/05");
	return 0;
}

static int ektf_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
	char procfs_buffer_size = 0; 
	int i, ret = 0;
	unsigned char procfs_buf[PROC_FS_MAX_LEN+1] = {0};
	unsigned int command;

	procfs_buffer_size = count;
	if(procfs_buffer_size > PROC_FS_MAX_LEN ) 
		procfs_buffer_size = PROC_FS_MAX_LEN+1;
	
	if( copy_from_user(procfs_buf, buffer, procfs_buffer_size) ) 
	{
		touch_debug(DEBUG_ERROR, " proc_write faied at copy_from_user\n");
		return -EFAULT;
	}

	command = 0;
	for(i=0; i<procfs_buffer_size-1; i++)
	{
		if( procfs_buf[i]>='0' && procfs_buf[i]<='9' )
			command |= (procfs_buf[i]-'0');
		else if( procfs_buf[i]>='A' && procfs_buf[i]<='F' )
			command |= (procfs_buf[i]-'A'+10);
		else if( procfs_buf[i]>='a' && procfs_buf[i]<='f' )
			command |= (procfs_buf[i]-'a'+10);
		
		if(i!=procfs_buffer_size-2)
			command <<= 4;
	}

	command = command&0xFFFFFFFF;
      switch(command){
      case 0xF1: 
	  	 gPrint_point = 1; 
	  	 break;
      case 0xF2: 
	  	 gPrint_point = 0; 
	  	 break;
      case 0xFF:
	      	ret = elan_ktf3k_ts_rough_calibrate(private_ts->client);
		break;
	}
	touch_debug(DEBUG_INFO, "Run command: 0x%08X  result:%d\n", command, ret);

	return count; // procfs_buffer_size;
}
#endif // #ifdef _ENABLE_DBG_LEV


#ifdef FIRMWARE_UPDATE_WITH_HEADER
//#define FIRMWARE_PAGE_SIZE 132
//static unsigned char touch_firmware[] = {
// #include "touchFW/fw_data.b"
// }; 
//#define SIZE_PER_PACKET 4

static int sendI2CPacket(struct i2c_client *client, const unsigned char *buf, unsigned int length){
     int ret, i, retry_times = 10;
     for(i = 0; i < length; i += ret){
            ret  = i2c_master_send(client, buf + i,  length < SIZE_PER_PACKET ? length : SIZE_PER_PACKET);
            if(ret <= 0){
	          retry_times--;
		    ret = 0;
	      }  
	     if(ret < (length < SIZE_PER_PACKET ? length : SIZE_PER_PACKET)){
	          touch_debug(DEBUG_INFO,"Sending packet broken\n");
		    //printk("[ektf3k]:Sending packet broken\n");	  
	     } 

	     if(retry_times < 0){
	          touch_debug(DEBUG_INFO,"Failed sending I2C touch firmware packet.\n");
		   //printk("[ektf3k]:Failed sending I2C touch firmware packet.\n");	  
	          break;
	     }
     }

     return i;
}

static int recvI2CPacket(struct i2c_client *client, unsigned char *buf, unsigned int length){
     int ret, i, retry_times = 10;
     for(i = 0; i < length; i += ret){
            ret  = i2c_master_recv(client, buf + i,  length - i);
            if(ret <= 0){
	          retry_times--;
		    ret = 0;
	      }  
				
	     if(retry_times < 0){
	          touch_debug(DEBUG_INFO,"Failed sending I2C touch firmware packet.\n");
	          //printk("[ektf3k]:Failed sending I2C touch firmware packet.\n");
	          break;
	     }
     }

     return i;
}

static int firmware_update_header(struct i2c_client *client, unsigned char *firmware, unsigned int pages_number){

    int ret, i, mode;
    int retry_times = 3, write_times; 
    unsigned char packet_data[8] = {0};
    unsigned char isp_cmd[4] = {0x54, 0x00, 0x12, 0x34};
    unsigned char nb_isp_cmd[4] = {0x45, 0x49, 0x41, 0x50};
    unsigned char *cursor; 
    int boot_code = 0;
    struct elan_ktf3k_ts_data *ts = i2c_get_clientdata(client);
	
    if(ts == NULL) 
        return -1;

    touch_debug(DEBUG_INFO, "Start firmware update!\n");
    disable_irq(client->irq);  // Blocking call no need to do extra wait
    wake_lock(&ts->wakelock);
    work_lock = 1;
    elan_ktf3k_ts_hw_reset(client);
    msleep(20); // add by leo
    // Step 1: Check boot code version
    //boot_code = gpio_get_value(ts->intr_gpio);
    boot_code = 1; // add by leo
    if(boot_code == 0){ // if the boot code is old
        touch_debug(DEBUG_INFO, "The firmware update of old boot code\n");
        if(recvI2CPacket(client, packet_data, 4) < 0) 
	      goto fw_update_failed;

	  touch_debug(DEBUG_INFO, "The received bytes 0x%X 0x%X 0x%X 0x%X\n", packet_data[0], packet_data[1], 
	  	           packet_data[2], packet_data[3]);
        if(packet_data[0] == 0x55 && packet_data[1] == 0x55 && packet_data[2] == 0x80 && packet_data[3] == 0x80)
	      touch_debug(DEBUG_INFO, "In the recovery mode\n");

        if(sendI2CPacket(client, isp_cmd, (unsigned int)sizeof(isp_cmd)) < 0) // get into ISP mode
	      goto fw_update_failed;	  
    }else{ // if the boot code is new
        touch_debug(DEBUG_INFO, "The firmware update of new boot code\n");
        if(sendI2CPacket(client, nb_isp_cmd, sizeof(nb_isp_cmd)) < 0) // get into ISP mode
	      goto fw_update_failed;
    }
	
    msleep(100);
    packet_data[0] = 0x10; 
    if(sendI2CPacket(client, packet_data, 1) < 0) // send dummy byte
	      goto fw_update_failed;
	  
    cursor = firmware;
    touch_debug(DEBUG_INFO, "pages_number=%d\n", pages_number);
    for(i = 0; i < pages_number; i++){
        write_times = 0; 
page_write_retry:
	  touch_debug(DEBUG_MESSAGES, "Update page number %d\n", i);

          int sendCount;
          if((sendCount = sendI2CPacket(client, cursor, FIRMWARE_PAGE_SIZE)) != FIRMWARE_PAGE_SIZE){
	      dev_err(&client->dev, "Fail to Update page number %d\n", i);
		goto fw_update_failed;
	  }
          touch_debug(DEBUG_INFO, "sendI2CPacket send %d bytes\n", sendCount);

          msleep(30); //add by leo
		  
          int recvCount;
          if((recvCount = recvI2CPacket(client, packet_data, FIRMWARE_ACK_SIZE)) != FIRMWARE_ACK_SIZE){
	      dev_err(&client->dev, "Fail to Update page number %d\n", i);
	      goto fw_update_failed;
	  }

          touch_debug(DEBUG_INFO, "recvI2CPacket recv %d bytes: %x %x\n", recvCount, packet_data[0], packet_data[1]);

	  if(packet_data[0] != 0xaa || packet_data[1] != 0xaa){
	      touch_debug(DEBUG_INFO, "message received: %02X %02X Page %d rewrite\n", packet_data[0], packet_data[1], i);
		if(write_times++ > 3)
		    goto fw_update_failed;
			
		goto page_write_retry;
	  }
		  
	  cursor += FIRMWARE_PAGE_SIZE;
    }
	
    elan_ktf3k_ts_hw_reset(client);
/*   	
    if(boot_code)
        msleep(2000);
    else		
        msleep(300);
*/
    //check irq
    wait_for_IRQ_Low(client, 500000);//500ms * 10
	
    if(recvI2CPacket(client, packet_data, 4) < 0) 
	      goto fw_update_failed;	
    __fw_packet_handler(ts->client, 1);
    ret = 0;
    goto fw_update_finish;
fw_update_failed:
    ret = -1;
fw_update_finish:  
    work_lock = 0;
    wake_unlock(&ts->wakelock);
    enable_irq(client->irq);	
    touch_debug(DEBUG_INFO, "Finish the touch firmware update!\n");
    return ret; 
}

#endif

/*
#ifdef TOUCH_STRESS_TEST
static void  stress_poll_data(struct work_struct * work)
{
	bool status;
	u8 count[7];
	status = elan_ktf3k_ts_recv_data(private_ts->client, count, 7);
	if(status < 0)
		printk("Read touch sensor data fail\n");

	if(poll_mode ==0)
		msleep(5);

	queue_delayed_work(stress_work_queue, &elan_poll_data_work, poll_mode);
}

int elan_stress_open(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	if( !atomic_dec_and_test(&touch_char_available)){
		atomic_inc(&touch_char_available);
		return -EBUSY; //already open
	}
	return 0;          //success
}


int elan_stress_release(struct inode *inode, struct file *filp)
{
	printk("%s\n", __func__);
	atomic_inc(&touch_char_available); //release the device
	return 0;          //success
}

long elan_stress_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int err = 1;

	printk("[elan_stress_ioctl]%d\n", cmd);
	if (_IOC_TYPE(cmd) != STRESS_IOC_MAGIC)
	return -ENOTTY;
	if (_IOC_NR(cmd) > STRESS_IOC_MAXNR)
	return -ENOTTY;
	if (_IOC_DIR(cmd) & _IOC_READ)
	err = !access_ok(VERIFY_WRITE, (void __user *)arg, _IOC_SIZE(cmd));
	else if (_IOC_DIR(cmd) & _IOC_WRITE)
	err =  !access_ok(VERIFY_READ, (void __user *)arg, _IOC_SIZE(cmd));
	if (err) return -EFAULT;
	switch (cmd) {
		case STRESS_POLL_DATA:
			if (arg == STRESS_IOCTL_START_HEAVY){
				printk("touch sensor heavey\n");
			poll_mode = START_HEAVY;
			queue_delayed_work(stress_work_queue, &elan_poll_data_work, poll_mode);
			}
			else if (arg == STRESS_IOCTL_START_NORMAL){
				printk("touch sensor normal\n");
				poll_mode = START_NORMAL;
				queue_delayed_work(stress_work_queue, &elan_poll_data_work, poll_mode);
			}
			else if  (arg == STRESS_IOCTL_END){
			printk("touch sensor end\n");
			cancel_delayed_work_sync(&elan_poll_data_work);
			}
			else
				return -ENOTTY;
			break;
		break;
	default: //redundant, as cmd was checked against MAXNR
	    return -ENOTTY;
	}
	return 0;
}

static struct file_operations stress_fops = {
		.owner =    THIS_MODULE,
		.unlocked_ioctl =	elan_stress_ioctl,
		.open =		elan_stress_open,
		.release =	elan_stress_release,
		};
#endif
*/

static int ektf_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, ektf_proc_read, NULL);
}
static int fw_check_ProcFile_open(struct inode *inode, struct  file *file) {
  return single_open(file, fw_check_ProcFile_read, NULL);
}
static int  fw_update_ProcFile_open(struct inode *inode, struct  file *file) {
  return single_open(file, fw_update_ProcFile_read, NULL);
}
static int calibration_ProcFile_open(struct inode *inode, struct  file *file) {
  return single_open(file, calibration_ProcFile_read, NULL);
}

static const struct file_operations ProcFile_fops = {
	.owner = THIS_MODULE,
	.open = ektf_proc_open,
	.read = seq_read,
	.write = ektf_proc_write,
};
static const struct file_operations fw_check_ProcFile_fops = {
	.owner = THIS_MODULE,
	.open = fw_check_ProcFile_open,
	.read = seq_read,
	.write = fw_check_ProcFile_write,
};
static const struct file_operations  fw_update_ProcFile_fops = {
	.owner = THIS_MODULE,
	.open = fw_update_ProcFile_open,
	.read = seq_read,
	.write = fw_update_ProcFile_write,
};
static const struct file_operations calibration_ProcFile_fops = {
	.owner = THIS_MODULE,
	.open = calibration_ProcFile_open,
	.read = seq_read,
	.write = calibration_ProcFile_write,
};

// add by leo for AP firmware update bar ++
static ssize_t fw_check_ProcFile_read(struct seq_file *buf, void *v){

	return seq_printf(buf, "%d\n",proc_fw_check_result);
}
static ssize_t fw_check_ProcFile_write(struct file *filp, const char __user *buff, unsigned long len, void *data){

	char messages[80] = {0};
			
	if (len >= 80) {
		printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	printk("[Elan] %s: input command:%s\n", __func__, messages);

	if (messages[0]!='1'){
		printk("[Elan] %s: command not support\n", __func__);
	}else{

		printk("[Elan] %s: Vendor = ELAN_KTF3K\n", __func__);
		printk("[Elan] %s: FW ID = 0x%4.4x\n", __func__, touch_chip->fw_id);
		printk("[Elan] %s: FW Version = 0x%4.4x\n", __func__, touch_chip->fw_ver);
		printk("[Elan] %s: FW Test Version = 0x%4.4x\n", __func__, touch_chip->fw_test_ver);
		printk("[Elan] %s: Touch Panel = %d\n", __func__, touch_chip->tp);

		if((touch_chip->tp==0)||(touch_chip->tp==2)){	//WINTEK Panel
			proc_fw_check_result = check_fw_version(touch_firmware_forWintek, sizeof(touch_firmware_forWintek), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
		}else if((touch_chip->tp==3)){ // AUO Panel
			proc_fw_check_result = check_fw_version(touch_firmware_forAUO, sizeof(touch_firmware_forAUO), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
		}else{
			proc_fw_check_result = -456;
		}

		if(proc_fw_check_result>0){
			printk("[Elan] %s: need to do FW update (%d)\n", __func__, proc_fw_check_result);
		}else if (proc_fw_check_result<0){
			printk("[Elan] %s: chipFW too old, touchFW unknow or tp unknow (%d)\n", __func__, proc_fw_check_result);
		}else{
			printk("[Elan] %s: FW version ok (%d)\n", __func__, proc_fw_check_result);
		}
	}
	return len;
}

static ssize_t fw_update_ProcFile_read(struct seq_file *buf, void *v){

	return seq_printf(buf, "%d\n",proc_fw_update_result);
}
static ssize_t fw_update_ProcFile_write(struct file *filp, const char __user *buff, unsigned long len, void *data){

	char messages[80] = {0};
			
	if (len >= 80) {
		printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	printk("[Elan] %s: input command:%s\n", __func__, messages);

	if (messages[0]!='1'){
		printk("[Elan] %s: command not support\n", __func__);
	}else{

		if((touch_chip->tp==0)||(touch_chip->tp==2)){	//WINTEK Panel
			proc_fw_update_result = firmware_update_header(touch_chip->client, touch_firmware_forWintek, sizeof(touch_firmware_forWintek)/FIRMWARE_PAGE_SIZE);
			
		}else if((touch_chip->tp==3)){ //AUO Panel
			proc_fw_update_result = firmware_update_header(touch_chip->client, touch_firmware_forAUO, sizeof(touch_firmware_forAUO)/FIRMWARE_PAGE_SIZE);
			
		}else{
			proc_fw_update_result = -456;
		}

		if(proc_fw_update_result<0){
			printk("[Elan] %s: touch firmware update failed (%d)\n", __func__, proc_fw_update_result);
		}else{
			printk("[Elan] %s: touch firmware update succeed (%d)\n", __func__, proc_fw_update_result);
		}
	}	
	return len;
}

static ssize_t calibration_ProcFile_read(struct seq_file *buf, void *v){

	return seq_printf(buf, "%d\n",proc_calibration_result);
}
static ssize_t calibration_ProcFile_write(struct file *filp, const char __user *buff, unsigned long len, void *data){

	char messages[80] = {0};

	if (len >= 80) {
		printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}	
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	printk("[Elan] %s: input command:%s\n", __func__, messages);

	if (messages[0]!='1'){
		printk("[Elan] %s: command not support\n", __func__);
	}else{

		proc_calibration_result =elan_ktf3k_ts_rough_calibrate(private_ts->client);
		if(proc_calibration_result<0){
			printk("[Elan] %s: Calibration Failed (%d)\n", __func__, proc_calibration_result);
		}
		printk("[Elan] %s: Calibration Succeed (%d)\n", __func__, proc_calibration_result);
	}	
	return len;
}

void EKTH3374AY_create_proc_file(void){

	printk("[Elan] %s: \n",__func__);
	
	// check fw version
	fw_check_ProcFile = proc_create(EKTH3374AY_PROC_FW_CHECK_NAME, 0666, NULL, &fw_check_ProcFile_fops);
	if(fw_check_ProcFile){
		printk("[Elan] %s: fw_check_ProcFile create sucessed!\n",__func__);
	}
	else{
		printk("[Elan] %s: fw_check_ProcFile create failed!\n",__func__);
	}

	// fw update
	fw_update_ProcFile = proc_create(EKTH3374AY_PROC_FW_UPDATE_NAME, 0666, NULL, &fw_update_ProcFile_fops);
	if(fw_update_ProcFile){
		printk("[Elan] %s: fw_update_ProcFile create sucessed!\n",__func__);
	}
	else{
		printk("[Elan] %s: fw_update_ProcFile create failed!\n",__func__);
	}

	//calibration
	calibration_ProcFile = proc_create(EKTH3374AY_PROC_CALIBRATION_NAME, 0666, NULL, &calibration_ProcFile_fops);
	if(calibration_ProcFile){
		printk("[Elan] %s: calibration_ProcFile create sucessed!\n",__func__);
	}
	else{
		printk("[Elan] %s: calibration_ProcFile create failed!\n",__func__);
	}
}

void EKTH3374AY_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;

    printk("[Elan] %s: \n",__func__);

    remove_proc_entry(EKTH3374AY_PROC_FW_CHECK_NAME, &proc_root);
    remove_proc_entry(EKTH3374AY_PROC_FW_UPDATE_NAME, &proc_root);
    remove_proc_entry(EKTH3374AY_PROC_CALIBRATION_NAME, &proc_root);
}
// add by leo for AP firmware update bar --

void elan_ktf3k_ts_gpio_config(void)
{
	int err = 0;

	printk("[Elan] %s: intr_gpio=%d, rst_gpio=%d, pwr_en_gpio=%d \n", __func__, touch_chip->intr_gpio,touch_chip->rst_gpio,touch_chip->pwr_en_gpio);
			
	/*init Touch power pin*/
	err = gpio_request(touch_chip->pwr_en_gpio, "ElanTouch-power");
	if (err < 0)
		printk(KERN_ERR "[Elan] %s:Failed to request GPIO%d (ElanTouch-power) error=%d\n", __func__, touch_chip->pwr_en_gpio, err);

	err = gpio_direction_output(touch_chip->pwr_en_gpio, 1);
	if (err){
		printk(KERN_ERR "[Elan] %s:Failed to set reset direction, error=%d\n", __func__, err);
		gpio_free(touch_chip->pwr_en_gpio);
	}
	
	/*init INTERRUPT pin*/
	err = gpio_request(touch_chip->intr_gpio, "ElanTouch-irq");
	if(err < 0)
		printk(KERN_ERR "[Elan] %s:Failed to request GPIO%d (ElanTouch-interrupt) error=%d\n", __func__, touch_chip->intr_gpio, err);

	err = gpio_direction_input(touch_chip->intr_gpio);
	if (err){
		printk(KERN_ERR "[Elan] %s:Failed to set interrupt direction, error=%d\n", __func__, err);
		gpio_free(touch_chip->intr_gpio);
	}
	
	touch_chip->irq = gpio_to_irq(touch_chip->intr_gpio);
	printk("[Elan] %s: irq=%d \n", __func__, touch_chip->irq);
	
	/*init RESET pin*/
	err = gpio_request(touch_chip->rst_gpio, "ElanTouch-reset");
	if (err < 0)
		printk(KERN_ERR "[Elan] %s:Failed to request GPIO%d (ElanTouch-reset) error=%d\n", __func__, touch_chip->rst_gpio, err);

	err = gpio_direction_output(touch_chip->rst_gpio, 0);
	if (err){
		printk(KERN_ERR "[Elan] %s:Failed to set reset direction, error=%d\n", __func__, err);
		gpio_free(touch_chip->rst_gpio);
	}
	
//	msleep(1);
//	gpio_set_value(touch_chip->rst_gpio, 1);
//	printk("[Elan] %s:[%d] Joe\n", __func__, __LINE__);
//	msleep(1000);
//	printk("[Elan] %s:[%d] Joe\n", __func__, __LINE__);
}

static int elan_ktf3k_ts_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int err = 0;
	struct elan_ktf3k_i2c_platform_data *pdata;
	
	printk("[Elan]: elan_ktf3k_ts_probe ++ \n");
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[Elan] %s:[%d]: i2c check functionality error.\n", __func__, __LINE__);
		err = -ENODEV;
		goto err_check_functionality_failed;
	}

	touch_chip = kzalloc(sizeof(struct elan_ktf3k_ts_data), GFP_KERNEL);
	if (touch_chip == NULL) {
		printk(KERN_ERR "[Elan] %s:[%d]: allocate elan_ktf3k_ts_data failed.\n", __func__, __LINE__);
		err = -ENOMEM;
		goto err_alloc_data_failed;
	}

	touch_chip->elan_wq = create_singlethread_workqueue("elan_wq");
	if (!touch_chip->elan_wq) {
		printk(KERN_ERR "[Elan] %s:[%d]: create workqueue failed.\n", __func__, __LINE__);
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

	INIT_WORK(&touch_chip->work, elan_ktf3k_ts_work_func);
	touch_chip->client = client;
	i2c_set_clientdata(client, touch_chip);
	pdata = client->dev.platform_data;

	if (likely(pdata != NULL)) {
		touch_chip->intr_gpio = pdata->intr_gpio;
		touch_chip->rst_gpio = pdata->rst_gpio;
		touch_chip->pwr_en_gpio = pdata->pwr_en_gpio;
	}
	
	elan_ktf3k_ts_gpio_config();
					
	sema_init(&pSem, 1);
	err = elan_ktf3k_ts_setup(touch_chip->client);
	if (err < 0) {
		printk(KERN_ERR "[Elan] %s:[%d]: Main code fail\n", __func__, __LINE__);
		touch_chip->status = 0;
		RECOVERY = 1;
		err = 0;
		goto err_detect_failed;
	}

	touch_chip->status = 1; // set I2C status is OK;
	wake_lock_init(&touch_chip->wakelock, WAKE_LOCK_SUSPEND, "elan_touch");
	if(err==0x80)
		printk("[Elan] %s:[%d]: Touch is in boot mode!\n", __func__, __LINE__);

	touch_chip->input_dev = input_allocate_device();
	if (touch_chip->input_dev == NULL) {
		err = -ENOMEM;
		printk(KERN_ERR "[Elan] %s:[%d]: Failed to allocate input device.\n", __func__, __LINE__);
		goto err_input_dev_alloc_failed;
	}
	touch_chip->input_dev->name = "elan-touchscreen";  

	//set_bit(BTN_TOUCH, ts->input_dev->keybit);
	touch_chip->abs_x_max =  pdata->abs_x_max;
	touch_chip->abs_y_max =	 pdata->abs_y_max;
	touch_chip->abs_x_min =  pdata->abs_x_min;
	touch_chip->abs_y_min =	 pdata->abs_y_min;
	printk("[Elan] %s:[%d]: Max X=%d, Max Y=%d, Min_X=%d, Min_Y=%d \n", __func__, __LINE__, touch_chip->abs_x_max, touch_chip->abs_y_max, touch_chip->abs_x_min, touch_chip->abs_y_min);

	input_mt_init_slots(touch_chip->input_dev, FINGER_NUM,0);
//	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_X, pdata->abs_y_min,  pdata->abs_y_max, 0, 0); // for 800 * 1280 
//	input_set_abs_params(ts->input_dev, ABS_MT_POSITION_Y, pdata->abs_x_min,  pdata->abs_x_max, 0, 0);// for 800 * 1280 
	input_set_abs_params(touch_chip->input_dev, ABS_MT_POSITION_X, touch_chip->abs_x_min,  touch_chip->abs_x_max, 0, 0);
	input_set_abs_params(touch_chip->input_dev, ABS_MT_POSITION_Y, touch_chip->abs_y_min,  touch_chip->abs_y_max, 0, 0);
	input_set_abs_params(touch_chip->input_dev, ABS_MT_TOUCH_MAJOR, 0, MAX_FINGER_PRESSURE, 0, 0);
	input_set_abs_params(touch_chip->input_dev, ABS_MT_PRESSURE, 0, MAX_FINGER_PRESSURE, 0, 0);
	input_set_abs_params(touch_chip->input_dev, ABS_MT_TRACKING_ID, 0, FINGER_NUM, 0, 0);	

	__set_bit(EV_ABS, touch_chip->input_dev->evbit);
	__set_bit(EV_SYN, touch_chip->input_dev->evbit);
	__set_bit(EV_KEY, touch_chip->input_dev->evbit);

	err = input_register_device(touch_chip->input_dev);
	if (err) {
		printk("[Elan] %s:[%d]: unable to register %s input device \n", __func__, __LINE__,touch_chip->input_dev->name);
		goto err_input_register_device_failed;
	}
//	printk("[Elan] %s:[%d] Joe\n", __func__, __LINE__);
	elan_ktf3k_ts_register_interrupt(touch_chip->client);

	if (gpio_get_value(touch_chip->intr_gpio) == 0) {
		printk("[Elan] %s:[%d]: handle missed interrupt \n", __func__, __LINE__);
		elan_ktf3k_ts_irq_handler(touch_chip->irq, touch_chip);
	}

// add by leo for different touch panel ++	
	touch_chip->tp = Read_TP_ID();
	//printk("[Elan] %s: touch panel is %d\n", __func__,touch_chip->tp);
	if(touch_chip->tp==0)printk("[Elan] %s: touch panel is Wintek 30 ohm (%d)\n", __func__,touch_chip->tp);
	if(touch_chip->tp==1)printk("[Elan] %s: touch panel is X (%d)\n", __func__,touch_chip->tp);
	if(touch_chip->tp==2)printk("[Elan] %s: touch panel is Wintek 65 ohm (%d)\n", __func__,touch_chip->tp);
	if(touch_chip->tp==3)printk("[Elan] %s: touch panel is AUO (%d)\n", __func__,touch_chip->tp);

#ifdef FIRMWARE_UPDATE_WITH_HEADER
	if(build_version==1){ // 1:eng
		if((touch_chip->tp==0)||(touch_chip->tp==2)){	//WINTEK Panel
			err = check_fw_version(touch_firmware_forWintek, sizeof(touch_firmware_forWintek), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
	      		printk("[Elan] %s:[%d]: check_Wintek_fw_version (%d) \n", __func__, __LINE__,err);
	      		if(RECOVERY ||err > 0)
					firmware_update_header(client, touch_firmware_forWintek, sizeof(touch_firmware_forWintek)/FIRMWARE_PAGE_SIZE);
				
		}else if(touch_chip->tp==3){	//AUO Panel
			err = check_fw_version(touch_firmware_forAUO, sizeof(touch_firmware_forAUO), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
	      		printk("[Elan] %s:[%d]: check_AUO_fw_version (%d) \n", __func__, __LINE__,err);
	      		if(RECOVERY ||err > 0)
					firmware_update_header(client, touch_firmware_forAUO, sizeof(touch_firmware_forAUO)/FIRMWARE_PAGE_SIZE);
		}
	}
#endif
// add by leo for different touch panel --

/*
#ifdef FIRMWARE_UPDATE_WITH_HEADER	
      err = check_fw_version(touch_firmware, sizeof(touch_firmware), touch_chip->fw_id,touch_chip->fw_ver,touch_chip->fw_test_ver);
      printk("[Elan] %s:[%d]: check_fw_version (%d) \n", __func__, __LINE__,err);
      if(RECOVERY ||err > 0)
          firmware_update_header(client, touch_firmware, sizeof(touch_firmware)/FIRMWARE_PAGE_SIZE);
#endif
*/

#ifdef CONFIG_HAS_EARLYSUSPEND
	touch_chip->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 21;
	touch_chip->early_suspend.suspend = elan_ktf3k_ts_early_suspend;
	touch_chip->early_suspend.resume = elan_ktf3k_ts_late_resume;
	register_early_suspend(&touch_chip->early_suspend);
#endif

	private_ts = touch_chip;

	//elan_ktf2k_touch_sysfs_init();
    	touch_chip->attrs.attrs = elan_attr;
	err = sysfs_create_group(&client->dev.kobj, &touch_chip->attrs);
	if (err) {
		printk("[Elan] %s:[%d]: Not able to create the sysfs \n", __func__, __LINE__);
	}
	
#ifdef _ENABLE_DBG_LEVEL
	dbgProcFile = proc_create(PROC_FS_NAME, 0600, NULL, &ProcFile_fops);
	if (dbgProcFile == NULL) 
	{
		remove_proc_entry(PROC_FS_NAME, NULL);
		printk("[Elan] %s:[%d]: Could not initialize /proc/%s \n", __func__, __LINE__, PROC_FS_NAME);
	}
	else
	{
		printk("[Elan] %s: /proc/%s created \n", __func__, PROC_FS_NAME);
	}
#endif // #ifdef _ENABLE_DBG_LEVEL

	EKTH3374AY_create_proc_file();// add by leo for AP firmware update bar ++

#ifdef TOUCH_STRESS_TEST
      	stress_work_queue = create_singlethread_workqueue("i2c_touchsensor_wq");
	if(!stress_work_queue){
		printk("[Elan] %s:[%d]: Unable to create stress_work_queue workqueue \n", __func__, __LINE__);
	}
	INIT_DELAYED_WORK(&elan_poll_data_work, stress_poll_data);
	touch_chip->misc_dev.minor  = MISC_DYNAMIC_MINOR;
	touch_chip->misc_dev.name = "touchpanel";
	touch_chip->misc_dev.fops = &stress_fops;
	err = misc_register(&touch_chip->misc_dev);
	if (err) {
		printk("[Elan] %s:[%d]: Unable to register %s \\misc device \n", __func__, __LINE__,touch_chip->misc_dev.name);
	}
#endif
	/* Register Switch file */
	touch_chip->touch_sdev.name = "touch";
	touch_chip->touch_sdev.print_name = elan_touch_switch_name;
	touch_chip->touch_sdev.print_state = elan_touch_switch_state;
	if(switch_dev_register(&touch_chip->touch_sdev) < 0){
		printk("[Elan] %s:[%d]: switch_dev_register for dock failed! \n", __func__, __LINE__);
		//goto exit;
	}
	switch_set_state(&touch_chip->touch_sdev, 0);
	
	printk("[Elan] %s:[%d]: Start touchscreen %s in interrupt mode \n", __func__, __LINE__, touch_chip->input_dev->name);

	touch_chip->firmware.minor = MISC_DYNAMIC_MINOR;
	touch_chip->firmware.name = "elan-iap";
	touch_chip->firmware.fops = &elan_touch_fops;
	touch_chip->firmware.mode = S_IFREG|S_IRWXUGO; 

	if (misc_register(&touch_chip->firmware) < 0)
		printk("[Elan] %s:[%d]: misc_register failed!! \n", __func__, __LINE__);
	else
		printk("[Elan] %s:[%d]: misc_register finished!! \n", __func__, __LINE__);

//Joe	update_power_source();
  
	printk("[Elan] Elan EKTH3374AY Touch driver ver.%s \n",ASUS_DRIVER_VERSION);
	return 0;

err_input_register_device_failed:
	if (touch_chip->input_dev)
		input_free_device(touch_chip->input_dev);

err_input_dev_alloc_failed:
err_detect_failed:
	if (touch_chip->elan_wq)
		destroy_workqueue(touch_chip->elan_wq);

err_create_wq_failed:
	kfree(touch_chip);

err_alloc_data_failed:
err_check_functionality_failed:

	return (build_version == 1 ? 0 : err);
}

static int elan_ktf3k_ts_remove(struct i2c_client *client)
{

	elan_touch_sysfs_deinit();

	unregister_early_suspend(&touch_chip->early_suspend);
	free_irq(touch_chip->irq, touch_chip);

	if (touch_chip->elan_wq)
		destroy_workqueue(touch_chip->elan_wq);
	input_unregister_device(touch_chip->input_dev);
	wake_lock_destroy(&touch_chip->wakelock);
#ifdef TOUCH_STRESS_TEST
	misc_deregister(&touch_chip->misc_dev);
#endif
	kfree(touch_chip);
#ifdef _ENABLE_DBG_LEVEL
	remove_proc_entry(PROC_FS_NAME, NULL);
#endif

	EKTH3374AY_remove_proc_file(); // add by leo for AP firmware update bar ++

	return 0;
}

void force_release_pos(struct i2c_client *client)
{
	int i;

	for (i=0; i < FINGER_NUM; i++) {
		if (mTouchStatus[i] == 0) continue;
		input_mt_slot(touch_chip->input_dev, i);
		input_mt_report_slot_state(touch_chip->input_dev, MT_TOOL_FINGER, 0);
		mTouchStatus[i] = 0;
	}
	
	input_sync(touch_chip->input_dev);
}

static int elan_ktf3k_ts_suspend(struct i2c_client *client, pm_message_t mesg)
{
	int rc = 0;

	printk("[Elan] %s:[%d]: enter (calibration_flag = %d)\n", __func__, __LINE__,calibration_flag);

	disable_irq(touch_chip->irq);
	force_release_pos(touch_chip->client);
	rc = cancel_work_sync(&touch_chip->work);

	if(work_lock == 0)
	    rc = elan_ktf3k_ts_set_power_state(touch_chip->client, PWR_STATE_DEEP_SLEEP);

#ifdef EKTH3374AY_POWER_OFF_WHEN_SUSPEND
	//gpio_set_value(160, 0); 					// +5V (=3.3v)
	gpio_set_value(touch_chip->pwr_en_gpio, 0);	// 1.8V
	gpio_set_value(touch_chip->rst_gpio, 0);		// Reset Pin
	CheckHelloPacket = 0; // add by leo for double trigger problem
#endif	

	return 0;
}

static int elan_ktf3k_ts_resume(struct i2c_client *client)
{
	int rc = 0, retry = 5;

	printk("[Elan] %s:[%d]: enter (calibration_flag = %d)\n", __func__, __LINE__,calibration_flag);

#ifndef EKTH3374AY_POWER_OFF_WHEN_SUSPEND
	if(work_lock == 0){
	    do {
			rc = elan_ktf3k_ts_set_power_state(touch_chip->client, PWR_STATE_NORMAL);
			rc = elan_ktf3k_ts_get_power_state(touch_chip->client);
			if (rc != PWR_NORMAL_STATE && rc != PWR_IDLE_STATE)
					printk("[Elan] %s:[%d]: wake up tp failed! err = %d \n", __func__, __LINE__, rc);
			else
				break;
			
	    } while (--retry);
	}
	//force_release_pos(client);

	enable_irq(touch_chip->irq);
#endif

	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_ktf3k_ts_early_suspend(struct early_suspend *h)
{
	if(!calibration_flag)	// add by leo for keeping touch chip awake when calibration
		elan_ktf3k_ts_suspend(touch_chip->client, PMSG_SUSPEND);
}

static void elan_ktf3k_ts_late_resume(struct early_suspend *h)
{
	int err=0, iRetry=10;
	//int power_status=0;
	unsigned char power_status_cmd[4] = {0x53, 0x50, 0x00, 0x01}; //read power status
	
	if(unlikely(gPrint_point))
		printk("[Elan] %s ++ \n", __func__);

#ifdef EKTH3374AY_POWER_OFF_WHEN_SUSPEND

	enable_irq(touch_chip->irq);

	//gpio_set_value(160, 1); 					// +5V (=3.3v)
	gpio_set_value(touch_chip->pwr_en_gpio, 1);	// 1.8V
	gpio_set_value(touch_chip->rst_gpio, 1);		// Reset Pin

//mark by leo for interrupt received hello packet ++
/*
	for(iRetry=10;iRetry>0;iRetry--){
		elan_ktf3k_ts_hw_reset(touch_chip->client);
		
		err = __hello_packet_handler(touch_chip->client);
		if (err < 0){ 	//received hello packet failed, if time out then retry 10 times else break
			if (err == -ETIME && iRetry>0) {
				printk(KERN_ERR "[Elan] %s: wait main hello timeout, reset it (Retry time = %d)\n", __func__, 10-iRetry);
			}else{
				printk("[Elan] %s: received hello packet failed (%d) \n", __func__, err);
				break;
			}
		}else{		//received hello packet success, check power status (DO NOT send cmd without recevied hello packet)
			power_status= elan_ktf3k_ts_get_power_state(touch_chip->client);
			if (power_status != PWR_NORMAL_STATE && power_status != PWR_IDLE_STATE){
				printk("[Elan] %s:[%d]: wake up tp failed (%d) \n", __func__, __LINE__, err);
			}
			break;
		}
	}
*/
//mark by leo for interrupt received hello packet --

	msleep(500);

	down(&pSem);
	err = i2c_master_send(touch_chip->client, power_status_cmd, 4); //note: DO NOT send command before recevied hello packet
	up(&pSem);
	if(err != 4){
		printk("[Elan] %s:[%d]: i2c_master_send check status command failed (%d) \n", __func__, __LINE__, err);
		//return -EINVAL;
	}

#endif

	elan_ktf3k_ts_resume(touch_chip->client);

	if(unlikely(gPrint_point))
		printk("[Elan] %s -- \n", __func__);
}
#endif

static const struct i2c_device_id elan_ktf3k_ts_id[] = {
	{ ELAN_KTF3K_NAME, 0 },
	{ }
};

static struct i2c_driver ektf3k_ts_driver = {
	.probe		= elan_ktf3k_ts_probe,
	.remove		= elan_ktf3k_ts_remove,
#ifndef CONFIG_HAS_EARLYSUSPEND
	.suspend	= elan_ktf3k_ts_suspend,
	.resume		= elan_ktf3k_ts_resume,
#endif
	.id_table	= elan_ktf3k_ts_id,
	.driver		= {
		.name = ELAN_KTF3K_NAME,
	},
};

static int elan_ktf3k_ts_init(void)
{
	printk("[Elan]: elan_ktf3k_ts_init ++ \n");
	return i2c_add_driver(&ektf3k_ts_driver);
}

static void __exit elan_ktf3k_ts_exit(void)
{
	i2c_del_driver(&ektf3k_ts_driver);
	return;
}

module_init(elan_ktf3k_ts_init);
module_exit(elan_ktf3k_ts_exit);

MODULE_DESCRIPTION("ELAN KTF3K Touchscreen Driver");
MODULE_LICENSE("GPL");
