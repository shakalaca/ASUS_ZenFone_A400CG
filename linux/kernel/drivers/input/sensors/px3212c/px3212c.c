#include <linux/delay.h>
#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
//#include <asm/mach-types.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/earlysuspend.h>
#include <linux/HWVersion.h>
#include <asm/intel_scu_ipc.h>//add by leo for power enable ++
#include <linux/earlysuspend.h>//add by leo for early_suspend ++
#include <linux/proc_fs.h>//add by leo for proc file ++
#include <linux/microp_notify.h>
#include <linux/microp_api.h>
#include <linux/microp_pin_def.h>

//add by leo for read build version ++
/*
 *build_vsersion mean TARGET_BUILD_VARIANT
 *eng:1
 *userdebug:2
 *user:3
 */
extern int build_version;
//add by leo for read build version --

#define MSIC_VPROG2CNT 0xd7
#define VPROG2_OFF 0xe4
#define VPROG2_ON 0xff

#define slaveAddr	0x1C
#define PX3212C_I2C_DEVICE_NAME "px3212c"
#define DRIVER_VERSION		"1.0.0.0"
static int  PX3212C_INTERRUPT_GPIO=0;	//pro_int#=63

/* Register Address Info */
#define SYSTEM_CONFIG 	0x00
#define INT_STATUS		0x01
#define INT_CLEANR_MNR 	0x02
#define MAKE_ID 			0x03
#define PRODUCT_ID 		0x04
#define REVISION_ID 		0x05

#define IR_DATA_LOW	0x0A
#define IR_DATA_HIGH	0x0B

#define PS_DATA_LOW	0x0E
#define PS_DATA_HIGH	0x0F
#define PS_OBJ_STATUS   	0x0F


#define PS_CONFIG		0x20
#define PS_LED_CONTROL	0x21
#define PS_MEAN_TIME	0x23
#define PS_VCALI_L		0x28	//PS Calibration Register Lower Byte
#define PS_VCALI_H		0x29	//PS Calibration Register Higher Byte
#define PS_THDL_L		0x2A	//PS Low Threshold Lower Byte
#define PS_THDL_H		0x2B	//PS Low Threshold Higher Byte
#define PS_THDH_L		0x2C	//PS High Threshold Lower  Byte
#define PS_THDH_H		0x2D	//PS High Threshold HigherByte


extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);

struct px3212c_platform_data {
	int gpio;
};

struct px3212c_i2c_data{
	struct i2c_client *i2c_client;

	struct input_dev *ps_input_dev;
	int id;

	int ps_status;
	unsigned int  interrupt_flag;
	struct early_suspend early_suspend;

	int ps_power_state;
	int irq;

	struct wake_lock wake_lock;
};

static struct px3212c_i2c_data *data = NULL;
static struct i2c_client *px3212c_i2c_client = NULL;


/* Sensors Interrupt Workqueue */
static struct workqueue_struct *px3212c_wq;
struct delayed_work px3212c_work;
static void px3212c_work_function(struct work_struct *work);
static int interruptStatus=0;

static DEFINE_MUTEX(px3212c_mutex);

/* Sensors Calibration Info */
#define CalibrationValue_len 11
#define CValue_len 5				 


//#define Default_PS_ThresholdValue 90 	 //mark by leo for proximity vendor calibration ++
#define Default_PS_ThresholdValue 300 //add by leo for proximity vendor calibration ++
#define CalibrationRetryTimes 5;

static int PS_CalibrationValue =Default_PS_ThresholdValue;
// add by leo for high/low threshold calibration ++
static int PS_HighCalibrationValue=175;
static int PS_LowCalibrationValue=50;
// add by leo for high/low threshold calibration --

// add by tom for high/low threshold calibration ++
static int PS_Default_High_Threshold=80;
static int PS_Default_Low_Threshold=50;
static int PS_ER_Threshold = 250;
static int PS_Crosstalk_Fail_Threshold = 300;
// add by tom for high/low threshold calibration --

static int PS_VendorCalibrationValue =0; // add by leo for proximity vendor calibration ++


static int PS_CalibrationRetryCount=CalibrationRetryTimes;
static int PS_VendorCalibrationRetryCount=CalibrationRetryTimes;

typedef enum{ 
	FALSE= 0,
	TRUE 
} boolean; 

static boolean PS_AlreadyCalibration = FALSE;
static boolean PS_AlreadyVendorCalibration = FALSE;	//add by leo for proximity vendor calibration ++



/* Sensors Threshold Info */


//static int PS_HIGH_THD =Default_PS_ThresholdValue;
//static int PS_LOW_THD	=Default_PS_ThresholdValue;
//18% gray card 4cm: ps_data=407, threshold(high,low)=(410,200), crosstalk=100, (410-100,200-100)=(310,100)
static int PS_HIGH_THD = 250; //310
static int PS_LOW_THD = 100; //100

/* Other */
static int POWER_OFF 	= 0;
static int POWER_ON 	= 1;
static int PS_CurrentStatus = -1;
//static int ALS_StatusBeforeSuspend = -1;// mask by leo because system may disable light sensor by ioctl from framework 	++
static int RESET_INPUT_DEVICE = -1;
static int limit_PS_CrossTalkValue = 350; //add by leo for proximity vendor calibration ++

/* Sensors IO Control Info */

#define PROXIMITYSENSOR_IOCTL_MAGIC 'p'
#define PROXIMITYSENSOR_IOCTL_GET_ENABLED	_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 1, int *) 
#define PROXIMITYSENSOR_IOCTL_ENABLE			_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 2, int *) 
#define PROXIMITYSENSOR_IOCTL_GET_STATUS		_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 3, int *)
#define PROXIMITYSENSOR_IOCTL_CALIBRATION	_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 4, int *) // modify by leo for high/low threshold calibration ++
#define PROXIMITYSENSOR_IOCTL_VCALIBRATION	_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 5, int *) // add by leo for proximity vendor calibration ++

static int ps_initial(void);
static int ps_enable(struct px3212c_i2c_data *px3212c_data,int  power);

static int ps_set_threshold(int high_thd,int low_thd);

static int ps_get_calibration_value(struct px3212c_i2c_data *px3212c_data);
static int ps_get_vendor_calibration_value(void);	// add by leo for proximity vendor calibration ++
static int CharToInt(unsigned char CValue[]);
static int ps_calibration(struct px3212c_i2c_data *px3212c_data, int iControl);
static int ps_vendor_calibration(void);	// add by leo for proximity vendor calibration ++
static int px3212c_EC_event_notify(struct notifier_block *this, unsigned long event, void *ptr); // add by tom for EC event notify
static void px3212c_early_suspend(struct early_suspend *h);
static void px3212c_later_resume(struct early_suspend *h);
//static int clear_interrupt_flag(void);
static int px3212c_default_calibration_proximity(void); // add by tom for default threshold
static int px3212c_calibration_proximity(struct px3212c_i2c_data *px3212c_data);// add by leo for proc file ++
static int px3212c_get_raw_data(void);
static int px3212c_get_systemMode(void); // add by tom for get system mode ++
	
//extern int proximity_sensor_status(int status); // add by leo for proximity notify touch

// add by leo for proc file ++
#define	PX3212C_PROC_FILE	"px3212c_debug"
//static struct proc_dir_entry *px3212c_proc_file;
// add by leo for proc file --

#define PX3212C_DEBUGMSG 	0

#define PS_DEBUGMSG		1
#define PX3212C_STATUSMSG	1
static unsigned int debug = 0;

/* Calibration Info*/
static int px3212c_calibration_init(void);
	
/* EC notity */
// If equal "1" when proximity sensor is Enable
static int IsEnableSensor = 0;
// If equal "1" when linked Pad
static int IsLinkedPad = 0;

static struct notifier_block pad_ps_notifier = {
	 .notifier_call = px3212c_EC_event_notify, // callback function
};
//=========================================================================================

// add by leo for proc file ++
static ssize_t px3212c_register_read(char *page, char **start, off_t off, int count, int *eof, void *idata)
{
	ssize_t sprintf_count = 0;
	if(debug==0)debug=1;
	else debug=0;
	if(PX3212C_STATUSMSG) printk( "[%s]	debug = %d \n",__FUNCTION__,debug);
	sprintf_count += sprintf(page + sprintf_count, "debug meaasge (%s)\n", ((debug==1)?"on":"off"));
	return sprintf_count;
}
static ssize_t px3212c_register_write(struct file *filp, const char __user *buff, unsigned long len, void *idata)
{
	char messages[80] = {0};
	int ret = 0, i = 0, iloop = 0, count = 0, cmd = 0, en = 0, iControl = 0, ithd = 0;
	u32 reg_address=0, reg_value=0;
	static u8 temp[4]={0};
	static u8 total_input[80]={0};
	struct px3212c_i2c_data *px3212c_data = data;
	u8* input_cmd = (char*)kcalloc(count,sizeof(char),GFP_KERNEL);
	
	if (len >= 80) {
		if(PX3212C_STATUSMSG) printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	   
	if(PX3212C_STATUSMSG) printk("[%s]	input command: %s\n",__FUNCTION__,messages);
    switch(messages[0]){
		case '0': 
			debug = 0; 
			if(PX3212C_STATUSMSG) printk( "[%s]	debug = %d \n",__FUNCTION__,debug);
			return len;
			break;
		case '1': 
			debug = 1; 
			if(PX3212C_STATUSMSG) printk( "[%s]	debug = %d \n",__FUNCTION__,debug);
			return len;
			break;
	 }
	if ((messages[0]=='l')&&(messages[1]=='o')&&(messages[2]=='g')){
		if(&messages[4]==NULL)
			if(PX3212C_STATUSMSG) printk("[%s]	show debug message\n",__FUNCTION__);
		if(&messages[4]!=NULL)
			sscanf (&messages[4], "%d", &en);

		debug=(en==1)?1:0;
		if(PX3212C_STATUSMSG) printk( "[%s]	debug meaasge (%s)\n",__FUNCTION__,(debug==1)?"on":"off");
		return len;

	

	}else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='e')&&(messages[4]=='n')){
		//ps_en+en
		if(&messages[6]!=NULL){
			sscanf (&messages[6], "%d", &en);
		}else{
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d)\n", __FUNCTION__,px3212c_data->ps_power_state);
			return len;
		}
		
		if(en==1){
			ret = ps_enable(px3212c_data,POWER_ON);
			if(ret < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
			}	
		}else if(en==0){
			ret = ps_enable(px3212c_data,POWER_OFF);
			if(ret < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
			}
		}
		return len;

	}else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[4]=='t')&&(messages[5]=='h')&&(messages[6]=='d')){
		//ps_lthd+value or ps_hthd+value
		if(&messages[8]!=NULL){
			sscanf (&messages[8], "%d", &ithd);
		}else{
			if(PX3212C_STATUSMSG) printk( "[%s]		PS_THDH=%d, PS_THDL=%d \n",__FUNCTION__,PS_HIGH_THD, PS_LOW_THD);
			return len;
		}

		if((messages[3]=='h')&&(ithd>0)){
			PS_HIGH_THD =ithd;
		}else if((messages[3]=='l')&&(ithd>0)){
			PS_LOW_THD=ithd;
		}
		ret = ps_set_threshold(PS_HIGH_THD, PS_LOW_THD);
		if(ret < 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_LOW_THD,PS_HIGH_THD);
		}
		return len;


	}else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='c')&&(messages[4]=='a')&&(messages[5]=='l')&&(messages[6]=='i')){
		//ps_cali+value
		if(&messages[8]!=NULL){
			sscanf (&messages[8], "%d", &iControl);
		}else{
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration Value = %d \n",__FUNCTION__,PS_CalibrationValue);
			return len;
		}

		ret = ps_calibration(px3212c_data,iControl);
		if(ret<0){
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration (set calibration value) Failed (%d)\n",__FUNCTION__,ret);
		}	

		PS_AlreadyCalibration=FALSE;
		PS_CalibrationRetryCount=CalibrationRetryTimes;
		ret = px3212c_calibration_proximity(px3212c_data);
		if(ret<0){
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
		}
		return len;
		
	}else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='v')&&(messages[4]=='c')&&(messages[5]=='a')&&(messages[6]=='l')&&(messages[7]=='i')){
		//ps_vcali+value
		if(&messages[9]!=NULL){
			sscanf (&messages[9], "%d", &iControl);
		}else{
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Value = %d \n",__FUNCTION__,PS_VendorCalibrationValue);
			return len;
		}

		if(iControl==0){
			//Clean PS Vendor Calibration Register
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_H,0);
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
				return ret;
			}
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_L,0);
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
				return ret;
			}

		}else if(iControl==1){
			PS_AlreadyVendorCalibration=TRUE;
			ret = ps_enable(px3212c_data,POWER_ON);
				if(ret < 0)  {
					if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
			}
			msleep(150);
			
			ret = ps_vendor_calibration();
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Failed (%d)\n",__FUNCTION__,ret);
			}

			ret = ps_enable(px3212c_data,POWER_OFF);
				if(ret < 0)  {
					if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
			}
			PS_AlreadyVendorCalibration=FALSE;
			PS_VendorCalibrationRetryCount=CalibrationRetryTimes;
		}
		return len;
		
	}else if(messages[0]=='w'){
		cmd=1;
	}else if(messages[0]=='r'){
		cmd=2;
		
	}else{
		if(PX3212C_STATUSMSG) printk("[%s]	Unknow Command\n",__FUNCTION__);
		return len;
	}

	// count input command
	for(i=0;i<100;i++){
		if(messages[i]=='x'){
			count ++;
		}else if(messages[i]=='\n'){
			if(unlikely(debug))printk("[%s]	command number = %d\n",__FUNCTION__,count);
			break;
		}
	}

	// transfor input command from ASCII code to HEX
	for(iloop = 0; iloop < count; iloop++){
		temp[0] = messages[iloop*5 + 2];
		temp[1] = messages[iloop*5 + 3];
		temp[2] = messages[iloop*5 + 4];
		temp[3] = messages[iloop*5 + 5];
		sscanf(temp, "%x", &total_input[iloop]);
	}
	

	if(PX3212C_STATUSMSG) printk("[%s]	input_cmd = %s ",__FUNCTION__,cmd==1?"write":"read");
	for(i=0;i<count;i++){
		input_cmd[i]=total_input[i]&0xff;
		if(PX3212C_STATUSMSG) printk("0x%02x ", input_cmd[i]);
	}
	if(PX3212C_STATUSMSG) printk("\n");

	// send test command
	reg_address = input_cmd[0];
	reg_value = input_cmd[1];

	if(cmd==1){ //write
		ret = i2c_smbus_write_byte_data(px3212c_i2c_client, reg_address, reg_value);
		if(ret< 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]	Write 0x%02X into 0x%02X failed (%d)\n",__FUNCTION__,reg_value,reg_address,ret);
			return ret;
		}
		if(PX3212C_STATUSMSG) printk( "[%s]	write 0x%02X into 0x%02X\n",__FUNCTION__,reg_value,reg_address);
		
	}else if(cmd==2){ //read
		ret = i2c_smbus_read_byte_data(px3212c_i2c_client, reg_address);
		if(ret< 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]	Read data from 0x%02X failed (%d)\n",__FUNCTION__,reg_address,ret);
			return ret;
		}
		if(PX3212C_STATUSMSG) printk( "[%s]	0x%02x value is 0x%02x \n",__FUNCTION__,reg_address,ret);
		reg_value=ret;
	}

	return len;
}
/*
void px3212c_create_proc_file(void)
{
	px3212c_proc_file = create_proc_entry(PX3212C_PROC_FILE, 0666, NULL);
	if(px3212c_proc_file){
		px3212c_proc_file->read_proc = px3212c_register_read;
		px3212c_proc_file->write_proc = px3212c_register_write;
	}
	else{
		if(PX3212C_STATUSMSG) printk( "[%s]	create_proc_entry failed\n",__FUNCTION__);
	}
}

void px3212c_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    if(unlikely(debug))printk( "[%s] remove proc files \n",__FUNCTION__);
    remove_proc_entry(PX3212C_PROC_FILE, &proc_root);
}
// add by leo for proc file --
*/
/*
static int px3212c_setup_power(struct px3212c_i2c_data *px3212c_data)
{
	int err=0;
	err = intel_scu_ipc_iowrite8(MSIC_VPROG2CNT, VPROG2_ON);
	if(err<0){
		if(PX3212C_STATUSMSG) printk( "[%s]	intel_scu_ipc_iowrite8 Failed\n",__FUNCTION__);
	}
	
	if(PX3212C_STATUSMSG) printk( "[%s]	px3212c_setup_power - FINISHED\n",__FUNCTION__);
	return 0;
}
*/
static int px3212c_check_device(struct px3212c_i2c_data *px3212c_data)
{
	int pid=0, rid=0,mid=0;

	pid = i2c_smbus_read_byte_data(px3212c_i2c_client, PRODUCT_ID);
	if(pid< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PRODUCT_ID Failed\n",__FUNCTION__);
		return pid;
	}
	rid = i2c_smbus_read_byte_data(px3212c_i2c_client, REVISION_ID);
	if(rid < 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read REVERSION_ID Failed\n",__FUNCTION__);
		return rid;
	}
	mid = i2c_smbus_read_byte_data(px3212c_i2c_client, MAKE_ID);
	if(mid < 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read MAKE_ID Failed\n",__FUNCTION__);
		return mid;
	}

	if(mid == 0x02 && pid == 0x02 && rid == 0x01){
		px3212c_data->id = pid;
		if(PX3212C_STATUSMSG) printk( "[%s]		Check Device OK !! Product ID = 0x%x \n",__FUNCTION__,pid);
		return 0;
	}else{
		if(PX3212C_STATUSMSG) printk( "[%s]		Check Device Failed\n",__FUNCTION__);
		if(PX3212C_STATUSMSG) printk( "[%s]		Product ID = 0x%x, Reversion ID = 0x%x, Make ID = 0x%x\n",__FUNCTION__,pid,rid,mid);
	}	
	return -ENODEV;
}

static int px3212c_default_calibration_proximity(void){
	
			int ret = 0;
			if( (AX_MicroP_getHWID() == HW_ID_SR1)||(AX_MicroP_getHWID() == P72_ER1_1_HWID)||(AX_MicroP_getHWID() == P72_ER1_2_HWID))
			{
				if(PX3212C_STATUSMSG) printk( "[%s]	ER PS Calibration Value = (%d, %d) \n",__FUNCTION__,(PS_Default_High_Threshold+PS_ER_Threshold),(PS_Default_Low_Threshold+PS_ER_Threshold));
				ret = ps_set_threshold((PS_Default_High_Threshold+PS_ER_Threshold),(PS_Default_Low_Threshold+PS_ER_Threshold));
				if(ret < 0)  {
					if(PX3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,(PS_Default_High_Threshold+PS_ER_Threshold),(PS_Default_Low_Threshold+PS_ER_Threshold));
					return ret;
				}
			}
			else if(PS_AlreadyVendorCalibration==FALSE)
			{
				if(PX3212C_STATUSMSG) printk( "[%s]	ER PS Calibration Value = (%d, %d) \n",__FUNCTION__,(PS_Default_High_Threshold+PS_Crosstalk_Fail_Threshold),(PS_Default_Low_Threshold+PS_Crosstalk_Fail_Threshold));
				ret = ps_set_threshold((PS_Default_High_Threshold+PS_Crosstalk_Fail_Threshold),(PS_Default_Low_Threshold+PS_Crosstalk_Fail_Threshold));
				if(ret < 0)  {
					if(PX3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,(PS_Default_High_Threshold+PS_Crosstalk_Fail_Threshold),(PS_Default_Low_Threshold+PS_Crosstalk_Fail_Threshold));
					return ret;
				}
			}
			else
			{
				if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration Value = (%d, %d) \n",__FUNCTION__,PS_Default_High_Threshold,PS_Default_Low_Threshold);
				ret = ps_set_threshold(PS_Default_High_Threshold,PS_Default_Low_Threshold);
				if(ret < 0)  {
					if(PX3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_HIGH_THD,PS_LOW_THD);
					return ret;
				}
			}
			return ret;
}
static int px3212c_calibration_proximity(struct px3212c_i2c_data *px3212c_data){

	int ret=0,err=0, iRetry=CalibrationRetryTimes ;
	if(unlikely(debug))printk( "[%s]		PS_AlreadyCalibration = %d \n",__FUNCTION__,PS_AlreadyCalibration);
	if(PS_AlreadyCalibration==FALSE){

		if(unlikely(debug))printk( "[%s]		PS Calibration Retry %d times.\n",__FUNCTION__,(iRetry-PS_CalibrationRetryCount));
		PS_CalibrationRetryCount --;
		
		ret = ps_get_calibration_value(px3212c_data);
		if(ret<0){
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Use Default Calibration Value (%d , %d) \n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);
			err = ps_set_threshold(PS_HighCalibrationValue,PS_LowCalibrationValue);
			if(err < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_HIGH_THD,PS_LOW_THD);
				return err;
			}
			return ret;
			
		}else{
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration Value = (%d, %d) \n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);

			PS_HIGH_THD = PS_HighCalibrationValue;
			PS_LOW_THD  = PS_LowCalibrationValue;

			ret = ps_set_threshold(PS_HIGH_THD,PS_LOW_THD);
			if(ret < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_HIGH_THD,PS_LOW_THD);
				return ret;
			}
			PS_AlreadyCalibration=TRUE;
			return 0;
		}
	}
	return 0;
}

//add by leo for proximity cross talk calibration ++
static int px3212c_vendor_calibration_proximity(struct px3212c_i2c_data *px3212c_data){

	unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;
	int ret=0,PS_CrossTalk=0, iRetry=CalibrationRetryTimes;;
	

	if(unlikely(debug))printk( "[%s]	PS Vendor Calibration Retry %d times.\n",__FUNCTION__,(iRetry-PS_VendorCalibrationRetryCount));
	PS_VendorCalibrationRetryCount --;
	

	PS_CrossTalk=ps_get_vendor_calibration_value();
	if(PS_CrossTalk<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		Use Default Crosstalk Value \n",__FUNCTION__);
		return PS_CrossTalk;
	}

	//write the value into ps vendor calibration register
	PS_VCALI_HByte = PS_CrossTalk >> 1;
	PS_VCALI_LByte = PS_CrossTalk & 0x01;	
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_H,PS_VCALI_HByte);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_L,PS_VCALI_LByte);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	//read the vendor calibration value from the register for double check
	PS_VCALI_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_H);
	PS_VCALI_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_L);
	if(PS_VCALI_HByte<0||PS_VCALI_LByte<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]Read PS_VCALI Failed\n",__FUNCTION__);
		return -1;
	}
	PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));
	if(PX3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Register = %d\n",__FUNCTION__,PS_CrossTalk);
	PS_VendorCalibrationValue = PS_CrossTalk;
	PS_AlreadyVendorCalibration=TRUE;
	
	return PS_CrossTalk;
}
//add by leo for proximity cross talk calibration --

//=========================================================================================


static ssize_t ProximitySensor_check_For_ATD_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	struct px3212c_i2c_data *px3212c_data = data;
	
	ret = px3212c_check_device(px3212c_data);
	if(ret < 0)  {
		return sprintf(buf,"0\n");
	}
	return sprintf(buf,"1\n");
}
static DEVICE_ATTR(pad_proximity_status, (S_IWUSR|S_IRUGO),ProximitySensor_check_For_ATD_test,NULL);

static ssize_t px3212c_get_ID(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	struct px3212c_i2c_data *px3212c_data = data;
	
	ret = px3212c_check_device(px3212c_data);
	if(ret < 0)  {
		//if(PX3212C_STATUSMSG) printk( "[%s]	px3212c_check_device Failed \n",__FUNCTION__);
		return sprintf(buf,"px3212c_check_device Failed\n");
	}
	if(PX3212C_STATUSMSG) printk( "[%s]	px3212c ID = 0x%x \n",__FUNCTION__,px3212c_data->id);

	return sprintf(buf,"px3212c ID = 0x%x \n",px3212c_data->id);
}
static DEVICE_ATTR(px3212c_id, (S_IWUSR|S_IRUGO),px3212c_get_ID,NULL);



static ssize_t ps_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{	
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long RAW_Data=0;

	value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
	value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
	if(value_LByte<0||value_HByte<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Read PS_DATA Failed \n",__FUNCTION__);
	}
	RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
	if(PX3212C_STATUSMSG) printk( "[%s]			PX3212C PS DATA = %d\n",__FUNCTION__,(int)RAW_Data);

	return sprintf(buf,"PS output data = %d\n",(int)RAW_Data);
}
static DEVICE_ATTR(ps_data, (S_IWUSR|S_IRUGO),ps_get_data,NULL);

static ssize_t ps_get_status(struct device *dev, struct device_attribute *attr, char *buf)
{	
	unsigned int value_HByte=0;
	struct px3212c_i2c_data *px3212c_data = data;
	
	value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
	px3212c_data->ps_status = (value_HByte & 0x80)>>7;

	if(PX3212C_STATUSMSG) printk( "[%s]	PS_OBJ = %d\n",__FUNCTION__,px3212c_data->ps_status);
	
	return sprintf(buf,"PS status = %d\n",px3212c_data->ps_status);
}
static DEVICE_ATTR(ps_status, (S_IWUSR|S_IRUGO),ps_get_status,NULL);

static ssize_t IR_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{	
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long IR_Data=0;
	
	value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
	value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
	IR_Data = ((value_HByte << 2) | (value_LByte & 0x03)); //0x03 = 000000 11

	if(PX3212C_STATUSMSG) printk( "[%s]	IR_DATA = %d\n",__FUNCTION__,(int)IR_Data);
	
	return sprintf(buf,"IR_DATA = %d\n",(int)IR_Data);
}
static DEVICE_ATTR(ir_data, (S_IWUSR|S_IRUGO),IR_get_data,NULL);

static ssize_t px3212c_interrupt_flag(struct device *dev, struct device_attribute *attr, char *buf)
{	
	int InterruptFlags=0;
	
	InterruptFlags = i2c_smbus_read_byte_data(px3212c_i2c_client, INT_STATUS);
	InterruptFlags = InterruptFlags & 0x03;

	return sprintf(buf,"INT_STATUS = 0x%x\n",InterruptFlags);
}
static DEVICE_ATTR(px3212c_int, (S_IWUSR|S_IRUGO),px3212c_interrupt_flag,NULL);



static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int SystemMode=0;
	struct px3212c_i2c_data *px3212c_data = data;

	SystemMode = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
	if(SystemMode< 0)  {
		//if(PX3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed\n",__FUNCTION__);
		return sprintf(buf,"Read SYSTEM_CONFIG Failed\n");
	}
	
	if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) ; SystemMode = 0x%x \n", __FUNCTION__,px3212c_data->ps_power_state, SystemMode);
	return sprintf(buf,"ps_enable(%d) \n",px3212c_data->ps_power_state);
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0, state=0;
	struct px3212c_i2c_data *px3212c_data = data;
	
	sscanf (buf, "%d", &state);
	
	if(buf[0]=='1'){
		ret = ps_enable(px3212c_data,POWER_ON);
		if(ret < 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
		}	
		IsEnableSensor= 1;
		if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor (%d)  \n",__FUNCTION__,IsEnableSensor);
	}else if(buf[0]=='0'){
		ret = ps_enable(px3212c_data,POWER_OFF);
		if(ret < 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
		}
		IsEnableSensor= 0;
		if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor (%d)  \n",__FUNCTION__,IsEnableSensor);
	}
	return count;
}
static DEVICE_ATTR(ps_en, (S_IWUSR|S_IRUGO), ps_enable_show , ps_enable_store);


static ssize_t ps_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
	unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
	unsigned long PS_THDH=0, PS_THDL=0;
	
	PS_THDH_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_H);
	PS_THDH_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_L);
	PS_THDL_HByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_H);
	PS_THDL_LByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_L);

	if(PS_THDH_HByte< 0||PS_THDH_LByte < 0||PS_THDL_HByte< 0||PS_THDL_LByte< 0)  {
		return sprintf(buf,"Read PS Threshold Failed\n");
	}
	PS_THDH = ((PS_THDH_HByte<<2)|(PS_THDH_LByte & 0x03));
	PS_THDL = ((PS_THDL_HByte <<2)|(PS_THDL_LByte & 0x03));
	
	return sprintf(buf,"PS Threshold = High(%d), Low(%d)\n",(int)PS_THDH,(int)PS_THDL);
}

static ssize_t ps_threshold_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0, ithreshold=0;

	sscanf (buf, "%d", &ithreshold);
	if(unlikely(debug))printk( "[%s]		ps_threshold_store ( ithreshold %d )\n",__FUNCTION__,(int)ithreshold);

	if(ithreshold>0){
		PS_HIGH_THD =ithreshold;
		PS_LOW_THD=ithreshold;
	}
	ret = ps_set_threshold(PS_HIGH_THD, PS_LOW_THD);
	if(ret < 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_LOW_THD,PS_HIGH_THD);
	}
	
	return count;
}
static DEVICE_ATTR(ps_thd, (S_IWUSR|S_IRUGO), ps_threshold_show , ps_threshold_store);

//add by leo for proximity threshold problem ++
static ssize_t ps_high_threshold_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0, high_thd=0;
	unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
	unsigned long PS_THDH=0;

	sscanf (buf, "%d", &high_thd);

mutex_lock(&px3212c_mutex);
	PS_THDH_HByte = high_thd >> 2;
	PS_THDH_LByte = high_thd & 0x03;

	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDH_H, PS_THDH_HByte);
	if(ret<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_THDH_HByte Failed (%d)\n",__FUNCTION__,ret);
	}
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDH_L, PS_THDH_LByte);
	if(ret<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_THDH_LByte Failed (%d)\n",__FUNCTION__,ret);
	}
mutex_unlock(&px3212c_mutex);

	PS_THDH_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_H);
	PS_THDH_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_L);

	if(PS_THDH_HByte< 0||PS_THDH_LByte < 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS threshold Failed\n",__FUNCTION__);
	}

	PS_THDH = ((PS_THDH_HByte<<2)|(PS_THDH_LByte & 0x03));
	if(PX3212C_STATUSMSG) printk( "[%s]		ps_set_high_threshold, PS_THDH=%d\n",__FUNCTION__,(int)PS_THDH);

	return count;
}
static DEVICE_ATTR(ps_hthd, (S_IWUSR|S_IRUGO), NULL , ps_high_threshold_store);

static ssize_t ps_low_threshold_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0,low_thd=0;
	unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
	unsigned long PS_THDL=0;

	sscanf (buf, "%d", &low_thd);

mutex_lock(&px3212c_mutex);
	PS_THDL_HByte = low_thd >> 2;
	PS_THDL_LByte = low_thd & 0x03;

	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDL_H, PS_THDL_HByte);
	if(ret<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_THDL_HByte Failed (%d)\n",__FUNCTION__,ret);
	}
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDL_L, PS_THDL_LByte);
	if(ret<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_THDL_LByte Failed (%d)\n",__FUNCTION__,ret);
	}
mutex_unlock(&px3212c_mutex);

	PS_THDL_LByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_L);
	if(PS_THDL_HByte< 0||PS_THDL_LByte< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_THDL_LByte Failed\n",__FUNCTION__);
	}

	PS_THDL = ((PS_THDL_HByte <<2)|(PS_THDL_LByte & 0x03));
	if(PX3212C_STATUSMSG) printk( "[%s]		ps_set_low_threshold, PS_THDL=%d \n",__FUNCTION__,(int)PS_THDL);

	return count;
}
static DEVICE_ATTR(ps_lthd, (S_IWUSR|S_IRUGO), NULL , ps_low_threshold_store);
//add by leo for proximity threshold problem --


static ssize_t ps_do_Calibration_show(struct device *dev, struct device_attribute *attr, char *buf){

	int ret=0;
	struct px3212c_i2c_data *px3212c_data=data;
	
	ret = ps_get_calibration_value(px3212c_data);
	if(ret<0){
		//if(PX3212C_STATUSMSG) printk( "[%s]	Read PS CalibrationValue Failed \n",__FUNCTION__);
		return sprintf(buf,"Read PS CalibrationValue Failed (%d)\n",ret);
	}
	
	return sprintf(buf,"PS Calibration Value =%010d&%010d\n",PS_HighCalibrationValue,PS_LowCalibrationValue);
}

static ssize_t ps_do_Calibration_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int iControl;
	int ret=0;
	struct px3212c_i2c_data *px3212c_data=data;
	
	sscanf (buf, "%d", &iControl); // 0: reset, 1: low threshold, 2: high threshold
	
	ret = ps_enable(px3212c_data,POWER_ON);
	if(ret < 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
			return ret;
	}
	msleep(150);
	
	ret = ps_calibration(px3212c_data,iControl);
	if(ret<0){
		if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration (set %s calibration value) Failed (%d)\n",__FUNCTION__,(iControl==1)?"low":"high",ret);
	}	
	
	ret = ps_enable(px3212c_data,POWER_OFF);
	if(ret < 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
			return ret;
	}
	
	PS_AlreadyCalibration=FALSE;
	PS_CalibrationRetryCount=CalibrationRetryTimes;
	ret = px3212c_calibration_proximity(px3212c_data);
	if(ret<0){
		if(PX3212C_STATUSMSG) printk( "[%s]	PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
	}

	return count;
}
static DEVICE_ATTR(ps_cali, (S_IWUSR|S_IRUGO), ps_do_Calibration_show, ps_do_Calibration_store);


static ssize_t ps_ConfigurationRegister_show(struct device *dev, struct device_attribute *attr, char *buf){

	int ps_configuration=0;

	ps_configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_CONFIG);
	if(ps_configuration< 0)  {
		return sprintf(buf,"Read PS_CONFIG Failed (%d) \n",ps_configuration);
	}

	return sprintf(buf,"PS Configuration Register =0x%x \n",ps_configuration);
}

static ssize_t ps_ConfigurationRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	u8 ps_configuration=0;
	int ret=0, temp=0;

	sscanf (buf, "%x", &temp);
	ps_configuration = (u8)temp;

	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_CONFIG, ps_configuration);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed (%d)\n",__FUNCTION__,ret);
	}

	return count;
}
static DEVICE_ATTR(ps_config, (S_IWUSR|S_IRUGO), ps_ConfigurationRegister_show, ps_ConfigurationRegister_store);

static ssize_t ps_LEDControlRegister_show(struct device *dev, struct device_attribute *attr, char *buf){

	int ps_led_control=0;

	ps_led_control = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_LED_CONTROL);
	if(ps_led_control< 0)  {
		return sprintf(buf,"Read PS_LED_CONTROL Failed (%d) \n",ps_led_control);
	}

	return sprintf(buf,"PS LED Control Register =0x%x \n",ps_led_control);
}

static ssize_t ps_LEDControlRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	u8 ps_led_control=0;
	int ret=0, temp=0;

	sscanf (buf, "%x", &temp);
	ps_led_control = (u8)temp;

	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_LED_CONTROL, ps_led_control);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_LED_CONTROL Failed (%d)\n",__FUNCTION__,ret);
	}

	return count;
}
static DEVICE_ATTR(ps_ledcontrol, (S_IWUSR|S_IRUGO), ps_LEDControlRegister_show, ps_LEDControlRegister_store);

static ssize_t ps_MeanTimeRegister_show(struct device *dev, struct device_attribute *attr, char *buf){

	int PS_MeanTime=0;

	PS_MeanTime = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_MEAN_TIME);
	if(PS_MeanTime< 0)  {
		return sprintf(buf,"Read PS_MEAN_TIME Failed (%d) \n",PS_MeanTime);
	}

	return sprintf(buf,"PS Mean Time Register =0x%x \n",PS_MeanTime);
}

static ssize_t ps_MeanTimeRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	u8 PS_MeanTime=0;
	int ret=0, temp=0;

	sscanf (buf, "%x", &temp);
	PS_MeanTime = (u8)temp;

	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_MEAN_TIME, PS_MeanTime);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,ret);
	}

	return count;
}
static DEVICE_ATTR(ps_mean, (S_IWUSR|S_IRUGO), ps_MeanTimeRegister_show, ps_MeanTimeRegister_store);
// add by leo for vendor testing --

//add by leo for proximity vendor calibration ++
static ssize_t ps_CalibrationRegister_show(struct device *dev, struct device_attribute *attr, char *buf){

	int PS_CrossTalk=0;
	unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;

	PS_VCALI_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_H);
	PS_VCALI_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_L);
	
	if(PS_VCALI_HByte<0||PS_VCALI_LByte<0)  {
		return sprintf(buf,"Read PS_VCALI Failed\n");
	}
	PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));
		
	return sprintf(buf,"PS Vendor Calibration Register =%d \n",PS_CrossTalk);
}
static DEVICE_ATTR(ps_crosstalk, (S_IWUSR|S_IRUGO),ps_CalibrationRegister_show,NULL);

static ssize_t ps_do_Vendor_Calibration_show(struct device *dev, struct device_attribute *attr, char *buf){

	int ret=0;
	
	ret = ps_get_vendor_calibration_value();
	if(ret<0){
		//if(PX3212C_STATUSMSG) printk( "[%s]	Read PS CalibrationValue Failed \n",__FUNCTION__);
		return sprintf(buf,"Read PS Vendor CalibrationValue Failed (%d)\n",ret);
	}

	return sprintf(buf,"PS Vendor Calibration Value =%d \n",PS_VendorCalibrationValue);
}

static ssize_t ps_do_Vendor_Calibration_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int ret=0,temp=0;
	struct px3212c_i2c_data *px3212c_data = data;

	sscanf (buf, "%x", &temp);
	if(temp==0){
		//Clean PS Crosstalk Register
		ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_H,0);
		if(ret< 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
			return ret;
		}
		ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_L,0);
		if(ret< 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
			return ret;
		}

	}else if(temp==1){
	
		PS_AlreadyVendorCalibration=TRUE;
		ret = ps_enable(px3212c_data,POWER_ON);
			if(ret < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
		}
		msleep(150);
		
		ret = ps_vendor_calibration();
		if(ret< 0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Failed (%d)\n",__FUNCTION__,ret);
		}

		ret = ps_enable(px3212c_data,POWER_OFF);
			if(ret < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
		}
		PS_AlreadyVendorCalibration=FALSE;
		PS_VendorCalibrationRetryCount=CalibrationRetryTimes;
	}
	
	return count;
}
static DEVICE_ATTR(ps_vcali, (S_IWUSR|S_IRUGO), ps_do_Vendor_Calibration_show,ps_do_Vendor_Calibration_store);
//add by leo for proximity vendor calibration --

//add by leo for modify MAX proximity crosstalk value ++
static ssize_t ps_MaxCrossTalkValue_show(struct device *dev, struct device_attribute *attr, char *buf){

	return sprintf(buf,"PS allow Maximum Crosstalk Value=%d \n",limit_PS_CrossTalkValue);
}

static ssize_t ps_MaxCrossTalkValue_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int temp=0;
	
	sscanf (buf, "%d", &temp);
	limit_PS_CrossTalkValue = temp;

	if(PX3212C_STATUSMSG) printk( "[%s]	Set PS maximum permit crosstalk value = %d \n",__FUNCTION__,limit_PS_CrossTalkValue);
	return count;
}
static DEVICE_ATTR(ps_crosstalk_limit, (S_IWUSR|S_IRUGO), ps_MaxCrossTalkValue_show, ps_MaxCrossTalkValue_store);
//add by leo for modify MAX proximity crosstalk value --


/* Dump ++ */
static ssize_t Set_dump(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){
	int option;
	unsigned int value_LByte=0, value_HByte=0;
	int PS_Configuration=0, PS_LEDControl=0, PS_MeanTime=0;
	unsigned long RAW_Data=0;
	struct px3212c_i2c_data *px3212c_data = data;

	int ret=0;
	sscanf (buf, "%x", &option);
	switch(option){
		case 0 : // set IsEnableSensor = 0
			IsEnableSensor=0;
			if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%s) \n", __FUNCTION__,(px3212c_data->ps_power_state==1)?"POWER_ON":"POWER_OFF");
		break;
		case 1 : // set IsEnableSensor = 1
			IsEnableSensor=1;
			if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%s) \n", __FUNCTION__,(px3212c_data->ps_power_state==1)?"POWER_ON":"POWER_OFF");
		break;
		case 3 : // set IsEnableSensor = 0 & Disable sensor
			IsEnableSensor=0;
			if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			ps_enable(px3212c_data,POWER_OFF);
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%s) \n", __FUNCTION__,(i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG)==0x02)?"POWER_ON":"POWER_OFF");
		break;
		case 4 : // set IsEnableSensor = 1 & Enable sensor
			IsEnableSensor=1;
			if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			ps_enable(px3212c_data,POWER_ON);
			if(PX3212C_STATUSMSG) printk( "[%s]	ps_enable(%s) \n", __FUNCTION__,(i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG)==0x02)?"POWER_ON":"POWER_OFF");
		break;
		case 5 : // read raw data
		    ret = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
			if(ret < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Read Status Failed \n",__FUNCTION__);
				return ret;
			}
			if(PX3212C_STATUSMSG) printk( "[%s]			PX3212C ps_enable = 0x%x \n", __FUNCTION__,ret);

			value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
			value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
			if(value_LByte<0||value_HByte<0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Read PS_DATA Failed \n",__FUNCTION__);
				return ret;
			}
			RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
			if(PX3212C_STATUSMSG) printk( "[%s]			PX3212C PS DATA = %d\n",__FUNCTION__,(int)RAW_Data);
			

		break;
		case 6 : // test fastly close open sensor
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, SYSTEM_CONFIG, 0x02); 
			if(PS_DEBUGMSG) printk( "[%s]	PX3212C  ps_enable(0x%x) \n", __FUNCTION__,i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG));

			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, SYSTEM_CONFIG, 0x00); 
			if(PS_DEBUGMSG) printk( "[%s]	PX3212C  ps_enable(0x%x) \n", __FUNCTION__,i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG));

		break;
		case 7 : // test input test
			input_report_abs(px3212c_data->ps_input_dev, ABS_DISTANCE, 7);
			input_sync(px3212c_data->ps_input_dev);
			input_report_abs(px3212c_data->ps_input_dev, ABS_DISTANCE, 8);
			input_sync(px3212c_data->ps_input_dev);
		break;
		case 8 :
			ps_initial();
			if(PS_DEBUGMSG) printk( "[%s]	PX3212C  ps_initial() \n", __FUNCTION__);
			PS_MeanTime = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_MEAN_TIME);
		    if(PS_MeanTime< 0)
		    {
		        if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,PS_MeanTime);
		        return PS_MeanTime;
		    }
		    if(PX3212C_STATUSMSG) printk( "[%s]			PS_MEAN_TIME = 0x%x\n",__FUNCTION__,PS_MeanTime);
			PS_LEDControl = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_LED_CONTROL);
			if(PS_LEDControl< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_LED_CONTROL Failed\n",__FUNCTION__);
				return PS_LEDControl;
			}
			if(PX3212C_STATUSMSG) printk( "[%s]			PS_LED_CONTROL = 0x%x\n",__FUNCTION__,PS_LEDControl);
			PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_CONFIG);
			if(PS_Configuration< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
				return PS_Configuration;
			}
			if(unlikely(debug))printk( "[%s]			ps_initial finished, PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);
		break;
		case 9 :
		    ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_MEAN_TIME, 0x03);// PS_MEAN_TIME(0x23) 00000011: mean time = 50ms
		    if(ret< 0)
		    {
		        if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,ret);
		        return ret;
		    }
		break;
		case 10 :
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, 0x05);//PS_LED_CONTROL(0x21) 0011 0011, LED pulse:11 (3 pulse), LED driver ratio:11 (100%)
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_LED_CONTROL Failed\n",__FUNCTION__);
				return ret;
			}
		break;
		case 11 :
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, 0x15);//PS_CONFIG(0x20) RS integrated time select:1111 (15T), PS gain:11, PS persist(interrupt filter):01
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed\n",__FUNCTION__);
				return ret;
			}
		break;
		
		case 12 :
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, 0xf5);//PS_CONFIG(0x20) RS integrated time select:1111 (15T), PS gain:11, PS persist(interrupt filter):01
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed\n",__FUNCTION__);
				return ret;
			}
		break;
		default:
			PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, 0x20);
			if(PS_Configuration< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
				return PS_Configuration;
			}
			if(PX3212C_STATUSMSG)printk( "[%s]			 PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);
			
			ret = i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, option);//PS_CONFIG(0x20) RS integrated time select:1111 (15T), PS gain:11, PS persist(interrupt filter):01
			if(PX3212C_STATUSMSG) printk( "[%s]	i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, 0x%x)\n",__FUNCTION__,option);
			if(ret< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed\n",__FUNCTION__);
				return ret;
			}

			PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, 0x20);
			if(PS_Configuration< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
				return PS_Configuration;
			}
			if(PX3212C_STATUSMSG)printk( "[%s]			PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);
			
			i2c_smbus_write_byte_data(px3212c_i2c_client, SYSTEM_CONFIG, 0x02);
			if(PX3212C_STATUSMSG) printk( "[%s]	Open senosr\n",__FUNCTION__);
		break;
	}
	return count;
}
static ssize_t Show_dump(struct device *dev,struct device_attribute *attr,char *buf)
{
	int ret = 0xff ,i =0 , PS_HighThresholdValue =0 ,PS_LowThresholdValue =0 , Raw_data =0;
	unsigned char ECValue[11]={0};
	unsigned char ECValue_print[12]={0};
	unsigned char PS_CValue_low[CValue_len] = {0}, PS_CValue_high[CValue_len] = {0};

	int PS_CrossTalk=0;
	unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;
	unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
	unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
	unsigned long PS_THDH=0, PS_THDL=0;
		
	if(IsLinkedPad == 1) ret = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
	if(PX3212C_STATUSMSG) printk( "[%s]	================Dump Start=============== \n",__FUNCTION__);
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C IsLinkedPad = %d \n",__FUNCTION__,IsLinkedPad);
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C ps_enable = 0x%x \n", __FUNCTION__,ret);
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C data->ps_power_state = %d \n", __FUNCTION__,data->ps_power_state);
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C CrossTalk Ready = %s \n", __FUNCTION__,(PS_AlreadyVendorCalibration==FALSE)?"False":"True");
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Cali. Threshold Ready = %s \n", __FUNCTION__,(PS_AlreadyVendorCalibration==FALSE)?"False":"True");
	
	ret = AX_MicroP_get_Proxm_crosstalk(ECValue);
	if(ret>=0){
		for(i = 0 ; i < 11 ; i ++) ECValue_print[i] = ECValue[i];
		ECValue_print[11] = '\0';
		if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Crosstalk from EC = %s \n", __FUNCTION__,ECValue_print);

		PS_VCALI_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_H);
		PS_VCALI_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_L);
		PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));
		if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Crosstalk from Register = %d \n", __FUNCTION__,PS_CrossTalk);		
	}
	else
	{
		if(PX3212C_STATUSMSG) printk( "[%s]	 AX_MicroP_get_Proxm_crosstalk return(%d) \n", __FUNCTION__,ret);		
	}
	ret = AX_MicroP_get_Proxm_threshold(ECValue);
	if(ret>=0){
		for(i = 0 ; i < 11 ; i ++) ECValue_print[i] = ECValue[i];
		ECValue_print[11] = '\0';
		if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Threshold from EC (string)= %s \n", __FUNCTION__,ECValue_print);
		for(i = 0;i < CalibrationValue_len;i++){
			if(i<CValue_len){
				PS_CValue_high[i]=ECValue[i];
			}
			if(i>CValue_len){
				PS_CValue_low[i-CValue_len-1]=ECValue[i];
			}
		}
		PS_HighThresholdValue = CharToInt(PS_CValue_high);
		PS_LowThresholdValue = CharToInt(PS_CValue_low);

		if(PX3212C_STATUSMSG)printk("[%s] PX3212C Threshold from EC (int) = (%d, %d) \n",__FUNCTION__,PS_HighThresholdValue,PS_LowThresholdValue);


		
		PS_THDH_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_H);
		PS_THDH_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_L);
		PS_THDL_HByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_H);
		PS_THDL_LByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_L);

		PS_THDH = ((PS_THDH_HByte<<2)|(PS_THDH_LByte & 0x03));
		PS_THDL = ((PS_THDL_HByte <<2)|(PS_THDL_LByte & 0x03));
		if(PX3212C_STATUSMSG)printk("[%s] PX3212C Threshold from Register = (%lu, %lu) \n",__FUNCTION__,PS_THDH,PS_THDL);
	}
	else
	{
		if(PX3212C_STATUSMSG) printk( "[%s]	 AX_MicroP_get_Proxm_crosstalk return(%d) \n", __FUNCTION__,ret);		
	}
	Raw_data = px3212c_get_raw_data();
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Raw data = %d \n", __FUNCTION__,Raw_data);
	if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Debug Flag = %d \n", __FUNCTION__,debug);
	if(PX3212C_STATUSMSG) printk( "[%s]	================Dump End=============== \n",__FUNCTION__);
	return sprintf(buf, "Dump!!\n");  
}
static DEVICE_ATTR(dump, (S_IWUSR|S_IRUGO), Show_dump, Set_dump);
/* Dump  --*/

// enable / disable irq ++ 
static ssize_t ps_irq_show(struct device *dev, struct device_attribute *attr, char *buf){

       return sprintf(buf," Please use echo 1 or 0 > ps_irq !\n");
}

static ssize_t ps_irq_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){
    int temp=0;
    sscanf (buf, "%x", &temp);
       
    if (temp == 1){
        AX_MicroP_enablePinInterrupt(INTR_EN_PROXM_INT,0);
               if(PX3212C_STATUSMSG) printk( "[%s]     Set irq Disable\n",__FUNCTION__);
       }else{
        AX_MicroP_enablePinInterrupt(INTR_EN_PROXM_INT,1);
               if(PX3212C_STATUSMSG) printk( "[%s]     Set irq Enable\n",__FUNCTION__);
       }
       return count;
}
static DEVICE_ATTR(ps_irq, (S_IWUSR|S_IRUGO), ps_irq_show, ps_irq_store);
// enable / disable irq --

// add by tom for set configuration ++
static ssize_t px3003b_get_configuration(struct device *dev, struct device_attribute *attr, char *buf)
{
	int PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_CONFIG);
	if(PS_Configuration< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
		return PS_Configuration;
	}
	if(PX3212C_STATUSMSG)printk( "[%s]			 PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);
	return sprintf(buf, "PS_Configuration= %x\n", PS_Configuration);
}

static ssize_t px3003b_set_configuration(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
    int option=0,PS_Configuration=0,ret=0;
    sscanf (buf, "%x", &option);
    
	PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, 0x20);
	if(PS_Configuration< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
		return PS_Configuration;
	}
	if(PX3212C_STATUSMSG)printk( "[%s]			 PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);
	
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, option);//PS_CONFIG(0x20) RS integrated time select:1111 (15T), PS gain:11, PS persist(interrupt filter):01
	if(PX3212C_STATUSMSG) printk( "[%s]	i2c_smbus_write_byte_data(px3212c_i2c_client, 0x20, 0x%x)\n",__FUNCTION__,option);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed\n",__FUNCTION__);
		return ret;
	}

	PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, 0x20);
	if(PS_Configuration< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
		return PS_Configuration;
	}
	if(PX3212C_STATUSMSG)printk( "[%s]			PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);

	return count;
}
static DEVICE_ATTR(ps_conf, (S_IWUSR|S_IRUGO) ,px3003b_get_configuration,px3003b_set_configuration);
// add by tom for set configuration --

// enable / disable irq ++ 
static ssize_t ps_power_show(struct device *dev, struct device_attribute *attr, char *buf){

       return sprintf(buf," Please use echo 1 or 0 > ps_irq !\n");
}

static ssize_t ps_power_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){
    int temp=0;
    sscanf (buf, "%x", &temp);
       
    if (temp == 1){
        AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,1) ;
               if(PX3212C_STATUSMSG) printk( "[%s]     Set Power ON\n",__FUNCTION__);
       }else{
        AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,0) ;
               if(PX3212C_STATUSMSG) printk( "[%s]     Set Power OFF\n",__FUNCTION__);
       }
       return count;
}
static DEVICE_ATTR(ps_power, (S_IWUSR|S_IRUGO), ps_power_show, ps_power_store);
// enable / disable irq --

//=========================================================================================

static struct attribute *px3212c_attributes[] = {
	&dev_attr_px3212c_id.attr,
	&dev_attr_px3212c_int.attr,
	&dev_attr_ir_data.attr,

	&dev_attr_ps_en.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_ps_status.attr,
	&dev_attr_ps_thd.attr,
	&dev_attr_ps_hthd.attr,
	&dev_attr_ps_lthd.attr,
	&dev_attr_ps_cali.attr,

	&dev_attr_pad_proximity_status.attr,	//for ATD testing
	&dev_attr_ps_irq.attr,                  //for enable/disalbe irq
	&dev_attr_ps_config.attr,		//for vendor testing
	&dev_attr_ps_ledcontrol.attr,		//for vendor testing
	&dev_attr_ps_mean.attr,		//for vendor testing
	&dev_attr_ps_vcali.attr,			//add by leo for proximity vendor calibration ++
	&dev_attr_ps_crosstalk.attr,		//add by leo for proximity vendor calibration ++
	&dev_attr_ps_crosstalk_limit.attr,	//add by leo for modify MAX proximity crosstalk value ++
	&dev_attr_dump.attr,				//add by tom for dump message
	&dev_attr_ps_conf.attr,				//add by tom for set configuration
	&dev_attr_ps_power.attr,				//add by tom for open/close sensor
	NULL
};
static const struct attribute_group px3212c_attr_group = {
	.attrs = px3212c_attributes,
};
//=========================================================================================
static int CharToInt(unsigned char CValue[]){

	int i=0,j=0,x=0,CalibrationValue=0;
	int temp[CValue_len]={0};
	
	if(unlikely(debug))printk( "[%s]	",__FUNCTION__);// add for test

	for(i=0;i<CValue_len;i++){
		if(unlikely(debug))printk( "%c",CValue[i]);// add for test
		temp[i]=(int)CValue[i]-48;
	}
	if(unlikely(debug))printk( "\n"); // add for test
	
	CalibrationValue = temp[CValue_len-1];
	for(i=1;i<CValue_len;i++){
		
		j=i;
		x=1;
		while(j>0){
			x*=10;
			j--;
		}
		CalibrationValue+=temp[(CValue_len-1)-i]*x;
	}
	return CalibrationValue;
}



//=========================================================================================
static int ps_open(struct inode *inode, struct file *file)
{
	if(unlikely(debug))printk( "[%s]	PS MISC Device OPEN \n",__FUNCTION__);
	return nonseekable_open(inode, file);		
}

static int ps_release(struct inode *inode, struct file *file)
{
	if(unlikely(debug))printk( "[%s]	PS MISC Device Release \n",__FUNCTION__);
	return 0;
}

static long ps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err=0, val=0, control=0 ,ret=0 ;
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long RAW_Data=0;
	struct px3212c_i2c_data *px3212c_data = data;
	if(unlikely(debug))printk( "[%s]		===========Enter PX3212C IOTCL ========== \n",__FUNCTION__);
	if(unlikely(debug))printk( "[%s]		PX3212C IOCTL (Cmd is %d) \n",__FUNCTION__, _IOC_NR(cmd));
	
	if (_IOC_TYPE(cmd) != PROXIMITYSENSOR_IOCTL_MAGIC) 
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
	
	if(unlikely(debug))printk( "[%s]		PX3212C Enter Before IsLinkedPad = %d\n",__FUNCTION__,IsLinkedPad);
	if(unlikely(debug))printk( "[%s]		PX3212C Enter Before IsEnableSensor = %d\n",__FUNCTION__,IsEnableSensor);
	
	if(IsLinkedPad == 0)
	{
		switch(cmd)
		{
			case PROXIMITYSENSOR_IOCTL_ENABLE:
				if (get_user(val, (unsigned long __user *)arg))
					return -EFAULT;
				if (val){
					IsEnableSensor= 1;
					if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_ENABLE (return IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
				}else{
					IsEnableSensor= 0;
					if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_ENABLE (return IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
				}
				return 0;
			break;	//LIGHTSENSOR_IOCTL_ENABLE
			case PROXIMITYSENSOR_IOCTL_GET_ENABLED:

			if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_GET_ENABLED (return %d) \n",__FUNCTION__,px3212c_data->ps_power_state);
			
			return put_user(px3212c_data->ps_power_state, (unsigned long __user *)arg);
			break;

			default:
				if(unlikely(debug))printk( "[%s]		PX3212C Other Cmd in IsLinkedPad == 0 (%d) \n",__FUNCTION__, _IOC_NR(cmd));
			
		}

		return 0;
		
	}else{
		//--------------------------------------------------
		switch (cmd) {
		case PROXIMITYSENSOR_IOCTL_ENABLE:
			 		
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val){
				IsEnableSensor= 1;
				if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_ENABLE - ps_enable  (IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
				return ps_enable(px3212c_data, POWER_ON);
			}else{
				IsEnableSensor= 0;
				if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_ENABLE - ps_disable (IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
				return ps_enable(px3212c_data, POWER_OFF);
			}
			break;
				
		case PROXIMITYSENSOR_IOCTL_GET_ENABLED:

			if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_GET_ENABLED (return %d) \n",__FUNCTION__,px3212c_data->ps_power_state);
			
			return put_user(px3212c_data->ps_power_state, (unsigned long __user *)arg);
			break;

		case PROXIMITYSENSOR_IOCTL_GET_STATUS:
			
			mdelay(300);
			value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
			value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
			RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
			if(unlikely(debug))printk( "[%s]		PX3212C DATA = %d\n",__FUNCTION__,(int)RAW_Data);
			
			px3212c_data->ps_status = (value_HByte & 0x80)>>7;
			if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_GET_STATUS (return %d)\n",__FUNCTION__,px3212c_data->ps_status);
			return put_user(px3212c_data->ps_status, (unsigned long __user *)arg);
			break;

		case PROXIMITYSENSOR_IOCTL_CALIBRATION:
			
			if (get_user(control, (unsigned long __user *)arg))
				return -EFAULT;

			// add by tom for check sensor is OPEN ++
			ret = px3212c_get_systemMode();
			if(ret == 0){ 
				err = ps_enable(px3212c_data,POWER_ON);
				if(err < 0)  {
						if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
						return err;
				}
				msleep(150);
			}
			// add by tom for check sensor is OPEN --
			
			if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_CALIBRATION (%d)\n",__FUNCTION__,control);
			err = ps_calibration(px3212c_data,control);
			if(err<0){
				if(PX3212C_STATUSMSG) printk( "[%s		PX3212C  Calibration (%d)(set calibration value) Failed (%d)\n",__FUNCTION__,control,err);
				return err;
			}	

			err = ps_enable(px3212c_data,POWER_OFF);
			if(err < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
				return err;
			}	
			
			PS_AlreadyCalibration=FALSE;
			PS_CalibrationRetryCount=CalibrationRetryTimes;
			err = px3212c_calibration_proximity(px3212c_data);
			if(err<0){
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C  Calibration (%d)(get calibration value) Failed (%d)\n",__FUNCTION__,control,err);
				return err;
			}

			//return put_user(PS_CalibrationValue, (unsigned long __user *)arg);
			break;

		case PROXIMITYSENSOR_IOCTL_VCALIBRATION:

			if(unlikely(debug))printk( "[%s]		PX3212C PROXIMITYSENSOR_IOCTL_VCALIBRATION \n",__FUNCTION__);

			PS_AlreadyVendorCalibration=TRUE;
			// add by tom for check sensor is OPEN ++
			ret = px3212c_get_systemMode();
			if(ret == 0){ 
				err = ps_enable(px3212c_data,POWER_ON);
				if(err < 0)  {
						if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
						return err;
				}
				msleep(150);
			}
			// add by tom for check sensor is OPEN --
			
			err = ps_vendor_calibration();
			if(err< 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C  Vendor Calibration Failed (%d)\n",__FUNCTION__,err);
				return err;
			}

			err = ps_enable(px3212c_data,POWER_OFF);
			if(err < 0)  {
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
				return err;
			}
			PS_AlreadyVendorCalibration=FALSE;
			PS_VendorCalibrationRetryCount=CalibrationRetryTimes;

			err = ps_get_vendor_calibration_value();
			if(err<0){
				if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Read PS Vendor CalibrationValue Failed (%d)\n",__FUNCTION__,err);
				return err;
			}

			printk( "[%s]		PX3212C Crosstalk Value = %d\n",__FUNCTION__,PS_VendorCalibrationValue);

			return put_user(PS_VendorCalibrationValue, (unsigned long __user *)arg);
			break;

		default:
			if(unlikely(debug))printk( "[%s]		PX3212C Incorrect Cmd  (%d) \n",__FUNCTION__, _IOC_NR(cmd));
			return -EINVAL;
		}
	}
	return 0;
}

static struct file_operations ps_fops = {
	.owner = THIS_MODULE,
	.open = ps_open,
	.release = ps_release,
	.unlocked_ioctl = ps_ioctl
};

static struct miscdevice ps_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "px3212c_ps_misc_dev",
	.fops = &ps_fops,
};

static int ps_setup(struct px3212c_i2c_data *px3212c_data)
{
	int ret=0;

	px3212c_data->ps_input_dev = input_allocate_device();
	if (!px3212c_data->ps_input_dev) {
		if(unlikely(debug))printk( "[%s]	Could Not Allocate PS Input Device\n",__FUNCTION__);
		return -ENOMEM;
	}
	px3212c_data->ps_input_dev->name = "px3212c_ps_input_dev";
	set_bit(EV_ABS, px3212c_data->ps_input_dev->evbit);
	set_bit(EV_SYN, px3212c_data->ps_input_dev->evbit);
	input_set_abs_params(px3212c_data->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	
	ret = input_register_device(px3212c_data->ps_input_dev);
	if (ret < 0) {
		if(unlikely(debug))printk( "[%s]	Could Not Register PS Input Device\n",__FUNCTION__);
		goto register_ps_input_device_err;
	}
	
	ret = misc_register(&ps_misc_dev);
	if (ret < 0) {
		if(unlikely(debug))printk( "[%s]	Could Not Register PS Misc Device\n",__FUNCTION__);
		goto register_ps_misc_device_err;
	}

	if(unlikely(debug))printk( "[%s]	PS_SETUP - FINISHED \n",__FUNCTION__);
	return 0;

register_ps_misc_device_err:
	input_unregister_device(px3212c_data->ps_input_dev);
register_ps_input_device_err:
	input_free_device(px3212c_data->ps_input_dev);
	return ret;
}

static int ps_enable(struct px3212c_i2c_data *px3212c_data,int  power)
{
	int SystemMode = 0, ret = 0;
	int retry = 3;
	// add by tom for open sensor power ++
	if(power == POWER_ON)
	{
		AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,1);
		if(PX3212C_STATUSMSG) printk( "[%s]         Proximity Power ON\n",__FUNCTION__);		
		msleep(30);
		while(retry > 0) 
		{
			ret = ps_initial();
			ret = px3212c_calibration_init();
			if(ret < 0)
			{
				if(PX3212C_STATUSMSG) printk( "[%s]         Power Fail (%d) Retry %d \n",__FUNCTION__,ret,retry);				
				AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,1);
				msleep(30);
			}
			else
			{
				break;
			}
			retry--;
		}
	}
	// add by tom for open sensor power --

	if((power==POWER_ON)&&(PS_AlreadyVendorCalibration==FALSE)&&(PS_VendorCalibrationRetryCount>0)){
		//if(unlikely(debug))printk( "[%s]	PS_CalibrationCrossTalk==FALSE\n",__FUNCTION__);
		ret = px3212c_vendor_calibration_proximity(px3212c_data);
		if(ret<0){
			if(PX3212C_STATUSMSG) printk( "[%s]			PS Vendor Calibration  (get cross talk value) Failed (%d)\n",__FUNCTION__,ret);
		}
		ret = px3212c_default_calibration_proximity();
		if(ret<0){
			if(PX3212C_STATUSMSG) printk( "[%s]			PS px3212c_default_calibration_proximity() Failed (%d)\n",__FUNCTION__,ret);
		}
/*
		else{
			if(PS_VendorCalibrationValue >= (PS_LOW_THD-20)){
				if(PX3212C_STATUSMSG) printk( "[%s]	Failed, PS Cross Talk (%d) > PS_LOW_THD-20 (%d)\n",__FUNCTION__,PS_VendorCalibrationValue,PS_LOW_THD-20);
				return -EDOM;
			}
			PS_HIGH_THD = Default_PS_ThresholdValue - PS_VendorCalibrationValue;
			PS_LOW_THD = Default_PS_ThresholdValue - PS_VendorCalibrationValue;

			ret = ps_set_threshold(PS_HIGH_THD, PS_LOW_THD);
			if(ret < 0)  {
				if(unlikely(debug))printk( "[%s]	ps_set_threshold (%d, %d) Failed\n",__FUNCTION__,PS_HIGH_THD, PS_LOW_THD);
				return ret;
			}
		}
*/
	}
/*	
	if((power==POWER_ON)&&((PS_AlreadyCalibration==FALSE)&&(PS_CalibrationRetryCount>0))){
		ret=px3212c_calibration_proximity(px3212c_data);
		if(ret<0){
			if(unlikely(debug))printk( "[%s]			PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
		}
	}
*/

	SystemMode = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
	if(unlikely(debug)) printk( "[px3212c_%s]		PX3212C Before Status : %s , SystemMode = 0x%x , To do %s\n",__FUNCTION__,(px3212c_data->ps_power_state==POWER_OFF)?"POWER_OFF":"POWER_ON",SystemMode,(power==POWER_OFF)?"POWER_OFF":"POWER_ON");
	if(SystemMode< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,SystemMode);
		return SystemMode;
	}

	power = power << 1;
	SystemMode = (SystemMode & 0xfd)|power; // 0xfd = 1111 1101
	if(unlikely(debug)) printk( "[px3212c_%s]		i2c_smbus_write_byte_data(0x%x, 0x%x, 0x%x) \n",__FUNCTION__,px3212c_i2c_client->addr,SYSTEM_CONFIG,SystemMode);
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, SYSTEM_CONFIG, SystemMode);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,SystemMode);
		return ret;
	}
	
	// Report reset 
	if(power==POWER_OFF){
		PS_CurrentStatus = -1;
		input_report_abs(px3212c_data->ps_input_dev, ABS_DISTANCE, RESET_INPUT_DEVICE);
		input_sync(px3212c_data->ps_input_dev);
		
		if(px3212c_data->ps_power_state!=(power>>1)){
			if(unlikely(debug))printk( "[%s]				disable_irq_wake  (px3212c_data->ps_power_state, power)=(%d,%d)\n",__FUNCTION__,px3212c_data->ps_power_state,power>>1);
			disable_irq_wake(px3212c_data->irq);
		}
	}else{
		if(px3212c_data->ps_power_state!=(power>>1)){
			if(unlikely(debug))printk( "[%s]				enable_irq_wake  (px3212c_data->ps_power_state, power)=(%d,%d)\n",__FUNCTION__,px3212c_data->ps_power_state,power>>1);
			enable_irq_wake(px3212c_data->irq);
		}
	}


	px3212c_data->ps_power_state= power>>1;
	SystemMode = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
	if(unlikely(debug)) printk( "[px3212c_%s]		PX3212C After Status : %s , SystemMode = 0x%x \n",__FUNCTION__,(px3212c_data->ps_power_state==POWER_OFF)?"POWER_OFF":"POWER_ON",SystemMode);
	if(PX3212C_STATUSMSG) printk( "[px3212c_%s]		PX3212C Now Status : %s, SystemMode = 0x%x \n",__FUNCTION__,(px3212c_data->ps_power_state==POWER_OFF)?"POWER_OFF":"POWER_ON",SystemMode);

	// add by tom for close sensor power ++
	if(power == POWER_OFF)
	{
		AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,0);
		PS_AlreadyVendorCalibration = false;
		PS_VendorCalibrationRetryCount = CalibrationRetryTimes;
		if(PX3212C_STATUSMSG) printk( "[%s]         Proximity Power OFF\n",__FUNCTION__);		
	}
	// add by tom for close sensor power --

	
	return 0;
}

// add by tom for get SystemMode ++ 
static int px3212c_get_systemMode(void)
{
	int SystemMode = 0 ;
	SystemMode = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
	if(PX3212C_STATUSMSG) printk( "[px3212c_%s]		SystemMode = 0x%x \n",__FUNCTION__,SystemMode);
	if(SystemMode< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,SystemMode);
		return -1;
	}
	if(SystemMode == 0x02)
	{
		return 1;
	}
	else
	{
		return 0;
	}
}
// add by tom for get SystemMode --


static int ps_initial(void)
{
	int ret=0, PS_Configuration=0, PS_LEDControl=0, PS_MeanTime=0;

    ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_MEAN_TIME, 0x03);// PS_MEAN_TIME(0x23) 00000011: mean time = 50ms
    if(ret< 0)
    {
        if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }

    PS_MeanTime = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_MEAN_TIME);
    if(PS_MeanTime< 0)
    {
        if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,PS_MeanTime);
        return PS_MeanTime;
    }
    


	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_LED_CONTROL, 0x33);//PS_LED_CONTROL(0x21) 0011 0011, LED pulse:11 (3 pulse), LED driver ratio:11 (100%)
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_LED_CONTROL Failed\n",__FUNCTION__);
		return ret;
	}

	PS_LEDControl = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_LED_CONTROL);
	if(PS_LEDControl< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_LED_CONTROL Failed\n",__FUNCTION__);
		return PS_LEDControl;
	}
	


	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_CONFIG, 0x08);//PS_CONFIG(0x20) RS integrated time select:1111 (15T), PS gain:11, PS persist(interrupt filter):00
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed\n",__FUNCTION__);
		return ret;
	}

	PS_Configuration = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_CONFIG);
	if(PS_Configuration< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
		return PS_Configuration;
	}
	if(unlikely(debug))printk( "[%s]			ps_initial : PS_CONFIG = 0x%x, PS_MEAN_TIME = 0x%x, PS_LED_CONTROL = 0x%x\n",__FUNCTION__,PS_Configuration,PS_MeanTime,PS_LEDControl);

	PS_AlreadyVendorCalibration = FALSE;
	PS_AlreadyCalibration = FALSE;
	PS_CalibrationRetryCount=CalibrationRetryTimes;

	return 0;
}

static int px3212c_calibration_init(void){
	int ret = 0;
	struct px3212c_i2c_data *px3212c_data = data;
	//ret = px3212c_calibration_proximity(px3212c_data);
	ret = px3212c_vendor_calibration_proximity(px3212c_data);
	ret = px3212c_default_calibration_proximity();
	return ret ;
}

static int ps_calibration(struct px3212c_i2c_data *px3212c_data, int iControl){

	int PS_HighThresholdValue=0, PS_LowThresholdValue=0;
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long RAW_Data=0;
	unsigned char PS_CValue[CalibrationValue_len]={0};
	int ret = 0 ;
	if(unlikely(debug)) printk("[%s]		===========PX3212C Calibration Threshold Start============= \n",__FUNCTION__);	
	if(iControl==0){
		PS_LowThresholdValue=Default_PS_ThresholdValue;
		PS_HighThresholdValue=Default_PS_ThresholdValue;
		
	}else{
		msleep(300);
		value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
		value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
		RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111

		switch (iControl){

			case 1: //calibration high threshold
				PS_LowThresholdValue = PS_LowCalibrationValue;
				PS_HighThresholdValue = (int)RAW_Data;
				break;

			case 2: //calibration low threshold
				PS_LowThresholdValue = (int)RAW_Data;
				PS_HighThresholdValue = PS_HighCalibrationValue;
				break;
			
			default: //calibration high and low threshold at the same time
				PS_LowThresholdValue = (int)RAW_Data;
				PS_HighThresholdValue = (int)RAW_Data;
		}
	}

	if(PX3212C_STATUSMSG) printk("[%s]			PS_CalibrationValue = (%d, %d) (Before Calibration)\n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);

	// set EC value
	sprintf(PS_CValue,"%05d&%05d",PS_HighThresholdValue, PS_LowThresholdValue);
	ret = AX_MicroP_set_Proxm_threshold(PS_CValue);
	if(ret < 0){
		if(PX3212C_STATUSMSG) printk("[%s]			AX_MicroP_set_Proxm_threshold() Failed (%d)\n",__FUNCTION__,ret);	
	}
	
	if(PX3212C_STATUSMSG) printk("[%s]			Save Calibration Value (%05d&%05d)\n",__FUNCTION__,PS_LowThresholdValue,PS_HighThresholdValue);	
	if(unlikely(debug)) printk("[%s]		===========PX3212C Calibration Threshold End============= \n",__FUNCTION__);	
	
	return 0; 
}

static int ps_get_calibration_value(struct px3212c_i2c_data *px3212c_data){
	
	int i = 0 , ret = 0;
	int PS_LowThresholdValue=0, PS_HighThresholdValue=0;
	unsigned char PS_CValue[CalibrationValue_len] = {0};
	unsigned char PS_CValue_low[CValue_len] = {0}, PS_CValue_high[CValue_len] = {0};

	if(unlikely(debug)) printk("[%s]		===========PX3212C Calibration Threshold Start============= \n",__FUNCTION__);	

	ret = AX_MicroP_get_Proxm_threshold(PS_CValue);
	if(ret < 0){
		if(PX3212C_STATUSMSG) printk("[%s]			AX_MicroP_get_Proxm_threshold() Failed (%d)\n",__FUNCTION__,ret);	
		return -EBADMSG;
	}
	
	
	if(PS_CValue[5]!='&')
	{
		if(PX3212C_STATUSMSG) printk("[%s]			AX_MicroP_get_Proxm_threshold() Failed , PS_CValue[5]!= '&' \n",__FUNCTION__);
		if(PX3212C_STATUSMSG) printk("[%s]			Get PS Calibration Threshold Failed , Use Default Value.  \n",__FUNCTION__);
		return -EBADMSG;
	}

	for(i=0;i<CalibrationValue_len;i++){
		if(i<CValue_len){
			PS_CValue_high[i]=PS_CValue[i];
		}
		if(i>CValue_len){
			PS_CValue_low[i-CValue_len-1]=PS_CValue[i];
		}
	}
	PS_HighThresholdValue = CharToInt(PS_CValue_high);
	PS_LowThresholdValue = CharToInt(PS_CValue_low);
	
	if((PS_HighThresholdValue>0)&&(PS_LowThresholdValue>0)){
		PS_HighCalibrationValue= PS_HighThresholdValue;
		PS_LowCalibrationValue= PS_LowThresholdValue;
		if(PX3212C_STATUSMSG)printk("[%s]	PS_CalibrationValue = (%d, %d) \n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);
		
	}else{	
		if(PX3212C_STATUSMSG) printk("[%s]	PS Calibration Failed \n",__FUNCTION__);
		return -EBADMSG;
	}
	if(unlikely(debug)) printk("[%s]		===========PX3212C Calibration Threshold End============= \n",__FUNCTION__);
	return 0;
}

// add by tom for proximity crosstalk ++
static int ps_vendor_calibration(void){

	int i=0,temp=0;
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long RAW_Data=0;
	int ret=0,PS_CrossTalk=0;
	unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;
	unsigned char PS_VCValue[CalibrationValue_len] = {0};

	if(unlikely(debug)) printk("[%s]		===========PX3212C Crosstalk Start============= \n",__FUNCTION__);	
	//Clean PS Crosstalk Register
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_H,0);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_L,0);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}
	if(unlikely(debug)) printk("[%s]		Clean PS Crosstalk Register Done. \n",__FUNCTION__);	
	//Read PS data 20 times to get average cross talk value
	for(i=0;i<21;i++){
		value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
		value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
		if(value_LByte<0||value_HByte<0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		Read PS_DATA Failed\n",__FUNCTION__);
			return -1;
		}
		RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
		if(i!=0)if(PX3212C_STATUSMSG) printk( "[%s]		PS DATA (%d) = %d\n",__FUNCTION__,i,(int)RAW_Data);

		if(i!=0)temp=temp+(int)RAW_Data;
		msleep(150);
	}
	PS_CrossTalk=temp/20;
	if(unlikely(debug)) printk("[%s]		======================== \n",__FUNCTION__);
	if(PX3212C_STATUSMSG) printk( "[%s]		PS Cross Talk = %d\n",__FUNCTION__,PS_CrossTalk);
	if(unlikely(debug)) printk("[%s]		======================== \n",__FUNCTION__);
	
	// Check the crosstalk value
	if(PS_CrossTalk>limit_PS_CrossTalkValue)return -EDOM;

	//Write cross talk value into PS Vendor Calibration Register
	PS_VCALI_HByte = PS_CrossTalk >> 1;
	PS_VCALI_LByte = PS_CrossTalk & 0x01;	
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_H,PS_VCALI_HByte);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		Read PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}
	ret = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_VCALI_L,PS_VCALI_LByte);
	if(ret< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		Read PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	PS_VCALI_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_H);
	PS_VCALI_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_VCALI_L);
	if(PS_VCALI_HByte<0||PS_VCALI_LByte<0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]		Read PS_VCALI Failed\n",__FUNCTION__);
		return -1;
	}
	PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));
	if(PX3212C_STATUSMSG) printk( "[%s]		PS Crosstalk value from Register = %d\n",__FUNCTION__,PS_CrossTalk);

	// set EC value
	sprintf(PS_VCValue,"%05d&%05d",PS_CrossTalk, PS_CrossTalk);
	
	ret = AX_MicroP_set_Proxm_crosstalk(PS_VCValue);
	if(ret < 0){
		if(PX3212C_STATUSMSG) printk("[%s]			AX_MicroP_set_Proxm_threshold() Failed (%d)\n",__FUNCTION__,ret);	
	}

	if(unlikely(debug)) printk("[%s]		===========PX3212C Crosstalk End============= \n",__FUNCTION__);
	return PS_CrossTalk;
}

static int ps_get_vendor_calibration_value(void){
	
	int i=0;
	int PS_CrossTalk=0;
	char PS_VCValue[CalibrationValue_len] = {0};
	char PS_VCValue_real[CValue_len] = {0};
	int ret = 0;
	if(unlikely(debug)) printk("[%s]		===========PX3212C Get Crosstalk Start============= \n",__FUNCTION__);
	
	ret = AX_MicroP_get_Proxm_crosstalk(PS_VCValue);
	if(ret < 0){
		if(PX3212C_STATUSMSG) printk("[%s]			AX_MicroP_get_Proxm_threshold() Failed (%d)\n",__FUNCTION__,ret);	
		return -EBADMSG;
	}
	
	if(PS_VCValue[5]!='&')
	{
		if(PX3212C_STATUSMSG) printk("[%s]			AX_MicroP_get_Proxm_threshold() Failed , PS_CValue[5] != '&'\n",__FUNCTION__);
		if(PX3212C_STATUSMSG) printk("[%s]			Get PS Crosstalk Failed \n",__FUNCTION__);
		return -EBADMSG;
	}
	
	for(i=0;i<CalibrationValue_len;i++){
		if(i>CValue_len){
			PS_VCValue_real[i-CValue_len-1]=PS_VCValue[i];
		}
	}
	
	PS_CrossTalk = CharToInt(PS_VCValue_real);

	if(PS_CrossTalk>0){		
		PS_VendorCalibrationValue= PS_CrossTalk;
		if(PX3212C_STATUSMSG)printk("[%s]		PS_CrossTalk from ini = %d \n",__FUNCTION__,PS_VendorCalibrationValue);
		
	}else{	
		if(PX3212C_STATUSMSG) printk("[%s]		PS Vendor Calibration Failed \n",__FUNCTION__);
		return -EBADMSG;
	}
	if(unlikely(debug)) printk("[%s]		===========PX3212C Get Crosstalk End============= \n",__FUNCTION__);
	return PS_CrossTalk ;
}
// add by tom for proximity crosstalk --


static int ps_set_threshold(int high_thd,int low_thd)
{
	int ret1=0,ret2=0,ret3=0,ret4=0;
	unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
	unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
	unsigned long PS_THDH=0, PS_THDL=0;
	
mutex_lock(&px3212c_mutex);
	PS_THDH_HByte = high_thd >> 2;
	PS_THDH_LByte = high_thd & 0x03;
	
	ret1 = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDH_H, PS_THDH_HByte);
	ret2 = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDH_L, PS_THDH_LByte);

	PS_THDL_HByte = low_thd >> 2;
	PS_THDL_LByte = low_thd & 0x03;

	ret3 = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDL_H, PS_THDL_HByte);
	ret4 = i2c_smbus_write_byte_data(px3212c_i2c_client, PS_THDL_L, PS_THDL_LByte);

mutex_unlock(&px3212c_mutex);	
	if(ret1< 0||ret2< 0||ret3< 0||ret4< 0)  {
		return -1;
	}
	
	PS_THDH_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_H);
	PS_THDH_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDH_L);
	
	PS_THDL_HByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_H);
	PS_THDL_LByte= i2c_smbus_read_byte_data(px3212c_i2c_client, PS_THDL_L);

	if(PS_THDH_HByte< 0||PS_THDH_LByte < 0||PS_THDL_HByte< 0||PS_THDL_LByte< 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	Read PS threshold Failed\n",__FUNCTION__);
		return -1;
	}
	
	PS_THDH = ((PS_THDH_HByte<<2)|(PS_THDH_LByte & 0x03));
	PS_THDL = ((PS_THDL_HByte <<2)|(PS_THDL_LByte & 0x03));
	if(PX3212C_STATUSMSG) printk( "[%s]		ps_set_threshold, PS_THDH=%d, PS_THDL=%d \n",__FUNCTION__,(int)PS_THDH, (int)PS_THDL);
	
	return 0;
}


static int ps_work_function(struct px3212c_i2c_data *px3212c_data)
{	
	if(PS_CurrentStatus != px3212c_data->ps_status){
		
		PS_CurrentStatus = px3212c_data->ps_status;	
		
		if(PS_CurrentStatus){
			input_report_abs(px3212c_data->ps_input_dev, ABS_DISTANCE, 0);
			input_sync(px3212c_data->ps_input_dev);
			if(PX3212C_STATUSMSG) printk( "[%s]		PS Report 'Close' \n",__FUNCTION__);
		}else{
			input_report_abs(px3212c_data->ps_input_dev, ABS_DISTANCE, 9);
			input_sync(px3212c_data->ps_input_dev);
			if(PX3212C_STATUSMSG) printk( "[%s]		PS Report 'Away' \n",__FUNCTION__);
		}

	}else{

		if(PS_DEBUGMSG)printk( "[%s]		PS_CurrentStatus Repeat !! \n",__FUNCTION__);
	}

	return 0;
}
static int px3212c_get_raw_data(void)
{
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long RAW_Data=0;
	int IR_OverflowFlag=0, PS_ObjectStatus=0;
	
	value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
	value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
	RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
	IR_OverflowFlag = (int)((value_HByte & 0x40)>>6);
	PS_ObjectStatus = (int)((value_HByte & 0x80)>>7);	
	
	if(unlikely(debug)) printk( "[%s]	OBJ = %d, IR_OF = %d, PS_DATA = %d\n",__FUNCTION__,PS_ObjectStatus,IR_OverflowFlag,(u32)RAW_Data);
	return (u32)RAW_Data;
}
static int px3212c_EC_event_notify(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct px3212c_i2c_data *px3212c_data = data;
	int ret = 0;
	if(unlikely(debug)) printk("[%s]		===========Enter PX3212C notify============= \n",__FUNCTION__);	
	switch (event){
			// pad in 
			case P01_ADD: 
				// add by tom for open power 
				AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,1);
				
				ret = px3212c_check_device(data); 
				if (ret<0){
					if(PX3212C_STATUSMSG) printk("[%s]		check_device Failed (%d)\n",__FUNCTION__,ret);
					goto px3212c_check_device_err;
				}

				// add by tom for ps initial ++ 
				ret = ps_initial();
				if(ret < 0)
					if(unlikely(debug)) printk( "[%s]		PX3212C ps_initial Failed \n",__FUNCTION__);
				// add by tom for ps initial --
				
				// add by tom for set Calibration value ++
				ret = px3212c_calibration_init();
				if(ret < 0)
				{
					if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Set Calibration Failed \n",__FUNCTION__);
				}
				else
				{
					if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Set Calibration Success , Threshold (H/L) =  %d / %d , Crosstalk = %d  \n",__FUNCTION__,PS_Default_High_Threshold,PS_Default_Low_Threshold,PS_VendorCalibrationValue);
				}
				// add by tom for set Calibration value --
				
				// Proximity sensor is Disable from HAL
				if(IsEnableSensor==0){	
					IsLinkedPad=1;
					// Close sensor
					px3212c_data->ps_power_state=0;
					ret = ps_enable(px3212c_data, POWER_OFF);	
					if(ret < 0)  {
							if(PX3212C_STATUSMSG) printk( "[%s]	PX3212C Shut down Failed \n",__FUNCTION__);
							goto px3212c_err;
					}
					//ret = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
					if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Sensor : %s  \n",__FUNCTION__,(data->ps_power_state==0x02)?"POWER_ON":"POWER_OFF");
					
					
					// Debug Message
					if(unlikely(debug)) printk( "[%s]		PX3212C IsEnableSensor : %d \n",__FUNCTION__,IsEnableSensor);
					if(unlikely(debug)) printk( "[%s]		PX3212C IsLinkedPad : %d \n",__FUNCTION__,IsLinkedPad);
					
				}else{ 					// Proximity sensor is Enable
					IsLinkedPad=1;
					/*
					// Check initial sensor status. If sensor status is open , close it. 
					int power  = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
					if(unlikely(debug)) printk( "[%s]		PX3212C Before Status : %s \n", __FUNCTION__,(power==0x00)?"POWER_OFF":"POWER_ON");
					if(power == 0x02){
						if(unlikely(debug)) printk( "[%s]		Initial sensor status is POWER_ON , close it.  \n", __FUNCTION__);
						ret = ps_enable(px3212c_data, POWER_OFF);	 // 0x00 = power down
						if(ret< 0)  {
							if(PX3212C_STATUSMSG) printk( "[%s]	Write SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,0x02);
							goto px3212c_err;
						}
						msleep(50);
					}
					*/
					
					// Open sensor because IsEnableSensor == 1
					ret = ps_enable(px3212c_data, POWER_ON);
					if(ret< 0)  {
						if(PX3212C_STATUSMSG) printk( "[%s]	Write SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,0x02);
						goto px3212c_err;
					}
					ret = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
					if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Sensor : %s  \n",__FUNCTION__,(ret==0x02)?"POWER_ON":"POWER_OFF");

					/*
					msleep(20);
					if(unlikely(debug)) printk( "[%s]		PX3212C Before Status : %s \n", __FUNCTION__,(i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG)==0x00)?"POWER_OFF":"POWER_ON");
						
					// Check read raw data is OK
					int rawData = 0;
					int retryCount = 0;
					
					do{
						rawData = px3212c_get_raw_data();
						 
						if(rawData <= 0)
						{
							ret = ps_enable(px3212c_data, POWER_OFF);
							if(ret< 0)  {
								if(PX3212C_STATUSMSG) printk( "[%s]		Write SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,ret);
								goto px3212c_err;
							}
							if(unlikely(debug)) printk( "[%s]		PX3212C Retry Status : %s \n", __FUNCTION__,(i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG)==0x00)?"POWER_OFF":"POWER_ON");
							msleep(20);
							ret = ps_enable(px3212c_data, POWER_ON);
							if(ret< 0)  {
								if(PX3212C_STATUSMSG) printk( "[%s]		Write SYSTEM_CONFIG Failed (%d)\n",__FUNCTION__,ret);
								goto px3212c_err;
							}
							if(unlikely(debug)) printk( "[%s]		PX3212C Retry Status : %s \n", __FUNCTION__,(i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG)==0x00)?"POWER_OFF":"POWER_ON");

						}
						
						if(retryCount>10){
							if(PX3212C_STATUSMSG) printk( "[%s]		PX3212C Retry Failed : Open sensor Failed or Read RawData Faided\n",__FUNCTION__);
							break;
						}
						retryCount++;
					}while(rawData <= 0);
					*/
				}

				// Debug Message
				if(unlikely(debug)) printk( "[%s]		PX3212C IsEnableSensor : %d \n",__FUNCTION__,IsEnableSensor);
				if(unlikely(debug)) printk( "[%s]		PX3212C IsLinkedPad : %d \n",__FUNCTION__,IsLinkedPad);
				if(unlikely(debug)) printk("[%s]		===========Exit P01_ADD============= \n",__FUNCTION__);	
			break;
			
			// Pad out
			case P01_REMOVE: 		
				PS_AlreadyVendorCalibration = FALSE;
				PS_AlreadyCalibration = FALSE;
				PS_CalibrationRetryCount=CalibrationRetryTimes;
				if(IsEnableSensor==0){
					IsLinkedPad=0;
				}else{ // Proximity sensor is Enable
					IsLinkedPad=0;
					data->ps_power_state=0;
				}
				// Debug Message
				if(unlikely(debug)) printk( "[%s]		PX3212C IsEnableSensor : %d \n",__FUNCTION__,IsEnableSensor);
				if(unlikely(debug)) printk( "[%s]		PX3212C IsLinkedPad : %d \n",__FUNCTION__,IsLinkedPad);
			break;

			// Proximity interrupt
			case P01_PROXM_SENSOR: 
					if(unlikely(debug)) printk( "[%s]		PX3212C Before Interrupt Status : %s \n", __FUNCTION__,(i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG)==0x00)?"POWER_OFF":"POWER_ON");
	    			if(unlikely(debug)) printk( "[%s]		PX3212C Interrupt (P01_PROXM_SENSOR) \n",__FUNCTION__);
					queue_delayed_work(px3212c_wq, &px3212c_work,0);
			break;
			

			
	}
	if(unlikely(debug)) printk("[%s]		===========Exit PX3212C notify============= \n",__FUNCTION__);	
	px3212c_check_device_err:
	px3212c_err:
	return NOTIFY_OK;
}


//px3212c_work_function: 1.find out which sensor occured the interrupt	2.read PS DATA 3.enable irq
static void px3212c_work_function(struct work_struct *work)
{
	int err = 0;
	int SystemMode=0;
	unsigned int value_LByte=0, value_HByte=0;
	unsigned long RAW_Data=0;
	struct px3212c_i2c_data *px3212c_data=data;

	// Debug Message
	if(unlikely(debug))printk( "[%s]		interruptStatus = %d \n",__FUNCTION__,interruptStatus);

	wake_lock(&px3212c_data->wake_lock);

	// add by tom for check sensor is Open ++
	SystemMode = i2c_smbus_read_byte_data(px3212c_i2c_client, SYSTEM_CONFIG);
	if(SystemMode==0x02)
	{
		if(PX3212C_STATUSMSG)printk( "[%s]			Sensor is Open (%d) \n",__FUNCTION__,SystemMode);
	}
	else
	{
		if(PX3212C_STATUSMSG)printk( "[%s]			Sensor is Close , Error interrupt !\n",__FUNCTION__);
		goto px3212c_work_function_finished;
	}
	// add by tom for check sensor is Open --	

	if(!interruptStatus){
		px3212c_data->interrupt_flag = i2c_smbus_read_byte_data(px3212c_i2c_client, INT_STATUS);
		if(unlikely(debug))printk( "[%s]		px3212c_data->interrupt_flag = 0x%x \n",__FUNCTION__,px3212c_data->interrupt_flag & 0x03);
		if(!(px3212c_data->interrupt_flag & 0x03)){
			if(unlikely(debug))printk( "[%s]		INT_STATUS = 0x%x goto px3212c_work_function_finished !!\n",__FUNCTION__,px3212c_data->interrupt_flag & 0x03);
			goto px3212c_work_function_finished;
		}
		interruptStatus = px3212c_data->interrupt_flag & 0x03;
	}
	/* Check whether imported Calibration value	*/
	if((px3212c_data->ps_power_state==POWER_ON)&&(PS_AlreadyVendorCalibration==FALSE)&&(PS_VendorCalibrationRetryCount>0)){
		if(PX3212C_STATUSMSG)printk( "[%s]			Set Crosstalk !\n",__FUNCTION__);
		err = px3212c_vendor_calibration_proximity(px3212c_data);
		if(err<0){
			if(PX3212C_STATUSMSG) printk( "[%s]			PS Vendor Calibration  (get crosstalk value) Failed (%d)\n",__FUNCTION__,err);
		}	
		err = px3212c_default_calibration_proximity();
		if(err<0){
			if(PX3212C_STATUSMSG) printk( "[%s]			PS px3212c_default_calibration_proximity() Failed (%d)\n",__FUNCTION__,err);
		}
	}
	
	/*
	if((px3212c_data->ps_power_state==POWER_ON)&&(PS_AlreadyCalibration==FALSE)&&(PS_CalibrationRetryCount>0)){
		if(PX3212C_STATUSMSG)printk( "[%s]			Set Threshold !\n",__FUNCTION__);
		err=px3212c_calibration_proximity(px3212c_data);
		if(err<0){
			if(PX3212C_STATUSMSG)printk( "[%s]			PS Calibration Threshold (get threshold value) Failed (%d)\n",__FUNCTION__,err);
		}
	}
	*/


	if(interruptStatus & 0x02){
		value_LByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_LOW);
		value_HByte = i2c_smbus_read_byte_data(px3212c_i2c_client, PS_DATA_HIGH);
		if(value_LByte<0||value_HByte<0)  {
			if(PX3212C_STATUSMSG) printk( "[%s]		Read PS_DATA Failed \n",__FUNCTION__);
		}
		RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
		if(PX3212C_STATUSMSG) printk( "[%s]			PS DATA = %d\n",__FUNCTION__,(int)RAW_Data);

		px3212c_data->ps_status = (value_HByte & 0x80)>>7;
		if(unlikely(debug))printk( "[%s]			PS STATUS = %d\n",__FUNCTION__,px3212c_data->ps_status);
		ps_work_function(px3212c_data);
	}
	
px3212c_work_function_finished:
	interruptStatus=0;
	wake_unlock(&px3212c_data->wake_lock);
	return;
}
//==========================================================================================
static int px3212c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{


	int ret=0;
	struct px3212c_platform_data *pdata;
	struct px3212c_i2c_data *px3212c_data=NULL;
	
	if(PX3212C_STATUSMSG) printk("=========Start Proximity PX3212C============= \n");
	if(PX3212C_STATUSMSG) printk( "[%s]		build version = %d\n",__FUNCTION__,build_version); //add by leo for read build version
    ret = register_microp_notifier(&pad_ps_notifier);
	if(ret < 0){
		if(PX3212C_STATUSMSG) printk( "[%s]		register_microp_notifier Failed (%d)\n",__FUNCTION__,ret);
	}
	if(!(px3212c_data = kzalloc(sizeof(struct px3212c_i2c_data), GFP_KERNEL))){
		if(build_version!=1)return -ENOMEM;
	}
	memset(px3212c_data, 0, sizeof(struct px3212c_i2c_data));

	px3212c_data->i2c_client = client;
	i2c_set_clientdata(client, px3212c_data);
	px3212c_i2c_client = px3212c_data->i2c_client;
	px3212c_i2c_client->flags = 1;
	strlcpy(px3212c_i2c_client->name, "px3212c", I2C_NAME_SIZE);

	data = px3212c_data;
	
	wake_lock_init(&px3212c_data->wake_lock, WAKE_LOCK_SUSPEND, "px3212c_wake_lock");
	
	/* Interrupt Setting */
	pdata = client->dev.platform_data;
	PX3212C_INTERRUPT_GPIO = pdata->gpio;

	/* Workqueue Setting */
	px3212c_wq = create_singlethread_workqueue("px3212c_wq");
	if (!px3212c_wq) {
		if(PX3212C_STATUSMSG) printk("[%s]	Create WorkQueue Failed\n",__FUNCTION__);
		ret = -ENOMEM;
		if(build_version!=1)goto create_singlethread_workqueue_err;
	}
	INIT_DELAYED_WORK(&px3212c_work, px3212c_work_function);

	/* Initialize Setting */
	//px3212c_create_proc_file(); // add by leo for proc file ++

	ps_setup(px3212c_data);

	/* sysfs Setting */
	ret = sysfs_create_group(&client->dev.kobj, &px3212c_attr_group);
	if (ret){
		if(PX3212C_STATUSMSG) printk("[%s] Register sysfs Failed\n",__FUNCTION__);
		if(build_version!=1)goto sysfs_create_group_err;
	}

	//Add by leo for early_suspend ++
	px3212c_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 5;
	px3212c_data->early_suspend.suspend = px3212c_early_suspend;
	px3212c_data->early_suspend.resume =  px3212c_later_resume;
	register_early_suspend(&px3212c_data->early_suspend);
	//Add by leo for early_suspend --


	if(PX3212C_STATUSMSG) printk("=========End Proximity PX3212C============= \n");
	return 0;

sysfs_create_group_err:
	//px3212c_remove_proc_file(); // add by leo for proc file ++
create_singlethread_workqueue_err:
	wake_lock_destroy(&px3212c_data->wake_lock);
	kfree(px3212c_data);
	return ret;
}

//Add by leo for early_suspend ++
static void px3212c_early_suspend(struct early_suspend *h)
{
// mask by leo because system may disable light sensor by ioctl from framework 	++
/*
	int ret=0;
	struct px3212c_i2c_data *px3212c_data = data;

	ALS_StatusBeforeSuspend = px3212c_data->als_power_state;
	if(unlikely(debug))printk( "[%s]	als_power_state = %d (Before Early Suspend)\n", __FUNCTION__,ALS_StatusBeforeSuspend);

	ret = als_enable(px3212c_data,POWER_OFF);
	if(ret < 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
	}
*/
// mask by leo because system may disable light sensor by ioctl from framework 	--
	if(PX3212C_STATUSMSG) printk( "[%s]\n", __FUNCTION__);
	return;	
}
static void px3212c_later_resume(struct early_suspend *h)
{
// mask by leo because system may disable light sensor by ioctl from framework 	++
/*
	int ret=0;
	struct px3212c_i2c_data *px3212c_data = data;
	
	if(unlikely(debug))printk( "[%s]	als_power_state = %d (Before Early Suspend)\n", __FUNCTION__,ALS_StatusBeforeSuspend); // add for test
	ret = als_enable(px3212c_data,ALS_StatusBeforeSuspend);
	if(ret < 0)  {
		if(PX3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,ALS_StatusBeforeSuspend);
	}
*/
// mask by leo because system may disable light sensor by ioctl from framework 	--
	if(PX3212C_STATUSMSG) printk( "[%s]\n", __FUNCTION__);
	return;	
}
//Add by leo for early_suspend --

static int px3212c_suspend(struct i2c_client *client , pm_message_t mesg)
{
	//struct px3212c_i2c_data *px3212c_data = data;
	
	if(PX3212C_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
	//enable_irq_wake(px3212c_data->irq); //mark by leo for proximity wake up source
	return 0;
}

static int px3212c_resume(struct i2c_client *client)
{
	//struct px3212c_i2c_data *px3212c_data = data;

	if(PX3212C_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
	//disable_irq_wake(px3212c_data->irq); //mark by leo for proximity wake up source
	return 0;
}

static int px3212c_i2c_remove(struct i2c_client *client)
{
	struct px3212c_i2c_data *px3212c_data =data;
	
	if(PX3212C_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
	//px3212c_remove_proc_file(); // add by leo for proc file ++
	sysfs_remove_group(&client->dev.kobj, &px3212c_attr_group);
	px3212c_data = i2c_get_clientdata(client);
	px3212c_i2c_client = NULL;
	unregister_early_suspend(&px3212c_data->early_suspend); //add by leo for early suspend ++ 
	kfree(px3212c_data);
	return 0;
}

static const struct i2c_device_id px3212c_i2c_idtable[] = {
	{"px3212c", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, px3212c_i2c_idtable);

static struct i2c_driver px3212c_i2c_driver = {
	
	.driver = {
		.name = PX3212C_I2C_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = 		px3212c_i2c_probe,
	.suspend	=	px3212c_suspend,
	.resume	=	px3212c_resume,
	.remove	=	px3212c_i2c_remove,
	.id_table = 	px3212c_i2c_idtable,
	
};

static int __init px3212c_init(void)
{
	int ret;
	if(Read_PROJ_ID() == PROJ_ID_PF400CG){
		if((Read_HW_ID() == HW_ID_SR1))
			{
 				printk("[%s] PF400CG-P72G P-sensor function don't work now!!\n",__FUNCTION__);
				 return 0;
			}
	}
 	ret = i2c_add_driver(&px3212c_i2c_driver);
	if ( ret != 0 ) {
		if(PX3212C_STATUSMSG) printk("[%s]	i2c_add_driver Failed (ret = %d) \n",__FUNCTION__,ret);
		return ret;
	}else{
		if(unlikely(debug))printk("[%s]	px3212c_init success !!\n",__FUNCTION__);
		return ret;
	}
}

static void __exit px3212c_exit(void)
{
	i2c_del_driver(&px3212c_i2c_driver);
}

module_init(px3212c_init);
module_exit(px3212c_exit);

MODULE_AUTHOR("Asus Tek. <asus@asus.com>");
MODULE_DESCRIPTION("CAPELLA px3212c Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

