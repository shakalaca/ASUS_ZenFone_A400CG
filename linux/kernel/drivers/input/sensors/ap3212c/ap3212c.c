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

#include <asm/intel_scu_ipc.h>//add by leo for power enable ++
#include <linux/earlysuspend.h>//add by leo for early_suspend ++
#include <linux/proc_fs.h>//add by leo for proc file ++
#include <linux/HWVersion.h>

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
#define AP3212C_I2C_DEVICE_NAME "ap3212c"
#define DRIVER_VERSION		"1.0.0.0"
static int  AP3212C_INTERRUPT_GPIO=0;	//pro_int#=63

/* Register Address Info */
#define SYSTEM_CONFIG 	0x00
#define INT_STATUS		0x01
#define INT_CLEANR_MNR 	0x02
#define MAKE_ID 			0x03
#define PRODUCT_ID 		0x04
#define REVISION_ID 		0x05

#define IR_DATA_LOW	0x0A
#define IR_DATA_HIGH	0x0B
#define ALS_DATA_LOW	0x0C
#define ALS_DATA_HIGH	0x0D
#define PS_DATA_LOW	0x0E
#define PS_DATA_HIGH	0x0F
#define PS_OBJ_STATUS   	0x0F

#define ALS_CONFIG 		0x10
#define ALS_THDL_L		0x1A	//ALS Low Threshold Lower Byte
#define ALS_THDL_H		0x1B	//ALS Low Threshold Higher Byte
#define ALS_THDH_L		0x1C	//ALS High Threshold Lower  Byte
#define ALS_THDH_H		0x1D	//ALS High Threshold HigherByte

#define PS_CONFIG		0x20
#define PS_LED_CONTROL	0x21
#define PS_MEAN_TIME	0x23
#define PS_VCALI_L		0x28	//PS Calibration Register Lower Byte
#define PS_VCALI_H		0x29	//PS Calibration Register Higher Byte
#define PS_THDL_L		0x2A	//PS Low Threshold Lower Byte
#define PS_THDL_H		0x2B	//PS Low Threshold Higher Byte
#define PS_THDH_L		0x2C	//PS High Threshold Lower  Byte
#define PS_THDH_H		0x2D	//PS High Threshold HigherByte

struct ap3212c_platform_data
{
    int gpio;
};

struct ap3212c_i2c_data
{
    struct i2c_client *i2c_client;
    struct input_dev *als_input_dev;
    struct input_dev *ps_input_dev;
    int id;
    int als_original_luxvalue;
    int als_luxvalue;
    int als_original_adcvalue;
    int als_adcvalue;
    int ps_status;
    unsigned int  interrupt_flag;
    struct early_suspend early_suspend;
    int als_power_state;
    int ps_power_state;
    int irq;
    //int als_calibration_value;
    //int ps_calibration_value;	//ie, ps_threshold_value
    struct wake_lock wake_lock;
};

static struct ap3212c_i2c_data *data = NULL;
static struct i2c_client *ap3212c_i2c_client = NULL;


/* Sensors Interrupt Workqueue */
static struct workqueue_struct *ap3212c_wq;
struct delayed_work ap3212c_work;
static void ap3212c_work_function(struct work_struct *work);
static int interruptStatus=0;

static DEFINE_MUTEX(ap3212c_mutex);

/* Sensors Calibration Info */
#define CalibrationValue_len 21
#define CValue_len 10				 //CValue_len = (CalibrationValue_len-1)/2
#define ALS_CONFIG_FILE_PATH	"/data/sensors/als_calibration.ini"
#define PS_CONFIG_FILE_PATH	"/data/sensors/ps_calibration.ini"
#define PS_CROSSTALK_FILE_PATH "/data/sensors/ps_crosstalk.ini"	// add by leo for proximity vendor calibration ++
#define Default_ALS_CalibrationValue 1
//#define Default_PS_ThresholdValue 90 	 //mark by leo for proximity vendor calibration ++
#define Default_PS_ThresholdValue 300 //add by leo for proximity vendor calibration ++
#define CalibrationRetryTimes 5;

static int ALS_CalibrationValue_standard=Default_ALS_CalibrationValue;
static int ALS_CalibrationValue_real =Default_ALS_CalibrationValue;
static int PS_CalibrationValue =Default_PS_ThresholdValue;
// add by leo for high/low threshold calibration ++
static int PS_HighCalibrationValue=175;
static int PS_LowCalibrationValue=50;
// add by leo for high/low threshold calibration --
static int PS_VendorCalibrationValue =0; // add by leo for proximity vendor calibration ++

static int ALS_CalibrationRetryCount=CalibrationRetryTimes;
static int PS_CalibrationRetryCount=CalibrationRetryTimes;
static int PS_VendorCalibrationRetryCount=CalibrationRetryTimes;

typedef enum
{
    FALSE= 0,
    TRUE
} boolean;
static boolean ALS_AlreadyCalibration = FALSE;
static boolean PS_AlreadyCalibration = FALSE;
static boolean PS_AlreadyVendorCalibration = FALSE;	//add by leo for proximity vendor calibration ++


/* Sensors Threshold Info */
//#define	ALS_LEVEL	18
//static int Default_als_threshold_lux[ALS_LEVEL+1]={0,5,30,60,100,150,200,250,300,350,450,550,700,900,1100,1300,1500,2000,65535};
#define	ALS_LEVEL	11
static int Default_als_threshold_lux[ALS_LEVEL+1]= {0,15,30,50,100,300,550,900,1100,1500,2200,65535};
static int Default_als_threshold_adc[ALS_LEVEL+1]= {0};
static int als_threshold_lux[ALS_LEVEL+1]= {0};
static int als_threshold_adc[ALS_LEVEL+1]= {0};

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
static int limit_PS_CrossTalkValue = 200; //add by leo for proximity vendor calibration ++

/* Sensors IO Control Info */
#define LIGHTSENSOR_IOCTL_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE				_IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)
#define LIGHTSENSOR_IOCTL_GET_RAWDATA		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 3, int *)
#define LIGHTSENSOR_IOCTL_GET_LUXVALUE		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 4, int *)
#define LIGHTSENSOR_IOCTL_CALIBRATION			_IOW(LIGHTSENSOR_IOCTL_MAGIC, 5, int *)

#define PROXIMITYSENSOR_IOCTL_MAGIC 'p'
#define PROXIMITYSENSOR_IOCTL_GET_ENABLED	_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 1, int *)
#define PROXIMITYSENSOR_IOCTL_ENABLE			_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 2, int *)
#define PROXIMITYSENSOR_IOCTL_GET_STATUS		_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 3, int *)
#define PROXIMITYSENSOR_IOCTL_CALIBRATION	_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 4, int *) // modify by leo for high/low threshold calibration ++
#define PROXIMITYSENSOR_IOCTL_VCALIBRATION	_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 5, int *) // add by leo for proximity vendor calibration ++

static int als_enable(struct ap3212c_i2c_data *ap3212c_data,int  power);
static int ps_enable(struct ap3212c_i2c_data *ap3212c_data,int  power);
static int als_set_threshold(int ilevel);
static int ps_set_threshold(int high_thd,int low_thd);
static int als_get_calibration_value(struct ap3212c_i2c_data *ap3212c_data);
static int ps_get_calibration_value(struct ap3212c_i2c_data *ap3212c_data);
static int ps_get_vendor_calibration_value(void);	// add by leo for proximity vendor calibration ++
static int als_calibration(struct ap3212c_i2c_data *ap3212c_data, int iControl);
static int ps_calibration(struct ap3212c_i2c_data *ap3212c_data, int iControl);
static int ps_vendor_calibration(void);	// add by leo for proximity vendor calibration ++
static irqreturn_t ap3212c_interrupt_handler(int irq, void *dev_id);
static void ap3212c_early_suspend(struct early_suspend *h);
static void ap3212c_later_resume(struct early_suspend *h);
//static int clear_interrupt_flag(void);
static int ap3212c_calibration_lightsensor(struct ap3212c_i2c_data *ap3212c_data);// add by leo for proc file ++
static int ap3212c_calibration_proximity(struct ap3212c_i2c_data *ap3212c_data);// add by leo for proc file ++

//extern int proximity_sensor_status(int status); // add by leo for proximity notify touch
extern int Read_PROJ_ID(void);
extern int Read_HW_ID();

// add by leo for proc file ++
#define	AP3212C_PROC_FILE	"ap3212c"
static struct proc_dir_entry *ap3212c_proc_file;
// add by leo for proc file --

#define AP3212C_DEBUGMSG 	0
#define ALS_DEBUGMSG		0
#define PS_DEBUGMSG		0
#define AP3212C_STATUSMSG	1
static unsigned int debug = 0;

//=========================================================================================

// add by leo for proc file ++
static ssize_t ap3212c_register_read(char *page, char **start, off_t off, int count, int *eof, void *idata)
{
    ssize_t sprintf_count = 0;
    if(AP3212C_STATUSMSG) printk( "[%s]	debug = %d \n",__FUNCTION__,debug);
    debug = (debug==1)?0:1; //switch debug message
    sprintf_count += sprintf(page + sprintf_count, "debug meaasge (%s)\n", ((debug==1)?"on":"off"));
    return sprintf_count;
}
static ssize_t ap3212c_register_write(struct file *filp, const char __user *buff, unsigned long len, void *idata)
{
    char messages[80] = {0};
    int ret = 0, i = 0, iloop = 0, count = 0, cmd = 0, en = 0, iControl = 0, ithd = 0;
    u32 reg_address=0, reg_value=0;
    static u8 temp[4]= {0};
    static u8 total_input[80]= {0};
    struct ap3212c_i2c_data *ap3212c_data = data;

    if (len >= 80)
    {
        if(AP3212C_STATUSMSG) printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
        return -EFAULT;
    }

    if (copy_from_user(messages, buff, len))
        return -EFAULT;

    if(AP3212C_STATUSMSG) printk("[%s]	input command: %s\n",__FUNCTION__,messages);

    if ((messages[0]=='l')&&(messages[1]=='o')&&(messages[2]=='g'))
    {
        if(&messages[4]==NULL)
            if(AP3212C_STATUSMSG) printk("[%s]	show debug message\n",__FUNCTION__);
        if(&messages[4]!=NULL)
            sscanf (&messages[4], "%d", &en);

        debug=(en==1)?1:0;
        if(AP3212C_STATUSMSG) printk( "[%s]	debug meaasge (%s)\n",__FUNCTION__,(debug==1)?"on":"off");
        return len;

    }
    else if((messages[0]=='a')&&(messages[1]=='l')&&(messages[2]=='s')&&(messages[3]=='_')&&(messages[4]=='e')&&(messages[5]=='n'))
    {
        //als_en+en
        if(AP3212C_STATUSMSG) printk( "[%s]	messages[7] =%c\n", __FUNCTION__,messages[7]);//add by leo for testtest
        if(AP3212C_STATUSMSG) printk( "[%s]	messages[7] =%d\n", __FUNCTION__,messages[7]);//add by leo for testtest
        if(&messages[7]!=NULL)
        {
            sscanf (&messages[7], "%d", &en);
        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d)\n", __FUNCTION__,ap3212c_data->als_power_state);
            return len;
        }

        if(AP3212C_STATUSMSG) printk( "[%s]	en = %d \n",__FUNCTION__,en);

        if(en==1)
        {
            ret = als_enable(ap3212c_data,POWER_ON);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
            }
        }
        else if(en==0)
        {
            ret = als_enable(ap3212c_data,POWER_OFF);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
            }
        }
        return len;

    }
    else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='e')&&(messages[4]=='n'))
    {
        //ps_en+en
        if(&messages[6]!=NULL)
        {
            sscanf (&messages[6], "%d", &en);
        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d)\n", __FUNCTION__,ap3212c_data->ps_power_state);
            return len;
        }

        if(en==1)
        {
            ret = ps_enable(ap3212c_data,POWER_ON);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
            }
        }
        else if(en==0)
        {
            ret = ps_enable(ap3212c_data,POWER_OFF);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
            }
        }
        return len;

    }
    else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[4]=='t')&&(messages[5]=='h')&&(messages[6]=='d'))
    {
        //ps_lthd+value or ps_hthd+value
        if(&messages[8]!=NULL)
        {
            sscanf (&messages[8], "%d", &ithd);
        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		PS_THDH=%d, PS_THDL=%d \n",__FUNCTION__,PS_HIGH_THD, PS_LOW_THD);
            return len;
        }

        if((messages[3]=='h')&&(ithd>0))
        {
            PS_HIGH_THD =ithd;
        }
        else if((messages[3]=='l')&&(ithd>0))
        {
            PS_LOW_THD=ithd;
        }
        ret = ps_set_threshold(PS_HIGH_THD, PS_LOW_THD);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_LOW_THD,PS_HIGH_THD);
        }
        return len;

    }
    else if((messages[0]=='a')&&(messages[1]=='l')&&(messages[2]=='s')&&(messages[3]=='_')&&(messages[4]=='c')&&(messages[5]=='a')&&(messages[6]=='l')&&(messages[7]=='i'))
    {
        //als_cali+value
        if(&messages[9]!=NULL)
        {
            sscanf (&messages[9], "%d", &iControl);
        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		ALS Calibration Value =%010d&%010d\n",__FUNCTION__,ALS_CalibrationValue_standard,ALS_CalibrationValue_real);
            return len;
        }

        ret = als_calibration(ap3212c_data, iControl);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ALS Calibration (set calibration value) Failed (%d)\n",__FUNCTION__,ret);
        }

        ALS_AlreadyCalibration=FALSE;
        ret = ap3212c_calibration_lightsensor(ap3212c_data);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ALS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
        }
        return len;

    }
    else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='c')&&(messages[4]=='a')&&(messages[5]=='l')&&(messages[6]=='i'))
    {
        //ps_cali+value
        if(&messages[8]!=NULL)
        {
            sscanf (&messages[8], "%d", &iControl);
        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration Value = %d \n",__FUNCTION__,PS_CalibrationValue);
            return len;
        }

        ret = ps_calibration(ap3212c_data,iControl);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration (set calibration value) Failed (%d)\n",__FUNCTION__,ret);
        }

        PS_AlreadyCalibration=FALSE;
        PS_CalibrationRetryCount=CalibrationRetryTimes;
        ret = ap3212c_calibration_proximity(ap3212c_data);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
        }
        return len;

    }
    else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='v')&&(messages[4]=='c')&&(messages[5]=='a')&&(messages[6]=='l')&&(messages[7]=='i'))
    {
        //ps_vcali+value
        if(&messages[9]!=NULL)
        {
            sscanf (&messages[9], "%d", &iControl);
        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Value = %d \n",__FUNCTION__,PS_VendorCalibrationValue);
            return len;
        }

        if(iControl==0)
        {
            //Clean PS Vendor Calibration Register
            ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_H,0);
            if(ret< 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
                return ret;
            }
            ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_L,0);
            if(ret< 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
                return ret;
            }

        }
        else if(iControl==1)
        {
            PS_AlreadyVendorCalibration=TRUE;
            ret = ps_enable(ap3212c_data,POWER_ON);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
            }
            msleep(150);

            ret = ps_vendor_calibration();
            if(ret< 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Failed (%d)\n",__FUNCTION__,ret);
            }

            ret = ps_enable(ap3212c_data,POWER_OFF);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
            }
            PS_AlreadyVendorCalibration=FALSE;
            PS_VendorCalibrationRetryCount=CalibrationRetryTimes;
        }
        return len;

    }
    else if(messages[0]=='w')
    {
        cmd=1;
    }
    else if(messages[0]=='r')
    {
        cmd=2;

    }
    else
    {
        if(AP3212C_STATUSMSG) printk("[%s]	Unknow Command\n",__FUNCTION__);
        return len;
    }

    // count input command
    for(i=0; i<100; i++)
    {
        if(messages[i]=='x')
        {
            count ++;
        }
        else if(messages[i]=='\n')
        {
            if(unlikely(debug))printk("[%s]	command number = %d\n",__FUNCTION__,count);
            break;
        }
    }

    // transfor input command from ASCII code to HEX
    for(iloop = 0; iloop < count; iloop++)
    {
        temp[0] = messages[iloop*5 + 2];
        temp[1] = messages[iloop*5 + 3];
        temp[2] = messages[iloop*5 + 4];
        temp[3] = messages[iloop*5 + 5];
        sscanf(temp, "%x", &total_input[iloop]);
    }
    u8* input_cmd = (char*)kcalloc(count,sizeof(char),GFP_KERNEL);

    if(AP3212C_STATUSMSG) printk("[%s]	input_cmd = %s ",__FUNCTION__,cmd==1?"write":"read");
    for(i=0; i<count; i++)
    {
        input_cmd[i]=total_input[i]&0xff;
        if(AP3212C_STATUSMSG) printk("0x%02x ", input_cmd[i]);
    }
    if(AP3212C_STATUSMSG) printk("\n");

    // send test command
    reg_address = input_cmd[0];
    reg_value = input_cmd[1];

    if(cmd==1)  //write
    {
        ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, reg_address, reg_value);
        if(ret< 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	Write 0x%02X into 0x%02X failed (%d)\n",__FUNCTION__,reg_value,reg_address,ret);
            return ret;
        }
        if(AP3212C_STATUSMSG) printk( "[%s]	write 0x%02X into 0x%02X\n",__FUNCTION__,reg_value,reg_address);

    }
    else if(cmd==2)   //read
    {
        ret = i2c_smbus_read_byte_data(ap3212c_i2c_client, reg_address);
        if(ret< 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	Read data from 0x%02X failed (%d)\n",__FUNCTION__,reg_address,ret);
            return ret;
        }
        if(AP3212C_STATUSMSG) printk( "[%s]	0x%02x value is 0x%02x \n",__FUNCTION__,reg_address,ret);
        reg_value=ret;
    }

    return len;
}

void ap3212c_create_proc_file(void)
{
    ap3212c_proc_file = create_proc_entry(AP3212C_PROC_FILE, 0666, NULL);
    if(ap3212c_proc_file)
    {
        ap3212c_proc_file->read_proc = ap3212c_register_read;
        ap3212c_proc_file->write_proc = ap3212c_register_write;
    }
    else
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	create_proc_entry failed\n",__FUNCTION__);
    }
}
void ap3212c_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    if(unlikely(debug))printk( "[%s] remove proc files \n",__FUNCTION__);
    remove_proc_entry(AP3212C_PROC_FILE, &proc_root);
}
// add by leo for proc file --

/*
static int ap3212c_setup_power(struct ap3212c_i2c_data *ap3212c_data)
{
	int err=0;
	err = intel_scu_ipc_iowrite8(MSIC_VPROG2CNT, VPROG2_ON);
	if(err<0){
		if(AP3212C_STATUSMSG) printk( "[%s]	intel_scu_ipc_iowrite8 Failed\n",__FUNCTION__);
	}

	if(AP3212C_STATUSMSG) printk( "[%s]	ap3212c_setup_power - FINISHED\n",__FUNCTION__);
	return 0;
}
*/
static int ap3212c_check_device(struct ap3212c_i2c_data *ap3212c_data)
{
    int pid=0, rid=0,mid=0;

    pid = i2c_smbus_read_byte_data(ap3212c_i2c_client, PRODUCT_ID);
    if(pid< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PRODUCT_ID Failed\n",__FUNCTION__);
        return pid;
    }
    rid = i2c_smbus_read_byte_data(ap3212c_i2c_client, REVISION_ID);
    if(rid < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read REVERSION_ID Failed\n",__FUNCTION__);
        return rid;
    }
    mid = i2c_smbus_read_byte_data(ap3212c_i2c_client, MAKE_ID);
    if(mid < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read MAKE_ID Failed\n",__FUNCTION__);
        return mid;
    }

    if(mid == 0x02 && pid == 0x02 && rid == 0x01)
    {
        ap3212c_data->id = pid;
        if(AP3212C_STATUSMSG) printk( "[%s]		Check Device OK !! Product ID = 0x%x \n",__FUNCTION__,pid);
        return 0;
    }
    else
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Check Device Failed\n",__FUNCTION__);
        if(AP3212C_STATUSMSG) printk( "[%s]		Product ID = 0x%x, Reversion ID = 0x%x, Make ID = 0x%x\n",__FUNCTION__,pid,rid,mid);
    }
    return -ENODEV;
}

static int ap3212c_setup_irq(struct ap3212c_i2c_data *ap3212c_data)
{
    int err=0;

    //AP3212C_INTERRUPT_GPIO = ap3212c_i2c_client->irq;	//gpio number is stored in i2c_client.irq

    err = gpio_request(AP3212C_INTERRUPT_GPIO, "ap3212c");
    if(err<0)
    {
        if(AP3212C_STATUSMSG) printk("[%s]	gpio_request Failed\n",__FUNCTION__);
        goto exit;
    }

    err = gpio_direction_input(AP3212C_INTERRUPT_GPIO) ;
    if(err<0)
    {
        if(AP3212C_STATUSMSG) printk("[%s]	gpio_direction_input Failed\n",__FUNCTION__);
        goto gpio_direction_input_err;
    }

    ap3212c_data->irq = gpio_to_irq(AP3212C_INTERRUPT_GPIO);
    if(ap3212c_data->irq<0)
    {
        if(AP3212C_STATUSMSG) printk("[%s]	gpio_to_irq Failed\n",__FUNCTION__);
        goto gpio_to_irq_err;
    }

    err = request_threaded_irq(ap3212c_data->irq, NULL, ap3212c_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_PERCPU | IRQF_FORCE_RESUME ,"ap3212c_interrupt",NULL);
    if (err)
    {
        dev_err(&ap3212c_i2c_client->adapter->dev,"Can NOT Register IRQ : %d , err:%d\n", ap3212c_data->irq, err);
        err = -EIO;
        goto request_irq_err;
    }

    //enable_irq_wake(ap3212c_data->irq); // add by leo for proximity wake up source ++
    disable_irq_nosync(ap3212c_data->irq);

    if(AP3212C_STATUSMSG) printk("[%s] 		AP3212C interrupt gpio = %d , value = %d\n",__FUNCTION__, AP3212C_INTERRUPT_GPIO, ap3212c_data->irq);
    return 0;

request_irq_err:
gpio_to_irq_err:
gpio_direction_input_err:
    gpio_free(AP3212C_INTERRUPT_GPIO);
exit:
    return err;
}
//=========================================================================================

static int ap3212c_calibration_lightsensor(struct ap3212c_i2c_data *ap3212c_data)
{

    int ilevel=0, ret=0, iRetry=CalibrationRetryTimes;

    if(ALS_AlreadyCalibration==FALSE)
    {

        if(unlikely(debug))printk( "[%s]	ALS Calibration Retry %d times.\n",__FUNCTION__,(iRetry-ALS_CalibrationRetryCount));
        ALS_CalibrationRetryCount --;

        ret = als_get_calibration_value(ap3212c_data);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]ALS Use Default Calibration Value \n",__FUNCTION__);
            return ret;

        }
        else
        {
            if(unlikely(debug))printk( "[%s]	ALS_CalibrationValue = %d/%d\n",__FUNCTION__,ALS_CalibrationValue_standard,ALS_CalibrationValue_real);

            for(ilevel=0; ilevel<ALS_LEVEL; ilevel++)
            {
                als_threshold_lux[ilevel]=Default_als_threshold_lux[ilevel]*ALS_CalibrationValue_real/ALS_CalibrationValue_standard; //ie. /ALS_CalibrationValue

                if(als_threshold_lux[ilevel]>4096) 	//0xFFFF (THD) *625/10000 =4096 (LUX),
                {
                    als_threshold_adc[ilevel]=0xFFFF;

                }
                else
                {
                    als_threshold_adc[ilevel] = als_threshold_lux[ilevel]*10000/625;
                }
            }

            for(ilevel=0; ilevel<ALS_LEVEL+1; ilevel++)
            {
                if(unlikely(debug))printk( "[%s]	als_threshold_lux[%d] = %d (0x%x) \n",__FUNCTION__,ilevel,als_threshold_adc[ilevel]*625/10000,als_threshold_adc[ilevel]*625/10000);
            }
            ALS_AlreadyCalibration=TRUE;
        }
    }
    return 1;
}

static int ap3212c_calibration_proximity(struct ap3212c_i2c_data *ap3212c_data)
{

    int ret=0, iRetry=CalibrationRetryTimes;

    if(PS_AlreadyCalibration==FALSE)
    {

        if(unlikely(debug))printk( "[%s]		PS Calibration Retry %d times.\n",__FUNCTION__,(iRetry-PS_CalibrationRetryCount));
        PS_CalibrationRetryCount --;

        ret = ps_get_calibration_value(ap3212c_data);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Use Default Calibration Value \n",__FUNCTION__);
            return ret;

        }
        else
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration Value = (%d, %d) \n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);

            PS_HIGH_THD = PS_HighCalibrationValue;
            PS_LOW_THD  = PS_LowCalibrationValue;

            ret = ps_set_threshold(PS_HIGH_THD,PS_LOW_THD);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_HIGH_THD,PS_LOW_THD);
                return ret;
            }
            PS_AlreadyCalibration=TRUE;
        }
    }
    return 1;
}

//add by leo for proximity cross talk calibration ++
static int ap3212c_vendor_calibration_proximity(struct ap3212c_i2c_data *ap3212c_data)
{

    int ret=0,PS_CrossTalk=0, iRetry=CalibrationRetryTimes;;
    unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;

    if(unlikely(debug))printk( "[%s]	PS Vendor Calibration Retry %d times.\n",__FUNCTION__,(iRetry-PS_VendorCalibrationRetryCount));
    PS_VendorCalibrationRetryCount --;

    // get corss talk value from /data/sensors/ps_corsstalk.ini
    PS_CrossTalk=ps_get_vendor_calibration_value();
    if(PS_CrossTalk<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]PS  Use Default Vendor Calibration Value \n",__FUNCTION__);
        return PS_CrossTalk;
    }

    //write the value into ps vendor calibration register
    PS_VCALI_HByte = PS_CrossTalk >> 1;
    PS_VCALI_LByte = PS_CrossTalk & 0x01;
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_H,PS_VCALI_HByte);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_L,PS_VCALI_LByte);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }

    //read the vendor calibration value from the register for double check
    PS_VCALI_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_VCALI_H);
    PS_VCALI_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_VCALI_L);
    if(PS_VCALI_HByte<0||PS_VCALI_LByte<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]Read PS_VCALI Failed\n",__FUNCTION__);
        return -1;
    }
    PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));
    if(AP3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Register = %d\n",__FUNCTION__,PS_CrossTalk);

    PS_AlreadyVendorCalibration=TRUE;

    return PS_CrossTalk;
}
//add by leo for proximity cross talk calibration --

//=========================================================================================
static ssize_t LightSensor_check_For_ATD_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    ret = ap3212c_check_device(ap3212c_data);
    if(ret < 0)
    {
        return sprintf(buf,"0\n");
    }
    return sprintf(buf,"1\n");
}
static DEVICE_ATTR(lightsensor_status, S_IRUGO,LightSensor_check_For_ATD_test,NULL);

static ssize_t Get_ADC_Value_For_ATD_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long ADC_Data=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    als_enable(ap3212c_data, POWER_ON);
    mdelay(300);

    value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_LOW);
    value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_HIGH);
    ADC_Data = ((value_HByte << 8) | value_LByte);
    //if(AP3212C_STATUSMSG) printk( "[%s]	ALS_DATA = %d (ADC)\n",__FUNCTION__,(int)ADC_Data);
    //mdelay(300);

    als_enable(ap3212c_data, POWER_OFF);

    return sprintf(buf,"%d\n",(int)ADC_Data);
}
static DEVICE_ATTR(lightsensor_adc, S_IRUGO,Get_ADC_Value_For_ATD_test,NULL);

static ssize_t ProximitySensor_check_For_ATD_test(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int value_HByte=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    ps_enable(ap3212c_data, POWER_ON);
    mdelay(300);

    value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
    ap3212c_data->ps_status = (value_HByte & 0x80)>>7;
    //if(AP3212C_STATUSMSG) printk( "[%s]	PS_ObjectStatus = %d \n",__FUNCTION__,ap3212c_data->ps_status);
    //mdelay(300);

    ps_enable(ap3212c_data, POWER_OFF);

    return sprintf(buf,"%d\n",ap3212c_data->ps_status);
}
static DEVICE_ATTR(proximity_status, S_IRUGO,ProximitySensor_check_For_ATD_test,NULL);

static ssize_t ap3212c_get_ID(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    ret = ap3212c_check_device(ap3212c_data);
    if(ret < 0)
    {
        //if(AP3212C_STATUSMSG) printk( "[%s]	ap3212c_check_device Failed \n",__FUNCTION__);
        return sprintf(buf,"ap3212c_check_device Failed\n");
    }
    if(AP3212C_STATUSMSG) printk( "[%s]	AP3212C ID = 0x%x \n",__FUNCTION__,ap3212c_data->id);

    return sprintf(buf,"AP3212C ID = 0x%x \n",ap3212c_data->id);
}
static DEVICE_ATTR(ap3212c_id, S_IRUGO,ap3212c_get_ID,NULL);


static ssize_t als_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long ADC_Data=0;

    value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_LOW);
    value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_HIGH);
    ADC_Data = ((value_HByte << 8) | value_LByte);

    if(AP3212C_STATUSMSG) printk( "[%s]	ALS_DATA = %d (ADC)\n",__FUNCTION__,(int)ADC_Data);
    if(AP3212C_STATUSMSG) printk( "[%s]	ALS_DATA = %d (Lux)\n",__FUNCTION__,(int)ADC_Data*625/10000); //Resolution = 0.0625 lux/count
    if(AP3212C_STATUSMSG) printk( "[%s]	ALS_DATA = %d (Lux After Calibration)\n",__FUNCTION__,(int)ADC_Data*625/10000*ALS_CalibrationValue_standard/ALS_CalibrationValue_real);

    return sprintf(buf,"ALS original data = %d (lux) \n",(int)ADC_Data*625/10000);
}
static DEVICE_ATTR(als_data, S_IRUGO,als_get_data,NULL);


static ssize_t ps_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long RAW_Data=0;
    int IR_OverflowFlag=0, PS_ObjectStatus=0;

    value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_LOW);
    value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
    RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
    IR_OverflowFlag = (int)((value_HByte & 0x40)>>6);
    PS_ObjectStatus = (int)((value_HByte & 0x80)>>7);

    if(AP3212C_STATUSMSG) printk( "[%s]	OBJ = %d, IR_OF = %d, PS_DATA = %d\n",__FUNCTION__,PS_ObjectStatus,IR_OverflowFlag,(u32)RAW_Data);
    return sprintf(buf,"PS output data = %d\n",(int)RAW_Data);
}
static DEVICE_ATTR(ps_data, S_IRUGO,ps_get_data,NULL);

static ssize_t ps_get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int value_HByte=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
    ap3212c_data->ps_status = (value_HByte & 0x80)>>7;

    if(AP3212C_STATUSMSG) printk( "[%s]	PS_OBJ = %d\n",__FUNCTION__,ap3212c_data->ps_status);

    return sprintf(buf,"PS status = %d\n",ap3212c_data->ps_status);
}
static DEVICE_ATTR(ps_status, S_IRUGO,ps_get_status,NULL);

static ssize_t IR_get_data(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long IR_Data=0;

    value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_LOW);
    value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
    IR_Data = ((value_HByte << 2) | (value_LByte & 0x03)); //0x03 = 000000 11

    if(AP3212C_STATUSMSG) printk( "[%s]	IR_DATA = %d\n",__FUNCTION__,(int)IR_Data);

    return sprintf(buf,"IR_DATA = %d\n",(int)IR_Data);
}
static DEVICE_ATTR(ir_data, S_IRUGO,IR_get_data,NULL);

static ssize_t ap3212c_interrupt_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
    int InterruptFlags=0;

    InterruptFlags = i2c_smbus_read_byte_data(ap3212c_i2c_client, INT_STATUS);
    InterruptFlags = InterruptFlags & 0x03;

    return sprintf(buf,"INT_STATUS = 0x%x\n",InterruptFlags);
}
static DEVICE_ATTR(ap3212c_int, S_IRUGO,ap3212c_interrupt_flag,NULL);

static ssize_t als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int SystemMode=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    SystemMode = i2c_smbus_read_byte_data(ap3212c_i2c_client, SYSTEM_CONFIG);
    if(SystemMode< 0)
    {
        //if(AP3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed\n",__FUNCTION__);
        return sprintf(buf,"Read SYSTEM_CONFIG Failed\n");
    }

    if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) ; SystemMode = 0x%x \n", __FUNCTION__,ap3212c_data->als_power_state, SystemMode);
    return sprintf(buf,"als_enable(%d) \n",ap3212c_data->als_power_state);
}

static ssize_t als_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    int ret=0, state=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    sscanf (buf, "%d", &state);

    if(buf[0]=='1')
    {
        ret = als_enable(ap3212c_data,POWER_ON);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
        }
    }
    else if(buf[0]=='0')
    {
        ret = als_enable(ap3212c_data,POWER_OFF);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
        }
    }
    return count;
}
static DEVICE_ATTR(als_en, S_IWUSR | S_IRUGO , als_enable_show , als_enable_store);


static ssize_t ps_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int SystemMode=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    SystemMode = i2c_smbus_read_byte_data(ap3212c_i2c_client, SYSTEM_CONFIG);
    if(SystemMode< 0)
    {
        //if(AP3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed\n",__FUNCTION__);
        return sprintf(buf,"Read SYSTEM_CONFIG Failed\n");
    }

    if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) ; SystemMode = 0x%x \n", __FUNCTION__,ap3212c_data->ps_power_state, SystemMode);
    return sprintf(buf,"ps_enable(%d) \n",ap3212c_data->ps_power_state);
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    int ret=0, state=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    sscanf (buf, "%d", &state);

    if(buf[0]=='1')
    {
        ret = ps_enable(ap3212c_data,POWER_ON);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
        }
    }
    else if(buf[0]=='0')
    {
        ret = ps_enable(ap3212c_data,POWER_OFF);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
        }
    }
    return count;
}
static DEVICE_ATTR(ps_en, S_IWUSR | S_IRUGO , ps_enable_show , ps_enable_store);


static ssize_t als_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int ALS_THDH_LByte=0, ALS_THDH_HByte=0;
    unsigned int ALS_THDL_LByte=0, ALS_THDL_HByte=0;
    unsigned long ALS_THDH=0, ALS_THDL=0;

    ALS_THDH_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDH_H);
    ALS_THDH_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDH_L);

    ALS_THDL_HByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDL_H);
    ALS_THDL_LByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDL_L);

    if(ALS_THDH_HByte< 0||ALS_THDH_LByte < 0||ALS_THDL_HByte< 0||ALS_THDL_LByte< 0)
    {
        return sprintf(buf,"Read ALS Threshold Failed\n");
    }
    ALS_THDH = ((ALS_THDH_HByte<<8)|ALS_THDH_LByte);
    ALS_THDL= ((ALS_THDL_HByte<<8)|ALS_THDL_LByte);

    return sprintf(buf,"ALS Threshold = Low(%d), High(%d)\n",(int)ALS_THDL*625/10000,(int)ALS_THDH*625/10000);
}

static ssize_t als_threshold_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    int ret=0, ilevel=0;

    sscanf (buf, "%d", &ilevel);
    if(unlikely(debug))printk( "[%s]		als_threshold_store ( level %d )\n",__FUNCTION__,(int)ilevel);

    ret = als_set_threshold(ilevel);
    if(ret < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		als_set_threshold ( level %d ) Failed \n",__FUNCTION__,(int)ilevel);
    }

    return count;
}
static DEVICE_ATTR(als_thd, S_IWUSR | S_IRUGO , als_threshold_show , als_threshold_store);

static ssize_t ps_threshold_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
    unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
    unsigned long PS_THDH=0, PS_THDL=0;

    PS_THDH_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDH_H);
    PS_THDH_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDH_L);
    PS_THDL_HByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDL_H);
    PS_THDL_LByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDL_L);

    if(PS_THDH_HByte< 0||PS_THDH_LByte < 0||PS_THDL_HByte< 0||PS_THDL_LByte< 0)
    {
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

    if(ithreshold>0)
    {
        PS_HIGH_THD =ithreshold;
        PS_LOW_THD=ithreshold;
    }
    ret = ps_set_threshold(PS_HIGH_THD, PS_LOW_THD);
    if(ret < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		ps_set_threshold (%d, %d) Failed \n",__FUNCTION__,PS_LOW_THD,PS_HIGH_THD);
    }

    return count;
}
static DEVICE_ATTR(ps_thd, S_IWUSR | S_IRUGO , ps_threshold_show , ps_threshold_store);

//add by leo for proximity threshold problem ++
static ssize_t ps_high_threshold_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    int ret=0, high_thd=0;
    unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
    unsigned long PS_THDH=0;

    sscanf (buf, "%d", &high_thd);

    mutex_lock(&ap3212c_mutex);
    PS_THDH_HByte = high_thd >> 2;
    PS_THDH_LByte = high_thd & 0x03;

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDH_H, PS_THDH_HByte);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_THDH_HByte Failed (%d)\n",__FUNCTION__,ret);
    }
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDH_L, PS_THDH_LByte);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_THDH_LByte Failed (%d)\n",__FUNCTION__,ret);
    }
    mutex_unlock(&ap3212c_mutex);

    PS_THDH_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDH_H);
    PS_THDH_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDH_L);

    if(PS_THDH_HByte< 0||PS_THDH_LByte < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PS threshold Failed\n",__FUNCTION__);
    }

    PS_THDH = ((PS_THDH_HByte<<2)|(PS_THDH_LByte & 0x03));
    if(AP3212C_STATUSMSG) printk( "[%s]		ps_set_high_threshold, PS_THDH=%d\n",__FUNCTION__,(int)PS_THDH);

    return count;
}
static DEVICE_ATTR(ps_hthd, S_IWUSR | S_IRUGO , NULL , ps_high_threshold_store);

static ssize_t ps_low_threshold_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
    int ret=0,low_thd=0;
    unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
    unsigned long PS_THDL=0;

    sscanf (buf, "%d", &low_thd);

    mutex_lock(&ap3212c_mutex);
    PS_THDL_HByte = low_thd >> 2;
    PS_THDL_LByte = low_thd & 0x03;

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDL_H, PS_THDL_HByte);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_THDL_HByte Failed (%d)\n",__FUNCTION__,ret);
    }
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDL_L, PS_THDL_LByte);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_THDL_LByte Failed (%d)\n",__FUNCTION__,ret);
    }
    mutex_unlock(&ap3212c_mutex);

    PS_THDL_LByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDL_L);
    if(PS_THDL_HByte< 0||PS_THDL_LByte< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PS_THDL_LByte Failed\n",__FUNCTION__);
    }

    PS_THDL = ((PS_THDL_HByte <<2)|(PS_THDL_LByte & 0x03));
    if(AP3212C_STATUSMSG) printk( "[%s]		ps_set_low_threshold, PS_THDL=%d \n",__FUNCTION__,(int)PS_THDL);

    return count;
}
static DEVICE_ATTR(ps_lthd, S_IWUSR | S_IRUGO , NULL , ps_low_threshold_store);
//add by leo for proximity threshold problem --

static ssize_t als_do_Calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int ret=0;
    struct ap3212c_i2c_data *ap3212c_data=data;

    ret = als_get_calibration_value(ap3212c_data);
    if(ret<0)
    {
        //if(AP3212C_STATUSMSG) printk( "[%s]	Read ALS CalibrationValue Failed \n",__FUNCTION__);
        return sprintf(buf,"Read ALS CalibrationValue Failed \n");
    }

    return sprintf(buf,"ALS Calibration Value =%010d&%010d\n",ALS_CalibrationValue_standard,ALS_CalibrationValue_real); //fix format for read_write_ACD and do not change it
}

static ssize_t als_do_Calibration_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    int ret=0, Correct_LuxValue;
    struct ap3212c_i2c_data *ap3212c_data=data;

    sscanf (buf, "%d", &Correct_LuxValue);

    ret = als_calibration(ap3212c_data, Correct_LuxValue);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	ALS Calibration (set calibration value) Failed (%d)\n",__FUNCTION__,ret);
    }

    ALS_AlreadyCalibration=FALSE;
    ret = ap3212c_calibration_lightsensor(ap3212c_data);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	ALS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
    }

    return count;
}
static DEVICE_ATTR(als_cali, S_IWUSR | S_IRUGO , als_do_Calibration_show, als_do_Calibration_store);

static ssize_t ps_do_Calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int ret=0;
    struct ap3212c_i2c_data *ap3212c_data=data;

    ret = ps_get_calibration_value(ap3212c_data);
    if(ret<0)
    {
        //if(AP3212C_STATUSMSG) printk( "[%s]	Read PS CalibrationValue Failed \n",__FUNCTION__);
        return sprintf(buf,"Read PS CalibrationValue Failed (%d)\n",ret);
    }

    return sprintf(buf,"PS Calibration Value =%010d&%010d\n",PS_HighCalibrationValue,PS_LowCalibrationValue);
}

static ssize_t ps_do_Calibration_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    int iControl;
    int ret=0;
    struct ap3212c_i2c_data *ap3212c_data=data;

    sscanf (buf, "%d", &iControl); // 0: reset, 1: low threshold, 2: high threshold

    ret = ps_calibration(ap3212c_data,iControl);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration (set %s calibration value) Failed (%d)\n",__FUNCTION__,(iControl==1)?"low":"high",ret);
    }

    PS_AlreadyCalibration=FALSE;
    PS_CalibrationRetryCount=CalibrationRetryTimes;
    ret = ap3212c_calibration_proximity(ap3212c_data);
    if(ret<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
    }

    return count;
}
static DEVICE_ATTR(ps_cali, S_IWUSR | S_IRUGO , ps_do_Calibration_show, ps_do_Calibration_store);

// add by leo for vendor testing ++
static ssize_t als_ConfigurationRegister_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int als_configuration=0;

    als_configuration = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_CONFIG);
    if(als_configuration< 0)
    {
        return sprintf(buf,"Read ALS Configuration Register Failed (%d)\n",als_configuration);
    }

    return sprintf(buf,"ALS Configuration Register =0x%x \n",als_configuration);
}

static ssize_t als_ConfigurationRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    u8 als_configuration=0;
    int ret=0, temp=0;

    sscanf (buf, "%x", &temp);
    als_configuration = (u8)temp;

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, ALS_CONFIG, als_configuration);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write ALS_CONFIG Failed (%d)\n",__FUNCTION__,ret);
    }

    return count;
}
static DEVICE_ATTR(als_config, S_IWUSR | S_IRUGO , als_ConfigurationRegister_show, als_ConfigurationRegister_store);

static ssize_t ps_ConfigurationRegister_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int ps_configuration=0;

    ps_configuration = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_CONFIG);
    if(ps_configuration< 0)
    {
        return sprintf(buf,"Read PS_CONFIG Failed (%d) \n",ps_configuration);
    }

    return sprintf(buf,"PS Configuration Register =0x%x \n",ps_configuration);
}

static ssize_t ps_ConfigurationRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    u8 ps_configuration=0;
    int ret=0, temp=0;

    sscanf (buf, "%x", &temp);
    ps_configuration = (u8)temp;

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_CONFIG, ps_configuration);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed (%d)\n",__FUNCTION__,ret);
    }

    return count;
}
static DEVICE_ATTR(ps_config, S_IWUSR | S_IRUGO , ps_ConfigurationRegister_show, ps_ConfigurationRegister_store);

static ssize_t ps_LEDControlRegister_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int ps_led_control=0;

    ps_led_control = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_LED_CONTROL);
    if(ps_led_control< 0)
    {
        return sprintf(buf,"Read PS_LED_CONTROL Failed (%d) \n",ps_led_control);
    }

    return sprintf(buf,"PS LED Control Register =0x%x \n",ps_led_control);
}

static ssize_t ps_LEDControlRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    u8 ps_led_control=0;
    int ret=0, temp=0;

    sscanf (buf, "%x", &temp);
    ps_led_control = (u8)temp;

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_LED_CONTROL, ps_led_control);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_LED_CONTROL Failed (%d)\n",__FUNCTION__,ret);
    }

    return count;
}
static DEVICE_ATTR(ps_ledcontrol, S_IWUSR | S_IRUGO , ps_LEDControlRegister_show, ps_LEDControlRegister_store);

static ssize_t ps_MeanTimeRegister_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int PS_MeanTime=0;

    PS_MeanTime = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_MEAN_TIME);
    if(PS_MeanTime< 0)
    {
        return sprintf(buf,"Read PS_MEAN_TIME Failed (%d) \n",PS_MeanTime);
    }

    return sprintf(buf,"PS Mean Time Register =0x%x \n",PS_MeanTime);
}

static ssize_t ps_MeanTimeRegister_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    u8 PS_MeanTime=0;
    int ret=0, temp=0;

    sscanf (buf, "%x", &temp);
    PS_MeanTime = (u8)temp;

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_MEAN_TIME, PS_MeanTime);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,ret);
    }

    return count;
}
static DEVICE_ATTR(ps_mean, S_IWUSR | S_IRUGO , ps_MeanTimeRegister_show, ps_MeanTimeRegister_store);
// add by leo for vendor testing --

//add by leo for proximity vendor calibration ++
static ssize_t ps_CalibrationRegister_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int PS_CrossTalk=0;
    unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;

    PS_VCALI_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_VCALI_H);
    PS_VCALI_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_VCALI_L);

    if(PS_VCALI_HByte<0||PS_VCALI_LByte<0)
    {
        return sprintf(buf,"Read PS_VCALI Failed\n");
    }
    PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));

    return sprintf(buf,"PS Vendor Calibration Register =%d \n",PS_CrossTalk);
}
static DEVICE_ATTR(ps_crosstalk, S_IRUGO,ps_CalibrationRegister_show,NULL);

static ssize_t ps_do_Vendor_Calibration_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int ret=0;

    ret = ps_get_vendor_calibration_value();
    if(ret<0)
    {
        //if(AP3212C_STATUSMSG) printk( "[%s]	Read PS CalibrationValue Failed \n",__FUNCTION__);
        return sprintf(buf,"Read PS Vendor CalibrationValue Failed (%d)\n",ret);
    }

    return sprintf(buf,"PS Vendor Calibration Value =%d \n",PS_VendorCalibrationValue);
}

static ssize_t ps_do_Vendor_Calibration_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    int ret=0,temp=0;
    struct ap3212c_i2c_data *ap3212c_data = data;

    sscanf (buf, "%x", &temp);
    if(temp==0)
    {
        //Clean PS Vendor Calibration Register
        ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_H,0);
        if(ret< 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
            return ret;
        }
        ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_L,0);
        if(ret< 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
            return ret;
        }

    }
    else if(temp==1)
    {

        PS_AlreadyVendorCalibration=TRUE;
        ret = ps_enable(ap3212c_data,POWER_ON);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
        }
        msleep(150);

        ret = ps_vendor_calibration();
        if(ret< 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Failed (%d)\n",__FUNCTION__,ret);
        }

        ret = ps_enable(ap3212c_data,POWER_OFF);
        if(ret < 0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
        }
        PS_AlreadyVendorCalibration=FALSE;
        PS_VendorCalibrationRetryCount=CalibrationRetryTimes;
    }

    return count;
}
static DEVICE_ATTR(ps_vcali, S_IWUSR | S_IRUGO ,ps_do_Vendor_Calibration_show,ps_do_Vendor_Calibration_store);
//add by leo for proximity vendor calibration --

//add by leo for modify MAX proximity crosstalk value ++
static ssize_t ps_MaxCrossTalkValue_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    return sprintf(buf,"PS allow Maximum Crosstalk Value=%d \n",limit_PS_CrossTalkValue);
}

static ssize_t ps_MaxCrossTalkValue_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    int temp=0;

    sscanf (buf, "%d", &temp);
    limit_PS_CrossTalkValue = temp;

    if(AP3212C_STATUSMSG) printk( "[%s]	Set PS maximum permit crosstalk value = %d \n",__FUNCTION__,limit_PS_CrossTalkValue);
    return count;
}
static DEVICE_ATTR(ps_crosstalk_limit, S_IWUSR | S_IRUGO , ps_MaxCrossTalkValue_show, ps_MaxCrossTalkValue_store);
//add by leo for modify MAX proximity crosstalk value --
static ssize_t als_threshold_dump_show(struct device *dev, struct device_attribute *attr, char *buf)
{

    int level=0;
    ssize_t sprintf_count = 0;

    for(level=0; level<(ALS_LEVEL+1); level++)
    {
        sprintf_count += sprintf(buf + sprintf_count, "Default_als_threshold_lux [%d] = %d (0x%02X)\n", level, Default_als_threshold_lux[level], Default_als_threshold_lux[level]);
        if(AP3212C_STATUSMSG) printk( "[%s]	Default_als_threshold_lux [%d] = %d (0x%02X)\n", __FUNCTION__, level, Default_als_threshold_lux[level], Default_als_threshold_lux[level]);
    }
    sprintf_count += sprintf(buf + sprintf_count, "\n");
    if(AP3212C_STATUSMSG) printk( "\n");

    for(level=0; level<(ALS_LEVEL+1); level++)
    {
        sprintf_count += sprintf(buf + sprintf_count, "Default_als_threshold_adc [%d] = %d (0x%02X)\n", level, Default_als_threshold_adc[level], Default_als_threshold_adc[level]);
        if(AP3212C_STATUSMSG) printk( "[%s]	Default_als_threshold_adc [%d] = %d (0x%02X)\n", __FUNCTION__, level, Default_als_threshold_adc[level], Default_als_threshold_adc[level]);
    }
    sprintf_count += sprintf(buf + sprintf_count, "\n");
    if(AP3212C_STATUSMSG) printk( "\n");

    for(level=0; level<(ALS_LEVEL+1); level++)
    {
        sprintf_count += sprintf(buf + sprintf_count, "als_threshold_lux [%d] = %d (0x%02X)\n", level, als_threshold_lux[level], als_threshold_lux[level]);
        if(AP3212C_STATUSMSG) printk( "[%s]	als_threshold_lux [%d] = %d (0x%02X)\n", __FUNCTION__, level, als_threshold_lux[level], als_threshold_lux[level]);
    }
    sprintf_count += sprintf(buf + sprintf_count, "\n");
    if(AP3212C_STATUSMSG) printk( "\n");

    for(level=0; level<(ALS_LEVEL+1); level++)
    {
        sprintf_count += sprintf(buf + sprintf_count, "als_threshold_adc [%d] = %d (0x%02X)\n", level, als_threshold_adc[level], als_threshold_adc[level]);
        if(AP3212C_STATUSMSG) printk( "[%s]	als_threshold_adc [%d] = %d (0x%02X)\n", __FUNCTION__, level, als_threshold_adc[level], als_threshold_adc[level]);
    }
    sprintf_count += sprintf(buf + sprintf_count, "\n");
    if(AP3212C_STATUSMSG) printk( "\n");

    return sprintf_count;
}

static ssize_t als_threshold_dump_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{

    int temp=0;
    sscanf (buf, "%d", &temp);
    return count;
}
static DEVICE_ATTR(als_thd_dump, S_IWUSR | S_IRUGO , als_threshold_dump_show, als_threshold_dump_store);
//=========================================================================================

static struct attribute *ap3212c_attributes[] =
{
    &dev_attr_ap3212c_id.attr,
    &dev_attr_ap3212c_int.attr,
    &dev_attr_ir_data.attr,
    &dev_attr_als_en.attr,
    &dev_attr_als_data.attr,
    &dev_attr_als_thd.attr,
    &dev_attr_als_thd_dump.attr,
    &dev_attr_als_cali.attr,
    &dev_attr_ps_en.attr,
    &dev_attr_ps_data.attr,
    &dev_attr_ps_status.attr,
    &dev_attr_ps_thd.attr,
    &dev_attr_ps_hthd.attr,
    &dev_attr_ps_lthd.attr,
    &dev_attr_ps_cali.attr,
    &dev_attr_lightsensor_status.attr,	//for ATD testing
    &dev_attr_lightsensor_adc.attr,	//for ATD testing
    &dev_attr_proximity_status.attr,	//for ATD testing
    &dev_attr_als_config.attr,		//for vendor testing
    &dev_attr_ps_config.attr,		//for vendor testing
    &dev_attr_ps_ledcontrol.attr,		//for vendor testing
    &dev_attr_ps_mean.attr,		//for vendor testing
    &dev_attr_ps_vcali.attr,			//add by leo for proximity vendor calibration ++
    &dev_attr_ps_crosstalk.attr,		//add by leo for proximity vendor calibration ++
    &dev_attr_ps_crosstalk_limit.attr,	//add by leo for modify MAX proximity crosstalk value ++
    NULL
};
static const struct attribute_group ap3212c_attr_group =
{
    .attrs = ap3212c_attributes,
};
//=========================================================================================
static int CharToInt(char CValue[])
{

    int i=0,j=0,x=0,CalibrationValue=0;
    int temp[CValue_len]= {0};

    if(unlikely(debug))printk( "[%s]	",__FUNCTION__);// add for test

    for(i=0; i<CValue_len; i++)
    {
        if(unlikely(debug))printk( "%c",CValue[i]);// add for test
        temp[i]=(int)CValue[i]-48;
    }
    if(unlikely(debug))printk( "\n"); // add for test

    CalibrationValue = temp[CValue_len-1];
    for(i=1; i<CValue_len; i++)
    {

        j=i;
        x=1;
        while(j>0)
        {
            x*=10;
            j--;
        }
        CalibrationValue+=temp[(CValue_len-1)-i]*x;
    }
    return CalibrationValue;
}
//=========================================================================================
static int als_open(struct inode *inode, struct file *file)
{
    if(unlikely(debug))printk( "[%s]	ALS MISC Device Open \n",__FUNCTION__);
    return nonseekable_open(inode, file);
}

static int als_release(struct inode *inode, struct file *file)
{
    if(unlikely(debug))printk( "[%s]	ALS MISC Device Release \n",__FUNCTION__);
    return 0;
}

static long als_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int err=0, val=0, Correct_LuxValue;
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long ADC_Data=0;
    struct ap3212c_i2c_data *ap3212c_data = data;
    //if(unlikely(debug))printk( "[%s] 	ALS IOCTL (Cmd is %d) \n",__FUNCTION__, _IOC_NR(cmd));

    if (_IOC_TYPE(cmd) != LIGHTSENSOR_IOCTL_MAGIC)
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

    switch (cmd)
    {
        case LIGHTSENSOR_IOCTL_ENABLE:

            if (get_user(val, (unsigned long __user *)arg))
                return -EFAULT;
            if (val)
            {
                if(unlikely(debug))printk( "[%s]	LIGHTSENSOR_IOCTL_ENABLE - als_enable \n",__FUNCTION__);
                return als_enable(ap3212c_data, POWER_ON);
            }
            else
            {
                if(unlikely(debug))printk( "[%s]	LIGHTSENSOR_IOCTL_ENABLE - als_disable \n",__FUNCTION__);
                return als_enable(ap3212c_data, POWER_OFF);
            }
            break;

        case LIGHTSENSOR_IOCTL_GET_ENABLED:

            if(unlikely(debug))printk( "[%s]	LIGHTSENSOR_IOCTL_GET_ENABLED (return %d) \n",__FUNCTION__,ap3212c_data->als_power_state);
            return put_user(ap3212c_data->als_power_state, (unsigned long __user *)arg);
            break;

        case LIGHTSENSOR_IOCTL_GET_RAWDATA:

            if(unlikely(debug))printk( "[%s]	LIGHTSENSOR_IOCTL_GET_RAWDATA \n",__FUNCTION__);
            value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_LOW);
            value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_HIGH);
            ADC_Data = ((value_HByte << 8) | value_LByte);

            return put_user(ADC_Data, (unsigned long __user *)arg);
            break;

        case LIGHTSENSOR_IOCTL_GET_LUXVALUE:

            mdelay(300);
            if(unlikely(debug))printk( "[%s]	LIGHTSENSOR_IOCTL_GET_LUXVALUE \n",__FUNCTION__);
            value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_LOW);
            value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_HIGH);
            ADC_Data = ((value_HByte << 8) | value_LByte);
            ap3212c_data->als_luxvalue = ADC_Data*625/10000*ALS_CalibrationValue_standard/ALS_CalibrationValue_real;

            if(unlikely(debug))printk( " [%s]	return %d (lux) \n",__FUNCTION__,ap3212c_data->als_luxvalue);
            return put_user(ap3212c_data->als_luxvalue, (unsigned long __user *)arg);
            break;

        case LIGHTSENSOR_IOCTL_CALIBRATION:

            if(unlikely(debug))printk( "[%s]	LIGHTSENSOR_IOCTL_CALIBRATION \n",__FUNCTION__);
            if (get_user(Correct_LuxValue, (unsigned long __user *)arg))
                return -EFAULT;

            err = als_calibration(ap3212c_data, Correct_LuxValue);
            if(err<0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ALS Calibration (set calibration value) Failed (%d)\n",__FUNCTION__,err);
                return err;
            }

            ALS_AlreadyCalibration=FALSE;
            err = ap3212c_calibration_lightsensor(ap3212c_data);
            if(err<0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ALS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,err);
                return err;
            }
            break;

        default:
            if(unlikely(debug))printk( "[%s]	Incorrect Cmd  (%d) \n",__FUNCTION__, _IOC_NR(cmd));
            return -EINVAL;
    }

    return 0;
}

static struct file_operations als_fops =
{
    .owner = THIS_MODULE,
    .open = als_open,
    .release = als_release,
    .unlocked_ioctl = als_ioctl
};

static struct miscdevice als_misc_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ap3212c_als_misc_dev",
    .fops = &als_fops,
};

static int als_setup(struct ap3212c_i2c_data *ap3212c_data)
{
    int ret=0;

    ap3212c_data->als_input_dev = input_allocate_device();
    if (!ap3212c_data->als_input_dev)
    {
        if(unlikely(debug))printk( "[%s]	Could Not Allocate ALS Input Device \n",__FUNCTION__);
        return -ENOMEM;
    }
    ap3212c_data->als_input_dev->name = "ap3212c_als_input_dev";
    set_bit(EV_ABS, ap3212c_data->als_input_dev->evbit);
    set_bit(EV_SYN, ap3212c_data->als_input_dev->evbit);
    input_set_abs_params(ap3212c_data->als_input_dev, ABS_MISC, 0, 5000, 0, 0);

    ret = input_register_device(ap3212c_data->als_input_dev);
    if (ret < 0)
    {
        if(unlikely(debug))printk( "[%s]	Could Not Register ALS Input Device \n",__FUNCTION__);
        goto register_als_input_device_err;
    }

    ret = misc_register(&als_misc_dev);
    if (ret < 0)
    {
        if(unlikely(debug))printk( "[%s]	Could Not Register ALS Misc Device\n",__FUNCTION__);
        goto register_als_misc_device_err;
    }

    if(unlikely(debug))printk( "[%s]	ALS_SETUP - FINISHED \n",__FUNCTION__);
    return 0;


register_als_misc_device_err:
    input_unregister_device(ap3212c_data->als_input_dev);
register_als_input_device_err:
    input_free_device(ap3212c_data->als_input_dev);
    return ret;
}

static int als_enable(struct ap3212c_i2c_data *ap3212c_data,int  power)
{
    int SystemMode=0, ret=0;

//add by leo for calibration ++
    if((power==POWER_ON)&&((ALS_AlreadyCalibration==FALSE)&&(ALS_CalibrationRetryCount>0)))
    {
        ret=ap3212c_calibration_lightsensor(ap3212c_data);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]			ALS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
        }
    }
//add by leo for calibration --

    SystemMode = i2c_smbus_read_byte_data(ap3212c_i2c_client, SYSTEM_CONFIG);
    if(SystemMode< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed\n",__FUNCTION__);
        return SystemMode;
    }

    SystemMode = (SystemMode & 0xfe)|power; // 0xfe = 1111 1110

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, SYSTEM_CONFIG, SystemMode);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write SYSTEM_CONFIG Failed\n",__FUNCTION__);
        return ret;
    }

    ap3212c_data->als_power_state= power;

    if(AP3212C_STATUSMSG) printk( "[%s]			Power(%d), SystemMode = 0x%x \n",__FUNCTION__,ap3212c_data->als_power_state,SystemMode);

    if(power==POWER_ON)
    {
        msleep(300); // add by leo for report current lux value when enable/disable, suspend/resume
        interruptStatus = 0x01;//assume there is a ALS interrupt occured , ie report event immediatly when light sensor is enabled
        disable_irq_nosync(ap3212c_data->irq);
        queue_delayed_work(ap3212c_wq, &ap3212c_work,0);
//add by leo for reset als_input_dev ++
    }
    else
    {
        input_report_abs(ap3212c_data->als_input_dev, ABS_MISC, RESET_INPUT_DEVICE);
        input_sync(ap3212c_data->als_input_dev);
//add by leo for reset als_input_dev --
    }
    return 0;
}

static int als_initial(struct ap3212c_i2c_data *ap3212c_data)
{
    int ret=0, ilevel=0, ALS_Configuration=0;

    for(ilevel=0; ilevel<(ALS_LEVEL+1); ilevel++)
    {

        Default_als_threshold_adc[ilevel]= Default_als_threshold_lux[ilevel]*10000/625;//Lux Value to ADC Count

        als_threshold_lux[ilevel]=Default_als_threshold_lux[ilevel];
        als_threshold_adc[ilevel]=Default_als_threshold_adc[ilevel];
    }
    Default_als_threshold_adc[ALS_LEVEL]=0xFFFF;	//Default_als_threshold_adc[18]=0
    als_threshold_adc[ALS_LEVEL]=0xFFFF;			//als_threshold_adc[18]=0

    ret = als_enable(ap3212c_data, POWER_OFF);
    if(ret < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	ALS Shut down Failed \n",__FUNCTION__);
        return ret;
    }

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, ALS_CONFIG, 0x21);//Reserved:00, ALS dynamic range:10, ALS persist:00001
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write ALS_CONFIG Failed\n",__FUNCTION__);
        return ret;
    }

    ALS_Configuration = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_CONFIG);
    if(ALS_Configuration< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read ALS_CONFIG Failed\n",__FUNCTION__);
        return ALS_Configuration;
    }

    if(unlikely(debug))printk( "[%s]			als_initial finished, ALS_CONFIG = 0x%x\n",__FUNCTION__,ALS_Configuration);
    return 0;
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
    int err=0, val=0, control=0;
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long RAW_Data=0;
    struct ap3212c_i2c_data *ap3212c_data = data;
    //if(unlikely(debug))printk( "[%s] 	PS IOCTL (Cmd is %d) \n",__FUNCTION__, _IOC_NR(cmd));

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

    //--------------------------------------------------
    switch (cmd)
    {
        case PROXIMITYSENSOR_IOCTL_ENABLE:

            if (get_user(val, (unsigned long __user *)arg))
                return -EFAULT;
            if (val)
            {
                if(unlikely(debug))printk( "[%s]	PROXIMITYSENSOR_IOCTL_ENABLE - ps_enable \n",__FUNCTION__);
                return ps_enable(ap3212c_data, POWER_ON);
            }
            else
            {
                if(unlikely(debug))printk( "[%s]	PROXIMITYSENSOR_IOCTL_ENABLE - ps_disable \n",__FUNCTION__);
                return ps_enable(ap3212c_data, POWER_OFF);
            }
            break;

        case PROXIMITYSENSOR_IOCTL_GET_ENABLED:

            if(unlikely(debug))printk( "[%s]	PROXIMITYSENSOR_IOCTL_GET_ENABLED (return %d) \n",__FUNCTION__,ap3212c_data->ps_power_state);

            return put_user(ap3212c_data->ps_power_state, (unsigned long __user *)arg);
            break;

        case PROXIMITYSENSOR_IOCTL_GET_STATUS:

            mdelay(300);
            value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_LOW);
            value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
            RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
            if(unlikely(debug))printk( "[%s]	PS DATA = %d\n",__FUNCTION__,(int)RAW_Data);

            ap3212c_data->ps_status = (value_HByte & 0x80)>>7;
            if(unlikely(debug))printk( "[%s]	PROXIMITYSENSOR_IOCTL_GET_STATUS (return %d)\n",__FUNCTION__,ap3212c_data->ps_status);
            return put_user(ap3212c_data->ps_status, (unsigned long __user *)arg);
            break;

        case PROXIMITYSENSOR_IOCTL_CALIBRATION:

            if (get_user(control, (unsigned long __user *)arg))
                return -EFAULT;

            if(unlikely(debug))printk( "[%s]	PROXIMITYSENSOR_IOCTL_CALIBRATION (%d)\n",__FUNCTION__,control);
            err = ps_calibration(ap3212c_data,control);
            if(err<0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration (%d)(set calibration value) Failed (%d)\n",__FUNCTION__,control,err);
                return err;
            }

            PS_AlreadyCalibration=FALSE;
            PS_CalibrationRetryCount=CalibrationRetryTimes;
            err = ap3212c_calibration_proximity(ap3212c_data);
            if(err<0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	PS Calibration (%d)(get calibration value) Failed (%d)\n",__FUNCTION__,control,err);
                //return err;
            }

            //return put_user(PS_CalibrationValue, (unsigned long __user *)arg);
            break;

        case PROXIMITYSENSOR_IOCTL_VCALIBRATION:

            if(unlikely(debug))printk( "[%s]	PROXIMITYSENSOR_IOCTL_VCALIBRATION \n",__FUNCTION__);

            PS_AlreadyVendorCalibration=TRUE;
            err = ps_enable(ap3212c_data,POWER_ON);
            if(err < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_ON);
                return err;
            }
            msleep(150);

            err = ps_vendor_calibration();
            if(err< 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration Failed (%d)\n",__FUNCTION__,err);
                return err;
            }

            err = ps_enable(ap3212c_data,POWER_OFF);
            if(err < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
                return err;
            }
            PS_AlreadyVendorCalibration=FALSE;
            PS_VendorCalibrationRetryCount=CalibrationRetryTimes;

            err = ps_get_vendor_calibration_value();
            if(err<0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]	Read PS Vendor CalibrationValue Failed (%d)\n",__FUNCTION__,err);
                return err;
            }

            printk( "[%s]	PS Vendor Calibration Value = %d\n",__FUNCTION__,PS_VendorCalibrationValue);

            return put_user(PS_VendorCalibrationValue, (unsigned long __user *)arg);
            break;

        default:
            if(unlikely(debug))printk( "[%s]	Incorrect Cmd  (%d) \n",__FUNCTION__, _IOC_NR(cmd));
            return -EINVAL;
    }

    return 0;
}

static struct file_operations ps_fops =
{
    .owner = THIS_MODULE,
    .open = ps_open,
    .release = ps_release,
    .unlocked_ioctl = ps_ioctl
};

static struct miscdevice ps_misc_dev =
{
    .minor = MISC_DYNAMIC_MINOR,
    .name = "ap3212c_ps_misc_dev",
    .fops = &ps_fops,
};

static int ps_setup(struct ap3212c_i2c_data *ap3212c_data)
{
    int ret=0;

    ap3212c_data->ps_input_dev = input_allocate_device();
    if (!ap3212c_data->ps_input_dev)
    {
        if(unlikely(debug))printk( "[%s]	Could Not Allocate PS Input Device\n",__FUNCTION__);
        return -ENOMEM;
    }
    ap3212c_data->ps_input_dev->name = "ap3212c_ps_input_dev";
    set_bit(EV_ABS, ap3212c_data->ps_input_dev->evbit);
    set_bit(EV_SYN, ap3212c_data->ps_input_dev->evbit);
    input_set_abs_params(ap3212c_data->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);

    ret = input_register_device(ap3212c_data->ps_input_dev);
    if (ret < 0)
    {
        if(unlikely(debug))printk( "[%s]	Could Not Register PS Input Device\n",__FUNCTION__);
        goto register_ps_input_device_err;
    }

    ret = misc_register(&ps_misc_dev);
    if (ret < 0)
    {
        if(unlikely(debug))printk( "[%s]	Could Not Register PS Misc Device\n",__FUNCTION__);
        goto register_ps_misc_device_err;
    }

    if(unlikely(debug))printk( "[%s]	PS_SETUP - FINISHED \n",__FUNCTION__);
    return 0;

register_ps_misc_device_err:
    input_unregister_device(ap3212c_data->ps_input_dev);
register_ps_input_device_err:
    input_free_device(ap3212c_data->ps_input_dev);
    return ret;
}

static int ps_enable(struct ap3212c_i2c_data *ap3212c_data,int  power)
{
    int SystemMode=0, ret=0;

//add by leo for proximity cross talk calibration ++
    if((power==POWER_ON)&&(PS_AlreadyVendorCalibration==FALSE)&&(PS_VendorCalibrationRetryCount>0))
    {
        //if(unlikely(debug))printk( "[%s]	PS_CalibrationCrossTalk==FALSE\n",__FUNCTION__);
        ret = ap3212c_vendor_calibration_proximity(ap3212c_data);
        if(ret<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]			PS Vendor Calibration  (get cross talk value) Failed (%d)\n",__FUNCTION__,ret);
        }
        /*
        		else{
        			if(PS_VendorCalibrationValue >= (PS_LOW_THD-20)){
        				if(AP3212C_STATUSMSG) printk( "[%s]	Failed, PS Cross Talk (%d) > PS_LOW_THD-20 (%d)\n",__FUNCTION__,PS_VendorCalibrationValue,PS_LOW_THD-20);
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

//add by leo for calibration ++
    if((power==POWER_ON)&&((PS_AlreadyCalibration==FALSE)&&(PS_CalibrationRetryCount>0)))
    {
        ret=ap3212c_calibration_proximity(ap3212c_data);
        if(ret<0)
        {
            if(unlikely(debug))printk( "[%s]			PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
        }
    }
//add by leo for calibration --
//add by leo for proximity cross talk calibration --

    SystemMode = i2c_smbus_read_byte_data(ap3212c_i2c_client, SYSTEM_CONFIG);
    if(SystemMode< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read SYSTEM_CONFIG Failed\n",__FUNCTION__);
        return SystemMode;
    }

    power = power << 1;
    SystemMode = (SystemMode & 0xfd)|power; // 0xfd = 1111 1101

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, SYSTEM_CONFIG, SystemMode);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write SYSTEM_CONFIG Failed\n",__FUNCTION__);
        return ret;
    }

//add by leo for proximity reset ++
    if(power==POWER_OFF)
    {
        PS_CurrentStatus = -1;
        input_report_abs(ap3212c_data->ps_input_dev, ABS_DISTANCE, RESET_INPUT_DEVICE);
        input_sync(ap3212c_data->ps_input_dev);
// add by leo for proximity wake up source ++
        if(ap3212c_data->ps_power_state!=(power>>1))
        {
            if(unlikely(debug))printk( "[%s]				disable_irq_wake  (ap3212c_data->ps_power_state, power)=(%d,%d)\n",__FUNCTION__,ap3212c_data->ps_power_state,power>>1);
            disable_irq_wake(ap3212c_data->irq);
        }
    }
    else
    {
        if(ap3212c_data->ps_power_state!=(power>>1))
        {
            if(unlikely(debug))printk( "[%s]				enable_irq_wake  (ap3212c_data->ps_power_state, power)=(%d,%d)\n",__FUNCTION__,ap3212c_data->ps_power_state,power>>1);
            enable_irq_wake(ap3212c_data->irq);
        }
    }
// add by leo for proximity wake up source --
//add by leo for proximity reset --

    ap3212c_data->ps_power_state= power>>1;
    if(AP3212C_STATUSMSG) printk( "[%s]			Power(%d), SystemMode = 0x%x \n",__FUNCTION__,ap3212c_data->ps_power_state,SystemMode);

    return 0;
}

static int ps_initial(struct ap3212c_i2c_data *ap3212c_data)
{
    int ret=0, PS_Configuration=0, PS_LEDControl=0, PS_MeanTime=0;

    ap3212c_data->ps_power_state=0;
    ret = ps_enable(ap3212c_data, POWER_OFF);
    if(ret < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	PS Shut down Failed \n",__FUNCTION__);
        return ret;
    }

//add by leo for PS Mean Time setting ++

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_MEAN_TIME, 0x00);// 000000 00: mean time = 12.5ms
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }

    PS_MeanTime = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_MEAN_TIME);
    if(PS_MeanTime< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PS_MEAN_TIME Failed (%d)\n",__FUNCTION__,PS_MeanTime);
        return PS_MeanTime;
    }
    if(AP3212C_STATUSMSG) printk( "[%s]			PS_MEAN_TIME = 0x%x\n",__FUNCTION__,PS_MeanTime);

//add by leo for PS Mean Time setting --

    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_LED_CONTROL, 0x33);//PS LED control: 0011 0011, LED pulse:11 (3 pulse), LED driver ratio:11 (100%)
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_LED_CONTROL Failed\n",__FUNCTION__);
        return ret;
    }

    PS_LEDControl = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_LED_CONTROL);
    if(PS_LEDControl< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PS_LED_CONTROL Failed\n",__FUNCTION__);
        return PS_LEDControl;
    }
    if(AP3212C_STATUSMSG) printk( "[%s]			PS_LED_CONTROL = 0x%x\n",__FUNCTION__,PS_LEDControl);


    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_CONFIG, 0xe4);//RS integrated time select:1110 (15T), PS gain:10, PS persist(interrupt filter):00
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Write PS_CONFIG Failed\n",__FUNCTION__);
        return ret;
    }

    PS_Configuration = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_CONFIG);
    if(PS_Configuration< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PS_CONFIG Failed\n",__FUNCTION__);
        return PS_Configuration;
    }
    if(unlikely(debug))printk( "[%s]			ps_initial finished, PS_CONFIG = 0x%x\n",__FUNCTION__,PS_Configuration);

    ret = ps_set_threshold(PS_HIGH_THD, PS_LOW_THD);
    if(ret < 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	ps_set_threshold (%d, %d) Failed\n",__FUNCTION__,PS_HIGH_THD, PS_LOW_THD);
        return ret;
    }

    return 0;
}
//=========================================================================================
static int als_calibration(struct ap3212c_i2c_data *ap3212c_data, int ALS_StandardValue)
{

    int ilevel=0, ALS_RealValue=0;
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long ADC_Data=0;
    struct file *fp=NULL;
    mm_segment_t old_fs;
    char ALS_CValue[CalibrationValue_len]= {0};

    if(ALS_StandardValue<0||ALS_StandardValue>10240)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Please Enter Correct Lux Value \n",__FUNCTION__);
        return -EBADMSG;

    }
    else if(ALS_StandardValue==0)
    {

        ALS_RealValue=Default_ALS_CalibrationValue;
        ALS_StandardValue=Default_ALS_CalibrationValue;

        for(ilevel=0; ilevel<(ALS_LEVEL+1); ilevel++)
        {
            als_threshold_lux[ilevel]=Default_als_threshold_lux[ilevel];
            als_threshold_adc[ilevel]=Default_als_threshold_adc[ilevel];
        }
    }
    else
    {
        value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_LOW);
        value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_HIGH);
        ADC_Data = ((value_HByte << 8) | value_LByte);
        ALS_RealValue = ADC_Data*625/10000;
    }

    if(AP3212C_STATUSMSG) printk( "[%s]		ALS_LuxValue = %d (Before Calibration)\n",__FUNCTION__,ALS_RealValue);

    sprintf(ALS_CValue,"%010d&%010d",ALS_StandardValue, ALS_RealValue);

    /* Write the Calibration Number Into File */
    fp=filp_open(ALS_CONFIG_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
    if (IS_ERR_OR_NULL(fp))
    {
        if(AP3212C_STATUSMSG) printk("[%s]		File Open Failed \n",__FUNCTION__);
        return -ENOENT;
    }
    if(fp->f_op != NULL && fp->f_op->write != NULL)
    {
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fp->f_op->write(fp,ALS_CValue,CalibrationValue_len,&fp->f_pos);

        set_fs(old_fs);
    }
    filp_close(fp,NULL);

    if(AP3212C_STATUSMSG) printk("[%s]		Save Calibration Value (%010d&%010d)\n",__FUNCTION__,ALS_StandardValue, ALS_RealValue);
    return 0;
}

static int als_get_calibration_value(struct ap3212c_i2c_data *ap3212c_data)
{

    int ilen=0, i=0;
    int ALS_CalibrationValue=0;
    char ALS_CValue[CalibrationValue_len] = {0};
    char ALS_CValue_standard[CValue_len] = {0}, ALS_CValue_real[CValue_len] = {0};
    struct file *fp=NULL;
    mm_segment_t old_fs;

    fp=filp_open(ALS_CONFIG_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
    if (IS_ERR_OR_NULL(fp))
    {
        if(AP3212C_STATUSMSG) printk("[%s]	File Open Failed \n",__FUNCTION__);
        return -ENOENT;
    }

    if(fp->f_op != NULL && fp->f_op->read != NULL)
    {

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        ilen = fp->f_op->read(fp,ALS_CValue,CalibrationValue_len,&fp->f_pos);
        if(unlikely(debug))printk("[%s]		fp->f_op->read (len = %d) \n",__FUNCTION__,ilen);

        set_fs(old_fs);
    }
    filp_close(fp,NULL);

    if(ilen!=CalibrationValue_len)
    {
        if(AP3212C_STATUSMSG) printk("[%s]	Can Not Find ALS Calibration Value (len = %d)\n",__FUNCTION__,ilen);
        return -EBADMSG;
    }

    for(i=0; i<CalibrationValue_len; i++)
    {
        if(i<CValue_len)
        {
            ALS_CValue_standard[i]=ALS_CValue[i];
        }
        if(i>CValue_len)
        {
            ALS_CValue_real[i-CValue_len-1]=ALS_CValue[i];
        }
    }

    mutex_lock(&ap3212c_mutex);
    ALS_CalibrationValue_standard = CharToInt(ALS_CValue_standard);
    ALS_CalibrationValue_real = CharToInt(ALS_CValue_real);

    if(ALS_CalibrationValue_standard==0 || ALS_CalibrationValue_real==0)
    {
        ALS_CalibrationValue_standard=Default_ALS_CalibrationValue;
        ALS_CalibrationValue_real=Default_ALS_CalibrationValue;
        ALS_CalibrationValue = -1;
    }
    else
    {
        ALS_CalibrationValue = ALS_CalibrationValue_standard/ALS_CalibrationValue_real;
    }
    mutex_unlock(&ap3212c_mutex);

    if(ALS_CalibrationValue<0)
    {
        if(AP3212C_STATUSMSG) printk("[%s]	ALS Calibration Failed \n",__FUNCTION__);
        return -EBADMSG;
    }
    else
    {
        //ap3212c_data->als_calibration_value = ALS_CalibrationValue;
        if(AP3212C_STATUSMSG)printk("[%s]	ALS_CalibrationValue =%d/%d \n",__FUNCTION__,ALS_CalibrationValue_standard,ALS_CalibrationValue_real);
    }
    return 0;
}

static int ps_calibration(struct ap3212c_i2c_data *ap3212c_data, int iControl)
{

    int PS_HighThresholdValue=0, PS_LowThresholdValue=0;
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long RAW_Data=0;
    struct file *fp=NULL;
    mm_segment_t old_fs;
    char PS_CValue[CalibrationValue_len]= {0};

    if(iControl==0)
    {
        PS_LowThresholdValue=Default_PS_ThresholdValue;
        PS_HighThresholdValue=Default_PS_ThresholdValue;

    }
    else
    {
        msleep(300);
        value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_LOW);
        value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
        RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111

        switch (iControl)
        {

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

    if(AP3212C_STATUSMSG) printk("[%s]			PS_CalibrationValue = (%d, %d) (Before Calibration)\n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);

    sprintf(PS_CValue,"%010d&%010d",PS_HighThresholdValue, PS_LowThresholdValue);

    fp=filp_open(PS_CONFIG_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
    if (IS_ERR_OR_NULL(fp))
    {
        if(AP3212C_STATUSMSG) printk("[%s]		File Open Failed \n",__FUNCTION__);
        return -ENOENT;
    }
    if(fp->f_op != NULL && fp->f_op->write != NULL)
    {
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fp->f_op->write(fp,PS_CValue,CalibrationValue_len,&fp->f_pos);

        set_fs(old_fs);
    }
    filp_close(fp,NULL);

    if(AP3212C_STATUSMSG) printk("[%s]			Save Calibration Value (%010d&%010d)\n",__FUNCTION__,PS_LowThresholdValue,PS_HighThresholdValue);

    return 0;
}

static int ps_get_calibration_value(struct ap3212c_i2c_data *ap3212c_data)
{

    int ilen=0,i=0;
    int PS_LowThresholdValue=0, PS_HighThresholdValue=0;
    char PS_CValue[CalibrationValue_len] = {0};
    char PS_CValue_low[CValue_len] = {0}, PS_CValue_high[CValue_len] = {0};
    struct file *fp=NULL;
    mm_segment_t old_fs;

    fp=filp_open(PS_CONFIG_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
    if (IS_ERR_OR_NULL(fp))
    {
        if(AP3212C_STATUSMSG) printk("[%s]	File Open Failed \n",__FUNCTION__);
        return -ENOENT;
    }

    if(fp->f_op != NULL && fp->f_op->read != NULL)
    {

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        ilen = fp->f_op->read(fp,PS_CValue,CalibrationValue_len,&fp->f_pos);
        if(unlikely(debug))printk("[%s]		fp->f_op->read (len = %d) \n",__FUNCTION__,ilen);

        set_fs(old_fs);
    }
    filp_close(fp,NULL);

    if(ilen!=CalibrationValue_len)
    {
        if(unlikely(debug)) printk("[%s]		Can Not Find PS Calibration Value (len = %d)\n",__FUNCTION__,ilen);
        return -EBADMSG;
    }

    for(i=0; i<CalibrationValue_len; i++)
    {
        if(i<CValue_len)
        {
            PS_CValue_high[i]=PS_CValue[i];
        }
        if(i>CValue_len)
        {
            PS_CValue_low[i-CValue_len-1]=PS_CValue[i];
        }
    }
    PS_HighThresholdValue = CharToInt(PS_CValue_high);
    PS_LowThresholdValue = CharToInt(PS_CValue_low);

    if((PS_HighThresholdValue>0)&&(PS_LowThresholdValue>0))
    {
        PS_HighCalibrationValue= PS_HighThresholdValue;
        PS_LowCalibrationValue= PS_LowThresholdValue;
        if(AP3212C_STATUSMSG)printk("[%s]	PS_CalibrationValue = (%d, %d) \n",__FUNCTION__,PS_HighCalibrationValue,PS_LowCalibrationValue);

    }
    else
    {
        if(AP3212C_STATUSMSG) printk("[%s]	PS Calibration Failed \n",__FUNCTION__);
        return -EBADMSG;
    }
    return 0;
}

// add by leo for proximity vendor calibration ++
static int ps_vendor_calibration(void)
{

    int i=0,temp=0;
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long RAW_Data=0;
    int ret=0,PS_CrossTalk=0;
    unsigned int PS_VCALI_HByte=0, PS_VCALI_LByte=0;
    char PS_VCValue[CalibrationValue_len] = {0};
    struct file *fp=NULL;
    mm_segment_t old_fs;

    //Clean PS Vendor Calibration Register
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_H,0);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_L,0);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Write PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }

    //Read PS data 20 times to get average cross talk value
    for(i=0; i<21; i++)
    {
        value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_LOW);
        value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
        if(value_LByte<0||value_HByte<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		Read PS_DATA Failed\n",__FUNCTION__);
            return -1;
        }
        RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
        if(i!=0)if(AP3212C_STATUSMSG) printk( "[%s]		PS DATA (%d) = %d\n",__FUNCTION__,i,(int)RAW_Data);

        if(i!=0)temp=temp+(int)RAW_Data;
        msleep(150);
    }
    PS_CrossTalk=temp/20;
    if(AP3212C_STATUSMSG) printk( "[%s]		PS Cross Talk = %d\n",__FUNCTION__,PS_CrossTalk);

    // check the crosstalk value
    if(PS_CrossTalk>limit_PS_CrossTalkValue)return -EDOM;

    //Write cross talk value into PS Vendor Calibration Register
    PS_VCALI_HByte = PS_CrossTalk >> 1;
    PS_VCALI_LByte = PS_CrossTalk & 0x01;
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_H,PS_VCALI_HByte);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Read PS_VCALI_H Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }
    ret = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_VCALI_L,PS_VCALI_LByte);
    if(ret< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Read PS_VCALI_L Failed (%d)\n",__FUNCTION__,ret);
        return ret;
    }

    PS_VCALI_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_VCALI_H);
    PS_VCALI_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_VCALI_L);
    if(PS_VCALI_HByte<0||PS_VCALI_LByte<0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]		Read PS_VCALI Failed\n",__FUNCTION__);
        return -1;
    }
    PS_CrossTalk = (int)((PS_VCALI_HByte<<1)|(PS_VCALI_LByte & 0x01));
    if(AP3212C_STATUSMSG) printk( "[%s]		PS Vendor Calibration Register =%d\n",__FUNCTION__,PS_CrossTalk);

    //Backup cross talk value in data/sensors/ps_crosstalk.ini
    sprintf(PS_VCValue,"%010d&%010d",PS_CrossTalk, PS_CrossTalk);

    fp=filp_open(PS_CROSSTALK_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
    if (IS_ERR_OR_NULL(fp))
    {
        if(AP3212C_STATUSMSG) printk("[%s]		File Open Failed \n",__FUNCTION__);
        return -ENOENT;
    }
    if(fp->f_op != NULL && fp->f_op->write != NULL)
    {
        old_fs = get_fs();
        set_fs(KERNEL_DS);

        fp->f_op->write(fp,PS_VCValue,CalibrationValue_len,&fp->f_pos);

        set_fs(old_fs);
    }
    filp_close(fp,NULL);

    return PS_CrossTalk;
}

static int ps_get_vendor_calibration_value(void)
{

    int ilen=0,i=0;
    int PS_CrossTalk=0;
    char PS_VCValue[CalibrationValue_len] = {0};
    char PS_VCValue_real[CValue_len] = {0};
    struct file *fp=NULL;
    mm_segment_t old_fs;

    fp=filp_open(PS_CROSSTALK_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
    if (IS_ERR_OR_NULL(fp))
    {
        if(AP3212C_STATUSMSG) printk("[%s]	File Open Failed \n",__FUNCTION__);
        return -ENOENT;
    }

    if(fp->f_op != NULL && fp->f_op->read != NULL)
    {

        old_fs = get_fs();
        set_fs(KERNEL_DS);

        ilen = fp->f_op->read(fp,PS_VCValue,CalibrationValue_len,&fp->f_pos);
        if(unlikely(debug))printk("[%s]	fp->f_op->read (len = %d) \n",__FUNCTION__,ilen);

        set_fs(old_fs);
    }
    filp_close(fp,NULL);

    if(ilen!=CalibrationValue_len)
    {
        if(AP3212C_STATUSMSG) printk("[%s]Can Not Find PS Vendor Calibration Value (len = %d)\n",__FUNCTION__,ilen);
        return -EBADMSG;
    }

    for(i=0; i<CalibrationValue_len; i++)
    {
        if(i>CValue_len)
        {
            PS_VCValue_real[i-CValue_len-1]=PS_VCValue[i];
        }
    }

    PS_CrossTalk = CharToInt(PS_VCValue_real);

    if(PS_CrossTalk>0)
    {
        PS_VendorCalibrationValue= PS_CrossTalk;
        if(AP3212C_STATUSMSG)printk("[%s]PS_CrossTalk =%d \n",__FUNCTION__,PS_VendorCalibrationValue);

    }
    else
    {
        if(AP3212C_STATUSMSG) printk("[%s]	PS Vendor Calibration Failed \n",__FUNCTION__);
        return -EBADMSG;
    }
    return PS_CrossTalk ;
}
// add by leo for proximity vendor calibration --

static int als_set_threshold(int ilevel)
{
    int ret1=0,ret2=0,ret3=0,ret4=0;
    unsigned int ALS_THDH_LByte=0, ALS_THDH_HByte=0;
    unsigned int ALS_THDL_LByte=0, ALS_THDL_HByte=0;
    unsigned long ALS_THDH=0, ALS_THDL=0;

    mutex_lock(&ap3212c_mutex);

    ALS_THDH_HByte = als_threshold_adc[ilevel]>>8;
    ALS_THDH_LByte = als_threshold_adc[ilevel];

    ret1 = i2c_smbus_write_byte_data(ap3212c_i2c_client, ALS_THDH_H, ALS_THDH_HByte);
    ret2 = i2c_smbus_write_byte_data(ap3212c_i2c_client, ALS_THDH_L, ALS_THDH_LByte);

    ALS_THDL_HByte = als_threshold_adc[ilevel-1]>>8;
    ALS_THDL_LByte = als_threshold_adc[ilevel-1];

    ret3 = i2c_smbus_write_byte_data(ap3212c_i2c_client, ALS_THDL_H, ALS_THDL_HByte);
    ret4 = i2c_smbus_write_byte_data(ap3212c_i2c_client, ALS_THDL_L, ALS_THDL_LByte);

    mutex_unlock(&ap3212c_mutex);

    if(ret1< 0||ret2< 0||ret3< 0||ret4< 0)
    {
        return -1;
    }

    ALS_THDH_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDH_H);
    ALS_THDH_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDH_L);

    ALS_THDL_HByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDL_H);
    ALS_THDL_LByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_THDL_L);

    if(ALS_THDH_HByte< 0||ALS_THDH_LByte < 0||ALS_THDL_HByte< 0||ALS_THDL_LByte< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read ALS threshold Failed\n",__FUNCTION__);
        return -1;
    }
    ALS_THDH = ((ALS_THDH_HByte<<8)|ALS_THDH_LByte);
    ALS_THDL= ((ALS_THDL_HByte<<8)|ALS_THDL_LByte);

    if(unlikely(debug))printk( "[%s]			als original threshold (%d), THDL=%d (ADC), THDH=%d (ADC)\n",__FUNCTION__,ilevel,(int)ALS_THDL,(int)ALS_THDH);
    if(unlikely(debug))printk( "[%s]			als original threshold (%d), THDL=%d (Lux), THDH=%d (Lux)\n",__FUNCTION__,ilevel,(int)ALS_THDL*625/10000,(int)ALS_THDH*625/10000);
    if(unlikely(debug))printk( "[%s]			als_set_threshold (%d), THDL=%d (ADC), THDH=%d (ADC)\n",__FUNCTION__,ilevel,(int)ALS_THDL*ALS_CalibrationValue_standard/ALS_CalibrationValue_real,(int)ALS_THDH*ALS_CalibrationValue_standard/ALS_CalibrationValue_real);
    if(unlikely(debug))printk( "[%s]			als_set_threshold (%d), THDL=%d (Lux), THDH=%d (Lux)\n",__FUNCTION__,ilevel,(int)ALS_THDL*625/10000*ALS_CalibrationValue_standard/ALS_CalibrationValue_real,(int)ALS_THDH*625/10000*ALS_CalibrationValue_standard/ALS_CalibrationValue_real);
    return 0;
}

static int ps_set_threshold(int high_thd,int low_thd)
{
    int ret1=0,ret2=0,ret3=0,ret4=0;
    unsigned int PS_THDH_LByte=0, PS_THDH_HByte=0;
    unsigned int PS_THDL_LByte=0, PS_THDL_HByte=0;
    unsigned long PS_THDH=0, PS_THDL=0;

    mutex_lock(&ap3212c_mutex);
    PS_THDH_HByte = high_thd >> 2;
    PS_THDH_LByte = high_thd & 0x03;

    ret1 = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDH_H, PS_THDH_HByte);
    ret2 = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDH_L, PS_THDH_LByte);

    PS_THDL_HByte = low_thd >> 2;
    PS_THDL_LByte = low_thd & 0x03;

    ret3 = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDL_H, PS_THDL_HByte);
    ret4 = i2c_smbus_write_byte_data(ap3212c_i2c_client, PS_THDL_L, PS_THDL_LByte);

    mutex_unlock(&ap3212c_mutex);
    if(ret1< 0||ret2< 0||ret3< 0||ret4< 0)
    {
        return -1;
    }

    PS_THDH_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDH_H);
    PS_THDH_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDH_L);

    PS_THDL_HByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDL_H);
    PS_THDL_LByte= i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_THDL_L);

    if(PS_THDH_HByte< 0||PS_THDH_LByte < 0||PS_THDL_HByte< 0||PS_THDL_LByte< 0)
    {
        if(AP3212C_STATUSMSG) printk( "[%s]	Read PS threshold Failed\n",__FUNCTION__);
        return -1;
    }

    PS_THDH = ((PS_THDH_HByte<<2)|(PS_THDH_LByte & 0x03));
    PS_THDL = ((PS_THDL_HByte <<2)|(PS_THDL_LByte & 0x03));
    if(AP3212C_STATUSMSG) printk( "[%s]		ps_set_threshold, PS_THDH=%d, PS_THDL=%d \n",__FUNCTION__,(int)PS_THDH, (int)PS_THDL);

    return 0;
}

static int als_work_function(struct ap3212c_i2c_data *ap3212c_data)
{
    int ret=0, ilevel=0;

    for(ilevel=0; ilevel<(ALS_LEVEL+1); ilevel++)
    {
        //if(Default_als_threshold_lux[ilevel]>ap3212c_data->als_luxvalue){
        //if(Default_als_threshold_adc[ilevel]>ap3212c_data->als_adcvalue){
        if(als_threshold_adc[ilevel]>ap3212c_data->als_original_adcvalue)
        {
            ret = als_set_threshold(ilevel);
            if(ret < 0)
            {
                if(AP3212C_STATUSMSG) printk( "[%s]		als_set_threshold ( level %d ) Failed (%d)\n",__FUNCTION__,ilevel,ret);
                return ret;
            }
            break;
        }
    }

    input_report_abs(ap3212c_data->als_input_dev, ABS_MISC, ap3212c_data->als_luxvalue);
    input_sync(ap3212c_data->als_input_dev);

    if(AP3212C_STATUSMSG) printk( "[%s]		ALS Report %d (lux), Level (%d) \n",__FUNCTION__,ap3212c_data->als_luxvalue,ilevel);
    return 0;
}

static int ps_work_function(struct ap3212c_i2c_data *ap3212c_data)
{
    if(PS_CurrentStatus != ap3212c_data->ps_status)
    {

        PS_CurrentStatus = ap3212c_data->ps_status;
        //input_report_abs(ap3212c_data->ps_input_dev, ABS_DISTANCE, PS_CurrentStatus);
        //input_sync(ap3212c_data->ps_input_dev);

        if(PS_CurrentStatus)
        {
            input_report_abs(ap3212c_data->ps_input_dev, ABS_DISTANCE, 0);
            input_sync(ap3212c_data->ps_input_dev);
            if(AP3212C_STATUSMSG) printk( "[%s]		PS Report 'Close' \n",__FUNCTION__);
        }
        else
        {
            input_report_abs(ap3212c_data->ps_input_dev, ABS_DISTANCE, 9);
            input_sync(ap3212c_data->ps_input_dev);
            if(AP3212C_STATUSMSG) printk( "[%s]		PS Report 'Away' \n",__FUNCTION__);
        }

        //proximity_sensor_status(PS_CurrentStatus); // add by leo for proximity notify touch

    }
    else
    {
        if(unlikely(debug))printk( "[%s]	PS_CurrentStatus Repeat !! \n",__FUNCTION__);
    }

    return 0;
}

static irqreturn_t ap3212c_interrupt_handler(int irq, void *dev_id)
{
    /*
    	struct  ap3212c_i2c_data * ap3212c_data = data;
    	//if(unlikely(debug))printk( "[%s]		ap3212c_interrupt_handler\n",__FUNCTION__); // add for test

    	wake_lock(&ap3212c_data->wake_lock);

    	//udelay(500);
    	ap3212c_data->interrupt_flag = i2c_smbus_read_byte_data(ap3212c_i2c_client, INT_STATUS);
    	//if(unlikely(debug))printk( "[%s]		INT_STATUS = 0x%x \n",__FUNCTION__,ap3212c_data->interrupt_flag & 0x03);

    	wake_unlock(&ap3212c_data->wake_lock);

    	if(ap3212c_data->interrupt_flag & 0x03){
    		interruptStatus = ap3212c_data->interrupt_flag & 0x03;
    		queue_delayed_work(ap3212c_wq, &ap3212c_work,0);
    		disable_irq_nosync(irq);
    		//if(unlikely(debug))printk( "[%s]		interruptStatus =0x%x \n",__FUNCTION__,interruptStatus); // add for test
    	}
    */
    disable_irq_nosync(irq);
    queue_delayed_work(ap3212c_wq, &ap3212c_work,0);
    return IRQ_HANDLED;
}

//ap3212c_work_function: 1.find out which sensor occured the interrupt	2.read ALS or PS DATA 3.enable irq
static void ap3212c_work_function(struct work_struct *work)
{
    int err=0;
    unsigned int value_LByte=0, value_HByte=0;
    unsigned long ADC_Data=0, RAW_Data=0;
    struct ap3212c_i2c_data *ap3212c_data=data;

    wake_lock(&ap3212c_data->wake_lock);

    if(!interruptStatus)
    {
        ap3212c_data->interrupt_flag = i2c_smbus_read_byte_data(ap3212c_i2c_client, INT_STATUS);
        if(!(ap3212c_data->interrupt_flag & 0x03))
        {
            if(unlikely(debug))printk( "[%s]		INT_STATUS = 0x%x \n",__FUNCTION__,ap3212c_data->interrupt_flag & 0x03);
            goto ap3212c_work_function_finished;
        }
        interruptStatus = ap3212c_data->interrupt_flag & 0x03;
    }

//add by leo for proximity cross talk calibration ++
    if((ap3212c_data->ps_power_state==POWER_ON)&&(PS_AlreadyVendorCalibration==FALSE)&&(PS_VendorCalibrationRetryCount>0))
    {
        err = ap3212c_vendor_calibration_proximity(ap3212c_data);
        if(err<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]	PS Vendor Calibration  (get cross talk value) Failed (%d)\n",__FUNCTION__,err);
        }
    }
//add by leo for proximity cross talk calibration --

//add by leo for calibration ++
    if((ap3212c_data->als_power_state==POWER_ON)&&(ALS_AlreadyCalibration==FALSE)&&(ALS_CalibrationRetryCount>0))
    {
        err=ap3212c_calibration_lightsensor(ap3212c_data);
        if(err<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		ALS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,err);
        }
    }

    if((ap3212c_data->ps_power_state==POWER_ON)&&(PS_AlreadyCalibration==FALSE)&&(PS_CalibrationRetryCount>0))
    {
        err=ap3212c_calibration_proximity(ap3212c_data);
        if(err<0)
        {
            if(unlikely(debug))printk( "[%s]			PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,err);
        }
    }
//add by leo for calibration --

    if(interruptStatus & 0x01)
    {

        value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_LOW);
        value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, ALS_DATA_HIGH);
        if(value_LByte<0||value_HByte<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		Read ALS_DATA Failed \n",__FUNCTION__);
        }
        ADC_Data = ((value_HByte << 8) | value_LByte);
        //if(AP3212C_STATUSMSG) printk( "[%s]	ALS_DATA = %d (ADC)\n",__FUNCTION__,ADC_Data);

        ap3212c_data->als_original_adcvalue=(int)ADC_Data;
        if(unlikely(debug))printk( "[%s]			ap3212c_data->als_original_adcvalue = %d (ADC)\n",__FUNCTION__,ap3212c_data->als_original_adcvalue);
        ap3212c_data->als_original_luxvalue=(int)ADC_Data*625/10000;
        if(unlikely(debug))printk( "[%s]			ap3212c_data->als_original_luxvalue = %d (Lux)\n",__FUNCTION__,ap3212c_data->als_original_luxvalue);

        ap3212c_data->als_adcvalue = ADC_Data*ALS_CalibrationValue_standard/ALS_CalibrationValue_real;
        if(unlikely(debug))printk( "[%s]			ap3212c_data->als_adcvalue = %d (ADC) \n",__FUNCTION__,ap3212c_data->als_adcvalue);
        ap3212c_data->als_luxvalue = ADC_Data*625/10000*ALS_CalibrationValue_standard/ALS_CalibrationValue_real; //Resolution = 0.0625 lux/count
        if(unlikely(debug))printk( "[%s]			ap3212c_data->als_luxvalue = %d (Lux)\n",__FUNCTION__,ap3212c_data->als_luxvalue);

        als_work_function(ap3212c_data);
    }

    if(interruptStatus & 0x02)
    {

        value_LByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_LOW);
        value_HByte = i2c_smbus_read_byte_data(ap3212c_i2c_client, PS_DATA_HIGH);
        if(value_LByte<0||value_HByte<0)
        {
            if(AP3212C_STATUSMSG) printk( "[%s]		Read PS_DATA Failed \n",__FUNCTION__);
        }
        RAW_Data = (((value_HByte & 0x3f) << 4) | (value_LByte & 0x0f)); //0x3f = 0011 1111; 0x0f = 0000 1111
        if(AP3212C_STATUSMSG) printk( "[%s]		PS DATA = %d\n",__FUNCTION__,(int)RAW_Data);

        ap3212c_data->ps_status = (value_HByte & 0x80)>>7;
        if(unlikely(debug))printk( "[%s]			PS STATUS = %d\n",__FUNCTION__,ap3212c_data->ps_status);
        ps_work_function(ap3212c_data);
    }

ap3212c_work_function_finished:

    interruptStatus=0;
    enable_irq(ap3212c_data->irq);

    wake_unlock(&ap3212c_data->wake_lock);
    return;
}
//==========================================================================================
static int ap3212c_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret=0;
    struct ap3212c_platform_data *pdata;
    struct ap3212c_i2c_data *ap3212c_data=NULL;

    if(AP3212C_STATUSMSG) printk("====================== \n");
    if(AP3212C_STATUSMSG) printk( "[%s]		build version = %d\n",__FUNCTION__,build_version); //add by leo for read build version

    if(!(ap3212c_data = kzalloc(sizeof(struct ap3212c_i2c_data), GFP_KERNEL)))
    {
        if(build_version!=1)return -ENOMEM;
    }
    memset(ap3212c_data, 0, sizeof(struct ap3212c_i2c_data));

    ap3212c_data->i2c_client = client;
    i2c_set_clientdata(client, ap3212c_data);
    ap3212c_i2c_client = ap3212c_data->i2c_client;
    ap3212c_i2c_client->flags = 1;
    strlcpy(ap3212c_i2c_client->name, "ap3212c", I2C_NAME_SIZE);

    data = ap3212c_data;

    wake_lock_init(&ap3212c_data->wake_lock, WAKE_LOCK_SUSPEND, "ap3212c_wake_lock");

    /* Check Device */
    ret = ap3212c_check_device(ap3212c_data);
    if (ret<0)
    {
        if(AP3212C_STATUSMSG) printk("[%s]		ap3212c_i2c_probe Failed\n",__FUNCTION__);
        if(build_version!=1)goto ap3212c_check_device_err;
    }
    /* Interrupt Setting */
    pdata = client->dev.platform_data;
    AP3212C_INTERRUPT_GPIO = pdata->gpio;

    /* Workqueue Setting */
    ap3212c_wq = create_singlethread_workqueue("ap3212c_wq");
    if (!ap3212c_wq)
    {
        if(AP3212C_STATUSMSG) printk("[%s]	Create WorkQueue Failed\n",__FUNCTION__);
        ret = -ENOMEM;
        if(build_version!=1)goto create_singlethread_workqueue_err;
    }
    INIT_DELAYED_WORK_DEFERRABLE(&ap3212c_work, ap3212c_work_function);

    /* Initialize Setting */
    ap3212c_create_proc_file(); // add by leo for proc file ++
    als_setup(ap3212c_data);
    ps_setup(ap3212c_data);

    ap3212c_setup_irq(ap3212c_data);

    als_initial(ap3212c_data);
    ps_initial(ap3212c_data);

    /* sysfs Setting */
    ret = sysfs_create_group(&client->dev.kobj, &ap3212c_attr_group);
    if (ret)
    {
        if(AP3212C_STATUSMSG) printk("[%s] Register sysfs Failed\n",__FUNCTION__);
        if(build_version!=1)goto sysfs_create_group_err;
    }

//Add by leo for early_suspend ++
    ap3212c_data->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 5;
    ap3212c_data->early_suspend.suspend = ap3212c_early_suspend;
    ap3212c_data->early_suspend.resume =  ap3212c_later_resume;
    register_early_suspend(&ap3212c_data->early_suspend);
//Add by leo for early_suspend --

    enable_irq(ap3212c_data->irq);
    if(AP3212C_STATUSMSG) printk("====================== \n");
    return 0;

sysfs_create_group_err:
    ap3212c_remove_proc_file(); // add by leo for proc file ++
create_singlethread_workqueue_err:
ap3212c_check_device_err:
    wake_lock_destroy(&ap3212c_data->wake_lock);
    kfree(ap3212c_data);
    return ret;
}

//Add by leo for early_suspend ++
static void ap3212c_early_suspend(struct early_suspend *h)
{
// mask by leo because system may disable light sensor by ioctl from framework 	++
    /*
    	int ret=0;
    	struct ap3212c_i2c_data *ap3212c_data = data;

    	ALS_StatusBeforeSuspend = ap3212c_data->als_power_state;
    	if(unlikely(debug))printk( "[%s]	als_power_state = %d (Before Early Suspend)\n", __FUNCTION__,ALS_StatusBeforeSuspend);

    	ret = als_enable(ap3212c_data,POWER_OFF);
    	if(ret < 0)  {
    		if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,POWER_OFF);
    	}
    */
// mask by leo because system may disable light sensor by ioctl from framework 	--
    if(AP3212C_STATUSMSG) printk( "[%s]\n", __FUNCTION__);
    return;
}
static void ap3212c_later_resume(struct early_suspend *h)
{
// mask by leo because system may disable light sensor by ioctl from framework 	++
    /*
    	int ret=0;
    	struct ap3212c_i2c_data *ap3212c_data = data;

    	if(unlikely(debug))printk( "[%s]	als_power_state = %d (Before Early Suspend)\n", __FUNCTION__,ALS_StatusBeforeSuspend); // add for test
    	ret = als_enable(ap3212c_data,ALS_StatusBeforeSuspend);
    	if(ret < 0)  {
    		if(AP3212C_STATUSMSG) printk( "[%s]	als_enable(%d) Failed \n",__FUNCTION__,ALS_StatusBeforeSuspend);
    	}
    */
// mask by leo because system may disable light sensor by ioctl from framework 	--
    if(AP3212C_STATUSMSG) printk( "[%s]\n", __FUNCTION__);
    return;
}
//Add by leo for early_suspend --

static int ap3212c_suspend(struct i2c_client *client , pm_message_t mesg)
{
    //struct ap3212c_i2c_data *ap3212c_data = data;

    if(AP3212C_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
    //enable_irq_wake(ap3212c_data->irq); //mark by leo for proximity wake up source
    return 0;
}

static int ap3212c_resume(struct i2c_client *client)
{
    //struct ap3212c_i2c_data *ap3212c_data = data;

    if(AP3212C_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
    //disable_irq_wake(ap3212c_data->irq); //mark by leo for proximity wake up source
    return 0;
}

static int __devexit ap3212c_i2c_remove(struct i2c_client *client)
{
    struct ap3212c_i2c_data *ap3212c_data =data;

    if(AP3212C_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
    ap3212c_remove_proc_file(); // add by leo for proc file ++
    sysfs_remove_group(&client->dev.kobj, &ap3212c_attr_group);
    ap3212c_data = i2c_get_clientdata(client);
    ap3212c_i2c_client = NULL;
    unregister_early_suspend(&ap3212c_data->early_suspend); //add by leo for early suspend ++
    kfree(ap3212c_data);
    return 0;
}

static const struct i2c_device_id ap3212c_i2c_idtable[] =
{
    {"ap3212c", 0},
    {}
};

MODULE_DEVICE_TABLE(i2c, ap3212c_i2c_idtable);

static struct i2c_driver ap3212c_i2c_driver =
{

    .driver = {
        .name = AP3212C_I2C_DEVICE_NAME,
        .owner = THIS_MODULE,
    },
    .probe = 		ap3212c_i2c_probe,
    .suspend	=	ap3212c_suspend,
    .resume	=	ap3212c_resume,
    .remove	=	__devexit_p(ap3212c_i2c_remove),
    .id_table = 	ap3212c_i2c_idtable,

};

static int __init ap3212c_init(void)
{
    int ret=0;

	if(build_version!=1){
	    if(Read_PROJ_ID() == PROJ_ID_ME372CG_CD)
			return 0;
    	if(Read_PROJ_ID() == PROJ_ID_ME372CL)
	    {
    	    if((Read_HW_ID() == HW_ID_SR1) || (Read_HW_ID() == HW_ID_SR2))
	        {
            	printk(" ME372CL function don't work now!!\n",__FUNCTION__);
        	    return 0;
    	    }
	    }
	}
    ret = i2c_add_driver(&ap3212c_i2c_driver);
    if ( ret != 0 )
    {
        if(AP3212C_STATUSMSG) printk("[%s]	i2c_add_driver Failed (ret = %d) \n",__FUNCTION__,ret);
        return ret;
    }
    else
    {
        if(unlikely(debug))printk("[%s]	ap3212c_init success !!\n",__FUNCTION__);
        return ret;
    }
}

static void __exit ap3212c_exit(void)
{
    i2c_del_driver(&ap3212c_i2c_driver);
}

module_init(ap3212c_init);
module_exit(ap3212c_exit);

MODULE_AUTHOR("Asus Tek. <asus@asus.com>");
MODULE_DESCRIPTION("CAPELLA AP3212C Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

