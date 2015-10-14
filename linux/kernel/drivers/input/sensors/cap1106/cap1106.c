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
#include <linux/gpio_event.h>
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

#include <linux/proc_fs.h>//add by leo for proc file ++
#include <linux/i2c/cap1106.h>// add by leo for cap1106_platfor_data 
// add by alp for proc file ++
#include <linux/seq_file.h>
// add by alp for proc file --


//add by leo for read hw id ++
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
//add by leo for read hw id --

/*Check System Mode*/
extern int entry_mode;//add by leo for charge mode (entry_mode==4)++

//add by leo for read build version ++
/*
 *build_vsersion mean TARGET_BUILD_VARIANT
 *eng:1
 *userdebug:2
 *user:3
 */
extern int build_version;
//add by leo for read build version --

#define CAP1106_I2C_DEVICE_NAME "cap1106"
#define DRIVER_VERSION		"1.0.0.0"
//#define CAP1106_INTERRUPT_GPIO	46		//H:N/A, L:Interrupt
static int  CAP1106_INTERRUPT_GPIO=0;	//78

struct cap1106_i2c_data{
	struct input_dev *cs_input_dev;
	struct i2c_client *i2c_client;
	//struct early_suspend early_suspend;
	u8 id;
	int cs_power_state;
	int irq;
	int cs_sensitivity;
	int cs1_data;
	int cs6_data;
	int cs_status;
	int cs1_rf_noise;
	int cs6_rf_noise;
};
static struct cap1106_i2c_data *data=NULL;

//add by josh for second sensitivity ++++++
struct cap1106_config_data {
	u8 cs_sensitivity_1;
	u8 cs_gain_1;
	u8 cs1_thd_1;
	u8 cs6_thd_1;
	u8 cs_sensitivity_2;
	u8 cs_gain_2;
	u8 cs1_thd_2;
	u8 cs6_thd_2;
	int overflow_status;
};
static struct cap1106_config_data *C_data=NULL; 
//add by josh for second sensitivity ------

static struct i2c_client *cap1106_i2c_client = NULL;

/*Registers of Cap Sensor*/
#define PRODUCT_ID 0xFD
#define SENSOR_INPUT_ENABLE 0x21
#define SENSITIVITY_CONTROL 0x1F	// full active mode
#define REPEAT_RATE_ENABLE 0x28
#define GENERAL_STATUS 0x02
#define SENSOR_INPUT_STATUS 0x03
#define MAIN_CONTROL 0x00
#define SENSOR_INPUT1_DELTA 0x10
#define SENSOR_INPUT2_DELTA 0x11
#define SENSOR_INPUT6_DELTA 0x15
#define SENSOR_INPUT1_THD 0x30
#define SENSOR_INPUT2_THD 0x31
#define SENSOR_INPUT6_THD 0x35
#define AVG_AND_SAMP_CONFIG 0x24	// full active mode
#define NOISE_FLAG_STATUS 0x0A
#define CALIBRATION_ACTIVATE 0x26
#define CONFIGRURATION2 0x44
#define RECALIBRATION_CONFIG 0x2F
#define SENSOR_INPUT_NOISE_THD 0x38
#define CONFIGRURATION1 0x20
#define SENSOR_INPUT_CONFIG 0x22
#define MULTI_TOUCH_BLOCK_CONFIG 0x2A

//add by leo for standby mode ++
#define STANDBY_CHANNEL		0x40	// standby mode
#define STANDBY_CONFIG			0x41	// standby mode
#define STANDBY_SENSITIVITY	0x42	// standby mode
#define STANDBY_THRESHOLD		0x43	// standby mode
//add by leo for standby mode --

//#define RF_NOISE_DETECT

typedef enum {
	NOT_SENSED = 0,
	SENSED,
	UNKNOW
}sensor_status;
static sensor_status    CS_OldStatus = UNKNOW;
static sensor_status    CS_Status = UNKNOW;

#define CLEAN_DSLEEP		0xEF	//0xEF = 1110 1111, DSLEEP = B4 (AND)
#define SET_DSLEEP			0x10	//0x10 = 0001 0000, DSLEEP = B4 (OR)
#define CLEAN_STBY			0xDF	//0x00 = 1101 1111
#define SET_STBY			0x20	//0x20 = 0010 0000

#define DOUBLE_SENSITIVITY
#define HIGH_SENSITIVITY	0
#define LOW_SENSITIVITY		1

/* Sensors Interrupt Workqueue */
static struct workqueue_struct *cap1106_wq=NULL;
static struct delayed_work cap1106_work;
#ifdef RF_NOISE_DETECT
	static struct delayed_work rf_noise_detect_work; // add by leo for rf noise
#endif
static void cap1106_work_function(struct work_struct *work);

#define ABS_SENSED ABS_HAT2X

//Linux signal ipc
static int notify_daemon_pid = 0;
static int cap_sensor_not_sensed_signal = 16;
static int cap_sensor_sensed_signal = 17;
static int check_table_list_signal = 18;
/*
static int sar_rf_state_default_signal = 50;
//static int sar_rf_state_1_signal = 51;
static int sar_rf_state_2_signal = 52;
static int sar_rf_state_3_signal = 53;
static int sar_rf_state_4_signal = 54;
static int sar_rf_state_5_signal = 55;
static int sar_rf_state_6_signal = 56;
static int sar_rf_state_7_signal = 57;
static int sar_rf_state_8_signal = 58;
static int cap_sensor_always_on_signal = 59;
static int cap_sensor_always_off_signal = 60;
*/

/* others */
static int isDaemonReady = 0;	// add by leo for system crash issue ++
static int isSendSignalSuccess = 0;	// add by leo for modem reset ++
static int isCapSensorExist = 0; // add by leo to detect cap sensor chip
static int isCheckDeviceOk = 0; // add by leo to detect cap sensor chip
static int isStandbyMode = 0;

static int status_stable_count = 0; // status stable count 
static int SensitivityMode = HIGH_SENSITIVITY;
static int work_count = 0;

#if RF_NOISE_DETECT
	static int old_rf_noise_flag=0;
#endif

#define CAPSENSOR_IOCTL_MAGIC 'c'
#define CAPSENSOR_IOCTL_GET_ENABLED	_IOR(CAPSENSOR_IOCTL_MAGIC, 1, int *) 
#define CAPSENSOR_IOCTL_ENABLE			_IOW(CAPSENSOR_IOCTL_MAGIC, 2, int *) 

static int reset_cap1106_interrupt(void);
static irqreturn_t cap1106_interrupt_handler(int irq, void *dev_id);
static int cap1106_send_signal (sensor_status s);
static void cap1106_change_sensitivity(int mode);
static void cap1106_get_data(viod);
static int rf_noise_check(void);

// add by leo for proc file ++
#define	CAP1106_PROC_FILE	"cap1106"
static struct proc_dir_entry *cap1106_proc_file;
// add by leo for proc file --

#define CS_DEBUGMSG 	0
#define CS_STATUSMSG 	1
static unsigned int debug = 0;

//=========================================================================================
static int cap1106_check_device(struct cap1106_i2c_data *cap1106_data)
{
	u8 whoami=0;
	
	whoami = i2c_smbus_read_byte_data(cap1106_i2c_client, PRODUCT_ID);
	if (whoami < 0){
		if(CS_STATUSMSG) printk( "[%s]		Read ID Register Failed \n",__FUNCTION__);
		return whoami;
	}
	
	if(whoami!=0x55){
		if(CS_STATUSMSG) printk( "[%s]		Check Device Failed \n",__FUNCTION__);
		if(CS_STATUSMSG) printk( "[%s]		Who am I ??? (PRODUCT ID = 0x%x) \n",__FUNCTION__,whoami);
		return -ENODEV;
	}
	cap1106_data->id= whoami;
	if(CS_STATUSMSG) printk( "[%s]		Check Device OK !! PRODUCT ID = 0x%x \n",__FUNCTION__,cap1106_data->id);

	isCheckDeviceOk=1; // add by leo to detect cap sensor chip
	return 0;
}

static int cap1106_setup_irq(struct cap1106_i2c_data *cap1106_data)
{
	int err = -EINVAL;
	
	/*gpio setting*/
	err = gpio_request(CAP1106_INTERRUPT_GPIO, CAP1106_I2C_DEVICE_NAME);	
	if(err<0){
		if(CS_STATUSMSG) printk("[%s]		gpio_request Failed\n",__FUNCTION__);
		goto exit;
	}

	err = gpio_direction_input(CAP1106_INTERRUPT_GPIO) ;
	if(err<0){
		if(CS_STATUSMSG) printk("[%s]		gpio_direction_input Failed\n",__FUNCTION__);
		goto gpio_direction_input_err;
	}

	//gpio_tlmm_config(GPIO_CFG(CAP1106_INTERRUPT_GPIO, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), 1);
	
	cap1106_data->irq = gpio_to_irq(CAP1106_INTERRUPT_GPIO);
	if(cap1106_data->irq<0){
		if(CS_STATUSMSG) printk("[%s]	gpio_direction_input Failed\n",__FUNCTION__);
		goto gpio_to_irq_err;
	}
	
	if(cap1106_data->irq > 0){
		
		err = request_irq(cap1106_data->irq, cap1106_interrupt_handler, IRQF_TRIGGER_FALLING, CAP1106_I2C_DEVICE_NAME, cap1106_data->i2c_client);
		if (err < 0){
			if(CS_STATUSMSG) printk("[%s]		request_irq Failed, irq = %d, err = %d\n",__FUNCTION__, cap1106_data->irq, err);
			err = -EIO;
			goto request_irq_err;
		}
	}
/*
	err = request_threaded_irq(cap1106_data->irq, NULL, cap1106_interrupt_handler, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,"cap1106_interrupt",NULL);
	if (err) {
		dev_err(&cap1106_i2c_client->adapter->dev,"Can NOT Register IRQ : %d , err:%d\n", cap1106_data->irq, err);
		err = -EIO;
		goto request_irq_err;
	}
*/
	if(CS_STATUSMSG) printk("[%s]		CAP1106 interrupt gpio = %d , value = %d\n",__FUNCTION__, CAP1106_INTERRUPT_GPIO, cap1106_data->irq);
	return 0;
	
request_irq_err:
gpio_to_irq_err:
gpio_direction_input_err:	
	gpio_free(CAP1106_INTERRUPT_GPIO);
exit:	
	return err;
}

// add by leo for proc file ++
static ssize_t cap1106_register_read(struct seq_file *buf, void *v )
{
	ssize_t sprintf_count = 0;
	debug = debug?0:1; //switch debug message
	seq_printf(buf,"cap1106 debug = %d \n",debug);
	sprintf_count += sprintf("debug meaasge \n", ((debug==1)?"on":"off"));
	return sprintf_count;
}
static ssize_t cap1106_register_write(struct file *filp, const char __user *buff, unsigned long len, void *data)
{
	char messages[80] = {0};
	int en=0, cmd=0, ret=0, count=0, iloop=0, i=0;
	u32 reg_address=0, reg_value=0;
	static u8 temp[4]={0};
	static u8 total_input[80]={0};

	if (len >= 80) {
		if(CS_STATUSMSG) printk(KERN_INFO "%s: no command exceeds 80 chars.\n", __func__);
		return -EFAULT;
	}
		
	if (copy_from_user(messages, buff, len))
		return -EFAULT;
	
	if ((messages[0]=='l')&&(messages[1]=='o')&&(messages[2]=='g')){
		if(&messages[4]!=NULL)
			sscanf (&messages[4], "%d", &en);
	
		if(CS_STATUSMSG) printk( "[%s]	en = %d\n",__FUNCTION__,en);
		debug=(en==1)?1:0;
		if(CS_STATUSMSG) printk( "[%s]	debug meaasge (%s)\n",__FUNCTION__,(debug==1)?"on":"off");
		return len;

	}else if(messages[0]=='w'){
		cmd=1;
	}else if(messages[0]=='r'){
		cmd=2;
	}else{
		if(CS_STATUSMSG) printk("[%s]	Unknow Command\n",__FUNCTION__);
		return len;
	}

	// count input command
	for(i=0;i<100;i++){
		if(messages[i]=='x'){
			count ++;
		}else if(messages[i]=='\n'){
			if((debug))printk("[%s]	command number = %d\n",__FUNCTION__,count);
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
	u8* input_cmd = (char*)kcalloc(count,sizeof(char),GFP_KERNEL);

	if(CS_STATUSMSG) printk("[%s]	input_cmd = %s ",__FUNCTION__,cmd==1?"write":"read");
	for(i=0;i<count;i++){
		input_cmd[i]=total_input[i]&0xff;
		if(CS_STATUSMSG) printk("0x%02x ", input_cmd[i]);
	}
	if(CS_STATUSMSG) printk("\n");

	// send test command
	reg_address = input_cmd[0];
	reg_value = input_cmd[1];

	if(cmd==1){ //write
		ret = i2c_smbus_write_byte_data(cap1106_i2c_client, reg_address, reg_value);
		if(ret< 0)  {
			if(CS_STATUSMSG) printk( "[%s]	Write 0x%02X into 0x%02X failed (%d)\n",__FUNCTION__,reg_value,reg_address,ret);
			return ret;
		}
		if(CS_STATUSMSG) printk( "[%s]	write 0x%02X into 0x%02X\n",__FUNCTION__,reg_value,reg_address);
		
	}else if(cmd==2){ //read
		ret = i2c_smbus_read_byte_data(cap1106_i2c_client, reg_address);
		if(ret< 0)  {
			if(CS_STATUSMSG) printk( "[%s]	Read data from 0x%02X failed (%d)\n",__FUNCTION__,reg_address,ret);
			return ret;
		}
		if(CS_STATUSMSG) printk( "[%s]	0x%02x value is 0x%02x \n",__FUNCTION__,reg_address,ret);
		reg_value=ret;
	}
	
	return len;
}

// add by alp for proc file ++
static int cap1106_register_open(struct inode *inode, struct file *file)
{
	return single_open(file,cap1106_register_read,NULL);
}

static const struct file_operations cap1106_register_fops = {
	.owner = THIS_MODULE,
	.open = cap1106_register_open,
	.read = seq_read,
	.write = cap1106_register_write,
};

void cap1106_create_proc_file(void)
{
    cap1106_proc_file = proc_create(CAP1106_PROC_FILE, 0666, NULL,&cap1106_register_fops);
    if(!cap1106_proc_file)
    {
        printk( "[%s]	create_proc_entry failed!!! \n",__FUNCTION__);
    }
}
// add by alp for proc file --
/*
void cap1106_create_proc_file(void)
{
	cap1106_proc_file = create_proc_entry(CAP1106_PROC_FILE, 0666, NULL);
	if(cap1106_proc_file){
		cap1106_proc_file->read_proc = cap1106_register_read;
		cap1106_proc_file->write_proc = cap1106_register_write;
	}
	else{
		if(CS_STATUSMSG) printk( "[%s]	create_proc_entry failed\n",__FUNCTION__);
	}
}
*/
void cap1106_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    if((debug))printk( "[%s] remove proc files \n",__FUNCTION__);
    remove_proc_entry(CAP1106_PROC_FILE, &proc_root);
}
// add by leo for proc file --
//========================================================================================
static ssize_t CapSensor_check_For_ATD_test(struct device *dev, struct device_attribute *attr, char *buf)
{
	int TOUCH=0,CS_GeneralStatus=0;
	
	CS_GeneralStatus = i2c_smbus_read_byte_data(cap1106_i2c_client, GENERAL_STATUS);
	if (CS_GeneralStatus  < 0)
	{	
		if(CS_STATUSMSG) printk( "[%s]		CS Read GENERAL_STATUS Failed\n",__FUNCTION__);
		return sprintf(buf,"Read GENERAL_STATUS Failed\n");
	}
	if((debug)) printk( "[%s]		CS GENERAL_STATUS = 0x%x\n",__FUNCTION__,CS_GeneralStatus);

	reset_cap1106_interrupt();
	
	TOUCH = (CS_GeneralStatus & 0x01); // 0000 0001
	if(TOUCH == 1)  {
		return sprintf(buf,"1\n");
	}
	return sprintf(buf,"0\n");
}
static DEVICE_ATTR(cap_status, S_IRUGO,CapSensor_check_For_ATD_test,NULL);

// add by leo to detect cap sensor chip ++
static ssize_t cs_exist_check(struct device *dev, struct devices_attribute *attr, char *buf)
{
	if(CS_STATUSMSG) printk("[%s]		isCapSensorExist = %d\n",__FUNCTION__,isCapSensorExist);

	if(isCapSensorExist==0){
		return sprintf(buf,"0\n");
	}else{
		return sprintf(buf,"1\n");
	}
}
static DEVICE_ATTR(capsensor_exist_check, S_IRUGO,cs_exist_check,NULL);

static ssize_t cs_status_check(struct device *dev, struct devices_attribute *attr, char *buf)
{
	if(CS_STATUSMSG) printk("[%s]		isCheckDeviceOk = %d\n",__FUNCTION__,isCheckDeviceOk);

	if(isCheckDeviceOk==0){
		return sprintf(buf,"0\n");
	}else{
		return sprintf(buf,"1\n");
	}
}
static DEVICE_ATTR(capsensor_status_check, S_IRUGO,cs_status_check,NULL);

static ssize_t cs_check(struct device *dev, struct devices_attribute *attr, char *buf)
{
	if(CS_STATUSMSG) printk("[%s]		isCheckDeviceOk = %d, isCapSensorExist = %d\n",__FUNCTION__,isCheckDeviceOk, isCapSensorExist);
	
	if(isCheckDeviceOk==1){
		if(CS_STATUSMSG) printk("[%s]		cap sensor ok\n",__FUNCTION__);
		return sprintf(buf,"cap sensor ok\n");
		
	}else if((isCheckDeviceOk==0)&&(isCapSensorExist==0)){
		if(CS_STATUSMSG) printk("[%s]		no cap sensor\n",__FUNCTION__);
		return sprintf(buf,"no cap sensor\n");
		
	}else if((isCheckDeviceOk==0)&&(isCapSensorExist!=0)){
		if(CS_STATUSMSG) printk("[%s]		cap sensor check device fail\n",__FUNCTION__);
		return sprintf(buf,"cap sensor check device fail\n");
	}


	if(CS_STATUSMSG) printk("[%s]		cap sensor unknow status\n",__FUNCTION__);
	return sprintf(buf,"cap sensor unknow status\n");
}
static DEVICE_ATTR(cap_check, S_IRUGO,cs_check,NULL);
// add by leo to detect cap sensor chip --

static ssize_t cs_get_id(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 whoami=0;
	
	whoami = i2c_smbus_read_byte_data(cap1106_i2c_client, PRODUCT_ID);
	if (whoami < 0)
	{	
		if(CS_STATUSMSG) printk( "[%s]		CS Read ID Register Failed\n",__FUNCTION__);
		return sprintf(buf,"Read ID Register Failed\n");
	}
	return sprintf(buf,"Cap1106 ID = 0x%x\n",whoami);
}
static DEVICE_ATTR(cap1106_id, S_IRUGO,cs_get_id,NULL);


static ssize_t cs_chip_info(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	u8 cs_info[3]={0}; //product_id, manufacturer_id, revision
	
	ret = i2c_smbus_read_i2c_block_data(cap1106_i2c_client, PRODUCT_ID, 3, cs_info);	//get chip info
	if (ret  < 0)
	{	
		if(CS_STATUSMSG) printk( "[%s]		CS Read PRODUCT_ID Failed\n",__FUNCTION__);
		return sprintf(buf,"Read CS Chip Info Failed\n");
	}
	return sprintf(buf,"Model Name: %s, Product ID = 0x%x, Manufacturer ID = 0x%x, Revision = 0x%x \n", CAP1106_I2C_DEVICE_NAME, cs_info[0], cs_info[1], cs_info[2]);
}
static DEVICE_ATTR(cs_info, S_IRUGO,cs_chip_info,NULL);

static ssize_t cs_report_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"CS Report (%s)\n",CS_OldStatus==SENSED?"1":"0");
}
static DEVICE_ATTR(cs_report, S_IRUGO,cs_report_status,NULL);

static ssize_t cs_get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 SensorsStatus=0;
	
	SensorsStatus = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
	if (SensorsStatus < 0)
	{	
		if(CS_STATUSMSG) printk( "[%s]		CS Read SENSOR_INPUT_STATUS Failed\n",__FUNCTION__);
		return sprintf(buf,"Read Status Registers Failed\n");
	}
	reset_cap1106_interrupt();
		
	return sprintf(buf,"CS Input Status = 0x%x\n",SensorsStatus);
}
static DEVICE_ATTR(cs_status, S_IRUGO,cs_get_status,NULL);


static ssize_t cs_noise_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 NoiseFlag=0;
	
	NoiseFlag = i2c_smbus_read_byte_data(cap1106_i2c_client, NOISE_FLAG_STATUS);
	if (NoiseFlag < 0)
	{	
		if(CS_STATUSMSG) printk( "[%s]		CS Read NOISE_FLAG_STATUS Failed\n",__FUNCTION__);
		return sprintf(buf,"Read Noise Flag Status Registers Failed\n");
	}
	
	return sprintf(buf,"CS Noise Flag Status = 0x%x\n",NoiseFlag);
}
static DEVICE_ATTR(cs_noise, S_IRUGO,cs_noise_flag,NULL);

static ssize_t cs_enable_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE);
	if (ret < 0){
		return sprintf(buf, "Read SENSOR_INPUT_ENABLE Failed\n");
	}
	return sprintf(buf, "Sensor Input Enable Register = 0x%x\n", ret);
};
static ssize_t cs_enable_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		Sensor Input Enable Register 0x%x Enable Failed (%d)\n",__FUNCTION__,regvalue,ret);
		return ret;
	}

	return count;
};
static DEVICE_ATTR(cs_en, S_IWUSR | S_IRUGO, cs_enable_show, cs_enable_store);

//add by leo for standby mode ++
static ssize_t cs_standby_enable_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CHANNEL);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_CHANNEL Failed\n");
	}
	return sprintf(buf, "Standby Channel Register = 0x%x\n", ret);
};
static ssize_t cs_standby_enable_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		Standby Channel Register 0x%x Enable Failed (%d)\n",__FUNCTION__,regvalue,ret);
		return ret;
	}

	return count;
};
static DEVICE_ATTR(cs_stby_en, S_IWUSR | S_IRUGO, cs_standby_enable_show, cs_standby_enable_store);
//add by leo for standby mode --

static ssize_t cs_cycle_time_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 regvalue =0, AVG=0, SAMP_TIME=0, CYCLE_TIME=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG);
	AVG= (regvalue & 0x70)>>4; //0x70 = 0111 0000
	SAMP_TIME= (regvalue & 0x0c)>>2; //0x0c = 0000 1100
	CYCLE_TIME= (regvalue & 0x03); //0x03 = 0000 0011
	if(CS_STATUSMSG) printk( "[%s]		Averaging and Sampling Configuration Register = 0x%x, AVG = 0x%x, SAMP_TIME = 0x%x, CYCLE_TIME = 0x%x\n",__FUNCTION__,regvalue, AVG, SAMP_TIME, CYCLE_TIME);

	return sprintf(buf, "Averaging and Sampling Configuration = 0x%x, CYCLE_TIME = 0x%x\n", regvalue, CYCLE_TIME);
};
static ssize_t cs_cycle_time_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0, CYCLE_TIME=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	CYCLE_TIME = ((u8) temp) & 0x3;// 0x03 = 0000 0011

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG);
	regvalue = (regvalue & 0xfc) | CYCLE_TIME;	//0xfc = 11111100

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write AVG_AND_SAMP_CONFIG Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new CYCLE_TIME is 0x%x \n",__FUNCTION__,CYCLE_TIME);

	return count;
};
static DEVICE_ATTR(cs_cycle, S_IWUSR | S_IRUGO, cs_cycle_time_show, cs_cycle_time_store);

//add by leo for standby mode ++
static ssize_t cs_stby_cycle_time_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 regvalue =0, AVG_SUM=0,STBY_AVG=0, STBY_SAMP_TIME=0, STBY_CYCLE_TIME=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CONFIG);
	AVG_SUM = regvalue >>7;
	STBY_AVG= (regvalue & 0x70)>>4; //0x70 = 0111 0000
	STBY_SAMP_TIME= (regvalue & 0x0c)>>2; //0x0c = 0000 1100
	STBY_CYCLE_TIME= (regvalue & 0x03); //0x03 = 0000 0011
	if(CS_STATUSMSG) printk( "[%s]		AVG_SUM = 0x%x, STBY_AVG = %x, STBY_SAMP_TIME = 0x%x, STBY_CYCLE_TIME = 0x%x\n",__FUNCTION__, AVG_SUM, STBY_AVG, STBY_SAMP_TIME, STBY_CYCLE_TIME);
	
	return sprintf(buf, "Standby Configuration Register = 0x%x, STBY_CYCLE_TIME = 0x%x\n", regvalue, STBY_CYCLE_TIME);
};
static ssize_t cs_stby_cycle_time_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0, STBY_CYCLE_TIME=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	STBY_CYCLE_TIME = ((u8) temp) & 0x3;// 0x03 = 0000 0011

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CONFIG);
	regvalue = (regvalue & 0xfc) | STBY_CYCLE_TIME;	//0xfc = 11111100

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CONFIG, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write STANDBY_SENSITIVITY Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new STBY_CYCLE_TIME is 0x%x \n",__FUNCTION__,STBY_CYCLE_TIME);

	return count;
};
static DEVICE_ATTR(cs_stby_cycle, S_IWUSR | S_IRUGO, cs_stby_cycle_time_show, cs_stby_cycle_time_store);
//add by leo for standby mode --

static ssize_t cs_gain_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 regvalue =0, GAIN=0, STBY=0, DSLEEP=0, INTBit=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	GAIN= regvalue >> 6; 
	STBY= (regvalue & 0x20)>>5; //0x20 = 0010 0000
	DSLEEP= (regvalue & 0x10)>>4; //0x10 = 0001 0000
	INTBit= (regvalue & 0x01); //0x01 = 0000 0001
	if(CS_STATUSMSG) printk( "[%s]		GAIN = 0x%x, STBY = 0x%x, DSLEEP = 0x%x, INT = 0x%x\n",__FUNCTION__,GAIN, STBY, DSLEEP, INTBit);
	
	return sprintf(buf, "Main Control Register = 0x%x, GAIN = 0x%x\n", regvalue, GAIN);
};
static ssize_t cs_gain_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0, GAIN=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	GAIN = ((u8) temp)<<6;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	regvalue = (regvalue & 0x3f) | GAIN;	//0x3f = 00111111

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, regvalue);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write  MAIN_CONTROL Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new GAIN is 0x%x \n",__FUNCTION__,GAIN);

	return count;
};
static DEVICE_ATTR(cs_gain, S_IWUSR | S_IRUGO, cs_gain_show, cs_gain_store);

// add for ME372CG ++
static ssize_t cs1_get_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS1_DeltaCount = 0;

	CS1_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);

	return sprintf(buf, "CS1 delta = 0x%x\n", CS1_DeltaCount );
}
static DEVICE_ATTR(cs1_delta, S_IRUGO,cs1_get_delta,NULL);
// add for ME372CG --

static ssize_t cs2_get_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS2_DeltaCount = 0;

	CS2_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_DELTA);

	return sprintf(buf, "CS2 delta = 0x%x\n", CS2_DeltaCount );
}
static DEVICE_ATTR(cs2_delta, S_IRUGO,cs2_get_delta,NULL);

static ssize_t cs6_get_delta(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS6_DeltaCount = 0;

	CS6_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);

	return sprintf(buf, "CS6 delta = 0x%x\n", CS6_DeltaCount );
}
static DEVICE_ATTR(cs6_delta, S_IRUGO,cs6_get_delta,NULL);

// add for ME372CG ++
static ssize_t cs1_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 CS1_Threshold =0;

	CS1_Threshold = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_THD);

	return sprintf(buf, "CS1 Threshold = 0x%x\n", CS1_Threshold);
};
static ssize_t cs1_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 CS1_Threshold=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	CS1_Threshold = (u8) temp;
	C_data->cs1_thd_2 = CS1_Threshold;
	
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT1_THD, C_data->cs1_thd_1);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSOR_INPUT1_THD Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new CS1 Threshold is 0x%x \n",__FUNCTION__,CS1_Threshold);

	return count;
};
static DEVICE_ATTR(cs1_thd, S_IWUSR | S_IRUGO, cs1_thd_show, cs1_thd_store);
// add for ME372CG --

static ssize_t cs2_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 CS2_Threshold =0;

	CS2_Threshold = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT2_THD);

	return sprintf(buf, "CS2 Threshold = 0x%x\n", CS2_Threshold);
};
static ssize_t cs2_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 CS2_Threshold=0;
	int ret, temp = 0;
	
	sscanf (buf, "%x", &temp);
	CS2_Threshold = (u8) temp;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT2_THD, CS2_Threshold);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSOR_INPUT2_THD Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new CS2 Threshold is 0x%x \n",__FUNCTION__,CS2_Threshold);

	return count;
};
static DEVICE_ATTR(cs2_thd, S_IWUSR | S_IRUGO, cs2_thd_show, cs2_thd_store);

static ssize_t cs6_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 CS6_Threshold =0;

	CS6_Threshold = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD);

	return sprintf(buf, "CS6 Threshold = 0x%x\n", CS6_Threshold);
};
static ssize_t cs6_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 CS6_Threshold=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	CS6_Threshold = (u8) temp;
	C_data->cs6_thd_2 = CS6_Threshold;
	
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD, C_data->cs6_thd_2);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSOR_INPUT6_THD Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new CS6 Threshold is 0x%x \n",__FUNCTION__,CS6_Threshold);

	return count;
};
static DEVICE_ATTR(cs6_thd, S_IWUSR | S_IRUGO, cs6_thd_show, cs6_thd_store);

//add by leo for standby mode ++
static ssize_t cs_standby_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_THRESHOLD);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_THRESHOLD Failed\n");
	}
	return sprintf(buf, "Standby Threshold Register = 0x%x\n", ret);
};
static ssize_t cs_standby_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 STBY_Threshold = 0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	STBY_Threshold = ((u8)temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_THRESHOLD, STBY_Threshold);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write Standby Threshold Register Failed (%d)\n",__FUNCTION__,ret);
	}

	return count;
};
static DEVICE_ATTR(cs_stby_thd, S_IWUSR | S_IRUGO, cs_standby_thd_show, cs_standby_thd_store);
//add by leo for standby mode --

static ssize_t cs_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 CS_Sensitivity =0;

	CS_Sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);

	return sprintf(buf, "CS Sensitivity = 0x%x\n", CS_Sensitivity);
};
static ssize_t cs_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 CS_Sensitivity=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	CS_Sensitivity = (u8) temp;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, CS_Sensitivity);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSITIVITY_CONTROL Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new CS Sensitivity is 0x%x \n",__FUNCTION__,CS_Sensitivity);

	return count;
};
static DEVICE_ATTR(cs_sensitivity, S_IWUSR | S_IRUGO, cs_sensitivity_show, cs_sensitivity_store);

//add by leo for standby mode ++
static ssize_t cs_stby_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	u8 CS_STBY_Sensitivity =0;

	CS_STBY_Sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_SENSITIVITY);

	return sprintf(buf, "CS Standby Sensitivity = 0x%x\n", CS_STBY_Sensitivity);
};
static ssize_t cs_stby_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 CS_STBY_Sensitivity=0;
	int ret, temp = 0;

	sscanf (buf, "%x", &temp);
	CS_STBY_Sensitivity = (u8) temp;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_SENSITIVITY, CS_STBY_Sensitivity);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write STANDBY_SENSITIVITY Failed (%d)\n",__FUNCTION__,ret);
	}
	if((debug)) printk( "//////////////////////////		[%s]	CS new Standby Sensitivity is 0x%x \n",__FUNCTION__,CS_STBY_Sensitivity);

	return count;
};
static DEVICE_ATTR(cs_stby_sensitivity, S_IWUSR | S_IRUGO, cs_stby_sensitivity_show, cs_stby_sensitivity_store);
//add by leo for standby mode --
//----------------------------------------------------------------------------------------------------add for test
static ssize_t cs_reset_interrupt(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cap1106_i2c_data *cap1106_data=data;

	reset_cap1106_interrupt();
	enable_irq(cap1106_data->irq);
	
	//if((debug)) printk( "//////////////////////////		[%s]	cs_reset_interrupt FINISHED \n",__FUNCTION__);
	return sprintf(buf,"cs_reset_interrupt FINISHED\n");
}
static DEVICE_ATTR(cs_reset, S_IRUGO,cs_reset_interrupt,NULL);

static ssize_t cs_repeat_rate(struct device *dev, struct device_attribute *attr, char *buf)
{
	u8 CS_RepeatRate=0;
	
	CS_RepeatRate = i2c_smbus_read_byte_data(cap1106_i2c_client, REPEAT_RATE_ENABLE);
	if (CS_RepeatRate < 0)
	{	
		if(CS_STATUSMSG) printk( "[%s]		CS Read REPEAT_RATE_ENABLE Failed\n",__FUNCTION__);
		return sprintf(buf,"Read ID Register Failure\n");
	}
	
	//if((debug)) printk( "//////////////////////////		[%s]	cs_reset_interrupt FINISHED \n",__FUNCTION__);
	return sprintf(buf,"CS Repeat Rate is 0x%x\n",CS_RepeatRate);
}
static DEVICE_ATTR(cs_repeat, S_IRUGO,cs_repeat_rate,NULL);


// add for threshood and sensitivity ++++++
static ssize_t cs_1_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs_1_sensitivity = 0x%x\n", C_data->cs_sensitivity_1);
};
static ssize_t cs_1_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs_sensitivity_1 = regvalue;

	return count;
};
static DEVICE_ATTR(cs_1_sensitivity, S_IWUSR | S_IRUGO, cs_1_sensitivity_show, cs_1_sensitivity_store);

static ssize_t cs_2_sensitivity_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs_2_sensitivity = 0x%x\n", C_data->cs_sensitivity_2);
};
static ssize_t cs_2_sensitivity_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs_sensitivity_2 = regvalue;

	return count;
};
static DEVICE_ATTR(cs_2_sensitivity, S_IWUSR | S_IRUGO, cs_2_sensitivity_show, cs_2_sensitivity_store);

static ssize_t cs1_1_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs1_1_thd = 0x%x\n", C_data->cs1_thd_1);
};
static ssize_t cs1_1_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs1_thd_1 = regvalue;

	return count;
};
static DEVICE_ATTR(cs1_1_thd, S_IWUSR | S_IRUGO, cs1_1_thd_show, cs1_1_thd_store);

static ssize_t cs6_1_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs6_1_thd = 0x%x\n", C_data->cs6_thd_1);
};
static ssize_t cs6_1_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs6_thd_1 = regvalue;

	return count;
};
static DEVICE_ATTR(cs6_1_thd, S_IWUSR | S_IRUGO, cs6_1_thd_show, cs6_1_thd_store);

static ssize_t cs1_2_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs1_2 = 0x%x\n", C_data->cs1_thd_2);
};
static ssize_t cs1_2_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs1_thd_2 = regvalue;

	return count;
};
static DEVICE_ATTR(cs1_2_thd, S_IWUSR | S_IRUGO, cs1_2_thd_show, cs1_2_thd_store);

static ssize_t cs6_2_thd_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "cs6_2_thd = 0x%x\n", C_data->cs6_thd_2);
};
static ssize_t cs6_2_thd_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	u8 regvalue = 0;
	int temp = 0;
	
	sscanf (buf, "%x", &temp);
	regvalue = ((u8)temp);

	C_data->cs6_thd_2 = regvalue;

	return count;
};
static DEVICE_ATTR(cs6_2_thd, S_IWUSR | S_IRUGO, cs6_2_thd_show, cs6_2_thd_store);
// add for threshood and sensitivity -------

//add by leo for test ++
static ssize_t cs1_clibration_register_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, 0xb1);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_CHANNEL Failed\n");
	}
	return sprintf(buf, "cs1_clibration_register = 0x%x\n", ret);
};
static DEVICE_ATTR(cs1_cali_reg, S_IRUGO,cs1_clibration_register_show,NULL);

static ssize_t cs6_clibration_register_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int ret=0;

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, 0xb6);
	if (ret < 0){
		return sprintf(buf, "Read STANDBY_CHANNEL Failed\n");
	}
	return sprintf(buf, "cs6_clibration_register = 0x%x\n", ret);
};
static DEVICE_ATTR(cs6_cali_reg, S_IRUGO,cs6_clibration_register_show,NULL);

static ssize_t cs1_get_max_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i=0,CS1_DeltaCount=0,cs1_max=0;

	for (i=0;i<30;i++){
		CS1_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
		if(CS_STATUSMSG) printk( "[%s]		CS1_DeltaCount = %+d\n",__FUNCTION__,(char)CS1_DeltaCount);
		if(CS1_DeltaCount>cs1_max){
			cs1_max=CS1_DeltaCount;
		}
		msleep(50);
	}
	return sprintf(buf, "cs1_max = %+d\n", (char)cs1_max);
};
static DEVICE_ATTR(cs1_get_max, S_IRUGO,cs1_get_max_show,NULL);

static ssize_t cs6_get_max_show(struct device *class,struct device_attribute *attr,char *buf)
{
	int i=0,CS6_DeltaCount=0,cs6_max=0;

	for (i=0;i<30;i++){
		CS6_DeltaCount  = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
		if(CS_STATUSMSG) printk( "[%s]		CS6_DeltaCount = %+d\n",__FUNCTION__,(char)CS6_DeltaCount);
		if(CS6_DeltaCount>cs6_max){
			cs6_max=CS6_DeltaCount;
		}
		msleep(50);
	}
	return sprintf(buf, "cs6_max = %+d\n", (char)cs6_max);
};
static DEVICE_ATTR(cs6_get_max, S_IRUGO,cs6_get_max_show,NULL);

static ssize_t cs_report_test_show(struct device *class,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "nothing\n");
};
static ssize_t cs_report_test_store(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	int temp=0;
	sscanf (buf, "%d", &temp);

	if(temp==1){
		cap1106_send_signal(SENSED);
	}else if(temp==0){
		cap1106_send_signal(NOT_SENSED);
	}

	return count;
};
static DEVICE_ATTR(cs_direct_report, S_IWUSR | S_IRUGO, cs_report_test_show, cs_report_test_store);

//add by leo for test --
	
static struct attribute *cap1106_attributes[] = {
	&dev_attr_cap1106_id.attr,	//cat only
	&dev_attr_cs_en.attr,
	&dev_attr_cs_info.attr,		//cat only
	&dev_attr_cs_status.attr,	//cat only
	&dev_attr_cs_noise.attr,	//cat only
	&dev_attr_cs1_delta.attr,	//cat only
	&dev_attr_cs1_thd.attr,
	&dev_attr_cs2_delta.attr,	//cat only
	&dev_attr_cs2_thd.attr,
	&dev_attr_cs6_delta.attr,	//cat only
	&dev_attr_cs6_thd.attr,
	&dev_attr_cs_sensitivity.attr,
	&dev_attr_cs_cycle.attr,
	&dev_attr_cs_gain.attr,	
	&dev_attr_cs_reset.attr,		//cat only
	&dev_attr_cs_repeat.attr,	//cat only
	&dev_attr_cs_stby_en.attr,			// add by leo for standby mode
	&dev_attr_cs_stby_cycle.attr,		// add by leo for standby mode
	&dev_attr_cs_stby_sensitivity.attr,	// add by leo for standby mode
	&dev_attr_cs_stby_thd.attr,			// add by leo for standby mode
	&dev_attr_cap_status.attr,
	&dev_attr_cap_check.attr, 				// add by leo to detect cap sensor chip
	&dev_attr_capsensor_exist_check.attr,		// add by leo to detect cap sensor chip
	&dev_attr_capsensor_status_check.attr, 	// add by leo to detect cap sensor chip
	//add by josh ++++++
	&dev_attr_cs_1_sensitivity.attr,
	&dev_attr_cs_2_sensitivity.attr,
	&dev_attr_cs1_1_thd.attr,	
	&dev_attr_cs6_1_thd.attr,
	&dev_attr_cs1_2_thd.attr,	
	&dev_attr_cs6_2_thd.attr,	
	//add by josh ------
	&dev_attr_cs1_cali_reg.attr,	
	&dev_attr_cs6_cali_reg.attr,
	&dev_attr_cs1_get_max.attr,
	&dev_attr_cs6_get_max.attr,
	&dev_attr_cs_report.attr,
	&dev_attr_cs_direct_report.attr,
	NULL
};

static const struct attribute_group cap1106_attr_group = {
	.attrs = cap1106_attributes,
};
//========================================================================================
static int cap1106_open(struct inode *inode, struct file *file)
{
	if((debug)) printk( "//////////////////////////		[%s]	CS MISC Device Open \n",__FUNCTION__);
	return nonseekable_open(inode, file);
}
static int cap1106_release(struct inode *inode, struct file *file)
{
	if((debug)) printk( "//////////////////////////		[%s]	CS MISC Device Release \n",__FUNCTION__);
	return 0;
}
static ssize_t cap1106_read (struct file *file, char __user *buf, size_t n, loff_t *ppos)
{
	if((debug)) printk( "//////////////////////////		[%s]	CS MISC Device Read \n",__FUNCTION__);
	return 0;
}
static ssize_t cap1106_write(struct file *file, const char __user *buf, size_t n, loff_t *ppos)
{
	char pidbuf[10];
	struct cap1106_i2c_data *cap1106_data = data;

	if(n > 10)
		return -EINVAL;

	if (copy_from_user (pidbuf, buf, n)) {
		if(CS_STATUSMSG) printk( "[%s]		CS MISC Device Write Failed\n",__FUNCTION__);
		return n;
	}
	sscanf (pidbuf, "%d", &notify_daemon_pid);	
	if((debug)) printk( "//////////////////////////		[%s]	CS MISC Device Write \n",__FUNCTION__);
	if((debug)) printk( "//////////////////////////		[%s]	notify_daemon_pid = %d\n",__FUNCTION__,notify_daemon_pid);

	isDaemonReady=1; // add by leo for system crash issue ++

// add by leo for modem reset ++
	if(isSendSignalSuccess<0){
		msleep(500);
		if((debug)) printk( "//////////////////////////		[%s]	do work function again\n",__FUNCTION__);
		//disable_irq_nosync(cap1106_data->irq);
		queue_delayed_work(cap1106_wq, &cap1106_work, 0);
	}
// add by leo for modem reset --

	return n;
}
static long cap1106_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val=0, err=0, temp=0; 
	
	//struct CAP1106_i2c_data *cap1106_data = data;
	if((debug)) printk( "//////////////////////////		[%s] 	CS IOCTL (Cmd is %d) \n",__FUNCTION__, _IOC_NR(cmd));
	
	if (_IOC_TYPE(cmd) != CAPSENSOR_IOCTL_MAGIC)
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
	case CAPSENSOR_IOCTL_ENABLE:
		 		
		if (get_user(val, (unsigned long __user *)arg))
			return -EFAULT;
		if (val){
			if((debug)) printk( "//////////////////////////		[%s]	CAPSENSOR_IOCTL_ENABLE - cs_enable \n",__FUNCTION__);
			temp =1;
		}else{
			if((debug)) printk( "//////////////////////////		[%s]	CAPSENSOR_IOCTL_ENABLE - cs_disable \n",__FUNCTION__);
			temp =0;
		}
		return temp;
		break;
			
	case CAPSENSOR_IOCTL_GET_ENABLED:

		if((debug)) printk( "//////////////////////////		[%s]	CAPSENSOR_IOCTL_GET_ENABLED (return %d)\n",__FUNCTION__,temp);
		
		return put_user(temp, (unsigned long __user *)arg);
		break;
	
	default:
		if(CS_STATUSMSG) printk( "[%s]		Incorrect Cmd  (%d) \n",__FUNCTION__, _IOC_NR(cmd));
		return -EINVAL;
	}
	
	return 0;
}
static struct file_operations cs_fops = {
	.owner   = THIS_MODULE,
	.open    = cap1106_open,
	.release = cap1106_release,
	.read    = cap1106_read,
	.write   = cap1106_write,
	.unlocked_ioctl = cap1106_ioctl
};
static struct miscdevice cs_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name  = "cap1106_cs_misc_dev",
	.fops  = &cs_fops,
};

static int cap1106_setup(struct cap1106_i2c_data *cap1106_data)
{
	int err = -EINVAL;

	/*register CS input device*/
	cap1106_data->cs_input_dev = input_allocate_device();
	if (!cap1106_data->cs_input_dev) {
		if(CS_STATUSMSG) printk( "[%s]		Could Not Allocate CS Input Device\n",__FUNCTION__);
		return -ENOMEM;
	}
	cap1106_data->cs_input_dev->name = "cap1106_cs_input_dev";
	set_bit(EV_ABS, cap1106_data->cs_input_dev->evbit);
	set_bit(EV_SYN, cap1106_data->cs_input_dev->evbit);
	input_set_abs_params(cap1106_data->cs_input_dev, ABS_SENSED, 0, 1, 0, 0);
	
	err = input_register_device(cap1106_data->cs_input_dev);
	if (err < 0) {
		if(CS_STATUSMSG) printk( "[%s]		Could Not Register CS Input Device\n",__FUNCTION__);
		goto register_cs_input_device_err;
	}


  	/*register CS misc device*/
	err = misc_register(&cs_misc_dev);
	if (err < 0) {
		if(CS_STATUSMSG) printk( "[%s]		Could Not Register CS Misc Device \n",__FUNCTION__);
		goto register_cs_misc_device_err;
	}

	if((debug)) printk( "//////////////////////////		[%s]	CS_SETUP - FINISHED \n",__FUNCTION__);
	return 0;

	
register_cs_misc_device_err:
	input_unregister_device(cap1106_data->cs_input_dev);
register_cs_input_device_err:
	input_free_device(cap1106_data->cs_input_dev);	
	return err;
}

static int cap1106_initial(struct cap1106_i2c_data *cap1106_data)
{
	int ret=0, regvalue=0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if (regvalue < 0){
		if(CS_STATUSMSG) printk("[%s]		Read MAIN_CONTROL Failed\n",__FUNCTION__);
		return regvalue;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue | SET_DSLEEP));
	if (ret<0){
		if(CS_STATUSMSG) printk("[%s]		SET_DSLEEP Failed\n",__FUNCTION__);
		return ret;
	}

	//CS_STBY_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, 0x00); //disable all standby mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		disable standby mode sensors failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, STANDBY_CHANNEL);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS read STANDBY_CHANNEL failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	if(CS_STATUSMSG) printk( "[%s]		CS_STBY_EN = 0x%02x\n",__FUNCTION__,ret);

	//CS_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, 0x00); //disable all normal mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		disable normal mode sensors failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS read SENSOR_INPUT_ENABLE failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	if(CS_STATUSMSG) printk( "[%s]		CS_EN = 0x%02x\n",__FUNCTION__,ret);
	return ret;
}

static int cap1106_set_standby_mode(struct cap1106_i2c_data *cap1106_data)
{
	int ret=0, regvalue=0, PowerState=0, enable=0;
	u8 GainValue=0;

	//CS_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, 0x00); //disable normal mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		disable normal mode sensors failed\n",__FUNCTION__);
		return ret;
	}

	//CS GAIN, STBY, DSLEEP
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if(regvalue<0){
		if(CS_STATUSMSG) printk( "[%s]		read MAIN_CONTROL failed\n",__FUNCTION__);
		return regvalue;	
	}

	GainValue = 0x03; //0x03 = 0000 0011
	regvalue = ((regvalue & 0x3f) | (GainValue<<6)); //0x3f = 0011 1111
	PowerState = (entry_mode==4? (regvalue | SET_DSLEEP):((regvalue |SET_STBY) &CLEAN_DSLEEP)); //add by leo for charge mode
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, PowerState);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS set standby mode 0x%02x failed (%d)\n",__FUNCTION__,PowerState,ret);
		return ret;
	}
	if(CS_STATUSMSG) printk( "[%s]		CS MAIN_CONTROL Register = 0x%02x\n",__FUNCTION__,PowerState);

	//CS_STBY_SENSITIVITY
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_SENSITIVITY, 0x05);// 0x05 = 00000 101, Sensitivity multiplier: 4x (default: 32x));
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write STANDBY_SENSITIVITY failed\n",__FUNCTION__);
		return ret;
	}

	//CS_STBY_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_THRESHOLD, 0x30); // 0x30 = 0011 0000, Threshold: 48 (default: 64));
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write STANDBY_THRESHOLD failed\n",__FUNCTION__);
		return ret;
	}

	//INT MODE
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, REPEAT_RATE_ENABLE, 0); //0: only generate INT when a touch is detected and a release is detected
	if (ret < 0)
	{
		if(CS_STATUSMSG) printk( "[%s]		CS write REPEAT_RATE_ENABLE failed\n",__FUNCTION__);
		return ret;
	}

	//CS_STBY_EN
	enable = (entry_mode==4? 0x00:0x21);//add by leo for charge mode
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, enable); //only enable cs1 & cs6, 0x21 = 00 100001
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS set enable 0x%02x failed (%d)\n",__FUNCTION__,enable,ret);
		return ret;
	}

	isStandbyMode=1;
	if(CS_STATUSMSG) printk( "[%s]		CS set to Standby Mode ++\n",__FUNCTION__);
	return ret;
}

// add by leo for normal mode ++
static int cap1106_set_normal_mode(struct cap1106_i2c_data *cap1106_data)
{
	int ret=0, regvalue=0, PowerState=0, enable=0, DIS_RF_NOISE=0, temp=0;
	u8 GainValue=0;

	//CS_STBY_EN
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, STANDBY_CHANNEL, 0x00); //disable standby mode sensors
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		disable standby mode sensors failed\n",__FUNCTION__);
		return ret;
	}

	//CS_SENSITIVITY
	//ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, 0x5f); // 0x5F = 0 101 1111, Sensitivity multiplier: 4x (default: 32x));
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, C_data->cs_sensitivity_1); // add by josh for test
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write SENSITIVITY_CONTROL failed\n",__FUNCTION__);
		return ret;
	}

	//CS GAIN, STBY, DSLEEP
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if(regvalue<0){
		if(CS_STATUSMSG) printk( "[%s]		read MAIN_CONTROL failed\n",__FUNCTION__);
		return regvalue;	
	}
	
	//GainValue = 0x01; //0x01 = 0000 0001
	GainValue = C_data->cs_gain_1; //add by josh for test
	regvalue = ((regvalue & 0x3f) | (GainValue<<6)); //0x3f = 00111111
	PowerState = (entry_mode==4? (regvalue | SET_DSLEEP):((regvalue & CLEAN_STBY) & CLEAN_DSLEEP));
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, PowerState);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS set normal mode 0x%02x failed (%d)\n",__FUNCTION__,PowerState,ret);
		return ret;
	}
	if(CS_STATUSMSG) printk( "[%s]		CS MAIN_CONTROL Register = 0x%02x\n",__FUNCTION__,PowerState);

	//CS1_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT1_THD,  C_data->cs1_thd_1); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write SENSOR_INPUT1_THD failed (%d)\n",__FUNCTION__,ret);
	}

	//CS6_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD,  C_data->cs6_thd_1); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write SENSOR_INPUT6_THD failed (%d)\n",__FUNCTION__,ret);
	}

	//INT MODE
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, REPEAT_RATE_ENABLE, 0); //0: only generate INT when a touch is detected and a release is detected
	if (ret < 0)
	{
		if(CS_STATUSMSG) printk( "[%s]		CS write REPEAT_RATE_ENABLE failed\n",__FUNCTION__);
		return ret;
	}
	
// add by leo for test ++
	ret = i2c_smbus_read_byte_data(cap1106_i2c_client, CONFIGRURATION2);
	if(ret<0){
		if(CS_STATUSMSG) printk( "[%s]		read CONFIGRURATION2 failed\n",__FUNCTION__);
		return ret;	
	}
	if(CS_STATUSMSG) printk( "[%s]		CONFIGRURATION2 =0x%02x (before)\n",__FUNCTION__,ret);
	
	temp=ret|0x04;

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION2, temp);
	if (ret < 0)
	{
		if(CS_STATUSMSG) printk( "[%s]		CS write CONFIGRURATION2 failed\n",__FUNCTION__);
		return ret;
	}
	if(CS_STATUSMSG) printk( "[%s]		CONFIGRURATION2 =0x%02x (after)\n",__FUNCTION__,temp);

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, AVG_AND_SAMP_CONFIG, 0x60);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write AVG_AND_SAMP_CONFIG Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, RECALIBRATION_CONFIG, 0x67);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write RECALIBRATION_CONFIG Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_NOISE_THD, 0x00);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSOR_INPUT_NOISE_THD Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_CONFIG, 0x04);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSOR_INPUT_CONFIG Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}
/*
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MULTI_TOUCH_BLOCK_CONFIG, 0x00);
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS Write SENSOR_INPUT_CONFIG Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}
*/	
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION1, 0x30); //0xb0
	if (ret < 0){
		if(CS_STATUSMSG) printk("[%s]		Read CONFIGRURATION1 Failed (%d)\n",__FUNCTION__,ret);
		return ret;
	}

// add by leo for test --

	//CS_EN
	enable = (entry_mode==4? 0x00:0x21); //add by leo for charge mode ++
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT_ENABLE, enable); //only enable cs1 & cs6, 0x21 = 00 100001
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS set enable 0x%02x failed (%d)\n",__FUNCTION__,enable,ret);
		return ret;
	}

	isStandbyMode=0;
	if(CS_STATUSMSG) printk( "[%s]		CS set to Normal Mode ++\n",__FUNCTION__);
	return ret;
}
// add by leo for normal mode --

//========================================================================================
// add by leo for bodySAR_notify_daemon ++
static int cap1106_send_signal (sensor_status s)
{
	int ret = 0;
	struct siginfo info;
	struct task_struct *t;
	int signal= 0;
	memset(&info, 0, sizeof(struct siginfo));

	if (s == NOT_SENSED){
		signal = cap_sensor_not_sensed_signal;
	}
	else if(s == SENSED){
		signal = cap_sensor_sensed_signal;
	}
	
	info.si_signo = signal;
	info.si_code = SI_QUEUE;
	info.si_int = 1234;

	rcu_read_lock ();
	t = find_task_by_vpid (notify_daemon_pid);
	
	if(t == NULL){
		rcu_read_unlock ();
		if(CS_STATUSMSG) printk( "[%s]		find_task_by_vpid Failed \n",__FUNCTION__);
		return -ESRCH;
	}
	rcu_read_unlock ();

	ret = send_sig_info (signal, &info, t);
	if (ret < 0) {
		if(CS_STATUSMSG) printk( "[%s]		send_sig_info Failed \n",__FUNCTION__);
		return ret;
	}

	if(CS_STATUSMSG) printk( "[%s]		CS send signal (%d) \n",__FUNCTION__,(signal==cap_sensor_sensed_signal?1:0));
	//if((debug)) printk( "//////////////////////////		[%s]	send_sig_info Failed \n",__FUNCTION__);

//add by leo ++	
	msleep(100);
	signal = check_table_list_signal;
	info.si_signo = signal;
	info.si_code = SI_QUEUE;
	info.si_int = 1234;

	ret = send_sig_info (signal, &info, t);
	if (ret < 0) {
		if((debug)) printk( "[%s]		send_sig_info Failed \n",__FUNCTION__);
		return ret;
	}

	if((debug)) printk( "[%s]		CS send signal (%d) \n",__FUNCTION__,signal);
//add by leo --
	
	return 0;
}
// add by leo for bodySAR_notify_daemon --

#ifdef DOUBLE_SENSITIVITY
static void cap1106_get_data(void){

	struct cap1106_i2c_data *cap1106_data=data;

	cap1106_data->cs_sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
	cap1106_data->cs1_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
	cap1106_data->cs6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
	cap1106_data->cs_status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
		
	if((debug)) printk( "[%s] cs1_data: 0x%02X, cs6_data: 0x%02X, status = 0x%02x, CS_Sensitivity: 0x%02x\n",__FUNCTION__,cap1106_data->cs1_data,cap1106_data->cs6_data,cap1106_data->cs_status,cap1106_data->cs_sensitivity);
	if((debug)) printk( "[%s] SensitivityMode: %s\n",__FUNCTION__,SensitivityMode==HIGH_SENSITIVITY?"HIGH_SENSITIVITY":"LOW_SENSITIVITY");

	return;
}

// add by josh for change sensitivity ++++++
static void cap1106_change_sensitivity(int mode){
	
	int ret=0;
	u8 cs_sensitivity=0, cs1_thd=0, cs6_thd=0;
	
	if(mode == HIGH_SENSITIVITY){
		cs_sensitivity = C_data->cs_sensitivity_1;
		cs1_thd = C_data->cs1_thd_1;
		cs6_thd = C_data->cs6_thd_1;
	}else{
		cs_sensitivity = C_data->cs_sensitivity_2;
		//cs1_thd = C_data->cs1_thd_2;
		//cs6_thd = C_data->cs6_thd_2;
		cs1_thd = 0x01;	//important !!
		cs6_thd = 0x01;	//important !!
	}
	
	//CS1_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT1_THD, cs1_thd ); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write SENSOR_INPUT1_THD failed (%d)\n",__FUNCTION__,ret);
	}

	//CS6_THD
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSOR_INPUT6_THD, cs6_thd ); //default:0x40
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write SENSOR_INPUT6_THD failed (%d)\n",__FUNCTION__,ret);
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL, cs_sensitivity ); // add by josh for test
	if (ret < 0){
		if(CS_STATUSMSG) printk( "[%s]		CS write SENSITIVITY_CONTROL failed\n",__FUNCTION__);
	}

	msleep(100);
	return;
}
// add by josh for change sensitivity ------

static int cap1106_recalibration_check(void){

	int cs1_data=0, cs6_data=0;
	
	cs1_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
	cs6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
	if((debug)) printk( "[%s] cs1_data: 0x%02X, cs6_data: 0x%02X\n",__FUNCTION__,cs1_data,cs6_data);
	//if ((cs1_data >= 0x0f && cs1_data <= C_data->cs1_thd_2)
	//||(cs6_data >= 0x0f && cs6_data <= C_data->cs6_thd_2)){
	//if((cs1_data>0x06)||(cs6_data>0x06)){
	if (( cs1_data <= C_data->cs1_thd_2)
	||( cs6_data <= C_data->cs6_thd_2)){
		return 1;
	}
	return 0;
}

static int cap1106_check_hands_or_table(int n){

	int ret=0, ac=0, recal=1, retries_times=10, debounce_delay=300;
	int status=0, cs1_data=0, cs6_data=0, regvalue=0;
	u8 CS_Sensitivity=0;

	if((debug)) printk( "[%s] ++++++\n",__FUNCTION__);

	// Retry time out
	if(n==0) 
	{
		if(CS_STATUSMSG) printk( "[%s]		Time Out , n = %d \n",__FUNCTION__,n);
		if(CS_STATUSMSG) printk( "[%s]		Change to High Sensitivity \n",__FUNCTION__);
		cap1106_change_sensitivity(HIGH_SENSITIVITY);
		SensitivityMode = HIGH_SENSITIVITY;
		return 0;
	}
	
	if (SensitivityMode == LOW_SENSITIVITY) {

		CS_Sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
		cs1_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
		cs6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
		status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
			
		if((debug)) printk( "[%s] cs1_data: 0x%02X, cs6_data: 0x%02X, status = 0x%02x, CS_Sensitivity: 0x%02x\n",__FUNCTION__,cs1_data,cs6_data,status,CS_Sensitivity);
		if((debug)) printk( "[%s] SensitivityMode: %s , n: %d\n",__FUNCTION__,SensitivityMode==HIGH_SENSITIVITY?"HIGH_SENSITIVITY":"LOW_SENSITIVITY",n);
			
		if ((cs1_data == 0x00 && cs6_data == 0x00)
		|| (cs1_data == 0xFF && cs6_data == 0xFF)
		|| (cs1_data == 0x00 && cs6_data == 0xFF)
		|| (cs1_data == 0xFF && cs6_data == 0x00)
		|| ((cs1_data > C_data->cs1_thd_2) && (cs1_data <= 0x7F))
		|| ((cs6_data > C_data->cs6_thd_2) && (cs6_data <= 0x7F))
		|| (C_data->overflow_status == 0x21 && (((cs1_data > C_data->cs1_thd_2) && (cs1_data <= 0x7F)) || ((cs6_data > C_data->cs6_thd_2) && (cs6_data <= 0x7F))))) {
			if((debug)) printk( "[%s] change back to high sensitivity\n",__FUNCTION__);

			cap1106_change_sensitivity(HIGH_SENSITIVITY);
			SensitivityMode = HIGH_SENSITIVITY;
			recal=0;
		}

		if(recal==1){

			for (retries_times = 10; retries_times > 0; retries_times--)
			{
				if (cap1106_recalibration_check() == 1)ac++;
				if((debug)) printk( "[%s] ac = %d\n",__FUNCTION__,ac);
				msleep (debounce_delay);
			}
			
			if (ac==10) {
// add by leo for test ++
				regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, CONFIGRURATION1);
				if (regvalue < 0){
					if(CS_STATUSMSG) printk("[%s]		Read CONFIGRURATION1 Failed (%d)\n",__FUNCTION__,regvalue);
					return regvalue;
				}
				regvalue=regvalue|0x08;
				//i2c_smbus_write_byte_data(cap1106_i2c_client, CALIBRATION_ACTIVATE, 0x21);
				ret = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION1, regvalue);
				if (ret < 0){
					if(CS_STATUSMSG) printk( "[%s]		CS Write CONFIGRURATION1 Failed (%d)\n",__FUNCTION__,ret);
					return ret;
				}
				if((debug)) printk( "[%s]	CONFIGRURATION1 = 0x%02x (after)\n",__FUNCTION__,regvalue);
// add by leo for test --

				recal = 0;
				status_stable_count = 0;
				if((debug)) printk("[%s] re-calibration cap sensor !!!!\n",__FUNCTION__);
			}
			ac=0;
			cap1106_check_hands_or_table(n-1);
		}
	}	
	if((debug)) printk( "[%s] ------\n",__FUNCTION__);
	return 0;
}
#endif

static sensor_status get_cap1106_status(void)
{
	u8 status = 0;
	//struct cap1106_i2c_data *cap1106_data = data;
	
	status = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT_STATUS);
	if (status < 0){	
		if(CS_STATUSMSG) printk( "[%s]		Read SENSOR_INPUT_STATUS Failed \n",__FUNCTION__);
	}

	if(status==0x01||status==0x20||status==0x21){
		if((debug)) printk( "[%s]		CS SENSED (0x%02x) \n",__FUNCTION__,status);
		return SENSED;
	}else{
		if((debug)) printk( "[%s]		CS NOT_SENSED (0x%02x) \n",__FUNCTION__,status);
		return NOT_SENSED;
	}

 	return UNKNOW;
}
static int reset_cap1106_interrupt(void)
{
	int ret = 0;
	u8 regvalue = 0;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if(regvalue<0){
		if(CS_STATUSMSG) printk( "[%s]	Read MAIN_CONTROL Failed \n",__FUNCTION__);
		return regvalue;
	}

	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue & 0xfe));	//0xfe = 11111110
	if(ret<0){
		if(CS_STATUSMSG) printk( "[%s]	Reset Interrupt Pin Failed \n",__FUNCTION__);
		return ret;	
	}

	return 0;
}
static irqreturn_t cap1106_interrupt_handler(int irq, void *dev_id)
{
	//if((debug)) printk( "//////////////////////////		[%s] INTTERUPT \n",__FUNCTION__);
	//disable_irq_nosync(irq);
	queue_delayed_work(cap1106_wq, &cap1106_work, 0);
	
	return IRQ_HANDLED;
}
static void cap1106_work_function(struct work_struct *work)
{	
	int err=-1, retries_times = 10, debounce_delay=100, retry=10, sense_retry=10, not_sense_retry=10;
	int cs1_data = 0, cs6_data = 0;
	int status = 0, regvalue=0, MAX_DUR_EN=0;
	u8 CS_Sensitivity = 0;
	struct cap1106_i2c_data *cap1106_data = data;

	struct timeval now;
	suseconds_t diff,diff_1,diff_2;

	do_gettimeofday(&now);	
	diff = now.tv_usec; /* microseconds */

	disable_irq(cap1106_data->irq);
#ifdef RF_NOISE_DETECT
	rf_noise_check();
#endif
	reset_cap1106_interrupt(); // add by leo for test

#ifdef RF_NOISE_DETECT
	if(old_rf_noise_flag!=0)goto direct_report_status;
#endif

// add by leo for test ++
	if(CS_Status==NOT_SENSED){
		 regvalue= i2c_smbus_read_byte_data(cap1106_i2c_client, CONFIGRURATION1);
		if(regvalue<0){
			if(CS_STATUSMSG) printk( "[%s]		read CONFIGRURATION1 failed\n",__FUNCTION__);
		}
		if((debug)) printk( "[%s]	CONFIGRURATION1 = 0x%02x (before)\n",__FUNCTION__,regvalue);
		
		MAX_DUR_EN=regvalue&0x08;
		if(MAX_DUR_EN){
			//MAX_DUR_EN=0;
			regvalue=regvalue&~0x08; //0xf7 = 1111 0111
			if((debug)) printk( "[%s]	clean MAX_DUR_EN\n",__FUNCTION__);
		}

		err = i2c_smbus_write_byte_data(cap1106_i2c_client, CONFIGRURATION1, regvalue);
		if (err < 0){
			if(CS_STATUSMSG) printk( "[%s]		CS Write CONFIGRURATION1 Failed (%d)\n",__FUNCTION__,err);
		}
		if((debug)) printk( "[%s]	CONFIGRURATION1 = 0x%02x (after)\n",__FUNCTION__,regvalue);
	}
// add by leo for test --

#ifdef DOUBLE_SENSITIVITY
	CS_Sensitivity = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSITIVITY_CONTROL);
	cs1_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT1_DELTA);
	cs6_data = i2c_smbus_read_byte_data(cap1106_i2c_client, SENSOR_INPUT6_DELTA);
	status = i2c_smbus_read_byte_data(cap1106_i2c_client, 0x03);
	
	if((debug)) printk( "[%s]	cs1_data: 0x%02X, cs6_data: 0x%02X, status = 0x%02x, CS_Sensitivity: 0x%02x\n",__FUNCTION__,cs1_data,cs6_data, status,CS_Sensitivity);
	if((debug)) printk( "[%s]	SensitivityMode: %s\n",__FUNCTION__,SensitivityMode==HIGH_SENSITIVITY?"HIGH_SENSITIVITY":"LOW_SENSITIVITY");

	if(SensitivityMode == HIGH_SENSITIVITY){
		if((status == 0x01 && (cs1_data == 0x7f || cs6_data == 0x7f)) ||
		   (status == 0x20 && (cs1_data == 0x7f || cs6_data == 0x7f)) ||
		   (status == 0x21 && (cs1_data == 0x7f || cs6_data == 0x7f))){
		   
			C_data->overflow_status = status;
			if((debug)) printk( "[%s]	status_stable_count = %d\n",__FUNCTION__,status_stable_count);

			if(status_stable_count < 5){
				cap1106_change_sensitivity(LOW_SENSITIVITY);
				SensitivityMode = LOW_SENSITIVITY;
				work_count = 0;
				status_stable_count++;
				cap1106_check_hands_or_table(10);
			}else{
				work_count = 6; // add by leo for test
			}
		}
#endif

direct_report_status:
		CS_Status = get_cap1106_status();

		if((CS_Status == CS_OldStatus)&&(isSendSignalSuccess == 0)){	// add by leo for modem reset ++
			if((debug)) printk( "[%s]	CS State doesn't changed \n",__FUNCTION__);
			goto cs_status_didnt_change;
		}

		retry=(CS_Status==SENSED?sense_retry:not_sense_retry);
		for (retries_times = retry; retries_times > 0; retries_times--)
		{
			if (gpio_get_value(CAP1106_INTERRUPT_GPIO) == 0){
				reset_cap1106_interrupt();
				goto cs_status_unstable;
			}
			msleep (debounce_delay);
			if((debug)) printk( "[%s]	retries_times = %d\n",__FUNCTION__,retries_times);
		}

		//input_report_abs(cap1106_data->cs_input_dev, ABS_SENSED, CS_Status);
		//input_sync(cap1106_data->cs_input_dev);	
	
// add by leo for bodySAR_notify_daemon ++
		if((build_version!=1)&&(isDaemonReady==1)){
			err=cap1106_send_signal(CS_Status);
			if(err<0){
				if(CS_STATUSMSG) printk( "[%s]		send signal to bodySAR_notify Failed (%d)\n",__FUNCTION__,err);
			}
			isSendSignalSuccess = err;

			do_gettimeofday(&now);
			diff_1 = now.tv_usec - diff;
		}
// add by leo for bodySAR_notify_daemon --

#ifdef RF_NOISE_DETECT
		if((CS_Status==NOT_SENSED)&&(old_rf_noise_flag==1)){
			// do calibration
			err=i2c_smbus_write_byte_data(cap1106_i2c_client, CALIBRATION_ACTIVATE, 0x21);
			if (err < 0){
				if(CS_STATUSMSG) printk( "[%s]		CS Write CALIBRATION_ACTIVATE Failed (%d)\n",__FUNCTION__,err);
			}
			if((debug)) printk( "[%s]	CALIBRATION_ACTIVATE = 0x%02x\n",__FUNCTION__,0x21);
		}
#endif

		CS_OldStatus=CS_Status;
		if((debug)) printk( "[%s]		CS Report (%d) \n",__FUNCTION__,CS_Status);

#ifdef DOUBLE_SENSITIVITY
	}
#endif

cs_status_unstable:
cs_status_didnt_change:

#ifdef DOUBLE_SENSITIVITY
	if((status == 0x01 || status == 0x20 || status == 0x21 )&& SensitivityMode == HIGH_SENSITIVITY && work_count < 5){
		if((debug)) printk( "[%s]		work_count : %d\n",__FUNCTION__,work_count);
		queue_delayed_work(cap1106_wq, &cap1106_work, msecs_to_jiffies(1000));
		work_count++;
		//status_stable_count = 0;
	}else{
		work_count = 0;
		//status_stable_count = 0;
	}

	if(status == 0){
		status_stable_count = 0;
	}
#endif

	if((debug)) printk( "[%s]	cap1106_work_function FINISHED \n",__FUNCTION__);
	enable_irq(cap1106_data->irq);

	do_gettimeofday(&now);	
	diff_2 = now.tv_usec - diff;
	printk("Elapsed time 1: %lu, Elapsed time 2: %lu\n", diff_1,diff_2);
  	return;
}

//add by leo for rf noise ++
#ifdef RF_NOISE_DETECT
static int rf_noise_check(void){

	int ret=0, regvalue=0, rf_noise_flag=0;
	struct cap1106_i2c_data *cap1106_data = data;

	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, NOISE_FLAG_STATUS);
	if (regvalue < 0){
		if(CS_STATUSMSG) printk("[%s]		Read MAIN_CONTROL Failed\n",__FUNCTION__);
		return regvalue ;
	}
	cap1106_data->cs1_rf_noise=regvalue&0x01;
	cap1106_data->cs6_rf_noise=regvalue&0x20;

	rf_noise_flag=cap1106_data->cs1_rf_noise|cap1106_data->cs6_rf_noise;

	if((debug)) printk("[%s]		cs1_noise (%d), cs6_noise (%d), old_rf_noise_flag(%d), rf_noise_flag(%d)\n",__FUNCTION__,cap1106_data->cs1_rf_noise,cap1106_data->cs6_rf_noise,old_rf_noise_flag,rf_noise_flag);

	if(old_rf_noise_flag!=rf_noise_flag){
		// do calibration
		ret=i2c_smbus_write_byte_data(cap1106_i2c_client, CALIBRATION_ACTIVATE, 0x21);
		if (ret < 0){
			if(CS_STATUSMSG) printk( "[%s]		CS Write CALIBRATION_ACTIVATE Failed (%d)\n",__FUNCTION__,ret);
		}
		if((debug)) printk( "[%s]	CALIBRATION_ACTIVATE = 0x%02x\n",__FUNCTION__,0x21);
	}
	old_rf_noise_flag=rf_noise_flag;

	if((debug)) printk("[%s]		old_rf_noise_flag(%d), rf_noise_flag(%d)\n",__FUNCTION__,old_rf_noise_flag,rf_noise_flag);
	return 0;
}

static void rf_noise_detect_function(struct work_struct *work){
	
	rf_noise_check();
	queue_delayed_work(cap1106_wq, &rf_noise_detect_work, msecs_to_jiffies(1000));
	if(CS_STATUSMSG) printk( "[%s]		++\n",__FUNCTION__);
	return;
}
#endif
//add by leo for rf noise --

static int cap1106_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int err=0;
	struct cap1106_platform_data *pdata; // add by leo for cap1106_platfor_data
	struct cap1106_i2c_data *cap1106_data=NULL;
	struct cap1106_config_data *config_data = NULL;//add by josh

	if(CS_STATUSMSG) printk("====================== \n");
	if(CS_STATUSMSG) printk( "[%s]		build version = %d\n",__FUNCTION__,build_version); //add by leo for read build version
	
	if(!(cap1106_data = kzalloc(sizeof(struct cap1106_i2c_data), GFP_KERNEL))){
		if(build_version!=1)return -ENOMEM;
	}
	memset(cap1106_data, 0, sizeof(struct cap1106_i2c_data));

	cap1106_data->i2c_client = client;
	i2c_set_clientdata(client, cap1106_data);
	cap1106_i2c_client = cap1106_data->i2c_client;	// we defined cap1106_i2c_client
	cap1106_i2c_client->flags = 1;
	strlcpy(cap1106_i2c_client->name, CAP1106_I2C_DEVICE_NAME, I2C_NAME_SIZE);
	data = cap1106_data;
	
	if(!(config_data = kzalloc(sizeof(struct cap1106_config_data), GFP_KERNEL))){
		if(build_version!=1)return -ENOMEM;
	}
	memset(config_data, 0, sizeof(struct cap1106_config_data));
	
#ifdef CONFIG_ME372CG
	config_data->cs_sensitivity_1 = 0x1f; // add by leo for test
#else
	config_data->cs_sensitivity_1 = 0x0f; // add by tom for ME372CL one cap-sensor 
#endif
	config_data->cs_gain_1 = 0x03;
	config_data->cs1_thd_1 = 0x0a;
	config_data->cs6_thd_1 = 0x0a;
	config_data->cs_sensitivity_2 = 0x3f;
	config_data->cs_gain_2 = config_data->cs_gain_1;
#ifdef CONFIG_ME372CG
	config_data->cs1_thd_2 = 0x45; 
	config_data->cs6_thd_2 = 0x6a; 
#else
	config_data->cs1_thd_2 = 0x2d; // modify by tom for fine tune threshold
	config_data->cs6_thd_2 = 0x4d; // modify by tom for fine tune threshold
#endif
	config_data->overflow_status = 0x0;
	C_data = config_data;
	
// add by leo for cap1106_platform_data ++
	pdata = client->dev.platform_data;
	CAP1106_INTERRUPT_GPIO = pdata->gpio;
// add by leo for cap sensor  exist or not (ie, gpio default high or low) ++
	cap1106_initial(cap1106_data);

	if(CS_STATUSMSG) printk("[%s]		CAP1106_INTERRUPT_GPIO default setting is %s\n",__FUNCTION__,((gpio_get_value(CAP1106_INTERRUPT_GPIO)==0)?"Low":"High"));
	isCapSensorExist=gpio_get_value(CAP1106_INTERRUPT_GPIO);
	if(CS_STATUSMSG) printk("[%s]		cap sensor %s\n",__FUNCTION__,(isCapSensorExist==0)?"doesn't exist ":"exist");
// add by leo for cap sensor  exist or not (ie, gpio default high or low) --
// add by leo for cap1106_platform_data --

	err = cap1106_check_device(cap1106_data);
	if (err<0){
		if(CS_STATUSMSG) printk("[%s]		cap1106_i2c_probe Failed\n",__FUNCTION__);
		if(build_version!=1)goto cap1106_check_device_err;
	}

	cap1106_wq = create_singlethread_workqueue("cap1106_wq");
	if (!cap1106_wq) {
		if(CS_STATUSMSG) printk("[%s]		Create WorkQueue Failed\n",__FUNCTION__);
		err = -ENOMEM;
		if(build_version!=1)goto create_singlethread_workqueue_err;
	}
	INIT_DEFERRABLE_WORK(&cap1106_work, cap1106_work_function);
#ifdef RF_NOISE_DETECT
	//INIT_DELAYED_WORK(&rf_noise_detect_work, rf_noise_detect_function); //add by leo for rf noise
#endif
	cap1106_create_proc_file(); // add by leo for proc file ++
	cap1106_setup(cap1106_data);	
	//cap1106_set_standby_mode(cap1106_data);
	cap1106_set_normal_mode(cap1106_data);
	
	err = sysfs_create_group(&client->dev.kobj, &cap1106_attr_group);
	if (err){
		if(CS_STATUSMSG) printk("[%s]		Register sysfs Failed\n",__FUNCTION__);
		if(build_version!=1)goto sysfs_create_group_err;
	}

	cap1106_setup_irq(cap1106_data); // modify flow by leo for system crash issue

	reset_cap1106_interrupt();
	
#ifdef RF_NOISE_DETECT
	//queue_delayed_work(cap1106_wq, &rf_noise_detect_work, msecs_to_jiffies(1000));
#endif
	if(CS_STATUSMSG) printk("====================== \n");

	return 0;
	
sysfs_create_group_err:	
	cap1106_remove_proc_file(); // add by leo for proc file ++
create_singlethread_workqueue_err:
cap1106_check_device_err:
//	gpio_free(CAP1106_INTERRUPT_GPIO);
	kfree(cap1106_data);
	return err;
}

static int cap1106_suspend(struct i2c_client *client , pm_message_t mesg)
{

	int ret=0;
	u8 regvalue = 0;
	
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if (regvalue < 0){
		if(build_version!=1)if(CS_STATUSMSG) printk("[%s]		Read MAIN_CONTROL Failed\n",__FUNCTION__);
		if(build_version!=1)return regvalue;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue | SET_DSLEEP));
	if (ret<0){
		if(build_version!=1)if(CS_STATUSMSG) printk("[%s]		SET_DSLEEP Failed\n",__FUNCTION__);
		if(build_version!=1)return ret;
	}
	
	if((debug)) printk("[%s] : SET_DSLEEP - Finished !!\n",__FUNCTION__);

	//enable_irq_wake(data->irq);
	return 0;
}

static int cap1106_resume(struct i2c_client *client)
{

	int ret=0;
	u8 regvalue = 0;
	
	regvalue = i2c_smbus_read_byte_data(cap1106_i2c_client, MAIN_CONTROL);
	if (regvalue < 0){
		if(build_version!=1)if(CS_STATUSMSG) printk("[%s]		Read MAIN_CONTROL Failed\n",__FUNCTION__);
		if(build_version!=1)return regvalue;
	}
	ret = i2c_smbus_write_byte_data(cap1106_i2c_client, MAIN_CONTROL, (regvalue & CLEAN_DSLEEP));
	if (ret<0){
		if(build_version!=1)if(CS_STATUSMSG) printk("[%s]		CLEAN_DSLEEP Failed\n",__FUNCTION__);
		if(build_version!=1)return ret;
	}
	
	if((debug)) printk("[%s] : CLEAN_DSLEEP - Finished !!\n",__FUNCTION__);

	//disable_irq_wake(data->irq);
	return 0;
}

static int __exit cap1106_i2c_remove(struct i2c_client *client)
{
	if(CS_STATUSMSG) printk( "[%s] \n", __FUNCTION__);
	cap1106_remove_proc_file(); // add by leo for proc file ++
	sysfs_remove_group(&client->dev.kobj, &cap1106_attr_group);
	kfree(i2c_get_clientdata(client));
	cap1106_i2c_client = NULL;
	return 0;
}

static const struct i2c_device_id cap1106_i2c_idtable[] = {
	{"cap1106", 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, cap1106_i2c_idtable);

static struct i2c_driver cap1106_i2c_driver = {
	
	.driver = {
		.name = CAP1106_I2C_DEVICE_NAME,
		.owner = THIS_MODULE,
	},
	.probe = 		cap1106_i2c_probe,
	.suspend	=	cap1106_suspend,
	.resume	=	cap1106_resume,
	.remove	=	cap1106_i2c_remove,
	.id_table = 	cap1106_i2c_idtable,	
};

static int __init cap1106_init(void)
{
	int ret;
	
 	ret = i2c_add_driver(&cap1106_i2c_driver);
	if ( ret != 0 ) {
		if(CS_STATUSMSG) printk("[%s]		Couldn't add cap1106_i2c_driver (ret = %d) \n",__FUNCTION__,ret);
		return ret;
	}else{
		if((debug)) printk("[%s]		Success !!\n",__FUNCTION__);
		return ret;
	}
}

static void __exit cap1106_exit(void)
{
	i2c_del_driver(&cap1106_i2c_driver);
	gpio_free(CAP1106_INTERRUPT_GPIO);//leo added
	misc_deregister(&cs_misc_dev);//leo added
	destroy_workqueue (cap1106_wq);//leo added
}

module_init(cap1106_init);
module_exit(cap1106_exit);

MODULE_AUTHOR("Asus Tek. <asus@asus.com>");
MODULE_DESCRIPTION("SMSC CAP1106 Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);
