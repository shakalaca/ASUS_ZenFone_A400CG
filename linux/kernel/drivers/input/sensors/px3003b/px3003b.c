#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <asm/setup.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/px3003b.h>
#include <linux/proc_fs.h>
#include <linux/wakelock.h>
#include <linux/HWVersion.h>
//  added for work queue 2014.03.06 ++
#include <linux/workqueue.h>
//  added for work queue 2014.03.06 --

#ifdef CONFIG_PF400CG
#include <linux/microp_notify.h> // add by tom for EC notify
#endif
#define PX3003B_DRV_NAME				"px3003b_misc_dev"

#define PX3003B_I2C_NAME				"px3003b"		

#define DRIVER_VERSION					"1.0.0.0"

#define PX3003B_NUM_CACHABLE_REGS	6

extern int build_version;

static struct i2c_client *px3003b_i2c_client = NULL;

// Sensors Interrupt Workqueue
static struct workqueue_struct *px3003b_struct_wq;
struct work_struct px3003b_workq;
static void px3003b_workqueue_function(struct work_struct *work);

struct px3003b_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct mutex lock;
	int px3003b_reg_status[PX3003B_NUM_CACHABLE_REGS];
	int px3003b_state;
	int int_gpio;
	int irq;
	struct wake_lock wake_lock;
};
static struct px3003b_data *ps_sensor_data;


//	px3003b Unit Function ++
static int px3003b_input_init(void);
static int px3003b_enable(int iEnable);
static int px3003b_get_adc_counter(void);
//	px3003b Unit  Function --


//	define for DEBUG message 
static int PS_MESSAGE	= 1;	// default 1 : for show status 
static int PS_DEBUGMSG	= 1;	// default 0 : for Test and debug

/* Debug flag for LogTool 
 * Open : adb shell "echo 1 > /proc/als3010_debug¡¨
 * Close : adb shell "echo 0 > /proc/als3010_debug" 
 */
#define	PX3003B_PROC_FILE	"px3003b_debug"
//static struct proc_dir_entry *px3003b_proc_file;
static unsigned int debug = 0;

typedef enum{ 
	FALSE= 0,
	TRUE 
} boolean; 

/* Sensors Calibration Info */
#define Calibration_length 21			 // 11~20 reserved 
#define Threshold_length 10				 // CValue_length = (Calibration_length-1)/2

/* Calibration Define */ 
#define PS_CALIBRATION_FILE_PATH "/data/sensors/ps_calibration.ini"
#define PS_CROSSTALK_FILE_PATH "/data/sensors/ps_crosstalk.ini"	
#define Default_PS_ThresholdValue 10 
#define CalibrationRetryTimes 5

static int PS_CurrentStatus = -1;
static int RESET_INPUT_DEVICE = -1;

static int Default_PS_Cali_Thres = 4;
static int limit_PS_CrossTalkValue = 8;

static int PS_ThresholdValue = Default_PS_ThresholdValue;
static int PS_CrossTalk =0; 

static boolean PS_AlreadyCalibration = FALSE;
static boolean PS_AlreadyCrossTalk = FALSE;	

static int PS_CalibrationRetryCount=CalibrationRetryTimes;
static int PS_CrosstalkRetryCount=CalibrationRetryTimes;

static int px3003b_set_threshold(int threshold); 
/* Calibration Function Define */ 
static int px3003b_get_calibration_value(void); 
static int px3003b_calibration(int iControl); 
static int px3003b_calibration_proximity(void);
static int px3003b_crosstalk_proximity(void);
/* Crosstalk Function Define */ 
static int px3003b_get_crosstalk_value(void);
static int px3003b_crosstalk(void); 
#ifdef CONFIG_PF400CG
static int CheckFactoryOpen(void);
static int CheckSensorOpen(void);
	
/* EC notity */
static int px3003b_EC_event_notify(struct notifier_block *this, unsigned long event, void *ptr);
#endif


// for ATTR 
static int IsDisableIrq = 0;

extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);

#ifdef CONFIG_PF400CG
/* EC notity */
// If equal "1" when proximity sensor is Enable from HAL
static int IsEnableSensor = 0;
// If equal "1" when linked in Pad
static int IsLinkedPad = 0;



static struct notifier_block ps_notifier = {
	 .notifier_call = px3003b_EC_event_notify, // callback function
};
static int px3003b_EC_event_notify(struct notifier_block *this, unsigned long event, void *ptr)
{
	if(unlikely(debug)) printk("[%s]		===========Enter PX3003C notify============= \n",__FUNCTION__);	
	int ret = 0;
	switch (event){
		// pad in
		case P01_ADD: 
			if(IsEnableSensor==0){
				IsLinkedPad=1;
			}else{ // Proximity sensor is Enable
				IsLinkedPad=1;
				px3003b_enable(0);
			}
		break;
		// pad out
		case P01_REMOVE: 
			if(IsEnableSensor==0){
				IsLinkedPad=0;
			}else{ // Proximity sensor is Enable
				IsLinkedPad=0;
				px3003b_enable(1);
				// fix bug 347758 ++
				input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE,-2);
				input_sync(ps_sensor_data->input_dev);
				if(PS_DEBUGMSG) printk( "[%s]	Pad Out Proximity report -2 to HAL\n",__FUNCTION__);	
				// fix bug 347758 --		
			}
		break;
	}
	// Debug Message
	if(unlikely(debug)) printk( "[%s]		PX3003B Sensor Status = %s \n",__FUNCTION__,IsEnableSensor,(i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_CONFIG_CONFIG)==0x01)?"POWER_ON":"POWER_OFF");
	if(unlikely(debug)) printk( "[%s]		PX3003B IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
	if(unlikely(debug)) printk( "[%s]		PX3003B IsLinkedPad = %d \n",__FUNCTION__,IsLinkedPad);
	if(unlikely(debug)) printk("[%s]		===========Exit PX3003C notify============= \n",__FUNCTION__);	
	return NOTIFY_DONE;
}
#endif
/*-------------------------------------------------------------------------------*/

/* add by Tom for proc file ++ */
static ssize_t px3003b_register_read(char *page, char **start, off_t off, int count, int *eof, void *idata)
{	
	ssize_t sprintf_count = 0;
	if(PS_DEBUGMSG) printk( "[%s]	debug = %d \n",__FUNCTION__,debug);	
	if(debug==0)debug=1;
	else debug=0;
	sprintf_count += sprintf(page + sprintf_count, "debug meaasge (%s)\n", ((debug==1)?"on":"off"));
	return sprintf_count;
}
static ssize_t px3003b_register_write(struct file *filp, const char __user *buff, unsigned long len, void *idata)
{
	char messages[80] = {0};
	char temp[2] = {0};
	int en = 0, ret = 0, raw_data = 0, threshold = 0, crosstalk = 0, iControl = 0;
			
	if (len >= 80) 
	{
		printk("[%s] %d: no command exceeds 80 chars\n",__FUNCTION__,__LINE__);
		return -EFAULT;
	}	

	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	printk("[%s] %d: input command is %s\n",__FUNCTION__,__LINE__,messages);

	// Switch Debug Flag 
	if(messages[0] == '0')
	{
		debug = 0; 
		printk( "[%s]	debug = %d \n",__FUNCTION__,debug);

	}
	else if(messages[0] == '1')
	{
		debug = 1; 
		printk( "[%s]	debug = %d \n",__FUNCTION__,debug);
		
	}
	else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='e')&&(messages[4]=='n'))
    {
    	// Enable/Disable Sensor
        if(&messages[6]!=NULL)
        {
            sscanf (&messages[6], "%d", &en);
        }
		
		if(en==0)
		{
			if(PS_MESSAGE) printk( "[%s]	PX3003B Open Sensor  \n",__FUNCTION__);
			ret = 0;
#ifdef CONFIG_PF400CG
			IsEnableSensor= 0;
			if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d  \n",__FUNCTION__,IsEnableSensor);
#endif
		}
		else if(en==1)
		{
			if(PS_MESSAGE) printk( "[%s]	PX3003B Close Sensor  \n",__FUNCTION__);
			ret = 1;
#ifdef CONFIG_PF400CG
			IsEnableSensor= 1;
			if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d  \n",__FUNCTION__,IsEnableSensor);
#endif
		}
		px3003b_enable(ret);
	}
	else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='d')&&(messages[4]=='a')&&(messages[3]=='d')&&(messages[4]=='a'))
    {
    	// Get Raw Data
		px3003b_enable(1);
		raw_data = px3003b_get_adc_counter();
		px3003b_enable(0);
		if(PS_MESSAGE) printk( "[%s]	PX3003B Raw Data = %d  \n",__FUNCTION__,raw_data);
	}
	else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='t')&&(messages[4]=='h')&&(messages[5]=='r')&&(messages[6]=='e'))
	{
		// Set Threshold
		if(&messages[9]!=NULL) 
		{
			temp[0] = messages[8];
			temp[1] = messages[9];
			sscanf (temp, "%d", &threshold);
			// Get Threshold
			if(threshold == 0)
			{
				threshold = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL);
				threshold &= 0x1f; // 0x1F = 0001 1111 = Bit 0~4 (PS threshold)
			    if(PS_MESSAGE) printk( "[%s]	PX3003B Threshold = %d  \n",__FUNCTION__,threshold);	
				return len;
			}
			ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL, (threshold+0x40));
		    if(PS_MESSAGE) printk( "[%s]	PX3003B Set Threshold ( %d ) \n",__FUNCTION__,threshold);	
		}
		else
		{
			temp[0] = '0';
			temp[1] = messages[8];
			sscanf (temp, "%d", &threshold);
			// Get Threshold
			if(threshold == 0)
			{
				threshold = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL);
				threshold &= 0x1f; // 0x1F = 0001 1111 = Bit 0~4 (PS threshold)
			    if(PS_MESSAGE) printk( "[%s]	PX3003B Threshold = %d  \n",__FUNCTION__,threshold);	
				return len;
			}
			ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL, (threshold+0x40));
			if(PS_MESSAGE) printk( "[%s]	PX3003B Set Threshold ( 0%d ) \n",__FUNCTION__,threshold);	
		}
	}
	else if((messages[0]=='c')&&(messages[1]=='r')&&(messages[2]=='o')&&(messages[3]=='s')&&(messages[4]=='s')&&(messages[5]=='t')&&(messages[6]=='a')&&(messages[7]=='l')&&(messages[8]=='k'))
	{
		// Crosstalk
	    if(&messages[10]!=NULL)
	    {
	        sscanf(&messages[10], "%d", &iControl);
		}
		
		if(iControl==0)	//Get crosstalk value
		{				
			crosstalk = px3003b_get_crosstalk_value();
			if(PS_MESSAGE) printk( "[%s]	PX3003B Crosstalk =  %d  \n",__FUNCTION__,crosstalk);				
			
		}
		else if(iControl==1)	//Set crosstalk value
		{		
		
			ret = px3003b_enable(1); 
			if(ret < 0)  {
					if(PS_MESSAGE) printk( "[%s]	Open P-sensor Failed \n",__FUNCTION__);
			}
			msleep(150);
			
			ret = px3003b_crosstalk();
			if(ret< 0)  {
				    if(PS_MESSAGE) printk( "[%s]	PS CrossTalk Failed (%d)\n",__FUNCTION__,ret);
			}

			ret = px3003b_enable(0);
			if(ret < 0)  {
					if(PS_MESSAGE) printk( "[%s]	Close P-sensor Failed \n",__FUNCTION__);
			}
			PS_CrosstalkRetryCount=CalibrationRetryTimes;
		}
	}
	else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='c')&&(messages[4]=='a')&&(messages[5]=='l')&&(messages[6]=='i'))
	{
		// Calibration
	    if(&messages[7]!=NULL)
	    {
	        sscanf(&messages[7], "%d", &iControl);
		}

		if(iControl==0)	//Get crosstalk value
		{				
			threshold = px3003b_get_calibration_value();
			if(PS_MESSAGE) printk( "[%s]	PX3003B Calibration threshold =  %d  \n",__FUNCTION__,threshold);				
			
		}
		else if(iControl==1)	//Set crosstalk value
		{
			ret = px3003b_enable(1); 
			if(ret < 0)  {
					if(PS_MESSAGE) printk( "[%s]	Open P-sensor Failed \n",__FUNCTION__);
			}
			
			ret = px3003b_calibration(1);
			if(ret<0)
			{
				if(PS_MESSAGE) printk( "[%s]	PS Calibration (set threshold calibration value) Failed (%d)\n",__FUNCTION__,ret);
			}	

			ret = px3003b_enable(0);
			if(ret < 0)  {
					if(PS_MESSAGE) printk( "[%s]	Close P-sensor Failed \n",__FUNCTION__);
			}
			
			PS_AlreadyCalibration=FALSE;
			PS_CalibrationRetryCount=CalibrationRetryTimes;
			ret = px3003b_calibration_proximity();
			if(ret<0)
			{
				if(PS_MESSAGE) printk( "[%s]	PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
			}
		}
	}
	else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='e')&&(messages[4]=='v')&&(messages[5]=='e')&&(messages[6]=='n')&&(messages[7]=='t'))
	{
		input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE,5);
		input_sync(ps_sensor_data->input_dev);
		input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE,6);
		input_sync(ps_sensor_data->input_dev);
		if(PS_MESSAGE) printk( "[%s]	PS send Event to HAL \n",__FUNCTION__);
	}
	else if((messages[0]=='p')&&(messages[1]=='s')&&(messages[2]=='_')&&(messages[3]=='c')&&(messages[4]=='t'))
	{
		// Change Default Calibration Threshold
		if(&messages[5]!=NULL)
	    {
	    	temp[0] = messages[5];
			temp[1] = messages[6];
			sscanf (temp, "%d", &threshold);
		}
		
		if(threshold == 0)
		{
			if(PS_MESSAGE) printk( "[%s]	Default Calibration Threshold = %d\n",__FUNCTION__, Default_PS_Cali_Thres);
		}
		else
		{
			Default_PS_Cali_Thres=threshold;
			if(PS_MESSAGE) printk( "[%s]	Set Default Calibration Threshold = %d\n",__FUNCTION__, Default_PS_Cali_Thres);
		}
	}
	else
	{
		if(PS_MESSAGE) printk( "[%s]	Unknow Command\n",__FUNCTION__);
	}
	return len;
}
/*
void px3003b_create_proc_file(void)
{
	px3003b_proc_file = create_proc_entry(PX3003B_PROC_FILE, 0666, NULL);
	if(px3003b_proc_file){
		px3003b_proc_file->read_proc = px3003b_register_read;
		px3003b_proc_file->write_proc = px3003b_register_write;
	}
	else{
		if(PS_DEBUGMSG) printk( "[%s]	create_proc_entry failed !!\n",__FUNCTION__);
	}
}

void px3003b_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    if(unlikely(debug))printk( "[%s] remove proc files \n",__FUNCTION__);
    remove_proc_entry(PX3003B_PROC_FILE, &proc_root);
}
*/
/* add by Tom for proc file -- */

static int CharToInt(char CValue[]){

	int i=0,j=0,x=0,CalibrationValue=0;
	int temp[Threshold_length]={0};
	
	if(unlikely(debug))printk( "[%s]	",__FUNCTION__);// add for test

	for(i=0;i<Threshold_length;i++){
		if(unlikely(debug))printk( "%c",CValue[i]);// add for test
		temp[i]=(int)CValue[i]-48;
	}
	if(unlikely(debug))printk( "\n"); // add for test
	
	CalibrationValue = temp[Threshold_length-1];
	for(i=1;i<Threshold_length;i++){
		
		j=i;
		x=1;
		while(j>0){
			x*=10;
			j--;
		}
		CalibrationValue+=temp[(Threshold_length-1)-i]*x;
	}
	return CalibrationValue;
}


//unit function  ++
static int px3003b_get_adc_counter(void)
{
	int ret=0 , RAW_Data=0, val = 0;
	// check factory mode 
	int mode = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE);

	if(mode != 0xe0){
		// Open factory mode
		ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0xe0);
		if (ret){
			if(PS_MESSAGE) printk("[%s] : px3003b_set_factory_mode fail !!\n",__FUNCTION__);
			return ret;
		}
		// check factory mode 
		val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE);
		if(PS_DEBUGMSG) printk("[%s] : Factory mode: %s !!\n",__FUNCTION__,(val == 0xe0) ? "ON" : "OFF");	
	    msleep(300);
	}
	// Read Raw Data
    RAW_Data=i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_DATA);
	if(RAW_Data<0)  {
		if(PS_MESSAGE) printk( "[%s]		Read PS_DATA Failed\n",__FUNCTION__);
		return -1;
	}
	RAW_Data &= 0x7f; //0x7f = 0111 1111
	if(PS_DEBUGMSG) printk("[%s] : px3003b_get_adc_counter (%d) !!\n",__FUNCTION__,RAW_Data);

	// Close factory mode	
	ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0x00);
	if (ret){
		if(PS_MESSAGE) printk("[%s] : px3003b_set_factory_mode fail !!\n",__FUNCTION__);
		return ret;
	}
	return RAW_Data;
}

static int px3003b_enable(int iEnable)
{
	int ret = 0;
	u8 reg_val = 0x03;


	if((iEnable==1) && (PS_AlreadyCrossTalk==FALSE) && (PS_CrosstalkRetryCount>0)){
		ret = px3003b_crosstalk_proximity();
		if(ret < 0){
			if(PS_MESSAGE) printk("[%s] : px3003b_crosstalk_proximity fail !! ret = (%d)\n",__FUNCTION__,ret);
		}
	}
	/*
	if((iEnable==1) && (PS_AlreadyCalibration==FALSE) && (PS_CalibrationRetryCount>0)){
		ret = px3003b_calibration_proximity();
		if(ret < 0){
			if(PS_MESSAGE) printk("[%s] : px3003b_calibration_proximity fail !! ret = (%d)\n",__FUNCTION__,ret);
		}
	}
	*/
	mutex_lock(&ps_sensor_data->lock);
	switch(iEnable)
	{
		case 0:
			reg_val = 0x0B; // 0x0B = 0000 1011 Mode Select(Power down mode) Operation Select (Idle mode)
		break;

		case 1:
			reg_val = 0x01; // 0x0B = 0000 0001 Mode Select(Power up mode) Operation Select (PS active mode)
		break;

		default:
			reg_val = 0x03;
		break;
	}

	ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_CONFIG_CONFIG, reg_val);
	
	if (ret){
		if(PS_MESSAGE) printk("[%s] : px3003b_set_mode ADDR_PX_CONFIG_CONFIG fail !! ret = (%d)\n",__FUNCTION__,ret);
		if(PS_MESSAGE) printk("[%s] : i2c_smbus_write_byte_data(%x,%x,%x) fail !!\n",__FUNCTION__,ps_sensor_data->client->addr,ADDR_PX_CONFIG_CONFIG,reg_val);
		mutex_unlock(&ps_sensor_data->lock);
		return ret;
	}

	/* Update px3003b_state */
	if(iEnable==0 && reg_val == 0x0B){
			if(PS_MESSAGE) printk(KERN_INFO "[%s] : px3003b_enable (%d) !!\n",__FUNCTION__,iEnable);
	}else if(iEnable==1 && reg_val == 0x01){
			if(PS_MESSAGE) printk(KERN_INFO "[%s] : px3003b_enable (%d) !!\n",__FUNCTION__,iEnable);
	}
	
	ps_sensor_data->px3003b_reg_status[INDEX_PX_CONFIG_CONFIG] = reg_val;	

	// Debug Message ++
	if(unlikely(debug))printk( "[%s]				ps_sensor_data->px3003b_state, iEnable = (%d,%d)\n",__FUNCTION__,ps_sensor_data->px3003b_state,iEnable);
	// Debug Message --	
	
	/* Add wakeup */ 
	if(iEnable==0){

		PS_CurrentStatus = -1;
		input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE, RESET_INPUT_DEVICE);
		input_sync(ps_sensor_data->input_dev);
		
		if(ps_sensor_data->px3003b_state!=iEnable){
			if(PS_DEBUGMSG)printk( "[%s]				disable_irq_wake  (ps_sensor_data->px3003b_state, iEnable)=(%d,%d)\n",__FUNCTION__,ps_sensor_data->px3003b_state,iEnable);
			disable_irq_wake(ps_sensor_data->irq);
		}
	}else{
		if(ps_sensor_data->px3003b_state!=iEnable){
			if(PS_DEBUGMSG)printk( "[%s]				enable_irq_wake  (ps_sensor_data->px3003b_state, iEnable)=(%d,%d)\n",__FUNCTION__,ps_sensor_data->px3003b_state,iEnable);
			enable_irq_wake(ps_sensor_data->irq);
		}
	}

	ps_sensor_data->px3003b_state = iEnable;
	mutex_unlock(&ps_sensor_data->lock);

	if(unlikely(debug))printk( "[%s] : i2c_smbus_write_byte_data(%x,%x,%x) \n",__FUNCTION__,ps_sensor_data->client->addr,ADDR_PX_CONFIG_CONFIG,reg_val);
	return 0;
}

static int px3003b_input_init(void)
{
	int ret;

	// allocate light input_device 
	ps_sensor_data->input_dev = input_allocate_device();
	if (!ps_sensor_data->input_dev) {
		printk("[%s] : could not allocate input device\n",__FUNCTION__);
		goto err_light_all;
		}
	// input_set_drvdata(input_dev, light_sensor_data);
	ps_sensor_data->input_dev->name = "px3003b_input_dev";
	set_bit(EV_ABS, ps_sensor_data->input_dev->evbit);
	set_bit(EV_SYN, ps_sensor_data->input_dev->evbit);
	//input_set_capability(ps_sensor_data->input_dev, EV_ABS, ABS_DISTANCE);
	input_set_abs_params(ps_sensor_data->input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	if(PS_MESSAGE) printk("[%s] : px3003b registering Proximity sensor input device\n",__FUNCTION__);
	ret = input_register_device(ps_sensor_data->input_dev);
	if (ret < 0) {
		printk("[%s] : could not register input device\n",__FUNCTION__);
		goto err_light_reg;
	}
	return 0;

err_light_reg:
	input_free_device(ps_sensor_data->input_dev);
err_light_all:
	return (-1);   
}




//unit function --
/*--------------------------------------------------------------------------------*/

static ssize_t px3003b_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	int val;
	val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_CONFIG_CONFIG);
	if(PS_MESSAGE) printk("[%s] : i2c_smbus_read_byte_data(%x,%x) \n",__FUNCTION__,ps_sensor_data->client->addr, ADDR_PX_CONFIG_CONFIG);
	if (val < 0) {
		if(PS_MESSAGE) printk("[%s] : Get mode fail !! (%d) \n",__FUNCTION__, val);
		if(PS_MESSAGE) printk("[%s] : i2c_smbus_read_byte_data(%x,%x) \n",__FUNCTION__,ps_sensor_data->client->addr, ADDR_PX_CONFIG_CONFIG);
		return sprintf(buf, "%d\n", val);
	}
	val &= 0x03; // 0x03 = 0000 0011 = Bit 1 2 (Device Enable(01) / Disable (11))

	if(val == 0x01)
		val=1;
	else if(val == 0x03)
		val=0;
	
	return sprintf(buf, "%d\n", val);  
	
}
static DEVICE_ATTR(status, (S_IWUSR|S_IRUGO),px3003b_show_status,NULL);

static ssize_t px3003b_show_adc(struct device *dev,struct device_attribute *attr,char *buf)
{
	int ps_adc=0;
	px3003b_enable(1);
	ps_adc = px3003b_get_adc_counter();
	px3003b_enable(0);
	return sprintf(buf, "%d \n", ps_adc);
}
static DEVICE_ATTR(ps_data, (S_IWUSR|S_IRUGO), px3003b_show_adc,NULL);
// mode ++
static ssize_t px3003b_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "status : %d\n", ps_sensor_data->px3003b_state);
}

static ssize_t px3003b_set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret=-1;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7)){
		printk("[%s] : px3003b_set_mode buf(%d) !!\n",__FUNCTION__,buf[0]);
		return -EINVAL;
	}
	if(buf[0]=='0'){
		if(PS_MESSAGE) printk("[%s] : px3003b_set_mode (0) !!\n",__FUNCTION__);
		ret = 0;
#ifdef CONFIG_PF400CG
		IsEnableSensor= 0;
		if(PS_MESSAGE) printk( "[%s]	PX3212C IsEnableSensor (%d)  \n",__FUNCTION__,IsEnableSensor);
#endif
	}
	else if(buf[0]=='1'){
		if(PS_MESSAGE) printk("[%s] : px3003b_set_mode (1) !!\n",__FUNCTION__);
		ret = 1;
#ifdef CONFIG_PF400CG
		IsEnableSensor= 1;
		if(PS_MESSAGE) printk( "[%s]	PX3212C IsEnableSensor (%d)  \n",__FUNCTION__,IsEnableSensor);
#endif
	}else if( (buf[0]=='c') && (buf[1]=='a') && (buf[2]=='l') ){
		if(PS_MESSAGE) printk("[%s] : apx3003b_set_mode cal !!\n",__FUNCTION__);
		ret = 2;
	}else{
		if(PS_MESSAGE) printk("[%s] : px3003b_set_mode fail !!\n",__FUNCTION__);
		return ret;
	}
	px3003b_enable(ret);

	return count;
}
// mode --
static DEVICE_ATTR(mode, (S_IWUSR|S_IRUGO), px3003b_show_mode, px3003b_set_mode);

/* Change client address ++ */
static ssize_t px3003b_change_client(struct device *dev,struct device_attribute *attr,char *buf)
{
	ps_sensor_data->client->addr = (ps_sensor_data->client->addr==0x1c)?0x1d:0x1c;
	printk("[%s] : ps_sensor_data->client->addr(%x) !!\n",__FUNCTION__,ps_sensor_data->client->addr);
	return sprintf(buf, "%x \n", ps_sensor_data->client->addr);
}
static DEVICE_ATTR(client, (S_IWUSR|S_IRUGO), px3003b_change_client, NULL);
/* Change client address -- */


/* Change factory mode ++ */
static ssize_t px3003b_change_factory(struct device *dev,struct device_attribute *attr,char *buf)
{
	int val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE);
	int mode = (val == 0xe0) ? 0x00 : 0xe0;
	int ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, mode);
	if(PS_DEBUGMSG) printk("[%s] : i2c_smbus_write_byte_data(%x,%x,%x) !!\n",__FUNCTION__,ps_sensor_data->client->addr, ADDR_PX_FACTORY_MODE, mode);
		if (ret){
			if(PS_MESSAGE) printk("[%s] : px3003b_change_factory fail !!\n",__FUNCTION__);
			return ret;
		}
	return sprintf(buf, "%x \n", mode);
}
static DEVICE_ATTR(factory, (S_IWUSR|S_IRUGO), px3003b_change_factory, NULL);
/* Change factory mode -- */


/* Get object status ++ */
static ssize_t px3003b_get_object(struct device *dev,struct device_attribute *attr,char *buf)
{
	int val;
	val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_DATA);
	printk("[%s] : val(%d)!!\n",__FUNCTION__,val);
	val &= 0x80; /* 0x80 = 1000 0000 = Bit 7 (PS DATA)*/
	return sprintf(buf, "%d\n", val >> 7);  	/* Bit 7 shift to Bit 1*/
}
static DEVICE_ATTR(object, (S_IWUSR|S_IRUGO),px3003b_get_object, NULL);
/* Get object status -- */


/* Get/Set Threshold ++ */
static ssize_t px3003b_read_threshold(struct device *dev,struct device_attribute *attr,char *buf)
{
	int threshold = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL);
	printk("[%s] : val(%d)!!\n",__FUNCTION__,threshold);
	threshold &= 0x1f; // 0x1F = 0001 1111 = Bit 0~4 (PS threshold)
	return sprintf(buf, "%d\n", threshold);  	
}
static ssize_t px3003b_write_threshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int threshold=0x0a , ret = 0;
	sscanf (buf, "%d", &threshold);
	printk("[%s] : threshold(%d)!!\n",__FUNCTION__,threshold);
	ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL, (threshold+0x40));
 	if(PS_DEBUGMSG) printk("[%s] : i2c_smbus_write_byte_data(%x,%x,%x) !!\n",__FUNCTION__,ps_sensor_data->client->addr, ADDR_PX_PS_CONTROL, (threshold+0x40));
	if (ret){
			if(PS_MESSAGE) printk("[%s] : set_threshold fail !!\n",__FUNCTION__);
			return ret;
	}
	return count;
}
static DEVICE_ATTR(threshold, (S_IWUSR|S_IRUGO),px3003b_read_threshold,px3003b_write_threshold);
/* Get/Set Threshold -- */


/* Get object status ++ */
static ssize_t px3003b_get_interrupt(struct device *dev,struct device_attribute *attr,char *buf)
{
	int val;
	
	input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE,0);
	input_sync(ps_sensor_data->input_dev);
	input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE,9);
	input_sync(ps_sensor_data->input_dev);
	input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE,0);
	input_sync(ps_sensor_data->input_dev);

	val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_INT_STATUS);
	printk("[%s] : val(%d)!!\n",__FUNCTION__,val);
	val &= 0x02; /* 0x02 = 0000 0010 = Bit 2 (PS_int)*/
	return sprintf(buf, "%d\n", val >> 1);  	/* Bit 7 shift to Bit 1*/
}
static DEVICE_ATTR(int_status, (S_IWUSR|S_IRUGO),px3003b_get_interrupt,NULL);
/* Get object status -- */


/* Proximity status ++ */
static ssize_t ProximitySensor_check_For_ATD_test(struct device *dev,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	ret = px3003b_enable(1); 
	if(ret<0)
	{
	 	return sprintf(buf, "%d\n", 0);  	
	}
	mdelay(300);
	
	ret = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_DATA);
	if(ret<0)
	{
	 	return sprintf(buf, "%d\n", 0);  	
	}
	
	ret = px3003b_enable(0);
	if(ret<0)
	{
	 	return sprintf(buf, "%d\n", 0);  	
	}
	
	return sprintf(buf, "%d\n", 1);  	/* Bit 7 shift to Bit 1*/
}
static DEVICE_ATTR(proximity_status, (S_IWUSR|S_IRUGO), ProximitySensor_check_For_ATD_test, NULL);
// Proximity status -- 

// Threshold Calibration  ++ 
static ssize_t ps_do_Calibration_show(struct device *dev, struct device_attribute *attr, char *buf){

	int ret=0;
	ret = px3003b_get_calibration_value();
	if(ret<0){
		return sprintf(buf,"Read PS CalibrationValue Failed (%d)\n",ret);
	}
	return sprintf(buf,"PS Calibration Value =%010d&%010d\n",PS_ThresholdValue,PS_ThresholdValue);
}

static ssize_t ps_do_Calibration_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int iControl;
	int ret=0;
	
	sscanf (buf, "%d", &iControl); // 0: reset, 1: threshold

	ret = px3003b_calibration(iControl);
	if(ret<0){
		if(PS_MESSAGE) printk( "[%s]	PS Calibration (set %s calibration value) Failed (%d)\n",__FUNCTION__,(iControl==1)?"threshold":"reset",ret);
	}	

	PS_AlreadyCalibration=FALSE;
	PS_CalibrationRetryCount=CalibrationRetryTimes;
	ret = px3003b_calibration_proximity();
	if(ret<0){
		if(PS_MESSAGE) printk( "[%s]	PS Calibration (get calibration value) Failed (%d)\n",__FUNCTION__,ret);
	}

	return count;
}
static DEVICE_ATTR(ps_cali, (S_IWUSR|S_IRUGO), ps_do_Calibration_show, ps_do_Calibration_store);
// Threshold Calibration -- 

// Proximity CrossTalk ++
static ssize_t get_crosstalk_value(struct device *dev, struct device_attribute *attr, char *buf){

	int ret=0;
	ret = px3003b_get_crosstalk_value();
	if(ret<0){
		return sprintf(buf,"Read px3003b_get_crosstalk_value() Failed (%d)\n",ret);
	}
	return sprintf(buf,"PS Vendor Calibration Value =%d \n",PS_CrossTalk);
}

static ssize_t set_crosstalk_value(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int ret=0,temp=0;

	sscanf (buf, "%x", &temp);
	if(temp==0){
		//Clean PS_CrossTalk Value 
		PS_CrossTalk=0;
		return count;
		
	}else if(temp==1){
	
		ret = px3003b_enable(1); 
		if(ret < 0)  {
				if(PS_MESSAGE) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,1);
		}
		msleep(150);
		
		ret = px3003b_crosstalk();
		if(ret< 0)  {
			if(PS_MESSAGE) printk( "[%s]	PS CrossTalk Failed (%d)\n",__FUNCTION__,ret);
		}

		ret = px3003b_enable(0);
		if(ret < 0)  {
				if(PS_MESSAGE) printk( "[%s]	ps_enable(%d) Failed \n",__FUNCTION__,0);
		}
		PS_CrosstalkRetryCount=CalibrationRetryTimes;
	}
	
	return count;
}
static DEVICE_ATTR(ps_crosstalk, (S_IWUSR|S_IRUGO),get_crosstalk_value,set_crosstalk_value);
//add by leo for proximity vendor calibration --

//add by Tom for modify MAX proximity crosstalk value ++
static ssize_t ps_MaxCrossTalkValue_show(struct device *dev, struct device_attribute *attr, char *buf){

	return sprintf(buf,"PS allow Maximum Crosstalk Value=%d \n",limit_PS_CrossTalkValue);
}

static ssize_t ps_MaxCrossTalkValue_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){

	int temp=0;
	
	sscanf (buf, "%d", &temp);
	limit_PS_CrossTalkValue = temp;

	if(PS_MESSAGE) printk( "[%s]	Set PS maximum permit crosstalk value = %d \n",__FUNCTION__,limit_PS_CrossTalkValue);
	return count;
}
static DEVICE_ATTR(ps_crosstalk_limit, (S_IWUSR|S_IRUGO), ps_MaxCrossTalkValue_show, ps_MaxCrossTalkValue_store);
//add by Tom for modify MAX proximity crosstalk value --
#ifdef CONFIG_PF400CG
/* Dump ++ */
static ssize_t Set_dump(struct device *dev, struct device_attribute *attr,const char *buf, size_t count){
	int option;
	int Raw_data = 0 ,ret = 0;
	sscanf (buf, "%x", &option);
	switch(option){
		case 0 :
			IsEnableSensor=0;
			if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			if(PS_MESSAGE) printk( "[%s]	PX3003B ps_enable(%d) \n", __FUNCTION__,ps_sensor_data->px3003b_state);
		break;
		case 1 :
			IsEnableSensor=1;
			if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			if(PS_MESSAGE) printk( "[%s]	PX3003B ps_enable(%d) \n", __FUNCTION__,ps_sensor_data->px3003b_state);
		break;
		case 3:
			IsEnableSensor=0;
			if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			px3003b_enable(0); 
			if(PS_MESSAGE) printk( "[%s]	PX3003B ps_enable(%d) \n", __FUNCTION__,ps_sensor_data->px3003b_state);
		break;
		case 4:
			IsEnableSensor=1;
			if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
			px3003b_enable(1); 
			if(PS_MESSAGE) printk( "[%s]	PX3003B ps_enable(%d) \n", __FUNCTION__,ps_sensor_data->px3003b_state);
		break;
		case 5:
			ret = CheckSensorOpen();
			if(ret < 0){
				if(PS_MESSAGE) printk( "[%s]			CheckSensorOpen Failed\n", __FUNCTION__);
				BUG();
				return sprintf(buf,"PX3003B Read Raw Data Failed !!");
			}
			Raw_data = px3003b_get_adc_counter();
			if(PS_MESSAGE) printk( "[%s]			PX3003B PS DATA = %d\n", __FUNCTION__,Raw_data);
			if(Raw_data<0){
				BUG();
				return sprintf(buf,"PX3003B Read Raw Data Failed !!");
			}
		break;
		
	}
	return count;
}
static ssize_t Show_dump(struct device *dev,struct device_attribute *attr,char *buf)
{
	if(PS_MESSAGE) printk( "[%s]	================Dump Start=============== \n",__FUNCTION__);
	if(PS_MESSAGE) printk( "[%s]	PX3003B IsEnableSensor = %d \n",__FUNCTION__,IsEnableSensor);
	if(PS_MESSAGE) printk( "[%s]	PX3003B IsLinkedPad = %d \n",__FUNCTION__,IsLinkedPad);
	CheckSensorOpen();
	CheckFactoryOpen();
	if(PS_MESSAGE) printk( "[%s]	================Dump End=============== \n",__FUNCTION__);
	return sprintf(buf, "Dump!!\n");  
}
static DEVICE_ATTR(dump, (S_IWUSR|S_IRUGO), Show_dump, Set_dump);
/* Dump  --*/
#endif


static ssize_t px3003b_get_irq(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "irq status : %s\n", (IsDisableIrq==0)?"disable":"enable");
}

static ssize_t px3003b_set_irq(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int option;
	sscanf (buf, "%x", &option);
	switch(option){
		case 0 :
			if(IsDisableIrq==1){
				enable_irq(ps_sensor_data->irq);
				IsDisableIrq=0;
			}
			printk( "enable_irq() \n");
		break;
		case 1 :
			if(IsDisableIrq==0){
				disable_irq(ps_sensor_data->irq);
				IsDisableIrq=1;
			}
			printk( "disable_irq() \n");  
		break;
		case 2 :
			enable_irq(ps_sensor_data->irq);
			printk( "ensable_irq() \n");  
		break;
		case 3 :
			disable_irq(ps_sensor_data->irq);
			printk( "disable_irq() \n");  
		break;

	}  
	return count;
}
static DEVICE_ATTR(ps_irq, (S_IWUSR|S_IRUGO) ,px3003b_get_irq,px3003b_set_irq);


static ssize_t px3003b_get_CalibrationThreshold(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "Calibration Threshold = %d\n", Default_PS_Cali_Thres);
}

static ssize_t px3003b_set_CalibrationThreshold(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value;
	sscanf (buf, "%d", &value);
	if(value > 0)
		Default_PS_Cali_Thres = value; 
	printk("Set Calibration Threshold (%d)\n", Default_PS_Cali_Thres);
	return count;
}
static DEVICE_ATTR(ps_ct, (S_IWUSR|S_IRUGO) ,px3003b_get_CalibrationThreshold,px3003b_set_CalibrationThreshold);


static struct attribute *px3003b_attributes[] = {
	&dev_attr_status.attr,
	&dev_attr_ps_data.attr,
	&dev_attr_mode.attr,
	&dev_attr_client.attr,
	&dev_attr_object.attr,
	&dev_attr_factory.attr,
	&dev_attr_int_status.attr,
	&dev_attr_threshold.attr,
	&dev_attr_proximity_status.attr,    //for ATD testing
	&dev_attr_ps_cali.attr,				//add by tom for proximity threshold calibration 
	&dev_attr_ps_crosstalk.attr,			//add by tom for proximity crosstalk
	&dev_attr_ps_crosstalk_limit.attr,	//add by tom for modify MAX proximity crosstalk value
	&dev_attr_ps_irq.attr,					//add by tom for enalbe/disable irq
#ifdef CONFIG_PF400CG	
	&dev_attr_dump.attr,				//add by tom for dump message
#endif
	&dev_attr_ps_ct.attr,				//add by tom for change Calibration Threshold
	NULL
};

static const struct attribute_group px3003b_attr_group = {
	.attrs = px3003b_attributes,
};

// add by Tom for check Sensor status
static int CheckSensorOpen(void)
{
	int val;
	val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_CONFIG_CONFIG);
	if (val < 0) {
		if(PS_MESSAGE) printk("[%s] : Get Sensor Status Failed !! (%d) \n",__FUNCTION__, val);
		if(PS_MESSAGE) printk("[%s] : i2c_smbus_read_byte_data(%x,%x) \n",__FUNCTION__,ps_sensor_data->client->addr, ADDR_PX_CONFIG_CONFIG);
	}
	val &= 0x03; // 0x03 = 0000 0011 = Bit 1 2 (Device Enable(01) / Disable (11))

	if(val == 0x01){
		if(PS_MESSAGE) printk("[%s] : Sensor OPEN! \n",__FUNCTION__);
		return 1;
	}else if(val == 0x03){
		if(PS_MESSAGE) printk("[%s] : Sensor CLOSE! \n",__FUNCTION__);
		return 0;
	}
	if(PS_MESSAGE) printk("[%s] : Check Failed! \n",__FUNCTION__);
	return -1;
}
// add by Tom for check factory mode
static int CheckFactoryOpen(void)
{
	// check factory mode 
	int mode = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE);

	if(mode == 0xe0){
		if(PS_MESSAGE) printk("[%s] : Factory mode : ON ! \n",__FUNCTION__);
		return 1;
	}else{
		if(PS_MESSAGE) printk("[%s] : Factory mode : OFF ! \n",__FUNCTION__);
		return 0;
	}
}

/* Calibration Function ++ */
static int px3003b_calibration(int iControl)
{
	int ThresholdValue = Default_PS_Cali_Thres , ret =0 ;
	int RAW_Data=0;
	int retry = 3;
	struct file *fp=NULL;
	mm_segment_t old_fs;
    char PS_CValue[Calibration_length]={0};
	
	ret = px3003b_get_crosstalk_value();
	if(ret<0){
		if(PS_MESSAGE) printk( "[%s]	Read PS CrossTalk CalibrationValue Failed (%d)\n",__FUNCTION__,ret);
			return ret;
	}

	if(iControl==0){
		ThresholdValue = Default_PS_ThresholdValue;
	}else{
		// Check Factory Mode
		int mode = CheckFactoryOpen();
		int retryCount = 0;
		do{
			if(mode == 0){
				ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0xe0); // change to factory mode 
				if (ret){
					if(PS_MESSAGE) printk("[%s] : Change to factory mode Failed!!\n",__FUNCTION__);
					return ret;
					msleep(300);
					}
				}
			mode =	CheckFactoryOpen();
			retryCount++;
			if(retryCount > 10)
			{
				if(PS_MESSAGE) printk("[%s] : Retry to factory mode Failed!!\n",__FUNCTION__);
				return -1;
			}
		}while(mode==0);
		CheckSensorOpen();
		// Read Raw Data
		retryCount = 0;
		do{
			RAW_Data=px3003b_get_adc_counter();
			retryCount++;
			if(retryCount > 10)
			{
				if(PS_MESSAGE) printk("[%s] : Retry to read RAW Data Failed!! RAW_Data = 0\n",__FUNCTION__);
				return -1;
			}
			msleep(300);
		}while(RAW_Data==0);

		// Check Gray card 4cm Raw Data > Crosstalk +  4 ++ & retry  
		while(retry > 0)
		{
			if(RAW_Data >= (PS_CrossTalk+ThresholdValue)){
				if(PS_MESSAGE) printk("[%s] PASS! Gray card 4cm Raw Data (%d) > Crosstalk(%d) + %d \n",__FUNCTION__,RAW_Data,PS_CrossTalk,ThresholdValue);
				retry = -1;
				break;
			}
			else{
				if(PS_MESSAGE) printk("[%s] ========================retry %d ========================== \n",__FUNCTION__,retry);
				if(PS_MESSAGE) printk("[%s] Failed ! Gray card 4cm Raw Data (%d) < Crosstalk(%d) + %d \n",__FUNCTION__,RAW_Data,PS_CrossTalk,ThresholdValue);
				if(PS_MESSAGE) printk("[%s] =========================================================== \n",__FUNCTION__);
				msleep(50);
				RAW_Data=px3003b_get_adc_counter();
			}
			retry--;
			if(retry == 0)
			{
				return -10;
			}
			
		}
		// Check Gray card 4cm Raw Data > Crosstalk +  4 --

		/*
		if(PS_MESSAGE) printk("[%s] : PS_RAW_DATA = %d (Before Subtract CrossTalk )\n",__FUNCTION__,RAW_Data);
		ThresholdValue = (int) RAW_Data - PS_CrossTalk;
		if(ThresholdValue < 0)
		{
			if(PS_MESSAGE) printk("[%s] : Threshold Value - CrossTalk Value < 0!!\n",__FUNCTION__);
			if(PS_MESSAGE) printk("[%s]	PS Use Default Calibration Value \n",__FUNCTION__);
			ThresholdValue=Default_PS_ThresholdValue;
		}
		*/
		// Close Factory mode
		ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0x00); // change to factory mode
		if (ret){
			if(PS_MESSAGE) printk("[%s] : Close factory mode Failed!!\n",__FUNCTION__);
		}
	}

	if(PS_MESSAGE) printk("[%s] : PS_Threshold = %d (Before Calibration)\n",__FUNCTION__,PS_ThresholdValue);
		
	sprintf(PS_CValue,"%010d&%010d",ThresholdValue, ThresholdValue); // &%010d reserved
	
	fp=filp_open(PS_CALIBRATION_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if (IS_ERR_OR_NULL(fp)){
		if(PS_MESSAGE) printk("[%s]		File Open Failed \n",__FUNCTION__);
		return -ENOENT;
	}
	if(fp->f_op != NULL && fp->f_op->write != NULL){
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		
		fp->f_op->write(fp,PS_CValue,Calibration_length,&fp->f_pos);	
		
		set_fs(old_fs);
	}
	filp_close(fp,NULL);
	
	if(PS_MESSAGE) printk("[%s]			Save Calibration Value (%21s)\n",__FUNCTION__,PS_CValue);	
	
	return 0; 
}
static int px3003b_crosstalk_proximity(void)
{
	int ret=0,err=0, iRetry=CalibrationRetryTimes;
	if(PS_AlreadyCrossTalk==FALSE){
		if(unlikely(debug))printk( "[%s]		PS Crosstalk Retry %d times.\n",__FUNCTION__,(iRetry-PS_CrosstalkRetryCount));
		PS_CrosstalkRetryCount --;
		
		ret = px3003b_get_crosstalk_value();
		if(ret<0){
			if(PS_MESSAGE) printk( "[%s]	PS Use Default Crosstalk Value \n",__FUNCTION__);
			err = px3003b_set_threshold(Default_PS_ThresholdValue+PS_CrossTalk);
			if(err < 0)  {
				if(PS_MESSAGE) printk( "[%s]	ps_set_threshold (%d) Failed \n",__FUNCTION__,PS_ThresholdValue+PS_CrossTalk);
				return err;
			}
			return ret;
			
		}else{
			if(PS_MESSAGE) printk( "[%s]	PS Crosstalk Value = (%d) \n",__FUNCTION__,PS_CrossTalk);

			ret = px3003b_set_threshold(Default_PS_Cali_Thres+PS_CrossTalk);
			if(ret < 0)  {
				if(PS_MESSAGE) printk( "[%s]	ps_set_threshold (%d) Failed \n",__FUNCTION__,PS_ThresholdValue+PS_CrossTalk);
				return ret;
			}
			PS_AlreadyCrossTalk=TRUE;
		}
	}
	return 0;
}
static int px3003b_calibration_proximity(void)
{
	int ret=0, iRetry=CalibrationRetryTimes;
	if(PS_AlreadyCalibration==FALSE){
		if(unlikely(debug))printk( "[%s]		PS Calibration Retry %d times.\n",__FUNCTION__,(iRetry-PS_CalibrationRetryCount));
		PS_CalibrationRetryCount --;
		
		ret = px3003b_get_calibration_value();
		if(ret<0){
			if(PS_MESSAGE) printk( "[%s]	PS Use Default Calibration Value \n",__FUNCTION__);
			return ret;
			
		}else{
			if(PS_MESSAGE) printk( "[%s]	PS Calibration Value = (%d) \n",__FUNCTION__,PS_ThresholdValue);

			ret = px3003b_set_threshold(Default_PS_Cali_Thres+PS_CrossTalk);
			if(ret < 0)  {
				if(PS_MESSAGE) printk( "[%s]	ps_set_threshold (%d) Failed \n",__FUNCTION__,Default_PS_Cali_Thres+PS_CrossTalk);
				return ret;
			}
			PS_AlreadyCalibration=TRUE;
		}
	}
	return 0;
}
static int px3003b_get_calibration_value(void)
{
	int length =0,i=0;
	char PS_CValue[Calibration_length] = {0};
	char PS_Threshold[Calibration_length] = {0};
	int NewThreshold =0;
	struct file *fp=NULL;
	mm_segment_t old_fs;
	
	fp=filp_open(PS_CALIBRATION_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if (IS_ERR_OR_NULL(fp)){	
		if(PS_MESSAGE) printk("[%s]	File Open Failed \n",__FUNCTION__);
		return -ENOENT;
	}
	
	if(fp->f_op != NULL && fp->f_op->read != NULL){
		
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		length = fp->f_op->read(fp,PS_CValue,Calibration_length,&fp->f_pos);
		if(unlikely(debug))printk("[%s]		fp->f_op->read (length = %d) \n",__FUNCTION__,length);

		set_fs(old_fs);		
	}
	filp_close(fp,NULL);

	if(length!=Calibration_length){
		if(PS_MESSAGE) printk("[%s]		Can Not Find PS Calibration Value (length = %d)\n",__FUNCTION__,length);
		return -EBADMSG;	
	}

	for(i=0;i<Threshold_length;i++){
			PS_Threshold[i]=PS_CValue[i];
	}
	NewThreshold = CharToInt(PS_Threshold);

	if(NewThreshold > 0){
		PS_ThresholdValue = NewThreshold;
		if(PS_MESSAGE)printk("[%s]	PS_ThresholdValue = (%d) \n",__FUNCTION__,PS_ThresholdValue);
	}else{
		if(PS_MESSAGE) printk("[%s]	PS Calibration Failed \n",__FUNCTION__);
		return -EBADMSG;
	}
	return 0;
}

static int px3003b_set_threshold(int threshold)
{
	int NowThreshold = 0 , ret = 0;
	if(threshold>31){
		NowThreshold = 31;
		if(PS_MESSAGE) printk( "[%s]		threshold > 31 Out of range , Threshold=%d \n",__FUNCTION__,(int)NowThreshold);
	}else{
		NowThreshold=threshold;
	}
	ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL, (NowThreshold+0x40)); // 0x40 = 0100 0000 PS Accuracy
	if (ret<0){
			return -1;
	}
	NowThreshold = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_PS_CONTROL);
	if (NowThreshold<0){
		if(PS_MESSAGE) printk( "[%s]	Read PS threshold Failed\n",__FUNCTION__);
		return -1;
	}
	NowThreshold -= 0x40; 
	if(PS_MESSAGE) printk( "[%s]		ps_set_threshold, Threshold=%d \n",__FUNCTION__,(int)NowThreshold);
	return 0;
}

 /* Calibration Function -- */

/* Crosstalk Function ++ */ 
static int px3003b_get_crosstalk_value(void)
{
	int length =0,i=0,PS_CTValue=0;
	char PS_VCValue[Calibration_length] = {0};
	char PS_VCValue_real[Calibration_length] = {0};

	struct file *fp=NULL;
	mm_segment_t old_fs;

	
	fp=filp_open(PS_CROSSTALK_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if (IS_ERR_OR_NULL(fp)){	
		if(PS_MESSAGE) printk("[%s]	File Open Failed \n",__FUNCTION__);
		return -ENOENT;
	}
	
	if(fp->f_op != NULL && fp->f_op->read != NULL){
		
		old_fs = get_fs();
		set_fs(KERNEL_DS);

		length = fp->f_op->read(fp,PS_VCValue,Calibration_length,&fp->f_pos);
		if(unlikely(debug))printk("[%s]		fp->f_op->read (length = %d) \n",__FUNCTION__,length);

		set_fs(old_fs);		
	}
	filp_close(fp,NULL);

	if(length!=Calibration_length){
		if(PS_MESSAGE) printk("[%s]		Can Not Find PS CrossTalk Value (length = %d)\n",__FUNCTION__,length);
		return -EBADMSG;	
	}

	for(i=0;i<Threshold_length;i++){	
			PS_VCValue_real[i]=PS_VCValue[i];
	}
	PS_CTValue = CharToInt(PS_VCValue_real);

	if(PS_CTValue >= 0){
		PS_CrossTalk = PS_CTValue;
		if(PS_MESSAGE)printk("[%s]	PS_CrossTalk = (%d) \n",__FUNCTION__,PS_CrossTalk);
	}else{
		if(PS_MESSAGE) printk("[%s]	PS CrossTalk Failed \n",__FUNCTION__);
		return -EBADMSG;
	}
	return PS_CTValue;
}
static int px3003b_crosstalk(void)
{
    int i=0,Sum=0,ret=0,mode=0;
	unsigned long RAW_Data=0;
	struct file *fp=NULL;
	mm_segment_t old_fs;
	char PS_VCValue[Calibration_length] = {0};

	// Check Factory Mode
	int retryCount = 0;
	mode = CheckFactoryOpen();
	do{
			if(mode == 0){
				ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0xe0); // change to factory mode 
				if (ret){
					if(PS_MESSAGE) printk("[%s] : Change to factory mode Failed!!\n",__FUNCTION__);
					return ret;
					msleep(300);
					}
				}
			mode =	CheckFactoryOpen();
			retryCount++;
			if(retryCount > 10)
			{
				if(PS_MESSAGE) printk("[%s] : Retry to factory mode Failed!!\n",__FUNCTION__);
				return -1;
			}
	}while(mode==0);
		
	CheckSensorOpen();
	//Read PS data 20 times to get average cross talk value
	for(i=0;i<21;i++){
		RAW_Data=i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_DATA);
		if(RAW_Data<0)  {
			if(PS_MESSAGE) printk( "[%s]		Read PS_DATA Failed\n",__FUNCTION__);
			return -1;
		}
		RAW_Data &= 0x7f; //0x7f = 0111 1111
		
		if(i!=0)if(PS_MESSAGE) printk( "[%s]		PS DATA (%d) = %d\n",__FUNCTION__,i,(int)RAW_Data);
		if(i!=0)Sum=Sum+(int)RAW_Data;
		msleep(150);
	}
	
	PS_CrossTalk=Sum/20;

	if(PS_MESSAGE) printk( "[%s]		PS Cross Talk = %d\n",__FUNCTION__,PS_CrossTalk);


	// Close Factory mode
	ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0x00); // change to factory mode
	if (ret){
		if(PS_MESSAGE) printk("[%s] : Close factory mode Failed!!\n",__FUNCTION__);
	}

	// add by tom for check leakage of light
	if(PS_CrossTalk > limit_PS_CrossTalkValue)
	{
		printk( "[%s]		=====================================\n",__FUNCTION__);
		printk( "[%s]		PS_CrossTalk (%d) > limit_PS_CrossTalkValue (%d) , Failed ! \n",__FUNCTION__,PS_CrossTalk,limit_PS_CrossTalkValue);
		printk( "[%s]		Devices maybe leakage of light ! \n",__FUNCTION__);
		printk( "[%s]		=====================================\n",__FUNCTION__);
		return -2;
	}
	
	// Check the crosstalk value
	//if(PS_CrossTalk>limit_PS_CrossTalkValue)return -EDOM;

	// Add to Threshold
	ret = px3003b_set_threshold(PS_ThresholdValue+PS_CrossTalk);
	if(ret < 0)  {
		if(PS_MESSAGE) printk( "[%s]	px3003b_set_threshold (Threshold(%d)+PS_CrossTalk(%d)) Failed \n",__FUNCTION__,PS_ThresholdValue,PS_CrossTalk);
		return ret;
	}
	if(PS_MESSAGE) printk( "[%s]	px3003b_set_threshold (Threshold(%d)+PS_CrossTalk(%d)) \n",__FUNCTION__,PS_ThresholdValue,PS_CrossTalk);

	
	//Backup cross talk value in data/sensors/ps_crosstalk.ini
	sprintf(PS_VCValue,"%010d&%010d",PS_CrossTalk, 0);
	
	fp=filp_open(PS_CROSSTALK_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if (IS_ERR_OR_NULL(fp)){
		if(PS_MESSAGE) printk("[%s]		File Open Failed \n",__FUNCTION__);
		return -ENOENT;
	}
	if(fp->f_op != NULL && fp->f_op->write != NULL){
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		
		fp->f_op->write(fp,PS_VCValue,Calibration_length,&fp->f_pos);	
		
		set_fs(old_fs);
	}
	filp_close(fp,NULL);
	//PS_AlreadyCrossTalk=TRUE;
	return PS_CrossTalk;
}
/* Crosstalk Function -- */ 

// internal interrupt function ++
static void px3003b_workqueue_function(struct work_struct *work)
{
	int PS_Status = 0 , ret = 0;
	// check factory mode ++
	int mode = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE);
	// add by tom to check crosstalk ++
//	printk( "alp.D : px3003b_workqueue_function +");
	if((PS_AlreadyCrossTalk==FALSE) && (PS_CrosstalkRetryCount>0)){
		ret = px3003b_crosstalk_proximity();
		if(ret < 0){
			if(PS_MESSAGE) printk("[%s] : px3003b_crosstalk_proximity fail !! ret = (%d)\n",__FUNCTION__,ret);
		}
	}
	// add by tom to check crosstalk --
	if(mode == 0xe0){
		// Close factory mode
		int ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0x00);
		if (ret){
			if(PS_MESSAGE) printk("[%s] : px3003b_set_factory_mode fail !! Enable irq\n",__FUNCTION__);
			//wake_unlock(&ps_sensor_data->wake_lock);
			enable_irq(ps_sensor_data->irq);	
			return;
		}
		mdelay(300);
	}
	// check factory mode --
	
	PS_Status = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_DATA);
	PS_Status &= 0x80; /* 0x80 = 1000 0000 = Bit 7 (PS DATA)*/
	PS_Status = PS_Status >> 7;  	/* Bit 7 shift to Bit 1*/

	// add by tom for check PS_CurrentStatus 'Repeat'
	if(PS_CurrentStatus != PS_Status)
	{
		PS_CurrentStatus=PS_Status;

		if(PS_CurrentStatus){
			input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE, 0);
			input_sync(ps_sensor_data->input_dev);
			if(PS_DEBUGMSG) printk( "[%s]		PS Report 'Close' \n",__FUNCTION__);
		}else{
			input_report_abs(ps_sensor_data->input_dev, ABS_DISTANCE, 9);
			input_sync(ps_sensor_data->input_dev);
			if(PS_DEBUGMSG) printk( "[%s]		PS Report 'Away' \n",__FUNCTION__);

		}
	}
	else
	{
		if(PS_DEBUGMSG) printk( "[%s]		PS_CurrentStatus 'Repeat' \n",__FUNCTION__);
	}
//	printk( "alp.D : px3003b_workqueue_function -");
	//wake_unlock(&ps_sensor_data->wake_lock);
	
	enable_irq(ps_sensor_data->irq);	

	return;
}

static irqreturn_t ps_interrupt(int irq, void *dev_id)
{
	int ret = 0;	
	wake_lock_timeout(&ps_sensor_data->wake_lock,1*HZ);
	disable_irq_nosync(irq);
	ret = queue_work(px3003b_struct_wq, &px3003b_workq);
	if(PS_DEBUGMSG) printk( "[%s]		PS Disable irq. Enter work queue return (%d) \n",__FUNCTION__,ret);
	if(ret == 0)
	{
		if(PS_DEBUGMSG) printk( "[%s]		Enter work queue FAIL , return (%d) \n",__FUNCTION__,ret);
		enable_irq(ps_sensor_data->irq);	
	}
	return IRQ_HANDLED;
}
// internal interrupt  function --

static int px3003b_open(struct inode *inode, struct file *file)
{
	if(unlikely(debug)) printk(KERN_INFO "[%s]\n", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int px3003b_release(struct inode *inode, struct file *file)
{
	if(unlikely(debug)) printk(KERN_INFO "[%s]\n", __FUNCTION__);
	return 0;
}


static long px3003b_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int control=0, err=0, val = 0, ret = 0, mode = 0, retryCount = 0;
#ifdef CONFIG_PF400CG
	if(unlikely(debug))printk( "[%s]		===========Enter PX3003B IOTCL ========== \n",__FUNCTION__);
	if(unlikely(debug))printk( "[%s]		PX3003B IOCTL (Cmd is %d) \n",__FUNCTION__, _IOC_NR(cmd));
	if(unlikely(debug))printk( "[%s]		PX3003B Before Enter IOCTL IsLinkedPad = %d\n",__FUNCTION__,IsLinkedPad);
	if(unlikely(debug))printk( "[%s]		PX3003B Before Enter IOCTL IsEnableSensor = %d\n",__FUNCTION__,IsEnableSensor);

	if(IsLinkedPad == 1)
	{
		switch(cmd)
		{
			case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
				if(unlikely(debug)) printk("[%s]		PX3003B PROXIMITYSENSOR_IOCTL_GET_ENABLED (return %d) \n",__FUNCTION__,ps_sensor_data->px3003b_state);
				return put_user(ps_sensor_data->px3003b_state, (unsigned long __user *)arg);
			break;
			
			case PROXIMITYSENSOR_IOCTL_ENABLE:
				if (get_user(val, (unsigned long __user *)arg))
					return -EFAULT;
				if (val){
					IsEnableSensor= 1;
					if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_ENABLE (return IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
				}else{
					IsEnableSensor= 0;
					if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_ENABLE (return IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
				}
				return 0;
			break;	//LIGHTSENSOR_IOCTL_ENABLE

			default:
				if(unlikely(debug))printk( "[%s]		PX3003B Other Cmd in IsLinkedPad == 1 (%d) \n",__FUNCTION__, _IOC_NR(cmd));
		}
		return 0;
		
	}else{
#endif
	switch(cmd)
	{
		case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
			if(PS_DEBUGMSG) printk("[%s]		PX3003B PROXIMITYSENSOR_IOCTL_GET_ENABLED  (return %d)\n",__FUNCTION__,ps_sensor_data->px3003b_state);
			return put_user(ps_sensor_data->px3003b_state, (unsigned long __user *)arg);
			break;

		case PROXIMITYSENSOR_IOCTL_ENABLE:
			if (get_user(val, (unsigned long __user *)arg))
				return -EFAULT;
			if (val){
				if(PS_DEBUGMSG)printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_ENABLE - ps_enable \n",__FUNCTION__);
#ifdef CONFIG_PF400CG
				IsEnableSensor= 1;
				if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_ENABLE (return IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
#endif				
				return px3003b_enable(1);
			}else{
				if(PS_DEBUGMSG)printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_ENABLE - ps_disable \n",__FUNCTION__);
#ifdef CONFIG_PF400CG
				IsEnableSensor= 0;
				if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_ENABLE (return IsEnableSensor = %d) \n",__FUNCTION__,IsEnableSensor);
#endif				
				return px3003b_enable(0);
			}
			break;	//LIGHTSENSOR_IOCTL_ENABLE
			
		case PROXIMITYSENSOR_IOCTL_GET_STATUS:
			// Check Sensor Open
			mode = CheckSensorOpen();
			if(mode == 0)
			{
				err = px3003b_enable(1); 
				if(err < 0)  {
					if(PS_MESSAGE) printk( "[%s]		PX3003B px3003b_enable(%d) Failed \n",__FUNCTION__,1);
					return err;
				}
				
			}
			msleep(300);
			mode = CheckFactoryOpen();
			while(mode!=0){
					if(unlikely(debug))printk( "[%s]		PX3003B Change to interrput mode\n",__FUNCTION__);
					if(mode != 0){
						ret = i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_FACTORY_MODE, 0x00); // change to interrput mode 
						if (ret){
							if(PS_MESSAGE) printk("[%s] : Change to factory mode Failed!!\n",__FUNCTION__);
							return ret;
							msleep(300);
							}
						}
					mode =	CheckFactoryOpen();
					retryCount++;
					if(retryCount > 10)
					{
						if(PS_MESSAGE) printk("[%s] : Retry to factory mode Failed!!\n",__FUNCTION__);
						return -10;
					}
			};
			msleep(300);
			val = i2c_smbus_read_byte_data(ps_sensor_data->client, ADDR_PX_DATA);
			if(unlikely(debug))printk( "[%s]		PX3003B ADDR_PX_DATA =   %d\n",__FUNCTION__,val);
			val &= 0x80; /* 0x80 = 1000 0000 = Bit 7 (PS DATA)*/
			ps_sensor_data->px3003b_state= val >> 7;/* Bit 7 shift to Bit 1*/
			if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_GET_STATUS (return %d)\n",__FUNCTION__,ps_sensor_data->px3003b_state);
			return put_user(ps_sensor_data->px3003b_state, (unsigned long __user *)arg); 

		break;

		case PROXIMITYSENSOR_IOCTL_CALIBRATION:
			// Disable irq for calibration 
			disable_irq(ps_sensor_data->irq);
			
			if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_CALIBRATION (%d)\n",__FUNCTION__,control);
			if (get_user(control, (unsigned long __user *)arg))
			{
				enable_irq(ps_sensor_data->irq);
				return -EFAULT;
			}
			// Check Sensor Open
			ret = CheckSensorOpen();
			if(ret == 0)
			{
				err = px3003b_enable(1); 
				if(err < 0)  {
					if(PS_MESSAGE) printk( "[%s]		PX3003B px3003b_enable(%d) Failed \n",__FUNCTION__,1);
					enable_irq(ps_sensor_data->irq);
					return err;
				}
				msleep(300);
			}
			
			
			err = px3003b_calibration(control);
			if(err<0){
				if(PS_MESSAGE) printk( "[%s]		PX3003B  Calibration (%d)(set calibration value) Failed (%d)\n",__FUNCTION__,control,err);
				enable_irq(ps_sensor_data->irq);
				return err;
			}	

			PS_AlreadyCalibration=FALSE;
			PS_CalibrationRetryCount=CalibrationRetryTimes;
			err = px3003b_calibration_proximity();
			if(err<0){
				if(PS_MESSAGE) printk( "[%s]		PX3003B  Calibration (%d)(get calibration value) Failed (%d)\n",__FUNCTION__,control,err);
			}
			// Enable irq for calibration 
			enable_irq(ps_sensor_data->irq);

		break;

		case PROXIMITYSENSOR_IOCTL_VCALIBRATION:
			// Disable irq for calibration 
			disable_irq(ps_sensor_data->irq);

			if(unlikely(debug))printk( "[%s]		PX3003B PROXIMITYSENSOR_IOCTL_VCALIBRATION \n",__FUNCTION__);
			PS_AlreadyCrossTalk=TRUE;

			// Check Sensor Open
			ret = CheckSensorOpen();
			if(ret == 0)
			{
				err = px3003b_enable(1);
				if(err < 0)  {
					if(PS_MESSAGE) printk( "[%s]	px3003b_enable(%d) Failed \n",__FUNCTION__,1);
					enable_irq(ps_sensor_data->irq);
					return err;
				}
				msleep(300);
			}

			
			err = px3003b_crosstalk();
			if(err< 0)  {
				if(PS_MESSAGE) printk( "[%s]	PS CrossTalk Failed (%d)\n",__FUNCTION__,err);
				enable_irq(ps_sensor_data->irq);
				return err;
			}

			PS_AlreadyCrossTalk=FALSE;
			PS_CrosstalkRetryCount=CalibrationRetryTimes;

			err = px3003b_get_crosstalk_value();
			if(err<0){
				if(PS_MESSAGE) printk( "[%s]	Read PS CrossTalk CalibrationValue Failed (%d)\n",__FUNCTION__,err);
				enable_irq(ps_sensor_data->irq);
				return err;
			}

			printk( "[%s]		PX3003B  Crosstalk Value = %d\n",__FUNCTION__,PS_CrossTalk);
			// Enable irq for calibration
			enable_irq(ps_sensor_data->irq);
			return put_user(PS_CrossTalk, (unsigned long __user *)arg);
			
		break;
		

		default:   //redundant, as cmd was checked against MAXNR 
			if(PS_DEBUGMSG) printk("[%s]		PX3003B  : ERROR IOCTL CONTROL COMMAND!!!\n",__FUNCTION__);
		return -ENOTTY;
	}
#ifdef CONFIG_PF400CG
}
#endif
	return 0;
}


static struct file_operations px3003b_fops = {
	.owner			= 	THIS_MODULE,
	.unlocked_ioctl	=	px3003b_ioctl,
	.open			=	px3003b_open,
	.release			=	px3003b_release,
};

static struct miscdevice px3003b_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = PX3003B_DRV_NAME,
	.fops = &px3003b_fops,
};

static int px3003b_probe(struct i2c_client *client, const struct i2c_device_id *id)
{


	struct px3003b_i2c_platform_data *pdata;
	int err = 0;
	if(PS_MESSAGE) printk("==================Start px3003b_probe ==========================\n");	
#ifdef CONFIG_PF400CG
	err =register_microp_notifier(&ps_notifier);
	if(err< 0){
		if(PS_MESSAGE) printk( "[%s]		register_microp_notifier Failed (%d)\n",__FUNCTION__,err);
	}
#endif	

	ps_sensor_data = kzalloc(sizeof(struct px3003b_data), GFP_KERNEL);
	if (!ps_sensor_data)
	{
		printk(KERN_ERR "alp : px3003b allocate px3003b_data failed.\n");
		return -ENOMEM;
	}
	ps_sensor_data->client = client;
	i2c_set_clientdata(client, ps_sensor_data);
	pdata = client->dev.platform_data;
	ps_sensor_data->int_gpio = pdata->int_gpio;	
	if(PS_MESSAGE) printk("[%s] : pdata->int_gpio = %d \n",__FUNCTION__,pdata->int_gpio);
	ps_sensor_data->px3003b_state = 0;	//init state.
	if(PS_MESSAGE) printk("[%s] : client->addr = %x \n",__FUNCTION__,client->addr);
	if(PS_MESSAGE) printk("[%s] : ps_sensor_data->client->addr = %x \n",__FUNCTION__,ps_sensor_data->client->addr);
	if(PS_MESSAGE) printk(KERN_INFO "[%s] : px3003b registered I2C driver!\n",__FUNCTION__);	
	wake_lock_init(&ps_sensor_data->wake_lock, WAKE_LOCK_SUSPEND, "PS_suspend_blocker");
 
	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &px3003b_attr_group);
	if (err)
		goto exit_sysfs_create_group_failed;
	
	err = misc_register(&px3003b_device);
	if (err) {
		if(PS_MESSAGE) printk(KERN_ERR "[%s] : px3003b register failed\n",__FUNCTION__);
		goto exit_misc_device_register_failed;
	}
	
	mutex_init(&ps_sensor_data->lock);
	/* Initialize Setting */
	//px3003b_create_proc_file();
	//if(PS_MESSAGE) printk("[%s] : px3003b_create_proc_file() : debug = %d \n",__FUNCTION__,debug);

	// init input subsystem	
	err = px3003b_input_init();
	if (err)
	{
		printk(KERN_ERR "[%s] :  px3003b_input_init failed.\n",__FUNCTION__);
		goto exit_kfree;
	}



	// init interrupt pin	
	err = gpio_request(ps_sensor_data->int_gpio, "px3003b-irq");
	if(err < 0){
		printk(KERN_ERR "[%s] : px3003b Failed to request GPIO%d (px3003b-irq) error=%d\n",__FUNCTION__, ps_sensor_data->int_gpio, err);
		printk(KERN_ERR "[%s] : gpio_request(%d,px3003b-irq)\n",__FUNCTION__, ps_sensor_data->int_gpio);
	}
	err = gpio_direction_input(ps_sensor_data->int_gpio);
	if (err){
		printk(KERN_ERR "[%s] : px3003b Failed to set interrupt direction, error=%d\n",__FUNCTION__, err);
		printk(KERN_ERR "[%s] : gpio_direction_input(%d)\n",__FUNCTION__, ps_sensor_data->int_gpio);
		gpio_free(ps_sensor_data->int_gpio);
	}
	ps_sensor_data->irq = gpio_to_irq(ps_sensor_data->int_gpio);
	if(PS_MESSAGE) printk(KERN_INFO "[%s] : px3003b irq = %d\n",__FUNCTION__,ps_sensor_data->irq);
	if(ps_sensor_data->irq > 0){
		err = request_threaded_irq(ps_sensor_data->irq, NULL,ps_interrupt, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"px3003b_interrupt",NULL);
	  	if (err) {
			dev_err(&client->adapter->dev,"cannot register IRQ %d,err:%d\n",ps_sensor_data->irq,err);
			printk("[%s] : request_threaded_irq(%d, NULL, ps_interrupt, IRQF_TRIGGER_FALLING ,px3003b_interrupt ,NULL)\n",__FUNCTION__, ps_sensor_data->irq);
		}
	}
	
	// Workqueue Setting
	px3003b_struct_wq = create_singlethread_workqueue("px3003b_wq");
	if (!px3003b_struct_wq)
	{
		if(PS_MESSAGE) printk("[%s]	Create WorkQueue Failed\n",__FUNCTION__);
		goto exit_kfree;
	}
	INIT_WORK(&px3003b_workq, px3003b_workqueue_function);
	
	// add by tom for check crosstalk 
	err = px3003b_crosstalk_proximity();
	if(err < 0){
		if(PS_MESSAGE) printk("[%s] : px3003b_crosstalk_proximity fail !! ret = (%d)\n",__FUNCTION__,err);
	}
	
	err = px3003b_enable(0);
	if(err < 0)  {
			if(PS_MESSAGE) printk( "[%s]	px3003b_enable(%d) Failed \n",__FUNCTION__,0);
			return err;
	}
	
	// add by tom for disable irq for A400CG (SR1 & SR2) ++
	if((Read_PROJ_ID() == PROJ_ID_A400CG) || (Read_PROJ_ID() == PROJ_ID_A450CG)){
		if((Read_HW_ID() == HW_ID_SR1) || (Read_HW_ID() == HW_ID_SR2))
		{
			if(build_version!=1){	// 1:eng ; 2:user ; 3:userdebug 
 				printk("[%s] : Disable irq because HW_ID is SR1 or SR2\n",__FUNCTION__);
				disable_irq(ps_sensor_data->irq);
				IsDisableIrq=1;
			}			
		}
	}    
    // add by tom for disable irq for A400CG (SR1 & SR2) --
	
	if(PS_MESSAGE) printk("==================End px3003b_probe ==========================\n");

	return 0;

exit_misc_device_register_failed:
exit_sysfs_create_group_failed:
exit_kfree:
	wake_lock_destroy(&ps_sensor_data->wake_lock);
	kfree(pdata);
	kfree(ps_sensor_data);
	return err;
}

static int px3003b_remove(struct i2c_client *client)
{
	struct px3003b_data *data;
	u8 reg_val = 0x00;

	// power down chip at remove
	i2c_smbus_write_byte_data(ps_sensor_data->client, ADDR_PX_CONFIG_CONFIG, reg_val);
	ps_sensor_data->px3003b_reg_status[INDEX_PX_CONFIG_CONFIG] = reg_val;	
	//px3003b_remove_proc_file();
	sysfs_remove_group(&client->dev.kobj, &px3003b_attr_group);
	data = i2c_get_clientdata(client);
	px3003b_i2c_client = NULL;
	wake_lock_destroy(&ps_sensor_data->wake_lock);
	kfree(data);

	return 0;
}

static int px3003b_suspend(struct i2c_client *client , pm_message_t mesg)
{
	printk("alp :  px3003b_suspend\n");
	return 0;
}

static int px3003b_resume(struct i2c_client *client)
{
	printk("alp :  px3003b_resume\n");
	return 0;
}

struct i2c_device_id px3003b_i2c_table[] = {
	{ "px3003b", 0 },
	{}
}

MODULE_DEVICE_TABLE(i2c, px3003b_i2c_table);

static struct i2c_driver px3003b_i2c_driver = {
	.driver = {
		.name  = PX3003B_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.suspend	=	px3003b_suspend,
	.resume	=	px3003b_resume,
	.probe	=	px3003b_probe,
	.remove	=	px3003b_remove,
	.id_table	=	px3003b_i2c_table,	
};

static int __init px3003b_init(void)
{
	int ret;	
#ifdef CONFIG_PF400CG
	if(Read_PROJ_ID() == PROJ_ID_PF400CG){
		if((Read_HW_ID() == HW_ID_SR1))
			{
 				printk("[%s] PF400CG-A12 P-sensor function don't work now!!\n",__FUNCTION__);
				 return 0;
			}
	}
#endif		
	printk("alp :  px3003b_init +\n");
	ret = i2c_add_driver(&px3003b_i2c_driver);
	if ( ret != 0 ) {
		if(PS_DEBUGMSG) printk(KERN_ERR "[%s]alp : can not add i2c driver\n",__FUNCTION__);
		return ret;
	}
	printk("alp :  px3003b_init -\n");
	return ret;
}

static void __exit px3003b_exit(void)
{
	i2c_del_driver(&px3003b_i2c_driver);
}

MODULE_AUTHOR("Alp kao");
MODULE_DESCRIPTION("Dyna IMAGE px3003b driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(px3003b_init);
module_exit(px3003b_exit);
