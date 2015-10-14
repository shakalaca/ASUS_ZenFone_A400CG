#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/kobject.h>
#include <linux/input.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <asm/intel_scu_pmic.h>
#include <asm/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/errno.h>
// 2013.07.04 cheng_kao early_suspend ++
#include <linux/earlysuspend.h>
// 2013.07.04 cheng_kao early_suspend --

#define DRIVER_NAME "als3010"
static struct kobject *als3010_kobj;

static struct als3010_device_data {
	int lux_value;
	int adc_value;
	int enable;
	int cal_value;
	int als3010_pw_before_suspend;
	struct input_dev *input_dev;
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend als3010_early_suspend;
#endif
	struct mutex lock;
	int als_status;
};

static struct als3010_device_data *als3010_dev = NULL;


/*Proc Nodes*/
#define PROC_DEBUG_MESSAGE_NAME	"als3010_debug"
static struct proc_dir_entry *dbgProcFile;

/* Other */
static int POWER_OFF 	= 0;
static int POWER_ON 	= 1;
static int RESET_INPUT_DEVICE = -1;
static unsigned int debug_msg = 0; 

#define ALS_DEBUGMSG	1
#define ALS_VALUE_LEN	10
#define ALS_CALIBRATION_FILE_PATH		"/data/als_calibration.ini"

/* Sensors IO Control Info */
#define LIGHTSENSOR_IOCTL_MAGIC 'l'
#define LIGHTSENSOR_IOCTL_GET_ENABLED		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *) 
#define LIGHTSENSOR_IOCTL_CAL_ENABLE			_IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)
#define LIGHTSENSOR_IOCTL_ENABLE				_IOW(LIGHTSENSOR_IOCTL_MAGIC, 3, int *)
#define LIGHTSENSOR_IOCTL_GET_RAWDATA		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 4, int *)
#define LIGHTSENSOR_IOCTL_GET_LUXVALUE		_IOR(LIGHTSENSOR_IOCTL_MAGIC, 5, int *)
#define LIGHTSENSOR_IOCTL_GET_CALLUX			_IOW(LIGHTSENSOR_IOCTL_MAGIC, 6, int *)
#define LIGHTSENSOR_IOCTL_CALIBRATION			_IOW(LIGHTSENSOR_IOCTL_MAGIC, 7, int *)

//#define	ALS_LEVEL	11
//#define	ALS_LEVEL_GAP ALS_LEVEL+1
#define	ALS_LEVEL_GAP				17
#define	ALS_GET_BUFFER_LENGTH	70
#define	ALS_MAX_BUFFER_LENGTH	128
//static int als3010_threshold_level[ALS_LEVEL_GAP]	= {0,5,30,50,100,300,550,900,1100,1500,2200,4863};
//static int als3010_threshold_Conut[ALS_LEVEL_GAP] = {0, 67, 404 ,674, 1348, 4043, 7412, 12129, 14825, 20216, 29650, 65535};
static int als3010_threshold_level[ALS_LEVEL_GAP]	= {0,25,50,100,150,200,300,400,500,600,700,800,900,1000,1250,1500,4863};
static int als3010_threshold_Conut[ALS_LEVEL_GAP] = {0, 337, 674 ,1348, 2022, 2696, 4044, 5391 ,6739 , 8086, 9434, 10782, 12130, 13478, 16847, 20216, 65535};


#define	ALS_LEVEL_BUFFER_LENGTH			36
#define	ALS_THRESHOLD_BUFFER_LENGTH		38
#define	ALS_CALI_VALUE_BUFFER_LENGTH	6

#define	ALS_CALI_DEFAULT_VALUE			1600

int als3010_get_calibration(void);
int als3010_set_level_table(int calivalue);
int als3010_set_threshold_table(int calivalue);
int als3010_set_calibration_value(int calivalue);

extern unsigned int factory_mode;
static int als3010_enable(int  power);

#ifdef CONFIG_HID_ASUS_PAD_EC
extern int ite_light_sensor_set_function(int cmd);
extern int ite_light_sensor_set_table(unsigned char *buf);
extern int ite_light_sensor_get_table(unsigned char *report_buf);
#endif

//=================================================================
static ssize_t als_status_check(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	
//	ret = als3010_enable(POWER_ON);
#ifdef CONFIG_HID_ASUS_PAD_EC
	ret=ite_light_sensor_set_function(0x02);
		if(ret < 0)  {
		printk( "[%s] %d: als_status_check enable (%d) failed \n",__FUNCTION__,__LINE__,POWER_ON);
	}
#endif

	msleep(500);

//	ret = als3010_enable(POWER_OFF);
#ifdef CONFIG_HID_ASUS_PAD_EC
	ret=ite_light_sensor_set_function(0x04);
	if(ret < 0)  {
		printk( "[%s] %d: als_status_check disable (%d) failed \n",__FUNCTION__,__LINE__,POWER_OFF);
	}
#endif
	return sprintf(buf, "%d \n", als3010_dev->als_status);
}

static ssize_t als_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf,"als_enable(%d) \n",als3010_dev->enable);
}

static ssize_t als_enable_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
	int ret=0, state=0;
	
	sscanf (buf, "%d", &state);
	printk( "[%s] %d: set power status = %d\n",__FUNCTION__,__LINE__,state);
	
	if(buf[0]=='1'){
//		ret = als3010_enable(POWER_ON);
#ifdef CONFIG_HID_ASUS_PAD_EC
		ret=ite_light_sensor_set_function(0x02);
		if(ret < 0)  {
			printk( "[%s] %d: als_enable(%d) failed \n",__FUNCTION__,__LINE__,POWER_ON);
		}	
#endif
	}else if(buf[0]=='0'){
//		ret = als3010_enable(POWER_OFF);
#ifdef CONFIG_HID_ASUS_PAD_EC
		ret=ite_light_sensor_set_function(0x04);
		if(ret < 0)  {
			printk( "[%s] %d: als_enable(%d) failed \n",__FUNCTION__,__LINE__,POWER_OFF);
		}
#endif
	}
	return count;
}

static ssize_t als_start_calibration(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err=0;
	als3010_dev->cal_value = als3010_get_calibration();
	printk( "als : cal_value=%d \n",als3010_dev->cal_value);
	err= als3010_set_level_table(als3010_dev->cal_value);
	err= als3010_set_threshold_table(als3010_dev->cal_value);
	err= als3010_set_calibration_value(als3010_dev->cal_value);

	return sprintf(buf,"als_start_calibration(%d) \n",err);
}

static ssize_t als_get_all_table(struct device *dev, struct device_attribute *attr, char *buf)
{
	int err=0,iloop=0;
	unsigned char buffer[ALS_MAX_BUFFER_LENGTH] ={0};
#ifdef CONFIG_HID_ASUS_PAD_EC	
	err = ite_light_sensor_get_table(&buffer);
	for(iloop = 0; iloop < ALS_GET_BUFFER_LENGTH; iloop++){
		printk( "als : als_get_all_table buffer[%d]=%d \n",iloop ,buffer[iloop]);
	}
#endif
	return sprintf(buf,"als_get_all_table(%d) \n",err);
}


static DEVICE_ATTR(als_en, S_IWUSR | S_IRUGO , als_enable_show , als_enable_store);
static DEVICE_ATTR(als_calibration, S_IRUGO,als_start_calibration,NULL);
static DEVICE_ATTR(als_status, S_IRUGO,als_status_check,NULL);
static DEVICE_ATTR(als_get_table, S_IRUGO,als_get_all_table,NULL);

static struct attribute *als3010_attributes[] = {
	&dev_attr_als_en.attr,
	&dev_attr_als_calibration.attr,
	&dev_attr_als_status.attr,
	&dev_attr_als_get_table.attr,
	NULL
};
static const struct attribute_group als3010_attr_group = {
	.attrs = als3010_attributes,
};
//=================================================================

static ssize_t als3010_proc_debug_msg_read(char *buf, char **start, off_t off, int count, int *eof, void *data){

	return sprintf(buf, "debug_msg(%d)\n",debug_msg);
}
static ssize_t als3010_proc_debug_msg_write(struct file *filp, const char __user *buff, unsigned long len, void *data){

	char messages[80] = {0};
			
	if (len >= 80) {
		printk("[%s] %d: no command exceeds 80 chars\n",__FUNCTION__,__LINE__);
		return -EFAULT;
	}	
	if (copy_from_user(messages, buff, len))
		return -EFAULT;

	printk("[%s] %d: input command is %s\n",__FUNCTION__,__LINE__,messages);

	switch(messages[0]){
	case 0: 
		debug_msg = 0; 
		break;
	case 1: 
		debug_msg = 1; 
		break;

	default:
		printk("[%s] %d: this command is not supported\n",__FUNCTION__,__LINE__);
		return -EFAULT;
	}
	
	return len;
}

void als3010_create_proc_file(void)
{
	dbgProcFile = create_proc_entry(PROC_DEBUG_MESSAGE_NAME, 0660, NULL);
	if (dbgProcFile == NULL){
		remove_proc_entry(PROC_DEBUG_MESSAGE_NAME, NULL);
		printk("[%s] %d: can't initialize /proc/%s\n",__FUNCTION__,__LINE__,PROC_DEBUG_MESSAGE_NAME);
	}else{
		dbgProcFile->read_proc = als3010_proc_debug_msg_read;
		dbgProcFile->write_proc = als3010_proc_debug_msg_write;
		printk("[%s] %d: /proc/%s created success\n",__FUNCTION__,__LINE__,PROC_DEBUG_MESSAGE_NAME);
	}
}

void als3010_remove_proc_file(void)
{
    extern struct proc_dir_entry proc_root;
    remove_proc_entry(PROC_DEBUG_MESSAGE_NAME, &proc_root);
}
//=================================================================

int als3010_get_calibration(void)
{
	struct file *fp=NULL;
	int ret = 0;
	int ilen = 0, ivalue=0;
	char c_data[ALS_VALUE_LEN]={0};
	mm_segment_t old_fs;
	
	fp=filp_open(ALS_CALIBRATION_FILE_PATH,O_RDWR ,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		if(ALS_DEBUGMSG) printk(KERN_INFO "als3010 : filp_open fail\n");
		return 0;
	}
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	ilen = fp->f_op->read(fp,c_data,ALS_VALUE_LEN,&fp->f_pos);
	ivalue = c_data[2]*128+c_data[3];
	if(ivalue>0)
		ret = ivalue;
	else 
		ret = ALS_CALI_DEFAULT_VALUE;
	if(ALS_DEBUGMSG) printk(KERN_INFO "als3010 : get calibration ivalue = %d \n",ivalue);
	return ret;
}

int als3010_set_level_table(int calivalue)
{
	int ret=0, iloop=0 , check_value=0, table_value=0, limit=(ALS_LEVEL_BUFFER_LENGTH-4-2)/2; // -2 for 4863 not need to re-cali
	unsigned char buffer[ALS_LEVEL_BUFFER_LENGTH] ={0};
	buffer[0]=ALS_LEVEL_BUFFER_LENGTH;
	buffer[1]=0x00;
	buffer[2]=0x01;
	buffer[3]=0x00;
	for(iloop=0;iloop<limit;iloop++){
		check_value = (als3010_threshold_level[iloop+1]*1000)/calivalue;
		if( (check_value%10)>4 )
			table_value =  check_value/10+1;
		else
			table_value =  check_value/10;
		buffer[4+2*iloop]=((table_value<<8)>>8);
		buffer[5+2*iloop]=(table_value>>8);
		printk(KERN_INFO "als3010 : als3010_set_level_table[%d] = %d & %d \n",iloop,buffer[4+2*iloop],buffer[5+2*iloop]);
	}
	buffer[4+2*iloop]=0xff;
	buffer[5+2*iloop]=0x12;
	printk(KERN_INFO "als3010 : als3010_set_level_table[%d] = %d & %d \n",iloop,buffer[4+2*iloop],buffer[5+2*iloop]);
#ifdef CONFIG_HID_ASUS_PAD_EC	
	ret = ite_light_sensor_set_table(buffer);
#endif
	return ret;
}


int als3010_set_threshold_table(int calivalue)
{
	int ret=0, iloop=0 , table_value=0, limit=(ALS_THRESHOLD_BUFFER_LENGTH-4-2)/2; // -2 for 65535 not need to re-cali
	unsigned char buffer[ALS_THRESHOLD_BUFFER_LENGTH] ={0};
	buffer[0]=ALS_THRESHOLD_BUFFER_LENGTH;
	buffer[1]=0x00;
	buffer[2]=0x02;
	buffer[3]=0x00;
	for(iloop=0;iloop<limit;iloop++){
		table_value = (als3010_threshold_Conut[iloop]*100)/calivalue;
		buffer[4+2*iloop]=((table_value<<8)>>8);
		buffer[5+2*iloop]=(table_value>>8);
		printk(KERN_INFO "als3010 : als3010_set_threshold_table[%d] = %d & %d \n",iloop,buffer[4+2*iloop],buffer[5+2*iloop]);
	}
	buffer[4+2*iloop]=0xff;
	buffer[5+2*iloop]=0xff;
	printk(KERN_INFO "als3010 : als3010_set_threshold_table[%d] = %d & %d \n",iloop,buffer[4+2*iloop],buffer[5+2*iloop]);
#ifdef CONFIG_HID_ASUS_PAD_EC	
	ret = ite_light_sensor_set_table(buffer);
#endif
	return ret;
}

int als3010_set_calibration_value(int calivalue)
{
	int ret=0;
	unsigned char buffer[ALS_CALI_VALUE_BUFFER_LENGTH] ={0};
	buffer[0]=ALS_CALI_VALUE_BUFFER_LENGTH;
	buffer[1]=0x00;
	buffer[2]=0x03;
	buffer[3]=0x00;
	buffer[4]=((calivalue<<8)>>8);
	buffer[5]=(calivalue>>8);
	printk(KERN_INFO "als3010 : als3010_set_calibration_value = %d & %d \n",buffer[4],buffer[5]);
#ifdef CONFIG_HID_ASUS_PAD_EC	
	ret = ite_light_sensor_set_table(buffer);
#endif
	return ret;
}

static int als3010_enable(int  power)
{
	als3010_dev->enable= power;
	printk( "[%s] als3010_enable (%d)\n",__FUNCTION__,als3010_dev->enable);

	if(power==POWER_ON){
		//msleep(300);
		// must check if EC report a current lux value to framework
	}else{	
		input_report_abs(als3010_dev->input_dev, ABS_MISC, RESET_INPUT_DEVICE);
		input_sync(als3010_dev->input_dev);	
	}
	return 0;
}

static int als3010_open(struct inode *inode, struct file *file)
{
	printk( "[%s] als3010 misc device is opened\n",__FUNCTION__);
	return nonseekable_open(inode, file);		
}

static int als3010_release(struct inode *inode, struct file *file)
{
	printk( "[%s] als3010 misc device is released\n",__FUNCTION__);
	return 0;
}

static long als3010_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int err=0, enable=0 , value=0;
	if(unlikely(debug_msg))printk( "[%s] %d: IOCTL command is %d\n",__FUNCTION__,__LINE__,_IOC_NR(cmd));
	
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
	
	switch (cmd) {
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		if(unlikely(debug_msg))printk( "[%s] %d: LIGHTSENSOR_IOCTL_GET_ENABLED (return %d)\n",__FUNCTION__,__LINE__,als3010_dev->enable);
		return put_user(als3010_dev->enable, (unsigned long __user *)arg);
		break;

	case LIGHTSENSOR_IOCTL_CAL_ENABLE:
		if (get_user(enable, (unsigned long __user *)arg))
			return -EFAULT;
		if (enable){
			if(unlikely(debug_msg))printk( "[%s] %d: LIGHTSENSOR_IOCTL_CAL_ENABLE - als3010_enable\n",__FUNCTION__,__LINE__);
#ifdef CONFIG_HID_ASUS_PAD_EC	
			err=ite_light_sensor_set_function(0x02);
#endif
			printk( "alp : als3010 cal-enable sensor err=%d\n",err);
		}else{
			if(unlikely(debug_msg))printk( "[%s] %d: LIGHTSENSOR_IOCTL_CAL_ENABLE - als3010_disable\n",__FUNCTION__,__LINE__);
#ifdef CONFIG_HID_ASUS_PAD_EC	
			err=ite_light_sensor_set_function(0x04);
#endif
			printk( "alp : als3010 cal-disable sensor err=%d\n",err);
		}
		break;

	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(enable, (unsigned long __user *)arg))
			return -EFAULT;
		if (enable){
			if(unlikely(debug_msg))printk( "[%s] %d: LIGHTSENSOR_IOCTL_ENABLE - als3010_enable\n",__FUNCTION__,__LINE__);
			return als3010_enable(POWER_ON);
		}else{
			if(unlikely(debug_msg))printk( "[%s] %d: LIGHTSENSOR_IOCTL_ENABLE - als3010_disable\n",__FUNCTION__,__LINE__);
			return als3010_enable(POWER_OFF);
		}
		break;
			
	case LIGHTSENSOR_IOCTL_GET_RAWDATA:
//		err=ite_light_sensor_set_function(0x08);
//		if(unlikely(debug_msg))
			printk( "[%s] %d: LIGHTSENSOR_IOCTL_GET_RAWDATA (return %d)\n",__FUNCTION__,__LINE__,als3010_dev->adc_value);		
		return put_user(als3010_dev->adc_value, (unsigned long __user *)arg);
		break;

	case LIGHTSENSOR_IOCTL_GET_LUXVALUE:
#ifdef CONFIG_HID_ASUS_PAD_EC	
		err=ite_light_sensor_set_function(0x08); 
#endif
//		if(unlikely(debug_msg))
			printk( "[%s] %d: LIGHTSENSOR_IOCTL_GET_LUXVALUE (return %d)\n",__FUNCTION__,__LINE__,als3010_dev->lux_value);
		return put_user(als3010_dev->lux_value, (unsigned long __user *)arg);
		break;

	case LIGHTSENSOR_IOCTL_GET_CALLUX:
//			als3010_dev->lux_value = als3010_dev->lux_value*(als3010_dev->cal_value)/100;
			printk( "alp : LIGHTSENSOR_IOCTL_GET_CALLUX (%d) \n",als3010_dev->lux_value);
		break;

	case LIGHTSENSOR_IOCTL_CALIBRATION:
		if (get_user(value, (unsigned long __user *)arg))
			return -EFAULT;
		if(unlikely(debug_msg))printk( "[%s] %d: LIGHTSENSOR_IOCTL_CALIBRATION \n",__FUNCTION__,__LINE__);
		// set calibration value
		if(value==0){
			printk( "alp : als3010 calibration use function \n");
			als3010_dev->cal_value = als3010_get_calibration();
		}else{
			printk( "alp : als3010 calibration use value (%d)\n",value);
			als3010_dev->cal_value = value;
		}
		// set calibration table
		if(als3010_dev->cal_value==0)
			als3010_dev->cal_value = ALS_CALI_DEFAULT_VALUE;
		printk( "alp : als3010 use cavalue = %d \n",als3010_dev->cal_value);
		err= als3010_set_level_table(als3010_dev->cal_value);
		err= als3010_set_threshold_table(als3010_dev->cal_value);
		err= als3010_set_calibration_value(als3010_dev->cal_value);
		break;

	default:
		printk( "[%s] %d: incorrect command (%d) \n",__FUNCTION__,__LINE__,_IOC_NR(cmd));
		return -EINVAL;
	}

	return 0;
}

// 2013.07.04 cheng_kao early_suspend for als3010 ++
#ifdef CONFIG_HAS_EARLYSUSPEND
static void als3010_als_early_suspend(struct early_suspend *h)
{
	int ret=0;
	mutex_lock(&als3010_dev->lock);
	if(als3010_dev->enable==1){
#ifdef CONFIG_HID_ASUS_PAD_EC	
		ret=ite_light_sensor_set_function(0x04);
#endif
		printk( "alp : als3010 early-suspend disable , ret=%d\n",ret);
		als3010_dev->als3010_pw_before_suspend = 1;
	}
	mutex_unlock(&als3010_dev->lock);
	printk("als3010 : early_suspend(%d), pw(%d) \n",ret,als3010_dev->als3010_pw_before_suspend);
	return;	
}

static void als3010_als_late_resume(struct early_suspend *h)
{
	int ret=0;
	printk("als3010 : late_resume pw(%d) \n",als3010_dev->als3010_pw_before_suspend);
	mutex_lock(&als3010_dev->lock);
	if(als3010_dev->als3010_pw_before_suspend==1){
		als3010_dev->als3010_pw_before_suspend = 0;
#ifdef CONFIG_HID_ASUS_PAD_EC	
		ret=ite_light_sensor_set_function(0x02);
#endif
	}
	mutex_unlock(&als3010_dev->lock);
	printk("als3010 : late_resume(%d)\n",ret);
	return;
}
#endif
// 2013.07.04 cheng_kao early_suspend for als3010 --

static struct file_operations als3010_fops = {
	.owner = THIS_MODULE,
	.open = als3010_open,
	.release = als3010_release,
	.unlocked_ioctl = als3010_ioctl
};

static struct miscdevice als3010_misc_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "als3010_misc_dev",
	.fops = &als3010_fops,
};

static int als3010_setup(struct als3010_device_data *als3010_dev)
{
	int ret=0;

	// register input device
	if(!(als3010_dev->input_dev = input_allocate_device())){
		printk( "[%s] %d: input_allocate_device failed \n",__FUNCTION__,__LINE__);
		return -ENOMEM;
	}
	als3010_dev->input_dev->name = "als3010_input_dev";
	set_bit(EV_ABS, als3010_dev->input_dev->evbit);
	set_bit(EV_SYN, als3010_dev->input_dev->evbit);
	input_set_abs_params(als3010_dev->input_dev, ABS_MISC, 0, 5000, 0, 0);

	ret = input_register_device(als3010_dev->input_dev);
	if (ret < 0) {
		printk( "[%s] %d: input_register_device failed (%d)\n",__FUNCTION__,__LINE__,ret);
		goto register_inout_device_failed;
	}

	// register misc device
	ret = misc_register(&als3010_misc_dev);
	if (ret < 0) {
		printk( "[%s] %d: misc_register failed (%d)\n",__FUNCTION__,__LINE__,ret);
		goto register_misc_device_failed;
	}

	printk( "[%s] %d: finished \n",__FUNCTION__,__LINE__);
	return 0;


register_inout_device_failed:
	input_unregister_device(als3010_dev->input_dev);
register_misc_device_failed:
	input_free_device(als3010_dev->input_dev);
	return ret;
}

int als3010_work_function(int lux, int ilevel, int raw_data){

	als3010_dev->lux_value = lux*(als3010_dev->cal_value)/100;
//	als3010_dev->lux_value = lux*(als3010_dev->cal_value)/1000;	// use /1000 for Test
	als3010_dev->adc_value = raw_data;
	printk("[%s] lux = %d, ilevel= %d, raw=%d\n",__FUNCTION__,als3010_dev->lux_value,ilevel,als3010_dev->adc_value);
	als3010_dev->als_status=1;
//	if( (als3010_dev->lux_value==0)&&(raw_data>6) ){
//		als3010_dev->lux_value = 10;
//	}
	
	input_report_abs(als3010_dev->input_dev, ABS_MISC, als3010_dev->lux_value);
	input_sync(als3010_dev->input_dev);

	return 0;
}
EXPORT_SYMBOL(als3010_work_function);

static int __init als3010_driver_init(void)
{	
	int ret=0;

	//memory allocate
	if(!(als3010_dev = kzalloc(sizeof (struct als3010_device_data), GFP_KERNEL))){
		printk("[%s] %d: %s memory allocate fail\n",__FUNCTION__,__LINE__,DRIVER_NAME);
		ret = -ENOMEM;
		goto memory_allocate_failed;
	}

	//sysfs setting
	if(!(als3010_kobj = kobject_create_and_add("als3010_kobject", kernel_kobj))){
		return -ENOMEM;
	}
	ret = sysfs_create_group(als3010_kobj, &als3010_attr_group);
	if (ret){
		kobject_put(als3010_kobj);
		goto sysfs_create_group_failed;
	}

	als3010_setup(als3010_dev);
	als3010_enable(POWER_OFF);
	als3010_create_proc_file();
	// init cal_value = ALS_CALI_DEFAULT_VALUE;
	als3010_dev->cal_value=ALS_CALI_DEFAULT_VALUE;
	als3010_dev->als_status=0;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	als3010_dev->als3010_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	als3010_dev->als3010_early_suspend.suspend = als3010_als_early_suspend;
	als3010_dev->als3010_early_suspend.resume = als3010_als_late_resume;
	register_early_suspend(&als3010_dev->als3010_early_suspend);
#endif

	mutex_init(&als3010_dev->lock);
	printk( "[%s] %d: finished \n",__FUNCTION__,__LINE__);
	return 0;


memory_allocate_failed:
sysfs_create_group_failed:
	input_free_device(als3010_dev->input_dev);
	als3010_dev->input_dev = NULL;
	kfree(als3010_dev);

	return ret;
}

static void __exit als3010_driver_exit(void)
{
	als3010_remove_proc_file();
	input_free_device(als3010_dev->input_dev);
	kfree(als3010_dev);
	kobject_put(als3010_kobj);
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&als3010_dev->als3010_early_suspend);
#endif
}

module_init(als3010_driver_init);
module_exit(als3010_driver_exit);

MODULE_DESCRIPTION("LITEON ALS3010 Driver");
MODULE_LICENSE("GPL v2");
