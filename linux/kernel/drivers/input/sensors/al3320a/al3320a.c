#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/irq.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/delay.h>
#include <linux/input.h>
#include <linux/kobject.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/al3320a.h>
// 2013.03.22 cheng_kao early_suspend ++
#include <linux/earlysuspend.h>
// 2013.03.22 cheng_kao early_suspend --

#define AL3320A_DRV_NAME			"al3320a_misc_dev"
#define DRIVER_VERSION				"1.0.0.0"
#define ALS_CALIBRATION_FILE_PATH	"/data/misc/als_calibration.ini"
#define AL3320A_NUM_CACHABLE_REGS	14
#define AL3320A_NUM_ADC_TO_LUX		8
#define AL3320A_NUM_LUX_LEVEL			11	// level = 10 + 1
#define AL3320A_NUM_LUX_TABLE_LEVEL	12

#define AL3320A_SENSOR_STABLE_NUM	0x14	// 20 times

#define AL3320A_INIT_STABLE_COUNTER	5		// for init

static struct i2c_client *al3320a_i2c_client = NULL;

struct al3320a_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct mutex lock;
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend al3320a_early_suspend;
#endif
	int al3320a_reg_status[AL3320A_NUM_CACHABLE_REGS];
	int al3320a_state;
	int al3320a_pw_before_suspend;
	int al3320a_adc_to_lux;
	int al3320a_counter_after_enable;
	int int_gpio;
//	int al3320a_test;
	int irq;
};
static struct al3320a_data *light_sensor_data;

// AL3320A register
static u8 al3320a_reg[AL3320A_NUM_CACHABLE_REGS] = {0x00,0x01,0x02,0x06,0x07,0x08,0x09,0x22,0x23,0x30,0x31,0x32,0x33,0x34};
//                                                                                           0       1      2      3      4       5      6      7       8     9     10     11    12     13

// AL3320A adc to lux mapping table
static int al3320a_adc_to_lux_table[AL3320A_NUM_ADC_TO_LUX] ={1530,512,380,128,96,32,30,11};

// AL3320A lux threshold level range  ; base on 0.096lux/counter
static int al3320a_threshold_level[AL3320A_NUM_LUX_LEVEL] = {5, 30, 50, 100, 300, 550, 900, 1100, 1500, 2200, 7000};

/*
//static int al3320a_threshold_table[AL3320A_NUM_LUX_TABLE_LEVEL] =
// {0, 52, 312, 520, 1041, 3125, 5729, 9375, 11458, 15628, 22916, 65535};
// base on reg_val = 0x04;
static u8 al3320a_threshold_table_uL[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0, 0x34, 0x38, 0x08, 0x11, 0x35, 0x61, 0x9F, 0xC2, 0x09, 0x84, 0xFF}; 
static u8 al3320a_threshold_table_uH[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0,  0x0,  0x1, 0x02, 0x04 , 0x0C, 0x16, 0x24, 0x2C, 0x3D, 0x59, 0xFF};
*/

//static int al3320a_threshold_table[AL3320A_NUM_LUX_TABLE_LEVEL] =
//{0, 52, 312, 520, 1041, 3125, 5729, 9375, 11458, 15628, 22916, 65535};
// base on reg_val = 0x07;
static u8 al3320a_threshold_table_uL[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0, 0x2E, 0x11, 0xC7, 0x8D, 0xA7, 0x88, 0xF6, 0x10, 0x44, 0x20, 0xFF}; 
static u8 al3320a_threshold_table_uH[AL3320A_NUM_LUX_TABLE_LEVEL] = {0x0,  0x0,  0x1, 0x01, 0x03 , 0x0A, 0x13, 0x1F, 0x27, 0x35, 0x4E, 0xFF};


//	al3320a Unit Function ++
static int al3320a_check_device(int state);
static int al3320a_input_init(void);
static int al3320a_read_calibration(void);
static int al3320a_chip_init(void);
static int al3320a_enable_init(void);
static int al3320a_enable(int iEnable);
static int al3320a_get_adc_counter(void);
static int al3320a_set_threshold(int ilux);
//	al3320a Unit  Function --


//	define for DEBUG message 
#define ALS_MESSAGE	1	// default 1 : for show status 
#define ALS_DEBUGMSG	0	// default 0 : for Test and debug


/*--------------------------------------------------------------------------------*/
//unit function  ++

static int al3320a_set_threshold(int ilux)
{
	int index=0,ret=0;
	u8 low_val_ul=0x00,low_val_uh=0x00,high_val_ul=0x00,high_val_uh=0x00;// ,als_persist=0x01;
	// get the threshold table index
	for(index=0;index<AL3320A_NUM_LUX_LEVEL;index++){
		if(ilux<al3320a_threshold_level[index])
			break;
	}
	low_val_ul = al3320a_threshold_table_uL[index];
	low_val_uh = al3320a_threshold_table_uH[index];
	high_val_ul = al3320a_threshold_table_uL[index+1];
	high_val_uh = al3320a_threshold_table_uH[index+1];
//	low_val_ul = ((al3320a_threshold_table[index] << 8) >> 8);
//	low_val_uh = (al3320a_threshold_table[index] >> 8);
//	high_val_ul = ((al3320a_threshold_table[index+1] << 8) >> 8);
//	high_val_uh = (al3320a_threshold_table[index+1] >> 8);
	if(ALS_DEBUGMSG) printk("al3320a : index(%d) , LL(%d), LH(%d), HL(%d), HH(%d)!!\n",index,low_val_ul,low_val_uh,high_val_ul,high_val_uh);

	mutex_lock(&light_sensor_data->lock);
	// change the threshold
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, low_val_ul);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_set_threshold low_val_ul fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = low_val_ul;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, low_val_uh);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_set_threshold low_val_uh fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = low_val_uh;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, high_val_ul);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_set_threshold high_val_ul fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = high_val_ul;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, high_val_uh);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_set_threshold high_val_uh fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = high_val_uh;

	//Persist set to 0x01 or 0x03
//	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_PERSIST, als_persist);
//	if (ret){
//		if(ALS_MESSAGE) printk("al3320a: al3320a_set_threshold als_persist fail !!\n");
//		return ret;
//	}
//	light_sensor_data->al3320a_reg_status[INDEX_ALS_PERSIST] = als_persist;
	mutex_unlock(&light_sensor_data->lock);

	if(ALS_DEBUGMSG) printk("al3320a: al3320a_set_threshold success!!\n");
	return 1;
}

static int al3320a_get_adc_counter(void)
{
	int iadc_low=0,iadc_high=0, iadc=0;
	// clean int flag
	iadc_low = i2c_smbus_read_byte_data(light_sensor_data->client, ADDR_ALS_DATA_LOW);
	if (iadc_low<0){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_DATA_LOW fail !!\n");
		return iadc_low;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_DATA_LOW] = iadc_low;

	iadc_high = i2c_smbus_read_byte_data(light_sensor_data->client, ADDR_ALS_DATA_HIGH);
	if (iadc_high<0){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_DATA_HIGH fail !!\n");
		return iadc_high;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_DATA_HIGH] = iadc_high;
	iadc = ( (iadc_high<<8) |iadc_low );

	if(ALS_DEBUGMSG) printk("al3320a: al3320a_get_adc_counter(%d) !!\n",iadc);
	return iadc;
}

static int al3320a_enable(int iEnable)
{
	int ret = 0;
	u8 reg_val = 0x00;
	switch(iEnable)
	{
		case 0:
			reg_val = 0x00;
		break;

		case 1:
			light_sensor_data->al3320a_counter_after_enable = 0;
			al3320a_enable_init();
			reg_val = 0x01;
		break;

		default:
			reg_val = 0x00;
		break;
	}
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_SYS_CONFIG, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_set_mode ADDR_ALS_SYS_CONFIG fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG] = reg_val;	
	if(ALS_MESSAGE) printk(KERN_INFO "al3320a_enable(%d) !!\n",iEnable);
	return 1;
}

static int al3320a_input_init(void)
{
	int ret;

	// allocate light input_device 
	light_sensor_data->input_dev = input_allocate_device();
	if (!light_sensor_data->input_dev) {
		printk("could not allocate input device\n");
		goto err_light_all;
		}
	// input_set_drvdata(input_dev, light_sensor_data);
	light_sensor_data->input_dev->name = "al3320a_input";
	input_set_capability(light_sensor_data->input_dev, EV_ABS, ABS_MISC);
	input_set_abs_params(light_sensor_data->input_dev, ABS_MISC, 0, 1, 0, 0);

	if(ALS_MESSAGE) printk("alp : al3320a registering light sensor input device\n");
	ret = input_register_device(light_sensor_data->input_dev);
	if (ret < 0) {
		printk("could not register input device\n");
		goto err_light_reg;
	}
	return 0;

err_light_reg:
	input_free_device(light_sensor_data->input_dev);
err_light_all:
	return (-1);   
}

static int al3320a_read_calibration(void)
{
	struct file *fp=NULL;
	char c_data[10]={0}; //c_data-> calibration data
	int ilen=0,ret=0;
	u8 reg_val = 0x00;
	mm_segment_t old_fs;

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp=filp_open(ALS_CALIBRATION_FILE_PATH,O_RDONLY,0660);
	if (IS_ERR_OR_NULL(fp)){
		if(ALS_DEBUGMSG) printk("al3320a : File Open Failed \n");
		return -ENOENT;
	}

	if(fp->f_op != NULL && fp->f_op->read != NULL){
		ilen = fp->f_op->read(fp,c_data,10,&fp->f_pos);
		if(ALS_DEBUGMSG) printk("al3320a : fp->f_op->read (len = %d) \n",ilen);
	}
	set_fs(old_fs);		
	filp_close(fp,NULL);

	if(ilen<=0){
			reg_val = 64;
			if(ALS_MESSAGE) printk(KERN_INFO "al3320a : read file fail\n");
	}else{
			if(ALS_MESSAGE) printk(KERN_INFO "al3320a : cal %d & %d\n",c_data[0],c_data[1]);
			reg_val = c_data[0]+128*c_data[1];
	}
	
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_CALIBRATION, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_read_calibration ADDR_ALS_CALIBRATION fail !!\n");
		return ret;
	}
	printk(KERN_INFO "al3320a : org_cal=%d , new_cal=%d\n",light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION],reg_val);
	light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION] = reg_val;
	return 1;   
}
static int al3320a_chip_init(void)
{
	int ret=0;
	u8 reg_val = 0x00;

	// power down chip at init
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_SYS_CONFIG, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_set_mode ADDR_ALS_SYS_CONFIG fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG] = reg_val;	

	// clean int flag
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_FLAG_STATUS fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;

	// set lux reg : 0~6.25K
//	reg_val = 0x04;
	reg_val = 0x07;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_CONFIG, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_CONFIG fail !!\n");
		return ret;
	}
	ret = reg_val;
	light_sensor_data->al3320a_reg_status[INDEX_ALS_CONFIG] = reg_val;
	light_sensor_data->al3320a_adc_to_lux = al3320a_adc_to_lux_table[ret];

	//Persist set to 0x01 or AL3320A_SENSOR_STABLE_NUM
	reg_val = AL3320A_SENSOR_STABLE_NUM;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_PERSIST, reg_val );
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init als_persist fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_PERSIST] = reg_val;

	// set threshold : low-65535, high:0 to trigger when al3320a starts.
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_THRES_LOW_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = reg_val;
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_THRES_LOW_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_THRES_HIGH_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init ADDR_ALS_THRES_HIGH_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = reg_val;


	if(ALS_MESSAGE) printk("al3320a: al3320a_chip_init success!!\n");
	return 0;
}

static int al3320a_enable_init(void)
{
	int ret=0;
	u8 reg_val = 0x00;

	// clean int flag
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init ADDR_ALS_FLAG_STATUS fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;

	//Persist set to 0x01 or AL3320A_SENSOR_STABLE_NUM
	reg_val = AL3320A_SENSOR_STABLE_NUM;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_PERSIST, reg_val );
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init als_persist fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_PERSIST] = reg_val;

	// set threshold : low-65535, high:0 to trigger when al3320a starts.
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init ADDR_ALS_THRES_LOW_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_L] = reg_val;
	reg_val = 0xFF;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_LOW_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init ADDR_ALS_THRES_LOW_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_LOW_H] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_L, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init ADDR_ALS_THRES_HIGH_L fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_L] = reg_val;
	reg_val = 0x00;
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_THRES_HIGH_H, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init ADDR_ALS_THRES_HIGH_H fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_THRES_HIGH_H] = reg_val;


	if(ALS_MESSAGE) printk("al3320a: al3320a_enable_init success!!\n");
	return 0;
}


static int al3320a_check_device(int state)
{
	int iloop=0, ret=0;
	char buf[32]={0};
	for (iloop = 0; iloop < AL3320A_NUM_CACHABLE_REGS; iloop++) {
		ret = i2c_smbus_read_byte_data(light_sensor_data->client, al3320a_reg[iloop]);
		light_sensor_data->al3320a_reg_status[iloop] = ret ;
		if (ret < 0)
			return sprintf(buf, " iloop %d \n , %d , status fail!!",iloop ,ret);
		if(ALS_DEBUGMSG || state) printk("al3320a: iloop[%d] = %d\n",iloop,light_sensor_data->al3320a_reg_status[iloop]);
	}
	light_sensor_data->al3320a_state = 1;
	if(ALS_MESSAGE) printk("al3320a: al3320a_check_device success!!\n");
	return 1;
}
//unit function --
/*--------------------------------------------------------------------------------*/

static ssize_t al3320a_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	int ret = 0;
	ret = light_sensor_data->al3320a_state;
	if(ALS_DEBUGMSG) printk("al3320a: al3320a_show_status !!\n");
	if(!ret){
		return sprintf(buf, "%d\n", 0 );
	}
	return sprintf(buf, "%d\n", 1 );
}

static ssize_t al3320a_show_adc(struct device *dev,struct device_attribute *attr,char *buf)
{
	int iadc=0;
	if(ALS_DEBUGMSG) printk("al3320a: al3320a_show_adc !!\n");
	iadc = al3320a_get_adc_counter();
	printk("al3320a: al3320a_show_adc(%d) !!\n",iadc);
	return sprintf(buf, "%d \n", iadc);
}

// mode ++
static ssize_t al3320a_show_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	if(ALS_DEBUGMSG) printk("als : al3320a_show_mode !!\n");
	ret = i2c_smbus_read_byte_data(light_sensor_data->client, al3320a_reg[INDEX_ALS_SYS_CONFIG]);
	if (ret < 0)
		return sprintf(buf, " SYS_CONFIG status fail (%d)!!",ret);
	return sprintf(buf, "status : %d\n", ret);
}

static ssize_t al3320a_set_mode(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret=-1;

	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 7)){
		printk("als : al3320a_set_mode buf(%d) !!\n",buf[0]);
		return -EINVAL;
	}
	if(buf[0]=='0'){
		if(ALS_MESSAGE) printk("als : al3320a_set_mode 0 !!\n");
		ret = 0;
	}
	else if(buf[0]=='1'){
		if(ALS_MESSAGE) printk("als : al3320a_set_mode 1 !!\n");
		ret = 1;
	}else if( (buf[0]=='c') && (buf[1]=='a') && (buf[2]=='l') ){
		if(ALS_MESSAGE) printk("als : al3320a_set_mode cal !!\n");
		ret = 2;
	}else{
		if(ALS_MESSAGE) printk("als : al3320a_set_mode fail !!\n");
		return ret;
	}
	al3320a_enable(ret);
	return count;
}
// mode --

static ssize_t al3320a_set_report(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret=-1;
	if (strict_strtoul(buf, 10, &val) < 0){
		printk("als : al3320a_set_report buf(%d) !!\n",buf[0]);
		return -EINVAL;
	}
	if(ALS_MESSAGE) printk("als : al3320a_set_report  %d  !!\n",val);
	input_report_abs(light_sensor_data->input_dev, ABS_MISC, val);
	input_sync(light_sensor_data->input_dev);
	return count;
}

static ssize_t al3320a_show_config(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret=0;
	ret = al3320a_check_device(1);
	ret = light_sensor_data->al3320a_adc_to_lux;
	printk("al3320a: al3320a_show_config(%d) !!\n",ret);
	al3320a_read_calibration();
	return sprintf(buf, "%d \n", ret);
}


static DEVICE_ATTR(status, S_IRUGO,al3320a_show_status,NULL);
static DEVICE_ATTR(adc, S_IRUGO,al3320a_show_adc,NULL);
static DEVICE_ATTR(mode, 0660 ,al3320a_show_mode,al3320a_set_mode);
static DEVICE_ATTR(report, 0660 ,NULL,al3320a_set_report);
static DEVICE_ATTR(config, S_IRUGO,al3320a_show_config,NULL);

static struct attribute *al3320a_attributes[] = {
	&dev_attr_status.attr,
	&dev_attr_adc.attr,
	&dev_attr_mode.attr,
	&dev_attr_report.attr,
	&dev_attr_config.attr,
	NULL
};

static const struct attribute_group al3320a_attr_group = {
	.attrs = al3320a_attributes,
};

// internal interrupt function ++
static irqreturn_t als_interrupt(int irq, void *dev_id)
{
	int ret=0,als_iadc=0,als_lux=0;
	u8 reg_val=0x00;
	if(ALS_DEBUGMSG) printk("al3320a: als_interrupt !!\n");
	als_iadc = al3320a_get_adc_counter();
	if(als_iadc<0){
		if(ALS_DEBUGMSG) printk("al3320a: als_interrupt fail(%d)!!\n",als_iadc);
		return IRQ_HANDLED;
	}		

	//	for Test
/*
	if(light_sensor_data->al3320a_test==1){
		light_sensor_data->al3320a_test = 0;
		als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / 100);
		input_report_abs(light_sensor_data->input_dev, ABS_MISC, als_lux);
		input_sync(light_sensor_data->input_dev);
		printk("al3320a: als_interrupt lux(%d)!!\n",als_lux);
	}else{
		printk("al3320a: als_interrupt add counter\n");
		light_sensor_data->al3320a_test++;
	}
*/

	//	change the adc value to lux and reprot lux to hal
	als_lux = (als_iadc * light_sensor_data->al3320a_adc_to_lux / 100);
	input_report_abs(light_sensor_data->input_dev, ABS_MISC, als_lux);
	input_sync(light_sensor_data->input_dev);
	if(ALS_DEBUGMSG) printk("al3320a: als_interrupt lux(%d)!!\n",als_lux);

	if(light_sensor_data->al3320a_counter_after_enable < AL3320A_INIT_STABLE_COUNTER){
		light_sensor_data->al3320a_counter_after_enable++;
	}
	//	change the threshold
	if(light_sensor_data->al3320a_counter_after_enable >= AL3320A_INIT_STABLE_COUNTER){
		al3320a_set_threshold(als_lux);
		udelay(500);
	}


	//	clean the interrupt pin
	ret = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_FLAG_STATUS, reg_val);
	if (ret){
		if(ALS_MESSAGE) printk("al3320a: als_interrupt clean flag fail !!\n");
		return ret;
	}
	light_sensor_data->al3320a_reg_status[INDEX_ALS_FLAG_STATUS] = reg_val;
	
	return IRQ_HANDLED;
}
// internal interrupt  function --

static int al3320a_open(struct inode *inode, struct file *file)
{
	if(ALS_DEBUGMSG) printk(KERN_INFO "[%s]\n", __FUNCTION__);
	return nonseekable_open(inode, file);
}

static int al3320a_release(struct inode *inode, struct file *file)
{
	if(ALS_DEBUGMSG) printk(KERN_INFO "[%s]\n", __FUNCTION__);
	return 0;
}


static long al3320a_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int retval =0, ivalue=0 ,ilux=0 ,pw_en=0;
	u8 reg_val = 0;
	switch(cmd)
	{
		case LIGHTSENSOR_IOCTL_GET_INIT:
			if(ALS_DEBUGMSG) printk("LIGHTSENSOR_IOCTL_GET_INIT  pw(%d)\n",light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG]);
			retval = put_user(light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG], (unsigned long __user *)arg );
		break;

		case LIGHTSENSOR_IOCTL_ENABLE:
			if (get_user(pw_en, (unsigned long __user *)arg)){
				if(ALS_DEBUGMSG) printk("ENABLE get arg fail\n");
				return -EFAULT;
			}
			if(ALS_DEBUGMSG) printk("al3320a : ENABLE arg=%d\n",pw_en);
			switch(pw_en)
			{
				case 0:
					retval = al3320a_enable(0);
					if(retval) put_user(0, (unsigned long __user *)arg );
				break;

				default :
					retval = al3320a_enable(1);
					if(retval) put_user(1, (unsigned long __user *)arg );
				break;
			}
			if(ALS_DEBUGMSG) printk("LIGHTSENSOR_IOCTL_ENABLE  retval(%d)\n",retval);
		break;	//LIGHTSENSOR_IOCTL_ENABLE

		case LIGHTSENSOR_IOCTL_GET_LUX:
			ivalue = al3320a_get_adc_counter();
			ilux = (ivalue * light_sensor_data->al3320a_adc_to_lux / 100);
			retval = put_user(ilux, (unsigned long __user *)arg );
			if(ALS_DEBUGMSG) printk("al3320a: LIGHTSENSOR_IOCTL_GET_LUX lux(%d), bias(%d)\n",ilux,light_sensor_data->al3320a_adc_to_lux);
		break;

		case LIGHTSENSOR_IOCTL_GET_ADC:
			ivalue = al3320a_get_adc_counter();
			retval = put_user(ivalue, (unsigned long __user *)arg );
			if(ALS_DEBUGMSG) printk("al3320a: LIGHTSENSOR_IOCTL_GET_ADC adc(%d)\n",ivalue);
		break;

		case LIGHTSENSOR_IOCTL_GET_CONFIG:
			retval = al3320a_check_device(0);
			ivalue = light_sensor_data->al3320a_adc_to_lux;
			retval = put_user(ivalue, (unsigned long __user *)arg );
			if(ALS_DEBUGMSG) printk("LIGHTSENSOR_IOCTL_GET_CONFIG  ivalue(%d)\n",ivalue);
		break;

		case LIGHTSENSOR_IOCTL_CAL_START:
			reg_val = 0x40;
			retval = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_CALIBRATION, reg_val);
			light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION] = reg_val ;
			if(ALS_DEBUGMSG) printk("al3320a : CAL_START  reg_val(%d)\n",reg_val);
		break;

		case LIGHTSENSOR_IOCTL_CAL_END:
			if (get_user(reg_val, (unsigned long __user *)arg)){
				if(ALS_DEBUGMSG) printk("CAL get arg fail\n");
				return -EFAULT;
			}
			retval = i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_CALIBRATION, reg_val);
			light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION] = reg_val ;
			if(ALS_DEBUGMSG) printk("al3320a : CAL_END reg_val(%d)\n",reg_val);
		break;

		case LIGHTSENSOR_IOCTL_CAL_INIT:
			printk("al3320a : LIGHTSENSOR_IOCTL_CAL_INIT  cal (%d)\n",light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION]);
			al3320a_read_calibration();
			retval = put_user(light_sensor_data->al3320a_reg_status[INDEX_ALS_CALIBRATION], (unsigned long __user *)arg );
		break;


		default:   //redundant, as cmd was checked against MAXNR 
			if(ALS_DEBUGMSG) printk("al3320a: ERROR IOCTL CONTROL COMMAND!!!\n");
		return -ENOTTY;
	}
	return 0;
}

// 2013.03.22 cheng_kao early_suspend for al3320a ++
#ifdef CONFIG_HAS_EARLYSUSPEND
static void al3320a_als_early_suspend(struct early_suspend *h)
{
	int ret=0;
	mutex_lock(&light_sensor_data->lock);
	if(light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG]==1){
//		input_report_abs(light_sensor_data->input_dev, ABS_MISC, 0);
//		input_sync(light_sensor_data->input_dev);
		ret = al3320a_enable(0);
		light_sensor_data->al3320a_pw_before_suspend = 1;
	}
	mutex_unlock(&light_sensor_data->lock);
	if(ALS_MESSAGE) printk("al3320a:  early_suspend(%d), pw(%d) \n",ret,light_sensor_data->al3320a_pw_before_suspend);
	return;	
}

static void al3320a_als_late_resume(struct early_suspend *h)
{
	int ret=0;
	if(ALS_DEBUGMSG) printk("al3320a:  late_resume pw(%d) \n",light_sensor_data->al3320a_pw_before_suspend);
	mutex_lock(&light_sensor_data->lock);
	if(light_sensor_data->al3320a_pw_before_suspend==1){
		light_sensor_data->al3320a_pw_before_suspend = 0;
		ret = al3320a_enable(1);
	}
	mutex_unlock(&light_sensor_data->lock);
	if(ALS_MESSAGE) printk("al3320a:  late_resume(%d)\n",ret);
	return;
}
#endif
// 2013.03.22 cheng_kao early_suspend for al3320a --

static struct file_operations al3320a_fops = {
	.owner			= 	THIS_MODULE,
	.unlocked_ioctl	=	al3320a_ioctl,
	.open			=	al3320a_open,
	.release			=	al3320a_release,
};

static struct miscdevice al3320a_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = AL3320A_DRV_NAME,
	.fops = &al3320a_fops,
};

static int al3320a_i2c_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct al3320a_i2c_platform_data *pdata;
	int err = 0;

	light_sensor_data = kzalloc(sizeof(struct al3320a_data), GFP_KERNEL);
	if (!light_sensor_data)
	{
		printk(KERN_ERR "alp : al3320a allocate al3320a_data failed.\n");
		return -ENOMEM;
	}
	light_sensor_data->client = client;
	i2c_set_clientdata(client, light_sensor_data);
	pdata = client->dev.platform_data;
	light_sensor_data->int_gpio = pdata->int_gpio;	
	light_sensor_data->al3320a_state = 0;	//init state.
	light_sensor_data->al3320a_counter_after_enable = 0; //init counter.
//	light_sensor_data->al3320a_test = 0; //for test.

	al3320a_check_device(0);
	if(ALS_MESSAGE) printk(KERN_INFO "alp : al3320a registered I2C driver!\n");	
	err = misc_register(&al3320a_device);
	if (err) {
		if(ALS_MESSAGE) printk(KERN_ERR "alp : al3320a_device register failed\n");
		goto exit_misc_device_register_failed;
	}
	
	mutex_init(&light_sensor_data->lock);

	// init chip status
	err = al3320a_chip_init();
	if (err)
	{
		printk(KERN_ERR "alp : al3320a_chip_init failed.\n");
		goto exit_kfree;
	}

	// init input subsystem	
	err = al3320a_input_init();
	if (err)
	{
		printk(KERN_ERR "alp : al3320a_input_init failed.\n");
		goto exit_kfree;
	}

	/* Register sysfs hooks */
	err = sysfs_create_group(&client->dev.kobj, &al3320a_attr_group);
	if (err)
		goto exit_sysfs_create_group_failed;
	
#ifdef CONFIG_HAS_EARLYSUSPEND
	light_sensor_data->al3320a_early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	light_sensor_data->al3320a_early_suspend.suspend = al3320a_als_early_suspend;
	light_sensor_data->al3320a_early_suspend.resume = al3320a_als_late_resume;
	register_early_suspend(&light_sensor_data->al3320a_early_suspend);
#endif

	// init interrupt pin	
	err = gpio_request(light_sensor_data->int_gpio, "al3320a-irq");
	if(err < 0)
		printk(KERN_ERR "alp : al3320a Failed to request GPIO%d (al3320a-irq) error=%d\n", light_sensor_data->int_gpio, err);
	err = gpio_direction_input(light_sensor_data->int_gpio);
	if (err){
		printk(KERN_ERR "alp : al3320a Failed to set interrupt direction, error=%d\n", err);
		gpio_free(light_sensor_data->int_gpio);
	}
	light_sensor_data->irq = gpio_to_irq(light_sensor_data->int_gpio);
	if(ALS_MESSAGE) printk(KERN_INFO "alp : al3320a irq: %d\n",light_sensor_data->irq);
	if(light_sensor_data->irq > 0){
		err = request_threaded_irq(light_sensor_data->irq, NULL,als_interrupt, IRQF_TRIGGER_FALLING,"al3320a_interrupt",NULL);
	  	if (err) {
			dev_err(&client->adapter->dev,"cannot register IRQ %d,err:%d\n",light_sensor_data->irq,err);
		}
	}
	return 0;

exit_misc_device_register_failed:
exit_sysfs_create_group_failed:
exit_kfree:
	kfree(pdata);
	kfree(light_sensor_data);
	return err;
}

static int __devexit al3320a_i2c_remove(struct i2c_client *client)
{
	struct al3320a_data *data;
	u8 reg_val = 0x00;

	// power down chip at remove
	i2c_smbus_write_byte_data(light_sensor_data->client, ADDR_ALS_SYS_CONFIG, reg_val);
	light_sensor_data->al3320a_reg_status[INDEX_ALS_SYS_CONFIG] = reg_val;	
	
	sysfs_remove_group(&client->dev.kobj, &al3320a_attr_group);
	data = i2c_get_clientdata(client);
	al3320a_i2c_client = NULL;
#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&light_sensor_data->al3320a_early_suspend);
#endif
	kfree(data);

	return 0;
}

static int al3320a_suspend(struct i2c_client *client , pm_message_t mesg)
{
	printk("al3320a:  al3320a_suspend\n");
	return 0;
}

static int al3320a_resume(struct i2c_client *client)
{
	printk("al3320a:  al3320a_resume\n");
	return 0;
}

struct i2c_device_id al3320a_idtable[] = {
	{ "al3320a", 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, al3320a_idtable);

static struct i2c_driver al3320a_i2c_driver = {
	.driver = {
		.name  = AL3320A_DRV_NAME,
		.owner = THIS_MODULE,
	},
	.suspend	=	al3320a_suspend,
	.resume	=	al3320a_resume,
	.probe	=	al3320a_i2c_probe,
	.remove	=	__devexit_p(al3320a_i2c_remove),
	.id_table	=	al3320a_idtable,	
};

static int __init al3320a_init(void)
{
	int ret;	
	ret = i2c_add_driver(&al3320a_i2c_driver);
	if ( ret != 0 ) {
		if(ALS_DEBUGMSG) printk(KERN_ERR "[%s]can not add i2c driver\n",__FUNCTION__);
		return ret;
	}
	return ret;
}

static void __exit al3320a_exit(void)
{
	i2c_del_driver(&al3320a_i2c_driver);
}

MODULE_AUTHOR("Alp kao");
MODULE_DESCRIPTION("Dyna IMAGE al3320a driver");
MODULE_LICENSE("GPL");
MODULE_VERSION(DRIVER_VERSION);

module_init(al3320a_init);
module_exit(al3320a_exit);
