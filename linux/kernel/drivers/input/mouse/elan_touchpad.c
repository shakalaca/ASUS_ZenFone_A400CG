/*
 * Elan I2C Touchpad diver
 *
 * Copyright (c) 2013 ELAN Microelectronics Corp.
 *
 * Author: ªL¬Fºû (Cheng Wei Lin) <dusonlin@emc.com.tw>
 * Version: 1.0 (2013.06.28)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 *
 * Trademarks are the property of their respective owners.
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/input/mt.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/cdev.h>
#include <linux/kernel.h>
#include <linux/major.h>
#include <linux/sched.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>
#include <linux/earlysuspend.h>
#include <linux/elan_touchpad.h>
#include <linux/ite8566.h>
#include <linux/HWVersion.h>

//define parameter --------------------------------------------------------------------------------
#define DRIVER_VERSION		"3.0.2"		//Joe modify

#define DRIVER_NAME		"elan_i2c"
#define HID_DESC_LENGTH			30
#define HID_REPORT_ID_OFFSET	2
#define ETP_PALM_SIZE			80
#define ETP_MIN_TAP_PRESSURE	25
#define ELAN_DRIVER_VERSION	"1.0"
#define ETP_DEF_MAX_X			2690
#define ETP_DEF_MAX_Y			1628
#define ETP_DEF_TRACENUM_X		21
#define ETP_DEF_TRACENUM_Y		12
#define ETP_DEF_RES_X			3
#define	ETP_DEF_RES_Y			6
#define ETP_I2C_COMMAND_TRIES	3

/* Length of Elan touchpad information */
#define ETP_INF_LENGTH			2
#define ETP_MAX_FINGERS			5
#define ETP_REPORT_DESC_LENGTH	148
#define ETP_REPORT_LENGTH		30
#define ETP_FINGER_DATA_OFFSET	4
#define ETP_FINGER_DATA_LEN		5

#define ETP_REPORT_ID		0x5d

#define HID_CMD_REGISTER	0x0005
#define ETP_CMD_REGISTER	0x0300
#define ETP_CTRL_REGISTER	0x0301

#define CMD_RESET			0x0100
#define CMD_WAKE_UP			0x0800
#define CMD_SLEEP			0x0801
#define CMD_ENABLE_ABS		0x0001

#define REG_DESC			0x0001
#define REG_REPORT_DESC		0x0002
#define REG_UNIQUE_ID		0x0101
#define REG_FW_VERSION		0x0102
#define REG_XY_TRACE_NUM	0x0105
#define REG_X_AXIS_MAX		0x0106
#define REG_Y_AXIS_MAX		0x0107
#define REG_RESOLUTION		0x0108

//Global variable --------------------------------------------------------------------------------

/* The main device structure */
struct elan_i2c_data {
	struct i2c_client	*client;
	struct input_dev	*input;
	struct workqueue_struct *touchpad_wq;
	struct attribute_group attrs;
	struct delayed_work elan_init_work;
	struct delayed_work elan_destroy_work;
	struct delayed_work elan_resume_work;
	struct work_struct elan_init_abs_work;
	struct early_suspend early_suspend;
	struct mutex mutex_lock;
	
	unsigned int		max_x;
	unsigned int		max_y;
	unsigned int		width_x;
	unsigned int		width_y;
	unsigned int		irq;
	unsigned int		int_gpio;
	unsigned int		tp_sw_gpio;
	
	bool	irq_wake;
	int		touchpadp_status;
	int		irq_status;
	int		panel_owner; //0x00:no use / 0x01:Android / 0x02:Windows
	int		desc_length;
	int		report_desc_length;
	int		report_length;
//	int		input_dev_status;
	int		press_button;
	int		button_key;
	int		tp_enable;
	int 	hw_id;
};
struct elan_i2c_data *elan_tp_data;

//extern function -------------------------------------------------------------------------------------------------
extern int register_dock_atow_late_notifier(struct notifier_block *nb);
extern int register_dock_wtoa_late_notifier(struct notifier_block *nb);
extern int register_dock_detach_notifier(struct notifier_block *nb);
extern int register_dock_attach_notifier(struct notifier_block *nb);

extern int unregister_dock_atow_late_notifier(struct notifier_block *nb);
extern int unregister_dock_wtoa_late_notifier(struct notifier_block *nb);
extern int unregister_dock_detach_notifier(struct notifier_block *nb);
extern int unregister_dock_attach_notifier(struct notifier_block *nb);

extern int ite_touchpad_switch(SYSTEM_OWNER mode);
extern int Read_HW_ID(void);

//static function -------------------------------------------------------------------------------------------------
static int elan_i2c_switch_to_windows_notify(struct notifier_block *this,unsigned long code, void *data);
static int elan_i2c_attach_notify(struct notifier_block *this,unsigned long code, void *data);
static int elan_i2c_detach_notify(struct notifier_block *this,unsigned long code, void *data);
static ssize_t elan_i2c_get_status(struct device *dev, struct device_attribute *attr, char *buf);

static ssize_t elan_i2c_gpio_sw(struct device *class,struct device_attribute *attr,const char *buf, size_t count);

static DEVICE_ATTR(touchpad_status, (S_IWUSR|S_IRUGO), elan_i2c_get_status, NULL);
static DEVICE_ATTR(tp_i2c_sw, (S_IWUSR|S_IRUGO), NULL, elan_i2c_gpio_sw);

static struct attribute *touchpad_attr[] = {
	&dev_attr_touchpad_status.attr,
	&dev_attr_tp_i2c_sw.attr,
	NULL
};

static struct notifier_block dock_atow_late_notifier = {
        .notifier_call =    elan_i2c_switch_to_windows_notify,
};

static struct notifier_block dock_wtoa_late_notifier = {
        .notifier_call =    elan_i2c_attach_notify,
};

static struct notifier_block dock_detach_notifier = {
        .notifier_call =    elan_i2c_detach_notify,
};

static struct notifier_block dock_attach_notifier = {
        .notifier_call =    elan_i2c_attach_notify,
};

//-----------------------------------------------------------------------------------------------------------------

static int __elan_i2c_read_reg(struct i2c_client *client,
				 u16 reg, u8 *val, u16 len)
{
	struct i2c_msg msgs[2];
	u8 buf[2];
	int ret;
	int tries = ETP_I2C_COMMAND_TRIES;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;

	msgs[0].addr = elan_tp_data->client->addr;
	msgs[0].flags = elan_tp_data->client->flags & I2C_M_TEN;
	msgs[0].len = 2;
	msgs[0].buf = buf;

	msgs[1].addr = elan_tp_data->client->addr;
	msgs[1].flags = elan_tp_data->client->flags & I2C_M_TEN;
	msgs[1].flags |= I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = val;
		
	do {
		ret = i2c_transfer(elan_tp_data->client->adapter, msgs, 2);
		
		if (ret > 0)
			break;
		tries--;
		printk("[Touchpad]: retrying __elan_i2c_read_reg: 0x%x (%d) \n", reg, tries);
		
	} while (tries > 0);
	
	return ret != 2 ? -EIO : 0;
}

static int elan_i2c_read_reg(struct i2c_client *client, u16 reg, u8 *val)
{
	int retval;

	retval = __elan_i2c_read_reg(elan_tp_data->client, reg, val, ETP_INF_LENGTH);
	if (retval < 0) {
		printk("[Touchpad]: %s: reading register (0x%04x) failed! \n", __func__, reg);
		return retval;
	}
	return 0;
}

static int elan_i2c_write_reg_cmd(struct i2c_client *client, u16 reg, u16 cmd)
{
	struct i2c_msg msg;
	u8 buf[4];
	int ret;
	int tries = ETP_I2C_COMMAND_TRIES;

	buf[0] = reg & 0xff;
	buf[1] = (reg >> 8) & 0xff;
	buf[2] = cmd & 0xff;
	buf[3] = (cmd >> 8) & 0xff;

	msg.addr = elan_tp_data->client->addr;
	msg.flags = elan_tp_data->client->flags & I2C_M_TEN;
	msg.len = 4;
	msg.buf = buf;
		
	do {
		ret = i2c_transfer(elan_tp_data->client->adapter, &msg, 1);
		
		if (ret > 0)
			break;
		tries--;
		printk("[Touchpad]: retrying elan_i2c_write_reg_cmd: 0x%x (%d) \n", reg, tries);
		
	} while (tries > 0);
	
	return ret != 1 ? -EIO : 0;
}

static int elan_i2c_reset(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_tp_data->client, HID_CMD_REGISTER, CMD_RESET);
}

static int elan_i2c_wake_up(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_tp_data->client, HID_CMD_REGISTER, CMD_WAKE_UP);
}

static int elan_i2c_sleep(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_tp_data->client, HID_CMD_REGISTER, CMD_SLEEP);
}

static int elan_i2c_enable_absolute_mode(struct i2c_client *client)
{
	return elan_i2c_write_reg_cmd(elan_tp_data->client, ETP_CMD_REGISTER, CMD_ENABLE_ABS);
}

static int elan_i2c_get_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(elan_tp_data->client, REG_DESC, val, HID_DESC_LENGTH);
}

static int elan_i2c_get_report_desc(struct i2c_client *client, u8 *val)
{
	return __elan_i2c_read_reg(elan_tp_data->client, REG_REPORT_DESC, val, elan_tp_data->report_desc_length);
}

static int elan_i2c_get_x_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_tp_data->client, REG_X_AXIS_MAX, val);
}

static int elan_i2c_get_y_max(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_tp_data->client, REG_Y_AXIS_MAX, val);
}

static int elan_i2c_get_trace_num(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_tp_data->client, REG_XY_TRACE_NUM, val);
}

static int elan_i2c_get_fw_version(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_tp_data->client, REG_FW_VERSION, val);
}

static int elan_i2c_get_unique_id(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_tp_data->client, REG_UNIQUE_ID, val);
}

static int elan_i2c_get_resolution(struct i2c_client *client, u8 *val)
{
	return elan_i2c_read_reg(elan_tp_data->client, REG_RESOLUTION, val);
}

/*
*******************************************************************************
* Elan HID over i2c trackpad input device driver
*******************************************************************************
*/

int elan_i2c_bus_enable(int enable)
{	
	if(enable == 1)
	{
		printk("[Touchpad] %s: touchpad i2c bus enable.  \n", __func__);
		gpio_direction_output(elan_tp_data->tp_sw_gpio, 0);
	}
	else
	{
		printk("[Touchpad] %s: touchpad i2c bus disable.  \n", __func__);
		gpio_direction_output(elan_tp_data->tp_sw_gpio, 1);
	}
	return 0;
}
EXPORT_SYMBOL(elan_i2c_bus_enable);

static int elan_i2c_initialize(struct i2c_client *client)
{
//	struct device *dev = &client->dev;
//	u8 val[ETP_REPORT_DESC_LENGTH];
	u8 val[256];
	int rc;
	int i = 0;

	rc = elan_i2c_reset(elan_tp_data->client);
	if (rc < 0) {
		printk("[Touchpad]: %s: device reset failed. \n", __func__);
		elan_i2c_bus_enable(0);
		return -1;
	}

	/* wait for get reset return flag */
	msleep(100);
	/* get reset return flag 0000 */
	rc = i2c_master_recv(elan_tp_data->client, val, ETP_INF_LENGTH);
	if (rc < 0) {
		printk("[Touchpad]: %s: get device reset return value failed. \n", __func__);
		return -1;
	}

	printk("[Touchpad]:Get ETP_INF_LENGTH = %x, %x \n", val[0],val[1]);
	
	rc = elan_i2c_get_desc(elan_tp_data->client, val);
	if (rc < 0) {
		printk("[Touchpad]: %s: couldn't get device descriptor. \n", __func__);
		return -1;
	}
	
	elan_tp_data->desc_length = (val[1] << 2 | val[0]);
	printk("[Touchpad]: report_desc_length = %d \n", elan_tp_data->desc_length);
	elan_tp_data->report_desc_length = (val[5] << 2 | val[4]);
	printk("[Touchpad]: report_desc_length = %d \n", elan_tp_data->report_desc_length);
	elan_tp_data->report_length = (val[11] << 2 | val[10]);
	printk("[Touchpad]: report_length = %d \n", elan_tp_data->report_length);
			
//	printk("[Touchpad]: HID_DESC_LENGTH: ");
//	for (i = 0 ; i < elan_tp_data->desc_length; i++)
//	{
//		printk("0x%x ", val[i] );
//	}
//	printk(" ..\n" );
		
	rc = elan_i2c_get_report_desc(elan_tp_data->client, val);
	if (rc < 0) {
		printk("[Touchpad]: %s: fetching report descriptor failed. \n", __func__);
		return -1;
	}
	
//	printk("[Touchpad]: ETP_REPORT_DESC_LENGTH: ");
//	for (i = 0 ; i < elan_tp_data->report_desc_length; i++)
//	{
//		printk("0x%x ", val[i] );
//	}
//	printk(" ..\n" );
	
	return 0;
}

static void elan_i2c_report_absolute(struct elan_i2c_data *data, u8 *packet)
{
//	struct input_dev *input = data->input;
	u8 *finger_data = &packet[ETP_FINGER_DATA_OFFSET];
	bool finger_on;
	int pos_x = 0, pos_y = 0;
	int pressure, mk_x, mk_y;
	int i;
	int area_x, area_y, major, minor, new_pressure;
	int press_key = 0;
	
	if(elan_tp_data->input)
	{
		for (i = 0 ; i < ETP_MAX_FINGERS ; i++) {
			finger_on = (packet[3] >> (3 + i)) & 0x01;
    	
			if (finger_on) {
				pos_x = ((finger_data[0] & 0xf0) << 4) | finger_data[1];
				pos_y = ((finger_data[0] & 0x0f) << 8) | finger_data[2];
				pos_y =  data->max_y - pos_y;
				mk_x = (finger_data[3] & 0x0f);
				mk_y = (finger_data[3] >> 4);
				pressure = finger_data[4];
    	
				area_x = mk_x * (data->width_x - 90);
				area_y = mk_y * (data->width_y - 90);
    	
				major = max(area_x, area_y);
				minor = min(area_x, area_y);
				new_pressure = pressure+ETP_MIN_TAP_PRESSURE;
				if (new_pressure > 255)
					new_pressure = 255;
				
			//	printk("[Touchpad]i=%d,pos_x=%d,pos_y=%d,mk_x=%d,mk_y=%d,pressure=%d \n",i,pos_x,pos_y,mk_x,mk_y,pressure);
				
				input_mt_slot(elan_tp_data->input, i);
				input_mt_report_slot_state(elan_tp_data->input, MT_TOOL_FINGER, true);
			//	input_report_abs(elan_tp_data->input, ABS_MT_TRACKING_ID, i);
				input_report_abs(elan_tp_data->input, ABS_MT_POSITION_X, pos_x);
				input_report_abs(elan_tp_data->input, ABS_MT_POSITION_Y, pos_y);
				input_report_abs(elan_tp_data->input, ABS_MT_PRESSURE, new_pressure);
				input_report_abs(elan_tp_data->input, ABS_TOOL_WIDTH, mk_x);
				input_report_abs(elan_tp_data->input, ABS_MT_TOUCH_MAJOR, major);
				input_report_abs(elan_tp_data->input, ABS_MT_TOUCH_MINOR, minor);
				finger_data += ETP_FINGER_DATA_LEN;
			} else {
				input_mt_slot(elan_tp_data->input, i);
				input_mt_report_slot_state(elan_tp_data->input, MT_TOOL_FINGER, false);
			}
		}
		
		press_key = (packet[3] & 0x01);
//		printk("[Touchpad]: press_key = %d \n", press_key);
		if((pos_x >= 0 && pos_x <= 1480) && (pos_y >= 987 && pos_y <= 1480) && press_key)
		{
		//	printk("[Touchpad]: Press Left Button. \n");
			elan_tp_data->button_key = BTN_LEFT;	
		}
		else if((pos_x >= 1480 && pos_x <= 2690) && (pos_y >= 987 && pos_y <= 1480) && press_key)
		{
		//	printk("[Touchpad]: Press Right Button. \n");
			elan_tp_data->button_key = BTN_RIGHT;			
			//if((factory_mode == 2) && (entry_mode == 1))
			//{
			//	printk("[Touchpad]: Factory image send KeyEvent.KEYCODE_BACK. \n");
			//	
			//	input_report_key(elan_tp_data->input, KEY_BACK, 1);
			//	input_sync(elan_tp_data->input); 
			//	input_report_key(elan_tp_data->input, KEY_BACK, 0);
			//	input_sync(elan_tp_data->input); 
			//}
		}
		
		if(press_key != elan_tp_data->press_button)
		{
			input_report_key(elan_tp_data->input, elan_tp_data->button_key, press_key);
			elan_tp_data->press_button = press_key;
		}
			
		input_mt_report_pointer_emulation(elan_tp_data->input, true);
		input_sync(elan_tp_data->input);
	}
}

static int elan_i2c_check_packet(struct elan_i2c_data *data, u8 *packet)
{
	u16 length = le16_to_cpup((__le16 *)packet);
	u8 report_id = packet[HID_REPORT_ID_OFFSET];
		
	if ((length != elan_tp_data->report_length) || (report_id != ETP_REPORT_ID))
	{
		printk(KERN_ERR "[Touchpad] %s: check package fail! length=%d, id=%x. \n", __func__, length, report_id);
		queue_work(elan_tp_data->touchpad_wq, &elan_tp_data->elan_init_abs_work);
		return -1;
	}

	return 0;
}

static irqreturn_t elan_i2c_isr(int irq, void *dev_id)
{
//	struct elan_i2c_data *data = dev_id;
	u8 packet[64];
	int retval;

	retval = i2c_master_recv(elan_tp_data->client, packet, elan_tp_data->report_length);

	if (retval != elan_tp_data->report_length || elan_i2c_check_packet(elan_tp_data, packet))
	{
		printk(KERN_ERR "[Touchpad] %s: wrong packet data.\n", __func__);
		goto elan_isr_end;
	}

	elan_i2c_report_absolute(elan_tp_data, packet);

elan_isr_end:
	return IRQ_HANDLED;
}

/*
 * (value from firmware) * 10 + 790 = dpi
 * we also have to convert dpi to dots/mm (*10/254 to avoid floating point)
 */
static unsigned int elan_i2c_convert_res(unsigned int val)
{
	return (val * 10 + 790) * 10 / 254;
}

//static int elan_i2c_input_dev_destroy(void)
static int elan_i2c_input_dev_destroy(struct work_struct *dat)
{
	printk("[Touchpad] %s: (0x%x) ++ \n", __func__, elan_tp_data->input);
	
	if(elan_tp_data->input)
	{
		printk("[Touchpad] %s: input_unregister_device \n", __func__);
		input_unregister_device(elan_tp_data->input);
		elan_tp_data->input = NULL;
	}
	return 0;
}

static int elan_i2c_input_dev_create(struct elan_i2c_data *data)
{
//	struct i2c_client *client = data->client;
//	struct input_dev *input;
	unsigned int x_res, y_res;
	u8 val[2];
	int ret;
	u16	unique_id;
	u16	fw_version;

	printk("[Touchpad] %s: (0x%x) ++\n", __func__, elan_tp_data->input);
	if(elan_tp_data->input)
	{
		printk("[Touchpad]%s: input device is exist. \n",__func__);
		input_unregister_device(elan_tp_data->input);
		elan_tp_data->input = NULL;
	}

	elan_tp_data->input = input_allocate_device();
	if (!elan_tp_data->input)
	{
		printk("[Touchpad] %s: input_allocate_device failed! \n",__func__);
		return -ENOMEM;
	}
		
	elan_tp_data->input->name = "Elan-I2C-Touchpad";
	elan_tp_data->input->id.bustype = BUS_I2C;
	elan_tp_data->input->dev.parent = &elan_tp_data->client->dev;
//	printk("[Touchpad] %s:[%d]: elan_tp_data->input->dev.parent =%x \n", __func__, __LINE__, elan_tp_data->input->dev.parent);

	__set_bit(INPUT_PROP_POINTER, elan_tp_data->input->propbit);
	__set_bit(INPUT_PROP_BUTTONPAD, elan_tp_data->input->propbit);
	__set_bit(EV_KEY, elan_tp_data->input->evbit);
	__set_bit(EV_ABS, elan_tp_data->input->evbit);

	__set_bit(BTN_LEFT, elan_tp_data->input->keybit);
	__set_bit(BTN_RIGHT, elan_tp_data->input->keybit);
	
	__set_bit(BTN_TOUCH, elan_tp_data->input->keybit);
	__set_bit(BTN_TOOL_FINGER, elan_tp_data->input->keybit);
	__set_bit(BTN_TOOL_DOUBLETAP, elan_tp_data->input->keybit);
	__set_bit(BTN_TOOL_TRIPLETAP, elan_tp_data->input->keybit);
	__set_bit(BTN_TOOL_QUADTAP, elan_tp_data->input->keybit);
	__set_bit(BTN_TOOL_QUINTTAP, elan_tp_data->input->keybit);

//	__set_bit(ABS_MT_TRACKING_ID, elan_tp_data->input->absbit);
	__set_bit(ABS_MT_TOUCH_MAJOR, elan_tp_data->input->absbit);
	__set_bit(ABS_MT_TOUCH_MINOR, elan_tp_data->input->absbit);
	__set_bit(ABS_MT_POSITION_X, elan_tp_data->input->absbit);
	__set_bit(ABS_MT_POSITION_Y, elan_tp_data->input->absbit);

	elan_i2c_get_unique_id(elan_tp_data->client, val);
	unique_id = le16_to_cpup((__le16 *)val);

	elan_i2c_get_fw_version(elan_tp_data->client, val);
	fw_version = le16_to_cpup((__le16 *)val);

	if (fw_version == -1) {
		data->max_x = ETP_DEF_MAX_X;
		data->max_y = ETP_DEF_MAX_Y;
		data->width_x = data->max_x / (ETP_DEF_TRACENUM_X - 1);
		data->width_y = data->max_y / (ETP_DEF_TRACENUM_Y - 1);
		x_res = elan_i2c_convert_res(ETP_DEF_RES_X);
		y_res = elan_i2c_convert_res(ETP_DEF_RES_Y);
	} else {

		elan_i2c_get_x_max(elan_tp_data->client, val);
		elan_tp_data->max_x = (0x0f & val[1]) << 8 | val[0];

		elan_i2c_get_y_max(elan_tp_data->client, val);
		elan_tp_data->max_y = (0x0f & val[1]) << 8 | val[0];

		elan_i2c_get_trace_num(elan_tp_data->client, val);
		elan_tp_data->width_x = elan_tp_data->max_x / (val[0] - 1);
		elan_tp_data->width_y = elan_tp_data->max_y / (val[1] - 1);

		elan_i2c_get_resolution(elan_tp_data->client, val);
		x_res = elan_i2c_convert_res(val[0]);
		y_res = elan_i2c_convert_res(val[1]);
	}

	printk("Elan I2C Trackpad Information:\n" \
		"    Module unique ID:  0x%04x\n" \
		"    Firmware Version:  0x%04x\n" \
		"    Max ABS X,Y:   %d,%d\n" \
		"    Width X,Y:   %d,%d\n" \
		"    Resolution X,Y:   %d,%d (dots/mm)\n",
		unique_id,
		fw_version,
		elan_tp_data->max_x, elan_tp_data->max_y,
		elan_tp_data->width_x, elan_tp_data->width_y,
		x_res, y_res);
		
//	input_set_abs_params(elan_tp_data->input, ABS_MT_TRACKING_ID, 0, ETP_MAX_FINGERS, 0, 0);
	input_set_abs_params(elan_tp_data->input, ABS_X, 0, elan_tp_data->max_x, 0, 0);
	input_set_abs_params(elan_tp_data->input, ABS_Y, 0, elan_tp_data->max_y, 0, 0);
	input_abs_set_res(elan_tp_data->input, ABS_X, x_res);
	input_abs_set_res(elan_tp_data->input, ABS_Y, y_res);
	input_set_abs_params(elan_tp_data->input, ABS_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(elan_tp_data->input, ABS_TOOL_WIDTH, 0, 15, 0, 0);

	input_mt_init_slots(elan_tp_data->input, ETP_MAX_FINGERS);
	input_set_abs_params(elan_tp_data->input, ABS_MT_POSITION_X, 0, elan_tp_data->max_x, 0, 0);
	input_set_abs_params(elan_tp_data->input, ABS_MT_POSITION_Y, 0, elan_tp_data->max_y, 0, 0);
	input_abs_set_res(elan_tp_data->input, ABS_MT_POSITION_X, x_res);
	input_abs_set_res(elan_tp_data->input, ABS_MT_POSITION_Y, y_res);
	input_set_abs_params(elan_tp_data->input, ABS_MT_PRESSURE, 0, 255, 0, 0);
	input_set_abs_params(elan_tp_data->input, ABS_MT_TOUCH_MAJOR, 0, 15 * max(elan_tp_data->width_x, elan_tp_data->width_y), 0, 0);
	input_set_abs_params(elan_tp_data->input, ABS_MT_TOUCH_MINOR, 0, 15 * min(elan_tp_data->width_x, elan_tp_data->width_y), 0, 0);

	/* Register the device in input subsystem */
	ret = input_register_device(elan_tp_data->input);
	if (ret) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: input device register failed, %d\n", __func__, __LINE__, ret);
		goto err_free_device;
	}
	
	return 0;

err_free_device:
	input_free_device(elan_tp_data->input);
	elan_tp_data->input = NULL;
	return ret;
}

static ssize_t elan_i2c_get_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	printk(KERN_INFO "[Touchpad]:touchpad status=%d. \n",elan_tp_data->touchpadp_status);
	return sprintf(buf, "%d \n", elan_tp_data->touchpadp_status);
}

static ssize_t elan_i2c_gpio_sw(struct device *class,struct device_attribute *attr,const char *buf, size_t count)
{
	if (buf[0] == '0')
	{
		elan_i2c_bus_enable(0);
	}
	else if(buf[0] == '1')
	{
		elan_i2c_bus_enable(1);
	}
	else
	{
		printk("[ITE] %s: Unknown Command. \n",__func__);
	}
	
	return count;
}

static int elan_i2c_register_irq(void)
{
	int ret = 0;
	
	printk("[Touchpad] %s irq_status=%d, tp_enable=%d ++ \n",__func__, elan_tp_data->irq_status, elan_tp_data->tp_enable);
	
	if(elan_tp_data->irq_status)
	{
		free_irq(elan_tp_data->irq, elan_tp_data);
		elan_tp_data->irq_status = 0;
	}
	if(elan_tp_data->tp_enable == 1)
	{
		ret = request_threaded_irq(elan_tp_data->irq, NULL, elan_i2c_isr, IRQF_TRIGGER_FALLING,
									elan_tp_data->client->name, elan_tp_data);
		elan_tp_data->irq_status = 1;
	}
	
	return ret;
}

static int elan_i2c_init_function(struct work_struct *dat)
{
	int ret;
	int owner;
	
	printk(KERN_INFO "[Touchpad]: %s \n",__func__);
	
	if(elan_tp_data->panel_owner == SYSTEM_WINDOWS)
	{
		printk("[Touchpad]: %s: panel owner is windows, do nothing. \n",__func__, owner);
		return 1;
	}	
		
	mutex_lock(&elan_tp_data->mutex_lock);
	
	elan_i2c_bus_enable(1);
	if((elan_tp_data->hw_id == HW_ID_ER)||(elan_tp_data->hw_id == HW_ID_ER2)||(elan_tp_data->hw_id == HW_ID_SR1)||(elan_tp_data->hw_id == HW_ID_SR2))
	{
		ite_touchpad_switch(SYSTEM_ANDROID);
	}
	msleep(10);
	
	ret = elan_i2c_initialize(elan_tp_data->client);
	if (ret < 0)
	{
		printk(KERN_ERR "[Touchpad]: Failed to i2c initialize, error=%d\n", ret);
		elan_i2c_bus_enable(0);
		goto err;
	}

	ret = elan_i2c_input_dev_create(elan_tp_data);
	if (ret < 0)
	{
		printk(KERN_ERR "[Touchpad]: Failed to input device create, error=%d\n", ret);	
		goto err;
	}
	
	ret = elan_i2c_register_irq();
	
//	ret = request_threaded_irq(elan_tp_data->irq, NULL, elan_i2c_isr, IRQF_TRIGGER_FALLING,
//								elan_tp_data->client->name, elan_tp_data);
//	if (ret < 0) {
//	 	printk(KERN_ERR "[Touchpad]: Could not register interrupt for, irq=%d, ret=%d\n", elan_tp_data->irq, ret);
//	 	return 1;
//	}
		
	ret = elan_i2c_enable_absolute_mode(elan_tp_data->client);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad] %s: can't switch to absolute mode.\n", __func__);
		goto err;
	}
	
	//Joe elan_i2c_sleep(elan_tp_data->client);
	//Joe msleep(10);
	
	ret = elan_i2c_wake_up(elan_tp_data->client);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad] %s: device wake up failed.\n",__func__);
		goto err;
	}

	elan_tp_data->touchpadp_status = 1;
	mutex_unlock(&elan_tp_data->mutex_lock);
	return 0;
	
err:
	mutex_unlock(&elan_tp_data->mutex_lock);
	return 1;
}

static int elan_i2c_resume_function(struct work_struct *dat)
{
	int ret = 0;
	
	printk("[Touchpad]: %s \n", __func__);
	ret = elan_i2c_initialize(elan_tp_data->client);
	if (ret < 0)
	{
		printk(KERN_ERR "[Touchpad] %s: elan_i2c_initialize fail!\n", __func__);
		goto err;
	}
	ret = elan_i2c_enable_absolute_mode(elan_tp_data->client);
	if (ret < 0)
	{
		printk(KERN_ERR "[Touchpad] %s: elan_i2c_enable_absolute_mode fail!\n", __func__);
		goto err;
	}
	ret = elan_i2c_wake_up(elan_tp_data->client);
	if (ret < 0)
	{
		printk("[Touchpad]: resume active power failed, %d\n", ret);
		goto err;
	}
	else
		printk("[Touchpad]: Elan tp wakeup.\n");
	
	if(elan_tp_data->irq_status == 1)
		enable_irq(elan_tp_data->irq);

	return 0;
	
err:
	elan_i2c_bus_enable(0);
	return 1;
}

static int elan_i2c_switch_to_windows_notify(struct notifier_block *this, unsigned long owner, void *data)
{
	printk("[Touchpad]: %s ++ : owner:%d \n", __func__, (int)owner);
	
	if(elan_tp_data->touchpadp_status)
	{
		printk("[Touchpad]: device change to win8 , touchpad sleep. \n");
		mutex_lock(&elan_tp_data->mutex_lock);
		elan_tp_data->panel_owner = SYSTEM_WINDOWS;
		elan_i2c_sleep(elan_tp_data->client);
		if(elan_tp_data->irq_status)
		{
			free_irq(elan_tp_data->irq, elan_tp_data);
			elan_tp_data->irq_status = 0;
		}
		elan_i2c_bus_enable(0);
		if((elan_tp_data->hw_id == HW_ID_ER)||(elan_tp_data->hw_id == HW_ID_ER2)||(elan_tp_data->hw_id == HW_ID_SR1)||(elan_tp_data->hw_id == HW_ID_SR2))
		{
			ite_touchpad_switch(SYSTEM_WINDOWS);
		}
		mutex_unlock(&elan_tp_data->mutex_lock);
	}
	
	return 1;
}

static int elan_i2c_attach_notify(struct notifier_block *this, unsigned long owner, void *data)
{
	printk("[Touchpad]: %s ++ : owner:%d, panel_owner =0x%x \n", __func__, (int)owner, elan_tp_data->panel_owner);
	if(owner == SYSTEM_WINDOWS)
	{
		elan_tp_data->panel_owner = SYSTEM_WINDOWS;
		printk("[Touchpad]: %s: attach and switch to windows, not need work! \n", __func__);
	}
	else
	{	
		printk("[Touchpad]: %s:  owner is android. \n", __func__);
		elan_tp_data->panel_owner = SYSTEM_ANDROID;
		queue_delayed_work(elan_tp_data->touchpad_wq, &elan_tp_data->elan_init_work, 1.2*HZ);
	}
	return 1;
}

static int elan_i2c_detach_notify(struct notifier_block *this, unsigned long owner, void *data)
{
	printk("[Touchpad]: PAD and Base detach , unregister touchpad.\n");
	mutex_lock(&elan_tp_data->mutex_lock);
	elan_tp_data->panel_owner = SYSTEM_WINDOWS;
	if(elan_tp_data->irq_status)
	{	
		free_irq(elan_tp_data->irq, elan_tp_data);
		elan_tp_data->irq_status = 0;
	}
	elan_i2c_bus_enable(0);
	if((elan_tp_data->hw_id == HW_ID_ER)||(elan_tp_data->hw_id == HW_ID_ER2)||(elan_tp_data->hw_id == HW_ID_SR1)||(elan_tp_data->hw_id == HW_ID_SR2))
	{
		ite_touchpad_switch(SYSTEM_WINDOWS);
	}
	mutex_unlock(&elan_tp_data->mutex_lock);
	queue_delayed_work(elan_tp_data->touchpad_wq, &elan_tp_data->elan_destroy_work, 0);
	
	return 1;
}

int elan_i2c_touchpad_enable(int enable)
{
	int ret = 0;
	
	if(elan_tp_data->panel_owner == SYSTEM_ANDROID)
	{
		printk("[Touchpad] %s: enable = %d \n", __func__, enable);
		if(enable == 1)
		{
			elan_tp_data->tp_enable = 1;
			if(elan_tp_data->irq_status == 0)
			{
				ret = elan_i2c_wake_up(elan_tp_data->client);
				if (ret < 0) {
					printk(KERN_ERR "[Touchpad]: device wake up failed.\n");
					return 1;
				}
				ret = elan_i2c_register_irq();
			}
		}
		else
		{
			elan_tp_data->tp_enable = 0;
			if(elan_tp_data->irq_status == 1)
			{
				elan_i2c_sleep(elan_tp_data->client);
				free_irq(elan_tp_data->irq, elan_tp_data);
				elan_tp_data->irq_status = 0;
			}
		}
	}
	return 0;
}
EXPORT_SYMBOL(elan_i2c_touchpad_enable);

static int elan_i2c_init_abs_mode(struct work_struct *dat)
{
	int ret = 0;
	
	printk("[Touchpad] %s ++ \n",__func__);
	
	ret = elan_i2c_sleep(elan_tp_data->client);
	if (ret < 0)
		printk("[Touchpad] %s: suspend mode failed, %d\n",__func__, ret);
	
	ret = elan_i2c_register_irq();
	
	if (ret < 0) {
	 	printk(KERN_ERR "[Touchpad] %s: Could not register interrupt for, irq=%d, ret=%d\n", __func__, elan_tp_data->irq, ret);
	 	return 1;
	}
	
	ret = elan_i2c_enable_absolute_mode(elan_tp_data->client);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad] %s: can't switch to absolute mode.\n",__func__);
	}
	
	ret = elan_i2c_wake_up(elan_tp_data->client);
	if (ret < 0) {
		printk(KERN_ERR "[Touchpad] %s: device wake up failed.\n",__func__);
		return 1;
	}
	
	return 0;
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void elan_i2c_early_suspend(struct early_suspend *h)
{
	int ret = 0;
	
//	printk("[Touchpad] %s:  \n",__func__);
	
	if(elan_tp_data->panel_owner == SYSTEM_ANDROID)
	{
		printk("[Touchpad] %s ++  \n", __func__);
		
		if(elan_tp_data->irq_status == 1)
			disable_irq(elan_tp_data->irq);
	
		ret = elan_i2c_sleep(elan_tp_data->client);
		if (ret < 0)
			printk("[Touchpad] %s: suspend mode failed, %d\n", __func__, ret);
		else
			printk("[Touchpad]: Elan tp going to suspend!\n");
	}
}

static void elan_i2c_late_resume(struct early_suspend *h)
{
		
	if(elan_tp_data->panel_owner == SYSTEM_ANDROID)
	{
		printk("[Touchpad] %s ++ \n",__func__);
		queue_delayed_work(elan_tp_data->touchpad_wq, &elan_tp_data->elan_resume_work, 1.2*HZ);
	}
}
#endif

static int __devinit elan_i2c_probe(struct i2c_client *client,
				    const struct i2c_device_id *dev_id)
{

	struct elan_touchpad_i2c_platform_data *pdata;
	int ret;
	
	printk("[Touchpad] %s ++ \n", __func__);
	
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: i2c check functionality error.\n", __func__, __LINE__);
		ret = -ENODEV;
		goto probe_err_check_functionality_failed;
	}

	elan_tp_data = kzalloc(sizeof(struct elan_i2c_data), GFP_KERNEL);
	if (elan_tp_data == NULL) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: allocate ite_chip failed.\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto probe_err_alloc_data_failed;
	}

	elan_tp_data->touchpad_wq = create_singlethread_workqueue("touchpad_wq");
	if (!elan_tp_data->touchpad_wq) {
		printk(KERN_ERR "[Touchpad] %s:[%d]: create workqueue failed.\n", __func__, __LINE__);
		ret = -ENOMEM;
		goto probe_err_create_wq_failed;
	}
	
	elan_tp_data->hw_id = Read_HW_ID();
	
	elan_tp_data->client = client;
	i2c_set_clientdata(client, elan_tp_data);
	pdata = client->dev.platform_data;
	if (likely(pdata != NULL)) {
		elan_tp_data->int_gpio = pdata->int_gpio;
				
		if((elan_tp_data->hw_id == HW_ID_PR)||(elan_tp_data->hw_id == HW_ID_MP))
			elan_tp_data->tp_sw_gpio = pdata->tp_sw_gpio;
		else
			elan_tp_data->tp_sw_gpio = 51;//hard coding.
	}	
	printk("[Touchpad] %s:[%d]: int_gpio =%d, tp_sw_gpio =%d \n", __func__, __LINE__, elan_tp_data->int_gpio, elan_tp_data->tp_sw_gpio);
	
	/*init interrupt pin*/
	ret = gpio_request(elan_tp_data->int_gpio, "touchpad-irq");
	if(ret < 0)
		printk(KERN_ERR "[Touchpad]: Failed to request GPIO%d (touchpad-irq) error=%d\n", elan_tp_data->int_gpio, ret);

	ret = gpio_direction_input(elan_tp_data->int_gpio);
	if (ret){
		printk(KERN_ERR "[Touchpad]: Failed to set interrupt direction, error=%d\n", ret);
		goto probe_err_gpio_direction_input_failed;
	}
	
	//init TP_I2C_SW pin
 	ret = gpio_request(elan_tp_data->tp_sw_gpio, "TP_I2C_SW");
	if (ret < 0)
		printk(KERN_ERR "[Touchpad] %s:Failed to request GPIO%d (TP_I2C_SW) error=%d\n", __func__, elan_tp_data->tp_sw_gpio, ret);

	ret = gpio_direction_output(elan_tp_data->tp_sw_gpio, 1);
	if (ret){
		printk(KERN_ERR "[Touchpad] %s:Failed to set reset direction, error=%d\n", __func__, ret);
		gpio_free(elan_tp_data->tp_sw_gpio);
	}
		
	elan_tp_data->irq = gpio_to_irq(elan_tp_data->int_gpio);
	printk("[Touchpad]: intr_gpio=%d, irq=%d \n", elan_tp_data->int_gpio, elan_tp_data->irq);
	
	elan_tp_data->touchpadp_status = 0;
	elan_tp_data->input = NULL;
	elan_tp_data->press_button = 0;
	elan_tp_data->button_key = KEY_RESERVED;
	elan_tp_data->tp_enable = 1;
	elan_tp_data->panel_owner = 0;
//	elan_tp_data->press_count = 0;
	
	mutex_init(&elan_tp_data->mutex_lock);
	
	INIT_DELAYED_WORK(&elan_tp_data->elan_init_work, elan_i2c_init_function);
	INIT_DELAYED_WORK(&elan_tp_data->elan_destroy_work, elan_i2c_input_dev_destroy);
	INIT_DELAYED_WORK(&elan_tp_data->elan_resume_work, elan_i2c_resume_function);
	INIT_WORK(&elan_tp_data->elan_init_abs_work, elan_i2c_init_abs_mode);
		
	elan_tp_data->attrs.attrs = touchpad_attr;
    ret = sysfs_create_group(&client->dev.kobj, &elan_tp_data->attrs);
    if (ret) {
        printk(KERN_ERR "[Touchpad]: Not able to create the sysfs.\n");
        goto probe_err_create_sysfs;
    }

	device_init_wakeup(&elan_tp_data->client->dev, 1);
	
	register_dock_atow_late_notifier(&dock_atow_late_notifier);
	register_dock_wtoa_late_notifier(&dock_wtoa_late_notifier);
	register_dock_detach_notifier(&dock_detach_notifier);
	register_dock_attach_notifier(&dock_attach_notifier);
	
	#ifdef CONFIG_HAS_EARLYSUSPEND
    elan_tp_data->early_suspend.level = EARLY_SUSPEND_LEVEL_DISABLE_FB - 1;
    elan_tp_data->early_suspend.suspend = elan_i2c_early_suspend;
    elan_tp_data->early_suspend.resume = elan_i2c_late_resume;
    register_early_suspend(&elan_tp_data->early_suspend);
#endif
	
	printk("[Touchpad]: Elan Touchpad Driver ver:%s . \n", DRIVER_VERSION);

	return 0;

//probe_err_switch_mode:
probe_err_create_sysfs:
//	free_irq(elan_tp_data->irq, elan_tp_data);
	elan_tp_data->irq_status = 0;
//probe_err_irq:
//	input_unregister_device(elan_tp_data->input);
//probe_err_input_dev:
//probe_err_i2c_init:
probe_err_gpio_direction_input_failed:
	gpio_free(elan_tp_data->int_gpio);
probe_err_create_wq_failed:
	if (elan_tp_data->touchpad_wq)
		destroy_workqueue(elan_tp_data->touchpad_wq);
        
	kfree(elan_tp_data);
probe_err_alloc_data_failed:
probe_err_check_functionality_failed:

//err_switch_mode:
//	free_irq(data->irq, data);
//err_irq:
//	input_free_device(data->input);
//err_input_dev:
//	kfree(data);
//err_init:
//	dev_err(&client->dev, "Elan I2C Trackpad probe and initial fail!\n");
	return ret;
}
 
static int __devexit elan_i2c_remove(struct i2c_client *client)
{
//	struct elan_i2c_data *data = i2c_get_clientdata(client);

	free_irq(elan_tp_data->irq, elan_tp_data);
	input_unregister_device(elan_tp_data->input);
	unregister_dock_atow_late_notifier(&dock_atow_late_notifier);
	unregister_dock_wtoa_late_notifier(&dock_wtoa_late_notifier);
	unregister_dock_detach_notifier(&dock_detach_notifier);
	unregister_dock_attach_notifier(&dock_attach_notifier);
	kfree(elan_tp_data);
	
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int elan_i2c_suspend(struct device *dev)
{
//	printk("[Touchpad] %s:  \n",__func__);
	return 0;
}

static int elan_i2c_resume(struct device *dev)
{	
//	printk("[Touchpad] %s:  \n",__func__);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(elan_i2c_pm_ops, elan_i2c_suspend, elan_i2c_resume);

static const struct i2c_device_id elan_i2c_id[] = {
	{ ELAN_TOUCHPAD_NAME, 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, elan_i2c_id);

static struct i2c_driver elan_i2c_driver = {
	.driver = {
		.name	= ELAN_TOUCHPAD_NAME,
		.owner	= THIS_MODULE,
		.pm	= &elan_i2c_pm_ops,
	},
	.probe		= elan_i2c_probe,
	.remove		= __devexit_p(elan_i2c_remove),
	.id_table	= elan_i2c_id,
};

static int __init elan_i2c_init(void)
{
	int ret;
	
	printk("[Touchpad]: %s ++ \n", __func__);
	ret = i2c_add_driver(&elan_i2c_driver);
	if (ret) {
		printk("[Touchpad]: elan driver register FAILED.\n");
		return ret;
	}

	return ret;
}

static void __exit elan_i2c_exit(void)
{
	i2c_del_driver(&elan_i2c_driver);
}

module_init(elan_i2c_init);
module_exit(elan_i2c_exit);

MODULE_AUTHOR("Duson Lin <dusonlin@emc.com.tw>");
MODULE_DESCRIPTION("Elan I2C Touchpad driver");
MODULE_LICENSE("GPL");
