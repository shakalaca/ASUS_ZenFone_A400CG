/*
 * microp_ene.c - driver for EEPROM of Ene
 *
 * Copyright (C) 2011 Sina Chou <sina_chou@asus.com>
 *
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/workqueue.h>
#include <linux/microp.h>
#include <linux/microp_notify.h>
#include <linux/microp_pin_def.h>
#include <linux/uaccess.h>
#include <linux/microp_api.h>
#include <linux/switch.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/mfd/pm8xxx/pm8921.h>
#include <linux/proc_fs.h>
#include <asm/intel-mid.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_scu_ipc.h>
#include <linux/input/intel_mid_vibra.h>
#include <linux/string.h>//Eric
#include <linux/cpufreq.h>
#include <linux/earlysuspend.h>
#include <linux/wakelock.h>
#include "microp_ene.h"

#ifdef CONFIG_FASTBOOT
#include <linux/fastboot.h>
#endif //#ifdef CONFIG_FASTBOOT

#define MICROP_NAME "microp"
#define PAD_INT_GPIO_NAME "PAD_IRQ_N"
#define PAD_PLUGIN_GPIO_NAME "PAD_PLUG_IN_N"
#define RF_SWITCH_GPIO_NAME "RF_SW"

//static char rf_switch_status[10]; // Walf

unsigned int g_firmwareSize = 16*1024;

extern int hex_to_bin(char ch);//Eric

static int is_first_bootup = 1;
static int rf_switch_gpio = -1;
static uint8_t is_microp_init = 0;

struct ioctl_data {
    int time_interval;
    int total_counts;
};
struct microP_fwinfo {
    char cmd_style[5];
    int ap_version;
    int ld_version;
    int battery_id;
    int config_id;
};
struct microP_fwinfo  fw_info;

void set_antenna_to_phone(void)
{
    pr_info("%s\r\n",__FUNCTION__);
    printk("MicroP: switch antenna to phone \r\n");
    if (rf_switch_gpio>=0)
        gpio_set_value(rf_switch_gpio,0);
    //strcpy(rf_switch_status, "phone");
}

void set_antenna_to_pad_main(void)
{
    pr_info("%s\r\n",__FUNCTION__);
    printk("MicroP: switch antenna to pad \r\n");
    if (rf_switch_gpio>=0)
        gpio_set_value(rf_switch_gpio,1);
    //strcpy(rf_switch_status, "pad_main");      
}

struct microP_info {
    struct i2c_client *i2c_client;
    struct microP_platform_data *pdata;
    struct delayed_work work;
    struct delayed_work initP01;
    struct delayed_work deinitPad;
    struct delayed_work poweroffP01;
    struct delayed_work paddetect;
#ifdef CONFIG_HAS_EARLYSUSPEND
    struct early_suspend es;
#endif
    struct mutex write_read_lock;
    int  paddetect_gpio_value;
};

struct i2c_microp_command{
    char *name;;
    u8 addr;
    u8 len;
    enum readwrite{
                E_READ=0,
                E_WRITE=1,
                E_READWRITE=2,
                E_NOUSE=3,

    }rw;

    enum   usedMode{
                E_MODE_APROM=0,
                E_MODE_LDROM=1,
                E_MODE_BOTH=2,
    }mode;

    enum   supportID{
                Prj_A12=0x01,
    }prj;

};

/*
*       addr: length
*       
*/
struct i2c_microp_command uP_CMD_Table[]={
    {"hw_id"             , 0x00,  2,      E_READ,  E_MODE_BOTH, Prj_A12},         //MICROP_HW_ID
//    {"fw_ver"            , 0x01,  2,      E_READ,  E_MODE_BOTH,       0},         //MICROP_FW_VER
    {"ldrom_id"          , 0x03,  2,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_LDROM_ID_CODE
    {"hw_factory_isn"    , 0x04, 32, E_READWRITE,  E_MODE_BOTH, Prj_A12},         //MICROP_ISN
    {"phone_ready"       , 0x05,  1, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_IND_PHONE_READY
    {"power_on_reason"   , 0x07,  1,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_POWER_ON_REASON
    {"factory_mode"      , 0x08,  1, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_DISABLE_CHARGING_FOR_FACTORY
    {"light_sensor_kData", 0x0A,  4, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_CALIBRATION_DATA
    {"ignore_phone_ready", 0x0B,  1, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_ALWAYS_IGNORE_PHONE_READY
    {"OEM_SSN"           , 0x0D, 32, E_READWRITE,  E_MODE_BOTH, Prj_A12},         //MICROP_OEM_SSN
    {"OEM_IMEI"          , 0x0E, 32, E_READWRITE,  E_MODE_BOTH, Prj_A12},         //MICROP_OEM_IMEI
    {"microp_oem_ver"    , 0x0F, 24,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_OEM_FW_VERSION
    {"input_lvl"         , 0x20,  4,      E_READ,  E_MODE_BOTH, Prj_A12},         //MICROP_GPIO_INPUT_LEVEL
    {"out_lvl"           , 0x21,  4, E_READWRITE,  E_MODE_BOTH, Prj_A12},         //MICROP_GPIO_OUTPUT_LEVEL
    {"out_lvr_bit_set"   , 0x22,  4,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_GPIO_OUTPUT_BIT_SET
    {"out_lvr_bit_clr"   , 0x23,  4,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_GPIO_OUTPUT_BIT_CLR
    {"pad_hwid"          , 0x26,  1,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_PAD_HWID
    {"proxim_kData"      , 0x27, 22, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_PROXIM_KDATA
    {"charging_status"   , 0x30,  1,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_CHARGING_STATUS
    {"gauge_id"          , 0x31,  2,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_GAUGE_I
    {"usb_det"           , 0x34,  1,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_USB_DET
    {"usb_type"          , 0x37,  1,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_USB_TYPE
    {"battery_info"      , 0x38,  6,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_BATTERY_INFO
    {"battery_soc"       , 0x39,  2,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_BATTERY_SOC
    {"battery_config"    , 0x3A,  4,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_BATTERY_CONFIG
    {"pwm"               , 0x40,  1, E_READWRITE,  E_MODE_BOTH, Prj_A12},         //MICROP_PWM
    {"intr_status"       , 0x41,  4,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_INTR_STATUS
    {"intr_en"           , 0x42,  4, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_INTR_EN
    {"intr_en_bit_set"   , 0x43,  4,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_INTR_EN_BIT_SET
    {"intr_en_bit_clr"   , 0x44,  4,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_INTR_EN_BIT_CLR
    {"boot_sel"          , 0x50,  1,      E_READ,  E_MODE_BOTH, Prj_A12},         //MICROP_BOOT_SELECTION
    {"boot_ld"           , 0x51,  1,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_SET_BOOT_LDROM
    {"boot_ap"           , 0x52,  1,     E_WRITE,  E_MODE_BOTH, Prj_A12},         //MICROP_SET_BOOT_APROM
    {"ap_chksum"         , 0x54,  4,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_APROM_CHECKSUM
    {"software_off"      , 0x55,  1,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_SOFTWARE_OFF
    {"ind_phone_sleep"   , 0x56,  1,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_IND_PHONE_SLEEP
    {"ind_phone_resume"  , 0x57,  1,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_IND_PHONE_RESUME
    {"up_opstate"        , 0x58,  1,      E_READ, E_MODE_APROM, Prj_A12},         //MICROP_OPERATING_STATE
    {"poweron"           , 0x5C,  1,     E_WRITE, E_MODE_APROM, Prj_A12},         //MICROP_POWER_ON
    {"swgauge_reset"     , 0x5E,  2, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_MISC_CMD_WOS
    {"otg_power"         , 0x5F,  2, E_READWRITE, E_MODE_APROM, Prj_A12},         //MICROP_OTG_POWER
};
 
//const unsigned int microP_hw_ID=0x00005200;
unsigned int g_microp_ver=0;
char mcirop_ver[32]={0};//Eric +++
unsigned int g_config_id = 0;//Eric +++
uint8_t fw_data[4]={0};//Eric +++
unsigned int g_ldrom_ver=0;
unsigned char g_curr_uP_mode=0;
unsigned char g_fw_updating=0;
unsigned char is_otg_mode = false;

static struct microP_info *g_uP_info=NULL;
uint8_t *img_buf=NULL;
static u32 g_slave_addr=0;
static int g_microp_irq_gpio=0;
static int g_microp_detect_gpio=0; 
static u32 g_slave_addr_update=0x60;
static unsigned int g_fw_update_progress=0;
uint8_t g_b_isP01Connected=0;
uint8_t g_b_isP01USBConnected=0;
uint8_t g_b_isP01ACConnected=0;
uint8_t g_b_isP01BtnPressed=0;
unsigned int g_i2c_bus_suspended=0;
struct switch_dev p01_switch_dev;
struct switch_dev p01_switch_usb_cable;
struct switch_dev pfs_switch_version;//Eric +++
//struct switch_dev pad_switch_proximity;
struct switch_dev pad_err_notify;
struct mutex microp_mutex_lock;
extern void msleep(unsigned int msecs);
static uint8_t g_uPadStationUnderPoweroff=0;
struct workqueue_struct *microp_slow_job_wq = NULL;
struct workqueue_struct *microp_ins_rev_wq = NULL;
struct workqueue_struct *microp_intr_wq = NULL;
struct workqueue_struct *microp_detect_wq = NULL;
struct wake_lock interrupt_lock_t;

extern uint8_t speaker_en;
extern uint8_t recevier_en;

int pad_detect_flag = 0;
char display_ready = false;

unsigned int g_i2c_microp_busy=0;
//+++define some symbol to measure version number(Eric)
int flag_battery = 0;
int flag_ld = 0;
int flag_ap = 0;
//---

enum _microp_state{
    st_DISCONNECTED=0,
    st_CONNECTED=1,
    st_INITIALIZING=2,
    st_PRE_CONNECTED = 3,
    st_PRE_DISCONNECTED =4,
};

uint8_t g_uPadErrStatus=0;

// disable it in default
int g_prop_virtualRemoveEnabled=1;

void reportPadStationI2CFail(char *devname);
int isFirmwareUpdating(void);
int is_Mode_APROM(void){
    return (g_curr_uP_mode==E_MODE_APROM)?1:0;
}
//Eric +++ switch char to hex.
static unsigned int atoh(const unsigned char *in, unsigned int len)
{
	unsigned int sum = 0;
	unsigned int mult = 1;
	unsigned char c;

	while (len) {
		int value;

		c = in[len - 1];
		value = hex_to_bin(c);
		if (value >= 0){
			sum += mult * value;
		       mult *= 16;
                }
	    --len;
	}
	return sum;
}
//Eric ---
static void microp_reconnected(void);

// need modification
bool pad_exist(void){
    if (g_microp_detect_gpio >= 0)
        return gpio_get_value(g_microp_detect_gpio)?0:1;
    else
        return 0;
}

static int uP_i2c_read(u8 addr, int len, void *data)
{
    int i=0;
    int retries=5;
    int status=0;

    struct i2c_msg msg[] = {
        {
            .addr = g_slave_addr,
            .flags = 0,
            .len = 1,
            .buf = &addr,
        },
        {
            .addr = g_slave_addr,
            .flags = I2C_M_RD,
            .len = len,
            .buf = data,
        },
    };

    if(g_uP_info){
        do{
            mutex_lock(&g_uP_info->write_read_lock);
            status = i2c_transfer(g_uP_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
            mutex_unlock(&g_uP_info->write_read_lock);
            if ((status < 0) && (i < retries)){
                msleep(20);
                printk("%s retry %d I2C cmd:0x%2x, status=0x%x\r\n", __FUNCTION__, i, addr ,status);
                i++;
            }
        } while ((status < 0) && (i < retries));
    }

    if(status < 0){
        reportPadStationI2CFail("MicroP");
    }

    return status;
}

static int uP_i2c_write(u8 addr, int len, void *data)
{
    int i=0;
    int status=0;
    u8 buf[len + 1];
    int retries = 5;
    struct i2c_msg msg[] = {
        {
            .addr = g_slave_addr,
            .flags = 0,
            .len = len + 1,
            .buf = buf,
        },
    };

    buf[0] = addr;
    memcpy(buf + 1, data, len);
    do {
        mutex_lock(&g_uP_info->write_read_lock);
        status = i2c_transfer(g_uP_info->i2c_client->adapter, msg, ARRAY_SIZE(msg));
	mutex_unlock(&g_uP_info->write_read_lock);
        if ((status < 0) && (i < retries)){
            msleep(20);
            printk("%s retry %d, I2C cmd:0x%2x, status=0x%x\r\n", __FUNCTION__, i, addr, status);
            i++;
        }
    } while ((status < 0) && (i < retries));

    if (status < 0) {
        reportPadStationI2CFail("MicroP");
    }
    return status;
}

// Walf test +++
#define CMD_SET_ADR     0x00
#define CMD_READ_BYTE   0x81
#define CMD_READ_WORD   0x82
#define CMD_READ_BLOCK  0x80
#define CMD_WRITE_BYTE  0x01
#define CMD_WRITE_WORD  0x02
#define CMD_WRITE_BLOCK 0x03

static int io373x_i2c_set_adr(struct i2c_client *client, unsigned short reg)
{
    unsigned short data = (reg << 8) | (reg >> 8);
    return i2c_smbus_write_word_data(client, CMD_SET_ADR, data);
}

static int io373x_i2c_read_reg_1byte(struct i2c_client *client, unsigned char *buf)
{
    buf[0] = i2c_smbus_read_byte_data(client, CMD_READ_BYTE);
    return 0;
}

static int io373x_i2c_read_reg_2byte(struct i2c_client *client, unsigned short *buf)
{
    buf[0] = i2c_smbus_read_word_data(client, CMD_READ_WORD);
    return 0;
}

static int io373x_i2c_read_regs_i2c_block(struct device *dev, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
    struct i2c_client *client = to_i2c_client(dev);
    unsigned char tmp_buf[1 + 31]; // 1 CNT + 31 data bytes.
    unsigned char cmd;
    int bytes_this_read;
    int bytes_left = byte_cnt;
    int ret = 0;

    ret = io373x_i2c_set_adr(client, start_reg);

    while ((ret >= 0) && bytes_left) {
        if (bytes_left == 1) {
            ret = io373x_i2c_read_reg_1byte(client, buf);
            bytes_left = 0;
        } else if (bytes_left == 2) {
            ret = io373x_i2c_read_reg_2byte(client, (unsigned short *) buf);
            bytes_left = 0;
        } else {

            // Our device returns CNT+max_32_bytes_data, but i2c_smbus_read_i2c_block_data() returns
            // max 32 bytes with CNT in bytes[0], so there are max 31 bytes *DATA*.
            bytes_this_read = min(31, bytes_left);
            cmd = CMD_READ_BLOCK | (bytes_this_read & 0x1F); // our cmd=0x80 | CNT, where CNT=0: 32 byte DATA, CNT=3~31: 3~31 byte DATA.

            // read one more byte for our CNT.
            if (i2c_smbus_read_i2c_block_data(client, cmd, bytes_this_read + 1, tmp_buf) == bytes_this_read + 1) {
                memcpy(buf, tmp_buf + 1, bytes_this_read); // skip first byte which is CNT returned by device.
                bytes_left -= bytes_this_read;
                buf += bytes_this_read;
            } else                
                ret = -EIO;
        }
    }

    return ret;
}


static int io373x_i2c_write_regs(struct device *dev, unsigned short start_reg, unsigned char *buf, int byte_cnt)
{
    struct i2c_client *client = to_i2c_client(dev);
    int bytes_this_write;
    int bytes_left = byte_cnt;
    int ret;

    ret = io373x_i2c_set_adr(client, start_reg);
    if (ret < 0)
        return ret;

    while (bytes_left) {

        bytes_this_write = min(32, bytes_left); // we can do block write for max 32 bytes.
        bytes_this_write = min(I2C_SMBUS_BLOCK_MAX, bytes_this_write); // i2c_smbus_write_block_data() supports max this many.

        ret = i2c_smbus_write_block_data(client, CMD_WRITE_BLOCK, bytes_this_write, buf);
        if (ret < 0)
            return ret;

        bytes_left -= bytes_this_write;
        buf += bytes_this_write;
    }

    return 0;
}

int isCMDSupportedForRead(int cmd){
    int ret=1;

    if(0==(uP_CMD_Table[cmd].prj & Prj_A12))
        ret=0;
    
    if(E_WRITE==uP_CMD_Table[cmd].rw 
            || E_NOUSE==uP_CMD_Table[cmd].rw 
            || (E_MODE_BOTH!=uP_CMD_Table[cmd].mode && g_curr_uP_mode!=uP_CMD_Table[cmd].mode))
                ret=0;

    return ret;
}
int isCMDSupportedForWrite(int cmd){
    int ret=1;

    if(0==(uP_CMD_Table[cmd].prj & Prj_A12))
        ret=0;
    
    if(E_READ==uP_CMD_Table[cmd].rw 
            || E_NOUSE==uP_CMD_Table[cmd].rw 
            || (E_MODE_BOTH!=uP_CMD_Table[cmd].mode && g_curr_uP_mode!=uP_CMD_Table[cmd].mode))
                ret=0;

    return ret;
}

/*
* return read data length of the reg
*/

int uP_i2c_read_reg(int cmd, void *data){
    int status=0;

    if(cmd>=0 && cmd < ARRAY_SIZE(uP_CMD_Table)){
        if(isCMDSupportedForRead(cmd)){
            status=uP_i2c_read(uP_CMD_Table[cmd].addr, uP_CMD_Table[cmd].len, data);
        }
    }
    else
        printk("MicroP: unknown cmd\r\n");
    return status;
}

EXPORT_SYMBOL_GPL(uP_i2c_read_reg);

/*
* return read data length of the reg
*/
int uP_i2c_write_reg(int cmd, void *data){
    int status=0;
    if(cmd>=0 && cmd < ARRAY_SIZE(uP_CMD_Table)){
        if(isCMDSupportedForWrite(cmd)){ 
            status=uP_i2c_write(uP_CMD_Table[cmd].addr, uP_CMD_Table[cmd].len, data);
        }
    }
    else
        printk("MicroP: unknown cmd\r\n");
            
    return status;
}

EXPORT_SYMBOL_GPL(uP_i2c_write_reg);

/*
* return 1 if microp is connected
*/
int isMicroPConnected(void){
    int status;
    int ret=0;
    int reg_id=0;
    int op_state=0;
    int retries=40; // 1.2 seconds to try
    char tmp[20] = {0};//Eric +++
    uint32_t st_jiffies;
#ifndef ASUS_FACTORY_BUILD
    uint8_t always_ignore=0;
    uint8_t uc_val=0;
#endif
    uint8_t a68_ready=0x96;
    uint8_t poweron = 0xBB;
    uint8_t resume = 0x69;
 
    printk("%s \r\n", __FUNCTION__);

    status = uP_i2c_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode);
    printk("Current Mode=%s\r\n",(g_curr_uP_mode==0)?"APROM":"LDROM");

    if(status > 0 && is_Mode_APROM()){

        uP_i2c_read_reg(MICROP_BATTERY_CONFIG,&fw_data);
        printk("Reg:0x3a 0~4 byte ==0x%02x%02x%02x%02x\n", fw_data[0], fw_data[1], fw_data[2],fw_data[3]);
        status=uP_i2c_read_reg(MICROP_OEM_FW_VERSION,&mcirop_ver);
        memcpy(tmp,mcirop_ver+10,4);
        printk("mcirop_ver=%32s,status = %d,ap_version =%s\r\n",mcirop_ver,status,tmp);
        reg_id = atoh(tmp,4);
        printk("MicroP found! fw ver=0x%x, %d\r\n",reg_id, status);
        g_microp_ver=reg_id;
        uP_i2c_read_reg(MICROP_LDROM_ID_CODE,&g_ldrom_ver);
        printk("LDROM ver=0x%x\r\n",g_ldrom_ver);

        st_jiffies=jiffies;
        uP_i2c_write_reg(MICROP_IND_PHONE_RESUME, &resume);
        uP_i2c_write_reg(MICROP_POWER_ON,&poweron);
        status = uP_i2c_read_reg(MICROP_OPERATING_STATE,&op_state);
        while (st_MICROP_Active!=op_state && status > 0 && retries>0) {
            // try to wake up EC by i2c cmd
            if (st_MICROP_Off == op_state)
                uP_i2c_write_reg(MICROP_POWER_ON,&poweron);

            msleep(30);
            printk("<try: %d, op_state=%d>\n", 40-retries, op_state);
            status = uP_i2c_read_reg(MICROP_OPERATING_STATE,&op_state);
            retries--;
        }

        printk("==> takes %lu jiffies~~\r\n", jiffies - st_jiffies);
        if(retries == 0 || status <= 0)
            printk("microp state failed!!\r\n");
        else {
            printk("state=> Active\r\n");
#ifndef ASUS_FACTORY_BUILD
            status=uP_i2c_read_reg(MICROP_ALWAYS_IGNORE_PHONE_READY,&always_ignore);
            if(status && 0x96==always_ignore){
                always_ignore=0;
                printk("%s: clear \"MICROP_ALWAYS_IGNORE_A68READY\" flag \r\n", __FUNCTION__);
                uP_i2c_write_reg(MICROP_ALWAYS_IGNORE_PHONE_READY,&always_ignore);
            }
            status=uP_i2c_read_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY,&uc_val);
            if(status && 0xcc==uc_val){
                uc_val=0;
                printk("%s: clear \"MICROP_DISABLE_CHARGING_FOR_FACTORY\" flag \r\n", __FUNCTION__);
                uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY,&uc_val);
            }
#endif
            printk("MicroP found! write MICROP_IND_A68_READY\r\n");
            status=uP_i2c_write_reg(MICROP_IND_PHONE_READY,&a68_ready);
            if(status > 0)
                ret=1;
        }
    } else if (status > 0) { //LDROM
        printk("LDROM,force upgrade firmware\n");
        micropSendNotify(P01_APROM_CRASH);
        g_ldrom_ver = 0;
        ret = 1;
    } else{
        printk("%s : not connected\r\n", __FUNCTION__);
    }
    return ret;
}

/*
*
*       for debug
*
*
*/
#ifdef CONFIG_DEBUG_FS
#include <linux/debugfs.h>
#include <linux/seq_file.h>

static int dbg_microp_regs_dump(struct seq_file *s, void *unused)
{
    int i=0,j=0;
    struct i2c_microp_command cmd;
    uint64_t val;
    //int16_t val2;
    char sz_PadIsn[32]={0};
    char sz_oemVerStr[24]={0};
    unsigned char proxm_kdata[22]={0};
    seq_printf(s, "\nMicroP CMD       ADDR        VAL       MODE\r\n");
    seq_printf(s, "============================== %d,\r\n",ARRAY_SIZE(uP_CMD_Table));
    for(i=0;i<ARRAY_SIZE(uP_CMD_Table);i++){
        cmd=uP_CMD_Table[i];
        val=0;
        if(MICROP_ISN==i || MICROP_OEM_SSN==i){
            uP_i2c_read_reg(i,sz_PadIsn);
            seq_printf(s, "%32s\t%4x\t%32s(ASCII)\r\n",cmd.name, cmd.addr, sz_PadIsn);
        }
        else if(MICROP_OEM_FW_VERSION==i){
            uP_i2c_read_reg(i,sz_oemVerStr);
            seq_printf(s, "%32s\t%4x\t%32s(ASCII)\r\n",cmd.name, cmd.addr, sz_oemVerStr);
        }
        else if(MICROP_PROXIM_KDATA==i){
            uP_i2c_read_reg(i,proxm_kdata);
            seq_printf(s, "%32s\t%4x\t",cmd.name, cmd.addr);
            for (j=0;j<PROXIMITY_KDATA_SIZE*2;j++) {
                seq_printf(s,"%2x",proxm_kdata[j]);
            }
            seq_printf(s,"\r\n");
        }
        else{
            uP_i2c_read_reg(i,&val);
            seq_printf(s, "%32s\t%4x\t%32llx(h)\r\n",cmd.name, cmd.addr, val);
        }
    }
    return 0;
}

static int dbg_microp_open(struct inode *inode, struct file *file)
{
	return single_open(file, dbg_microp_regs_dump, &inode->i_private);
}
static const struct file_operations debug_fops = {
	.open		= dbg_microp_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};
#endif


static int updateMicroPFirmware_ENE(unsigned long arg);
/************************************
*
* macro definition
*
*************************************/
// check reg_intr_sta val

// Pad-Related
#define IsPadACUSBInOut()          (((reg_intr_sta>>INTR_STA_AC_USB_IN_OUT)  & 0x1)?1:0 )
#define IsPadVolDnEvt()            (((reg_intr_sta>>INTR_STA_VOL_DOWN)       & 0x1)?1:0 )
#define IsPadVolUpEvt()            (((reg_intr_sta>>INTR_STA_VOL_UP)         & 0x1)?1:0 )
#define IsPadPwrBtnEvt()           (((reg_intr_sta>>INTR_STA_PWR_BTN)        & 0x1)?1:0 )
#define IsPadHavingBat5Percent()   (((reg_intr_sta>>INTR_STA_IND_GAUGE_5P)   & 0x1)?1:0 )
#define IsPadRevSleepReminder()    (((reg_intr_sta>>INTR_STA_SLEEP_REMINDER) & 0x1)?1:0 )
#define IsPadProxmEvt()            (((reg_intr_sta>>INTR_STA_PROXM_INT)      & 0x1)?1:0 )
#define IsPadBatStatChange()       (((reg_intr_sta>>INTR_STA_BAT_STAT_CHANGE)& 0x1)?1:0 )
#define IsPadPwrBtnPressEvt()      (((reg_intr_sta>>INTR_STA_PWR_PRESS)      & 0x1)?1:0 )
#define IsPadPwrBtnReleaseEvt()    (((reg_intr_sta>>INTR_STA_PWR_RELEASE)    & 0x1)?1:0 )
#define IsPadPowerOnEvt()          (((reg_intr_sta>>INTR_STA_POWER_ON)       & 0x1)?1:0 )
#define IsPadLowLowBatt()          (((reg_intr_sta>>INTR_STA_LOWLOW_BAT)     & 0x1)?1:0 )
#define IsPadNoIntrEvt()           ((reg_intr_sta==0)?1:0)

/*
* check reg_input val
*/
// Pad
#define IsPadVolUpPressed()        (((reg_input>>IN_VOL_UP_R)  & 0x1)?0:1 )
#define IsPadVolDnPressed()        (((reg_input>>IN_VOL_DOWN_R) & 0x1)?0:1 )
#define IsPadPwrBtnPressed()       (((reg_input>>IN_PWR_BTN_R)  & 0x1)?0:1 )
int isAlwaysPowerOnMicroP(void){    

       int status;
       unsigned usb_detect=0;
       status=uP_i2c_read_reg(MICROP_USB_DET,&usb_detect);
       if(status){
            printk("%s usb type=%d\r\n", __FUNCTION__, usb_detect);
            if(usb_detect==P01_CABLE_CHARGER)
                return 1;
       }
       return 0;

}

static void microp_reconnected(void)
{
    printk("MicroP: Trigger Driver Re-Connecting \r\n");
    if(g_uP_info){
        printk("%s,  g_b_isP01Connected=%d\r\n",__FUNCTION__,  g_b_isP01Connected);
        if(st_CONNECTED!=g_b_isP01Connected){
            printk("%s: reinit P01\r\n",__FUNCTION__);
            g_b_isP01Connected=st_INITIALIZING; // 2: pad inserted but not intialized ready
            switch_set_state(&p01_switch_dev,st_INITIALIZING);
            switch_set_state(&pfs_switch_version,st_INITIALIZING);
            //schedule_delayed_work(&g_uP_info->initP01,0);
            queue_delayed_work(microp_ins_rev_wq, &g_uP_info->initP01, 0);
        }
        else{
            printk("MicroP: already connected skip\r\n");
        }
    }
    else{
        printk("MicroP: not yet init \r\n");
    }
}

void notify_microp_insert(void)
{
    is_first_bootup = 0;
    printk("MicroP: Pad -> INSERT \r\n");
    if(g_uP_info){
        printk("%s,  g_b_isP01Connected=%d\r\n",__FUNCTION__,  g_b_isP01Connected);
        if(st_CONNECTED!=g_b_isP01Connected){
            printk("MicroP is initializing...\r\n");
            g_b_isP01Connected=st_INITIALIZING; // 2: pad inserted but not intialized ready
            switch_set_state(&p01_switch_dev,st_INITIALIZING);
            switch_set_state(&pfs_switch_version,st_INITIALIZING);
            //schedule_delayed_work(&g_uP_info->initP01,0);
            queue_delayed_work(microp_ins_rev_wq, &g_uP_info->initP01, msecs_to_jiffies(200));
        }
        else{
            printk("MicroP: already connected skip\r\n");
        }
    }
    else{
        printk("MicroP: not yet init \r\n");
    }
}

void notify_microp_remove(int virtual)
{
    if(virtual){
        printk("MicroP: Pad -> VIRTUAL REMOVE \r\n");
    }
    else{
        printk("MicroP: Pad -> REMOVE \r\n");
    }

#ifdef CONFIG_FASTBOOT
    if(is_fastboot_enable()){
        ready_to_wake_up_and_send_power_key_press_event_in_fastboot();
    }
#endif //#ifdef CONFIG_FASTBOOT

    if(g_uP_info){
        printk("%s,  g_b_isP01Connected=%d\r\n",__FUNCTION__,  g_b_isP01Connected);
        if(st_DISCONNECTED!=g_b_isP01Connected){
            g_b_isP01Connected=st_PRE_DISCONNECTED;
            switch_set_state(&p01_switch_dev,st_DISCONNECTED);
            switch_set_state(&pfs_switch_version,st_DISCONNECTED);
            //switch antenna back to phone by RF req. 2012/07/30
            set_antenna_to_phone();

            queue_delayed_work(microp_ins_rev_wq, &g_uP_info->deinitPad, msecs_to_jiffies(200));

            if(virtual){
                switch_set_state(&pad_err_notify, g_uPadErrStatus);
            }
        }
        else
            printk("MicroP: P01 already removed, skip \r\n");
    }
    else{
        printk("MicroP: not yet init \r\n");
    }
}

void TriggerPadStationPowerOff(void){
    if(g_uP_info && microp_slow_job_wq){
        if(!g_uPadStationUnderPoweroff){
            printk("power off P02 : 6 secs later");
            queue_delayed_work(microp_slow_job_wq, &g_uP_info->poweroffP01,0);
        }else{
            printk("power off work is onGoing now\r\n");
        }
    }
    else{
        printk("MicroP: not yet init \r\n");
    }
}

void reportPadStationI2CFail(char *devname){
    printk("%s: ++\r\n", __FUNCTION__);
    if(g_prop_virtualRemoveEnabled){
        if(g_i2c_bus_suspended)
            printk("%s: Bus Suspended: Skip\r\n", __FUNCTION__);
        else if( is_Mode_APROM() && ( st_CONNECTED==g_b_isP01Connected ||
                 (st_PRE_CONNECTED==g_b_isP01Connected))){
            if(pad_exist()){
                printk("%s: %s Triggerd Virtual Remove\r\n", __FUNCTION__, devname);
                g_uPadErrStatus=3; //i2c error
                AX_MicroP_set_VBusPower(0);
                notify_microp_remove(1);
            }
            else{
                printk("%s: Pad Not Exist. %s Triggerd Normal Remove\r\n", __FUNCTION__, devname);
                notify_microp_remove(0);
            }
        }
    }
    else{
        printk("%s: g_prop_virtualRemoveEnabled=%d\r\n", __FUNCTION__, g_prop_virtualRemoveEnabled);
    }
    printk("%s: --\r\n", __FUNCTION__);
}


EXPORT_SYMBOL_GPL(reportPadStationI2CFail);

static void vibrator_pad(void)
{
    int ret;
    /*set CLKDIV register*/
    ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x01);
    ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x4E);

    /*enable vibrator*/
    ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, 0X63);

    msleep(500);

    /*disable vibrator*/
    ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, 0X00);
}

static void initP01(struct work_struct *work){
    int reg_intr_sta=0;
    int reg_input=0;
    uint8_t i2cdata[32]={0};
    uint8_t usb_det=0;
    uint8_t usb_type=0;
    uint8_t clear_power_reason = MICROP_CLEAR_POWER_REASON;

    printk(" %s +\r\n",__FUNCTION__);

    g_firmwareSize = 28*1024;

    if(isMicroPConnected()==1){
        if(st_CONNECTED!=g_b_isP01Connected){
            g_b_isP01Connected=st_CONNECTED;
            // default: not enable  HP Hookkey Intr bit 12
#ifdef ASUS_FACTORY_BUILD
            i2cdata[0]=0xff;
            i2cdata[1]=0xff;
            i2cdata[2]=0xff;
#else
            i2cdata[0]=0xff;
            i2cdata[1]=0xff;
            i2cdata[2]=0xff;
#endif
            uP_i2c_write_reg(MICROP_INTR_EN,i2cdata);
            uP_i2c_write_reg(MICROP_MISC_CMD_WOS,&clear_power_reason);
            set_cpufreq_boost(2);
            micropSendNotify(P01_ADD);

            vibrator_pad();

            switch_set_state(&p01_switch_dev,st_CONNECTED);
            switch_set_state(&pfs_switch_version,st_CONNECTED);
            g_uPadErrStatus=0;      // reset error status to 0
            switch_set_state(&pad_err_notify, 0);

            set_antenna_to_pad_main();
        }
        else{
            printk("MicroP: P01 already added, skip \r\n");
        }
    }
    else{
        printk("uP_Init: failed to access microp\r\n");
        g_b_isP01Connected=st_DISCONNECTED;
        switch_set_state(&p01_switch_dev,st_DISCONNECTED);
        switch_set_state(&pfs_switch_version,st_DISCONNECTED);
    }

    if(st_CONNECTED==g_b_isP01Connected){
        uP_i2c_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
        is_microp_init = 1;
        uP_i2c_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
        uP_i2c_read_reg(MICROP_USB_DET, &usb_det);
        uP_i2c_read_reg(MICROP_USB_TYPE,&usb_type);
        printk("skip first intr status = %x, input = %x, usb_det=%x,usb_type=%x\r\nCheck Devices Status on Pad\r\n",reg_intr_sta,reg_input, usb_det,usb_type);
        if(usb_det==P01_CABLE_CHARGER)
        {
            printk(KERN_INFO "uP_Init: AC in !!\n");
            micropSendNotify(P01_AC_IN);
            g_b_isP01ACConnected = 1;
        }
        else if(usb_det==P01_CABLE_USB)
        {
            printk(KERN_INFO "uP_Init: USB in !!\n");
            micropSendNotify(P01_USB_IN);
            g_b_isP01USBConnected = 1;
        }
        else if(usb_det==P01_CABLE_OTG){
            printk(KERN_INFO "uP_Init: OTG in !!\n");
            is_otg_mode = true;
            AX_MicroP_setOTGPower(1);
            micropSendNotify(PAD_USB_OTG_ENABLE);
        } else if (usb_det==P01_CABLE_NO){
            micropSendNotify(P01_AC_USB_OUT);
        }
        if (IsPadProxmEvt()){
            printk("uP_Init:Proximity interrupt\n");
            micropSendNotify(P01_PROXM_SENSOR);
        }

#if 0
		//work around, dont charge padfone in default
		printk(KERN_INFO "uP_Init: WorkAround Set Vbus=0 !not to charge PadFone !!\n");
		AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, 0);
#endif
    }
    printk("%s -\r\n",__FUNCTION__);
}

static void deinitPad(struct work_struct *work){
    //int ret = 0;
    printk("%s +\r\n",__FUNCTION__);
    printk("MicroP: cancel work...\r\n");

//    cancel_delayed_work_sync(&g_uP_info->initP01);
    printk("MicroP: Done\r\n");
    // cancel all works of init wq
    set_cpufreq_boost(2);
    micropSendNotify(P01_REMOVE);
    if ( g_b_isP01USBConnected ==1 || g_b_isP01ACConnected ==1) {
        micropSendNotify(P01_AC_USB_OUT);
        g_b_isP01USBConnected == 0;
        g_b_isP01ACConnected == 0;
    }
    else if (is_otg_mode == true) {
        is_otg_mode = false;
        micropSendNotify(PAD_USB_OTG_DISABLE);
    }

    if (g_b_isP01BtnPressed) {
        if (g_b_isP01BtnPressed == PWR_BTN_PRESS)
            micropSendNotify(P01_PWR_KEY_RELEASED);
        if (g_b_isP01BtnPressed == VOL_UP_PRESS)
            micropSendNotify(P01_VOLUP_KEY_RELEASED);
        if (g_b_isP01BtnPressed == VOL_DOWN_PRESS)
            micropSendNotify(P01_VOLDN_KEY_RELEASED);
        g_b_isP01BtnPressed = 0;
    }
    g_b_isP01Connected=st_DISCONNECTED;

    printk("%s -\r\n",__FUNCTION__);
}

static void microp_poweroff(struct work_struct *work){
    unsigned short off=0xAA;
    uint32_t bit_vbus=OUT_uP_VBUS_EN;

    g_uPadStationUnderPoweroff=1;
    printk("[%s] ++\r\n", __FUNCTION__);
    // turn off vbus first
    if(!isAlwaysPowerOnMicroP()) //if no charger, turn off vbus
        uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR,  &bit_vbus);

    uP_i2c_write_reg(MICROP_SOFTWARE_OFF,  &off);
//        msleep(5500); //wait P01 truly power off its power. microp fw need 5 seconds to power off
    micropSendNotify(P01_BATTERY_POWER_BAD);
    g_uPadStationUnderPoweroff=0;
    printk("[%s] REMOVE PAD DUE TO BATT CAPACITY IS LOW!!\r\n", __FUNCTION__);
    g_uPadErrStatus=4; // power bad
    notify_microp_remove(1);
    printk("[%s] --\r\n", __FUNCTION__);
}

static void microP_work(struct work_struct *work)
{
    int reg_intr_sta=0;
    int reg_input=0;
    uint8_t reg_powerOnReason=0;
    uint8_t usb_det=0;
    uint8_t usb_type=0;
    uint8_t a68_ready=0x96;
    uint8_t clear_power_reason = MICROP_CLEAR_POWER_REASON;
    //printk("Walf MicroP work\n");
    //printk("MicroP wq <%d,%d>+++\r\n", g_b_isP01Connected, gpio_get_value(g_uP_info->pdata->intr_gpio));
	
    if(is_first_bootup == 1){
        //printk("is_first_bootup = 1, don't handle interrupt from microp\n");
        uP_i2c_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
        uP_i2c_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
        return;
    }

    if (is_microp_init == 0) {
        printk("Microp:not initial,ignore interrupt\n");
        return;
    }

    if(st_CONNECTED==g_b_isP01Connected){
	while(0==gpio_get_value(g_microp_irq_gpio)){
            if(g_i2c_bus_suspended){
                printk("MicroP Intr: i2c not ready. pospone intr handling for 30ms\r\n");
                msleep(30);
                queue_delayed_work(microp_intr_wq, &g_uP_info->work,0);
                return;
            }
            if( is_Mode_APROM()){
//                uP_i2c_write_reg(MICROP_IND_PHONE_READY,&a68_ready);
                uP_i2c_read_reg(MICROP_INTR_STATUS,&reg_intr_sta);
//                uP_i2c_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
//                printk("MicroP intr: status=%8x, input=%8x\r\n",reg_intr_sta,reg_input);

                if(IsPadNoIntrEvt()){
                    pr_debug("MicroP No Event !! wait 200ms.\r\n");
                    msleep(0); //wait gpio 107 from low -> high again
                }
                else{
                    uP_i2c_read_reg(MICROP_GPIO_INPUT_LEVEL,&reg_input);
                    if(IsPadRevSleepReminder()){
                        printk("Rev MicroP Sleep Reminder!!\r\n");
                    }
                    if (IsPadPowerOnEvt()){
                        printk("Microp PowerOn event\n");
                        uint8_t a68_ready=0x96;
                        int rc;
                        rc = uP_i2c_write_reg(MICROP_IND_PHONE_READY,&a68_ready);
                        if( rc <= 0){
                             printk("%s: set microp to ready failed\n", __func__);
                             return -1;
                        }
                        if (g_b_isP01BtnPressed & PWR_BTN_PRESS) {
                            micropSendNotify(P01_PWR_KEY_RELEASED);
                            g_b_isP01BtnPressed &= ~PWR_BTN_PRESS;
                        }
                        if (speaker_en)
                            AX_MicroP_setSPK_EN(speaker_en);
                        if (recevier_en)
                            AX_MicroP_setRCV_EN(recevier_en);
                        micropSendNotify(P01_ADD);
                    }
                    if (IsPadLowLowBatt()){
                        printk("Microp reach low low battery,force remove\n");
                        notify_microp_remove(0);
                    }
                    if(IsPadHavingBat5Percent()){
                        printk("MicroP HavingBat Reach 5P!!\r\n");
                        micropSendNotify(P01_LOW_BATTERY);
                    }
                    if(IsPadACUSBInOut()){
                        uP_i2c_read_reg(MICROP_USB_DET, &usb_det);
                        if(usb_det==P01_CABLE_CHARGER){
                            uP_i2c_read_reg(MICROP_USB_TYPE,&usb_type);
                            printk("MicroP Pad AC In!!\r\n");
                            printk("usb type is 0x%x\n",usb_type);
                            micropSendNotify(P01_AC_IN);
                            g_b_isP01ACConnected = 1;
                        }
                        else if(usb_det==P01_CABLE_USB){
                            uP_i2c_read_reg(MICROP_USB_TYPE,&usb_type);
                            printk("MicroP Pad USB In!!\r\n");
                            printk("usb type is 0x%x\n",usb_type);
                            micropSendNotify(P01_USB_IN);
                            g_b_isP01USBConnected=1;
                        }
                        else if(usb_det==P01_CABLE_OTG){
                            printk("MicroP Pad OTG In!!\r\n");
                            is_otg_mode = true;
                            AX_MicroP_setOTGPower(1);
                            micropSendNotify(PAD_USB_OTG_ENABLE);
                        }
                        else if(usb_det==P01_CABLE_NO){
                            printk("MicroP Pad AC_USB_OUT!!\r\n");
                            if (is_otg_mode == true) {
                                AX_MicroP_setOTGPower(0);
                                is_otg_mode = false;
                                micropSendNotify(PAD_USB_OTG_DISABLE);
                            } else {
                                micropSendNotify(P01_AC_USB_OUT);
                            }
                            g_b_isP01ACConnected=0;
                            g_b_isP01USBConnected=0;
                        }
			switch_set_state(&p01_switch_usb_cable,g_b_isP01USBConnected);
                    }

                    if(IsPadVolDnEvt()){
                        printk("MicroP VolDnKey !!\r\n");
                        if(IsPadVolDnPressed()){
                            micropSendNotify(P01_VOLDN_KEY_PRESSED);
                            g_b_isP01BtnPressed |= VOL_DOWN_PRESS;
                        }
                        else {
                            micropSendNotify(P01_VOLDN_KEY_RELEASED);
                            g_b_isP01BtnPressed &= ~VOL_DOWN_PRESS;
                        }
                    }

                    if(IsPadVolUpEvt()){
                        printk("MicroP VolUpKey !!\r\n");
                        if(IsPadVolUpPressed()){
                            micropSendNotify(P01_VOLUP_KEY_PRESSED);
                            g_b_isP01BtnPressed |= VOL_UP_PRESS;
                        }
                        else {
                            micropSendNotify(P01_VOLUP_KEY_RELEASED);
                            g_b_isP01BtnPressed &= ~VOL_UP_PRESS;
                        }
                    }

                    if(IsPadPwrBtnEvt()){
                        printk("MicroP PwrKey !!\r\n");
                        if (IsPadPwrBtnPressEvt() || IsPadPwrBtnReleaseEvt()){
                            if (IsPadPwrBtnPressEvt()) {
                                micropSendNotify(P01_PWR_KEY_PRESSED);
                                g_b_isP01BtnPressed |= PWR_BTN_PRESS;
                            }
                            if (IsPadPwrBtnReleaseEvt()) {
                                micropSendNotify(P01_PWR_KEY_RELEASED);
                                g_b_isP01BtnPressed &= ~PWR_BTN_PRESS;
                            }
                        } else {
                            if(IsPadPwrBtnPressed()) {
                                micropSendNotify(P01_PWR_KEY_PRESSED);
                                g_b_isP01BtnPressed |= PWR_BTN_PRESS;
                            }
                            else {
                                micropSendNotify(P01_PWR_KEY_RELEASED);
                                g_b_isP01BtnPressed &= ~PWR_BTN_PRESS;
                            }
                        }
                    }

                    if (IsPadProxmEvt()){
                        printk("Microp proximity interrupt\n");
                        micropSendNotify(P01_PROXM_SENSOR);
                    }
                    if (IsPadBatStatChange()) {
                        printk("MicroP battery status change\n");
                        micropSendNotify(P01_BATTERY_STAT_CHANGE);
                    }
                }
            }
            else{
                printk("LDROM mode skip Evt, g_curr_uP_mode=%d\r\n", g_curr_uP_mode);
                break;
            }
        }
    }
    else if(st_DISCONNECTED==g_b_isP01Connected){
        msleep(400);
        uP_i2c_read_reg(MICROP_POWER_ON_REASON, &reg_powerOnReason);
        if(reg_powerOnReason > 0){
            uP_i2c_write_reg(MICROP_MISC_CMD_WOS,&clear_power_reason);
            printk("MicroP reconnected, PowerOn reason=%d\r\n", reg_powerOnReason);
            microp_reconnected();
        }
    }

    //pr_debug("MicroP wq <%d, %d>---\r\n", g_b_isP01Connected,gpio_get_value(g_uP_info->pdata->intr_gpio));
    pr_debug("MicroP wq <%d, %d>---\r\n", g_b_isP01Connected,gpio_get_value(g_microp_irq_gpio));
}

static void microp_paddetect(struct work_struct *work)
{
    int pad_detect_gpio = 0;
    printk("MicroP: microp_paddetect do next 2\n");

    msleep(40);
    cancel_delayed_work_sync(&g_uP_info->initP01);
    cancel_delayed_work_sync(&g_uP_info->deinitPad);
    pad_detect_gpio = gpio_get_value(g_microp_detect_gpio);
    printk("MicroP: microp_paddetect GPIO=%d\n", pad_detect_gpio);

    if(pad_detect_gpio == 0)
    {
        notify_microp_insert();
    }
    else
    {
        is_microp_init = 0;
        notify_microp_remove(0);
    }

    wake_lock_timeout(&interrupt_lock_t, 3*HZ);
    pad_detect_flag = 0;
}

int checkPadExist(int trigger_power_on) {
    if (!trigger_power_on)
        display_ready = true;
    if(g_uP_info){
       int paddetect_gpio_value = gpio_get_value(g_microp_detect_gpio);
       g_uP_info->paddetect_gpio_value = paddetect_gpio_value;
       pr_info("%s: microp_paddetect GPIO=%d\n", __func__ ,g_uP_info->paddetect_gpio_value);

       if(paddetect_gpio_value == 0){
           if(!trigger_power_on) {
               queue_delayed_work(microp_detect_wq, &g_uP_info->paddetect, msecs_to_jiffies(0)); //display is ready, do paddetect
           }
           else {  //display is not ready, only power on microp
               uint8_t a68_ready=0x96;
               uint16_t  value=0x0069;
               int rc;
               rc = uP_i2c_write_reg(MICROP_IND_PHONE_RESUME, &value);
               if( rc <= 0){
                   printk("%s: power on microp failed\n", __func__);
                   return -1;
               }
               rc = uP_i2c_write_reg(MICROP_IND_PHONE_READY,&a68_ready);
               if( rc <= 0){
                   printk("%s: set microp to ready failed\n", __func__);
                   return -1;
               }

               g_b_isP01Connected=st_PRE_CONNECTED;
               switch_set_state(&p01_switch_dev,st_PRE_CONNECTED);
               switch_set_state(&pfs_switch_version,st_PRE_CONNECTED);
           }
       } else {
           g_b_isP01Connected=st_DISCONNECTED;
       }
   }else{
       printk("%s: microp is not inited\n", __func__);
   }
   return 0;
}
EXPORT_SYMBOL(checkPadExist);

void microp_recheck_interrupt()
{
    printk("enter %s\n",__FUNCTION__);
    queue_delayed_work(microp_intr_wq, &g_uP_info->work, 0);
}

static irqreturn_t microP_irq_handler(int irq, void *dev_id)
{
    struct microP_info *info =
        (struct microP_info *)dev_id;
    queue_delayed_work(microp_intr_wq, &info->work, 0);
        
    return IRQ_HANDLED;
}

static irqreturn_t microP_detect_handler(int irq, void *dev_id)
{
    struct microP_info *info =
        (struct microP_info *)dev_id;

    printk("enter %s\n",__FUNCTION__);
    if(display_ready == false)
    {
        printk("display not ready,wait for display ready\n");
        return IRQ_HANDLED;
    }

    queue_delayed_work(microp_detect_wq, &info->paddetect, msecs_to_jiffies(100));
        
    return IRQ_HANDLED;
}

int getCmdID(int addr){
    int i=0;
    for(i=0;i < ARRAY_SIZE(uP_CMD_Table);i++){
        if(addr==uP_CMD_Table[i].addr)
            return i;
    }
    return -1;
}

int isFirmwareUpdating(void){        
    return (!is_Mode_APROM() || g_fw_updating);
}

/*
* file operation implementation
*/
static int updateMicroPFirmware_ENE(unsigned long arg){
	int8_t i2cdata[32]={0};
	int ret=0;
	int i=0;
	uint8_t j = 0x00;
	int flag = 0;
	int img_offset = 0;
	int upfw_retry = 0;
	uint8_t a68_ready=0x96;
	uint8_t always_ignore=0;
	uint8_t second_ignore = 0;
        uint8_t clear_power_reason = MICROP_CLEAR_POWER_REASON;
	//+++ Eric
	if(strcmp(fw_info.cmd_style,"-u") == 0){
		if((fw_info.battery_id > 0) && (fw_data[2] < fw_info.config_id)){
			flag_battery = 1;
			//printk("Eric flag_battery \r\n");
		}
		if(g_microp_ver < fw_info.ap_version){
			flag_ap = 1;
			//printk("Eric flag_ap \r\n");
		}
		if(g_ldrom_ver < fw_info.ld_version){
#ifdef ASUS_USER_BUILD
                    flag_ld = 0;
#else
                    printk("g_ldrom_ver:0x%x,fw_info.ld_version:0x%x\n",g_ldrom_ver,fw_info.ld_version);
                    flag_ld = 1;
#endif
		}
	}
	else if(strcmp(fw_info.cmd_style,"-r") == 0){
		flag_ap = 1;
		flag_ld = 1;
		flag_battery = 0;
		if(fw_info.battery_id > 0)
		{
			flag_battery = 1;
		}

	}
	//+++ Eric

	//set different fw size
	if((flag_ld && flag_ap && !flag_battery)||(flag_ld && !flag_ap && !flag_battery)){
		//g_firmwareSize = 28 * 1024;
		printk("Eric 28k \r\n");
	}
	else if(flag_ap && !flag_ld && !flag_battery){
		//g_firmwareSize = 24 * 1024;
		printk("Eric 24k \r\n");
	}
	else if(flag_battery && !flag_ap && !flag_ld){
		//g_firmwareSize = 1 * 1024;
		printk("Eric 1k \r\n");
	}
	else if(flag_ap && flag_battery && !flag_ld ){
		//g_firmwareSize = 25 * 1024;
		printk("Eric 25k \r\n");
	}
	else{
		//g_firmwareSize = 29 * 1024;
		printk("Eric 29k \r\n");
	}
	//---
	//---
	g_uP_info->i2c_client->addr = g_slave_addr_update;
	printk("\nMicriP updateMicroPFirmware: Set addr=%d\n", g_uP_info->i2c_client->addr);
	g_firmwareSize = 29 * 1024;
        if(img_buf==NULL)
			img_buf = kzalloc(sizeof(uint8_t)*g_firmwareSize, GFP_KERNEL);
        
        if(!img_buf){
                pr_err("%s: Unable to allocate memory!\n",__FUNCTION__);
                return -ENOMEM;
        }
        
       memset(img_buf,0,sizeof(uint8_t)*g_firmwareSize);
		//printk("Walf updateMicroPFirmwareTest arg=%s\n", (const char __user *)arg);
       /* first element of arg must be u32 with id of module to talk to */
        if(copy_from_user(img_buf, (const void __user *)arg, sizeof(uint8_t)*g_firmwareSize)) {
                pr_err("%s: Failed to copy arg from user", __FUNCTION__);
        }
		
		printk("MicroP disable irq first\r\n");
        disable_irq(gpio_to_irq(g_microp_irq_gpio));

        /*if(uP_i2c_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode))
            printk("Microp %s\r\n",g_curr_uP_mode?"@LDROM":"@APROM");


    	i2cdata[0] = 0x5A;
    	uP_i2c_write_reg(MICROP_SET_BOOT_LDROM, i2cdata);

       if(uP_i2c_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode)){
            printk("Boot selection => %s\r\n",g_curr_uP_mode?"@LDROM":"@APROM");
            if(g_curr_uP_mode!=1){
                    printk("Cannot Change To LDROM: Fatal Error\r\n");
                    g_fw_update_progress=0;
                    return -EAGAIN;
            }
            //uP_i2c_read_reg(MICROP_FW_VER,&g_microp_ver);
            //printk("MicroP: enter into LDROM ver=0x%x\r\n", g_microp_ver);

        }*/
begin_upfw:
        g_fw_updating = 1;
	always_ignore=0x97;
	printk("%s: set \"MICROP_ALWAYS_IGNORE_A68READY\" flag \r\n", __FUNCTION__);
	uP_i2c_write_reg(MICROP_ALWAYS_IGNORE_PHONE_READY,&always_ignore);

        uP_i2c_read_reg(MICROP_ALWAYS_IGNORE_PHONE_READY,&always_ignore);
        printk("after set MICROP_ALWAYS_IGNORE_PHONE_READY,result is:0x%x\n",always_ignore);

	g_fw_update_progress=0;
	img_offset = 0;
	// stop watchdog
	i2cdata[0]=0x01;
	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0xFE80, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 1\n");
	}
	
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=
    // Set Update Flag
    printk("\nMicroP updateMicroPFirmwareTest: Set Update Flag\n");
	i2cdata[0]=0x01;
	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0xF012, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 1\n");
	}
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=
	// Switch 8051 Fetch Code
	printk("MicroP updateMicroPFirmwareTest: Switch 8051 Fetch Code\n");
	//1. Set E51_RST to reset 8051
	i2cdata[0]=0x01;
	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0xF010, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 2\n");
	}
	
	// 2. Wait XBIEFCFG[7]=1 for XBI IDLE
	i2cdata[0]=0x00;
	i = 0;
	while(i<100)
	{
		ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xFEA0, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest 0xF012 fail 3\n");
		}
		if((i2cdata[0] & 0x80) == 0x80)
		{
			printk("MicroP updateMicroPFirmwareTest 0xFEA0=0x%x\r\n",i2cdata[0]);     
			flag = 1;
			break;
		}
		printk("MicroP updateMicroPFirmwareTest not read 0xFEA0\n");
		msleep(50);
		i++;
	}
	if(flag == 0) 
	{
		//if(img_buf) kfree(img_buf);
		//img_buf=NULL;
		//return 0;
		upfw_retry++;
		printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
		if(upfw_retry < 10)
		{
			goto begin_upfw;
		}
		else
		{
			goto end_upfw;
		}
	}
	flag = 0;
	
	// 3. Set CODE_SEL[0]=0 for XBI IDLE
	i2cdata[0]=0x00;
	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 2\n");
	}
	
	// 4. Set E51_RST = 0 to re-start 8051
	i2cdata[0]=0x00;
	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0xF010, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 2\n");
	}
	
	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++=
	// If ownership flag == ROM
	printk("MicroP updateMicroPFirmwareTest: If ownership flag == ROM\n");
	i2cdata[0]=0x00;
	i = 0;
	while(i<100)
	{
		ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF012, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest 0xF012 fail 3\n");
		}
		if((i2cdata[0] & 0x02) == 0x02)
		{
			printk("MicroP updateMicroPFirmwareTest 0xF012=0x%x\r\n",i2cdata[0]);     
			flag = 1;
			break;
		}
		printk("MicroP updateMicroPFirmwareTest not read 0xF012\n");
		msleep(50);
		i++;
	}
	
	if(flag == 0) 
	{
		//if(img_buf) kfree(img_buf);
		//img_buf=NULL;
		//return 0;
		upfw_retry++;
		printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
		if(upfw_retry < 10)
		{
			goto begin_upfw;
		}
		else
		{
			goto end_upfw;
		}
	}
	flag = 0;
	
	if(flag_ld == 1){//Eric +++	
		//+++++++++++++++++++++++++++++++++++ begin erase LDROM
		printk("MicroP updateMicroPFirmwareTest begin erase LDROM\n");
		j = 0x00;
		while(j < 0x10)
		{
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x80F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x11;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x00;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;

		//printk("Walf updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F3\n");
		}

		// if Buffer[0]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x80F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x81F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x11;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F7\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x80;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		//printk("Walf updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F0\n");
		}
		
		// if Buffer[1]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x81F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		j++;
		}
		printk("MicroP updateMicroPFirmwareTest end erase LDROM\n");
		//----------------------------------- end erase LDROM
		g_fw_update_progress=7;
		//----------------------------------- begin update LDROM
		printk("MicroP updateMicroPFirmwareTest begin update LDROM\n");
		// if Buffer[0]Status == 2
		j = 0x00;
		while(j < 0x10)
		{
		//printk("Walf updateMicroPFirmwareTest start write buffer 0\n");
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x80F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		// Write Buffer 0
		// send 128 bytes data
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x8000, img_buf+img_offset, 128);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x8000\n");
		}
		
		img_offset += 128;
		//printk("Walf updateMicroPFirmwareTest j=%d, img_offset=%d\n", j, img_offset);

		// set 373x flash CMD
		i2cdata[0]=0x20;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x00;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		//printk("Walf updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
		}
		
		// if Buffer[0]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x80F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		// if Buffer[1]Status == 2
		//printk("Walf updateMicroPFirmwareTest start write buffer 1\n");
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x81F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		// Write Buffer 1
		// send 128 bytes data
			ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x8100, img_buf + img_offset, 128);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest fail 0x8100\n");
			}
			img_offset += 128;
			//printk("Walf updateMicroPFirmwareTest j=%d, img_offset=%d\n", j, img_offset);

		// set 373x flash CMD
		i2cdata[0]=0x20;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F7\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x80;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		//printk("Walf updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F0\n");
		}
		
		// if Buffer[1]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x81F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		j++;
		
		}
		printk("MicroP updateMicroPFirmwareTest end update LDROM\n");
		//----------------------------------- end update LDROM
		if(!flag_ap)
			flag_ap = 1;
	}
	if(flag_ap == 1){//Eric +++
		g_fw_update_progress=14;
		img_offset = 4*1024;//Eric +++
		//+++++++++++++++++++++++++++++++++++ begin erase APROM
		printk("MicroP updateMicroPFirmwareTest begin erase APROM\n");
		j = 0x10;
		while(j < 0x70)
		{
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x80F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x11;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x00;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;

		//printk("Walf updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F3\n");
		}

		// if Buffer[0]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x80F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x81F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x11;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F7\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x80;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		//printk("Walf updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F0\n");
		}
		
		// if Buffer[1]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x81F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		j++;
		}
		printk("MicroP updateMicroPFirmwareTest end erase APROM\n");
		//----------------------------------- end erase APROM
		g_fw_update_progress=64;
		//----------------------------------- begin update APROM
		printk("MicroP updateMicroPFirmwareTest begin update APROM\n");
		// if Buffer[0]Status == 2
		j = 0x10;
		while(j < 0x70)
		{
		//printk("Walf updateMicroPFirmwareTest start write buffer 0\n");
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x80F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		// Write Buffer 0
		// send 128 bytes data
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x8000, img_buf+img_offset, 128);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x8000\n");
		}
		
		img_offset += 128;
		//printk("Walf updateMicroPFirmwareTest j=%d, img_offset=%d\n", j, img_offset);

		// set 373x flash CMD
		i2cdata[0]=0x20;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x00;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		//printk("Walf updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
		}
		
		// if Buffer[0]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x80F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			printk("MicroP updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x80F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		// if Buffer[1]Status == 2
		//printk("Walf updateMicroPFirmwareTest start write buffer 1\n");
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest not read Buffer[0]Status == 2 0x81F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		// Write Buffer 1
		// send 128 bytes data
			ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x8100, img_buf+img_offset, 128);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest fail 0x8100\n");
			}
			img_offset += 128;
			//printk("Walf updateMicroPFirmwareTest j=%d, img_offset=%d\n", j, img_offset);

		// set 373x flash CMD
		i2cdata[0]=0x20;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F7\n");
		}
		
		// set 373x flash start address,size,status
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x80;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		//printk("Walf updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("MicroP updateMicroPFirmwareTest fail 0x81F0\n");
		}
		
		// if Buffer[1]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("MicroP updateMicroPFirmwareTest 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Walf updateMicroPFirmwareTest 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			printk("MicroP updateMicroPFirmwareTest update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			printk("MicroP updateMicroPFirmwareTest not read 0x81F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			printk("MicroP updateMicroPFirmwareTest not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			//if(img_buf) kfree(img_buf);
			//img_buf=NULL;
			//return 0;
			upfw_retry++;
			printk("MicroP updateMicroPFirmwareTest fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		j++;
		
		}
		printk("MicroP updateMicroPFirmwareTest end update APROM\n");
		//----------------------------------- end update APROM
	}
	if(flag_battery == 1){//Eric +++
		g_fw_update_progress=90;
		img_offset = 28*1024;//Eric +++
		//++++++++++++++++++++++++++++++++++ start earse battery 
		printk("MicroP updateMicroPFirmwareTest begin erase Battery config\n");
		j = 0x70;
		while(j < 0x74)
		{
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Erci battery 0x80F6 host clear fail \n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			//printk("Eric battery 0x80F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric  fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x11;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs fail write 0x80F7\n");
		}
		
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x00;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs fail write 0x80F0\n");
		}

		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block read  0x80F6 fail\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			//printk("Eric erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			//printk("Eric not read 0x80F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			//printk("Eric not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry 2 =%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			//printk("Eric not read Buffer[0]Status == 2 0x81F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry 3 =%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		i2cdata[0]=0x11;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs write 0x81F7\n");
		}
		
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x80;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs fail 0x81F0\n");
		}
		
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			//printk("Eric erase address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			//printk("Eric not read 0x81F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			//printk("Eric not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		j++;
		}
		printk("MicroP updateMicroPFirmwareTest end erase Battery config\n");

		//---------------------------------- end earse battery
		
		//++++++++++++++++++++++++++++++++++ start update battery info	
		printk("MicroP updateMicroPFirmwareTest begin update Battery config\n");
		j = 0x70;
		while(j < 0x74)
		{
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block 0x80F6 fail 1\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			//printk("Eric not read Buffer[0]Status == 2 0x80F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;

		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x8000, img_buf+img_offset, 128);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs write fail 0x8000\n");
		}
		
		img_offset += 128;

		// set 373x flash CMD
		i2cdata[0]=0x20;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs write fail 0x80F7\n");
		}
		
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x00;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs write fail 0x80F0\n");
		}

		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block read fail 0x80F6 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x80F6=0x%x\r\n",i2cdata[0]);     
				flag = 1;
				break;
			}
			//printk("Eric update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			//printk("Eric not read 0x80F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			//printk("Eric not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;

		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block read 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x02)
			{
				//printk("Eric break 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			//printk("Eric not read Buffer[0]Status == 2 0x81F6=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		// Write Buffer 1
		// send 128 bytes data
			ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x8100, img_buf+img_offset, 128);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_write_regs write fail 0x8100\n");
			}
			img_offset += 128;
			//printk("Walf updateMicroPFirmwareTest j=%d, img_offset=%d\n", j, img_offset);

		// set 373x flash CMD
		i2cdata[0]=0x20;
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F7, i2cdata, 1);
		if(ret < 0)
		{
			printk("Eric fail 0x81F7\n");
		}
		
		i2cdata[0]=0x00;
		i2cdata[1]=j;
		i2cdata[2]=0x80;
		i2cdata[3]=0x00;
		i2cdata[4]=0x00;
		i2cdata[5]=0x80;
		i2cdata[6]=0x00;
		
		ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x81F0, i2cdata, 7);
		if(ret < 0)
		{
			printk("Eric io373x_i2c_write_regs write fail 0x81F0\n");
		}
		
		// if Buffer[1]Status == 1
		i2cdata[0]=0x00;
		i=0;
		while(i<100)
		{
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0x81F6, i2cdata, 1);
			if(ret < 0)
			{
				printk("Eric io373x_i2c_read_regs_i2c_block read 0x81F6 fail 3\n");
			}
			if(i2cdata[0] == 0x01 || i2cdata[0] == 0x02)
			{
				//printk("Eric 0x81F6=0x%x\r\n",i2cdata[0]);   
				flag = 1;  
				break;
			}
			//printk("Eric update address=0x%02x%02x%02x\n", i2cdata[0], i2cdata[1], i2cdata[2]);
			//printk("Eric not read 0x81F6=0x%x\r\n",i2cdata[0]);  
			ret = io373x_i2c_read_regs_i2c_block(&(g_uP_info->i2c_client->dev), 0xF011, i2cdata, 1);
			//printk("Eric not read 0xF011=0x%x\r\n",i2cdata[0]);  
			msleep(50);
			i++;
		}
		if(flag == 0) 
		{
			upfw_retry++;
			//printk("Eric fail upfw_retry=%d\n", upfw_retry);
			if(upfw_retry < 10)
			{
				goto begin_upfw;
			}
			else
			{
				goto end_upfw;
			}
		}
		flag = 0;
		
		j++;
		
		}
		printk("MicroP updateMicroPFirmwareTest end update Battery config\n");
		//---------------------------------- end update battery info
	}
	//+++++++++++++++++++++++++++++++++++ finish update FW
	i2cdata[0]=0x80;
	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F7, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 0x80F7\n");
	}

	i2cdata[0]=0x00;

	ret = io373x_i2c_write_regs(&(g_uP_info->i2c_client->dev), 0x80F6, i2cdata, 1);
	if(ret < 0)
	{
		printk("MicroP updateMicroPFirmwareTest fail 0x80F6\n");
	}
	//----------------------------------- finish update FW

	flag_ld = 0;
	flag_ap = 0;
	flag_battery = 0;
	g_fw_update_progress=100;
	
	msleep(1000);
	uP_i2c_read_reg(MICROP_ALWAYS_IGNORE_PHONE_READY,&second_ignore);
	printk("update finish ignore_phone_ready value is =0x%x\n", second_ignore);

	uP_i2c_write_reg(MICROP_IND_PHONE_READY,&a68_ready);
	always_ignore=0;
	printk("%s: clear \"MICROP_ALWAYS_IGNORE_A68READY\" flag \r\n", __FUNCTION__);                                    
	uP_i2c_write_reg(MICROP_ALWAYS_IGNORE_PHONE_READY,&always_ignore);
	uP_i2c_write_reg(MICROP_MISC_CMD_WOS,&clear_power_reason);
end_upfw:
	g_uP_info->i2c_client->addr = g_slave_addr;
	printk("\nMicroP updateMicroPFirmware: Set addr=%d\n", g_uP_info->i2c_client->addr);
        uP_i2c_read_reg(MICROP_BOOT_SELECTION,&g_curr_uP_mode); //update boot mode
	enable_irq(gpio_to_irq(g_microp_irq_gpio));
	if(img_buf)
            kfree(img_buf);
        img_buf=NULL;
        g_fw_updating = 0;
        micropSendNotify(PAD_UPDATE_FINISH);
	      
	return ret;
}

static int asus_microp_open(struct inode *inode, struct file *flip){
        pr_debug("%s\r\n",__FUNCTION__);
        return 0;
}
static int asus_microp_release(struct inode *inode, struct file *flip){
        pr_debug("%s\r\n",__FUNCTION__);
        return 0;

}

static int testPlugInOut(unsigned long arg){
    int i,j;
    int k = 0;
    struct ioctl_data val;
    if(copy_from_user(&val, (struct ioctl_data *)arg, sizeof(struct ioctl_data))) {
        pr_err("%s: Failed to copy arg from user", __FUNCTION__);
        return -EFAULT;
    }

    printk("Eric total_counts = %d\r,time_interval = %d\r\n",val.total_counts,val.time_interval);
    i = val.total_counts;
    j = val.time_interval;
    while(k<i) {
        // test plug in
        printk("simulate pad insert time=%d\n", k);
        notify_microp_insert();
        switch_set_state(&p01_switch_dev,st_CONNECTED);
        switch_set_state(&pfs_switch_version,st_CONNECTED);
        g_uPadErrStatus=0;      // reset error status to 0
        switch_set_state(&pad_err_notify, 0);
        msleep(j*1000);
        k++;

        // test plug out
        printk("simulate pad remove time=%d\n", k);
        notify_microp_remove(0);
    }
    return 0;
}

static long asus_microp_ioctl(struct file *flip, unsigned int cmd, unsigned long arg){

	int ret=0;
	int status = 0;
	char tmp_pro[5]={0};
	uint16_t test_hwid =0;
	int pad_hwid = 0;
	uint8_t ts_id=0;
        uint8_t board_hwid =0;
	
#ifdef ASUS_FACTORY_BUILD
	unsigned int sel_offset = 0;
	uint8_t set_factory_mode = 0xCC;
#endif

//       int val=0;
	printk("%s\r\n",__FUNCTION__);
           
	if(_IOC_TYPE(cmd) != ASUS_MICROP_IOC_TYPE)
		return -ENOTTY;
	if(_IOC_NR(cmd) > ASUS_MICROP_MAX_NR)
		return -ENOTTY;

	if(g_uP_info==NULL)
		return -EFAULT;
       
	switch (cmd) {
		case ASUS_MICROP_PAD_TEST_PLUG_IN_OUT:
			printk("Eric: ASUS_MICROP_PAD_PLUG_IN_OUT+++\r\n");
			ret=testPlugInOut(arg); 
			printk("Eric: ASUS_MICROP_PAD_PLUG_IN_OUT---\r\n");
			break;
		case ASUS_MICROP_SEND_CFG_INFO:
			printk("Eric: ASUS_MICROP_SEND_CFG_INFO+++\r\n");
			if(copy_from_user(&fw_info, (struct microP_fwinfo *)arg, sizeof(struct microP_fwinfo))) {
				pr_err("%s: Failed to copy arg from user", __FUNCTION__);
				return -EFAULT;
			}
			printk("Eric Target ap_version == 0x%x\r\n---\r\n",fw_info.ap_version);
			printk("Eric Target ld_version == 0x%x\r\n---\r\n",fw_info.ld_version);
			printk("Eric Target battery_id == 0x%x\r\n---\r\n",fw_info.battery_id);
			printk("Eric Target config_id == 0x%x\r\n---\r\n",fw_info.config_id);
			printk("Eric: ASUS_MICROP_SEND_CFG_INFO---\r\n");
			break;
		case ASUS_MICROP_FW_UPDATE:
			pr_info("ioctl: ASUSEC_FW_UPDATE ++\r\n");
			status = uP_i2c_read_reg(MICROP_OEM_FW_VERSION,&mcirop_ver);
			memcpy(tmp_pro,mcirop_ver+4,1);
			printk("Eric: mcirop_ver=%32s,status = %d,tmp_pro=%s\r\n",mcirop_ver,status,tmp_pro);	
			if(status > 0 || strcmp(fw_info.cmd_style,"-r") == 0){
			    printk("A12 fw update+++\r\n");
  			    ret=updateMicroPFirmware_ENE(arg);  
			    printk("A12 fw update---\r\n");
                        }
			else
			    ret=-EINVAL;

			pr_info("ioctl: ASUSEC_FW_UPDATE --\r\n");
			break;
		case ASUS_MICROP_CHECK_CONNECTED:      
			ret=__put_user((g_b_isP01Connected==st_CONNECTED),(int __user *)arg);
			break;
		case ASUS_MICROP_GET_BATTERY_ID://MICROP_GAUGE_ID
			printk("Eric: ASUS_MICROP_GET_BATTERY_ID+++\r\n");
			uP_i2c_read_reg(MICROP_GAUGE_ID,&g_config_id);
			if(pad_exist())
				ret=__put_user(g_config_id,(int __user *)arg);
			else
				ret=-EINVAL;
			printk("Eric: ASUS_MICROP_GET_BATTERY_ID---\r\n");
			break;
		case ASUS_MICROP_GET_CONFIG_ID:
			printk("Eric: ASUS_MICROP_GET_CONFIG_ID+++\r\n");
			//uP_i2c_read_reg(MICROP_BATTERY_CONFIG,&g_battery_cfg_ver);
			if(pad_exist())
				ret=__put_user(fw_data[2],(int __user *)arg);
			else
				ret=-EINVAL;
			printk("Eric: ASUS_MICROP_GET_CONFIG_ID---\r\n");
			break;
		case ASUS_MICROP_GET_FW_VERSION:
			/*status = uP_i2c_read_reg(MICROP_OEM_FW_VERSION,&mcirop_ver);
			if(status > 0){
				memcpy(tmp,mcirop_ver+10,4);
				printk("Eric: mcirop_ver=%32s,status = %d,tmp=%s\r\n",mcirop_ver,status,tmp);	
				num = atoh(tmp,4);
				ret=__put_user(num,(int __user *)arg);
			}
			else
				ret=-EINVAL;*/	

			if(pad_exist())
				ret=__put_user(g_microp_ver,(int __user *)arg);
			else
				ret=-EINVAL;
			break;
		case ASUS_MICROP_GET_LDROM_VERSION:
			if(pad_exist())
				ret=__put_user(g_ldrom_ver,(int __user *)arg);
			else
				ret=-EINVAL;

			break;
		case ASUS_MICROP_GET_PADPHONE_HW_ID:
			pad_hwid = AX_MicroP_getMICROPID();
			ret=__put_user(pad_hwid, (int __user *)arg);
			break;
                case ASUS_MICROP_GET_HW_ID:
                        board_hwid = AX_MicroP_getHWID();
                        ret=__put_user(board_hwid, (int __user *)arg);
                        break;
		case ASUS_MICROP_GET_TS_ID:
			ts_id = AX_MicroP_getTSID();
			ret=__put_user(ts_id, (int __user *)arg);
                        break;

#ifdef ASUS_FACTORY_BUILD
		case ASUS_MICROP_POWER_ON_LED_O:
			uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = OUT_uP_LED_O;
			ret = uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset);
			break;
		case ASUS_MICROP_POWER_OFF_LED_O:
			uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = OUT_uP_LED_O;
			ret = uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR, &sel_offset);
			break;
		case ASUS_MICROP_POWER_ON_LED_G:
			uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = OUT_uP_LED_G;
			ret = uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset);
			break;
		case ASUS_MICROP_POWER_OFF_LED_G:
			uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
			sel_offset = OUT_uP_LED_G;
			ret = uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR, &sel_offset);
			break;
#endif
		case ASUS_MICROP_SEND_VREMOVE_UEVENT:
			switch_set_state(&pad_err_notify, 3);
			break;
		default:
			pr_err("Invalid ioctl code\r\n");
			ret= -EINVAL;
			break;
        }

	return ret;
}


static struct file_operations asus_microp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asus_microp_ioctl,
	.open = asus_microp_open,
	.release = asus_microp_release,
};

static struct miscdevice asus_microp_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = MICROP_NAME,
	.fops = &asus_microp_fops,
	.mode= 0666,
};


static ssize_t pad_ReportI2CFail_show(struct device *dev, struct device_attribute *attr, char *buf)
{
        int count =0;
        if(g_prop_virtualRemoveEnabled)
            count+=sprintf(buf,"virtual remove enabled\r\n");
        else
            count+=sprintf(buf,"virtual remove disabled\r\n");

        return count;
}
static ssize_t pad_ReportI2CFail_store(struct device *dev, struct device_attribute *attr,const char *buf, size_t count)
{
       int val=0;

       val = simple_strtol(buf, NULL, 10);       

        if(val > 0){
                printk("[MicroP] enable Virtual Remove!!\r\n");
                g_prop_virtualRemoveEnabled=1;
        }
        else{
                printk("[MicroP] disable Virtual Remove!!\r\n");
                g_prop_virtualRemoveEnabled=0;
        }
 
 
	return count;
}


/*
* show/restory interface
*/
static int g_cmd_idx=0;
static ssize_t reg_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
    uint64_t val=0;
    int count =0;
    int status=0;
    char sz_isn[32]={0};
 
    if(uP_CMD_Table[g_cmd_idx].len >= 32){
        status=uP_i2c_read_reg(g_cmd_idx, sz_isn);
        if(status > 0){
            count+=sprintf(buf, "\"%16s\": addr[0x%2x] = %32s(ASCII)\r\n",uP_CMD_Table[g_cmd_idx].name,
                                                                         uP_CMD_Table[g_cmd_idx].addr,
                                                                         sz_isn);
        }
        else
            count+=sprintf(buf,"reg read failed %d\r\n",status);
    }
    else{
        status=uP_i2c_read_reg(g_cmd_idx,&val);
        if(status > 0){
            count+=sprintf(buf, "\"%16s\": addr[0x%2x] = %16llx(h)\r\n",uP_CMD_Table[g_cmd_idx].name,
                                                                        uP_CMD_Table[g_cmd_idx].addr,
                                                                        val);
        }
        else
            count+=sprintf(buf,"reg read failed %d\r\n",status);
    }
    return count;
}
static ssize_t reg_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
    	int addr=0;
       uint64_t val=0;
	/*if(st_CONNECTED!=g_b_isP01Connected){
		pr_info("microp: failed, not connected\r\n");
		return 0;
	}*/
       if(buf[0]=='-'){
            addr = simple_strtol(buf+1, NULL, 16);       
            g_cmd_idx=getCmdID(addr);
            pr_info("set addr=0x%2x, ID=%d\r\n",addr,g_cmd_idx);
            if(g_cmd_idx==-1)
                    g_cmd_idx=0;
       }
       else{    //write value to idx
                if(g_cmd_idx<0 && g_cmd_idx > ARRAY_SIZE(uP_CMD_Table))
                        pr_info("set addr first\r\n");
                else{
                        val=simple_strtol(buf, NULL, 16);
                        pr_info("write val 0x%llx to addr [0x%x]\r\n", val, uP_CMD_Table[g_cmd_idx].addr);
                        uP_i2c_write_reg(g_cmd_idx, &val);
                }
       }

	return count;
}


/*
* show/restory interface
*/
static ssize_t pad_update_progress_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	return sprintf(buf, "%d\r\n",g_fw_update_progress);
}

static ssize_t pad_station_check(struct device *dev, struct device_attribute *attr,
                                 char *buf)
{
    checkPadExist(0);
    return sprintf(buf,"0");
}

/*
* show/restory interface
*/
/*static ssize_t pad_proximity_sensor_status_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
	int count=0;
	uint8_t ext_io_status=0;
	if(st_CONNECTED!=g_b_isP01Connected){
		count+= sprintf(buf,"0\r\n");   // return 0 if Pad not connected
	}
	else{
		if(isPhoneA68()){ //a68
			if(AX_MicroP_getGPIOPinLevel(IN_CAP_RPOX_INT_R) == 0)
				count+= sprintf(buf,"1\r\n");        // return 1 if Pad connected & pin is low
			else
				count+= sprintf(buf,"0\r\n");        // return 0 if Pad connected & pin is high or i2c err
		}
		else if(isPhoneA80()){ //a80
			uP_i2c_read_reg(MICROP_EXTEND_IO_STATUS , &ext_io_status);
			count+= sprintf(buf,"(s1,s2,s3) = (%d, %d, %d)\r\n", (ext_io_status & 0x01)?1:0,
					(ext_io_status & 0x02)?1:0,
					(ext_io_status & 0x04)?1:0);        // return 1 if Pad connected & pin is low
		}
	}
	
	return count;
}*/

static ssize_t isn_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
       char isn[32]={0};
       int count =0;
       int status=0;

	if(st_CONNECTED!=g_b_isP01Connected){
            	count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}
       
       status=uP_i2c_read_reg(MICROP_ISN,isn);
       if(status > 0){                    
            count+=sprintf(buf, "%s\r\n", isn);
       }
       else 
            count+=sprintf(buf,"0\r\n");
        
	return count;
}

#ifdef ASUS_FACTORY_BUILD
static ssize_t isn_store(struct device *dev, struct device_attribute *attr,
                            const char *buf, size_t count)
{
    char oem_isn[32]="";
    u8 ldrom_mode = 0x5A;
    u8 aprom_mode = 0xA5;

    if(st_CONNECTED!=g_b_isP01Connected){
        pr_info("microp: failed, not connected\r\n");
        return 0;
    }
    if(buf){
        uP_i2c_write_reg(MICROP_SET_BOOT_LDROM, &ldrom_mode);
        strcpy(oem_isn,buf);
        printk("try to write oem isn %32s\r\n", oem_isn);
        uP_i2c_write_reg(MICROP_ISN, oem_isn);
        uP_i2c_write_reg(MICROP_SET_BOOT_APROM, &aprom_mode);
    }
    return count;
}
#endif

ssize_t oem_ssn_show(struct device *dev, struct device_attribute *attr,
				 char *buf)
{
       char oem_ssn[32]="";
       int count =0;
       int status=0;

	if(st_CONNECTED!=g_b_isP01Connected){
            	count+=sprintf(buf,"microp: failed, not connected\r\n");
		return count;
	}
       
       status=uP_i2c_read_reg(MICROP_OEM_SSN,oem_ssn);
       if(status > 0){                    
            count+=sprintf(buf, "%s\r\n", oem_ssn);
       }
       else 
            count+=sprintf(buf,"0\r\n");
        
	return count;
}

static ssize_t oem_ssn_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
       char oem_ssn[32]="";        
        if(st_CONNECTED!=g_b_isP01Connected){
                pr_info("microp: failed, not connected\r\n");
                return 0;
        }
        if(buf){
                strcpy(oem_ssn,buf);
                printk("try to write oem isn %32s\r\n", oem_ssn);
                uP_i2c_write_reg(MICROP_OEM_SSN, oem_ssn);
                
                
        }
        return count;
}

static ssize_t gpio_36_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int count=0;
	
	if(pad_exist()){
		count += sprintf(buf,"1\r\n");
	}
	else{
		count += sprintf(buf,"0\r\n");
	}

	return count;
}

static ssize_t hs_5v_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    int count =0;
        if(st_CONNECTED!=g_b_isP01Connected){
                count+=sprintf(buf,"microp: failed, not connected\r\n");
                return count;
        }
    if(AX_MicroP_getGPIOOutputPinLevel(OUT_uP_VBUS_EN) == 1){
            count += sprintf(buf,"1\r\n");
    }else{
            count += sprintf(buf,"0\r\n");
    }

        return count;
}
static ssize_t hs_5v_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    printk("hs_5v_store %c\n", buf[0]);
    if(st_CONNECTED!=g_b_isP01Connected){
        printk("microp: failed, not connected\r\n");
        return 0;
    }

    if(buf[0] == '1')
    {
        printk("set VBUS =1\n");
        AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, 1);

    }
    else if(buf[0] == '0')
    {
        printk("set VBUS =0\n");
        AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN, 0);
    }

    return count;
}

static ssize_t pad_action_store(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	printk("pad_action_store %c\n", buf[0]);
	if(buf[0] == '1')
	{
		printk("simulate pad insert\n");
		g_b_isP01Connected=st_CONNECTED;
        #ifdef ASUS_A68M_PROJECT
            change_backlight_mode(P01_ADD);
            mdss_microp_event_handler(P01_ADD);
        #else
		micropSendNotify(P01_ADD);
        #endif
		switch_set_state(&p01_switch_dev,st_CONNECTED);
        switch_set_state(&pfs_switch_version,st_CONNECTED);
		g_uPadErrStatus=0;      // reset error status to 0
		switch_set_state(&pad_err_notify, 0);
        set_antenna_to_pad_main();
	}
	else if(buf[0] == '0')
	{
		printk("simulate pad remove\n");
		g_b_isP01Connected=st_DISCONNECTED;
		switch_set_state(&p01_switch_dev,st_DISCONNECTED);
        switch_set_state(&pfs_switch_version,st_DISCONNECTED);
        #ifdef ASUS_A68M_PROJECT
            change_backlight_mode(P01_REMOVE);
            mdss_microp_event_handler(P01_REMOVE);
        #endif
		micropSendNotify(P01_REMOVE);
        set_antenna_to_phone(); // Walf
	}

    return count;
}

static ssize_t led_test_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    uint8_t set_factory_mode = 0xCC;
    uint8_t hwid;
    unsigned int sel_offset = 0;
    int value;
    printk("led_test_store %c\n", buf[0]);
    if(st_CONNECTED!=g_b_isP01Connected){
        printk("microp: failed, not connected\r\n");
        return 0;
    }

    hwid = AX_MicroP_getHWID();
    if ( hwid == P72_PR_HWID || hwid == P72_MP_HWID) {
        if(buf[1] == '1')
            value = 0;
        else if (buf[1] == '0')
            value = 1;
    } else {
        if(buf[1] == '1')
            value = 1;
        else if (buf[1] == '0')
            value = 0;
    }

    if(buf[0] == 'o') {
        if(value == 1)
        {
            printk("set LED o=1\n");
            uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
            sel_offset = OUT_uP_LED_O;
            uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset);
        }
        else if(value == 0)
        {
            printk("set LED o=0\n");
            uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
            sel_offset = OUT_uP_LED_O;
            uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR, &sel_offset);
        }
    }
    else if ( buf[0] == 'g')
    {
        if(value == 1)
        {
            printk("set LED g=1\n");
            uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
            sel_offset = OUT_uP_LED_G;
            uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_SET, &sel_offset);
        }
        else if(value == 0)
        {
            printk("set LED g=0\n");
            uP_i2c_write_reg(MICROP_DISABLE_CHARGING_FOR_FACTORY, &set_factory_mode);
            sel_offset = OUT_uP_LED_G;
            uP_i2c_write_reg(MICROP_GPIO_OUTPUT_BIT_CLR, &sel_offset);
        }
    }

    return count;
}

static ssize_t notify_test_store(struct device *dev, struct device_attribute *attr,
                const char *buf, size_t count)
{
    int value =0;
    printk("notify_test_store %c\n", buf[0]);
#if 0
uP_i2c_write_reg(MICROP_GPIO_OUTPUT_LEVEL, &value);
AX_MicroP_setGPIOOutputPin(OUT_uP_SPK_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_LCD_RST,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_EN_1V8,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_TS_RST_R,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_TS_PWR_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_EN_3V3_1V2,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_5V_PWR_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_VBUS_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_AUD_PWR_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_LED_O,1);
AX_MicroP_setGPIOOutputPin(OUT_uP_LED_G,1);
AX_MicroP_setGPIOOutputPin(OUT_up_PROXM_PWR_EN,1);
AX_MicroP_setGPIOOutputPin(OUT_up_SPK_SEL,1);
AX_MicroP_setGPIOOutputPin(OUT_up_RCV_SEL,1);
#endif

    if (kstrtoul(buf, 0, &value))
        return -EINVAL;

    micropSendNotify(value);
    return count;
}

static DEVICE_ATTR(pad_ReportI2CFail, 0644, pad_ReportI2CFail_show, pad_ReportI2CFail_store);
#ifdef ASUS_FACTORY_BUILD
	static DEVICE_ATTR(reg, 0666, reg_show, reg_store);
#else
	static DEVICE_ATTR(reg, 0664, reg_show, reg_store);
#endif
static DEVICE_ATTR(pad_update_progress, 0644, pad_update_progress_show, NULL);
//static DEVICE_ATTR(pad_proximity_sensor_status, 0644, pad_proximity_sensor_status_show, NULL);
#ifdef ASUS_FACTORY_BUILD
static DEVICE_ATTR(oem_ssn, 0666, oem_ssn_show, oem_ssn_store);
static DEVICE_ATTR(isn, 0644, isn_show, isn_store);
#else
static DEVICE_ATTR(oem_ssn, 0664, oem_ssn_show, oem_ssn_store);
static DEVICE_ATTR(isn, 0644, isn_show, NULL);
#endif
#ifdef ASUS_FACTORY_BUILD
static DEVICE_ATTR(hs_5v, 0666, hs_5v_show, hs_5v_store);
#else
static DEVICE_ATTR(hs_5v, 0664, hs_5v_show, hs_5v_store);
#endif
static DEVICE_ATTR(gpio_36, 0644, gpio_36_show, NULL);
static DEVICE_ATTR(pad_action, 0664, NULL, pad_action_store);
static DEVICE_ATTR(led_test, 0664, NULL, led_test_store);
static DEVICE_ATTR(notify_test,0664, NULL, notify_test_store);
static DEVICE_ATTR(pad_station_check,0664,pad_station_check,NULL);

//gpio switch class related-functions
static ssize_t p01_switch_name(struct switch_dev *sdev, char *buf)
{
       char model_name[30]={0};
       if(st_CONNECTED==g_b_isP01Connected)
                sprintf(model_name,"p%03x",g_microp_ver);
       else
                sprintf(model_name,"unknown");
       
	return sprintf(buf, "%s\n", model_name);
}


static ssize_t p01_switch_state(struct switch_dev *sdev, char *buf)
{        
    return sprintf(buf, "%s\n", (g_b_isP01Connected > 0 && g_b_isP01Connected < 4)?"1":"0");
}

static ssize_t p01_usb_switch_name(struct switch_dev *sdev, char *buf)
{ 
    return sprintf(buf, "Asus-P01\n");
}

static ssize_t p01_usb_switch_state(struct switch_dev *sdev, char *buf)
{
//	return sprintf(buf, "%s\n", (g_b_isP01USBConnected ? "1" : "0"));
       return sprintf(buf, "%d\n", g_b_isP01USBConnected);
}

static ssize_t pfs_switch_name(struct switch_dev *sdev, char *buf)
{
       char oem_version[30]={0};
       if(st_CONNECTED==g_b_isP01Connected)
                sprintf(oem_version,"%s\n",mcirop_ver);
       else
                sprintf(oem_version,"unknown");    
	return sprintf(buf, "%s\n", oem_version);
}

static ssize_t pad_notify_switch_state(struct switch_dev *sdev, char *buf)
{
	int count=0;
       count+= sprintf(buf,"%d\n", g_uPadErrStatus);
       return count;
}

#define MIPI_PWR_EN 40 // GP_XDP_BLK_DP = gp_aon_040 = 40
#define MIPI_SW_SEL 159 //GP_CORE_063 = 63 + 96 = 159
#ifdef CONFIG_HAS_EARLYSUSPEND
static void microp_early_suspend(struct early_suspend *h)
{
    printk("enter %s\n", __func__);
    if ( g_b_isP01USBConnected != 1 || pad_exist()==0) {
        // set mipi switch off when system suspend
        if (gpio_direction_output(MIPI_PWR_EN, 0))
            gpio_set_value_cansleep(MIPI_PWR_EN, 0);
        if (gpio_direction_output(MIPI_SW_SEL, 0))
            gpio_set_value_cansleep(MIPI_SW_SEL, 0);
    }

    if(AX_MicroP_IsP01Connected() && pad_exist() ){
        AX_MicroP_enterSleeping();
    }
    return 0;

}

static void microp_late_resume(struct early_suspend *h)
{
    // for debug
    uint32_t reg_input=0, out_reg=0, state=0 ;
    uint8_t reg_powerOnReason = 0;

    printk("%s ++\r\n",__FUNCTION__);
    if(AX_MicroP_IsP01Connected() && pad_exist()){
        AX_MicroP_enterResuming();
    }
    return 0;
}

#else
#define microp_early_suspend NULL
#define microp_late_resume NULL
#endif

static int microp_suspend(struct i2c_client *client,
	pm_message_t mesg)
{
    int cur_state = 0;
    printk("%s ++\r\n",__FUNCTION__);

    if(AX_MicroP_IsP01Connected() && pad_exist()){
        AX_MicroP_enablePinInterrupt(INTR_EN_VOL_DOWN | INTR_EN_VOL_UP , 0);
    
        uP_i2c_read_reg(MICROP_OPERATING_STATE, &cur_state);
        printk("Microp:cur_state:%d\n",cur_state);
        if (cur_state == st_MICROP_Active)
            AX_MicroP_enterSleeping();
    }
    return 0;
}

static int microp_resume(struct i2c_client *client)
{
    uint32_t reg_intr_sta = 0,reg_input=0;
    int cur_state = 0;

#if 1
    // for debug
    uint32_t out_reg=0, state=0 ;
    uint8_t reg_powerOnReason = 0;
    uint8_t clear_power_reason = MICROP_CLEAR_POWER_REASON;

    printk("%s ++\r\n",__FUNCTION__);
    if(AX_MicroP_IsP01Connected() && pad_exist()){
        uP_i2c_read_reg(MICROP_OPERATING_STATE, &cur_state);
        printk("MicroP:cur_state:%d\n",cur_state);
        AX_MicroP_enablePinInterrupt(INTR_EN_VOL_DOWN | INTR_EN_VOL_UP , 1);
        if(uP_i2c_read_reg(MICROP_GPIO_INPUT_LEVEL, &reg_input) > 0
               && uP_i2c_read_reg(MICROP_GPIO_OUTPUT_LEVEL, &out_reg) > 0
               && uP_i2c_read_reg(MICROP_OPERATING_STATE, &state) > 0)
            printk("[PAD DEBUG] state=%d, uP In=0x%8x, Out=0x%8x\r\n", state, reg_input, out_reg);
                    //ASUSEvtlog("[PAD DEBUG] state=%d, uP In=0x%8x, Out=0x%8x, gpio 7=%d\r\n", state, reg_input, out_reg, gpio_get_value(7));
        if(st_MICROP_Off==state){
            uP_i2c_read_reg(MICROP_POWER_ON_REASON, &reg_powerOnReason);
            printk("reg_powerOnReason=%d\r\n", reg_powerOnReason);
            uP_i2c_write_reg(MICROP_MISC_CMD_WOS,&clear_power_reason);
            if(E_ON_PWR_KEY_LONGPRESS==reg_powerOnReason){
                printk("resume might due to long press pwr key\r\n");
                if(IsPadPwrBtnPressed()){
                    printk("compensate sending pwr key pressed\r\n");
                    micropSendNotify(P01_PWR_KEY_PRESSED);
                }
            }
        }
    }
#endif
    printk("%s --\r\n",__FUNCTION__);
    return 0;
}

static int microP_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
    struct microP_info *info;
    //struct microP_platform_data *microp_pdata; // Walf
    int ret=0;
//    uint8_t reg_powerOnReason=0;
//    int pad_detect_gpio = 0;
    //struct proc_dir_entry *entry;       

    printk("microP_probe\n");
    pr_info("microP: %s +++\n", __FUNCTION__);
    g_uP_info=info = kzalloc(sizeof(struct microP_info), GFP_KERNEL);
    if (!info) {
        pr_err("microP: Unable to allocate memory!\n");
        return -ENOMEM;
    }

    mutex_init(&info->write_read_lock);

//	info->carkit_inited=0;
//	microp_pdata=info->pdata = client->dev.platform_data; // Walf
    g_slave_addr=client->addr;
    info->i2c_client = client;
    i2c_set_clientdata(client, info);

	/*if(microp_pdata==NULL){
		pr_err("microP: microp_pdata NULL!\n");
		return -ENOMEM;
	}*/ // Walf

    microp_slow_job_wq = create_singlethread_workqueue("microp_slow_job");
    microp_ins_rev_wq = create_singlethread_workqueue("microp_ins_rev_wq");
    microp_intr_wq = create_singlethread_workqueue("microp_intr_wq");
    microp_detect_wq = create_singlethread_workqueue("microp_detect_wq");

    wake_lock_init(&interrupt_lock_t, WAKE_LOCK_SUSPEND, "asus_ec_wakelock_timeout");

    // initialize work struct
    INIT_DELAYED_WORK(&info->work, microP_work);
    INIT_DELAYED_WORK(&info->initP01, initP01);
    INIT_DELAYED_WORK(&info->deinitPad, deinitPad);
    INIT_DELAYED_WORK(&info->poweroffP01, microp_poweroff);
    INIT_DELAYED_WORK(&info->paddetect, microp_paddetect);
    // init mutex
//    mutex_init(&microp_mutex_lock);

    g_microp_irq_gpio = get_gpio_by_name(PAD_INT_GPIO_NAME);
    g_microp_detect_gpio = get_gpio_by_name(PAD_PLUGIN_GPIO_NAME);
    rf_switch_gpio = get_gpio_by_name(RF_SWITCH_GPIO_NAME);
    if ( g_microp_irq_gpio >=0) {
//        ret = gpio_request(g_microp_irq_gpio, client->name);
        ret = AX_request_gpio_33();
        if (ret < 0){
            printk("microP: gpio_request fail gpio=%d!\n", g_microp_irq_gpio);
            goto init_uP_fail;
        }
        ret = gpio_direction_input(g_microp_irq_gpio);
        if (ret < 0){
            printk("microP: gpio_direction_input fail gpio=%d!\n", g_microp_irq_gpio);
            goto init_uP_fail;
        }
        ret = request_irq(gpio_to_irq(g_microp_irq_gpio), microP_irq_handler,
                    IRQF_TRIGGER_FALLING | IRQF_SHARED , client->name, info); // Walf
        if (ret < 0){
            printk("microP: gpio_direction_input fail gpio=%d!\n", g_microp_irq_gpio);
            goto init_uP_fail;
        }
    }

    if ( g_microp_detect_gpio >= 0) {
        ret = gpio_request(g_microp_detect_gpio, client->name);
        if (ret < 0){
            printk("microP: gpio_request fail gpio=%d!\n", g_microp_detect_gpio);
            goto init_uP_fail;
        }
        ret = gpio_direction_input(g_microp_detect_gpio);
        if (ret < 0){
            printk("microP: gpio_direction_input fail gpio=%d!\n", g_microp_detect_gpio);
            goto init_uP_fail;
        }

        ret = request_irq(gpio_to_irq(g_microp_detect_gpio), microP_detect_handler,
                    IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_SHARED, client->name, info);
        if (ret < 0){
            printk("microP: gpio_direction_input fail gpio=%d!\n", g_microp_detect_gpio);
            goto init_uP_fail;
        }
        enable_irq_wake(gpio_to_irq(g_microp_irq_gpio));
        enable_irq_wake(gpio_to_irq(g_microp_detect_gpio));

    } else {
//        goto init_uP_fail;
    }

    if ( rf_switch_gpio >= 0) {
        ret = gpio_request(rf_switch_gpio, client->name);
        if (ret < 0){
            printk("microP: gpio_request fail gpio=%d!\n", rf_switch_gpio);
            goto init_uP_fail;
        }
        ret = gpio_direction_output(rf_switch_gpio,0);
        if (ret < 0){
            printk("microP: gpio_direction_output fail gpio=%d!\n", rf_switch_gpio);
            goto init_uP_fail;
        }
    }


    ret = misc_register(&asus_microp_device);
    if (ret < 0) {
        pr_err("%s: Unable to register misc device!\n", __FUNCTION__);
        goto init_uP_fail;
    }

    ret = device_create_file(&client->dev, &dev_attr_reg);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n",  __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_pad_update_progress);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }
    
	/*ret = device_create_file(&client->dev, &dev_attr_pad_proximity_sensor_status);
	if (ret < 0) {
		pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
	}*/

    ret = device_create_file(&client->dev, &dev_attr_pad_ReportI2CFail);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_isn);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_oem_ssn);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_gpio_36);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }
	
    ret = device_create_file(&client->dev, &dev_attr_pad_action);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_led_test);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_notify_test);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    ret = device_create_file(&client->dev, &dev_attr_hs_5v);
    if (ret < 0) {
        pr_err("%s: Unable to create file! %d\n", __FUNCTION__,ret);
    }

    //Eric +++ add file note "sys/class/switch/pfs_pad_ec"	
    pfs_switch_version.name="pfs_pad_ec";
    pfs_switch_version.print_name=pfs_switch_name;
    pfs_switch_version.print_state=p01_switch_state;
    ret=switch_dev_register(&pfs_switch_version);

    if (ret < 0){
        pr_err("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);
    }
    //Eric ---
	
    // registered as gpio switch device
    p01_switch_dev.name="p01";
    p01_switch_dev.print_state=p01_switch_state;
    p01_switch_dev.print_name=p01_switch_name;
    ret=switch_dev_register(&p01_switch_dev);

    if (ret < 0){
        pr_err("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);
    }

    p01_switch_usb_cable.name="usbcable";
    p01_switch_usb_cable.print_state=p01_usb_switch_state;
    p01_switch_usb_cable.print_name=p01_usb_switch_name;
    ret=switch_dev_register(&p01_switch_usb_cable);
    if (ret < 0){
        pr_err("%s: Unable to register switch dev! %d\n", __FUNCTION__,ret);
    }

#ifdef CONFIG_HAS_EARLYSUSPEND
    info->es.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 1;
    info->es.suspend = microp_early_suspend;
    info->es.resume = microp_late_resume;
    register_early_suspend(&info->es);
#endif

    pad_err_notify.name="pad_notify";
    pad_err_notify.print_state=pad_notify_switch_state;

    ret=switch_dev_register(&pad_err_notify);        
    if (ret < 0){
        pr_err("%s: Unable to register switch dev! %d\n",  __FUNCTION__,ret);
    }

	// wait to implement, now, default set to phone mode while initial
	//strcpy(rf_switch_status, "phone");
    set_antenna_to_phone();

#ifdef CONFIG_DEBUG_FS
    debugfs_create_file("microp", S_IRUGO, NULL, NULL, &debug_fops);
#endif    

//	queue_delayed_work(microp_slow_job_wq, &g_uP_info->initCarkit,1*HZ);
	//pr_info("microP: addr= %x, pin=%d\r\n", g_slave_addr,  microp_pdata->intr_gpio);
    pr_info("MicroP: addr= %x, int pin=%d, detect pin=%d\r\n", g_slave_addr,  g_microp_irq_gpio, g_microp_detect_gpio); // Walf
//    pr_info("microP: irq1= %x, irq2=%d, irq=%d\r\n", info->i2c_client->irq,  gpio_to_irq(g_microp_irq_gpio), gpio_to_irq(g_microp_detect_gpio)); // Walf
    pr_info("MicroP: %s ---\n", __FUNCTION__);

    checkPadExist(1);
#if 0
    printk("microp_paddetect in probe\n");

    pad_detect_gpio = gpio_get_value(g_microp_detect_gpio);
    printk("MicroP: microp_paddetect GPIO=%d\n", pad_detect_gpio);
    if(pad_detect_gpio == 0)
    {
        if(st_DISCONNECTED==g_b_isP01Connected){
            //msleep(400);
            uP_i2c_read_reg(MICROP_POWER_ON_REASON, &reg_powerOnReason);
            printk("reg_powerOnReason:%d\n",reg_powerOnReason);
            if(reg_powerOnReason > 0){
                printk("MicroP PowerOn reason=%d\r\n", reg_powerOnReason);
                notify_microp_insert();
            }
        }
    }
#endif
    return 0;
	
init_uP_fail:
	return ret;
}

static int microP_remove(struct i2c_client *client)
{
    struct microP_info *info = i2c_get_clientdata(client);
    g_uP_info=NULL;
    wake_lock_destroy(&interrupt_lock_t);
    switch_dev_unregister(&p01_switch_dev);
    misc_deregister(&asus_microp_device);
    kfree(info);
    return 0;
}

static int microP_shutdown(struct i2c_client *client)
{
    uint32_t state=0 ;
    uint16_t value=0x0069;
    int rc;
    printk("%s ++\r\n",__FUNCTION__);
    if(AX_MicroP_IsP01Connected() && pad_exist()){
        uP_i2c_read_reg(MICROP_OPERATING_STATE, &state);
        printk("MicroP:cur_state:%d\n",state);
        if(state == st_MICROP_Sleep){
            rc = uP_i2c_write_reg(MICROP_IND_PHONE_RESUME, &value);
        }
    }
}
static const struct i2c_device_id microP_id[] = {
    { MICROP_NAME, 0 },
    { }
};

static struct of_device_id microp_i2c_table[] = {
    { .compatible = "misc,microp",},
    { },
};

static struct i2c_driver microp_i2c_driver = {
    .driver = {
        .name   = MICROP_NAME,
        .owner  = THIS_MODULE,
        .of_match_table = microp_i2c_table,
    },
    .probe      = microP_probe,
    .remove     = microP_remove,
    .suspend    = microp_suspend,
    .resume     = microp_resume,
    .shutdown   = microP_shutdown,
    .id_table   = microP_id,
};

static int __init microP_init(void)
{
    printk("microP_init\n");
    return i2c_add_driver(&microp_i2c_driver);
}

static void __exit microP_exit(void)
{
    i2c_del_driver(&microp_i2c_driver);
}
 
module_init(microP_init);
module_exit(microP_exit);

MODULE_AUTHOR("ASUS");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("CAPELLA mics proximity sensor with ALS");
 
