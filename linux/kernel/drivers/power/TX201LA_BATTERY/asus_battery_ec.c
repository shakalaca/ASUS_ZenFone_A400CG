#include <linux/module.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/wakelock.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/usb/penwell_otg.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/debugfs.h>
#include <linux/proc_fs.h>
#include <linux/HWVersion.h>
#include <linux/switch.h>
#include "smb345_charger.h"
#include "asus_battery_ec.h"
#include "bq27520_battery_core.h"
#include "bq27520_battery_upt_i2c.h"
#include "util.h"
//#include <linux/earlysuspend.h>
#define DRIVER_NAME "asus_battery_ec"
#define EC_FAIL_MAX_COUNT               3
#define FIRST_RUN_TIME                  3
#define HIGH_POLLING_TIME               60
#define NORMAL_POLLING_TIME		30
#define LOW_POLLING_TIME		10
#define CRITICAL_POLLING_TIME   	5
#define READ_FAIL_POLLING_TIME		60
#define CHARGER_INOK_NAME "CHG_INOK#"
#define BATT_LOW_NAME "P_BAT_LBO"
#define RETRY_COUNT 3
extern int Read_HW_ID(void);
extern int entry_mode;
extern int ite8566_read_pad_battery_info(int *bat_present, int *bat_status, int *bat_temp, int *bat_vol, int *bat_current, int *bat_capacity, int *bat_energy);
extern int ite8566_read_dock_battery_info(int *bat_present, int *bat_status, int *bat_temp, int *bat_vol, int *bat_current, int *bat_capacity, int *bat_energy);
extern int register_dock_attach_notifier(struct notifier_block *nb);
extern int unregister_dock_attach_notifier(struct notifier_block *nb);
extern int register_dock_detach_notifier(struct notifier_block *nb);
extern int unregister_dock_detach_notifier(struct notifier_block *nb);
extern int ite_read_chargeric_reg(u8 reg_lsb, u8 reg_msb, u8 data_lsb, u8 data_msb);
extern int ite_write_chargeric_reg(u8 reg_lsb, u8 reg_msb, u8 data_lsb, u8 data_msb);
extern int ite_gauge_stop_polling(void);
extern int ite_gauge_start_polling(void);
extern int ite_read_base_charger_status_info(unsigned char *return_buf);
extern int ite_ram_gauge_compare_result(unsigned char *buf);
extern int ite_ram_gauge_temp_control(int enable);

static int asus_battery_driver_ready = 0;
struct workqueue_struct *battery_work_queue;
struct delayed_work battery_poll_data_work;                     //polling data
struct delayed_work base_ac_data_work;
static DEFINE_MUTEX(battery_mutex);
static int cable_status = CABLE_OUT;
static int pre_cable_status = CABLE_OUT;
static int base_ac_status = 0;
static int pre_base_ac_status = 0;
static char dc_charging = false;
static char base_in = false;
static u8 data_lsb,data_msb;
static u8 recive_charger_data;
static u8 write_gauge_result;
static u8 recive_gauge_status;
/* wake lock to prevent S3 during charging */
struct wake_lock wakelock;
struct wake_lock wakelock_t;    // for wake_lokc_timout() useage
static char gbuffer[64];
static char gaugekey_buf[64];

static int asus_dock_notify(struct notifier_block *this,unsigned long code, void *data);
static int asus_pad_batt_get_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val);
static int asus_pad_batt_get_ac_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val);
static int asus_base_batt_get_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val);
static void asus_update_all(int delay);

typedef enum {
    BATT_LG = 0,
} batt_manufacture_t;

typedef enum {
    SYSTEM_NONE = 0, //base detach
    SYSTEM_ANDROID,
    SYSTEM_WINDOWS
} base_attach_type_t; 

typedef enum {
        PAD_BATTERY = 0,
        PAD_AC,
        PAD_USB,
	BASE_BATTERY,
        BASE_AC
} pad_type_t;

static enum power_supply_property asus_pad_online_props[] = {
    POWER_SUPPLY_PROP_ONLINE,
};

static enum power_supply_property asus_pad_batt_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,
    POWER_SUPPLY_PROP_TEMP,
    POWER_SUPPLY_PROP_FIRMWARE_VERSION,
    POWER_SUPPLY_PROP_CHEMICAL_ID,
    POWER_SUPPLY_PROP_FW_CFG_VER,
    POWER_SUPPLY_PROP_BATTERY_ID,
};

static enum power_supply_property asus_base_batt_props[] = {
    POWER_SUPPLY_PROP_STATUS,
    POWER_SUPPLY_PROP_HEALTH,
    POWER_SUPPLY_PROP_PRESENT,
    POWER_SUPPLY_PROP_TECHNOLOGY,
    POWER_SUPPLY_PROP_VOLTAGE_NOW,
    POWER_SUPPLY_PROP_CURRENT_NOW,
    POWER_SUPPLY_PROP_CAPACITY,
    POWER_SUPPLY_PROP_CAPACITY_LEVEL,
    POWER_SUPPLY_PROP_TEMP,
};


static char *asus_pad_batt_online_supplied_to[] = {
        "battery",
//        "ac",
//        "usb",
};

static char *asus_base_power_supplied_to[] = {
        "battery",
        "dock_bat",
};

static struct notifier_block attach_dock_notifier = {
        .notifier_call =    asus_dock_notify,
};

static struct notifier_block detach_dock_notifier = {
        .notifier_call =    asus_dock_notify,
};

struct asus_pad_device_info {

//        struct timer_list dock_charging_to_pad_detect_timer;
//        struct wake_lock wake_lock;
        struct switch_dev batt_dev;
        wait_queue_head_t  charger_status_event;
        wait_queue_head_t  gaugefw_status_event;
//        struct early_suspend battery_early_suspend;
        struct mutex lock;
        struct dentry *charger_dentry;
//        bool dock_charging_to_pad_interrupt_enabled;
        bool low_battery;
        //EC read ++
        int ec_bat_voltage;
        int ec_bat_current;
        int ec_bat_capacity;    //percentage
        int ec_bat_status;
        int ec_bat_temp;
        int ec_bat_remaining_capacity;
	int ec_gauge_status;

        int base_ec_bat_voltage;
        int base_ec_bat_current;
        int base_ec_bat_capacity;    //percentage
        int base_ec_bat_status;
        int base_ec_bat_temp;
        int base_ec_bat_remaining_capacity;
        int base_ec_gauge_status;

        //EC read --
        //return to system ++
        int bat_status;
        int bat_temperature;
        int bat_percentage;     //capacity
        int bat_capacity_level;
        int bat_present;
        int bat_voltage;
        int bat_current;
        //return to system --
        //storage old data ++
        int base_bat_status;
        int base_bat_temperature;
        int base_bat_percentage;         //capacity
	int base_bat_capacity_level;
        int base_bat_voltage;
        int base_bat_current;
	int base_bat_present;
        //storage old data --
        int ec_suspend_status;
	int inok_gpio;
	int batt_low_gpio;
        int inok_irq;
        int batt_low_irq;
        int ec_fail_count;
        int base_fail_count;
        int fw_cfg_ver;
        int df_ver;
        int chem_id;
        int manufacture;
#ifdef TX201LA_ENG_BUILD
        int limit_charger_disable;
        int charger_disabled;
#endif
}; //Joe *pad_device;

static struct asus_pad_device_info *asus_pad_device;


static struct power_supply asus_pad_batt_supplies[] = {
        {
                .name = "battery",
                .type = POWER_SUPPLY_TYPE_BATTERY,
                .properties = asus_pad_batt_props,
                .num_properties = ARRAY_SIZE(asus_pad_batt_props),
                .get_property = asus_pad_batt_get_property,
        },
        {
                .name = "ac",
                .type = POWER_SUPPLY_TYPE_MAINS,
                .supplied_to = asus_pad_batt_online_supplied_to,
                .num_supplicants = ARRAY_SIZE(asus_pad_batt_online_supplied_to),
                .properties = asus_pad_online_props,
                .num_properties = ARRAY_SIZE(asus_pad_online_props),
                .get_property = asus_pad_batt_get_ac_property,
        },
        {
                .name = "usb",
                .type = POWER_SUPPLY_TYPE_USB,
                .supplied_to = asus_pad_batt_online_supplied_to,
                .num_supplicants = ARRAY_SIZE(asus_pad_batt_online_supplied_to),
                .properties = asus_pad_online_props,
                .num_properties = ARRAY_SIZE(asus_pad_online_props),
                .get_property = asus_pad_batt_get_ac_property,
        },
        {
                .name           = "dock_bat",
                .type           = POWER_SUPPLY_TYPE_BASE_BATTERY,
                .properties     = asus_base_batt_props,
                .num_properties = ARRAY_SIZE(asus_base_batt_props),
                .get_property   = asus_base_batt_get_property,
        },
        {
                .name = "dock_ac",
		.supplied_to = asus_base_power_supplied_to,
		.type = POWER_SUPPLY_TYPE_BASE_AC,
                .properties = asus_pad_online_props,
                .num_properties = ARRAY_SIZE(asus_pad_online_props),
                .get_property = asus_pad_batt_get_ac_property,
        },
};

int batt_gaguefw_write(u8 *buf) {
    int ret;
    recive_gauge_status = 0;
    write_gauge_result = 0;
    ret = ite_ram_gauge_compare_result(buf);
    if (ret == 0) {
        wait_event_interruptible_timeout(asus_pad_device->gaugefw_status_event,write_gauge_result,5*HZ);
        if (write_gauge_result & BIT(1) && recive_gauge_status == 1)
            ret = 0;
        else
            ret = -1;
    } else {
        dbg_e("%s write fail,err = %d",__func__,ret);
    }
    return ret;
}

void batt_gauge_fw_result_cb(u8 data)
{
   dbg_d("enter %s:\n",__func__);
   write_gauge_result = data;
   recive_gauge_status = 1;
   wake_up(&asus_pad_device->gaugefw_status_event);
   return;
}
EXPORT_SYMBOL(batt_gauge_fw_result_cb);

void asus_pad_batt_set_charger_data(u8 lsb, u8 msb) {
    dbg_d("enter %s:\n",__func__);
    if (asus_battery_driver_ready ==1) {
        data_msb = msb;
        data_lsb = lsb;
        recive_charger_data = 1;
        wake_up(&asus_pad_device->charger_status_event);
    }
}
EXPORT_SYMBOL(asus_pad_batt_set_charger_data);

int asus_pad_batt_write_charger_reg(u8 reg_lsb,u8 lsb) {
    int ret = 0;
    recive_charger_data = 0;
    ite_write_chargeric_reg(reg_lsb, 0 , lsb, 0);
    wait_event_interruptible_timeout(asus_pad_device->charger_status_event,recive_charger_data,1*HZ);
    dbg_d("msb:0x%x,lsb:0x%x,recived:%d for write reg 0x%x\n",data_msb,data_lsb,recive_charger_data,reg_lsb);
    if ( (data_msb== 0xFF && data_lsb == 0xFF) || recive_charger_data ==0)
        ret = -1;
    return ret;
}

int asus_pad_batt_read_charger_reg(u8 reg) {
    int ret = 0;
    recive_charger_data = 0;
    ite_read_chargeric_reg(reg,0,0,0);
    wait_event_interruptible_timeout(asus_pad_device->charger_status_event,recive_charger_data,1*HZ);
    dbg_d("msb:0x%x,lsb:0x%x,recived:%d for read reg 0x%x\n",data_msb,data_lsb,recive_charger_data,reg);
    if ( (data_msb== 0xFF && data_lsb == 0xFF) || recive_charger_data ==0) {
        err("can not read reg:0x%x\n",reg);
        ret = -1;
    } else {
        ret = data_lsb;
    }
    return ret;
}

/* Acquire the UUID */
static ssize_t get_updateks(struct device *dev, struct device_attribute *attr, char *buf)
{
    generate_key(gaugekey_buf);
    return sprintf(buf, "%s\n", gaugekey_buf);
}

static DEVICE_ATTR(updateks, S_IRUGO, get_updateks, NULL);

/* Acquire the UUID */
static ssize_t get_charge_keys(struct device *dev, struct device_attribute *attr, char *buf)
{
        generate_key(gbuffer);
        return sprintf(buf, "%s\n", gbuffer);
}
static DEVICE_ATTR(charge_keys, S_IRUGO, get_charge_keys, NULL);

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev, struct device_attribute *attr, char *buf)
{
        int ret;
        if ( asus_pad_device->bat_status== POWER_SUPPLY_STATUS_CHARGING ||
		asus_pad_device->bat_status == POWER_SUPPLY_STATUS_FULL)
                ret = 1;
        else
                ret = 0;
        return sprintf(buf, "%d\n", ret);
}

static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
#ifdef TX201LA_ENG_BUILD
static ssize_t set_charger_disable(struct device *dev, struct device_attribute *attr,
                                     const char *buf,size_t count)
{
    int value;
    dbg_d("Enter %s\n", __func__);

    if (kstrtoul(buf, 0, &value))
        return -EINVAL;

    asus_pad_device->limit_charger_disable = value;
    asus_update_all(0.1);

    return count;
}

static DEVICE_ATTR(charger_disable, S_IWUSR, NULL, set_charger_disable);

static ssize_t set_gauge_temp_ctrl(struct device *dev, struct device_attribute *attr,
                                     const char *buf,size_t count)
{
    int value;
    dbg_d("Enter %s\n", __func__);

    if (kstrtoul(buf, 0, &value))
        return -EINVAL;

    ite_ram_gauge_temp_control(value);
    return count;
}

static DEVICE_ATTR(gauge_temp_enable, S_IWUSR, NULL, set_gauge_temp_ctrl);

#endif
static struct attribute *dev_attrs[] = {
    &dev_attr_charge_status.attr,
    &dev_attr_charge_keys.attr,
    &dev_attr_updateks.attr,
#ifdef TX201LA_ENG_BUILD
    &dev_attr_charger_disable.attr,
    &dev_attr_gauge_temp_enable.attr,
#endif
    NULL,
};
static struct attribute_group dev_attr_grp = {
        .attrs = dev_attrs,
};

static void asus_update_all(int delay) {
    if ( asus_battery_driver_ready ==1 ) {
        cancel_delayed_work(&battery_poll_data_work);
        queue_delayed_work(battery_work_queue,&battery_poll_data_work, 0.3*HZ);
    }
}

void pad_bat_set_base_ac_status(int ac_status) {
    if (base_in) {
        dbg_i("ac_status=%d\n",ac_status);
        pre_base_ac_status = base_ac_status;
        base_ac_status = ac_status & BIT(0);
        dc_charging = (ac_status & BIT(3)) ?  true:false;
    } else {
        dbg_e("base is not attach,ingore the base ac event\n");
    }
    if ( asus_battery_driver_ready == 1 ) {
        wake_lock_timeout(&wakelock_t, HZ);
        asus_update_all(0.1);
    }
    return;
}
EXPORT_SYMBOL(pad_bat_set_base_ac_status);

void asus_pad_battery_set_cable_status(int usb_state) {
    dbg_i("Get usb cable status:%d\n",usb_state);
    if (base_in) {
        dbg_i("base attach,ingore the cable status\n");
        return;
    }
    pre_cable_status = cable_status;
    cable_status = usb_state;
    if ( asus_battery_driver_ready == 1 ) {
        /* prevent system from entering s3 in COS while AC charger is connected */
        if (entry_mode == 4) {
            if (cable_status == AC_IN) {
                if (!wake_lock_active(&wakelock)) {
                    dbg_i(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
            else if (cable_status == CABLE_OUT) {
                if (wake_lock_active(&wakelock)) {
                    dbg_i(" %s: asus_battery_power_wakelock -> wake unlock\n", __func__);
                    wake_lock_timeout(&wakelock_t, 3*HZ);  // timeout value as same as the <charger.exe>\asus_global.h #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
                    wake_unlock(&wakelock);
                } else { // for PC case
                    wake_lock_timeout(&wakelock_t, 3*HZ);
                }
            }
        }
        asus_update_all(0.1);
    }
}

EXPORT_SYMBOL(asus_pad_battery_set_cable_status);

static int asus_dock_notify(struct notifier_block *this,
                            unsigned long owner, void *data)
{
    int bat_ret=0;
    unsigned char base_charger_status;

    dbg_i("BATT:base battery notify:%ld\n",owner);
    if (owner == SYSTEM_NONE) { // no base plug-in
        base_in = false;
        pre_base_ac_status = base_ac_status;
        base_ac_status = false;
        dc_charging = false;
        asus_update_all(0.3);
    } else {
        base_in = true;
        if ( asus_battery_driver_ready ==1 ) {
            queue_delayed_work(battery_work_queue,&base_ac_data_work, 1*HZ);
        }
    } 
    return NOTIFY_OK;
}

static int asus_pad_battery_get_capacity(int ec_percentage)
{
    static pre_capacity = -1;
    int temp_capacity = 0;

    //pad spec says that this can be >100 % even if max value is 100 %
    temp_capacity = ((ec_percentage >= 100) ? 100 : ec_percentage);

    /* start: for mapping %99 to 100%. Lose 84%*/
    if(temp_capacity==99)
        temp_capacity=100;
    else if(temp_capacity >=84 && temp_capacity <=98)
        temp_capacity++;
    /* for mapping %99 to 100% */
    /* lose 26% 47% 58%,69%,79% */
    else if(temp_capacity >70 && temp_capacity <80)
        temp_capacity-=1;
    else if(temp_capacity >60&& temp_capacity <=70)
        temp_capacity-=2;
    else if(temp_capacity >50&& temp_capacity <=60)
        temp_capacity-=3;
    else if(temp_capacity >30&& temp_capacity <=50)
        temp_capacity-=4;
    else if(temp_capacity >=0&& temp_capacity <=30)
        temp_capacity-=5;

    /*Re-check capacity to avoid  that temp_capacity <0*/
    temp_capacity = ((temp_capacity <0) ? 0 : temp_capacity);

    if (pre_capacity <0)
         pre_capacity= temp_capacity;
    else if( (cable_status == CABLE_OUT) && (pre_cable_status == CABLE_OUT) &&
             (pre_base_ac_status == false) && (base_ac_status ==false) &&
             (pre_capacity < temp_capacity) && dc_charging == false) {
        dbg_i("keep the battery percentage = (pre=%d,now=%d)\n",pre_capacity,temp_capacity);
        temp_capacity = pre_capacity;
    } else {
        /* FIX: suddently battery percentage drop while
           it is nearly battery low (TT259005).
           We adopt the Dichotomy method to report the percentage smoothly
        */
        if (temp_capacity < 4 && pre_capacity > 5) {
            dbg_i("modify dropping percentage = (now:%d, mod:%d)\n",temp_capacity,(temp_capacity+pre_capacity)/2);
            temp_capacity = (temp_capacity + pre_capacity) / 2;
        }
#ifndef TX201LA_USER_BUILD
        /* Charger IC registers dump */
        if (pre_capacity != 0 && pre_capacity != 5)
            if (temp_capacity == 0 || temp_capacity == 5)
                smb345_dump_registers(NULL);
#endif
        pre_capacity = temp_capacity;
    }
    pre_base_ac_status = base_ac_status;
    pre_cable_status = cable_status;
    return temp_capacity;
}

static int asus_pad_battery_get_status(int Percentage, int temperature,char isbasebat) {
    int status;
    int ret;

    if (temperature > 600) {
        status = POWER_SUPPLY_STATUS_DISCHARGING;
        dbg_e("temperature higher than 60. system should be shutdown\n");
#ifdef TX201LA_ENG_BUILD
    } else if (asus_pad_device->charger_disabled == 1) {
        status = POWER_SUPPLY_STATUS_DISCHARGING;
#endif
    } else if(Percentage==100) {
        status = POWER_SUPPLY_STATUS_FULL;
        dbg_i("Battery Status Full = 0x%x \n",status);
    } else if(cable_status != CABLE_OUT || base_ac_status == true) {
        status = POWER_SUPPLY_STATUS_CHARGING;
        dbg_i("Battery Status Charging by ac charging\n");
    } else if (dc_charging == true && isbasebat == false) {
        status = POWER_SUPPLY_STATUS_CHARGING;
        dbg_i("Battery Status Charging by base dc mode\n");
    } else {
        status = POWER_SUPPLY_STATUS_NOT_CHARGING;
        dbg_i("Battery Status not Charging = 0x%x \n",status);
    }

    return status;
}

static int asus_pad_battery_get_capacity_level(int Percentage)
{
    //Enter the percentage value is adjusted after.
    if (Percentage == 100)
    {
        dbg_i("Level = POWER_SUPPLY_CAPACITY_LEVEL_FULL \n");
        return POWER_SUPPLY_CAPACITY_LEVEL_FULL;
    }
    else if((100 > Percentage)&&(Percentage >= 80))
    {
        dbg_i("Level = POWER_SUPPLY_CAPACITY_LEVEL_HIGH \n");
        return POWER_SUPPLY_CAPACITY_LEVEL_HIGH;
    }
    else if((80 > Percentage)&&(Percentage >= 15))
    {
        dbg_i("Level = POWER_SUPPLY_CAPACITY_LEVEL_NORMAL \n");
        return POWER_SUPPLY_CAPACITY_LEVEL_NORMAL;
    }
    else if((15 > Percentage)&&(Percentage >= 5))
    {
        dbg_i("Level = POWER_SUPPLY_CAPACITY_LEVEL_LOW \n");
        return POWER_SUPPLY_CAPACITY_LEVEL_LOW;
    }
    else if((5 > Percentage)&&(Percentage >= 0))
    {
        dbg_i("Level = POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL \n");
        return POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL;
    }
    else
    {
        dbg_i("Level = POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN \n");
        return POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN;
    }
}


static int asus_pad_battery_get_temperature(int ec_temperature)
{
        int temperature;

        //temp = Kelvin temperature , here transform to Celsius temperature
        temperature = (ec_temperature - 2731);

        return temperature;
}

#ifdef TX201LA_ENG_BUILD
static void asus_pad_batt_set_limit_charger()
{
    int ret;
    if (cable_status != CABLE_OUT) {
        if ( asus_pad_device->charger_disabled == 0 && 
             asus_pad_device->ec_bat_capacity > 60 &&
             asus_pad_device->limit_charger_disable == 1) 
        {
            ret = ite_ram_gauge_temp_control(false);
            if (ret == -1) {
                dbg_e("disable gauge temp control fail.\n");
            }

            ret = smb345_charger_toggle(false);// disable charger
            if (ret == -1) {
                dbg_e("disable charging fail.\n");
            }
            else {
                asus_pad_device->charger_disabled = 1;
                asus_pad_device->bat_status = POWER_SUPPLY_STATUS_DISCHARGING;
                dbg_i("percentage > 60. stop charging\n");
            }
        } else if ( (asus_pad_device->charger_disabled == 1 && asus_pad_device->ec_bat_capacity < 60) ||
                    (asus_pad_device->limit_charger_disable == 0 && asus_pad_device->charger_disabled == 1)) 
        {
            ret = smb345_charger_toggle(true); // enable charger
            if (ret == -1) {
                dbg_e("enable charging fail.\n");
            }
            else {
                asus_pad_device->charger_disabled = 0;
                asus_pad_device->bat_status = POWER_SUPPLY_STATUS_CHARGING;
                dbg_i("percentage < 60. re-charging now\n");
            }
            ret = ite_ram_gauge_temp_control(true);
            if (ret == -1) {
                dbg_e("enable gauge temp control fail.\n");
            }
        } 
    } else { // usb remove,need to reset the charger stop charger
        asus_pad_device->charger_disabled = 0;
    }
}
#endif

static ret_type_t asus_pad_battery_get_info(void)
{
    ret_type_t ret=RET_EC_OK;
    int HW_ID = Read_HW_ID();
 
    mutex_lock(&battery_mutex);
    ret = ite8566_read_pad_battery_info(&asus_pad_device->ec_gauge_status,
             &asus_pad_device->ec_bat_status,&asus_pad_device->ec_bat_temp,
             &asus_pad_device->ec_bat_voltage,&asus_pad_device->ec_bat_current,
             &asus_pad_device->ec_bat_capacity,&asus_pad_device->ec_bat_remaining_capacity);
    if (ret == RET_EC_UPDATE) {
        dbg_i("EC firmware updating..can not read battery status now");
	mutex_unlock(&battery_mutex);
	return RET_EC_UPDATE;
    }
    if (ret == RET_EC_FAIL) {
        dbg_e("EC can not read battery status,retry cnt:%d\n",asus_pad_device->ec_fail_count);
        asus_pad_device->ec_fail_count++;
        if ( asus_pad_device->ec_fail_count > EC_FAIL_MAX_COUNT) {
            asus_pad_device->bat_percentage = -99;
            asus_pad_device->bat_current = -9999;
            asus_pad_device->bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
        }
    } else if (asus_pad_device->ec_gauge_status == 0) {
        dbg_e("read gauge ic fail\n");
        asus_pad_device->bat_percentage = -98;
        asus_pad_device->bat_current = -9999;
        asus_pad_device->bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
        ret = RET_GAUGE_FAIL;
    // error handle for ec read fail. we do not know the real reason.
    } else if (asus_pad_device->ec_gauge_status != 1) {
        dbg_e("read gauge status value not normal:%d,retry cnt:%d\n",
                   asus_pad_device->ec_gauge_status,asus_pad_device->ec_fail_count);
        asus_pad_device->ec_fail_count++;
        if ( asus_pad_device->ec_fail_count > EC_FAIL_MAX_COUNT) {
            asus_pad_device->bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
        }
        ret = RET_GAUGE_FAIL;
    } else {
        asus_pad_device->ec_fail_count = 0;
        asus_pad_device->bat_voltage = asus_pad_device->ec_bat_voltage*1000;
        asus_pad_device->bat_temperature = asus_pad_battery_get_temperature(asus_pad_device->ec_bat_temp);
        if ( HW_ID == HW_ID_SR1 || HW_ID == HW_ID_SR2) {
            dbg_i("sr board,read percentage from gauge now\n");
            ite_gauge_stop_polling();
            bq27541_read_percentage(&asus_pad_device->ec_bat_capacity);
            ite_gauge_start_polling();
            dbg_i("percentage read from gauge:%d\n",asus_pad_device->ec_bat_capacity);
            asus_pad_device->bat_temperature = 300; // force set avoid system shutdown.
        } else if ( asus_pad_device->bat_temperature < -200) {
            dbg_i("the temp < -200 should be sr battery.force set percentage as 50\n");
            asus_pad_device->ec_bat_capacity = 50;
        }

        if (asus_pad_device->low_battery == true)
            asus_pad_device->bat_percentage = asus_pad_battery_get_capacity(0);
        else
            asus_pad_device->bat_percentage = asus_pad_battery_get_capacity(asus_pad_device->ec_bat_capacity);
        asus_pad_device->bat_capacity_level = asus_pad_battery_get_capacity_level(asus_pad_device->bat_percentage);
        asus_pad_device->bat_status = asus_pad_battery_get_status(asus_pad_device->bat_percentage,asus_pad_device->bat_temperature,false);
#ifdef TX201LA_ENG_BUILD
        asus_pad_batt_set_limit_charger();
#endif
        asus_pad_device->bat_current = asus_pad_device->ec_bat_current*1000;
        dbg_i("gauge status:%d,status:%d,temp:%d,volt:%d,current:%d,percentage:%d,remian:%d\n",
            asus_pad_device->ec_gauge_status,asus_pad_device->bat_status,
            asus_pad_device->bat_temperature,asus_pad_device->bat_voltage,
            asus_pad_device->bat_current,asus_pad_device->bat_percentage,
            asus_pad_device->ec_bat_remaining_capacity);
    }
    if (base_in) {
        ret = ite8566_read_dock_battery_info(&asus_pad_device->base_ec_gauge_status,
	          &asus_pad_device->base_ec_bat_status,&asus_pad_device->base_ec_bat_temp,
                  &asus_pad_device->base_ec_bat_voltage,&asus_pad_device->base_ec_bat_current,
                  &asus_pad_device->base_ec_bat_capacity,&asus_pad_device->base_ec_bat_remaining_capacity);
        if (ret == RET_EC_UPDATE) {
            dbg_i("EC firmware updating..can not read battery status now");
        } else if (ret == RET_EC_FAIL) {
            dbg_e("Base EC can not read battery status\n");
            asus_pad_device->base_bat_percentage = -99;
            asus_pad_device->base_bat_current = -9999;
            asus_pad_device->base_bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
        } else if (asus_pad_device->base_ec_gauge_status == 0) {
            dbg_e("read base gauge ic fail\n");
            asus_pad_device->base_fail_count++;
            if (asus_pad_device->base_fail_count > EC_FAIL_MAX_COUNT){
                asus_pad_device->base_bat_percentage = -98;
                asus_pad_device->base_bat_current = -9999;
                asus_pad_device->base_bat_status = POWER_SUPPLY_STATUS_UNKNOWN;
            }
            ret = RET_GAUGE_FAIL;
        } else {
            asus_pad_device->base_fail_count = 0;
            asus_pad_device->base_bat_voltage = asus_pad_device->base_ec_bat_voltage*1000;
            asus_pad_device->base_bat_temperature = asus_pad_battery_get_temperature(asus_pad_device->base_ec_bat_temp);
            asus_pad_device->base_bat_percentage = asus_pad_device->base_ec_bat_capacity;
            if (asus_pad_device->base_bat_percentage > 100)
                asus_pad_device->base_bat_percentage = 100;
            asus_pad_device->base_bat_capacity_level = asus_pad_battery_get_capacity_level(asus_pad_device->base_bat_percentage);
            asus_pad_device->base_bat_status = asus_pad_battery_get_status(asus_pad_device->base_bat_percentage,0,true);
            asus_pad_device->base_bat_current = asus_pad_device->base_ec_bat_current*1000;
            dbg_i("base gauge status:%d,status:%d,temp:%d,volt:%d,current:%d,percentage:%d,remian:%d\n",
                     asus_pad_device->base_ec_gauge_status,asus_pad_device->base_bat_status,
                     asus_pad_device->base_bat_temperature,asus_pad_device->base_bat_voltage,
                     asus_pad_device->base_bat_current,asus_pad_device->base_bat_percentage,
                     asus_pad_device->base_ec_bat_remaining_capacity);
        }
    }
    mutex_unlock(&battery_mutex);
    if(asus_battery_driver_ready) {
        power_supply_changed(&asus_pad_batt_supplies[PAD_BATTERY]);
	power_supply_changed(&asus_pad_batt_supplies[PAD_AC]);
        power_supply_changed(&asus_pad_batt_supplies[PAD_USB]);
//	if (base_in) {
        power_supply_changed(&asus_pad_batt_supplies[BASE_AC]);
        power_supply_changed(&asus_pad_batt_supplies[BASE_BATTERY]);
//        }
    }
    return ret;
}

static void base_ac_poll(struct work_struct *work) {
	int bat_ret;
	unsigned char base_charger_status;
	
    bat_ret = ite_read_base_charger_status_info(&base_charger_status);
    if ( bat_ret == RET_GAUGE_FAIL || bat_ret == RET_EC_FAIL) {
        dbg_e("%s:fail to get base charger info\n",__func__);
    } else {
        base_ac_status = (base_charger_status >> 1) & BIT(0);
        dc_charging = (base_charger_status & BIT(3)) ?  true:false;
        dbg_i("base charger status:%d\n",base_charger_status);
    }
    asus_update_all(0.1);
}

static void asus_pad_battery_status_poll(struct work_struct *work)
{
    ret_type_t ret;

    ret = asus_pad_battery_get_info();
    if(!asus_pad_device->ec_suspend_status) {
        if (asus_pad_device->bat_percentage < 0 || ret == RET_EC_UPDATE) {
            queue_delayed_work(battery_work_queue,&battery_poll_data_work, READ_FAIL_POLLING_TIME*HZ);
        } else if(asus_pad_device->bat_percentage > 50) {
            queue_delayed_work(battery_work_queue,&battery_poll_data_work, HIGH_POLLING_TIME*HZ);
        } else if (asus_pad_device->bat_percentage <= 50 && asus_pad_device->bat_percentage > 20) {
            queue_delayed_work(battery_work_queue,&battery_poll_data_work, NORMAL_POLLING_TIME*HZ);
        } else if (asus_pad_device->bat_percentage <= 20 && asus_pad_device->bat_percentage > 5) {
            queue_delayed_work(battery_work_queue,&battery_poll_data_work, LOW_POLLING_TIME*HZ);
        } else if(asus_pad_device->bat_percentage <= 5) { //if battery status is critical, polling time shorten. 
            queue_delayed_work(battery_work_queue,&battery_poll_data_work, CRITICAL_POLLING_TIME*HZ);
	}
    }
}

static int asus_pad_batt_get_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val)
{
    static const char tmp_buf[10];
    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        val->intval = asus_pad_device->bat_status;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = asus_pad_device->bat_present;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = asus_pad_device->bat_voltage;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = asus_pad_device->bat_current;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = asus_pad_device->bat_percentage;
        break;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = asus_pad_device->bat_capacity_level;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = asus_pad_device->bat_temperature;
        break;
    case POWER_SUPPLY_PROP_FIRMWARE_VERSION:
        sprintf(tmp_buf,"%04x",asus_pad_device->fw_cfg_ver);
        val->strval = tmp_buf;
        break;
    case POWER_SUPPLY_PROP_CHEMICAL_ID:
        sprintf(tmp_buf,"%04x",asus_pad_device->chem_id);
        val->strval = tmp_buf;
        break;
    case POWER_SUPPLY_PROP_FW_CFG_VER:
        sprintf(tmp_buf,"%04x",asus_pad_device->df_ver);
        val->strval = tmp_buf;
        break;
    case POWER_SUPPLY_PROP_BATTERY_ID:
        sprintf(tmp_buf,"%d",asus_pad_device->manufacture);
        val->strval = tmp_buf;
        break;
    default:
        dbg_e("%s: some properties(%d) deliberately report errors.\n",__func__,psp);
        return -EINVAL;
    }
    return 0;
}

static int asus_base_batt_get_property(struct power_supply *psy,
                                           enum power_supply_property psp,
                                           union power_supply_propval *val)
{
    switch (psp) {
    case POWER_SUPPLY_PROP_STATUS:
        if (base_in)
            val->intval = asus_pad_device->base_bat_status;
        else
            val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        break;
    case POWER_SUPPLY_PROP_HEALTH:
        val->intval = POWER_SUPPLY_HEALTH_GOOD;
        break;
    case POWER_SUPPLY_PROP_PRESENT:
        val->intval = base_in;
        break;
    case POWER_SUPPLY_PROP_TECHNOLOGY:
        val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
        break;
    case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        val->intval = asus_pad_device->base_bat_voltage;
        break;
    case POWER_SUPPLY_PROP_CURRENT_NOW:
        val->intval = asus_pad_device->base_bat_current;
        break;
    case POWER_SUPPLY_PROP_CAPACITY:
        val->intval = asus_pad_device->base_bat_percentage;
        break;
    case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
        val->intval = asus_pad_device->base_bat_capacity_level;
        break;
    case POWER_SUPPLY_PROP_TEMP:
        val->intval = asus_pad_device->base_bat_temperature;
        break;
    default:
        dbg_e("%s: some properties deliberately(%d) report errors.\n",__func__,psp);
        return -EINVAL;
    }
    return 0;
}

static int asus_pad_batt_get_ac_property(struct power_supply *psy,
                                      enum power_supply_property psp,
                                      union power_supply_propval *val)
{
    int ret = 0;
    switch (psp) {
    case POWER_SUPPLY_PROP_ONLINE:
        if (psy->type == POWER_SUPPLY_TYPE_USB) {
	    val->intval = (cable_status == USB_IN) ? 1 : 0;
        } else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
            val->intval = (cable_status == AC_IN) ? 1 : 0;
        } else if (psy->type == POWER_SUPPLY_TYPE_BASE_AC) {
            val->intval = (base_ac_status == true ? 1 : 0);
        } else {
            ret = -EINVAL;
        }
        break;
    default:
        ret = -EINVAL;
        break;
    }
    return ret;
}


static irqreturn_t asus_battery_low_interrupt(int irq, void *data) {
    irqreturn_t ret = IRQ_HANDLED;

//    pm_runtime_get_sync(&smb->client->dev);
    dbg_i("%s:BATT_LOW is trigger\n", __func__);
//#ifndef TX201LA_USER_BUILD
    /* Charger IC registers dump */
//    smb345_dump_registers(NULL);
//#endif
//    asus_pad_device->low_battery = true;
//    asus_update_all(0.1);

//    pm_runtime_put_sync(&smb->client->dev);
    return ret;
}

static irqreturn_t asus_battery_inok_interrupt(int irq, void *data) {
    irqreturn_t ret = IRQ_HANDLED;

//    pm_runtime_get_sync(&smb->client->dev);
    if (gpio_get_value(asus_pad_device->inok_gpio)) {
        dbg_i("%s: >>> INOK pin (HIGH) <<<\n", __func__);
    } else {
        dbg_i("%s: >>> INOK pin (LOW) <<<\n", __func__);
    }
    /* wake lock to prevent system instantly
       enter S3 while it's in resuming flow */
    if (asus_battery_driver_ready == 1)
        wake_lock_timeout(&wakelock_t, HZ);
//    pm_runtime_put_sync(&smb->client->dev);
    return ret;
}

static int smb345_debugfs_show(struct seq_file *s, void *data)
{
        smb345_dump_registers(s);

        return 0;
}

static int smb345_debugfs_open(struct inode *inode, struct file *file)
{
        return single_open(file, smb345_debugfs_show, inode->i_private);
}

static bool flag_create_bdge = false;
static int bq27520_proc_bridge_read(char *page, char **start, off_t off, int count, int *eof, void *
data)
{
    int len;

    if (bq27520_is_rom_mode()) {
        len = sprintf(page, "7");
    } else {
        len = sprintf(page, "8");
    }

    return len;
}

static int bq27520_proc_bridge_write(struct file *file, const char *buf, unsigned long count, void *data)
{
    u8 recv_buf[1024];
    int i,iloop=0;
    int unsealedkey,fullkey;
    int ret;
    int cmd_num = 0,cmd_loop = 0;
    u8 buffer_temp[5]={0};
    u8 temp[255]={0};

    if (copy_from_user(recv_buf,buf,count)) {
        return -EFAULT;
    }
    if (recv_buf[0] == 'k') {
        sscanf (recv_buf, "k %x %x",&unsealedkey, &fullkey);
        dbg_d("0x%x,0x%x\n",unsealedkey,fullkey);
        bq27520_to_unsealed(unsealedkey);
        bq27520_to_full(fullkey);
        ret = bq27520_enter_rom_mode();
        if (ret == 0) {
            dbg_i("enter rom mode ok\n");
        } else {
            return -EFAULT;
        }
    } else if (recv_buf[0] == 's') {
        ite_gauge_stop_polling();
    } else if (recv_buf[0] == 'e') {
        ite_gauge_start_polling();
    } else if (recv_buf[0] == 'l') {
        TIgauge_LockStep();
    }
    else {
        sscanf(recv_buf, "%x",&cmd_num);
        dbg_d("recv_buf:%s\n",recv_buf);
//        printk("cmd_num %d\n",cmd_num);

        for(cmd_loop = 0; cmd_loop < cmd_num+1; cmd_loop++){
            buffer_temp[0] = recv_buf[cmd_loop*5 + 0];
            buffer_temp[1] = recv_buf[cmd_loop*5 + 1];
            buffer_temp[2] = recv_buf[cmd_loop*5 + 2];
            buffer_temp[3] = recv_buf[cmd_loop*5 + 3];
            buffer_temp[4] = '\0';
            dbg_d("0x%x 0x%x 0x%x 0x%x\n",recv_buf[cmd_loop*5 + 0],recv_buf[cmd_loop*5 + 1],recv_buf[cmd_loop*5 + 2],recv_buf[cmd_loop*5 + 3]);
            dbg_d("0x%x 0x%x 0x%x 0x%x\n",buffer_temp[0],buffer_temp[ 1],buffer_temp[ 2],buffer_temp[ 3]);
            sscanf(buffer_temp, "%x", &temp[cmd_loop]);
            dbg_d("0x%x\n",temp[cmd_loop]);
        }
        for (i=0;i<RETRY_COUNT; i++) {
            ret = batt_gaguefw_write(temp);
            if (ret == 0) break;
        }
        if (ret < 0)
            return -EFAULT;
    }
    return count;
}

int bq27520_proc_fs_update_bridge(void)
{
    struct proc_dir_entry *entry=NULL;

    /* not create again */
    if (flag_create_bdge)
        return 0;

    entry = create_proc_entry("ubridge", 0666, NULL);
    if (!entry) {
        dbg_e("Unable to create ubridge\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_bridge_read;
    entry->write_proc = bq27520_proc_bridge_write;

    /* lock on the flag */
    flag_create_bdge = true;

    return 0;
}

static int bq27520_proc_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int ret;

    if (!gaugekey_buf)
        return count;

    /* print the key to screen for debugging */
    ret = sprintf(page, "%s\n", gaugekey_buf);

    /* re-generate the key to clean the memory */
    generate_key(gaugekey_buf);

    return ret;
}

static int bq27520_proc_write(struct file *file, const char *buffer, unsigned long count, void *data
)
{
    char proc_buf[64];

    if (count > sizeof(gbuffer)) {
        dbg_e("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buffer, count)) {
        dbg_e("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    if (!memcmp(proc_buf, gaugekey_buf, 36)) {
        dbg_e("%s: EQ\n", __func__);

        /* create update bridge */
        bq27520_proc_fs_update_bridge();
    } else {
        dbg_e("%s: NOT EQ\n", __func__);

        /* re-generate the key to clean memory to avoid
           "brute-force attack". we do not allow someone
           who using "repetitive try-error" to find out
           the correct one.
        */
        generate_key(gaugekey_buf);
    }

    return count;
}

static const struct file_operations smb345_debugfs_fops = {
        .open           = smb345_debugfs_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
};

static int twinshead_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
    char proc_buf[64];

    if (count > sizeof(gbuffer)) {
        dbg_e("%s: data error\n", __func__);
        return -EINVAL;
    }

    if (copy_from_user(proc_buf, buffer, count)) {
        dbg_e("%s: read data from user space error\n", __func__);
        return -EFAULT;
    }

    if (!memcmp(proc_buf, gbuffer, 36)) {
        dbg_i("%s: EQ\n", __func__);

        if (!gpio_get_value(asus_pad_device->inok_gpio)) {

            /* ME372CG charge current control algorithm:
               config the charge current only when
               Vbus is legal (a valid input voltage
               is present)
            */
            /* force max 1.8A charge current for ME372CG */
            smb345_config_max_current_twinheadeddragon(asus_pad_device->inok_gpio);
        }
    }
    else {
        dbg_e("%s: NOT EQ\n", __func__);

        /* re-generate the key to clean memory to avoid
           "brute-force attack". we do not allow someone
           who using "repetitive try-error" to find out
           the correct one.
        */
        generate_key(gbuffer);
    }
    return count;
}

static int bq27520_proc_info_dump_read(struct seq_file *s, void *data)
{
    int tmp_buf;
    static int bq_batt_current = 0;
    static int bq_batt_full_charge_capacity = 0;
    static int bq_batt_nominal_available_capacity = 0;
    static int bq_batt_cc = 0;

    //update battery info from ec
    asus_update_all(0);

    ite_gauge_stop_polling();

    tmp_buf = bq27520_asus_battery_dev_read_full_charge_capacity();
    if (tmp_buf >= 0) bq_batt_full_charge_capacity = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_nominal_available_capacity();
    if (tmp_buf >= 0) bq_batt_nominal_available_capacity = tmp_buf;

    tmp_buf = bq27520_asus_battery_dev_read_cycle_count();
    if (tmp_buf >= 0) bq_batt_cc = tmp_buf;

    ite_gauge_start_polling();

    seq_printf(s,"LMD(mAh): %d\n", bq_batt_full_charge_capacity);
    seq_printf(s,"NAC(mAh): %d\n", bq_batt_nominal_available_capacity);
    seq_printf(s,"RM(mAh): %d\n", asus_pad_device->ec_bat_remaining_capacity);
    seq_printf(s,"RSOC: %d\n", asus_pad_device->bat_percentage);
    seq_printf(s,"voltage(mV): %d\n", asus_pad_device->bat_voltage/1000);
    seq_printf(s,"average_current(mA): %d\n", asus_pad_device->bat_current/1000);
    seq_printf(s,"temp: %d\n", asus_pad_device->bat_temperature);
    seq_printf(s,"chemical_id: 0x%04X\n", asus_pad_device->chem_id);
    seq_printf(s,"fw_version: 0x%04X\n", asus_pad_device->df_ver);
    seq_printf(s,"fw_cfg_version: 0x%04X\n", asus_pad_device->fw_cfg_ver);
    seq_printf(s,"CC(num): %d\n", bq_batt_cc);
//    seq_printf("update_status: %s\n",
//        str_batt_update_status[tmp_dev_info.update_status - UPDATE_PROCESS_FAIL]);

    return 0;
}

static int dumpinfo_proc_open(struct inode *inode, struct file *file)
{
        return single_open(file, bq27520_proc_info_dump_read, inode->i_private);
}

static const struct file_operations dumpinfo_proc_fops = {
        .owner          = THIS_MODULE,
        .open           = dumpinfo_proc_open,
        .read           = seq_read,
        .llseek         = seq_lseek,
        .release        = single_release,
        .write          = NULL,
};

static int batt_remove_proc_entry(void)
{
    remove_proc_entry("twinheadeddragon",NULL);
    remove_proc_entry("twinsheadeddragon",NULL);
    remove_proc_entry("bq27520_test_info_dump",NULL);
} 

static int batt_create_proc_entry(void)
{
    struct proc_dir_entry *entry=NULL;

    entry = create_proc_entry("twinheadeddragon", 0666, NULL);
    if (!entry) {
        dbg_e("Unable to create twinheadeddragon\n");
        return -EINVAL;
    }
    entry->read_proc = NULL;
    entry->write_proc = twinshead_proc_write;

    entry = create_proc_entry("twinsheadeddragon", 0666, NULL);
    if (!entry) {
        dbg_e("Unable to create twinsheadeddragon\n");
        return -EINVAL;
    }
    entry->read_proc = bq27520_proc_read;
    entry->write_proc = bq27520_proc_write;

    entry = proc_create("bq27520_test_info_dump", 0644, NULL,&dumpinfo_proc_fops);
    if (!entry) {
        dbg_e("Unable to create bq27520_test_info_dump\n");
        return -EINVAL;
    }

    return 0;
}

static ssize_t batt_switch_name(struct switch_dev *sdev, char *buf)
{
    /* firmware_version + firmware config version + chemical id + battery id */
    const char* FAIL = "0xFFFF";

    if ( asus_battery_driver_ready ==1 )
        return sprintf(buf, "%04x-%04x-%04x-%d\n",
            asus_pad_device->fw_cfg_ver,
            asus_pad_device->df_ver,
            asus_pad_device->chem_id,
            asus_pad_device->manufacture);

    return sprintf(buf, "%s\n", FAIL);
}

static int __devinit asus_battery_ec_probe(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;
    int irq;
    int i;
    int ret;
    ret_type_t bat_ret;

    dbg_i("%s\n", __func__);

    asus_pad_device = kzalloc(sizeof(struct asus_pad_device_info), GFP_KERNEL);
    if (!asus_pad_device) {
        ret = -ENOMEM;
        goto pad_dev_alloc_fail;
    }
    memset(asus_pad_device, 0, sizeof(*asus_pad_device));

    //Init
    asus_pad_device->bat_present = 1;
    asus_pad_device->bat_status = 3;
    asus_pad_device->bat_percentage = 50;
    asus_pad_device->bat_temperature = 270;
    asus_pad_device->bat_voltage = 8000;
    asus_pad_device->bat_current = 0;
    asus_pad_device->base_bat_status = 3;
    asus_pad_device->ec_suspend_status = 0;
    asus_pad_device->low_battery = false;
    asus_pad_device->manufacture = BATT_LG;
    asus_pad_device->inok_gpio = get_gpio_by_name(CHARGER_INOK_NAME);
    asus_pad_device->batt_low_gpio = get_gpio_by_name(BATT_LOW_NAME);
#ifdef TX201LA_ENG_BUILD
    asus_pad_device->limit_charger_disable = true;
#endif
    init_waitqueue_head(&asus_pad_device->charger_status_event);
    init_waitqueue_head(&asus_pad_device->gaugefw_status_event);

#ifndef TX201LA_USER_BUILD
    /* Do it only in MOS, COS. Not support in other conditions */
    if (entry_mode == 1 || entry_mode == 4) {
        ite_gauge_stop_polling();
        ret = bq27520_bat_upt_main_update_flow();
        ite_gauge_start_polling();
        if (ret < 0 && ret != UPDATE_VOLT_NOT_ENOUGH)
            dbg_e("update firmware fail\n");
        else if (ret == UPDATE_OK) { /* confirm "sealed" to acquire correct firmware config version */
            msleep(500);
            TIgauge_LockStep();
        }
    }
#endif


    /* init wake lock in COS */
    wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock");
    wake_lock_init(&wakelock_t, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock_timeout");

        /* prevent system from entering s3 in COS while AC charger is connected */
    if (entry_mode == 4) {
        if (cable_status == AC_IN) {
            if (!wake_lock_active(&wakelock)) {
                dbg_i(" %s: asus_battery_power_wakelock -> wake lock\n", __func__);
                wake_lock(&wakelock);
            }
        }
    }

    if (asus_pad_device->inok_gpio >=0) {
        ret = gpio_request_one(asus_pad_device->inok_gpio, GPIOF_IN, "CHG_INOK");
        if (ret <0) {
            dbg_e("request INOK gpio fail!\n");
            goto request_inok_gpio_fail;
        }
        asus_pad_device->inok_irq = gpio_to_irq(asus_pad_device->inok_gpio);
        ret = request_threaded_irq(asus_pad_device->inok_irq, asus_battery_inok_interrupt,
                                        asus_battery_inok_interrupt,
                                        IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
					"CHG_INOK",dev);
        if (ret <0) {
            dbg_e("request INOK gpio as irq fail!\n");
            goto request_inok_irq_fail;
        }
    }

    if (asus_pad_device->batt_low_gpio >=0) {
        ret = gpio_request_one(asus_pad_device->batt_low_gpio, GPIOF_IN, BATT_LOW_NAME);
        asus_pad_device->batt_low_irq = gpio_to_irq(asus_pad_device->batt_low_gpio);
        if (ret <0) {
            dbg_e("request BATT_LOW gpio fail!\n");
            goto request_battlow_gpio_fail;
        }
        ret = request_threaded_irq(asus_pad_device->batt_low_irq, asus_battery_low_interrupt,
                                        asus_battery_low_interrupt,
                                        IRQF_TRIGGER_FALLING, BATT_LOW_NAME,dev);
        if (ret <0) {
            dbg_e("request battery low gpio as irq fail!\n");
            goto request_battlow_irq_fail;
        }
    }


    ret = power_supply_register(dev, &asus_pad_batt_supplies[PAD_BATTERY]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[PAD_BATTERY].name);
	goto batt_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[PAD_AC]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[PAD_AC].name);
        goto ac_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[PAD_USB]);
    if (ret) {
        pr_err("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[PAD_USB].name);
        goto usb_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[BASE_BATTERY]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[BASE_BATTERY].name);
        goto base_batt_reg_fail;
    }

    ret = power_supply_register(dev, &asus_pad_batt_supplies[BASE_AC]);
    if (ret) {
        dbg_e("%s: failed to register %s\n", __func__,
                asus_pad_batt_supplies[BASE_AC].name);
        goto base_ac_reg_fail;
    }

//        wake_lock_init(&asus_pad_device->wake_lock, WAKE_LOCK_SUSPEND, "battery_wake_lock");
    bat_ret = asus_pad_battery_get_info();
    if ( bat_ret == RET_GAUGE_FAIL || bat_ret == RET_EC_FAIL) {
//        ret = -EIO;
        dbg_e("%s:fail to get battery info\n",__func__);
//	goto get_batt_info_err;
    }

#ifndef TX201LA_USER_BUILD
    asus_pad_device->charger_dentry = debugfs_create_file("smb345", S_IRUGO, NULL, asus_pad_device,
                                          &smb345_debugfs_fops);
#endif

    register_dock_attach_notifier(&attach_dock_notifier);
    register_dock_detach_notifier(&detach_dock_notifier);
    sysfs_create_group(&dev->kobj, &dev_attr_grp);

// create proc for twinsheadeddragon
    generate_key(gbuffer);
    batt_create_proc_entry();
    
    ite_gauge_stop_polling();
    asus_pad_device->fw_cfg_ver = bq27520_asus_battery_dev_read_fw_cfg_version();
    if ( asus_pad_device->fw_cfg_ver == ERROR_CODE_I2C_FAILURE)
        asus_pad_device->fw_cfg_ver = 0;
    dbg_i("fw_cfg_version:%d\n",asus_pad_device->fw_cfg_ver);
    asus_pad_device->chem_id = bq27520_asus_battery_dev_read_chemical_id();
    if ( asus_pad_device->chem_id == ERROR_CODE_I2C_FAILURE)
        asus_pad_device->chem_id = 0;
    dbg_i("chemical_id=%d\n", asus_pad_device->chem_id);
    asus_pad_device->df_ver = bq27520_asus_battery_dev_read_df();
    if ( asus_pad_device->df_ver == ERROR_CODE_I2C_FAILURE)
        asus_pad_device->df_ver = 0;
    dbg_i("df:%d\n",asus_pad_device->df_ver);
    ite_gauge_start_polling();

    /* switch added for battery version info */
    asus_pad_device->batt_dev.name = "battery";
    asus_pad_device->batt_dev.print_name = batt_switch_name;
    if (switch_dev_register(&asus_pad_device->batt_dev) < 0) {
        dbg_e("fail to register battery switch\n");
    }

/*
    bat_ret = ite_read_base_charger_status_info(&base_charger_status);
    if ( bat_ret == RET_GAUGE_FAIL || bat_ret == RET_EC_FAIL) {
        dbg_e("%s:fail to get base charger info\n",__func__);
    } else {
        base_ac_status = (base_charger_status >> 1) & BIT(0);
        dbg_i("base charger status:%d\n",base_charger_status);
    }
*/
    queue_delayed_work(battery_work_queue, &battery_poll_data_work, FIRST_RUN_TIME*HZ);
    asus_battery_driver_ready = 1;
    return 0;

get_batt_info_err:
    power_supply_unregister(&asus_pad_batt_supplies[BASE_AC]);
base_ac_reg_fail:
    power_supply_unregister(&asus_pad_batt_supplies[BASE_BATTERY]);
base_batt_reg_fail:
    power_supply_unregister(&asus_pad_batt_supplies[PAD_USB]);
usb_reg_fail:
    power_supply_unregister(&asus_pad_batt_supplies[PAD_AC]);
ac_reg_fail:
    power_supply_unregister(&asus_pad_batt_supplies[PAD_BATTERY]);
batt_reg_fail:
    free_irq(asus_pad_device->batt_low_irq,dev);
request_battlow_irq_fail:
    gpio_free(asus_pad_device->batt_low_gpio);
request_battlow_gpio_fail:
    free_irq(asus_pad_device->inok_irq,dev);
request_inok_irq_fail:
    gpio_free(asus_pad_device->inok_gpio);
request_inok_gpio_fail:
    wake_lock_destroy(&wakelock);
    wake_lock_destroy(&wakelock_t);
    kfree(asus_pad_device);
pad_dev_alloc_fail:
    return ret;

}

static int __devexit asus_battery_ec_remove(struct platform_device *pdev)
{
    struct device *dev = &pdev->dev;

    batt_remove_proc_entry(); 
    if (!IS_ERR_OR_NULL(asus_pad_device->charger_dentry))
        debugfs_remove(asus_pad_device->charger_dentry);

    wake_lock_destroy(&wakelock);
    wake_lock_destroy(&wakelock_t);

    return 0;
}

static int asus_pad_batt_suspend(struct platform_device *pdev, pm_message_t state)
{
//                cancel_delayed_work_sync(&battery_poll_data_work);
//                flush_workqueue(battery_work_queue);
    dbg_i("asus_pad_batt_suspend. \n");
    asus_pad_device->ec_suspend_status = 1;
    return 0;
}

static int asus_pad_batt_resume(struct platform_device *pdev)
{
    asus_pad_device->ec_suspend_status = 0;
    asus_update_all(0.5);
//	queue_delayed_work(battery_work_queue,&battery_poll_data_work,0.5*HZ);
        return 0;
}


static const struct platform_device_id asus_battery_ec_table[] = {
	{DRIVER_NAME, 1},
};

static struct platform_driver asus_battery_ec_driver = {
	.driver = {
		.name		= DRIVER_NAME,
		.owner		= THIS_MODULE,
	},
	.probe		= asus_battery_ec_probe,
	.remove = __devexit_p(asus_battery_ec_remove),
        .suspend  = asus_pad_batt_suspend,
        .resume   = asus_pad_batt_resume,
	.id_table = asus_battery_ec_table,
};

static int __init asus_battery_ec_module_init(void)
{
    int rc;

    dbg_i("%s\n", __func__);
    battery_work_queue = create_singlethread_workqueue("asus_pad_battery");
    if(battery_work_queue == NULL)
    {
        dbg_e("create battery thread fail");
        return -ENOMEM;
    }

    INIT_DELAYED_WORK_DEFERRABLE(&battery_poll_data_work, asus_pad_battery_status_poll);
    INIT_DELAYED_WORK_DEFERRABLE(&base_ac_data_work, base_ac_poll);

    rc = platform_driver_register(&asus_battery_ec_driver);
    if (rc < 0)
    {
        dbg_e("%s: FAIL: platform_driver_register. rc = %d\n", __func__, rc);
        goto register_fail;
    }
    return 0;

register_fail:
    destroy_workqueue(battery_work_queue);
    return rc;
}

static void __exit asus_battery_ec_module_exit(void)
{
    int i;
    unregister_dock_attach_notifier(&attach_dock_notifier);
    unregister_dock_detach_notifier(&detach_dock_notifier);
    for (i = 0; i < ARRAY_SIZE(asus_pad_batt_supplies); i++)
        power_supply_changed(&asus_pad_batt_supplies[i]);
    dbg_i("%s: 'changed' event sent, sleeping for 10 seconds...\n",
            __func__);
    ssleep(10);

    for (i = 0; i < ARRAY_SIZE(asus_pad_batt_supplies); i++)
        power_supply_unregister(&asus_pad_batt_supplies[i]);

    kfree(asus_pad_device);
    destroy_workqueue(battery_work_queue);
    platform_driver_unregister(&asus_battery_ec_driver);
    asus_battery_driver_ready = 0;
}

module_init(asus_battery_ec_module_init);
module_exit(asus_battery_ec_module_exit);

MODULE_AUTHOR("ASUS BSP");
MODULE_DESCRIPTION("battery ec driver");
MODULE_LICENSE("GPL v2");
