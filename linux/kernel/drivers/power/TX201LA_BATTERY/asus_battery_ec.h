#ifndef _ASUS_EC_BATTERY_H_
#define _ASUS_EC_BATTERY_H_
#include <linux/kernel.h>

#define EC_CHARGER_CTRL_STARTCHARGE          1
#define EC_CHARGER_CTRL_STOPCHARGE           2
#define EC_CHARGER_CTRL_CHG_CURRENT_NORMAL   3
#define EC_CHARGER_CTRL_CHG_CURRENT_2A       4

typedef enum {
    RET_EC_OK = 0,
    RET_EC_FAIL = -1, // i2c fail
    RET_EC_UPDATE = -2, // EC firmware updateing,can not do any i2c command
    RET_GAUGE_FAIL = -3,
} ret_type_t;

int asus_pad_batt_write_charger_reg(u8 reg_lsb,u8 lsb);
int asus_pad_batt_read_charger_reg(u8 reg);
int batt_gaguefw_write(u8 *buf);
#endif
