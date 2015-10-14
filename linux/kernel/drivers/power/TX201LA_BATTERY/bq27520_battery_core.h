#ifndef _BQ27520_BATTERY_CORE_H_
#define _BQ27520_BATTERY_CORE_H_

#define BQ27520_NORMAL_ADDR 0xAA
#define BQ27520_ROM_ADDR    0x16

#define PROC_TRUE -256
#define PROC_FALSE -257

#define ERROR_CODE_I2C_FAILURE    -99999

int bq27520_write_i2c(u8 addr,u8 reg, int value, int b_single);
int bq27520_read_i2c(u8 addr,u8 reg, int *rt_value, int b_single);
int bq27520_cmp_i2c(int reg_off, int value);
int bq27520_asus_battery_dev_read_df(void);

int bq27520_asus_battery_dev_read_fw_cfg_version(void);
int bq27520_is_normal_mode(void) ;
int bq27520_asus_battery_dev_read_volt(void);
int bq27520_asus_battery_dev_read_chemical_id(void);
void bq27541_read_percentage(int *percentage);
int bq27520_to_unsealed(int unsealedkey);
int bq27520_to_full(int fullkey);
int TIgauge_LockStep(void);
int bq27520_asus_battery_dev_read_cycle_count(void);
int bq27520_asus_battery_dev_read_full_charge_capacity(void);
int bq27520_asus_battery_dev_nominal_available_capacity(void);
#endif
