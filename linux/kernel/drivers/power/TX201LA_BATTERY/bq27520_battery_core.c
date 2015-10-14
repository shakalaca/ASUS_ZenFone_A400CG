#include <linux/module.h>
#include <linux/delay.h>
#include <asm/unaligned.h>
#include "bq27520_battery_core.h"
#include "bq27520_reg.h"
#include "asus_battery_ec.h"
#include "util.h"
#define RETRY_COUNT 3

extern int ite_gauge_stop_polling(void);
extern int ite_gauge_start_polling(void);
extern int ite_gauge_write_reg(unsigned char *buf);
extern int ite_gauge_read_reg(int addr, int reg, int length, unsigned char *return_buf);

int bq27520_write_i2c(u8 addr,u8 reg, int value, int b_single)
{
    unsigned char data[5];
    int length;
    int ret;

    memset(data, 0, sizeof(data));
//    data[0] = addr;
    data[1] = reg;
    data[0] = b_single ? 1 : 2;
    data[2] = value& 0x00FF;
    data[3] = (value & 0xFF00) >> 8;
    ret = ite_gauge_write_reg(data);

    return ret;
}

int bq27520_read_i2c(u8 addr, u8 reg, int *rt_value,int b_single)
{
    unsigned char data[2];
    int length;
    int err;

    if (b_single)
        length = 1;
    else
        length = 2;
    addr = addr >> 1;
    err = ite_gauge_read_reg(addr, reg, length, data);

    if (err == RET_EC_OK) {
        if (!b_single)
            *rt_value = get_unaligned_le16(data);
        else
            *rt_value = data[0];
    }

    return err;
}

static int bq27520_i2c_txsubcmd(u8 addr,
                u8 reg, unsigned short subcmd)
{
    unsigned char data[5];
    int ret;
    int i;
    memset(data, 0, sizeof(data));
//    data[0] = addr;
    data[1] = reg;
    data[0] = 2;
    data[2] = subcmd& 0x00FF;
    data[3] = (subcmd & 0xFF00) >> 8;
    ret = ite_gauge_write_reg(data);

    return ret;
}


static int bq27520_cntl_cmd(u8 addr,u16 sub_cmd)
{
    return bq27520_i2c_txsubcmd(addr, BQ27520_REG_CNTL, sub_cmd);
}

int bq27520_send_subcmd(u8 addr, int *rt_value, u16 sub_cmd)
{
    int ret, tmp_buf = 0;

    ret = bq27520_cntl_cmd(addr, sub_cmd);
    if (ret != RET_EC_OK) {
        dbg_e("Send subcommand 0x%04X error.\n", sub_cmd);
        return ret;
    }
    udelay(200);

    if (!rt_value) return ret;

    //need read data to rt_value
    ret = bq27520_read_i2c( addr, BQ27520_REG_CNTL, &tmp_buf, 0);
    if (ret != RET_EC_OK)
        dbg_e("Error!! %s subcommand %04X\n",
                         __func__, sub_cmd);
    *rt_value = tmp_buf;
    return ret;
}

int bq27520_to_unsealed(int unsealedkey) {
    int ret;
    ret = bq27520_write_i2c(BQ27520_NORMAL_ADDR,0x00, (unsealedkey >>16) & 0xFFFF, 0);
    if (ret) {
        printk("set sealed to unsealed error(1) %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    ret = bq27520_write_i2c(BQ27520_NORMAL_ADDR,0x00, unsealedkey & 0xFFFF, 0);
    if (ret) {
        printk("set sealed to unsealed error(2) %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    return ret;
}

int bq27520_to_full(int fullkey) {
    int ret;

    ret = bq27520_write_i2c(BQ27520_NORMAL_ADDR,0x00, (fullkey>>16) &0xFFFF, 0);
    if (ret) {
        printk("Set unsealed to full(1) error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    ret = bq27520_write_i2c(BQ27520_NORMAL_ADDR,0x00, fullkey & 0xFFFF , 0);
    if (ret) {
        printk("Set unseal to full(2) error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    return ret;
}

int TIgauge_LockStep(void) {
    int status;
    uint8_t i2cdata[32] = {0};

    dbg_d("%s enter\n", __func__);

    i2cdata[0] = 0x20;
    i2cdata[1] = 0x00;

    status = bq27520_write_i2c(BQ27520_NORMAL_ADDR, 0x00 ,i2cdata,0);

    if (status < 0)
        dbg_e("%s: i2c write error %d\n", __func__, status);

}

int bq27520_is_normal_mode(void)
{
    int retry = RETRY_COUNT;
    int val=0;
    int ret=0;

    dbg_i("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR,0x00, &val, 1);
        if (ret != RET_EC_OK) continue;

        break;
    };
    if (ret < 0) {
        return 0; //not normal mode
    }
    return 1; //it's normal mode
}

int bq27520_cmp_i2c(int reg_off, int value)
{
    int retry = RETRY_COUNT;
    int val=0;
    int ret=0;

    dbg_i("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR,reg_off, &val, 1);
        if (ret != RET_EC_OK) continue;

        break;
    };
    if (!retry && ret < 0) {
        return ret;
    }

    return val == value ? PROC_TRUE : PROC_FALSE;
}

int bq27520_asus_battery_dev_read_fw_cfg_version(void)
{
    int fw_cfg_ver=0;
    int ret;
/*
    ret = bq27520_write_i2c(BQ27520_NORMAL_ADDR,0x3F, 0x01, 1);
    if (ret) {
        printk("Get fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    udelay(800); //delay awhile to get version. Otherwise version data not transfer complete
    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR,0x40, &fw_cfg_ver, 0);
    if (ret) {
        printk("Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
*/
    ret = bq27520_send_subcmd(BQ27520_NORMAL_ADDR, &fw_cfg_ver, BQ27520_SUBCMD_FW_VER);
    if (ret != RET_EC_OK) {
        dbg_e("Read fw cfg version error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    return fw_cfg_ver;
}

int bq27520_asus_battery_dev_read_df(void)
{
    int ret;
    int df=0;

    ret = bq27520_send_subcmd(BQ27520_NORMAL_ADDR,&df,BQ27520_SUBCMD_DF_VERSION);
    if (ret != RET_EC_OK) {
        dbg_e("Read df error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    return df;
}

int bq27520_asus_battery_dev_read_volt(void)
{
    int ret;
    int volt=0;

    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR,BQ27520_REG_VOLT, &volt, 0);
    if (ret != RET_EC_OK) {
        dbg_e("Read voltage error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    return volt;
}

void bq27541_read_percentage(int *percentage) {
    int ret;
    int volt=0;

    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR,BQ27541_REG_SOC, percentage, 0);
    if (ret != RET_EC_OK) {
        dbg_e("Read percentage error %d.\n", ret);
    }

    return;
}

int bq27520_asus_battery_dev_read_chemical_id(void)
{
    int chem_id=0;
    int ret;

    ret = bq27520_send_subcmd(BQ27520_NORMAL_ADDR, &chem_id, BQ27520_SUBCMD_CHEM_ID);
    if (ret != RET_EC_OK) {
        dbg_e("Read chemical ID error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    return chem_id;
}

int bq27520_asus_battery_dev_read_cycle_count(void)
{
    int ret;
    int cc=0;

    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR, BQ27520_REG_CC, &cc, 0);
    if (ret != RET_EC_OK) {
        dbg_e("Read Cycle Count(CC) error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    return cc;
}

int bq27520_asus_battery_dev_read_full_charge_capacity(void) 
{
    int ret;
    int mAhr=0;

    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR, BQ27520_REG_FCC, &mAhr, 0);
    if (ret != RET_EC_OK) {
        dbg_e("Read full charge capacity error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }
    //dev_info(&client->dev, ":: av power %04X\n", mWhr);
    return mAhr;
}

int bq27520_asus_battery_dev_nominal_available_capacity(void)
{
    int ret;
    int mAhr=0;

    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR, BQ27520_REG_NAC, &mAhr, 0);
    if (ret) {
        dbg_e("Read NAC error %d.\n", ret);
        return ERROR_CODE_I2C_FAILURE;
    }

    return mAhr;
}
