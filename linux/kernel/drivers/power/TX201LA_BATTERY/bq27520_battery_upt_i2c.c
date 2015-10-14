/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
//#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/idr.h>

#include "bq27520_battery_upt_i2c.h"
#include "asus_battery_ec.h"
#include "bq27520_battery_core.h"
//#include "bq27520_proc_fs.h"
#include "bq27520_reg.h"
#include "gaugefw.h"
#define RETRY_COUNT 3

#define bq27520_send_subcmd
//struct battery_dev_info batt_upt_dev_info;
DEFINE_MUTEX(batt_dev_upt_mutex);

int bq27520_rom_mode_write_i2c(u8 reg, int value, int b_single)
{
    return bq27520_write_i2c(BQ27520_ROM_ADDR,reg, value, b_single);
}

int bq27520_rom_mode_read_i2c(u8 reg, int *rt_value, int b_single)
{
    return bq27520_read_i2c(BQ27520_ROM_ADDR,reg, rt_value, b_single);
}

int bq27520_is_rom_mode(void)
{
    int retry = RETRY_COUNT;
    int val=0,val1=0;
    int ret=0;

    printk("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(BQ27520_ROM_ADDR,0x00, &val, 1);
        if (ret < 0) continue;

        break;
    };

    if (ret < 0) {
        return 0; //not rom mode
    }
    return 1;
}

int bq27520_enter_rom_mode()
{
    int retry = RETRY_COUNT;
    int val=0;
    int ret=0;

    printk("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_write_i2c(BQ27520_NORMAL_ADDR,0x00, 0x0F00, 0);
        if (ret < 0) continue;

        break;
    };
    if (ret < 0) {
        printk("Enter ROM mode FAIL \n");
        return ret;
    }

    /* 
     * verify it's ROM node.
     * Yes if read registers FAIL from now on.
     */
/*how to check fail????*/
    ret = bq27520_read_i2c(BQ27520_NORMAL_ADDR,0x00, &val, 1);
    if (ret <0)  {
        printk("Enter ROM Mode\n");
        return 0;
    } else {
        printk("Can not enter rom mode\n");
    }

    return -1;
}

int bq27520_exit_rom_mode() 
{
    int retry = RETRY_COUNT;
    int val=0;
    int ret=0;

    printk("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_write_i2c( BQ27520_ROM_ADDR,0x00, 0x0F, 1);
        if (ret < 0) continue;

        ret = bq27520_write_i2c( BQ27520_ROM_ADDR,0x64, 0x0F, 1);
        if (ret < 0) continue;

        ret = bq27520_write_i2c( BQ27520_ROM_ADDR,0x65, 0x00, 1);
        if (ret < 0) continue;

        break;
    };
    if (!retry) {
        printk("Exit ROM mode FAIL \n");
        return ret;
    }

    /* 
     * verify it's NOT ROM node.
     * Yes if read registers FAIL from now on.
     */
    ret = bq27520_read_i2c( BQ27520_ROM_ADDR,0x00, &val, 1);
    if (!ret)  {
        printk("Exit ROM Mode verification FAIL.\n");
        return -EACCES;
    }

    /* 
     * wait 1s and send IT_ENABLE command 
     * (battery team request)
     */
    msleep(1000);

    ret = bq27520_send_subcmd(NULL, BQ27520_SUBCMD_ENABLE_IT);
    if (ret)
        return ret;

    return 0;
}

int bq27520_rom_mode_cmp(int reg_off, int value)
{
    int retry = RETRY_COUNT;
    struct i2c_client *i2c = NULL;
    int val=0;
    int ret=0;

    printk("[%s] enter \n", __func__);

    while (retry--) {
        ret = bq27520_read_i2c(BQ27520_ROM_ADDR,reg_off, &val, 1);
        if (ret < 0) continue;

        break;
    };
    if (!retry && ret < 0) {
        return ret;
    }

    return val == value ? PROC_TRUE : PROC_FALSE;
}

int bq27520_rom_mode_wait(int m_secs)
{
    printk("[%s] enter \n", __func__);

    if (m_secs < 1) return -EINVAL;

    msleep(m_secs);

    return 0;
}

int update_normal(void)
{
    int ret;
    int update_ret = UPDATE_OK;
    int i,j,k;
    int len;
    u8 buf[255];

    printk(" fw flashing... please wait for about 10s at least.\n");
    len = sizeof(fw)/sizeof(unsigned char)/(sizeof(fw[0])/sizeof(unsigned char));
//    printk("fw[0] size:%d\n",sizeof(fw[0])/sizeof(unsigned char));
//    printk("fw[] size:%d\n",len);
    for (i=0;i<len;i++) {
/*
        memset(buf,0,sizeof(buf));
        k = 0;
        for (j=0;j<=fw[i][0];j++) {
            buf[k++] = fw[i][j];
        }
        printk("buf[0]: %d\n",buf[0]);
*/
        for (j=0;j<RETRY_COUNT; j++) {
            ret = batt_gaguefw_write(fw[i]);
            if (ret == 0) break;
        }
        // block update fail
        if (ret < 0) {
            update_ret = UPDATE_PROCESS_FAIL;
            break;
        }
    }

    return update_ret;
}

int update_from_normal_mode(void)
{
    int ret = UPDATE_NONE;
    int curr_volt;
    int i;
    int df_ver;

    printk("(%s) enter\n", __func__);

    curr_volt = bq27520_asus_battery_dev_read_volt();
    df_ver = bq27520_asus_battery_dev_read_df();

    printk("(%s) current_volt %d, df version 0x%04X\n",
            __func__, curr_volt, df_ver
    );

    if (df_ver == LATEST_FW_CFG_VERSION) {
        printk(" No need to flash battery cell data due to that both firmware config version are equal");
        return ret;
    }

    //Need update, check update voltage
    ret = UPDATE_VOLT_NOT_ENOUGH;
    if (curr_volt < 0 || curr_volt < 3700) {
        printk("Voltage not enough \n");
        return ret;
    }

    bq27520_to_unsealed(0x04141672);
    bq27520_to_full(0xFFFFFFFF);
    ret = bq27520_enter_rom_mode();

    if (ret == 0) {
        ret = update_normal();
    } else {
        ret = UPDATE_PROCESS_FAIL;
    }

//    bq27520_exit_rom_mode();
    return ret;

}

int update_from_rom_mode(void) 
{
    int i = 0;
    int ret;

    for (i=0;i<RETRY_COUNT;i++) {
        ret = update_normal();
        if (ret == UPDATE_OK);break;
    }

    return ret;
}

int bq27520_bat_upt_main_update_flow(void)
{
    int ret = UPDATE_NONE;

    printk("[%s] enter \n", __func__);

    if (bq27520_is_rom_mode()) {
        printk("is rom mode\n");
        ret = update_from_rom_mode();
    }
    else if (bq27520_is_normal_mode()) {
        printk("is normal mode\n");
        ret = update_from_normal_mode();
    }
    else {
        return UPDATE_CHECK_MODE_FAIL;
    }

    return ret;
}

