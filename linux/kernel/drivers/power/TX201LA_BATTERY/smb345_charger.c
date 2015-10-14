#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/usb/penwell_otg.h>
#include "smb345_reg.h"
#include "asus_battery_ec.h"
#include "util.h"

#define MAINS_CURRENT_LIMIT   1350
#define TWINS_H_CURRENT_LIMIT 1800
#define USB_HC_CURRENT_LIMIT  500

extern int ite_read_chargeric_reg(int addr, int reg, int length, unsigned char *return_buf);
extern int ite_write_chargeric_reg(unsigned char *buf);
/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl, size_t size, unsigned int val)
{
        size_t i;

        for (i = 0; i < size; i++)
                if (val < tbl[i])
                        break;
        return i > 0 ? i - 1 : -EINVAL;
}

int smb345_write_i2c(u8 reg, u8 value)
{
    unsigned char data[5];
    int length;
    int ret;

    memset(data, 0, sizeof(data));
    data[0] = 1;
    data[1] = reg;
    data[2] = value & 0xFF;
    ret = ite_write_chargeric_reg(data);

    return ret;
}

int smb345_read_i2c(u8 reg)
{
    unsigned char data[2];
    int length;
    int err;

    length = 1;
    err = ite_read_chargeric_reg(0x6A, reg, length, data);

    if (err == RET_EC_OK) {
        err = data[0];
    }

    return err;
}

static int smb345_set_writable(bool writable) {
    int ret,tmp;
    ret = smb345_read_i2c(CMD_A);
    if (ret < 0) {
       return ret;
    }
    if (writable)
        ret |= CMD_A_ALLOW_WRITE;
    else
        ret &= ~CMD_A_ALLOW_WRITE;
    return smb345_write_i2c(CMD_A,ret);
}

/* enable/disable AICL function */
static int smb345_OptiCharge_Toggle(bool on)
{
    int ret;

    ret = smb345_read_i2c(CFG_VARIOUS_FUNCS);
    if (ret < 0)
        goto fail;

    if (on)
        ret |= CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;
    else
        ret &= ~CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;

    ret = smb345_write_i2c(CFG_VARIOUS_FUNCS, ret);
    if (ret < 0)
        goto fail;

fail:
    return ret;
}

static int smb345_set_current_limits(int usb_state, bool is_twinsheaded)
{
    int ret, index=-1;

    ret = smb345_set_writable(true);
    if (ret < 0)
        return ret;

    ret = smb345_read_i2c(CFG_CURRENT_LIMIT);
    if (ret < 0)
        return ret;

    if (usb_state == AC_IN) {
        index = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
                        MAINS_CURRENT_LIMIT);
    }
    else if (usb_state == USB_IN) {
        if (is_twinsheaded)
            index = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
                        TWINS_H_CURRENT_LIMIT);
        else
            index = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
                        USB_HC_CURRENT_LIMIT);
    }
    if (index < 0)
        return index;

    return smb345_write_i2c(CFG_CURRENT_LIMIT, index);
}

/* enable USB5 or USB9 and HC mode function */
static int smb345_USB9_HC_Toggle(bool on)
{
    int ret;

    ret = smb345_read_i2c(CMD_B);
    if (ret < 0)
        goto fail;

    if (on)
        ret |= CMD_B_USB9_AND_HC_MODE;
    else
        ret &= ~CMD_B_USB9_AND_HC_MODE;
    ret = smb345_write_i2c(CMD_B, ret);

fail:
    return ret;
}

/* enable USB5 or USB9 and HC mode pin control function */
static int smb345_USB9_HC_PIN_Control(bool on)
{
    int ret;
    u8 b = BIT(4);

    ret = smb345_read_i2c(CFG_PIN);
    if (ret < 0)
        goto fail;

    if (on)
        ret |= b;
    else
        ret &= ~b;
    ret = smb345_write_i2c(CFG_PIN, ret);

fail:
    return ret;
}

static int smb345_masked_write(int reg,u8 mask, u8 val)
{
    int ret;

    ret = smb345_read_i2c(reg);
    if (ret <0) {
        printk("smb345_read_reg failed: reg=%03X, ret=%d\n", reg, ret);
        return ret;
    }

    ret &= ~mask;
    ret |= val & mask;
    ret = smb345_write_i2c(reg, ret);
    if (ret) {
        pr_err("smb345_write failed: reg=%03X, ret=%d\n", reg, ret);
        return ret;
    }
    return 0;
}

/* ME372CG: JEITA function for cell temperature control
 *          by SOC
 */
int smb345_soc_control_jeita(void)
{
    int ret;

    pr_info("%s:", __func__);
    ret = smb345_set_writable(true);
    if (ret < 0)
        return ret;

    /* write 0bh[5:4]="11" */
    ret = smb345_masked_write(HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
                HARD_LIMIT_HOT_CELL_TEMP_MASK,0xff);
    if (ret) {
        printk("Failed to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
        return ret;
    }

    /* write 0bh[1:0]="11" */
    ret = smb345_masked_write(HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
                SOFT_LIMIT_HOT_CELL_TEMP_MASK,0xff);
    if (ret) {
        printk("Failed to set SOFT_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
        return ret;
    }

    return ret;
}

int smb345_charger_toggle(bool on) {
    int ret = 0;
    ret = smb345_set_writable(true);
    if (ret <0)
        return ret;
    ret = smb345_read_i2c(CFG_PIN);
    if (ret < 0)
        goto out;
    /*
     * Make the charging functionality controllable by a write to the
     * command register unless pin control is specified in the platform
     * data.
     */
    ret &= ~CFG_PIN_EN_CTRL_MASK;
    if (on) {
        /* set Pin Controls - active low (ME371MG connect EN to GROUND) */
        ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
    } else {
        /* Do nothing, 0 means i2c control
            . I2C Control - "0" in Command Register disables charger */
    }

    ret = smb345_write_i2c(CFG_PIN,ret);
    if (ret < 0)
        goto out;
out:
    return ret;
}

void smb345_dump_registers(struct seq_file *s)
{
    u8 reg,ret,tmp;

    printk(" Control registers:\n");
    printk(" ==================\n");
    printk(" #Addr\t#Value\n");
    if (s) {
        seq_printf(s, " Control registers:\n");
        seq_printf(s, " ==================\n");
        seq_printf(s, " #Addr\t#Value\n");
    }

    for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
        ret = smb345_read_i2c(reg);
        printk(" 0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
        if (s) seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
    }

    printk("\n");
    if (s) seq_printf(s, "\n");

    printk("Command registers:\n");
    printk("==================\n");
    printk("#Addr\t#Value\n");

    if (s) {
        seq_printf(s, "Command registers:\n");
        seq_printf(s, "==================\n");
        seq_printf(s, "#Addr\t#Value\n");
    }

    ret = smb345_read_i2c(CMD_A);
    printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_A, BYTETOBINARY(ret));
    if (s) seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_A, BYTETOBINARY(ret));
    ret = smb345_read_i2c(CMD_B);
    printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_B, BYTETOBINARY(ret));
    if (s) seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_B, BYTETOBINARY(ret));
    ret = smb345_read_i2c(CMD_C);
    printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_C, BYTETOBINARY(ret));
    if (s) seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_C, BYTETOBINARY(ret));
    printk("\n");
    if (s) seq_printf(s, "\n");

    printk("Interrupt status registers:\n");
    printk("===========================\n");
    printk("#Addr\t#Value\n");
    if (s) {
        seq_printf(s, "Interrupt status registers:\n");
        seq_printf(s, "===========================\n");
        seq_printf(s, "#Addr\t#Value\n");
    }

    for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
        ret = smb345_read_i2c(reg);
        printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
        if (s) seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
    }
    printk("\n");
    if (s) seq_printf(s, "\n");

    printk("Status registers:\n");
    printk("===========================\n");
    printk("#Addr\t#Value\n");
    if (s) {
        seq_printf(s, "Status registers:\n");
        seq_printf(s, "=================\n");
        seq_printf(s, "#Addr\t#Value\n");
    }
    for (reg = STAT_A; reg <= STAT_E; reg++) {
        ret = smb345_read_i2c(reg);
        printk("0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
        if (s) seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
    }
}

void smb345_config_max_current_twinheadeddragon(int inok_gpio)
{
    /* Allow violate register can be written - Write 30h[7]="1" */
    if (smb345_set_writable(true) < 0) {
        printk("%s: smb347_set_writable failed!\n", __func__);
         return;
    }

    /* Disable AICL - Write 02h[4]="0" */
    if (smb345_OptiCharge_Toggle(false) < 0) {
        printk("%s: fail to disable AICL\n", __func__);
        return;
    }

    /* Set I_USB_IN=1800mA - 01h[3:0]="0110" */
    if (smb345_set_current_limits(USB_IN, true) < 0) {
        printk("%s: fail to set max current limits for TWINSHEADED\n", __func__);
        return;
    }

    /* Set USB to HC mode - Write 31h[1:0]="11" */
    if (smb345_USB9_HC_Toggle(true) < 0) {
        printk("%s: fail to enable USB9 and HC mode!\n", __func__);
        return;
    }

    /* Set USB5/1/HC to register control - Write 06h[4]="0" */
    if (smb345_USB9_HC_PIN_Control(false) < 0) {
        printk("%s: fail to disable USB9 and HC mode pin control!\n", __func__);
        return;
    }

    smb345_soc_control_jeita();

    /* check if ACOK# = 0 */
    if (gpio_get_value(inok_gpio)) {
        printk("%s: system input voltage is not valid after charge current settings\n", __func__);
        return;
    }

    printk("%s: charger type: TWINHEADEDDRAGON done.\n", __func__);
}
