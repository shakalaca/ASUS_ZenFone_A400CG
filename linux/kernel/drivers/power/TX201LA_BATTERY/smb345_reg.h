#ifndef _SMB345_REG_
#define _SMB345_REG_

/* Input current limit in mA */
static const unsigned int icl_tbl[] = {
        300,
        500,
        700,
        900,
        1200,
        1500,
        1800,
        2000,
        2200,
        2500,
};

#define SMB347_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

#define CFG_CHARGE_CURRENT                      0x00
#define CFG_CHARGE_CURRENT_FCC_MASK             0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT            5
#define CFG_CHARGE_CURRENT_PCC_MASK             0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT            3
#define CFG_CHARGE_CURRENT_TC_MASK              0x07
#define CFG_CURRENT_LIMIT                       0x01
#define CFG_CURRENT_LIMIT_DC_MASK               0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT              4
#define CFG_CURRENT_LIMIT_USB_MASK              0x0f
#define CFG_VARIOUS_FUNCS                       0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB          BIT(2)
#define CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE     BIT(4)
#define CFG_FLOAT_VOLTAGE                       0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK        0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT       6
#define CFG_STAT                                0x05
#define CFG_STAT_DISABLED                       BIT(5)
#define CFG_STAT_ACTIVE_HIGH                    BIT(7)
#define CFG_PIN                                 0x06
#define CFG_PIN_EN_CTRL_MASK                    0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH             0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW              0x60
#define CFG_PIN_EN_APSD_IRQ                     BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR                BIT(2)
#define CFG_THERM                               0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK    0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT   0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK   0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT  2
#define CFG_THERM_MONITOR_DISABLED              BIT(4)
#define CFG_SYSOK                               0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED   BIT(2)
#define CFG_OTHER                               0x09
#define CFG_OTHER_RID_MASK                      0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN          0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C           0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG          0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW            BIT(5)
#define CFG_OTG                                 0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK             0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT            4
#define CFG_OTG_CC_COMPENSATION_MASK            0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT           6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK     0x03
#define CFG_TEMP_LIMIT                          0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK            0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT           0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK           0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT          2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK            0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT           4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK           0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT          6
#define CFG_FAULT_IRQ                           0x0c
#define CFG_FAULT_IRQ_DCIN_UV                   BIT(2)
#define CFG_FAULT_IRQ_OTG_UV                    BIT(5)
#define CFG_STATUS_IRQ                          0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT           BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER     BIT(4)
#define CFG_ADDRESS                             0x0e

/* Command registers */
#define CMD_A                                   0x30
#define CMD_A_CHG_ENABLED                       BIT(1)
#define CMD_A_SUSPEND_ENABLED                   BIT(2)
#define CMD_A_OTG_ENABLED                       BIT(4)
#define CMD_A_ALLOW_WRITE                       BIT(7)
#define CMD_B                                   0x31
#define CMD_B_USB9_AND_HC_MODE  0x03
#define CMD_C                                   0x33

/* Interrupt Status registers */
#define IRQSTAT_A                               0x35
#define IRQSTAT_C                               0x37
#define IRQSTAT_C_TERMINATION_STAT              BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ               BIT(1)
#define IRQSTAT_C_TAPER_IRQ                     BIT(3)
#define IRQSTAT_D                               0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT           BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ            BIT(3)
#define IRQSTAT_E                               0x39
#define IRQSTAT_E_USBIN_UV_STAT                 BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ                  BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT                  BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ                   BIT(5)
#define IRQSTAT_F                               0x3a
#define IRQSTAT_F_OTG_UV_IRQ                    BIT(5)
#define IRQSTAT_F_OTG_UV_STAT                   BIT(4)

/* Status registers */
#define STAT_A                                  0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK               0x3f
#define STAT_B                                  0x3c
#define STAT_C                                  0x3d
#define STAT_C_CHG_ENABLED                      BIT(0)
#define STAT_C_HOLDOFF_STAT                     BIT(3)
#define STAT_C_CHG_MASK                         0x06
#define STAT_C_CHG_SHIFT                        1
#define STAT_C_CHG_TERM                         BIT(5)
#define STAT_C_CHARGER_ERROR                    BIT(6)
#define STAT_E                                  0x3f

#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG   0x0B
#define SOFT_LIMIT_HOT_CELL_TEMP_MASK                   SMB347_MASK(2, 0)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK                  SMB347_MASK(2, 2)
#define HARD_LIMIT_HOT_CELL_TEMP_MASK                   SMB347_MASK(2, 4)
#define HARD_LIMIT_COLD_CELL_TEMP_MASK                  SMB347_MASK(2, 6)

#endif
