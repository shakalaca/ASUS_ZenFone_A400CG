#ifndef _BQ27520_REG_H_
#define _BQ27520_REG_H_

#define BQ27541_REG_SOC                 0x2c

/* IC version: G4 */
#define CURRENT_IC_VERSION                      4
#define IC_VERSION_G4                           4
#define IC_VERSION_G3                           3

/* Bq27520 standard data commands */
#if CURRENT_IC_VERSION == IC_VERSION_G3
    #define BQ27520_REG_CNTL            0x00
    #define BQ27520_REG_AR                      0x02
    #define BQ27520_REG_ARTTE           0x04
    #define BQ27520_REG_TEMP            0x06
    #define BQ27520_REG_VOLT            0x08
    #define BQ27520_REG_FLAGS           0x0A
    #define BQ27520_REG_NAC                     0x0C
    #define BQ27520_REG_FAC                     0x0e
    #define BQ27520_REG_RM                      0x10
    #define BQ27520_REG_FCC                     0x12
    #define BQ27520_REG_AI                      0x14
    #define BQ27520_REG_TTE                     0x16
    #define BQ27520_REG_TTF                     0x18
    #define BQ27520_REG_SI                      0x1a
    #define BQ27520_REG_STTE            0x1c
    #define BQ27520_REG_MLI                     0x1e
    #define BQ27520_REG_MLTTE           0x20
    #define BQ27520_REG_AE                      0x22
    #define BQ27520_REG_AP                      0x24
    #define BQ27520_REG_TTECP           0x26
    #define BQ27520_REG_SOH                     0x28
    #define BQ27520_REG_SOC                     0x2c
    #define BQ27520_REG_NIC                     0x2e
    #define BQ27520_REG_ICR                     0x30
    #define BQ27520_REG_LOGIDX          0x32
    #define BQ27520_REG_LOGBUF          0x34
    #define BQ27520_FLAG_DSC            BIT(0)
    #define BQ27520_FLAG_FC                     BIT(9)
    #define BQ27520_FLAG_BAT_DET        BIT(3)
    #define BQ27520_CS_DLOGEN           BIT(15)
    #define BQ27520_CS_SS                   BIT(13)

#elif CURRENT_IC_VERSION == IC_VERSION_G4

    #define BQ27520_REG_CNTL            0x00
    #define BQ27520_REG_AR                      0x02
    #define BQ27520_REG_ARTTE           0x04
    #define BQ27520_REG_TEMP            0x06
    #define BQ27520_REG_VOLT            0x08
    #define BQ27520_REG_FLAGS           0x0A
    #define BQ27520_REG_NAC                     0x0C
    #define BQ27520_REG_FAC                     0x0e
    #define BQ27520_REG_RM                      0x10
    #define BQ27520_REG_FCC                     0x12
    #define BQ27520_REG_AI                      0x14
    #define BQ27520_REG_TTE                     0x16
    #define BQ27520_REG_SI                      0x18
    #define BQ27520_REG_STTE            0x1a
    #define BQ27520_REG_SOH                     0x1c
    #define BQ27520_REG_CC                      0x1e
    #define BQ27520_REG_SOC                     0x20

    #define BQ27520_REG_IC                      0x22
    #define BQ27520_REG_IT                      0x28
    #define BQ27520_REG_RS                      0x2a
    #define BQ27520_REG_OC                      0x2c
    #define BQ27520_REG_DC                      0x2e

    #define BQ27520_FLAG_DSC            BIT(0)
    #define BQ27520_FLAG_FC                     BIT(9)
    #define BQ27520_FLAG_BAT_DET        BIT(3)
    #define BQ27520_CS_DLOGEN           BIT(15)
    #define BQ27520_CS_SS                   BIT(13)
#else
    /* may compile error */
#endif

/* Control subcommands */
#if CURRENT_IC_VERSION == IC_VERSION_G3
    #define BQ27520_SUBCMD_CTNL_STATUS          0x0000
    #define BQ27520_SUBCMD_DEVICE_TYPE          0x0001
    #define BQ27520_SUBCMD_FW_VER                       0x0002
    #define BQ27520_SUBCMD_PREV_MACW            0x0007
    #define BQ27520_SUBCMD_CHEM_ID                      0x0008
    #define BQ27520_SUBCMD_OCV                          0x000c
    #define BQ27520_SUBCMD_BAT_INS                      0x000d
    #define BQ27520_SUBCMD_BAT_REM                      0x000e
    #define BQ27520_SUBCMD_SET_HIB                      0x0011
    #define BQ27520_SUBCMD_CLR_HIB                      0x0012
    #define BQ27520_SUBCMD_SET_SLP                      0x0013
    #define BQ27520_SUBCMD_CLR_SLP                      0x0014
    #define BQ27520_SUBCMD_FCT_RES                      0x0015
    #define BQ27520_SUBCMD_ENABLE_DLOG          0x0018
    #define BQ27520_SUBCMD_DISABLE_DLOG         0x0019
    #define BQ27520_SUBCMD_DF_VERSION           0x001f
    #define BQ27520_SUBCMD_SEALED                       0x0020
    #define BQ27520_SUBCMD_ENABLE_IT            0x0021
    #define BQ27520_SUBCMD_DISABLE_IT           0x0023
    #define BQ27520_SUBCMD_RESET                        0x0041

#elif CURRENT_IC_VERSION == IC_VERSION_G4
    #define BQ27520_SUBCMD_CTNL_STATUS          0x0000
    #define BQ27520_SUBCMD_DEVICE_TYPE          0x0001
    #define BQ27520_SUBCMD_FW_VER                       0x0002
    #define BQ27520_SUBCMD_PREV_MACW            0x0007
    #define BQ27520_SUBCMD_CHEM_ID                      0x0008
    #define BQ27520_SUBCMD_OCV                          0x000c
    #define BQ27520_SUBCMD_BAT_INS                      0x000d
    #define BQ27520_SUBCMD_BAT_REM                      0x000e
    #define BQ27520_SUBCMD_SET_HIB                      0x0011
    #define BQ27520_SUBCMD_CLR_HIB                      0x0012
    #define BQ27520_SUBCMD_SET_SLP                      0x0013
    #define BQ27520_SUBCMD_CLR_SLP                      0x0014
    #define BQ27520_SUBCMD_DF_VERSION           0x001f
    #define BQ27520_SUBCMD_SEALED                       0x0020
    #define BQ27520_SUBCMD_ENABLE_IT            0x0021
    #define BQ27520_SUBCMD_RESET                        0x0041

#else
    /* may compile error */
#endif

#endif
