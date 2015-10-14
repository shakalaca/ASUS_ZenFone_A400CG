#ifndef MICROP_PIN_DEF_H
#define MICROP_PIN_DEF_H

enum MICROP_INPUT{
    IN_HANDSET_IN_R= 0,
    IN_PWR_BTN_R   = 1,
    IN_NO_USE_2    = 2,
    IN_AC_USB_IN   = 3,
    IN_NO_USE_4    = 4,
    IN_NO_USE_5    = 5,
    IN_NO_USE_6    = 6,
    IN_NO_USE_7    = 7,
    IN_VOL_DOWN_R  = 8,
    IN_VOL_UP_R    = 9,
    IN_NO_USE_10   =10,
    IN_NO_USE_11   =11,
    IN_NO_USE_12   =12,
    IN_PROXY_INT   =13,
};


enum MICROP_INTR_MASK{
    INTR_EN_HANDSET_IN  = BIT(0),
    INTR_EN_PWR_BTN     = BIT(1),
    INTR_EN_USB_ID      = BIT(2),
    INTR_EN_AC_USB_IN   = BIT(3),
    INTR_EN_NO_USE_5    = BIT(4),
    INTR_EN_NO_USE_6    = BIT(5),
    INTR_EN_NO_USE_7    = BIT(6),
    INTR_EN_NO_USE_8    = BIT(7),
    INTR_EN_VOL_DOWN    = BIT(8),
    INTR_EN_VOL_UP      = BIT(9),
    INTR_EN_NO_USE_10   = BIT(10),
    INTR_EN_NO_USE_11   = BIT(11),
    INTR_EN_NO_USE_12   = BIT(12),
    INTR_EN_PROXM_INT   = BIT(13),
    INTR_EN_NO_USE_14   = BIT(14),
    INTR_EN_NO_USE_15   = BIT(15),
    INTR_EN_NO_USE_16   = BIT(16),
    INTR_EN_NO_USE_17   = BIT(17),
    INTR_EN_NO_USE_18   = BIT(18),
    INTR_EN_NO_USE_19   = BIT(19),
    INTR_EN_NO_USE_20   = BIT(20),
    INTR_EN_NO_USE_21   = BIT(21),
    INTR_EN_NO_USE_22   = BIT(22),
    INTR_EN_NO_USE_23   = BIT(23),
    INTR_EN_PWR_ON      = BIT(24),
    INTR_EN_GAUGE_5P    = BIT(25),
    INTR_EN_RSTS_WDT    = BIT(26),
    INTR_EN_SLEEP_REMD  = BIT(27),
    INTR_EN_NO_USE_28   = BIT(28),
    INTR_EN_NO_USE_29   = BIT(29),
    INTR_EN_PWR_PRESS   = BIT(30),
    INTR_EN_PWR_RELEASE = BIT(31),
};

enum MICROP_INTR_STATUS{
    INTR_STA_HANDSET_IN    = 0,
    INTR_STA_PWR_BTN       = 1,
    INTR_STA_NO_USE_2      = 2,
    INTR_STA_AC_USB_IN_OUT = 3,
    INTR_STA_NO_USE_4      = 4,
    INTR_STA_NO_USE_5      = 5,
    INTR_STA_NO_USE_6      = 6,
    INTR_STA_NO_USE_7      = 7,
    INTR_STA_VOL_DOWN      = 8,
    INTR_STA_VOL_UP        = 9,
    INTR_STA_NO_USE_10     =10,
    INTR_STA_NO_USE_11     =11,
    INTR_STA_NO_USE_12     =12,
    INTR_STA_PROXM_INT     =13,
    INTR_STA_NO_USE_14     =14,
    INTR_STA_NO_USE_15     =15,
    INTR_STA_NO_USE_16     =16,
    INTR_STA_NO_USE_17     =17,
    INTR_STA_NO_USE_18     =18,
    INTR_STA_NO_USE_19     =19,
    INTR_STA_NO_USE_20     =20,
    INTR_STA_NO_USE_21     =21,
    INTR_STA_NO_USE_22     =22,
    INTR_STA_NO_USE_23     =23,
    INTR_STA_POWER_ON      =24,
    INTR_STA_IND_GAUGE_5P  =25,
    INTR_STA_IND_RSTS_WDT  =26,
    INTR_STA_SLEEP_REMINDER=27,
    INTR_STA_BAT_STAT_CHANGE=28,
    INTR_STA_LOWLOW_BAT    =29,
    INTR_STA_PWR_PRESS     =30,
    INTR_STA_PWR_RELEASE   =31,
};



/*
 *      g_microp_ver >=5 has the following defintion
*/
#if 1
enum MICROP_OUTPUT{
    OUT_uP_SPK_EN  =BIT(0),
    OUT_uP_LCD_EN  =BIT(1),
    OUT_uP_LCD_RST =BIT(2),
    OUT_uP_EN_1V8  =BIT(3),
    OUT_uP_TS_RST_R=BIT(4),
    OUT_uP_TS_PWR_EN=BIT(5),
    OUT_uP_EN_3V3_1V2=BIT(6),
    OUT_uP_5V_PWR_EN=BIT(7),
    OUT_uP_VBUS_EN=BIT(8),
    OUT_uP_AUD_PWR_EN=BIT(9),
    OUT_uP_LED_O=BIT(11),
    OUT_uP_LED_G=BIT(12),
    OUT_up_PROXM_PWR_EN=BIT(15),
    OUT_up_SPK_SEL=BIT(16),
    OUT_up_RCV_SEL=BIT(17),
    OUT_up_AMP_G1 =BIT(30),
    OUT_up_AMP_G2 =BIT(31),
//    OUT_uP_SIZE,
};
#else
enum MICROP_OUTPUT{
    OUT_uP_SPK_EN  = 0,
    OUT_uP_LCD_EN  = 1,
    OUT_uP_LCD_RST = 2,
    OUT_uP_EN_1V8  = 3,
    OUT_uP_TS_RST_R= 4,
    OUT_uP_TS_PWR_EN=5,
    OUT_uP_EN_3V3_1V2=6,
    OUT_uP_5V_PWR_EN=7,
    OUT_uP_VBUS_EN=8,
    OUT_uP_AUD_PWR_EN=9,
//    OUT_uP_HUB_PWR_EN=10, // only for P101
    OUT_uP_LED_O=11,
    OUT_uP_LED_G=12,
//    OUT_uP_LVDS_EN=13, // only for P101
//    OUT_uP_CAM_PWR_EN=14, // only for P101
    OUT_up_PROXM_PWR_EN=15,
    OUT_up_SPK_SEL=16,
    OUT_up_RCV_SEL=17,
    OUT_uP_SIZE,
};

#endif

#endif
