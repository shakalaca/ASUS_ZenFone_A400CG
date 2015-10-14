#ifndef MICROP_NOTIFY_H
#define MICROP_NOTIFY_H

/*
*   Notification Msg definition
*/
enum microp_msg{
    P01_UNDEFINE=0,
    P01_ADD,
    P01_REMOVE,
    P01_BATTERY_POWER_BAD,
    P01_BATTERY_TO_CHARGING,
    P01_BATTERY_TO_NON_CHARGING,
    PAD_PHONEJACK_IN,
    PAD_PHONEJACK_OUT,
    P01_VOLUP_KEY_PRESSED,
    P01_VOLUP_KEY_RELEASED,
    P01_VOLDN_KEY_PRESSED,
    P01_VOLDN_KEY_RELEASED,
    P01_PWR_KEY_PRESSED,
    P01_PWR_KEY_RELEASED,
    P01_LIGHT_SENSOR,
    P01_AC_IN,
    P01_USB_IN,
    P01_AC_USB_OUT,
    P01_PROXM_SENSOR,
    P01_DEAD,
    PAD_UPDATE_FINISH,
    PAD_EXTEND_CAP_SENSOR,
    PAD_USB_OTG_ENABLE,
    PAD_USB_OTG_DISABLE,
    P01_LOW_BATTERY,
    P01_BATTERY_STAT_CHANGE,
    P01_APROM_CRASH,
};

/*
*   Priority here is for hall sensor call function
*   The functions registered with higher prority are called/notified earlier
*
*/
enum microp_notify_id {
    CAMERA_MP_NOTIFY=0,
    BT_PEN_MP_NOTIFY,
    USB_MP_NOTIFY,
    BATTERY_MP_NOTIFY,
    AUDIO_MP_NOTIFY,
    DOCK_MP_NOTIFY,
    TOUCH_MP_NOTIFY,
    TOUCH_PEN_NOTIFY,
    LCD_MP_NOTIFY,
    NINE_AXIS_SENSOR_MP_NOTIFY,
    VIBRATOR_MP_NOTIFY,
    MICROP_MP_NOTIFY,
    PHONE_LIGHTSENSOR_MP_NOTIFY,
    PAD_LIGHTSENSOR_MP_NOTIFY,
    AMI306_ECOMPASS_MP_NOTIFY,
    GPIOKEY_MP_NOTIFY,
};

int micropSendNotify(unsigned long val);
int register_microp_notifier(struct notifier_block *nb);
int unregister_microp_notifier(struct notifier_block *nb);

#endif
