
#ifndef _LINUX_ASUS_EC_H
#define _LINUX_ASUS_EC_H

#define ASUS_EC_NAME "ITE8566"

struct asus_ec_i2c_platform_data {

	int int0_gpio;
	int int1_gpio;
};

typedef enum {
	S0 = 0x00,
	S1 = 0x01,
	EARLY_SUSPEND = 0x02,
	S3 = 0x03,
	LATE_RESUME = 0x04,
    S5 = 0x05,
    AUTO_EARLY_SUSPEND = 0x06,
    AUTO_SUSPEND = 0x07,
    BOOTING = 0x08,
    SF = 0xFF
}APOWER_STATUS;

typedef enum {
	BASE_DETACH = 0,
    BASE_ATTACH
}BASE_IN_OUT;

typedef enum {
	SYSTEM_NONE = 0, //base detach
	SYSTEM_ANDROID,
    SYSTEM_WINDOWS,
    SYSTEM_UPDATE
}SYSTEM_OWNER;

typedef enum {
	WIN8_S0 = 0x00,
	WIN8_S1 = 0x01,
	WIN8_S3 = 0x03,
    WIN8_S5 = 0x05,
    WIN8_DETACH = 0xFF
}WIN_POWER_STATUS;

typedef enum {
	HOST,
	CLIENT,
}USB_SWITCH_MODE;

typedef enum {
	MOS = 0x01,
	COS = 0x02,
	RECOVERY = 0x04,
	POS = 0x08	
}OS_CLASS;

typedef enum {
    LEDOFF = 0x00,
	NUM_LOCK = 0x01,
	CAPS_LOCK = 0x02,
	SCROLL_LOCK = 0x04,
	COMPOSE = 0x08,
	KANA = 0x10
}KB_LED;

#endif /* _LINUX_ASUS_EC_H */
