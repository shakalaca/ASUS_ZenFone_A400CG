
#ifndef _LINUX_ELAN_TOUCHPAD_H
#define _LINUX_ELAN_TOUCHPAD_H

#define ELAN_TOUCHPAD_NAME "T200"

struct elan_touchpad_i2c_platform_data {

	int int_gpio;
	int tp_sw_gpio;
};

#endif /* _LINUX_ELAN_TOUCHPAD_H */
