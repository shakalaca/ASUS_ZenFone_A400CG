#define MAPPING_TOUCH_RESOLUTION_TO_DISPLAY  
#ifdef MAPPING_TOUCH_RESOLUTION_TO_DISPLAY
	#define ELAN_X_MAX 	1920
	#define ELAN_Y_MAX	1200
	#define ELAN_X_RESOLUTION 2752	// add by leo for display resolution mapping
	#define ELAN_Y_RESOLUTION 1664	// add by leo for display resolution mapping
#else
	#define ELAN_X_MAX 	2752
	#define ELAN_Y_MAX	1664
#endif

#ifndef _LINUX_ELAN_KTF2K_H
#define _LINUX_ELAN_KTF2K_H

#define ELAN_KTF3K_NAME "ekth3374"

struct elan_ktf3k_i2c_platform_data {
	uint16_t version;
	int abs_x_min;
	int abs_x_max;
	int abs_y_min;
	int abs_y_max;
	int intr_gpio;
	int rst_gpio;
	int pwr_en_gpio;
};

#endif /* _LINUX_ELAN_KTF2K_H */
