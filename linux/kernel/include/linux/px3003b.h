
#ifndef _LINUX_PX3003B
#define _LINUX_PX3003B

//	define for IOCTL ++
#define PROXIMITYSENSOR_IOCTL_MAGIC 'p'
#define PROXIMITYSENSOR_IOCTL_GET_ENABLED			_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 1, int *) 
#define PROXIMITYSENSOR_IOCTL_ENABLE				_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 2, int *)
#define PROXIMITYSENSOR_IOCTL_GET_STATUS			_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 3, int *)
#define PROXIMITYSENSOR_IOCTL_CALIBRATION			_IOW(PROXIMITYSENSOR_IOCTL_MAGIC, 4, int *) // modify by tom for threshold calibration ++
#define PROXIMITYSENSOR_IOCTL_VCALIBRATION		_IOR(PROXIMITYSENSOR_IOCTL_MAGIC, 5, int *) // add by tom for crosstalk ++
//	define for IOCTL --

//	define for px3003b address ++
#define ADDR_PX_CONFIG_CONFIG		0x00
#define ADDR_PX_TIME_CONTROL		0x01
#define ADDR_PX_INT_STATUS		0x03
#define ADDR_PX_PS_CONTROL		0x04
#define ADDR_PX_DATA				0x05
#define ADDR_PX_FACTORY_MODE		0x08
//	define for px3003b address --

//	define for px3003b address ++
#define INDEX_PX_CONFIG_CONFIG	0
#define INDEX_PX_TIME_CONTROL		1
#define INDEX_PX_INT_STATUS		2
#define INDEX_PX_PS_CONTROL		3
#define INDEX_PX_DATA				4
#define INDEX_PX_FACTORY_MODE		5
//	define for px3003b address --



struct px3003b_i2c_platform_data {	
	int int_gpio;
};

#endif /* _LINUX_PX3003B */
