#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/i2c/atmel_mxt_1664s.h>
#include "platform_mxt1664s.h"

void *mxt1664s_platform_data(void *info)
{
	static struct mxt_platform_data mxt_pdata;

        mxt_pdata.reset_gpio = get_gpio_by_name("Touch_RST_N");
        mxt_pdata.int_gpio = get_gpio_by_name("TP_INT_N");

	return &mxt_pdata;
}

