/*
 * platform_spca700xa.c: SPCA700XA platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lnw_gpio.h>
#include <linux/spi/intel_mid_ssp_spi.h>
#include <asm/intel-mid.h>
#include "platform_spca700xa.h"


void __init *spca700xa_platform_data(void *info)
{
	struct spi_board_info *spi_info = info;
	int intr;


	spi_info->mode = SPI_MODE_0;

	return NULL;
}


