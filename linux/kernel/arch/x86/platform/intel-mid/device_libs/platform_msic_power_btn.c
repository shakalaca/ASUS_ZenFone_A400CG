/*
 * platform_msic_power_btn.c: MSIC power btn platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/init.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include <asm/intel_mid_powerbtn.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>
#include "platform_msic_power_btn.h"

static int mrfl_pb_irq_ack(struct intel_msic_power_btn_platform_data *pdata)
{
	intel_scu_ipc_update_register(pdata->pb_irq, 0, MSIC_PWRBTNM);
	intel_scu_ipc_update_register(pdata->pb_irq_mask, 0, MSIC_PWRBTNM);

	return 0;
}

void __init *msic_power_btn_platform_data(void *info)
{
	int ret;
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;
	static struct intel_msic_power_btn_platform_data msic_power_btn_pdata;

	pdev = platform_device_alloc(INTEL_MID_POWERBTN_DEV_NAME, -1);
	if (!pdev) {
		pr_err("%s(): out of memory\n", __func__);
		goto out;
	}

	if (INTEL_MID_BOARD(1, PHONE, MRFL)) {
		msic_power_btn_pdata.pbstat = 0xfffff61a;
		msic_power_btn_pdata.pb_level = (1 << 4);
		msic_power_btn_pdata.irq_lvl1_mask = 0x0c;
		msic_power_btn_pdata.pb_irq = 0x02;
		msic_power_btn_pdata.pb_irq_mask = 0x0d;
		msic_power_btn_pdata.irq_ack = mrfl_pb_irq_ack;
	} else if (INTEL_MID_BOARD(1, PHONE, CLVTP)) {
		msic_power_btn_pdata.pbstat = 0xffffefcb;
		msic_power_btn_pdata.pb_level = (1 << 3);
		msic_power_btn_pdata.irq_lvl1_mask = 0x21;
	} else if (INTEL_MID_BOARD(1, TABLET, CLVT)) {
		msic_power_btn_pdata.pbstat = 0xffff7fcb;
		msic_power_btn_pdata.pb_level = (1 << 3);
		msic_power_btn_pdata.irq_lvl1_mask = 0x21;
	} else {
		msic_power_btn_pdata.pbstat = 0xffff7fd0;
		msic_power_btn_pdata.pb_level = (1 << 3);
		msic_power_btn_pdata.irq_lvl1_mask = 0x21;
	}

	pdev->dev.platform_data = &msic_power_btn_pdata;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("%s(): platform_device_add() fialed\n", __func__);
		platform_device_put(pdev);
		goto out;
	}

	install_irq_resource(pdev, entry->irq);
	register_rpmsg_service("rpmsg_mid_powerbtn",
			RPROC_SCU, RP_MSIC_POWER_BTN);
out:
	return &msic_power_btn_pdata;
}
