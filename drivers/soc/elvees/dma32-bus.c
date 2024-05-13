// SPDX-License-Identifier: GPL-2.0
/*
 * DMA bus driver for MCom-03
 * Copyright 2023 RnD Center "ELVEES", JSC
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <soc/elvees/mcom03/mcom03_sip.h>

#define mcom03_ddr_subs_sip(id, param) \
	mcom03_sip_smccc_smc(MCOM03_SIP_DDR_SUBS, (id), (param), 0, 0, 0, 0, 0)

static const struct of_device_id mcom03_dma32_bus_of_match[] = {
	{ .compatible = "elvees,mcom03-dma32-hsperiph", },
	{ .compatible = "elvees,mcom03-dma32-lsperiph0", },
	{ .compatible = "elvees,mcom03-dma32-lsperiph1", },
	{ .compatible = "elvees,mcom03-dma32-media", },
	{ /* sentinel */ },
};

static int mcom03_dma32_bus_probe(struct platform_device *pdev)
{
	u32 bar;
	u64 ddrhigh_addr;
	struct device_node *np = pdev->dev.of_node;
	int ret = of_property_read_u64_index(np, "dma-ranges", 1, &ddrhigh_addr);

	if (ret) {
		dev_err(&pdev->dev, "Can't find 'dma-ranges' DTS property\n");
		return ret;
	}

	/* The address in 'dma-ranges' property can be obtained using the following equation:
	   AxADDR* = {BAR + AxADDR[31:30], AxADDR[29:0]}
	   This setting is needed in order for the devices to use DDR high address range */
	bar = ddrhigh_addr >> 30U;

	if (of_device_is_compatible(np, "elvees,mcom03-dma32-hsperiph"))
		mcom03_ddr_subs_sip(MCOM03_SIP_DDR_SUBS_SET_HSPERIPH_BAR, bar);
	else if (of_device_is_compatible(np, "elvees,mcom03-dma32-lsperiph0"))
		mcom03_ddr_subs_sip(MCOM03_SIP_DDR_SUBS_SET_LSPERIPH0_BAR, bar);
	else if (of_device_is_compatible(np, "elvees,mcom03-dma32-lsperiph1"))
		mcom03_ddr_subs_sip(MCOM03_SIP_DDR_SUBS_SET_LSPERIPH1_BAR, bar);
	else if (of_device_is_compatible(np, "elvees,mcom03-dma32-media"))
		mcom03_ddr_subs_sip(MCOM03_SIP_DDR_SUBS_SET_GPU_BAR, bar);

	return 0;
}

static struct platform_driver mcom03_dma32_bus_driver = {
	.probe	= mcom03_dma32_bus_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= of_match_ptr(mcom03_dma32_bus_of_match),
	},
};
builtin_platform_driver(mcom03_dma32_bus_driver);
