/*
 *  Copyright 2015 ELVEES NeoTek CJSC
 *  Copyright 2017 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/cacheflush.h>
#include <linux/init.h>
#include <linux/smp.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/mfd/syscon.h>
#include <linux/regmap.h>
#include <asm/smp_scu.h>
#include <asm/smp_plat.h>

#include "common.h"

static void __iomem *spram_base_addr;
static void __iomem *pmctr_base_addr;
static struct regmap *smctr_regmap;

static void __iomem *scu_base_addr;

static void __init mcom02_map_device(char *compatible, void **base_addr)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, compatible);
	if (!node) {
		pr_err("%s: could not find %s node\n", __func__, compatible);
		return;
	}

	*base_addr = of_iomap(node, 0);
	if (!*base_addr) {
		pr_err("%s: could not map %s registers\n",
			__func__, node->name);
		return;
	}
}

static int mcom02_boot_secondary(unsigned int cpu, struct task_struct *idle)
{
	if (IS_ERR(smctr_regmap)) {
		pr_err("%s: SMCTR is not mapped\n", __func__);
		return -ENXIO;
	}

	/* Remap BOOT so that CPU1 boots from SPRAM where the boot
	 * vector is stored */
	regmap_write(smctr_regmap, SMCTR_BOOT_REMAP,
		     SMCTR_BOOT_REMAP_SPRAM);

	if (!pmctr_base_addr) {
		pr_err("%s: PMCTR is not mapped\n", __func__);
		return -ENXIO;
	}

	/* Unmask on CPU wake-up register IRQ_PMU0 interrupt */
	writel(PMCTR_CPU_INT_BIT(PMCTR_SYS_PWR_INT), pmctr_base_addr +
	       PMCTR_CPU_WKP_MASK(cpu, PMCTR_SYS_PWR_INT));
	/* Turn on power domain for CPU */
	writel(BIT(0), pmctr_base_addr + PMCTR_SYS_PWR_IMASK);
	writel(BIT(0), pmctr_base_addr + PMCTR_SYS_PWR_IRSTAT);

	return 0;
}

static void __init mcom02_smp_prepare_cpus(unsigned int max_cpus)
{
	u32 trampoline_sz = &mcom02_secondary_trampoline_end -
			    &mcom02_secondary_trampoline;

	mcom02_map_device("elvees,mcom-spram", &spram_base_addr);

	if (!spram_base_addr) {
		pr_err("%s: SPRAM is not mapped\n", __func__);
		return;
	}

	mcom02_secondary_boot_vector = virt_to_phys(mcom02_secondary_startup);

	memcpy(spram_base_addr, &mcom02_secondary_trampoline, trampoline_sz);
	flush_cache_all();
	outer_clean_range(0, trampoline_sz);

	mcom02_map_device("elvees,mcom-pmctr", &pmctr_base_addr);
	smctr_regmap = syscon_regmap_lookup_by_compatible("elvees,mcom-smctr");

	/* Disable SYS_PWR_INT interrupt and clean any pending interrupt */
	writel(0, pmctr_base_addr + PMCTR_SYS_PWR_IMASK);
	writel(BIT(0), pmctr_base_addr + PMCTR_SYS_PWR_ICLR);

	scu_base_addr = ioremap(scu_a9_get_base(), SZ_4K);

	scu_enable(scu_base_addr);
}

#ifdef CONFIG_HOTPLUG_CPU
static int mcom02_cpu_kill(unsigned int cpu)
{
	unsigned long timeout = jiffies + msecs_to_jiffies(50);

	while (!(readl(pmctr_base_addr + PMCTR_SYS_PWR_STATUS) & BIT(cpu + 1)))
		if (time_after(jiffies, timeout))
			return 0;

	return 1;
}

static void mcom02_cpu_die(unsigned int cpu)
{
	scu_power_mode(scu_base_addr, SCU_PM_POWEROFF);
	while (1)
		cpu_do_idle();
}
#endif

static struct smp_operations mcom02_smp_ops __initdata = {
	.smp_prepare_cpus	= mcom02_smp_prepare_cpus,
	.smp_boot_secondary	= mcom02_boot_secondary,
#ifdef CONFIG_HOTPLUG_CPU
	.cpu_kill		= mcom02_cpu_kill,
	.cpu_die		= mcom02_cpu_die,
#endif
};
CPU_METHOD_OF_DECLARE(mcom02_smp, "elvees,mcom-smp", &mcom02_smp_ops);
