// SPDX-License-Identifier: GPL-2.0+
// Copyright 2018-2019 RnD Center "ELVEES", JSC

#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdomain.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/reset.h>
#include <linux/reset-controller.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "common.h"

struct pmctr_domain {
	struct list_head list;
	struct generic_pm_domain base;
	void __iomem *pmctr_base_addr;
	int pwr_mask;
};

static int pmctr_domain_enable(struct generic_pm_domain *genpd, bool enable)
{
	struct pmctr_domain *domain = container_of(genpd, struct pmctr_domain,
						   base);
	void __iomem *pmctr_base_addr = domain->pmctr_base_addr;

	writel_relaxed(domain->pwr_mask, pmctr_base_addr +
		       (enable ? PMCTR_CORE_PWR_UP : PMCTR_CORE_PWR_DOWN));

	return 0;
}

static int pmctr_domain_power_off(struct generic_pm_domain *domain)
{
	return pmctr_domain_enable(domain, false);
}

static int pmctr_domain_power_on(struct generic_pm_domain *domain)
{
	return pmctr_domain_enable(domain, true);
}

static void pmctr_domain_register(struct pmctr_domain *domain,
				  struct device_node *np)
{
	unsigned int val = readl_relaxed(domain->pmctr_base_addr +
					 PMCTR_CORE_PWR_STATUS);

	domain->base.power_off = pmctr_domain_power_off;
	domain->base.power_on = pmctr_domain_power_on;

	pm_genpd_init(&domain->base, NULL, !(val & domain->pwr_mask));
	of_genpd_add_provider_simple(np, &domain->base);
}

int __init mcom02_init_pmctr(void)
{
	struct device_node *np_pmctr, *domains_node, *np;
	void __iomem *pmctr_base_addr;
	u32 pwr_mask;
	struct list_head *pos;
	struct pmctr_domain *domain;

	struct pmctr_domain domain_list;

	INIT_LIST_HEAD(&domain_list.list);

	/* Lookup the PMCTR node */
	np_pmctr = of_find_compatible_node(NULL, NULL, "elvees,mcom-pmctr");
	if (!np_pmctr)
		return 0;

	domains_node = of_get_child_by_name(np_pmctr, "domains");
	if (!domains_node) {
		pr_err("%s: failed to find domains sub-node\n", __func__);
		return 0;
	}

	pmctr_base_addr = of_iomap(np_pmctr, 0);
	if (!pmctr_base_addr) {
		pr_err("%s: failed to map PMCTR\n", __func__);
		return 0;
	}

	for_each_available_child_of_node(domains_node, np) {

		of_property_read_u32(np, "pwr-mask", &pwr_mask);
		if (!pwr_mask) {
			pr_err("%s: failed to get domain mask\n", __func__);
			continue;
		}

		domain = kzalloc(sizeof(*domain), GFP_KERNEL);
		if (!domain)
			goto err_domain;

		domain->pmctr_base_addr = pmctr_base_addr;
		domain->pwr_mask = pwr_mask;
		domain->base.name = kstrdup(np->name, GFP_KERNEL);
		if (!domain->base.name) {
			kfree(domain);
			goto err_domain;
		}

		pmctr_domain_register(domain, np);
		list_add(&(domain->list), &(domain_list.list));
	}

	return 0;

err_domain:
	list_for_each(pos, &domain_list.list) {
		domain = list_entry(pos, struct pmctr_domain, list);
		kfree(domain);
	}
	return -ENOMEM;
}
