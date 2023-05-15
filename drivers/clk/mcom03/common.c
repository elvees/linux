// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 */

#include <linux/of_address.h>
#include <linux/slab.h>

#include "mcom03-clk.h"

struct mcom03_clk_provider *mcom03_clk_alloc_provider(struct device_node *node,
						      int count)
{
	struct mcom03_clk_provider *p;

	p = kzalloc(sizeof(*p), GFP_KERNEL);
	if (!p)
		return NULL;

	p->clk_data.clks = kcalloc(count, sizeof(struct clk *), GFP_KERNEL);
	if (!p->clk_data.clks)
		goto free_provider;

	p->clk_data.clk_num = count;
	p->node = node;
	p->base = of_iomap(node, 0);
	if (!p->base) {
		pr_err("Failed to map clock provider registers (%s)\n",
		       node->name);
		goto free_clks;
	}
	return p;

free_clks:
	kfree(p->clk_data.clks);
free_provider:
	kfree(p);
	return NULL;
}
