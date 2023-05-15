// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 */

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of_address.h>

#include "mcom03-clk.h"

static DEFINE_SPINLOCK(mcom03_clk_dsp_lock);

static void unregister_gate_free_provider(struct mcom03_clk_provider *p)
{
	int i;

	for (i = 0; i < 2; i++)
		clk_unregister_gate(p->clk_data.clks[i]);

	kfree(p->clk_data.clks);
	kfree(p);
}

static void __init mcom03_clk_dsp_gate_init(struct device_node *np)
{
	struct mcom03_clk_provider *p;
	const char *clk_names[2];
	const char *parent_name = of_clk_get_parent_name(np, 0);
	void __iomem *base;
	int count = of_property_count_strings(np, "clock-output-names");
	int ret;
	int i;

	if (count < 0) {
		pr_err("%s: Failed to get clock-output-names (%d)\n",
		       np->name, count);
		return;
	} else if (count != 2) {
		pr_err("%s: Must be 2 names in clock-output-names (found %d)\n",
		       np->name, count);
		return;
	}

	ret = of_property_read_string_array(np, "clock-output-names",
					    clk_names, 2);
	if (ret < 0) {
		pr_err("%s: Failed to get clock-output-names (%d)\n",
		       np->name, ret);
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_err("%s: Failed to map device memory\n", np->name);
		return;
	}

	p = mcom03_clk_alloc_provider(np, 2);
	if (!p)
		return;

	for (i = 0; i < 2; i++) {
		p->clk_data.clks[i] = clk_register_gate(NULL, clk_names[i],
							parent_name,
							CLK_SET_RATE_PARENT,
							base, 8 + i, 0,
							&mcom03_clk_dsp_lock);
		if (IS_ERR(p->clk_data.clks[i])) {
			pr_err("%s: Failed to register gate (%ld)\n",
			       clk_names[i], PTR_ERR(p->clk_data.clks[i]));

			unregister_gate_free_provider(p);
			return;
		}
	}

	ret = of_clk_add_provider(p->node, of_clk_src_onecell_get,
				  &p->clk_data);
	if (ret < 0) {
		pr_err("%s: Failed to add clk provider (%d)\n", np->name, ret);
		unregister_gate_free_provider(p);
	}
}

CLK_OF_DECLARE(mcom03_clk_dsp_gate, "elvees,mcom03-dsp-gate",
	       mcom03_clk_dsp_gate_init);
