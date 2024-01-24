// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "mcom03-clk.h"

#define UCG_SIZE 0x48

struct mcom03_clk_refmux {
	struct clk_mux mux;
	void __iomem *base_ucg;
};

struct mcom03_refmux_data {
	const char *name;
	phys_addr_t base_ref;
	phys_addr_t base_ucg;
	u32 shift;
	u32 mask;
};

static struct mcom03_clk_refmux *to_mcom03_refmux(struct clk_mux *mux)
{
	return container_of(mux, struct mcom03_clk_refmux, mux);
}

static DEFINE_SPINLOCK(mcom03_clk_refmux_lock);

static int mcom03_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct clk_mux *mux = to_clk_mux(hw);
	struct mcom03_clk_refmux *priv = to_mcom03_refmux(mux);
	unsigned long flags = 0;
	u32 old_bp;
	int ret;

	/* Ref MUX is not glitch free. All enabled channels must be turned to
	 * bypass before ref will be changed.
	 */
	spin_lock_irqsave(&mcom03_clk_refmux_lock, flags);
	old_bp = mcom03_clk_ucg_bypass_enable(priv->base_ucg);
	ret = clk_mux_ops.set_parent(hw, index);
	mcom03_clk_ucg_bypass_disable(priv->base_ucg, old_bp);
	spin_unlock_irqrestore(&mcom03_clk_refmux_lock, flags);

	return ret;
}

static u8 mcom03_clk_mux_get_parent(struct clk_hw *hw)
{
	return clk_mux_ops.get_parent(hw);
}

static int mcom03_clk_mux_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	return clk_mux_ops.determine_rate(hw, req);
}

static const struct clk_ops mcom03_clk_mux_ops = {
	.get_parent = mcom03_clk_mux_get_parent,
	.set_parent = mcom03_clk_mux_set_parent,
	.determine_rate = mcom03_clk_mux_determine_rate,
};

static void __init mcom03_clk_refmux_init(struct device_node *np,
					  struct mcom03_refmux_data *data)
{
	struct clk_init_data init = {};
	struct mcom03_clk_refmux *priv;
	unsigned int parent_count;
	const char *parent_names[4];
	int ret;
	int i;

	ret = of_property_read_string(np, "clock-output-names", &init.name);
	if (ret) {
		if (unlikely(ret != -EINVAL)) {
			pr_err("%s: Failed to get clock-output-names (%d)\n",
			       np->full_name, ret);
			return;
		}
		init.name = data->name;
	}

	parent_count = of_clk_get_parent_count(np);
	if (parent_count > ARRAY_SIZE(parent_names)) {
		pr_err("%s: Too many parent clocks (%d but maximum %ld)\n",
		       np->full_name, parent_count, ARRAY_SIZE(parent_names));
		return;
	}
	for (i = 0; i < parent_count; i++)
		parent_names[i] = of_clk_get_parent_name(np, i);

	priv = kzalloc(sizeof(struct mcom03_clk_refmux), GFP_KERNEL);
	if (!priv)
		return;

	init.ops = &mcom03_clk_mux_ops;
	init.parent_names = parent_names;
	init.num_parents = parent_count;

	priv->base_ucg = ioremap(data->base_ucg, UCG_SIZE);
	priv->mux.reg = ioremap(data->base_ref, 0x4);
	priv->mux.shift = data->shift;
	priv->mux.mask = data->mask;
	priv->mux.hw.init = &init;

	ret = clk_hw_register(NULL, &priv->mux.hw);
	if (ret) {
		pr_err("%s: Failed to register clock (%d)\n", np->full_name,
		       ret);
		kfree(priv);
		return;
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_simple_get, &priv->mux.hw);
	if (ret < 0) {
		pr_err("%s: Failed to add provider (%d)\n", np->full_name, ret);
		kfree(priv);
		return;
	}
}

struct mcom03_refmux_data mcom03_hsp_refmux0_data = {
	.name = "hsperiph_refmux0",
	.base_ref = 0x1040000c,
	.base_ucg = 0x10410000,
	.shift = 0,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_hsp_refmux1_data = {
	.name = "hsperiph_refmux1",
	.base_ref = 0x1040000c,
	.base_ucg = 0x10420000,
	.shift = 2,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_hsp_refmux2_data = {
	.name = "hsperiph_refmux2",
	.base_ref = 0x1040000c,
	.base_ucg = 0x10430000,
	.shift = 4,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_hsp_refmux3_data = {
	.name = "hsperiph_refmux3",
	.base_ref = 0x1040000c,
	.base_ucg = 0x10440000,
	.shift = 6,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_lsp1_refmux_i2s_data = {
	.name = "lsperiph1_refmux_i2s",
	.base_ref = 0x17e0010,
	.base_ucg = 0x17d0000,
	.shift = 0,
	.mask = 0x1,
};

struct mcom03_refmux_data mcom03_sdr_refmux_pcie0_data = {
	.name = "sdr_refmux_pcie0",
	.base_ref = 0x1900020,
	.base_ucg = 0x1908000,
	.shift = 0,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_sdr_refmux_pcie1_data = {
	.name = "sdr_refmux_pcie1",
	.base_ref = 0x1900030,
	.base_ucg = 0x1c00000,
	.shift = 0,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_sdr_refmux_jesd0tx_data = {
	.name = "sdr_refmux_jesd0tx",
	.base_ref = 0x1900024,
	.base_ucg = 0x19e0000,
	.shift = 0,
	.mask = 0x3,
};

struct mcom03_refmux_data mcom03_sdr_refmux_jesd0rx_data = {
	.name = "sdr_refmux_jesd0rx",
	.base_ref = 0x1900028,
	.base_ucg = 0x19e8000,
	.shift = 0,
	.mask = 0x3,
};

static void __init mcom03_clk_hsp_refmux0_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_hsp_refmux0_data);
}
CLK_OF_DECLARE(mcom03_clk_hsp_refmux0, "elvees,mcom03-clk-hsp-refmux0",
	       mcom03_clk_hsp_refmux0_init);

static void __init mcom03_clk_hsp_refmux1_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_hsp_refmux1_data);
}

CLK_OF_DECLARE(mcom03_clk_hsp_refmux1, "elvees,mcom03-clk-hsp-refmux1",
	       mcom03_clk_hsp_refmux1_init);

static void __init mcom03_clk_hsp_refmux2_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_hsp_refmux2_data);
}
CLK_OF_DECLARE(mcom03_clk_hsp_refmux2, "elvees,mcom03-clk-hsp-refmux2",
	       mcom03_clk_hsp_refmux2_init);

static void __init mcom03_clk_hsp_refmux3_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_hsp_refmux3_data);
}

CLK_OF_DECLARE(mcom03_clk_hsp_refmux3, "elvees,mcom03-clk-hsp-refmux3",
	       mcom03_clk_hsp_refmux3_init);

static void __init mcom03_clk_lsp1_refmux_i2s_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_lsp1_refmux_i2s_data);
}

CLK_OF_DECLARE(mcom03_clk_lsp1_refmux_i2s, "elvees,mcom03-clk-lsp1-refmux-i2s",
	       mcom03_clk_lsp1_refmux_i2s_init);
static void __init mcom03_clk_sdr_refmux_pcie0_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_sdr_refmux_pcie0_data);
}

CLK_OF_DECLARE(mcom03_clk_sdr_refmux_pcie0,
	       "elvees,mcom03-clk-sdr-refmux-pcie0",
	       mcom03_clk_sdr_refmux_pcie0_init);

static void __init mcom03_clk_sdr_refmux_pcie1_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_sdr_refmux_pcie1_data);
}

CLK_OF_DECLARE(mcom03_clk_sdr_refmux_pcie1,
	       "elvees,mcom03-clk-sdr-refmux-pcie1",
	       mcom03_clk_sdr_refmux_pcie1_init);

static void __init mcom03_clk_sdr_refmux_jesd0tx_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_sdr_refmux_jesd0tx_data);
}
CLK_OF_DECLARE(mcom03_clk_sdr_refmux_jesd0tx,
	       "elvees,mcom03-clk-sdr-refmux-jesd0tx",
	       mcom03_clk_sdr_refmux_jesd0tx_init);

static void __init mcom03_clk_sdr_refmux_jesd0rx_init(struct device_node *np)
{
	mcom03_clk_refmux_init(np, &mcom03_sdr_refmux_jesd0rx_data);
}
CLK_OF_DECLARE(mcom03_clk_sdr_refmux_jesd0rx,
	       "elvees,mcom03-clk-sdr-refmux-jesd0rx",
	       mcom03_clk_sdr_refmux_jesd0rx_init);
