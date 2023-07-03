// SPDX-License-Identifier: GPL-2.0

// Copyright 2023-2025 RnD Center "ELVEES", JSC

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include "mcom03-clk.h"

#define to_mcom03_gate(_hw) container_of(_hw, struct mcom03_clk_gate, hw)

int mcom03_clk_gate_set_nolock(struct mcom03_clk_gate *g)
{
	int ret = 0;

	if (!g->pd || g->pd->is_enabled)
		ret = regmap_update_bits(g->sdr_urb,
					 g->reg,
					 BIT(g->bit_idx),
					 g->is_enabled ? BIT(g->bit_idx) : 0);

	return ret;
}

static int mcom03_clk_gate_set(struct mcom03_clk_gate *g, bool enable)
{
	bool old_is_enabled = g->is_enabled;
	int ret;

	g->is_enabled = enable;
	mcom03_clk_pd_lock(g->pd);
	ret = mcom03_clk_gate_set_nolock(g);
	mcom03_clk_pd_unlock(g->pd);
	if (ret)
		g->is_enabled = old_is_enabled;

	return ret;
}

static int mcom03_clk_gate_enable(struct clk_hw *hw)
{
	struct mcom03_clk_gate *g = to_mcom03_gate(hw);

	return mcom03_clk_gate_set(g, true);
}

static void mcom03_clk_gate_disable(struct clk_hw *hw)
{
	struct mcom03_clk_gate *g = to_mcom03_gate(hw);

	mcom03_clk_gate_set(g, false);
}

int mcom03_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 value;
	struct mcom03_clk_gate *g = to_mcom03_gate(hw);

	mcom03_clk_pd_lock(g->pd);
	if (!g->pd || g->pd->is_enabled) {
		regmap_read(g->sdr_urb, g->reg, &value);
		g->is_enabled = value & BIT(g->bit_idx);
	}

	mcom03_clk_pd_unlock(g->pd);

	return g->is_enabled;
}

static const struct clk_ops mcom03_clk_gate_ops = {
	.enable = mcom03_clk_gate_enable,
	.disable = mcom03_clk_gate_disable,
	.is_enabled = mcom03_clk_gate_is_enabled,
};

int mcom03_clk_gate_register(struct mcom03_clk_gate *gate)
{
	struct clk_init_data init = {
		.name = gate->name,
		.ops = &mcom03_clk_gate_ops,
		.flags = CLK_SET_RATE_PARENT,
		.parent_names = gate->parent_name ? &gate->parent_name : NULL,
		.num_parents = gate->parent_name ? 1 : 0,
	};

	gate->hw.init = &init;

	return clk_hw_register(NULL, &gate->hw);
}
