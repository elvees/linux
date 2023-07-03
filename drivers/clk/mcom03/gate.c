// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023-2024 RnD Center "ELVEES", JSC
 *
 */

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include "mcom03-clk.h"

#define to_mcom03_gate(_hw) container_of(_hw, struct mcom03_clk_gate, hw)

static DEFINE_SPINLOCK(mcom03_clk_gate_lock);

static void mcom03_clk_gate_set(struct mcom03_clk_gate *g, int enable)
{
	unsigned long flags;

	spin_lock_irqsave(&mcom03_clk_gate_lock, flags);
	regmap_update_bits(g->sdr_urb,
			   g->reg,
			   BIT(g->bit_idx),
			   enable ? BIT(g->bit_idx) : 0);
	spin_unlock_irqrestore(&mcom03_clk_gate_lock, flags);
}

static int mcom03_clk_gate_enable(struct clk_hw *hw)
{
	struct mcom03_clk_gate *g = to_mcom03_gate(hw);

	mcom03_clk_gate_set(g, 1);

	return 0;
}

static void mcom03_clk_gate_disable(struct clk_hw *hw)
{
	struct mcom03_clk_gate *g = to_mcom03_gate(hw);

	mcom03_clk_gate_set(g, 0);
}

static int mcom03_clk_gate_is_enabled(struct clk_hw *hw)
{
	u32 value;
	struct mcom03_clk_gate *g = to_mcom03_gate(hw);

	regmap_read(g->sdr_urb, g->reg, &value);
	value &= BIT(g->bit_idx);

	return value ? 1 : 0;
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
