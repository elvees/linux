// SPDX-License-Identifier: GPL-2.0

// Copyright 2023-2025 RnD Center "ELVEES", JSC

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "mcom03-clk.h"

static struct mcom03_clk_refmux *to_mcom03_refmux(struct clk_hw *hw)
{
	return container_of(hw, struct mcom03_clk_refmux, hw);
}

int mcom03_clk_mux_set_parent_nolock(struct clk_hw *hw, u8 index)
{
	struct mcom03_clk_refmux *refmux = to_mcom03_refmux(hw);
	u32 val;
	u32 old_bp;

	refmux->index = index;
	if (!refmux->pd || refmux->pd->is_enabled) {
		/* Ref MUX is not glitch free. All enabled channels must be
		 * turned to bypass before ref will be changed.
		 */
		val = clk_mux_index_to_val(NULL, 0, index);
		val = val << refmux->shift;
		old_bp = mcom03_clk_ucg_bypass_enable_nolock(refmux->base_ucg);
		regmap_update_bits(refmux->regmap,
				   refmux->offset,
				   refmux->mask << refmux->shift,
				   val);

		mcom03_clk_ucg_bypass_disable_nolock(refmux->base_ucg, old_bp);
	}

	return 0;
}

static int mcom03_clk_mux_set_parent(struct clk_hw *hw, u8 index)
{
	struct mcom03_clk_refmux *refmux = to_mcom03_refmux(hw);
	int ret;

	mcom03_clk_pd_lock(refmux->pd);
	ret = mcom03_clk_mux_set_parent_nolock(hw, index);
	mcom03_clk_pd_unlock(refmux->pd);

	return ret;
}

u8 mcom03_clk_mux_get_parent(struct clk_hw *hw)
{
	struct mcom03_clk_refmux *refmux = to_mcom03_refmux(hw);
	u32 val;

	mcom03_clk_pd_lock(refmux->pd);
	if (!refmux->pd || refmux->pd->is_enabled) {
		regmap_read(refmux->regmap, refmux->offset, &val);
		val >>= refmux->shift;
		val &= refmux->mask;
		refmux->index = clk_mux_val_to_index(hw, NULL, 0, val);
	}

	mcom03_clk_pd_unlock(refmux->pd);

	return refmux->index;
}

static int mcom03_clk_mux_determine_rate(struct clk_hw *hw,
					 struct clk_rate_request *req)
{
	return clk_mux_determine_rate_flags(hw, req, 0);
}

static const struct clk_ops mcom03_clk_mux_ops = {
	.get_parent = mcom03_clk_mux_get_parent,
	.set_parent = mcom03_clk_mux_set_parent,
	.determine_rate = mcom03_clk_mux_determine_rate,
};

int mcom03_clk_refmux_register(struct mcom03_clk_refmux *refmux,
			       const char **parent_names, u32 parent_count)
{
	struct clk_init_data init = {
		.ops = &mcom03_clk_mux_ops,
		.name = refmux->name,
		.parent_names = parent_names,
		.num_parents = parent_count,
	};

	refmux->hw.init = &init;

	return clk_hw_register(NULL, &refmux->hw);
}
