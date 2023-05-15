// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 */

#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/of_address.h>
#include <linux/regmap.h>

#include "mcom03-clk.h"

/* offset of SDR_DSP_CTL register in SDR_URB */
#define SDR_DSP_CTL 0x4c

/* index of DSP_CLK_EN bit in SDR_DSP_CTL register */
#define SDR_DSP_CTL_DSP0_CLK_EN_BIT 8

struct mcom03_clk_gate {
	struct clk_hw hw;
	struct regmap *sdr_urb;
	spinlock_t *lock;
	unsigned int reg;
	u8 bit_idx;
};

#define to_mcom03_gate(_hw) container_of(_hw, struct mcom03_clk_gate, hw)

static DEFINE_SPINLOCK(mcom03_clk_dsp_lock);

static void mcom03_clk_gate_set(struct mcom03_clk_gate *g, int enable)
{
	unsigned long flags;
	u32 value;

	spin_lock_irqsave(g->lock, flags);
	regmap_read(g->sdr_urb, g->reg, &value);
	if (enable)
		value |= BIT(g->bit_idx);
	else
		value &= ~BIT(g->bit_idx);

	regmap_write(g->sdr_urb, g->reg, value);
	spin_unlock_irqrestore(g->lock, flags);
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


static struct clk *mcom03_clk_gate_register(const char *name,
					    const char *parent_name,
					    struct regmap *urb,
					    unsigned int reg,
					    u8 bit_idx,
					    spinlock_t *lock)
{
	struct clk *clk;
	struct clk_init_data init;
	struct mcom03_clk_gate *g;

	g = kzalloc(sizeof(*g), GFP_KERNEL);
	if (!g)
		return NULL;

	init.name = name;
	init.ops = &mcom03_clk_gate_ops;
	init.flags = CLK_SET_RATE_PARENT;
	init.parent_names = parent_name ? &parent_name : NULL;
	init.num_parents = parent_name ? 1 : 0;
	g->sdr_urb = urb;
	g->reg = reg;
	g->bit_idx = bit_idx;
	g->lock = lock;
	g->hw.init = &init;
	clk = clk_register(NULL, &g->hw);
	if (IS_ERR(clk)) {
		kfree(g);
		return NULL;
	}

	return clk;
}

struct regmap_config urb_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.cache_type = REGCACHE_NONE,
	.max_register = SDR_DSP_CTL,
};

static void __init mcom03_clk_dsp_gate_init(struct device_node *np)
{
	struct mcom03_clk_provider *p;
	const char *clk_names[2];
	const char *parent_name = of_clk_get_parent_name(np, 0);
	void __iomem *base;
	struct regmap *urb;
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

	urb = regmap_init_mmio(NULL, base, &urb_regmap_config);
	if (IS_ERR(urb)) {
		pr_err("%s: failed to regmap mmio region: %ld\n", __func__,
		       PTR_ERR(urb));
		goto err_iounmap;
	}

	p = mcom03_clk_alloc_provider(np, 2);
	if (!p)
		goto err_regmap_exit;

	for (i = 0; i < 2; i++) {
		p->clk_data.clks[i] = mcom03_clk_gate_register(clk_names[i],
							       parent_name,
							       urb,
							       SDR_DSP_CTL,
							       SDR_DSP_CTL_DSP0_CLK_EN_BIT + i,
							       &mcom03_clk_dsp_lock);
		if (IS_ERR(p->clk_data.clks[i])) {
			pr_err("%s: Failed to register gate (%ld)\n",
			       clk_names[i], PTR_ERR(p->clk_data.clks[i]));
			goto err_unregister;
		}
	}

	ret = of_clk_add_provider(p->node, of_clk_src_onecell_get,
				  &p->clk_data);
	if (ret >= 0)
		return;

	pr_err("%s: Failed to add clk provider (%d)\n", np->name, ret);
err_unregister:
	for (i = 0; i < 2; i++) {
		if (!IS_ERR_OR_NULL(p->clk_data.clks[i]))
			clk_unregister(p->clk_data.clks[i]);
	}
	kfree(p->clk_data.clks);
	kfree(p);
err_regmap_exit:
	regmap_exit(urb);
err_iounmap:
	iounmap(base);
}

CLK_OF_DECLARE(mcom03_clk_dsp_gate, "elvees,mcom03-dsp-gate",
	       mcom03_clk_dsp_gate_init);
