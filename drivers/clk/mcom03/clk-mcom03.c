// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#define XTI_FREQ 27000000
#define UCG_MAX_DIVIDER 0xfffffU
#define BP_CTR_REG 0x40

#define LPI_EN BIT(0)
#define CLK_EN BIT(1)
#define Q_FSM_STATE GENMASK(9, 7)
#define DIV_COEFF GENMASK(29, 10)
#define DIV_LOCK BIT(30)

#define Q_STOPPED 0
#define Q_RUN FIELD_PREP(Q_FSM_STATE, 0x6)

struct mcom03_clk_provider {
	struct device_node *node;
	void __iomem *base;
	struct clk_onecell_data clk_data;
};

struct mcom03_clk_ucg_chan {
	struct clk_hw hw;
	struct clk *parent;
	void __iomem *base;
	unsigned int id;
};

static DEFINE_SPINLOCK(mcom03_clk_dsp_lock);

static struct mcom03_clk_ucg_chan *to_mcom03_ucg_chan(struct clk_hw *hw)
{
	return container_of(hw, struct mcom03_clk_ucg_chan, hw);
}

static u32 ucg_readl(struct mcom03_clk_ucg_chan *ucg_chan, u32 reg)
{
	return readl(ucg_chan->base + reg);
}

static void ucg_writel(struct mcom03_clk_ucg_chan *ucg_chan, u32 val, u32 reg)
{
	writel(val, ucg_chan->base + reg);
}

static int ucg_chan_enable(struct clk_hw *hw)
{
	struct mcom03_clk_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg_offset = ucg_chan->id * sizeof(u32);
	u32 value = ucg_readl(ucg_chan, reg_offset);
	int res;

	value |= CLK_EN;
	value &= ~LPI_EN;
	ucg_writel(ucg_chan, value, reg_offset);
	res = readl_poll_timeout(ucg_chan->base + reg_offset, value,
				 (value & Q_FSM_STATE) == Q_RUN, 0, 10000);
	if (res)
		pr_err("Failed to enable clock %s\n", clk_hw_get_name(hw));

	return res;
}

static void ucg_chan_disable(struct clk_hw *hw)
{
	struct mcom03_clk_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg_offset = ucg_chan->id * sizeof(u32);
	u32 value = ucg_readl(ucg_chan, reg_offset);

	value &= ~(LPI_EN | CLK_EN);
	ucg_writel(ucg_chan, value, reg_offset);
	if (readl_poll_timeout(ucg_chan->base + reg_offset, value,
			       (value & Q_FSM_STATE) == Q_STOPPED, 0, 10000))
		pr_err("Failed to disable clock %s\n", clk_hw_get_name(hw));
}

static int ucg_chan_is_enabled(struct clk_hw *hw)
{
	struct mcom03_clk_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg = ucg_readl(ucg_chan, ucg_chan->id * sizeof(u32));

	return FIELD_GET(CLK_EN, reg);
}

static unsigned long ucg_chan_recalc_rate(struct clk_hw *hw,
					  unsigned long parent_rate)
{
	struct mcom03_clk_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg = ucg_readl(ucg_chan, ucg_chan->id * sizeof(u32));
	u32 div = FIELD_GET(DIV_COEFF, reg);
	bool bp = ucg_readl(ucg_chan, BP_CTR_REG) & BIT(ucg_chan->id);

	return bp ? XTI_FREQ : DIV_ROUND_UP(parent_rate, div);
}

static long ucg_chan_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	u32 div = DIV_ROUND_UP(*parent_rate, rate);

	div = min(div, UCG_MAX_DIVIDER);
	return DIV_ROUND_UP(*parent_rate, div);
}

static int ucg_chan_set_rate(struct clk_hw *hw, unsigned long rate,
			     unsigned long parent_rate)
{
	struct mcom03_clk_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg_offset = ucg_chan->id * sizeof(u32);
	u32 div = min_t(u32, DIV_ROUND_UP(parent_rate, rate), UCG_MAX_DIVIDER);
	u32 value = ucg_readl(ucg_chan, reg_offset);
	u32 bp = ucg_readl(ucg_chan, BP_CTR_REG);
	int is_enabled = value & CLK_EN;
	int ret = 0;

	/* Check for divider already correct */
	if (FIELD_GET(DIV_COEFF, value) == div)
		return 0;

	/* Use bypass mode if channel is enabled */
	if (is_enabled)
		ucg_writel(ucg_chan, bp | BIT(ucg_chan->id), BP_CTR_REG);

	value &= ~DIV_COEFF;
	value |= FIELD_PREP(DIV_COEFF, div);
	ucg_writel(ucg_chan, value, reg_offset);
	if (readl_poll_timeout(ucg_chan->base + reg_offset, value,
			       value & DIV_LOCK, 0, 10000)) {
		pr_err("Failed to lock divider %s\n", clk_hw_get_name(hw));
		ret = -EIO;
	}

	if (is_enabled)
		ucg_writel(ucg_chan, bp & ~BIT(ucg_chan->id), BP_CTR_REG);

	return ret;
}

static const struct clk_ops ucg_chan_ops = {
	.enable = ucg_chan_enable,
	.disable = ucg_chan_disable,
	.is_enabled = ucg_chan_is_enabled,
	.recalc_rate = ucg_chan_recalc_rate,
	.round_rate = ucg_chan_round_rate,
	.set_rate = ucg_chan_set_rate,
};

static const struct clk_ops ucg_chan_fixed_ops = {
	.enable = ucg_chan_enable,
	.disable = ucg_chan_disable,
	.is_enabled = ucg_chan_is_enabled,
	.recalc_rate = ucg_chan_recalc_rate,
};

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

static struct clk *mcom03_ucg_chan_register(unsigned int id,
					    const char *name,
					    const char *parent_name,
					    void __iomem *base,
					    u32 fixed_freq_mask)
{
	struct mcom03_clk_ucg_chan *ucg_chan;
	struct clk_init_data init;
	struct clk *clk;

	ucg_chan = kzalloc(sizeof(*ucg_chan), GFP_KERNEL);
	if (!ucg_chan)
		return ERR_PTR(-ENOMEM);
	init.name = name;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	if (fixed_freq_mask & BIT(id))
		init.ops = &ucg_chan_fixed_ops;
	else
		init.ops = &ucg_chan_ops;

	ucg_chan->hw.init = &init;
	ucg_chan->base = base;
	ucg_chan->id = id;

	clk = clk_register(NULL, &ucg_chan->hw);
	if (IS_ERR(clk))
		kfree(ucg_chan);

	return clk;
}

static void enable_clocks(struct mcom03_clk_provider *provider,
			  u32 max_channel)
{
	struct property *prop;
	const __be32 *p;
	u32 clk_id;
	int err;

	of_property_for_each_u32(provider->node, "enabled-clocks", prop, p,
				 clk_id) {
		struct clk *clk;

		if (clk_id > max_channel) {
			pr_err("Unknown clock channel %d\n", clk_id);
			continue;
		}
		clk = provider->clk_data.clks[clk_id];
		if (IS_ERR(clk))
			continue;
		err = clk_prepare_enable(clk);
		if (err)
			pr_err("Failed to enable clock %s : %d\n",
			       __clk_get_name(clk), err);
	}
}

static void __init mcom03_clk_ucg_init(struct device_node *np)
{
	struct mcom03_clk_provider *p;
	u32 channels[16];
	u32 max_channel = 0;
	u32 fixed_freq_mask = 0;
	const char *names[16];
	const char *parent_name = of_clk_get_parent_name(np, 0);
	int count;
	int ret;
	int i;

	if (!parent_name) {
		pr_err("%s: Failed to get parent clock name\n", np->name);
		return;
	}

	count = of_property_count_strings(np, "clock-output-names");
	if (count < 0) {
		pr_err("%s: Failed to get clock-output-names (%d)\n",
		       np->name, count);
		return;
	}
	ret = of_property_count_elems_of_size(np, "clock-indices",
					      sizeof(u32));
	if (count > 16 || ret > 16) {
		pr_err("%s: Maximum count of clock-output-names and clock-indices is 16, but found %d\n",
		       np->name, count);
		return;
	}
	if (ret != count && ret != -EINVAL) {
		pr_err("%s: Length of clock-output-names and clock-indices must be equal\n",
		       np->name);
		return;
	}

	ret = of_property_read_u32_array(np, "clock-indices", channels, count);
	if (ret == -EINVAL) {
		/* Channels numbers is linear from zero if clock-indices is
		 * not specified */
		for (i = 0; i < count; i++)
			channels[i] = i;
	} else if (ret) {
		pr_err("%s: Failed to get clock-indices (%d)\n", np->name,
		       ret);
		return;
	}

	ret = of_property_read_string_array(np, "clock-output-names", names,
					    count);
	if (ret < 0) {
		pr_err("%s: Failed to get clock-output-names (%d)\n",
		       np->name, ret);
		return;
	}
	of_property_read_u32(np, "elvees,fixed-freq-mask", &fixed_freq_mask);
	for (i = 0; i < count; i++)
		max_channel = max(max_channel, channels[i]);

	p = mcom03_clk_alloc_provider(np, max_channel + 1);
	if (!p)
		return;

	for (i = 0; i < count; i++) {
		p->clk_data.clks[channels[i]] = mcom03_ucg_chan_register(
			channels[i], names[i], parent_name, p->base,
			fixed_freq_mask);
		if (IS_ERR(p->clk_data.clks[channels[i]]))
			pr_warn("%s: Failed to register clock %s: %ld\n",
				np->name, names[i],
				PTR_ERR(p->clk_data.clks[channels[i]]));
	}
	of_clk_add_provider(p->node, of_clk_src_onecell_get, &p->clk_data);
	enable_clocks(p, max_channel);
}

CLK_OF_DECLARE(mcom03_clk, "elvees,mcom03-clk-ucg", mcom03_clk_ucg_init);

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
