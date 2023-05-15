// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 *
 */

#include <linux/clk.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#include "mcom03-clk.h"

#define XTI_FREQ 27000000
#define BP_CTR_REG 0x40
#define UCG_MAX_DIVIDER 0xfffffU

#define LPI_EN BIT(0)
#define CLK_EN BIT(1)
#define Q_FSM_STATE GENMASK(9, 7)
#define DIV_COEFF GENMASK(29, 10)
#define DIV_LOCK BIT(30)

#define Q_STOPPED 0
#define Q_RUN FIELD_PREP(Q_FSM_STATE, 0x6)

static struct mcom03_ucg_chan *to_mcom03_ucg_chan(struct clk_hw *hw)
{
	return container_of(hw, struct mcom03_ucg_chan, hw);
}

static u32 ucg_readl(struct mcom03_ucg_chan *chan, u32 reg)
{
	return readl(chan->base + reg);
}

static void ucg_writel(struct mcom03_ucg_chan *chan, u32 val, u32 reg)
{
	writel(val, chan->base + reg);
}

int mcom03_clk_ucg_chan_is_enabled(struct mcom03_ucg_chan *chan)
{
	const u32 reg = ucg_readl(chan, chan->id * sizeof(u32));

	return FIELD_GET(CLK_EN, reg);
}

int mcom03_clk_ucg_chan_set_divisor(struct mcom03_ucg_chan *chan, u32 div)
{
	int ret = 0;
	u32 reg_offset = chan->id * sizeof(u32);
	u32 value = ucg_readl(chan, reg_offset);
	u32 bp = ucg_readl(chan, BP_CTR_REG);
	const int is_enabled = value & CLK_EN;

	div = min_t(u32, div, UCG_MAX_DIVIDER);
	/* Check for divider already correct */
	if (FIELD_GET(DIV_COEFF, value) == div)
		return 0;

	/* Use bypass mode if channel is enabled */
	if (is_enabled)
		ucg_writel(chan, bp | BIT(chan->id), BP_CTR_REG);

	value &= ~DIV_COEFF;
	value |= FIELD_PREP(DIV_COEFF, div);
	ucg_writel(chan, value, reg_offset);

	if (readl_poll_timeout(chan->base + reg_offset, value,
			       value & DIV_LOCK, 0, 10000))
		ret = -EIO;

	if (is_enabled)
		ucg_writel(chan, bp & ~BIT(chan->id), BP_CTR_REG);

	if (ret)
		pr_err("Failed to lock divider %s\n",
		       clk_hw_get_name(&chan->hw));

	return ret;
}

void mcom03_clk_ucg_chan_set_bypass(struct mcom03_ucg_chan *chan, bool enable)
{
	u32 bp = ucg_readl(chan, BP_CTR_REG);
	u32 mask = enable ? (bp | BIT(chan->id)) : (bp & ~BIT(chan->id));

	ucg_writel(chan, mask, BP_CTR_REG);
}

u32 mcom03_clk_ucg_bypass_enable(void __iomem *ucg_base)
{
	u32 mask = 0;
	u32 chan_id;
	u32 val;

	for (chan_id = 0; chan_id < 16; chan_id++) {
		val = readl(ucg_base + chan_id * sizeof(u32));
		if (FIELD_GET(CLK_EN, val))
			mask |= BIT(chan_id);
	}
	val = readl(ucg_base + BP_CTR_REG);
	val |= mask;
	writel(val, ucg_base + BP_CTR_REG);

	return mask;
}

void mcom03_clk_ucg_bypass_disable(void __iomem *ucg_base, u32 mask)
{
	u32 val;

	val = readl(ucg_base + BP_CTR_REG);
	val &= ~mask;
	writel(val, ucg_base + BP_CTR_REG);
}

int mcom03_clk_ucg_chan_enable(struct mcom03_ucg_chan *ucg_chan)
{
	const u32 reg_offset = ucg_chan->id * sizeof(u32);
	u32 value = ucg_readl(ucg_chan, reg_offset);
	int res;

	value |= CLK_EN;
	value &= ~LPI_EN;
	ucg_writel(ucg_chan, value, reg_offset);
	res = readl_poll_timeout(ucg_chan->base + reg_offset, value,
				 (value & Q_FSM_STATE) == Q_RUN,
				 0, 10000);
	if (res)
		pr_err("Failed to enable clock %s\n",
		       clk_hw_get_name(&ucg_chan->hw));

	return res;
}

static int mcom03_clk_ucg_chan_enable_hw(struct clk_hw *hw)
{
	struct mcom03_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);

	return mcom03_clk_ucg_chan_enable(ucg_chan);
}

int mcom03_clk_ucg_chan_disable(struct mcom03_ucg_chan *ucg_chan)
{
	const u32 reg_offset = ucg_chan->id * sizeof(u32);
	u32 value = ucg_readl(ucg_chan, reg_offset);
	int res;

	value &= ~(LPI_EN | CLK_EN);
	ucg_writel(ucg_chan, value, reg_offset);
	res = readl_poll_timeout(ucg_chan->base + reg_offset, value,
				 (value & Q_FSM_STATE) == Q_STOPPED,
				 0, 10000);
	if (res)
		pr_err("Failed to disable clock %s\n",
		       clk_hw_get_name(&ucg_chan->hw));

	return res;
}

static void mcom03_clk_ucg_chan_disable_hw(struct clk_hw *hw)
{
	struct mcom03_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);

	mcom03_clk_ucg_chan_disable(ucg_chan);
}

static int _mcom03_clk_ucg_chan_is_enabled(struct clk_hw *hw)
{
	struct mcom03_ucg_chan *chan = to_mcom03_ucg_chan(hw);

	return mcom03_clk_ucg_chan_is_enabled(chan);
}

static unsigned long mcom03_clk_ucg_chan_recalc_rate(struct clk_hw *hw,
					unsigned long parent_rate)
{
	struct mcom03_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg = ucg_readl(ucg_chan, ucg_chan->id * sizeof(u32));
	u32 div = FIELD_GET(DIV_COEFF, reg);
	bool bp = ucg_readl(ucg_chan, BP_CTR_REG) & BIT(ucg_chan->id);

	if (!div)
		div = 1;

	return bp ? XTI_FREQ : DIV_ROUND_UP(parent_rate, div);
}

static long mcom03_clk_ucg_chan_round_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long *parent_rate)
{
	struct mcom03_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 div = ucg_chan->freq_round_up ? *parent_rate / rate :
			DIV_ROUND_UP(*parent_rate, rate);

	div = min(div, UCG_MAX_DIVIDER);

	return ucg_chan->freq_round_up ? *parent_rate / div :
			DIV_ROUND_UP(*parent_rate, div);
}

static int mcom03_clk_ucg_chan_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct mcom03_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 div = ucg_chan->freq_round_up ? parent_rate / rate :
			DIV_ROUND_UP(parent_rate, rate);

	return mcom03_clk_ucg_chan_set_divisor(ucg_chan, div);
}

static const struct clk_ops ucg_chan_ops = {
	.enable = mcom03_clk_ucg_chan_enable_hw,
	.disable = mcom03_clk_ucg_chan_disable_hw,
	.is_enabled = _mcom03_clk_ucg_chan_is_enabled,
	.recalc_rate = mcom03_clk_ucg_chan_recalc_rate,
	.round_rate = mcom03_clk_ucg_chan_round_rate,
	.set_rate = mcom03_clk_ucg_chan_set_rate,
};

static const struct clk_ops ucg_chan_fixed_ops = {
	.enable = mcom03_clk_ucg_chan_enable_hw,
	.disable = mcom03_clk_ucg_chan_disable_hw,
	.is_enabled = _mcom03_clk_ucg_chan_is_enabled,
	.recalc_rate = mcom03_clk_ucg_chan_recalc_rate,
};

static struct clk *mcom03_ucg_chan_register(unsigned int id,
					    const char *name,
					    const char *parent_name,
					    void __iomem *base,
					    u32 fixed_freq_mask,
					    u32 round_up_mask)
{

	struct mcom03_ucg_chan *chan_clk;
	struct clk_init_data init;
	struct clk *clk;

	chan_clk = kzalloc(sizeof(*chan_clk), GFP_KERNEL);
	if (!chan_clk)
		return ERR_PTR(-ENOMEM);
	init.name = name;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	if (fixed_freq_mask & BIT(id))
		init.ops = &ucg_chan_fixed_ops;
	else
		init.ops = &ucg_chan_ops;

	chan_clk->hw.init = &init;
	chan_clk->base = base;
	chan_clk->id = id;
	chan_clk->freq_round_up = round_up_mask & BIT(id);

	clk = clk_register(NULL, &chan_clk->hw);
	if (IS_ERR(clk))
		kfree(chan_clk);

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
	u32 round_up_mask = 0;
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
	of_property_read_u32(np, "elvees,round-up-mask", &round_up_mask);
	for (i = 0; i < count; i++)
		max_channel = max(max_channel, channels[i]);

	p = mcom03_clk_alloc_provider(np, max_channel + 1);
	if (!p)
		return;

	for (i = 0; i < count; i++) {
		p->clk_data.clks[channels[i]] = mcom03_ucg_chan_register(
			channels[i], names[i], parent_name, p->base,
			fixed_freq_mask, round_up_mask);
		if (IS_ERR(p->clk_data.clks[channels[i]]))
			pr_warn("%s: Failed to register clock %s: %ld\n",
				np->name, names[i],
				PTR_ERR(p->clk_data.clks[channels[i]]));
	}
	of_clk_add_provider(p->node, of_clk_src_onecell_get, &p->clk_data);
	enable_clocks(p, max_channel);
}

CLK_OF_DECLARE(mcom03_clk, "elvees,mcom03-clk-ucg", mcom03_clk_ucg_init);
