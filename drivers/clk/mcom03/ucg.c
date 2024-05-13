// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2023-2024 RnD Center "ELVEES", JSC
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

int mcom03_clk_ucg_chan_set_divisor(struct mcom03_ucg_chan *chan, u32 div,
				    bool use_bypass)
{
	int ret = 0;
	u32 reg_offset = chan->chan_id * sizeof(u32);
	u32 value = ucg_readl(chan, reg_offset);
	u32 bp = ucg_readl(chan, BP_CTR_REG);
	const int is_enabled = value & CLK_EN;

	div = min_t(u32, div, UCG_MAX_DIVIDER);
	/* Check for divider already correct */
	if (FIELD_GET(DIV_COEFF, value) == div)
		return 0;

	/* Use bypass mode if channel is enabled */
	if (use_bypass && is_enabled)
		ucg_writel(chan, bp | BIT(chan->chan_id), BP_CTR_REG);

	value &= ~DIV_COEFF;
	value |= FIELD_PREP(DIV_COEFF, div);
	ucg_writel(chan, value, reg_offset);

	if (readl_poll_timeout(chan->base + reg_offset, value,
			       value & DIV_LOCK, 0, 10000))
		ret = -EIO;

	if (use_bypass && is_enabled)
		ucg_writel(chan, bp & ~BIT(chan->chan_id), BP_CTR_REG);

	if (ret)
		pr_err("Failed to lock divider %s\n",
		       clk_hw_get_name(&chan->hw));

	return ret;
}

/* Update divisor to save rate with new parent_rate */
int mcom03_clk_ucg_chan_update_divisor(struct mcom03_ucg_chan *ucg_chan,
				       unsigned long parent_rate)
{
	unsigned long rate = clk_hw_get_rate(&ucg_chan->hw);
	u32 div = ucg_chan->freq_round_up ? parent_rate / rate :
			DIV_ROUND_UP(parent_rate, rate);

	if (ucg_chan->is_fixed)
		return 0;

	return mcom03_clk_ucg_chan_set_divisor(ucg_chan, div, false);
}

void mcom03_clk_ucg_chan_set_bypass(struct mcom03_ucg_chan *chan, bool enable)
{
	u32 bp = ucg_readl(chan, BP_CTR_REG);
	u32 mask = enable ? (bp | BIT(chan->chan_id)) : (bp & ~BIT(chan->chan_id));

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
	const u32 reg_offset = ucg_chan->chan_id * sizeof(u32);
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
	const u32 reg_offset = ucg_chan->chan_id * sizeof(u32);
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

int mcom03_clk_ucg_chan_is_enabled(struct mcom03_ucg_chan *chan)
{
	const u32 reg = ucg_readl(chan, chan->chan_id * sizeof(u32));

	return FIELD_GET(CLK_EN, reg);
}

static int mcom03_clk_ucg_chan_is_enabled_hw(struct clk_hw *hw)
{
	struct mcom03_ucg_chan *chan = to_mcom03_ucg_chan(hw);

	return mcom03_clk_ucg_chan_is_enabled(chan);
}

static unsigned long mcom03_clk_ucg_chan_recalc_rate(struct clk_hw *hw,
						     unsigned long parent_rate)
{
	struct mcom03_ucg_chan *ucg_chan = to_mcom03_ucg_chan(hw);
	u32 reg = ucg_readl(ucg_chan, ucg_chan->chan_id * sizeof(u32));
	u32 div = FIELD_GET(DIV_COEFF, reg);
	bool bp = ucg_readl(ucg_chan, BP_CTR_REG) & BIT(ucg_chan->chan_id);

	/* Linux call this callback with incorrect parent_rate when setting new
	 * rate for PLL (specify new parent_rate before changing PLL rate).
	 * In this case this function will return incorrect rate and Linux will
	 * save it to cache. After complete of PLL rate change Linux will call
	 * set_rate with saved (incorrect) value.
	 * Use real parent rate to prevent this.
	 */
	parent_rate = clk_get_rate(clk_get_parent(hw->clk));
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

	return mcom03_clk_ucg_chan_set_divisor(ucg_chan, div, true);
}

static const struct clk_ops ucg_chan_ops = {
	.enable = mcom03_clk_ucg_chan_enable_hw,
	.disable = mcom03_clk_ucg_chan_disable_hw,
	.is_enabled = mcom03_clk_ucg_chan_is_enabled_hw,
	.recalc_rate = mcom03_clk_ucg_chan_recalc_rate,
	.round_rate = mcom03_clk_ucg_chan_round_rate,
	.set_rate = mcom03_clk_ucg_chan_set_rate,
};

static const struct clk_ops ucg_chan_fixed_ops = {
	.enable = mcom03_clk_ucg_chan_enable_hw,
	.disable = mcom03_clk_ucg_chan_disable_hw,
	.is_enabled = mcom03_clk_ucg_chan_is_enabled_hw,
	.recalc_rate = mcom03_clk_ucg_chan_recalc_rate,
};

int mcom03_ucg_chan_register(struct mcom03_ucg_chan *ucg_chan)
{

	struct clk_init_data init = {
		.name = ucg_chan->name,
		.flags = 0,
		.parent_names = &ucg_chan->parent_name,
		.num_parents = 1,
		.flags = ucg_chan->ignore_unused ? CLK_IGNORE_UNUSED : 0,
		.ops = ucg_chan->is_fixed ? &ucg_chan_fixed_ops : &ucg_chan_ops,
	};

	ucg_chan->hw.init = &init;

	return clk_hw_register(NULL, &ucg_chan->hw);
}

void mcom03_of_clks_enable(struct device_node *np,
			   struct clk_hw_onecell_data *clk_data)
{
	struct property *prop;
	const __be32 *p;
	u32 clk_id;
	int err;

	of_property_for_each_u32(np, "enabled-clocks", prop, p, clk_id) {
		struct clk_hw *hw;

		if (clk_id > clk_data->num) {
			pr_err("Unknown clock channel %d\n", clk_id);
			continue;
		}
		hw = clk_data->hws[clk_id];
		if (IS_ERR(hw))
			continue;

		err = clk_prepare_enable(hw->clk);
		if (err)
			pr_err("Failed to enable clock %s : %d\n",
			       clk_hw_get_name(hw), err);
	}
}
