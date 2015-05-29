/*
 * Copyright 2016, Elvees NeoTek JSC
 * Copyright 2012 Tony Prisk <linux@prisktech.co.nz>
 *
 * Based on clk-vt8500.c
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>

#define to_mcom_clk_pll(_hw) container_of(_hw, struct mcom_clk_pll, hw)

/* All clocks share the same lock. It means none can be changed concurrently */
static DEFINE_SPINLOCK(mcom_clk_lock);

static void __iomem *cmctr_base;

struct mcom_clk_pll {
	struct clk_hw	hw;
	void __iomem	*reg;
	spinlock_t	*lock;
	u8 mult;
};

static inline void mcom_clk_pll_wait_for_lock(struct mcom_clk_pll *pll)
{
	u32 val;

	do {
		val = readl(pll->reg);
	} while (!(val & BIT(31)));
}

/* The only way to enable PLL is to set
 * multiplier to none zero value.
 */
static int mcom_clk_pll_enable(struct clk_hw *hw)
{
	struct mcom_clk_pll *pll = to_mcom_clk_pll(hw);
	unsigned long flags = 0;

	spin_lock_irqsave(pll->lock, flags);
	writel((pll->mult - 1), pll->reg);
	spin_unlock_irqrestore(pll->lock, flags);

	/* We should wait for PLL lock before exiting */
	mcom_clk_pll_wait_for_lock(pll);

	return 0;
}

/* Set PLL multiplier to 0 to disable PLL */
static void mcom_clk_pll_disable(struct clk_hw *hw)
{
	struct mcom_clk_pll *pll = to_mcom_clk_pll(hw);
	unsigned long flags = 0;

	spin_lock_irqsave(pll->lock, flags);
	pll->mult = (readl(pll->reg) + 1);
	writel(0, pll->reg);
	spin_unlock_irqrestore(pll->lock, flags);
}

static int mcom_clk_pll_is_enabled(struct clk_hw *hw)
{
	struct mcom_clk_pll *pll = to_mcom_clk_pll(hw);
	u32 en_val = readl(pll->reg);

	return en_val ? 1 : 0;
}

static unsigned long mcom_clk_pll_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct mcom_clk_pll *pll = to_mcom_clk_pll(hw);
	u8 mult = readl(pll->reg);

	return parent_rate * (mult + 1);
}

static long mcom_clk_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long *prate)
{
	u8 mult;

	mult = rate / *prate;

	if (!mult)
		mult = 1;

	return *prate * mult;
}

static int mcom_clk_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				 unsigned long parent_rate)
{
	struct mcom_clk_pll *pll = to_mcom_clk_pll(hw);
	u8 mult;
	unsigned long flags = 0;

	mult =  rate / parent_rate;

	spin_lock_irqsave(pll->lock, flags);
	writel((mult - 1), pll->reg);
	spin_unlock_irqrestore(pll->lock, flags);

	/* We should wait for PLL lock before exiting */
	mcom_clk_pll_wait_for_lock(pll);

	return 0;
}

static const struct clk_ops mcom_clk_pll_ops = {
	.enable = mcom_clk_pll_enable,
	.disable = mcom_clk_pll_disable,
	.is_enabled = mcom_clk_pll_is_enabled,
	.recalc_rate = mcom_clk_pll_recalc_rate,
	.round_rate = mcom_clk_pll_round_rate,
	.set_rate = mcom_clk_pll_set_rate,
};

static __init void mcom_clk_pll_init(struct device_node *node)
{
	u32 reg, clk_mult;
	struct clk *clk;
	struct mcom_clk_pll *pll;
	struct clk_init_data init;
	const char *clk_name = node->name;
	const char *parent_name = of_clk_get_parent_name(node, 0);
	int rc;

	if (!cmctr_base) {
		pr_err_once("%s: cmctr_base is not defined\n", node->name);
		return;
	}

	pll = kzalloc(sizeof(struct mcom_clk_pll), GFP_KERNEL);
	if (!pll)
		return;

	rc = of_property_read_u32(node, "reg", &reg);
	if (rc) {
		pr_err("%s: Can not read reg property\n", clk_name);
		kfree(pll);
		return;
	}

	pll->reg = cmctr_base + reg;

	rc = of_property_read_u32(node, "clk-mult-initial", &clk_mult);
	if (!rc) {
		unsigned long flags = 0;

		pll->mult = clk_mult;
		spin_lock_irqsave(&mcom_clk_lock, flags);
		writel((clk_mult - 1), pll->reg);
		/* Wait for pll lock ? */
		spin_unlock_irqrestore(&mcom_clk_lock, flags);
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	init.name = clk_name;
	init.ops = &mcom_clk_pll_ops;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;

	pll->lock = &mcom_clk_lock;
	pll->hw.init = &init;

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: Can not register clk\n", clk_name);
		kfree(pll);
		return;
	}

	rc = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (rc) {
		pr_err("%s: Can not add clk provider\n", clk_name);
		return;
	}
}

static void __init mcom_clk_gate_init(struct device_node *node)
{
	u32 reg, bit;
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name = of_clk_get_parent_name(node, 0);
	int rc;

	if (!cmctr_base) {
		pr_err_once("%s: cmctr_base is not defined\n", node->name);
		return;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	rc = of_property_read_u32(node, "reg", &reg);
	if (rc) {
		pr_err("%s: Can not read reg property\n", clk_name);
		return;
	}

	rc = of_property_read_u32(node, "reg-bit", &bit);
	if (rc) {
		pr_err("%s: Can not read reg-bit property\n", clk_name);
		return;
	}

	clk = clk_register_gate(NULL, clk_name, parent_name,
				CLK_SET_RATE_PARENT,
				cmctr_base + reg,
				bit, 0, &mcom_clk_lock);

	if (IS_ERR(clk)) {
		pr_err("%s: Can not register clk\n", clk_name);
		return;
	}

	rc = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (rc) {
		pr_err("%s: Can not add clk provider\n", clk_name);
		return;
	}
}

static __init void mcom_clk_divider_init(struct device_node *node)
{
	u32 reg, shift, width, clk_div;
	struct clk *clk;
	const char *clk_name = node->name;
	const char *parent_name = of_clk_get_parent_name(node, 0);
	int rc;

	if (!cmctr_base) {
		pr_err_once("%s: cmctr_base is not defined\n", node->name);
		return;
	}

	of_property_read_string(node, "clock-output-names", &clk_name);

	rc = of_property_read_u32(node, "reg", &reg);
	if (rc) {
		pr_err("%s: Can not read reg property\n", clk_name);
		return;
	}

	rc = of_property_read_u32(node, "reg-shift", &shift);
	if (rc) {
		pr_err("%s: Can not read reg-shift property\n", clk_name);
		return;
	}

	rc = of_property_read_u32(node, "reg-width", &width);
	if (rc) {
		pr_err("%s: Can not read reg-width property\n", clk_name);
		return;
	}

	rc = of_property_read_u32(node, "clk-div-initial", &clk_div);
	if (!rc) {
		unsigned long flags = 0;

		spin_lock_irqsave(&mcom_clk_lock, flags);
		writel(clk_div, cmctr_base + reg);
		spin_unlock_irqrestore(&mcom_clk_lock, flags);
	}

	clk = clk_register_divider(NULL, clk_name, parent_name,
			CLK_SET_RATE_PARENT,
			cmctr_base + reg, shift, width,
			CLK_DIVIDER_POWER_OF_TWO, &mcom_clk_lock);

	if (IS_ERR(clk)) {
		pr_err("%s: Can not register clk\n", clk_name);
		return;
	}

	rc = of_clk_add_provider(node, of_clk_src_simple_get, clk);
	if (rc) {
		pr_err("%s: Can not add clk provider\n", clk_name);
		return;
	}
}

static __init void mcom_cmctr_init(struct device_node *node)
{
	cmctr_base = of_iomap(node, 0);
	if (!cmctr_base)
		pr_err("%s: Can not map requested memory region\n", node->name);
}

CLK_OF_DECLARE(mcom_cmctr, "elvees,mcom-cmctr", mcom_cmctr_init);
CLK_OF_DECLARE(mcom_clk_pll, "elvees,mcom-clk-pll", mcom_clk_pll_init);
CLK_OF_DECLARE(mcom_clk_gate, "elvees,mcom-clk-gate", mcom_clk_gate_init);
CLK_OF_DECLARE(mcom_clk_divider, "elvees,mcom-clk-divider",
	       mcom_clk_divider_init);
