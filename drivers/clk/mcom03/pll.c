// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/slab.h>

#define PLL_VCO_MAX_FREQ 3600000000UL
#define PLL_VCO_MIN_FREQ 720000000UL
#define PLL_NF_MAX 4096

#define PLL_SEL GENMASK(7, 0)
#define PLL_MAN BIT(9)
#define PLL_OD GENMASK(13, 10)
#define PLL_NF GENMASK(26, 14)
#define PLL_NR GENMASK(30, 27)
#define PLL_LOCK BIT(31)

struct mcom03_pll {
	struct clk_hw hw;
	void __iomem *base;
	u8 max_nr;
	u8 nr;
	u16 nf;
	u8 od;
	bool bypass;
};

static unsigned long pll_recalc_rate(struct clk_hw *hw,
				     unsigned long parent_rate)
{
	struct mcom03_pll *pll = container_of(hw, struct mcom03_pll, hw);
	u32 reg = readl(pll->base);

	if (reg & PLL_SEL) {
		if (!(reg & PLL_LOCK)) {
			pr_warn("%s: PLL is not locked\n", clk_hw_get_name(hw));
			return 0;
		} else if (reg & PLL_MAN) {
			pll->nf = FIELD_GET(PLL_NF, reg) + 1;
			pll->nr = FIELD_GET(PLL_NR, reg) + 1;
			pll->od = FIELD_GET(PLL_OD, reg) + 1;
			return parent_rate * pll->nf / pll->nr / pll->od;
		}
	}
	return parent_rate * (FIELD_GET(PLL_SEL, reg) + 1);
}

static long _pll_round_rate(struct mcom03_pll *pll, unsigned long rate,
			    unsigned long parent_rate, bool update_param)
{
	const u8 od_list[] = {16, 14, 12, 10, 8, 6, 4, 2, 1};
	unsigned long closest_freq = parent_rate;
	unsigned long vco;
	unsigned long freq;
	unsigned long nf_frac;
	unsigned long nf;
	u16 closest_nf = 0;
	u8 nr, od, closest_nr = 0, closest_od = 0;
	int i;

	if (!parent_rate) {
		pr_err("%s: Failed to round rate, parent rate is 0\n",
		       clk_hw_get_name(&pll->hw));
		return -EINVAL;
	}

	/* Maximum PLL output rate is PLL_VCO_MAX_FREQ */
	rate = min(rate, PLL_VCO_MAX_FREQ);

	/* If bypass is used then pll_rate = parent_rate
	 * If bypass is not used then:
	 * pll_rate = VCO / OD
	 * where
	 * VCO = parent_rate * NF / NR
	 * VCO must be in range [720 MHz ... 3600 MHz]
	 * NR should be minimal to jitter minimization.
	 * VCO and OD should be maximum as possible.
	 */
	for (nr = 1; nr <= pll->max_nr; nr++) {
		for (i = 0; i < ARRAY_SIZE(od_list); i++) {
			od = od_list[i];
			nf = rate * nr * od;
			nf_frac = do_div(nf, (u32)parent_rate);
			vco = parent_rate * nf / nr;
			if (nf == 0 || vco < PLL_VCO_MIN_FREQ) {
				/* Current od is not suitable. Break the cycle
				 * since all subsequent ods in the list are less
				 * than the current one. */
				break;
			} else if (nf > PLL_NF_MAX || vco > PLL_VCO_MAX_FREQ) {
				/* Try next OD */
				continue;
			}

			if (nf_frac == 0) {
				/* Found coefficients to get rate frequency */
				if (update_param) {
					pll->nf = nf;
					pll->nr = nr;
					pll->od = od;
					pll->bypass = false;
				}

				/* Use DIV_ROUND_CLOSEST() because of
				 * rounding errors */
				return DIV_ROUND_CLOSEST(vco, od);
			}
			freq = vco / od;
			if (freq > closest_freq) {
				closest_freq = freq;
				closest_nf = nf;
				closest_nr = nr;
				closest_od = od;
			}
		}
	}
	if (update_param) {
		pll->nf = closest_nf;
		pll->nr = closest_nr;
		pll->od = closest_od;
		pll->bypass = (closest_freq == parent_rate);
	}

	return closest_freq;
}

static long pll_round_rate(struct clk_hw *hw, unsigned long rate,
			   unsigned long *parent_rate)
{
	struct mcom03_pll *pll = container_of(hw, struct mcom03_pll, hw);

	return _pll_round_rate(pll, rate, *parent_rate, false);
}

static int pll_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct mcom03_pll *pll = container_of(hw, struct mcom03_pll, hw);
	u32 reg = readl(pll->base);
	long res = _pll_round_rate(pll, rate, parent_rate, true);

	if (res < 0)
		return res;

	if (pll->bypass)
		reg = 0;
	else
		reg = FIELD_PREP(PLL_NF, pll->nf - 1) |
		      FIELD_PREP(PLL_NR, pll->nr - 1) |
		      FIELD_PREP(PLL_OD, pll->od - 1) |
		      FIELD_PREP(PLL_SEL, 1) | PLL_MAN;

	if ((readl(pll->base) & ~PLL_LOCK) == reg)
		return 0;

	writel(reg, pll->base);

	/* PLL resets LOCK bit in few clock cycles after
	 * new PLL settings are set
	 */
	udelay(1);

	if (!pll->bypass) {
		if (readl_poll_timeout(pll->base, reg, reg & PLL_LOCK, 0,
				       1000)) {
			pr_err("Failed to lock PLL %s\n", clk_hw_get_name(hw));
			return -EIO;
		}
	}

	return 0;
}

static const struct debugfs_reg32 pll_debug_regs[] = {
	{ .name = "PLLCFG", .offset = 0 },
	{ .name = "PLLDIAG", .offset = 0x4 },
};

static void pll_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct mcom03_pll *pll = container_of(hw, struct mcom03_pll, hw);
	struct debugfs_regset32 *regset;

	debugfs_create_u8("max-nr", 0400, dentry, &pll->max_nr);
	debugfs_create_u8("nr", 0400, dentry, &pll->nr);
	debugfs_create_u16("nf", 0400, dentry, &pll->nf);
	debugfs_create_u8("od", 0400, dentry, &pll->od);
	regset = kzalloc(sizeof(*regset), GFP_KERNEL);
	if (!regset)
		return;

	regset->regs = pll_debug_regs;
	regset->nregs = ARRAY_SIZE(pll_debug_regs);
	regset->base = pll->base;
	debugfs_create_regset32("registers", 0400, dentry, regset);
}

static const struct clk_ops pll_ops = {
	.recalc_rate = pll_recalc_rate,
	.round_rate = pll_round_rate,
	.set_rate = pll_set_rate,
	.debug_init = pll_debug_init,
};

static void __init mcom03_clk_pll_init(struct device_node *np)
{
	struct mcom03_pll *pll;
	struct clk_init_data init;
	struct clk *clk;
	const char *name;
	const char *parent_name = of_clk_get_parent_name(np, 0);
	int ret;

	if (!parent_name) {
		pr_err("%s: Failed to get parent clock name\n", np->name);
		return;
	}

	ret = of_property_read_string(np, "clock-output-names", &name);
	if (ret) {
		/* If property not found then use name of node */
		if (ret != -EINVAL) {
			pr_err("%s: Failed to get clock-output-names (%d)\n",
			       np->name, ret);
			return;
		}
		name = np->name;
	}

	pll = kzalloc(sizeof(*pll), GFP_KERNEL);
	if (!pll)
		return;

	ret = of_property_read_u8(np, "elvees,pll-max-nr", &pll->max_nr);
	if (!ret && (pll->max_nr < 1 || pll->max_nr > 64)) {
		pr_err("%s: Failed to get valid pll-max-nr property (%d)\n",
		       np->name, pll->max_nr);
		goto fail;
	} else if (ret)
		pll->max_nr = 1;

	init.name = name;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.ops = &pll_ops;

	pll->hw.init = &init;
	pll->base = of_iomap(np, 0);

	clk = clk_register(NULL, &pll->hw);
	if (IS_ERR(clk)) {
		pr_err("%s: Failed to register clock\n", np->name);
		goto fail;
	}

	of_clk_add_provider(np, of_clk_src_simple_get, clk);
	return;

fail:
	kfree(pll);
}

CLK_OF_DECLARE(mcom03_clk_pll, "elvees,mcom03-clk-pll", mcom03_clk_pll_init);
