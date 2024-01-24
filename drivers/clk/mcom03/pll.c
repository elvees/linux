// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022-2024 RnD Center "ELVEES", JSC
 *
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/debugfs.h>
#include <linux/kernel.h>
#include <linux/of_address.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
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

#define REG_PLLCFG 0
#define REG_PLLDIAG 0x4

struct mcom03_pll {
	struct clk_hw hw;
	struct regmap *regmap;  /* URB region mapping */
	u32 offset;  /* offset to PLLCFG register */
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
	u32 reg;

	regmap_read(pll->regmap, pll->offset + REG_PLLCFG, &reg);

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
	u32 reg, val;
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

	regmap_read(pll->regmap, pll->offset + REG_PLLCFG, &val);
	if ((val & ~PLL_LOCK) == reg)
		return 0;

	regmap_write(pll->regmap, pll->offset + REG_PLLCFG, reg);

	/* PLL resets LOCK bit in few clock cycles after
	 * new PLL settings are set
	 */
	udelay(1);

	if (!pll->bypass) {
		if (regmap_read_poll_timeout(pll->regmap, pll->offset + REG_PLLCFG,
			reg, reg & PLL_LOCK, 0, 1000)) {
			pr_err("Failed to lock PLL %s\n", clk_hw_get_name(hw));
			return -EIO;
		}
	}

	return 0;
}

static void pll_debug_init(struct clk_hw *hw, struct dentry *dentry)
{
	struct mcom03_pll *pll = container_of(hw, struct mcom03_pll, hw);

	debugfs_create_u8("max-nr", 0400, dentry, &pll->max_nr);
	debugfs_create_u8("nr", 0400, dentry, &pll->nr);
	debugfs_create_u16("nf", 0400, dentry, &pll->nf);
	debugfs_create_u8("od", 0400, dentry, &pll->od);
	/* pll->regmap is also present in /sys/kernel/debug/regmap/ */
}

static const struct clk_ops pll_ops = {
	.recalc_rate = pll_recalc_rate,
	.round_rate = pll_round_rate,
	.set_rate = pll_set_rate,
	.debug_init = pll_debug_init,
};

static struct regmap * __init mcom03_clk_pll_configure_regmap(struct device_node *np,
							      void __iomem *pll_base)
{
	struct regmap *regmap;
	struct regmap_config pll_regmap_config = {
		.reg_bits = 32,
		.val_bits = 32,
		.reg_stride = 4,
		.cache_type = REGCACHE_NONE,
		.max_register = REG_PLLDIAG,
		.name = np->name,
	};

	regmap = regmap_init_mmio(NULL, pll_base, &pll_regmap_config);
	if (IS_ERR(regmap)) {
		pr_err("%s: failed to regmap mmio region: %ld\n", np->name,
			PTR_ERR(regmap));
		return ERR_PTR(-EINVAL);
	}

	return regmap;
}

struct clk_hw * __init mcom03_clk_pll_register(const char *parent_name,
					    const char *name,
					    struct regmap *urb,
					    u32 offset,
					    int max_nr)
{
	struct clk_init_data init;
	int ret;

	struct mcom03_pll *pll = kzalloc(sizeof(*pll), GFP_KERNEL);

	if (!pll)
		return ERR_PTR(-ENOMEM);

	init.name = name;
	init.flags = 0;
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.ops = &pll_ops;

	pll->hw.init = &init;
	pll->max_nr = max_nr;
	pll->regmap = urb;
	pll->offset = offset;

	ret = clk_hw_register(NULL, &pll->hw);
	if (ret) {
		pr_err("%s: Failed to register clock\n", name);
		kfree(pll);
	}

	return &pll->hw;
}

static void __init mcom03_clk_pll_init(struct device_node *np)
{
	struct clk_hw *hw;
	struct regmap *regmap;
	const char *name;
	const char *parent_name = of_clk_get_parent_name(np, 0);
	void __iomem *pll_base;
	int ret;
	u8 max_nr;

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

	ret = of_property_read_u8(np, "elvees,pll-max-nr", &max_nr);
	if (!ret && (max_nr < 1 || max_nr > 64)) {
		pr_err("%s: Failed to get valid pll-max-nr property (%d)\n",
		       np->name, max_nr);
		return;
	} else if (ret)
		max_nr = 1;

	pll_base = of_iomap(np, 0);
	if (!pll_base) {
		pr_err("%s: Unable to map pll base\n", np->name);
		return;
	}

	regmap = mcom03_clk_pll_configure_regmap(np, pll_base);
	if (IS_ERR(regmap)) {
		pr_err("%s: Failed to configure regmap\n", np->name);
		iounmap(pll_base);
		return;
	}

	hw = mcom03_clk_pll_register(parent_name, np->name,
				     regmap, 0, max_nr);
	if (IS_ERR(hw)) {
		pr_err("%s: Failed to register clock\n", np->name);
		goto fail;
	}

	of_clk_add_hw_provider(np, of_clk_hw_simple_get, hw);
	return;

fail:
	regmap_exit(regmap);
	iounmap(pll_base);
}

CLK_OF_DECLARE(mcom03_clk_pll, "elvees,mcom03-clk-pll", mcom03_clk_pll_init);
