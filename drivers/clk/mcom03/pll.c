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

#include "mcom03-clk.h"

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

/* Find first ucg_chan with requested ucg_id. */
static struct mcom03_ucg_chan *find_ucg_chan_by_ucg_id(struct mcom03_pll *pll,
						       u32 ucg_id)
{
	struct mcom03_ucg_chan *ucg_chan = pll->sclk->ucg_chans;
	int i;

	for (i = 0; i < pll->sclk->nr_ucg_chans; i++) {
		if (ucg_chan[i].ucg_id == ucg_id)
			return &ucg_chan[i];
	}

	return NULL;
}

static bool is_child_of_pll(struct clk *clk, struct clk *pll_clk)
{
	while (clk) {
		clk = clk_get_parent(clk);
		if (clk == pll_clk)
			return true;
	}

	return false;
}

/* Enable bypass for all children UCGs. */
static void pll_children_bypass_enable(struct mcom03_pll *pll)
{
	struct mcom03_ucg_chan *ucg_chan;
	int i;

	for (i = 0; i < pll->ucg_count; i++) {
		ucg_chan = find_ucg_chan_by_ucg_id(pll, pll->ucg_ids[i]);
		if (ucg_chan && !ucg_chan->is_fixed &&
		    is_child_of_pll(ucg_chan->hw.clk, pll->hw.clk))
			pll->ucg_bypass[i] = mcom03_clk_ucg_bypass_enable(ucg_chan->base);
		else
			pll->ucg_bypass[i] = 0;
	}
}

/* Update dividers to respect new PLL rate and disable UCG
 * bypass for all children UCGs.
 */
static void pll_children_bypass_disable(struct mcom03_pll *pll,
					unsigned long pll_rate)
{
	struct mcom03_ucg_chan *ucg_chan;
	void __iomem *base;
	u32 ucg_id;
	int i, j;

	for (i = 0; i < pll->ucg_count; i++) {
		base = NULL;
		ucg_id = pll->ucg_ids[i];
		for (j = 0; j < pll->sclk->nr_ucg_chans; j++) {
			ucg_chan = &pll->sclk->ucg_chans[j];

			if (ucg_chan->ucg_id == ucg_id)
				base = ucg_chan->base;

			/* Update divider is required here only for enabled
			 * channels (if channel bypass is enabled). Linux will
			 * restore rate for all children clocks later after
			 * return to clk_change_rate().
			 */
			if ((ucg_chan->ucg_id == ucg_id) &&
			    (pll->ucg_bypass[i] & BIT(ucg_chan->chan_id)))
				mcom03_clk_ucg_chan_update_divisor(ucg_chan,
								   pll_rate);
		}
		if (base)
			mcom03_clk_ucg_bypass_disable(base, pll->ucg_bypass[i]);
	}
}

static int pll_set_rate(struct clk_hw *hw, unsigned long rate,
			unsigned long parent_rate)
{
	struct mcom03_pll *pll = container_of(hw, struct mcom03_pll, hw);
	u32 reg, val;
	long result_rate = _pll_round_rate(pll, rate, parent_rate, true);
	int ret = 0;

	if (result_rate < 0)
		return result_rate;

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

	/* Before changing PLL rate we need to enable bypass for all enabled
	 * channels for all children UCGs. It is required to prevent high
	 * frequency on UCG output after increase PLL rate.
	 * After PLL rate changed we need to recalc dividers for UCGs and
	 * disable bypass.
	 */
	pll_children_bypass_enable(pll);

	regmap_write(pll->regmap, pll->offset + REG_PLLCFG, reg);

	/* PLL resets LOCK bit in few clock cycles after
	 * new PLL settings are set
	 */
	udelay(1);

	if (!pll->bypass) {
		if (regmap_read_poll_timeout(pll->regmap, pll->offset + REG_PLLCFG,
			reg, reg & PLL_LOCK, 0, 1000)) {
			pr_err("Failed to lock PLL %s\n", clk_hw_get_name(hw));
			ret = -EIO;
		}
	}

	pll_children_bypass_disable(pll, result_rate);

	return ret;
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

int mcom03_clk_pll_register(const char *parent_name, struct mcom03_pll *pll)
{
	struct clk_init_data init = {
		.name = pll->name,
		.flags = 0,
		.parent_names = &parent_name,
		.num_parents = 1,
		.ops = &pll_ops,
	};

	pll->hw.init = &init;

	return clk_hw_register(NULL, &pll->hw);
}
