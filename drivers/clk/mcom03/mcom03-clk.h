/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2023-2025 RnD Center "ELVEES", JSC
 */

#ifndef __MCOM03_CLK_H
#define __MCOM03_CLK_H

#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm_domain.h>
#include <linux/regmap.h>
#include <linux/clk-provider.h>

#define mcom03_clk_pd_lock(pd) \
	do { \
		if (pd) \
			mutex_lock(&pd->lock); \
	} while (0)

#define mcom03_clk_pd_unlock(pd) \
	do { \
		if (pd) \
			mutex_unlock(&pd->lock); \
	} while (0)

struct mcom03_clk_provider;

struct mcom03_subsystem_clk {
	struct mcom03_pll *plls;
	u32 nr_plls;

	struct mcom03_clk_refmux *refmuxes;
	u32 nr_refmuxes;

	struct mcom03_ucg_chan *ucg_chans;
	u32 nr_ucg_chans;
	u32 max_ucg_id;

	struct mcom03_clk_gate *gates;
	u32 nr_gates;

	u32 nr_clocks;

	/* Some actions needed to during power-on or bootup
	 * not related to PLL or UCG, needed to init subsystem
	 */
	void (*init)(struct mcom03_clk_provider *unit);
};

struct mcom03_pm_domain {
	struct generic_pm_domain genpd;
	struct regmap *service_subs_urb;
	struct mcom03_clk_provider *clk_provider;

	/* lock is used to prevent changing is_enabled while functions are
	 * accessing to registers */
	struct mutex lock;

	/* offset in URB mmio to target PPOLICY register */
	u16 offset;
	u32 id;
	bool is_enabled;
	bool is_supported;
};

struct mcom03_ucg_chan {
	u32 clk_id;
	u32 ucg_id;
	unsigned int chan_id;
	char *name;
	const char *parent_name;
	bool is_fixed;
	bool freq_round_up;
	bool ignore_unused;  /* Do not disable channel even it is not used */
	void __iomem *base;
	struct clk_hw hw;
	struct clk *parent;
	struct mcom03_pm_domain *pd;
	u32 divisor;
	bool is_enabled;
};

struct mcom03_pll {
	u32 clk_id;
	char *name;
	u32 offset;
	u8 max_nr;
	u32 ucg_ids_mask;  /* Mask where each bit is ID of children UCG */
	u32 ucg_count;  /* Count of children UCGs */
	u32 *ucg_ids;  /* IDs of children UCGs */
	u16 *ucg_bypass;  /* Variable to save bypassed channels */
	struct mcom03_subsystem_clk *sclk;
	struct clk_hw hw;
	struct regmap *regmap;
	struct mcom03_pm_domain *pd;
	u8 nr;
	u16 nf;
	u8 od;
	u8 sel;
	u8 man;
	bool bypass;
};

struct mcom03_clk_gate {
	u32 clk_id;
	char *name;
	const char *parent_name;
	unsigned int reg;
	u8 bit_idx;
	struct clk_hw hw;
	struct regmap *sdr_urb;
	struct mcom03_pm_domain *pd;
	bool is_enabled;
};

struct mcom03_clk_refmux {
	u32 clk_id;
	u32 ucg_id;
	u32 offset;
	u32 shift;
	u32 mask;
	char *name;
	struct clk_hw hw;
	struct regmap *regmap;
	void __iomem *base_ucg;
	struct mcom03_pm_domain *pd;
	u8 index;
};

void mcom03_clk_restore(struct mcom03_clk_provider *prov);
int mcom03_clk_pll_update_regs_nolock(struct mcom03_pll *pll);
int mcom03_clk_mux_set_parent_nolock(struct clk_hw *hw, u8 index);
int mcom03_clk_gate_set_nolock(struct mcom03_clk_gate *g);
int mcom03_clk_ucg_chan_enable_nolock(struct mcom03_ucg_chan *ucg_chan);
u32 mcom03_clk_ucg_bypass_enable_nolock(void __iomem *ucg_base);
void mcom03_clk_ucg_bypass_disable_nolock(void __iomem *ucg_base, u32 mask);
void mcom03_clk_ucg_chan_set_bypass_nolock(struct mcom03_ucg_chan *chan, bool enable);
int mcom03_clk_ucg_chan_is_enabled_nolock(struct mcom03_ucg_chan *chan);
int mcom03_clk_ucg_chan_set_divisor_nolock(struct mcom03_ucg_chan *chan,
					   bool use_bypass);
int mcom03_clk_ucg_chan_update_divisor_nolock(struct mcom03_ucg_chan *ucg_chan,
					      unsigned long parent_rate);

int mcom03_clk_pll_register(const char *parent_name, struct mcom03_pll *pll);
int mcom03_clk_gate_register(struct mcom03_clk_gate *gate);
int mcom03_clk_refmux_register(struct mcom03_clk_refmux *refmux,
			       const char **parent_names,
			       u32 parent_count);
int mcom03_ucg_chan_register(struct mcom03_ucg_chan *ucg_chan);

struct mcom03_pm_domain *mcom03_power_domain_init(struct device_node *node,
						  u32 id);

void mcom03_of_clks_enable(struct device_node *np,
			   struct clk_hw_onecell_data *clk_data);

#endif /* __MCOM03_CLK_H */
