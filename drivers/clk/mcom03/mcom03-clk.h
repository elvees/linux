/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2023-2024 RnD Center "ELVEES", JSC
 */

#ifndef __MCOM03_CLK_H
#define __MCOM03_CLK_H

#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/clk-provider.h>

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
	u8 nr;
	u16 nf;
	u8 od;
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
};

int mcom03_clk_ucg_chan_enable(struct mcom03_ucg_chan *ucg_chan);
int mcom03_clk_ucg_chan_disable(struct mcom03_ucg_chan *ucg_chan);
u32 mcom03_clk_ucg_bypass_enable(void __iomem *ucg_base);
void mcom03_clk_ucg_bypass_disable(void __iomem *ucg_base, u32 mask);
void mcom03_clk_ucg_chan_set_bypass(struct mcom03_ucg_chan *chan, bool enable);
int mcom03_clk_ucg_chan_is_enabled(struct mcom03_ucg_chan *chan);
int mcom03_clk_ucg_chan_set_divisor(struct mcom03_ucg_chan *chan, u32 div,
				    bool use_bypass);
int mcom03_clk_ucg_chan_update_divisor(struct mcom03_ucg_chan *ucg_chan,
				       unsigned long parent_rate);

int mcom03_clk_pll_register(const char *parent_name, struct mcom03_pll *pll);
int mcom03_clk_gate_register(struct mcom03_clk_gate *gate);
int mcom03_clk_refmux_register(struct mcom03_clk_refmux *refmux,
			       const char **parent_names, u32 parent_count);
int mcom03_ucg_chan_register(struct mcom03_ucg_chan *ucg_chan);

void mcom03_of_clks_enable(struct device_node *np,
			   struct clk_hw_onecell_data *clk_data);

#endif /* __MCOM03_CLK_H */
