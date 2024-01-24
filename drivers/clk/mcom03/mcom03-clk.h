/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2023 RnD Center "ELVEES", JSC
 */

#ifndef __MCOM03_CLK_H
#define __MCOM03_CLK_H

#include <linux/of.h>
#include <linux/clk-provider.h>

struct mcom03_clk_provider {
	struct device_node *node;
	void __iomem *base;
	struct clk_hw_onecell_data *clk_data;
};

struct mcom03_clk_provider *mcom03_clk_alloc_provider(struct device_node *node,
						      int count);

struct mcom03_ucg_chan {
	unsigned int id;
	void __iomem *base;
	struct clk_hw hw;
	struct clk *parent;
	bool freq_round_up;
};

int mcom03_clk_ucg_chan_enable(struct mcom03_ucg_chan *ucg_chan);
int mcom03_clk_ucg_chan_disable(struct mcom03_ucg_chan *ucg_chan);
u32 mcom03_clk_ucg_bypass_enable(void __iomem *ucg_base);
void mcom03_clk_ucg_bypass_disable(void __iomem *ucg_base, u32 mask);
void mcom03_clk_ucg_chan_set_bypass(struct mcom03_ucg_chan *chan, bool enable);
int mcom03_clk_ucg_chan_is_enabled(struct mcom03_ucg_chan *chan);
int mcom03_clk_ucg_chan_set_divisor(struct mcom03_ucg_chan *chan, u32 div);

#endif /* __MCOM03_CLK_H */
