
#ifndef VPOUTFB_H
#define VPOUTFB_H

#include <linux/clk-provider.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_data/vpoutfb.h>
#include <linux/spinlock.h>
#include "it66121.h"

#define PSEUDO_PALETTE_SIZE 16

struct vpoutfb_par {
	u32 palette[PSEUDO_PALETTE_SIZE];
	void __iomem			*mmio_base;
	void __iomem			*mem_virt;
	dma_addr_t			mem_phys;
	size_t				mem_size;
	void				*mem_handle;
	struct it66121_device_data	hdmidata;
	struct vpoutfb_format		*color_fmt;
	struct tasklet_struct		reset_tasklet;

#if defined CONFIG_OF && defined CONFIG_COMMON_CLK
	int clk_count;
	struct clk **clks;
#endif
};

#endif /* VPOUTFB_H */
