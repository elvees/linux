/* SPDX-License-Identifier: GPL-2.0
 *
 * Copyright 2022 RnD Center "ELVEES", JSC
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * ELVEES Multi-Functional Buffered Serial Port (MFBSP)
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; version 2.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 *
 */

#ifndef __MFBSP_H__
#define __MFBSP_H__

#include <linux/bitops.h>
#include <linux/io.h>

/* MFBSP I2S Register Map */
#define MFBSP_I2S_TX			0x00
#define MFBSP_I2S_RX			0x00
#define MFBSP_I2S_CSR			0x04
#define MFBSP_I2S_DIR			0x08
#define MFBSP_I2S_TCTR			0x10
#define MFBSP_I2S_RCTR			0x14
#define MFBSP_I2S_TSR			0x18
#define MFBSP_I2S_RSR			0x1C
#define MFBSP_I2S_TCTR_RATE		0x20
#define MFBSP_I2S_RCTR_RATE		0x24
#define MFBSP_I2S_TSTART		0x28
#define MFBSP_I2S_RSTART		0x2C
#define MFBSP_I2S_EMERG			0x30
#define MFBSP_I2S_IMASK			0x34

#define MFBSP_I2S_CSR_EN		BIT(9)
#define MFBSP_I2S_CSR_LSTAT(v)		((v) << 3)
#define MFBSP_I2S_CSR_LTRAN		BIT(1)
#define MFBSP_I2S_CSR_LEN		BIT(0)

#define MFBSP_I2S_DIR_TD		BIT(5)
#define MFBSP_I2S_DIR_RD		BIT(4)
#define MFBSP_I2S_DIR_TCS		BIT(3)
#define MFBSP_I2S_DIR_RCS		BIT(2)
#define MFBSP_I2S_DIR_TCLK		BIT(1)
#define MFBSP_I2S_DIR_RCLK		BIT(0)

#define MFBSP_I2S_TCTR_CS_CONT		BIT(29)
#define MFBSP_I2S_TCTR_CLK_CONT		BIT(28)
#define MFBSP_I2S_TCTR_SWAP		BIT(27)
#define MFBSP_I2S_TCTR_PACK		BIT(25)
#define MFBSP_I2S_TCTR_WORDLEN(v)	((v) << 20)
#define MFBSP_I2S_TCTR_MBF		BIT(19)
#define MFBSP_I2S_TCTR_CSNEG		BIT(18)
#define MFBSP_I2S_TCTR_WORDCNT(v)	((v) << 12)
#define MFBSP_I2S_TCTR_DEL		BIT(11)
#define MFBSP_I2S_TCTR_NEG		BIT(10)
#define MFBSP_I2S_TCTR_DSPMODE		BIT(9)
#define MFBSP_I2S_TCTR_D_ZER_EN		BIT(2)
#define MFBSP_I2S_TCTR_MODE		BIT(1)
#define MFBSP_I2S_TCTR_EN		BIT(0)

#define MFBSP_I2S_RCTR_CS_CONT		BIT(29)
#define MFBSP_I2S_RCTR_CLK_CONT		BIT(28)
#define MFBSP_I2S_RCTR_SWAP		BIT(27)
#define MFBSP_I2S_RCTR_SIGN		BIT(26)
#define MFBSP_I2S_RCTR_PACK		BIT(25)
#define MFBSP_I2S_RCTR_WORDLEN(v)	((v) << 20)
#define MFBSP_I2S_RCTR_MBF		BIT(19)
#define MFBSP_I2S_RCTR_CSNEG		BIT(18)
#define MFBSP_I2S_RCTR_WORDCNT(v)	((v) << 12)
#define MFBSP_I2S_RCTR_DEL		BIT(11)
#define MFBSP_I2S_RCTR_NEG		BIT(10)
#define MFBSP_I2S_RCTR_DSPMODE		BIT(9)
#define MFBSP_I2S_RCTR_CS_CP		BIT(3)
#define MFBSP_I2S_RCTR_CLK_CP		BIT(2)
#define MFBSP_I2S_RCTR_MODE		BIT(1)
#define MFBSP_I2S_RCTR_EN		BIT(0)

#define MFBSP_I2S_TSTART_EN		BIT(0)

#define MFBSP_I2S_RSTART_EN		BIT(0)

/* MFBSP DMA Register Map */
#define MFBSP_DMA_IR			0x00
#define MFBSP_DMA_CP			0x08
#define MFBSP_DMA_CSR			0x10
#define MFBSP_DMA_RUN			0x14

#define MFBSP_DMA_CSR_WCX(v)		((v) << 16)
#define MFBSP_DMA_CSR_DONE		BIT(15)
#define MFBSP_DMA_CSR_END		BIT(14)
#define MFBSP_DMA_CSR_IM		BIT(13)
#define MFBSP_DMA_CSR_CHEN		BIT(12)
#define MFBSP_DMA_CSR_WN(v)		((v) << 2)
#define MFBSP_DMA_CSR_RUN		BIT(0)

static inline u32 mfbsp_readl(void __iomem *base, u32 offset)
{
	return readl(base + offset);
}

static inline u64 mfbsp_readq(void __iomem *base, u32 offset)
{
	return readq(base + offset);
}

static inline void mfbsp_writel(void __iomem *base, u32 offset, u32 value)
{
	writel(value, base + offset);
}

static inline void mfbsp_writeq(void __iomem *base, u32 offset, u64 value)
{
	writeq(value, base + offset);
}

struct mfbsp_dma_desc {
	u64 ir;
	u64 cp;
	u64 csr;
} __attribute__ ((__packed__));

struct mfbsp_dma_data {
	int irq;
	void __iomem *base;
	dma_addr_t desc_addr;
	struct mfbsp_dma_desc *desc_list;
};

struct mfbsp_data {
	void __iomem *base;
	struct clk *clk;
	struct dma_pool *dma_desc_pool;
	struct mfbsp_dma_data playback_dma;
	struct mfbsp_dma_data capture_dma;
};

int mfbsp_register_platform(struct platform_device *pdev);

#endif
