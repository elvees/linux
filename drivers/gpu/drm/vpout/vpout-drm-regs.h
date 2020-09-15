/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Based on tilcdc:
 * Copyright (C) 2012 Texas Instruments
 * Author: Rob Clark <robdclark@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __VPOUT_DRM_REGS_H__
#define __VPOUT_DRM_REGS_H__

#include <linux/bitops.h>

#include "vpout-drm-drv.h"

#define SMCTR_MIPI_MUX            0x24

#define SMCTR_MIPI_MUX_DSI        0x1

#define LCDC_CSR			0x00
#define LCDC_DIV			0x04
#define LCDC_MODE			0x08
#define LCDC_HT0			0x0C
#define LCDC_HT1			0x10
#define LCDC_VT0			0x14
#define LCDC_VT1			0x18
#define LCDC_AB0			0x2C
#define LCDC_AB1			0x30
#define LCDC_OFF0			0x34
#define LCDC_OFF1			0x38
#define LCDC_INT			0x44
#define LCDC_INTMASK			0x48

#define LCDC_CSR_CLR			BIT(3)
#define LCDC_CSR_INIT			BIT(2)
#define LCDC_CSR_RUN			BIT(1)
#define LCDC_CSR_EN			BIT(0)

#define LCDC_MODE_CLK_ON		BIT(17)
#define LCDC_MODE_VDEF			BIT(16)
#define LCDC_MODE_HDEF			BIT(15)
#define LCDC_MODE_DEN_EN		BIT(14)
#define LCDC_MODE_INSYNC		BIT(13)
#define LCDC_MODE_CCM			BIT(12)
#define LCDC_MODE_PINV			BIT(11)
#define LCDC_MODE_DINV			BIT(10)
#define LCDC_MODE_VINV			BIT(9)
#define LCDC_MODE_HINV			BIT(8)
#define LCDC_MODE_BUF_NUMB		BIT(7)
#define LCDC_MODE_BUF_MODE		BIT(6)
#define LCDC_MODE_HWC_MODE		BIT(5)
#define LCDC_MODE_HWCEN			BIT(4)
#define LCDC_MODE_INSIZE(v)		((v) << 0)

#define LCDC_MODE_INSIZE_8BPP		0
#define LCDC_MODE_INSIZE_12BPP		1
#define LCDC_MODE_INSIZE_15BPP		2
#define LCDC_MODE_INSIZE_16BPP		3
#define LCDC_MODE_INSIZE_18BPP		4
#define LCDC_MODE_INSIZE_24BPP		5
#define LCDC_MODE_INSIZE_32BPP		6

#define LCDC_HT0_HGDEL(v)		((v) << 16)
#define LCDC_HT0_HSW(v)			((v) << 0)
#define LCDC_HT1_HLEN(v)		((v) << 16)
#define LCDC_HT1_HGATE(v)		((v) << 0)

#define LCDC_VT0_VGDEL(v)		((v) << 16)
#define LCDC_VT0_VSW(v)			((v) << 0)
#define LCDC_VT1_VLEN(v)		((v) << 16)
#define LCDC_VT1_VGATE(v)		((v) << 0)

#define LCDC_INT_SYNC_DONE		BIT(5)
#define LCDC_INT_OUT_FIFO_EMPTY		BIT(3)
#define LCDC_INT_OUT_FIFO_INT		BIT(2)
#define LCDC_INT_DMA_FIFO_EMPTY		BIT(1)
#define LCDC_INT_DMA_DONE		BIT(0)

#define MIPI_DEVICE_READY		0x00
#define MIPI_INTR_STAT			0x04
#define MIPI_INTR_EN			0x08
#define MIPI_DSI_FUNC_PRG		0x0C
#define MIPI_HS_TX_TIMEOUT		0x10
#define MIPI_LP_RX_TIMEOUT		0x14
#define MIPI_TURN_AROUND_TIMEOUT	0x18
#define MIPI_DEVICE_RESET		0x1C
#define MIPI_DPI_RESOLUTION		0x20
#define MIPI_HSYNC_COUNT		0x28
#define MIPI_HORIZ_BACK_PORCH_COUNT	0x2C
#define MIPI_HORIZ_FRONT_PORCH_COUNT	0x30
#define MIPI_HORIZ_ACTIVE_AREA_COUNT	0x34
#define MIPI_VSYNC_COUNT		0x38
#define MIPI_VERT_BACK_PORCH_COUNT	0x3C
#define MIPI_VERT_FRONT_PORCH_COUNT	0x40
#define MIPI_HIGH_LOW_SWITCH_COUNT	0x44
#define MIPI_DPI_CONTROL		0x48
#define MIPI_PLL_LOCK_COUNT		0x4C
#define MIPI_INIT_COUNT			0x50
#define MIPI_MAX_RETURN_PACK		0x54
#define MIPI_VIDEO_MODE_FORMAT		0x58
#define MIPI_CLK_EOT			0x5C
#define MIPI_POLARITY			0x60
#define MIPI_CLK_LANE_SWT		0x64
#define MIPI_LP_BYTECLK			0x68
#define MIPI_DPHY_PARAM			0x6C
#define MIPI_CLK_LANE_TIMING_PARAM	0x70
#define MIPI_RST_ENB_DFE		0x74
#define MIPI_TRIM1			0x7C
#define MIPI_TRIM2			0x80
#define MIPI_AUTO_ERR_REC		0x98
#define MIPI_DIR_DPI_DIFF		0x9C
#define MIPI_DATA_LANE_POLARITY_SWAP	0xA0

#define DSI_DEVICE_READY			BIT(0)

#define DSI_DPI_VCHANNEL_VIDEO(n)		(((n) & 0x3) << 3)
#define DSI_DPI_VCHANNEL_CMD(n)			(((n) & 0x3) << 5)
#define DSI_DPI_COLOR_FORMAT_RGB565		(0x1 << 7)
#define DSI_DPI_COLOR_FORMAT_RGB666		(0x2 << 7)
#define DSI_DPI_COLOR_FORMAT_RGB666_UNPACK	(0x3 << 7)
#define DSI_DPI_COLOR_FORMAT_RGB888		(0x4 << 7)

#define DSI_DPI_SHUT_DOWN			BIT(0)
#define DSI_DPI_TURN_ON				BIT(1)
#define DSI_DPI_COLOR_MODE_ON			BIT(2)
#define DSI_DPI_COLOR_MODE_OFF			BIT(3)

#define DSI_VIDEO_MODE_NON_BURST_SYNC_PULSE	0x1
#define DSI_VIDEO_MODE_NON_BURST_SYNC_EVENTS	0x2
#define DSI_VIDEO_MODE_BURST			0x3

#define DSI_CLK_EOT_BTA_DISABLE			BIT(2)

#define DSI_TRIM1_DIVIDER_MASK			GENMASK(6, 0)

#define DSI_RST_DFE_ENABLE			BIT(0)

#define DSI_ECC_MUL_ERR_CLR			BIT(0)

/*
 * Helpers:
 */

static inline void vpout_drm_write(struct drm_device *dev, u32 reg, u32 data)
{
	struct vpout_drm_private *priv = dev->dev_private;

	iowrite32(data, priv->mmio + reg);
}

static inline u32 vpout_drm_read(struct drm_device *dev, u32 reg)
{
	struct vpout_drm_private *priv = dev->dev_private;

	return ioread32(priv->mmio + reg);
}

static inline void vpout_drm_set(struct drm_device *dev, u32 reg, u32 mask)
{
	vpout_drm_write(dev, reg, vpout_drm_read(dev, reg) | mask);
}

static inline void vpout_drm_clear(struct drm_device *dev, u32 reg, u32 mask)
{
	vpout_drm_write(dev, reg, vpout_drm_read(dev, reg) & ~mask);
}

static inline void vpout_mipi_write(struct drm_device *dev, u32 reg, u32 data)
{
	struct vpout_drm_private *priv = dev->dev_private;

	iowrite32(data, priv->dsi + reg);
}

static inline u32 vpout_mipi_read(struct drm_device *dev, u32 reg)
{
	struct vpout_drm_private *priv = dev->dev_private;

	return ioread32(priv->dsi + reg);
}

static inline void vpout_mipi_set(struct drm_device *dev, u32 reg, u32 mask)
{
	vpout_mipi_write(dev, reg, vpout_mipi_read(dev, reg) | mask);
}

static inline void vpout_mipi_clear(struct drm_device *dev, u32 reg, u32 mask)
{
	vpout_mipi_write(dev, reg, vpout_mipi_read(dev, reg) & ~mask);
}

#endif /* __VPOUT_DRM_REGS_H__ */
