// SPDX-License-Identifier: GPL-2.0+
/*
 * ELVEES VPOUT Controller DRM Driver
 *
 * Copyright 2017-2018 RnD Center "ELVEES", JSC
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

#include <linux/regmap.h>

#include <drm/drm_crtc.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_flip_work.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_plane_helper.h>

#include "vpout-drm-drv.h"
#include "vpout-drm-link.h"
#include "vpout-drm-regs.h"

struct vpout_drm_crtc {
	struct drm_crtc base;
	bool enabled;
	bool frame_done;
	spinlock_t irq_lock;
	wait_queue_head_t frame_done_wq;
	struct drm_flip_work unref_work;
	struct drm_pending_vblank_event *event;
	const struct vpout_drm_info *info;
};

#define to_vpout_drm_crtc(x) container_of(x, struct vpout_drm_crtc, base)

static void unref_worker(struct drm_flip_work *work, void *val)
{
	struct vpout_drm_crtc *vpout_drm_crtc =
		container_of(work, struct vpout_drm_crtc, unref_work);
	struct drm_device *dev = vpout_drm_crtc->base.dev;

	mutex_lock(&dev->mode_config.mutex);
	drm_framebuffer_unreference(val);
	mutex_unlock(&dev->mode_config.mutex);
}

static void vpout_drm_crtc_enable_irqs(struct drm_device *dev)
{
	vpout_drm_write(dev, LCDC_INTMASK,
			LCDC_INT_SYNC_DONE | LCDC_INT_DMA_DONE);
}

static void vpout_drm_crtc_disable_irqs(struct drm_device *dev)
{
	vpout_drm_write(dev, LCDC_INTMASK, 0);
}

static void vpout_drm_crtc_enable(struct drm_crtc *crtc)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct vpout_drm_private *priv = dev->dev_private;
	bool dsi = vpout_drm_crtc->info->dsi;

	if (vpout_drm_crtc->enabled)
		return;

	if (dsi) {
		vpout_mipi_set(dev, MIPI_RST_ENB_DFE, DSI_RST_DFE_ENABLE);
		vpout_mipi_set(dev, MIPI_DEVICE_READY, DSI_DEVICE_READY);
		regmap_update_bits(priv->smctr, SMCTR_MIPI_MUX,
				   SMCTR_MIPI_MUX_DSI, SMCTR_MIPI_MUX_DSI);
	}

	vpout_drm_set(dev, LCDC_CSR, LCDC_CSR_EN);

	vpout_drm_set(dev, LCDC_CSR, LCDC_CSR_INIT);
	while (vpout_drm_read(dev, LCDC_CSR) & LCDC_CSR_INIT)
		cpu_relax();

	vpout_drm_crtc_enable_irqs(dev);

	vpout_drm_set(dev, LCDC_CSR, LCDC_CSR_RUN);

	if (dsi)
		vpout_mipi_set(dev, MIPI_DPI_CONTROL, DSI_DPI_TURN_ON);

	drm_crtc_vblank_on(crtc);

	vpout_drm_crtc->enabled = true;
}

static void vpout_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct vpout_drm_private *priv = dev->dev_private;
	bool dsi = vpout_drm_crtc->info->dsi;

	if (!vpout_drm_crtc->enabled)
		return;

	vpout_drm_clear(dev, LCDC_CSR, LCDC_CSR_RUN);

	/* NOTE: If an interrupt occurs here, wait_event_timeout() returns only
	 * after the timeout elapsed. */

	vpout_drm_crtc->frame_done = false;

	wait_event_timeout(vpout_drm_crtc->frame_done_wq,
			   vpout_drm_crtc->frame_done,
			   msecs_to_jiffies(50));

	drm_crtc_vblank_off(crtc);

	vpout_drm_crtc_disable_irqs(dev);

	vpout_drm_clear(dev, LCDC_CSR, LCDC_CSR_EN);

	vpout_drm_crtc->enabled = false;

	if (dsi)
		regmap_update_bits(priv->smctr, SMCTR_MIPI_MUX,
				   SMCTR_MIPI_MUX_DSI, ~SMCTR_MIPI_MUX_DSI);

}

static void vpout_drm_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	if (mode == DRM_MODE_DPMS_ON)
		vpout_drm_crtc_enable(crtc);
	else
		vpout_drm_crtc_disable(crtc);
}

static void vpout_drm_crtc_prepare(struct drm_crtc *crtc)
{
	vpout_drm_crtc_dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void vpout_drm_crtc_commit(struct drm_crtc *crtc)
{
	vpout_drm_crtc_dpms(crtc, DRM_MODE_DPMS_ON);
}

static void vpout_drm_crtc_set_clk(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct vpout_drm_private *priv = dev->dev_private;
	unsigned long clk_rate, clk_div;

	clk_rate = clk_get_rate(priv->clk);
	clk_div = DIV_ROUND_UP(clk_rate, crtc->mode.clock * 1000);

	vpout_drm_write(dev, LCDC_DIV, clk_div - 1);
}

static void vpout_drm_crtc_set_scanout(struct drm_crtc *crtc,
				       struct drm_framebuffer *fb)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct drm_gem_cma_object *gem;
	unsigned int depth, bpp;
	dma_addr_t start;

	drm_fb_get_bpp_depth(fb->pixel_format, &depth, &bpp);
	gem = drm_fb_cma_get_gem_obj(fb, 0);

	start = gem->paddr + fb->offsets[0] +
		crtc->y * fb->pitches[0] +
		crtc->x * bpp / 8;

	vpout_drm_write(dev, LCDC_AB0, start);

	drm_flip_work_queue(&vpout_drm_crtc->unref_work, fb);
}

static bool vpout_drm_crtc_mode_fixup(struct drm_crtc *crtc,
				      const struct drm_display_mode *mode,
				      struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void vpout_drm_crtc_dphy_timing_set(struct drm_crtc *crtc,
					   struct drm_display_mode *mode,
					   unsigned int *pclk,
					   unsigned int *byteclk)
{
	struct drm_device *dev = crtc->dev;
	struct vpout_drm_private *priv = dev->dev_private;
	unsigned long clk_rate, clk_div;

	unsigned int pclk_freq, ddrclk_freq, byteclk_freq; /* MHz */
	/* FIXME: Reference frequency = XTI / 2, for now hardcoded 12MHz */
	unsigned int clkesc_freq = 12;

	unsigned int pixel_format = 0;
	unsigned int video_mode_format = 1;
	/* TODO: Get number of lanes to use from devicetree.*/
	unsigned int lane_count = 4;

	/* txclkesc = 12MHz - Escape mode Transmit Clock */
	int t_hs_prep, t_hs_zero, t_hs_trail, t_hs_exit;
	int t_clk_prep, t_clk_zero, t_clk_trail;

	int dln_cnt_hs_prep, dln_cnt_hs_zero, dln_cnt_hs_trail, dln_cnt_hs_exit;
	int cln_cnt_hs_prep, cln_cnt_hs_zero, cln_cnt_hs_trail, cln_cnt_hs_exit;

	int lp_byteclk_cnt, hs_ls_cnt, hs_to_lp, lp_to_hs;
	int lp_init_cnt;

	u32 a, b, reg;

	reg = lane_count;
	reg |= DSI_DPI_VCHANNEL_CMD(1) | DSI_DPI_VCHANNEL_VIDEO(0);

	switch (crtc->primary->fb->pixel_format) {
	case DRM_FORMAT_RGB565:
		pixel_format = 16;
		reg |= DSI_DPI_COLOR_FORMAT_RGB565;
		break;
	case DRM_FORMAT_RGB888:
		pixel_format = 24;
		reg |= DSI_DPI_COLOR_FORMAT_RGB888;
		break;
	default:
		break;
	}

	vpout_mipi_write(dev, MIPI_DSI_FUNC_PRG, reg);

	clk_rate = clk_get_rate(priv->clk);
	clk_div = DIV_ROUND_UP(clk_rate, mode->clock * 1000);

	clk_rate /= 1000000; /* Hz -> MHz */
	pclk_freq = clk_rate / clk_div;
	ddrclk_freq = (pclk_freq * pixel_format * video_mode_format) /
			(2 * lane_count);
	byteclk_freq = ddrclk_freq / 4;

	*pclk = pclk_freq;
	*byteclk = byteclk_freq;

	/* MIPI D-PHY: Ths-prepare Min = 40ns+4*UI Max = 85ns+6*UI */
	t_hs_prep = 40 + DIV_ROUND_UP(4 * 1000, 2 * ddrclk_freq) + 20;
	/* MIPI D-PHY: Ths-prepare + Ths_zero Min = 145ns+10*UI */
	t_hs_zero = 145 + DIV_ROUND_UP(10 * 1000, 2 * ddrclk_freq) -
			t_hs_prep + 20;
	/* MIPI D-PHY: Ths-trail Min = max(n*8*UI, 60ns+n*4*UI) */
	t_hs_trail = max(DIV_ROUND_UP(1 * 8 * 1000, 2 * ddrclk_freq),
			 60 + DIV_ROUND_UP(1 * 4 * 1000, 2 * ddrclk_freq)) + 30;
	/* MIPI D-PHY: Ths-exit Min = 100 ns */
	t_hs_exit = 100 + 20;

	/* MIPI D-PHY: Tclk-trail Min = 60ns */
	t_clk_trail = 60 + 15;
	/* MIPI D-PHY: Tclk-prepare Min = 38ns Max = 95ns */
	t_clk_prep = 38 + 20;
	/* MIPI D-PHY: Tclk-prepare + Tclk_zero Min = 300ns */
	t_clk_zero = 300 - t_clk_prep + 30;

	/* Ths-prepare = (dln_cnt_hs_prep+1)*t_byteclk+9*t_ddrclk */
	dln_cnt_hs_prep = t_hs_prep - DIV_ROUND_UP(9 * 1000, ddrclk_freq);
	dln_cnt_hs_prep = DIV_ROUND_UP(dln_cnt_hs_prep * 1000, byteclk_freq);
	dln_cnt_hs_prep -= 1;
	/* Ths-zero = (dln_cnt_hs_zero+1)*t_byteclk */
	dln_cnt_hs_zero = DIV_ROUND_UP(t_hs_zero * 1000, byteclk_freq) - 1;
	/* Ths-trail = (dln_cnt_hs_trail+2)*t_byteclk */
	dln_cnt_hs_trail = DIV_ROUND_UP(t_hs_trail * 1000, byteclk_freq) - 2;
	/* Ths-exit = (dln_cnt_hs_exit+1)*t_byteclk */
	dln_cnt_hs_exit = DIV_ROUND_UP(t_hs_exit * 1000, byteclk_freq) - 1;

	/* Tclk-prepare = (cln_cnt_hs_prep+1)*t_byteclk */
	cln_cnt_hs_prep = DIV_ROUND_UP(t_clk_prep * 1000, byteclk_freq) - 1;
	/* Tclk-zero = (cln_cnt_hs_zero+1)*t_byteclk */
	cln_cnt_hs_zero = DIV_ROUND_UP(t_clk_zero * 1000, byteclk_freq) - 1;
	/* Tclk-trail = (cln_cnt_hs_trail) + (3)*t_ddr_clk ??? */
	cln_cnt_hs_trail = t_clk_trail - DIV_ROUND_UP(3 * 1000, ddrclk_freq);
	cln_cnt_hs_trail = DIV_ROUND_UP(cln_cnt_hs_trail * 1000, byteclk_freq);
	cln_cnt_hs_trail += 3;
	/* Ths-exit = (cln_cnt_hs_exit+1)*t_byteclk + (2)*t_ddr_clk */
	cln_cnt_hs_exit = t_hs_exit - DIV_ROUND_UP(2 * 1000, ddrclk_freq);
	cln_cnt_hs_exit = DIV_ROUND_UP(cln_cnt_hs_exit * 1000, byteclk_freq);
	cln_cnt_hs_exit -= 1;

	/* MIPI D-PHY: Tlpx Min = 50ns */
	lp_byteclk_cnt = DIV_ROUND_UP(byteclk_freq, clkesc_freq);
	/* Data lane = 4*Tlpx+Ths_prep+Ths_zero+4byteclk */
	hs_ls_cnt = 4 * lp_byteclk_cnt + dln_cnt_hs_prep + dln_cnt_hs_zero + 4;
	/* HS to LP = Tclk_trail + Ths_exit + 3byteclk */
	hs_to_lp = cln_cnt_hs_trail + cln_cnt_hs_exit + 3;
	/* LP to HS = 4*Tlpx+[Tclk_prep+1]+[Tclk_zero+1]+Tclk_pre+2 */
	lp_to_hs = 4 * lp_byteclk_cnt;
	lp_to_hs += (cln_cnt_hs_prep + 1) + (cln_cnt_hs_zero + 1);
	lp_to_hs += DIV_ROUND_UP(8 * 1000, 2 * ddrclk_freq) + 2;

	/* MIPI D-PHY: Tinit Min = 100μs */
	lp_init_cnt = 100 * clkesc_freq;

	vpout_mipi_write(dev, MIPI_HS_TX_TIMEOUT, 0xffffff);
	vpout_mipi_write(dev, MIPI_LP_RX_TIMEOUT, 0xffffff);
	vpout_mipi_write(dev, MIPI_TURN_AROUND_TIMEOUT, 0x1f);
	vpout_mipi_write(dev, MIPI_DEVICE_RESET, 0xff);

	vpout_mipi_write(dev, MIPI_HIGH_LOW_SWITCH_COUNT, hs_ls_cnt);
	vpout_mipi_write(dev, MIPI_CLK_LANE_SWT, (lp_to_hs << 16) | hs_to_lp);
	vpout_mipi_write(dev, MIPI_LP_BYTECLK, lp_byteclk_cnt);
	vpout_mipi_write(dev, MIPI_DPHY_PARAM,
			 (dln_cnt_hs_exit << 24) | (dln_cnt_hs_trail << 16) |
			 (dln_cnt_hs_zero << 8) | dln_cnt_hs_prep);
	vpout_mipi_write(dev, MIPI_CLK_LANE_TIMING_PARAM,
			 (cln_cnt_hs_exit << 24) | (cln_cnt_hs_trail << 16) |
			 (cln_cnt_hs_zero << 8) | cln_cnt_hs_prep);

	vpout_mipi_write(dev, MIPI_INIT_COUNT, lp_init_cnt);
	vpout_mipi_write(dev, MIPI_CLK_EOT, DSI_CLK_EOT_BTA_DISABLE);
	vpout_mipi_write(dev, MIPI_POLARITY, 0x0);
	vpout_mipi_write(dev, MIPI_AUTO_ERR_REC, DSI_ECC_MUL_ERR_CLR);

	/* FIXME: According Arasan User& SoC Integration Guide p.79, when
	 * changing reference frequency besides configuring divider
	 * (trim_1<6:0>), should also configure LPF resistor (trim_1<19:17>)
	 */
	reg = DIV_ROUND_UP(ddrclk_freq, clkesc_freq);
	a = reg >> 1;
	b = reg & 0x1;
	reg = vpout_mipi_read(dev, MIPI_TRIM1) & ~DSI_TRIM1_DIVIDER_MASK;
	vpout_mipi_write(dev, MIPI_TRIM1, reg | (b << 6) | a);
}

#define PCLK_TO_BYTECLK(val)	DIV_ROUND_UP(val * byteclk_freq, pclk_freq)

static int vpout_drm_crtc_mode_set(struct drm_crtc *crtc,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode,
				   int x, int y,
				   struct drm_framebuffer *old_fb)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	uint32_t hfp, hbp, hsw, vfp, vbp, vsw;
	unsigned int pclk_freq, byteclk_freq; /* MHz */
	unsigned int depth, bpp;
	bool dsi = vpout_drm_crtc->info->dsi;
	int ret;

	ret = vpout_drm_crtc_mode_valid(crtc, mode);
	if (ret != MODE_OK)
		return ret;

	hfp = mode->hsync_start - mode->hdisplay;
	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
	vfp = mode->vsync_start - mode->vdisplay;
	vbp = mode->vtotal - mode->vsync_end;
	vsw = mode->vsync_end - mode->vsync_start;

	vpout_drm_write(dev, LCDC_HT0,
			LCDC_HT0_HGDEL(hbp - 1) |
			LCDC_HT0_HSW(hsw - 1));
	vpout_drm_write(dev, LCDC_HT1,
			LCDC_HT1_HGATE(mode->hdisplay - 1) |
			LCDC_HT1_HLEN(mode->htotal - 1));

	vpout_drm_write(dev, LCDC_VT0,
			LCDC_VT0_VGDEL(vbp - 1) |
			LCDC_VT0_VSW(vsw - 1));
	vpout_drm_write(dev, LCDC_VT1,
			LCDC_VT1_VGATE(mode->vdisplay - 1) |
			LCDC_VT1_VLEN(mode->vtotal - 1));

	if (dsi) {
		vpout_mipi_write(dev, MIPI_DPI_RESOLUTION,
				 mode->vdisplay << 16 | mode->hdisplay);

		vpout_drm_crtc_dphy_timing_set(crtc, mode, &pclk_freq,
					       &byteclk_freq);

		vpout_mipi_write(dev, MIPI_HSYNC_COUNT,
				 PCLK_TO_BYTECLK(hsw));
		vpout_mipi_write(dev, MIPI_HORIZ_BACK_PORCH_COUNT,
				 PCLK_TO_BYTECLK(hbp));
		vpout_mipi_write(dev, MIPI_HORIZ_FRONT_PORCH_COUNT,
				 PCLK_TO_BYTECLK(hfp));
		vpout_mipi_write(dev, MIPI_HORIZ_ACTIVE_AREA_COUNT,
				 PCLK_TO_BYTECLK(mode->hdisplay));

		vpout_mipi_write(dev, MIPI_VSYNC_COUNT, vsw);
		vpout_mipi_write(dev, MIPI_VERT_BACK_PORCH_COUNT, vbp);
		vpout_mipi_write(dev, MIPI_VERT_FRONT_PORCH_COUNT, vfp);

		vpout_mipi_write(dev, MIPI_VIDEO_MODE_FORMAT,
				 DSI_VIDEO_MODE_NON_BURST_SYNC_EVENTS);
	}

	drm_fb_get_bpp_depth(crtc->primary->fb->pixel_format, &depth, &bpp);

	switch (bpp) {
	case 16:
		vpout_drm_write(dev, LCDC_MODE,
				LCDC_MODE_INSIZE(LCDC_MODE_INSIZE_16BPP));
		break;
	case 24:
		vpout_drm_write(dev, LCDC_MODE,
				LCDC_MODE_INSIZE(LCDC_MODE_INSIZE_24BPP));
		break;
	case 32:
		vpout_drm_write(dev, LCDC_MODE,
				LCDC_MODE_INSIZE(LCDC_MODE_INSIZE_32BPP));
		break;
	default:
		return -EINVAL;
	}

	if (vpout_drm_crtc->info->invert_pxl_clk)
		vpout_drm_set(dev, LCDC_MODE, LCDC_MODE_PINV);

	if (!(mode->flags & DRM_MODE_FLAG_NHSYNC))
		vpout_drm_set(dev, LCDC_MODE, LCDC_MODE_HINV);

	if (!(mode->flags & DRM_MODE_FLAG_NVSYNC))
		vpout_drm_set(dev, LCDC_MODE, LCDC_MODE_VINV);

	vpout_drm_crtc_set_clk(crtc);

	drm_framebuffer_reference(crtc->primary->fb);

	vpout_drm_crtc_set_scanout(crtc, crtc->primary->fb);

	return 0;
}

static const struct drm_crtc_helper_funcs vpout_drm_crtc_helper_funcs = {
	.dpms = vpout_drm_crtc_dpms,
	.prepare = vpout_drm_crtc_prepare,
	.commit = vpout_drm_crtc_commit,
	.mode_fixup = vpout_drm_crtc_mode_fixup,
	.mode_set = vpout_drm_crtc_mode_set,
};

static void vpout_drm_crtc_destroy(struct drm_crtc *crtc)
{
	vpout_drm_crtc_disable(crtc);

	drm_crtc_cleanup(crtc);
}

static int vpout_drm_crtc_page_flip(struct drm_crtc *crtc,
				    struct drm_framebuffer *fb,
				    struct drm_pending_vblank_event *event,
				    uint32_t page_flip_flags)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	unsigned long flags;

	if (vpout_drm_crtc->event)
		return -EBUSY;

	drm_framebuffer_reference(fb);

	crtc->primary->fb = fb;

	spin_lock_irqsave(&vpout_drm_crtc->irq_lock, flags);

	vpout_drm_crtc_set_scanout(crtc, fb);

	vpout_drm_crtc->event = event;

	spin_unlock_irqrestore(&vpout_drm_crtc->irq_lock, flags);

	return 0;
}

void vpout_drm_crtc_set_panel_info(struct drm_crtc *crtc,
				   const struct vpout_drm_info *info)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);

	vpout_drm_crtc->info = info;
}

int vpout_drm_crtc_set_config(struct drm_mode_set *set)
{
	struct vpout_drm_info *info;
	struct drm_connector *conn;

	/* support one encoder/connector in one time */
	if (set->num_connectors > 1)
		return -EINVAL;

	/* prepare external info */
	conn = set->num_connectors ? set->connectors[0] : NULL;

	if (conn) {
		info = vpout_drm_get_encoder_info(conn->encoder);
		vpout_drm_crtc_set_panel_info(set->crtc, info);
	}

	return drm_crtc_helper_set_config(set);
}

static const struct drm_crtc_funcs vpout_drm_crtc_funcs = {
	.destroy = vpout_drm_crtc_destroy,
	.set_config = vpout_drm_crtc_set_config,
	.page_flip = vpout_drm_crtc_page_flip,
};

int vpout_drm_crtc_mode_valid(struct drm_crtc *crtc,
			      struct drm_display_mode *mode)
{
	uint32_t hbp, hfp, hsw, vbp, vfp, vsw;

	if (mode->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;

	if (mode->hdisplay > 2048)
		return MODE_VIRTUAL_X;

	if (mode->vdisplay > 2048)
		return MODE_VIRTUAL_Y;

	hbp = mode->htotal - mode->hsync_end;
	hfp = mode->hsync_start - mode->hdisplay;
	hsw = mode->hsync_end - mode->hsync_start;
	vbp = mode->vtotal - mode->vsync_end;
	vfp = mode->vsync_start - mode->vdisplay;
	vsw = mode->vsync_end - mode->vsync_start;

	if (hbp > 1024)
		return MODE_HBLANK_WIDE;

	if (hfp > 1024)
		return MODE_HBLANK_WIDE;

	if (hsw > 1024)
		return MODE_HSYNC_WIDE;

	if (vbp > 256)
		return MODE_VBLANK_WIDE;

	if (vfp > 256)
		return MODE_VBLANK_WIDE;

	if (vsw > 64)
		return MODE_VSYNC_WIDE;

	return MODE_OK;
}

irqreturn_t vpout_drm_crtc_irq(struct drm_crtc *crtc)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	struct vpout_drm_private *priv = dev->dev_private;
	uint32_t stat;

	stat = vpout_drm_read(dev, LCDC_INT);
	vpout_drm_write(dev, LCDC_INT, stat);

	if (stat & LCDC_INT_DMA_DONE) {
		unsigned long flags;

		drm_flip_work_commit(&vpout_drm_crtc->unref_work, priv->wq);

		drm_crtc_handle_vblank(crtc);

		spin_lock_irqsave(&dev->event_lock, flags);

		if (vpout_drm_crtc->event) {
			drm_crtc_send_vblank_event(crtc,
						   vpout_drm_crtc->event);
			vpout_drm_crtc->event = NULL;
		}

		spin_unlock_irqrestore(&dev->event_lock, flags);
	}

	/* NOTE: This interrupt occurs at the end of each frame. */
	if (stat & LCDC_INT_SYNC_DONE) {
		vpout_drm_crtc->frame_done = true;

		wake_up(&vpout_drm_crtc->frame_done_wq);
	}

	return IRQ_HANDLED;
}

struct drm_crtc *vpout_drm_crtc_create(struct drm_device *dev)
{
	struct vpout_drm_crtc *vpout_drm_crtc;
	struct drm_crtc *crtc;
	int ret;

	vpout_drm_crtc = devm_kzalloc(dev->dev, sizeof(*vpout_drm_crtc),
				      GFP_KERNEL);
	if (!vpout_drm_crtc)
		return NULL;

	crtc = &vpout_drm_crtc->base;

	init_waitqueue_head(&vpout_drm_crtc->frame_done_wq);

	drm_flip_work_init(&vpout_drm_crtc->unref_work, "unref", unref_worker);

	spin_lock_init(&vpout_drm_crtc->irq_lock);

	ret = drm_crtc_init(dev, crtc, &vpout_drm_crtc_funcs);
	if (ret < 0)
		goto fail;

	drm_crtc_helper_add(crtc, &vpout_drm_crtc_helper_funcs);

	return crtc;

fail:
	vpout_drm_crtc_destroy(crtc);
	return NULL;
}
