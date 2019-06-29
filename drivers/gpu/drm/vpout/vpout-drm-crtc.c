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

	if (vpout_drm_crtc->enabled)
		return;

	vpout_drm_set(dev, LCDC_CSR, LCDC_CSR_EN);

	vpout_drm_set(dev, LCDC_CSR, LCDC_CSR_INIT);
	while (vpout_drm_read(dev, LCDC_CSR) & LCDC_CSR_INIT)
		cpu_relax();

	vpout_drm_crtc_enable_irqs(dev);

	vpout_drm_set(dev, LCDC_CSR, LCDC_CSR_RUN);

	drm_crtc_vblank_on(crtc);

	vpout_drm_crtc->enabled = true;
}

static void vpout_drm_crtc_disable(struct drm_crtc *crtc)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;

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

static int vpout_drm_crtc_mode_set(struct drm_crtc *crtc,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode,
				   int x, int y,
				   struct drm_framebuffer *old_fb)
{
	struct vpout_drm_crtc *vpout_drm_crtc = to_vpout_drm_crtc(crtc);
	struct drm_device *dev = crtc->dev;
	uint32_t hbp, hsw, vbp, vsw;
	unsigned int depth, bpp;
	int ret;

	ret = vpout_drm_crtc_mode_valid(crtc, mode);
	if (ret != MODE_OK)
		return ret;

	hbp = mode->htotal - mode->hsync_end;
	hsw = mode->hsync_end - mode->hsync_start;
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

	if (mode->hdisplay & 0x0f)
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
