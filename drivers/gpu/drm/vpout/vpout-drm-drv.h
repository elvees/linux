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

#ifndef __VPOUT_DRM_DRV_H__
#define __VPOUT_DRM_DRV_H__

#include <linux/clk.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/list.h>

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>

struct connector_proxy {
	struct drm_connector *connector;
	const struct drm_connector_helper_funcs *funcs;
};

struct vpout_drm_private {
	void __iomem *mmio;
	void __iomem *dsi;

	struct regmap *smctr;

	struct clk *clk;
	struct workqueue_struct *wq;
	struct drm_fbdev_cma *fbdev;
	struct drm_crtc *crtc;

	int num_slaves;
	struct connector_proxy slaves[8];
};

int vpout_drm_crtc_mode_valid(struct drm_crtc *crtc,
			      struct drm_display_mode *mode);

irqreturn_t vpout_drm_crtc_irq(struct drm_crtc *crtc);

struct drm_crtc *vpout_drm_crtc_create(struct drm_device *dev);

#endif /* __VPOUT_DRM_DRV_H__ */
