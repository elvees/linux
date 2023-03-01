// SPDX-License-Identifier: GPL-2.0+
/*
 * ELVEES VPOUT Controller DRM DSI Driver
 *
 * Copyright 2019-2020 RnD Center "ELVEES", JSC
 *
 * Based on exynos:
 * Samsung SoC MIPI DSI Master driver.
 * Copyright (c) 2014 Samsung Electronics Co., Ltd
 * Contacts: Tomasz Figa <t.figa@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <drm/drmP.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_of.h>

#include <linux/component.h>

#include <video/of_display_timing.h>
#include <video/videomode.h>

#define DSIM_STATE_ENABLED		BIT(0)
#define DSIM_STATE_INITIALIZED		BIT(1)
#define DSIM_STATE_CMD_LPM		BIT(2)
#define DSIM_STATE_VIDOUT_AVAILABLE	BIT(3)

struct vpout_dsi {
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct device_node *panel_node;
	struct display_timings *timings;
	struct device *dev;

	u32 lanes;
	u32 mode_flags;
	u32 format;
	struct videomode vm;

	int state;
};

#define host_to_dsi(host) container_of(host, struct vpout_dsi, dsi_host)
#define connector_to_dsi(c) container_of(c, struct vpout_dsi, connector)

static inline struct vpout_dsi *encoder_to_dsi(struct drm_encoder *e)
{
	return container_of(e, struct vpout_dsi, encoder);
}

static void vpout_dsi_set_display_mode(struct vpout_dsi *dsi)
{
	struct videomode *vm = &dsi->vm;

	dev_dbg(dsi->dev, "LCD size = %dx%d\n", vm->hactive, vm->vactive);
}

static void vpout_dsi_set_display_enable(struct vpout_dsi *dsi, bool enable)
{
}

static int vpout_dsi_poweron(struct vpout_dsi *dsi)
{
	return 0;
}

static void vpout_dsi_poweroff(struct vpout_dsi *dsi)
{
	dsi->state &= ~DSIM_STATE_INITIALIZED;
}

static void dsi_encoder_disable(struct drm_encoder *encoder)
{
	struct vpout_dsi *dsi = encoder_to_dsi(encoder);

	if (!(dsi->state & DSIM_STATE_ENABLED))
		return;

	dsi->state &= ~DSIM_STATE_VIDOUT_AVAILABLE;

	vpout_dsi_set_display_enable(dsi, false);

	dsi->state &= ~DSIM_STATE_ENABLED;

	vpout_dsi_poweroff(dsi);
}

/*
 * Connector:
 */
static enum drm_connector_status
dsi_connector_detect(struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static void dsi_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
	connector->dev = NULL;
}

static const struct drm_connector_funcs vpout_dsi_connector_funcs = {
	.dpms       = drm_helper_connector_dpms,
	.detect     = dsi_connector_detect,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy    = dsi_connector_destroy,
};

static int dsi_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm_dev = connector->dev;
	struct vpout_dsi *dsi = connector_to_dsi(connector);
	struct display_timings *timings = dsi->timings;
	int i;

	for (i = 0; i < timings->num_timings; i++) {
		struct drm_display_mode *mode = drm_mode_create(drm_dev);
		struct videomode vm;

		if (videomode_from_timings(timings, &vm, i))
			break;

		drm_display_mode_from_videomode(&vm, mode);

		mode->type = DRM_MODE_TYPE_DRIVER;

		if (timings->native_mode == i)
			mode->type |= DRM_MODE_TYPE_PREFERRED;

		drm_mode_set_name(mode);
		drm_mode_probed_add(connector, mode);
	}

	return i;
}

static int dsi_connector_mode_valid(struct drm_connector *connector,
				    struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder *
dsi_connector_best_encoder(struct drm_connector *connector)
{
	struct vpout_dsi *dsi = connector_to_dsi(connector);

	return &dsi->encoder;
}

static struct drm_connector_helper_funcs vpout_dsi_connector_helper_funcs = {
	.get_modes    = dsi_connector_get_modes,
	.mode_valid   = dsi_connector_mode_valid,
	.best_encoder = dsi_connector_best_encoder,
};

static int vpout_dsi_create_connector(struct drm_encoder *encoder)
{
	struct vpout_dsi *dsi = encoder_to_dsi(encoder);
	struct drm_connector *connector = &dsi->connector;
	int ret;

	ret = drm_connector_init(encoder->dev, connector,
				 &vpout_dsi_connector_funcs,
				 DRM_MODE_CONNECTOR_DSI);
	if (ret) {
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;
	connector->encoder = encoder;

	drm_connector_helper_add(connector, &vpout_dsi_connector_helper_funcs);
	drm_connector_register(connector);
	drm_mode_connector_attach_encoder(connector, encoder);

	return 0;
}

/*
 * Encoder:
 */
static void dsi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
}

static void dsi_encoder_prepare(struct drm_encoder *encoder)
{
	dsi_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void dsi_encoder_commit(struct drm_encoder *encoder)
{
	dsi_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static bool dsi_encoder_mode_fixup(struct drm_encoder *encoder,
				   const struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	return true;
}

static void dsi_encoder_mode_set(struct drm_encoder *encoder,
				 struct drm_display_mode *mode,
				 struct drm_display_mode *adjusted_mode)
{
	struct vpout_dsi *dsi = encoder_to_dsi(encoder);
	struct videomode *vm = &dsi->vm;
	struct drm_display_mode *m = adjusted_mode;

	vm->hactive = m->hdisplay;
	vm->vactive = m->vdisplay;
	vm->vfront_porch = m->vsync_start - m->vdisplay;
	vm->vback_porch = m->vtotal - m->vsync_end;
	vm->vsync_len = m->vsync_end - m->vsync_start;
	vm->hfront_porch = m->hsync_start - m->hdisplay;
	vm->hback_porch = m->htotal - m->hsync_end;
	vm->hsync_len = m->hsync_end - m->hsync_start;
}

static void dsi_encoder_enable(struct drm_encoder *encoder)
{
	struct vpout_dsi *dsi = encoder_to_dsi(encoder);
	int ret;

	if (dsi->state & DSIM_STATE_ENABLED)
		return;

	ret = vpout_dsi_poweron(dsi);
	if (ret < 0)
		return;

	dsi->state |= DSIM_STATE_ENABLED;

	vpout_dsi_set_display_mode(dsi);
	vpout_dsi_set_display_enable(dsi, true);

	dsi->state |= DSIM_STATE_VIDOUT_AVAILABLE;
}

static const struct drm_encoder_funcs vpout_dsi_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static const struct drm_encoder_helper_funcs vpout_dsi_encoder_helper_funcs = {
	.dpms       = dsi_encoder_dpms,
	.mode_fixup = dsi_encoder_mode_fixup,
	.prepare    = dsi_encoder_prepare,
	.commit     = dsi_encoder_commit,
	.mode_set   = dsi_encoder_mode_set,
	.disable    = dsi_encoder_disable,
	.enable     = dsi_encoder_enable,
};

static int vpout_dsi_parse_dt(struct vpout_dsi *dsi)
{
	struct device *dev = dsi->dev;
	struct device_node *node = dev->of_node;

	dsi->timings = of_get_display_timings(node);
	if (!dsi->timings) {
		dev_err(dsi->dev, "could not get panel timings\n");
		return -EINVAL;
	}

	return 0;
}

static int vpout_dsi_bind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);
	struct drm_device *drm_dev = data;
	uint32_t crtcs_mask = 0;
	int ret;

	if (dev->of_node)
		crtcs_mask = drm_of_find_possible_crtcs(drm_dev, dev->of_node);

	/* If no CRTCs were found, fall back to our old behaviour */
	if (crtcs_mask == 0) {
		dev_warn(dev, "Falling back to first CRTC\n");
		crtcs_mask = 1;
	}

	encoder->possible_crtcs = crtcs_mask;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	drm_encoder_init(drm_dev, encoder, &vpout_dsi_encoder_funcs,
			 DRM_MODE_ENCODER_DSI);

	drm_encoder_helper_add(encoder, &vpout_dsi_encoder_helper_funcs);

	ret = vpout_dsi_create_connector(encoder);
	if (ret) {
		DRM_ERROR("failed to create connector ret = %d\n", ret);
		drm_encoder_cleanup(encoder);
		return ret;
	}

	return 0;
}

static void vpout_dsi_unbind(struct device *dev, struct device *master,
				void *data)
{
	struct drm_encoder *encoder = dev_get_drvdata(dev);

dsi_encoder_disable(encoder);
}

static const struct component_ops vpout_dsi_component_ops = {
	.bind	= vpout_dsi_bind,
	.unbind	= vpout_dsi_unbind,
};

static int vpout_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct vpout_dsi *dsi;
	int ret;

	dsi = devm_kzalloc(dev, sizeof(*dsi), GFP_KERNEL);
	if (!dsi)
		return -ENOMEM;

	dsi->dev = dev;

	ret = vpout_dsi_parse_dt(dsi);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, &dsi->encoder);

	return component_add(dev, &vpout_dsi_component_ops);
}

static int vpout_dsi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &vpout_dsi_component_ops);

	return 0;
}

static const struct of_device_id vpout_dsi_of_match[] = {
	{ .compatible = "elvees,mcom02-vpout-dsi" },
	{ }
};

MODULE_DEVICE_TABLE(of, vpout_dsi_of_match);

struct platform_driver vpout_dsi_driver = {
	.probe = vpout_dsi_probe,
	.remove = vpout_dsi_remove,
	.driver = {
		   .name = "vpout-dsi",
		   .owner = THIS_MODULE,
		   .of_match_table = vpout_dsi_of_match,
	},
};

module_platform_driver(vpout_dsi_driver);

MODULE_DESCRIPTION("ELVEES VPOUT MIPI DSI Driver");
MODULE_LICENSE("GPL");
