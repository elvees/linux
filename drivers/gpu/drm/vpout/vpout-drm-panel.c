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

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pinctrl/pinmux.h>
#include <video/display_timing.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>
#include <linux/component.h>
#include <drm/drm_of.h>
#include "vpout-drm-drv.h"

struct panel {
	struct drm_encoder encoder;
	struct drm_connector connector;
	struct display_timings *timings;
	struct backlight_device *backlight;
	struct gpio_desc *enable_gpio;
};

#define cast_from_encoder(x) container_of(x, struct panel, encoder)
#define cast_from_connector(x) container_of(x, struct panel, connector)

/*
 * Encoder:
 */

static void panel_encoder_destroy(struct drm_encoder *encoder)
{
	drm_encoder_cleanup(encoder);
}

static void panel_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct panel *panel = cast_from_encoder(encoder);
	struct backlight_device *backlight = panel->backlight;
	struct gpio_desc *gpio = panel->enable_gpio;

	if (backlight) {
		backlight->props.power = mode == DRM_MODE_DPMS_ON ?
					 FB_BLANK_UNBLANK : FB_BLANK_POWERDOWN;
		backlight_update_status(backlight);
	}

	if (gpio)
		gpiod_set_value_cansleep(gpio,
					 mode == DRM_MODE_DPMS_ON ? 1 : 0);
}

static bool panel_encoder_mode_fixup(struct drm_encoder *encoder,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
	return true;
}

static void panel_encoder_prepare(struct drm_encoder *encoder)
{
	panel_encoder_dpms(encoder, DRM_MODE_DPMS_OFF);
}

static void panel_encoder_commit(struct drm_encoder *encoder)
{
	panel_encoder_dpms(encoder, DRM_MODE_DPMS_ON);
}

static void panel_encoder_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	/* nothing needed */
}

static const struct drm_encoder_funcs panel_encoder_funcs = {
	.destroy	= panel_encoder_destroy,
};

static const struct drm_encoder_helper_funcs panel_encoder_helper_funcs = {
	.dpms		= panel_encoder_dpms,
	.prepare	= panel_encoder_prepare,
	.commit		= panel_encoder_commit,
	.mode_fixup	= panel_encoder_mode_fixup,
	.mode_set	= panel_encoder_mode_set,
};

/*
 * Connector:
 */

static void panel_connector_destroy(struct drm_connector *connector)
{
	drm_connector_unregister(connector);
	drm_connector_cleanup(connector);
}

static enum drm_connector_status panel_connector_detect(
		struct drm_connector *connector, bool force)
{
	return connector_status_connected;
}

static int panel_connector_get_modes(struct drm_connector *connector)
{
	struct drm_device *drm_dev = connector->dev;
	struct panel *panel = cast_from_connector(connector);
	struct display_timings *timings = panel->timings;
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

static int panel_connector_mode_valid(struct drm_connector *connector,
				      struct drm_display_mode *mode)
{
	return MODE_OK;
}

static struct drm_encoder*
panel_connector_best_encoder(struct drm_connector *connector)
{
	struct panel *panel = cast_from_connector(connector);

	return &panel->encoder;
}

static const struct drm_connector_funcs panel_connector_funcs = {
	.dpms		= drm_helper_connector_dpms,
	.detect		= panel_connector_detect,
	.fill_modes	= drm_helper_probe_single_connector_modes,
	.destroy	= panel_connector_destroy,
};

static const struct drm_connector_helper_funcs panel_connector_helper_funcs = {
	.get_modes	= panel_connector_get_modes,
	.mode_valid	= panel_connector_mode_valid,
	.best_encoder	= panel_connector_best_encoder,
};

/*
 * Device:
 */

static int
vpout_panel_bind(struct device *dev, struct device *master, void *data)
{
	struct drm_device *drm_dev = data;
	struct panel *panel = dev_get_drvdata(dev);

	struct drm_encoder *encoder = &panel->encoder;
	struct drm_connector *connector = &panel->connector;
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

	ret = drm_encoder_init(drm_dev, encoder, &panel_encoder_funcs,
			       DRM_MODE_ENCODER_LVDS);
	if (ret < 0)
		goto fail;

	drm_encoder_helper_add(encoder, &panel_encoder_helper_funcs);

	ret = drm_connector_init(drm_dev, connector, &panel_connector_funcs,
				 DRM_MODE_CONNECTOR_LVDS);
	if (ret)
		goto encoder_cleanup;

	drm_connector_helper_add(connector, &panel_connector_helper_funcs);

	connector->interlace_allowed = 0;
	connector->doublescan_allowed = 0;

	ret = drm_mode_connector_attach_encoder(connector, encoder);
	if (ret)
		goto connector_cleanup;

	connector->encoder = encoder;

	ret = drm_connector_register(connector);
	if (ret)
		goto connector_cleanup;

	return 0;

connector_cleanup:
	drm_connector_cleanup(connector);

encoder_cleanup:
	drm_encoder_cleanup(encoder);

fail:
	return ret;
}

static void
vpout_panel_unbind(struct device *dev, struct device *master, void *data)
{
	struct panel *panel = dev_get_drvdata(dev);

	panel_connector_destroy(&panel->connector);
	panel_encoder_destroy(&panel->encoder);
}

static const struct component_ops panel_ops = {
	.bind = vpout_panel_bind,
	.unbind = vpout_panel_unbind,
};

static int vpout_drm_panel_probe(struct platform_device *plat_dev)
{
	struct device *dev;
	struct device_node *dev_node;
	struct device_node *backlight_node;
	struct panel *panel;
	int ret;

	dev = &plat_dev->dev;
	dev_node = dev->of_node;

	if (!dev_node) {
		dev_err(dev, "device-tree data is missing\n");
		return -ENXIO;
	}

	panel = devm_kzalloc(dev, sizeof(*panel), GFP_KERNEL);
	if (!panel)
		return -ENOMEM;

	backlight_node = of_parse_phandle(dev_node, "backlight", 0);

	if (backlight_node) {
		panel->backlight = of_find_backlight_by_node(backlight_node);
		of_node_put(backlight_node);

		if (!panel->backlight)
			return -EPROBE_DEFER;

		dev_info(dev, "found backlight\n");
	}

	panel->enable_gpio = devm_gpiod_get_optional(dev, "enable",
						     GPIOD_OUT_LOW);

	if (IS_ERR(panel->enable_gpio)) {
		ret = PTR_ERR(panel->enable_gpio);
		dev_err(dev, "failed to request enable GPIO\n");
		goto fail_backlight;
	}

	if (panel->enable_gpio)
		dev_info(dev, "found enable GPIO\n");

	if (IS_ERR(devm_pinctrl_get_select_default(dev)))
		dev_warn(dev, "pins are not configured\n");

	panel->timings = of_get_display_timings(dev_node);
	if (!panel->timings) {
		dev_err(dev, "could not get panel timings\n");
		ret = -EINVAL;
		goto fail_backlight;
	}

	dev_set_drvdata(dev, panel);

	ret = component_add(dev, &panel_ops);
	if (ret)
		goto fail_timing_release;

	return 0;

fail_timing_release:
	display_timings_release(panel->timings);

fail_backlight:
	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return ret;
}

static int vpout_drm_panel_remove(struct platform_device *plat_dev)
{
	struct device *dev = &plat_dev->dev;
	struct panel *panel = dev_get_drvdata(dev);

	component_del(dev, &panel_ops);

	display_timings_release(panel->timings);

	if (panel->backlight)
		put_device(&panel->backlight->dev);

	return 0;
}

static const struct of_device_id vpout_drm_panel_of_match[] = {
	{ .compatible = "elvees,mcom02-vpout,panel", },
	{ },
};

MODULE_DEVICE_TABLE(of, vpout_drm_panel_of_match);

static struct platform_driver vpout_drm_panel_platform_driver = {
	.probe = vpout_drm_panel_probe,
	.remove = vpout_drm_panel_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = "vpout-drm-panel",
		.of_match_table = vpout_drm_panel_of_match,
	},
};

static int __init vpout_drm_panel_init(void)
{
	return platform_driver_register(&vpout_drm_panel_platform_driver);
}

static void __exit vpout_drm_panel_fini(void)
{
	platform_driver_unregister(&vpout_drm_panel_platform_driver);
}

module_init(vpout_drm_panel_init);
module_exit(vpout_drm_panel_fini);

MODULE_AUTHOR("Mikhail Rasputin <mrasputin@elvees.com");
MODULE_DESCRIPTION("ELVEES VPOUT Panel DRM Driver");
MODULE_LICENSE("GPL");
