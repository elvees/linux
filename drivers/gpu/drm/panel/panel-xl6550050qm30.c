// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>

#include <video/mipi_display.h>

struct xl_panel {
	struct drm_panel base;
	struct mipi_dsi_device *dsi;
	struct backlight_device *backlight;
	struct regulator *supply;
	struct gpio_desc *reset_gpio;
	bool prepared;
	bool enabled;
};

static inline struct xl_panel *to_xl_panel(struct drm_panel *panel)
{
	return container_of(panel, struct xl_panel, base);
}

static int xl_panel_disable(struct drm_panel *panel)
{
	struct xl_panel *xl = to_xl_panel(panel);

	if (!xl->enabled)
		return 0;

	backlight_disable(xl->backlight);

	xl->enabled = false;

	return 0;
}

static int xl_panel_unprepare(struct drm_panel *panel)
{
	struct xl_panel *xl = to_xl_panel(panel);
	int err;

	if (!xl->prepared)
		return 0;

	err = mipi_dsi_dcs_set_display_off(xl->dsi);
	if (err < 0)
		dev_err(panel->dev, "failed to set display off: %d\n", err);

	err = mipi_dsi_dcs_enter_sleep_mode(xl->dsi);
	if (err < 0)
		dev_err(panel->dev, "failed to enter sleep mode: %d\n", err);

	if (!IS_ERR(xl->reset_gpio))
		gpiod_set_value_cansleep(xl->reset_gpio, 1);

	regulator_disable(xl->supply);

	xl->prepared = false;

	return 0;
}

static int xl_panel_prepare(struct drm_panel *panel)
{
	struct xl_panel *xl = to_xl_panel(panel);
	int err;

	if (xl->prepared)
		return 0;

	err = regulator_enable(xl->supply);
	if (err < 0)
		return err;

	msleep(20);
	if (!IS_ERR(xl->reset_gpio)) {
		gpiod_set_value_cansleep(xl->reset_gpio, 1);
		usleep_range(6000, 10000);  /* Required minimum 5 ms */
		gpiod_set_value_cansleep(xl->reset_gpio, 0);
		usleep_range(10000, 15000);
	}

	err = mipi_dsi_dcs_exit_sleep_mode(xl->dsi);
	if (err < 0)
		dev_err(panel->dev, "failed to exit from sleep mode: %d\n",
			err);

	usleep_range(10000, 15000);

	err = mipi_dsi_dcs_set_display_on(xl->dsi);
	if (err < 0)
		dev_err(panel->dev, "failed to set display on: %d\n", err);

	xl->prepared = true;

	return 0;
}

static int xl_panel_enable(struct drm_panel *panel)
{
	struct xl_panel *xl = to_xl_panel(panel);

	if (xl->enabled)
		return 0;

	backlight_enable(xl->backlight);

	xl->enabled = true;

	return 0;
}

static const struct drm_display_mode default_mode = {
	.clock = 118800,
	.hdisplay = 720,
	.hsync_start = 720 + 150,
	.hsync_end = 720 + 150 + 50,
	.htotal = 720 + 150 + 50 + 140,
	.vdisplay = 1600,
	.vsync_start = 1600 + 60,
	.vsync_end = 1600 + 60 + 110,
	.vtotal = 1600 + 60 + 110 + 96,
	.vrefresh = 60,
};

static int xl_panel_get_modes(struct drm_panel *panel)
{
	struct drm_display_mode *mode;

	mode = drm_mode_duplicate(panel->drm, &default_mode);
	if (!mode) {
		dev_err(panel->drm->dev, "failed to add mode %ux%ux@%u\n",
			default_mode.hdisplay, default_mode.vdisplay,
			default_mode.vrefresh);
		return -ENOMEM;
	}

	drm_mode_set_name(mode);
	drm_mode_probed_add(panel->connector, mode);

	panel->connector->display_info.width_mm = 70;
	panel->connector->display_info.height_mm = 157;

	return 1;
}

static const struct drm_panel_funcs xl_panel_funcs = {
	.disable = xl_panel_disable,
	.unprepare = xl_panel_unprepare,
	.prepare = xl_panel_prepare,
	.enable = xl_panel_enable,
	.get_modes = xl_panel_get_modes,
};

static int xl_panel_add(struct xl_panel *xl)
{
	struct device *dev = &xl->dsi->dev;

	xl->supply = devm_regulator_get(dev, "power");
	if (IS_ERR(xl->supply))
		return PTR_ERR(xl->supply);

	xl->backlight = devm_of_find_backlight(dev);
	if (IS_ERR(xl->backlight))
		return PTR_ERR(xl->backlight);

	xl->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(xl->reset_gpio) && PTR_ERR(xl->reset_gpio) != -ENOENT)
		return PTR_ERR(xl->reset_gpio);

	drm_panel_init(&xl->base);
	xl->base.funcs = &xl_panel_funcs;
	xl->base.dev = dev;

	return drm_panel_add(&xl->base);
}

static void xl_panel_del(struct xl_panel *xl)
{
	if (xl->base.dev)
		drm_panel_remove(&xl->base);
}

static int xl_panel_probe(struct mipi_dsi_device *dsi)
{
	struct xl_panel *xl;
	int err;

	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->lanes = 4;
	dsi->channel = 0;

	xl = devm_kzalloc(&dsi->dev, sizeof(*xl), GFP_KERNEL);
	if (!xl)
		return -ENOMEM;

	xl->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, xl);

	err = xl_panel_add(xl);
	if (err < 0)
		return err;

	err = mipi_dsi_attach(dsi);
	if (err)
		drm_panel_remove(&xl->base);

	return err;
}

static int xl_panel_remove(struct mipi_dsi_device *dsi)
{
	struct xl_panel *xl = mipi_dsi_get_drvdata(dsi);
	int err;

	err = xl_panel_disable(&xl->base);
	if (err < 0)
		dev_err(&dsi->dev, "failed to disable panel: %d\n", err);

	err = mipi_dsi_detach(dsi);
	if (err < 0)
		dev_err(&dsi->dev, "failed to detach from DSI host: %d\n",
			err);

	xl_panel_del(xl);

	return 0;
}

static void xl_panel_shutdown(struct mipi_dsi_device *dsi)
{
	struct xl_panel *xl = mipi_dsi_get_drvdata(dsi);

	if (!xl)
		return;

	xl_panel_disable(&xl->base);
}

#ifdef CONFIG_OF
static const struct of_device_id xl_of_match[] = {
	{ .compatible = "xingliang,xl6550050qm30", },
	{ }
};
MODULE_DEVICE_TABLE(of, xl_of_match);
#endif

static struct mipi_dsi_driver xl_panel_driver = {
	.driver = {
		.name = "panel-xl6550050qm30",
		.of_match_table = of_match_ptr(xl_of_match),
	},
	.probe = xl_panel_probe,
	.remove = xl_panel_remove,
	.shutdown = xl_panel_shutdown,
};
module_mipi_dsi_driver(xl_panel_driver);

MODULE_DESCRIPTION("Xing Liang XL6550050QM-30 panel driver");
MODULE_ALIAS("platform:xl6550050qm30");
MODULE_LICENSE("GPL v2");
