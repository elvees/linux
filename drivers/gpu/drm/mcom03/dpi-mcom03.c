// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 * This is DPI DRM encoder driver for mcom03.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_of.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/platform_device.h>

enum MCOM03_DPI_CLK {
	MCOM03_DPI_CLK_PXL,
	MCOM03_DPI_CLK_SYS,
	MCOM03_DPI_CLK_DISP_A,
	MCOM03_DPI_CLK_CMOS0,
	MCOM03_DPI_CLK_NUM
};

static const char * const mcom03_dpi_clks_names[] = {
	[MCOM03_DPI_CLK_PXL] = "pxlclk",
	[MCOM03_DPI_CLK_SYS] = "pclk",
	[MCOM03_DPI_CLK_DISP_A] = "disp_aclk",
	[MCOM03_DPI_CLK_CMOS0] = "cmos0_clk"
};

enum {
	PORT_IN,
	PORT_OUT,
};

struct mcom03_dpi_device {
	struct device *dev;

	struct drm_bridge *bridge;
	struct drm_panel *panel;
	struct drm_encoder encoder;

	struct clk_bulk_data clocks[MCOM03_DPI_CLK_NUM];
	size_t num_clocks;
};

#define encoder_to_mcom03(ptr) \
	container_of(ptr, struct mcom03_dpi_device, encoder)

static struct clk *
mcom03_get_clk(struct mcom03_dpi_device *de, enum MCOM03_DPI_CLK id)
{
	return de->clocks[id].clk;
}

static enum drm_mode_status
mcom03_dpi_mode_valid(struct drm_encoder *encoder,
		      const struct drm_display_mode *mode)
{
	struct mcom03_dpi_device *de = encoder_to_mcom03(encoder);
	struct clk *pxl_clk = mcom03_get_clk(de, MCOM03_DPI_CLK_PXL);
	long requested_rate = mode->clock * 1000;
	long real_rate = clk_round_rate(pxl_clk, requested_rate);

	return (requested_rate == real_rate) ? MODE_OK : MODE_NOCLOCK;
}

void mcom03_dpi_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct mcom03_dpi_device *de = encoder_to_mcom03(encoder);
	struct clk *cmos0_clk = mcom03_get_clk(de, MCOM03_DPI_CLK_CMOS0);

	clk_set_rate(cmos0_clk, mode->clock * 1000);
}

void mcom03_dpi_enable(struct drm_encoder *encoder)
{
	struct mcom03_dpi_device *de = encoder_to_mcom03(encoder);

	clk_enable(mcom03_get_clk(de, MCOM03_DPI_CLK_PXL));
	clk_enable(mcom03_get_clk(de, MCOM03_DPI_CLK_CMOS0));
}

void mcom03_dpi_disable(struct drm_encoder *encoder)
{
	struct mcom03_dpi_device *de = encoder_to_mcom03(encoder);

	clk_disable(mcom03_get_clk(de, MCOM03_DPI_CLK_PXL));
	clk_disable(mcom03_get_clk(de, MCOM03_DPI_CLK_CMOS0));
}

static const struct drm_encoder_helper_funcs mcom03_dpi_encoder_helper_funcs = {
	.mode_valid = mcom03_dpi_mode_valid,
	.mode_set = mcom03_dpi_mode_set,
	.enable = mcom03_dpi_enable,
	.disable = mcom03_dpi_disable,
};

static int mcom03_dpi_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct mcom03_dpi_device *de = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &de->encoder;
	struct drm_device *drm = data;
	int ret = 0;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm,
							     dev->of_node);
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	// when drm master binds all components here, we enable all
	// required clocks for dpi interface.
	// External outputs clocks (cmos0 and pixclk) are not needed now.
	// We enable them later when usespace will use our connector.
	// So we enable internal clocks manually here instead of
	// calling single clk_bulk_enable.
	clk_enable(mcom03_get_clk(de, MCOM03_DPI_CLK_SYS));
	clk_enable(mcom03_get_clk(de, MCOM03_DPI_CLK_DISP_A));

	if (ret) {
		DRM_DEV_ERROR(dev, "failed to enable clocks: %d!\n", ret);
		return ret;
	}

	drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_DPI);
	drm_encoder_helper_add(encoder, &mcom03_dpi_encoder_helper_funcs);

	if (de->bridge) {
		ret = drm_bridge_attach(encoder, de->bridge, NULL, 0);
		if (ret) {
			DRM_DEV_ERROR(dev, "failed to attach bridge\n");
			drm_encoder_cleanup(encoder);
		}
	}

	return ret;
}

static void mcom03_dpi_unbind(struct device *dev, struct device *master,
			      void *data)
{
	struct mcom03_dpi_device *de = dev_get_drvdata(dev);

	drm_encoder_cleanup(&de->encoder);
	clk_disable(mcom03_get_clk(de, MCOM03_DPI_CLK_DISP_A));
	clk_disable(mcom03_get_clk(de, MCOM03_DPI_CLK_SYS));
}

static const struct component_ops mcom03_dpi_ops = {
	.bind	= mcom03_dpi_bind,
	.unbind	= mcom03_dpi_unbind,
};

static int mcom03_dpi_clocks_init(struct mcom03_dpi_device *de)
{
	int i, ret;

	for (i = 0; i < MCOM03_DPI_CLK_NUM; i++)
		de->clocks[i].id = mcom03_dpi_clks_names[i];

	de->num_clocks = MCOM03_DPI_CLK_NUM;

	ret = devm_clk_bulk_get(de->dev, de->num_clocks, de->clocks);
	if (ret) {
		dev_err(de->dev, "failed to find clocks in DTB: %d!\n", ret);
		return ret;
	}

	ret = clk_bulk_prepare(de->num_clocks, de->clocks);
	if (ret) {
		dev_err(de->dev, "failed to prepare clocks: %d!\n", ret);
		return ret;
	}

	return 0;
}

static int mcom03_dpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcom03_dpi_device *de;
	int ret;

	if (!dev->of_node) {
		DRM_DEV_ERROR(dev, "failed to find mcom03 DPI node in DRB\n");
		return -ENODEV;
	}

	de = devm_kzalloc(dev, sizeof(*de), GFP_KERNEL);
	if (!de)
		return -ENOMEM;

	de->dev = dev;
	platform_set_drvdata(pdev, de);

	ret = drm_of_find_panel_or_bridge(dev->of_node,
		PORT_OUT, 0, &de->panel, &de->bridge);

	if (ret) {
		DRM_DEV_INFO(dev, "failed to find endpoints, errno %d\n", ret);
		return ret;
	}

	if (!de->panel && !de->bridge) {
		DRM_DEV_ERROR(dev, "failed to find panel or bridge endpoints");
		return -ENODEV;
	}

	if (de->panel) {
		de->bridge = drm_panel_bridge_add_typed(de->panel,
			DRM_MODE_CONNECTOR_DPI);
		if (IS_ERR(de->bridge)) {
			ret = PTR_ERR(de->bridge);
			return ret;
		}
	} else if (de->bridge) {
		dev_info(dev, "Using non-panel bridge\n");
	} else {
		dev_err(dev, "failed to find bridge!\n");
		return -ENODEV;
	}

	ret = mcom03_dpi_clocks_init(de);
	if (ret)
		return ret;

	ret = component_add(&pdev->dev, &mcom03_dpi_ops);

	return ret;
}

static int mcom03_dpi_remove(struct platform_device *pdev)
{
	component_del(&pdev->dev, &mcom03_dpi_ops);
	return 0;
}

static const struct of_device_id mcom03_dpi_dt_ids[] = {
	{ .compatible = "elvees,mcom03-drm-dpi", },
	{ },
};
MODULE_DEVICE_TABLE(of, mcom03_dpi_dt_ids);

struct platform_driver mcom03_dpi_driver = {
	.probe  = mcom03_dpi_probe,
	.remove = mcom03_dpi_remove,
	.driver = {
		.name = "mcom03-encoder-dpi",
		.of_match_table = mcom03_dpi_dt_ids,
	},
};

module_platform_driver(mcom03_dpi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ELVEES MCom-03 DPI Encoder Driver");
