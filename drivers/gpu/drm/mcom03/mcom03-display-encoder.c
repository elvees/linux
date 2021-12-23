// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2021 RnD Center "ELVEES", JSC
 *
 * This is intermediate driver between malidp driver and drivers with
 * bridge interface.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/module.h>
#include <drm/bridge/dw_mipi_dsi.h>

#define PHY_TST_CTRL0 0xb4
#define PHY_TESTCLK BIT(1)

#define PHY_TST_CTRL1 0xb8
#define PHY_TESTEN BIT(16)

#define MIPI_TX_CTRL 0
#define MIPI_TX_CLK_PARAM 0x4
#define MIPI_TX_PLL_STATUS0 0x8
#define MIPI_TX_PLL_STATUS1 0xc
#define MIPI_TX_TEST_CTRL 0x10
#define MIPI_TX_TEST_DATA 0x14

#define MIPI_TX_CTRL_SHADOW_CLEAR BIT(21)
#define MIPI_TX_CTRL_PLL_CLK_SEL_MASK GENMASK(26, 25)

#define PHY_PLL_1 0x15e
#define PHY_PLL_5 0x162
#define PHY_PLL_17 0x16e
#define PHY_PLL_22 0x173
#define PHY_PLL_23 0x174
#define PHY_PLL_24 0x175
#define PHY_PLL_25 0x176
#define PHY_PLL_26 0x177
#define PHY_PLL_27 0x178
#define PHY_PLL_28 0x179
#define PHY_PLL_29 0x17a
#define PHY_PLL_30 0x17b
#define PHY_PLL_31 0x17c
#define PHY_SLEW_0 0x26b

#define OUTPUT_RGB 0
#define OUTPUT_DSI 1

struct mcom03_dsi_mode {
	int mode_clock_max;
	int lanes;
	int pll_freq;
	u8 vco_cntrl;
	u8 hs_freq_range;
};

struct mcom03_priv {
	struct device *dev;
	struct clk *pixel_clock;
	struct clk *mipi_tx_cfg_clock;
	struct clk *mipi_tx_ref_clock;
	struct clk *mipi_txclkesc_clock;
	struct clk *pclk_clock;
	struct clk *disp_aclk_clock;
	struct clk *cmos0_clk_clock;
	struct drm_encoder encoder;
	struct drm_bridge *output_bridge;
	struct dw_mipi_dsi *dsi;
	struct dw_mipi_dsi_plat_data dsi_plat_data;
	struct mcom03_dsi_mode *dsi_mode;
	struct drm_panel *panel;
	void __iomem *dsi_base;
	void __iomem *mipi_tx_base;
	int mode_clock;
	u8 type;
};

static struct mcom03_dsi_mode dsi_modes[] = {
	{
		.mode_clock_max = 148500,
		.lanes = 4,
		.pll_freq = 499500,
		.vco_cntrl = 0x9,
		.hs_freq_range = 0xa,
	},
};

static enum drm_mode_status
drm_mcom03_mode_valid(struct drm_encoder *crtc,
		      const struct drm_display_mode *mode)
{
	struct mcom03_priv *priv = container_of(crtc, struct mcom03_priv,
						encoder);
	long requested_rate = mode->clock * 1000;
	long real_rate = clk_round_rate(priv->pixel_clock, requested_rate);

	return (requested_rate == real_rate) ? MODE_OK : MODE_NOCLOCK;
}

void drm_mcom03_mode_set(struct drm_encoder *encoder,
			 struct drm_display_mode *mode,
			 struct drm_display_mode *adjusted_mode)
{
	struct mcom03_priv *priv = container_of(encoder, struct mcom03_priv,
						encoder);

	if (priv->type == OUTPUT_RGB)
		clk_set_rate(priv->cmos0_clk_clock, mode->clock * 1000);
}

void drm_mcom03_enable(struct drm_encoder *encoder)
{
	struct mcom03_priv *priv = container_of(encoder, struct mcom03_priv,
						encoder);

	if (priv->type == OUTPUT_RGB)
		clk_prepare_enable(priv->cmos0_clk_clock);
}

void drm_mcom03_disable(struct drm_encoder *encoder)
{
	struct mcom03_priv *priv = container_of(encoder, struct mcom03_priv,
						encoder);

	if (priv->type == OUTPUT_RGB)
		clk_disable_unprepare(priv->cmos0_clk_clock);
}

static const struct drm_encoder_helper_funcs drm_mcom03_encoder_helper_funcs = {
	.mode_valid = drm_mcom03_mode_valid,
	.mode_set = drm_mcom03_mode_set,
	.enable = drm_mcom03_enable,
	.disable = drm_mcom03_disable,
};

static const struct drm_encoder_funcs drm_mcom03_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static void dphy_writel(void __iomem *base, u32 reg, u32 data)
{
	writel(PHY_TESTEN, base + PHY_TST_CTRL1);
	writel(PHY_TESTCLK, base + PHY_TST_CTRL0);
	writel(PHY_TESTEN, base + PHY_TST_CTRL1);
	writel(0, base + PHY_TST_CTRL0);
	writel(reg >> 8, base + PHY_TST_CTRL1);
	writel(PHY_TESTCLK, base + PHY_TST_CTRL0);
	writel(0, base + PHY_TST_CTRL0);
	writel(PHY_TESTEN | (reg >> 8), base + PHY_TST_CTRL1);
	writel(PHY_TESTCLK, base + PHY_TST_CTRL0);
	writel(PHY_TESTEN | (reg & 0xff), base + PHY_TST_CTRL1);
	writel(0, base + PHY_TST_CTRL0);
	writel(reg & 0xff, base + PHY_TST_CTRL1);
	writel(data, base + PHY_TST_CTRL1);
	writel(PHY_TESTCLK, base + PHY_TST_CTRL0);
	writel(0, base + PHY_TST_CTRL0);
}

static int drm_mcom03_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct mcom03_priv *priv = dev_get_drvdata(dev);
	struct drm_device *drm = data;
	struct drm_encoder *encoder = &priv->encoder;
	struct platform_device *pdev = to_platform_device(dev);
	int encoder_type;
	int ret = 0;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm,
							     dev->of_node);
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	encoder_type = priv->type == OUTPUT_RGB ? DRM_MODE_ENCODER_DPI :
				     DRM_MODE_ENCODER_DSI;
	drm_encoder_helper_add(encoder, &drm_mcom03_encoder_helper_funcs);
	drm_encoder_init(drm, encoder, &drm_mcom03_encoder_funcs,
			 encoder_type, NULL);

	if (priv->type == OUTPUT_RGB) {
		ret = drm_bridge_attach(encoder, priv->output_bridge, NULL);
		if (ret)
			dev_err(priv->dev, "failed to bridge attach (%d)\n",
				ret);

		return ret;
	}

	priv->dsi = dw_mipi_dsi_bind(pdev, encoder, &priv->dsi_plat_data);
	if (IS_ERR(priv->dsi)) {
		ret = PTR_ERR(priv->dsi);
		dev_err(priv->dev, "failed to bind DSI (%d)\n", ret);
		drm_encoder_cleanup(encoder);
	}

	return ret;
}

static void drm_mcom03_unbind(struct device *dev, struct device *master,
			      void *data)
{
	struct mcom03_priv *priv = dev_get_drvdata(dev);

	dw_mipi_dsi_unbind(priv->dsi);
	drm_encoder_cleanup(&priv->encoder);
}

static const struct component_ops drm_mcom03_ops = {
	.bind	= drm_mcom03_bind,
	.unbind	= drm_mcom03_unbind,
};

static int drm_mcom03_phy_init(void *priv_data)
{
	struct mcom03_priv *priv = (struct mcom03_priv *)priv_data;
	u32 mipi_tx_cfg_freq;
	u32 mipi_tx_ref_freq;
	u32 cfg_clk_freq_range;
	u32 pll_n = 5;
	u32 pll_m;
	int ret;

	if (!priv->dsi_mode)
		return -EINVAL;

	mipi_tx_cfg_freq = clk_round_rate(priv->mipi_tx_cfg_clock, 27000000);
	ret = clk_set_rate(priv->mipi_tx_cfg_clock, mipi_tx_cfg_freq);
	if (ret) {
		dev_err(priv->dev, "failed to set rate for mipi_tx_cfg %d\n",
			mipi_tx_cfg_freq);
		return ret;
	}

	mipi_tx_ref_freq = clk_round_rate(priv->mipi_tx_ref_clock, 27000000);
	ret = clk_set_rate(priv->mipi_tx_ref_clock, mipi_tx_ref_freq);
	if (ret) {
		dev_err(priv->dev, "failed to set rate for mipi_tx_ref %d\n",
			mipi_tx_ref_freq);
		return ret;
	}

	/* TODO Need to calculate pll_n
	 * pll_freq = ref * pll_m / pll_n
	 * 64 <= pll_m <= 625
	 * 1 <= pll_n <= 16
	 * 2 MHz <= (ref / pll_n) <= 8 MHz
	 */
	pll_m = DIV_ROUND_UP(priv->dsi_mode->pll_freq * pll_n,
			     mipi_tx_ref_freq / 1000);

	cfg_clk_freq_range = (mipi_tx_cfg_freq - 17000000) * 4 / 1000000;

	writel(MIPI_TX_CTRL_SHADOW_CLEAR, priv->mipi_tx_base + MIPI_TX_CTRL);
	writel(0, priv->mipi_tx_base + MIPI_TX_CTRL);
	writel((priv->dsi_mode->hs_freq_range << 8) | cfg_clk_freq_range,
	       priv->mipi_tx_base + MIPI_TX_CLK_PARAM);

	dphy_writel(priv->dsi_plat_data.base, PHY_SLEW_0, 0x44);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_22, 0x3);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_23, 0);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_24, 0x50);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_25, 0x3);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_30,
		    0x81 | (priv->dsi_mode->vco_cntrl << 1));
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_27,
		    0x80 | ((pll_n - 1) << 3));
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_28, (pll_m - 2) & 0xff);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_29, (pll_m - 2) >> 8);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_1, 0x10);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_17, 0xc);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_5, 0x4);

	writel(FIELD_PREP(MIPI_TX_CTRL_PLL_CLK_SEL_MASK, 1),
	       priv->mipi_tx_base + MIPI_TX_CTRL);

	return 0;
}

static int drm_mcom03_get_line_mbps(void *priv_data,
				    struct drm_display_mode *mode,
				    unsigned long mode_flags, u32 lanes,
				    u32 format, unsigned int *lane_mbps)
{
	struct mcom03_priv *priv = (struct mcom03_priv *)priv_data;
	int i;

	for (i = 0; i < ARRAY_SIZE(dsi_modes); i++) {
		if (dsi_modes[i].lanes == lanes &&
		    dsi_modes[i].mode_clock_max >= mode->clock) {
			priv->dsi_mode = &dsi_modes[i];
			*lane_mbps = (dsi_modes[i].pll_freq * 2) / 1000;
			return 0;
		}
	}
	priv->dsi_mode = NULL;

	return -EINVAL;
}

static const struct dw_mipi_dsi_phy_ops drm_mcom03_phy_ops = {
	.init = drm_mcom03_phy_init,
	.get_lane_mbps = drm_mcom03_get_line_mbps,
};

static enum drm_mode_status drm_mcom03_dsi_mode_valid(void *priv_data,
		const struct drm_display_mode *mode)
{
	return MODE_OK;
}

static int drm_mcom03_setup_dsi(struct platform_device *pdev)
{
	struct mcom03_priv *priv =
		(struct mcom03_priv *)dev_get_drvdata(&pdev->dev);
	struct resource *res;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsi");
	if (!res)
		return -ENODEV;

	priv->dsi_plat_data.max_data_lanes = 4;
	priv->dsi_plat_data.mode_valid = &drm_mcom03_dsi_mode_valid;
	priv->dsi_plat_data.phy_ops = &drm_mcom03_phy_ops;
	priv->dsi_plat_data.base = devm_ioremap_resource(&pdev->dev, res);
	priv->dsi_plat_data.priv_data = priv;
	if (IS_ERR(priv->dsi_plat_data.base))
		return -ENODEV;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mipi-tx");
	if (!res)
		return -ENODEV;

	priv->mipi_tx_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->dsi_plat_data.base))
		return -ENODEV;

	return 0;
}

static int drm_mcom03_probe(struct platform_device *pdev)
{
	struct mcom03_priv *priv;
	struct device_node *np;
	struct device_node *panel_node;
	const char *video_output;
	int ret;

	if (!pdev->dev.of_node)
		return -ENODEV;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	dev_set_drvdata(&pdev->dev, priv);
	if (of_property_read_string(pdev->dev.of_node, "video-output",
				    &video_output)) {
		dev_err(&pdev->dev, "failed to get video-output string property\n");
		return -EINVAL;
	}

	if (!strcmp(video_output, "rgb")) {
		priv->type = OUTPUT_RGB;
		panel_node = of_parse_phandle(pdev->dev.of_node,
					      "panel", 0);
		if (panel_node) {
			priv->panel = of_drm_find_panel(panel_node);
			if (IS_ERR(priv->panel))
				return PTR_ERR(priv->panel);

			priv->output_bridge = drm_panel_bridge_add(priv->panel,
					DRM_MODE_CONNECTOR_DPI);
			if (IS_ERR(priv->output_bridge))
				return PTR_ERR(priv->output_bridge);

			dev_info(priv->dev, "using panel %s\n",
				 panel_node->name);
		} else {
			np = of_graph_get_remote_node(pdev->dev.of_node, 2, 0);
			if (np) {
				priv->output_bridge = of_drm_find_bridge(np);
				if (!priv->output_bridge)
					return -EPROBE_DEFER;

				dev_info(priv->dev, "using bridge %s\n",
					priv->output_bridge->of_node->name);
			} else {
				dev_err(priv->dev,
					"failed to get node on port 2\n");
				return -EINVAL;
			}
		}
	} else if (!strcmp(video_output, "dsi")) {
		priv->type = OUTPUT_DSI;
		ret = drm_mcom03_setup_dsi(pdev);
		if (ret)
			return ret;

		dev_info(priv->dev, "using DSI\n");
	} else {
		dev_err(&pdev->dev, "invalid video-output value. Can be 'rgb' or 'dsi'\n");
		return -EINVAL;
	}

	priv->pixel_clock = devm_clk_get(&pdev->dev, "pxlclk");
	if (IS_ERR(priv->pixel_clock)) {
		dev_err(&pdev->dev, "failed to get pxlclk clock\n");
		return PTR_ERR(priv->pixel_clock);
	}

	priv->mipi_tx_cfg_clock = devm_clk_get(&pdev->dev, "mipi_tx_cfg");
	if (IS_ERR(priv->mipi_tx_cfg_clock)) {
		dev_err(&pdev->dev, "failed to get mipi_tx_cfg clock\n");
		return PTR_ERR(priv->mipi_tx_cfg_clock);
	}

	priv->mipi_tx_ref_clock = devm_clk_get(&pdev->dev, "mipi_tx_ref");
	if (IS_ERR(priv->mipi_tx_ref_clock)) {
		dev_err(&pdev->dev, "failed to get mipi_tx_ref clock\n");
		return PTR_ERR(priv->mipi_tx_ref_clock);
	}

	priv->mipi_txclkesc_clock = devm_clk_get(&pdev->dev, "mipi_txclkesc");
	if (IS_ERR(priv->mipi_txclkesc_clock)) {
		dev_err(&pdev->dev, "failed to get mipi_txclkesc clock\n");
		return PTR_ERR(priv->mipi_txclkesc_clock);
	}

	/* In MCom-03 name of this clock is sys_aclk, but we named it pclk
	 * because dw-mipi-dsi driver requires pclk clock */
	priv->pclk_clock = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(priv->pclk_clock)) {
		dev_err(&pdev->dev, "failed to get pclk clock\n");
		return PTR_ERR(priv->pclk_clock);
	}

	priv->disp_aclk_clock = devm_clk_get(&pdev->dev, "disp_aclk");
	if (IS_ERR(priv->disp_aclk_clock)) {
		dev_err(&pdev->dev, "failed to get disp_aclk clock\n");
		return PTR_ERR(priv->disp_aclk_clock);
	}

	priv->cmos0_clk_clock = devm_clk_get(&pdev->dev, "cmos0_clk");
	if (IS_ERR(priv->cmos0_clk_clock)) {
		dev_err(&pdev->dev, "failed to get cmos0_clk clock\n");
		return PTR_ERR(priv->cmos0_clk_clock);
	}

	ret = clk_prepare_enable(priv->pclk_clock);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable pclk clock\n");
		return ret;
	}
	ret = clk_prepare_enable(priv->disp_aclk_clock);
	if (ret) {
		dev_err(&pdev->dev, "failed to enable disp_aclk clock\n");
		goto disable_sys_aclk;
	}
	if (priv->type == OUTPUT_DSI) {
		ret = clk_prepare_enable(priv->mipi_tx_cfg_clock);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable mipi_tx_cfg clock\n");
			goto disable_disp_aclk;
		}
		ret = clk_prepare_enable(priv->mipi_tx_ref_clock);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable mipi_tx_ref clock\n");
			goto disable_mipi_tx_cfg;
		}
		ret = clk_prepare_enable(priv->mipi_txclkesc_clock);
		if (ret) {
			dev_err(&pdev->dev, "failed to enable mipi_txclkesc clock\n");
			goto disable_mipi_tx_ref;
		}
	}

	ret = component_add(&pdev->dev, &drm_mcom03_ops);
	if (ret) {
		if (priv->type == OUTPUT_DSI)
			goto disable_mipi_txclkesc;
		else
			goto disable_disp_aclk;
	}

	return ret;

disable_mipi_txclkesc:
	clk_disable_unprepare(priv->mipi_txclkesc_clock);
disable_mipi_tx_ref:
	clk_disable_unprepare(priv->mipi_tx_ref_clock);
disable_mipi_tx_cfg:
	clk_disable_unprepare(priv->mipi_tx_cfg_clock);
disable_disp_aclk:
	clk_disable_unprepare(priv->disp_aclk_clock);
disable_sys_aclk:
	clk_disable_unprepare(priv->pclk_clock);
	return ret;
}

static int drm_mcom03_remove(struct platform_device *pdev)
{
	struct mcom03_priv *priv =
		(struct mcom03_priv *)dev_get_drvdata(&pdev->dev);

	component_del(&pdev->dev, &drm_mcom03_ops);
	if (priv->type == OUTPUT_DSI) {
		clk_disable_unprepare(priv->mipi_txclkesc_clock);
		clk_disable_unprepare(priv->mipi_tx_cfg_clock);
		clk_disable_unprepare(priv->mipi_tx_ref_clock);
	}
	clk_disable_unprepare(priv->disp_aclk_clock);
	clk_disable_unprepare(priv->pclk_clock);

	return 0;
}

static const struct of_device_id drm_mcom03_dt_ids[] = {
	{ .compatible = "elvees,mcom03-display-encoder", },
	{ },
};
MODULE_DEVICE_TABLE(of, drm_mcom03_dt_ids);

struct platform_driver drm_mcom03_pltfm_driver = {
	.probe  = drm_mcom03_probe,
	.remove = drm_mcom03_remove,
	.driver = {
		.name = "mcom03-display-encoder",
		.of_match_table = drm_mcom03_dt_ids,
	},
};

module_platform_driver(drm_mcom03_pltfm_driver);

MODULE_LICENSE("GPL");
