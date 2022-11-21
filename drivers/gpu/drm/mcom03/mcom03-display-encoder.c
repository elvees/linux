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

#define MIPI_TX_CFG_FREQ 27000000
#define MIPI_TX_REF_FREQ 27000000

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

struct mcom03_dsi_pll_cfg {
	int min;
	int max;
	u32 val;
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
	struct drm_panel *panel;
	void __iomem *dsi_base;
	void __iomem *mipi_tx_base;
	long mipi_tx_cfg_freq;
	long mipi_tx_ref_freq;
	u32 dsi_clock_freq;
	u32 pll_n;
	u32 pll_m;
	u8 vco_cntrl;
	u8 hsfreqrange;
	u8 type;
};

/* Table from DesignWare Cores MIPI D-PHY Databook
 * min/max in kbps (frequency * 2)
 */
static struct mcom03_dsi_pll_cfg hsfreqranges[] = {
	{ .min = 80000,   .max = 97125,   .val = 0, },
	{ .min = 80000,   .max = 107625,  .val = 0x10, },
	{ .min = 83125,   .max = 118125,  .val = 0x20, },
	{ .min = 92625,   .max = 128625,  .val = 0x30, },
	{ .min = 102125,  .max = 139125,  .val = 0x1, },
	{ .min = 111625,  .max = 149625,  .val = 0x11, },
	{ .min = 121125,  .max = 160125,  .val = 0x21, },
	{ .min = 130625,  .max = 170625,  .val = 0x31, },
	{ .min = 140125,  .max = 181125,  .val = 0x2, },
	{ .min = 149625,  .max = 191625,  .val = 0x12, },
	{ .min = 159125,  .max = 202125,  .val = 0x22, },
	{ .min = 168625,  .max = 212625,  .val = 0x32, },
	{ .min = 182875,  .max = 228375,  .val = 0x3, },
	{ .min = 197125,  .max = 244125,  .val = 0x13, },
	{ .min = 211375,  .max = 259875,  .val = 0x23, },
	{ .min = 225625,  .max = 275625,  .val = 0x33, },
	{ .min = 249375,  .max = 301875,  .val = 0x4, },
	{ .min = 273125,  .max = 328125,  .val = 0x14, },
	{ .min = 296875,  .max = 354375,  .val = 0x25, },
	{ .min = 320625,  .max = 380625,  .val = 0x35, },
	{ .min = 368125,  .max = 433125,  .val = 0x5, },
	{ .min = 415625,  .max = 485625,  .val = 0x16, },
	{ .min = 463125,  .max = 538125,  .val = 0x26, },
	{ .min = 510625,  .max = 590625,  .val = 0x37, },
	{ .min = 558125,  .max = 643125,  .val = 0x7, },
	{ .min = 605625,  .max = 695625,  .val = 0x18, },
	{ .min = 653125,  .max = 748125,  .val = 0x28, },
	{ .min = 700625,  .max = 800625,  .val = 0x39, },
	{ .min = 748125,  .max = 853125,  .val = 0x9, },
	{ .min = 795625,  .max = 905625,  .val = 0x19, },
	{ .min = 843125,  .max = 958125,  .val = 0x29, },
	{ .min = 890625,  .max = 1010625, .val = 0x3a, },
	{ .min = 938125,  .max = 1063125, .val = 0xa, },
	{ .min = 985625,  .max = 1115625, .val = 0x1a, },
	{ .min = 1033125, .max = 1168125, .val = 0x2a, },
	{ .min = 1080625, .max = 1220625, .val = 0x3b, },
	{ .min = 1128125, .max = 1273125, .val = 0xb, },
	{ .min = 1175625, .max = 1325625, .val = 0x1b, },
	{ .min = 1223125, .max = 1378125, .val = 0x2b, },
	{ .min = 1270625, .max = 1430625, .val = 0x3c, },
	{ .min = 1318125, .max = 1483125, .val = 0xc, },
	{ .min = 1365625, .max = 1535625, .val = 0x1c, },
	{ .min = 1413125, .max = 1588125, .val = 0x2c, },
	{ .min = 1460625, .max = 1640625, .val = 0x3d, },
	{ .min = 1508125, .max = 1693125, .val = 0xd, },
	{ .min = 1555625, .max = 1745625, .val = 0x1d, },
	{ .min = 1603125, .max = 1798125, .val = 0x2e, },
	{ .min = 1650625, .max = 1850625, .val = 0x3e, },
	{ .min = 1698125, .max = 1903125, .val = 0xe, },
	{ .min = 1745625, .max = 1955625, .val = 0x1e, },
	{ .min = 1793125, .max = 2008125, .val = 0x2f, },
	{ .min = 1840625, .max = 2060625, .val = 0x3f, },
	{ .min = 1888125, .max = 2113125, .val = 0xf, },
	{ .min = 1935625, .max = 2165625, .val = 0x40, },
	{ .min = 1983125, .max = 2218125, .val = 0x41, },
	{ .min = 2030625, .max = 2270625, .val = 0x42, },
	{ .min = 2078125, .max = 2323125, .val = 0x43, },
	{ .min = 2125625, .max = 2375625, .val = 0x44, },
	{ .min = 2173125, .max = 2428125, .val = 0x45, },
	{ .min = 2220625, .max = 2480625, .val = 0x46, },
	{ .min = 2268125, .max = 2500000, .val = 0x47, },
	{ .min = 2315625, .max = 2500000, .val = 0x48, },
	{ .min = 2363125, .max = 2500000, .val = 0x49, },
};

/* Table from DesignWare Cores MIPI D-PHY Databook
 * min/max is frequency in kHz
 */
static struct mcom03_dsi_pll_cfg vco_ranges[] = {
	{ .min = 40000,   .max = 55000,   .val = 0x3f, },
	{ .min = 52500,   .max = 82500,   .val = 0x39, },
	{ .min = 80000,   .max = 110000,  .val = 0x2f, },
	{ .min = 105000,  .max = 165000,  .val = 0x29, },
	{ .min = 160000,  .max = 220000,  .val = 0x1f, },
	{ .min = 210000,  .max = 330000,  .val = 0x19, },
	{ .min = 320000,  .max = 440000,  .val = 0xf, },
	{ .min = 420000,  .max = 660000,  .val = 0x9, },
	{ .min = 630000,  .max = 1149000, .val = 0x3, },
	{ .min = 1100000, .max = 1152000, .val = 0x1, },
	{ .min = 1150000, .max = 1250000, .val = 0x1, },
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
	u32 cfg_clk_freq_range;

	if (!priv->pll_m)
		return -EINVAL;

	cfg_clk_freq_range = (priv->mipi_tx_cfg_freq - 17000000) * 4 / 1000000;

	writel(MIPI_TX_CTRL_SHADOW_CLEAR, priv->mipi_tx_base + MIPI_TX_CTRL);
	writel(0, priv->mipi_tx_base + MIPI_TX_CTRL);
	writel((priv->hsfreqrange << 8) | cfg_clk_freq_range,
	       priv->mipi_tx_base + MIPI_TX_CLK_PARAM);

	dphy_writel(priv->dsi_plat_data.base, PHY_SLEW_0, 0x44);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_22, 0x3);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_23, 0);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_24, 0x50);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_25, 0x3);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_30,
		    0x81 | (priv->vco_cntrl << 1));
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_27,
		    0x80 | ((priv->pll_n - 1) << 3));
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_28,
		    (priv->pll_m - 2) & 0xff);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_29,
		    (priv->pll_m - 2) >> 8);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_1, 0x10);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_17, 0xc);
	dphy_writel(priv->dsi_plat_data.base, PHY_PLL_5, 0x4);

	writel(FIELD_PREP(MIPI_TX_CTRL_PLL_CLK_SEL_MASK, 1),
	       priv->mipi_tx_base + MIPI_TX_CTRL);

	return 0;
}

/* Frequency can be in one or two ranges. Function chooses range where
 * frequency is closest to center. */
static u32 drm_mcom03_get_best_cfg(int clock,
				   struct mcom03_dsi_pll_cfg *ranges,
				   int len)
{
	int i;
	struct mcom03_dsi_pll_cfg *suitable_ranges[2] = {NULL, NULL};

	for (i = 0; i < len; i++) {
		if (clock >= ranges[i].min && clock <= ranges[i].max) {
			if (suitable_ranges[0]) {
				suitable_ranges[1] = &ranges[i];
				break;
			}
			suitable_ranges[0] = &ranges[i];
		}
	}

	if (!suitable_ranges[1])
		return suitable_ranges[0] ? suitable_ranges[0]->val : 0;

	/* If clock is in two ranges then we choose range where clock further
	 * from min/max values */
	if ((suitable_ranges[0]->max - clock) <
	    (clock - suitable_ranges[1]->min))
		return suitable_ranges[1]->val;
	else
		return suitable_ranges[0]->val;
}

static int drm_mcom03_get_line_mbps(void *priv_data,
				    struct drm_display_mode *mode,
				    unsigned long mode_flags, u32 lanes,
				    u32 format, unsigned int *lane_mbps)
{
	struct mcom03_priv *priv = (struct mcom03_priv *)priv_data;
	int bpp = mipi_dsi_pixel_format_to_bpp(format);
	int dsi_clock;
	int dsi_clock_actual = 0;
	u32 pll_n, pll_n_mul, pll_m = 0;

	WARN_ON(bpp < 0);
	if (priv->dsi_clock_freq)
		dsi_clock = priv->dsi_clock_freq / 1000;
	else
		dsi_clock = DIV_ROUND_UP(mode->clock * bpp, lanes * 2);

	priv->vco_cntrl = drm_mcom03_get_best_cfg(dsi_clock, vco_ranges,
						  ARRAY_SIZE(vco_ranges));
	priv->hsfreqrange = drm_mcom03_get_best_cfg(dsi_clock * 2,
						    hsfreqranges,
						    ARRAY_SIZE(hsfreqranges));
	if (!priv->vco_cntrl || !priv->hsfreqrange) {
		dev_err(priv->dev, "DSI frequency (%d kHz) is out of range\n",
			dsi_clock);
		return -EINVAL;
	}

	/* pll_freq = ref * pll_m / (pll_n * pll_n_mul)
	 * pll_n_mul = 2 ^^ vco_cntrl[5:4]
	 * 64 <= pll_m <= 625
	 * 1 <= pll_n <= 16
	 * 2 MHz <= (ref / pll_n) <= 8 MHz
	 */
	pll_n_mul = BIT(priv->vco_cntrl >> 4);
	for (pll_n = 1; pll_n < 16; pll_n++) {
		if ((priv->mipi_tx_ref_freq / pll_n) < 2000000 ||
		    DIV_ROUND_UP(priv->mipi_tx_ref_freq, pll_n) > 8000000)
			continue;

		/* If dsi-clock-frequency is specified then must be used
		 * frequency not above than dsi-clock-frequency.
		 * If dsi_clock is calculated from pixel clock then frequency
		 * must be not less than this calculated value. */
		if (priv->dsi_clock_freq)
			pll_m = dsi_clock * pll_n * pll_n_mul /
				(priv->mipi_tx_ref_freq / 1000);
		else
			pll_m = DIV_ROUND_UP(dsi_clock * pll_n * pll_n_mul,
					     priv->mipi_tx_ref_freq / 1000);

		dsi_clock_actual = (priv->mipi_tx_ref_freq / 1000) * pll_m /
				   (pll_n * pll_n_mul);
		if (pll_m >= 64 && pll_m <= 625 &&
		    dsi_clock_actual >= 40000 && dsi_clock_actual <= 1250000)
			break;

		pll_m = 0;
	}

	priv->pll_n = pll_n;
	priv->pll_m = pll_m;
	if (!pll_m) {
		dev_err(priv->dev, "failed to setup PLL for DSI clock %d kHz\n",
			dsi_clock);
		return -EINVAL;
	}

	*lane_mbps = dsi_clock_actual * 2 / 1000;
	dev_info(priv->dev, "DSI frequency %d kHz (%d Mbps)\n",
		 dsi_clock_actual, *lane_mbps);

	return 0;
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
		dev_err(&pdev->dev,
			"failed to get video-output string property\n");
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

		if (device_property_present(priv->dev, "dsi-clock-frequency")) {
			ret = device_property_read_u32(priv->dev,
						       "dsi-clock-frequency",
						       &priv->dsi_clock_freq);
			if (ret) {
				dev_err(priv->dev,
					"failed to get dsi-clock-frequency");
				return ret;
			}
		}

		dev_info(priv->dev, "using DSI\n");
	} else {
		dev_err(&pdev->dev,
			"invalid video-output value. Can be 'rgb' or 'dsi'\n");
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

	priv->mipi_tx_cfg_freq = clk_round_rate(priv->mipi_tx_cfg_clock,
						MIPI_TX_CFG_FREQ);
	ret = clk_set_rate(priv->mipi_tx_cfg_clock, priv->mipi_tx_cfg_freq);
	if (ret) {
		dev_err(priv->dev, "failed to set rate for mipi_tx_cfg %ld\n",
			priv->mipi_tx_cfg_freq);
		return ret;
	}

	priv->mipi_tx_ref_clock = devm_clk_get(&pdev->dev, "mipi_tx_ref");
	if (IS_ERR(priv->mipi_tx_ref_clock)) {
		dev_err(&pdev->dev, "failed to get mipi_tx_ref clock\n");
		return PTR_ERR(priv->mipi_tx_ref_clock);
	}

	priv->mipi_tx_ref_freq = clk_round_rate(priv->mipi_tx_ref_clock,
						MIPI_TX_REF_FREQ);
	ret = clk_set_rate(priv->mipi_tx_ref_clock, priv->mipi_tx_ref_freq);
	if (ret) {
		dev_err(priv->dev, "failed to set rate for mipi_tx_ref %ld\n",
			priv->mipi_tx_ref_freq);
		return ret;
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
