// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 * This is driver to configure DSI on DesignWare MIPI IP-block in mcom03 SoC.
 */

#include <drm/drm_atomic_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_of.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_simple_kms_helper.h>
#include <linux/clk.h>
#include <linux/component.h>
#include <linux/module.h>
#include <linux/platform_device.h>

#include <drm/bridge/dw_mipi_dsi.h>

#define MIPI_TX_REF_FREQ 27000000
#define MIPI_TX_CFG_FREQ 27000000

#define PHY_TST_CTRL0 0xb4
#define PHY_TESTCLK BIT(1)

#define PHY_TST_CTRL1 0xb8
#define PHY_TESTEN BIT(16)

#define MIPI_TX_CTRL 0
#define MIPI_TX_CLK_PARAM 0x4

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

enum MCOM03_DSI_CLK {
	MCOM03_DSI_CLK_PXL,
	MCOM03_DSI_CLK_SYS,
	MCOM03_DSI_CLK_DISP_A,
	MCOM03_DSI_CLK_MIPI_TX_CFG,
	MCOM03_DSI_CLK_MIPI_TX_REF,
	MCOM03_DSI_CLK_MIPI_TX_ESC,
	MCOM03_DSI_CLK_NUM
};

static const char * const mcom03_dsi_clks_names[] = {
	[MCOM03_DSI_CLK_PXL] = "pxlclk",
	[MCOM03_DSI_CLK_SYS] = "pclk",
	[MCOM03_DSI_CLK_DISP_A] = "disp_aclk",
	[MCOM03_DSI_CLK_MIPI_TX_CFG] = "mipi_tx_cfg",
	[MCOM03_DSI_CLK_MIPI_TX_REF] = "mipi_tx_ref",
	[MCOM03_DSI_CLK_MIPI_TX_ESC] = "mipi_txclkesc",
};

struct mcom03_dsi_device {
	struct device *dev;
	struct drm_encoder encoder;

	struct clk_bulk_data clocks[MCOM03_DSI_CLK_NUM];
	size_t num_clocks;

	struct dw_mipi_dsi *dsi;
	struct dw_mipi_dsi_plat_data dsi_plat_data;

	void __iomem *dsi_base;
	void __iomem *mipi_tx_base;

	u32 mipi_tx_cfg_freq;
	u32 mipi_tx_ref_freq;
	u32 dsi_clock_freq;
	u32 pll_n;
	u32 pll_m;
	u8 vco_cntrl;
	u8 hsfreqrange;
};

struct mcom03_dsi_pll_cfg {
	int min;
	int max;
	u32 val;
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

struct hstt {
	unsigned int maxfreq;
	struct dw_mipi_dsi_dphy_timing timing;
};

#define HSTT(_maxfreq, _c_lp2hs, _c_hs2lp, _d_lp2hs, _d_hs2lp)	\
{					\
	.maxfreq = _maxfreq,		\
	.timing = {			\
		.clk_lp2hs = _c_lp2hs,	\
		.clk_hs2lp = _c_hs2lp,	\
		.data_lp2hs = _d_lp2hs,	\
		.data_hs2lp = _d_hs2lp,	\
	}				\
}

/* Synopsys DesignWare MIPI DPHY
 * Table A-3 High-Speed Transition Times */
struct hstt hstt_table[] = {
	HSTT(80, 21, 17, 15, 10),
	HSTT(90, 23, 17, 16, 10),
	HSTT(100, 22, 17, 16, 10),
	HSTT(110, 25, 18, 17, 11),
	HSTT(120, 26, 20, 18, 11),
	HSTT(130, 27, 19, 19, 11),
	HSTT(140, 27, 19, 19, 11),
	HSTT(150, 28, 20, 20, 12),
	HSTT(160, 30, 21, 22, 13),
	HSTT(170, 30, 21, 23, 13),
	HSTT(180, 31, 21, 23, 13),
	HSTT(190, 32, 22, 24, 13),
	HSTT(205, 35, 22, 25, 13),
	HSTT(220, 37, 26, 27, 15),
	HSTT(235, 38, 28, 27, 16),
	HSTT(250, 41, 29, 30, 17),
	HSTT(275, 43, 29, 32, 18),
	HSTT(300, 45, 32, 35, 19),
	HSTT(325, 48, 33, 36, 18),
	HSTT(350, 51, 35, 40, 20),
	HSTT(400, 59, 37, 44, 21),
	HSTT(450, 65, 40, 49, 23),
	HSTT(500, 71, 41, 54, 24),
	HSTT(550, 77, 44, 57, 26),
	HSTT(600, 82, 46, 64, 27),
	HSTT(650, 87, 48, 67, 28),
	HSTT(700, 94, 52, 71, 29),
	HSTT(750, 99, 52, 75, 31),
	HSTT(800, 105, 55, 82, 32),
	HSTT(850, 110, 58, 85, 32),
	HSTT(900, 115, 58, 88, 35),
	HSTT(950, 120, 62, 93, 36),
	HSTT(1000, 128, 63, 99, 38),
	HSTT(1050, 132, 65, 102, 38),
	HSTT(1100, 138, 67, 106, 39),
	HSTT(1150, 146, 69, 112, 42),
	HSTT(1200, 151, 71, 117, 43),
	HSTT(1250, 153, 74, 120, 45),
	HSTT(1300, 160, 73, 124, 46),
	HSTT(1350, 165, 76, 130, 47),
	HSTT(1400, 172, 78, 134, 49),
	HSTT(1450, 177, 80, 138, 49),
	HSTT(1500, 183, 81, 143, 52),
	HSTT(1550, 191, 84, 147, 52),
	HSTT(1600, 194, 85, 152, 52),
	HSTT(1650, 201, 86, 155, 53),
	HSTT(1700, 208, 88, 161, 53),
	HSTT(1750, 212, 89, 165, 53),
	HSTT(1800, 220, 90, 171, 54),
	HSTT(1850, 223, 92, 175, 54),
	HSTT(1900, 231, 91, 180, 55),
	HSTT(1950, 236, 95, 185, 56),
	HSTT(2000, 243, 97, 190, 56),
	HSTT(2050, 248, 99, 194, 58),
	HSTT(2100, 252, 100, 199, 59),
	HSTT(2150, 259, 102, 204, 61),
	HSTT(2200, 266, 105, 210, 62),
	HSTT(2250, 269, 109, 213, 63),
	HSTT(2300, 272, 109, 217, 65),
	HSTT(2350, 281, 112, 225, 66),
	HSTT(2400, 283, 115, 226, 66),
	HSTT(2450, 282, 115, 226, 67),
	HSTT(2500, 281, 118, 227, 67)
};

static struct clk *
mcom03_get_clk(struct mcom03_dsi_device *dev,
	enum MCOM03_DSI_CLK id)
{
	return dev->clocks[id].clk;
}

static inline void dsi_write(struct mcom03_dsi_device *dev, u32 reg, u32 val)
{
	writel(val, dev->dsi_plat_data.base + reg);
}

static void dphy_writel(struct mcom03_dsi_device *dev, u32 code, u32 data)
{
	dsi_write(dev, PHY_TST_CTRL1, PHY_TESTEN);
	dsi_write(dev, PHY_TST_CTRL0, PHY_TESTCLK);
	dsi_write(dev, PHY_TST_CTRL1, PHY_TESTEN);
	dsi_write(dev, PHY_TST_CTRL0, 0);

	dsi_write(dev, PHY_TST_CTRL1, code >> 8);
	dsi_write(dev, PHY_TST_CTRL0, PHY_TESTCLK);

	dsi_write(dev, PHY_TST_CTRL0, 0);

	dsi_write(dev, PHY_TST_CTRL1, PHY_TESTEN | (code >> 8));
	dsi_write(dev, PHY_TST_CTRL0, PHY_TESTCLK);
	dsi_write(dev, PHY_TST_CTRL1, PHY_TESTEN | (code & 0xff));

	dsi_write(dev, PHY_TST_CTRL0, 0);
	dsi_write(dev, PHY_TST_CTRL1, code & 0xff);
	dsi_write(dev, PHY_TST_CTRL1, data);

	dsi_write(dev, PHY_TST_CTRL0, PHY_TESTCLK);
	dsi_write(dev, PHY_TST_CTRL0, 0);
}

static int mcom03_dsi_phy_init(void *data)
{
	struct mcom03_dsi_device *de = (struct mcom03_dsi_device *)data;
	u32 cfg_clk_freq_range;

	if (!de->pll_m)
		return -EINVAL;

	cfg_clk_freq_range = (de->mipi_tx_cfg_freq - 17000000) * 4 / 1000000;

	writel(MIPI_TX_CTRL_SHADOW_CLEAR, de->mipi_tx_base + MIPI_TX_CTRL);
	writel(0, de->mipi_tx_base + MIPI_TX_CTRL);
	writel((de->hsfreqrange << 8) | cfg_clk_freq_range,
	       de->mipi_tx_base + MIPI_TX_CLK_PARAM);

	dphy_writel(de, PHY_SLEW_0, 0x44);
	dphy_writel(de, PHY_PLL_22, 0x3);
	dphy_writel(de, PHY_PLL_23, 0);
	dphy_writel(de, PHY_PLL_24, 0x50);
	dphy_writel(de, PHY_PLL_25, 0x3);
	dphy_writel(de, PHY_PLL_30, 0x81 | (de->vco_cntrl << 1));
	dphy_writel(de, PHY_PLL_27, 0x80 | ((de->pll_n - 1) << 3));
	dphy_writel(de, PHY_PLL_28, (de->pll_m - 2) & 0xff);
	dphy_writel(de, PHY_PLL_29, (de->pll_m - 2) >> 8);
	dphy_writel(de, PHY_PLL_1, 0x10);
	dphy_writel(de, PHY_PLL_17, 0xc);
	dphy_writel(de, PHY_PLL_5, 0x4);

	writel(FIELD_PREP(MIPI_TX_CTRL_PLL_CLK_SEL_MASK, 1),
	       de->mipi_tx_base + MIPI_TX_CTRL);

	return 0;
}

/* Frequency can be in one or two ranges. Function chooses range where
 * frequency is closest to center. */
static u32 mcom03_dsi_get_best_cfg(int clock,
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

static int mcom03_dsi_get_line_mbps(void *de_data,
				    const struct drm_display_mode *mode,
				    unsigned long mode_flags, u32 lanes,
				    u32 format, unsigned int *lane_mbps)
{
	struct mcom03_dsi_device *de = (struct mcom03_dsi_device *)de_data;
	int bpp = mipi_dsi_pixel_format_to_bpp(format);
	int dsi_clock;
	int dsi_clock_actual = 0;
	u32 pll_n, pll_n_mul, pll_m = 0;

	WARN_ON(bpp < 0);
	if (de->dsi_clock_freq)
		dsi_clock = de->dsi_clock_freq / 1000;
	else
		dsi_clock = DIV_ROUND_UP(mode->clock * bpp, lanes * 2);

	de->vco_cntrl = mcom03_dsi_get_best_cfg(dsi_clock, vco_ranges,
						  ARRAY_SIZE(vco_ranges));
	de->hsfreqrange = mcom03_dsi_get_best_cfg(dsi_clock * 2,
						    hsfreqranges,
						    ARRAY_SIZE(hsfreqranges));
	if (!de->vco_cntrl || !de->hsfreqrange) {
		dev_err(de->dev,
		"failed to get DSI frequency : (%d kHz) is out of range\n",
		dsi_clock);
		return -EINVAL;
	}

	/* pll_freq = ref * pll_m / (pll_n * pll_n_mul)
	 * pll_n_mul = 2 ^^ vco_cntrl[5:4]
	 * 64 <= pll_m <= 625
	 * 1 <= pll_n <= 16
	 * 2 MHz <= (ref / pll_n) <= 8 MHz
	 */
	pll_n_mul = BIT(de->vco_cntrl >> 4);
	for (pll_n = 1; pll_n < 16; pll_n++) {
		if ((de->mipi_tx_ref_freq / pll_n) < 2000000 ||
			DIV_ROUND_UP(de->mipi_tx_ref_freq, pll_n) > 8000000)
			continue;

		/* If dsi-clock-frequency is specified then must be used
		 * frequency not above than dsi-clock-frequency.
		 * If dsi_clock is calculated from pixel clock then frequency
		 * must be not less than this calculated value. */
		if (de->dsi_clock_freq)
			pll_m = dsi_clock * pll_n * pll_n_mul /
				(de->mipi_tx_ref_freq / 1000);
		else
			pll_m = DIV_ROUND_UP(dsi_clock * pll_n * pll_n_mul,
					     de->mipi_tx_ref_freq / 1000);

		dsi_clock_actual = (de->mipi_tx_ref_freq / 1000) * pll_m /
				   (pll_n * pll_n_mul);
		if (pll_m >= 64 && pll_m <= 625 &&
		    dsi_clock_actual >= 40000 && dsi_clock_actual <= 1250000)
			break;

		pll_m = 0;
	}

	de->pll_n = pll_n;
	de->pll_m = pll_m;
	if (!pll_m) {
		dev_err(de->dev, "failed to setup PLL for DSI clock %d kHz\n",
			dsi_clock);
		return -EINVAL;
	}

	*lane_mbps = dsi_clock_actual * 2 / 1000;
	dev_dbg(de->dev, "DSI frequency %d kHz (%d Mbps)\n",
		 dsi_clock_actual, *lane_mbps);

	return 0;
}

static int
mcom03_dsi_phy_get_timing(void *de_data, unsigned int lane_mbps,
			   struct dw_mipi_dsi_dphy_timing *timing)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(hstt_table); i++)
		if (lane_mbps < hstt_table[i].maxfreq)
			break;

	if (i == ARRAY_SIZE(hstt_table))
		i--;

	*timing = hstt_table[i].timing;

	return 0;
}

static const struct dw_mipi_dsi_phy_ops mcom03_dsi_phy_ops = {
	.init = mcom03_dsi_phy_init,
	.get_lane_mbps = mcom03_dsi_get_line_mbps,
	.get_timing = mcom03_dsi_phy_get_timing,
};

static enum drm_mode_status mcom03_dsi_mode_valid(void *de_data,
		const struct drm_display_mode *mode)
{
	struct mcom03_dsi_device *de = de_data;
	struct clk *pxl_clk = mcom03_get_clk(de, MCOM03_DSI_CLK_PXL);
	long requested_rate = mode->clock * 1000;
	long real_rate = clk_round_rate(pxl_clk, requested_rate);

	return (requested_rate == real_rate) ? MODE_OK : MODE_NOCLOCK;
}

static int mcom03_dsi_setup(struct platform_device *pdev)
{
	struct mcom03_dsi_device *de =
		(struct mcom03_dsi_device *)dev_get_drvdata(&pdev->dev);
	struct resource *res;
	int ret;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dsi");
	if (!res)
		return -ENODEV;

	de->dsi_plat_data.max_data_lanes = 4;
	de->dsi_plat_data.mode_valid = &mcom03_dsi_mode_valid;
	de->dsi_plat_data.phy_ops = &mcom03_dsi_phy_ops;
	de->dsi_plat_data.base = devm_ioremap_resource(&pdev->dev, res);
	de->dsi_plat_data.priv_data = de;
	if (IS_ERR(de->dsi_plat_data.base))
		return -ENODEV;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "mipi-tx");
	if (!res)
		return -ENODEV;

	de->mipi_tx_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(de->mipi_tx_base))
		return -ENODEV;

	de->dsi = dw_mipi_dsi_probe(pdev, &de->dsi_plat_data);
	if (IS_ERR(de->dsi)) {
		ret = PTR_ERR(de->dsi);
		dev_err(de->dev, "failed to probe DSI (%d)\n", ret);
		return ret;
	}

	return 0;
}

static int mcom03_dsi_bind(struct device *dev, struct device *master,
			   void *data)
{
	struct mcom03_dsi_device *de = dev_get_drvdata(dev);
	struct drm_encoder *encoder = &de->encoder;
	struct drm_device *drm = data;
	int ret = 0;

	encoder->possible_crtcs = drm_of_find_possible_crtcs(drm,
							dev->of_node);
	if (encoder->possible_crtcs == 0)
		return -EPROBE_DEFER;

	DRM_DEBUG_KMS("possible_crtcs = 0x%x\n", encoder->possible_crtcs);

	ret = clk_bulk_enable(de->num_clocks, de->clocks);
	if (ret) {
		dev_err(de->dev, "failed to enable clocks (%d)\n", ret);
		return ret;
	}

	drm_simple_encoder_init(drm, encoder, DRM_MODE_ENCODER_DSI);

	ret = dw_mipi_dsi_bind(de->dsi, encoder);
	if (ret) {
		dev_err(de->dev, "failed to bind DSI encoder (%d)\n", ret);
		drm_encoder_cleanup(encoder);
	}

	return ret;
}

static void mcom03_dsi_unbind(struct device *dev, struct device *master,
			      void *data)
{
	struct mcom03_dsi_device *de = dev_get_drvdata(dev);

	dw_mipi_dsi_unbind(de->dsi);
	drm_encoder_cleanup(&de->encoder);
	clk_bulk_disable(de->num_clocks, de->clocks);
}

static const struct component_ops mcom03_dsi_ops = {
	.bind	= mcom03_dsi_bind,
	.unbind	= mcom03_dsi_unbind,
};

static int mcom03_dsi_clocks_init(struct mcom03_dsi_device *de)
{
	struct clk *tx_ref, *tx_cfg;
	int i, ret;

	for (i = 0; i < MCOM03_DSI_CLK_NUM; i++)
		de->clocks[i].id = mcom03_dsi_clks_names[i];

	de->num_clocks = MCOM03_DSI_CLK_NUM;

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

	tx_cfg = mcom03_get_clk(de, MCOM03_DSI_CLK_MIPI_TX_CFG);
	de->mipi_tx_cfg_freq = clk_round_rate(tx_cfg, MIPI_TX_CFG_FREQ);
	ret = clk_set_rate(tx_cfg, de->mipi_tx_cfg_freq);
	if (ret) {
		dev_err(de->dev, "failed to set rate for mipi_tx_cfg %d\n",
			de->mipi_tx_cfg_freq);
		return ret;
	}

	tx_ref = mcom03_get_clk(de, MCOM03_DSI_CLK_MIPI_TX_REF);
	de->mipi_tx_ref_freq = clk_round_rate(tx_ref, MIPI_TX_REF_FREQ);
	ret = clk_set_rate(tx_ref, de->mipi_tx_ref_freq);
	if (ret) {
		dev_err(de->dev, "failed to set rate for mipi_tx_ref %d\n",
			de->mipi_tx_ref_freq);
		return ret;
	}

	if (device_property_present(de->dev, "dsi-clock-frequency")) {
		ret = device_property_read_u32(de->dev, "dsi-clock-frequency",
					       &de->dsi_clock_freq);
		if (ret) {
			dev_err(de->dev,
				"failed to get dsi-clock-frequency");
			return ret;
		}
	}

	return 0;
}

static int mcom03_dsi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcom03_dsi_device *de;
	int ret;

	if (!dev->of_node) {
		DRM_DEV_ERROR(dev, "failed to find mcom03 dsi node in DTB\n");
		return -ENODEV;
	}

	de = devm_kzalloc(dev, sizeof(*de), GFP_KERNEL);
	if (!de)
		return -ENOMEM;

	de->dev = dev;
	platform_set_drvdata(pdev, de);

	ret = mcom03_dsi_setup(pdev);
	if (ret) {
		DRM_DEV_INFO(dev, "dsi setup failed %d", ret);
		return ret;
	}

	ret = mcom03_dsi_clocks_init(de);
	if (ret)
		goto dw_cleanup;

	ret = component_add(&pdev->dev, &mcom03_dsi_ops);

dw_cleanup:
	if (ret) {
		DRM_DEV_ERROR(dev, "failed to probe mcom03 dsi (%d)", ret);
		dw_mipi_dsi_remove(de->dsi);
	}
	return ret;
}

static int mcom03_dsi_remove(struct platform_device *pdev)
{
	struct mcom03_dsi_device *de =
		(struct mcom03_dsi_device *)dev_get_drvdata(&pdev->dev);

	component_del(&pdev->dev, &mcom03_dsi_ops);
	dw_mipi_dsi_remove(de->dsi);
	return 0;
}

static const struct of_device_id mcom03_dsi_dt_ids[] = {
	{ .compatible = "elvees,mcom03-drm-dsi", },
	{ },
};
MODULE_DEVICE_TABLE(of, mcom03_dsi_dt_ids);

struct platform_driver mcom03_dsi_driver = {
	.probe  = mcom03_dsi_probe,
	.remove = mcom03_dsi_remove,
	.driver = {
		.name = "mcom03-encoder-dsi",
		.of_match_table = mcom03_dsi_dt_ids,
	},
};

module_platform_driver(mcom03_dsi_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ELVEES MCom-03 DSI Encoder Driver");
