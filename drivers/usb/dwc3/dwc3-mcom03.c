// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2021 RnD Center "ELVEES", JSC
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define USB_PHY_CTR_OFFSET(x) (0xdc + (x) * 0x34)

#define USB_PHY_CTR_FSEL            GENMASK(29, 24)
#define USB_PHY_CTR_MPLL_MULT       GENMASK(23, 17)
#define USB_PHY_CTR_REF_CLKDIV2     BIT(15)
#define USB_PHY_CTR_REF_SSP_EN      BIT(14)
#define USB_PHY_CTR_REF_USE_PAD     BIT(13)
#define USB_PHY_CTR_SSC_EN          BIT(12)
#define USB_PHY_CTR_SSC_RANGE       GENMASK(11, 9)
#define USB_PHY_CTR_SSC_REF_CLK_SEL GENMASK(8, 0)

static const char *const clknames[] = { "sys", "ref", "suspend" };

struct dwc3_mcom03_priv {
	void __iomem *regs;
	struct clk_bulk_data clks[ARRAY_SIZE(clknames)];
	int ctrl_id;
	struct reset_control *rst;
	struct regmap *urb;
};

static int dwc3_mcom03_phy_init(struct device *dev,
				struct dwc3_mcom03_priv *priv)
{
	u32 val;
	size_t i;
	unsigned long rate = 0;
	const struct dwc3_mcom03_phy_cfg {
		u32 refclk_rate;
		u8 fsel, mpll, refclk_div;
		u16 ssc_refclk_sel;
	} phy_cfg[] = {
		/* From Synopsys DesignWare SuperSpeed USB 3.0 femtoPHY
		 * for TSMC 28-nm HPCP 0.9/1.8 V databook, table 3-1 */
		{ 19200000, 0x38, 0, 0, 0 },
		{ 20000000, 0x31, 0, 0, 0 },
		{ 24000000, 0x2a, 0, 0, 0 },
		{ 25000000, 0x2, 0x64, 0, 0 },
		{ 26000000, 0x2, 0x60, 0, 0x108 },
		{ 38400000, 0x38, 0, 1, 0 },
		{ 40000000, 0x31, 0, 1, 0 },
		{ 48000000, 0x2a, 0, 1, 0 },
		{ 50000000, 0x2, 0x64, 1, 0 },
		{ 52000000, 0x2, 0x60, 1, 0x108 },
		{ 100000000, 0x27, 0, 0, 0 },
		{ 200000000, 0x27, 0, 1, 0 },
	};

	for (i = 0; i < ARRAY_SIZE(priv->clks); i++)
		if (strcmp(priv->clks[i].id, "ref") == 0)
			rate = clk_get_rate(priv->clks[i].clk);

	for (i = 0; i < ARRAY_SIZE(phy_cfg); i++)
		if (rate == phy_cfg[i].refclk_rate)
			break;

	if (i == ARRAY_SIZE(phy_cfg)) {
		dev_err(dev, "Unsupported refclk frequency: %lu", rate);
		return -EINVAL;
	}

	/* TODO: Only refclk from external pad is supported */
	val = FIELD_PREP(USB_PHY_CTR_FSEL, phy_cfg[i].fsel) |
	      FIELD_PREP(USB_PHY_CTR_MPLL_MULT, phy_cfg[i].mpll) |
	      FIELD_PREP(USB_PHY_CTR_REF_CLKDIV2, phy_cfg[i].refclk_div) |
	      FIELD_PREP(USB_PHY_CTR_REF_SSP_EN, 1) |
	      FIELD_PREP(USB_PHY_CTR_REF_USE_PAD, 1) |
	      FIELD_PREP(USB_PHY_CTR_SSC_EN, 1) |
	      FIELD_PREP(USB_PHY_CTR_SSC_RANGE, 0) |
	      FIELD_PREP(USB_PHY_CTR_SSC_REF_CLK_SEL,
			 phy_cfg[i].ssc_refclk_sel);

	return regmap_write(priv->urb, USB_PHY_CTR_OFFSET(priv->ctrl_id), val);
}

static int dwc3_mcom03_probe(struct platform_device *pdev)
{
	struct dwc3_mcom03_priv *priv;
	struct device *dev = &pdev->dev;
	int ret;
	size_t i;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->urb = syscon_regmap_lookup_by_phandle(dev->of_node, "elvees,urb");
	if (IS_ERR(priv->urb)) {
		dev_err(&pdev->dev, "Failed to get subsystem URB");
		return PTR_ERR(priv->urb);
	}

	ret = of_property_read_u32(dev->of_node, "elvees,ctrl-id",
				   &priv->ctrl_id);
	if (ret) {
		dev_err(dev, "Failed to get controller ID\n");
		return ret;
	}

	priv->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(priv->rst)) {
		dev_err(dev, "Failed to get reset\n");
		return PTR_ERR(priv->rst);
	}

	/* Require all three clocks */
	for (i = 0; i < ARRAY_SIZE(clknames); i++)
		priv->clks[i].id = clknames[i];

	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(priv->clks), priv->clks);
	if (ret) {
		dev_err(dev, "Failed to get clocks\n");
		return ret;
	}

	ret = clk_bulk_prepare_enable(ARRAY_SIZE(priv->clks), priv->clks);
	if (ret) {
		dev_err(dev, "Failed to enable clocks\n");
		return ret;
	}

	/* PHY regs should be programmed in reset state */
	ret = reset_control_assert(priv->rst);
	if (ret) {
		dev_err(dev, "Failed to assert reset\n");
		goto err_clk_disable;
	}

	/* Wait until PHY goes into reset state, 1-2 ms should be enough */
	usleep_range(1000, 2000);

	ret = dwc3_mcom03_phy_init(dev, priv);
	if (ret)
		goto err_clk_disable;

	ret = reset_control_deassert(priv->rst);
	if (ret) {
		dev_err(dev, "Failed to deassert reset\n");
		goto err_clk_disable;
	}

	/* Wait until PHY goes from reset state, 1-2 ms should be enough */
	usleep_range(1000, 2000);

	platform_set_drvdata(pdev, priv);

	ret = of_platform_populate(dev->of_node, NULL, NULL, dev);
	if (ret) {
		dev_err(dev, "Failed to add dwc3 core\n");
		goto err_reset_assert;
	}

	/* BUG: Strange that PHY registers are accessed before PM is enabled */
	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	return 0;

err_reset_assert:
	reset_control_assert(priv->rst);

err_clk_disable:
	clk_bulk_disable_unprepare(ARRAY_SIZE(priv->clks), priv->clks);

	return ret;
}

static int dwc3_mcom03_remove(struct platform_device *pdev)
{
	struct dwc3_mcom03_priv *priv = platform_get_drvdata(pdev);
	struct device *dev = &pdev->dev;

	reset_control_assert(priv->rst);
	clk_bulk_disable_unprepare(ARRAY_SIZE(priv->clks), priv->clks);

	of_platform_depopulate(dev);

	pm_runtime_put_sync(dev);
	pm_runtime_disable(dev);

	return 0;
}

static int __maybe_unused dwc3_mcom03_runtime_suspend(struct device *dev)
{
	struct dwc3_mcom03_priv *priv = dev_get_drvdata(dev);

	clk_bulk_disable(ARRAY_SIZE(priv->clks), priv->clks);

	return 0;
}

static int __maybe_unused dwc3_mcom03_runtime_resume(struct device *dev)
{
	struct dwc3_mcom03_priv *priv = dev_get_drvdata(dev);

	return clk_bulk_enable(ARRAY_SIZE(priv->clks), priv->clks);
}

static int __maybe_unused dwc3_mcom03_suspend(struct device *dev)
{
	struct dwc3_mcom03_priv *priv = dev_get_drvdata(dev);

	return reset_control_assert(priv->rst);
}

static int __maybe_unused dwc3_mcom03_resume(struct device *dev)
{
	struct dwc3_mcom03_priv *priv = dev_get_drvdata(dev);

	return reset_control_deassert(priv->rst);
}

static const struct dev_pm_ops dwc3_mcom03_dev_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(dwc3_mcom03_suspend, dwc3_mcom03_resume)
	SET_RUNTIME_PM_OPS(dwc3_mcom03_runtime_suspend,
			   dwc3_mcom03_runtime_resume, NULL)
};

static const struct of_device_id dwc3_mcom03_match[] = {
	{ .compatible = "elvees,mcom03-dwc3" },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, dwc3_mcom03_match);

static struct platform_driver dwc3_mcom03_driver = {
	.probe		= dwc3_mcom03_probe,
	.remove		= dwc3_mcom03_remove,
	.driver		= {
		.name	= "dwc3-mcom03",
		.of_match_table = dwc3_mcom03_match,
		.pm	= &dwc3_mcom03_dev_pm_ops,
	},
};

module_platform_driver(dwc3_mcom03_driver);

MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 MCom-03 Glue Layer");
