// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2015, National Instruments Corp
 * Copyright 2021 RnD Center "ELVEES", JSC
 *
 * Based on Xilinx Zynq Reset controller driver.
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/regmap.h>
#include <linux/types.h>

#define WRITE_ENABLE_OFFSET	16
#define HSPERIPH_NR_RESETS	10

struct mcom03_hsperiph_private {
	struct regmap *urb;
	struct reset_controller_dev rcdev;
	u32 offset;
};

#define to_mcom03_hsperiph_private(p) \
	container_of((p), struct mcom03_hsperiph_private, rcdev)

static int mcom03_hsperiph_reset_assert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct mcom03_hsperiph_private *priv =
		to_mcom03_hsperiph_private(rcdev);

	dev_dbg(rcdev->dev, "assert reset for %lu\n", id);

	return regmap_update_bits(priv->urb,
				  priv->offset,
				  BIT(id) | BIT(id + WRITE_ENABLE_OFFSET),
				  BIT(id) | BIT(id + WRITE_ENABLE_OFFSET));
}

static int mcom03_hsperiph_reset_deassert(struct reset_controller_dev *rcdev,
					  unsigned long id)
{
	struct mcom03_hsperiph_private *priv =
		to_mcom03_hsperiph_private(rcdev);

	dev_dbg(rcdev->dev, "deassert reset for %lu\n", id);

	return regmap_update_bits(priv->urb,
				  priv->offset,
				  BIT(id) | BIT(id + WRITE_ENABLE_OFFSET),
				  BIT(id + WRITE_ENABLE_OFFSET));
}

static int mcom03_hsperiph_reset_status(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct mcom03_hsperiph_private *priv =
		to_mcom03_hsperiph_private(rcdev);
	int ret;
	unsigned long reg;

	ret = regmap_read(priv->urb, priv->offset, (unsigned int *)&reg);
	if (ret)
		return ret;

	reg = test_bit(id, &reg);

	dev_dbg(rcdev->dev, "reset status for %lu: %lu\n", id, reg);

	return reg;
}

static const struct reset_control_ops mcom03_hsperiph_reset_ops = {
	.assert		= mcom03_hsperiph_reset_assert,
	.deassert	= mcom03_hsperiph_reset_deassert,
	.status		= mcom03_hsperiph_reset_status,
};

static int mcom03_hsperiph_reset_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mcom03_hsperiph_private *priv;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	priv->urb = syscon_regmap_lookup_by_phandle(pdev->dev.of_node,
						    "urb");
	if (IS_ERR(priv->urb)) {
		dev_err(&pdev->dev, "Failed to get HSPERIPH URB node");
		return PTR_ERR(priv->urb);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get resource\n");
		return -ENODEV;
	}

	priv->offset = res->start;

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.nr_resets = HSPERIPH_NR_RESETS;
	priv->rcdev.ops = &mcom03_hsperiph_reset_ops;
	priv->rcdev.of_node = pdev->dev.of_node;

	return devm_reset_controller_register(&pdev->dev, &priv->rcdev);
}

static const struct of_device_id mcom03_hsperiph_reset_dt_ids[] = {
	{ .compatible = "elvees,mcom03-hsperiph-reset", },
	{ /* sentinel */ },
};

static struct platform_driver mcom03_hsperiph_reset_driver = {
	.probe	= mcom03_hsperiph_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= mcom03_hsperiph_reset_dt_ids,
	},
};
builtin_platform_driver(mcom03_hsperiph_reset_driver);
