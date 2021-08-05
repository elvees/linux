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
#include <dt-bindings/soc/elvees,mcom03.h>

#define WRITE_ENABLE_OFFSET	16

static const unsigned int nr_resets[MCOM03_SUBSYSTEM_MAX] = {
	[MCOM03_SUBSYSTEM_MEDIA] = 4,
	[MCOM03_SUBSYSTEM_HSPERIPH] = 10
};

#define PP_MASK		GENMASK(4, 0)
#define PP_ON		BIT(4)
#define PP_WARM_RST	BIT(3)
#define PP_OFF		BIT(0)

#define BTN_RST_N	BIT(1)

struct mcom03_reset_private {
	struct reset_controller_dev rcdev;
	struct regmap *urb;
	u32 offset;
	u32 subsystem;
};

static int mcom03_reset_media_assert(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct mcom03_reset_private *priv =
		(struct mcom03_reset_private *)rcdev;

	return regmap_update_bits(priv->urb, priv->offset + 0x8 * id,
				  PP_MASK, PP_WARM_RST);
}

static int mcom03_reset_media_deassert(struct reset_controller_dev *rcdev,
				       unsigned long id)
{
	struct mcom03_reset_private *priv =
		(struct mcom03_reset_private *)rcdev;

	return regmap_update_bits(priv->urb, priv->offset + 0x8 * id,
				  PP_MASK, PP_ON);
}

static int mcom03_reset_media_status(struct reset_controller_dev *rcdev,
				     unsigned long id)
{
	struct mcom03_reset_private *priv =
		(struct mcom03_reset_private *)rcdev;
	unsigned long reg;
	int ret;

	ret = regmap_read(priv->urb, priv->offset + 0x8 * id + 0x4,
			  (unsigned int *)&reg);
	if (ret)
		return ret;

	reg = (reg & PP_MASK) == PP_WARM_RST;

	return reg;
}

static const struct reset_control_ops mcom03_reset_media_ops = {
	.assert		= mcom03_reset_media_assert,
	.deassert	= mcom03_reset_media_deassert,
	.status		= mcom03_reset_media_status,
};

static int mcom03_reset_hsperiph_assert(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct mcom03_reset_private *priv =
		(struct mcom03_reset_private *)rcdev;

	dev_dbg(rcdev->dev, "assert reset for %lu\n", id);

	return regmap_update_bits(priv->urb,
				  priv->offset,
				  BIT(id) | BIT(id + WRITE_ENABLE_OFFSET),
				  BIT(id) | BIT(id + WRITE_ENABLE_OFFSET));
}

static int mcom03_reset_hsperiph_deassert(struct reset_controller_dev *rcdev,
					  unsigned long id)
{
	struct mcom03_reset_private *priv =
		(struct mcom03_reset_private *)rcdev;

	dev_dbg(rcdev->dev, "deassert reset for %lu\n", id);

	return regmap_update_bits(priv->urb,
				  priv->offset,
				  BIT(id) | BIT(id + WRITE_ENABLE_OFFSET),
				  BIT(id + WRITE_ENABLE_OFFSET));
}

static int mcom03_reset_hsperiph_status(struct reset_controller_dev *rcdev,
					unsigned long id)
{
	struct mcom03_reset_private *priv =
		(struct mcom03_reset_private *)rcdev;
	unsigned long reg;
	int ret;

	ret = regmap_read(priv->urb, priv->offset, (unsigned int *)&reg);
	if (ret)
		return ret;

	reg = test_bit(id, &reg);

	dev_dbg(rcdev->dev, "reset status for %lu: %lu\n", id, reg);

	return reg;
}

static const struct reset_control_ops mcom03_reset_hsperiph_ops = {
	.assert		= mcom03_reset_hsperiph_assert,
	.deassert	= mcom03_reset_hsperiph_deassert,
	.status		= mcom03_reset_hsperiph_status,
};

static int mcom03_reset_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct mcom03_reset_private *priv;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	platform_set_drvdata(pdev, priv);

	ret = device_property_read_u32(&pdev->dev, "elvees,subsystem",
				       &priv->subsystem);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get subsystem id");
		return -ENODEV;
	}

	if (priv->subsystem >= MCOM03_SUBSYSTEM_MAX)
		return -EINVAL;

	priv->urb = syscon_node_to_regmap(pdev->dev.parent->of_node);
	if (IS_ERR(priv->urb)) {
		dev_err(&pdev->dev, "Failed to get subsystem URB node");
		return PTR_ERR(priv->urb);
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get resource\n");
		return -ENODEV;
	}

	priv->offset = res->start;

	priv->rcdev.owner = THIS_MODULE;
	priv->rcdev.of_node = pdev->dev.of_node;
	priv->rcdev.nr_resets = nr_resets[priv->subsystem];

	switch (priv->subsystem) {
	case MCOM03_SUBSYSTEM_MEDIA:
		priv->rcdev.ops = &mcom03_reset_media_ops;
		break;
	case MCOM03_SUBSYSTEM_HSPERIPH:
		priv->rcdev.ops = &mcom03_reset_hsperiph_ops;
		break;
	default:
		return -ENOTSUPP;
	}

	return devm_reset_controller_register(&pdev->dev, &priv->rcdev);
}

#ifdef CONFIG_OF
static const struct of_device_id mcom03_reset_dt_ids[] = {
	{ .compatible = "elvees,mcom03-reset", },
	{ /* sentinel */ },
};
#endif

static struct platform_driver mcom03_reset_driver = {
	.probe	= mcom03_reset_probe,
	.driver = {
		.name		= KBUILD_MODNAME,
		.of_match_table	= of_match_ptr(mcom03_reset_dt_ids),
	},
};
builtin_platform_driver(mcom03_reset_driver);
