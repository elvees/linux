// SPDX-License-Identifier: GPL-2.0+
/*
 * Pinctrl driver for MEDIA subsystem of MCom-03 SoC.
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/mfd/syscon.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "../core.h"
#include "../pinctrl-utils.h"

#define MCOM03_MEDIA_URB_SUBSYSTEM_CFG	0x2000
#define DISPLAY_PARALLEL_PORT_EN	BIT(0)

struct mcom03_media_pinctrl {
	struct device		*dev;
	struct regmap		*media_syscon;
	struct pinctrl_dev	*pctrl_dev;
};

static const struct pinctrl_pin_desc mcom03_media_pins[] = {
	PINCTRL_PIN(0, "PIXCLK"),
	PINCTRL_PIN(1, "HSYNK"),
	PINCTRL_PIN(2, "VSYNK"),
	PINCTRL_PIN(3, "VDATA0"),
	PINCTRL_PIN(4, "VDATA1"),
	PINCTRL_PIN(5, "VDATA2"),
	PINCTRL_PIN(6, "VDATA3"),
	PINCTRL_PIN(7, "VDATA4"),
	PINCTRL_PIN(8, "VDATA5"),
	PINCTRL_PIN(9, "VDATA6"),
	PINCTRL_PIN(10, "VDATA7"),
	PINCTRL_PIN(11, "VDATA8"),
	PINCTRL_PIN(12, "VDATA9"),
	PINCTRL_PIN(13, "VDATA10"),
	PINCTRL_PIN(14, "VDATA11"),
};

const unsigned int media_mux_grp[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12,
				       13, 14};

static const char *const mcom03_media_groups[] = {
	"media_mux_grp",
};

static const char *const mcom03_media_pinmux_functions[] = {
	"ISP", "DP"
};

int mcom03_media_get_functions_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(mcom03_media_pinmux_functions);
}

const char *mcom03_media_get_function_name(struct pinctrl_dev *pctldev,
					   unsigned int selector)
{
	return mcom03_media_pinmux_functions[selector];
}

int mcom03_media_get_function_groups(struct pinctrl_dev *pctldev,
				     unsigned int selector,
				     const char * const **groups,
				     unsigned int *num_groups)
{
	*groups = mcom03_media_groups;
	*num_groups = ARRAY_SIZE(mcom03_media_groups);

	return 0;
}

int mcom03_media_set_mux(struct pinctrl_dev *pctldev,
			 unsigned int func_selector,
			 unsigned int group_selector)
{
	struct mcom03_media_pinctrl *pctrl = pinctrl_dev_get_drvdata(pctldev);
	int ret;

	ret = regmap_update_bits(pctrl->media_syscon,
				 MCOM03_MEDIA_URB_SUBSYSTEM_CFG,
				 DISPLAY_PARALLEL_PORT_EN, func_selector);
	if (ret) {
		dev_err(pctrl->dev,
			"failed to %s display parallel port (%d)\n",
			func_selector ? "disable" : "enable",
			ret);

		return ret;
	}

	dev_dbg(pctrl->dev, "Set %s function to [media_mux_grp] group\n",
		mcom03_media_pinmux_functions[func_selector]);

	return 0;
}

static const struct pinmux_ops mcom03_media_pinmux_ops = {
	.get_functions_count	= mcom03_media_get_functions_count,
	.get_function_name	= mcom03_media_get_function_name,
	.get_function_groups	= mcom03_media_get_function_groups,
	.set_mux		= mcom03_media_set_mux,
};

int mcom03_media_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	return ARRAY_SIZE(mcom03_media_groups);
}

const char *mcom03_media_pinctrl_get_group_name(struct pinctrl_dev *pctldev,
						unsigned int selector)
{
	return mcom03_media_groups[selector];
}

int mcom03_media_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					unsigned int selector,
					const unsigned int **pins,
					unsigned int *num_pins)
{
	*pins = media_mux_grp;
	*num_pins = ARRAY_SIZE(media_mux_grp);

	return 0;
}

static const struct pinctrl_ops mcom03_media_pinctrl_ops = {
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
	.get_groups_count	= mcom03_media_pinctrl_get_groups_count,
	.get_group_name		= mcom03_media_pinctrl_get_group_name,
	.get_group_pins		= mcom03_media_pinctrl_get_group_pins,
};

static struct pinctrl_desc mcom03_media_desc = {
	.name			= "mcom03-media-pinctrl",
	.owner			= THIS_MODULE,
	.pins			= mcom03_media_pins,
	.npins			= ARRAY_SIZE(mcom03_media_pins),
	.pmxops			= &mcom03_media_pinmux_ops,
	.pctlops		= &mcom03_media_pinctrl_ops,
};

static int mcom03_media_pinctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcom03_media_pinctrl *pctrl;
	int ret;

	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	pctrl->media_syscon = syscon_regmap_lookup_by_phandle(dev->of_node,
							      "elvees,urb");
	if (IS_ERR(pctrl->media_syscon)) {
		dev_err(&pdev->dev, "unable to get media-syscon\n");
		return PTR_ERR(pctrl->media_syscon);
	}

	pctrl->dev = dev;
	ret = devm_pinctrl_register_and_init(dev, &mcom03_media_desc, pctrl,
					     &pctrl->pctrl_dev);
	if (ret) {
		dev_err(pctrl->dev, "Failed to register pinctrl (%d)\n", ret);
		return ret;
	}

	ret = pinctrl_enable(pctrl->pctrl_dev);
	if (ret) {
		dev_err(pctrl->dev, "Failed to enable pinctrl (%d)\n", ret);
		return ret;
	}

	dev_info(dev, "%ld pins & %ld groups registered\n",
		 ARRAY_SIZE(mcom03_media_pins),
		 ARRAY_SIZE(mcom03_media_groups));
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mcom03_media_pinctrl_of_match[] = {
	{ .compatible = "elvees,mcom03-media-pinctrl" },
	{ /* sentinel */ },
};
#endif

static struct platform_driver mcom03_media_pinctrl_driver = {
	.probe = mcom03_media_pinctrl_probe,
	.driver = {
		.name = "mcom03-media-pinctrl",
		.of_match_table = mcom03_media_pinctrl_of_match,
	},
};

static int __init mcom03_media_api_pinctrl_register(void)
{
	return platform_driver_register(&mcom03_media_pinctrl_driver);
}

arch_initcall(mcom03_media_api_pinctrl_register);
