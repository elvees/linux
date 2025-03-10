// SPDX-License-Identifier: GPL-2.0+
/*
 * Pinctrl driver for LSPERIPH0 and LSPERIPH1 subsystems of MCom-03 SoC.
 * Copyright 2021 RnD Center "ELVEES", JSC
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#include "../core.h"
#include "../pinctrl-utils.h"

/* LSPERIPH IDs */
#define PINCTRL_LSPERIPH0	0
#define PINCTRL_LSPERIPH1	1

/* LSPERIPH0 defines */
#define LSPERIPH0_PULL_OFFSET		0x8 /* GPIO0_IO_PULL_CTR */

/* LSPERIPH1 default values */
#define LSPERIPH1_CTR_DEFAULT		0xF8

/* LSPERIPH1 offsets */
#define LSPERIPH1_CTR(pin)		(((pin) * 0x04) + 0x20)
#define LSPERIPH1_GROUP_REG		0xA0

/* LSPERIPH1 defines for CTR register */
#define LSPERIPH1_DS_MASK		GENMASK(5, 0)
#define LSPERIPH1_CTL_MASK		GENMASK(10, 5)
#define LSPERIPH1_ARG_TO_CTL(arg)	(arg << 5)
#define LSPERIPH1_CTL_TO_VAL(ctl)	((ctl >> 5) & LSPERIPH1_DS_MASK)
#define LSPERIPH1_2mA			(LSPERIPH1_DS_MASK >> 5)
#define LSPERIPH1_4mA			(LSPERIPH1_DS_MASK >> 4)
#define LSPERIPH1_6mA			(LSPERIPH1_DS_MASK >> 3)
#define LSPERIPH1_8mA			(LSPERIPH1_DS_MASK >> 2)
#define LSPERIPH1_10mA			(LSPERIPH1_DS_MASK >> 1)
#define LSPERIPH1_12mA			LSPERIPH1_DS_MASK
#define LSPERIPH1_SLEW_RATE_MASK	GENMASK(4, 3)
#define LSPERIPH1_SLEW_RATE(arg)	(arg << 3)
#define LSPERIPH1_GET_SLEW_RATE(val)	((val >> 3) & GENMASK(1, 0))
#define LSPERIPH1_MAX_SLEW_RATE		0x03
#define LSPERIPH1_MIN_SLEW_RATE		0
#define LSPERIPH1_PAD_CTR_SUS		BIT(0)
#define LSPERIPH1_PAD_CTR_PU		BIT(1)
#define LSPERIPH1_PAD_CTR_PD		BIT(2)
#define LSPERIPH1_PULL_MASK		(LSPERIPH1_PAD_CTR_SUS | \
					 LSPERIPH1_PAD_CTR_PU | \
					 LSPERIPH1_PAD_CTR_PD)
#define LSPERIPH1_EN			BIT(12)
#define LSPERIPH1_OPEN_DRAIN_EN		BIT(14)

/* LSPERIPH1 defines for GPIO1_V18 register */
#define LSPERIPH1_1P8V			BIT(0)

struct mcom03_lsperiph_pinctrl_desc {
	const unsigned int			id;
	struct pinctrl_desc			*pinctrl_desc;
};

struct mcom03_lsperiph_pinctrl {
	struct device					*dev;
	struct pinctrl_dev				*pctrl_dev;
	struct regmap					*urb;
	const struct mcom03_lsperiph_pinctrl_desc	*desc;
};

static const struct pinctrl_pin_desc mcom03_lsperiph_pins[] = {
	PINCTRL_PIN(0, "GPIOA_0"),
	PINCTRL_PIN(1, "GPIOA_1"),
	PINCTRL_PIN(2, "GPIOA_2"),
	PINCTRL_PIN(3, "GPIOA_3"),
	PINCTRL_PIN(4, "GPIOA_4"),
	PINCTRL_PIN(5, "GPIOA_5"),
	PINCTRL_PIN(6, "GPIOA_6"),
	PINCTRL_PIN(7, "GPIOA_7"),
	PINCTRL_PIN(8, "GPIOB_0"),
	PINCTRL_PIN(9, "GPIOB_1"),
	PINCTRL_PIN(10, "GPIOB_2"),
	PINCTRL_PIN(11, "GPIOB_3"),
	PINCTRL_PIN(12, "GPIOB_4"),
	PINCTRL_PIN(13, "GPIOB_5"),
	PINCTRL_PIN(14, "GPIOB_6"),
	PINCTRL_PIN(15, "GPIOB_7"),
	PINCTRL_PIN(16, "GPIOC_0"),
	PINCTRL_PIN(17, "GPIOC_1"),
	PINCTRL_PIN(18, "GPIOC_2"),
	PINCTRL_PIN(19, "GPIOC_3"),
	PINCTRL_PIN(20, "GPIOC_4"),
	PINCTRL_PIN(21, "GPIOC_5"),
	PINCTRL_PIN(22, "GPIOC_6"),
	PINCTRL_PIN(23, "GPIOC_7"),
	PINCTRL_PIN(24, "GPIOD_0"),
	PINCTRL_PIN(25, "GPIOD_1"),
	PINCTRL_PIN(26, "GPIOD_2"),
	PINCTRL_PIN(27, "GPIOD_3"),
	PINCTRL_PIN(28, "GPIOD_4"),
	PINCTRL_PIN(29, "GPIOD_5"),
	PINCTRL_PIN(30, "GPIOD_6"),
	PINCTRL_PIN(31, "GPIOD_7"),
};

static const unsigned int lsperiph1_pins[] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10,
					       11, 12, 13, 14, 15, 16, 17, 18,
					       19, 20, 21, 22, 23, 24, 25, 26,
					       27, 28, 29, 30, 31};

struct mcom03_lsperiph1_group {
	const char	*name;
	const u32	*pins;
};

static const struct mcom03_lsperiph1_group gpio1_grp = {
	.name = "all",
	.pins = lsperiph1_pins,
};

static int mcom03_lsperiph0_pconf_get(struct pinctrl_dev *pctldev,
				      unsigned int pin, unsigned long *config)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param = pinconf_to_config_param(*config);
	unsigned int val;

	if (param != PIN_CONFIG_BIAS_PULL_UP &&
	    param != PIN_CONFIG_BIAS_DISABLE)
		return -ENOTSUPP;

	regmap_read(pctrl->urb, LSPERIPH0_PULL_OFFSET, &val);

	if (param == PIN_CONFIG_BIAS_PULL_UP)
		return val & BIT(pin) ? -EINVAL : 0;

	return val & BIT(pin) ? 0 : -EINVAL;
}

static int mcom03_lsperiph1_pconf_get(struct pinctrl_dev *pctldev,
				      unsigned int pin, unsigned long *config)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;
	unsigned int val, arg = 1;

	param = pinconf_to_config_param(*config);
	regmap_read(pctrl->urb, LSPERIPH1_CTR(pin), &val);

	switch (param) {
	case PIN_CONFIG_BIAS_BUS_HOLD:
		if (!(val & LSPERIPH1_PAD_CTR_SUS))
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!(val & LSPERIPH1_PAD_CTR_PD))
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!(val & LSPERIPH1_PAD_CTR_PU))
			return -EINVAL;
		break;
	case PIN_CONFIG_BIAS_DISABLE:
		if (val & LSPERIPH1_PULL_MASK)
			return -EINVAL;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		val = LSPERIPH1_CTL_TO_VAL(val);
		switch (val) {
		case LSPERIPH1_2mA:
			val = 2;
			break;
		case LSPERIPH1_4mA:
			val = 4;
			break;
		case LSPERIPH1_6mA:
			val = 6;
			break;
		case LSPERIPH1_8mA:
			val = 8;
			break;
		case LSPERIPH1_10mA:
			val = 10;
			break;
		case LSPERIPH1_12mA:
			val = 12;
			break;
		}

		arg = val;
		break;
	case PIN_CONFIG_INPUT_ENABLE:
		if (!(val & LSPERIPH1_EN))
			return -EINVAL;
		break;
	case PIN_CONFIG_SLEW_RATE:
		arg = LSPERIPH1_GET_SLEW_RATE(val);
		break;
	case PIN_CONFIG_DRIVE_OPEN_DRAIN:
		if (!(val & LSPERIPH1_OPEN_DRAIN_EN))
			return -EINVAL;
		break;
	default:
		return -ENOTSUPP;
	}

	*config = pinconf_to_config_packed(param, arg);
	return 0;
}

static int mcom03_lsperiph0_pconf_set(struct pinctrl_dev *pctldev,
				      unsigned int pin,
				      unsigned long *configs,
				      unsigned int num_configs)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);
	enum pin_config_param param;

	if (num_configs > 1) {
		dev_warn(pctldev->dev, "LSPERIPH0 supports only one config at a time\n");
		return -ENOTSUPP;
	}

	param = pinconf_to_config_param(configs[0]);
	if (param != PIN_CONFIG_BIAS_PULL_UP &&
	    param != PIN_CONFIG_BIAS_DISABLE) {
		dev_err(pctldev->dev, "LSPERIPH0 subsystem pins supports only bias-pull-up and bias-disable params\n");
		return -ENOTSUPP;
	}

	regmap_update_bits(pctrl->urb, LSPERIPH0_PULL_OFFSET,
			   BIT(pin), param == PIN_CONFIG_BIAS_PULL_UP ?
			   0 : BIT(pin));

	return 0;
}

static int mcom03_lsperiph1_pconf_set(struct pinctrl_dev *pctldev,
				      unsigned int pin,
				      unsigned long *configs,
				      unsigned int num_configs)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);
	struct regmap *map = pctrl->urb;
	enum pin_config_param param;
	int i;
	u32 arg;
	u32 reg = LSPERIPH1_CTR_DEFAULT;

	for (i = 0; i < num_configs; i++) {
		param = pinconf_to_config_param(configs[i]);
		arg = pinconf_to_config_argument(configs[i]);
		switch (param) {
		case PIN_CONFIG_BIAS_BUS_HOLD:
			u32p_replace_bits(&reg, LSPERIPH1_PAD_CTR_SUS,
					  LSPERIPH1_PULL_MASK);
			break;
		case PIN_CONFIG_BIAS_PULL_DOWN:
			u32p_replace_bits(&reg, LSPERIPH1_PAD_CTR_PD,
					  LSPERIPH1_PULL_MASK);
			break;
		case PIN_CONFIG_BIAS_PULL_UP:
			u32p_replace_bits(&reg, LSPERIPH1_PAD_CTR_PU,
					  LSPERIPH1_PULL_MASK);
			break;
		case PIN_CONFIG_BIAS_DISABLE:
			u32p_replace_bits(&reg, 0, LSPERIPH1_PULL_MASK);
			break;
		case PIN_CONFIG_DRIVE_STRENGTH:
			switch (arg) {
			case 0:
				break;
			case 2:
				arg = LSPERIPH1_2mA;
				break;
			case 4:
				arg = LSPERIPH1_4mA;
				break;
			case 6:
				arg = LSPERIPH1_6mA;
				break;
			case 8:
				arg = LSPERIPH1_8mA;
				break;
			case 10:
				arg = LSPERIPH1_10mA;
				break;
			case 12:
				arg = LSPERIPH1_12mA;
				break;
			default:
				return -EINVAL;
			}

			u32p_replace_bits(&reg, arg, LSPERIPH1_CTL_MASK);
			break;
		case PIN_CONFIG_INPUT_ENABLE:
			u32p_replace_bits(&reg, arg, LSPERIPH1_EN);
			break;
		case PIN_CONFIG_SLEW_RATE:
			if (arg != LSPERIPH1_MIN_SLEW_RATE &&
			    arg != LSPERIPH1_MAX_SLEW_RATE)
				return -EINVAL;

			u32p_replace_bits(&reg, arg, LSPERIPH1_SLEW_RATE_MASK);
			break;
		case PIN_CONFIG_DRIVE_OPEN_DRAIN:
			u32p_replace_bits(&reg, 1, LSPERIPH1_OPEN_DRAIN_EN);
			break;
		default:
			dev_notice(pctrl->dev, "Property %d is not supported for %s\n",
				   param, mcom03_lsperiph_pins[pin].name);

			dev_dbg(pctrl->dev, "Configuration for pin %s is not applied\n",
				mcom03_lsperiph_pins[pin].name);

			return -ENOTSUPP;
		}

		dev_dbg(pctrl->dev, "Try to set %d config to pin %s with argumet %d\n",
			param, mcom03_lsperiph_pins[pin].name, arg);
	}

	regmap_write(map, LSPERIPH1_CTR(pin), reg);

	dev_dbg(pctrl->dev, "Configs successfully set\n");

	return 0;
}

static int mcom03_lsperiph1_pin_config_group_get(struct pinctrl_dev *pctldev,
						 unsigned int selector,
						 unsigned long *config)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);
	unsigned int val, arg = 1;
	enum pin_config_param param = pinconf_to_config_param(*config);

	if (param != PIN_CONFIG_POWER_SOURCE)
		return -ENOTSUPP;

	regmap_read(pctrl->urb, LSPERIPH1_GROUP_REG, &val);
	arg = val & LSPERIPH1_1P8V ? 1800 : 3300;
	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int mcom03_lsperiph1_pin_config_group_set(struct pinctrl_dev *pctldev,
						 unsigned int selector,
						 unsigned long *configs,
						 unsigned int num_configs)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);
	struct regmap *map = pctrl->urb;
	enum pin_config_param param;
	u32 arg;

	param = pinconf_to_config_param(configs[0]);
	arg = pinconf_to_config_argument(configs[0]);

	if (param != PIN_CONFIG_POWER_SOURCE || num_configs != 1)
		return -ENOTSUPP;

	if (arg != 1800 && arg != 3300)
		return -EINVAL;

	regmap_update_bits(map, LSPERIPH1_GROUP_REG, LSPERIPH1_1P8V,
			   arg == 1800 ? LSPERIPH1_1P8V : 0);

	return 0;
}

static const struct pinconf_ops mcom03_lsperiph0_conf_ops = {
	.is_generic		= true,
	.pin_config_get		= mcom03_lsperiph0_pconf_get,
	.pin_config_set		= mcom03_lsperiph0_pconf_set,
};

static const struct pinconf_ops mcom03_lsperiph1_conf_ops = {
	.is_generic		= true,
	.pin_config_get		= mcom03_lsperiph1_pconf_get,
	.pin_config_set		= mcom03_lsperiph1_pconf_set,
	.pin_config_group_get	= mcom03_lsperiph1_pin_config_group_get,
	.pin_config_group_set	= mcom03_lsperiph1_pin_config_group_set,
};

static int mcom03_lsperiph_pinctrl_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);

	return pctrl->desc->id == PINCTRL_LSPERIPH1 ? 1 : 0;
}

static const char *mcom03_lsperiph_pinctrl_get_group_name(
						struct pinctrl_dev *pctldev,
						unsigned int group)
{
	struct mcom03_lsperiph_pinctrl *pctrl =
					pinctrl_dev_get_drvdata(pctldev);

	return pctrl->desc->id == PINCTRL_LSPERIPH1 ? gpio1_grp.name : 0;
}

static int mcom03_lsperiph_pinctrl_get_group_pins(struct pinctrl_dev *pctldev,
					 unsigned int group,
					 const unsigned int **pins,
					 unsigned int *num_pins)
{
	/*
	 * No need to check the subsystem id, if get_groups_count() returns 0
	 * the get_group_pins() will not be called.
	 */
	*pins = gpio1_grp.pins;
	*num_pins = ARRAY_SIZE(lsperiph1_pins);

	return 0;
}

static const struct pinctrl_ops mcom03_lsperiph_pinctrl_ops = {
	.dt_node_to_map		= pinconf_generic_dt_node_to_map_all,
	.dt_free_map		= pinctrl_utils_free_map,
	.get_groups_count	= mcom03_lsperiph_pinctrl_get_groups_count,
	.get_group_name		= mcom03_lsperiph_pinctrl_get_group_name,
	.get_group_pins		= mcom03_lsperiph_pinctrl_get_group_pins,
};

static struct pinctrl_desc pinctrl_lsperiph0_desc = {
	.name			= "mcom03-lsperiph0-pinctrl",
	.owner			= THIS_MODULE,
	.pins			= mcom03_lsperiph_pins,
	.npins			= ARRAY_SIZE(mcom03_lsperiph_pins),
	.confops		= &mcom03_lsperiph0_conf_ops,
	.pctlops		= &mcom03_lsperiph_pinctrl_ops,
};

struct mcom03_lsperiph_pinctrl_desc mcom03_lsperiph0_pinctrl = {
	.id			= PINCTRL_LSPERIPH0,
	.pinctrl_desc		= &pinctrl_lsperiph0_desc,
};

static struct pinctrl_desc pinctrl_lsperiph1_desc = {
	.name			= "mcom03-lsperiph1-pinctrl",
	.owner			= THIS_MODULE,
	.pins			= mcom03_lsperiph_pins,
	.npins			= ARRAY_SIZE(mcom03_lsperiph_pins),
	.confops		= &mcom03_lsperiph1_conf_ops,
	.pctlops		= &mcom03_lsperiph_pinctrl_ops,
};

struct mcom03_lsperiph_pinctrl_desc mcom03_lsperiph1_pinctrl = {
	.id			= PINCTRL_LSPERIPH1,
	.pinctrl_desc		= &pinctrl_lsperiph1_desc,
};

#ifdef CONFIG_OF
static const struct of_device_id mcom03_lsperiph_pinctrl_of_match[] = {
	{
		.compatible	= "elvees,mcom03-lsperiph0-pinctrl",
		.data		= &mcom03_lsperiph0_pinctrl,
	},
	{
		.compatible	= "elvees,mcom03-lsperiph1-pinctrl",
		.data		= &mcom03_lsperiph1_pinctrl,
	},
	{ /* sentinel */ },
};
#endif

static int mcom03_lsperiph_pinctrl_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct mcom03_lsperiph_pinctrl *pctrl;
	const struct mcom03_lsperiph_pinctrl_desc *desc;
	const struct of_device_id *match =
			of_match_device(mcom03_lsperiph_pinctrl_of_match, dev);
	if (!match)
		return -EINVAL;

	pctrl = devm_kzalloc(dev, sizeof(*pctrl), GFP_KERNEL);
	if (!pctrl)
		return -ENOMEM;

	desc = match->data;
	pctrl->urb = syscon_regmap_lookup_by_phandle(dev->of_node,
							   "elvees,urb");
	if (IS_ERR(pctrl->urb)) {
		dev_err(dev, "Failed to get elvees,urb\n");
		return PTR_ERR(pctrl->urb);
	}

	pctrl->desc = desc;
	pctrl->dev = dev;

	platform_set_drvdata(pdev, pctrl);

	ret = devm_pinctrl_register_and_init(dev, desc->pinctrl_desc, pctrl,
					     &pctrl->pctrl_dev);
	if (ret) {
		dev_err(dev, "Failed to register pinctrl (%d)\n", ret);
		return ret;
	}

	ret = pinctrl_enable(pctrl->pctrl_dev);
	if (ret) {
		dev_err(dev, "Failed to enable pinctrl (%d)\n", ret);
		return ret;
	}

	dev_info(dev, "%d pins & %d groups registered\n",
		 desc->pinctrl_desc->npins,
		 desc->id == PINCTRL_LSPERIPH0 ? 0 : 1);

	return 0;
}

static struct platform_driver mcom03_lsperiph_pinctrl_driver = {
	.probe = mcom03_lsperiph_pinctrl_probe,
	.driver = {
		.name = "mcom03-lsperiph-pinctrl",
		.of_match_table = mcom03_lsperiph_pinctrl_of_match,
	},
};

static int __init mcom03_lsperiph_api_pinctrl_register(void)
{
	return platform_driver_register(&mcom03_lsperiph_pinctrl_driver);
}

arch_initcall(mcom03_lsperiph_api_pinctrl_register);
