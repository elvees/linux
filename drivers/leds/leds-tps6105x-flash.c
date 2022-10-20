// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/bits.h>
#include <linux/led-class-flash.h>
#include <linux/leds.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/mfd/tps6105x.h>
#include <linux/regmap.h>
#include <linux/delay.h>

#define TPS6105X_REG1_FLASHC_SHIFT 0
#define TPS6105X_REG1_FLASHC_MASK GENMASK(2, 0)
#define TPS6105X_REG1_SFT BIT(3)
#define TPS6105X_REG1_STT BIT(4)
#define TPS6105X_REG1_TIMEOUT BIT(5)
#define TPS6105X_REG2_WRITE_DIR BIT(0)
#define TPS6105X_REG2_WRITE_GPIO BIT(1)
#define TPS6105X_REG2_WRITE_TXMASK BIT(2)
#define TPS6105X_REG2_READ_LEDFAIL BIT(6)
#define TPS6105X_REG2_READ_OVERTEMP BIT(7)
#define TPS6105X_REG2_ILIM_SHIFT 5
#define TPS6105X_REG2_ILIM_MASK GENMASK(6, 5)
#define TPS6105X_REG2_ILIM_1000 0x0
#define TPS6105X_REG2_ILIM_1500 0x1
#define TPS6105X_REG2_ILIM_2000 0x3
#define TPS6105X_REG3_STIM_MASK GENMASK(4, 0)
#define TPS6105X_REG3_DCTIM_MASK GENMASK(7, 5)

#define TPS6105X_REG3_STIM_STEP_USEC 32800
#define TPS6105X_REG3_STIM_MAX 0x1f

enum flash_current {
	TPS6105X_REG1_FLASHC_150 = 0,
	TPS6105X_REG1_FLASHC_200,
	TPS6105X_REG1_FLASHC_300,
	TPS6105X_REG1_FLASHC_400,
	TPS6105X_REG1_FLASHC_500, // default
	TPS6105X_REG1_FLASHC_700,
	TPS6105X_REG1_FLASHC_900,
	TPS6105X_REG1_FLASHC_1200,
	TPS6105X_REG1_FLASHC_MAX
};

static u32 flash_current_arr[] = {
	150000,
	200000,
	300000,
	400000,
	500000,
	700000,
	900000,
	1200000,
};

static u32 torch_current_arr[] = {
	0,
	50000,
	75000,
	100000,
	150000,
	200000,
	250000, // 400000, read datasheet for terms
	250000, // 500000, read datasheet for terms
};

struct tps6105x_priv {
	struct regmap	*regmap;
	struct led_classdev_flash	fled_cdev;
	struct platform_device	*pdev;
	unsigned int	faults;
};

struct tps6105x_led_config_data {
	const char *label;
	/* maximum LED current in torch mode */
	u32		torch_max_microamp;
	/* maximum LED current in flash mode */
	u32		flash_max_microamp;
	/* maximum flash timeout */
	u32		flash_max_timeout;
	/* Safety timer edge or level sensitive */
	bool	edge_sensitive;
};

static void check_for_faults(unsigned int reg, struct tps6105x_priv *priv, unsigned int regval)
{
	if (priv) {
		switch (reg) {
		case TPS6105X_REG_0:
			break;
		case TPS6105X_REG_1:
			if (regval & TPS6105X_REG1_TIMEOUT)
				priv->faults |= LED_FAULT_TIMEOUT;
			break;
		case TPS6105X_REG_2:
			if (regval & TPS6105X_REG2_READ_LEDFAIL)
				priv->faults |= LED_FAULT_SHORT_CIRCUIT;
			if (regval & TPS6105X_REG2_READ_OVERTEMP)
				priv->faults |= LED_FAULT_OVER_TEMPERATURE;
			break;
		case TPS6105X_REG_3:
			break;
		default:
			break;
		}
	}
};

static int read_reg_check_faults(struct tps6105x_priv *priv, unsigned int reg, unsigned int *val)
{
	int ret;

	ret = regmap_read(priv->regmap, reg, val);
	check_for_faults(reg, priv, *val);
	return ret;
};

static inline unsigned int get_valid_max_flash_current_code(u32 val)
{
	u32 ilim_ua[] = { 1000000, 1500000, 1500000, 2000000 };

	if (val <= ilim_ua[TPS6105X_REG2_ILIM_1000])
		return TPS6105X_REG2_ILIM_1000;

	if (val >= ilim_ua[TPS6105X_REG2_ILIM_2000])
		return TPS6105X_REG2_ILIM_2000;

	return TPS6105X_REG2_ILIM_1500;
}

static inline unsigned int get_valid_flash_current_code(u32 val)
{
	int i;

	for (i = TPS6105X_REG1_FLASHC_1200; i >= TPS6105X_REG1_FLASHC_150; i--)
		if (val >= flash_current_arr[i])
			return i;

	return TPS6105X_REG1_FLASHC_150;
}

static inline unsigned int get_valid_torch_current_code(u32 val)
{
	int i;

	for (i = TPS6105X_REG0_TORCHC_250_500; i >= TPS6105X_REG0_TORCHC_0; i--)
		if (val >= torch_current_arr[i])
			return i;

	return TPS6105X_REG0_TORCHC_0;
}

static inline unsigned int get_valid_timeout(unsigned int us_timeout)
{
	if (us_timeout > TPS6105X_REG3_STIM_STEP_USEC * TPS6105X_REG3_STIM_MAX)
		return TPS6105X_REG3_STIM_STEP_USEC * TPS6105X_REG3_STIM_MAX;
	return us_timeout;
}

static int tps6105x_torch_brightness_set(struct led_classdev *cdev, enum led_brightness brightness)
{
	struct led_classdev_flash *fled_cdev = lcdev_to_flcdev(cdev);
	struct tps6105x_priv *priv = container_of(fled_cdev, struct tps6105x_priv,
						  fled_cdev);
	int ret;
	unsigned int regval = (brightness << TPS6105X_REG0_TORCHC_SHIFT) |
			(brightness ? TPS6105X_REG0_MODE_TORCH << TPS6105X_REG0_MODE_SHIFT : 0);

	ret = regmap_update_bits(priv->regmap, TPS6105X_REG_0,
				 TPS6105X_REG0_TORCHC_MASK | TPS6105X_REG0_MODE_MASK,
				 regval);

	return ret;
}

static int tps6105x_flash_brightness_set(struct led_classdev_flash *fled_cdev, u32 brightness)
{
	struct tps6105x_priv *priv = container_of(fled_cdev, struct tps6105x_priv,
						  fled_cdev);
	int ret;
	unsigned int regval;

	ret = read_reg_check_faults(priv, TPS6105X_REG_1, &regval);
	if (ret < 0)
		return ret;

	regval = regval & (~TPS6105X_REG1_FLASHC_MASK);
	regval |= get_valid_flash_current_code(brightness);
	ret = regmap_write(priv->regmap, TPS6105X_REG_1, regval);

	return ret;
}

static int tps6105x_flash_brightness_get(struct led_classdev_flash *fled_cdev, u32 *brightness)
{
	struct tps6105x_priv *priv = container_of(fled_cdev, struct tps6105x_priv,
						  fled_cdev);
	u32 regval;
	int ret;

	ret = read_reg_check_faults(priv, TPS6105X_REG_1, &regval);
	if (ret == 0)
		*brightness = flash_current_arr[regval & TPS6105X_REG1_FLASHC_MASK];

	return ret;
}

static int tps6105x_strobe_set(struct led_classdev_flash *fled_cdev, bool state)
{
	struct tps6105x_priv *priv = container_of(fled_cdev, struct tps6105x_priv,
						  fled_cdev);
	int ret = 0;

	if (state) {
		ret = regmap_update_bits(priv->regmap, TPS6105X_REG_1,
				TPS6105X_REG1_MODE_MASK | TPS6105X_REG1_SFT,
				(TPS6105X_REG1_MODE_TORCH_FLASH << TPS6105X_REG1_MODE_SHIFT) |
				TPS6105X_REG1_SFT);

		msleep(priv->fled_cdev.timeout.val / 1000);

		ret = regmap_update_bits(priv->regmap, TPS6105X_REG_1,
				TPS6105X_REG1_MODE_MASK,
				(fled_cdev->led_cdev.brightness ? TPS6105X_REG1_MODE_TORCH :
				TPS6105X_REG1_MODE_SHUTDOWN) << TPS6105X_REG1_MODE_SHIFT);
	}

	return ret;
}

static int tps6105x_strobe_get(struct led_classdev_flash *fled_cdev, bool *state)
{
	struct tps6105x_priv *priv = container_of(fled_cdev, struct tps6105x_priv,
						  fled_cdev);
	unsigned int regval;
	int ret;

	ret = read_reg_check_faults(priv, TPS6105X_REG_1, &regval);
	if (ret == 0)
		*state = regval & TPS6105X_REG1_SFT ? true : false;

	return ret;
}

static int tps6105x_timeout_set(struct led_classdev_flash *fled_cdev, u32 timeout)
{
	fled_cdev->timeout.val = get_valid_timeout(timeout);

	return 0;
}

static int tps6105x_fault_get(struct led_classdev_flash *fled_cdev, u32 *fault)
{
	struct tps6105x_priv *priv = container_of(fled_cdev, struct tps6105x_priv,
						  fled_cdev);
	unsigned int regval;
	int i, ret;

	for (i = TPS6105X_REG_0; i <= TPS6105X_REG_3; i++) {
		ret = read_reg_check_faults(priv, i, &regval);
		if (ret < 0)
			return ret;
	}

	*fault = priv->faults;
	priv->faults = 0; // all flags are reset after readout

	return ret;
}

struct led_flash_ops tps6105x_flash_ops = {
	.flash_brightness_set	= tps6105x_flash_brightness_set,
	.flash_brightness_get	= tps6105x_flash_brightness_get,
	.strobe_set		= tps6105x_strobe_set,
	.strobe_get		= tps6105x_strobe_get,
	.fault_get		= tps6105x_fault_get,
	.timeout_set	= tps6105x_timeout_set,
};

static int tps6105x_flash_parse_dt(struct tps6105x_priv *led,
				   struct tps6105x_led_config_data *cfg)
{
	struct device *dev = &led->pdev->dev;
	struct device_node *node = dev_of_node(dev);
	int ret;

	if (!dev_of_node(dev))
		return -ENXIO;

	cfg->label = of_get_property(node, "label", NULL) ? : node->name;

	ret = of_property_read_u32(node, "led-max-microamp",
				   &cfg->torch_max_microamp);
	if (ret < 0) {
		cfg->torch_max_microamp = 75000; //default
		dev_warn(dev, "led-max-microamp DT property missing\n");
	}

	ret = of_property_read_u32(node, "flash-max-microamp",
				   &cfg->flash_max_microamp);
	if (ret < 0) {
		cfg->flash_max_microamp = 500000; // default
		dev_warn(dev, "flash-max-microamp DT property missing\n");
	}

	ret = of_property_read_u32(node, "flash-max-timeout-us",
				   &cfg->flash_max_timeout);
	if (ret < 0) {
		cfg->flash_max_timeout = 557600; // default
		dev_warn(dev, "flash-max-timeout-us DT property missing\n");
	}
	cfg->flash_max_timeout = get_valid_timeout(cfg->flash_max_timeout);

	cfg->edge_sensitive = of_property_read_bool(node, "ti,edge-sensitive");

	return 0;
}

static int tps6105x_flash_setup(struct tps6105x_priv *led,
				struct tps6105x_led_config_data *cfg)
{
	unsigned int regval;
	int ret;

	regval = TPS6105X_REG3_STIM_MASK & (cfg->flash_max_timeout /
					    TPS6105X_REG3_STIM_STEP_USEC);
	ret = regmap_write(led->regmap, TPS6105X_REG_3, regval);
	if (ret < 0)
		return ret;

	regval = TPS6105X_REG2_ILIM_MASK |
		get_valid_max_flash_current_code(cfg->flash_max_microamp) <<
		TPS6105X_REG2_ILIM_SHIFT;
	ret = regmap_write(led->regmap, TPS6105X_REG_2, regval);
	if (ret < 0)
		return ret;

	regval = get_valid_flash_current_code(cfg->flash_max_microamp) |
		cfg->edge_sensitive ? TPS6105X_REG1_STT : 0;
	ret = regmap_write(led->regmap, TPS6105X_REG_1, regval);
	if (ret < 0)
		return ret;

	regval = (TPS6105X_REG0_MODE_SHUTDOWN << TPS6105X_REG0_MODE_SHIFT) |
		 (TPS6105X_REG0_VOLTAGE_500 << TPS6105X_REG0_VOLTAGE_SHIFT);
	ret = regmap_write(led->regmap, TPS6105X_REG_0, regval);

	return ret;
}

static int tps6105x_flash_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct tps6105x *tps6105x = dev_get_drvdata(dev->parent);
	struct tps6105x_priv *priv;
	struct tps6105x_led_config_data flash_cfg = {};
	struct led_classdev *led_cdev;
	struct led_classdev_flash *fled_cdev;
	int ret;

	/* This instance is not set for flash mode so bail out */
	if (tps6105x->pdata->mode != TPS6105X_MODE_TORCH_FLASH) {
		dev_info(&pdev->dev,
			 "chip not in flash mode, exit probe");
		return -EINVAL;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->regmap = tps6105x->regmap;
	priv->pdev = pdev;
	fled_cdev = &priv->fled_cdev;
	fled_cdev->ops = &tps6105x_flash_ops;

	platform_set_drvdata(pdev, priv);
	ret = tps6105x_flash_parse_dt(priv, &flash_cfg);
	if (ret < 0)
		goto probe_fail;

	led_cdev = &priv->fled_cdev.led_cdev;
	led_cdev->name = flash_cfg.label;
	led_cdev->brightness_set_blocking = tps6105x_torch_brightness_set;
	led_cdev->max_brightness = get_valid_torch_current_code(
			flash_cfg.torch_max_microamp);
	led_cdev->flags |= LED_DEV_CAP_FLASH;

	fled_cdev->timeout.min = TPS6105X_REG3_STIM_STEP_USEC;
	fled_cdev->timeout.max = flash_cfg.flash_max_timeout;
	fled_cdev->timeout.step = TPS6105X_REG3_STIM_STEP_USEC;
	fled_cdev->timeout.val = flash_cfg.flash_max_timeout;

	fled_cdev->brightness.min = 0;
	fled_cdev->brightness.max = flash_cfg.flash_max_microamp;
	fled_cdev->brightness.step = 50000;
	fled_cdev->brightness.val = flash_current_arr[get_valid_flash_current_code(
					flash_cfg.flash_max_microamp)];

	ret = tps6105x_flash_setup(priv, &flash_cfg);
	if (ret < 0)
		goto probe_fail;

	ret = led_classdev_flash_register(dev, fled_cdev);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot register flash\n");
		goto probe_fail;
	}

	return 0;

probe_fail:
	return ret;
}

static int tps6105x_flash_remove(struct platform_device *pdev)
{
	struct tps6105x_priv *priv = platform_get_drvdata(pdev);

	led_classdev_flash_unregister(&priv->fled_cdev);

	return 0;
}

static const struct of_device_id tps6105x_flash_of_id_table[] = {
	{ .compatible = "ti,tps6105x-flash" },
	{ }
};
MODULE_DEVICE_TABLE(of, tps6105x_flash_of_id_table);

static struct platform_driver flash_driver = {
	.probe			= tps6105x_flash_probe,
	.remove			= tps6105x_flash_remove,
	.driver			= {
		.name		= "tps6105x-flash",
		.of_match_table = tps6105x_flash_of_id_table,
	},
};

module_platform_driver(flash_driver);

MODULE_DESCRIPTION("TPS6105x FLASH driver");
MODULE_AUTHOR("Ilya Popov <ipopov@elvees.com>");
MODULE_LICENSE("GPL v2");
