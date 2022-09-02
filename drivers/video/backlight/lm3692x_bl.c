// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 * Based on leds-lm3692x.c and adapted to backlight.
 *
 * This is temprorary driver for Kernel 4.19. In newer kernels should be
 * used led_bl driver that allows to use leds drivers for backlight purposes.
 */

#include <linux/backlight.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#define LM36922_MODEL	0
#define LM36923_MODEL	1

#define LM3692X_REV		0x0
#define LM3692X_RESET		0x1
#define LM3692X_EN		0x10
#define LM3692X_BRT_CTRL	0x11
#define LM3692X_PWM_CTRL	0x12
#define LM3692X_BOOST_CTRL	0x13
#define LM3692X_AUTO_FREQ_HI	0x15
#define LM3692X_AUTO_FREQ_LO	0x16
#define LM3692X_BL_ADJ_THRESH	0x17
#define LM3692X_BRT_LSB		0x18
#define LM3692X_BRT_MSB		0x19
#define LM3692X_FAULT_CTRL	0x1e
#define LM3692X_FAULT_FLAGS	0x1f

#define LM3692X_SW_RESET	BIT(0)
#define LM3692X_DEVICE_EN	BIT(0)
#define LM3692X_LED1_EN		BIT(1)
#define LM3692X_LED2_EN		BIT(2)
#define LM36923_LED3_EN		BIT(3)
#define LM3692X_ENABLE_MASK	(LM3692X_DEVICE_EN | LM3692X_LED1_EN | \
				 LM3692X_LED2_EN | LM36923_LED3_EN)

/* Brightness Control Bits */
#define LM3692X_BL_ADJ_POL	BIT(0)
#define LM3692X_RAMP_RATE_125us	0x00
#define LM3692X_RAMP_RATE_250us	BIT(1)
#define LM3692X_RAMP_RATE_500us BIT(2)
#define LM3692X_RAMP_RATE_1ms	(BIT(1) | BIT(2))
#define LM3692X_RAMP_RATE_2ms	BIT(3)
#define LM3692X_RAMP_RATE_4ms	(BIT(3) | BIT(1))
#define LM3692X_RAMP_RATE_8ms	(BIT(2) | BIT(3))
#define LM3692X_RAMP_RATE_16ms	(BIT(1) | BIT(2) | BIT(3))
#define LM3692X_RAMP_EN		BIT(4)
#define LM3692X_BRHT_MODE_REG	0x00
#define LM3692X_BRHT_MODE_PWM	BIT(5)
#define LM3692X_BRHT_MODE_MULTI_RAMP BIT(6)
#define LM3692X_BRHT_MODE_RAMP_MULTI (BIT(5) | BIT(6))
#define LM3692X_MAP_MODE_EXP	BIT(7)

/* PWM Register Bits */
#define LM3692X_PWM_FILTER_100	BIT(0)
#define LM3692X_PWM_FILTER_150	BIT(1)
#define LM3692X_PWM_FILTER_200	(BIT(0) | BIT(1))
#define LM3692X_PWM_HYSTER_1LSB BIT(2)
#define LM3692X_PWM_HYSTER_2LSB	BIT(3)
#define LM3692X_PWM_HYSTER_3LSB (BIT(3) | BIT(2))
#define LM3692X_PWM_HYSTER_4LSB BIT(4)
#define LM3692X_PWM_HYSTER_5LSB (BIT(4) | BIT(2))
#define LM3692X_PWM_HYSTER_6LSB (BIT(4) | BIT(3))
#define LM3692X_PWM_POLARITY	BIT(5)
#define LM3692X_PWM_SAMP_4MHZ	BIT(6)
#define LM3692X_PWM_SAMP_24MHZ	BIT(7)

/* Boost Control Bits */
#define LM3692X_OCP_PROT_1A	BIT(0)
#define LM3692X_OCP_PROT_1_25A	BIT(1)
#define LM3692X_OCP_PROT_1_5A	(BIT(0) | BIT(1))
#define LM3692X_OVP_21V		BIT(2)
#define LM3692X_OVP_25V		BIT(3)
#define LM3692X_OVP_29V		(BIT(2) | BIT(3))
#define LM3692X_MIN_IND_22UH	BIT(4)
#define LM3692X_BOOST_SW_1MHZ	BIT(5)
#define LM3692X_BOOST_SW_NO_SHIFT	BIT(6)

/* Fault Control Bits */
#define LM3692X_FAULT_CTRL_OVP BIT(0)
#define LM3692X_FAULT_CTRL_OCP BIT(1)
#define LM3692X_FAULT_CTRL_TSD BIT(2)
#define LM3692X_FAULT_CTRL_OPEN BIT(3)

/* Fault Flag Bits */
#define LM3692X_FAULT_FLAG_OVP BIT(0)
#define LM3692X_FAULT_FLAG_OCP BIT(1)
#define LM3692X_FAULT_FLAG_TSD BIT(2)
#define LM3692X_FAULT_FLAG_SHRT BIT(3)
#define LM3692X_FAULT_FLAG_OPEN BIT(4)

struct lm3692x_bl {
	struct mutex lock;
	struct i2c_client *client;
	struct backlight_device	*bl_dev;
	struct regmap *regmap;
	struct gpio_desc *enable_gpio;
	struct regulator *regulator;
	int max_brightness;
	int default_brightness;
	int led_enable;
	int model_id;
	u8 boost_ctrl;
};

static const struct reg_default lm3692x_reg_defs[] = {
	{LM3692X_EN, 0xf},
	{LM3692X_BRT_CTRL, 0x61},
	{LM3692X_PWM_CTRL, 0x73},
	{LM3692X_BOOST_CTRL, 0x6f},
	{LM3692X_AUTO_FREQ_HI, 0x0},
	{LM3692X_AUTO_FREQ_LO, 0x0},
	{LM3692X_BL_ADJ_THRESH, 0x0},
	{LM3692X_BRT_LSB, 0x7},
	{LM3692X_BRT_MSB, 0xff},
	{LM3692X_FAULT_CTRL, 0x7},
};

static const struct regmap_config lm3692x_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = LM3692X_FAULT_FLAGS,
	.reg_defaults = lm3692x_reg_defs,
	.num_reg_defaults = ARRAY_SIZE(lm3692x_reg_defs),
	.cache_type = REGCACHE_RBTREE,
};

static int lm3692x_bl_fault_check(struct lm3692x_bl *priv)
{
	int ret;
	unsigned int read_buf;

	ret = regmap_read(priv->regmap, LM3692X_FAULT_FLAGS, &read_buf);
	if (ret)
		return ret;

	if (read_buf)
		dev_err(&priv->client->dev, "Detected a fault 0x%X\n",
			read_buf);

	/* The first read may clear the fault.  Check again to see if the fault
	 * still exits and return that value.
	 */
	regmap_read(priv->regmap, LM3692X_FAULT_FLAGS, &read_buf);
	if (read_buf)
		dev_err(&priv->client->dev, "Second read of fault flags 0x%X\n",
			read_buf);

	return read_buf;
}

static int lm3692x_bl_brightness_set(struct lm3692x_bl *priv, u32 brt_val)
{
	int ret;

	mutex_lock(&priv->lock);

	ret = lm3692x_bl_fault_check(priv);
	if (ret) {
		dev_err(&priv->client->dev, "Cannot read/clear faults\n");
		goto out;
	}

	ret = regmap_write(priv->regmap, LM3692X_BRT_MSB, brt_val);
	if (ret) {
		dev_err(&priv->client->dev, "Cannot write MSB\n");
		goto out;
	}

	ret = regmap_write(priv->regmap, LM3692X_BRT_LSB, 0);
	if (ret) {
		dev_err(&priv->client->dev, "Cannot write LSB\n");
		goto out;
	}
out:
	mutex_unlock(&priv->lock);
	return ret;
}

static int lm3692x_bl_update_status(struct backlight_device *bl)
{
	struct lm3692x_bl *priv = dev_get_drvdata(&bl->dev);
	int brightness = bl->props.brightness;

	lm3692x_bl_brightness_set(priv, brightness);

	return 0;
}

static const struct backlight_ops lm3692x_bl_ops = {
	.update_status = lm3692x_bl_update_status,
};

static int lm3692x_bl_init(struct lm3692x_bl *priv)
{
	int enable_state;
	int ret;

	if (priv->regulator) {
		ret = regulator_enable(priv->regulator);
		if (ret) {
			dev_err(&priv->client->dev,
				"Failed to enable regulator\n");
			return ret;
		}
	}

	if (priv->enable_gpio)
		gpiod_direction_output(priv->enable_gpio, 1);

	ret = lm3692x_bl_fault_check(priv);
	if (ret) {
		dev_err(&priv->client->dev, "Cannot read/clear faults\n");
		goto out;
	}

	ret = regmap_write(priv->regmap, LM3692X_BRT_CTRL, 0x00);
	if (ret)
		goto out;

	/*
	 * For glitch free operation, the following data should
	 * only be written while LEDx enable bits are 0 and the device enable
	 * bit is set to 1.
	 * per Section 7.5.14 of the data sheet
	 */
	ret = regmap_write(priv->regmap, LM3692X_EN, LM3692X_DEVICE_EN);
	if (ret)
		goto out;

	/* Set the brightness to 0 so when enabled the LEDs do not come
	 * on with full brightness.
	 */
	ret = regmap_write(priv->regmap, LM3692X_BRT_MSB, 0);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_BRT_LSB, 0);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_PWM_CTRL,
		LM3692X_PWM_FILTER_100 | LM3692X_PWM_SAMP_24MHZ);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_BOOST_CTRL, priv->boost_ctrl);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_AUTO_FREQ_HI, 0x00);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_AUTO_FREQ_LO, 0x00);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_BL_ADJ_THRESH, 0x00);
	if (ret)
		goto out;

	ret = regmap_write(priv->regmap, LM3692X_BRT_CTRL,
			LM3692X_BL_ADJ_POL | LM3692X_RAMP_EN);
	if (ret)
		goto out;

	switch (priv->led_enable) {
	case 0:
	default:
		if (priv->model_id == LM36923_MODEL)
			enable_state = LM3692X_LED1_EN | LM3692X_LED2_EN |
			       LM36923_LED3_EN;
		else
			enable_state = LM3692X_LED1_EN | LM3692X_LED2_EN;

		break;
	case 1:
		enable_state = LM3692X_LED1_EN;
		break;
	case 2:
		enable_state = LM3692X_LED2_EN;
		break;

	case 3:
		if (priv->model_id == LM36923_MODEL) {
			enable_state = LM36923_LED3_EN;
			break;
		}

		ret = -EINVAL;
		dev_err(&priv->client->dev,
			"LED3 sync not available on this device\n");
		goto out;
	}

	ret = regmap_update_bits(priv->regmap, LM3692X_EN, LM3692X_ENABLE_MASK,
				 enable_state | LM3692X_DEVICE_EN);

	return ret;
out:
	dev_err(&priv->client->dev, "Fail writing initialization values\n");

	if (priv->enable_gpio)
		gpiod_direction_output(priv->enable_gpio, 0);

	if (priv->regulator) {
		ret = regulator_disable(priv->regulator);
		if (ret)
			dev_err(&priv->client->dev,
				"Failed to disable regulator\n");
	}

	return ret;
}

static u32 lm3692x_bl_max_brightness(u32 max_cur)
{
	u32 max_code;

	/* see p.12 of LM36922 data sheet for brightness formula */
	max_code = ((max_cur * 1000) - 37806) / 12195;
	if (max_code > 0x7FF)
		max_code = 0x7FF;

	return max_code >> 3;
}

static int lm3692x_bl_probe_dt(struct lm3692x_bl *priv)
{
	struct fwnode_handle *child = NULL;
	u32 ovp, max_cur, def_br;
	int ret;

	priv->enable_gpio = devm_gpiod_get_optional(&priv->client->dev,
						   "enable", GPIOD_OUT_LOW);
	if (IS_ERR(priv->enable_gpio)) {
		ret = PTR_ERR(priv->enable_gpio);
		dev_err(&priv->client->dev, "Failed to get enable gpio: %d\n",
			ret);
		return ret;
	}

	priv->regulator = devm_regulator_get_optional(&priv->client->dev,
						      "vled");
	if (IS_ERR(priv->regulator)) {
		ret = PTR_ERR(priv->regulator);
		if (ret != -ENODEV) {
			if (ret != -EPROBE_DEFER)
				dev_err(&priv->client->dev,
					"Failed to get vled regulator: %d\n",
					ret);
			return ret;
		}
		priv->regulator = NULL;
	}

	priv->boost_ctrl = LM3692X_BOOST_SW_1MHZ |
		LM3692X_BOOST_SW_NO_SHIFT |
		LM3692X_OCP_PROT_1_5A;
	ret = device_property_read_u32(&priv->client->dev,
				       "ti,ovp-microvolt", &ovp);
	if (ret) {
		priv->boost_ctrl |= LM3692X_OVP_29V;
	} else {
		switch (ovp) {
		case 17000000:
			break;
		case 21000000:
			priv->boost_ctrl |= LM3692X_OVP_21V;
			break;
		case 25000000:
			priv->boost_ctrl |= LM3692X_OVP_25V;
			break;
		case 29000000:
			priv->boost_ctrl |= LM3692X_OVP_29V;
			break;
		default:
			dev_err(&priv->client->dev, "Invalid OVP %d\n", ovp);
			return -EINVAL;
		}
	}

	child = device_get_next_child_node(&priv->client->dev, child);
	if (!child) {
		dev_err(&priv->client->dev, "No LED Child node\n");
		return -ENODEV;
	}

	ret = fwnode_property_read_u32(child, "reg", &priv->led_enable);
	if (ret) {
		dev_err(&priv->client->dev, "reg DT property missing\n");
		return ret;
	}

	ret = fwnode_property_read_u32(child, "led-max-microamp", &max_cur);
	priv->max_brightness = ret ? 255 :
		lm3692x_bl_max_brightness(max_cur);

	ret = fwnode_property_read_u32(child, "default-brightness-level",
				       &def_br);
	if (!ret && def_br <= priv->max_brightness)
		priv->default_brightness = def_br;
	else if (!ret && def_br > priv->max_brightness)
		dev_warn(&priv->client->dev,
			 "Invalid default brightness. Ignoring it\n");

	return 0;
}

static int lm3692x_bl_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct lm3692x_bl *priv;
	struct backlight_properties props;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	mutex_init(&priv->lock);
	priv->client = client;
	priv->model_id = id->driver_data;
	i2c_set_clientdata(client, priv);

	priv->regmap = devm_regmap_init_i2c(client, &lm3692x_regmap_config);
	if (IS_ERR(priv->regmap)) {
		ret = PTR_ERR(priv->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	ret = lm3692x_bl_probe_dt(priv);
	if (ret)
		return ret;

	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_RAW;
	props.max_brightness = priv->max_brightness;
	props.brightness = priv->default_brightness;
	props.power = (priv->default_brightness > 0) ? FB_BLANK_POWERDOWN :
		      FB_BLANK_UNBLANK;

	dev_set_drvdata(&client->dev, priv);
	ret = lm3692x_bl_init(priv);
	if (ret)
		return ret;

	priv->bl_dev = devm_backlight_device_register(&client->dev,
						      dev_name(&client->dev),
						      &client->dev,
						      priv,
						      &lm3692x_bl_ops,
						      &props);
	if (IS_ERR(priv->bl_dev)) {
		dev_err(&client->dev, "Failed to register backlight\n");
		return PTR_ERR(priv->bl_dev);
	}

	return 0;
}

static int lm3692x_bl_remove(struct i2c_client *client)
{
	struct lm3692x_bl *priv = i2c_get_clientdata(client);
	int ret;

	ret = regmap_update_bits(priv->regmap, LM3692X_EN, LM3692X_DEVICE_EN,
				 0);
	if (ret) {
		dev_err(&priv->client->dev, "Failed to disable regulator\n");
		return ret;
	}

	if (priv->enable_gpio)
		gpiod_direction_output(priv->enable_gpio, 0);

	if (priv->regulator) {
		ret = regulator_disable(priv->regulator);
		if (ret)
			dev_err(&priv->client->dev,
				"Failed to disable regulator\n");
	}

	mutex_destroy(&priv->lock);

	return 0;
}

static const struct i2c_device_id lm3692x_bl_id[] = {
	{ "lm36922-bl", LM36922_MODEL },
	{ "lm36923-bl", LM36923_MODEL },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lm3692x_bl_id);

#ifdef CONFIG_OF
static const struct of_device_id of_lm3692x_bl_match[] = {
	{ .compatible = "ti,lm36922-bl", },
	{ .compatible = "ti,lm36923-bl", },
	{},
};
MODULE_DEVICE_TABLE(of, of_lm3692x_bl_match);
#endif

static struct i2c_driver lm3692x_bl_driver = {
	.driver = {
		.name	= "lm3692x-bl",
		.of_match_table = of_match_ptr(of_lm3692x_bl_match),
	},
	.probe		= lm3692x_bl_probe,
	.remove		= lm3692x_bl_remove,
	.id_table	= lm3692x_bl_id,
};
module_i2c_driver(lm3692x_bl_driver);

MODULE_DESCRIPTION("Texas Instruments LM3692X backlight driver");
MODULE_LICENSE("GPL v2");
