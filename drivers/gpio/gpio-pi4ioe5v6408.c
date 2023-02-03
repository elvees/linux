// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the Pericom PI4IOE5V6408 GPIO Expander.
 *
 * Copyright (C) 2018 Google, LLC.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define PI4IO_CHIP_ID 0x1
#define PI4IO_IO_DIRECTION 0x3
#define PI4IO_OUTPUT 0x5
#define PI4IO_OUTPUT_HI_IMPEDANCE 0x7
#define PI4IO_INPUT_STATUS 0xF
#define PI4IO_INTERRUPT_STATUS 0x13

#define PI4IO_CHIP_ID_VAL 0xA0
#define PI4IO_CHIP_ID_MASK 0xFC

#define PI4IO_DIRECTION_TO_GPIOD(x) ((x) ? GPIOF_DIR_OUT : GPIOF_DIR_IN)
#define GPIOD_DIRECTION_TO_PI4IO(x) ((x) == GPIOF_DIR_OUT ? 1 : 0)

#define GPIO_OUT_LOW 0
#define GPIO_OUT_HIGH 1

static bool pi4io_readable_reg(struct device *dev, unsigned int reg)
{
	// All readable registers are odd-numbered.
	return (reg % 2) == 1;
}

static bool pi4io_writeable_reg(struct device *dev, unsigned int reg)
{
	// All odd-numbered registers are writable except for 0xF.
	if ((reg % 2) == 1) {
		if (reg != PI4IO_INPUT_STATUS)
			return true;
	}
	return false;
}

static bool pi4io_volatile_reg(struct device *dev, unsigned int reg)
{
	if (reg == PI4IO_INPUT_STATUS || reg == PI4IO_INTERRUPT_STATUS)
		return true;

	return false;
}

static const struct regmap_config pi4io_regmap = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = 0x13,
	.writeable_reg = pi4io_writeable_reg,
	.readable_reg = pi4io_readable_reg,
	.volatile_reg = pi4io_volatile_reg,
};

struct pi4io_priv {
	struct i2c_client *i2c;
	struct regmap *regmap;
	struct gpio_chip gpio;
};

static int pi4io_gpio_get_direction(struct gpio_chip *chip, unsigned int offset)
{
	int ret, io_dir;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_read(pi4io->regmap, PI4IO_IO_DIRECTION, &io_dir);
	if (ret) {
		dev_err(dev, "Failed to read I/O direction: %d", ret);
		return ret;
	}

	return PI4IO_DIRECTION_TO_GPIOD((io_dir >> offset) & 1);
}

static int pi4io_gpio_set_direction(
	struct gpio_chip *chip, unsigned int offset, int direction)
{
	int ret;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_update_bits(pi4io->regmap, PI4IO_IO_DIRECTION, 1 << offset,
		GPIOD_DIRECTION_TO_PI4IO(direction) << offset);
	if (ret) {
		dev_err(dev, "Failed to set direction: %d", ret);
		return ret;
	}

	// We desire the hi-impedance state to track the output state.
	ret = regmap_update_bits(pi4io->regmap, PI4IO_OUTPUT_HI_IMPEDANCE,
		1 << offset, direction << offset);

	return ret;
}

static int pi4io_gpio_get(struct gpio_chip *chip, unsigned int offset)
{
	int ret, out;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_read(pi4io->regmap, PI4IO_OUTPUT, &out);
	if (ret) {
		dev_err(dev, "Failed to read output: %d", ret);
		return ret;
	}

	if (out & (1 << offset))
		return 1;

	return 0;
}

static void pi4io_gpio_set(struct gpio_chip *chip, unsigned int offset, int value)
{
	int ret;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = regmap_update_bits(
		pi4io->regmap, PI4IO_OUTPUT, 1 << offset, value << offset);
	if (ret)
		dev_err(dev, "Failed to write output: %d", ret);
}

static int pi4io_gpio_direction_input(struct gpio_chip *chip, unsigned int offset)
{
	return pi4io_gpio_set_direction(chip, offset, GPIOF_DIR_IN);
}

static int pi4io_gpio_direction_output(
	struct gpio_chip *chip, unsigned int offset, int value)
{
	int ret;
	struct pi4io_priv *pi4io = gpiochip_get_data(chip);
	struct device *dev = &pi4io->i2c->dev;

	ret = pi4io_gpio_set_direction(chip, offset, GPIOF_DIR_OUT);
	if (ret) {
		dev_err(dev, "Failed to set direction: %d", ret);
		return ret;
	}
	pi4io_gpio_set(chip, offset, value);
	return 0;
}

static int pi4io_gpio_setup(struct pi4io_priv *pi4io)
{
	int ret;
	struct device *dev = &pi4io->i2c->dev;
	struct gpio_chip *gc = &pi4io->gpio;

	gc->ngpio = 8;
	gc->label = pi4io->i2c->name;
	gc->parent = &pi4io->i2c->dev;
	gc->owner = THIS_MODULE;
	gc->base = -1;
	gc->can_sleep = true;

	gc->get_direction = pi4io_gpio_get_direction;
	gc->direction_input = pi4io_gpio_direction_input;
	gc->direction_output = pi4io_gpio_direction_output;
	gc->get = pi4io_gpio_get;
	gc->set = pi4io_gpio_set;

	ret = devm_gpiochip_add_data(dev, gc, pi4io);
	if (ret) {
		dev_err(dev, "devm_gpiochip_add_data failed: %d", ret);
		return ret;
	}
	return 0;
}

static int pi4io_probe(
	struct i2c_client *client, const struct i2c_device_id *id)
{
	int ret, chip_id;
	struct device *dev = &client->dev;
	struct pi4io_priv *pi4io;

	pi4io = devm_kzalloc(dev, sizeof(struct pi4io_priv), GFP_KERNEL);
	if (!pi4io)
		return -ENOMEM;

	i2c_set_clientdata(client, pi4io);
	pi4io->i2c = client;

	pi4io->regmap = devm_regmap_init_i2c(client, &pi4io_regmap);
	ret = regmap_read(pi4io->regmap, PI4IO_CHIP_ID, &chip_id);
	if (ret < 0) {
		dev_err(dev, "Failed to read Chip ID: %d", ret);
		return ret;
	}

	if ((chip_id & PI4IO_CHIP_ID_MASK) != PI4IO_CHIP_ID_VAL) {
		dev_err(dev, "Invalid Chip ID!");
		return -EINVAL;
	}

	ret = pi4io_gpio_setup(pi4io);
	if (ret < 0) {
		dev_err(dev, "Failed to setup GPIOs: %d", ret);
		return ret;
	}

	dev_dbg(dev, "PI4IO probe finished");
	return 0;
}

static const struct of_device_id pi4io_of_match_table[] = {
	{ .compatible = "pericom,pi4ioe5v6408" },
	{ }
};
MODULE_DEVICE_TABLE(of, pi4io_of_match_table);

static const struct i2c_device_id pi4io_id_table[] = { { "pi4io", 0 }, {} };
MODULE_DEVICE_TABLE(i2c, pi4io_id_table);

static struct i2c_driver pi4io_driver = {
	.driver = {
		.name = "pi4io-gpio",
		.of_match_table = pi4io_of_match_table,
	},
	.probe = pi4io_probe,
	.id_table = pi4io_id_table,
};
module_i2c_driver(pi4io_driver);

MODULE_AUTHOR("Alex Van Damme <atv@google.com>");
MODULE_DESCRIPTION("PI4IOE5V6408");
MODULE_LICENSE("GPL v2");
