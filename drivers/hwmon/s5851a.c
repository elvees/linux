// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 * An hwmon driver for the Ablic S-5851A Series temperature sensor.
 *
 */

#include <linux/bitops.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>

#define S5851A_REG_TEMP		0x00
#define S5851A_REG_CONFIG	0x01

struct s5851a_data {
	struct i2c_client	*client;
	struct mutex		lock;
};

static s32 s5851a_read_temp_reg(struct device *dev)
{
	struct s5851a_data *data = dev_get_drvdata(dev);
	s32 value;
	int ret;

	ret = mutex_lock_interruptible(&data->lock);
	if (ret)
		return ret;

	value = i2c_smbus_read_word_data(data->client, S5851A_REG_TEMP);
	mutex_unlock(&data->lock);

	if (value < 0) {
		dev_dbg(&data->client->dev, "Temperature read err %d\n",
			(int)value);

		return value;
	}

	return be16_to_cpu(value);
}

static int s5851a_read(struct device *dev, enum hwmon_sensor_types type,
		       u32 attr, int channel, long *val)
{
	s32 value;

	if (attr != hwmon_temp_input)
		return -EOPNOTSUPP;

	value = s5851a_read_temp_reg(dev);
	if (value < 0)
		return value;

	value = (s16)value / 16;

	/* Convert from 0.0625 to 0.001 resolution */
	*val = value * 125 / 2;

	return 0;
}

static umode_t s5851a_is_visible(const void *_data,
				 enum hwmon_sensor_types type,
				 u32 attr, int channel)
{
	return (attr == hwmon_temp_input) ? 0444 : 0;
}

static const u32 s5851a_temp_config[] = {
	HWMON_T_INPUT,
	0
};

static const struct hwmon_channel_info s5851a_temp = {
	.type = hwmon_temp,
	.config = s5851a_temp_config,
};

static const struct hwmon_channel_info *s5851a_info[] = {
	&s5851a_temp,
	NULL
};

static const struct hwmon_ops s5851a_hwmon_ops = {
	.is_visible = s5851a_is_visible,
	.read = s5851a_read,
};

static const struct hwmon_chip_info s5851a_chip_info = {
	.ops = &s5851a_hwmon_ops,
	.info = s5851a_info,
};

static int s5851a_probe(struct i2c_client *client,
		      const struct i2c_device_id *dev_id)
{
	struct device *dev = &client->dev;
	struct s5851a_data *data;
	struct device *hdev;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_SMBUS_BYTE_DATA))
		return -EOPNOTSUPP;

	data = devm_kzalloc(dev, sizeof(struct s5851a_data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->client = client;
	mutex_init(&data->lock);
	hdev = devm_hwmon_device_register_with_info(dev, client->name, data,
						    &s5851a_chip_info, NULL);

	return PTR_ERR_OR_ZERO(hdev);
}

static const struct i2c_device_id s5851a_id[] = {
	{ "s5851a", 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, s5851a_id);

static struct i2c_driver s5851a_driver = {
	.driver = {
		.name	= "s5851a",
	},
	.probe	= s5851a_probe,
	.id_table = s5851a_id,
};

module_i2c_driver(s5851a_driver);

MODULE_DESCRIPTION("S-5851A driver");
MODULE_LICENSE("GPL");
