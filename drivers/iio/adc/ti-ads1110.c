// SPDX-License-Identifier: GPL-2.0+
/*
 * ADS1110 - Texas Instruments Analog-to-Digital Converter
 *
 * Copyright 2011 Flytec AG.
 *
 * Datasheet can be found on: http://www.ti.com/lit/ds/symlink/ads1110.pdf
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>

#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>

#define ADS1110_DATA_BYTES		2
#define ADS1110_CONFIG_BYTES		3

#define ADS1110_CYC_MASK		0x0C
#define ADS1110_CYC_SHIFT		2
#define ADS1110_CYC_15			3
#define ADS1110_CYC_30			2
#define ADS1110_CYC_60			1
#define ADS1110_CYC_240			0

#define ADS1110_PGA_MASK		0x03
#define ADS1110_PGA_COUNT		4

struct ads1110 {
	struct i2c_client *client;

	u8 config;
};

static const unsigned int ads1110_frequencies[] = {
	[ADS1110_CYC_240] = 240,
	[ADS1110_CYC_60] = 60,
	[ADS1110_CYC_30] = 30,
	[ADS1110_CYC_15] = 15,
};

static const struct iio_chan_spec ads1110_channels[] = {
	{
		.type = IIO_VOLTAGE,
		.channel = 0,
		.channel2 = 0,
		.address = 0,
		.scan_index = 0,
		.scan_type = {
			.sign = 's',
			.realbits = 16,
			.storagebits = 16,
			.shift = 0,
		},
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.extend_name = NULL,
		.modified = 0,
		.indexed = 1,
	},
	IIO_CHAN_SOFT_TIMESTAMP(1),
};

static int ads1110_i2c_read_config(struct ads1110 *chip, u8 *data)
{
	int ret = 0;
	u8 tmp[ADS1110_CONFIG_BYTES];

	ret = i2c_master_recv(chip->client, tmp, ADS1110_CONFIG_BYTES);
	if (ret != ADS1110_CONFIG_BYTES) {
		dev_err(&chip->client->dev, "I2C read error\n");
		return -EIO;
	}

	*data = tmp[2];

	return 0;
}

static int ads1110_i2c_read_data(struct ads1110 *chip, int *data)
{
	int ret = 0;
	__be16 tmp;

	ret = i2c_master_recv(chip->client, (char *)&tmp, ADS1110_DATA_BYTES);
	if (ret != ADS1110_DATA_BYTES) {
		dev_err(&chip->client->dev, "I2C read error\n");
		return -EIO;
	}

	*data = be16_to_cpu(tmp);

	return 0;
}

static ssize_t ads1110_read_frequency(struct device *dev,
				      struct device_attribute *attr,
				      char *buf)
{
	struct iio_dev *iio = dev_get_drvdata(dev);
	struct ads1110 *chip = iio_priv(iio);
	u8 cfg;

	mutex_lock(&iio->mlock);
	cfg = ((chip->config & ADS1110_CYC_MASK) >> ADS1110_CYC_SHIFT);
	mutex_unlock(&iio->mlock);

	return sprintf(buf, "%d\n", ads1110_frequencies[cfg]);
}

static ssize_t ads1110_write_frequency(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf,
				       size_t len)
{
	struct iio_dev *iio = dev_get_drvdata(dev);
	struct ads1110 *chip = iio_priv(iio);
	unsigned long lval;
	int ret, i;

	mutex_lock(&iio->mlock);
	ret = kstrtoul(buf, 10, &lval);
	if (ret)
		goto out;

	for (i = 0; i < ARRAY_SIZE(ads1110_frequencies); i++)
		if (lval == ads1110_frequencies[i])
			break;

	if (i == ARRAY_SIZE(ads1110_frequencies)) {
		ret = -EINVAL;
		goto out;
	}

	chip->config &= ~ADS1110_CYC_MASK;
	chip->config |= (i << ADS1110_CYC_SHIFT);

	ret = i2c_master_send(chip->client, &chip->config, 1);
	if (ret < 0) {
		ret = -EIO;
		dev_err(&chip->client->dev, "I2C write error\n");
	}

out:
	mutex_unlock(&iio->mlock);
	return ret < 0 ? ret : len;
}

static IIO_DEV_ATTR_SAMP_FREQ(S_IWUSR | S_IRUGO,
		ads1110_read_frequency,
		ads1110_write_frequency);

static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("240 60 30 15");

static IIO_CONST_ATTR(in_voltage_scale_available, "1 2 4 8");

static int ads1110_read_raw(struct iio_dev *iio,
			    struct iio_chan_spec const *chan,
			    int *val,
			    int *val2,
			    long mask)
{
	struct ads1110 *chip = iio_priv(iio);
	int ret, data, gain;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		mutex_lock(&iio->mlock);
		ret = ads1110_i2c_read_data(chip, &data);
		mutex_unlock(&iio->mlock);

		if (ret < 0)
			return -EIO;

		*val = data;

		return IIO_VAL_INT;

	case IIO_CHAN_INFO_SCALE:
		mutex_lock(&iio->mlock);
		gain = 1 << (chip->config & ADS1110_PGA_MASK);
		mutex_unlock(&iio->mlock);

		*val = gain;

		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static int ads1110_write_raw(struct iio_dev *iio,
			     struct iio_chan_spec const *chan,
			     int val,
			     int val2,
			     long mask)
{
	struct ads1110 *chip = iio_priv(iio);
	int ret, i;

	mutex_lock(&iio->mlock);
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		ret = -EINVAL;

		for (i = 0; i < ADS1110_PGA_COUNT; i++)
			if (val == (1 << i))
				break;

		if (i == ADS1110_PGA_COUNT) {
			ret = -EINVAL;
			goto out;
		}

		if (chip->config && i) {
			ret = 0;
			goto out;
		}

		chip->config &= ~ADS1110_PGA_MASK;
		chip->config |= i;

		ret = i2c_master_send(chip->client, &chip->config, 1);
		if (ret < 0) {
			ret = -EIO;
			dev_err(&chip->client->dev, "I2C write error\n");
			goto out;
		}
		break;

	default:
		ret = -EINVAL;
	}

out:
	mutex_unlock(&iio->mlock);

	return ret;
}


static struct attribute *ads1110_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_in_voltage_scale_available.dev_attr.attr,
	NULL,
};

static const struct attribute_group ads1110_attribute_group = {
	.attrs = ads1110_attributes,
};

static const struct iio_info ads1110_info = {
	.attrs = &ads1110_attribute_group,
	.read_raw = &ads1110_read_raw,
	.write_raw = &ads1110_write_raw,
};

static int ads1110_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct iio_dev *iio;
	struct ads1110 *chip;
	int ret;

	iio = devm_iio_device_alloc(&client->dev, sizeof(*chip));
	if (!iio)
		return -ENOMEM;

	chip = iio_priv(iio);
	chip->client = client;

	/* Establish that the iio_dev is a child of the i2c device */
	iio->dev.parent = &client->dev;
	iio->name = id->name;
	iio->modes = INDIO_DIRECT_MODE;
	iio->info = &ads1110_info;

	iio->channels = ads1110_channels;
	iio->num_channels = ARRAY_SIZE(ads1110_channels);

	/* read the config register from the chip */
	ret = ads1110_i2c_read_config(chip, &chip->config);
	if (ret < 0)
		goto error_free_dev;

	ret = iio_device_register(iio);
	if (ret < 0)
		goto error_free_dev;

	/* this is only used for device removal purposes  */
	i2c_set_clientdata(client, iio);

	dev_err(&client->dev, "%s ADC registered.\n", id->name);

	return 0;

error_free_dev:

	return ret;
}

static int ads1110_remove(struct i2c_client *client)
{
	struct iio_dev *iio = i2c_get_clientdata(client);

	iio_device_unregister(iio);

	return 0;
}

static const struct i2c_device_id ads1110_id[] = {
	{ "ads1110", 0x00 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ads1110_id);

#ifdef CONFIG_OF
static const struct of_device_id ads1110_of_match[] = {
	{ .compatible = "ti,ads1110" },
	{ }
};
MODULE_DEVICE_TABLE(of, ads1110_of_match);
#endif

static struct i2c_driver ads1110_driver = {
	.driver = {
		.name = "ads1110",
		.of_match_table = of_match_ptr(ads1110_of_match),
	},
	.probe = ads1110_probe,
	.remove = ads1110_remove,
	.id_table = ads1110_id,
};

module_i2c_driver(ads1110_driver);

MODULE_AUTHOR("Duss Pirmin <pirmin.duss@flytec.ch>");
MODULE_DESCRIPTION("Texas Instruments ADS1110 ADC driver");
MODULE_LICENSE("GPL v2");
