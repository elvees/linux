// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2023 RnD Center "ELVEES", JSC
 */

#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/hwmon.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_device.h>

struct alarm_desc {
	struct gpio_desc *gpio;
	int alarm;
	int irq;
};

struct gpio_alarm {
	struct device *dev;
	struct device *hwmon_dev;
	struct alarm_desc *intrusion;
	int num_intrusion;
	struct alarm_desc *temp;
	int num_temp;
};

static irqreturn_t temp_alarm_irq_handler(int irq, void *dev_id)
{
	struct gpio_alarm *data = dev_id;
	int i;

	for (i = 0; i < data->num_temp; i++)
		if (data->temp[i].irq == irq)
			data->temp[i].alarm = 1;

	return IRQ_HANDLED;
}

static irqreturn_t intrusion_alarm_irq_handler(int irq, void *dev_id)
{
	struct gpio_alarm *data = dev_id;
	int i;

	for (i = 0; i < data->num_intrusion; i++)
		if (data->intrusion[i].irq == irq)
			data->intrusion[i].alarm = 1;

	return IRQ_HANDLED;
}

static int temp_alarm_init(struct gpio_alarm *data)
{
	struct device *dev = data->dev;
	int ret;
	int i;

	for (i = 0; i < data->num_temp; i++) {
		data->temp[i].irq = gpiod_to_irq(data->temp[i].gpio);
		if (data->temp[i].irq <= 0)
			return data->temp[i].irq;

		irq_set_irq_type(data->temp[i].irq, IRQ_TYPE_EDGE_BOTH);

		ret = devm_request_irq(dev, data->temp[i].irq,
				       temp_alarm_irq_handler, IRQF_SHARED,
				       "GPIO temperature alarm", data);
		if (ret)
			return ret;
	}

	return ret;
}

static int intrusion_alarm_init(struct gpio_alarm *data)
{
	struct device *dev = data->dev;
	int ret;
	int i;

	for (i = 0; i < data->num_intrusion; i++) {
		data->intrusion[i].irq = gpiod_to_irq(data->intrusion[i].gpio);
		if (data->intrusion[i].irq <= 0)
			return data->temp[i].irq;

		irq_set_irq_type(data->intrusion[i].irq, IRQ_TYPE_EDGE_BOTH);

		ret = devm_request_irq(dev, data->intrusion[i].irq,
				       intrusion_alarm_irq_handler, IRQF_SHARED,
				       "GPIO intrusion alarm", data);
		if (ret)
			return ret;
	}

	return ret;
}

static umode_t
gpio_alarm_is_visible(const void *drvdata, enum hwmon_sensor_types type,
		      u32 attr, int channel)
{
	switch (type) {
	case hwmon_temp:
		return 0644;

	case hwmon_intrusion:
		return 0644;

	default: /* Shouldn't happen */
		return 0;
	}

	return 0; /* Shouldn't happen */
}

static int
gpio_alarm_read(struct device *dev, enum hwmon_sensor_types type,
		u32 attr, int channel, long *val)
{
	struct gpio_alarm *data = dev_get_drvdata(dev);

	switch (type) {
	case hwmon_temp:
		*val = data->temp[channel].alarm;
		return 0;
	case hwmon_intrusion:
		*val = data->intrusion[channel].alarm;
		return 0;

	default:
		break;
	}

	return -EOPNOTSUPP;
}

static int
gpio_alarm_write(struct device *dev, enum hwmon_sensor_types type,
		 u32 attr, int channel, long val)
{
	struct gpio_alarm *data = dev_get_drvdata(dev);

	if (type == hwmon_temp && attr == hwmon_temp_alarm) {
		data->temp[channel].alarm = 0;
		return 0;
	}

	if (type == hwmon_intrusion && attr == hwmon_intrusion_alarm) {
		data->intrusion[channel].alarm = 0;
		return 0;
	}

	return -EOPNOTSUPP;
}

static const struct hwmon_ops gpio_alarm_ops = {
	.is_visible = gpio_alarm_is_visible,
	.read = gpio_alarm_read,
	.write = gpio_alarm_write,
};

static struct hwmon_channel_info alarm_temp = {
	.type = hwmon_temp,
	.config = NULL,
};

static struct hwmon_channel_info alarm_intrusion = {
	.type = hwmon_intrusion,
	.config = NULL,
};

static const struct hwmon_channel_info **gpio_alarm_info;

static struct hwmon_chip_info gpio_alarm_chip_info = {
	.ops = &gpio_alarm_ops,
	.info = NULL,
};

static int gpio_alarm_get_of_intrusion(struct gpio_alarm *data)
{
	struct device *dev = data->dev;
	struct alarm_desc *alarms;
	int i;

	/* Fill GPIO pin array */
	data->num_intrusion = gpiod_count(dev, "intrusion");
	if (data->num_intrusion <= 0)
		return -ENODEV;

	alarms = devm_kcalloc(dev,
			      data->num_intrusion, sizeof(struct alarm_desc),
			      GFP_KERNEL);
	if (!alarms)
		return -ENOMEM;

	for (i = 0; i < data->num_intrusion; i++) {
		alarms[i].gpio = devm_gpiod_get_index(dev, "intrusion", i,
						      GPIOD_IN);
		if (IS_ERR(alarms[i].gpio))
			return PTR_ERR(alarms[i].gpio);
	}
	data->intrusion = alarms;

	return 0;
}

static int gpio_alarm_get_of_temp(struct gpio_alarm *data)
{
	struct device *dev = data->dev;
	struct alarm_desc *alarms;
	int i;

	/* Fill GPIO pin array */
	data->num_temp = gpiod_count(dev, "temp");
	if (data->num_temp <= 0)
		return -ENODEV;

	alarms = devm_kcalloc(dev,
			      data->num_temp, sizeof(struct alarm_desc),
			      GFP_KERNEL);
	if (!alarms)
		return -ENOMEM;

	for (i = 0; i < data->num_temp; i++) {
		alarms[i].gpio = devm_gpiod_get_index(dev, "temp", i, GPIOD_IN);
		if (IS_ERR(alarms[i].gpio))
			return PTR_ERR(alarms[i].gpio);
	}
	data->temp = alarms;

	return 0;
}

static int gpio_alarm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gpio_alarm *hwmon;
	u32 *config;
	int channel = 0;
	int err;
	int i;

	hwmon = devm_kzalloc(dev, sizeof(struct gpio_alarm), GFP_KERNEL);
	if (!hwmon)
		return -ENOMEM;

	hwmon->dev = dev;

	err = gpio_alarm_get_of_intrusion(hwmon);
	if (err && err != -ENODEV)
		return err;

	err = gpio_alarm_get_of_temp(hwmon);
	if (err && err != -ENODEV)
		return err;

	platform_set_drvdata(pdev, hwmon);

	i = hwmon->num_temp ? 1 : 0;
	i += hwmon->num_intrusion ? 1 : 0;
	if (i == 0) {
		dev_err(dev, "DT properties empty / missing");
		return -ENODEV;
	}
	i++;

	gpio_alarm_info = devm_kcalloc(dev, i, sizeof(struct hwmon_chip_info *),
				       GFP_KERNEL);
	if (!gpio_alarm_info)
		return -ENOMEM;

	if (hwmon->num_temp > 0) {
		i = hwmon->num_temp + 1;
		config = devm_kcalloc(dev, i, sizeof(u32), GFP_KERNEL);
		if (!config)
			return -ENOMEM;

		for (i = 0; i < hwmon->num_temp; i++)
			config[i] = HWMON_T_ALARM;

		alarm_temp.config = config;
		gpio_alarm_info[channel] = &alarm_temp;
		channel++;
	}

	if (hwmon->num_intrusion > 0) {
		i = hwmon->num_intrusion + 1;
		config = devm_kcalloc(dev, i, sizeof(u32), GFP_KERNEL);
		if (!config)
			return -ENOMEM;

		for (i = 0; i < hwmon->num_intrusion; i++)
			config[i] = HWMON_INTRUSION_ALARM;

		alarm_intrusion.config = config;
		gpio_alarm_info[channel] = &alarm_intrusion;
		channel++;
	}

	gpio_alarm_info[channel] = NULL;
	gpio_alarm_chip_info.info = gpio_alarm_info;

	/* Make this driver part of hwmon class. */
	hwmon->hwmon_dev =
		devm_hwmon_device_register_with_info(dev, "gpio_alarm", hwmon,
						     &gpio_alarm_chip_info,
						     NULL);
	if (IS_ERR(hwmon->hwmon_dev))
		return PTR_ERR(hwmon->hwmon_dev);

	/* Configure temp alarm GPIO if available. */
	if (hwmon->num_temp) {
		err = temp_alarm_init(hwmon);
		if (err)
			return err;
	}

	/* Configure intrusion alarm GPIO if available. */
	if (hwmon->num_intrusion) {
		err = intrusion_alarm_init(hwmon);
		if (err)
			return err;
	}

	dev_info(dev, "GPIO alarm initialized\n");

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id of_gpio_alarm_match[] = {
	{ .compatible = "gpio-alarm", },
	{},
};
MODULE_DEVICE_TABLE(of, of_gpio_alarm_match);
#endif

static struct platform_driver gpio_alarm_driver = {
	.probe		= gpio_alarm_probe,
	.driver	= {
		.name	= "gpio-alarm",
		.of_match_table = of_match_ptr(of_gpio_alarm_match),
	},
};

module_platform_driver(gpio_alarm_driver);

MODULE_DESCRIPTION("GPIO alarm driver");
MODULE_LICENSE("GPL");
