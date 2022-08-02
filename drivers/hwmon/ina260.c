// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 * Driver for Texas Instruments INA260 power monitor chip
 *
 * INA260:
 * Bi-directional Current/Power Monitor with I2C Interface
 * Datasheet: http://www.ti.com/product/ina260
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/jiffies.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <linux/util_macros.h>
#include <linux/regmap.h>

/* INA260 register definitions */
#define INA260_CONFIG			0x00
#define INA260_CURRENT			0x01
#define INA260_BUS_VOLTAGE		0x02
#define INA260_POWER			0x03
#define INA260_DIE_ID			0xFF

/* register count */
#define INA260_REGISTERS		8
#define INA260_CURRENT_LSB_uA		1250
#define INA260_BUS_VOLTAGE_LSB_uV	1250
#define INA260_POWER_LSB_mW		10

/* settings - depend on use case */
#define INA260_CONFIG_DEFAULT		0x6527	/* averages=16 */

/* bit mask for reading the averaging setting in the configuration register */
#define INA260_AVG_RD_MASK		0x0E00

#define INA260_READ_AVG(reg)		(((reg) & INA260_AVG_RD_MASK) >> 9)
#define INA260_SHIFT_AVG(val)		((val) << 9)

/* ina260 attrs and NULL */
#define INA260_MAX_ATTRIBUTE_GROUPS	2

/*
 * Bus voltage conversion time for ina260 is set
 * to 0b0100 on POR, which translates to 2200 microseconds in total.
 */
#define INA260_TOTAL_CONV_TIME_DEFAULT	2200

static struct regmap_config ina260_regmap_config = {
	.reg_bits = 8,
	.val_bits = 16,
};

struct ina260_data {
	u16 config_default;
	int registers;
	int current_lsb_uA;
	int bus_voltage_shift;
	int bus_voltage_lsb_uV;
	int power_lsb_mW;
	struct mutex config_lock;
	struct regmap *regmap;

	const struct attribute_group *groups[INA260_MAX_ATTRIBUTE_GROUPS];
};

/*
 * Available averaging rates for ina260. The indices correspond with
 * the bit values expected by the chip (according to the ina260 datasheet,
 * table 3 AVG bit settings, found at
 * http://www.ti.com/lit/ds/symlink/ina260.pdf.
 */
static const int ina260_avg_tab[] = { 1, 4, 16, 64, 128, 256, 512, 1024 };

static int ina260_reg_to_interval(u16 config)
{
	int avg = ina260_avg_tab[INA260_READ_AVG(config)];

	/*
	 * Multiply the total conversion time by the number of averages.
	 * Return the result in milliseconds.
	 */
	return DIV_ROUND_CLOSEST(avg * INA260_TOTAL_CONV_TIME_DEFAULT, 1000);
}

/*
 * Return the new, shifted AVG field value of CONFIG register,
 * to use with regmap_update_bits
 */
static u16 ina260_interval_to_reg(int interval)
{
	int avg, avg_bits;

	avg = DIV_ROUND_CLOSEST(interval * 1000,
				INA260_TOTAL_CONV_TIME_DEFAULT);
	avg_bits = find_closest(avg, ina260_avg_tab,
				ARRAY_SIZE(ina260_avg_tab));

	return INA260_SHIFT_AVG(avg_bits);
}

/*
 * Initialize the configuration register.
 */
static int ina260_init(struct ina260_data *data)
{
	int ret = regmap_write(data->regmap, INA260_CONFIG,
			       data->config_default);
	return ret;
}

static int ina260_read_reg(struct device *dev, int reg, unsigned int *regval)
{
	struct ina260_data *data = dev_get_drvdata(dev);
	int ret, retry;

	dev_dbg(dev, "Starting register %d read\n", reg);

	ret = regmap_read(data->regmap, reg, regval);
	if (ret < 0)
		return ret;

	dev_dbg(dev, "read %d, val = 0x%04x\n", reg, *regval);

	return 0;
}

static int ina260_get_value(struct ina260_data *data, u8 reg,
			    unsigned int regval)
{
	int val;

	switch (reg) {
	case INA260_BUS_VOLTAGE:
		/* Calculated power in mV */
		val = (regval >> data->bus_voltage_shift)
		  * data->bus_voltage_lsb_uV;
		val = DIV_ROUND_CLOSEST(val, 1000);
		break;
	case INA260_POWER:
		/* Calculated power in uW */
		val = regval * data->power_lsb_mW * 1000;
		break;
	case INA260_CURRENT:
		/* Signed register, calculated current in mA */
		val = (s16)regval * data->current_lsb_uA;
		val = DIV_ROUND_CLOSEST(val, 1000);
		break;
	default:
		/* programmer goofed */
		WARN_ON_ONCE(1);
		val = 0;
		break;
	}

	return val;
}

static ssize_t ina260_value_show(struct device *dev,
				 struct device_attribute *da, char *buf)
{
	struct sensor_device_attribute *attr = to_sensor_dev_attr(da);
	struct ina260_data *data = dev_get_drvdata(dev);
	unsigned int regval;

	int err = ina260_read_reg(dev, attr->index, &regval);

	if (err < 0)
		return err;

	return snprintf(buf, PAGE_SIZE, "%d\n",
			ina260_get_value(data, attr->index, regval));
}

static ssize_t ina260_interval_store(struct device *dev,
				     struct device_attribute *da,
				     const char *buf, size_t count)
{
	struct ina260_data *data = dev_get_drvdata(dev);
	unsigned long val;
	int status;

	status = kstrtoul(buf, 10, &val);
	if (status < 0)
		return status;

	if (val > INT_MAX || val == 0)
		return -EINVAL;

	status = regmap_update_bits(data->regmap, INA260_CONFIG,
				    INA260_AVG_RD_MASK,
				    ina260_interval_to_reg(val));
	if (status < 0)
		return status;

	return count;
}

static ssize_t ina260_interval_show(struct device *dev,
				    struct device_attribute *da, char *buf)
{
	struct ina260_data *data = dev_get_drvdata(dev);
	int status;
	unsigned int regval;

	status = regmap_read(data->regmap, INA260_CONFIG, &regval);
	if (status)
		return status;

	return snprintf(buf, PAGE_SIZE, "%d\n", ina260_reg_to_interval(regval));
}

/* calculated current */
static SENSOR_DEVICE_ATTR_RO(curr1_input, ina260_value, INA260_CURRENT);

/* calculated bus voltage */
static SENSOR_DEVICE_ATTR_RO(in1_input, ina260_value, INA260_BUS_VOLTAGE);

/* calculated power */
static SENSOR_DEVICE_ATTR_RO(power1_input, ina260_value, INA260_POWER);

/* update interval */
static SENSOR_DEVICE_ATTR_RW(update_interval, ina260_interval, 0);

/* pointers to created device attributes */
static struct attribute *ina260_attrs[] = {
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_power1_input.dev_attr.attr,
	&sensor_dev_attr_update_interval.dev_attr.attr,
	NULL,
};

static const struct attribute_group ina260_group = {
	.attrs = ina260_attrs,
};

static int ina260_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ina260_data *data;
	struct device *hwmon_dev;
	u32 val;
	int ret, group = 0;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	/* set fixed device attributes */
	data->config_default = INA260_CONFIG_DEFAULT;
	data->registers = INA260_REGISTERS;
	data->current_lsb_uA = INA260_CURRENT_LSB_uA;
	data->bus_voltage_shift = 0;
	data->bus_voltage_lsb_uV = INA260_BUS_VOLTAGE_LSB_uV;
	data->power_lsb_mW = INA260_POWER_LSB_mW;

	mutex_init(&data->config_lock);

	ina260_regmap_config.max_register = data->registers;

	data->regmap = devm_regmap_init_i2c(client, &ina260_regmap_config);
	if (IS_ERR(data->regmap)) {
		dev_err(dev, "failed to allocate register map\n");
		return PTR_ERR(data->regmap);
	}

	ret = ina260_init(data);
	if (ret < 0) {
		dev_err(dev, "error configuring the device: %d\n", ret);
		return -ENODEV;
	}

	data->groups[group++] = &ina260_group;

	hwmon_dev = devm_hwmon_device_register_with_groups(dev, client->name,
							   data, data->groups);
	if (IS_ERR(hwmon_dev))
		return PTR_ERR(hwmon_dev);

	dev_info(dev, "power monitor %s (integrated Rshunt = 2 mOhm)\n", client->name);

	return 0;
}

static const struct i2c_device_id ina260_id[] = {
	{ "ina260", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ina260_id);

static const struct of_device_id ina260_of_match[] = {
	{ .compatible = "ti,ina260" },
	{ },
};
MODULE_DEVICE_TABLE(of, ina260_of_match);

static struct i2c_driver ina260_driver = {
	.driver = {
		.name	= "ina260",
		.of_match_table = of_match_ptr(ina260_of_match),
	},
	.probe_new		= ina260_probe,
	.id_table	= ina260_id,
};

module_i2c_driver(ina260_driver);

MODULE_DESCRIPTION("ina260 driver");
MODULE_LICENSE("GPL");
