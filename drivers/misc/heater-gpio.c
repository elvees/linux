// SPDX-License-Identifier: GPL-2.0-or-later

// Copyright 2025 RnD Center "ELVEES", JSC

#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/miscdevice.h>
#include <linux/of.h>
#include <linux/platform_device.h>

struct heater_gpio {
	struct device *dev;
	struct miscdevice mdev;
	struct gpio_desc *gpiod;
	bool enable;
};

#define to_heater(x)	container_of((x), struct heater_gpio, mdev)

static int heater_gpio_set_state(struct heater_gpio *heater, int state)
{
	gpiod_set_value(heater->gpiod, state);
	heater->enable = state;

	return 0;
}

static ssize_t
heater_gpio_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct miscdevice *mdev = file->private_data;
	struct heater_gpio *heater = to_heater(mdev);
	uint32_t state;
	int ret = sizeof(uint32_t);

	state = heater->enable ? 1 : 0;

	if (copy_to_user(buf, &state, ret))
		return -EFAULT;

	return ret;
}

static ssize_t
heater_gpio_write(struct file *file, const char __user *buf, size_t count,
		  loff_t *ppos)
{
	struct miscdevice *mdev = file->private_data;
	struct heater_gpio *heater = to_heater(mdev);
	uint32_t state;

	if (count != sizeof(uint32_t))
		return -EIO;

	if (copy_from_user(&state, buf, sizeof(uint32_t)))
		return -EFAULT;

	if (!state)
		heater_gpio_set_state(heater, 0);
	else
		heater_gpio_set_state(heater, 1);

	return count;
}

static int
heater_gpio_open(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct heater_gpio *heater = to_heater(mdev);

	return 0;
}

static int
heater_gpio_release(struct inode *inode, struct file *file)
{
	struct miscdevice *mdev = file->private_data;
	struct heater_gpio *heater = to_heater(mdev);

	heater_gpio_set_state(heater, 0);

	return 0;
}

static const struct file_operations heater_gpio_fops = {
	.owner = THIS_MODULE,
	.read = heater_gpio_read,
	.write = heater_gpio_write,
	.open = heater_gpio_open,
	.release = heater_gpio_release,
};

static int heater_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct heater_gpio *heater;
	int ret;

	heater = devm_kzalloc(dev, sizeof(struct heater_gpio), GFP_KERNEL);
	if (!heater)
		return -ENOMEM;

	heater->dev = dev;

	heater->gpiod = devm_gpiod_get(dev, NULL, GPIOD_OUT_LOW);
	if (IS_ERR(heater->gpiod)) {
		ret = PTR_ERR(heater->gpiod);
		dev_err(dev, "Failed to request heater GPIO: %d\n", ret);
		return ret;
	}

	heater->enable = gpiod_get_value_cansleep(heater->gpiod);

	heater->mdev.minor = MISC_DYNAMIC_MINOR;
	heater->mdev.name = "heater-gpio";
	heater->mdev.fops = &heater_gpio_fops;
	heater->mdev.parent = dev;

	ret = misc_register(&heater->mdev);
	if (ret) {
		dev_err(dev, "Failed to register misc device\n");
		return ret;
	}

	platform_set_drvdata(pdev, heater);

	return 0;
}

static int heater_gpio_remove(struct platform_device *pdev)
{
	struct heater_gpio *heater = platform_get_drvdata(pdev);

	heater_gpio_set_state(heater, 0);
	misc_deregister(&heater->mdev);

	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id heater_gpio_match[] = {
	{ .compatible = "heater-gpio", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, heater_gpio_match);
#endif

static struct platform_driver heater_gpio_driver = {
	.probe = heater_gpio_probe,
	.remove = heater_gpio_remove,
	.driver = {
		.name = "heater-gpio",
		.of_match_table = of_match_ptr(heater_gpio_match),
	},
};

module_platform_driver(heater_gpio_driver);

MODULE_DESCRIPTION("Driver for heater controlled via GPIO");
MODULE_LICENSE("GPL");
