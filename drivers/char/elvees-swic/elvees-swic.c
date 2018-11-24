/*
 * Copyright 2018 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/cdev.h>
#include <linux/clk.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/of_device.h>

#define ELVEES_SWIC_MAX_DEVICES		2

static u32 swic_major;

static DECLARE_BITMAP(swic_dev, ELVEES_SWIC_MAX_DEVICES);

static struct class *swic_class;

struct elvees_swic_private_data {
	void __iomem *regs;

	struct clk *aclk;
	struct clk *txclk;

	struct device *dev;
	struct cdev cdev;
};

static const struct file_operations elvees_swic_fops = {
	.owner = THIS_MODULE,
};

static int elvees_swic_dev_register(struct elvees_swic_private_data *pdata)
{
	struct device *dev;
	int minor, ret;

	static DEFINE_SPINLOCK(lock);

	/* Find a free minor number */
	spin_lock(&lock);

	minor = find_first_zero_bit(swic_dev, ELVEES_SWIC_MAX_DEVICES);
	if (minor == ELVEES_SWIC_MAX_DEVICES) {
		spin_unlock(&lock);
		dev_err(pdata->dev, "Failed to get free minor\n");
		return -ENFILE;
	}

	set_bit(minor, swic_dev);

	spin_unlock(&lock);

	cdev_init(&pdata->cdev, &elvees_swic_fops);
	pdata->cdev.owner = THIS_MODULE;
	ret = cdev_add(&pdata->cdev, MKDEV(swic_major, minor), 1);
	if (ret) {
		dev_err(pdata->dev, "Failed to add cdev\n");
		goto free_minor;
	}

	dev = device_create(swic_class, pdata->dev, MKDEV(swic_major, minor),
			    NULL, "spacewire%d", minor);
	if (IS_ERR(dev)) {
		dev_err(pdata->dev, "Failed to create files\n");
		ret = PTR_ERR(dev);
		goto cdev_free;
	}
	return 0;

cdev_free:
	cdev_del(&pdata->cdev);

free_minor:
	clear_bit(minor, swic_dev);
	return ret;
}

static void elvees_swic_dev_unregister(struct elvees_swic_private_data *pdata)
{
	int minor = MINOR(pdata->cdev.dev);

	device_destroy(swic_class, MKDEV(swic_major, minor));

	cdev_del(&pdata->cdev);

	clear_bit(minor, swic_dev);
}

static irqreturn_t elvees_swic_dma_rx_desc_ih(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_dma_rx_data_ih(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_dma_tx_desc_ih(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_dma_tx_data_ih(int irq, void *data)
{
	return IRQ_HANDLED;
}

static irqreturn_t elvees_swic_ih(int irq, void *data)
{
	return IRQ_HANDLED;
}

static int elvees_swic_irq_request(struct platform_device *pdev,
				   struct elvees_swic_private_data *pdata)
{
	int irq, ret, i;

	struct {
		irqreturn_t (*handler)(int, void*);
		const char *name;
	} descs[] = {
		{ elvees_swic_dma_rx_desc_ih, "dma_rx_desc_irq" },
		{ elvees_swic_dma_rx_data_ih, "dma_rx_data_irq" },
		{ elvees_swic_dma_tx_desc_ih, "dma_tx_desc_irq" },
		{ elvees_swic_dma_tx_data_ih, "dma_tx_data_irq" },
		{ elvees_swic_ih, "connected_irq" }
	};

	for (i = 0; i < ARRAY_SIZE(descs); i++) {
		irq = platform_get_irq(pdev, i);
		ret = devm_request_irq(&pdev->dev, irq, descs[i].handler,
				       0, descs[i].name, pdata);
		if (ret) {
			dev_err(&pdev->dev, "Failed to get interrupt resource\n");
			return ret;
		}
	}

	return 0;
}

static int elvees_swic_probe(struct platform_device *pdev)
{
	struct elvees_swic_private_data *pdata;
	struct resource *regs;
	int ret;

	regs = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!regs) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENOENT;
	}

	pdata = devm_kzalloc(&pdev->dev,
			     sizeof(struct elvees_swic_private_data),
			     GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = &pdev->dev;

	pdata->txclk = devm_clk_get(&pdev->dev, "txclk");
	if (IS_ERR(pdata->txclk)) {
		dev_err(&pdev->dev, "Failed to found txclk\n");
		return PTR_ERR(pdata->txclk);
	}

	pdata->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(pdata->aclk)) {
		dev_err(&pdev->dev, "Failed to found aclk\n");
		return PTR_ERR(pdata->aclk);
	}

	ret = clk_prepare_enable(pdata->txclk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable txclk\n");
		return ret;
	}

	ret = clk_prepare_enable(pdata->aclk);
	if (ret) {
		dev_err(&pdev->dev, "Failed to enable aclk\n");
		goto disable_txclk;
	}

	pdata->regs = devm_ioremap_resource(&pdev->dev, regs);
	if (IS_ERR(pdata->regs)) {
		dev_err(&pdev->dev, "Failed to remap mem resource\n");
		goto disable_aclk;
	}

	ret = elvees_swic_irq_request(pdev, pdata);
	if (ret)
		goto disable_aclk;

	ret = elvees_swic_dev_register(pdata);
	if (ret)
		goto disable_aclk;

	platform_set_drvdata(pdev, pdata);

	dev_info(&pdev->dev, "ELVEES SWIC @ 0x%p\n", pdata->regs);

	return 0;

disable_aclk:
	clk_disable_unprepare(pdata->aclk);

disable_txclk:
	clk_disable_unprepare(pdata->txclk);
	return ret;
}

static int elvees_swic_remove(struct platform_device *pdev)
{
	struct elvees_swic_private_data *pdata = platform_get_drvdata(pdev);

	clk_disable_unprepare(pdata->txclk);
	clk_disable_unprepare(pdata->aclk);

	elvees_swic_dev_unregister(pdata);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id elvees_swic_ids[] = {
	{ .compatible = "elvees,mcom02-swic" },
	{}
};
MODULE_DEVICE_TABLE(of, elvees_swic_ids);
#endif

static struct platform_driver elvees_swic_driver = {
	.probe = elvees_swic_probe,
	.remove = elvees_swic_remove,
	.driver = {
		.name = "elvees-swic",
		.of_match_table = of_match_ptr(elvees_swic_ids)
	}
};

static int __init elvees_swic_module_init(void)
{
	int ret;
	dev_t dev;

	swic_class = class_create(THIS_MODULE, "spacewire");
	if (IS_ERR(swic_class)) {
		ret = PTR_ERR(swic_class);
		goto out;
	}

	ret = alloc_chrdev_region(&dev, 0, ELVEES_SWIC_MAX_DEVICES,
				  "elvees-swic");
	if (ret)
		goto class_destroy;

	swic_major = MAJOR(dev);
	ret = platform_driver_register(&elvees_swic_driver);
	if (ret)
		goto chr_remove;

	return 0;

chr_remove:
	unregister_chrdev_region(dev, ELVEES_SWIC_MAX_DEVICES);

class_destroy:
	class_destroy(swic_class);

out:
	return ret;
}

static void __exit elvees_swic_module_exit(void)
{
	platform_driver_unregister(&elvees_swic_driver);
	unregister_chrdev_region(MKDEV(swic_major, 0), ELVEES_SWIC_MAX_DEVICES);
	class_destroy(swic_class);
}

module_init(elvees_swic_module_init);
module_exit(elvees_swic_module_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ELVEES SWIC driver");
