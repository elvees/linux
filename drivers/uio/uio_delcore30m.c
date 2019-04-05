/*
 * Userspace I/O platform driver for Elvees DELCore-30m.
 *
 * Copyright 2016 ELVEES NeoTek JSC
 * Copyright 2008 Magnus Damm
 *
 * Based on uio_pdrv_genirq.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

#include <linux/platform_device.h>
#include <linux/uio_driver.h>
#include <linux/spinlock.h>
#include <linux/bitops.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/stringify.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>

#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>

#define DRIVER_NAME "uio-delcore-30m"

struct uio_delcore30m_platdata {
	struct uio_info *uioinfo;
	spinlock_t lock;
	unsigned long flags;
	struct platform_device *pdev;
	struct clk **clks;
	int clk_count;
};

/* Bits in uio_delcore30m_platdata.flags */
enum {
	UIO_IRQ_DISABLED = 0,
};

static int uio_delcore30m_open(struct uio_info *info, struct inode *inode)
{
	struct uio_delcore30m_platdata *priv = info->priv;

	/* Wait until the Runtime PM code has woken up the device */
	pm_runtime_get_sync(&priv->pdev->dev);
	return 0;
}

static int uio_delcore30m_release(struct uio_info *info, struct inode *inode)
{
	struct uio_delcore30m_platdata *priv = info->priv;

	/* Tell the Runtime PM code that the device has become idle */
	pm_runtime_put_sync(&priv->pdev->dev);
	return 0;
}

static irqreturn_t uio_delcore30m_handler(int irq, struct uio_info *dev_info)
{
	struct uio_delcore30m_platdata *priv = dev_info->priv;

	/* Just disable the interrupt in the interrupt controller, and
	 * remember the state so we can allow user space to enable it later.
	 */

	spin_lock(&priv->lock);
	if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
		disable_irq_nosync(irq);
	spin_unlock(&priv->lock);

	return IRQ_HANDLED;
}

static int uio_delcore30m_irqcontrol(struct uio_info *dev_info, s32 irq_on)
{
	struct uio_delcore30m_platdata *priv = dev_info->priv;
	unsigned long flags;

	/* Allow user space to enable and disable the interrupt
	 * in the interrupt controller, but keep track of the
	 * state to prevent per-irq depth damage.
	 *
	 * Serialize this operation to support multiple tasks and concurrency
	 * with irq handler on SMP systems.
	 */

	spin_lock_irqsave(&priv->lock, flags);
	if (irq_on) {
		if (__test_and_clear_bit(UIO_IRQ_DISABLED, &priv->flags))
			enable_irq(dev_info->irq);
	} else {
		if (!__test_and_set_bit(UIO_IRQ_DISABLED, &priv->flags))
			disable_irq_nosync(dev_info->irq);
	}
	spin_unlock_irqrestore(&priv->lock, flags);

	return 0;
}

static int uio_delcore30m_clk_init(struct uio_delcore30m_platdata *priv)
{
	struct device_node *np = priv->pdev->dev.of_node;
	struct clk *clock;
	int i, ret;

	priv->clk_count = of_clk_get_parent_count(np);
	if (priv->clk_count <= 0) {
		dev_err(&priv->pdev->dev, "clocks count is invalid (%d)\n",
			priv->clk_count);
		return -ENXIO;
	}

	priv->clks = kcalloc(priv->clk_count, sizeof(struct clk *), GFP_KERNEL);
	if (!priv->clks)
		return -ENOMEM;

	for (i = 0; i < priv->clk_count; i++) {
		clock = of_clk_get(np, i);
		if (IS_ERR(clock)) {
			if (PTR_ERR(clock) == -EPROBE_DEFER) {
				while (--i >= 0) {
					if (priv->clks[i])
						clk_put(priv->clks[i]);
				}
				kfree(priv->clks);
				return -EPROBE_DEFER;
			}
			dev_err(&priv->pdev->dev, "clock %d not found: %ld\n",
				i, PTR_ERR(clock));
			return -ENXIO;
		}
		priv->clks[i] = clock;
	}

	for (i = 0; i < priv->clk_count; i++) {
		ret = clk_prepare_enable(priv->clks[i]);
		if (ret) {
			dev_err(&priv->pdev->dev,
				"failed to enable clock %d\n", i);
			clk_put(priv->clks[i]);
			priv->clks[i] = NULL;
			return ret;
		}
	}

	return 0;
}

static void uio_delcore30m_clk_destroy(struct uio_delcore30m_platdata *priv)
{
	int i;

	if (!priv->clks)
		return;

	for (i = 0; i < priv->clk_count; i++) {
		if (priv->clks[i]) {
			clk_disable_unprepare(priv->clks[i]);
			clk_put(priv->clks[i]);
		}
	}

	kfree(priv->clks);
}

static int uio_delcore30m_probe(struct platform_device *pdev)
{
	struct uio_info *uioinfo = dev_get_platdata(&pdev->dev);
	struct uio_delcore30m_platdata *priv;
	struct uio_mem *uiomem;
	int ret = -EINVAL;
	int i;
	struct resource *resources;
	int num_resources;
	static const char * const xyram_names[] = { "xyram0", "xyram1" };

	if (pdev->dev.of_node) {
		/* alloc uioinfo for one device */
		uioinfo = devm_kzalloc(&pdev->dev, sizeof(*uioinfo),
				       GFP_KERNEL);
		if (!uioinfo)
			return -ENOMEM;

		uioinfo->name = pdev->dev.of_node->name;
		uioinfo->version = "0.1";
		/* Multiple IRQs are not supported */
	}

	if (!uioinfo || !uioinfo->name || !uioinfo->version) {
		dev_err(&pdev->dev, "missing platform_data\n");
		return ret;
	}

	if (uioinfo->handler || uioinfo->irqcontrol ||
	    uioinfo->irq_flags & IRQF_SHARED) {
		dev_err(&pdev->dev, "interrupt configuration error\n");
		return ret;
	}

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->uioinfo = uioinfo;
	spin_lock_init(&priv->lock);
	priv->flags = 0; /* interrupt is enabled to begin with */
	priv->pdev = pdev;

	ret = uio_delcore30m_clk_init(priv);
	if (ret) {
		dev_err(&pdev->dev, "clk init error\n");
		return ret;
	}

	if (!uioinfo->irq) {
		ret = platform_get_irq(pdev, 0);
		uioinfo->irq = ret;
		if (ret == -ENXIO && pdev->dev.of_node)
			uioinfo->irq = UIO_IRQ_NONE;
		else if (ret < 0) {
			dev_err(&pdev->dev, "failed to get IRQ\n");
			goto error_clocks;
		}
	}

	uiomem = &uioinfo->mem[0];

	num_resources = pdev->num_resources + 2;
	resources = kmalloc_array(num_resources, sizeof(struct resource),
				  GFP_KERNEL);
	if (!resources) {
		ret = -ENOMEM;
		goto error_clocks;
	}

	memcpy(resources, pdev->resource,
	       pdev->num_resources * sizeof(struct resource));

	for (i = 0; i < 2; ++i) {
		struct device_node *np;
		struct platform_device *pdev_xyram;
		struct resource *regs;

		np = of_parse_phandle(pdev->dev.of_node, "xyram", i);
		if (!np) {
			dev_err(&pdev->dev, "Parse XYRAM error\n");
			break;
		}

		pdev_xyram = of_find_device_by_node(np);
		if (!pdev_xyram) {
			dev_err(&pdev->dev, "Failed to find device by name\n");
			break;
		}

		regs = platform_get_resource(pdev_xyram, IORESOURCE_MEM, 0);
		memcpy(&resources[pdev->num_resources + i], regs,
		       sizeof(struct resource));
		resources[pdev->num_resources + i].name = xyram_names[i];
		of_node_put(np);
	}

	for (i = 0; i < num_resources; ++i) {
		struct resource *r = &resources[i];

		if (!strcmp(r->name, "spinlock"))
			continue;

		if (r->flags != IORESOURCE_MEM)
			continue;

		if (uiomem >= &uioinfo->mem[MAX_UIO_MAPS]) {
			dev_warn(&pdev->dev, "device has more than "
					__stringify(MAX_UIO_MAPS)
					" I/O memory resources.\n");
			break;
		}

		uiomem->memtype = UIO_MEM_PHYS;
		uiomem->addr = r->start;
		uiomem->size = resource_size(r);
		uiomem->name = r->name;
		++uiomem;
	}

	kfree(resources);

	while (uiomem < &uioinfo->mem[MAX_UIO_MAPS]) {
		uiomem->size = 0;
		++uiomem;
	}

	/* This driver requires no hardware specific kernel code to handle
	 * interrupts. Instead, the interrupt handler simply disables the
	 * interrupt in the interrupt controller. User space is responsible
	 * for performing hardware specific acknowledge and re-enabling of
	 * the interrupt in the interrupt controller.
	 *
	 * Interrupt sharing is not supported.
	 */

	uioinfo->handler = uio_delcore30m_handler;
	uioinfo->irqcontrol = uio_delcore30m_irqcontrol;
	uioinfo->open = uio_delcore30m_open;
	uioinfo->release = uio_delcore30m_release;
	uioinfo->priv = priv;

	/* Enable Runtime PM for this device:
	 * The device starts in suspended state to allow the hardware to be
	 * turned off by default. The Runtime PM bus code should power on the
	 * hardware and enable clocks at open().
	 */
	pm_runtime_enable(&pdev->dev);

	ret = uio_register_device(&pdev->dev, priv->uioinfo);
	if (ret) {
		dev_err(&pdev->dev, "unable to register uio device\n");
		pm_runtime_disable(&pdev->dev);
		goto error_clocks;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

error_clocks:
	uio_delcore30m_clk_destroy(priv);
	return ret;
}

static int uio_delcore30m_remove(struct platform_device *pdev)
{
	struct uio_delcore30m_platdata *priv = platform_get_drvdata(pdev);

	uio_unregister_device(priv->uioinfo);
	pm_runtime_disable(&pdev->dev);

	priv->uioinfo->handler = NULL;
	priv->uioinfo->irqcontrol = NULL;

	uio_delcore30m_clk_destroy(priv);

	return 0;
}

static int uio_delcore30m_runtime_nop(struct device *dev)
{
	/* Runtime PM callback shared between ->runtime_suspend()
	 * and ->runtime_resume(). Simply returns success.
	 *
	 * In this driver pm_runtime_get_sync() and pm_runtime_put_sync()
	 * are used at open() and release() time. This allows the
	 * Runtime PM code to turn off power to the device while the
	 * device is unused, ie before open() and after release().
	 *
	 * This Runtime PM callback does not need to save or restore
	 * any registers since user space is responsbile for hardware
	 * register reinitialization after open().
	 */
	return 0;
}

static const struct dev_pm_ops uio_delcore30m_dev_pm_ops = {
	.runtime_suspend = uio_delcore30m_runtime_nop,
	.runtime_resume = uio_delcore30m_runtime_nop,
};

#ifdef CONFIG_OF
static const struct of_device_id uio_of_delcore30m_match[] = {
	{ .compatible = "elvees,delcore30m" },
	{ /* Sentinel */ },
};
MODULE_DEVICE_TABLE(of, uio_of_delcore30m_match);
#endif

static struct platform_driver uio_delcore30m = {
	.probe = uio_delcore30m_probe,
	.remove = uio_delcore30m_remove,
	.driver = {
		.name = DRIVER_NAME,
		.pm = &uio_delcore30m_dev_pm_ops,
		.of_match_table = of_match_ptr(uio_of_delcore30m_match),
	},
};

module_platform_driver(uio_delcore30m);

MODULE_AUTHOR("Dmitriy Zagrebin");
MODULE_DESCRIPTION("Userspace I/O platform driver for Elvees DELCore-30m");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:" DRIVER_NAME);
