// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright 2020-2023 RnD Center "ELVEES", JSC
 */
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_irq.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#include "irq-elvees-qlic.h"

#define QLIC_NR_IRQS 128
#define QLIC_PRIO 2

struct qlic_priv {
	struct irq_domain *domain;
	void __iomem *reg_base;
	u32 target_map[QLIC_NR_IRQS];
	u32 target_list[QLIC_MAX_TARGET];
	u32 ntargets;
	u32 reset_targets_mask;
	u32 current_target;
	spinlock_t lock;
	struct platform_device *pdev;
	int base_irq;
	unsigned int *irqs;
	int nirqs;
};

struct irq_handler_data {
	struct qlic_priv *priv;
	int target;
};

static inline u32 qlic_read(struct qlic_priv *priv, unsigned int const reg)
{
	return ioread32(priv->reg_base + reg);
}

static inline void qlic_write(u32 const value, struct qlic_priv *priv,
			      unsigned int const reg)
{
	iowrite32(value, priv->reg_base + reg);
}

/*
 * Handler for the cascaded IRQ.
 */
static void qlic_handle_irq(struct irq_desc *desc)
{
	struct irq_handler_data *handler_priv = irq_desc_get_handler_data(desc);
	struct qlic_priv *priv = handler_priv->priv;
	struct irq_chip *chip = irq_desc_get_chip(desc);
	int target = handler_priv->target;
	int hwirq, virq;
	u32 qlic_offset;

	chained_irq_enter(chip, desc);

	qlic_offset = QLIC_CC0 + QLIC_CCNEXT * target;
	while (true) {
		// get the most priority interrupt for current target
		hwirq = qlic_read(priv, qlic_offset);
		if (hwirq == 0)
			break;

		virq = irq_find_mapping(priv->domain, hwirq);
		if (virq > 0)
			generic_handle_irq(virq);

		qlic_write(hwirq, priv, qlic_offset); // reset
	}

	chained_irq_exit(chip, desc);
}

static u32 qlic_get_next_target(struct qlic_priv *priv)
{
	u32 target = priv->target_list[priv->current_target];

	priv->current_target += 1;
	if (priv->current_target >= priv->ntargets)
		priv->current_target = 0;

	return target;
}

static void qlic_irq_enable(struct irq_data *data)
{
	struct qlic_priv *priv = data->chip_data;
	int hwirq = data->hwirq;
	int target = qlic_get_next_target(priv);
	int temp;
	unsigned long flags;

	priv->target_map[hwirq] = target;

	qlic_write(QLIC_PRIO, priv, QLIC_PRI0 + hwirq * QLIC_PRI_NEXT);

	spin_lock_irqsave(&priv->lock, flags);
	// Enable target for hwirq
	temp = qlic_read(priv, QLIC_ENS0 + QLIC_ENSNEXT * target +
			 hwirq / 32 * 4);
	temp |= BIT(hwirq % 32);
	qlic_write(temp, priv, QLIC_ENS0 + QLIC_ENSNEXT * target +
					hwirq / 32 * 4);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void qlic_irq_disable(struct irq_data *data)
{
	struct qlic_priv *priv = data->chip_data;
	int hwirq = data->hwirq;
	int target = priv->target_map[hwirq];
	int temp;
	unsigned long flags;

	qlic_write(0, priv, QLIC_PRI0 + hwirq * QLIC_PRI_NEXT);

	spin_lock_irqsave(&priv->lock, flags);
	//Disable target for hwirq
	temp = qlic_read(priv, QLIC_ENS0 + QLIC_ENSNEXT * target +
			 hwirq / 32 * 4);
	temp &= ~BIT(hwirq % 32);
	qlic_write(temp, priv, QLIC_ENS0 + QLIC_ENSNEXT * target +
					hwirq / 32 * 4);
	spin_unlock_irqrestore(&priv->lock, flags);
}

static void qlic_irq_mask(struct irq_data *data)
{
	(void) data;
}

static void qlic_irq_unmask(struct irq_data *data)
{
	(void) data;
}

static struct irq_chip qlic_irq_chip = {
	.name = "QLIC",
	.irq_enable = qlic_irq_enable,
	.irq_disable = qlic_irq_disable,
	.irq_mask = qlic_irq_mask,
	.irq_unmask = qlic_irq_unmask,
};

static int qlic_domain_translate(struct irq_domain *domain,
				 struct irq_fwspec *fwspec,
				 unsigned long *out_hwirq,
				 unsigned int *out_type)
{
	if (is_of_node(fwspec->fwnode)) {
		*out_hwirq = fwspec->param[0];
		if (fwspec->param[0] > QLIC_NR_IRQS)
			return -EINVAL;

		*out_type = IRQ_TYPE_LEVEL_HIGH;
	}
	return 0;
}

static int qlic_irq_map(struct irq_domain *domain, unsigned int irq,
			irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &qlic_irq_chip, handle_level_irq);
	irq_set_chip_data(irq, domain->host_data);
	irq_set_noprobe(irq);
	return 0;
}

static void qlic_irq_unmap(struct irq_domain *domain, unsigned int irq)
{
	irq_domain_reset_irq_data(irq_get_irq_data(irq));
}

static const struct irq_domain_ops qlic_domain_ops = {
	.map = qlic_irq_map,
	.unmap = qlic_irq_unmap,
	.translate = qlic_domain_translate
};

static void __init qlic_hwreset(struct qlic_priv *priv)
{
	int hwirq, target;

	dev_info(&priv->pdev->dev, "Reset targets according to mask 0x%x\n",
		 priv->reset_targets_mask);
	for (target = 0; target < QLIC_MAX_TARGET; ++target) {
		if (!(BIT(target) & priv->reset_targets_mask))
			continue;

		qlic_write(0, priv, QLIC_THD0 + target * QLIC_THDNEXT);
		for (hwirq = 0; hwirq < QLIC_NR_IRQS; ++hwirq)
			qlic_write(0, priv,
				   QLIC_ENS0 + QLIC_ENSNEXT * target +
					   hwirq / 32 * 4);
	}
}

static int __init fill_targets(struct qlic_priv *priv, struct device_node *np)
{
	priv->ntargets = of_property_count_elems_of_size(np, "targets",
							 sizeof(u32));
	if (priv->ntargets == 0)
		return -EINVAL;

	of_property_read_variable_u32_array(np, "targets",
					    priv->target_list,
					    priv->ntargets, 0);
	return 0;
}

static void qlic_free_irqs(struct qlic_priv *priv)
{
	int i;

	for (i = 0; i < priv->nirqs; i++) {
		if (!priv->irqs[i])
			break;
		irq_set_chained_handler_and_data(priv->irqs[i], NULL, NULL);
	}

	devm_kfree(&priv->pdev->dev, priv->irqs);
}

static int qlic_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct device *dev = &pdev->dev;
	struct qlic_priv *priv;
	struct irq_handler_data *handler_priv;
	int i, ret;

	priv = devm_kzalloc(dev, sizeof(struct qlic_priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	priv->pdev = pdev;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	priv->reg_base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->reg_base))
		return PTR_ERR(priv->reg_base);

	ret = of_property_read_u32(pdev->dev.of_node, "reset-targets-mask",
				   &priv->reset_targets_mask);
	if (ret == -EINVAL) {
		/*
		 * If reset-targets-mask isn't present in Devicetree,
		 * set it's parameter to max value and reset all targets.
		 */
		priv->reset_targets_mask = BIT(QLIC_MAX_TARGET) - 1;
	} else if (ret) {
		/* If reset-targets-mask has mistakes, return error */
		dev_err(&pdev->dev, "Failed to get reset-targets-mask.");
		return ret;
	}

	if (priv->reset_targets_mask)
		qlic_hwreset(priv);

	ret = fill_targets(priv, dev->of_node);
	if (ret) {
		dev_err(&pdev->dev, "Failed to get targets.");
		return ret;
	}

	priv->current_target = 0;

	spin_lock_init(&priv->lock);

	priv->domain = irq_domain_create_linear(pdev->dev.fwnode, QLIC_NR_IRQS,
						&qlic_domain_ops, priv);
	if (!priv->domain) {
		/* Errors printed by irq_domain_create_linear */
		dev_err(&pdev->dev, "Failed to create irq domain.");
		return -ENODEV;
	}

	priv->nirqs = of_property_count_elems_of_size(pdev->dev.of_node,
						      "interrupts",
						      sizeof(int));
	priv->nirqs /= QLIC_INTERRUPT_PROPERTIES;
	if (priv->nirqs != priv->ntargets) {
		dev_err(&pdev->dev, "Invalid irq number, %d\n", priv->nirqs);
		return -EINVAL;
	}

	priv->irqs = devm_kcalloc(&pdev->dev, priv->nirqs,
				  sizeof(unsigned int),
				  GFP_KERNEL | __GFP_ZERO);
	if (!priv->irqs) {
		dev_err(&pdev->dev, "Failed to allocate irqs.");
		return -ENOMEM;
	}

	for (i = 0; i < priv->nirqs; ++i) {
		priv->irqs[i] = platform_get_irq(pdev, i);
		if (priv->irqs[i] <= 0) {
			ret = -EPROBE_DEFER;
			goto irq_err;
		}
		if (i == 0)
			priv->base_irq = priv->irqs[i];

		handler_priv = devm_kzalloc(&pdev->dev,
					    sizeof(struct irq_handler_data),
					    GFP_KERNEL);
		if (!handler_priv) {
			ret = -ENOMEM;
			goto irq_err;
		}
		handler_priv->target = priv->target_list[i];
		handler_priv->priv = priv;
		irq_set_chained_handler_and_data(priv->irqs[i],
						 qlic_handle_irq,
						 handler_priv);
	}

	platform_set_drvdata(pdev, priv);

	dev_info(&pdev->dev, "Initialized successfully\n");

	return 0;
irq_err:
	qlic_free_irqs(priv);
	return ret;
}

static int qlic_remove(struct platform_device *pdev)
{
	struct qlic_priv *priv = platform_get_drvdata(pdev);

	qlic_free_irqs(priv);

	irq_domain_remove(priv->domain);
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id qlic_dt_ids[] = {
	{ .compatible = "elvees,qlic" },
	{}
};
MODULE_DEVICE_TABLE(of, qlic_dt_ids);
#endif

static struct platform_driver qlic_plat_driver = {
	.driver = {
		.name = KBUILD_MODNAME,
		.of_match_table = of_match_ptr(qlic_dt_ids),
	},
	.probe = qlic_probe,
	.remove = qlic_remove
};

module_platform_driver(qlic_plat_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("ELVEES QLIC driver");
