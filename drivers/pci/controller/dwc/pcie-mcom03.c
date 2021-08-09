// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for MCom-03
 *
 * Copyright 2021 RnD Center "ELVEES", JSC
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/resource.h>
#include <linux/types.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "pcie-designware.h"

#define SYS_CTRL_OFF			0x0
#define SYS_CTRL_APP_LTSSM_EN		BIT(4)
#define SYS_CTRL_OVRD_LTSSM_EN		BIT(31)
#define SYS_CTRL_DEVICE_TYPE_MASK	GENMASK(3, 0)

#define SYS_JESD_EN_OFF			0x300

#define DEVICE_TYPE_EP	0
#define DEVICE_TYPE_RC	4

#define SDR_PCI0_CTL		0x50
#define SDR_PCI1_CTL		0x54
#define PCIE_BTN_RSTN		BIT(1)

#define SDR_PCIE_PERSTN		0x150
#define SDR_PCI0_PERSTN_MODE	BIT(0)
#define SDR_PCI0_PERSTN		BIT(1)
#define SDR_PCI1_PERSTN_MODE	BIT(8)
#define SDR_PCI1_PERSTN		BIT(9)

#define to_mcom03_pcie(x)	dev_get_drvdata((x)->dev)

struct mcom03_pcie {
	struct dw_pcie			*pci;
	struct regmap			*sdr_base;
	void __iomem			*apb_base;
	u32				sdr_ctl_offset;
	int				id;
	struct reset_control		*reset;
};

static void mcom03_pcie_writel(struct mcom03_pcie *pcie, u32 reg, u32 val)
{
	writel(val, pcie->apb_base + reg);
}

static u32 mcom03_pcie_readl(struct mcom03_pcie *pcie, u32 reg)
{
	return readl(pcie->apb_base + reg);
}

static void mcom03_pcie_ltssm_toggle(struct mcom03_pcie *pcie, u32 val)
{
	u32 reg;

	reg = mcom03_pcie_readl(pcie, SYS_CTRL_OFF);
	reg &= ~SYS_CTRL_APP_LTSSM_EN;
	reg |= SYS_CTRL_OVRD_LTSSM_EN | (val ? SYS_CTRL_APP_LTSSM_EN : 0);
	mcom03_pcie_writel(pcie, SYS_CTRL_OFF, reg);
}

static int mcom03_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct mcom03_pcie *pcie = to_mcom03_pcie(pci);

	dw_pcie_setup_rc(pp);
	// If we need to program PHY, registers are programmed and
	// app_hold_phy_rst is unset here
	mcom03_pcie_ltssm_toggle(pcie, 1);
	dw_pcie_wait_for_link(pci);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		dw_pcie_msi_init(pp);

	return 0;
}

static void dw_plat_set_num_vectors(struct pcie_port *pp)
{
	pp->num_vectors = MAX_MSI_IRQS;
}

static const struct dw_pcie_host_ops mcom03_pcie_host_ops = {
	.host_init = mcom03_pcie_host_init,
	.set_num_vectors = dw_plat_set_num_vectors,
};

static void mcom03_pcie_set_dev_type(struct mcom03_pcie *pcie,
				     unsigned int device_type)
{
	u32 reg;

	reg = mcom03_pcie_readl(pcie, SYS_CTRL_OFF);
	reg &= ~SYS_CTRL_DEVICE_TYPE_MASK;
	mcom03_pcie_writel(pcie, SYS_CTRL_OFF, reg | device_type);
}

static void mcom03_pcie_set_jesd_en_zero(struct mcom03_pcie *pcie)
{
	mcom03_pcie_writel(pcie, SYS_JESD_EN_OFF, 0);
}

static void mcom03_pcie_btn_reset(struct mcom03_pcie *pcie)
{
	regmap_update_bits(pcie->sdr_base, pcie->sdr_ctl_offset,
			   PCIE_BTN_RSTN, 0);
	regmap_update_bits(pcie->sdr_base, pcie->sdr_ctl_offset,
			   PCIE_BTN_RSTN, PCIE_BTN_RSTN);
}

static void mcom03_pcie_unset_perst(struct mcom03_pcie *pcie)
{
	if (pcie->id == 0)
		regmap_update_bits(pcie->sdr_base, SDR_PCIE_PERSTN,
				   SDR_PCI0_PERSTN_MODE | SDR_PCI0_PERSTN,
				   SDR_PCI0_PERSTN);
	else
		regmap_update_bits(pcie->sdr_base, SDR_PCIE_PERSTN,
				   SDR_PCI1_PERSTN_MODE | SDR_PCI1_PERSTN,
				   SDR_PCI1_PERSTN);
}

static int mcom03_add_pcie_port(struct mcom03_pcie *pcie,
				struct platform_device *pdev)
{
	struct dw_pcie *pci = pcie->pci;
	struct pcie_port *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	int ret;

	// TODO: Add Legacy, Hotplug, LEQ, and other IRQ support
	pp->msi_irq = platform_get_irq(pdev, 0);
	if (pp->msi_irq < 0)
		return pp->msi_irq;

	pp->ops = &mcom03_pcie_host_ops;

	mcom03_pcie_unset_perst(pcie);
	mcom03_pcie_set_dev_type(pcie, DEVICE_TYPE_RC);
	mcom03_pcie_set_jesd_en_zero(pcie);
	mcom03_pcie_ltssm_toggle(pcie, 0);
	// If we need to program PHY, app_hold_phy_rst is asserted here
	mcom03_pcie_btn_reset(pcie);

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize PCIe host\n");
		return ret;
	}

	return 0;
}

static const struct dw_pcie_ops dw_pcie_ops = {
};

static int mcom03_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mcom03_pcie *pcie;
	struct dw_pcie *pci;
	struct resource *res;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie)
		return -ENOMEM;

	pci = devm_kzalloc(dev, sizeof(*pci), GFP_KERNEL);
	if (!pci)
		return -ENOMEM;

	pci->dev = dev;
	pci->ops = &dw_pcie_ops;

	pcie->pci = pci;

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	pci->dbi_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pci->dbi_base)) {
		dev_err(dev, "Failed to remap dbi memory\n");
		return PTR_ERR(pci->dbi_base);
	}

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	pcie->apb_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->apb_base)) {
		dev_err(dev, "Failed to remap dbi memory\n");
		return PTR_ERR(pcie->apb_base);
	}

	pcie->sdr_base = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "elvees,urb");
	if (IS_ERR(pcie->sdr_base)) {
		if (PTR_ERR(pcie->sdr_base) != -EPROBE_DEFER)
			dev_err(dev, "Failed to initialize property\n");
		return PTR_ERR(pcie->sdr_base);
	}

	ret = of_property_read_u32(dev->of_node, "elvees,ctrl-id", &pcie->id);
	if (ret) {
		dev_err(dev, "Not found elvees,ctrl-id property\n");
		return ret;
	}
	switch (pcie->id) {
	case 0:
		pcie->sdr_ctl_offset = SDR_PCI0_CTL;
		break;
	case 1:
		pcie->sdr_ctl_offset = SDR_PCI1_CTL;
		break;
	default:
		dev_err(dev, "Invalid elvees,ctrl-id %u\n", pcie->id);
		return -EINVAL;
	}

	pcie->reset = devm_reset_control_array_get_shared(dev);
	if (IS_ERR(pcie->reset)) {
		dev_err(dev, "Failed to get PCIe resets\n");
		return PTR_ERR(pcie->reset);
	}
	ret = reset_control_deassert(pcie->reset);
	if (ret) {
		dev_err(dev, "Failed to deassert PCIe resets\n");
		return ret;
	}

	platform_set_drvdata(pdev, pcie);

	ret = mcom03_add_pcie_port(pcie, pdev);
	if (ret)
		reset_control_assert(pcie->reset);

	return ret;
}

static int mcom03_pcie_remove(struct platform_device *pdev)
{
	struct mcom03_pcie *pcie = platform_get_drvdata(pdev);

	mcom03_pcie_ltssm_toggle(pcie, 0);
	reset_control_assert(pcie->reset);
	return 0;
}

static const struct of_device_id mcom03_pcie_of_match[] = {
	{ .compatible = "elvees,mcom03-pcie", },
	{ /* sentinel */ },
};

MODULE_DEVICE_TABLE(of, mcom03_pcie_of_match);

static struct platform_driver mcom03_pcie_driver = {
	.driver = {
		.name	= "mcom03-pcie",
		.of_match_table = mcom03_pcie_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = mcom03_pcie_probe,
	.remove = mcom03_pcie_remove,
};
module_platform_driver(mcom03_pcie_driver);

MODULE_AUTHOR("RnD Center ELVEES, JSC <support@elvees.com>");
MODULE_DESCRIPTION("MCom-03 PCIe host controller driver");
MODULE_LICENSE("GPL v2");
