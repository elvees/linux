// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for MCom-03
 *
 * Copyright 2021-2024 RnD Center "ELVEES", JSC
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

#define EDMA_INTx_INT			0x10
#define EDMA_INTx_INT_ASSERTED		GENMASK(3, 0)
#define EDMA_INTx_INT_DEASSERTED	GENMASK(7, 4)

#define DEBUG_ST_OFF			0x34
#define DEBUG_ST_RDLH_LINK_UP		BIT(0)
#define DEBUG_ST_SMLH_LINK_UP		BIT(31)

#define SYS_JESD_EN_OFF			0x300

#define DEVICE_TYPE_EP	0
#define DEVICE_TYPE_RC	4

#define SDR_PCIE_PERSTN		0x150
#define SDR_PCI0_PERSTN_MODE	BIT(0)
#define SDR_PCI0_PERSTN		BIT(1)
#define SDR_PCI1_PERSTN_MODE	BIT(8)
#define SDR_PCI1_PERSTN		BIT(9)

// PHY viewport access
#define PHY_VIEWPORT_CTLSTS_OFF	0xb70
#define PHY_VIEWPORT_CTLSTS_ADDR	GENMASK(15, 0)
#define PHY_VIEWPORT_CTLSTS_NUM		GENMASK(19, 16)
#define PHY_VIEWPORT_CTLSTS_READ	BIT(20)
#define PHY_VIEWPORT_CTLSTS_BCWR	BIT(21)
#define PHY_VIEWPORT_CTLSTS_STATUS	BIT(30)
#define PHY_VIEWPORT_CTLSTS_PENDING	BIT(31)
#define PHY_VIEWPORT_DATA_OFF		0xb74

#define to_mcom03_pcie(x)	dev_get_drvdata((x)->dev)

struct mcom03_pcie {
	struct dw_pcie			*pci;
	struct regmap			*sdr_base;
	void __iomem			*apb_base;
	int				id;
	struct reset_control		*reset;
	struct irq_domain		*irq_domain;
	struct gpio_desc		*reset_gpio;
	bool				embed_msi:1;
};

#if defined(CONFIG_PCIE_MCOM03_DEBUG)
#define MCOM03_PCIE_PHY_REG(name, addr) { \
	.reg_name = name, \
	.offset = addr, \
}

struct phy_reg {
	const char reg_name[60];
	u16 offset;
	u16 val;
};

static const struct phy_reg phy_regs_dump[] = {
	MCOM03_PCIE_PHY_REG("SUP_DIG_IDCODE_LO", 0x0),
	MCOM03_PCIE_PHY_REG("SUP_DIG_IDCODE_HI", 0x1),
	MCOM03_PCIE_PHY_REG("LANE0_DIG_ASIC_RX_ASIC_IN_1", 0x1012),
	MCOM03_PCIE_PHY_REG("LANE1_DIG_ASIC_RX_ASIC_IN_1", 0x1112),
	MCOM03_PCIE_PHY_REG("LANE2_DIG_ASIC_RX_ASIC_IN_1", 0x1212),
	MCOM03_PCIE_PHY_REG("LANE3_DIG_ASIC_RX_ASIC_IN_1", 0x1312),
};

static int mcom03_pcie_phy_read(struct dw_pcie *pci, u16 reg, u16 *val)
{
	u32 tmp, retry = 0;

	// Details in PCIe DM controller databook $3.5.6, $4.13
	dw_pcie_writel_dbi(pci, PHY_VIEWPORT_CTLSTS_OFF,
			   reg | PHY_VIEWPORT_CTLSTS_READ);

	while (retry < 3) {
		retry++;
		tmp = dw_pcie_readl_dbi(pci, PHY_VIEWPORT_CTLSTS_OFF);

		if (tmp & PHY_VIEWPORT_CTLSTS_PENDING)
			usleep_range(1, 10);
		else
			break;
	}

	if ((tmp & PHY_VIEWPORT_CTLSTS_PENDING) ||
	    (tmp & PHY_VIEWPORT_CTLSTS_STATUS))
		return -ETIMEDOUT;

	*val = dw_pcie_readw_dbi(pci, PHY_VIEWPORT_DATA_OFF);

	return 0;
};

static void mcom03_pcie_dump_phy_regs(struct dw_pcie *pci)
{
	int i, ret;
	u16 val;

	dev_info(pci->dev, "PHY registers dump begin:\n");
	for (i = 0; i < ARRAY_SIZE(phy_regs_dump); i++) {
		ret = mcom03_pcie_phy_read(pci, phy_regs_dump[i].offset, &val);
		if (!ret)
			dev_info(pci->dev, "%s[%#x]:%#x\n",
				 phy_regs_dump[i].reg_name, phy_regs_dump[i].offset, val);
	}
	dev_info(pci->dev, "PHY registers dump end\n");
}
#else
static void mcom03_pcie_dump_phy_regs(struct dw_pcie *pci) { }
#endif

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

static void mcom03_pcie_unset_perst(struct mcom03_pcie *pcie)
{
	if (pcie->reset_gpio) {
		/* "Power Sequencing and Reset Signal Timings" table in
		 * PCI EXPRESS CARD ELECTROMECHANICAL SPECIFICATION, REV. 3.0
		 * indicates PERST# should be deasserted after minimum of 100us
		 * once REFCLK is stable. */
		usleep_range(100, 200);
		gpiod_set_value_cansleep(pcie->reset_gpio, 1);
		usleep_range(1000, 1500);
	}

	if (pcie->id == 0)
		regmap_update_bits(pcie->sdr_base, SDR_PCIE_PERSTN,
				   SDR_PCI0_PERSTN_MODE | SDR_PCI0_PERSTN,
				   SDR_PCI0_PERSTN);
	else
		regmap_update_bits(pcie->sdr_base, SDR_PCIE_PERSTN,
				   SDR_PCI1_PERSTN_MODE | SDR_PCI1_PERSTN,
				   SDR_PCI1_PERSTN);
}

static int mcom03_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct mcom03_pcie *pcie = to_mcom03_pcie(pci);
	int ret;

	ret = reset_control_deassert(pcie->reset);
	if (ret) {
		dev_err(dev, "Failed to deassert PCIe resets\n");
		return ret;
	}

	mcom03_pcie_unset_perst(pcie);
	mcom03_pcie_set_dev_type(pcie, DEVICE_TYPE_RC);
	mcom03_pcie_set_jesd_en_zero(pcie);

	// Set BAR0/BAR1 to 4 KiB to preserve space in ranges
	dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_0, 0xFFF);
	dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_1, 0xFFF);
	// Disable DBI_RO_WR_EN, since setup_rc() expects it to be off
	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static void mcom03_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct mcom03_pcie *pcie = to_mcom03_pcie(pci);

	reset_control_assert(pcie->reset);
}

static int mcom03_pcie_msi_host_init(struct dw_pcie_rp *pp)
{
	return 0;
}

static const struct dw_pcie_host_ops mcom03_pcie_host_ops = {
	.host_init = mcom03_pcie_host_init,
	.host_deinit = mcom03_pcie_host_deinit,
	.msi_host_init = mcom03_pcie_msi_host_init,
};

static const struct dw_pcie_host_ops mcom03_pcie_host_ops_embed = {
	.host_init = mcom03_pcie_host_init,
	.host_deinit = mcom03_pcie_host_deinit,
};

static void mcom03_pcie_legacy_irq_handler(struct irq_desc *desc)
{
	struct mcom03_pcie *pcie = irq_desc_get_handler_data(desc);
	struct irq_chip *chip = irq_desc_get_chip(desc);
	unsigned long reg;
	u32 virq, bit;

	chained_irq_enter(chip, desc);
	reg = mcom03_pcie_readl(pcie, EDMA_INTx_INT);

	if (reg & EDMA_INTx_INT_ASSERTED) {
		mcom03_pcie_writel(pcie, EDMA_INTx_INT, reg & EDMA_INTx_INT_ASSERTED);

		for_each_set_bit(bit, &reg, PCI_NUM_INTX) {
			virq = irq_find_mapping(pcie->irq_domain, bit);
			if (virq)
				generic_handle_irq(virq);
		}
	}

	if (reg & EDMA_INTx_INT_DEASSERTED)
		mcom03_pcie_writel(pcie, EDMA_INTx_INT, reg & EDMA_INTx_INT_DEASSERTED);

	chained_irq_exit(chip, desc);
}

static int mcom03_pcie_intx_map(struct irq_domain *domain, unsigned int irq,
				irq_hw_number_t hwirq)
{
	irq_set_chip_and_handler(irq, &dummy_irq_chip, handle_simple_irq);
	irq_set_chip_data(irq, domain->host_data);
	return 0;
}

static const struct irq_domain_ops intx_domain_ops = {
	.map = mcom03_pcie_intx_map,
	.xlate = pci_irqd_intx_xlate,
};

static int mcom03_pcie_config_legacy_irq(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct device *dev = pci->dev;
	struct mcom03_pcie *mcom03 = to_mcom03_pcie(pci);
	struct device_node *node = dev->of_node;
	struct device_node *intc;

	intc = of_get_child_by_name(node, "legacy-interrupt-controller");
	if (!intc) {
		dev_err(dev, "No 'legacy-interrupt-controller' node found\n");
		return -ENODEV;
	}

	mcom03->irq_domain = irq_domain_add_linear(intc, PCI_NUM_INTX,
						   &intx_domain_ops, pp);
	of_node_put(intc);
	if (!mcom03->irq_domain) {
		dev_err(dev, "Failed to create INTx IRQ domain\n");
		return -EINVAL;
	}

	irq_set_chained_handler_and_data(pp->irq,
					 mcom03_pcie_legacy_irq_handler, mcom03);

	return 0;
}

static int mcom03_add_dw_pcie_rp(struct mcom03_pcie *pcie,
				struct platform_device *pdev)
{
	struct dw_pcie *pci = pcie->pci;
	struct dw_pcie_rp *pp = &pci->pp;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	int ret;

	// TODO: Add Hotplug, LEQ, and other IRQ support
	pp->irq = platform_get_irq_byname(pdev, "legacy");
	if (pp->irq < 0) {
		dev_info(dev, "Legacy IRQ not found: only MSI will work\n");
	} else {
		ret = mcom03_pcie_config_legacy_irq(pp);
		if (ret < 0)
			return ret;
	}

	if (of_property_read_bool(np, "msi-parent") ||
	    of_property_read_bool(np, "msi-map")) {
		pp->ops = &mcom03_pcie_host_ops;
		dev_info(dev, "Using external MSI interrupt-controller\n");
	} else {
		pcie->embed_msi = true;
		pp->ops = &mcom03_pcie_host_ops_embed;
		dev_info(dev, "Using embedded MSI interrupt-controller\n");
	}

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize PCIe host\n");
		return ret;
	}

	/* MCom-03 does not have available memory for DMA allocation below 4 Gbytes
	 * so 32-bit MSI devices will not work with current embedded MSI controller
	 * implementation. As mentioned in PCI Express DM Controller Databook v5.30
	 * $3.8.2.2 inbound MWr request detected as MSI interrupt "is dropped and
	 * never appears on the AXI bus." we can actually use almost any address
	 * for MSI interrupt messages. So as a workaround we do the following -
	 * redefine address for MSI interrupts with same address as GITS_TRANSLATER
	 * register in GIC ITS after embedded MSI controller is initialized in
	 * dw_pcie_host_init(). */
	if (pcie->embed_msi) {
		pp->msi_data = 0x1130040;
		dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO, lower_32_bits(pp->msi_data));
		dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI, upper_32_bits(pp->msi_data));
		dev_info(dev, "Redirecting MSI to %#llx\n", pp->msi_data);
	}

	mcom03_pcie_dump_phy_regs(pci);

	return 0;
}

static int mcom03_pcie_start_link(struct dw_pcie *pci)
{
	struct mcom03_pcie *pcie = to_mcom03_pcie(pci);

	mcom03_pcie_ltssm_toggle(pcie, 1);

	return 0;
};

static void mcom03_pcie_stop_link(struct dw_pcie *pci)
{
	struct mcom03_pcie *pcie = to_mcom03_pcie(pci);

	mcom03_pcie_ltssm_toggle(pcie, 0);
};

static int mcom03_pcie_link_up(struct dw_pcie *pci)
{
	struct mcom03_pcie *pcie = to_mcom03_pcie(pci);
	u32 debug_st;

	debug_st = mcom03_pcie_readl(pcie, DEBUG_ST_OFF);

	return (debug_st & DEBUG_ST_SMLH_LINK_UP) &&
		(debug_st & DEBUG_ST_RDLH_LINK_UP);
}

static const struct dw_pcie_ops dw_pcie_ops = {
	.start_link = mcom03_pcie_start_link,
	.stop_link = mcom03_pcie_stop_link,
	.link_up = mcom03_pcie_link_up,
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

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "apb");
	pcie->apb_base = devm_ioremap_resource(dev, res);
	if (IS_ERR(pcie->apb_base)) {
		dev_err(dev, "Failed to remap apb memory\n");
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

	if (pcie->id > 1 || pcie->id < 0) {
		dev_err(dev, "Invalid elvees,ctrl-id %u\n", pcie->id);
		return -EINVAL;
	}

	pcie->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(pcie->reset_gpio)) {
		dev_err(dev, "Can't get 'reset-gpio'\n");
		return PTR_ERR(pcie->reset_gpio);
	} else if (!pcie->reset_gpio) {
		dev_warn(dev, "'reset-gpio' isn't specified in PCIe node\n");
		dev_info(dev, "PERST must be set with other means\n");
	}

	pcie->reset = devm_reset_control_array_get_shared(dev);
	if (IS_ERR(pcie->reset)) {
		if (PTR_ERR(pcie->reset) != -EPROBE_DEFER)
			dev_err(dev, "Failed to get PCIe resets\n");
		return PTR_ERR(pcie->reset);
	}

	platform_set_drvdata(pdev, pcie);

	return mcom03_add_dw_pcie_rp(pcie, pdev);
}

static int mcom03_pcie_remove(struct platform_device *pdev)
{
	struct mcom03_pcie *pcie = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&pcie->pci->pp);

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
MODULE_LICENSE("GPL");
