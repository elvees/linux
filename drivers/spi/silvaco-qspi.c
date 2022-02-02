// SPDX-License-Identifier: GPL-2.0+
/*
 * Driver for Silvaco AXI QSPI controller on MCom-03.
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/bitfield.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi_bitbang.h>
#include <linux/spi/xilinx_spi.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/clk.h>

#define SILVACO_DRIVER_NAME	"silvaco_qspi"

#define SILVACO_CTRL_XFER	BIT(0)
#define SILVACO_CTRL_MSB1ST	BIT(2)
#define SILVACO_CTRL_MASTER	BIT(5)

#define SILVACO_AUX_INHDIN	BIT(3)
#define SILVACO_AUX_CNTXFEREXT	BIT(7)
#define SILVACO_AUX_BITSIZE	GENMASK(12, 8)

#define SILVACO_STAT_XFER	BIT(0)
#define SILVACO_STAT_TXFULL	BIT(4)
#define SILVACO_STAT_RXEMPTY	BIT(5)

#define SILVACO_MAX_DUMMY	(u16)255

#define GPO_OUTPUT_SISO_0_1	(0x3)
#define GPO_OUTPUT_SISO_2_3	(0xC)
#define GPO_TRISTATE_SISO_0_1	(0x3 << 4)
#define GPO_TRISTATE_SISO_2_3	(0xC << 4)
#define GPO_ENABLE_SISO_0_1	(0x3 << 8)
#define GPO_ENABLE_SISO_2_3	(0xC << 8)
#define GPO_DISABLE_FULL_DUPLEX	BIT(12)

struct silvaco_qspi_regs {
	u32 tx_data;      /* Serial Transmit Data Register */
	u32 rx_data;      /* Serial Receive Data Register */
	u32 reserv;       /* Reserved */
	u32 ctrl;         /* Primary Control Register */
	u32 ctrl_aux;     /* Auxiliary Control Register */
	u32 stat;         /* Status Register */
	u32 ss;           /* Slave Select Register */
	u32 ss_polar;     /* Slave Select Polarity Register */
	u32 intr_en;      /* Interrupt Enable Register */
	u32 intr_stat;    /* Interrupt Status Register */
	u32 inet_clr;     /* Interrupt Clear Register */
	u32 tx_fifo_lvl;  /* Transmit FIFO Fill Level Register (RO) */
	u32 rx_fifo_lvl;  /* Receive FIFO Fill Level Register (RO) */
	u32 unused;       /* Reserved */
	u32 master_delay; /* Master Mode Inter-transfer Delay Register */
	u32 en;           /* Enable/Disable Register */
	u32 gpo_set;      /* General Purpose Outputs Set Register */
	u32 gpo_clr;      /* General Purpose Outputs Clear Register */
	u32 fifo_depth;   /* Configured FIFO Depth Register (RO) */
	u32 fifo_wmark;   /* TX/RX Watermark Level Register */
	u32 tx_dummy;     /* TX FIFO Dummy Load Register */
};

struct silvaco_qspi {
	struct device *dev;
	struct silvaco_qspi_regs *regs;
	struct spi_transfer *xfer;

	struct clk *clk_axi;
	struct clk *clk_ext;
	struct reset_control *rst_ctrl;

	unsigned int tx_count;
	unsigned int rx_count;
	unsigned int speed_hz;
	unsigned int fifo_depth;

	const u8 *txp;
	u8 *rxp;
	u8 bpw;

	bool fdx;
	bool use_tx_dummy;

	u16 remain_oneshot;

};

static void silvaco_qspi_reinit_bitsize(struct silvaco_qspi *hw, u8 bpw)
{
	u32p_replace_bits(&hw->regs->ctrl_aux, bpw - 1, SILVACO_AUX_BITSIZE);
}

static void silvaco_qspi_inhibit(struct silvaco_qspi *hw, bool set)
{
	if (set)
		writel(readl(&hw->regs->ctrl_aux) | SILVACO_AUX_INHDIN,
		       &hw->regs->ctrl_aux);
	else
		writel(readl(&hw->regs->ctrl_aux) & ~SILVACO_AUX_INHDIN,
		       &hw->regs->ctrl_aux);
}

static int silvaco_qspi_tx_fifo_not_full(struct silvaco_qspi_regs *regs)
{
	unsigned int reg = 0;

	return readl_poll_timeout(&regs->stat, reg,
				  (~(reg) & SILVACO_STAT_TXFULL), 0, 1000);
}

static int silvaco_qspi_tx_fifo_ready(struct silvaco_qspi_regs *regs)
{
	unsigned int reg = 0;

	return readl_poll_timeout(&regs->stat, reg,
					(~(reg) & SILVACO_STAT_TXFULL) &&
					(~(reg) & SILVACO_STAT_XFER), 0, 1000);
}

static int silvaco_qspi_rx_fifo_ready(struct silvaco_qspi_regs *regs)
{
	unsigned int reg = 0;

	return readl_poll_timeout(&regs->stat, reg,
				  ~(reg) & SILVACO_STAT_RXEMPTY, 0, 1000);
}

static int silvaco_end_pio(struct silvaco_qspi *hw)
{
	struct device *dev = hw->dev;
	int ret = 0;

	if (hw->rxp)
		silvaco_qspi_inhibit(hw, true);

	if (hw->xfer->tx_buf) {
		ret = silvaco_qspi_tx_fifo_ready(hw->regs);
		if (ret)
			dev_err(dev, "Last %d bit word transmit timed out\n",
				hw->bpw);
	}

	return ret;
}

static void silvaco_calc_remain_oneshot(struct silvaco_qspi *hw,
					unsigned int bytes)
{
	switch (hw->bpw) {
	case 8:
		hw->remain_oneshot = min(hw->fifo_depth, bytes);
		break;
	case 32:
		hw->remain_oneshot = min(hw->fifo_depth, bytes / 4);
		break;
	}

	if (hw->use_tx_dummy)
		hw->remain_oneshot = min(hw->remain_oneshot, SILVACO_MAX_DUMMY);
}

static void silvaco_write_word(struct silvaco_qspi *hw)
{
	unsigned int word = 0;

	if (likely(hw->bpw == 32)) {
		word = cpu_to_be32(*(u32 *)&hw->txp[hw->tx_count]);
		hw->tx_count += 4;
	} else {
		word = hw->txp[hw->tx_count];
		hw->tx_count++;
	}

	writel(word, &hw->regs->tx_data);
}

static void silvaco_read_word(struct silvaco_qspi *hw)
{
	unsigned int word = readl(&hw->regs->rx_data);

	if (likely(hw->bpw == 32)) {
		*(u32 *)&hw->rxp[hw->rx_count] = be32_to_cpu(word);
		hw->rx_count += 4;
	} else {
		hw->rxp[hw->rx_count] = word;
		hw->rx_count++;
	}
}

static int silvaco_qspi_handle_packets(struct silvaco_qspi *hw,
				       unsigned int *bytes, bool set_8_bpw)
{
	struct device *dev = hw->dev;
	unsigned int w_size = hw->bpw == 32 ? 4 : 1;
	unsigned int cur_oneshot = hw->remain_oneshot;
	int ret = 0;

	if (set_8_bpw) {
		ret = silvaco_qspi_tx_fifo_not_full(hw->regs);
		if (ret) {
			dev_err(dev, "%s: Transmit last 32 bit timed out\n",
				__func__);
			goto end_handle_packets;
		}

		hw->bpw = 8;
		silvaco_qspi_reinit_bitsize(hw, hw->bpw);
	}

	if (hw->use_tx_dummy)
		writel(hw->remain_oneshot, &hw->regs->tx_dummy);

	while (*bytes >= w_size) {

		if (hw->txp) {
			if (!cur_oneshot) {
				ret = silvaco_qspi_tx_fifo_not_full(hw->regs);
				if (ret) {
					dev_err(dev, "%s: On %d transmit %d bit timed out\n",
						__func__, hw->tx_count,
						hw->bpw);
					goto end_handle_packets;
				}
			}

			silvaco_write_word(hw);
		}

		if (hw->rxp) {
			if (!cur_oneshot && hw->use_tx_dummy)
				writel(1, &hw->regs->tx_dummy);

			ret = silvaco_qspi_rx_fifo_ready(hw->regs);
			if (ret) {
				dev_err(dev, "%s: On %d receive %d bit timed out\n",
					__func__, hw->rx_count, hw->bpw);
				goto end_handle_packets;
			}

			silvaco_read_word(hw);
		}

		if (cur_oneshot)
			cur_oneshot--;

		*bytes -= w_size;
	}

end_handle_packets:
	return ret;
}

static int silvaco_qspi_fdx_hdx_pio(struct silvaco_qspi *hw)
{
	struct device *dev = hw->dev;
	struct silvaco_qspi_regs *regs = hw->regs;
	unsigned int bytes = hw->xfer->len;

	bool set_8_bpw = false;
	int ret = 0;

	hw->fdx = hw->xfer->tx_buf && hw->xfer->rx_buf;
	hw->use_tx_dummy = !hw->fdx && hw->xfer->rx_buf;

	hw->bpw = bytes >= 4 ? 32 : 8;

	silvaco_qspi_reinit_bitsize(hw, hw->bpw);

	silvaco_calc_remain_oneshot(hw, bytes);

	if (hw->rxp)
		silvaco_qspi_inhibit(hw, false);

	while (bytes) {
		ret = silvaco_qspi_handle_packets(hw, &bytes, set_8_bpw);
		if (ret) {
			dev_err(dev, "Error %s packet handling\n",
				hw->fdx ? "fdx" : "hdx");
			goto silvaco_end_pio;
		}

		// reconfig oneshot if there is tail
		if (bytes) {
			set_8_bpw = true;
			silvaco_calc_remain_oneshot(hw, bytes);
		}
	}

silvaco_end_pio:
	silvaco_end_pio(hw);

	return ret;
}

int silvaco_qspi_setup(struct spi_device *spi)
{
	dev_dbg(&spi->controller->dev, "%s reg %d\n", __func__,
		spi->chip_select);

	return 0;
}

void silvaco_qspi_set_cs(struct spi_device *spi, bool enable)
{
	struct silvaco_qspi *silvaco = spi_master_get_devdata(spi->master);

	/* Chip select logic is inverted from spi_set_cs */
	dev_dbg(silvaco->dev, "%s chip select %d\n",
		enable ? "Disable" : "Enable", spi->chip_select);

	writel(enable ? 0 : BIT(spi->chip_select), &silvaco->regs->ss);
}

static int silvaco_qspi_transfer_one(struct spi_controller *master,
				     struct spi_device *spi,
				     struct spi_transfer *xfer)
{
	struct silvaco_qspi *hw = spi_master_get_devdata(master);
	int ret = 0;

	if (xfer->speed_hz != hw->speed_hz) {
		hw->speed_hz = xfer->speed_hz;
		clk_set_rate(hw->clk_ext, xfer->speed_hz);
		dev_dbg(hw->dev, "Switch to new speed (%d HZ) for cs %d\n",
			hw->speed_hz, spi->chip_select);
	}

	hw->xfer = xfer;
	hw->txp = xfer->tx_buf;
	hw->rxp = xfer->rx_buf;
	hw->tx_count = 0;
	hw->rx_count = 0;

	ret = silvaco_qspi_fdx_hdx_pio(hw);

	spi_finalize_current_transfer(master);

	return ret;
}

static int silvaco_qspi_init(struct device *dev, struct silvaco_qspi *silvaco)
{
	struct silvaco_qspi_regs *regs = silvaco->regs;
	int ret = 0;
	u32 reg = 0;

	silvaco->dev = dev;
	writel(0, &regs->en);
	ret = readl_poll_timeout(&regs->en, reg, ~(reg) & BIT(0), 0, 1000);
	if (ret) {
		dev_err(dev, "Timeout upon disabling Silvaco controller\n");
		return ret;
	}

	writel(0, &regs->ss_polar);
	writel(SILVACO_CTRL_MASTER | SILVACO_CTRL_XFER | SILVACO_CTRL_MSB1ST,
	       &regs->ctrl);
	writel(SILVACO_AUX_INHDIN | SILVACO_AUX_CNTXFEREXT, &regs->ctrl_aux);

	writel(1, &regs->en);
	ret = readl_poll_timeout(&regs->en, reg, reg & BIT(0), 0, 1000);
	if (ret) {
		dev_err(dev, "Timeout upon enabling Silvaco controller\n");
		return ret;
	}

	writel(GPO_OUTPUT_SISO_2_3 | GPO_ENABLE_SISO_2_3, &regs->gpo_set);

	/* full-duplex on */
	writel(GPO_ENABLE_SISO_0_1 | GPO_TRISTATE_SISO_2_3 |
	       GPO_DISABLE_FULL_DUPLEX, &regs->gpo_clr);

	silvaco->fifo_depth = readl(&silvaco->regs->fifo_depth);
	return 0;
}

#if defined(CONFIG_OF)
static const struct of_device_id silvaco_qspi_of_match[] = {
	{ .compatible = "silvaco,axi-qspi" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, silvaco_qspi_of_match);
#endif

int silvaco_qspi_probe(struct platform_device *pdev)
{
	struct silvaco_qspi *silvaco;
	struct resource *res;
	struct spi_master *master;
	struct device *dev = &pdev->dev;
	int ret, num_cs = 0;

	master = spi_alloc_master(dev, sizeof(struct silvaco_qspi));
	if (!master)
		return -ENODEV;

	if (of_property_read_u32(pdev->dev.of_node, "num-cs", &num_cs)) {
		dev_err(dev, "Missing chip select configuration data\n");
		return -EINVAL;
	}

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(dev, "Unable to get iomem resource\n");
		return -ENODEV;
	}

	silvaco = spi_master_get_devdata(master);

	silvaco->clk_axi = devm_clk_get(&pdev->dev, "clk_axi");
	if (IS_ERR(silvaco->clk_axi)) {
		dev_err(&pdev->dev, "clk_axi clock not found.\n");
		ret = PTR_ERR(silvaco->clk_axi);
		goto silvaco_master_free;
	}

	silvaco->clk_ext = devm_clk_get(&pdev->dev, "clk_ext");
	if (IS_ERR(silvaco->clk_ext)) {
		dev_err(&pdev->dev, "clk_ext clock not found.\n");
		ret = PTR_ERR(silvaco->clk_ext);
		goto silvaco_master_free;
	}

	ret = clk_prepare_enable(silvaco->clk_axi);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable AXI clock.\n");
		goto silvaco_master_free;
	}

	ret = clk_prepare_enable(silvaco->clk_ext);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable EXT clock.\n");
		goto disable_clk_axi;
	}

	silvaco->regs = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(silvaco->regs)) {
		ret = PTR_ERR(silvaco->regs);
		goto disable_clk_ext;
	}

	silvaco->rst_ctrl =
		devm_reset_control_get_optional_exclusive(&pdev->dev, NULL);
	if (IS_ERR(silvaco->rst_ctrl)) {
		ret = PTR_ERR(silvaco->rst_ctrl);
		if (ret != -ENOENT) {
			dev_err(&pdev->dev, "Failed to get reset control.\n");
			goto disable_clk_ext;
		}
	}

	ret = ret == -ENOENT ? 0 : reset_control_deassert(silvaco->rst_ctrl);
	if (ret) {
		dev_err(&pdev->dev, "Failed to deassert reset.\n");
		goto disable_clk_ext;
	}

	/* setup the master state. */
	master->dev.of_node = pdev->dev.of_node;
	master->bus_num = -1;
	master->num_chipselect = num_cs;
	master->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(32);
	master->setup = silvaco_qspi_setup;
	master->set_cs = silvaco_qspi_set_cs;
	master->transfer_one = silvaco_qspi_transfer_one;

	ret = silvaco_qspi_init(dev, silvaco);
	if (ret) {
		dev_err(&pdev->dev, "Could not initlize Silvaco controller.\n");
		goto assert_reset;
	}

	ret = devm_spi_register_master(&pdev->dev, master);
	if (ret) {
		dev_err(&pdev->dev, "Unable to register spi master.\n");
		goto assert_reset;
	}

	dev_dbg(&pdev->dev, "Register Silvaco QSPI driver.\n");

	return 0;

assert_reset:
	reset_control_assert(silvaco->rst_ctrl);
disable_clk_ext:
	clk_disable_unprepare(silvaco->clk_ext);
disable_clk_axi:
	clk_disable_unprepare(silvaco->clk_axi);
silvaco_master_free:
	spi_master_put(master);

	return ret;
}

int silvaco_qspi_remove(struct platform_device *pdev)
{
	struct spi_master *master = spi_master_get(platform_get_drvdata(pdev));
	struct silvaco_qspi *silvaco = spi_master_get_devdata(master);

	clk_disable_unprepare(silvaco->clk_ext);
	clk_disable_unprepare(silvaco->clk_axi);

	spi_master_put(master);

	return 0;
}

static struct platform_driver silvaco_qspi_driver = {
	.probe = silvaco_qspi_probe,
	.remove = silvaco_qspi_remove,
	.driver = {
		.name = SILVACO_DRIVER_NAME,
		.of_match_table = silvaco_qspi_of_match,
	},
};

module_platform_driver(silvaco_qspi_driver);
