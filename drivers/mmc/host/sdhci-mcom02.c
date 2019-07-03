/*
 * MCOM02 Secure Digital Host Controller Interface.
 *
 * Copyright 2017 - 2019 RnD Center "ELVEES", JSC
 * Copyright 2011 - 2012 Michal Simek <monstr@monstr.eu>
 * Copyright 2012 Wind River Systems, Inc.
 * Copyright 2013 Pengutronix e.K.
 * Copyright 2013 Xilinx Inc.
 *
 * Based on sdhci-of-arasan.c
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or (at
 * your option) any later version.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/of_device.h>

#include "sdhci-pltfm.h"

#define SDHCI_MCOM02_CLK_CTRL_OFFSET	0x2c

#define CLK_CTRL_TIMEOUT_SHIFT		16
#define CLK_CTRL_TIMEOUT_MASK		(0xf << CLK_CTRL_TIMEOUT_SHIFT)
#define CLK_CTRL_TIMEOUT_MIN_EXP	13

#define SDMMC_INIT_CONFIG_1		0x100
#define SDMMC_INIT_CONFIG_ITAPDLYSEL	GENMASK(31, 27)
#define SDMMC_INIT_CONFIG_ITAPDLYENA	BIT(26)
#define SDMMC_INIT_CONFIG_ITAPCHGWIN	BIT(25)
#define SDMMC_INIT_CONFIG_OTAPDLYSEL	GENMASK(24, 21)
#define SDMMC_INIT_CONFIG_OTAPDLYENA	BIT(20)

/**
 * struct sdhci_mcom02_data
 * @clk_ahb:	Pointer to the AHB clock
 * @clk_phase:	Array of input/output clock phase delays for all speed modes
 */
struct sdhci_mcom02_data {
	struct clk *clk_ahb;
	int  clk_phase[MMC_TIMING_MMC_HS400 + 1][2];
};

static unsigned int sdhci_mcom02_get_timeout_clock(struct sdhci_host *host)
{
	u32 div;
	unsigned long freq;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	div = readl(host->ioaddr + SDHCI_MCOM02_CLK_CTRL_OFFSET);
	div = (div & CLK_CTRL_TIMEOUT_MASK) >> CLK_CTRL_TIMEOUT_SHIFT;

	freq = clk_get_rate(pltfm_host->clk);
	freq /= 1 << (CLK_CTRL_TIMEOUT_MIN_EXP + div);

	return freq;
}

static inline u32 prep_field(u32 value, u32 mask)
{
	return (value << (ffs(mask) - 1)) & mask;
}

static void sdhci_mcom02_set_clk_delays(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_mcom02_data *sdhci_mcom02 = pltfm_host->priv;
	u32 reg;

	/* Set Tap Delay Lines */
	reg = readl(host->ioaddr + SDMMC_INIT_CONFIG_1);
	if (sdhci_mcom02->clk_phase[host->timing][0] != -1) {
		/* This signal should be asserted few clocks before
		 * the corectrl_itapdlysel changes and should be
		 * asserted for few clocks after.
		 */
		reg |= SDMMC_INIT_CONFIG_ITAPCHGWIN;
		writel(reg, host->ioaddr + SDMMC_INIT_CONFIG_1);

		reg &= ~SDMMC_INIT_CONFIG_ITAPDLYSEL;
		reg |= prep_field(sdhci_mcom02->clk_phase[host->timing][0],
				  SDMMC_INIT_CONFIG_ITAPDLYSEL);
		reg |= SDMMC_INIT_CONFIG_ITAPDLYENA;
		udelay(1); /* Asserted few clocks before */
		writel(reg, host->ioaddr + SDMMC_INIT_CONFIG_1);

		reg &= ~SDMMC_INIT_CONFIG_ITAPCHGWIN;
		udelay(1); /* Asserted few clocks after */
		writel(reg, host->ioaddr + SDMMC_INIT_CONFIG_1);
	} else if (reg & SDMMC_INIT_CONFIG_ITAPDLYENA) {
		reg &= ~SDMMC_INIT_CONFIG_ITAPDLYENA;
		writel(reg, host->ioaddr + SDMMC_INIT_CONFIG_1);
	}

	reg = readl(host->ioaddr + SDMMC_INIT_CONFIG_1);
	if (sdhci_mcom02->clk_phase[host->timing][1] != -1) {
		reg &= ~SDMMC_INIT_CONFIG_OTAPDLYSEL;
		reg |= prep_field(sdhci_mcom02->clk_phase[host->timing][1],
				  SDMMC_INIT_CONFIG_OTAPDLYSEL);
		reg |= SDMMC_INIT_CONFIG_OTAPDLYENA;
		writel(reg, host->ioaddr + SDMMC_INIT_CONFIG_1);
	} else if (reg & SDMMC_INIT_CONFIG_OTAPDLYENA) {
		reg &= ~SDMMC_INIT_CONFIG_OTAPDLYENA;
		writel(reg, host->ioaddr + SDMMC_INIT_CONFIG_1);
	}
}

static void sdhci_mcom02_set_clock(struct sdhci_host *host, unsigned int clock)
{
	/* Set the input and output clock phase delays */
	sdhci_mcom02_set_clk_delays(host);

	sdhci_set_clock(host, clock);
}

static void mcom02_of_read_clk_phase(struct device *dev,
				     struct sdhci_mcom02_data *sdhci_mcom02,
				     unsigned int timing, const char *prop)
{
	struct device_node *np = dev->of_node;

	int clk_phase[2] = { -1, -1 };

	/*
	 * Read Tap Delay values from DT, if the DT does not contain the
	 * tap values then disable tap delay for this mode
	 */
	if (prop)
		of_property_read_u32_array(np, prop, clk_phase, 2);

	/* The values read are input and output clock delays in order */
	sdhci_mcom02->clk_phase[timing][0] = clk_phase[0];
	sdhci_mcom02->clk_phase[timing][1] = clk_phase[1];
}

/**
 * mcom02_of_parse_clk_phases - parse DeviceTree node for clock delay values
 * @dev:	Pointer to our struct device.
 *
 * Called at initialization to parse the values of clock delays.
 */
static void mcom02_of_parse_clk_phases(struct device *dev,
				       struct sdhci_mcom02_data *sdhci_mcom02)
{
	static const char *const names[MMC_TIMING_MMC_HS400 + 1] = {
		[MMC_TIMING_MMC_HS]      = "mcom02,clk-phase-mmc-hs",
		[MMC_TIMING_SD_HS]       = "mcom02,clk-phase-sd-hs",
		[MMC_TIMING_UHS_SDR12]   = "mcom02,clk-phase-uhs-sdr12",
		[MMC_TIMING_UHS_SDR25]   = "mcom02,clk-phase-uhs-sdr25",
		[MMC_TIMING_UHS_SDR50]   = "mcom02,clk-phase-uhs-sdr50",
		[MMC_TIMING_UHS_SDR104]  = "mcom02,clk-phase-uhs-sdr104",
		[MMC_TIMING_UHS_DDR50]   = "mcom02,clk-phase-uhs-ddr50",
		[MMC_TIMING_MMC_DDR52]   = "mcom02,clk-phase-mmc-ddr52",
		[MMC_TIMING_MMC_HS200]   = "mcom02,clk-phase-mmc-hs200",
	};
	int i;

	for (i = 0; i <= MMC_TIMING_MMC_HS400; i++)
		mcom02_of_read_clk_phase(dev, sdhci_mcom02, i, names[i]);
}

static struct sdhci_ops sdhci_mcom02_ops = {
	.set_clock = sdhci_mcom02_set_clock,
	.get_max_clock = sdhci_pltfm_clk_get_max_clock,
	.get_timeout_clock = sdhci_mcom02_get_timeout_clock,
	.set_bus_width = sdhci_set_bus_width,
	.reset = sdhci_reset,
	.set_uhs_signaling = sdhci_set_uhs_signaling,
};

static struct sdhci_pltfm_data sdhci_mcom02_pdata = {
	.ops = &sdhci_mcom02_ops,
	.quirks = SDHCI_QUIRK_CAP_CLOCK_BASE_BROKEN,
	.quirks2 = SDHCI_QUIRK2_PRESET_VALUE_BROKEN |
		   SDHCI_QUIRK2_CLOCK_DIV_ZERO_BROKEN,
};

#ifdef CONFIG_PM_SLEEP
/**
 * sdhci_mcom02_suspend - Suspend method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Put the device in a low power state.
 */
static int sdhci_mcom02_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_mcom02_data *sdhci_mcom02 = pltfm_host->priv;
	int ret;

	ret = sdhci_suspend_host(host);
	if (ret)
		return ret;

	clk_disable(pltfm_host->clk);
	clk_disable(sdhci_mcom02->clk_ahb);

	return 0;
}

/**
 * sdhci_mcom02_resume - Resume method for the driver
 * @dev:	Address of the device structure
 * Returns 0 on success and error value on error
 *
 * Resume operation after suspend
 */
static int sdhci_mcom02_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_mcom02_data *sdhci_mcom02 = pltfm_host->priv;
	int ret;

	ret = clk_enable(sdhci_mcom02->clk_ahb);
	if (ret) {
		dev_err(dev, "Cannot enable AHB clock.\n");
		return ret;
	}

	ret = clk_enable(pltfm_host->clk);
	if (ret) {
		dev_err(dev, "Cannot enable SD clock.\n");
		clk_disable(sdhci_mcom02->clk_ahb);
		return ret;
	}

	return sdhci_resume_host(host);
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sdhci_mcom02_dev_pm_ops, sdhci_mcom02_suspend,
			 sdhci_mcom02_resume);

static int sdhci_mcom02_probe(struct platform_device *pdev)
{
	int ret;
	struct clk *clk_xin;
	struct sdhci_host *host;
	struct sdhci_pltfm_host *pltfm_host;
	struct sdhci_mcom02_data *sdhci_mcom02;

	sdhci_mcom02 = devm_kzalloc(&pdev->dev, sizeof(*sdhci_mcom02),
				    GFP_KERNEL);
	if (!sdhci_mcom02)
		return -ENOMEM;

	sdhci_mcom02->clk_ahb = devm_clk_get(&pdev->dev, "clk_ahb");
	if (IS_ERR(sdhci_mcom02->clk_ahb)) {
		dev_err(&pdev->dev, "clk_ahb clock not found.\n");
		return PTR_ERR(sdhci_mcom02->clk_ahb);
	}

	clk_xin = devm_clk_get(&pdev->dev, "clk_xin");
	if (IS_ERR(clk_xin)) {
		dev_err(&pdev->dev, "clk_xin clock not found.\n");
		return PTR_ERR(clk_xin);
	}

	ret = clk_prepare_enable(sdhci_mcom02->clk_ahb);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable AHB clock.\n");
		return ret;
	}

	ret = clk_prepare_enable(clk_xin);
	if (ret) {
		dev_err(&pdev->dev, "Unable to enable SD clock.\n");
		goto clk_dis_ahb;
	}

	host = sdhci_pltfm_init(pdev, &sdhci_mcom02_pdata, 0);
	if (IS_ERR(host)) {
		ret = PTR_ERR(host);
		goto clk_disable_all;
	}

	sdhci_get_of_property(pdev);
	pltfm_host = sdhci_priv(host);
	pltfm_host->priv = sdhci_mcom02;
	pltfm_host->clk = clk_xin;

	mcom02_of_parse_clk_phases(&pdev->dev, sdhci_mcom02);

	ret = mmc_of_parse(host->mmc);
	if (ret) {
		dev_err(&pdev->dev, "parsing dt failed (%u)\n", ret);
		goto err_pltfm_free;
	}

	ret = sdhci_add_host(host);
	if (ret)
		goto err_pltfm_free;

	return 0;

err_pltfm_free:
	sdhci_pltfm_free(pdev);
clk_disable_all:
	clk_disable_unprepare(clk_xin);
clk_dis_ahb:
	clk_disable_unprepare(sdhci_mcom02->clk_ahb);

	return ret;
}

static int sdhci_mcom02_remove(struct platform_device *pdev)
{
	struct sdhci_host *host = platform_get_drvdata(pdev);
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct sdhci_mcom02_data *sdhci_mcom02 = pltfm_host->priv;

	clk_disable_unprepare(sdhci_mcom02->clk_ahb);

	return sdhci_pltfm_unregister(pdev);
}

static const struct of_device_id sdhci_mcom02_of_match[] = {
	{ .compatible = "elvees,sdhci-mcom02" },
	{ }
};
MODULE_DEVICE_TABLE(of, sdhci_mcom02_of_match);

static struct platform_driver sdhci_mcom02_driver = {
	.driver = {
		.name = "sdhci-mcom02",
		.of_match_table = sdhci_mcom02_of_match,
		.pm = &sdhci_mcom02_dev_pm_ops,
	},
	.probe = sdhci_mcom02_probe,
	.remove = sdhci_mcom02_remove,
};

module_platform_driver(sdhci_mcom02_driver);

MODULE_DESCRIPTION("Driver for the MCom-02 SDHCI Controller");
MODULE_LICENSE("GPL");
