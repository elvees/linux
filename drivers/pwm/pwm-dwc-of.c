// SPDX-License-Identifier: GPL-2.0
/*
 * DesignWare PWM Controller driver OF
 *
 * Copyright (C) 2022 SiFive, Inc.
 */

#define DEFAULT_MODULE_NAMESPACE dwc_pwm

#include <linux/bitops.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pwm.h>
#include <linux/io.h>

#include "pwm-dwc.h"

static int dwc_pwm_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dwc_pwm *dwc;
	struct clk *bus;
	u32 nr_pwm;
	int ret;

	dwc = dwc_pwm_alloc(dev);
	if (!dwc)
		return -ENOMEM;

	if (!device_property_read_u32(dev, "snps,pwm-number", &nr_pwm)) {
		if (nr_pwm > DWC_TIMERS_TOTAL)
			dev_err(dev, "too many PWMs (%d) specified, capping at %d\n",
				nr_pwm, dwc->chip.npwm);
		else
			dwc->chip.npwm = nr_pwm;
	}

	dwc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dwc->base))
		return PTR_ERR(dwc->base);

	bus = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(bus))
		return dev_err_probe(dev, PTR_ERR(bus),
				     "failed to get clock\n");

	dwc->clk = devm_clk_get_enabled(dev, "timer");
	if (IS_ERR(dwc->clk))
		return dev_err_probe(dev, PTR_ERR(dwc->clk),
				     "failed to get timer clock\n");

	ret = clk_rate_exclusive_get(dwc->clk);
	if (ret)
		return dev_err_probe(dev, ret,
				     "clk_rate_exclusive_get() failed\n");

	dwc->clk_rate = clk_get_rate(dwc->clk);
	return devm_pwmchip_add(dev, &dwc->chip);
}

static int dwc_pwm_plat_remove(struct platform_device *pdev)
{
	struct dwc_pwm *dwc = dev_get_drvdata(&pdev->dev);

	clk_rate_exclusive_put(dwc->clk);
	return 0;
}

static const struct of_device_id dwc_pwm_dt_ids[] = {
	{ .compatible = "snps,dw-apb-timers-pwm2" },
	{ },
};
MODULE_DEVICE_TABLE(of, dwc_pwm_dt_ids);

static struct platform_driver dwc_pwm_plat_driver = {
	.driver = {
		.name		= "dwc-pwm",
		.of_match_table  = dwc_pwm_dt_ids,
	},
	.probe	= dwc_pwm_plat_probe,
	.remove = dwc_pwm_plat_remove,
};

module_platform_driver(dwc_pwm_plat_driver);

MODULE_ALIAS("platform:dwc-pwm-of");
MODULE_AUTHOR("Ben Dooks <ben.dooks@codethink.co.uk>");
MODULE_DESCRIPTION("DesignWare PWM Controller");
MODULE_LICENSE("GPL");
