// SPDX-License-Identifier: GPL-2.0
/*
 * Watchdog driver for MCom-03
 *
 * Copyright 2022 RnD Center "ELVEES", JSC
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

#include <soc/elvees/mcom03/mcom03_sip.h>

#define to_mcom03_wdt(x)	container_of((x), struct mcom03_wdt, wdd)
#define mcom03_wdt_sip(id) \
	mcom03_sip_smccc_smc(MCOM03_SIP_WDT, (id), 0, 0, 0, 0, 0, 0)
#define mcom03_wdt_sip_param(id, param) \
	mcom03_sip_smccc_smc(MCOM03_SIP_WDT, (id), (param), 0, 0, 0, 0, 0)

struct mcom03_wdt {
	struct watchdog_device	wdd;
};

static int mcom03_wdt_start(struct watchdog_device *wdd)
{
	mcom03_wdt_sip_param(MCOM03_SIP_WDT_START, wdd->timeout);
	return 0;
}

static int mcom03_wdt_stop(struct watchdog_device *wdd)
{
	set_bit(WDOG_HW_RUNNING, &wdd->status);
	return 0;
}

static int mcom03_wdt_ping(struct watchdog_device *wdd)
{
	mcom03_wdt_sip(MCOM03_SIP_WDT_PING);
	return 0;
}

static int mcom03_wdt_set_timeout_s(struct watchdog_device *wdd, unsigned int seconds)
{
	mcom03_wdt_sip_param(MCOM03_SIP_WDT_SET_TIMEOUT_S, seconds);
	wdd->timeout = mcom03_wdt_sip(MCOM03_SIP_WDT_GET_TIMEOUT_S);
	return 0;
}

static const struct watchdog_info mcom03_wdt_ident = {
	.identity	= "ELVEES MCom-03 SoC Watchdog",
	.options	= WDIOF_KEEPALIVEPING | WDIOF_SETTIMEOUT |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops mcom03_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= mcom03_wdt_start,
	.stop		= mcom03_wdt_stop,
	.ping		= mcom03_wdt_ping,
	.set_timeout	= mcom03_wdt_set_timeout_s,
};

static int mcom03_wdt_drv_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct watchdog_device *wdd;
	struct mcom03_wdt *mcom03_wdt;
	int ret;

	mcom03_wdt = devm_kzalloc(dev, sizeof(*mcom03_wdt), GFP_KERNEL);
	if (!mcom03_wdt)
		return -ENOMEM;

	wdd = &mcom03_wdt->wdd;
	wdd->info = &mcom03_wdt_ident;
	wdd->ops = &mcom03_wdt_ops;
	wdd->min_timeout = mcom03_wdt_sip(MCOM03_SIP_WDT_GET_MIN_TIMEOUT_S);
	wdd->max_hw_heartbeat_ms = mcom03_wdt_sip(MCOM03_SIP_WDT_GET_MAX_TIMEOUT_S) * 1000;
	wdd->parent = dev;
	watchdog_set_drvdata(wdd, mcom03_wdt);

	watchdog_set_nowayout(wdd, WATCHDOG_NOWAYOUT);
	watchdog_init_timeout(wdd, 0, dev);

	wdd->timeout = mcom03_wdt_sip(MCOM03_SIP_WDT_GET_TIMEOUT_S);

	/*
	 * If the watchdog is already running, use its already configured timeout.
	 */
	if (mcom03_wdt_sip(MCOM03_SIP_WDT_IS_ENABLE))
		set_bit(WDOG_HW_RUNNING, &wdd->status);

	platform_set_drvdata(pdev, mcom03_wdt);

	watchdog_set_restart_priority(wdd, 128);

	ret = watchdog_register_device(wdd);
	if (ret)
		goto exit;

	dev_dbg(dev, "Watchdog registered (timeout: %d sec)\n", wdd->timeout);
	return 0;

exit:
	return ret;
}

static int mcom03_wdt_drv_remove(struct platform_device *pdev)
{
	struct mcom03_wdt *mcom03_wdt = platform_get_drvdata(pdev);

	watchdog_unregister_device(&mcom03_wdt->wdd);

	dev_dbg(&pdev->dev, "Watchdog unregistered\n");
	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mcom03_wdt_of_match[] = {
	{ .compatible = "elvees,mcom03-wdt", },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mcom03_wdt_of_match);
#endif

static struct platform_driver mcom03_wdt_driver = {
	.probe		= mcom03_wdt_drv_probe,
	.remove		= mcom03_wdt_drv_remove,
	.driver		= {
		.name	= "mcom03-wdt",
#ifdef CONFIG_OF
		.of_match_table	= of_match_ptr(mcom03_wdt_of_match),
#endif
	},
};

module_platform_driver(mcom03_wdt_driver);

MODULE_LICENSE("GPL");
