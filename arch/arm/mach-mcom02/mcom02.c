/*
 *  Copyright 2015-2019 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/mach/arch.h>
#include <asm/hardware/cache-l2x0.h>
#include <asm/mach/map.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/platform_data/brcmfmac-sdio.h>
#include <linux/platform_device.h>

#include "common.h"
#include "suspend.h"

static const char * const mcom02_dt_board_compat[] = {
	"elvees,mcom02",
	NULL
};

static struct map_desc uart_io_desc __initdata = {
	.virtual	= 0xf8028000,
	.pfn		= __phys_to_pfn(0x38028000),
	.length		= SZ_4K,
	.type		= MT_DEVICE,
};

static void __init mcom02_map_io(void)
{
	iotable_init(&uart_io_desc, 1);
}

static struct platform_device brcmfmac_device;

static int get_reg_on_gpio(void)
{
	struct device_node *node;

	node = of_find_compatible_node(NULL, NULL, "brcm,bcm4329-fmac");
	if (!node)
		return -1;
	return of_get_named_gpio(node, "reg-on-gpio", 0);
}

static void brcmfmac_power_on(void)
{
	static bool reg_on_gpio_requested;
	int err;
	int reg_on_gpio = get_reg_on_gpio();

	if (!gpio_is_valid(reg_on_gpio))
		return;

	if (!reg_on_gpio_requested) {
		err = devm_gpio_request_one(&brcmfmac_device.dev, reg_on_gpio,
					    GPIOF_OUT_INIT_HIGH, "reg_on");

		if (err) {
			dev_err(&brcmfmac_device.dev,
				"failed to request gpio %d (reg_on) : %d\n",
				reg_on_gpio, err);
			return;
		}
		reg_on_gpio_requested = true;
	}
	gpio_set_value(reg_on_gpio, 1);
}

static void brcmfmac_power_off(void)
{
	int reg_on_gpio = get_reg_on_gpio();

	if (!gpio_is_valid(reg_on_gpio))
		return;

	gpio_set_value(reg_on_gpio, 0);
}

static struct brcmfmac_sdio_platform_data brcmfmac_sdio_pdata = {
	.power_on		= brcmfmac_power_on,
	.power_off		= brcmfmac_power_off,
};

static struct platform_device brcmfmac_device = {
	.name			= BRCMFMAC_SDIO_PDATA_NAME,
	.id			= PLATFORM_DEVID_NONE,
	.dev.platform_data	= &brcmfmac_sdio_pdata
};

void __init mcom02_init_machine(void)
{
	platform_device_register(&brcmfmac_device);
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_simple("cpufreq-dt", 0, NULL, 0);
	mcom02_suspend_init();
	mcom02_init_pmctr();
}

static struct device bt_device;

static void bt_gpio_init(void)
{
	struct device_node *node;
	struct gpio_desc *gpio_wake;
	struct gpio_desc *gpio_reset;

	node = of_find_compatible_node(NULL, NULL, "brcm,bcm43438-bt");
	if (!node)
		return;

	bt_device.of_node = node;
	gpio_wake = gpiod_get_optional(&bt_device, "wake", GPIOD_OUT_HIGH);
	gpio_reset = gpiod_get_optional(&bt_device, "reset", GPIOD_OUT_HIGH);
	if (!IS_ERR_OR_NULL(gpio_reset)) {
		msleep(20);
		gpiod_set_value(gpio_reset, 0);
	}
}

void __init mcom02_init_late(void)
{
	/* TODO: Driver bcm_hci support device-tree since kernel 4.14.
	 * Wake and reset control must be moved to bcm_hci driver after merge.
	 */
	bt_gpio_init();
}

DT_MACHINE_START(MCOM02, "ELVEES MCom-02 (Flattened Device Tree)")
	/* TODO: Replace with device tree arm,shared-override
	 * attribute when it will be upstreamed.
	 */
	.l2c_aux_val = L2C_AUX_CTRL_SHARED_OVERRIDE,
	.l2c_aux_mask = ~0,
	.map_io = mcom02_map_io,
	.dt_compat = mcom02_dt_board_compat,
	.init_machine = mcom02_init_machine,
	.init_late = mcom02_init_late,
MACHINE_END
