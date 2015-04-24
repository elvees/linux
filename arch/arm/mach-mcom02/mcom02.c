/*
 *  Copyright 2015 ELVEES NeoTek CJSC
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

void __init mcom02_init_machine(void)
{
	of_platform_populate(NULL, of_default_bus_match_table, NULL, NULL);
	platform_device_register_simple("cpufreq-dt", 0, NULL, 0);
	mcom02_suspend_init();
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
MACHINE_END
