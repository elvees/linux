/*
 *  Copyright 2017 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/kernel.h>
#include <linux/suspend.h>

#include "suspend.h"

static int mcom02_suspend_valid(suspend_state_t state)
{
	return state == PM_SUSPEND_STANDBY;
}

static int mcom02_suspend_enter(suspend_state_t state)
{
	switch (state) {
	case PM_SUSPEND_STANDBY:
		cpu_do_idle();
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct platform_suspend_ops mcom02_suspend_ops = {
	.valid = mcom02_suspend_valid,
	.enter = mcom02_suspend_enter,
};

int __init mcom02_suspend_init(void)
{
	suspend_set_ops(&mcom02_suspend_ops);
	return 0;
}
