/*
 *  Copyright 2018 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ARCH_MCOM02_SUSPEND_H
#define __ARCH_MCOM02_SUSPEND_H

#ifdef CONFIG_SUSPEND
int __init mcom02_suspend_init(void);
#else
static int __init mcom02_suspend_init(void) { }
#endif

#endif
