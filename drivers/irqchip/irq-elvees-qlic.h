/* SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note */
/*
 * Copyright 2020 RnD Center "ELVEES", JSC
 */
#ifndef _LINUX_QLIC_H
#define _LINUX_QLIC_H

#define QLIC_PRI0		0x0
#define QLIC_PRI_NEXT		0x4

#define QLIC_ENS0		0x1000
#define QLIC_ENSNEXT		0x80

#define QLIC_THD0		0x10000
#define QLIC_THDNEXT		0x1000

#define QLIC_CC0		0x10004
#define QLIC_CCNEXT		0x1000

#define QLIC_MAX_PRIO		7
#define QLIC_MAX_TARGET		17

#define QLIC_INTERRUPT_PROPERTIES 3

#endif
