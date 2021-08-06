/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * Copyright 2021 RnD Center "ELVEES", JSC
 */

#ifndef __DT_BINDINGS_ELVEES_MCOM03_H
#define __DT_BINDINGS_ELVEES_MCOM03_H

/* The order of the subsystems corresponds to the order in the universal
 * register block (URB) of the service subsystem.
 */
#define MCOM03_SUBSYSTEM_CPU		0
#define MCOM03_SUBSYSTEM_SDR		1
#define MCOM03_SUBSYSTEM_MEDIA		2
#define MCOM03_SUBSYSTEM_CORE		3
#define MCOM03_SUBSYSTEM_HSPERIPH	4
#define MCOM03_SUBSYSTEM_LSPERIPH0	5
#define MCOM03_SUBSYSTEM_LSPERIPH1	6
#define MCOM03_SUBSYSTEM_DDR		7
#define MCOM03_SUBSYSTEM_TOP		8
#define MCOM03_SUBSYSTEM_RISC0		9
#define MCOM03_SUBSYSTEM_SERVICE	10
#define MCOM03_SUBSYSTEM_MAX		11

#define SDR_RST_PCI0			0
#define SDR_RST_PCI1			1
#define SDR_RST_DSP0			2
#define SDR_RST_DSP1			3
#define SDR_RST_RISC1			4

#define MEDIA_RST_ISP			0
#define MEDIA_RST_GPU			1
#define MEDIA_RST_VPU			2
#define MEDIA_RST_DISPLAY		3

#define HSPERIPH_RST_QSPI		2
#define HSPERIPH_RST_NFC		3
#define HSPERIPH_RST_SDMMC0		4
#define HSPERIPH_RST_SDMMC1		5
#define HSPERIPH_RST_USB0		6
#define HSPERIPH_RST_USB1		7
#define HSPERIPH_RST_EMAC0		8
#define HSPERIPH_RST_EMAC1		9

#endif /* __DT_BINDINGS_ELVEES_MCOM03_H */
