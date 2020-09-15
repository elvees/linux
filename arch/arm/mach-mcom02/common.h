/*
 *  Copyright 2017-2019 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef __ARCH_MCOM02_COMMON_H__
#define __ARCH_MCOM02_COMMON_H__

#define PMCTR_SYS_PWR_STATUS  0x0c
#define PMCTR_SYS_PWR_IMASK   0x10
#define PMCTR_SYS_PWR_IRSTAT  0x14
#define PMCTR_SYS_PWR_ISTAT   0x18
#define PMCTR_SYS_PWR_ICLR    0x1c
#define PMCTR_SYS_PWR_DELAY   0x20

#define PMCTR_DDR_PIN_RET     0x24
#define PMCTR_DDR_INIT_END    0x28
#define PMCTR_WARM_RST_EN     0x2c
#define PMCTR_SW_RST          0x40
#define PMCTR_WARM_RST_STATUS 0x44
#define PMCTR_PDM_RST_STATUS  0x48
#define PMCTR_NVMODE          0x4c

#define PMCTR_CPU0_WKP_MASK0  0x50
#define PMCTR_CPU0_WKP_MASK1  0x54
#define PMCTR_CPU0_WKP_MASK2  0x58
#define PMCTR_CPU0_WKP_MASK3  0x5c

#define PMCTR_CPU1_WKP_MASK0  0x60
#define PMCTR_CPU1_WKP_MASK1  0x64
#define PMCTR_CPU1_WKP_MASK2  0x68
#define PMCTR_CPU1_WKP_MASK3  0x6c

#define PMCTR_ALWAYS_MISC0    0x70
#define PMCTR_ALWAYS_MISC1    0x74
#define PMCTR_WARM_BOOT_OVRD  0x78

#define PMCTR_CORE_PWR_UP     0x80
#define PMCTR_CORE_PWR_DOWN   0x84
#define PMCTR_CORE_PWR_STATUS 0x8c
#define PMCTR_CORE_PWR_IMASK  0x90
#define PMCTR_CORE_PWR_IRSTAT 0x94
#define PMCTR_CORE_PWR_ISTAT  0x98
#define PMCTR_CORE_PWR_ICLR   0x9c
#define PMCTR_CORE_PWR_DELAY  0xa0

#define PMCTR_SYS_PWR_INT     122
#define PMCTR_CORE_PWR_INT    123

#define PMCTR_CPU_WKP_BASE(cpu)     (PMCTR_CPU0_WKP_MASK0 + ((cpu) * 0x10))
#define PMCTR_CPU_WKP_MASK(cpu, n)  (PMCTR_CPU_WKP_BASE(cpu) + (((n) / 32) * 4))
#define PMCTR_CPU_INT_BIT(n)        (1 << ((n) % 32))

#define SMCTR_BOOT                0x00
#define SMCTR_BOOT_REMAP          0x04

#define SMCTR_BOOT_REMAP_NORMPORT 0x0
#define SMCTR_BOOT_REMAP_SPRAM    0x1
#define SMCTR_BOOT_REMAP_SPL      0x2
#define SMCTR_BOOT_REMAP_BOOTROM  0x3

#define SMCTR_MPU_CFGNMFI         0x08
#define SMCTR_DDR_REMAP           0x0c
#define SMCTR_ACP_CTL             0x20
#define SMCTR_MIPI_MUX            0x24
#define SMCTR_CHIP_ID             0x28
#define SMCTR_CHIP_CONFIG         0x2c

#define SMCTR_EMA_ARM             0x30
#define SMCTR_EMA_L2              0x34
#define SMCTR_EMA_DSP             0x38
#define SMCTR_EMA_CORE            0x3c

#define SMCTR_IOPULL_CTR          0x40
#define SMCTR_COMM_DCLOCK         0x44

extern char mcom02_secondary_trampoline;
extern char mcom02_secondary_trampoline_end;

extern unsigned long mcom02_secondary_boot_vector;
extern void mcom02_secondary_startup(void);

int mcom02_init_pmctr(void);

#endif
