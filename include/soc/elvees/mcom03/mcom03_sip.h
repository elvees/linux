/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright 2022 RnD Center "ELVEES", JSC
 *
 */

#ifndef __SOC_MCOM03_SIP_H
#define __SOC_MCOM03_SIP_H

#include <linux/arm-smccc.h>

#define MCOM03_SIP_WDT							0xC2000001
#define MCOM03_SIP_WDT_IS_ENABLE				0x01
#define MCOM03_SIP_WDT_START					0x02
#define MCOM03_SIP_WDT_PING						0x03
#define MCOM03_SIP_WDT_SET_TIMEOUT_S			0x04
#define MCOM03_SIP_WDT_GET_TIMEOUT_S			0x05
#define MCOM03_SIP_WDT_GET_MAX_TIMEOUT_S		0x06
#define MCOM03_SIP_WDT_GET_MIN_TIMEOUT_S		0x07

static inline unsigned long mcom03_sip_smccc_smc(unsigned long a0, unsigned long a1,
			    unsigned long a2, unsigned long a3,
			    unsigned long a4, unsigned long a5,
			    unsigned long a6, unsigned long a7)
{
	struct arm_smccc_res res = {0};

	arm_smccc_smc(a0, a1, a2, a3, a4, a5, a6, a7, &res);

	return res.a0;
}

#endif
