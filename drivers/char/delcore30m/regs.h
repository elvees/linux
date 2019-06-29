/* SPDX-License-Identifier: GPL-2.0+ */
/*
 *  Copyright 2018 RnD Center "ELVEES", JSC
 */

#ifndef _REGS_H_
#define _REGS_H_

/*--------common registers-----------*/
#define DELCORE30M_MASKR_DSP			0x000
#define DELCORE30M_QSTR_DSP			0x004
#define DELCORE30M_CSR_DSP			0x008
/*-----------------------------------*/

/*---------private registers-----------*/
#define DELCORE30M_A0				0x080
#define DELCORE30M_A1				0x084
#define DELCORE30M_A5				0x094
#define DELCORE30M_A6				0x098
#define DELCORE30M_A7				0x09C

#define DELCORE30M_R0				0x000
#define DELCORE30M_R2				0x004
#define DELCORE30M_R4				0x008

#define DELCORE30M_DCSR				0x100
#define DELCORE30M_SR				0x104
#define DELCORE30M_PC				0x120
#define DELCORE30M_LA				0x128
#define DELCORE30M_CSL				0x12C
#define DELCORE30M_LC				0x130
#define DELCORE30M_CSH				0x134
#define DELCORE30M_SP				0x138
/*-------------------------------------*/

#define DELCORE30M_CSR_PMCONFIG(x)		((x) << 0x2)
#define DELCORE30M_QSTR_MASK			(GENMASK(3, 0) | GENMASK(11, 8))
#define DELCORE30M_QSTR_CORE_MASK(x)		(0xF << (8 * (x)))
#define DELCORE30M_DCSR_PI			BIT(0)
#define DELCORE30M_DCSR_SE			BIT(1)
#define DELCORE30M_DCSR_BRK			BIT(2)
#define DELCORE30M_DCSR_RUN			BIT(14)
#define DELCORE30M_CSR_SYNSTART			BIT(0)
#define DELCORE30M_CSR_PMCONFIG_MASK		GENMASK(3, 2)

#endif
