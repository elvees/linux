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
#define DELCORE30M_QSTR_STOPPED_MASK		(BIT(3) | BIT(11))
#define DELCORE30M_QSTR_STOP_OFFSET(x)		(3 + 8 * (x))
#define DELCORE30M_QSTR_STOPPED_CORE_MASK(x)	BIT(3 + 8 * (x))
#define DELCORE30M_QSTR_STOPPED_CORES(x)	((x) & \
						 DELCORE30M_QSTR_STOPPED_MASK)
#define DELCORE30M_QSTR_MASK(x)			(0xF << (8 * (x)))
#define DELCORE30M_DCSR_RUN			BIT(14)
#define DELCORE30M_CSR_SYNSTART			BIT(0)
#define DELCORE30M_CSR_PMCONFIG_MASK		GENMASK(3, 2)

#endif
