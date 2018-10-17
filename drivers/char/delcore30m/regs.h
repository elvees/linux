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

#define DELCORE30M_INVAR			0x0FC
#define DELCORE30M_DCSR				0x100
#define DELCORE30M_SR				0x104
#define DELCORE30M_IMASKR			0x114
#define DELCORE30M_PC				0x120
#define DELCORE30M_LA				0x128
#define DELCORE30M_CSL				0x12C
#define DELCORE30M_LC				0x130
#define DELCORE30M_CSH				0x134
#define DELCORE30M_SP				0x138
#define DELCORE30M_QMASKR0			0x170

/* ---------SDMA commands----------- */
#define SDMA_DMAMOVE_SAR			0x00BC
#define SDMA_DMAMOVE_CCR			0x01BC
#define SDMA_DMAMOVE_DAR			0x02BC

#define SDMA_DMAADDH_DAR			0x56
#define SDMA_DMAADDH_SAR			0x54

#define SDMA_DMALP(loop_counter)		(0x20 + ((loop_counter) << 1))
#define SDMA_DMALPEND(loop_counter)		(0x38 + ((loop_counter) << 2))

#define SDMA_DMAWFE				0x36
#define SDMA_DMASEV				0x34

#define SDMA_DMAWMB				0x13
#define SDMA_DMARMB				0x12

#define SDMA_DMAST				0x0B
#define SDMA_DMALD				0x07

#define SDMA_DMAKILL				0x1
#define SDMA_DMAEND				0x0
/*---------------------------------*/

/* ----------SDMA registers------------- */
#define INTEN					0x020
#define CHANNEL_STATUS(x)			(0x100 + (8 * (x)))
#define DBGSTATUS				0xD00
#define DBGCMD					0xD04
#define DBGINST0				0xD08
#define DBGINST1				0xD0C
/* ------------------------------------- */

#define SPINLOCK_REG_OFFSET			0x804

#define DELCORE30M_CSR_PMCONFIG(x)		((x) << 0x2)
#define DELCORE30M_QSTR_MASK			(GENMASK(3, 0) | GENMASK(11, 8))
#define DELCORE30M_QSTR_CORE_MASK(x)		(0xF << (8 * (x)))
#define DELCORE30M_DCSR_PI			BIT(0)
#define DELCORE30M_DCSR_SE			BIT(1)
#define DELCORE30M_DCSR_BRK			BIT(2)
#define DELCORE30M_DCSR_RUN			BIT(14)
#define DELCORE30M_CSR_SYNSTART			BIT(0)
#define DELCORE30M_CSR_PMCONFIG_MASK		GENMASK(3, 2)

#define SDMA_BURST_SIZE(ccr)			(1 << (((ccr) >> 1) & 0x7))

#endif
