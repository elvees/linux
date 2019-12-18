/*
 * Copyright 2018 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef _REGS_H_
#define _REGS_H_

#define SWIC_DMA_RX_DESC			0x1000
#define SWIC_DMA_RX_DATA			0x1040
#define SWIC_DMA_TX_DESC			0x1080
#define SWIC_DMA_TX_DATA			0x10C0

#define SWIC_DMA_CSR				0x00
#define SWIC_DMA_CSR_RUN			BIT(0)
#define SWIC_DMA_CSR_CHEN			BIT(12)
#define SWIC_DMA_CSR_IM				BIT(13)
#define SWIC_DMA_CSR_DONE			BIT(15)
#define SWIC_DMA_CSR_WN				GENMASK(5, 2)
#define SWIC_DMA_CSR_WCX			GENMASK(31, 16)

#define SWIC_DMA_CP				0x04

#define SWIC_DMA_IR				0x08

#define SWIC_DMA_RUN				0x0C

#define SWIC_HW_VER				0x00

#define SWIC_STATUS				0x04
#define SWIC_STATUS_DC_ERR			BIT(0)
#define SWIC_STATUS_P_ERR			BIT(1)
#define SWIC_STATUS_ESC_ERR			BIT(2)
#define SWIC_STATUS_CREDIT_ERR			BIT(3)
#define SWIC_STATUS_ERR				(SWIC_STATUS_DC_ERR | \
						SWIC_STATUS_P_ERR | \
						SWIC_STATUS_ESC_ERR | \
						SWIC_STATUS_CREDIT_ERR)
#define SWIC_STATUS_LINK_STATE			GENMASK(7, 5)
#define SWIC_STATUS_LINK_STATE_RESET		0
#define SWIC_STATUS_LINK_STATE_WAIT		BIT(5)
#define SWIC_STATUS_LINK_STATE_READY		BIT(6)
#define SWIC_STATUS_LINK_STATE_STARTED		GENMASK(6, 5)
#define SWIC_STATUS_LINK_STATE_CONNECTING	BIT(7)
#define SWIC_STATUS_LINK_STATE_RUN		(BIT(5) | BIT(7))
#define SWIC_STATUS_GOT_FIRST_BIT		BIT(12)
#define SWIC_STATUS_CONNECTED			BIT(13)

#define SWIC_MODE_CR				0x0C
#define SWIC_MODE_CR_LINK_DISABLE		BIT(0)
#define SWIC_MODE_CR_AUTO_START			BIT(1)
#define SWIC_MODE_CR_LINK_START			BIT(2)
#define SWIC_MODE_CR_LINK_RESET			BIT(6)
#define SWIC_MODE_CR_COEFF_10_WR		BIT(14)
#define SWIC_MODE_CR_LINK_MASK			BIT(18)
#define SWIC_MODE_CR_ERR_MASK			BIT(19)

#define SWIC_TX_SPEED				0x10
#define SWIC_TX_SPEED_TX_SPEED			GENMASK(7, 0)
#define SWIC_TX_SPEED_PLL_TX_EN			BIT(8)
#define SWIC_TX_SPEED_LVDS_EN			BIT(9)
#define SWIC_TX_SPEED_COEFF_10			GENMASK(28, 20)

#define SWIC_RX_SPEED				0x18

#define SWIC_CNT_RX_PACK			0x20

#define SET_FIELD(mask, val) (((val) << (ffs(mask) - 1)) & (mask))
#define GET_FIELD(reg, mask) (((reg) & (mask)) >> (ffs(mask) - 1))

#endif
