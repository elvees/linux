/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VINC_DEV_H
#define VINC_DEV_H

#include <linux/interrupt.h>
#include <linux/vinc.h>

#include <media/soc_camera.h>
#include <media/soc_mediabus.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-mediabus.h>

#define MODULE_NAME "vinc"

/* VINC registers */
#define ID				0x0
#define AXI_MASTER_CFG			0x4
#define AXI_MASTER_STATUS		0x8
#define INTERRUPT			0x20
#define INTERRUPT_RESET			0x24
#define INTERRUPT_MASK			0x28

#define PPORT_BASE(p)			(0x80 * (p) + 0x100)
#define PPORT_CFG(p)			(PPORT_BASE(p) + 0x0)

#define PINTERFACE_BASE(p)		(0x100 * (p) + 0x300)
#define PINTERFACE_CFG(p)		(PINTERFACE_BASE(p) + 0x0)
#define PINTERFACE_CCMOV(p, c)		(PINTERFACE_BASE(p) + 0x20 + ((c) * 4))
#define PINTERFACE_HVFSYNC(p)		(PINTERFACE_BASE(p) + 0x80)

#define PPORT_INP_MUX_CFG		0x700
#define PPORT_STATUS			0x704
#define PPORT_TEST_SRC			0x720

#define CMOS_BASE(c)			(0x20 * (c) + 0x780)
#define CMOS_CTR(c)			(CMOS_BASE(c) + 0x0)
#define CMOS_TIMER_HIGH(c)		(CMOS_BASE(c) + 0x4)
#define CMOS_TIMER_LOW(c)		(CMOS_BASE(c) + 0x8)

#define CSI2_PORT_BASE(c)		(0x200 * (c) + 0x800)

#define CSI2_DEVICE_READY(c)		(CSI2_PORT_BASE(c) + 0x0)
#define CSI2_INTR(c)			(CSI2_PORT_BASE(c) + 0x4)
#define CSI2_INTR_EN(c)			(CSI2_PORT_BASE(c) + 0x8)
#define CSI2_FUNC_PROG(c)		(CSI2_PORT_BASE(c) + 0xc)
#define CSI2_DPHY_TIM3(c)		(CSI2_PORT_BASE(c) + 0x10)
#define CSI2_DPHY_TIM1(c)		(CSI2_PORT_BASE(c) + 0x14)
#define CSI2_DPHY_TIM2(c)		(CSI2_PORT_BASE(c) + 0x18)
#define CSI2_FSLS(c)			(CSI2_PORT_BASE(c) + 0x1c)
#define CSI2_LSDV(c)			(CSI2_PORT_BASE(c) + 0x20)
#define CSI2_DVLE(c)			(CSI2_PORT_BASE(c) + 0x24)
#define CSI2_LEFE(c)			(CSI2_PORT_BASE(c) + 0x28)
#define CSI2_FEFS(c)			(CSI2_PORT_BASE(c) + 0x2c)
#define CSI2_LELS(c)			(CSI2_PORT_BASE(c) + 0x30)
#define CSI2_TWO_PIX_EN(c)		(CSI2_PORT_BASE(c) + 0x34)
#define CSI2_TRIM0(c)			(CSI2_PORT_BASE(c) + 0x38)
#define CSI2_SYNC_COUNT(c)		(CSI2_PORT_BASE(c) + 0x3c)
#define CSI2_RCV_COUNT(c)		(CSI2_PORT_BASE(c) + 0x40)
#define CSI2_TRIM1(c)			(CSI2_PORT_BASE(c) + 0x44)
#define CSI2_VCH01(c)			(CSI2_PORT_BASE(c) + 0x48)
#define CSI2_VCH02(c)			(CSI2_PORT_BASE(c) + 0x4c)
#define CSI2_VCH11(c)			(CSI2_PORT_BASE(c) + 0x50)
#define CSI2_VCH12(c)			(CSI2_PORT_BASE(c) + 0x54)
#define CSI2_VCH21(c)			(CSI2_PORT_BASE(c) + 0x58)
#define CSI2_VCH22(c)			(CSI2_PORT_BASE(c) + 0x5c)
#define CSI2_VCH31(c)			(CSI2_PORT_BASE(c) + 0x60)
#define CSI2_VCH32(c)			(CSI2_PORT_BASE(c) + 0x64)
#define CSI2_RAW8(c)			(CSI2_PORT_BASE(c) + 0x68)
#define CSI2_TRIM2(c)			(CSI2_PORT_BASE(c) + 0x6c)
#define CSI2_TRIM3(c)			(CSI2_PORT_BASE(c) + 0x70)
#define CSI2_LOOP_BACK(c)		(CSI2_PORT_BASE(c) + 0x80)

#define CSI2_PORT_SYS_CTR(c)		(CSI2_PORT_BASE(c) + 0x180)
#define CSI2_PORT_SYS_STATUS(c)		(CSI2_PORT_BASE(c) + 0x184)
#define CSI2_PORT_SYS_LB_CTRL(c)	(CSI2_PORT_BASE(c) + 0x18c)
#define CSI2_PORT_GENFIFO_CTR(c)	(CSI2_PORT_BASE(c) + 0x1c0)
#define CSI2_PORT_GENFIFO_DATA(c)	(CSI2_PORT_BASE(c) + 0x1c4)
#define CSI2_PORT_GENFIFO_STATUS(c)	(CSI2_PORT_BASE(c) + 0x1c8)

#define STREAM_BASE(s)			(0x400 * (s) + 0x1000)
#define STREAM_INP_CFG(s)		(STREAM_BASE(s) + 0x0)
#define STREAM_INP_HCROP_CTR(s)		(STREAM_BASE(s) + 0x4)
#define STREAM_INP_VCROP_CTR(s)		(STREAM_BASE(s) + 0x8)
#define STREAM_INP_VCROP_ODD_CTR(s)	(STREAM_BASE(s) + 0xc)
#define STREAM_INP_DECIM_CTR(s)		(STREAM_BASE(s) + 0x10)
#define STREAM_INP_MIN_SPACE_CTR(s)	(STREAM_BASE(s) + 0x14)
#define STREAM_STATUS(s)		(STREAM_BASE(s) + 0x20)
#define STREAM_INTERRUPT(s)		(STREAM_BASE(s) + 0x40)
#define STREAM_INTERRUPT_RESET(s)	(STREAM_BASE(s) + 0x44)
#define STREAM_INTERRUPT_MASK(s)	(STREAM_BASE(s) + 0x48)
#define STREAM_PROC_CFG(s)		(STREAM_BASE(s) + 0x50)
#define STREAM_PROC_CTR(s)		(STREAM_BASE(s) + 0x58)
#define STREAM_PROC_CLEAR(s)		(STREAM_BASE(s) + 0x60)
#define STREAM_PROC_BP_MAP_CTR(s)	(STREAM_BASE(s) + 0x68)
#define STREAM_PROC_BP_MAP_DATA(s)	(STREAM_BASE(s) + 0x6c)
#define STREAM_PROC_BP_BAD_LINE(s, l)	(STREAM_BASE(s) + 0x70 + ((l) * 4))
#define STREAM_PROC_BP_BAD_COLUMN(s, c)	(STREAM_BASE(s) + 0x90 + ((c) * 4))
#define STREAM_PROC_DR_CTR(s)		(STREAM_BASE(s) + 0xb0)
#define STREAM_PROC_DR_DATA(s)		(STREAM_BASE(s) + 0xb4)
#define STREAM_PROC_DR_COUNT(s)		(STREAM_BASE(s) + 0xb8)
#define STREAM_PROC_CC_COEFF(s, c)	(STREAM_BASE(s) + 0xc0 + ((c) * 4))
#define STREAM_PROC_CC_OFFSET(s, o)	(STREAM_BASE(s) + 0xd4 + ((o) * 4))
#define STREAM_PROC_GC_CTR(s)		(STREAM_BASE(s) + 0xe0)
#define STREAM_PROC_GC_DATA(s)		(STREAM_BASE(s) + 0xe4)
#define STREAM_PROC_CT_COEFF(s, c)	(STREAM_BASE(s) + 0xf0 + ((c) * 4))
#define STREAM_PROC_CT_OFFSET(s, o)	(STREAM_BASE(s) + 0x104 + ((o) * 4))

#define STREAM_PROC_STAT_ZONE_BASE(s, z) (0xc * (z) + STREAM_BASE(s) + 0x110)
#define STREAM_PROC_STAT_ZONE_LT(s, z)	(STREAM_PROC_STAT_ZONE_BASE(s, z) + 0x0)
#define STREAM_PROC_STAT_ZONE_RB(s, z)	(STREAM_PROC_STAT_ZONE_BASE(s, z) + 0x4)

#define STREAM_PROC_STAT_CTR(s)		(STREAM_BASE(s) + 0x140)
#define STREAM_PROC_STAT_DATA(s)	(STREAM_BASE(s) + 0x144)
#define STREAM_PROC_STAT_MIN(s)		(STREAM_BASE(s) + 0x148)
#define STREAM_PROC_STAT_MAX(s)		(STREAM_BASE(s) + 0x14c)
#define STREAM_PROC_STAT_SUM_B(s)	(STREAM_BASE(s) + 0x150)
#define STREAM_PROC_STAT_SUM_G(s)	(STREAM_BASE(s) + 0x154)
#define STREAM_PROC_STAT_SUM_R(s)	(STREAM_BASE(s) + 0x158)
#define STREAM_PROC_STAT_SUM2_B(s)	(STREAM_BASE(s) + 0x15c)
#define STREAM_PROC_STAT_SUM2_G(s)	(STREAM_BASE(s) + 0x160)
#define STREAM_PROC_STAT_SUM2_R(s)	(STREAM_BASE(s) + 0x164)
#define STREAM_PROC_STAT_SUM2_HI(s)	(STREAM_BASE(s) + 0x168)
#define STREAM_PROC_STAT_TH(s)		(STREAM_BASE(s) + 0x16c)
#define STREAM_PROC_STAT_HSOBEL(s)	(STREAM_BASE(s) + 0x170)
#define STREAM_PROC_STAT_VSOBEL(s)	(STREAM_BASE(s) + 0x174)
#define STREAM_PROC_STAT_LSOBEL(s)	(STREAM_BASE(s) + 0x178)
#define STREAM_PROC_STAT_RSOBEL(s)	(STREAM_BASE(s) + 0x17c)

#define STREAM_DMA_BASE(s, d)		(0x100 * (d) + STREAM_BASE(s) + 0x200)
#define STREAM_DMA_FBUF_CFG(s, d)	(STREAM_DMA_BASE(s, d) + 0x0)
#define STREAM_DMA_PIXEL_FMT(s, d)	(STREAM_DMA_BASE(s, d) + 0x4)
#define STREAM_DMA_FBUF_HORIZ(s, d)	(STREAM_DMA_BASE(s, d) + 0x8)
#define STREAM_DMA_FBUF_VERT(s, d)	(STREAM_DMA_BASE(s, d) + 0xc)
#define STREAM_DMA_FBUF_VERT_ODD(s, d)	(STREAM_DMA_BASE(s, d) + 0x10)
#define STREAM_DMA_FBUF_DECIM(s, d)	(STREAM_DMA_BASE(s, d) + 0x14)

#define STREAM_DMA_FBUF_BASE(s, d, b)	(0x10 * (b) + STREAM_DMA_BASE(s, d)\
					+ 0x20)
#define STREAM_DMA_FBUF_LSTEP(s, d, b)	(STREAM_DMA_FBUF_BASE(s, d, b) + 0x4)
#define STREAM_DMA_FBUF_FSTEP(s, d, b)	(STREAM_DMA_FBUF_BASE(s, d, b) + 0x8)

#define STREAM_DMA_WR_CTR(s, d)		(STREAM_DMA_BASE(s, d) + 0x60)
#define STREAM_DMA_WR_STATUS(s, d)	(STREAM_DMA_BASE(s, d) + 0x64)
#define STREAM_DMA_WR_COUNT(s, d, c)	(STREAM_DMA_BASE(s, d) + 0x68\
					+ ((c) * 4))
#define STREAM_DMA_CUR_ADDR(s, d, a)	(STREAM_DMA_BASE(s, d) + 0x70\
					+ ((a) * 4))
#define STREAM_DMA_TEST_DATA(s, d)	(STREAM_DMA_BASE(s, d) + 0x80)
#define STREAM_DMA_TEST_CTR(s, d)	(STREAM_DMA_BASE(s, d) + 0x84)

#define STREAM_CTR			0x3f00

#define CC_CT_OFFSET_COEFF8		0x10
#define CC_CT_OFFSET_OFFSET0_1		0x14
#define CC_CT_OFFSET_OFFSET2		0x18

/* Bits for AXI_MASTER_CFG register */
#define AXI_MASTER_CFG_MAX_BURST(v)	((v) & 0x7)
#define AXI_MASTER_CFG_MAX_WR_ID(v)	(((v) & 0xF) << 4)
#define AXI_MASTER_CFG_BUF_LAYOUT(v)	(((v) & 0xF) << 8)
#define AXI_MASTER_CFG_4K_BOUND_EN	BIT(19)
#define AXI_MASTER_CFG_GLOBAL_EN	BIT(31)

/* Bits for INTERRUPT register */
#define INTERRUPT_AXI_ERROR		BIT(0)
#define INTERRUPT_PPORT_ERROR		BIT(12)
#define INTERRUPT_CSI0			BIT(16)
#define INTERRUPT_CSI0_GEN		BIT(17)
#define INTERRUPT_CSI1			BIT(18)
#define INTERRUPT_CSI1_GEN		BIT(19)

/* Bits for PORT_CFG register */
#define PORT_CFG_PIXEL_MODE(v)		((v) & 0x3)
#define PORT_CFG_PCLK_DIV		BIT(2)
#define PORT_CFG_PCLK_NEG_DIV		BIT(3)
#define PORT_CFG_PCLK_HALF_CLK		BIT(4)
#define PORT_CFG_DIGITAL_DELAY(v)	(((v) & 0x7) << 5)
#define PORT_CFG_VIN_SRC(v)		(((v) & 0x3) << 12)

/* Bits for PINTERFACE_CFG register */
#define PINTERFACE_CFG_CYCLE_NUM(v)	((v) & 0x7)
#define PINTERFACE_CFG_PIXEL_NUM_EVEN(v)	(((v) & 0x3) << 4)
#define PINTERFACE_CFG_PIXEL_NUM_ODD(v)	(((v) & 0x3) << 8)
#define PINTERFACE_CFG_PV_ALGN_MODE(v)	(((v) & 0xF) << 12)
#define PINTERFACE_CFG_PORT_NUM_SYNC(v)	(((v) & 0x3) << 16)
#define PINTERFACE_CFG_EMB_SYNC		BIT(18)
#define PINTERFACE_CFG_EMB_SYNC_CORR	BIT(19)
#define PINTERFACE_CFG_PHASE_CORR(v)	(((v) & 0x7) << 20)
#define PINTERFACE_CFG_FORW_H(v)	(((v) & 0xFF) << 24)

/* Bits for PINTERFACE_HVFSYNC register */
#define PINTERFACE_HVFSYNC_INVERS_H	BIT(0)
#define PINTERFACE_HVFSYNC_INVERS_V	BIT(1)
#define PINTERFACE_HVFSYNC_INVERS_F	BIT(2)
#define PINTERFACE_HVFSYNC_BUILT_MODE(v)	(((v) & 0x7) << 3)
#define PINTERFACE_HVFSYNC_DELAY_F_EN	BIT(6)
#define PINTERFACE_HVFSYNC_DELAY_V(v)	(((v) & 0x1F) << 8)
#define PINTERFACE_HVFSYNC_DELAY_F(v)	(((v) & 0x3F) << 16)
#define PINTERFACE_HVFSYNC_PRE_DELAY_V(v)	(((v) & 0x1F) << 24)
#define PINTERFACE_HVFSYNC_DELAY_VF_ODD_OFS(v)	(((v) & 0x7) << 29)

/* Bits for PPORT_INP_MUX_CFG register */
#define PP_VIN_TYPE(v)			((v) & 0x7)
#define PP_VIN_CLK			BIT(8)

/* Bits for CMOS_CTR register */
#define CMOS_CTR_RESET			BIT(0)
#define CMOS_CTR_PCLK_EN		BIT(1)
#define CMOS_CTR_PCLK_SRC(v)		(((v) & 0x3) << 2)
#define CMOS_CTR_CLK_DIV(v)		(((v) & 0xF) << 4)
#define CMOS_CTR_FSYNC_EN		BIT(8)

/* Bits for CSI2_PORT_SYS_CTR register */
#define CSI2_PORT_SYS_CTR_ENABLE	BIT(0)
#define CSI2_PORT_SYS_CTR_TWO_PORTS	BIT(1)
#define CSI2_PORT_SYS_CTR_FREQ_RATIO(v)	(((v) & 0x3F) << 8)

/* Bits for CSI2_DPHY_TIM1 register */
#define CSI2_TIM1_DLN_CNT_HS_PREP(v)	((v) & 0xFF)
#define CSI2_TIM1_DLN_CNT_HS_ZERO(v)	(((v) & 0xFF) << 8)
#define CSI2_TIM1_DLN_CNT_HS_TRAIL(v)	(((v) & 0xFF) << 16)
#define CSI2_TIM1_DLN_CNT_HS_EXIT(v)	(((v) & 0xFF) << 24)

/* Bits for CSI2_DPHY_TIM2 register */
#define CSI2_TIM2_CLN_CNT_HS_PREP(v)	((v) & 0xFF)
#define CSI2_TIM2_CLN_CNT_HS_ZERO(v)	(((v) & 0xFF) << 8)
#define CSI2_TIM2_CLN_CNT_HS_TRAIL(v)	(((v) & 0xFF) << 16)
#define CSI2_TIM2_CLN_CNT_HS_EXIT(v)	(((v) & 0xFF) << 24)

/* Bits for CSI2_DPHY_TIM3 register */
#define CSI2_TIM3_CLN_CNT_LPX(v)	((v) & 0xFF)
#define CSI2_TIM3_DLN_CNT_LPX(v)	(((v) & 0xFF) << 8)
#define CSI2_TIM3_CLN_CNT_PLL(v)	(((v) & 0xFFFF) << 16)

/* Bits for STREAM_INP_DECIM_CTR register */
#define STREAM_INP_DECIM_HDECIM(v)	((v) & 0xF)
#define STREAM_INP_DECIM_VDECIM(v)	(((v) & 0xF) << 4)
#define STREAM_INP_DECIM_FDECIM(v)	(((v) & 0x3F) << 8)
#define STREAM_INP_DECIM_INTERLACE_MODE	BIT(16)

/* Bits for STREAM_INTERRUPT register */
#define STREAM_INTERRUPT_PROC		BIT(0)
#define STREAM_INTERRUPT_DMA0		BIT(8)
#define STREAM_INTERRUPT_DMA1		BIT(9)

/* Bits for STREAM_PROC_CFG register */
#define STREAM_PROC_CFG_BPC_EN		BIT(0)
#define STREAM_PROC_CFG_ADR_EN		BIT(1)
#define STREAM_PROC_CFG_CFA_EN		BIT(2)
#define STREAM_PROC_CFG_CC_EN		BIT(3)
#define STREAM_PROC_CFG_GC_EN		BIT(4)
#define STREAM_PROC_CFG_CT_EN		BIT(5)
#define STREAM_PROC_CFG_422TO444_EN	BIT(6)
#define STREAM_PROC_CFG_444TO422_EN	BIT(7)
#define STREAM_PROC_CFG_422TO420_EN	BIT(8)
#define STREAM_PROC_CFG_STT_EN(v)	(((v) & 0x7) << 9)
#define STREAM_PROC_CFG_STT_ZONE_OFFSET	12
#define STREAM_PROC_CFG_STT_ZONE_EN(v)	(((v) & 0xF) << 12)
#define STREAM_PROC_CFG_IM_EN		BIT(16)
#define STREAM_PROC_CFG_CT_SRC(v)	(((v) & 0x1) << 23)
#define STREAM_PROC_CFG_444TO422_SRC(v)	(((v) & 0x1) << 24)
#define STREAM_PROC_CFG_422TO420_SRC(v)	(((v) & 0x1) << 25)
#define STREAM_PROC_CFG_DMA0_SRC(v)	(((v) & 0x7) << 26)
#define STREAM_PROC_CFG_DMA1_SRC(v)	(((v) & 0x7) << 29)

#define STT_EN_HIST			0x1
#define STT_EN_AF			0x2
#define STT_EN_ADD			0x4

#define DMA_SRC_CROP			0
#define DMA_SRC_DR			0x1
#define DMA_SRC_444			0x2
#define DMA_SRC_CT			0x3
#define DMA_SRC_422			0x4
#define DMA_SRC_420			0x5
#define DMA_SRC_IS			0x6
#define DMA_SRC_MASK			0x7

/* Bits for STREAM_PROC_CTR register */
#define STREAM_PROC_CTR_BAYER_MONO	BIT(0)
#define STREAM_PROC_CTR_BAYER_MODE(v)	(((v) & 0x3) << 1)
#define STREAM_PROC_CTR_422TO444ALG(v)	(((v) & 0x3) << 6)
#define STREAM_PROC_CTR_422TO444FILL	BIT(8)
#define STREAM_PROC_CTR_444TO422ALG(v)	(((v) & 0x3) << 9)
#define STREAM_PROC_CTR_444TO422FILL	BIT(11)
#define STREAM_PROC_CTR_422TO420ALG	BIT(12)
#define STREAM_PROC_CTR_HIST_THR	BIT(13)
#define STREAM_PROC_CTR_AF_THR		BIT(14)
#define STREAM_PROC_CTR_ADD_THR		BIT(15)
#define STREAM_PROC_CTR_AF_COLOR(v)	(((v) & 0x3) << 16)
#define STREAM_PROC_CTR_IM_COLOR(v)	(((v) & 0x3) << 21)

/* Bits for STREAM_PROC_CLEAR register */
#define STREAM_PROC_CLEAR_AF_CLR	BIT(0)
#define STREAM_PROC_CLEAR_ADD_CLR	BIT(1)
#define STREAM_PROC_CLEAR_THR_CLR	BIT(2)

/* Bits for STREAM_PROC_STAT_CTR register */
#define STREAM_PROC_STAT_CTR_ADDR_HIST(v)	((v) & 0xFF)
#define STREAM_PROC_STAT_CTR_NUM_ZONE(v)	(((v) & 0x3) << 16)
#define STREAM_PROC_STAT_CTR_COLOR_HIST(v)	(((v) & 0x3) << 18)

/* Bits for STREAM_DMA_PIXEL_FMT register */
#define STREAM_DMA_PIXEL_FMT_PLANES(v)	((v) & 0xF)
#define STREAM_DMA_PIXEL_FMT_WIDTH(v)	(((v) & 0xF) << 4)
#define STREAM_DMA_PIXEL_FMT_FORMAT(v)	(((v) & 0xF) << 8)
#define STREAM_DMA_PIXEL_FMT_WITH_ALPHA	BIT(12)
#define STREAM_DMA_PIXEL_FMT_PACK_TYPE(v)	(((v) & 0x1) << 16)

#define PLANES_SINGLE			1
#define PLANES_DUAL			3
#define WIDTH_8				0
#define WIDTH_16			8
#define FORMAT_BGR			0x0
#define FORMAT_RGB			0x1
#define FORMAT_444			0x4
#define FORMAT_422			0x5
#define FORMAT_420			0x6
#define FORMAT_BAYER			0x8
#define FORMAT_MONO			0xC

/* Bits for STREAM_DMA_WR_CTR register */
#define DMA_WR_CTR_DMA_EN		BIT(0)
#define DMA_WR_CTR_LINE_INT_PERIOD(v)	(((v) & 0xFFF) << 1)
#define DMA_WR_CTR_FIELD_EN		BIT(13)
#define DMA_WR_CTR_FRAME_END_EN		BIT(14)
#define DMA_WR_CTR_FRAME_BEGIN_EN	BIT(15)
#define DMA_WR_CTR_TEST_MODE		BIT(16)

/* Bits for STREAM_DMA_WR_STATUS register */
#define DMA_WR_STATUS_LINE		BIT(0)
#define DMA_WR_STATUS_FIELD		BIT(1)
#define DMA_WR_STATUS_FRAME_END		BIT(2)
#define DMA_WR_STATUS_FRAME_BEGIN	BIT(3)
#define DMA_WR_STATUS_DMA_OVF		BIT(4)

/* Bits for STREAM_CTR register */
#define STREAM_CTR_STREAM0_ENABLE	BIT(0)
#define STREAM_CTR_STREAM1_ENABLE	BIT(1)
#define STREAM_CTR_STREAM_ENABLE(n)	BIT(n)
#define STREAM_CTR_DMA_CHANNELS_ENABLE	BIT(8)

#define CTRL_BAD_PIXELS_COUNT		4096
#define CTRL_BAD_ROWSCOLS_COUNT		16
#define CTRL_GC_ELEMENTS_COUNT		4096
#define CTRL_DR_ELEMENTS_COUNT		4096

#define MAX_WIDTH_HEIGHT		4095U
#define MAX_COMP_VALUE			4095

/* Historgam hardware bug correction */
#define HISTOGRAM_BAD_THRESHOLD		800000

#define CLUSTER_SIZE(c) (sizeof(c) / sizeof(struct v4l2_ctrl *))

#define AWB_MEMBER_NUM 4
#define ABR_MEMBER_NUM 2

enum vinc_input_format { UNKNOWN, BAYER, RGB, YCbCr };

struct vinc_cluster_bp {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *pix;
	struct v4l2_ctrl *row;
	struct v4l2_ctrl *col;
};

struct vinc_cluster_gamma {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *curve;
	struct v4l2_ctrl *gamma;
	struct v4l2_ctrl *bklight;
};

struct vinc_cluster_cc {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *cc;
	struct v4l2_ctrl *bl;
	struct v4l2_ctrl *wbcc;
	struct v4l2_ctrl *ck;
	struct v4l2_ctrl *ab;
	struct v4l2_ctrl *brightness;
	struct v4l2_ctrl *contrast;
	struct v4l2_ctrl *saturation;
	struct v4l2_ctrl *hue;
	struct v4l2_ctrl *fx;
	struct v4l2_ctrl *cbcr;
	struct v4l2_ctrl *awb;
	struct v4l2_ctrl *dowb;
	struct v4l2_ctrl *rb;
	struct v4l2_ctrl *bb;
	struct v4l2_ctrl *wbt;
};

struct vinc_cluster_ct {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *ct;
};

struct vinc_cluster_dr {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *dr;
};

struct vinc_cluster_stat {
	struct v4l2_ctrl *enable;
	struct v4l2_ctrl *af_color;
	struct v4l2_ctrl *af_th;
	struct v4l2_ctrl *zone[4];
	struct v4l2_ctrl *hist[4];
	struct v4l2_ctrl *af[4];
	struct v4l2_ctrl *add[4];
};

struct vinc_cluster_exposure {
	struct v4l2_ctrl *ae;
	struct v4l2_ctrl *sensor_ae;
	struct v4l2_ctrl *sensor_ag;
};

struct vinc_cluster {
	struct vinc_cluster_bp bp;
	struct vinc_cluster_gamma gamma;
	struct vinc_cluster_cc cc;
	struct vinc_cluster_ct ct;
	struct vinc_cluster_dr dr;
	struct vinc_cluster_stat stat;
	struct vinc_cluster_exposure exp;
};

struct ctrl_priv {
	void *blacklevel;
	void *dowb;
	void *brightness;
	void *contrast;
	void *saturation;
	void *hue;
	void *ck;
	void *fx;
};

enum vinc_ycbcr_encoding {
	VINC_YCBCR_ENC_601            = 0,
	VINC_YCBCR_ENC_709            = 1,
	VINC_YCBCR_ENC_BT2020         = 2,
	VINC_YCBCR_ENC_SYCC           = 3
};

enum vinc_quantization {
	VINC_QUANTIZATION_LIM_RANGE   = 0,
	VINC_QUANTIZATION_FULL_RANGE  = 1
};

struct vinc_stream {
	struct tasklet_struct stat_tasklet;

	spinlock_t lock;		/* Protects video buffer lists */
	struct list_head capture;
	struct vb2_buffer *active;
	struct vb2_alloc_ctx *alloc_ctx;
	bool started;

	struct completion complete;

	enum v4l2_mbus_type video_source;
	int csi2_lanes;
	enum vinc_input_format input_format;
	struct v4l2_mbus_framefmt input_framefmt;
	u32 bayer_mode;

	struct vinc_cluster cluster;
	struct v4l2_ctrl *test_pattern;
	struct v4l2_ctrl *sensor_awb;
	bool stat_odd;

	struct v4l2_crop crop1;
	struct v4l2_crop crop2;
	u32 fdecim;

	struct ctrl_priv ctrl_privs;

	enum vinc_ycbcr_encoding ycbcr_enc;
	enum vinc_quantization quantization;

	int sequence;

	u8 devnum;
	bool pport_low_bits;
	u8 ifacenum;

	struct work_struct stat_work;

	struct {
		struct vinc_stat_hist hist;
		struct vinc_stat_add add;
	} summary_stat;
	/* Set during driver loading, reset during first s_ctrl() call */
	int first_load;
};

struct vinc_dev {
	struct soc_camera_host ici;

	int irq_vio;
	int irq_stream[2];
	void __iomem *base;

	struct vinc_stream stream[4];

	u32 reset_active;

	struct clk *pclk;
	struct clk *aclk;
	struct clk *sclk;

	u32 max_gain;
	u32 max_exp;
};

#endif /* VINC_DEV_H */
