/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include "vinc-hw.h"

void vinc_write(struct vinc_dev *priv,
		unsigned long reg_offs, u32 data)
{
	iowrite32(data, priv->base + reg_offs);
}

u32 vinc_read(struct vinc_dev *priv, unsigned long reg_offs)
{
	return ioread32(priv->base + reg_offs);
}

void vinc_stream_enable(struct vinc_dev *priv, u8 channel, bool enable)
{
	u32 stream_ctr = vinc_read(priv, STREAM_CTR);

	if (enable)
		stream_ctr |= STREAM_CTR_STREAM_ENABLE(channel);
	else
		stream_ctr &= ~STREAM_CTR_STREAM_ENABLE(channel);

	vinc_write(priv, STREAM_CTR, stream_ctr);
}

void set_cc_ct(struct vinc_dev *priv, u8 channel, struct vinc_cc *cc,
	       int is_ct)
{
	int i;
	u32 start_reg = is_ct ? STREAM_PROC_CT_COEFF(channel, 0) :
			STREAM_PROC_CC_COEFF(channel, 0);
	u32 val;

	for (i = 0; i < 4; i++) {
		val = cc->coeff[i * 2] | (cc->coeff[i * 2 + 1] << 16);
		vinc_write(priv, start_reg + i * 4, val);
	}
	vinc_write(priv, start_reg + CC_CT_OFFSET_COEFF8, cc->coeff[8]);
	val = cc->offset[0] | (cc->offset[1] << 16);
	vinc_write(priv, start_reg + CC_CT_OFFSET_OFFSET0_1, val);
	val = cc->offset[2] | (cc->scaling << 16);
	vinc_write(priv, start_reg + CC_CT_OFFSET_OFFSET2, val);
}

void vinc_stat_start(struct vinc_stream *stream)
{
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[stream->devnum]);

	stream->stat_odd = true;
	vinc_write(priv, STREAM_PROC_CLEAR(0),
		   STREAM_PROC_CLEAR_THR_CLR);
}

void set_stat_zone(struct vinc_stream *stream, u32 zone_id,
		   struct vinc_stat_zone *zone)
{
	const u8 devnum = stream->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));

	if (zone->enable) {
		u32 lt = (zone->x_lt + stream->crop2.c.left) |
				((zone->y_lt +
				stream->crop2.c.top) << 16);
		u32 rb = (zone->x_rb + stream->crop2.c.left) |
				((zone->y_rb +
				stream->crop2.c.top) << 16);

		vinc_write(priv, STREAM_PROC_STAT_ZONE_LT(channel, zone_id),
			   lt);
		vinc_write(priv, STREAM_PROC_STAT_ZONE_RB(channel, zone_id),
			   rb);
		proc_cfg |= BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + zone_id);
	} else
		proc_cfg &= ~(BIT(STREAM_PROC_CFG_STT_ZONE_OFFSET + zone_id));
	vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);
}

void vinc_configure_input(struct vinc_stream *stream)
{
	const u8 devnum = stream->devnum;
	const u8 ifacenum = stream->ifacenum & 0x01;
	const u8 channel = devnum & 0x01;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);

	if (stream->test_pattern->val) {
		u32 test_src = 0;

		test_src |= stream->crop1.c.width + stream->crop1.c.left;
		test_src |= (stream->crop1.c.height +
				stream->crop1.c.top) << 12;
		test_src |= 5 << 24;
		test_src |= (stream->test_pattern->val - 1) << 29;

		vinc_write(priv, PPORT_INP_MUX_CFG, 0x101);
		vinc_write(priv, PPORT_CFG(0),
				PORT_CFG_PIXEL_MODE(1) | PORT_CFG_VIN_SRC(1));
		vinc_write(priv, PPORT_CFG(1),
				PORT_CFG_PIXEL_MODE(1) | PORT_CFG_VIN_SRC(2));
		vinc_write(priv, PPORT_CFG(2),
				PORT_CFG_PIXEL_MODE(1) | PORT_CFG_VIN_SRC(3));
		vinc_write(priv, PPORT_TEST_SRC, test_src);
		vinc_write(priv, PINTERFACE_CFG(0),
				PINTERFACE_CFG_CYCLE_NUM(1) |
				PINTERFACE_CFG_PIXEL_NUM_EVEN(1) |
				PINTERFACE_CFG_PORT_NUM_SYNC(2));
		vinc_write(priv, PINTERFACE_CCMOV(0, 0), 0x321);
		vinc_write(priv, PINTERFACE_HVFSYNC(0),
				PINTERFACE_HVFSYNC_DELAY_V(0x11) |
				PINTERFACE_HVFSYNC_PRE_DELAY_V(1));
		vinc_write(priv, STREAM_INP_CFG(0), 0x0);
	} else if (stream->video_source == V4L2_MBUS_CSI2) {
		vinc_write(priv, PPORT_INP_MUX_CFG, 0x0);
		vinc_write(priv, PPORT_CFG(0), 0x0);
		vinc_write(priv, PPORT_CFG(1), 0x0);
		vinc_write(priv, PPORT_CFG(2), 0x0);

		vinc_write(priv, CSI2_DEVICE_READY(ifacenum), 0x0);
		vinc_write(priv, CSI2_INTR(ifacenum), 0x0007FFFF);
		vinc_write(priv, CSI2_PORT_GENFIFO_CTR(ifacenum), 0x0);

		/* 1lane, timeout=max */
		vinc_write(priv, CSI2_FUNC_PROG(ifacenum), 0x1ffffc |
				((stream->csi2_lanes - 1) & 0x3));
		vinc_write(priv, CSI2_DPHY_TIM3(ifacenum),
			   CSI2_TIM3_CLN_CNT_LPX(0xBF) |
			   CSI2_TIM3_DLN_CNT_LPX(0x12));
		vinc_write(priv, CSI2_SYNC_COUNT(ifacenum), 0x14141414);
		vinc_write(priv, CSI2_RCV_COUNT(ifacenum), 0x04040404);

		vinc_write(priv, CSI2_FSLS(ifacenum), 0x2);
		vinc_write(priv, CSI2_LSDV(ifacenum), 0x2);
		vinc_write(priv, CSI2_DVLE(ifacenum), 0x2);
		vinc_write(priv, CSI2_LEFE(ifacenum), 0x2);
		vinc_write(priv, CSI2_FEFS(ifacenum), 0x2);
		vinc_write(priv, CSI2_LELS(ifacenum), 0x4);
		vinc_write(priv, CSI2_LOOP_BACK(ifacenum), 0x0);
		vinc_write(priv, CSI2_RAW8(ifacenum), 0x0);
		vinc_write(priv, CSI2_DPHY_TIM1(ifacenum),
			   CSI2_TIM1_DLN_CNT_HS_PREP(0x06) |
			   CSI2_TIM1_DLN_CNT_HS_ZERO(0x04) |
			   CSI2_TIM1_DLN_CNT_HS_TRAIL(0x07) |
			   CSI2_TIM1_DLN_CNT_HS_EXIT(0x02));
		vinc_write(priv, CSI2_DPHY_TIM2(ifacenum),
			   CSI2_TIM2_CLN_CNT_HS_PREP(0x06) |
			   CSI2_TIM2_CLN_CNT_HS_ZERO(0x04) |
			   CSI2_TIM2_CLN_CNT_HS_TRAIL(0x11) |
			   CSI2_TIM2_CLN_CNT_HS_EXIT(0x03));
		vinc_write(priv, CSI2_TRIM0(ifacenum), 0x02000000);
		vinc_write(priv, CSI2_DEVICE_READY(ifacenum), 0x1);
		vinc_write(priv, STREAM_INP_CFG(channel), 0x2 + ifacenum);
	} else if (stream->video_source == V4L2_MBUS_PARALLEL) {
		vinc_write(priv, PPORT_INP_MUX_CFG, 0);
		vinc_write(priv, PPORT_CFG(ifacenum), PORT_CFG_PIXEL_MODE(1));
		vinc_write(priv, PPORT_CFG(2), PORT_CFG_PIXEL_MODE(0));
		if (stream->input_format == BAYER) {
			vinc_write(priv, PINTERFACE_CFG(ifacenum),
					PINTERFACE_CFG_CYCLE_NUM(1) |
					PINTERFACE_CFG_PIXEL_NUM_EVEN(1) |
					PINTERFACE_CFG_PORT_NUM_SYNC(ifacenum));
			vinc_write(priv, PINTERFACE_CCMOV(ifacenum, 0),
				   0x1 + ifacenum);
		} else if (stream->input_format == YCbCr) {
			vinc_write(priv, PINTERFACE_CFG(ifacenum),
					PINTERFACE_CFG_CYCLE_NUM(4) |
					PINTERFACE_CFG_PIXEL_NUM_EVEN(2) |
					PINTERFACE_CFG_PORT_NUM_SYNC(ifacenum));
			/* TODO: Add macros for CCMOV settings */
			vinc_write(priv, PINTERFACE_CCMOV(ifacenum, 0),
				   0x091D0915);
		} else
			dev_err(priv->ici.v4l2_dev.dev, "Unknown input format %#x",
				stream->input_format);

		vinc_write(priv, PINTERFACE_HVFSYNC(ifacenum),
				PINTERFACE_HVFSYNC_DELAY_V(0x11) |
				PINTERFACE_HVFSYNC_PRE_DELAY_V(1));
		vinc_write(priv, STREAM_INP_CFG(channel), ifacenum);
	} else
		dev_err(priv->ici.v4l2_dev.dev, "Unknown input source %#x",
			stream->video_source);
}

/*
 * TODO: These macros are here to support vinc_configure_m420(). When they will
 * be converted to use functions from vinc-neon.c, the macros must be removed.
 */
#define COEFF_FLOAT_TO_U16(coeff, scaling) ((u16)((s16)((coeff) *  \
		(1 << (15 - (scaling))) + ((coeff) < 0 ? -0.5 : 0.5))))

#define OFFSET_FLOAT_TO_U16(offset) ((u16)((s16)(offset * 4)))

static void vinc_configure_bgr(struct vinc_dev *priv,
			       struct soc_camera_device *icd)
{
	const u8 devnum = icd->devnum;
	u8 ct_en;
	u32 proc_cfg;
	struct vinc_stream *const stream = &priv->stream[devnum];
	const u8 channel = devnum & 0x01;
	struct vinc_cc *ct = stream->cluster.ct.ct->p_cur.p;

	if (stream->input_format == YCbCr) {
		ct_en = 1;
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));
		proc_cfg |= STREAM_PROC_CFG_422TO444_EN |
				STREAM_PROC_CFG_CT_SRC(1);
		vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);
		if (stream->pport_low_bits) {
			/* YCbCr->RGB conversion matrix and Y*4, Cb*4, Cr*4 */
			ct->scaling = 3;
			ct->coeff[0] = COEFF_FLOAT_TO_U16(4, 3);
			ct->coeff[1] = COEFF_FLOAT_TO_U16(-1.376, 3);
			ct->coeff[2] = COEFF_FLOAT_TO_U16(-2.856, 3);
			ct->coeff[3] = COEFF_FLOAT_TO_U16(4, 3);
			ct->coeff[4] = COEFF_FLOAT_TO_U16(7.088, 3);
			ct->coeff[5] = COEFF_FLOAT_TO_U16(0, 3);
			ct->coeff[6] = COEFF_FLOAT_TO_U16(4, 3);
			ct->coeff[7] = COEFF_FLOAT_TO_U16(0, 3);
			ct->coeff[8] = COEFF_FLOAT_TO_U16(5.608, 3);
		} else {
			ct->scaling = 1;
			ct->coeff[0] = COEFF_FLOAT_TO_U16(1, 1);
			ct->coeff[1] = COEFF_FLOAT_TO_U16(-0.3918, 1);
			ct->coeff[2] = COEFF_FLOAT_TO_U16(-0.7141, 1);
			ct->coeff[3] = COEFF_FLOAT_TO_U16(1, 1);
			ct->coeff[4] = COEFF_FLOAT_TO_U16(1.772, 1);
			ct->coeff[5] = COEFF_FLOAT_TO_U16(0, 1);
			ct->coeff[6] = COEFF_FLOAT_TO_U16(1, 1);
			ct->coeff[7] = COEFF_FLOAT_TO_U16(0, 1);
			ct->coeff[8] = COEFF_FLOAT_TO_U16(1.402, 1);
		}
		ct->offset[0] = OFFSET_FLOAT_TO_U16(2167.3422);
		ct->offset[1] = OFFSET_FLOAT_TO_U16(-3629.056);
		ct->offset[2] = OFFSET_FLOAT_TO_U16(-2871.296);

		set_cc_ct(priv, channel, ct, 1);
	} else
		ct_en = 0;

	v4l2_ctrl_s_ctrl(priv->stream[devnum].cluster.ct.enable, ct_en);

	vinc_write(priv, STREAM_DMA_FBUF_CFG(channel, 0), 0x00001);
	vinc_write(priv, STREAM_DMA_FBUF_LSTEP(channel, 0, 0),
		   icd->bytesperline);
	vinc_write(priv, STREAM_DMA_FBUF_FSTEP(channel, 0, 0), icd->sizeimage);
	vinc_write(priv, STREAM_DMA_PIXEL_FMT(channel, 0),
		   STREAM_DMA_PIXEL_FMT_PLANES(PLANES_SINGLE) |
		   STREAM_DMA_PIXEL_FMT_FORMAT(FORMAT_BGR));
}

/* TODO: Add support for YCbCr input format */
static void vinc_configure_m420(struct vinc_dev *priv,
				struct soc_camera_device *icd)
{
	u32 proc_cfg;
	const u8 devnum = icd->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_stream * const stream = &priv->stream[devnum];
	struct vinc_cc *ct = stream->cluster.ct.ct->p_cur.p;
	int i;

	ct->scaling = 0;
	ct->coeff[0] = COEFF_FLOAT_TO_U16(0.587005, 0);
	ct->coeff[1] = COEFF_FLOAT_TO_U16(0.113983, 0);
	ct->coeff[2] = COEFF_FLOAT_TO_U16(0.299011, 0);
	ct->coeff[3] = COEFF_FLOAT_TO_U16(-0.338836, 0);
	ct->coeff[4] = COEFF_FLOAT_TO_U16(0.511413, 0);
	ct->coeff[5] = COEFF_FLOAT_TO_U16(-0.172576, 0);
	ct->coeff[6] = COEFF_FLOAT_TO_U16(-0.428253, 0);
	ct->coeff[7] = COEFF_FLOAT_TO_U16(-0.083160, 0);
	ct->coeff[8] = COEFF_FLOAT_TO_U16(0.511413, 0);
	ct->offset[0] = 0x0;
	ct->offset[1] = 0x2000;
	ct->offset[2] = 0x2000;
	set_cc_ct(priv, channel, ct, 1);
	v4l2_ctrl_s_ctrl(stream->cluster.ct.enable, 1);

	proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));
	proc_cfg |= STREAM_PROC_CFG_CT_EN |
			STREAM_PROC_CFG_444TO422_EN |
			STREAM_PROC_CFG_422TO420_EN |
			STREAM_PROC_CFG_444TO422_SRC(1) |
			STREAM_PROC_CFG_422TO420_SRC(1);
	vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);

	vinc_write(priv, STREAM_DMA_FBUF_CFG(channel, 0), 0x00001);
	for (i = 0; i < 2; i++) {
		vinc_write(priv, STREAM_DMA_FBUF_LSTEP(channel, 0, i),
			   icd->bytesperline | BIT(31));
		vinc_write(priv, STREAM_DMA_FBUF_FSTEP(channel, 0, i),
			   icd->sizeimage);
	}
	vinc_write(priv, STREAM_DMA_PIXEL_FMT(channel, 0),
		   STREAM_DMA_PIXEL_FMT_PLANES(PLANES_DUAL) |
		   STREAM_DMA_PIXEL_FMT_FORMAT(FORMAT_420));
}

void vinc_configure(struct vinc_dev *priv, struct soc_camera_device *icd)
{
	u32 axi_master_cfg, proc_ctr, proc_cfg;
	struct vinc_stat_zone *zone;
	const u8 devnum = icd->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_stream * const stream = &priv->stream[devnum];

	vinc_write(priv, STREAM_INP_HCROP_CTR(channel),
		   (stream->crop1.c.width << 16) | stream->crop1.c.left);
	vinc_write(priv, STREAM_INP_VCROP_CTR(channel),
		   (stream->crop1.c.height << 16) | stream->crop1.c.top);
	vinc_write(priv, STREAM_INP_VCROP_ODD_CTR(channel), 0);
	vinc_write(priv, STREAM_INP_DECIM_CTR(channel),
		   STREAM_INP_DECIM_FDECIM(stream->fdecim - 1));

	zone = stream->cluster.stat.zone[3]->p_cur.p;
	/* Zone boundaries should not match with image boundaries
	 * for sobel filter correct calculation. See also rf#2159.
	 */
	zone->enable = 1;
	zone->x_lt = 1;
	zone->y_lt = 1;
	zone->x_rb = stream->crop2.c.width - 2;
	zone->y_rb = stream->crop2.c.height - 2;
	proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));
	proc_cfg |= STREAM_PROC_CFG_STT_EN(stream->cluster.stat.enable->val);
	vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);
	set_stat_zone(stream, 3, stream->cluster.stat.zone[3]->p_cur.p);

	proc_ctr = STREAM_PROC_CTR_BAYER_MODE(stream->bayer_mode);
	proc_ctr |= STREAM_PROC_CTR_AF_COLOR(
			stream->cluster.stat.af_color->val);
	proc_ctr |= STREAM_PROC_CTR_HIST_THR |
			STREAM_PROC_CTR_AF_THR |
			STREAM_PROC_CTR_ADD_THR;
	vinc_write(priv, STREAM_PROC_CTR(channel), proc_ctr);
	if (stream->cluster.stat.enable->val)
		vinc_stat_start(stream);

	vinc_write(priv, STREAM_DMA_FBUF_HORIZ(channel, 0),
		   (stream->crop2.c.width << 16) | stream->crop2.c.left);
	vinc_write(priv, STREAM_DMA_FBUF_VERT(channel, 0),
		   (stream->crop2.c.height << 16) | stream->crop2.c.top);
	vinc_write(priv, STREAM_DMA_FBUF_DECIM(channel, 0), 0x10000);

	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		vinc_configure_bgr(priv, icd);
		break;
	case V4L2_PIX_FMT_M420:
		vinc_configure_m420(priv, icd);
		break;
	default:
		dev_warn(priv->ici.v4l2_dev.dev, "Unknown output format %#x\n",
			 icd->current_fmt->host_fmt->fourcc);
		break;
	}

	axi_master_cfg = vinc_read(priv, AXI_MASTER_CFG);
	vinc_write(priv, AXI_MASTER_CFG,
		   axi_master_cfg | AXI_MASTER_CFG_GLOBAL_EN);
}
