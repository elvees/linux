/*
 * Copyright 2015 ELVEES NeoTek CJSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Based on V4L2 Driver for SuperH Mobile CEU
 * interface - "sh_mobile_ceu_camera.c"
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/of_reserved_mem.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <linux/pm_runtime.h>
#include <linux/sched.h>
#include <linux/clk.h>

#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-dev.h>
#include <media/videobuf2-dma-contig.h>
#include <media/v4l2-mediabus.h>

#include "vinc-dev.h"
#include "vinc-hw.h"
#include "vinc-ctrls.h"

/* Maximum PCLK divider. Determined by hardware. */
#define PCLKDIV_MAX 30

struct vinc_cam {
	/* Client output */
	unsigned int width;
	unsigned int height;
	u32 code;
};

static struct soc_mbus_pixelfmt vinc_formats[] = {
	{
		.name = "BGR 8+8+8+8",
		.fourcc = V4L2_PIX_FMT_BGR32,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
		.layout = SOC_MBUS_LAYOUT_PACKED,
		.bits_per_sample = 32,
	},
	{
		.name = "YUV 4:2:0 2 lines Y, 1 line UV interleaved",
		.fourcc = V4L2_PIX_FMT_M420,
		.packing = SOC_MBUS_PACKING_NONE,
		.order = SOC_MBUS_ORDER_LE,
		.layout = SOC_MBUS_LAYOUT_PACKED,
		.bits_per_sample = 8,
	},
};

/* per video frame buffer */
struct vinc_buffer {
	struct vb2_v4l2_buffer vb; /* v4l buffer must be first */
	struct list_head queue;
};

/*
 * .queue_setup() is called to check, whether the driver can accept the
 *		  requested number of buffers and to fill in plane sizes
 *		  for the current frame format if required
 */
static int vinc_queue_setup(struct vb2_queue *vq, const void *parg,
			    unsigned int *count, unsigned int *num_planes,
			    unsigned int sizes[], void *alloc_ctxs[])
{
	struct soc_camera_device *icd = container_of(vq,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	const struct v4l2_format *fmt = parg;

	if (fmt) {
		const struct soc_camera_format_xlate *xlate;
		unsigned int bytes_per_line;
		int ret;

		if (fmt->fmt.pix.sizeimage < icd->sizeimage)
			return -EINVAL;

		xlate = soc_camera_xlate_by_fourcc(icd,
						   fmt->fmt.pix.pixelformat);
		if (!xlate)
			return -EINVAL;
		ret = soc_mbus_bytes_per_line(fmt->fmt.pix.width,
					      xlate->host_fmt);
		if (ret < 0)
			return ret;

		bytes_per_line = max_t(u32, fmt->fmt.pix.bytesperline, ret);

		if (xlate->host_fmt->fourcc == V4L2_PIX_FMT_M420)
			ret = fmt->fmt.pix.height * bytes_per_line * 3 / 2;
		else
			ret = soc_mbus_image_size(xlate->host_fmt,
				bytes_per_line, fmt->fmt.pix.height);
		if (ret < 0)
			return ret;

		sizes[0] = max_t(u32, fmt->fmt.pix.sizeimage, ret);
	} else {
		sizes[0] = icd->sizeimage;
	}

	dev_dbg(icd->parent, "Requested %u buffers\n", *count);

	dev_dbg(icd->parent, "%s: image_size=%d\n", __func__,
		icd->sizeimage);

	alloc_ctxs[0] = priv->stream[icd->devnum].alloc_ctx;
	*num_planes = 1;

	return 0;
}

static struct vinc_buffer *to_vinc_vb(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	return container_of(vbuf, struct vinc_buffer, vb);
}

static int vinc_buf_prepare(struct vb2_buffer *vb)
{
	return 0;
}

static void vinc_start_capture(struct vinc_dev *priv,
			       struct soc_camera_device *icd)
{
	dma_addr_t phys_addr_top;
	const u8 devnum = icd->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 wr_ctr = vinc_read(priv, STREAM_DMA_WR_CTR(channel, 0));

	wr_ctr &= ~DMA_WR_CTR_DMA_EN;
	vinc_write(priv, STREAM_DMA_WR_CTR(channel, 0), wr_ctr);
	if (stream->started)
		vinc_stream_enable(priv, channel, true);

	phys_addr_top = vb2_dma_contig_plane_dma_addr(stream->active, 0);
	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		vinc_write(priv, STREAM_DMA_FBUF_BASE(channel, 0, 0),
			   phys_addr_top);
		break;
	case V4L2_PIX_FMT_M420:
		vinc_write(priv, STREAM_DMA_FBUF_BASE(channel, 0, 0),
			   phys_addr_top + (stream->crop2.c.width << 1));
		vinc_write(priv, STREAM_DMA_FBUF_BASE(channel, 0, 1),
			   phys_addr_top);
		break;
	default:
		dev_err(priv->ici.v4l2_dev.dev, "Unknown output fourcc %#x",
			icd->current_fmt->host_fmt->fourcc);
		return;
	}
	wr_ctr |= DMA_WR_CTR_DMA_EN;
	vinc_write(priv, STREAM_DMA_WR_CTR(channel, 0), wr_ctr);
}

static void vinc_buf_queue(struct vb2_buffer *vb)
{
	struct soc_camera_device *icd = container_of(vb->vb2_queue,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_buffer *buf = to_vinc_vb(vb);
	unsigned long size = icd->sizeimage;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];

	dev_dbg(icd->parent, "Add buffer #%u to queue\n", vb->index);

	if (vb2_plane_size(vb, 0) < size) {
		dev_err(icd->parent, "Buffer #%u too small (%lu < %lu)\n",
			vb->index, vb2_plane_size(vb, 0), size);
		vb2_buffer_done(vb, VB2_BUF_STATE_ERROR);
	}
	vb2_set_plane_payload(vb, 0, size);

	spin_lock_irq(&stream->lock);
	list_add_tail(&buf->queue, &stream->capture);
	if (!stream->active) {
		stream->active = vb;
		vinc_start_capture(priv, icd);
	}
	spin_unlock_irq(&stream->lock);
}

static int vinc_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct soc_camera_device *icd = container_of(q,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	unsigned long timeout;
	const u8 devnum = icd->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_stream * const stream = &priv->stream[devnum];
	const u8 ifacenum = stream->ifacenum & 0x01;
	u32 csi2_intr;
	u32 reg;

	if (priv->stream[((~devnum) & 0x02) | channel].started)
		return -EBUSY;

	dev_dbg(icd->parent, "Start streaming (count: %u)\n", count);

	v4l2_ctrl_grab(stream->test_pattern, 1);
	if (stream->video_source == V4L2_MBUS_CSI2) {
		reg = vinc_read(priv, CSI2_PORT_SYS_CTR(ifacenum));
		vinc_write(priv, CSI2_PORT_SYS_CTR(ifacenum),
			   reg & ~CSI2_PORT_SYS_CTR_ENABLE);
		vinc_write(priv, CSI2_PORT_SYS_CTR(ifacenum),
			   reg | CSI2_PORT_SYS_CTR_ENABLE);
		vinc_configure_input(stream);

		timeout = jiffies + msecs_to_jiffies(30);
		do {
			csi2_intr = vinc_read(priv, CSI2_INTR(ifacenum));
			if (!(csi2_intr & BIT(9)))
				schedule();
			else
				break;
		} while (time_before(jiffies, timeout));

		if (!(csi2_intr & BIT(9))) {
			struct vinc_buffer *buf, *tmp;

			list_for_each_entry_safe(buf, tmp,
						 &stream->capture, queue) {
				list_del_init(&buf->queue);
				vb2_buffer_done(&buf->vb.vb2_buf,
						VB2_BUF_STATE_QUEUED);
			}
			v4l2_ctrl_grab(stream->test_pattern, 0);
			stream->active = NULL;
			dev_err(icd->parent,
				"Can not receive video from sensor (%#x)\n",
				csi2_intr);
			return -EIO;
		}
	} else if (stream->video_source == V4L2_MBUS_PARALLEL)
		vinc_configure_input(stream);

	vinc_write(priv, STREAM_DMA_WR_CTR(channel, 0),
		   DMA_WR_CTR_FRAME_END_EN);

	reg = vinc_read(priv, STREAM_PROC_CFG(channel));
	if (stream->input_format == BAYER && !stream->test_pattern->cur.val)
		reg |= STREAM_PROC_CFG_CFA_EN;
	else
		reg &= ~STREAM_PROC_CFG_CFA_EN;
	vinc_write(priv, STREAM_PROC_CFG(channel), reg);

	reg = vinc_read(priv, STREAM_CTR);
	reg |= STREAM_CTR_DMA_CHANNELS_ENABLE;
	reg |= STREAM_CTR_STREAM_ENABLE(channel);
	vinc_write(priv, STREAM_CTR, reg);

	stream->sequence = 0;
	spin_lock_irq(&stream->lock);
	stream->started = true;
	if (stream->active)
		vinc_start_capture(priv, icd);
	spin_unlock_irq(&stream->lock);

	return 0;
}

static void vinc_stop_streaming(struct vb2_queue *q)
{
	struct soc_camera_device *icd = container_of(q,
			struct soc_camera_device, vb2_vidq);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_buffer *buf, *tmp;
	const u8 devnum = icd->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 csi2_port_sys_ctr;

	dev_dbg(icd->parent, "Stop streaming\n");

	vinc_stream_enable(priv, channel, false);

	v4l2_ctrl_grab(stream->test_pattern, 0);
	vinc_write(priv, STREAM_DMA_WR_CTR(channel, 0), 0x0);
	csi2_port_sys_ctr = vinc_read(priv, CSI2_PORT_SYS_CTR(channel));
	vinc_write(priv, CSI2_PORT_SYS_CTR(channel),
		   csi2_port_sys_ctr & ~CSI2_PORT_SYS_CTR_ENABLE);
	/* GLOBAL_ENABLE still enable for sensor clocks */

	spin_lock_irq(&stream->lock);

	stream->active = NULL;
	stream->started = false;

	list_for_each_entry_safe(buf, tmp,
				 &stream->capture, queue) {
		list_del_init(&buf->queue);
		if (buf->vb.vb2_buf.state == VB2_BUF_STATE_ACTIVE)
			vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_ERROR);
	}

	spin_unlock_irq(&stream->lock);
}

static struct vb2_ops vinc_videobuf_ops = {
	.queue_setup		= vinc_queue_setup,
	.buf_prepare		= vinc_buf_prepare,
	.buf_queue		= vinc_buf_queue,
	.wait_prepare		= vb2_ops_wait_prepare,
	.wait_finish		= vb2_ops_wait_finish,
	.start_streaming	= vinc_start_streaming,
	.stop_streaming		= vinc_stop_streaming,
};

static int vinc_add(struct soc_camera_device *icd)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct v4l2_queryctrl query;
	int rc;

	query.id = V4L2_CID_GAIN;
	rc = v4l2_subdev_queryctrl(sd, &query);
	if (rc < 0)
		return rc;
	priv->max_gain = query.maximum;
	query.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	rc = v4l2_subdev_queryctrl(sd, &query);
	priv->max_exp = query.maximum;
	if (rc < 0)
		return rc;
	return 0;
}

static int vinc_get_formats(struct soc_camera_device *icd, unsigned int idx,
			    struct soc_camera_format_xlate *xlate)
{
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct device *dev = icd->parent;
	struct soc_camera_host *ici = to_soc_camera_host(dev);
	struct vinc_dev *priv = ici->priv;
	struct vinc_cam *cam;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	int ret, i;
	int formats_count;
	struct v4l2_subdev_mbus_code_enum code = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.index = idx,
	};

	ret = v4l2_subdev_call(sd, pad, enum_mbus_code, NULL, &code);
	if (ret < 0)
		return ret;

	switch (code.code) {
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
	case MEDIA_BUS_FMT_UYVY8_2X8:
	case MEDIA_BUS_FMT_RGB888_1X24:
		formats_count = ARRAY_SIZE(vinc_formats);
		if (xlate) {
			for (i = 0; i < formats_count; i++) {
				xlate->host_fmt = &vinc_formats[i];
				xlate->code = code.code;
				xlate++;
			}
		}
		break;
	default:
		formats_count = 0;
		break;
	}

	if (!icd->host_priv) {
		struct v4l2_mbus_config mbus_cfg;
		struct v4l2_subdev_format format = {
			.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		};
		struct v4l2_mbus_framefmt *mf = &format.format;

		ret = vinc_create_controls(&icd->ctrl_handler, stream);
		if (ret)
			return ret;

		ret = v4l2_subdev_call(sd, video, g_mbus_config, &mbus_cfg);
		if (ret >= 0) {
			stream->video_source = mbus_cfg.type;

			if (mbus_cfg.type != V4L2_MBUS_CSI2 &&
					mbus_cfg.type != V4L2_MBUS_PARALLEL) {
				dev_err(dev,
					"Interface type %d is not supported\n",
					mbus_cfg.type);
				return -EINVAL;
			}

			if (mbus_cfg.flags & V4L2_MBUS_CSI2_4_LANE)
				stream->csi2_lanes = 4;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_3_LANE)
				stream->csi2_lanes = 3;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_2_LANE)
				stream->csi2_lanes = 2;
			else if (mbus_cfg.flags & V4L2_MBUS_CSI2_1_LANE)
				stream->csi2_lanes = 1;
		} else {
			dev_err(dev, "Failed to get mbus config from sensor\n");
			return ret;
		}

		cam = kzalloc(sizeof(*cam), GFP_KERNEL);
		if (!cam)
			return -ENOMEM;

		ret = v4l2_subdev_call(sd, pad, get_fmt, NULL, &format);
		if (ret < 0)
			return ret;
		cam->width = mf->width;
		cam->height = mf->height;
		cam->code = mf->code;

		icd->host_priv = cam;
	}

	return formats_count;
}

static struct soc_mbus_pixelfmt *vinc_get_mbus_pixelfmt(u32 fourcc)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(vinc_formats); i++) {
		if (fourcc == vinc_formats[i].fourcc)
			return &vinc_formats[i];
	}
	return NULL;
}

static bool is_colorspace_supported(enum v4l2_colorspace colorspace)
{
	switch (colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
	case V4L2_COLORSPACE_REC709:
	case V4L2_COLORSPACE_BT2020:
	case V4L2_COLORSPACE_SRGB:
		return true;
	default:
		return false;
	}
}

static bool is_ycbcr_enc_supported(enum v4l2_ycbcr_encoding ycbcr_enc)
{
	switch (ycbcr_enc) {
	case V4L2_YCBCR_ENC_601:
	case V4L2_YCBCR_ENC_709:
	case V4L2_YCBCR_ENC_BT2020:
	case V4L2_YCBCR_ENC_SYCC:
		return true;
	default:
		return false;
	}
}

static enum v4l2_ycbcr_encoding ycbcr_enc_default(
				enum v4l2_colorspace colorspace)
{
	switch (colorspace) {
	case V4L2_COLORSPACE_REC709:
		return V4L2_YCBCR_ENC_709;
	case V4L2_COLORSPACE_BT2020:
		return V4L2_YCBCR_ENC_BT2020;
	case V4L2_COLORSPACE_SRGB:
		return V4L2_YCBCR_ENC_SYCC;
	default:
		return V4L2_YCBCR_ENC_601;
	}
}

static u32 colorspace_adjust(u32 colorspace_cam, u32 colorspace_user)
{
	enum v4l2_colorspace colorspace = colorspace_cam;

	if (!is_colorspace_supported(colorspace))
		colorspace = colorspace_user;
	if (!is_colorspace_supported(colorspace))
		colorspace = V4L2_COLORSPACE_SMPTE170M;
	return colorspace;
}

static u32 ycbcr_enc_adjust(u16 ycbcr_enc_cam, u32 ycbcr_enc_user,
			    u32 colorspace_user)
{
	enum v4l2_ycbcr_encoding ycbcr_enc = ycbcr_enc_user;

	if (!is_ycbcr_enc_supported(ycbcr_enc))
		ycbcr_enc = (u32)ycbcr_enc_cam;
	if (!is_ycbcr_enc_supported(ycbcr_enc))
		ycbcr_enc = ycbcr_enc_default(colorspace_user);
	return ycbcr_enc;
}

static int __vinc_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);

	struct v4l2_mbus_framefmt *mbus_fmt = &format->format;
	struct v4l2_subdev_pad_config pad_cfg;

	struct soc_mbus_pixelfmt *pixelfmt;
	const struct soc_camera_format_xlate *xlate;
	u32 width, height;
	u32 min_bytesperline;
	int ret;

	pix->field = V4L2_FIELD_NONE;

	pixelfmt = vinc_get_mbus_pixelfmt(pix->pixelformat);
	if (!pixelfmt)
		return -EINVAL;

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %#x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}

	v4l_bound_align_image(&pix->width, 16, MAX_WIDTH_HEIGHT, 3,
			      &pix->height, 2, MAX_WIDTH_HEIGHT, 1, 0);

	width = pix->width;
	height = pix->height;
	mbus_fmt->code = xlate->code;
	mbus_fmt->colorspace = pix->colorspace;
	mbus_fmt->width = pix->width;
	mbus_fmt->height = pix->height;
	mbus_fmt->field = pix->field;
	mbus_fmt->ycbcr_enc = pix->ycbcr_enc;
	mbus_fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;

	ret = v4l2_subdev_call(sd, pad, set_fmt, &pad_cfg, format);
	if (ret) {
		dev_warn(icd->parent, "Sensor can not negotiate format\n");
		return ret;
	}

	pix->colorspace = colorspace_adjust(mbus_fmt->colorspace,
					    pix->colorspace);
	pix->ycbcr_enc = ycbcr_enc_adjust(mbus_fmt->ycbcr_enc, pix->ycbcr_enc,
					  pix->colorspace);
	pix->quantization = V4L2_QUANTIZATION_DEFAULT;

	pix->width = min3(pix->width, mbus_fmt->width, MAX_WIDTH_HEIGHT);
	pix->height = min3(pix->height, mbus_fmt->height, MAX_WIDTH_HEIGHT);

	if (pix->bytesperline > 0xFFF8)
		pix->bytesperline = 0xFFF8;

	pix->bytesperline &= ~0x7;
	min_bytesperline = (pix->width * pixelfmt->bits_per_sample) / 8;
	if (pix->bytesperline < min_bytesperline)
		pix->bytesperline = min_bytesperline;

	switch (pixelfmt->fourcc) {
	case V4L2_PIX_FMT_M420:
		pix->sizeimage = pix->bytesperline * ((pix->height * 3) / 2);
		break;
	default:
		pix->sizeimage = pix->bytesperline * pix->height;
		break;
	}

	dev_dbg(icd->parent,
		"%s : result resolution: %dx%d, pixelformat: %#x (%s)\n",
		__func__, pix->width, pix->height, pix->pixelformat,
		pixelfmt->name);

	return 0;
}

static int vinc_try_fmt(struct soc_camera_device *icd, struct v4l2_format *f)
{
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
	};

	return __vinc_try_fmt(icd, f, &format);
}

static enum vinc_ycbcr_encoding stream_set_ycbcr_enc(u32 ycbcr_enc)
{
	switch (ycbcr_enc) {
	case V4L2_YCBCR_ENC_709:
		return VINC_YCBCR_ENC_709;
	case V4L2_YCBCR_ENC_BT2020:
		return VINC_YCBCR_ENC_BT2020;
	case V4L2_YCBCR_ENC_SYCC:
		return VINC_YCBCR_ENC_SYCC;
	default:
		return VINC_YCBCR_ENC_601;
	}
}

static enum vinc_quantization stream_set_quantization(u32 quantization)
{
	switch (quantization) {
	case V4L2_QUANTIZATION_FULL_RANGE:
		return VINC_QUANTIZATION_FULL_RANGE;
	default:
		return VINC_QUANTIZATION_LIM_RANGE;
	}
}

static int vinc_set_fmt(struct soc_camera_device *icd, struct v4l2_format *f)
{
	struct v4l2_pix_format *pix = &f->fmt.pix;
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct v4l2_subdev_format format = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
	};
	struct v4l2_mbus_framefmt *mbus_fmt = &format.format;
	const struct soc_camera_format_xlate *xlate;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	int ret;
	int offset_x, offset_y;

	ret = __vinc_try_fmt(icd, f, &format);
	if (ret)
		return ret;

	stream->ycbcr_enc = stream_set_ycbcr_enc(pix->ycbcr_enc);
	stream->quantization = stream_set_quantization(mbus_fmt->quantization);
	stream->input_framefmt = *mbus_fmt;

	switch (mbus_fmt->code) {
	case MEDIA_BUS_FMT_SRGGB8_1X8:
	case MEDIA_BUS_FMT_SRGGB10_1X10:
	case MEDIA_BUS_FMT_SRGGB12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 0;
		break;
	case MEDIA_BUS_FMT_SGRBG8_1X8:
	case MEDIA_BUS_FMT_SGRBG10_1X10:
	case MEDIA_BUS_FMT_SGRBG12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 1;
		break;
	case MEDIA_BUS_FMT_SGBRG8_1X8:
	case MEDIA_BUS_FMT_SGBRG10_1X10:
	case MEDIA_BUS_FMT_SGBRG12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 2;
		break;
	case MEDIA_BUS_FMT_SBGGR8_1X8:
	case MEDIA_BUS_FMT_SBGGR10_1X10:
	case MEDIA_BUS_FMT_SBGGR12_1X12:
		stream->input_format = BAYER;
		stream->bayer_mode = 3;
		break;
	case MEDIA_BUS_FMT_UYVY8_2X8:
		stream->input_format = YCbCr;
		break;
	case MEDIA_BUS_FMT_RGB888_1X24:
		stream->input_format = RGB;
		break;
	default:
		stream->input_format = UNKNOWN;
		dev_warn(icd->parent,
			 "Sensor reported invalid media bus format %#x\n",
			 mbus_fmt->code);
		return -EINVAL;
	}

	offset_x = (mbus_fmt->width - pix->width) / 2;
	offset_y = (mbus_fmt->height - pix->height) / 2;

	ret = v4l2_subdev_call(sd, pad, set_fmt, NULL, &format);
	if (ret) {
		dev_warn(icd->parent, "Sensor can't set format %#x, %dx%d\n",
			 mbus_fmt->code, mbus_fmt->width, mbus_fmt->height);
		return ret;
	}

	xlate = soc_camera_xlate_by_fourcc(icd, pix->pixelformat);
	if (!xlate) {
		dev_warn(icd->parent, "Format %#x not found\n",
			 pix->pixelformat);
		return -EINVAL;
	}
	icd->current_fmt = xlate;

	stream->crop1.c.width = pix->width + offset_x;
	stream->crop1.c.height = pix->height + offset_y;
	stream->crop1.c.left = 0;
	stream->crop1.c.top = 0;

	stream->crop2.c.width = pix->width;
	stream->crop2.c.height = pix->height;
	stream->crop2.c.left = offset_x;
	stream->crop2.c.top = offset_y;

	dev_dbg(icd->parent, "crop1: %dx%d (%d,%d)\n",
		stream->crop1.c.width, stream->crop1.c.height,
		stream->crop1.c.left, stream->crop1.c.top);
	dev_dbg(icd->parent, "crop2: %dx%d (%d,%d)\n",
		stream->crop2.c.width, stream->crop2.c.height,
		stream->crop2.c.left, stream->crop2.c.top);

	return 0;
}

static unsigned int vinc_poll(struct file *file, poll_table *pt)
{
	struct soc_camera_device *icd = file->private_data;

	return vb2_poll(&icd->vb2_vidq, file, pt);
}

static int vinc_querycap(struct soc_camera_host *ici,
			 struct v4l2_capability *cap)
{
	/* cap->driver is filled in soc_camera_querycap() using ici->drv_name
	 * cap->version is filled in v4l_querycap() using LINUX_VERSION_CODE
	 */
	strlcpy(cap->card, "VINC", sizeof(cap->card));
	cap->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
	cap->capabilities = cap->device_caps | V4L2_CAP_DEVICE_CAPS;
	return 0;
}

static int vinc_set_bus_param(struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	vinc_configure(priv, icd);

	return 0;
}

static int vinc_init_videobuf(struct vb2_queue *q,
			      struct soc_camera_device *icd)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);

	dev_dbg(icd->parent, "%s\n", __func__);

	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	q->io_modes = VB2_MMAP;
	q->drv_priv = icd;
	q->ops = &vinc_videobuf_ops;
	q->mem_ops = &vb2_dma_contig_memops;
	q->buf_struct_size = sizeof(struct vinc_buffer);
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->lock = &ici->host_lock;

	return vb2_queue_init(q);
}

static int vinc_get_parm(struct soc_camera_device *icd,
			 struct v4l2_streamparm *parm)
{
	int ret;
	struct v4l2_streamparm sensor_parm = {
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE
	};
	struct v4l2_subdev_frame_interval sensor_interval = {0};
	struct v4l2_fract *tpf = &parm->parm.capture.timeperframe;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;

	parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	parm->parm.capture.capturemode = 0;
	parm->parm.capture.extendedmode = priv->stream[icd->devnum].ifacenum;
	tpf->denominator = 0;
	tpf->numerator = 0;

	ret = v4l2_subdev_call(sd, video, g_parm, &sensor_parm);
	if (!ret && (sensor_parm.parm.capture.capability &
			V4L2_CAP_TIMEPERFRAME))
		*tpf = sensor_parm.parm.capture.timeperframe;

	if (!tpf->denominator || !tpf->numerator) {
		ret = v4l2_subdev_call(sd, video, g_frame_interval,
				       &sensor_interval);
		if (!ret)
			*tpf = sensor_interval.interval;
	}

	if (!tpf->denominator || !tpf->numerator) {
		dev_notice(icd->parent,
			   "Can not get framerate from sensor. Fallback to 30 FPS.\n");
		tpf->denominator = 30;
		tpf->numerator = 1;
	}
	tpf->numerator *= priv->stream[icd->devnum].fdecim;
	/* TODO: Add fraction reduction */

	return 0;
}

static int vinc_set_parm(struct soc_camera_device *icd,
			 struct v4l2_streamparm *parm)
{
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct v4l2_fract tpf, *current_tpf = &parm->parm.capture.timeperframe;
	u32 decim_ctr = vinc_read(priv, STREAM_INP_DECIM_CTR(0));
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];

	stream->fdecim = 1;
	tpf = parm->parm.capture.timeperframe;
	vinc_get_parm(icd, parm);

	/*
	 * Here we have:
	 * tpf - timeperframe requested by user (can contain zeros)
	 * current_tpf - timeperframe returned by sensor (can not contain zeros)
	 */
	if (tpf.denominator)
		stream->fdecim = (tpf.numerator * current_tpf->denominator) /
				(tpf.denominator * current_tpf->numerator);

	stream->fdecim = clamp_val(stream->fdecim, 1, 64);

	current_tpf->numerator *= stream->fdecim;
	/* TODO: Add fraction reduction */

	decim_ctr &= ~STREAM_INP_DECIM_FDECIM(0x3F);
	decim_ctr |= STREAM_INP_DECIM_FDECIM(stream->fdecim - 1);
	vinc_write(priv, STREAM_INP_DECIM_CTR(0), decim_ctr);
	return 0;
}

static struct soc_camera_host_ops vinc_host_ops = {
	.owner		= THIS_MODULE,
	.add		= vinc_add,
	.get_formats	= vinc_get_formats,
	.set_fmt	= vinc_set_fmt,
	.try_fmt	= vinc_try_fmt,
	.poll		= vinc_poll,
	.querycap	= vinc_querycap,
	.set_bus_param	= vinc_set_bus_param,
	.init_videobuf2	= vinc_init_videobuf,
	.get_parm       = vinc_get_parm,
	.set_parm       = vinc_set_parm,
};

static irqreturn_t vinc_irq_vio(int irq, void *data)
{
	struct vinc_dev *priv = data;
	u32 int_status = vinc_read(priv, INTERRUPT);

	dev_dbg(priv->ici.v4l2_dev.dev, "Interrupt vio: 0x%x\n", int_status);
	if (int_status & INTERRUPT_PPORT_ERROR) {
		u32 int_pport = vinc_read(priv, PPORT_STATUS);

		dev_warn(priv->ici.v4l2_dev.dev,
			 "Pport interrupt. status: 0x%x\n", int_pport);
	}
	vinc_write(priv, INTERRUPT_RESET, int_status);

	return IRQ_HANDLED;
}

static void vinc_next_buffer(struct vinc_stream *stream,
			     enum vb2_buffer_state state)
{
	struct vb2_buffer *vb = stream->active;
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[stream->devnum]);
	const u8 channel = stream->devnum & 0x01;

	if (!vb)
		return;
	spin_lock(&stream->lock);
	list_del_init(&to_vinc_vb(vb)->queue);

	if (!list_empty(&stream->capture)) {
		stream->active = &list_entry(stream->capture.next,
					     struct vinc_buffer,
					     queue)->vb.vb2_buf;
		vinc_start_capture(priv, priv->ici.icds[stream->devnum]);
	} else {
		u32 wr_ctr = vinc_read(priv,
				       STREAM_DMA_WR_CTR(channel, 0));

		stream->active = NULL;

		/* Without active buffer stream will be disabled in tasklet.
		 * If tasklet is not scheduled then disable stream here.
		 */
		if (stream->stat_odd) {
			vinc_stream_enable(priv, channel, false);
			stream->stat_odd = true;
		}
		wr_ctr &= ~DMA_WR_CTR_DMA_EN;
		vinc_write(priv, STREAM_DMA_WR_CTR(channel, 0), wr_ctr);
	}

	v4l2_get_timestamp(&vbuf->timestamp);

	vbuf->sequence = stream->sequence++;

	vb2_buffer_done(vb, state);
	spin_unlock(&stream->lock);
}

static void vinc_eof_handler(struct vinc_stream *stream)
{
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[stream->devnum]);

	if (stream->cluster.stat.enable->val) {
		/* TODO: Tasklet must complete before the next frame starts.
		 * Otherwise it will read broken statistic. We need to take
		 * into account that tasklet can run when the next frame starts
		 * (or protect ourselves from this situation). */
		if (stream->stat_odd)
			tasklet_schedule(&stream->stat_tasklet);
		stream->stat_odd = !stream->stat_odd;
	}
	if (stream->active) {
		dev_dbg(priv->ici.v4l2_dev.dev, "Frame end\n");
		vinc_next_buffer(stream, VB2_BUF_STATE_DONE);
	} else
		dev_warn(priv->ici.v4l2_dev.dev,
			 "Unexpected interrupt. VINC started without driver?\n");
}

static irqreturn_t vinc_irq_stream(int irq, void *data)
{
	struct vinc_stream *stream = (struct vinc_stream *)data;
	u8 devnum = stream->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	u32 int_status = vinc_read(priv, STREAM_INTERRUPT(channel));

	if (!stream->started) {
		devnum = ((~devnum) & 0x02) | channel;
		stream = &priv->stream[devnum];
	}

	dev_dbg(priv->ici.v4l2_dev.dev, "Interrupt stream%d 0x%x\n", devnum,
		int_status);
	if (int_status & STREAM_INTERRUPT_PROC) {
		u32 int_proc = vinc_read(priv, STREAM_STATUS(channel));
		u32 stream_ctr = vinc_read(priv, STREAM_CTR);
		u32 wr_ctr = vinc_read(priv, STREAM_DMA_WR_CTR(channel, 0));

		stream_ctr &= ~STREAM_CTR_STREAM_ENABLE(channel);
		vinc_write(priv, STREAM_CTR, stream_ctr);

		wr_ctr &= ~DMA_WR_CTR_DMA_EN;
		vinc_write(priv, STREAM_DMA_WR_CTR(channel, 0), wr_ctr);

		vinc_next_buffer(stream, VB2_BUF_STATE_ERROR);

		dev_warn(priv->ici.v4l2_dev.dev,
			 "Short frame/line. Stream%d_status: 0x%x\n", devnum,
			 int_proc);

		stream_ctr |= STREAM_CTR_STREAM_ENABLE(channel);
		vinc_write(priv, STREAM_CTR, stream_ctr);
	}
	if (int_status & STREAM_INTERRUPT_DMA0) {
		u32 int_d0 = vinc_read(priv, STREAM_DMA_WR_STATUS(channel, 0));

		if (int_d0 & DMA_WR_STATUS_FRAME_END)
			vinc_eof_handler(stream);
		if (int_d0 & DMA_WR_STATUS_DMA_OVF) {
			u32 stream_ctr = vinc_read(priv, STREAM_CTR);

			stream_ctr &= ~STREAM_CTR_DMA_CHANNELS_ENABLE;
			vinc_write(priv, STREAM_CTR, stream_ctr);
			vinc_next_buffer(stream, VB2_BUF_STATE_ERROR);
			dev_warn(priv->ici.v4l2_dev.dev,
				 "s%dd0: DMA overflow\n", devnum);
			stream_ctr |= STREAM_CTR_DMA_CHANNELS_ENABLE;
			vinc_write(priv, STREAM_CTR, stream_ctr);
		}
	}
	if (int_status & STREAM_INTERRUPT_DMA1) {
		u32 int_d1 = vinc_read(priv, STREAM_DMA_WR_STATUS(channel, 1));

		if (int_d1 & DMA_WR_STATUS_DMA_OVF)
			dev_warn(priv->ici.v4l2_dev.dev,
				 "s%dd1: DMA overflow\n", devnum);
	}
	vinc_write(priv, STREAM_INTERRUPT_RESET(channel), int_status);

	return IRQ_HANDLED;
}

static int vinc_clk_init(struct vinc_dev *priv, struct platform_device *pdev)
{
	int err;

	priv->pclk = devm_clk_get(&pdev->dev, "pclk");
	if (IS_ERR(priv->pclk)) {
		err = PTR_ERR(priv->pclk);
		dev_err(&pdev->dev,
			"failed to get pclk (%u)\n", err);
		return err;
	}

	priv->aclk = devm_clk_get(&pdev->dev, "aclk");
	if (IS_ERR(priv->aclk)) {
		err = PTR_ERR(priv->aclk);
		dev_err(&pdev->dev,
			"failed to get aclk (%u)\n", err);
		return err;
	}

	priv->sclk = devm_clk_get(&pdev->dev, "sclk");
	if (IS_ERR(priv->sclk)) {
		err = PTR_ERR(priv->sclk);
		dev_err(&pdev->dev,
			"failed to get sclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(priv->pclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable pclk (%u)\n", err);
		return err;
	}

	err = clk_prepare_enable(priv->aclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable aclk (%u)\n", err);
		goto disable_pclk;
	}

	err = clk_prepare_enable(priv->sclk);
	if (err) {
		dev_err(&pdev->dev,
			"failed to enable sclk (%u)\n", err);
		goto disable_aclk;
	}

	return 0;

disable_aclk:
	clk_disable_unprepare(priv->aclk);
disable_pclk:
	clk_disable_unprepare(priv->pclk);

	return err;
}

static int vinc_probe(struct platform_device *pdev)
{
	struct vinc_dev *priv;
	struct resource *res;
	struct device_node *np = pdev->dev.of_node, *endpoint = NULL;
	int err;
	u32 id;
	u32 cmos_ctr = 0, pclkdiv = 0, pclkdiv_scale = 1;
	bool fsync;
	int i;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!res) {
		dev_err(&pdev->dev, "Failed to get memory resource\n");
		return -ENODEV;
	}

	err = of_reserved_mem_device_init(&pdev->dev);
	if (err && err != -ENODEV)
		dev_info(&pdev->dev, "Failed to init reserved memory\n");

	/* Try to get and enable clocks */
	err = vinc_clk_init(priv, pdev);
	if (err)
		return err;

	priv->reset_active = 1;
	if (np)
		of_property_read_u32(np, "reset-active", &priv->reset_active);

	priv->base = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	id = vinc_read(priv, ID);
	if (id != 0x76494e01)
		dev_err(&pdev->dev, "Bad magic: %#08x\n", id);

	for (i = 0; i < ARRAY_SIZE(priv->stream); i++) {
		INIT_LIST_HEAD(&priv->stream[i].capture);
		spin_lock_init(&priv->stream[i].lock);
		init_completion(&priv->stream[i].complete);

		priv->stream[i].video_source = V4L2_MBUS_CSI2;
		priv->stream[i].input_format = BAYER;
		priv->stream[i].fdecim = 1;
		priv->stream[i].devnum = i;
		endpoint = of_graph_get_next_endpoint(pdev->dev.of_node,
						      endpoint);
		if (!endpoint)
			break;

		priv->stream[i].pport_low_bits = of_property_read_bool(endpoint,
						"elvees,pport-low-bits");
		of_property_read_u8(endpoint, "elvees,ifacenum",
				     &priv->stream[i].ifacenum);
	}

	priv->ici.priv = priv;
	priv->ici.v4l2_dev.dev = &pdev->dev;
	priv->ici.nr = pdev->id;
	priv->ici.drv_name = "vinc";
	priv->ici.ops = &vinc_host_ops;
	priv->ici.capabilities = SOCAM_HOST_CAP_STRIDE;

	for (i = 0; i < ARRAY_SIZE(priv->stream); i++) {
		priv->stream[i].alloc_ctx = vb2_dma_contig_init_ctx(&pdev->dev);
		if (IS_ERR(priv->stream[i].alloc_ctx))
			return PTR_ERR(priv->stream[i].alloc_ctx);
	}

	priv->irq_vio = platform_get_irq(pdev, 0);
	err = priv->irq_vio;
	for (i = 0; i < 2; i++) {
		priv->irq_stream[i] = platform_get_irq(pdev, i + 1);
		err |= priv->irq_stream[i];
	}
	if (err < 0) {
		dev_err(&pdev->dev, "Failed to get required IRQs\n");
		return -ENODEV;
	}

	/* request irq */
	err = devm_request_irq(&pdev->dev, priv->irq_vio, vinc_irq_vio,
			       0, dev_name(&pdev->dev), priv);
	for (i = 0; i < 2; i++)
		err |= devm_request_irq(&pdev->dev, priv->irq_stream[i],
					vinc_irq_stream, 0,
					dev_name(&pdev->dev), &priv->stream[i]);
	if (err) {
		dev_err(&pdev->dev, "Failed to request required IRQs\n");
		return err;
	}
	for (i = 0; i < ARRAY_SIZE(priv->stream); i++)
		tasklet_init(&priv->stream[i].stat_tasklet, vinc_stat_tasklet,
			     (unsigned long)&priv->stream[i]);

	of_property_read_u32(np, "elvees,pixel-clock-divider", &pclkdiv);
	fsync = of_property_read_bool(np, "elvees,pixel-clock-over-fsync");

	if (fsync) {
		/*
		 * When pixel clock is obtained from FSYNCO, it is always
		 * additionally divided by 2. Therefore the value of PCLK
		 * divider should be 2 times smaller.
		 */
		pclkdiv /= 2;
		pclkdiv_scale *= 2;
	}

	if (pclkdiv > PCLKDIV_MAX)
		pclkdiv = PCLKDIV_MAX;
	else if (pclkdiv != 1)
		pclkdiv = round_down(pclkdiv, 2);

	if (pclkdiv != 0) {
		cmos_ctr |= CMOS_CTR_PCLK_EN | CMOS_CTR_PCLK_SRC(0) |
			    CMOS_CTR_CLK_DIV(pclkdiv / 2);

		if (fsync) {
			cmos_ctr |= CMOS_CTR_FSYNC_EN;
			vinc_write(priv, CMOS_TIMER_HIGH(0), 1);
			vinc_write(priv, CMOS_TIMER_LOW(0), 1);
		}

		dev_info(&pdev->dev, "pixel clock divider is set to %u\n",
			 pclkdiv * pclkdiv_scale);
	} else
		dev_info(&pdev->dev, "pixel clock generation is disabled\n");

	if (priv->reset_active == 0)
		cmos_ctr |= CMOS_CTR_RESET;

	vinc_write(priv, CMOS_CTR(0), cmos_ctr);

	/* GLOBAL_ENABLE need for generate clocks to sensor */
	vinc_write(priv, AXI_MASTER_CFG, AXI_MASTER_CFG_MAX_BURST(2) |
			AXI_MASTER_CFG_MAX_WR_ID(1) |
			AXI_MASTER_CFG_BUF_LAYOUT(0x5) |
			AXI_MASTER_CFG_4K_BOUND_EN |
			AXI_MASTER_CFG_GLOBAL_EN);

	err = soc_camera_host_register(&priv->ici);
	if (err) {
		dev_err(&pdev->dev, "Failed to register SoC camera host\n");
		return err;
	}

	return 0;
}

static int vinc_remove(struct platform_device *pdev)
{
	struct soc_camera_host *soc_host = to_soc_camera_host(&pdev->dev);
	struct vinc_dev *priv = container_of(soc_host, struct vinc_dev, ici);
	u32 reg = vinc_read(priv, AXI_MASTER_CFG);
	int i;

	reg &= ~AXI_MASTER_CFG_GLOBAL_EN;
	vinc_write(priv, AXI_MASTER_CFG, reg);

	for (i = 0; i < 2; i++) {
		if (priv->ici.icds[i] && priv->ici.icds[i]->host_priv)
			kfree(priv->ici.icds[i]->host_priv);
	}
	soc_camera_host_unregister(soc_host);
	if (platform_get_resource(pdev, IORESOURCE_MEM, 1))
		dma_release_declared_memory(&pdev->dev);
	for (i = 0; i < 2; i++)
		vb2_dma_contig_cleanup_ctx(priv->stream[i].alloc_ctx);

	clk_disable_unprepare(priv->pclk);
	clk_disable_unprepare(priv->aclk);
	clk_disable_unprepare(priv->sclk);

	of_reserved_mem_device_release(&pdev->dev);

	return 0;
}

static const struct of_device_id vinc_of_match[] = {
	{ .compatible = "elvees,vinc" },
	{ }
};
MODULE_DEVICE_TABLE(of, vinc_of_match);

static struct platform_driver vinc_driver = {
	.driver		= {
		.name	= "vinc",
		.of_match_table = vinc_of_match,
	},
	.probe		= vinc_probe,
	.remove		= vinc_remove,
};

module_platform_driver(vinc_driver);

MODULE_DESCRIPTION("VINC driver");
MODULE_AUTHOR("Vasiliy Zasukhin <vzasukhin@elvees.com>");
MODULE_LICENSE("GPL");
MODULE_VERSION("0.1.0");
MODULE_ALIAS("platform:vinc");
MODULE_SUPPORTED_DEVICE("video");
