/*
 * Copyright 2016 ELVEES NeoTek JSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <asm/neon.h>
#include <linux/workqueue.h>

#include "vinc-hw.h"
#include "vinc-ctrls.h"
#include "vinc-neon.h"

static u32 vinc_get_dma_src(struct vinc_dev *priv,
			    struct soc_camera_device *icd)
{
	switch (icd->current_fmt->host_fmt->fourcc) {
	case V4L2_PIX_FMT_BGR32:
		return (priv->stream[icd->devnum].cluster.ct.enable->val ?
				DMA_SRC_CT : DMA_SRC_444);
	case V4L2_PIX_FMT_M420:
		return DMA_SRC_420;
	default:
		dev_warn(priv->ici.v4l2_dev.dev, "Unknown output format %#x\n",
			 icd->current_fmt->host_fmt->fourcc);
		return DMA_SRC_CROP;
	}
}

static void set_bad_pixels(struct vinc_dev *priv, u8 channel,
			   struct vinc_bad_pixel *bp)
{
	int i;

	vinc_write(priv, STREAM_PROC_BP_MAP_CTR(channel), 0);
	for (i = 0; i < CTRL_BAD_PIXELS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_BP_MAP_DATA(channel),
			   (bp[i].y << 12) | bp[i].x);
}

static void set_bad_rows_cols(struct vinc_dev *priv, u8 channel, u16 *data,
			      int is_cols)
{
	int i;
	u32 reg_start = is_cols ? STREAM_PROC_BP_BAD_COLUMN(channel, 0) :
			STREAM_PROC_BP_BAD_LINE(channel, 0);

	for (i = 0; i < (CTRL_BAD_ROWSCOLS_COUNT / 2); i++)
		vinc_write(priv, reg_start + i * sizeof(u32),
			   (data[i * 2] & 0xFFF) |
			   ((data[i * 2 + 1] & 0xFFF) << 16));
}

static void set_gc_curve(struct vinc_dev *priv, u8 channel,
			 struct vinc_gamma_curve *gc)
{
	int i;

	vinc_write(priv, STREAM_PROC_GC_CTR(channel), 0);
	for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_GC_DATA(channel),
			   (gc->green[i] << 16) | gc->red[i]);
	vinc_write(priv, STREAM_PROC_GC_CTR(channel), 0x1000);
	for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_GC_DATA(channel),
			   gc->blue[i]);
}

static void set_dr(struct vinc_dev *priv, u8 channel, u16 *dr)
{
	int i;

	vinc_write(priv, STREAM_PROC_DR_CTR(channel), 0);
	for (i = 0; i < CTRL_DR_ELEMENTS_COUNT; i++)
		vinc_write(priv, STREAM_PROC_DR_DATA(channel), dr[i]);
}

static void set_stat_af_color(struct vinc_dev *priv, u8 channel, u32 color)
{
	u32 proc_ctr = vinc_read(priv, STREAM_PROC_CTR(channel));

	proc_ctr &= ~STREAM_PROC_CTR_AF_COLOR(0x3);
	proc_ctr |= STREAM_PROC_CTR_AF_COLOR(color);
	vinc_write(priv, STREAM_PROC_CTR(channel), proc_ctr);
}

/* TODO: Split this function into two: the one that activates cluster and
 * the other that enables/disables HW block.
 */
static void cluster_activate(struct vinc_dev *priv, u8 channel, u32 block_mask,
			     struct v4l2_ctrl **cluster)
{
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));
	struct v4l2_ctrl *master = cluster[0];
	int i;

	if (master->val)
		proc_cfg |= block_mask;
	else
		proc_cfg &= ~block_mask;
	vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);
	for (i = 1; i < master->ncontrols; i++)
		v4l2_ctrl_activate(cluster[i], master->val);
}

static void cluster_activate_auto(bool activate, struct v4l2_ctrl **cluster,
				  u8 num)
{
	int i;

	for (i = 0; i < num; i++) {
		v4l2_ctrl_activate(cluster[i], activate);
		if (!activate)
			cluster[i]->flags |= V4L2_CTRL_FLAG_VOLATILE;
		else
			cluster[i]->flags &= ~V4L2_CTRL_FLAG_VOLATILE;
	}
}

static void cluster_activate_only(struct v4l2_ctrl **cluster)
{
	struct v4l2_ctrl *master = cluster[0];
	int i;

	for (i = 1; i < master->ncontrols; i++)
		v4l2_ctrl_activate(cluster[i], master->val);
}

static void enable_block(struct vinc_dev *priv, u8 channel, u32 block_mask,
			 bool const enable)
{
	u32 proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));

	if (enable)
		proc_cfg |= block_mask;
	else
		proc_cfg &= ~block_mask;

	vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);
}

static void change_write_only(struct v4l2_ctrl **cluster,
			      u8 first, u8 last, bool const write_only)
{
	u8 i;

	for (i = first; i < last; i++)
		if (write_only) {
			cluster[i]->flags |= V4L2_CTRL_FLAG_WRITE_ONLY;
			cluster[i]->flags |=
			V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
		} else {
			cluster[i]->flags &= ~V4L2_CTRL_FLAG_WRITE_ONLY;
			cluster[i]->flags &=
			~V4L2_CTRL_FLAG_EXECUTE_ON_WRITE;
		}
}

static void activate_sensor_exp_gain(struct v4l2_ctrl_handler *hdl, u8 first,
				     u8 last, bool const activate)
{
	struct v4l2_ctrl *s_exp[3];
	int i;

	s_exp[0] = v4l2_ctrl_find(hdl, V4L2_CID_EXPOSURE);
	s_exp[1] = v4l2_ctrl_find(hdl, V4L2_CID_EXPOSURE_ABSOLUTE);
	s_exp[2] = v4l2_ctrl_find(hdl, V4L2_CID_GAIN);
	for (i = first; i <= last; i++) {
		if (!activate) {
			s_exp[i]->flags |= V4L2_CTRL_FLAG_INACTIVE;
		} else {
			s_exp[i]->flags &= ~V4L2_CTRL_FLAG_INACTIVE;
		}
	}
}

static void vinc_calculate_cdf(struct vinc_stat_hist *p_hist,
			       struct bc_stat *p_stat)
{
	int i;

	for (i = 0; i < 256; i++) {
		p_stat->hist_brightness[i] = p_hist->red[i] + p_hist->green[i] +
					p_hist->blue[i];
		if (i == 0)
			p_stat->cumulate[i] = p_stat->hist_brightness[i];
		else
			p_stat->cumulate[i] = p_stat->cumulate[i-1] +
					p_stat->hist_brightness[i];
	}
}

static int vinc_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct soc_camera_device *icd = container_of(ctrl->handler,
			struct soc_camera_device, ctrl_handler);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	struct vinc_cluster_bp *bp;
	struct vinc_cluster_gamma *gamma;
	struct vinc_cluster_cc *cc;
	struct vinc_cluster_ct *ct;
	struct vinc_cluster_dr *dr;
	struct vinc_cluster_stat *stat;
	struct vinc_cluster_exposure *exposure;
	const u8 devnum = icd->devnum;
	const u8 channel = devnum & 0x01;
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	struct vinc_stream * const stream = &priv->stream[devnum];
	u32 proc_cfg, stream_ctr;
	int rc, i, init, std_is_new, ret;

	switch (ctrl->id) {
	case V4L2_CID_BAD_CORRECTION_ENABLE:
		bp = (struct vinc_cluster_bp *)ctrl->cluster;
		/* For programming BP block we need to stop video */
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR,
			   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(channel));
		cluster_activate(priv, channel, STREAM_PROC_CFG_BPC_EN,
				 ctrl->cluster);
		if (bp->enable->val) {
			if (bp->enable->is_new || bp->pix->is_new)
				set_bad_pixels(priv, channel, bp->pix->p_new.p);
			if (bp->enable->is_new || bp->row->is_new)
				set_bad_rows_cols(priv, channel,
						  bp->row->p_new.p_u16, 0);
			if (bp->enable->is_new || bp->col->is_new)
				set_bad_rows_cols(priv, channel,
						  bp->col->p_new.p_u16, 1);
		}
		vinc_write(priv, STREAM_CTR, stream_ctr);
		break;
	case V4L2_CID_GAMMA_CURVE_ENABLE: {
		struct vinc_gamma_curve *p_gamma;

		gamma = (struct vinc_cluster_gamma *)ctrl->cluster;
		p_gamma = gamma->curve->p_cur.p;
		init = gamma->enable->is_new && gamma->gamma->is_new &&
		       gamma->curve->is_new && !stream->first_load;
		if (!init) {
			if ((gamma->gamma->is_new || gamma->bklight->is_new) &&
			    !gamma->bklight->val) {
				kernel_neon_begin();
				vinc_neon_calculate_gamma_curve(
					gamma->gamma->val,
					stream->cluster.gamma.bklight->priv, 0,
					gamma->curve->p_cur.p);
				kernel_neon_end();
				change_write_only(ctrl->cluster, 2, 3, 0);
				gamma->curve->flags |= V4L2_CTRL_FLAG_UPDATE;
			} else if (gamma->curve->is_new &&
				  !gamma->bklight->val) {
				change_write_only(ctrl->cluster, 2, 3, 1);
				gamma->curve->flags &= ~V4L2_CTRL_FLAG_UPDATE;
				p_gamma = gamma->curve->p_new.p;
			}
			if (gamma->bklight->is_new) {
				if (gamma->bklight->val) {
					change_write_only(ctrl->cluster, 2, 3,
							  0);
					gamma->curve->flags &=
						~V4L2_CTRL_FLAG_UPDATE;
				} else
					gamma->curve->flags |=
						V4L2_CTRL_FLAG_UPDATE;
			}
			if (gamma->enable->is_new)
				cluster_activate_only(ctrl->cluster);
			cluster_activate_auto(!gamma->bklight->val,
					      &gamma->curve, 1);
			stream->first_load = 0;
		}

		if (gamma->enable->val && (gamma->gamma->val != 16 ||
			gamma->gamma->flags &
			V4L2_CTRL_FLAG_WRITE_ONLY ||
			gamma->bklight->val)) {
			enable_block(priv, channel,
				     STREAM_PROC_CFG_GC_EN, true);
			if (!gamma->bklight->val)
				set_gc_curve(priv, channel, p_gamma);
		} else {
			if (!gamma->bklight->val)
				enable_block(priv, channel,
					     STREAM_PROC_CFG_GC_EN, false);
		}

		if (!init && gamma->bklight->is_new && !gamma->bklight->val) {
			if (!(stream->cluster.cc.awb->cur.val) &&
			    !(stream->cluster.cc.ab->cur.val) &&
			    stream->cluster.exp.ae->cur.val)
				cancel_work_sync(&stream->stat_work);
		}
		break;
	}
	case V4L2_CID_CC_ENABLE: {
		struct vinc_cc *p_cc;
		bool activate;
		u8 wr_only_num;

		cc = (struct vinc_cluster_cc *)ctrl->cluster;
		wr_only_num = cc->enable->ncontrols;
		p_cc = cc->cc->p_cur.p;

		/*TODO: is_new flags for other cc controls must be
		 * added to std_is_new condition
		 */
		std_is_new = cc->dowb->is_new     | cc->brightness->is_new |
			     cc->contrast->is_new | cc->saturation->is_new |
			     cc->hue->is_new      | cc->ck->is_new         |
			     cc->fx->is_new       | cc->cbcr->is_new       |
			     cc->rb->is_new       | cc->bb->is_new         |
			     cc->wbt->is_new      | cc->bl->is_new         |
			     cc->wbcc->is_new;

		init = cc->enable->is_new & cc->cc->is_new &
			std_is_new;

		cluster_activate(priv, channel, STREAM_PROC_CFG_CC_EN,
				 ctrl->cluster);
		activate = (cc->awb->val | cc->ab->val) ? 0 : 1;
		if (cc->enable->val) {
			cluster_activate_auto(activate, &cc->cc, 1);
			cluster_activate_auto(!cc->awb->val, &cc->dowb,
					      AWB_MEMBER_NUM);
			cluster_activate_auto(!cc->ab->val, &cc->brightness,
					      ABR_MEMBER_NUM);
		}
		if ((cc->dowb->is_new || cc->wbt->is_new) && !init &&
					!cc->awb->cur.val) {
			kernel_neon_begin();
			if (cc->wbt->is_new) {
				u32 temp = cc->wbt->val;
				change_write_only(ctrl->cluster, &cc->wbt - ctrl->cluster,
						  wr_only_num, 0);
				vinc_neon_wb_temp(temp, cc->wbcc->p_new.p,
						  &cc->rb->val, &cc->bb->val);
			} else {
				struct vinc_stat_add *add = &stream->summary_stat.add;
				vinc_neon_wb_stat(add->sum_r, add->sum_g, add->sum_b,
						  &cc->rb->val, &cc->bb->val);
			}
			kernel_neon_end();
			cc->rb->has_changed = 1;
			cc->bb->has_changed = 1;
		}

		if ((cc->dowb->is_new || cc->rb->is_new || cc->bb->is_new ||
			cc->wbt->is_new || cc->wbcc->is_new) && !cc->awb->cur.val) {
			kernel_neon_begin();
			vinc_neon_calculate_m_wb(cc->rb->val, cc->bb->val,
						 cc->wbcc->p_new.p, cc->dowb->priv);
			kernel_neon_end();
		}

		if ((cc->dowb->is_new || cc->rb->is_new || cc->bb->is_new) &&
		    !init && !cc->awb->cur.val) {
			change_write_only(ctrl->cluster, &cc->wbt - ctrl->cluster,
					  wr_only_num, 1);
			/* write only flag for wbt should not reset */
			wr_only_num--;
		}

		if (cc->bl->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_v_bl(cc->bl->priv, cc->bl->val);
			kernel_neon_end();
		}

		if (cc->brightness->is_new && !cc->ab->cur.val) {
			kernel_neon_begin();
			vinc_neon_calculate_v_bri(cc->brightness->priv,
						cc->brightness->val);
			kernel_neon_end();
		}

		if (cc->contrast->is_new && !cc->ab->cur.val) {
			kernel_neon_begin();
			vinc_neon_calculate_m_con(cc->contrast->priv,
						  cc->contrast->val);
			kernel_neon_end();
		}

		if (cc->saturation->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_m_sat(cc->saturation->priv,
						  cc->saturation->val);
			kernel_neon_end();
		}

		if (cc->hue->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_m_hue(cc->hue->priv, cc->hue->val);
			kernel_neon_end();
		}

		if (cc->ck->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_m_ck(cc->ck->priv, cc->ck->val);
			kernel_neon_end();
		}

		if (cc->fx->is_new || cc->cbcr->is_new) {
			kernel_neon_begin();
			vinc_neon_calculate_fx(cc->fx->priv, cc->cbcr->val,
					       cc->fx->val);
			kernel_neon_end();
		}

		if ((cc->awb->is_new || cc->ab->is_new) && !cc->awb->val &&
			!cc->ab->val && stream->cluster.exp.ae->cur.val ==
			V4L2_EXPOSURE_MANUAL &&
			!stream->cluster.gamma.bklight->cur.val)
			cancel_work_sync(&stream->stat_work);

		if (cc->awb->is_new || cc->ab->is_new) {
			if (cc->awb->val || cc->awb->val) {
				change_write_only(ctrl->cluster, &cc->ck - ctrl->cluster,
						  wr_only_num, 0);
				cc->cc->flags &= ~V4L2_CTRL_FLAG_UPDATE;
			} else
				cc->cc->flags |= V4L2_CTRL_FLAG_UPDATE;
		}

		if (std_is_new) {
			kernel_neon_begin();
			rc = vinc_neon_calculate_cc(&stream->ctrl_privs,
						    stream->ycbcr_enc,
						    cc->cc->p_cur.p);
			kernel_neon_end();

			if (rc < 0)
				return rc;

			change_write_only(ctrl->cluster, &cc->bl - ctrl->cluster,
					  wr_only_num, 0);
			cc->cc->flags |= V4L2_CTRL_FLAG_UPDATE;
		} else if (cc->cc->is_new && !cc->awb->cur.val &&
			!cc->ab->cur.val) {
			p_cc = cc->cc->p_new.p;
			change_write_only(ctrl->cluster, &cc->bl - ctrl->cluster,
					  wr_only_num, 1);
			cc->cc->flags &= ~V4L2_CTRL_FLAG_UPDATE;
		}
		set_cc_ct(priv, channel, p_cc, 0);
		break;
	}
	case V4L2_CID_CT_ENABLE:
		ct = (struct vinc_cluster_ct *)ctrl->cluster;
		proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));

		if (ctrl->val)
			proc_cfg |= STREAM_PROC_CFG_CT_EN;
		else
			proc_cfg &= ~STREAM_PROC_CFG_CT_EN;
		proc_cfg &= ~STREAM_PROC_CFG_DMA0_SRC(DMA_SRC_MASK);
		proc_cfg |= STREAM_PROC_CFG_DMA0_SRC(
				vinc_get_dma_src(priv, icd));
		vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);

		v4l2_ctrl_activate(ct->ct, ct->enable->val);
		if (ct->ct->is_new)
			set_cc_ct(priv, channel, ct->ct->p_new.p, 1);
		break;
	case V4L2_CID_DR_ENABLE:
		dr = (struct vinc_cluster_dr *)ctrl->cluster;
		/* To enable/disable DR block we need to stop video */
		stream_ctr = vinc_read(priv, STREAM_CTR);
		vinc_write(priv, STREAM_CTR,
			   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(channel));
		cluster_activate(priv, channel, STREAM_PROC_CFG_ADR_EN,
				 ctrl->cluster);
		vinc_write(priv, STREAM_CTR, stream_ctr);
		if (dr->enable->val && (dr->enable->is_new || dr->dr->is_new))
			set_dr(priv, channel, dr->dr->p_new.p_u16);
		break;
	case V4L2_CID_STAT_ENABLE:
		stat = (struct vinc_cluster_stat *)ctrl->cluster;
		if (stat->enable->is_new) {
			stream_ctr = vinc_read(priv, STREAM_CTR);
			vinc_write(priv, STREAM_CTR,
				   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(
						   channel));
			proc_cfg = vinc_read(priv, STREAM_PROC_CFG(channel));
			proc_cfg &= ~STREAM_PROC_CFG_STT_EN(0x7);
			proc_cfg |= STREAM_PROC_CFG_STT_EN(stat->enable->val);
			if (stat->enable->val)
				vinc_stat_start(stream);
			vinc_write(priv, STREAM_PROC_CFG(channel), proc_cfg);
			vinc_write(priv, STREAM_CTR, stream_ctr);
		}
		set_stat_af_color(priv, channel, stat->af_color->val);
		vinc_write(priv, STREAM_PROC_STAT_TH(channel),
			   stat->af_th->val);
		for (i = 0; i < 4; i++) {
			if (!stat->zone[i]->is_new)
				continue;
			stream_ctr = vinc_read(priv, STREAM_CTR);
			vinc_write(priv, STREAM_CTR,
				   stream_ctr & ~STREAM_CTR_STREAM_ENABLE(
						   channel));
			set_stat_zone(stream,
				      stat->zone[i]->id - V4L2_CID_STAT_ZONE0,
				      stat->zone[i]->p_new.p);
			vinc_write(priv, STREAM_CTR, stream_ctr);
		}
		break;
	case V4L2_CID_TEST_PATTERN:
		/* Test pattern can be changed only when stream is off */
		break;
	case V4L2_CID_SENSOR_AUTO_WHITE_BALANCE: {
		struct v4l2_control awb = {
			.id = V4L2_CID_AUTO_WHITE_BALANCE,
			.value = ctrl->val
		};

		ret = v4l2_subdev_s_ctrl(sd, &awb);
		return ret;
	}

	case V4L2_CID_EXPOSURE_AUTO: {
		struct  v4l2_control exp = {
			.id = V4L2_CID_EXPOSURE_AUTO
		};
		struct v4l2_control gain = {
			.id =  V4L2_CID_AUTOGAIN
		};
		exposure = (struct vinc_cluster_exposure *)ctrl->cluster;

		if (exposure->ae->is_new) {
			int activate_exp = 0, activate_gain = 0;

			if (exposure->ae->val == V4L2_EXPOSURE_AUTO) {
				exp.value = V4L2_EXPOSURE_MANUAL;
				gain.value = 0;
				exposure->sensor_ae->cur.val = 0;
				exposure->sensor_ag->cur.val = 0;

				ret = v4l2_subdev_s_ctrl(sd, &exp);
				if (ret < 0)
					return ret;
				ret = v4l2_subdev_s_ctrl(sd, &gain);
				if (ret < 0)
					return ret;
			}
			activate_exp = (exposure->ae->val ==
				V4L2_EXPOSURE_MANUAL &&
				!exposure->sensor_ae->cur.val) ? 1 : 0;
			activate_sensor_exp_gain(sd->ctrl_handler,
						 0, 1, activate_exp);
			activate_gain = (exposure->ae->val ==
				V4L2_EXPOSURE_MANUAL &&
				!exposure->sensor_ag->cur.val) ? 1 : 0;
			activate_sensor_exp_gain(sd->ctrl_handler,
						 2, 2, activate_gain);

			cluster_activate_only(ctrl->cluster);
			if (exposure->ae->val == V4L2_EXPOSURE_MANUAL &&
			    !(stream->cluster.cc.awb->cur.val) &&
			    !(stream->cluster.cc.ab->cur.val))
				cancel_work_sync(&stream->stat_work);
		} else if (exposure->ae->cur.val == V4L2_EXPOSURE_MANUAL) {
			if (exposure->sensor_ae->is_new) {
				if (exposure->sensor_ae->val)
					exp.value = V4L2_EXPOSURE_AUTO;
				else
					exp.value = V4L2_EXPOSURE_MANUAL;
				ret = v4l2_subdev_s_ctrl(sd, &exp);
				if (ret < 0)
					return ret;
				activate_sensor_exp_gain(sd->ctrl_handler, 0, 1,
						!exposure->sensor_ae->val);
			}
			if (exposure->sensor_ag->is_new) {
				gain.value = exposure->sensor_ag->val;
				ret = v4l2_subdev_s_ctrl(sd, &gain);
				if (ret < 0)
					return ret;
				activate_sensor_exp_gain(sd->ctrl_handler, 2, 2,
						!exposure->sensor_ag->val);
			}
		}
	}
	break;

	default:
		return -EINVAL;
	}
	dev_dbg(priv->ici.v4l2_dev.dev, "%s: %#x (%s), is_ptr: %d, val: %d\n",
		__func__, ctrl->id, ctrl->name, ctrl->is_ptr, ctrl->val);

	return 0;
}

/* This function will be call only for controls
 * with V4L2_CTRL_FLAG_VOLATILE flag
 */
static int vinc_g_ctrl(struct v4l2_ctrl *ctrl)
{
	struct vinc_dev *priv = ctrl->priv;

	dev_dbg(priv->ici.v4l2_dev.dev, "%s: %#x (%s)\n",
		__func__, ctrl->id, ctrl->name);

	return 0;
}

static int vinc_try_ctrl(struct v4l2_ctrl *ctrl)
{
	struct soc_camera_device *icd = container_of(ctrl->handler,
			struct soc_camera_device, ctrl_handler);
	struct soc_camera_host *ici = to_soc_camera_host(icd->parent);
	struct vinc_dev *priv = ici->priv;
	const u8 devnum = icd->devnum;
	struct vinc_stream * const stream = &priv->stream[devnum];
	struct v4l2_crop crop2 = stream->crop2;
	struct vinc_cluster_bp *bp;
	struct vinc_cluster_gamma *gamma;
	struct vinc_cluster_stat *stat;
	struct vinc_gamma_curve *gc;
	struct vinc_stat_zone *zone;
	int i;

	switch (ctrl->id) {
	case V4L2_CID_BAD_CORRECTION_ENABLE:
		bp = (struct vinc_cluster_bp *)ctrl->cluster;
		if (bp->pix->is_new) {
			struct vinc_bad_pixel *pixel = bp->pix->p_new.p;

			for (i = 0; i < CTRL_BAD_PIXELS_COUNT; i++) {
				if (((pixel[i].x > MAX_WIDTH_HEIGHT) ||
				     (pixel[i].y > MAX_WIDTH_HEIGHT)) &&
				     (pixel[i].x != 0xFFFF) &&
				     (pixel[i].y != 0xFFFF))
					return -ERANGE;
			}
		}
		if (bp->row->is_new) {
			u16 *row = bp->row->p_new.p_u16;

			for (i = 0; i < CTRL_BAD_ROWSCOLS_COUNT; i++) {
				if (row[i] > MAX_WIDTH_HEIGHT &&
				    row[i] != 0xFFFF)
					return -ERANGE;
			}
		}
		if (bp->col->is_new) {
			u16 *col = bp->col->p_new.p_u16;

			for (i = 0; i < CTRL_BAD_ROWSCOLS_COUNT; i++) {
				if (col[i] > MAX_WIDTH_HEIGHT &&
				    col[i] != 0xFFFF)
					return -ERANGE;
			}
		}
		return 0;
	case V4L2_CID_GAMMA_CURVE_ENABLE:
		gamma = (struct vinc_cluster_gamma *)ctrl->cluster;
		if (gamma->curve->is_new) {
			gc = gamma->curve->p_new.p;
			for (i = 0; i < CTRL_GC_ELEMENTS_COUNT; i++) {
				if (gc->red[i] > MAX_COMP_VALUE ||
				    gc->green[i] > MAX_COMP_VALUE ||
				    gc->blue[i] > MAX_COMP_VALUE)
					return -ERANGE;
			}
		}
		return 0;
	case V4L2_CID_STAT_ENABLE:
		stat = (struct vinc_cluster_stat *)ctrl->cluster;
		if ((u32)stat->enable->val > 7)
			return -ERANGE;

		for (i = 0; i < 4; i++) {
			zone = stat->zone[i]->p_new.p;
			if (!stat->zone[i]->is_new || !zone->enable)
				continue;
			/* Zone boundaries should not match with image
			 * boundaries for sobel filter correct calculation.
			 * See also rf#2159.
			 */
			if (zone->x_lt > MAX_WIDTH_HEIGHT ||
					zone->y_lt > MAX_WIDTH_HEIGHT ||
					zone->x_lt >= zone->x_rb ||
					zone->y_lt >= zone->y_rb ||
					zone->x_lt < 1 ||
					zone->y_lt < 1 ||
					zone->x_rb > (crop2.c.width - 2) ||
					zone->y_rb > (crop2.c.height - 2)
					)
				return -ERANGE;
		}
		return 0;
	case V4L2_CID_CT_ENABLE:
	case V4L2_CID_DR_ENABLE:
	case V4L2_CID_CC_ENABLE:
	case V4L2_CID_TEST_PATTERN:
	case V4L2_CID_SENSOR_EXPOSURE_AUTO:
	case V4L2_CID_SENSOR_AUTOGAIN:
	case V4L2_CID_SENSOR_AUTO_WHITE_BALANCE:
	case V4L2_CID_EXPOSURE_AUTO:
		return 0;
	default:
		return -EINVAL;
	}
}

static struct v4l2_ctrl_ops ctrl_ops = {
	.g_volatile_ctrl = vinc_g_ctrl,
	.s_ctrl = vinc_s_ctrl,
	.try_ctrl = vinc_try_ctrl
};

static const char * const vinc_af_color_menu[] = {
	"Red/Cr",
	"Green/Y",
	"Blue/Cb",
};

static const char * const vinc_test_pattern_menu[] = {
	"Disabled",
	"Vertical bars",
	"Diagonal stripes",
	"Horizontal bars",
	"Increment",
};

static const char * const vinc_color_effect_menu[] = {
	"Color effect is disabled",
	"Black and white",
	"Sepia tone",
	"Negative",
	"Emboss",
	"Sketch",
	"Sky blue",
	"Grass green",
	"Skin whiten",
	"Vivid colors",
	"Cool tone",
	"Frost color effect",
	"Silhouette",
	"Solarization",
	"Old photo",
	"Set Cb and Cr components"
};

static const char * const vinc_exposure_auto_menu[] = {
	"Auto Mode",
	"Manual Mode",
};

static struct v4l2_ctrl_config ctrl_cfg[] = {
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BRIGHTNESS,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -2048,
		.max = 2048,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CONTRAST,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 255,
		.step = 1,
		.def = 128,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_SATURATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 255,
		.step = 1,
		.def = 128,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_HUE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -128,
		.max = 127,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_DO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BUTTON,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_RED_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -112,
		.max = 112,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BLUE_BALANCE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = -112,
		.max = 112,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BACKLIGHT_COMPENSATION,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 10,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_GAMMA,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 31,
		.step = 1,
		.def = 16,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_COLOR_KILLER,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(vinc_color_effect_menu) - 1,
		.step = 0,
		.def = 0,
		.menu_skip_mask = 0x39F0,
		.qmenu = vinc_color_effect_menu
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_COLORFX_CBCR,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 65535,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_AUTOBRIGHTNESS,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 2000,
		.max = 9000,
		.step = 10,
		.def = 6500,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_AUTO_WHITE_BALANCE,
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_EXPOSURE_AUTO,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = V4L2_EXPOSURE_AUTO,
		.max = ARRAY_SIZE(vinc_exposure_auto_menu) - 1,
		.step = 0,
		.def = V4L2_EXPOSURE_MANUAL,
		.qmenu = vinc_exposure_auto_menu,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_CORRECTION_ENABLE,
		.name = "Bad Pixels/Rows/Columns Repair Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_PIXELS,
		.name = "Bad Pixels",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0xFF,
		.dims[0] = sizeof(struct vinc_bad_pixel) *
				CTRL_BAD_PIXELS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_ROWS,
		.name = "Bad Rows",
		.type = V4L2_CTRL_TYPE_U16,
		.min = 0,
		.max = 0xFFF,
		.step = 1,
		.def = 0xFFF,
		.dims[0] = CTRL_BAD_ROWSCOLS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BAD_COLS,
		.name = "Bad Columns",
		.type = V4L2_CTRL_TYPE_U16,
		.min = 0,
		.max = 0xFFF,
		.step = 1,
		.def = 0xFFF,
		.dims[0] = CTRL_BAD_ROWSCOLS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_GAMMA_CURVE_ENABLE,
		.name = "Gamma Curve Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_GAMMA_CURVE,
		.name = "Gamma Curve",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_gamma_curve),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CC_ENABLE,
		.name = "Color Correction Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CC,
		.name = "Color Correction",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_cc),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_BLACK_LEVEL_CORR,
		.name = "Black Level Correction",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0xFFF,
		.step = 1,
		.def = 0,
		.flags = V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_WHITE_BALANCE_CC,
		.name = "White Balance Color Correction",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_wb_cc),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD | V4L2_CTRL_FLAG_UPDATE
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CT_ENABLE,
		.name = "Color Tranformation Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_CT,
		.name = "Color Tranformation",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_cc),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_DR_ENABLE,
		.name = "Dynamic Range Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_DR,
		.name = "Dynamic Range",
		.type = V4L2_CTRL_TYPE_U16,
		.min = 0,
		.max = 0xFFF,
		.step = 1,
		.def = 0,
		.dims[0] = CTRL_DR_ELEMENTS_COUNT,
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ENABLE,
		.name = "Statistics Enable",
		.type = V4L2_CTRL_TYPE_BITMASK,
		.min = 0,
		.max = 0x7,
		.step = 0,
		.def = 0x5,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF_COLOR,
		.name = "Autofocus Component",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(vinc_af_color_menu) - 1,
		.step = 0,
		.def = 0,
		.qmenu = vinc_af_color_menu
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF_TH,
		.name = "Autofocus Threshold",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.max = 0x7FF,
		.step = 1,
		.def = 0,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ZONE0,
		.name = "Window of Zone 0",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_zone),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ZONE1,
		.name = "Window of Zone 1",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_zone),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ZONE2,
		.name = "Window of Zone 2",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_zone),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ZONE3,
		.name = "Window of Zone 3",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_zone),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_HIST0,
		.name = "Histogram in Zone 0",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_hist),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_HIST1,
		.name = "Histogram in Zone 1",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_hist),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_HIST2,
		.name = "Histogram in Zone 2",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_hist),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_HIST3,
		.name = "Histogram in Zone 3",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_hist),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF0,
		.name = "Autofocus in Zone 0",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_af),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF1,
		.name = "Autofocus in Zone 1",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_af),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF2,
		.name = "Autofocus in Zone 2",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_af),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_AF3,
		.name = "Autofocus in Zone 3",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_af),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ADD0,
		.name = "Additional Statistics in Zone 0",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_add),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ADD1,
		.name = "Additional Statistics in Zone 1",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_add),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ADD2,
		.name = "Additional Statistics in Zone 2",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_add),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_STAT_ADD3,
		.name = "Additional Statistics in Zone 3",
		.type = V4L2_CTRL_COMPOUND_TYPES,
		.min = 0,
		.max = 0xFF,
		.step = 1,
		.def = 0,
		.dims[0] = sizeof(struct vinc_stat_add),
		.flags = V4L2_CTRL_FLAG_HAS_PAYLOAD |
			 V4L2_CTRL_FLAG_READ_ONLY
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_TEST_PATTERN,
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = ARRAY_SIZE(vinc_test_pattern_menu) - 1,
		.step = 0,
		.def = 0,
		.qmenu = vinc_test_pattern_menu
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_SENSOR_EXPOSURE_AUTO,
		.name = "Sensor Autoexposure Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_SENSOR_AUTOGAIN,
		.name = "Sensor Autogain Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = 0
	},
	{
		.ops = &ctrl_ops,
		.id = V4L2_CID_SENSOR_AUTO_WHITE_BALANCE,
		.name = "Sensor Auto White Balance Enable",
		.type = V4L2_CTRL_TYPE_BOOLEAN,
		.min = 0,
		.max = 1,
		.step = 1,
		.def = 1,
		.flags = 0
	},
};

static int auto_exp_step(struct v4l2_subdev *sd, struct vinc_dev *priv,
			 struct vinc_stream *stream, struct vinc_stat_add *add,
			 struct vinc_stat_zone *zone, u32 value)
{
	struct v4l2_control cur_gain, cur_exp;
	u32 luma, gain, exp;
	int rc;

	if (value == V4L2_EXPOSURE_MANUAL)
		return 0;

	cur_gain.id = V4L2_CID_GAIN;
	cur_exp.id = V4L2_CID_EXPOSURE_ABSOLUTE;
	rc = v4l2_subdev_g_ctrl(sd, &cur_gain);
	if (rc < 0)
		return rc;
	rc = v4l2_subdev_g_ctrl(sd, &cur_exp);
	if (rc < 0)
		return rc;

	kernel_neon_begin();
	luma = vinc_neon_calculate_luma_avg(stream->input_format, add,
					    stream->ycbcr_enc, zone,
					    stream->pport_low_bits);
	vinc_neon_calculate_gain_exp(luma, cur_gain.value, cur_exp.value * 100,
				     priv->max_gain, priv->max_exp * 100, &gain,
					&exp);
	kernel_neon_end();
	if (cur_gain.value != gain) {
		cur_gain.value = gain;
		rc = v4l2_subdev_s_ctrl(sd, &cur_gain);
		if (rc < 0)
			return rc;
	}
	exp = exp / 100;
	if (cur_exp.value != exp) {
		cur_exp.value = exp;
		v4l2_subdev_s_ctrl(sd, &cur_exp);
		if (rc < 0)
			return rc;
	}
	return 0;
}

static void auto_stat_work(struct work_struct *work)
{
	struct vinc_stream *stream = container_of(work, struct vinc_stream,
						  stat_work);
	struct soc_camera_device *icd =
		container_of(stream->cluster.exp.ae->handler,
			     struct soc_camera_device, ctrl_handler);
	struct v4l2_subdev *sd = soc_camera_to_subdev(icd);
	const u8 devnum = stream->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	struct vinc_cluster_cc *cc = &stream->cluster.cc;
	struct vinc_cluster_gamma *gamma = &stream->cluster.gamma;

	struct vinc_stat_add *add = &stream->summary_stat.add;
	struct vinc_stat_hist *hist = &stream->summary_stat.hist;
	struct vinc_stat_zone *zone = stream->cluster.stat.zone[3]->p_cur.p;

	int rc;

	if (cc->ab->val || gamma->bklight->val)
		vinc_calculate_cdf(hist, stream->cluster.cc.ab->priv);

	if (cc->awb->val || cc->ab->val) {
		kernel_neon_begin();
		if (cc->awb->val) {
			vinc_neon_wb_stat(add->sum_r, add->sum_g, add->sum_b,
					  &cc->rb->val, &cc->bb->val);
			vinc_neon_calculate_m_wb(cc->rb->val, cc->bb->val,
						 cc->wbcc->p_new.p, cc->dowb->priv);
		}
		if (cc->ab->val) {
			vinc_calculate_cdf(hist, stream->cluster.cc.ab->priv);
			vinc_neon_bc_stat(stream->cluster.cc.ab->priv,
					  &cc->brightness->val,
					  &cc->contrast->val);
			vinc_neon_calculate_v_bri(cc->brightness->priv,
						  cc->brightness->val);
			vinc_neon_calculate_m_con(cc->contrast->priv,
						  cc->contrast->val);
		}

		rc = vinc_neon_calculate_cc(&stream->ctrl_privs,
					    stream->ycbcr_enc,
					    cc->cc->p_new.p);

		kernel_neon_end();

		if (rc < 0)
			return;

		cc->rb->cur.val = cc->rb->val;
		cc->bb->cur.val = cc->bb->val;

		cc->brightness->cur.val = cc->brightness->val;
		cc->contrast->cur.val = cc->contrast->val;

		memcpy(cc->cc->p_cur.p, cc->cc->p_new.p,
		       sizeof(struct vinc_cc));

		set_cc_ct(priv, channel, cc->cc->p_cur.p, 0);
	}
	if (gamma->enable->val && gamma->bklight->val) {
		u16 h, w;

		h = zone->y_rb - zone->y_lt + 1;
		w = zone->x_rb - zone->x_lt + 1;
		kernel_neon_begin();
		vinc_neon_calculate_he(cc->ab->priv, gamma->bklight->cur.val,
				       h, w, gamma->bklight->priv);
		vinc_neon_calculate_gamma_curve(gamma->gamma->cur.val,
						gamma->bklight->priv,
						gamma->bklight->cur.val,
						gamma->curve->p_cur.p);
		kernel_neon_end();
		set_gc_curve(priv, channel, gamma->curve->p_cur.p);
	}

	rc = auto_exp_step(sd, priv, stream, add, zone,
			   stream->cluster.exp.ae->val);
	if (rc < 0)
		dev_dbg(priv->ici.v4l2_dev.dev,
			"Sensor control get/set error\n");
	return;
}

void vinc_stat_tasklet(unsigned long data)
{
	struct vinc_stream *stream = (struct vinc_stream *)data;
	const u8 devnum = stream->devnum;
	const u8 channel = devnum & 0x01;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);
	struct vinc_stat_zone *zone;
	struct vinc_stat_hist *hist;
	struct vinc_stat_af *af;
	struct vinc_stat_add *add;
	u32 stat_en;
	u32 reg;
	int z, i;

	stat_en = stream->cluster.stat.enable->val;
	if (!stat_en)
		return;

	for (z = 0; z < 4; z++) {
		zone = stream->cluster.stat.zone[z]->p_cur.p;
		if (!zone->enable)
			continue;

		hist = stream->cluster.stat.hist[z]->p_cur.p;
		if (stat_en & STT_EN_HIST) {
			__u32 *component[3] = { hist->red, hist->green,
						hist->blue };
			int c;

			for (c = 0; c < ARRAY_SIZE(component); c++) {
				vinc_write(priv, STREAM_PROC_STAT_CTR(channel),
					   STREAM_PROC_STAT_CTR_NUM_ZONE(z) |
					   STREAM_PROC_STAT_CTR_COLOR_HIST(c));
				for (i = 0; i < VINC_STAT_HIST_COUNT; i++)
					component[c][i] = vinc_read(priv,
						STREAM_PROC_STAT_DATA(channel));
			}
		}
		if (stat_en & STT_EN_AF) {
			af = stream->cluster.stat.af[z]->p_cur.p;
			vinc_write(priv, STREAM_PROC_STAT_CTR(channel),
				   STREAM_PROC_STAT_CTR_NUM_ZONE(z));
			af->hsobel = vinc_read(priv,
					STREAM_PROC_STAT_HSOBEL(channel));
			af->vsobel = vinc_read(priv,
					STREAM_PROC_STAT_VSOBEL(channel));
			af->lsobel = vinc_read(priv,
					STREAM_PROC_STAT_LSOBEL(channel));
			af->rsobel = vinc_read(priv,
					STREAM_PROC_STAT_RSOBEL(channel));
		}
		if (stat_en & STT_EN_ADD) {
			add = stream->cluster.stat.add[z]->p_cur.p;
			vinc_write(priv, STREAM_PROC_STAT_CTR(channel),
				   STREAM_PROC_STAT_CTR_NUM_ZONE(z));
			reg = vinc_read(priv, STREAM_PROC_STAT_MIN(channel));
			add->min_b = reg & 0xFF;
			add->min_g = (reg >> 8) & 0xFF;
			add->min_r = (reg >> 16) & 0xFF;
			reg = vinc_read(priv, STREAM_PROC_STAT_MAX(channel));
			add->max_b = reg & 0xFF;
			add->max_g = (reg >> 8) & 0xFF;
			add->max_r = (reg >> 16) & 0xFF;
			add->sum_b = vinc_read(priv,
					       STREAM_PROC_STAT_SUM_B(channel));
			add->sum_g = vinc_read(priv,
					       STREAM_PROC_STAT_SUM_G(channel));
			add->sum_r = vinc_read(priv,
					       STREAM_PROC_STAT_SUM_R(channel));
			reg = vinc_read(priv,
					STREAM_PROC_STAT_SUM2_HI(channel));
			add->sum2_b = reg & 0xFF;
			add->sum2_g = (reg >> 8) & 0xFF;
			add->sum2_r = (reg >> 16) & 0xFF;
			add->sum2_b = (add->sum2_b << 32) |
				vinc_read(priv,
					  STREAM_PROC_STAT_SUM2_B(channel));
			add->sum2_g = (add->sum2_g << 32) |
				vinc_read(priv,
					  STREAM_PROC_STAT_SUM2_G(channel));
			add->sum2_r = (add->sum2_r << 32) |
				vinc_read(priv,
					  STREAM_PROC_STAT_SUM2_R(channel));
		}
	}
	vinc_write(priv, STREAM_PROC_CLEAR(channel),
			STREAM_PROC_CLEAR_AF_CLR | STREAM_PROC_CLEAR_ADD_CLR);

	spin_lock(&stream->lock);
	if (!stream->active) {
		vinc_stream_enable(priv, channel, false);
		stream->stat_odd = true;
	}
	spin_unlock(&stream->lock);

	/* Calculate summary statistics, used inside the driver (histogram and
	 * sum)
	 */
	memset(&stream->summary_stat.add, 0, sizeof(struct vinc_stat_add));
	memset(&stream->summary_stat.hist, 0, sizeof(struct vinc_stat_hist));

	for (z = 0; z < 4; z++) {
		zone = stream->cluster.stat.zone[z]->p_cur.p;
		add = stream->cluster.stat.add[z]->p_cur.p;
		hist = stream->cluster.stat.hist[z]->p_cur.p;
		if (!zone->enable)
			continue;
		if (stat_en & STT_EN_ADD) {
			stream->summary_stat.add.sum_r += add->sum_r;
			stream->summary_stat.add.sum_g += add->sum_g;
			stream->summary_stat.add.sum_b += add->sum_b;
		}
		if (stat_en & STT_EN_HIST) {
			for (i = 0; i < VINC_STAT_HIST_COUNT; i++) {
				stream->summary_stat.hist.red[i] +=
					hist->red[i];
				stream->summary_stat.hist.green[i] +=
					hist->green[i];
				stream->summary_stat.hist.blue[i] +=
					hist->blue[i];
			}
		}
	}

	schedule_work(&stream->stat_work);
}

int vinc_create_controls(struct v4l2_ctrl_handler *hdl,
			 struct vinc_stream *stream)
{
	int i;
	struct v4l2_ctrl *tmp_ctrl;
	const u8 devnum = stream->devnum;
	struct vinc_dev *priv = container_of(stream, struct vinc_dev,
					     stream[devnum]);

	for (i = 0; i < ARRAY_SIZE(ctrl_cfg); i++) {
		if (!ctrl_cfg[i].name)
			ctrl_cfg[i].name = v4l2_ctrl_get_name(ctrl_cfg[i].id);

		tmp_ctrl = v4l2_ctrl_new_custom(hdl, &ctrl_cfg[i], NULL);
		if (!tmp_ctrl) {
			dev_err(priv->ici.v4l2_dev.dev,
				"Can not create control %#x\n", ctrl_cfg[i].id);
			return hdl->error;
		}
	}
	stream->first_load = 1;
	stream->cluster.bp.enable = v4l2_ctrl_find(hdl,
			V4L2_CID_BAD_CORRECTION_ENABLE);
	stream->cluster.bp.pix = v4l2_ctrl_find(hdl, V4L2_CID_BAD_PIXELS);
	stream->cluster.bp.row = v4l2_ctrl_find(hdl, V4L2_CID_BAD_ROWS);
	stream->cluster.bp.col = v4l2_ctrl_find(hdl, V4L2_CID_BAD_COLS);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_bp),
			  &stream->cluster.bp.enable);

	stream->cluster.gamma.enable = v4l2_ctrl_find(hdl,
			V4L2_CID_GAMMA_CURVE_ENABLE);
	stream->cluster.gamma.curve = v4l2_ctrl_find(hdl, V4L2_CID_GAMMA_CURVE);
	stream->cluster.gamma.gamma = v4l2_ctrl_find(hdl, V4L2_CID_GAMMA);
	stream->cluster.gamma.bklight = v4l2_ctrl_find(hdl,
			V4L2_CID_BACKLIGHT_COMPENSATION);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_gamma),
			  &stream->cluster.gamma.enable);

	stream->cluster.cc.enable = v4l2_ctrl_find(hdl, V4L2_CID_CC_ENABLE);
	stream->cluster.cc.cc = v4l2_ctrl_find(hdl, V4L2_CID_CC);
	stream->cluster.cc.brightness = v4l2_ctrl_find(hdl,
			V4L2_CID_BRIGHTNESS);
	stream->cluster.cc.contrast = v4l2_ctrl_find(hdl, V4L2_CID_CONTRAST);
	stream->cluster.cc.saturation = v4l2_ctrl_find(hdl,
						       V4L2_CID_SATURATION);
	stream->cluster.cc.hue = v4l2_ctrl_find(hdl, V4L2_CID_HUE);
	stream->cluster.cc.dowb = v4l2_ctrl_find(hdl,
			V4L2_CID_DO_WHITE_BALANCE);
	stream->cluster.cc.ck = v4l2_ctrl_find(hdl, V4L2_CID_COLOR_KILLER);
	stream->cluster.cc.fx = v4l2_ctrl_find(hdl, V4L2_CID_COLORFX);
	stream->cluster.cc.cbcr = v4l2_ctrl_find(hdl, V4L2_CID_COLORFX_CBCR);
	stream->cluster.cc.rb = v4l2_ctrl_find(hdl, V4L2_CID_RED_BALANCE);
	stream->cluster.cc.bb = v4l2_ctrl_find(hdl, V4L2_CID_BLUE_BALANCE);
	stream->cluster.cc.wbt = v4l2_ctrl_find(hdl,
			V4L2_CID_WHITE_BALANCE_TEMPERATURE);
	stream->cluster.cc.awb = v4l2_ctrl_find(hdl,
			V4L2_CID_AUTO_WHITE_BALANCE);
	stream->cluster.cc.ab = v4l2_ctrl_find(hdl,
			V4L2_CID_AUTOBRIGHTNESS);
	stream->cluster.cc.bl = v4l2_ctrl_find(hdl, V4L2_CID_BLACK_LEVEL_CORR);
	stream->cluster.cc.wbcc = v4l2_ctrl_find(hdl, V4L2_CID_WHITE_BALANCE_CC);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_cc),
			  &stream->cluster.cc.enable);

	stream->cluster.ct.enable = v4l2_ctrl_find(hdl, V4L2_CID_CT_ENABLE);
	stream->cluster.ct.ct = v4l2_ctrl_find(hdl, V4L2_CID_CT);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_ct),
			  &stream->cluster.ct.enable);

	stream->cluster.dr.enable = v4l2_ctrl_find(hdl, V4L2_CID_DR_ENABLE);
	stream->cluster.dr.dr = v4l2_ctrl_find(hdl, V4L2_CID_DR);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_dr),
			  &stream->cluster.dr.enable);

	stream->cluster.stat.enable = v4l2_ctrl_find(hdl, V4L2_CID_STAT_ENABLE);
	stream->cluster.stat.af_color = v4l2_ctrl_find(hdl,
			V4L2_CID_STAT_AF_COLOR);
	stream->cluster.stat.af_th = v4l2_ctrl_find(hdl, V4L2_CID_STAT_AF_TH);
	for (i = 0; i < 4; i++) {
		stream->cluster.stat.zone[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_ZONE0 + i);
		stream->cluster.stat.hist[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_HIST0 + i);
		stream->cluster.stat.af[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_AF0 + i);
		stream->cluster.stat.add[i] = v4l2_ctrl_find(hdl,
				V4L2_CID_STAT_ADD0 + i);
	}
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_stat),
			  &stream->cluster.stat.enable);

	stream->cluster.exp.sensor_ae = v4l2_ctrl_find(hdl,
						V4L2_CID_SENSOR_EXPOSURE_AUTO);
	stream->cluster.exp.sensor_ag = v4l2_ctrl_find(hdl,
						V4L2_CID_SENSOR_AUTOGAIN);
	stream->cluster.exp.ae = v4l2_ctrl_find(hdl, V4L2_CID_EXPOSURE_AUTO);
	v4l2_ctrl_cluster(CLUSTER_SIZE(struct vinc_cluster_exposure),
			  &stream->cluster.exp.ae);

	stream->test_pattern = v4l2_ctrl_find(hdl, V4L2_CID_TEST_PATTERN);

	stream->sensor_awb = v4l2_ctrl_find(hdl,
					    V4L2_CID_SENSOR_AUTO_WHITE_BALANCE);

	stream->cluster.cc.bl->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev,
			sizeof(struct vector), GFP_KERNEL);
	stream->cluster.gamma.bklight->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev,
			256 * sizeof(u32), GFP_KERNEL);
	stream->cluster.cc.brightness->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev,
			sizeof(struct vector), GFP_KERNEL);
	stream->cluster.cc.contrast->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.saturation->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.hue->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.ck->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct matrix),
			GFP_KERNEL);
	stream->cluster.cc.fx->priv = devm_kmalloc(
			priv->ici.v4l2_dev.dev, sizeof(struct col_fx),
			GFP_KERNEL);
	stream->cluster.cc.dowb->priv = kzalloc(sizeof(struct matrix),
						GFP_KERNEL);
	stream->cluster.cc.ab->priv = devm_kmalloc(priv->ici.v4l2_dev.dev,
						   sizeof(struct bc_stat),
						   GFP_KERNEL);

	stream->ctrl_privs.blacklevel = stream->cluster.cc.bl->priv;
	stream->ctrl_privs.dowb       = stream->cluster.cc.dowb->priv;
	stream->ctrl_privs.brightness = stream->cluster.cc.brightness->priv;
	stream->ctrl_privs.contrast   = stream->cluster.cc.contrast->priv;
	stream->ctrl_privs.saturation = stream->cluster.cc.saturation->priv;
	stream->ctrl_privs.hue        = stream->cluster.cc.hue->priv;
	stream->ctrl_privs.ck         = stream->cluster.cc.ck->priv;
	stream->ctrl_privs.fx         = stream->cluster.cc.fx->priv;

	for (i = 0; i < ARRAY_SIZE(priv->stream); i++)
		INIT_WORK(&priv->stream[i].stat_work, auto_stat_work);

	return hdl->error;
}
