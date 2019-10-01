/*
 * ELVEES Avico (a.k.a. VELcore-01) driver - Bitstream functions
 *
 * Copyright 2015 ELVEES NeoTek JSC
 * Copyright 2017 RnD Center "ELVEES", JSC
 *
 * Author: Anton Leontiev <aleontiev@elvees.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/types.h>

#include "avico.h"
#include "avico-bitstream.h"

#define MAX_SPS_COUNT 32
#define MAX_PPS_COUNT 256
#define EXTENDED_SAR 255

/* video_format (Rec. ITU-T H.264) */
enum h264_format {
	H264_FMT_COMPONENT = 0,
	H264_FMT_PAL   = 1,
	H264_FMT_NTSC  = 2,
	H264_FMT_SECAM = 3,
	H264_FMT_MAC   = 4,
	H264_FMT_UNSPECIFIED = 5,
	H264_FMT_NB /* Not part of ABI */
};

/* Colour primaries (Rec. ITU-T H.264) */
enum h264_colorprimaries {
	H264_PRI_RESERVED0   = 0,
	H264_PRI_BT709       = 1,
	H264_PRI_UNSPECIFIED = 2,
	H264_PRI_RESERVED    = 3,
	H264_PRI_BT470M      = 4,
	H264_PRI_BT470BG     = 5,
	H264_PRI_SMPTE170M   = 6,
	H264_PRI_SMPTE240M   = 7,
	H264_PRI_FILM        = 8,
	H264_PRI_BT2020      = 9,
	H264_PRI_SMPTE428    = 10,
	H264_PRI_SMPTEST428_1 = H264_PRI_SMPTE428,
	H264_PRI_SMPTE431    = 11,
	H264_PRI_SMPTE432    = 12,
	H264_PRI_JEDEC_P22   = 22,
	H264_PRI_NB /* Not part of ABI */
};

/* Transfer characteristics (Rec. ITU-T H.264) */
enum h264_colortransfer {
	H264_TRC_RESERVED0    = 0,
	H264_TRC_BT709        = 1,
	H264_TRC_UNSPECIFIED  = 2,
	H264_TRC_RESERVED     = 3,
	H264_TRC_GAMMA22      = 4,
	H264_TRC_GAMMA28      = 5,
	H264_TRC_SMPTE170M    = 6,
	H264_TRC_SMPTE240M    = 7,
	H264_TRC_LINEAR       = 8,
	H264_TRC_LOG          = 9,
	H264_TRC_LOG_SQRT     = 10,
	H264_TRC_IEC61966_2_4 = 11,
	H264_TRC_BT1361_ECG   = 12,
	H264_TRC_IEC61966_2_1 = 13,
	H264_TRC_BT2020_10    = 14,
	H264_TRC_BT2020_12    = 15,
	H264_TRC_SMPTE2084    = 16,
	H264_TRC_SMPTE428     = 17,
	H264_TRC_ARIB_STD_B67 = 18,
	H264_TRC_NB /* Not part of ABI */
};

/* Matrix coefficients (Rec. ITU-T H.264) */
enum h264_matrix {
	H264_MTX_RGB         = 0,
	H264_MTX_BT709       = 1,
	H264_MTX_UNSPECIFIED = 2,
	H264_MTX_RESERVED    = 3,
	H264_MTX_FCC         = 4,
	H264_MTX_BT470BG     = 5,
	H264_MTX_SMPTE170M   = 6,
	H264_MTX_SMPTE240M   = 7,
	H264_MTX_YCGCO       = 8,
	H264_MTX_BT2020_NCL  = 9,
	H264_MTX_BT2020_CL   = 10,
	H264_MTX_SMPTE2085   = 11,
	H264_MTX_CHROMA_DERIVED_NCL = 12,
	H264_MTX_CHROMA_DERIVED_CL = 13,
	H264_MTX_ICTCP       = 14,
	H264_MTX_NB /* Not part of ABI */
};

struct h264_signal_type {
	enum h264_format f;
	enum h264_matrix m;
	enum h264_colorprimaries p;
	enum h264_colortransfer t;
	int r;
};

static const struct v4l2_fract aspect_ratio_table[17] = {
	{   0,  1 },
	{   1,  1 },
	{  12, 11 },
	{  10, 11 },
	{  16, 11 },
	{  40, 33 },
	{  24, 11 },
	{  20, 11 },
	{  32, 11 },
	{  80, 33 },
	{  18, 11 },
	{  15, 11 },
	{  64, 33 },
	{ 160, 99 },
	{   4,  3 },
	{   3,  2 },
	{   2,  1 },
};

static inline int get_bits_left(struct bitstream *bs)
{
	if (bs->p <= bs->end)
		return (bs->end - bs->p) * 8 + bs->bitsleft;
	else if (bs->p - 1 == bs->end && bs->bitsleft == 8)
		return 0;

	/* Negative number of overread/overwritten bits */
	return (bs->end - bs->p) * 8 - (8 - bs->bitsleft);
}

static int read_lzb(struct bitstream *bs)
{
	uint8_t cb, cbmask;
	int lzb;

	/* unexpected end of bitstream */
	if (bs->p > bs->end)
		return -EINVAL;

	cbmask = (1 << bs->bitsleft) - 1;
	cb = *bs->p & cbmask;
	lzb = bs->bitsleft - fls(cb);
	bs->bitsleft -= lzb;

	while (cb == 0) {
		unsigned int z;

		bs->bitsleft = 8;

		/* unexpected end of bitstream */
		if (++bs->p > bs->end)
			return -EINVAL;

		/* skip emulation_prevention_three_byte */
		if (bs->nulls == 2 && *bs->p == 3) {
			bs->p++;
			bs->nulls = 0;
		}
		if (*bs->p == 0)
			bs->nulls++;
		else
			bs->nulls = 0;

		cb = *bs->p;
		z = 8 - fls(cb);
		bs->bitsleft -= z;
		lzb += z;
	}

	return lzb;
}

static uint32_t readu(struct bitstream *bs, uint8_t bits)
{
	uint32_t value = 0;

	while (bits) {
		unsigned int const readbits = min(bs->bitsleft, bits);
		unsigned int const cbmask = (1 << readbits) - 1;
		/* Number of top readbits */
		unsigned int const read = *bs->p >> (bs->bitsleft - readbits);

		/* unexpected end of bitstream */
		if (bs->p > bs->end) {
			__WARN();
			return value;
		}

		value <<= readbits;
		value |= read & cbmask;
		bits -= readbits; /* Decrease number of bits to be read */
		bs->bitsleft -= readbits;

		if (bs->bitsleft == 0) {
			bs->bitsleft = 8;
			bs->p++;
			/* skip emulation_prevention_three_byte */
			if (bs->nulls == 2 && *bs->p == 3) {
				bs->p++;
				bs->nulls = 0;
			}
			if (*bs->p == 0)
				bs->nulls++;
			else
				bs->nulls = 0;
		}
	}

	return value;
}

static uint32_t readue(struct bitstream *bs)
{
	uint32_t value, vmask;
	int lzb = read_lzb(bs);

	/* bad bitstream */
	WARN_ON(lzb < 0 || lzb > 31);

	vmask = (1 << lzb) - 1;
	value = readu(bs, lzb + 1) & vmask;

	return vmask + value;
}

static int32_t readse(struct bitstream *bs)
{
	uint32_t ue = readue(bs);
	int32_t se = DIV_ROUND_UP(ue, 2);

	return ue % 2 ? se : -se;
}

int avico_bitstream_read_delimiter(struct bitstream *bs)
{
	for (; bs->p <= bs->end; ++bs->p) {
		if (*bs->p == 1 && bs->nulls >= 2) {
			bs->nulls = 0;
			++bs->p;

			return 0;
		}

		if (*bs->p == 0)
			bs->nulls++;
		else
			bs->nulls = 0;
	}

	/* No start code found  */
	return -EINVAL;
}

int avico_bitstream_read_nalu_type(struct avico_ctx *ctx,
				   unsigned int *nalu_type,
				   unsigned int *nalu_priority)
{
	struct bitstream *const bs = &ctx->bs;

	if ((*bs->p & 0x80) == 1) { /* forbidden_zero_bit */
		v4l2_err(&ctx->dev->v4l2_dev, "invalid forbidden_zero_bit\n");

		return -EINVAL;
	}
	*nalu_priority = *bs->p >> 5; /* nal_ref_idc */
	*nalu_type = *bs->p & 0x1f; /* nal_unit_type */

	bs->p++;
	bs->bitsleft = 8;
	bs->nulls = 0;

	return 0;
}

static void h264_to_v4l2_colorpsace(struct avico_ctx *ctx,
				    enum h264_matrix s,
				    enum h264_colorprimaries p,
				    enum h264_colortransfer t, int full_range)
{
	struct avico_sps *const sps = &ctx->sps;

	sps->colorspace = 0;
	sps->range = 0;
	sps->matrix = 0;
	sps->transfer = 0;

	/* We first pick the main colorspace from the primaries */
	switch (p) {
	case H264_PRI_BT709:
	  /* There is two colorspaces using these primaries, use the range to
	   * differentiate */
		if (full_range == 0)
			sps->colorspace = V4L2_COLORSPACE_REC709;
		else
			sps->colorspace = V4L2_COLORSPACE_SRGB;
		break;
	case H264_PRI_BT2020:
		sps->colorspace = V4L2_COLORSPACE_BT2020;
		break;
	case H264_PRI_BT470M:
		sps->colorspace = V4L2_COLORSPACE_470_SYSTEM_M;
		break;
	case H264_PRI_BT470BG:
		sps->colorspace = V4L2_COLORSPACE_470_SYSTEM_BG;
		break;
	case H264_PRI_SMPTE170M:
		sps->colorspace = V4L2_COLORSPACE_SMPTE170M;
		break;
	case H264_PRI_SMPTE240M:
		sps->colorspace = V4L2_COLORSPACE_SMPTE240M;
		break;
	case H264_PRI_FILM:
	case H264_PRI_UNSPECIFIED:
		/* We don't know, we will guess */
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Unknown colorimetry primaries %d", p);
		break;
	}

	if (full_range == 0)
		sps->range = V4L2_QUANTIZATION_FULL_RANGE;
	else
		sps->range = V4L2_QUANTIZATION_LIM_RANGE;

	switch (s) {
	case H264_MTX_RGB:
		/* Unspecified, leave to default */
		break;
	/* FCC is about the same as BT601 with less digit */
	case H264_MTX_FCC:
	case H264_MTX_SMPTE170M:
		sps->matrix = V4L2_YCBCR_ENC_601;
		break;
	case H264_MTX_BT709:
		sps->matrix = V4L2_YCBCR_ENC_709;
		break;
	case H264_MTX_SMPTE240M:
		sps->matrix = V4L2_YCBCR_ENC_SMPTE240M;
		break;
	case H264_MTX_BT2020_NCL:
	case H264_MTX_BT2020_CL:
		sps->matrix = V4L2_YCBCR_ENC_BT2020;
		break;
	case H264_MTX_UNSPECIFIED:
		/* We let the driver pick a default one */
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Unknown colorimetry matrix %d", sps->matrix);
		break;
	}

	switch (t) {
	case H264_TRC_GAMMA22:
	case H264_TRC_GAMMA28:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "GAMMA 28, 22 transfer functions not supported");
		/* fallthrough */
	case H264_TRC_LINEAR:
		sps->transfer = V4L2_XFER_FUNC_NONE;
		break;
	case H264_TRC_BT2020_12:
	case H264_TRC_BT709:
		sps->transfer = V4L2_XFER_FUNC_709;
		break;
	case H264_TRC_SMPTE240M:
		sps->transfer = V4L2_XFER_FUNC_SMPTE240M;
		break;
	case H264_TRC_IEC61966_2_1:
		sps->transfer = V4L2_XFER_FUNC_SRGB;
		break;
	case H264_TRC_LOG:
	case H264_TRC_LOG_SQRT:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "LOG 100, 316 transfer functions not supported");
		/* FIXME No known sensible default, maybe AdobeRGB ? */
		break;
	case H264_TRC_SMPTE2084:
		sps->transfer = V4L2_XFER_FUNC_SMPTE2084;
		break;
	case H264_TRC_UNSPECIFIED:
		/* We let the driver pick a default one */
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Unknown colorimetry transfer %d", sps->transfer);
		break;
	}

	/* Try to guess colorspace according to size */
	if (sps->colorspace == 0) {
		int width  = 16 * sps->mb_width;
		int height = 16 * sps->mb_height;

		/* SD streams likely use SMPTE170M and HD streams REC709 */
		if (width <= 720 && height <= 576)
			sps->colorspace = V4L2_COLORSPACE_SMPTE170M;
		else
			sps->colorspace = V4L2_COLORSPACE_REC709;
	}
}

static inline int decode_vui_parameters(struct avico_ctx *ctx)
{
	struct avico_sps *const sps = &ctx->sps;
	struct bitstream *const bs = &ctx->bs;
	enum h264_colorprimaries cp = H264_PRI_UNSPECIFIED;
	enum h264_colortransfer ct = H264_TRC_UNSPECIFIED;
	enum h264_matrix cs = H264_MTX_UNSPECIFIED;
	int full_range = 0;

	sps->sar_flag = readu(bs, 1); /* aspect_ratio_info_present_flag */
	if (sps->sar_flag) {
		unsigned int aspect_ratio_idc = readu(bs, 8);

		if (aspect_ratio_idc == EXTENDED_SAR) {
			sps->sar.numerator = readu(bs, 16);
			sps->sar.denominator = readu(bs, 16);
		} else if (aspect_ratio_idc < ARRAY_SIZE(aspect_ratio_table)) {
			sps->sar = aspect_ratio_table[aspect_ratio_idc];
		} else {
			v4l2_err(&ctx->dev->v4l2_dev, "illegal aspect ratio\n");

			return -EINVAL;
		}
	} else {
		sps->sar.numerator = sps->sar.denominator = 0;
	}

	if (readu(bs, 1)) /* overscan_info_present_flag */
		readu(bs, 1);	/* skip overscan_appropriate_flag */

	if (readu(bs, 1)) { /* video_signal_type_present_flag */
		readu(bs, 3); /* skip video_format */
		full_range = readu(bs, 1); /* video_full_range_flag */

		if (readu(bs, 1)) { /* colour_description_present_flag */
			cp = readu(bs, 8); /* colour_primaries */
			ct = readu(bs, 8); /* transfer_characteristics */
			cs = readu(bs, 8); /* matrix_coefficients */
		}
	}
	h264_to_v4l2_colorpsace(ctx, cs, cp, ct, full_range);

	if (readu(bs, 1)) { /* chroma_loc_info_present_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "chroma_loc_info not supported.\n");

		return -EINVAL;
	}

	sps->timing_flag = readu(bs, 1); /* timing_info_present_flag */
	if (sps->timing_flag) {
		unsigned int num_units_in_tick = readu(bs, 32);
		unsigned int time_scale        = readu(bs, 32);

		if (num_units_in_tick == 0 || time_scale == 0) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "timing_info invalid or unsupported (%u/%u)\n",
				 time_scale, num_units_in_tick);
			sps->timing_flag = 0;
		} else {
			sps->timing.numerator = num_units_in_tick;
			sps->timing.denominator = time_scale;
		}
		sps->fixed_framerate = readu(bs, 1); /* fixed_frame_rate_flag */
	}

	if (readu(bs, 1)) { /* nal_hrd_parameters_present_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "nal_hrd_parameters_present_flag not supported.\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* vcl_hrd_parameters_present_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "vcl_hrd_parameters not supported.\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* pic_struct_present_flag */
		v4l2_err(&ctx->dev->v4l2_dev, "pic_struct not supported.\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* bitstream_restriction_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "bitstream restriction parameters not supported.\n");

		return -EINVAL;
	}

	return 0;
}

int avico_bitstream_read_sps(struct avico_ctx *ctx)
{
	struct bitstream *const bs = &ctx->bs;
	struct avico_sps *const sps = &ctx->sps;
	int bits_left;
	int vui_flag;
	unsigned int ref_frame_count;
	int frame_mbs_only_flag;

	/* profile should be Constrained Baseline */
	if (readu(bs, 8) != 66) { /* profile_idc */
		v4l2_err(&ctx->dev->v4l2_dev,
			"profile should be Constrained Baseline.\n");

		return -EINVAL;
	}
	if (readu(bs, 8) != 0x40) { /* constrained_set_flags */
		v4l2_err(&ctx->dev->v4l2_dev,
			"profile should be Constrained Baseline.\n");

		return -EINVAL;
	}

	/* level should be up to 4.0*/
	if (readu(bs, 8) > 40) { /* level_idc  */
		v4l2_err(&ctx->dev->v4l2_dev, "level should be 4.0 or less.\n");

		return -EINVAL;
	}

	sps->sps_id = readue(bs); /* seq_parameter_set_id */
	if (sps->sps_id >= MAX_SPS_COUNT) {
		v4l2_err(&ctx->dev->v4l2_dev, "sps_id is out of range.\n");

		return -EINVAL;
	}

	/* log2_max_frame_num_minus4 */
	sps->log2_max_frame_num = readue(bs) + 4;
	if (sps->log2_max_frame_num < 4 ||
	    sps->log2_max_frame_num > 16) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "log2_max_frame_num_minus4 is out of range.\n");

		return -EINVAL;
	}

	sps->poc_type = readue(bs); /* pic_order_cnt_type */
	if (sps->poc_type == 0) {
		/* log2_max_pic_order_cnt_lsb_minus4 */
		sps->log2_max_poc_lsb = readue(bs) + 4;

		if (sps->log2_max_poc_lsb > 16) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "log2_max_poc_lsb (%d) is out of range\n",
				 sps->log2_max_poc_lsb);

			return -EINVAL;
		}
	} else if (sps->poc_type == 1) {
		int i;

		sps->delta_pic_order_always_zero_flag = readu(bs, 1);
		sps->offset_for_non_ref_pic           = readse(bs);
		sps->offset_for_top_to_bottom_field   = readse(bs);
		sps->poc_cycle_length                 = readue(bs);

		if ((unsigned int)sps->poc_cycle_length >=
		    ARRAY_SIZE(sps->offset_for_ref_frame)) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "poc_cycle_length overflow (%d)\n",
				 sps->poc_cycle_length);

			return -EINVAL;
		}

		for (i = 0; i < sps->poc_cycle_length; i++)
			sps->offset_for_ref_frame[i] = readse(bs);
	} else if (sps->poc_type != 2) {
		v4l2_err(&ctx->dev->v4l2_dev, "illegal POC type %d\n",
			 sps->poc_type);

		return -EINVAL;
	}

	ref_frame_count = readue(bs); /* max_num_ref_frames */
	if (ref_frame_count > 1) {
		v4l2_err(&ctx->dev->v4l2_dev,
			"too many reference frames %d, must be 1\n",
			ref_frame_count);

		return -EINVAL;
	}

	sps->gaps_allowed = readu(bs, 1); /* gaps_in_frame_num_allowed_flag */
	sps->mb_width = readue(bs) + 1; /* pic_width_in_mbs_minus1 */
	if (sps->mb_width > AVICO_WMAX / 16) {
		v4l2_err(&ctx->dev->v4l2_dev, "width overflow\n");

		return -EINVAL;
	}
	sps->mb_height = readue(bs) + 1; /* pic_height_in_map_units_minus1 */

	frame_mbs_only_flag = readu(bs, 1);
	/* Constrained Baseline doesn't support frame_mbs_only_flag = 0 */
	if (frame_mbs_only_flag != 1) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "frame_mbs_only_flag (%d) isn't supported\n",
			 frame_mbs_only_flag);

		return -EINVAL;
	}
	sps->mb_height *= 2 - frame_mbs_only_flag;

	if (sps->mb_height > AVICO_HMAX / 16) {
		v4l2_err(&ctx->dev->v4l2_dev, "height overflow\n");

		return -EINVAL;
	}

	readu(bs, 1); /* skip direct_8x8_inference_flag */

	sps->crop_flag = readu(bs, 1); /* frame_cropping_flag */
	if (sps->crop_flag) {
		unsigned int crop_left   = readue(bs);
		unsigned int crop_right  = readue(bs);
		unsigned int crop_top    = readue(bs);
		unsigned int crop_bottom = readue(bs);
		int width  = 16 * sps->mb_width;
		int height = 16 * sps->mb_height;

		if ((crop_left + crop_right) * 2 >= width ||
		    (crop_top  + crop_bottom) * 2 >= height) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "crop values are invalid (%d %d %d %d)\n",
				 crop_left, crop_right, crop_top, crop_bottom);

			return -EINVAL;
		}

		sps->crop.left   = crop_left   * 2;
		sps->crop.right  = crop_right  * 2;
		sps->crop.top    = crop_top    * 2;
		sps->crop.bottom = crop_bottom * 2;
	} else {
		sps->crop.left = sps->crop.right = 0;
		sps->crop.top = sps->crop.bottom = 0;
	}

	vui_flag = readu(bs, 1); /* vui_parameters_present_flag */
	if (vui_flag) {
		int ret = decode_vui_parameters(ctx);

		if (ret < 0)
			return ret;
	}

	bits_left = get_bits_left(bs);
	if (bits_left < 0) {
		v4l2_err(&ctx->dev->v4l2_dev, "Overread %s by %d bits\n",
			 vui_flag ? "VUI" : "SPS", -bits_left);

		return -EINVAL;
	}

	return 0;
}

int avico_bitstream_read_pps(struct avico_ctx *ctx)
{
	struct bitstream *const bs = &ctx->bs;
	struct avico_pps *const pps = &ctx->pps;
	int bits_left;

	pps->pps_id = readue(bs);
	if (pps->pps_id >= MAX_PPS_COUNT) {
		v4l2_err(&ctx->dev->v4l2_dev, "pps_id %u is out of range\n",
			 pps->pps_id);

		return -EINVAL;
	}

	pps->sps_id = readue(bs);
	if (pps->sps_id >= MAX_SPS_COUNT) {
		v4l2_err(&ctx->dev->v4l2_dev, "sps_id %u is out of range\n",
			 pps->sps_id);

		return -EINVAL;
	}

	if (pps->sps_id != ctx->sps.sps_id) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "Multiple SPS aren't supported\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* entropy_coding_mode_flag */
		v4l2_err(&ctx->dev->v4l2_dev, "cabac isn't supported\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* bottom_field_pic_order_in_frame_present_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "delta_pic_order_cnt isn't supported\n");

		return -EINVAL;
	}

	if (readue(bs) > 0) { /* num_slice_groups_minus1 */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "num_slice_groups > 1 isn't supported\n");

		return -EINVAL;
	}

	if (readue(bs) > 0) { /* num_ref_idx_l0_default_active_minus1 */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "num_ref_idx_l0_active > 1 isn't supported\n");

		return -EINVAL;
	}

	if (readue(bs) > 0) { /* num_ref_idx_l1_active_minus1 */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "num_ref_idx_l1_active > 1 isn't supported\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* weighted_pred_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "explicit weighted prediction isn't supported\n");

		return -EINVAL;
	}

	if (readu(bs, 2)) { /* weighted_bipred_idc  */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "weighted_bipred_idc > 0 isn't supported\n");

		return -EINVAL;
	}

	pps->init_qp = readse(bs) + 26; /* pic_init_qp_minus26 */
	pps->init_qs = readse(bs) + 26;
	pps->chroma_qp_index_offset = readse(bs);
	if (pps->chroma_qp_index_offset < -12 ||
	    pps->chroma_qp_index_offset > 12) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "chroma_qp_index_offset is out of range\n");

		return -EINVAL;
	}

	/* deblocking_filter_control_present */
	pps->dbf_ctrl_present = readu(bs, 1);
	if (pps->dbf_ctrl_present == 0) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "deblocking_filter isn't supported\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* constrained_intra_pred_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "constrained intra prediction isn't supported\n");

		return -EINVAL;
	}

	if (readu(bs, 1)) { /* redundant_pic_cnt_present_flag */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "redundant_pic_cnt isn't supported\n");

		return -EINVAL;
	}

	bits_left = get_bits_left(bs);
	if (bits_left < 0) {
		v4l2_err(&ctx->dev->v4l2_dev, "Overread PPS by %d bits\n",
			 -bits_left);

		return -EINVAL;
	}

	return 0;
}

int avico_bitstream_read_slice_header(struct avico_ctx *ctx,
				      unsigned int nalu_type,
				      unsigned int nalu_priority)
{
	struct bitstream *const bs = &ctx->bs;
	unsigned int slice_type;
	int qp;
	int bits_left;

	if (readue(bs) > 0) { /* first_mb_in_slice */
		v4l2_err(&ctx->dev->v4l2_dev,
			 "first_mb_in_slice > 0 isn't supported\n");

		return -EINVAL;
	}

	slice_type = readue(bs); /* slice_type */
	if (slice_type > 9) {
		v4l2_err(&ctx->dev->v4l2_dev, "slice_type %d is too large\n",
			 slice_type);

		return -EINVAL;
	}
	if (slice_type > 4)
		slice_type -= 5;

	if (slice_type != VE_FR_I && slice_type != VE_FR_P) {
		v4l2_err(&ctx->dev->v4l2_dev, "slice_type %d isn't supported\n",
			 slice_type);

		return -EINVAL;
	}
	ctx->par.frame_type = slice_type;

	if (nalu_type == NALU_IDR && ctx->par.frame_type != VE_FR_I) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "slice type %d with NAL unit type %d isn't supported\n",
			 slice_type, nalu_type);

		return -EINVAL;
	}

	ctx->par.pps = readue(bs); /* pic_parameter_set_id */
	if (ctx->par.pps != 0) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "pic_parameter_set_id > 0 isn't supported\n");

		return -EINVAL;
	}

	ctx->par.frame = readu(bs, ctx->sps.log2_max_frame_num); /* frame_num */

	if (nalu_type == NALU_IDR) {
		ctx->par.idr = 1;
		ctx->par.idr_id = readue(bs); /* idr_pic_id */
	} else {
		ctx->par.idr = 0;
	}

	/* TODO: add support for poc_type 0 and 1 */
	if (ctx->sps.poc_type == 0)
		readu(bs, ctx->sps.log2_max_poc_lsb); /* pic_order_cnt_lsb */
	else if (ctx->sps.poc_type == 1 &&
		 ctx->sps.delta_pic_order_always_zero_flag == 0)
		/* delta_pic_order_cnt_bottom */
		ctx->par.delta_poc[0] = readse(bs);

	if (ctx->par.frame_type == VE_FR_P) {
		if (readu(bs, 1)) { /* num_ref_idx_active_override_flag */
			v4l2_err(&ctx->dev->v4l2_dev,
				 "num_ref_idx_active_override isn't supported\n");

			return -EINVAL;
		}
		if (readu(bs, 1)) { /* ref_pic_list_modification_flag_l0 */
			v4l2_err(&ctx->dev->v4l2_dev,
				 "ref_pic_list_modification isn't supported\n");

			return -EINVAL;
		}
	}

	if (nalu_priority == 0) {
		v4l2_err(&ctx->dev->v4l2_dev,
			 "nal_ref_idc %d isn't supported\n", nalu_priority);

		return -EINVAL;
	}
	if (nalu_type == NALU_IDR) {
		if (readu(bs, 1)) { /* no_output_of_prior_pics_flag */
			v4l2_err(&ctx->dev->v4l2_dev,
				 "no_output_of_prior_pics isn't supported\n");

			return -EINVAL;
		}
		if (readu(bs, 1)) { /* long_term_reference_flag */
			v4l2_err(&ctx->dev->v4l2_dev,
				 "long_term_reference isn't supported\n");

			return -EINVAL;
		}
	} else {
		if (readu(bs, 1)) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "adaptive_ref_pic_marking isn't supported\n");

			return -EINVAL;
		}
	}

	qp = ctx->pps.init_qp + readse(bs); /* slice_qp_delta */
	if (qp > 51) {
		v4l2_err(&ctx->dev->v4l2_dev, "QP %d is out of range\n", qp);

		return -EINVAL;
	}

	WARN_ON(ctx->pps.dbf_ctrl_present == 0);

	/* disable_deblocking_filter_idc */
	if (ctx->pps.dbf_ctrl_present) {
		if (readue(bs) != 1) {
			v4l2_err(&ctx->dev->v4l2_dev,
				 "deblocking must be disabled\n");

			return -EINVAL;
		}
	}

	if (ctx->par.frame_type == VE_FR_P)
		ctx->par.qp_p = qp;
	else
		ctx->par.qp_i = qp;

	bits_left = get_bits_left(bs);
	if (bits_left < 0) {
		v4l2_err(&ctx->dev->v4l2_dev, "Overread slice by %d bits\n",
			 -bits_left);

		return -EINVAL;
	}

	return 0;
}

/* Zero_byte followed by a start_code_prefix_one_3bytes */
static void write_delimiter(struct bitstream *bs)
{
	const uint8_t delimiter[4] = { 0, 0, 0, 1 };

	/* The code must start from byte boundary */
	WARN_ON(bs->bitsleft != 8);

	if (bs->p + 4 > bs->end) {
		bs->p = bs->end + 1;
		return;
	}

	memcpy(bs->p, delimiter, 4);
	bs->p += 4;

	/* \note Function call to convert_to_nalu(prior_to_sps_bytes + 5) in
	 * strom-ve causes one byte skip after the synchronization marker.
	 * I do not think this is necessary. Otherwise set bs->nulls to -1. */
	bs->nulls = 0;
}

/* With 8-bit buffer
 * 32-bit buffer is more complex for bit-stuffing */
static void writeu(struct bitstream *bs, uint8_t bits, uint32_t value)
{
	while (bits) {
		unsigned const writebits = min(bs->bitsleft, bits);
		unsigned const vmask = ((1 << writebits) - 1) << (bits -
								  writebits);

		/* Number of top writebits */
		unsigned const write = value >> (bits - writebits);

		if (bs->p > bs->end)
			return;

		bs->cb <<= writebits;
		bs->cb |= write;   /* Write to bs->cb */
		value &= ~vmask;   /* Clean written bits in value */
		bits -= writebits; /* Decrease number of bits to be written */
		bs->bitsleft -= writebits; /* Increase current bit offset */

		if (bs->bitsleft == 0) {
			bs->bitsleft = 8;
			/* insert emulation_prevention_three_byte */
			if (bs->nulls == 2 && !(bs->cb & 0xfc)) {
				*bs->p++ = 0x03;
				bs->nulls = 0;
			}
			if (bs->cb == 0)
				bs->nulls++;
			else
				bs->nulls = 0;
			*bs->p++ = bs->cb;
			bs->cb = 0;
		}
	}
}

static void writeue(struct bitstream *bs, uint32_t value)
{
	unsigned const bits = 32 - __builtin_clz(value + 1);

	WARN_ON(value == U32_MAX);

	writeu(bs, bits * 2 - 1, value + 1);
}

static void writese(struct bitstream *bs, int32_t value)
{
	uint32_t const uvalue = value <= 0 ? (-2 * value) : (2 * value - 1);

	writeue(bs, uvalue);
}

static void write_trailing_bits(struct bitstream *bs)
{
	writeu(bs, 1, 1); /* rbsp_stop_one_bit */
	if (bs->bitsleft != 8)
		writeu(bs, bs->bitsleft, 0); /* rbsp_alignment_zero bits */

	/* Trailing bits must be byte aligned */
	WARN_ON(bs->bitsleft != 8);
}

void avico_bitstream_init(struct avico_ctx *ctx, void *ptr, unsigned int size)
{
	struct bitstream *const bs = &ctx->bs;

	pr_devel("avico_bitstream_init: ptr = %p, size = %u\n", ptr, size);

	bs->start = (uint8_t *)ptr;
	bs->end = bs->start + size - 1;
	bs->p = bs->start;
	bs->cb = 0;
	bs->bitsleft = 8;
	bs->nulls = 0;
}

static int v4l2_to_h264_colorspace(struct avico_ctx *ctx,
				   struct h264_signal_type *st)
{
	/* TODO: Add support for VIDIOC_S_STD */
	st->f = H264_FMT_UNSPECIFIED;

	/* First step, set the defaults for each primaries */
	switch (ctx->colorspace) {
	case V4L2_COLORSPACE_SMPTE170M:
		st->r = 0;
		st->m = H264_MTX_SMPTE170M;
		st->t = H264_TRC_BT709;
		st->p = H264_PRI_SMPTE170M;
		break;
	case V4L2_COLORSPACE_REC709:
		st->r = 0;
		st->m = H264_MTX_BT709;
		st->t = H264_TRC_BT709;
		st->p = H264_PRI_BT709;
		break;
	case V4L2_COLORSPACE_SRGB:
		st->r = 1;
		st->m = H264_MTX_SMPTE170M;
		st->t = H264_TRC_IEC61966_2_1;
		st->p = H264_PRI_BT709;
		break;
	case V4L2_COLORSPACE_BT2020:
		st->r = 0;
		st->m = H264_MTX_BT2020_NCL;
		st->t = H264_TRC_BT2020_12;
		st->p = H264_PRI_BT2020;
		break;
	case V4L2_COLORSPACE_SMPTE240M:
		st->r = 0;
		st->m = H264_MTX_SMPTE240M;
		st->t = H264_TRC_SMPTE240M;
		st->p = H264_PRI_SMPTE240M;
		break;
	case V4L2_COLORSPACE_470_SYSTEM_M:
		st->r = 0;
		st->m = H264_MTX_SMPTE170M;
		st->t = H264_TRC_BT709;
		st->p = H264_PRI_BT470M;
		break;
	case V4L2_COLORSPACE_470_SYSTEM_BG:
		st->r = 0;
		st->m = H264_MTX_BT470BG;
		st->t = H264_TRC_BT709;
		st->p = H264_PRI_BT470BG;
		break;
	case V4L2_COLORSPACE_RAW:
		/* Explicitly unknown */
		st->r = 0;
		st->m = H264_MTX_UNSPECIFIED;
		st->t = H264_TRC_UNSPECIFIED;
		st->p = H264_PRI_UNSPECIFIED;
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Colorspace %d isn't supported\n", ctx->colorspace);

		return -EINVAL;
	}

	switch (ctx->range) {
	case V4L2_QUANTIZATION_FULL_RANGE:
		st->r = 1;
		break;
	case V4L2_QUANTIZATION_LIM_RANGE:
		st->r = 0;
		break;
	case V4L2_QUANTIZATION_DEFAULT:
		/* nothing, just use defaults for colorspace */
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Quantization %d isn't supported\n", ctx->range);
		st->r = 0;
		break;
	}

	switch (ctx->matrix) {
	case V4L2_YCBCR_ENC_XV601:
		st->m = H264_MTX_BT470BG;
		break;
	case V4L2_YCBCR_ENC_SYCC:
		st->m = H264_MTX_BT709;
		break;
	case V4L2_YCBCR_ENC_601:
		st->m = H264_MTX_SMPTE170M;
		break;
	case V4L2_YCBCR_ENC_XV709:
		st->m = H264_MTX_BT709;
		break;
	case V4L2_YCBCR_ENC_709:
		st->m = H264_MTX_BT709;
		break;
	case V4L2_YCBCR_ENC_BT2020_CONST_LUM:
		st->m = H264_MTX_BT2020_CL;
		break;
	case V4L2_YCBCR_ENC_BT2020:
		st->m = H264_MTX_BT2020_NCL;
		break;
	case V4L2_YCBCR_ENC_SMPTE240M:
		st->m = H264_MTX_SMPTE240M;
		break;
	case V4L2_YCBCR_ENC_DEFAULT:
		/* nothing, just use defaults for colorspace */
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Encoding %d isn't supported\n", ctx->matrix);
		st->m = H264_MTX_UNSPECIFIED;
		break;
	}

	switch (ctx->transfer) {
	case V4L2_XFER_FUNC_709:
		if (ctx->colorspace == V4L2_COLORSPACE_BT2020 &&
		    ctx->height >= 2160)
			st->t = H264_TRC_BT2020_12;
		else
			st->t = H264_TRC_BT709;
		break;
	case V4L2_XFER_FUNC_SRGB:
		st->t = H264_TRC_IEC61966_2_1;
		break;
	case V4L2_XFER_FUNC_SMPTE240M:
		st->t = H264_TRC_SMPTE240M;
		break;
	case V4L2_XFER_FUNC_NONE:
		st->t = H264_TRC_LINEAR;
		break;
	case V4L2_XFER_FUNC_DEFAULT:
		/* nothing, just use defaults for colorspace */
		break;
	default:
		v4l2_warn(&ctx->dev->v4l2_dev,
			  "Transfer function %d isn't supported\n",
			  ctx->transfer);
		st->t = H264_TRC_UNSPECIFIED;
		break;
	}

	return 0;
}

void avico_bitstream_write_sps(struct avico_ctx *ctx)
{
	struct bitstream *const bs = &ctx->bs;
	struct avico_frame_params *par = &ctx->par;
	struct h264_signal_type st;

	/* Write SPS */
	write_delimiter(bs);

	writeu(bs, 1, 0); /* forbidden_zero_bit */
	writeu(bs, 2, NALU_PRIOR_HIGHEST); /* nal_ref_idc */
	writeu(bs, 5, NALU_SPS); /* nal_unit_type */

	/* H.264 Profile. 66 for (Constrained) Baseline */
	writeu(bs, 8, 66); /* profile_idc */
	writeu(bs, 1, 0);  /* constraint_set0_flag */

	/* Constrained profile */
	writeu(bs, 1, 1); /* constraint_set1_flag */
	writeu(bs, 1, 0); /* constraint_set2_flag */
	writeu(bs, 1, 0); /* constraint_set3_flag */
	writeu(bs, 1, 0); /* constraint_set4_flag */
	writeu(bs, 1, 0); /* constraint_set5_flag */
	writeu(bs, 2, 0); /* reserved_zero_2bits */

	/* H.264 Level 4.0 */
	writeu(bs, 8, 40); /* level_idc */

	/* \todo Missing unessential elements */
	writeue(bs, par->sps); /* seq_parameter_set_id */
	writeue(bs, par->log2_max_frame - 4); /* log2_max_frame_num_minus4 */
	writeue(bs, par->poc_type); /* pic_order_cnt_type */
	/* \todo Missing unessential elements */
	writeue(bs, 1);   /* num_ref_frames */
	writeu(bs, 1, 0); /* gaps_in_frame_num_value_allowed_flag */
	writeue(bs, ctx->mbx - 1); /* pic_width_in_mbs_minus1 */
	writeue(bs, ctx->mby - 1); /* pic_height_in_map_units_minus1 */
	writeu(bs, 1, 1); /* frame_mbs_only_flag */
	/* \todo Missing unessential elements */
	writeu(bs, 1, 0); /* direct_8x8_inference_flag */

	if (par->crop.left || par->crop.right || par->crop.top ||
	    par->crop.bottom) {
		writeu(bs, 1, 1); /* frame_cropping_flag */
		writeue(bs, par->crop.left);   /* frame_crop_left_offset */
		writeue(bs, par->crop.right);  /* frame_crop_right_offset */
		writeue(bs, par->crop.top);    /* frame_crop_top_offset */
		writeue(bs, par->crop.bottom); /* frame_crop_bottom_offset */
	} else {
		writeu(bs, 1, 0); /* frame_cropping_flag */
	}

	/* \todo Missing unessential elements */
	writeu(bs, 1, 1); /* vui_parameters_present_flag */
	writeu(bs, 1, 0); /* vui_aspect_ratio_present_flag */
	writeu(bs, 1, 0); /* vui_overscan_info_present_flag */
	if (!v4l2_to_h264_colorspace(ctx, &st)) {
		writeu(bs, 1, 1); /* vui_video_signal_type_present_flag */
		writeu(bs, 3, st.f); /* vui_video_format */
		writeu(bs, 1, st.r); /* vui_video_full_range_flag */
		writeu(bs, 1, 1); /* vui_colour_description_present_flag */
		writeu(bs, 8, st.p); /* vui_colour_primaries */
		writeu(bs, 8, st.t); /* vui_transfer_characteristics */
		writeu(bs, 8, st.m); /* vui_matrix_coefficients */
	} else {
		writeu(bs, 1, 0); /* vui_video_signal_type_present_flag */
	}

	writeu(bs, 1, 0); /* vui_chroma_loc_info_present_flag */

	writeu(bs, 1, 1); /* vui_timing_info_present_flag */
	writeu(bs, 32, ctx->timeperframe.numerator); /* vui_num_units_in_tick */
	writeu(bs, 32, ctx->timeperframe.denominator * 2); /* vui_time_scale */
	writeu(bs, 1, 1); /* vui_fixed_frame_rate_flag */

	writeu(bs, 1, 0); /* vui_nal_hrd_parameters_present_flag */
	writeu(bs, 1, 0); /* vui_vcl_hrd_parameters_present_flag */
	writeu(bs, 1, 0); /* vui_pic_struct_present_flag */
	writeu(bs, 1, 0); /* vui_bitstream_restriction_flag */

	write_trailing_bits(bs);
}

void avico_bitstream_write_pps(struct avico_ctx *ctx)
{
	struct bitstream *const bs = &ctx->bs;
	struct avico_frame_params *par = &ctx->par;

	/* Write PPS */
	write_delimiter(bs);
	writeu(bs, 1, 0); /* forbidden_zero_bit */
	writeu(bs, 2, NALU_PRIOR_HIGHEST);
	writeu(bs, 5, NALU_PPS);

	writeue(bs, par->pps); /* pic_parameter_set_id */
	writeue(bs, par->sps); /* seq_parameter_set_id */
	writeu(bs, 1, 0); /* entropy_coding_mode_flag. 0 for CAVLC */
	writeu(bs, 1, 0); /* bottom_field_pic_order_in_frame_present_flag */
	writeue(bs, 0);   /* num_slice_groups - 1 */

	writeue(bs, 0);   /* num_ref_idx_l0_default_active - 1 */
	writeue(bs, 0);   /* num_ref_idx_l1_default_active - 1 */
	writeu(bs, 1, 0); /* weighted_pred_flag */
	writeu(bs, 2, 0); /* weighted_bipred_idc */

	par->pps_qp = par->qp_i;
	writese(bs, par->pps_qp - 26); /* pic_init_qp - 26 */
	writese(bs, par->pps_qp - 26); /* pic_init_qs - 26 */
	writese(bs, par->qpc_offset);  /* chroma_qp_index_offset */

	/* \todo DBF should depend on off_a and off_b (see Rolschikov's code */
	writeu(bs, 1, !par->dbf);  /* dbf_control_present_flag */
	writeu(bs, 1, 0); /* constrained_intra_pred_flag */
	writeu(bs, 1, 0); /* redundant_pic_cnt_present_flag */

	write_trailing_bits(bs);
}

void avico_bitstream_write_slice_header(struct avico_ctx *ctx)
{
	struct bitstream *bs = &ctx->bs;
	struct avico_frame_params *par = &ctx->par;

	write_delimiter(bs);
	writeu(bs, 1, 0); /* forbidden_zero_bit */
	writeu(bs, 2, par->idr ? NALU_PRIOR_HIGHEST : NALU_PRIOR_HIGH);
	writeu(bs, 5, par->idr ? NALU_IDR : NALU_SLICE);

	writeue(bs, 0); /* first_mb_addr_in_slice */
	writeue(bs, par->frame_type); /* frame_type */
	writeue(bs, par->pps);        /* pps_id */
	writeu(bs, par->log2_max_frame, par->frame); /* frame_num */

	if (par->idr)
		writeue(bs, par->idr_id); /* idr_pic_id */

	/* POC type is always 2 */
	/* \todo Support different POC types */

	if (par->frame_type == VE_FR_P) {
		writeu(bs, 1, 0); /* num_ref_idx_active_override_flag */
		writeu(bs, 1, 0); /* ref_pic_list_l0_reordering_flag */
	}

	if (par->idr) {
		writeu(bs, 1, 0); /* no_output_of_prior_pics_flag */
		writeu(bs, 1, 0); /* long_term_reference_flag */
	} else {
		writeu(bs, 1, 0); /* adaptive_ref_pic_buffering_flag */
	}

	if (par->frame_type == VE_FR_I)
		writese(bs, par->qp_i - par->pps_qp); /* slice_qp_delta */
	else
		writese(bs, par->qp_p - par->pps_qp); /* slice_qp_delta */

	/* \todo DBF should depend on off_a and off_b (see Rolschikov's code */
	if (!par->dbf)
		writeue(bs, par->dbf ? 0 : 1); /* disable_dbf_flag */
}

void avico_bitstream_get64(struct avico_ctx *ctx, uint32_t data[2],
			   unsigned int bits[2])
{
	struct bitstream *bs = &ctx->bs;
	uint8_t *p = (uint8_t *)((uintptr_t)bs->p & ~0x7);
	int b = (bs->p - p) * 8 + 8 - bs->bitsleft;

	*bs->p = bs->cb;

	data[0] = data[1] = 0;

	if (b)
		memcpy(data, p, b / 8 + 1);
	bits[0] = min(b, 32);
	bits[1] = max(b - 32, 0);
}

int avico_bitstream_ecd_stuff_pos(struct avico_ctx *ctx)
{
	/* \todo Here I assume that start is at least 8-byte aligned */
	uint8_t *p = ctx->bs.p;
	int pos = (uintptr_t)p % 4;

	/* As our bitstream always start with synchronization marker we can not
	 * cross bitstream buffer boundary. */
	while (pos != -2 && *(--p) == 0)
		pos--;

	return pos;
}

void avico_bitstream_cut64(struct avico_ctx *ctx)
{
	ctx->bs.p = IS_ALIGNED((uintptr_t)ctx->bs.p, 8) ? ctx->bs.p :
			PTR_ALIGN(ctx->bs.p - 8, 8);
	ctx->bs.bitsleft = 8;
	ctx->bs.cb = 0;
	ctx->bs.nulls = 0;
}

void avico_bitstream_dump(struct bitstream *bs)
{
	size_t len = bs->p - bs->start + (bs->bitsleft == 8 ? 0 : 1);
	*bs->p = bs->cb;
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, bs->start,
		       len, true);
	if (bs->bitsleft != 8)
		pr_info("Bits in last byte: %u\n", 8 - bs->bitsleft);
}

