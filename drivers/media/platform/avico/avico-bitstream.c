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

/* Zero_byte followed by a start_code_prefix_one_3bytes */
static void write_delimiter(struct bitstream *bs)
{
	const uint8_t delimiter[4] = { 0, 0, 0, 1 };

	/* The code must start from byte boundary */
	WARN_ON(bs->freebits != 8);

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
		unsigned const writebits = min(bs->freebits, bits);
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
		bs->freebits -= writebits; /* Increase current bit offset */

		if (bs->freebits == 0) {
			bs->freebits = 8;
			/* \dontknow */
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
	if (bs->freebits != 8)
		writeu(bs, bs->freebits, 0); /* rbsp_alignment_zero bits */

	/* Trailing bits must be byte aligned */
	WARN_ON(bs->freebits != 8);
}

void avico_bitstream_init(struct avico_ctx *ctx, void *ptr, unsigned int size)
{
	struct bitstream *const bs = &ctx->bs;

	pr_devel("avico_bitstream_init: ptr = %p, size = %u\n", ptr, size);

	bs->start = (uint8_t *)ptr;
	bs->end = bs->start + size - 1;
	bs->p = bs->start;
	bs->cb = 0;
	bs->freebits = 8;
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
	int b = (bs->p - p) * 8 + 8 - bs->freebits;

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
	ctx->bs.freebits = 8;
	ctx->bs.cb = 0;
	ctx->bs.nulls = 0;
}

void avico_bitstream_dump(struct bitstream *bs)
{
	size_t len = bs->p - bs->start + (bs->freebits == 8 ? 0 : 1);
	*bs->p = bs->cb;
	print_hex_dump(KERN_INFO, "", DUMP_PREFIX_OFFSET, 16, 1, bs->start,
		       len, true);
	if (bs->freebits != 8)
		pr_info("Bits in last byte: %u\n", 8 - bs->freebits);
}

