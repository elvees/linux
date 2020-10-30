/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#ifndef VINC_NEON_H
#define VINC_NEON_H

#include <linux/vinc.h>
#include <media/v4l2-ctrls.h>

#include "vinc-dev.h"

struct matrix {
	double coeff[VINC_CC_COEFF_COUNT];
};

struct vector {
	double offset[VINC_CC_OFFSET_COUNT];
};

struct col_fx {
	struct matrix m_fx_ycbcr;
	struct matrix m_fx_rgb;
	struct vector v_fx_ycbcr;
	struct vector v_fx_rgb;
};

struct bc_stat {
	u32 hist_brightness[256];
	u32 cumulate[256];
};

void vinc_neon_calculate_v_bl(void *vector, s32 val);

void vinc_neon_calculate_v_bri(void *vector, s32 val);

void vinc_neon_calculate_m_con(void *matrix, s32 val);

void vinc_neon_calculate_m_sat(void *matrix, s32 val);

void vinc_neon_calculate_m_hue(void *matrix, s32 val);

void vinc_neon_calculate_fx(void *col_fx, s32 cbcr, s32 val);

void vinc_neon_wb_temp(u32 temp, struct vinc_wb_cc *wbcc, s32 *rb, s32 *bb);

void vinc_neon_wb_stat(u32 red, u32 green, u32 blue, s32 *rb, s32 *bb);

void vinc_neon_bc_stat(struct bc_stat *p_stat, s32 *bri, s32 *con);

void vinc_neon_calculate_m_wb(s32 rb, s32 bb, struct vinc_wb_cc *wbcc,
			      void *matrix);

void vinc_neon_calculate_m_ck(void *matrix, s32 val);

void vinc_neon_calculate_he(struct bc_stat *p_stat, u32 n, u16 h, u16 w,
			    u32 *conv);

void vinc_neon_calculate_gamma_curve(int value, u32 *conv, u8 bklight,
		struct vinc_gamma_curve *gamma_ptr);

/* Calculate CC matrix and offset */
int vinc_neon_calculate_cc(struct ctrl_priv *ctrl_privs,
			   enum vinc_ycbcr_encoding ycbcr_enc,
			   struct vinc_cc *cc);

/* Calculate gain and exposure values */
u32 vinc_neon_calculate_luma_avg(enum vinc_input_format input_format,
				 struct vinc_stat_add *add,
				 enum vinc_ycbcr_encoding ycbcr_enc,
				 struct vinc_stat_zone *zone,
				 bool pport_low_bits);

void vinc_neon_calculate_gain_exp(u32 luma, u32 cur_gain, u32 cur_exp,
				  u32 max_gain, u32 max_exp, u32 *gain,
				  u32 *exp);

#endif /* VINC_NEON_H */
