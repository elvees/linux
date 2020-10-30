/*
 * Copyright 2016 ELVEES NeoTek JSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/log2.h>
#include "vinc-neon.h"
#include "math/math.h"

#define CC_OFFSET_MAX 4096.0

#define is_in_range(val, min, max) ((val) >= (min) && (val) <= (max))

/* Matrices for RGB->YCbCr conversion depends on YCbCr ENCODING.
 * Output QUANTIZATION is always in FULL RANGE.
 * Table of coefficients:

   coefficient || R  G  B
   =======================
     for Y     || 0  1  2
     for Cb    || 3  4  5
     for Cr    || 6  7  8 */
const struct matrix m_ycbcr[4] = {
	[VINC_YCBCR_ENC_601] = {
		.coeff[0] =  0.299,
		.coeff[1] =  0.587,
		.coeff[2] =  0.114,
		.coeff[3] = -0.168735891647856,
		.coeff[4] = -0.331264108352144,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.418687589158345,
		.coeff[8] = -0.0813124108416548,
	},
	[VINC_YCBCR_ENC_709] = {
		.coeff[0] =  0.2126,
		.coeff[1] =  0.7152,
		.coeff[2] =  0.0722,
		.coeff[3] = -0.11457210605734,
		.coeff[4] = -0.38542789394266,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.454152908305817,
		.coeff[8] = -0.0458470916941834,
	},
	[VINC_YCBCR_ENC_BT2020] = {
		.coeff[0] =  0.2627,
		.coeff[1] =  0.678,
		.coeff[2] =  0.0593,
		.coeff[3] = -0.139630062719252,
		.coeff[4] = -0.38542789394266,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.459785704597857,
		.coeff[8] = -0.040214295402143,
	},
	[VINC_YCBCR_ENC_SYCC] = {
		.coeff[0] =  0.299,
		.coeff[1] =  0.587,
		.coeff[2] =  0.114,
		.coeff[3] = -0.1687,
		.coeff[4] = -0.3313,
		.coeff[5] =  0.5,
		.coeff[6] =  0.5,
		.coeff[7] = -0.4187,
		.coeff[8] = -0.0813,
	}
};

/* Offsets for RGB->YCbCr conversion.
 * Output QUANTIZATION is always in FULL RANGE.
 * Vector elements go in this order:
 * 0: Y offset
 * 1: Cb offset
 * 2: Cr offset */
const struct vector v_ycbcr = {
	.offset[0] = 0,
	.offset[1] = 2048,
	.offset[2] = 2048,
};

/* Matrices for YCbCr->RGB conversion.
 * Depend on YCbCr ENCODING and input QUANTIZATION.
 * Table of coefficients:

   coefficient || Y Cb Cr
   =======================
     for R     || 0  1  2
     for G     || 3  4  5
     for B     || 6  7  8 */
const struct matrix m_rgb[4][2] = {
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] =  5.59894392753486e-17,
		.coeff[2] =  1.59602678571429,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.391762290094914,
		.coeff[5] = -0.81296764723777,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.01723214285714,
		.coeff[8] =  1.11022302462516e-16,

	},
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] =  4.91828799908944e-17,
		.coeff[2] =  1.402,
		.coeff[3] =  1,
		.coeff[4] = -0.344136286201022,
		.coeff[5] = -0.714136286201022,
		.coeff[6] =  1,
		.coeff[7] =  1.772,
		.coeff[8] =  0,
	},

	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] =  0,
		.coeff[2] =  1.79274107142857,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.21324861427373,
		.coeff[5] = -0.532909328559444,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.11240178571429,
		.coeff[8] = -5.55111512312578e-17,
	},
	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] =  0,
		.coeff[2] =  1.5748,
		.coeff[3] =  1,
		.coeff[4] = -0.187324272930649,
		.coeff[5] = -0.468124272930649,
		.coeff[6] =  1,
		.coeff[7] =  1.8556,
		.coeff[8] =  5.55111512312578e-17,
	},

	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -5.94461236188718e-17,
		.coeff[2] =  1.67867410714286,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.187326104219343,
		.coeff[5] = -0.650424318505057,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.14177232142857,
		.coeff[8] =  0,
	},
	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] = -2.61096699816221e-17,
		.coeff[2] =  1.4746,
		.coeff[3] =  1,
		.coeff[4] = -0.164553126843658,
		.coeff[5] = -0.571353126843658,
		.coeff[6] =  1,
		.coeff[7] =  1.8814,
		.coeff[8] =  0,
	},

	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_LIM_RANGE] = {
		.coeff[0] =  1.16438356164384,
		.coeff[1] = -4.19156139872783e-05,
		.coeff[2] =  1.59601264338613,
		.coeff[3] =  1.16438356164384,
		.coeff[4] = -0.391736101495071,
		.coeff[5] = -0.812930689215866,
		.coeff[6] =  1.16438356164384,
		.coeff[7] =  2.01720723110692,
		.coeff[8] = -0.000153208795953375,
	},
	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_FULL_RANGE] = {
		.coeff[0] =  1,
		.coeff[1] = -3.68199903261289e-05,
		.coeff[2] =  1.40198757693526,
		.coeff[3] =  1,
		.coeff[4] = -0.344113281313317,
		.coeff[5] = -0.714103821115113,
		.coeff[6] =  1,
		.coeff[7] =  1.77197811673706,
		.coeff[8] = -0.000134583412916056,
	}
};

/* Offsets for YCbCr->RGB conversion.
 * Depend on YCbCr ENCODING and input QUANTIZATION
 * Vector elements go in this order:
 * 0: R offset
 * 1: G offset
 * 2: B offset */
const struct vector v_rgb[4][2] = {
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3566.74504892368,
		.offset[1] =  2169.20471987651,
		.offset[2] = -4429.37362035225,
	},
	[VINC_YCBCR_ENC_601][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -2871.296,
		.offset[1] =  2167.34222827939,
		.offset[2] = -3629.056,
	},

	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3969.61590606654,
		.offset[1] =  1230.04927514152,
		.offset[2] = -4624.28104892368,
	},
	[VINC_YCBCR_ENC_709][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -3225.1904,
		.offset[1] =  1342.35862192394,
		.offset[2] = -3800.2688,
	},

	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3736.00676320939,
		.offset[1] =  1417.63067395875,
		.offset[2] = -4684.43190606654,
	},
	[VINC_YCBCR_ENC_BT2020][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -3019.9808,
		.offset[1] =  1507.13600755162,
		.offset[2] = -3853.1072,
	},

	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_LIM_RANGE] = {
		.offset[0] = -3566.63024225816,
		.offset[1] =  2169.07539559518,
		.offset[2] = -4429.00882947368,
	},
	[VINC_YCBCR_ENC_SYCC][VINC_QUANTIZATION_FULL_RANGE] = {
		.offset[0] = -2871.19515022323,
		.offset[1] =  2167.22862577343,
		.offset[2] = -3628.73555624785,
	}
};

/**
 * float_to_u16() - converts floating point value to fixed-point representation
 * @value: floating point value to convert
 * @fbits: number of fractional bits
 *
 * Return: fixed-point representation of a floating point value
 */
static u16 float_to_u16(double value, u8 fbits)
{
	/* (u16)(s16) is a portable way to convert negative floating point
	 * number to unsigned integer. For more info see C99 spec 6.3.1.4.
	 *
	 * I'm using scalbn() instead of ldexp(). In our math library it is
	 * indicated that they are the same. In general they are different only
	 * when FLT_RADIX != 2 (which is false for any modern architecture). */
	return (u16)(s16)rint(scalbn(value, fbits));
}

void vinc_neon_calculate_he(struct bc_stat *p_stat, u32 n, u16 h, u16 w,
			    u32 *conv)
{
	int i;
	const double nmax = 10.;
	double conv_he, n_dbl = (double)n / nmax;

	for (i = 0; i < 256; i++) {
		conv_he = (p_stat->cumulate[i] * 255) / (h * w * 3);
		conv[i] = rint((1 - n_dbl) * i + n_dbl * conv_he);
	}
}

void vinc_neon_calculate_gamma_curve(int value, u32 *conv, u8 bklight,
				     struct vinc_gamma_curve *gamma)
{
	double gamma_dbl, cur_dbl;
	u32 lut_inp;
	int i, j, cur;

	gamma_dbl = (value / 32.0) / (1 - value / 32.0);
	for (i = 0; i < 256; i++) {
		for (j = 0; j < 16; j++) {
			lut_inp = (bklight) ? conv[i] * 16 : i*16 + j;
			cur_dbl = 4095 * (pow(lut_inp / 4095.0, gamma_dbl));
			cur = (int) rint(cur_dbl);
			gamma->red[i*16 + j]   = cur;
			gamma->green[i*16 + j] = cur;
			gamma->blue[i*16 + j]  = cur;
		}
	}
}

/* Matrices and vectors for CC controls */
void vinc_neon_calculate_v_bl(void *vector, s32 val)
{
	struct vector *bl = (struct vector *)vector;

	*bl = (struct vector) {
		.offset[0] = (double)val,
		.offset[1] = (double)val,
		.offset[2] = (double)val,
	};
}

void vinc_neon_calculate_v_bri(void *vector, s32 val)
{
	struct vector *bri = (struct vector *)vector;

	*bri = (struct vector) {
		.offset[0] = (double)val
	};
}

void vinc_neon_calculate_m_con(void *matrix, s32 val)
{
	struct matrix *con = (struct matrix *)matrix;

	*con = (struct matrix) {
		.coeff[0] = tan((double)val * PI / 512),
		.coeff[4] = 1,
		.coeff[8] = 1
	};
}

void vinc_neon_calculate_m_sat(void *matrix, s32 val)
{
	struct matrix *sat = (struct matrix *)matrix;

	*sat = (struct matrix) {
		.coeff[0] = 1,
		.coeff[4] = tan((double)val * PI / 512),
		.coeff[8] = tan((double)val * PI / 512)
	};
}

void vinc_neon_calculate_m_hue(void *matrix, s32 val)
{
	struct matrix *hue = (struct matrix *)matrix;

	*hue = (struct matrix) {
		.coeff[0] =  1,
		.coeff[4] =  cos((double)val * PI / 128),
		.coeff[5] = -sin((double)val * PI / 128),
		.coeff[7] =  sin((double)val * PI / 128),
		.coeff[8] =  cos((double)val * PI / 128)
	};
}

static void interpolate_wbcc(u32 temp, double rb_gains_ratio,
			     struct vinc_wb_cc *wbcc, double *gain_r,
			     double *gain_b, struct matrix *coeff)
{
	int i, i0, i1, j;
	double v_in, v, v0, v1, alpha;
	const double fp_factor = scalbn(1, -12);

	/* Interpolate for temperature or ratio of gains */
	v_in = temp ? temp : rb_gains_ratio;

	i0 = i1 = -1;
	v0 = 0; v1 = 1e6;
	for (i = 0; i < VINC_WB_CC_COUNT; i++) {
		/* Ignore records with zero temperature */
		if (!wbcc->temp[i])
			continue;

		v = temp ? wbcc->temp[i] : (double)wbcc->gain_r[i] / wbcc->gain_b[i];
		if (v_in < v) {
			if (v < v1) {
				i1 = i; v1 = v;
			}
		} else {
			if (v > v0) {
				i0 = i; v0 = v;
			}
		}
	}

	/* Use identity conversion if wbcc is not configured */
	if (i0 < 0 && i1 < 0) {
		if (gain_r != NULL)
			*gain_r = 1;
		if (gain_b != NULL)
			*gain_b = 1;
		if (coeff != NULL)
			*coeff = (struct matrix) {
				.coeff[0] =  1,
				.coeff[4] =  1,
				.coeff[8] =  1
			};
		return;
	}

	/* Define interpolation coefficient */
	alpha = 0;
	if (i0 < 0)
		i0 = i1;
	else if (i1 < 0)
		i1 = i0;
	else
		alpha = (v1 - v_in) / (v1 - v0);

	/* Interpolate gains */
	if (gain_r != NULL)
		*gain_r = (alpha * wbcc->gain_r[i0] +
			(1 - alpha) * wbcc->gain_r[i1]) * fp_factor;
	if (gain_b != NULL)
		*gain_b = (alpha * wbcc->gain_b[i0] +
			(1 - alpha) * wbcc->gain_b[i1]) * fp_factor;

	/* Interpolate matrix */
	if (coeff != NULL)
		for (j = 0; j < VINC_CC_COEFF_COUNT; j++)
			coeff->coeff[j] = fp_factor *
				(alpha * (s16)wbcc->coeff[i0][j] +
				(1 - alpha) * (s16)wbcc->coeff[i1][j]);
}

static s32 gain_to_balance(double gain)
{
	return rint((gain / (gain + 1)) * 256 - 128);
}

static double balance_to_gain(s32 balance)
{
	double v = (balance + 128) / 256.0;
	return v / (1 - v);
}

void vinc_neon_wb_temp(u32 temp, struct vinc_wb_cc *wbcc, s32 *rb, s32 *bb)
{
	double gain_r, gain_b;

	interpolate_wbcc(temp, 0, wbcc, &gain_r, &gain_b, NULL);

	*rb = gain_to_balance(gain_r);
	*bb = gain_to_balance(gain_b);
}

void vinc_neon_wb_stat(u32 red, u32 green, u32 blue, s32 *rb, s32 *bb)
{
	double gain_r = (double)green / red;
	double gain_b = (double)green / blue;

	*rb = gain_to_balance(gain_r);
	*bb = gain_to_balance(gain_b);
}

void vinc_neon_bc_stat(struct bc_stat *p_stat, s32 *bri, s32 *con)
{
	u32 max, min, clip_hist_percent = 0;
	double alpha;


	clip_hist_percent = p_stat->cumulate[255] / 100;
	min = 0;
	while (p_stat->cumulate[min] < clip_hist_percent)
		min++;

	max = 255;
	while (p_stat->cumulate[max] >= (p_stat->cumulate[255] -
		clip_hist_percent))
		max--;

	alpha = (double)255 / (max - min);
	*con = rint(atan(alpha) * 512 / PI);
	*bri = rint(-alpha * min);
}

void vinc_neon_calculate_m_wb(s32 rb, s32 bb, struct vinc_wb_cc *wbcc,
			      void *matrix)
{
	int i;
	struct matrix *wb = (struct matrix *)matrix;
	double gain_r = balance_to_gain(rb);
	double gain_b = balance_to_gain(bb);

	interpolate_wbcc(0, gain_r / gain_b, wbcc, NULL, NULL, wb);

	for (i = 0; i < VINC_CC_COEFF_COUNT; i += VINC_CC_OFFSET_COUNT) {
		wb->coeff[i] *= gain_r;
		wb->coeff[i + 2] *= gain_b;
	}
}

void vinc_neon_calculate_m_ck(void *matrix, s32 val)
{
	struct matrix *ck = (struct matrix *)matrix;

	*ck = (struct matrix) {
		.coeff[0] =  1,
		.coeff[4] =  1 - (double)val,
		.coeff[8] =  1 - (double)val
	};
}

void vinc_neon_calculate_fx(void *col_fx, s32 cbcr, s32 val)
{
	struct col_fx *fx = (struct col_fx *)col_fx;

	switch (val) {
	case V4L2_COLORFX_BW:
		fx->m_fx_rgb   = (struct matrix) {
			.coeff[0] =  1,
			.coeff[4] =  1,
			.coeff[8] =  1
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {
			.offset[1] = 2048,
			.offset[2] = 2048
		};
		break;
	case V4L2_COLORFX_SEPIA:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = 0.393,
			.coeff[1] = 0.769,
			.coeff[2] = 0.189,
			.coeff[3] = 0.349,
			.coeff[4] = 0.686,
			.coeff[5] = 0.168,
			.coeff[6] = 0.272,
			.coeff[7] = 0.534,
			.coeff[8] = 0.131
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] =  1,
			.coeff[4] =  1,
			.coeff[8] =  1
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {0};
		break;
	case V4L2_COLORFX_NEGATIVE:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = -1,
			.coeff[4] = -1,
			.coeff[8] = -1
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->v_fx_rgb = (struct vector) {
			.offset[0] = 4095,
			.offset[1] = 4095,
			.offset[2] = 4095
		};
		fx->v_fx_ycbcr = (struct vector) {0};
		break;
	case V4L2_COLORFX_VIVID:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1.3143,
			.coeff[8] = 1.3143
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {
			.offset[1] = -643.6864,
			.offset[2] = -643.6864
		};
		break;
	case V4L2_COLORFX_AQUA:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = 0.9,
			.coeff[1] = 0.1,
			.coeff[2] = 0.1,
			.coeff[3] = 0.1,
			.coeff[4] = 0.85,
			.coeff[5] = 0.1,
			.coeff[6] = 0.3,
			.coeff[7] = 0.1,
			.coeff[8] = 0.7
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {0};
		break;
	case V4L2_COLORFX_ANTIQUE:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = 0.85,
			.coeff[1] = 0.1,
			.coeff[2] = 0.3,
			.coeff[3] = 0.1,
			.coeff[4] = 0.45,
			.coeff[5] = 0.5,
			.coeff[6] = 0.1,
			.coeff[7] = 0.3,
			.coeff[8] = 0.5
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {0};
		break;
	case V4L2_COLORFX_SET_CBCR:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {
			/* implementation for 8 bit image */
			.offset[1] = (cbcr & 0x0000FF00) >> 4,
			.offset[2] = (cbcr & 0x000000FF) << 4
		};
		break;
	default:
		fx->m_fx_rgb = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->m_fx_ycbcr = (struct matrix) {
			.coeff[0] = 1,
			.coeff[4] = 1,
			.coeff[8] = 1
		};
		fx->v_fx_rgb   = (struct vector) {0};
		fx->v_fx_ycbcr = (struct vector) {0};
		break;
	}
}

/*  Matrix and matrix multiplication
 *  m1 - left matrix, m2 - right matrix. Order of multiplication matters. */
static void mxm_mult(struct matrix *prod, const struct matrix *m1,
		     const struct matrix *m2)
{
	u8 i, j, k;
	double sum;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++) {
		for (j = 0; j < VINC_CC_OFFSET_COUNT; j++) {
			sum = 0;
			for (k = 0; k < VINC_CC_OFFSET_COUNT; k++)
				sum += m1->coeff[k + VINC_CC_OFFSET_COUNT * i]
				     * m2->coeff[j + VINC_CC_OFFSET_COUNT * k];
			prod->coeff[j + VINC_CC_OFFSET_COUNT * i] = sum;
		}
	}
}

/*  Matrix and vector multiplication
 *  m - left matrix, v - right vector. */
static void mxv_mult(struct vector *prod, const struct matrix *m,
		     const struct vector *v)
{
	u8 i, j;
	double sum;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++) {
		sum = 0;
		for (j = 0; j < VINC_CC_OFFSET_COUNT; j++)
			sum += m->coeff[j + VINC_CC_OFFSET_COUNT * i] *
			       v->offset[j];
		prod->offset[i] = sum;
	}
}

/*  Vector addition. Vectors must have the same size */
static void vxv_add(struct vector *sum, const struct vector *v1,
		    const struct vector *v2)
{
	u8 i;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++)
		sum->offset[i] = v1->offset[i] + v2->offset[i];
}

/*  Vector subtraction. Vectors must have the same size */
static void vxv_sub(struct vector *res, const struct vector *v1,
		    const struct vector *v2)
{
	u8 i;

	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++)
		res->offset[i] = v1->offset[i] - v2->offset[i];
}

/*  Calculate CC coefficient matrix. Uses controls matrices and vectors */
static void cc_matrix_calc(struct matrix *coeff, struct ctrl_priv *ctrl_privs,
		enum vinc_ycbcr_encoding ycbcr_enc)
{
	struct matrix tmp1;
	struct matrix tmp2;

	struct col_fx *fx = (struct col_fx *)ctrl_privs->fx;

	struct matrix *m_wb       = (struct matrix *)ctrl_privs->dowb;
	struct matrix *m_con      = (struct matrix *)ctrl_privs->contrast;
	struct matrix *m_sat      = (struct matrix *)ctrl_privs->saturation;
	struct matrix *m_hue      = (struct matrix *)ctrl_privs->hue;
	struct matrix *m_ck       = (struct matrix *)ctrl_privs->ck;
	struct matrix *m_fx_rgb   = &(fx->m_fx_rgb);
	struct matrix *m_fx_ycbcr = &(fx->m_fx_ycbcr);

	/* M_cc = M_fx_rgb*M_rgb*M_fx_ycbcr*M_ck*M_sat*M_con*M_hue*M_ycbcr*M_wb
	 *                8     7          6    5     4     3     2       1  */
	mxm_mult(&tmp1, &m_ycbcr[ycbcr_enc], m_wb);   /* [1] */
	mxm_mult(&tmp2, m_hue, &tmp1);		      /* [2] */
	mxm_mult(&tmp1, m_con, &tmp2);		      /* [3] */
	mxm_mult(&tmp2, m_sat, &tmp1);		      /* [4] */
	mxm_mult(&tmp1, m_ck, &tmp2);		      /* [5] */
	mxm_mult(&tmp2, m_fx_ycbcr, &tmp1);	      /* [6] */
	mxm_mult(&tmp1, &m_rgb[ycbcr_enc][VINC_QUANTIZATION_FULL_RANGE],
		 &tmp2);			      /* [7] */
	mxm_mult(coeff, m_fx_rgb, &tmp1);	      /* [8] */
}

/* Calculate CC offset vector according to control matrices and vectors */
static void cc_vector_calc(struct vector *offset, struct ctrl_priv *ctrl_privs,
		enum vinc_ycbcr_encoding ycbcr_enc)
{
	struct vector tmp1;
	struct vector tmp2;
	struct vector half = {
		.offset[0] = 2048,
		.offset[1] = 2048,
		.offset[2] = 2048
	};
	struct col_fx *fx = (struct col_fx *)ctrl_privs->fx;

	struct vector *v_bri      = (struct vector *)ctrl_privs->brightness;
	struct vector *v_bl       = (struct vector *)ctrl_privs->blacklevel;
	struct matrix *m_wb       = (struct matrix *)ctrl_privs->dowb;
	struct matrix *m_con      = (struct matrix *)ctrl_privs->contrast;
	struct matrix *m_sat      = (struct matrix *)ctrl_privs->saturation;
	struct matrix *m_hue      = (struct matrix *)ctrl_privs->hue;
	struct matrix *m_ck       = (struct matrix *)ctrl_privs->ck;
	struct matrix *m_fx_rgb   = &(fx->m_fx_rgb);
	struct matrix *m_fx_ycbcr = &(fx->m_fx_ycbcr);
	struct vector *v_fx_rgb   = &(fx->v_fx_rgb);
	struct vector *v_fx_ycbcr = &(fx->v_fx_ycbcr);

	/* Vcc = Mfx_rgb * (Mrgb * (Mfx_ycbcr * (Mck * Msat * Mcon * Mhue *
	 *               15      13           11     8      7      6      5
	 *
	 * * (Vycbcr - Mycbcr * Mwb * Vbl - Vhalf) + Vbri + Vhalf) +
	 * 5         3        2     1     4        9      10       12
	 *
	 * + Vfx_ycbcr) + Vrgb) + Vfx_rgb
	 * 12           14      16	*/
	mxv_mult(&tmp1, m_wb, v_bl);			/* [1] */
	mxv_mult(&tmp2, &m_ycbcr[ycbcr_enc], &tmp1);	/* [2] */
	vxv_sub(&tmp1, &v_ycbcr, &tmp2);		/* [3] */
	vxv_sub(&tmp2, &tmp1, &half);			/* [4] */
	mxv_mult(&tmp1, m_hue, &tmp2);			/* [5] */
	mxv_mult(&tmp2, m_con, &tmp1);			/* [6] */
	mxv_mult(&tmp1, m_sat, &tmp2);			/* [7] */
	mxv_mult(&tmp2, m_ck, &tmp1);			/* [8] */
	vxv_add(&tmp1, v_bri, &tmp2);			/* [9] */
	vxv_add(&tmp2, &half, &tmp1);			/* [10] */
	mxv_mult(&tmp1, m_fx_ycbcr, &tmp2);		/* [11] */
	vxv_add(&tmp2, v_fx_ycbcr, &tmp1);		/* [12] */
	mxv_mult(&tmp1, &m_rgb[ycbcr_enc][VINC_QUANTIZATION_FULL_RANGE],
		 &tmp2);				/* [13] */
	vxv_add(&tmp2, &v_rgb[ycbcr_enc][VINC_QUANTIZATION_FULL_RANGE],
		 &tmp1);				/* [14] */
	mxv_mult(&tmp1, m_fx_rgb, &tmp2);		/* [15] */
	vxv_add(offset, v_fx_rgb, &tmp1);		/* [16] */
}

static inline bool check_row_overflow(struct matrix *const coeff,
				      struct vector *const offset,
				      int const row)
{
	return coeff->coeff[row * 3] +
	       coeff->coeff[row * 3 + 1] +
	       coeff->coeff[row * 3 + 2] +
	       offset->offset[row] / CC_OFFSET_MAX >= 16;
}

/**
 * check_cc_overflow() - checks coefficient matrix and offset vector for
 *                        possibility of overflow (see rf#1954)
 * @coeff: coefficient matrix
 * @offset: offset vector
 *
 * Return: true if overflow bug is possible, false otherwise.
 */
static bool check_cc_overflow(struct matrix *coeff, struct vector *offset)
{
	return check_row_overflow(coeff, offset, 0) ||
	       check_row_overflow(coeff, offset, 1) ||
	       check_row_overflow(coeff, offset, 2) ||
	       !is_in_range(offset->offset[0], -CC_OFFSET_MAX,
			    CC_OFFSET_MAX - 1) ||
	       !is_in_range(offset->offset[1], -CC_OFFSET_MAX,
			    CC_OFFSET_MAX - 1) ||
	       !is_in_range(offset->offset[2], -CC_OFFSET_MAX,
			    CC_OFFSET_MAX - 1);
}

/* Color Correction coefficient matrix, offset vector and scaling register
 * calculation routine */
int vinc_neon_calculate_cc(struct ctrl_priv *ctrl_privs,
			   enum vinc_ycbcr_encoding ycbcr_enc,
			   struct vinc_cc *cc)
{
	struct matrix coeff;
	struct vector offset;
	u8 scaling;

	u16 i;
	double max_abs = 0;

	cc_matrix_calc(&coeff, ctrl_privs, ycbcr_enc);
	cc_vector_calc(&offset, ctrl_privs, ycbcr_enc);

	if (check_cc_overflow(&coeff, &offset))
		return -ERANGE;

	/*  Scaling calculation */
	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		if (fabs(coeff.coeff[i]) > max_abs)
			max_abs = fabs(coeff.coeff[i]);

	if (max_abs < 1)
		scaling = 0;
	else
		scaling = ilog2((u16)max_abs) + 1;

	max_abs = max_abs + pow(2, scaling - 16);

	if (max_abs < 1)
		scaling = 0;
	else
		scaling = ilog2((u16)max_abs) + 1;

	/* Vector, matrix and scaling write to CC cache */
	cc->scaling = scaling;
	for (i = 0; i < VINC_CC_COEFF_COUNT; i++)
		cc->coeff[i] = float_to_u16(coeff.coeff[i], 15 - scaling);
	for (i = 0; i < VINC_CC_OFFSET_COUNT; i++)
		cc->offset[i] = float_to_u16(offset.offset[i], 0);

	return 0;
}

/*
 * FIXME: These functions are sensor-specific. They should be moved
 * to proper place and should not be used here.
 */

static u32 gain_to_ctrl(double *gain)
{
	u32 gain_int, gain_round, ctrl;
	u8 power;
	double frac;

	gain_int = (u32)(*gain);
	power = ilog2(gain_int);
	gain_round = rounddown_pow_of_two(gain_int);

	frac = (double)(*gain - gain_round);
	ctrl = power * 16 + ((frac * 16) / gain_round);
	return ctrl;
}

static double ctrl_to_gain(u32 ctrl)
{
	double tmp, gain;
	u32 value;
	u8 power;

	power = ctrl / 16;
	tmp = (double)(ctrl % 16) / 16;
	value = pow(2, power);

	gain = (double)value * (tmp + 1);
	return gain;
}

u32 vinc_neon_calculate_luma_avg(enum vinc_input_format input_format,
				 struct vinc_stat_add *add,
				 enum vinc_ycbcr_encoding ycbcr_enc,
				 struct vinc_stat_zone *zone,
				 bool pport_low_bits)
{
	double luma_stat, luma_average;

	if (input_format == YCbCr) {
		luma_stat = add->sum_g;
		if (pport_low_bits)
			luma_stat *= 4;
	} else
		luma_stat = m_ycbcr[ycbcr_enc].coeff[0] * add->sum_r +
			    m_ycbcr[ycbcr_enc].coeff[1] * add->sum_g +
			    m_ycbcr[ycbcr_enc].coeff[2] * add->sum_b;

	luma_average = luma_stat / ((zone->x_rb + 1) * (zone->y_rb + 1));
	return rint(luma_average);
}

void vinc_neon_calculate_gain_exp(u32 luma, u32 cur_gain, u32 cur_exp,
				  u32 max_gain, u32 max_exp, u32 *gain,
				  u32 *exp)
{
	const double th = 110.0, smoothness = 0.5;
	double adjustment = 1.0f;
	double cur_gain_dbl, gain_dbl, exp_dbl, brightness, desired_brightness;
	double max_gain_dbl;

	max_gain_dbl = ctrl_to_gain(max_gain);

	adjustment = th / luma;

	adjustment = clamp(adjustment, 1.0 / 16, 4.0);

	cur_gain_dbl = ctrl_to_gain(cur_gain);
	brightness = cur_exp * cur_gain_dbl;
	desired_brightness = brightness * adjustment;
	desired_brightness = brightness * smoothness + desired_brightness *
			     (1 - smoothness);

	if (desired_brightness > max_exp) {
		exp_dbl = max_exp;
		gain_dbl = desired_brightness / max_exp;

	} else {
		exp_dbl = desired_brightness;
		gain_dbl = 1.0f;
	}
	if (gain_dbl > max_gain_dbl) {
		exp_dbl = desired_brightness/max_gain_dbl;
		gain_dbl = max_gain_dbl;
	}

	*exp = rint(exp_dbl);
	*gain = gain_to_ctrl(&gain_dbl);
}
