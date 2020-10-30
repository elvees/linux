/*
 * Copyright 2015 ELVEES NeoTek CJSC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */
#ifndef _UAPI__LINUX_VINC_H
#define _UAPI__LINUX_VINC_H

#include <linux/types.h>
#include <linux/v4l2-controls.h>

#define V4L2_CID_BAD_CORRECTION_ENABLE	(V4L2_CID_CAMERA_CLASS_BASE + 0x1000)
#define V4L2_CID_BAD_PIXELS		(V4L2_CID_CAMERA_CLASS_BASE + 0x1001)
#define V4L2_CID_BAD_ROWS		(V4L2_CID_CAMERA_CLASS_BASE + 0x1002)
#define V4L2_CID_BAD_COLS		(V4L2_CID_CAMERA_CLASS_BASE + 0x1003)
#define V4L2_CID_GAMMA_CURVE_ENABLE	(V4L2_CID_CAMERA_CLASS_BASE + 0x1004)
#define V4L2_CID_GAMMA_CURVE		(V4L2_CID_CAMERA_CLASS_BASE + 0x1005)
#define V4L2_CID_CC_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x1006)
#define V4L2_CID_CC			(V4L2_CID_CAMERA_CLASS_BASE + 0x1007)
#define V4L2_CID_CT_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x1008)
#define V4L2_CID_CT			(V4L2_CID_CAMERA_CLASS_BASE + 0x1009)
#define V4L2_CID_DR_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x100A)
#define V4L2_CID_DR			(V4L2_CID_CAMERA_CLASS_BASE + 0x100B)
#define V4L2_CID_STAT_ENABLE		(V4L2_CID_CAMERA_CLASS_BASE + 0x100C)
#define V4L2_CID_STAT_AF_COLOR		(V4L2_CID_CAMERA_CLASS_BASE + 0x100D)
#define V4L2_CID_STAT_AF_TH		(V4L2_CID_CAMERA_CLASS_BASE + 0x100E)
#define V4L2_CID_STAT_ZONE0		(V4L2_CID_CAMERA_CLASS_BASE + 0x100F)
#define V4L2_CID_STAT_ZONE1		(V4L2_CID_CAMERA_CLASS_BASE + 0x1010)
#define V4L2_CID_STAT_ZONE2		(V4L2_CID_CAMERA_CLASS_BASE + 0x1011)
#define V4L2_CID_STAT_ZONE3		(V4L2_CID_CAMERA_CLASS_BASE + 0x1012)
#define V4L2_CID_STAT_HIST0		(V4L2_CID_CAMERA_CLASS_BASE + 0x1013)
#define V4L2_CID_STAT_HIST1		(V4L2_CID_CAMERA_CLASS_BASE + 0x1014)
#define V4L2_CID_STAT_HIST2		(V4L2_CID_CAMERA_CLASS_BASE + 0x1015)
#define V4L2_CID_STAT_HIST3		(V4L2_CID_CAMERA_CLASS_BASE + 0x1016)
#define V4L2_CID_STAT_AF0		(V4L2_CID_CAMERA_CLASS_BASE + 0x1017)
#define V4L2_CID_STAT_AF1		(V4L2_CID_CAMERA_CLASS_BASE + 0x1018)
#define V4L2_CID_STAT_AF2		(V4L2_CID_CAMERA_CLASS_BASE + 0x1019)
#define V4L2_CID_STAT_AF3		(V4L2_CID_CAMERA_CLASS_BASE + 0x101A)
#define V4L2_CID_STAT_ADD0		(V4L2_CID_CAMERA_CLASS_BASE + 0x101B)
#define V4L2_CID_STAT_ADD1		(V4L2_CID_CAMERA_CLASS_BASE + 0x101C)
#define V4L2_CID_STAT_ADD2		(V4L2_CID_CAMERA_CLASS_BASE + 0x101D)
#define V4L2_CID_STAT_ADD3		(V4L2_CID_CAMERA_CLASS_BASE + 0x101E)
#define V4L2_CID_SENSOR_EXPOSURE_AUTO	(V4L2_CID_CAMERA_CLASS_BASE + 0x101F)
#define V4L2_CID_SENSOR_AUTOGAIN	(V4L2_CID_CAMERA_CLASS_BASE + 0x1020)
#define V4L2_CID_SENSOR_AUTO_WHITE_BALANCE (V4L2_CID_CAMERA_CLASS_BASE + 0x1021)
#define V4L2_CID_BLACK_LEVEL_CORR	(V4L2_CID_CAMERA_CLASS_BASE + 0x1022)
#define V4L2_CID_WHITE_BALANCE_CC	(V4L2_CID_CAMERA_CLASS_BASE + 0x1023)


#define VINC_COMP_VALUES_COUNT		(1 << 12)
#define VINC_CC_COEFF_COUNT		9
#define VINC_CC_OFFSET_COUNT		3
#define VINC_WB_CC_COUNT		5
#define VINC_STAT_HIST_COUNT		256

struct vinc_bad_pixel {
	__u16 x;
	__u16 y;
};

struct vinc_gamma_curve {
	__u16 red[VINC_COMP_VALUES_COUNT];
	__u16 green[VINC_COMP_VALUES_COUNT];
	__u16 blue[VINC_COMP_VALUES_COUNT];
};

struct vinc_cc {
	__u16 coeff[VINC_CC_COEFF_COUNT];
	__u16 offset[VINC_CC_OFFSET_COUNT];
	__u8 scaling;
};

struct vinc_wb_cc {
	__u16 temp[VINC_WB_CC_COUNT];
	__u16 gain_r[VINC_WB_CC_COUNT];
	__u16 gain_b[VINC_WB_CC_COUNT];
	__u16 coeff[VINC_WB_CC_COUNT][VINC_CC_COEFF_COUNT];
};

struct vinc_stat_zone {
	__u16 enable;
	__u16 x_lt;
	__u16 y_lt;
	__u16 x_rb;
	__u16 y_rb;
};

struct vinc_stat_hist {
	__u32 red[VINC_STAT_HIST_COUNT];
	__u32 green[VINC_STAT_HIST_COUNT];
	__u32 blue[VINC_STAT_HIST_COUNT];
};

struct vinc_stat_af {
	__u32 hsobel;
	__u32 vsobel;
	__u32 lsobel;
	__u32 rsobel;
};

struct vinc_stat_add {
	__u64 sum2_r;
	__u64 sum2_g;
	__u64 sum2_b;
	__u32 sum_r;
	__u32 sum_g;
	__u32 sum_b;
	__u8 min_r;
	__u8 min_g;
	__u8 min_b;
	__u8 max_r;
	__u8 max_g;
	__u8 max_b;
};

#endif /* _UAPI__LINUX_VINC_H */
