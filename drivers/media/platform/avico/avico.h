/*
 * ELVEES Avico (a.k.a. VELcore-01) driver
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

#ifndef AVICO_H
#define AVICO_H

#include <linux/dmaengine.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>

#include "avico-bitstream.h"

#define NCLKS 4

#define AVICO_CTRL_BASE     0x00
#define AVICO_CTRL_EVENTS   0x00
#define AVICO_CTRL_MSKI_CPU 0x04
#define AVICO_CTRL_MSKO_CPU 0x08
#define AVICO_CTRL_MSKO_DSP 0x0c
#define AVICO_CTRL_VRAM_CFG 0x10
#define AVICO_CTRL_MSK_INT  0x14
#define AVICO_CTRL_MSK_EV   0x18

#define AVICO_THREAD_BASE(i) (0x1000 + 0x40 * (i))
#define AVICO_THREAD_TASK   0x00
#define AVICO_THREAD_ADR    0x04
#define AVICO_THREAD_MBPOS  0x08
#define AVICO_THREAD_FRMN   0x0c
#define AVICO_THREAD_CFG    0x10
#define AVICO_THREAD_ON     0x18
#define AVICO_THREAD_OFF    0x1c
#define AVICO_THREAD_STATUS 0x20
#define AVICO_THREAD_SMBPOS 0x28

#define AVICO_MD_SYS_BASE 0x2000
#define AVICO_MD_SYS_CFG    0x00

#define AVICO_MD_BASE(i) (AVICO_MD_SYS_BASE + 0x80 + 0x20 * (i))
#define AVICO_MD_MBPOS  0x00
#define AVICO_MD_FRMN   0x04

#define AVICO_TQ_BASE(i) (0x3000 + 0x40 * (i))
#define AVICO_TQ_MBPOS  0x00
#define AVICO_TQ_FRMN   0x04

#define AVICO_VDMA_BASE 0x5000

#define AVICO_VDMA_SYS_BASE 0x800
#define AVICO_VDMA_SYS_RUN   0x00
#define AVICO_VDMA_SYS_BUSY  0x04
#define AVICO_VDMA_SYS_ACTV  0x08
#define AVICO_VDMA_SYS_DONE  0x0c
#define AVICO_VDMA_SYS_READY 0x10
#define AVICO_VDMA_SYS_MODE  0x14

#define AVICO_VDMA_CHANNEL_BASE(i) (0x80 * (i))
#define AVICO_VDMA_CHANNEL_A0E    0x00
#define AVICO_VDMA_CHANNEL_AECUR  0x04
#define AVICO_VDMA_CHANNEL_A0I    0x08
#define AVICO_VDMA_CHANNEL_AICUR  0x0c
#define AVICO_VDMA_CHANNEL_BEIDX  0x10
#define AVICO_VDMA_CHANNEL_CEIDX  0x14
#define AVICO_VDMA_CHANNEL_HEIDX  0x18
#define AVICO_VDMA_CHANNEL_VEIDX  0x1c
#define AVICO_VDMA_CHANNEL_BIIDX  0x20
#define AVICO_VDMA_CHANNEL_CIIDX  0x24
#define AVICO_VDMA_CHANNEL_HIIDX  0x28
#define AVICO_VDMA_CHANNEL_VIIDX  0x2c
#define AVICO_VDMA_CHANNEL_ACNT   0x30
#define AVICO_VDMA_CHANNEL_BCCNT  0x34
#define AVICO_VDMA_CHANNEL_HVECNT 0x38
#define AVICO_VDMA_CHANNEL_HVICNT 0x3c
#define AVICO_VDMA_CHANNEL_RUN    0x40
#define AVICO_VDMA_CHANNEL_DONE   0x44
#define AVICO_VDMA_CHANNEL_IMRDY  0x48
#define AVICO_VDMA_CHANNEL_CFG    0x4c

#define AVICO_ECD_BASE 0x8000
#define AVICO_EC_BASE(i) (AVICO_ECD_BASE + 0x4000 * (i))
#define AVICO_ED_BASE(i) (AVICO_ECD_BASE + 0x2000 + 0x4000 * (i))
#define AVICO_EC_REGC     0x0000
#define AVICO_EC_CAVLC    0x0a00
#define AVICO_EC_HDRC     0x1800
#define AVICO_EC_PACKER   0x1a00
#define AVICO_EC_VRAMCTRC 0x1c00
#define AVICO_EC_TASKCTRC 0x1e00

#define AVICO_REGC_CP0 0x000
#define AVICO_REGC_CP1 0x010
#define AVICO_REGC_CP2 0x020
#define AVICO_REGC_CP3 0x030
#define AVICO_REGC_CPN 0x040
#define AVICO_REGC_CPA 0x050
#define AVICO_REGC_CPB 0x060
#define AVICO_REGC_CQC 0x080
#define AVICO_REGC_CBS 0x100

#define AVICO_REGC_CP0_SIZE 0x010
#define AVICO_REGC_CP1_SIZE 0x010
#define AVICO_REGC_CP2_SIZE 0x010
#define AVICO_REGC_CP3_SIZE 0x010
#define AVICO_REGC_CPN_SIZE 0x010
#define AVICO_REGC_CPA_SIZE 0x010
#define AVICO_REGC_CPB_SIZE 0x010
#define AVICO_REGC_CQC_SIZE 0x080
#define AVICO_REGC_CBS_SIZE 0x100

#define AVICO_TASKCTRC_TASK   0x00
#define AVICO_TASKCTRC_CS     0x04
#define AVICO_TASKCTRC_MBPOS  0x08
#define AVICO_TASKCTRC_FRMN   0x0C
#define AVICO_TASKCTRC_DMALEN 0x10

#define AVICO_VRAMCTRC_BASE_CP0 0x00
#define AVICO_VRAMCTRC_BASE_CP1 0x04
#define AVICO_VRAMCTRC_BASE_CP2 0x08
#define AVICO_VRAMCTRC_BASE_CP3 0x0c
#define AVICO_VRAMCTRC_BASE_CPN 0x10
#define AVICO_VRAMCTRC_BASE_CQC 0x20
#define AVICO_VRAMCTRC_BASE_CBS 0x24
#define AVICO_VRAMCTRC_SIZE_CP0 0x40
#define AVICO_VRAMCTRC_SIZE_CP1 0x44
#define AVICO_VRAMCTRC_SIZE_CP2 0x48
#define AVICO_VRAMCTRC_SIZE_CP3 0x4c
#define AVICO_VRAMCTRC_SIZE_CQC 0x60
#define AVICO_VRAMCTRC_SIZE_CBS 0x64
#define AVICO_VRAMCTRC_ADDR_CP0 0x80
#define AVICO_VRAMCTRC_ADDR_CP1 0x84
#define AVICO_VRAMCTRC_ADDR_CP2 0x88
#define AVICO_VRAMCTRC_ADDR_CP3 0x8c
#define AVICO_VRAMCTRC_ADDR_CPN 0x90
#define AVICO_VRAMCTRC_ADDR_CPA 0x94
#define AVICO_VRAMCTRC_ADDR_CPB 0x98
#define AVICO_VRAMCTRC_ADDR_CQC 0xa0
#define AVICO_VRAMCTRC_ADDR_CBS 0xa4

#define AVICO_CAVLC_CR 0x00
#define AVICO_CAVLC_SR 0x04

#define AVICO_HDRC_CR 0x00

#define AVICO_PACKER_CR              0x00
#define AVICO_PACKER_CBS_TOTAL_LEN   0x04
#define AVICO_PACKER_CBS_EXTBITS     0x08
#define AVICO_PACKER_CBS_EXTBITS_LEN 0x0c
#define AVICO_PACKER_CBS_STUFF_MODE  0x10
#define AVICO_PACKER_CBS_STUFF_POS   0x14

#define EV_0 BIT(0)
#define EV_1 BIT(1)
#define EV_2 BIT(2)
#define EV_3 BIT(3)
#define EV_4 BIT(4)
#define EV_5 BIT(5)
#define EV_6 BIT(6)
#define EV_7 BIT(7)

enum {
	ECD_CS_CABAC = 1,
	ECD_CS_CAVLC = 2,
	ECD_CS_JPGHUF = 3,
	ECD_CS_MPG2VL = 4,
	ECD_CS_MPG4VL = 5
};

enum frame_type {
	VE_FR_I    = 2,
	VE_FR_P    = 0,
	VE_FR_NONE = 8
};

/*
 * Global registers
 */

union events {
	uint32_t val;
	struct {
		uint8_t enc[2];
		uint8_t dec[2];
	} __packed;
};

/* \todo No documentation */
union mski_cpu {
	uint32_t val;
};

/* \todo No documentation */
union msko_cpu {
	uint32_t val;
};

/* \todo No documentation */
union msko_dsp {
	uint32_t val;
};

union vram_cfg {
	uint32_t val;
	struct {
		unsigned rm  :2; /* \todo No documentation */
		unsigned rme :1; /* \todo No documentation */
		unsigned ls  :1; /* \todo No documentation */
		unsigned ds  :1; /* \todo No documentation */
		unsigned sd  :1; /* \todo No documentation */
		unsigned reserved :24;
	};
};

/* \todo No documentation */
union msk_int {
	uint32_t val;
};

/* \todo No documentation */
union msk_ev {
	uint32_t val;
	struct {
		uint8_t enc[2];
		uint8_t dec[2];
	} __packed;
};

struct control {
	union events   events;
	union mski_cpu mski_cpu;
	union msko_cpu msko_cpu;
	union vram_cfg vram_cfg;
	union msk_int msk_int;
	union msk_ev  msk_ev;
};

/*
 * Thread control registers
 */

union task {
	uint32_t val;
	struct {
		unsigned std :2;
		unsigned qpy :6;
		unsigned qpc :6;
		unsigned run :1;
		unsigned dbf :1;
		unsigned m6eof :1;
		unsigned m6pos :1;
		unsigned:2; /* \todo Name all reserved fields */
		unsigned m7eof :1;
		unsigned m7pos :1;
	};
};

union adr {
	uint32_t val;
	struct {
		unsigned acur :10;
		unsigned aref :10;
		unsigned ares :9;
	};
};

union mbpos {
	uint32_t val;
	struct {
		uint8_t posx;
		uint8_t posy;
		uint8_t nx;
		uint8_t ny;
	} __packed;
};

union frmn {
	uint32_t val;
	struct {
		unsigned frmn  :24;
		unsigned gop   :6;
		unsigned idr   :1;
		unsigned ftype :1;
	};
};

union cfg {
	uint32_t val;
	struct {
		unsigned slice_type :4;
		unsigned num_ref    :5;
		unsigned auto_trailing :1;
		unsigned auto_flush    :1;
		unsigned offset_qpskip :5;
		unsigned offset_a      :4;
		unsigned offset_b      :4;
		unsigned offset_qp     :5;
	};
};

union status {
	uint32_t val;
	struct {
		unsigned eof0 :1;
		unsigned eof1 :1;
		unsigned:1;
		unsigned errmv :1;
		unsigned erridx :1;
		unsigned errmode :1;
		unsigned:25;
		unsigned busy :1;
	};
};

union smbpos {
	uint32_t val;
	struct {
		uint8_t x6, y6, x7, y7;
	};
};

struct thread {
	union task task;
	union adr adr;
	union mbpos mbpos;
	union frmn frmn;
	union cfg cfg;
	uint32_t on;
	uint32_t off;
	union status status;
	union smbpos smbpos;
};

enum ecd_task_id {
	ECD_TASK_ADDEXBITS  = 0x92,
	ECD_TASK_ADDEXBYTES = 0x93,
	ECD_TASK_H264_ENC_RESET = 0xc0,
	ECD_TASK_H264_ENC_SLICE_RESET,
	ECD_TASK_H264_ENC_MB420,
	ECD_TASK_H264_ENC_TRAIL,
	ECD_TASK_H264_ENC_FLUSH
};

union ecd_task {
	uint32_t val;
	struct {
		unsigned repn   :16;
		unsigned id     :8;
		unsigned:1;
		unsigned m7frmn :1;
		unsigned m7pos  :1;
		unsigned m7eof  :1;
		unsigned:1;
		unsigned rep    :1;
		unsigned ready  :1;
		unsigned run    :1;
	};
};

union vdma_acnt {
	uint32_t val;
	struct {
		uint16_t acnt;
		uint16_t arld;
	};
};

union vdma_bccnt {
	uint32_t val;
	struct {
		uint8_t bcnt;
		uint8_t ccnt;
		uint8_t brld;
		uint8_t crld;
	};
};

union vdma_hvecnt {
	uint32_t val;
	struct {
		uint8_t hecnt;
		uint8_t vecnt;
		uint8_t herld;
		uint8_t verld;
	};
};

union vdma_hvicnt {
	uint32_t val;
	struct {
		uint8_t hicnt;
		uint8_t vicnt;
		uint8_t hirld;
		uint8_t virld;
	};
};

union vdma_cfg {
	uint32_t val;
	struct {
		unsigned dir     :1;
		unsigned dim     :1;
		unsigned cycle   :1;
		unsigned prt     :4;
		unsigned brst_ae :1;
		unsigned brst_ai :1;
		unsigned size    :2;
	};
};

struct avico_dev {
	struct v4l2_device v4l2_dev;
	struct video_device vfd;

	struct device *dev;

	void __iomem *regs;
	phys_addr_t vram;

	struct dma_chan *dma_ch;

	struct clk *clk[NCLKS];

	spinlock_t irqlock;
	struct mutex mutex;
	struct v4l2_m2m_dev *m2m_dev;
	struct vb2_alloc_ctx *alloc_ctx;
};

enum cdc_std {
	VE_STD_H264,
	VE_STD_MPEG4,
	VE_STD_MPEG2,
	VE_STD_JPEG
};

/* \todo Use kerneldoc format to describe structs and fields */
struct task2 {
	enum cdc_std enc_std; /* Video encoding standard */
	uint8_t qp_y;         /* Luma QP */
	uint8_t qp_c;         /* Chroma QP */
	uint8_t enable_dbf;   /* Whether to use DBF */
};

struct cdc_h264_cfg {
	enum frame_type slice_type; /* according to ITU */
	uint8_t num_ref_idx_l0_active_minus1;

	/* whether EC should add trailing bits to NALUs automatically */
	uint8_t auto_trailing_bits;

	/* whether EC should write encoded NAL to external memory with
	 * VDMA automatically */
	uint8_t auto_flush;

	int8_t dbf_off_a;       /* DBF alpha offset */
	int8_t dbf_off_b;       /* DBF beta offset */
	int8_t qp_off;          /* QP offset */
	int8_t skip_qp_off;     /* skip QP offset */
	uint8_t md_accur_shift; /* MD accuracy shift */
};

struct mb_pos {
	uint8_t pos_x; /* MB horiz. position */
	uint8_t pos_y; /* MB vert. position */
	uint8_t nx;    /* frame width in MBs */
	uint8_t ny;    /* frame height in MBs */
};

struct md_cfg {
	unsigned ic_dc :1;
	unsigned ic_h  :1;
	unsigned ic_v  :1;
	unsigned ic_plane :1;

	unsigned il16_v  :1;
	unsigned il16_h  :1;
	unsigned il16_dc :1;
	unsigned il16_plane :1;

	unsigned il84_v   :1;
	unsigned il84_h   :1;
	unsigned il84_dc  :1;
	unsigned il84_ddl :1;
	unsigned il84_ddr :1;
	unsigned il84_vr  :1;
	unsigned il84_hd  :1;
	unsigned il84_vl  :1;
	unsigned il84_hu  :1;

	unsigned p_16x16 :1;
	unsigned p_8x8   :1;
	unsigned p_4x4   :1;
	unsigned p_16x8  :1;
	unsigned p_8x16  :1;
	unsigned p_8x4   :1;
	unsigned p_4x8   :1;
	unsigned p_skip  :1;

	unsigned mv_zero   :1;
	unsigned sub_pel   :1;
	unsigned p_skip_16 :1;
	unsigned p_skip_b  :1;
	unsigned pnlt_i4   :1;
};

struct avico_enc_params {
	uint16_t sps_pps_id;  /* let's consider them to be always equal */
	uint16_t gop_size;
	uint16_t crop_left;   /* in pixels, for luma */
	uint16_t crop_right;  /* in pixels, for luma */
	uint16_t crop_top;    /* in pixels, for luma */
	uint16_t crop_bottom; /* in pixels, for luma */
	struct md_cfg md_cfg;
	uint32_t num_units_in_tick;
	uint32_t time_scale;
	uint8_t poc_type;
	uint8_t vdma_trans_size_m1;
	struct task2 task;
	struct cdc_h264_cfg cdc_h264_cfg;
};

struct avico_frame_params {
	uint8_t sps;
	uint8_t pps;
	uint32_t gop;
	uint8_t poc_type;
	uint8_t qp_i, qp_p, pps_qp;
	int8_t qpc_offset;
	int dbf;
	enum frame_type frame_type;
	bool idr;
	uint16_t idr_id;
	unsigned int frame;
	uint8_t log2_max_frame;
	unsigned int i_period;
	struct {
		uint16_t left, right, top, bottom;
	} crop;
};

struct avico_ctx {
	struct v4l2_fh fh;
	struct avico_dev *dev;

	/* Abort requested by m2m */
	int aborting;

	unsigned long state;

	struct avico_frame_params par;

	/* \todo Crop */
	/* \todo MD configuration */

	struct v4l2_fract timeperframe;
	uint8_t vdma_trans_size_m1;

	uint8_t mbx, mby;
	bool force_key;
	unsigned int bitstream_size;

	bool error;

	struct bitstream bs;

	unsigned int id; /* Hardware thread ID */
	void *vref;
	dma_addr_t dmaref, dmainp, dmaout;
	dma_addr_t bounceref[2], bounceout[2];
	uint32_t ref_ptr_off, out_ptr_off;  /* Offsets for new data */
	int bounce_active;  /* Active bounce buffer */

	int bounce_count;
	spinlock_t bounce_lock;

	void *vmbref, *vmbcur;
	dma_addr_t dmambref, dmambcur;
	unsigned int mbrefsize, mbcursize;

	enum v4l2_colorspace colorspace;
	enum v4l2_quantization range;
	enum v4l2_ycbcr_encoding matrix;
	enum v4l2_xfer_func transfer;

	unsigned int width, height;
	unsigned int capfmt, outfmt;
	unsigned int outsize, capsize, refsize;
	unsigned int outseq, capseq;

	struct v4l2_ctrl	*ctrl_qp_i;
	struct v4l2_ctrl	*ctrl_qp_p;
	struct v4l2_ctrl	*ctrl_qpc_off;
	struct v4l2_ctrl	*ctrl_gop;
	struct v4l2_ctrl_handler ctrl_handler;
};

#define MAX_RBSP_LENGTH 0x0ffffc

#define MB_CUR_SIZE     512
#define MB_REF_SIZE     512
#define MB_RES_SIZE     1024
#define MB_PRED_SIZE    512
#define MB_PRED_RESERVE 512

#endif /* AVICO_H */
